/*
  ==========================================================================
  ESP32 Cabinet Project — Fan + DHT22 + Tach + Wood Moisture (ADS1115) + SSD1306 OLED
  ==========================================================================

  WHAT THIS PROGRAM DOES (in plain English)
  -----------------------------------------
  • Reads room humidity & temperature from a DHT22 sensor.
  • Drives a 4-wire PC fan using high-frequency PWM (about 25 kHz) plus a "kick start".
  • Measures fan RPM using ESP32's hardware pulse counter (PCNT) and smooths the reading.
  • Reads a wood moisture probe via an ADS1115 ADC (very accurate) and smooths the signal.
  • Shows the important stuff on a small OLED screen (SSD1306 SPI).
  • Wakes the OLED automatically on power-up, whenever you press SW1, and whenever the fan
    changes between OFF and ON.
  • Generic 4-minute auto-off timer per LED channel (LED1 and LED2) if commanded ON continuously.

  NEW: LED1 “courtesy” with your physical layout in mind
  ------------------------------------------------------
  • SW1 = Door. SW2 = Top drawer above the door (can open independently).
    SW3–SW5 = Drawers behind the door (won’t change state when SW1 is ON).
  • When SW1 turns ON (door opened), LED1 holds a **3% floor** for **4 minutes**.
    - While courtesy is active, LED1 output = max(requested by switches, 3%).
    - If **SW2** changes while courtesy is active, the 4-minute timer resets.
    - When the 4-minute window expires, **LED1 turns OFF** and enters **lockout**:
         stays OFF until either SW1 turns OFF (door closes) OR SW2 changes,
         both of which clear lockout and (if SW1 is ON) restart courtesy.
    - If SW1 turns OFF at any time, courtesy cancels and lockout clears.

  ==========================================================================
*/

#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <Adafruit_ADS1X15.h>

extern "C" {
  #include "driver/pcnt.h"
}

/* =========================== FEATURE / DEBUG ============================ */
#define DEBUG_ADS        1   // 1 = Print ADS1115 debug lines on Serial, 0 = be quiet
#define PROBE_PULSE      1   // 1 = Energize moisture probe only during reads (reduces corrosion)

/* ============================= USER SETTINGS ============================ */
#define TEMP_MODE 2                 // 0 = C only, 1 = F only, 2 = Both on Serial (OLED shows F only)
const float IDEAL_MC_LOW  = 8.0f;   // Wood moisture "ideal" window lower marker (bar line)
const float IDEAL_MC_HIGH = 12.0f;  // Wood moisture "ideal" window upper marker (bar line)
const float MOIST_BAR_MAX = 15.0f;  // OLED bar scale only

/* ---- SW1 behavior ---- */
const unsigned long SW1_HOLD_MS     = 3000;
const unsigned long SW1_DEBOUNCE_MS = 50;

/* ---- Manual fan timings (default 6 minutes total) ---- */
const unsigned long FAN_MANUAL_RAMP_MS  = 1UL * 60UL * 1000UL;
const unsigned long FAN_MANUAL_HOLD_MS  = 4UL * 60UL * 1000UL;
const unsigned long FAN_MANUAL_DECEL_MS = 1UL * 60UL * 1000UL;
const unsigned long FAN_MANUAL_TOTAL_MS =
  (FAN_MANUAL_RAMP_MS + FAN_MANUAL_HOLD_MS + FAN_MANUAL_DECEL_MS);

/* ---- Manual fan limits ---- */
const float MANUAL_MIN_PCT = 5.0f;
const float MANUAL_MAX_PCT = 75.0f;

/* ---- DHT22 read rate ---- */
#ifndef DHT_INTERVAL_MS
const unsigned long DHT_INTERVAL_MS = 2000UL;
#endif

/* ---- OLED wake timer ---- */
const unsigned long OLED_WAKE_MS = 10UL * 60UL * 1000UL;

/* ---- LED auto-off (generic) ---- */
const unsigned long LED_AUTO_OFF_MS = 4UL * 60UL * 1000UL;  // 4 minutes

/* ---- LED1 courtesy-on when SW1 turns ON ---- */
/* Courtesy acts as a floor (3%) for 4 minutes; resets on SW2 changes. */
const int LED1_COURTESY_LEVEL = (int)(255 * 0.03f + 0.5f);   // ≈3% -> ~8/255
const unsigned long LED1_COURTESY_MS = 4UL * 60UL * 1000UL;  // 4 minutes

/* ================================ PINS ================================= */
const int LED1_PIN = 25;
const int LED2_PIN = 26;
const bool INVERT  = false;  // If your LED board is active-LOW, set true.

const int SW1_PIN = 4;   // Door (active-LOW)
const int SW2_PIN = 19;  // Top drawer above door (independent)
const int SW3_PIN = 13;  // Drawer behind door
const int SW4_PIN = 14;  // Drawer behind door
const int SW5_PIN = 27;  // Drawer behind door

// Fan control (hybrid: relay for hard OFF, PWM for speed)
const int  FAN_PIN = 22;
const int  FAN_RELAY_PIN = 16;
const bool RELAY_ACTIVE_LOW = false;

// Tach input (from fan TACH, with 10k pull-up to 3.3V)
const int FAN_TACH_PIN = 34;

// ADS1115 I2C pins (fixed to match your wiring)
#define I2C_SDA 33
#define I2C_SCL 32

// Moisture probe power (we "pulse" this during reads)
const int SENSOR_PWR_PIN = 2;

/* ================================ DHT22 ================================ */
#define DHTPIN   21
#define DHTTYPE  DHT22
DHT dht(DHTPIN, DHTTYPE);
unsigned long lastDhtMs = 0;
float lastHumidity = NAN, lastTempC = NAN, lastTempF = NAN;

/* ============================== LED LEVELS ============================= */
const int SW1_LED1_LEVEL = 50;   // LED1 baseline when only SW1 is ON
const int SW1_LED2_LEVEL = 74;  // LED2 follows SW1
const int SW2_LEVEL = 25;
const int SW3_LEVEL = 90;
const int SW4_LEVEL = 150;
const int SW5_LEVEL = 255;

/* ============================ LED FADE TUNING ========================== */
const unsigned long FADE_TIME_MS_LED1 = 2000;
const unsigned long FADE_TIME_MS_LED2 = 2000;
const unsigned long FADE_TICK_MS      = 5;
const int PWM_FREQ_HZ = 200;                   // LED PWM frequency
const uint8_t PWM_DEADBAND = 2;                // Treat tiny values as 0

/* ============================= FAN / PWM =============================== */
bool FAN_ACTIVE_HIGH = false;
int  FAN_PWM_FREQ_HZ = 25000;
const int  FAN_MIN_RUN_DUTY   = 25;   // ~10%
const unsigned long FAN_KICK_MS = 400;

unsigned long fanOffAtMs    = 0;
unsigned long manualStartMs = 0;
int  fanSpeed               = 0;        // 0..255 (applied)
int  prevFanSpeed           = -1;
bool fanKickActive          = false;
unsigned long fanKickEndMs  = 0;
bool fanRelayOn             = false;

unsigned long manualDecelStartMs = 0;  // 0=not forced
float         manualDecelFromPct = MANUAL_MAX_PCT;

unsigned long sw1PressStartMs = 0;
bool          sw1HoldFired    = false;

volatile bool     sw1DecelIrq        = false;
volatile uint32_t sw1LastIrqMs       = 0;
const uint32_t    SW1_IRQ_DEBOUNCE_MS = 100;

/* ============================== TACH / RPM ============================= */
#define TACH_PPR                  2
#define TACH_GPIO                 FAN_TACH_PIN
#define TACH_COUNT_BOTH_EDGES     1
#define TACH_GATE_MS              500
#define TACH_TIMEOUT_MS           1500
#define TACH_GLITCH_US            20
#define TACH_MAX_RPM_SPEC         1700
const float RPM_EMA_ALPHA = 0.20f;

static pcnt_unit_t TACH_UNIT = PCNT_UNIT_0;
static unsigned long tachLastGateMs   = 0;
static unsigned long tachLastEdgeMs   = 0;
static int16_t       tachLastCount    = 0;
int lastRpm = 0;

/* =========================== LED FADE STATE ============================ */
float current1 = 0.0f, current2 = 0.0f;
int   target1  = 0,    target2  = 0;
unsigned long lastTick1Ms = 0, lastTick2Ms = 0;

/* --------------------- LED auto-off state (4 min) ---------------------- */
unsigned long led1OnSinceMs = 0, led2OnSinceMs = 0;
bool          led1AutoOff   = false, led2AutoOff   = false;

/* ---- LED1 courtesy-on state ---- */
unsigned long led1CourtesyUntilMs = 0;
bool          led1CourtesyActive  = false;
bool          led1CourtesyLockout = false;   // after timeout, force OFF until SW1 OFF or SW2 changes

/* ============ ADS1115 + Moisture TABLE + "Anchor" Shift ================= */
Adafruit_ADS1115 ads;   // 0x48
#define ADS_GAIN  GAIN_ONE
#define DIFF_SWAP 1
const unsigned long PULSE_SETTLE_MS = 300;

const int NPTS = 8;
const int16_t PT_COUNTS_BASE[NPTS] = {
  -25503,  // 0.0%
  -25500,  // 2.5%
  -25497,  // 5.0%
  -25489,  // 8.0%
  -25483,  // 9.0%
  -25478,  // 12.5%
  -25429,  // 14.5%
  -24815   // 17.0%
};
const float PT_PCT[NPTS] = { 0.0f, 2.5f, 5.0f, 8.0f, 9.0f, 12.5f, 14.5f, 17.0f };

const float  USER_ANCHOR_PCT        = 2.5f;
int16_t      USER_ANCHOR_COUNTS_2P5 = -25500;

int anchorIndex() {
  for (int i = 0; i < NPTS; ++i) {
    if (fabsf(PT_PCT[i] - USER_ANCHOR_PCT) < 0.001f) return i;
  }
  return 1;
}

/* ========================= Moisture Filtering =========================== */
#define MEDIAN_WIN   11
#define EMA_ALPHA    0.15f
#define OVERSAMPLE_N 3
const unsigned long MOIST_SAMPLE_INTERVAL_MS = 1200;

int16_t medBuf[MEDIAN_WIN] = {0};
uint8_t medIdx = 0;
bool    medFilled = false;
float   emaCounts = NAN;
unsigned long lastMoistReadMs = 0;
float   lastMoistPct = NAN;
int16_t lastSmoothedCounts = 0;

/* =============================== OLED =================================== */
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(
  U8G2_R0,
  /* cs=*/ 5,
  /* dc=*/ 15,
  /* reset=*/ 17
);
const uint16_t OLED_REFRESH_MS = 500;
unsigned long lastOledMs = 0;
bool oledOn = false;
unsigned long oledWakeUntilMs = 0;

/* ============================== HELPERS ================================= */
inline void writePWM(int pin, float lin) {
  int v = (int)(lin + 0.5f);
  if (v <= PWM_DEADBAND) v = 0;
  if (v > 255) v = 255;
  int out = INVERT ? (255 - v) : v;
  analogWrite(pin, out);
}

inline void fanSetFreq(int hz) {
  FAN_PWM_FREQ_HZ = hz;
  analogWriteResolution(FAN_PIN, 8);
  analogWriteFrequency(FAN_PIN, FAN_PWM_FREQ_HZ);
}

inline void relayWrite(bool on) {
  fanRelayOn = on;
  int level = RELAY_ACTIVE_LOW ? (on ? LOW : HIGH) : (on ? HIGH : LOW);
  digitalWrite(FAN_RELAY_PIN, level);
}

inline void fanWritePWM(int duty) {
  if (duty <= 0) {
    relayWrite(false);
    fanSpeed = 0;
    analogWrite(FAN_PIN, FAN_ACTIVE_HIGH ? 255 : 0);
    lastRpm = 0;
    return;
  }
  relayWrite(true);
  fanSpeed = constrain(duty, 0, 255);
  int out = FAN_ACTIVE_HIGH ? fanSpeed : (255 - fanSpeed);
  analogWrite(FAN_PIN, out);
}

void fadeTickSmooth(float &current, int target, unsigned long &lastTickMs,
                    unsigned long fadeTimeMs, unsigned long now, int outPin) {
  if (now - lastTickMs < FADE_TICK_MS) return;
  lastTickMs = now;
  if ((int)current == target) return;

  float diff  = (float)target - current;
  float adiff = fabsf(diff);
  if (adiff < 0.25f) { current = (float)target; writePWM(outPin, current); return; }

  unsigned long totalTicks = fadeTimeMs / FADE_TICK_MS;
  if (totalTicks == 0) totalTicks = 1;
  float step = adiff / (float)totalTicks;
  if (step < 0.25f) step = 0.25f;

  current += (diff > 0 ? +step : -step);
  if ((diff > 0 && current > target) || (diff < 0 && current < target))
    current = (float)target;

  writePWM(outPin, current);
}

static inline float adsFSR_V() {
  switch (ADS_GAIN) {
    case GAIN_TWOTHIRDS: return 6.144f;
    case GAIN_ONE:       return 4.096f;
    case GAIN_TWO:       return 2.048f;
    case GAIN_FOUR:      return 1.024f;
    case GAIN_EIGHT:     return 0.512f;
    case GAIN_SIXTEEN:   return 0.256f;
    default:             return 4.096f;
  }
}
static inline float adsLSB_V() { return adsFSR_V() / 32768.0f; }

inline const char* onOff(bool b) { return b ? "ON " : "OFF"; }

/* ====================== Moisture: mapping & reads ======================= */
// Forward declare mapping helper used by readMoisturePctADS
int16_t shiftedCountAt(uint8_t i);
float   mapCountsToPct(int16_t c);

int16_t shiftedCountAt(uint8_t i) {
  int     idx          = anchorIndex();
  int16_t baseAtAnchor = PT_COUNTS_BASE[idx];
  int32_t delta        = (int32_t)USER_ANCHOR_COUNTS_2P5 - (int32_t)baseAtAnchor;
  int32_t val          = (int32_t)PT_COUNTS_BASE[i] + delta;
  if (val < -32768) val = -32768;
  if (val >  32767) val =  32767;
  return (int16_t)val;
}

float mapCountsToPct(int16_t c) {
  int16_t c0 = shiftedCountAt(0);
  int16_t cN = shiftedCountAt(NPTS - 1);
  bool asc = (c0 <= cN);

  int16_t cmin = asc ? c0 : cN;
  int16_t cmax = asc ? cN : c0;
  float   pmin = asc ? PT_PCT[0]      : PT_PCT[NPTS-1];
  float   pmax = asc ? PT_PCT[NPTS-1] : PT_PCT[0];

  if (c <= cmin) return pmin;
  if (c >= cmax) return pmax;

  for (int i = 0; i < NPTS - 1; ++i) {
    int16_t a = shiftedCountAt(i);
    int16_t b = shiftedCountAt(i + 1);
    bool inSeg = asc ? (c >= a && c <= b) : (c <= a && c >= b);
    if (inSeg) {
      float ap = PT_PCT[i], bp = PT_PCT[i + 1];
      int32_t dx = (int32_t)b - (int32_t)a;
      if (dx == 0) return (ap + bp) * 0.5f;
      float t = float((int32_t)c - (int32_t)a) / float(dx);
      return ap + t * (bp - ap);
    }
  }
  return pmin;
}

// Median-of-11 helper (removes spikes)
int16_t median11() {
  int16_t tmp[MEDIAN_WIN];
  memcpy(tmp, medBuf, sizeof(tmp));
  for (int i = 1; i < MEDIAN_WIN; ++i) {
    int16_t v = tmp[i]; int j = i - 1;
    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; --j; }
    tmp[j + 1] = v;
  }
  return tmp[MEDIAN_WIN / 2];
}

// Average N reads of differential (0-1); if swap==true, flip sign
int16_t readDiffAvgSwap(uint8_t n, bool swap) {
  long sum = 0;
  for (uint8_t i = 0; i < n; ++i) {
    int16_t v = ads.readADC_Differential_0_1();
    if (swap) v = -v;
    sum += v;
  }
  return (int16_t)((sum + (n/2)) / n);
}

float clampDelta(float v, float prev, float maxDelta) {
  if (isnan(prev)) return v;
  float d = v - prev;
  if (d >  maxDelta) return prev + maxDelta;
  if (d < -maxDelta) return prev - maxDelta;
  return v;
}

// Returns % moisture; also updates global lastSmoothedCounts
float readMoisturePctADS() {
  unsigned long now = millis();
  if (now - lastMoistReadMs < MOIST_SAMPLE_INTERVAL_MS) {
    return lastMoistPct;
  }
  lastMoistReadMs = now;

  if (PROBE_PULSE) {
    pinMode(SENSOR_PWR_PIN, OUTPUT);
    digitalWrite(SENSOR_PWR_PIN, HIGH);
    delay(PULSE_SETTLE_MS);
  }

  int16_t d01 = readDiffAvgSwap(OVERSAMPLE_N, DIFF_SWAP);

  if (PROBE_PULSE) {
    digitalWrite(SENSOR_PWR_PIN, LOW);
  }

  // Fill median buffer
  medBuf[medIdx++] = d01;
  if (medIdx >= MEDIAN_WIN) { medIdx = 0; medFilled = true; }
  int16_t med = medFilled ? median11() : d01;

  // EMA smoothing on top of the median
  if (isnan(emaCounts)) emaCounts = med;
  else                 emaCounts = EMA_ALPHA * med + (1.0f - EMA_ALPHA) * emaCounts;

  int16_t smoothed = (int16_t)lrintf(emaCounts);
  lastSmoothedCounts = smoothed;

  // Convert counts -> % moisture
  float pct = mapCountsToPct(smoothed);

  // Gentle rate limit in % units (keeps the number from jumping visually)
  pct = clampDelta(pct, lastMoistPct, 0.20f);
  lastMoistPct = pct;

#if DEBUG_ADS
  // Optional single-ended prints to help understand signals
  int16_t a0 = ads.readADC_SingleEnded(0);
  int16_t a1 = ads.readADC_SingleEnded(1);
  const float LSB = adsLSB_V();
  float v0 = a0 * LSB;
  float v1 = a1 * LSB;
  float vd = d01 * LSB;

  Serial.print("[ADS] A0:"); Serial.print(a0);  Serial.print(" ("); Serial.print(v0, 5); Serial.print(" V)  ");
  Serial.print("A1:");       Serial.print(a1);  Serial.print(" ("); Serial.print(v1, 5); Serial.print(" V)  ");
  Serial.print(DIFF_SWAP ? "DIFF(1-0):" : "DIFF(0-1):");
  Serial.print(d01); Serial.print(" ("); Serial.print(vd, 5); Serial.print(" V)  ");
  Serial.print("med:");      Serial.print(med);
  Serial.print("  smooth:"); Serial.print(smoothed);
  Serial.print("  Wood Moisture: "); Serial.print(pct, 2); Serial.println("%");
#endif

  return pct;
}

/* ============================ TACH / PCNT =============================== */
void tachBegin() {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = TACH_GPIO;
  cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;

#if TACH_COUNT_BOTH_EDGES
  cfg.pos_mode   = PCNT_COUNT_INC;
  cfg.neg_mode   = PCNT_COUNT_INC;
#else
  cfg.pos_mode   = PCNT_COUNT_DIS;
  cfg.neg_mode   = PCNT_COUNT_INC;
#endif

  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.unit       = TACH_UNIT;
  cfg.channel    = PCNT_CHANNEL_0;

  pcnt_unit_config(&cfg);

  uint16_t cycles = (uint16_t)min((uint32_t)(TACH_GLITCH_US * 80), (uint32_t)1023);
  pcnt_set_filter_value(TACH_UNIT, cycles);
  pcnt_filter_enable(TACH_UNIT);

  pcnt_counter_pause(TACH_UNIT);
  pcnt_counter_clear(TACH_UNIT);
  pcnt_counter_resume(TACH_UNIT);

  tachLastGateMs = millis();
  tachLastEdgeMs = millis();
  tachLastCount  = 0;
}

void tachUpdate() {
  unsigned long now = millis();

  int16_t count = 0;
  pcnt_get_counter_value(TACH_UNIT, &count);

  if (now - tachLastGateMs >= TACH_GATE_MS) {
    int16_t delta = count - tachLastCount;
    tachLastCount = count;
    tachLastGateMs = now;

    if (delta > 0) tachLastEdgeMs = now;

    const int EDGES_PER_REV = TACH_PPR * (TACH_COUNT_BOTH_EDGES ? 2 : 1);
    const float COEFF = 60000.0f / ( (float)TACH_GATE_MS * (float)EDGES_PER_REV );

    int rpmInstant = lastRpm;

    if (fanRelayOn && fanSpeed > 0) {
      if (delta > 0) {
        rpmInstant = (int)(delta * COEFF + 0.5f);
      } else if (now - tachLastEdgeMs > TACH_TIMEOUT_MS) {
        rpmInstant = 0;
      }
    } else {
      rpmInstant = 0;
      tachLastEdgeMs = now;
    }

    if (rpmInstant < 0) rpmInstant = 0;
    if (rpmInstant > TACH_MAX_RPM_SPEC) rpmInstant = TACH_MAX_RPM_SPEC;

    if (lastRpm == 0) lastRpm = rpmInstant;
    else lastRpm = (int)(RPM_EMA_ALPHA * rpmInstant + (1.0f - RPM_EMA_ALPHA) * lastRpm);

    Serial.print("Fan RPM: "); Serial.println(lastRpm);
    int fanPctNow = (fanSpeed * 100 + 127) / 255;
    Serial.print("Fan Status - Speed: "); Serial.print(fanPctNow);
    Serial.print("%  RPM: "); Serial.println(lastRpm);
  }
}

/* =============================== SW1 ISR ================================ */
void IRAM_ATTR sw1ISR() {
  uint32_t now = millis();
  if ((uint32_t)(now - sw1LastIrqMs) > SW1_IRQ_DEBOUNCE_MS) {
    sw1DecelIrq  = true;
    sw1LastIrqMs = now;
  }
}

/* ============================== OLED UI ================================= */
void drawOLED_UI(
  bool s1, bool s2, bool s3, bool s4, bool s5,
  int current1, int /*target1*/, int current2, int /*target2*/ ,
  int fanSpeed, bool /*manualActive*/, unsigned long /*fanOffAtMs*/,
  int rpm,
  float humidity, float /*tempC*/, float tempF,
  float moistPct, int16_t smoothedCounts,
  int32_t manualSecs
) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);

  const uint8_t y0=7, y1=15, y2=23, y3=31, y4=39, y5=47, y6=55, y7=63;

  u8g2.drawStr(0, y0, "STONE CABINET");

  char ledLine[40];
  snprintf(ledLine, sizeof(ledLine), "LED1 %3d%%  | LED2 %3d%%",
           (current1 <= 0 ? 0 : (current1 * 100 + 127) / 255),
           (current2 <= 0 ? 0 : (current2 * 100 + 127) / 255));
  u8g2.drawStr(0, y1, ledLine);

  char swLineA[32], swLineB[32];
  const char* sw1Text = s1 ? "OFF" : "ON"; // active-LOW; HIGH=released
  snprintf(swLineA, sizeof(swLineA), "SW1:%s SW2:%s SW3:%s",
           sw1Text, onOff(s2), onOff(s3));
  snprintf(swLineB, sizeof(swLineB), "SW4:%s SW5:%s",
           onOff(s4), onOff(s5));
  u8g2.drawStr(0, y2, swLineA);
  u8g2.drawStr(0, y3, swLineB);

  const int fanPct = (fanSpeed <= 0) ? 0 : (fanSpeed * 100 + 127) / 255;

  char spdBuf[16];
  snprintf(spdBuf, sizeof(spdBuf), "SPD:%3d%%", fanPct);
  u8g2.drawStr(0, y4, spdBuf);

  char rpmBuf[16];
  if (fanPct == 0 || rpm < 120) { strcpy(rpmBuf, "RPM: --"); }
  else { snprintf(rpmBuf, sizeof(rpmBuf), "RPM:%4d", rpm); }
  u8g2.drawStr(47, y4, rpmBuf);

  if (manualSecs >= 0) {
    char tBuf[16];
    uint8_t mm = (manualSecs / 60) % 100;
    uint8_t ss = (uint8_t)(manualSecs % 60);
    snprintf(tBuf, sizeof(tBuf), "T:%02u:%02u", mm, ss);
    u8g2.drawStr(93, y4, tBuf);
  }

  char envLine[40];
  if (!isnan(humidity) && !isnan(tempF))
    snprintf(envLine, sizeof(envLine), "RH:%6.1f%%  TEMP:%5.1fF", humidity, tempF);
  else
    snprintf(envLine, sizeof(envLine), "RH: --.-%%  TEMP: --.-F");
  u8g2.drawStr(0, y5, envLine);

  const int barW = 28, barX = 128 - barW - 1, barY = y6 - 6, barH = 8;

  char moistLine[40];
  if (!isnan(moistPct)) snprintf(moistLine, sizeof(moistLine), "Wood Moisture: %4.1f%%", moistPct);
  else                  snprintf(moistLine, sizeof(moistLine), "Wood Moisture:");
  while (u8g2.getStrWidth(moistLine) > barX - 2) {
    size_t L = strlen(moistLine);
    if (L <= 3) break;
    moistLine[L-1] = '\0';
  }
  u8g2.drawStr(0, y6, moistLine);

  u8g2.drawFrame(barX, barY, barW, barH);
  if (!isnan(moistPct)) {
    float mc = moistPct; if (mc < 0) mc = 0; if (mc > MOIST_BAR_MAX) mc = MOIST_BAR_MAX;
    int fill = (int)(barW * (mc / MOIST_BAR_MAX) + 0.5f);
    if (fill > 1) u8g2.drawBox(barX + 1, barY + 1, fill - 2, barH - 2);
    int ix1 = barX + (int)(barW * (IDEAL_MC_LOW  / MOIST_BAR_MAX));
    int ix2 = barX + (int)(barW * (IDEAL_MC_HIGH / MOIST_BAR_MAX));
    if (ix1 < barX) ix1 = barX; if (ix1 > barX + barW - 1) ix1 = barX + barW - 1;
    if (ix2 < barX) ix2 = barX; if (ix2 > barX + barW - 1) ix2 = barX + barW - 1;
    u8g2.drawVLine(ix1, barY - 1, barH + 2);
    u8g2.drawVLine(ix2, barY - 1, barH + 2);
  }

  char smLine[28];
  snprintf(smLine, sizeof(smLine), "SM: %d", (int)smoothedCounts);
  u8g2.drawStr(0, y7, smLine);

  u8g2.sendBuffer();
}

/* ================================ SETUP ================================= */
void setup() {
  pinMode(FAN_RELAY_PIN, OUTPUT);
  digitalWrite(FAN_RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);

  pinMode(SENSOR_PWR_PIN, OUTPUT);
  digitalWrite(SENSOR_PWR_PIN, PROBE_PULSE ? LOW : HIGH);

  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("ESP32 Cabinet Project - start");

  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  pinMode(SW3_PIN, INPUT_PULLUP);
  pinMode(SW4_PIN, INPUT_PULLUP);
  pinMode(SW5_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(SW1_PIN), sw1ISR, FALLING);

  SPI.begin(/*SCLK=*/18, /*MISO=*/-1, /*MOSI=*/23, /*SS=*/-1);

  analogWriteResolution(LED1_PIN, 8);
  analogWriteFrequency(LED1_PIN, PWM_FREQ_HZ);
  analogWriteResolution(LED2_PIN, 8);
  analogWriteFrequency(LED2_PIN, PWM_FREQ_HZ);
  writePWM(LED1_PIN, 0.0f);
  writePWM(LED2_PIN, 0.0f);
  lastTick1Ms = lastTick2Ms = millis();

  pinMode(FAN_PIN, OUTPUT);
  fanSetFreq(FAN_PWM_FREQ_HZ);
  analogWrite(FAN_PIN, FAN_ACTIVE_HIGH ? 255 : 0);
  relayWrite(false);

  pinMode(FAN_TACH_PIN, INPUT);
  tachBegin();

  dht.begin();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  if (!ads.begin(0x48)) {
    Serial.println("[ERR] ADS1115 not found at 0x48. Check wiring.");
  } else {
    ads.setGain(ADS_GAIN);
    ads.setDataRate(RATE_ADS1115_16SPS);
    Serial.print("[OK] ADS1115 ready @ "); Serial.print(adsFSR_V(), 3);
    Serial.println(" V FS, 16 SPS");
  }

  u8g2.begin();
  u8g2.setContrast(255);
  oledOn = true;
  oledWakeUntilMs = millis() + OLED_WAKE_MS;
  u8g2.setPowerSave(0);
  lastOledMs = 0;
}

/* ================================= LOOP ================================= */
void loop() {
  unsigned long now = millis();

  // Switches (active-LOW; released=HIGH)
  bool s1 = (digitalRead(SW1_PIN) == HIGH); // Door: HIGH=released (OFF), LOW=pressed (ON)
  bool s2 = (digitalRead(SW2_PIN) == HIGH);
  bool s3 = (digitalRead(SW3_PIN) == HIGH);
  bool s4 = (digitalRead(SW4_PIN) == HIGH);
  bool s5 = (digitalRead(SW5_PIN) == HIGH);

  // Edge detection for SW1 (door)
  static bool s1EdgeInit = false;
  static bool prevS1 = true;
  if (!s1EdgeInit) { prevS1 = s1; s1EdgeInit = true; }
  bool s1FallingEdge = (prevS1 && !s1);  // SW1 ON (door opened; press -> LOW)
  bool s1RisingEdge  = (!prevS1 && s1);  // SW1 OFF (door closed; release -> HIGH)
  prevS1 = s1;

 // Track SW2 edge (independent drawer)
static bool prevS2 = true;
bool s2FallingEdge = (prevS2 && !s2);   // SW2 ON (drawer opened; LOW)
bool s2RisingEdge  = (!prevS2 && s2);   // SW2 OFF (drawer closed; HIGH)
bool s2Changed     = (prevS2 != s2);
prevS2 = s2;



  // Wake OLED for OLED_WAKE_MS on any SW1 edge
  if (s1FallingEdge || s1RisingEdge) {
    oledOn = true;
    oledWakeUntilMs = now + OLED_WAKE_MS;
    u8g2.setPowerSave(0);
    lastOledMs = 0;
  }

 // --------------------- LED1 Courtesy / Lockout Control ------------------
// Start courtesy when SW1 OR SW2 turns ON (falling edge, active-LOW)
if (s1FallingEdge || s2FallingEdge) {
  led1CourtesyActive  = true;
  led1CourtesyLockout = false;                 // clear any previous lockout
  led1CourtesyUntilMs = now + LED1_COURTESY_MS;
  Serial.println("[LED1] Courtesy start (S1/S2 ON) -> 3% for 4 minutes");
}

// Cancel courtesy immediately when SW1 OR SW2 turns OFF (rising edge)
if (s1RisingEdge || s2RisingEdge) {
  if (led1CourtesyActive) Serial.println("[LED1] Courtesy cancel (S1/S2 OFF)");
  led1CourtesyActive  = false;
  // Do NOT force lockout on cancel; only on timeout
  led1CourtesyUntilMs = 0;
}

// While courtesy is active, an ON edge from EITHER switch resets the timer
if (led1CourtesyActive && (s1FallingEdge || s2FallingEdge)) {
  led1CourtesyUntilMs = now + LED1_COURTESY_MS;
  Serial.println("[LED1] Courtesy timer reset (S1/S2 ON edge)");
}

// Apply courtesy or lockout effects
if (led1CourtesyActive) {
  // If still within the window, hold LED1 at least 3%
  if ((long)(led1CourtesyUntilMs - now) > 0) {
    if (target1 < LED1_COURTESY_LEVEL) target1 = LED1_COURTESY_LEVEL;
  } else {
    // Timer expired → force OFF and latch lockout
    led1CourtesyActive  = false;
    led1CourtesyLockout = true;
    led1CourtesyUntilMs = 0;
    target1 = 0;
    Serial.println("[LED1] Courtesy expired -> OFF (lockout)");
  }

} else if (led1CourtesyLockout) {
  // Clear lockout when the door closes OR SW2 changes (toggle)
  if (s1 || s2Changed) {
    led1CourtesyLockout = false;
    Serial.println("[LED1] Lockout cleared (door closed OR SW2 toggled)");
  } else {
    target1 = 0; // keep forced OFF until cleared
  }
}


  // ----------------------- LED targets with auto-off ---------------------
  // LED2 follows SW1; LED1 uses the lowest of SW2..SW5 when ON (else SW1 level)
  int target2Raw = s1 ? SW1_LED2_LEVEL : 0;

  int lowest = 0;
  if (s2) lowest = (lowest==0) ? SW2_LEVEL : min(lowest, SW2_LEVEL);
  if (s3) lowest = (lowest==0) ? SW3_LEVEL : min(lowest, SW3_LEVEL);
  if (s4) lowest = (lowest==0) ? SW4_LEVEL : min(lowest, SW4_LEVEL);
  if (s5) lowest = (lowest==0) ? SW5_LEVEL : min(lowest, SW5_LEVEL);
  int target1Raw = (lowest > 0) ? lowest : (s1 ? SW1_LED1_LEVEL : 0);

  // 4-minute auto-off per LED if left on continuously
  auto updateLedAutoOff = [&](int raw, unsigned long &sinceMs, bool &autoOffFlag) -> int {
    if (raw > 0) {
      if (sinceMs == 0) sinceMs = now;                 // just turned on (start timer)
      if (!autoOffFlag && (now - sinceMs >= LED_AUTO_OFF_MS)) {
        autoOffFlag = true;                            // hit the limit -> force off
        Serial.println("[LED] Auto-off triggered after 4 minutes");
      }
    } else {                                           // off or requested off
      sinceMs = 0;
      autoOffFlag = false;                             // clear the override
    }
    return autoOffFlag ? 0 : raw;                      // apply override if needed
  };

  target1 = updateLedAutoOff(target1Raw, led1OnSinceMs, led1AutoOff);
  target2 = updateLedAutoOff(target2Raw, led2OnSinceMs, led2AutoOff);

  // Courtesy behavior:
  // • While active AND door is open, hold LED1 at least 3%.
  // • When the 4-min window expires, force LED1 OFF and latch lockout.
  if (led1CourtesyActive) {
    bool doorOpen = !s1; // pressed=LOW => ON/open
    if (!doorOpen) {
      // Door closed -> cancel courtesy immediately
      led1CourtesyActive  = false;
      led1CourtesyLockout = false;
      led1CourtesyUntilMs = 0;
    } else if ((long)(led1CourtesyUntilMs - now) > 0) {
      if (target1 < LED1_COURTESY_LEVEL) target1 = LED1_COURTESY_LEVEL;
    } else {
      // Timer expired: force OFF and latch lockout (until SW1 OFF or SW2 change)
      led1CourtesyActive  = false;
      led1CourtesyLockout = true;
      led1CourtesyUntilMs = 0;
      target1 = 0;  // force off NOW
      Serial.println("[LED1] Courtesy expired -> OFF (lockout)");
    }
} else if (led1CourtesyLockout) {
  // Clear lockout when the door closes OR SW2 changes (toggle)
  if (s1 || s2Changed) {
    led1CourtesyLockout = false;
  } else {
    target1 = 0; // keep forced OFF until cleared
  }
}


  // =================== SENSOR + FAN LOGIC (unchanged) ====================
  if (now - lastDhtMs >= DHT_INTERVAL_MS) {
    lastDhtMs = now;
    float h  = dht.readHumidity();
    float tC = dht.readTemperature();
    if (!isnan(h) && !isnan(tC)) {
      float tF = tC * 1.8f + 32.0f;
      lastHumidity = h;
      lastTempC    = tC;
      lastTempF    = tF;

      if (TEMP_MODE == 0) {
        Serial.print("Humidity: "); Serial.print(h); Serial.print("%  | ");
        Serial.print("Temp: "); Serial.print(tC); Serial.println(" C");
      } else if (TEMP_MODE == 1) {
        Serial.print("Humidity: "); Serial.print(h); Serial.print("%  | ");
        Serial.print("Temp: "); Serial.print(tF); Serial.println(" F");
      } else {
        Serial.print("Humidity: "); Serial.print(h); Serial.print("%  | ");
        Serial.print("Temp: "); Serial.print(tC); Serial.print(" C / ");
        Serial.print(tF); Serial.println(" F");
      }
    } else {
      Serial.println("DHT read failed");
    }
  }

  float moistPct = readMoisturePctADS();

  static bool fanWasOn = false;
  int humidityDuty = 0;
  if (!isnan(lastHumidity)) {
    float h = lastHumidity;
    if (h >= 82.0f)      { humidityDuty = 255; fanWasOn = true; }
    else if (h >= 80.0f) { humidityDuty = 191; fanWasOn = true; }
    else if (h >= 78.0f) { humidityDuty = 128; fanWasOn = true; }
    else if (h >= 75.0f) { humidityDuty =  64; fanWasOn = true; }
    else if (h >= 74.0f) { humidityDuty =  26; fanWasOn = true; }
    else if (h >= 72.0f) { humidityDuty = fanWasOn ? 26 : 0; }
    else                 { humidityDuty =   0; fanWasOn = false; }
  }

  // Manual override via SW1 (hold)
  bool sw1Pressed = !s1;  // physical press = LOW
  static bool manualActive = false;

  if (!manualActive) {
    if (sw1Pressed) {
      if (sw1PressStartMs == 0) { sw1PressStartMs = now; sw1HoldFired = false; }
      else if (!sw1HoldFired && (now - sw1PressStartMs >= SW1_HOLD_MS)) {
        fanOffAtMs    = now + FAN_MANUAL_TOTAL_MS;
        manualStartMs = now;
        manualActive  = true;
        sw1HoldFired  = true;

        manualDecelStartMs = 0;
        manualDecelFromPct = MANUAL_MAX_PCT;

        sw1DecelIrq  = false;
        sw1LastIrqMs = now;

        Serial.println("[SW1] Hold 3s - manual fan 6 min started");
      }
    } else {
      sw1PressStartMs = 0;
      sw1HoldFired = false;
    }
  } else {
    if (!sw1Pressed) { sw1PressStartMs = 0; sw1HoldFired = false; }
  }

  // Forced decel (tap during manual)
  bool wantDecel = (manualActive && manualDecelStartMs == 0) &&
                   (s1FallingEdge || sw1DecelIrq) &&
                   (millis() - manualStartMs > 150);
  if (wantDecel) {
    sw1DecelIrq = false;
    manualDecelStartMs = millis();

    int fanPctNow = (fanSpeed * 100 + 127) / 255;
    if (fanPctNow < (int)MANUAL_MIN_PCT) fanPctNow = (int)MANUAL_MIN_PCT;
    manualDecelFromPct = (float)fanPctNow;

    fanOffAtMs = manualDecelStartMs + FAN_MANUAL_DECEL_MS;

    Serial.print("[SW1] Forced decel start from ~");
    Serial.print(fanPctNow);
    Serial.println("% for 1 min");
  }

  if (fanOffAtMs != 0 && (long)(fanOffAtMs - now) > 0) manualActive = true;
  else if (manualActive && fanOffAtMs != 0) {
    manualActive = false;
    fanOffAtMs = 0;
    manualDecelStartMs = 0;
  }

  int desiredDuty;
  if (manualActive) {
    float dutyPct;
    if (manualDecelStartMs != 0) {
      unsigned long ed = now - manualDecelStartMs;
      float p = min(1.0f, (float)ed / (float)FAN_MANUAL_DECEL_MS);
      dutyPct = manualDecelFromPct - p * (manualDecelFromPct - MANUAL_MIN_PCT);
    } else {
      unsigned long e = now - manualStartMs;
      if (e <= FAN_MANUAL_RAMP_MS) {
        float p = (float)e / (float)FAN_MANUAL_RAMP_MS;
        dutyPct = MANUAL_MIN_PCT + p * (MANUAL_MAX_PCT - MANUAL_MIN_PCT);
      } else if (e <= (FAN_MANUAL_RAMP_MS + FAN_MANUAL_HOLD_MS)) {
        dutyPct = MANUAL_MAX_PCT;
      } else if (e <= FAN_MANUAL_TOTAL_MS) {
        unsigned long d = e - (FAN_MANUAL_RAMP_MS + FAN_MANUAL_HOLD_MS);
        float p = (float)d / (float)FAN_MANUAL_DECEL_MS;
        dutyPct = MANUAL_MAX_PCT - p * (MANUAL_MAX_PCT - MANUAL_MIN_PCT);
      } else {
        dutyPct = 0.0f;
      }
    }
    desiredDuty = (int)(dutyPct * 255.0f / 100.0f + 0.5f);
  } else {
    desiredDuty = humidityDuty;
  }

  static bool prevFanOn = false;
  bool willBeOn = (desiredDuty > 0);
  if (willBeOn != prevFanOn) {
    oledOn = true;
    oledWakeUntilMs = now + OLED_WAKE_MS;
    u8g2.setPowerSave(0);
    lastOledMs = 0;
    prevFanOn = willBeOn;
  }

  bool inFinalDecel =
    manualActive && (
      manualDecelStartMs != 0 ||
      ( (now - manualStartMs) > (FAN_MANUAL_RAMP_MS + FAN_MANUAL_HOLD_MS) &&
        (now - manualStartMs) <=  FAN_MANUAL_TOTAL_MS )
    );

  if (desiredDuty <= 0) {
    fanKickActive = false;
    fanWritePWM(0);
  } else {
    if (fanSpeed == 0 && !fanKickActive) {
      fanKickActive = true;
      fanKickEndMs  = now + FAN_KICK_MS;
      fanWritePWM(255);
    } else if (fanKickActive) {
      if ((long)(fanKickEndMs - now) <= 0) {
        fanKickActive = false;
        int postDuty = desiredDuty;
        if (postDuty > 0 && postDuty < FAN_MIN_RUN_DUTY && !inFinalDecel) postDuty = FAN_MIN_RUN_DUTY;
        fanWritePWM(postDuty);
      } else {
        fanWritePWM(255);
      }
    } else {
      int runDuty = desiredDuty;
      if (runDuty > 0 && runDuty < FAN_MIN_RUN_DUTY && !inFinalDecel) runDuty = FAN_MIN_RUN_DUTY;
      fanWritePWM(runDuty);
    }
  }

  tachUpdate();

  // LED fades
  fadeTickSmooth(current1, target1, lastTick1Ms, FADE_TIME_MS_LED1, now, LED1_PIN);
  fadeTickSmooth(current2, target2, lastTick2Ms, FADE_TIME_MS_LED2, now, LED2_PIN);

  // Manual countdown (seconds) for OLED
  int32_t manualSecs = -1;
  if (fanOffAtMs != 0 && (long)(fanOffAtMs - now) > 0) {
    unsigned long msLeft = fanOffAtMs - now;
    manualSecs = (int32_t)(msLeft / 1000UL);
    if (manualSecs < 0) manualSecs = 0;
  }

  if (oledOn) {
    if (now - lastOledMs >= OLED_REFRESH_MS) {
      lastOledMs = now;
      drawOLED_UI(
        s1, s2, s3, s4, s5,
        (int)current1, target1, (int)current2, target2,
        fanSpeed, (fanOffAtMs!=0 && (long)(fanOffAtMs - now) > 0), fanOffAtMs,
        lastRpm,
        lastHumidity, lastTempC, lastTempF,
        lastMoistPct, lastSmoothedCounts,
        manualSecs
      );
    }
    if ((long)(oledWakeUntilMs - now) <= 0) {
      oledOn = false;
      u8g2.clearBuffer(); u8g2.sendBuffer();
      u8g2.setPowerSave(1);
    }
  }

  delay(1);
}
