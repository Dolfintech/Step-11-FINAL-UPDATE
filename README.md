ESP32 Cabinet Project – Settings Cheat Sheet
1) LED Row — Brightness & Timers
What controls LED brightness
•	LED1 brightness comes from the lowest (dimmest) active source among SW2–SW5.
•	If no drawers are open, LED1 falls back to the SW1 baseline brightness.
•	LED2 brightness simply follows SW1.
•	Brightness is on a 0–255 scale (≈0–100%).
Quick picks:
25% → 64    50% → 128    75% → 191    100% → 255
Where to edit brightness levels
These values appear near the top of your sketch:
// Baselines when SW1 is ON (door)
const int SW1_LED1_LEVEL = 50;   // LED1 when only SW1 is ON
const int SW1_LED2_LEVEL = 74;   // LED2 follows SW1

// Drawer levels (used when drawers are open)
const int SW2_LEVEL = 25;
const int SW3_LEVEL = 90;
const int SW4_LEVEL = 150;
const int SW5_LEVEL = 255;
How to change LED1 brightness
•	When only SW1 is ON → change SW1_LED1_LEVEL.
•	When a drawer switch is ON → change SW2_LEVEL, SW3_LEVEL, SW4_LEVEL, SW5_LEVEL.
Note: If multiple drawers are open, LED1 uses the dimmest value.
How to change LED2 brightness
// LED2 follows SW1
const int SW1_LED2_LEVEL = 74;
// Example: set LED2 ≈ 75%
// const int SW1_LED2_LEVEL = 191;
Courtesy Floor (LED1 only)
This sets the minimum brightness for LED1 when the door (SW1) or SW2 first turns ON:
const int LED1_COURTESY_LEVEL = (int)(255 * 0.03f + 0.5f);  // ≈3%
Examples:
• 5% → 13
• 10% → 26
If your regular brightness is lower than this during courtesy mode, LED1 will still show at least the courtesy floor.
⭐ LED Timers (NEW)
Your LEDs have two timer types you can adjust:
A) LED Auto-Off Timer (LED1 + LED2)
This is the timer that turns an LED OFF after being ON continuously:
const unsigned long LED_AUTO_OFF_MS = 4UL * 60UL * 1000UL;  // 4 minutes
Change it to:
• 6 minutes → 6UL * 60UL * 1000UL
• 10 minutes → 10UL * 60UL * 1000UL
• Disable auto-off → set to 0
B) LED1 Courtesy Timer (LED1 only)
Controls how long LED1 stays at the courtesy brightness after SW1 or SW2 triggers it:
const unsigned long LED1_COURTESY_MS = 4UL * 60UL * 1000UL;  // 4 minutes
Change to:
• 6 minutes → 6UL * 60UL * 1000UL
• 1 minute → 1UL * 60UL * 1000UL
• Disable courtesy → 0
Optional: Fade Feel & Polarity
const unsigned long FADE_TIME_MS_LED1 = 2000;
const unsigned long FADE_TIME_MS_LED2 = 2000;

const bool INVERT = false;   // Set true if LED driver is active-LOW
2) Switch Row (SW1–SW5)
OLED row example:
SW1:ON  SW2:OFF  SW3:ON  SW4:OFF  SW5:OFF
This row shows the live state of all switches.
Because the switches are wired active-LOW, the OLED will show:
• ON → switch pin is LOW (pressed / door open / drawer open)
• OFF → switch pin is HIGH (released / door closed / drawer closed)
⭐ Switch Pins (hardware connections)
(Do not modify these unless you physically rewire the cabinet.)
const int SW1_PIN = 4;    // Door
const int SW2_PIN = 19;   // Top drawer above door
const int SW3_PIN = 13;   // Drawer behind door
const int SW4_PIN = 14;   // Drawer behind door
const int SW5_PIN = 27;   // Drawer behind door
⭐ Switch Logic Interpretation (important)
Right now, each switch uses active-LOW logic:
bool s1 = (digitalRead(SW1_PIN) == HIGH); // HIGH = released (OFF), LOW = pressed (ON)
Meaning:
• LOW (0V) → ON
• HIGH (3.3V) → OFF
If any switch acts backwards on the screen (e.g., shows ON when it should be OFF), then invert its interpretation.
⭐ How to flip a switch that reacts backward
(You normally won’t need this, but it’s good to know.)
Current logic:
bool s1 = (digitalRead(SW1_PIN) == HIGH);
To reverse it, change to:
bool s1 = (digitalRead(SW1_PIN) == LOW);
Example — flipping only SW3:
bool s3 = (digitalRead(SW3_PIN) == LOW);
⭐ Edge Detection (advanced)
Your system uses edges (changes) from SW1 & SW2 for:
• LED1 courtesy timer
• LED1 lockout reset
• Door-open events
• Drawer-open timer resets
Example:
bool s1FallingEdge = (prevS1 && !s1);  // OFF → ON
bool s1RisingEdge  = (!prevS1 && s1);  // ON → OFF
If you ever flip the logic of a switch, edge definitions must follow the same interpretation.
(Your current setup already matches active-LOW wiring correctly.)
⭐ Summary (Switch Row behavior)
• Switches are active-LOW
• OLED shows ON/OFF based on your defined logic
• SW1 and SW2 directly influence LED courtesy-timer behavior
• All switches contribute to determining LED1 brightness
• SW1 is also used for manual fan activation (3-sec press)
3) Fan Speed / RPM Row
OLED row example:
SPD: 50%  RPM:1234  T:01:45
• SPD: 50% → current fan PWM duty (0–100%).
• RPM:1234 → smoothed tach reading from the fan.
• T:01:45 → manual-fan timer only shown while manual mode is active (driven by fanOffAtMs and the manual 6-minute curve).
⭐ Humidity-Based Fan Trigger Points
The fan normally runs automatically from humidity, using this block:
if (h >= 82.0f)      { humidityDuty = 255; }
else if (h >= 80.0f) { humidityDuty = 191; }
else if (h >= 78.0f) { humidityDuty = 128; }
else if (h >= 75.0f) { humidityDuty = 64;  }
else if (h >= 74.0f) { humidityDuty = 26;  }
else if (h >= 72.0f) { humidityDuty = fanWasOn ? 26 : 0; }
else                 { humidityDuty = 0;   }
Rough percent equivalents (0–255 scale):
• 255 ≈ 100%
• 191 ≈ 75%
• 128 ≈ 50%
• 64 ≈ 25%
• 26 ≈ 10%
How to adjust behavior:
• To make the fan start earlier, lower the humidity thresholds.
Example: start strong fan at 78% instead of 82%:
if (h >= 78.0f)      { humidityDuty = 255; }
else if (h >= 76.0f) { humidityDuty = 191; }
...
• To make the fan less aggressive, lower the duty values.
Example: make the top stage ≈75% instead of 100%:
if (h >= 82.0f)      { humidityDuty = 191; }   // 75% instead of 100%
else if (h >= 80.0f) { humidityDuty = 128; }   // 50%
...
• To keep fan from chattering ON/OFF around a threshold, notice this line:
else if (h >= 72.0f) { humidityDuty = fanWasOn ? 26 : 0; }
That creates a little hysteresis:
• If fan was already ON, it keeps running down to 72% at a low speed.
• If fan was OFF, it won’t suddenly turn on at 72% unless a higher stage triggers it.
⭐ Manual Fan Start (Hold SW1 ~3 seconds)
When you hold SW1 for 3 seconds, the code starts a 6-minute fan curve:
const unsigned long FAN_MANUAL_RAMP_MS  = 1UL * 60UL * 1000UL;  // 1 minute
const unsigned long FAN_MANUAL_HOLD_MS  = 4UL * 60UL * 1000UL;  // 4 minutes
const unsigned long FAN_MANUAL_DECEL_MS = 1UL * 60UL * 1000UL;  // 1 minute

// Total manual time:
// FAN_MANUAL_TOTAL_MS =
//    (FAN_MANUAL_RAMP_MS + FAN_MANUAL_HOLD_MS + FAN_MANUAL_DECEL_MS);
Shape of the curve:
1. Ramp up (1 min): from MANUAL_MIN_PCT → MANUAL_MAX_PCT
2. Hold (4 min): at MANUAL_MAX_PCT
3. Decel (1 min): from MANUAL_MAX_PCT → MANUAL_MIN_PCT, then OFF
const float MANUAL_MIN_PCT = 5.0f;   // bottom of manual curve
const float MANUAL_MAX_PCT = 75.0f;  // top of manual curve
So you can tune:
• Softer / quieter manual mode → reduce MANUAL_MAX_PCT.
• Gentle breeze only → lower both MANUAL_MIN_PCT and MANUAL_MAX_PCT.
⭐ Minimum Running Duty (stall protection — in depth)
const int  FAN_MIN_RUN_DUTY   = 25;   // ~10% duty
const unsigned long FAN_KICK_MS = 400;
On the 0–255 scale:
• FAN_MIN_RUN_DUTY = 25 → about 10% PWM
This prevents the fan from trying to run at uselessly low duty cycles.
There is also kick-start logic that briefly runs the fan at full power to overcome static friction.
🔧 How to choose a good FAN_MIN_RUN_DUTY for your fan
1. Find the “stall point” by testing low duties.
2. Set FAN_MIN_RUN_DUTY slightly above that value.
3. Re-test humidity stages so your lowest active duty is ≥ FAN_MIN_RUN_DUTY.
4) Humidity Control Row
Your cabinet automatically adjusts the fan speed based on humidity readings from the DHT22 sensor.
The OLED indirectly reflects this by showing:
SPD: 50%   RPM:1234   T:01:45
(Where SPD is the humidity-driven or manual fan duty.)
⭐ Humidity Trigger Table (Fan Speed Based on RH%)
This block determines WHEN the fan turns on and HOW FAST it runs:
if (h >= 82.0f)      { humidityDuty = 255; }
else if (h >= 80.0f) { humidityDuty = 191; }
else if (h >= 78.0f) { humidityDuty = 128; }
else if (h >= 75.0f) { humidityDuty = 64;  }
else if (h >= 74.0f) { humidityDuty = 26;  }
else                 { humidityDuty = 0;   }
What this means in plain English:
Humidity (RH%)   Approx Fan %   Duty (0–255)   Description
≥ 82%             100%            255             Full power
≥ 80%             ~75%            191             Strong
≥ 78%             ~50%            128             Medium
≥ 75%             ~25%            64              Low/quiet
≥ 74%             ~10%            26              Minimum “breeze”
< 74%              0%             0               Fan OFF
⭐ How to adjust humidity activation points
You may change both the thresholds (RH%) and the duty values (0–255).
Example: Start the fan earlier at 70% humidity:
else if (h >= 70.0f) { humidityDuty = 26; }
Example: Make the 78% tier stronger:
else if (h >= 78.0f) { humidityDuty = 160; }
⭐ How to change “no-fan” zone
If you want the fan completely off below 72% instead of 74%:
else if (h >= 72.0f) { humidityDuty = 26; }
else                 { humidityDuty = 0; }
⭐ How to tune humidity sensitivity
Raise thresholds = fan runs less often.
Lower thresholds = fan activates sooner.
You can also simplify to a 3-speed system:
if (h >= 80.0f)      { humidityDuty = 255; }
else if (h >= 75.0f) { humidityDuty = 128; }
else if (h >= 70.0f) { humidityDuty = 64;  }
else                 { humidityDuty = 0;   }
⭐ Interaction with manual mode
Humidity control is disabled while manual fan mode is active.
Manual mode overrides:
• humidity thresholds
• minimum duty rules
• auto-off rules
When manual mode ends, humidity control resumes automatically.
⭐ Minimum Running Duty (stall protection)
Even if humidity wants a very low fan speed, the system prevents the fan from stalling:
const int FAN_MIN_RUN_DUTY = 25;
If your fan still stalls at low duty: increase FAN_MIN_RUN_DUTY.
If your fan works great at low speed: you can lower it.
Summary: fan speed increases with RH%, all values are editable, and minimum duty prevents stalling.
5) Wood Moisture Row
OLED example:
Wood Moisture: 11.2%
SM: -25488
The wood-moisture system is built around:
• ADS1115 ADC
• Differential electrode probe
• Median + EMA smoothing
• Calibration table with anchor shifting
• Nonlinear mapping from counts → % moisture
⭐ Where Moisture Is Read
Main read function:
float moistPct = readMoisturePctADS();
This returns the smoothed % moisture.
⭐ Probe Power (pulsed to prevent corrosion)
#define PROBE_PULSE 1
const int SENSOR_PWR_PIN = 2;
const unsigned long PULSE_SETTLE_MS = 300;
If PROBE_PULSE = 1, the probe is powered only during reads.
If you want the probe always powered, set PROBE_PULSE to 0.
⭐ Calibration Table (Counts → Moisture Mapping)
The code uses 8 calibration points:
const int16_t PT_COUNTS_BASE[NPTS] = {
  -25503, -25500, -25497, -25489,
  -25483, -25478, -25429, -24815
};

const float PT_PCT[NPTS] = {
  0.0f, 2.5f, 5.0f, 8.0f,
  9.0f, 12.5f, 14.5f, 17.0f
};
These describe what ADS1115 counts correspond to what % moisture.
⭐ Anchor Shifting (Your Custom Calibration)
You can shift the entire calibration curve up or down using the 2.5% anchor:
const float  USER_ANCHOR_PCT        = 2.5f;
int16_t      USER_ANCHOR_COUNTS_2P5 = -25500;
Changing USER_ANCHOR_COUNTS_2P5 shifts the whole curve.
Example:
If your wood reads wetter than reality:
USER_ANCHOR_COUNTS_2P5 = -25490;   // shifts curve upward (lower moisture %)
If readings look too dry:
USER_ANCHOR_COUNTS_2P5 = -25510;   // shifts curve downward (higher moisture %)
⭐ Smoothing Layers
1) Median filter (spike removal)
#define MEDIAN_WIN 11
Increase to 15 for extra stability; decrease to 7 for faster response.
2) EMA smoothing (gentle averaging):
#define EMA_ALPHA 0.15f
Smaller alpha → smoother, slower.
Larger alpha → more responsive but noisier.
Example:
#define EMA_ALPHA 0.25f   // quicker updates
3) Rate-limiter (prevents big jumps)
Inside readMoisturePctADS():
pct = clampDelta(pct, lastMoistPct, 0.20f);
This means moisture on the screen will change at most ±0.2% per sample.
Adjust to 0.10f for extra stability or 0.50f for more responsiveness.
⭐ Moisture Bar Graph Scaling
const float MOIST_BAR_MAX = 15.0f;
If you expect wetter wood, increase to 20.0f.
If always dry, reduce to 10.0f.
⭐ Ideal Range Markers
const float IDEAL_MC_LOW  = 8.0f;
const float IDEAL_MC_HIGH = 12.0f;
OLED shows two vertical ticks on the bar representing your ideal moisture window.
Adjust these to match your target range for your specific wood species.
