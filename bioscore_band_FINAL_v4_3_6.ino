// ================================================================
//  BioScore Band — Firmware v4.3  REVIEWED FINAL
//  Hardware: ESP32-C3 Super Mini + TP4056 (single chip) +
//            MAX30102 + MPU-6050 + SSD1306 OLED 0.96" + LiPo
//
//  ── Wiring ──────────────────────────────────────────────────────
//  SDA          → GPIO 8
//  SCL          → GPIO 9
//  Voltage div  → GPIO 2  (LiPo+ → 100kΩ → GPIO2 → 100kΩ → GND)
//  Button       → GPIO 3  (other leg → GND, internal pull-up used)
//  Power        → 5V pin  ← LiPo(+) / TP4056 BAT+ node
//  Ground       → GND     ← LiPo(−) / TP4056 BAT− node
//
//  TP4056 NOTE: Single-chip version (no DW01 protection IC).
//  BAT+ shorted to OUT+, BAT− shorted to OUT− on the module.
//  ESP32-C3 powered directly from battery via BAT+/BAT− node.
//  TP4056 handles charging only via its USB port.
//
//  ── platformio.ini ──────────────────────────────────────────────
//  [env:bioscore_band]
//  platform        = espressif32
//  board           = esp32-c3-devkitm-1
//  framework       = arduino
//  monitor_speed   = 115200
//  upload_protocol = esptool
//  upload_speed    = 115200
//  build_flags     =
//      -DARDUINO_USB_MODE=1
//      -DARDUINO_USB_CDC_ON_BOOT=1
//  lib_deps =
//      sparkfun/SparkFun MAX3010x Pulse and Proximity Sensor Library @ ^1.1.2
//      electroniccats/MPU6050 @ ^1.3.0
//      adafruit/Adafruit SSD1306 @ ^2.5.7
//      adafruit/Adafruit GFX Library @ ^1.11.9
//
//  ── Upload procedure ────────────────────────────────────────────
//  1. Hold BOOT button on ESP32-C3
//  2. Tap RESET button
//  3. Release BOOT button
//  4. Run: pio run --target upload
//
//  ── Complete change log ─────────────────────────────────────────
//  v4.3 vs v4.2:
//   FIX 1. Scoring now only runs after calibration is complete.
//   FIX 2. prev_daily updated after daily reset — no false trend ▼.
//   FIX 3. hrv_valid_window dead code removed.
//   FIX 4. All OLED y-coordinates pushed to y=9 for bezel clearance.
//   FIX 5. Light sleep disabled when BLE connected — prevents GATT
//           service discovery drops on ESP32-C3.
//   FIX 6. delay(500) in onConnect — gives GATT stack time to settle
//           before phone starts characteristic discovery.
//   FIX 7. Removed delay(1000) before BLEDevice::init() — USB CDC
//           already settled by time sensors initialise (~800ms).
//   FIX 8. Boot screen shown once only — no toggle/flicker.
//   FIX 9. Step counter added (peak detection, SPARK_SIZE=8).
//   FIX 10. Screen layouts redesigned — nothing clips at bottom.
// ================================================================

#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include "heartRate.h"
#include "MPU6050.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "esp_sleep.h"
#include "Preferences.h"

// ================================================================
//  CONFIGURATION — change values here, never touch logic below
// ================================================================

// Pins
#define SDA_PIN           8
#define SCL_PIN           9
#define BAT_PIN           2
#define BTN_PIN           3

// Display
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64
#define OLED_ADDR         0x3C
#define DISPLAY_HZ_MS     200       // OLED refresh interval (~5 Hz)

// Heart rate sensor
#define IR_THRESHOLD      20000     // min IR to attempt beat detection
#define BEAT_MIN_MS       400       // 150 BPM max
#define BEAT_MAX_MS       1500      // 40 BPM min
#define RR_ARTIFACT_MS    200       // reject RR intervals differing > this
#define RR_SIZE           120       // RR ring buffer size
#define HRV_MIN_BEATS     10        // min beats needed for RMSSD
#define HRV_INTERVAL_MS   120000    // recompute HRV every 2 minutes

// Motion
#define MOTION_THRESHOLD  1500.0f   // raw accel deviation from 1g at rest

// Scoring
#define WINDOW_MS         300000    // 5-minute scoring window
#define CALIB_WINDOWS     6         // windows to build baseline (~30 min)
#define ALPHA_HR          0.2f      // EMA smoothing — heart rate
#define ALPHA_HRV         0.15f     // EMA smoothing — HRV
#define SCORE_NEUTRAL     60.0f     // miniscore pivot (above=up, below=down)
#define SCORE_DELTA_MAX   0.5f      // max daily score change per window
#define STREAK_MULTIPLIER 1.2f      // bonus when streak >= STREAK_THRESHOLD
#define STREAK_THRESHOLD  2         // consecutive calm windows for bonus
#define HR_CALM_BAND      5.0f      // max HR deviation from baseline for calm

// Battery
#define BAT_WARN_PCT      15        // flash warning below this %
#define BAT_CRIT_PCT      5         // save + deep sleep below this %
#define BAT_CHECK_MS      30000     // battery check interval ms
#define BAT_SAMPLES       16        // ADC samples to average

// Button
#define BTN_DEBOUNCE_MS   50        // ignore presses shorter than this ms
#define BTN_HOLD_MS       3000      // hold duration for recalibration

// Daily reset
#define DAY_MS            72000000UL  // 20 hours — triggers daily score reset
#define DAY_CHECK_MS      60000       // check every 60 s

// BLE UUIDs — must match dashboard
#define SERVICE_UUID      "11111111-1111-1111-1111-111111111111"
#define MINI_UUID         "22222222-2222-2222-2222-222222222222"
#define DAILY_UUID        "33333333-3333-3333-3333-333333333333"
#define STREAK_UUID       "44444444-4444-4444-4444-444444444444"

// NVS flash storage keys
#define NVS_NS            "bioscore"
#define NVS_DAILY         "daily"
#define NVS_STREAK        "streak"
#define NVS_BL_HR         "bl_hr"
#define NVS_BL_HRV        "bl_hrv"
#define NVS_CALIBRATED    "calibrated"
#define NVS_CAL_COUNT     "cal_count"
#define NVS_CAL_HR_SUM    "cal_hr_sum"
#define NVS_CAL_HRV_SUM   "cal_hrv_sum"
#define NVS_STEPS         "steps"

// Sparkline history (Screen 1)
#define SPARK_SIZE        8

// ================================================================
//  GLOBALS
// ================================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
MAX30105         particleSensor;
MPU6050          mpu;
Preferences      prefs;

// BLE
BLEServer*         pServer     = nullptr;
BLECharacteristic* miniChar    = nullptr;
BLECharacteristic* dailyChar   = nullptr;
BLECharacteristic* streakChar  = nullptr;
bool               bleConnected = false;

// Heart rate
unsigned long lastBeat  = 0;
float         hr_smooth = 70.0f;

// HRV ring buffer
float rr_buffer[RR_SIZE];
int   rr_head  = 0;
int   rr_tail  = 0;
int   rr_count = 0;

unsigned long lastHRVCalc = 0;
float         hrv_smooth  = 40.0f;

// 5-minute window accumulators
unsigned long lastWindow  = 0;
float  hr_sum             = 0.0f;
float  hrv_sum            = 0.0f;
float  motion_sum         = 0.0f;
int    hr_samples         = 0;
int    hrv_samples        = 0;
int    motion_samples     = 0;

// Scores (loaded from NVS on boot)
float miniscore      = 50.0f;
float daily_score    = 50.0f;
int   deep_streak    = 0;
float prev_miniscore = 50.0f;   // synced from NVS — correct trend arrows on boot
float prev_daily     = 50.0f;   // synced from NVS — correct trend arrows on boot

// Personal calibration baselines
float baseline_hr   = 75.0f;
float baseline_hrv  = 55.0f;
float calib_hr_sum  = 0.0f;
float calib_hrv_sum = 0.0f;
int   calib_count   = 0;
bool  calibrated    = false;

// Battery
int lastBatPct    = 100;

// Display
int currentScreen = 0;   // 0=live  1=scores  2=system

// Sparkline
float sparkBuf[SPARK_SIZE];
int   sparkIdx   = 0;
int   sparkCount = 0;

// Daily reset timer
unsigned long dayResetAt = 0;

// ================================================================
//  STEP COUNTER
//  Peak detection on accelerometer magnitude.
//  A step is counted when magnitude crosses STEP_THRESHOLD upward
//  then falls back below it — one full footstrike cycle.
//  STEP_THRESHOLD tuned for wrist wear at walking pace.
// ================================================================
#define STEP_THRESHOLD    20500.0f  // raw accel magnitude threshold
                                    // 16384 = 1g at rest, ~20500 filters arm
                                    // movements and captures only real footstrikes
                                    // Raise further (21000-22000) if still overcounting
#define STEP_COOLDOWN_MS  350       // min ms between steps (max ~2.8 steps/s)
                                    // 350ms = comfortable walking cadence filter

int           stepCount     = 0;
bool          stepHigh      = false;   // true = magnitude currently above threshold
unsigned long lastStepTime  = 0;

// ================================================================
//  FUNCTION DECLARATIONS
// ================================================================
void  nvsLoad();
void  nvsSave();
void  nvsResetCalibration();
float readBatteryVoltage();
int   batteryPercent(float v);
void  checkBatteryCritical();
void  checkDailyReset();
float computeRMSSD();
void  handleButton();
void  sparkPush(float val);
void  drawBatteryIcon(int x, int y, int pct);
void  drawTrend(int x, int y, float delta);
void  drawSparkline(int x, int y, int w, int h);
void  updateDisplay();
void  showBootScreen(bool bleReady);
void  oledError(const char* line1, const char* line2);

// ================================================================
//  BLE CALLBACKS
// ================================================================
class BLECallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
    // delay(500) lets the GATT service discovery complete on the
    // phone side before the ESP32-C3 resumes heavy I2C sensor reads.
    // Without this the connection drops immediately on ESP32-C3.
    delay(500);
    Serial.println("[BLE] Phone connected.");
  }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Disconnected — restarting advertising.");
  }
};

// ================================================================
//  NVS — non-volatile storage
//  Persists daily score, streak, baselines across reboots/swaps
// ================================================================
void nvsLoad()
{
  prefs.begin(NVS_NS, true);
  daily_score   = prefs.getFloat(NVS_DAILY,      50.0f);
  deep_streak   = prefs.getInt  (NVS_STREAK,      0);
  baseline_hr   = prefs.getFloat(NVS_BL_HR,       75.0f);
  baseline_hrv  = prefs.getFloat(NVS_BL_HRV,      55.0f);
  calibrated    = prefs.getBool (NVS_CALIBRATED,   false);
  calib_count   = prefs.getInt  (NVS_CAL_COUNT,    0);
  calib_hr_sum  = prefs.getFloat(NVS_CAL_HR_SUM,  0.0f);
  calib_hrv_sum = prefs.getFloat(NVS_CAL_HRV_SUM, 0.0f);
  stepCount     = prefs.getInt  (NVS_STEPS,         0);
  prefs.end();

  // Sync prev values — prevents false trend arrows on first window after reboot
  prev_miniscore = miniscore;
  prev_daily     = daily_score;

  Serial.printf("[NVS] daily=%.1f streak=%d steps=%d calib=%s bl_hr=%.1f bl_hrv=%.1f\n",
                daily_score, deep_streak, stepCount,
                calibrated ? "DONE" : "NEEDED",
                baseline_hr, baseline_hrv);
}

void nvsSave()
{
  prefs.begin(NVS_NS, false);
  prefs.putFloat(NVS_DAILY,       daily_score);
  prefs.putInt  (NVS_STREAK,      deep_streak);
  prefs.putFloat(NVS_BL_HR,       baseline_hr);
  prefs.putFloat(NVS_BL_HRV,      baseline_hrv);
  prefs.putBool (NVS_CALIBRATED,  calibrated);
  prefs.putInt  (NVS_CAL_COUNT,   calib_count);
  prefs.putFloat(NVS_CAL_HR_SUM,  calib_hr_sum);
  prefs.putFloat(NVS_CAL_HRV_SUM, calib_hrv_sum);
  prefs.putInt  (NVS_STEPS,       stepCount);
  prefs.end();
  Serial.println("[NVS] Saved.");
}

void nvsResetCalibration()
{
  calibrated    = false;
  calib_count   = 0;
  calib_hr_sum  = 0.0f;
  calib_hrv_sum = 0.0f;
  baseline_hr   = 75.0f;
  baseline_hrv  = 55.0f;
  // Also reset scores to neutral so stale scores from old baselines are cleared
  miniscore   = 50.0f;
  daily_score = 50.0f;
  deep_streak = 0;
  stepCount   = 0;
  prev_miniscore = 50.0f;
  prev_daily     = 50.0f;
  nvsSave();
  Serial.println("[NVS] Calibration reset — scores cleared.");
}

// ================================================================
//  BATTERY
// ================================================================
float readBatteryVoltage()
{
  // Average BAT_SAMPLES reads to reduce ADC noise
  long raw = 0;
  for (int i = 0; i < BAT_SAMPLES; i++) raw += analogRead(BAT_PIN);
  raw /= BAT_SAMPLES;
  // 12-bit ADC (0-4095), 3.3V ref, 1:1 divider → ×2
  return (raw / 4095.0f) * 3.3f * 2.0f;
}

int batteryPercent(float v)
{
  // LiPo: 3.3V = 0%, 4.2V = 100%
  return constrain((int)((v - 3.3f) / (4.2f - 3.3f) * 100.0f), 0, 100);
}

void checkBatteryCritical()
{
  if (lastBatPct > BAT_CRIT_PCT) return;
  nvsSave();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(8, 20);
  display.print("Battery critical!");
  display.setCursor(14, 34);
  display.print("Saving & sleeping");
  display.setCursor(30, 48);
  display.print("Please charge.");
  display.display();
  delay(2500);
  esp_deep_sleep_start();
}

// ================================================================
//  DAILY RESET
//  Resets daily_score to 50 (neutral) after 20 hours of uptime.
//  Streak is preserved. Checked every 60 s.
// ================================================================
void checkDailyReset()
{
  if (millis() - dayResetAt < DAY_MS) return;
  dayResetAt  = millis();
  daily_score = 50.0f;
  prev_daily  = 50.0f;
  stepCount   = 0;       // reset steps at start of new day
  nvsSave();
  Serial.println("[DAY] Daily score reset for new day.");
}

// ================================================================
//  RMSSD — Heart Rate Variability metric
//  Iterates ring buffer in insertion order using modular arithmetic.
//  High RMSSD (>40ms) = parasympathetic = recovered/calm
//  Low  RMSSD (<30ms) = sympathetic     = stressed/fatigued
// ================================================================
float computeRMSSD()
{
  if (rr_count < 2) return 0.0f;
  float sum    = 0.0f;
  int   n      = 0;
  int prev_idx = rr_head;
  int curr_idx = (rr_head + 1) % RR_SIZE;
  for (int i = 1; i < rr_count; i++) {
    float diff = rr_buffer[curr_idx] - rr_buffer[prev_idx];
    sum += diff * diff;
    n++;
    prev_idx = curr_idx;
    curr_idx = (curr_idx + 1) % RR_SIZE;
  }
  return (n > 0) ? sqrtf(sum / n) : 0.0f;
}

// ================================================================
//  BUTTON
//  Short press  → cycle screens (0 → 1 → 2 → 0)
//  Hold 3 s     → recalibrate + clear scores (visual flash)
//  Debounce     → ignore presses < BTN_DEBOUNCE_MS
// ================================================================
unsigned long btnPressTime = 0;
bool          btnWasDown   = false;
bool          btnHoldFired = false;

void handleButton()
{
  bool down = (digitalRead(BTN_PIN) == LOW);   // active LOW

  if (down && !btnWasDown) {
    btnPressTime = millis();
    btnWasDown   = true;
    btnHoldFired = false;
  }

  if (down && btnWasDown && !btnHoldFired) {
    if (millis() - btnPressTime >= BTN_HOLD_MS) {
      btnHoldFired = true;
      nvsResetCalibration();
      display.invertDisplay(true);
      delay(500);
      display.invertDisplay(false);
      Serial.println("[BTN] Hold — recalibration triggered.");
    }
  }

  if (!down && btnWasDown) {
    unsigned long held = millis() - btnPressTime;
    btnWasDown = false;
    if (held >= BTN_DEBOUNCE_MS && !btnHoldFired) {
      currentScreen = (currentScreen + 1) % 3;
      Serial.printf("[BTN] Screen → %d\n", currentScreen);
    }
  }
}

// ================================================================
//  SPARKLINE
// ================================================================
void sparkPush(float val)
{
  sparkBuf[sparkIdx] = val;
  sparkIdx = (sparkIdx + 1) % SPARK_SIZE;
  if (sparkCount < SPARK_SIZE) sparkCount++;
}

void drawSparkline(int x, int y, int w, int h)
{
  if (sparkCount < 2) return;
  float mn = 100.0f, mx = 0.0f;
  for (int i = 0; i < sparkCount; i++) {
    mn = min(mn, sparkBuf[i]);
    mx = max(mx, sparkBuf[i]);
  }
  if (mx - mn < 1.0f) mx = mn + 1.0f;
  float range = mx - mn;
  int   start = (sparkIdx - sparkCount + SPARK_SIZE) % SPARK_SIZE;
  float px = -1.0f, py = -1.0f;
  for (int i = 0; i < sparkCount; i++) {
    int   idx = (start + i) % SPARK_SIZE;
    float nx  = x + (float)i / (sparkCount - 1) * (w - 1);
    float ny  = y + h - 1 - ((sparkBuf[idx] - mn) / range) * (h - 1);
    if (i > 0) display.drawLine((int)px, (int)py, (int)nx, (int)ny, SSD1306_WHITE);
    px = nx; py = ny;
  }
}

// ================================================================
//  DISPLAY HELPERS
// ================================================================

// Battery icon 16×7px — fill level + X mark when low
void drawBatteryIcon(int x, int y, int pct)
{
  display.drawRect(x, y, 14, 7, SSD1306_WHITE);
  display.fillRect(x + 14, y + 2, 2, 3, SSD1306_WHITE);   // positive terminal
  int fill = (int)(pct / 100.0f * 12.0f);
  if (fill > 0) display.fillRect(x + 1, y + 1, fill, 5, SSD1306_WHITE);
  if (pct <= BAT_WARN_PCT) {
    display.drawLine(x + 2, y + 1, x + 11, y + 5, SSD1306_WHITE);
    display.drawLine(x + 11, y + 1, x + 2,  y + 5, SSD1306_WHITE);
  }
}

// Trend arrow: ▲ up (delta>1) / — flat / ▼ down (delta<-1)
void drawTrend(int x, int y, float delta)
{
  if      (delta >  1.0f) display.fillTriangle(x+3,y,   x,y+5, x+6,y+5, SSD1306_WHITE);
  else if (delta < -1.0f) display.fillTriangle(x,  y,   x+6,y, x+3,y+5, SSD1306_WHITE);
  else                    display.fillRect    (x,  y+2,  7, 2,            SSD1306_WHITE);
}

// Error screen — shown on sensor init failure
void oledError(const char* line1, const char* line2)
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 20);
  display.print(line1);
  display.setCursor(2, 36);
  display.print(line2);
  display.display();
}

// ================================================================
//  BOOT SCREEN
//  bleReady=false → "BLE: Starting..."
//  bleReady=true  → "BLE: Advertising"
//  Called twice in setup() — full redraw each time (no partial update)
// ================================================================
void showBootScreen(bool bleReady)
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(8, 9);
  display.print("BioScore Band v4.3");
  display.drawFastHLine(0, 19, 128, SSD1306_WHITE);

  display.setCursor(2, 23);
  display.print("Daily:  ");
  display.print((int)daily_score);

  display.setCursor(2, 33);
  display.print("Streak: ");
  display.print(deep_streak);

  display.setCursor(2, 43);
  display.print(calibrated ? "Calib:  Loaded" : "Calib:  Needed");

  display.setCursor(2, 53);
  display.print(bleReady ? "BLE:    Advertising" : "BLE:    Starting...");

  display.display();
}

// ================================================================
//  MAIN DISPLAY
//
//  Safe pixel area: x 2–126, y 9–54 (conservative — avoids all edges)
//  First row at y=9 — this module has ~8px physical bezel at top.
//
//  Screen 0 — Live:   HR, HRV, Calib status, Streak
//  Screen 1 — Scores: Mini (large), Daily (large), sparkline
//  Screen 2 — System: Battery, Calib, Base HR, Base HRV
// ================================================================
void updateDisplay()
{
  float battV = readBatteryVoltage();
  int   battP = batteryPercent(battV);
  lastBatPct  = battP;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // ── Low battery warning — overrides all screens ─────────────────
  static bool          flashOn   = false;
  static unsigned long lastFlash = 0;
  if (battP <= BAT_WARN_PCT && battP > BAT_CRIT_PCT) {
    if (millis() - lastFlash >= 800) { flashOn = !flashOn; lastFlash = millis(); }
    if (flashOn) {
      display.setCursor(14, 18);
      display.setTextSize(2);
      display.print("LOW BAT");
      display.setTextSize(1);
      display.setCursor(24, 46);
      display.print(battP);
      display.print("% remaining");
      display.display();
      return;
    }
  }

  // ================================================================
  //  SCREEN 0 — Live sensor view
  //  Shows: HR, HRV, CAL progress, Streak
  //  Mini/Daily removed — they live on Screen 1
  //  No progress bar — avoids bottom-edge clipping
  //  4 rows evenly spaced between y=9 and y=52
  // ================================================================
  if (currentScreen == 0)
  {
    // Header: HR | battery icon
    display.setCursor(2, 9);
    display.print("HR ");
    display.print((int)hr_smooth);
    display.print(" bpm");
    drawBatteryIcon(100, 9, battP);

    // Row 1: HRV
    display.setCursor(2, 22);
    display.print("HRV ");
    display.print((int)hrv_smooth);
    display.print(" ms");

    // Row 2: Calibration status
    display.setCursor(2, 35);
    if (calibrated) {
      display.print("Calib: DONE");
    } else {
      display.print("Calib: ");
      display.print(calib_count);
      display.print("/");
      display.print(CALIB_WINDOWS);
    }

    // Row 3: Streak and Steps on same line
    display.setCursor(2, 48);
    display.print("Streak:");
    display.print(calibrated ? deep_streak : 0);
    display.setCursor(64, 48);
    display.print("Steps:");
    display.print(stepCount);
  }

  // ================================================================
  //  SCREEN 1 — Score view
  //  Shows: Mini (large), Daily (large), trend arrows, sparkline
  //  This is the main score screen
  // ================================================================
  else if (currentScreen == 1)
  {
    // Mini score — large
    display.setCursor(2, 9);
    display.print("Mini");
    display.setCursor(50, 9);
    display.setTextSize(2);
    if (calibrated) {
      display.print((int)miniscore);
      display.setTextSize(1);
      display.print("/100");
      drawTrend(110, 11, miniscore - prev_miniscore);
    } else {
      display.print("--");
      display.setTextSize(1);
    }
    display.setTextSize(1);

    // Daily score — large
    display.setCursor(2, 28);
    display.print("Daily");
    display.setCursor(50, 28);
    display.setTextSize(2);
    if (calibrated) {
      display.print((int)daily_score);
      display.setTextSize(1);
      display.print("/100");
      drawTrend(110, 30, daily_score - prev_daily);
    } else {
      display.print("--");
      display.setTextSize(1);
    }
    display.setTextSize(1);

    // Divider and sparkline
    display.drawFastHLine(0, 48, 128, SSD1306_WHITE);
    drawSparkline(2, 51, 124, 10);
  }

  // ================================================================
  //  SCREEN 2 — System info
  //  Shows: Battery, Calib status, Base HR, Base HRV
  //  "Hold btn: recalib" removed — was getting cut at bottom
  //  Recalibration still works via hold button
  // ================================================================
  else if (currentScreen == 2)
  {
    // Title
    display.setCursor(2, 9);
    display.print("-- System Info --");

    // Battery
    display.setCursor(2, 21);
    display.print("Bat: ");
    display.print(battP);
    display.print("% (");
    display.print(battV, 2);
    display.print("V)");

    // Calibration
    display.setCursor(2, 32);
    if (calibrated) {
      display.print("Calib: DONE");
    } else {
      display.print("Calib: ");
      display.print(calib_count);
      display.print("/");
      display.print(CALIB_WINDOWS);
    }

    // Base HR
    display.setCursor(2, 43);
    display.print("Base HR:  ");
    display.print(baseline_hr, 1);

    // Base HRV
    display.setCursor(2, 54);
    display.print("Base HRV: ");
    display.print(baseline_hrv, 1);
  }

  display.display();
}

// ================================================================
//  SETUP
// ================================================================
void setup()
{
  Serial.begin(115200);

  // ESP32-C3 native USB CDC: wait for serial monitor to open.
  // Without this all startup output is lost before monitor connects.
  // Holds up to 5 seconds then continues — band works without PC too.
  unsigned long _t = millis();
  while (!Serial && millis() - _t < 5000);
  delay(200);

  Serial.println("\n╔══════════════════════════╗");
  Serial.println(  "║  BioScore Band  v4.3     ║");
  Serial.println(  "╚══════════════════════════╝");

  // ── GPIO ────────────────────────────────────────────────────────
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);           // 400 kHz — needed for 3 I2C + 5Hz OLED

  pinMode(BAT_PIN, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // allows reading up to ~3.9V

  pinMode(BTN_PIN, INPUT_PULLUP);  // active LOW — button pulls GPIO3 to GND

  // ── NVS load — must happen before boot screen ──────────────────
  nvsLoad();

  // ── OLED ────────────────────────────────────────────────────────
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[ERR] OLED not found — check SDA=G8 SCL=G9 VCC=3V3");
    while (1) delay(1000);
  }
  display.setTextColor(SSD1306_WHITE);
  Serial.println("[OK] OLED");

  // ── MAX30102 ────────────────────────────────────────────────────
  if (!particleSensor.begin(Wire)) {
    Serial.println("[ERR] MAX30102 not found — check wiring");
    oledError("  MAX30102 FAIL!", "Check SDA/SCL/VCC");
    while (1) delay(1000);
  }
  // ledBrightness=60, sampleAvg=4, ledMode=2(red+IR),
  // sampleRate=100Hz, pulseWidth=411us, adcRange=4096
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  Serial.println("[OK] MAX30102");

  // ── MPU-6050 ────────────────────────────────────────────────────
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[WARN] MPU6050 not responding — check AD0=GND");
  } else {
    Serial.println("[OK]  MPU6050");
  }

  // Show boot screen — BLE init happens immediately after
  // No delay needed — USB CDC is already settled by the time
  // Wire, OLED, MAX30102 and MPU6050 have all initialised (~800ms)
  showBootScreen(false);

  // ── BLE ─────────────────────────────────────────────────────────
  BLEDevice::init("BioScoreBand");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BLECallbacks());

  BLEService* pSvc = pServer->createService(SERVICE_UUID);

  miniChar = pSvc->createCharacteristic(MINI_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  dailyChar = pSvc->createCharacteristic(DAILY_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  streakChar = pSvc->createCharacteristic(STREAK_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  miniChar->addDescriptor(new BLE2902());
  dailyChar->addDescriptor(new BLE2902());
  streakChar->addDescriptor(new BLE2902());

  pSvc->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->start();

  Serial.println("[OK] BLE advertising as BioScoreBand");

  // Show final boot screen once — no toggle, no flicker
  showBootScreen(true);

  // ── Timers ──────────────────────────────────────────────────────
  lastWindow  = millis();
  lastHRVCalc = millis();
  dayResetAt  = millis();

  Serial.println("[OK] System ready.\n");
}

// ================================================================
//  LOOP
// ================================================================
void loop()
{
  unsigned long now = millis();

  // ── Button ──────────────────────────────────────────────────────
  handleButton();

  // ── Battery critical check — every 30 s ─────────────────────────
  static unsigned long lastBatCheck = 0;
  if (now - lastBatCheck >= BAT_CHECK_MS) {
    lastBatCheck = now;
    lastBatPct   = batteryPercent(readBatteryVoltage());
    checkBatteryCritical();
  }

  // ── Daily reset — every 60 s ─────────────────────────────────────
  static unsigned long lastDayCheck = 0;
  if (now - lastDayCheck >= DAY_CHECK_MS) {
    lastDayCheck = now;
    checkDailyReset();
  }

  // ── Heart rate from MAX30102 ────────────────────────────────────
  long irValue = particleSensor.getIR();

  if (irValue >= IR_THRESHOLD && checkForBeat(irValue))
  {
    unsigned long delta = now - lastBeat;
    lastBeat = now;

    if (delta > BEAT_MIN_MS && delta < BEAT_MAX_MS)
    {
      float bpm = 60000.0f / delta;
      hr_smooth = ALPHA_HR * bpm + (1.0f - ALPHA_HR) * hr_smooth;
      hr_sum   += hr_smooth;
      hr_samples++;

      // Artifact rejection — skip RR if too different from previous
      bool valid_rr = true;
      if (rr_count > 0) {
        int   prev_idx = (rr_tail - 1 + RR_SIZE) % RR_SIZE;
        float prev_rr  = rr_buffer[prev_idx];
        if (fabsf((float)delta - prev_rr) > RR_ARTIFACT_MS) valid_rr = false;
      }

      if (valid_rr) {
        rr_buffer[rr_tail] = (float)delta;
        rr_tail = (rr_tail + 1) % RR_SIZE;
        if (rr_count < RR_SIZE) rr_count++;
        else                    rr_head = (rr_head + 1) % RR_SIZE;
      }
    }
  }

  // ── Motion + Step counting from MPU-6050 ───────────────────────
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  // Cast to long before squaring — prevents int16_t overflow
  float magnitude = sqrtf((float)((long)ax*ax + (long)ay*ay + (long)az*az));
  float motion    = fabsf(magnitude - 16384.0f);   // deviation from 1g
  motion_sum     += motion;
  motion_samples++;

  // Step detection — peak crossing algorithm
  // A step = magnitude rises above STEP_THRESHOLD then falls back below it
  // STEP_COOLDOWN_MS prevents double-counting a single footstrike
  if (magnitude > STEP_THRESHOLD) {
    stepHigh = true;
  } else if (stepHigh) {
    // Falling edge — magnitude just dropped back below threshold
    stepHigh = false;
    if (now - lastStepTime >= STEP_COOLDOWN_MS) {
      stepCount++;
      lastStepTime = now;
    }
  }

  // ── HRV — recomputed every 2 minutes ───────────────────────────
  if (now - lastHRVCalc >= HRV_INTERVAL_MS)
  {
    lastHRVCalc = now;

    if (rr_count > HRV_MIN_BEATS) {
      float rmssd = constrain(computeRMSSD(), 0.0f, 200.0f);
      hrv_smooth  = ALPHA_HRV * rmssd + (1.0f - ALPHA_HRV) * hrv_smooth;
      hrv_sum    += hrv_smooth;
      hrv_samples++;
    }
    // If rr_count too low (loose band): hrv_smooth keeps last valid value.
    // Scoring uses it as fallback rather than skipping the window.

    // Reset RR buffer for next 2-minute window
    rr_head = rr_tail = rr_count = 0;
  }

  // ── 5-minute scoring window ─────────────────────────────────────
  if (now - lastWindow >= WINDOW_MS)
  {
    lastWindow = now;

    if (hr_samples > 0 && motion_samples > 0)
    {
      float avg_hr     = hr_sum     / hr_samples;
      float avg_motion = motion_sum / motion_samples;

      // HRV: use measured if available, else last known value (fallback)
      float avg_hrv;
      if (hrv_samples > 0) {
        avg_hrv = hrv_sum / hrv_samples;
      } else {
        avg_hrv = hrv_smooth;
        Serial.println("[HRV] Fallback used — band may be loose.");
      }

      // ── Calibration (first 30 min = 6 windows × 5 min) ─────────
      // FIX 1: scoring only runs AFTER calibration is complete.
      // During calibration windows accumulators build baselines only.
      // Scores show "--" on the display until CAL 6/6 is reached.
      if (!calibrated) {
        calib_hr_sum  += avg_hr;
        calib_hrv_sum += avg_hrv;
        calib_count++;
        Serial.printf("[CAL] Window %d/%d — HR=%.1f HRV=%.1f\n",
                      calib_count, CALIB_WINDOWS, avg_hr, avg_hrv);
        if (calib_count >= CALIB_WINDOWS) {
          baseline_hr  = calib_hr_sum  / calib_count;
          baseline_hrv = calib_hrv_sum / calib_count;
          calibrated   = true;
          // Compute first meaningful scores at the moment cal completes
          float r0 = constrain(50.0f + (avg_hrv - baseline_hrv), 0.0f, 100.0f);
          float s0 = constrain(80.0f - fabsf(avg_hr - baseline_hr), 0.0f, 100.0f);
          miniscore      = 0.6f * r0 + 0.4f * s0;
          daily_score    = 50.0f;
          prev_miniscore = miniscore;
          prev_daily     = daily_score;
          sparkPush(miniscore);
          Serial.printf("[CAL] Done — HR=%.1f HRV=%.1f  first mini=%.1f\n",
                        baseline_hr, baseline_hrv, miniscore);
        }
        // Save after every calibration window — partial progress survives reboots
        nvsSave();
      }
      else
      {
        // ── Scoring — only runs after calibration is complete ─────
        float readiness = constrain(50.0f + (avg_hrv - baseline_hrv), 0.0f, 100.0f);
        float stability = constrain(80.0f - fabsf(avg_hr - baseline_hr), 0.0f, 100.0f);

        prev_miniscore = miniscore;
        prev_daily     = daily_score;

        miniscore = 0.6f * readiness + 0.4f * stability;
        sparkPush(miniscore);

        // ── Deep streak ───────────────────────────────────────────
        bool calm = (fabsf(avg_hr - baseline_hr) < HR_CALM_BAND &&
                     avg_hrv   >= baseline_hrv                   &&
                     avg_motion < MOTION_THRESHOLD);
        if (calm) deep_streak++;
        else       deep_streak = 0;

        // ── Daily score ───────────────────────────────────────────
        // miniscore > 60 nudges daily up, below nudges down
        // streak >= 2 adds 20% bonus multiplier
        float delta_score = ((miniscore - SCORE_NEUTRAL) / SCORE_NEUTRAL) * SCORE_DELTA_MAX;
        if (deep_streak >= STREAK_THRESHOLD) delta_score *= STREAK_MULTIPLIER;
        daily_score = constrain(daily_score + delta_score, 0.0f, 100.0f);

        Serial.printf(
          "[WIN] Mini=%.1f  Daily=%.1f  Streak=%d  "
          "HR=%.1f  HRV=%.1f  Motion=%.0f  HRV=%s\n",
          miniscore, daily_score, deep_streak,
          avg_hr, avg_hrv, avg_motion,
          (hrv_samples > 0) ? "measured" : "fallback"
        );

        // ── BLE notify ────────────────────────────────────────────
        if (bleConnected) {
          miniChar->setValue(miniscore);    miniChar->notify();
          dailyChar->setValue(daily_score); dailyChar->notify();
          uint8_t sb = (uint8_t)constrain(deep_streak, 0, 255);
          streakChar->setValue(&sb, 1);     streakChar->notify();
        }

        // ── NVS save — once per window ────────────────────────────
        nvsSave();
      }
    }

    // Reset all accumulators for next window
    hr_sum     = hrv_sum     = motion_sum     = 0.0f;
    hr_samples = hrv_samples = motion_samples = 0;
  }

  // ── Display ~5 Hz ───────────────────────────────────────────────
  static unsigned long lastDisp = 0;
  if (now - lastDisp >= DISPLAY_HZ_MS) {
    lastDisp = now;
    updateDisplay();
  }

  // ── Light sleep 10 ms ───────────────────────────────────────────
  // Only sleep when BLE is not connected — light sleep can interrupt
  // GATT service discovery and cause connection drops on ESP32-C3
  if (!bleConnected) {
    esp_sleep_enable_timer_wakeup(10000);
    esp_light_sleep_start();
  }
}
