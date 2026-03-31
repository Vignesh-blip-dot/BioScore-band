# 💓 BioScore Band

> A wearable IoT health band that tracks your heart rate, HRV, motion, and steps — and turns them into a real-time **BioScore** to help you understand your body's readiness and stress levels.

![Platform](https://img.shields.io/badge/Hardware-ESP32--C3-blue)
![Firmware](https://img.shields.io/badge/Firmware-v4.3-green)
![Framework](https://img.shields.io/badge/Framework-Arduino-teal)
![License](https://img.shields.io/badge/License-MIT-yellow)

---

## 📖 What is BioScore Band?

BioScore Band is a wrist-worn health monitoring device built around the **ESP32-C3 Super Mini**. It continuously measures your biometrics and calculates a personalized wellness score — no cloud, no subscriptions, just your body talking to you.

It shows:
- 📈 A **Mini Score** (every 5 minutes) — your current stress/calm level
- 📅 A **Daily Score** — how your day is going overall
- 🔥 A **Streak** — consecutive calm windows for motivation
- 🦶 **Step Count** — daily footstrike detection
- 🫀 **Heart Rate** and **HRV (Heart Rate Variability)** live

Data streams over **Bluetooth (BLE)** to a companion web dashboard.

---

## 🛠️ Hardware Components

| Component | Description |
|-----------|-------------|
| **ESP32-C3 Super Mini** | Main microcontroller with BLE |
| **MAX30102** | Heart rate & SpO2 optical sensor |
| **MPU-6050** | 6-axis accelerometer/gyroscope for motion & steps |
| **SSD1306 OLED 0.96"** | 128×64 display for live readings |
| **TP4056** | Single-chip LiPo charger |
| **LiPo Battery** | Wearable power source |

---

## 🔌 Wiring

| Signal | ESP32-C3 Pin |
|--------|-------------|
| SDA (I2C) | GPIO 8 |
| SCL (I2C) | GPIO 9 |
| Battery ADC | GPIO 2 (voltage divider) |
| Button | GPIO 3 (active LOW, internal pull-up) |
| Power | 5V pin ← LiPo+ / TP4056 BAT+ |
| Ground | GND ← LiPo− / TP4056 BAT− |

> See [`wiring/bioscore_band_wiring_guide.html`](wiring/bioscore_band_wiring_guide.html) for the full visual wiring guide.

---

## 📁 Project Structure

```
BioScore-band/
├── firmware/
│   └── bioscore_band_FINAL_v4_3_6.ino   # Arduino firmware (ESP32-C3)
├── dashboard/
│   └── bioscore_dashboard.html           # BLE web dashboard (open in Chrome)
├── wiring/
│   └── bioscore_band_wiring_guide.html   # Visual wiring reference
└── README.md
```

---

## 🚀 Getting Started

### 1. Flash the Firmware

**Requirements:**
- [PlatformIO](https://platformio.org/) or Arduino IDE with ESP32-C3 board support
- Required libraries (see `platformio.ini` config in the `.ino` file header):
  - `SparkFun MAX3010x` — Heart rate sensor
  - `electroniccats/MPU6050` — Motion sensor
  - `Adafruit SSD1306` — OLED display
  - `Adafruit GFX Library` — Graphics

**Upload Steps:**
1. Hold the **BOOT** button on the ESP32-C3
2. Tap the **RESET** button
3. Release **BOOT**
4. Run: `pio run --target upload`

### 2. First Boot & Calibration

On first boot, the device needs **~30 minutes** of calibration (6 × 5-minute windows). During this time:
- The OLED shows `Calib: 1/6`, `Calib: 2/6`, etc.
- Scores display `--` until calibration is complete
- Wear it at rest for best baseline accuracy

### 3. Open the Dashboard

1. Open `dashboard/bioscore_dashboard.html` in **Google Chrome** (required for Web Bluetooth)
2. Click **Connect** and pair with `BioScoreBand`
3. Watch your live scores stream in!

---

## 📊 How Scoring Works

### Calibration (first 30 min)
The band learns your personal baseline HR and HRV over 6 windows.

### Mini Score (every 5 min)
```
Readiness  = 50 + (current_HRV − baseline_HRV)    → clipped 0–100
Stability  = 80 − |current_HR − baseline_HR|       → clipped 0–100
Mini Score = 0.6 × Readiness + 0.4 × Stability
```

### Daily Score
Nudged up/down each window based on Mini Score vs. neutral (60).
A **streak bonus (×1.2)** applies when 2+ consecutive calm windows occur.

### Deep Streak
Increments when: HR is calm, HRV is above baseline, and motion is low.

---

## 🖥️ OLED Screens

Press the button to cycle through 3 screens:

| Screen | Shows |
|--------|-------|
| **Screen 0 — Live** | HR, HRV, Calibration progress, Streak, Steps |
| **Screen 1 — Scores** | Mini Score, Daily Score, trend arrows, sparkline |
| **Screen 2 — System** | Battery %, Voltage, Calibration status, Baselines |

**Hold button 3 seconds** → triggers recalibration and clears all scores.

---

## 🔋 Battery Management

| Level | Action |
|-------|--------|
| < 15% | Flashing `LOW BAT` warning on OLED |
| < 5% | Auto-save + deep sleep to protect data |

Battery % is calculated from a voltage divider on GPIO2 (100kΩ : 100kΩ).

---

## 📡 BLE Characteristics

| Characteristic | UUID | Data |
|----------------|------|------|
| Mini Score | `22222222-...` | Float, notified every 5 min |
| Daily Score | `33333333-...` | Float, notified every 5 min |
| Streak | `44444444-...` | UInt8, notified every 5 min |

Service UUID: `11111111-1111-1111-1111-111111111111`

---

## 📝 Firmware Changelog

### v4.3 (Current)
- Scoring now only runs **after** calibration is complete
- `prev_daily` updated correctly — no false trend arrows after daily reset
- Dead code removed from HRV validation
- All OLED coordinates adjusted for bezel clearance (y ≥ 9)
- Light sleep disabled while BLE is connected (fixes GATT drops on ESP32-C3)
- 500ms delay on BLE connect for GATT stack stabilisation
- Boot screen shown once — no flicker
- Step counter added (peak detection algorithm)
- Screen layouts redesigned — nothing clips at display edges

---

## 🧰 Troubleshooting

| Symptom | Fix |
|---------|-----|
| OLED blank | Check SDA=GPIO8, SCL=GPIO9, VCC=3.3V |
| MAX30102 not found | Check I2C wiring; sensor address 0x57 |
| BLE disconnects immediately | Ensure `delay(500)` is present in `onConnect()` |
| Score stuck at `--` | Wait for calibration (30 min of wear) |
| Steps overcounting | Raise `STEP_THRESHOLD` to 21000–22000 in firmware |
| Battery reads wrong | Check voltage divider is exactly 100kΩ : 100kΩ |

---

## 📜 License

MIT License — feel free to use, modify, and build on this project.

---

*Built with ❤️ using ESP32-C3, Arduino framework, and a passion for personal wellness tech.*
