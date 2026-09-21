# ✨ Twinkle Bot - Smart Motion-Reactive Companion & BLE Game Console

[![Platform](https://img.shields.io/badge/Platform-ESP32--C3-blue.svg)](https://www.espressif.com/)
[![Framework](https://img.shields.io/badge/Framework-Arduino-green.svg)](https://www.arduino.cc/)
[![IDE](https://img.shields.io/badge/PlatformIO-Compatible-orange.svg)](https://platformio.org/)
[![License](https://img.shields.io/badge/License-MIT-purple.svg)](LICENSE)

**Twinkle Bot** is an interactive, motion-reactive cyber keychain and miniature game console powered by the **ESP32-C3 RISC-V microcontroller**. It features dynamic animated companion themes, a Bluetooth Low Energy (BLE) air mouse, a web-based multimedia studio for custom photo/GIF and tune uploads, and a 4-in-1 retro arcade gaming suite.

---

## 📸 Overview & Visual Layout

```text
┌─────────────────────────────────────────────────────────────┐
│                     TWINKLE BOT CONSOLE                     │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                                                       │  │
│  │   [★] TWINKLE STAR   /   [◉_◉] BLINK CYBER EYE       │  │
│  │   [⊙‿⊙] BMO BOT       /   [🎮] 4-IN-1 RETRO GAMES    │  │
│  │                                                       │  │
│  │   • ST7789 IPS Color LCD (240x135)                    │  │
│  │   • MPU6050 6-Axis Motion & Tilt Control              │  │
│  │   • BLE Air Mouse + Web MP3/GIF Studio Portal         │  │
│  │                                                       │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│       [ LEFT (GPIO 0) ]    [ OK (GPIO 8) ]    [ RIGHT (GPIO 1) ]
└─────────────────────────────────────────────────────────────┘
```

---

## 🌟 Key Features

### 1. 🎭 Interactive Animated Companions
- **Twinkle (Pink Star)**: A cheerful star that glides and reacts to real-world physics, tilts, bounces off walls, and emits glowing star sparkles.
- **Blink (Cyber Eye)**: High-tech cybernetic robot eyes with dynamic pupil gaze tracking, natural blinking, happy crescent emotes, winking, and dizzy spiral shake reactions.
- **BMO (Beemo)**: Retro console companion with smiling animations and interactive expressive reactions on button taps.

### 2. 🖱️ BLE Motion Air Mouse
- Transforms Twinkle Bot into a wireless gyroscopic PC / mobile mouse via Bluetooth Low Energy (BLE).
- Tilt left/right/up/down to smoothly glide the cursor.
- **Left Button**: Left Click
- **Right Button**: Right Click
- **Confirm Button (Tap)**: Center / Re-calibrate cursor position.

### 3. 🎵 8-Bit Chiptune Player & Web MP3 Converter
- Built-in playlist of classic chiptune tracks (*Happy Birthday*, *Twinkle Star*, *Tetris Korobeiniki*, *Cyber Bass*).
- **Web Audio Pitch Extraction Engine**: Connect to `Twinkle-Bot` WiFi hotspot (192.168.4.1) and drop any MP3, WAV, or audio file. The browser automatically extracts dominant musical frequencies and streams custom buzzer chiptunes to LittleFS flash storage!

### 4. 🖼️ Web Photo & Animated GIF Studio
- Open `http://192.168.4.1` on any smartphone or computer without installing any apps.
- Convert and stream custom 240x135 photos or multi-frame animated GIFs directly to the IPS display.

### 5. 🎮 4-in-1 Retro Arcade Gaming Suite
1. **Space Dodge**: Control your plasma-thruster cyber starship across an asteroid field using IMU tilt or buttons.
2. **Cyber Snake**: Fast-paced directional snake with high score tracking.
3. **Brick Breaker**: Breakout arcade game with smoothed accelerometer paddle physics.
4. **Geometry Dash**: Rhythm platformer with scrolling obstacles, spikes, elevated platforms, yellow jump pads, progress tracker (`%`), and chiptune bassline. Custom character transforms to match your active theme!

### 6. ⚙️ Interactive On-Device Settings
- Display brightness adjustment (0–10).
- Passive buzzer volume scaling (0–10).
- Air Mouse motion sensitivity calibration (1–10).
- LittleFS storage manager.

---

## 🔌 Hardware Components & Pinout

| Component | Pin / Interface | ESP32-C3 GPIO | Description |
|---|---|---|---|
| **ST7789 TFT LCD** | SPI MOSI | **GPIO 6** | Hardware SPI Data |
| **ST7789 TFT LCD** | SPI SCLK | **GPIO 7** | Hardware SPI Clock |
| **ST7789 TFT LCD** | CS | **GPIO 10** | Chip Select |
| **ST7789 TFT LCD** | DC | **GPIO 3** | Data / Command |
| **ST7789 TFT LCD** | RST | **GPIO 2** | Display Reset |
| **MPU6050 IMU** | I2C SDA | **GPIO 4** | Motion Sensor I2C Data |
| **MPU6050 IMU** | I2C SCL | **GPIO 5** | Motion Sensor I2C Clock |
| **Left Button** | Input Pull-Up | **GPIO 0** | Active-Low Button |
| **Right Button** | Input Pull-Up | **GPIO 1** | Active-Low Button |
| **Confirm / OK** | Input Pull-Up | **GPIO 8** | Active-Low Button |
| **Buzzer** | PWM / LEDC | **GPIO 20** | Passive Piezo Buzzer |
| **Haptic Motor** | Digital Output | **GPIO 21** | Vibration Motor Gate |

---

## 🚀 Step-by-Step Setup Guide

### 1. Prerequisites
- Install [VS Code](https://code.visualstudio.com/) and the [PlatformIO IDE Extension](https://platformio.org/install/ide?install=vscode).
- Or install the [PlatformIO Core CLI](https://docs.platformio.org/en/latest/core/installation/index.html).

### 2. Clone the Repository
```bash
git clone https://github.com/jbc-cmd/Twinkle-bot.git
cd Twinkle-bot
```

### 3. Build Firmware
Connect your ESP32-C3 board via USB and run:
```bash
pio run -e esp32-c3-devkitm-1
```

### 4. Flash / Upload to Board
```bash
pio run -e esp32-c3-devkitm-1 --target upload
```

### 5. Open Serial Monitor (Optional)
```bash
pio device monitor -b 115200
```

---

## 📖 How to Use

### Navigation
- **Left Button / Right Button**: Cycle through items, adjust values, and trigger theme interactions.
- **Confirm / OK Button (Tap)**: Select / Enter mode.
- **Hold Confirm (1.0 second)**: Return to the **Main Menu** from any mode at any time.

### Web Studio (Upload Photos, GIFs & Tunes)
1. Navigate to **Menu ➔ Tune ➔ Custom Tune** or **Menu ➔ Themes ➔ Custom Display**.
2. On your phone or PC, join the WiFi hotspot:
   * **SSID**: `Twinkle-Bot`
   * **Password**: None (Open network)
3. Open your browser to `http://192.168.4.1`.
4. Upload your favorite image, GIF, or audio track!

---

## 🛠️ Tech Stack & Architecture

- **Core**: C++ / Arduino ESP32 Framework (v3.x / ESP-IDF 5.x compatible)
- **Graphics**: `Adafruit_GFX` + `Adafruit_ST7789` with scanline line-buffer optimizations
- **Sensors**: `Adafruit_MPU6050` with auto-scanning I2C detection & low-pass filtering
- **Wireless**: Native ESP32 BLE HID (Keyboard/Mouse) & Captive Portal SoftAP WebServer + DNS
- **Storage**: `LittleFS` for persistent GIF/binary payloads & `Preferences` (NVS) for high scores

---

## 📄 License
This project is open source and available under the [MIT License](LICENSE).
