// Twinkle: motion-reactive ESP32-C3 keychain
// ─────────────────────────────────────────────────────────────────────────────
// Libraries (added automatically via platformio.ini lib_deps):
//   Adafruit GFX Library, Adafruit ST7735 and ST7789 Library,
//   Adafruit MPU6050, Adafruit Unified Sensor, Adafruit BusIO

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <BLESecurity.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include "star_sprites.h"
#include "driver/gpio.h"   // for gpio_reset_pin()

// ---- Twinkle wiring ---------------------------------------------------------
constexpr int PIN_TFT_MOSI = 6;
constexpr int PIN_TFT_SCLK = 7;
constexpr int PIN_TFT_CS   = 10;
constexpr int PIN_TFT_DC   = 3;
constexpr int PIN_TFT_RST  = 2;
constexpr int PIN_TFT_BL   = -1; // Set to GPIO pin if TFT BL is wired to a GPIO, or -1 if wired to 3.3V
constexpr int PIN_I2C_SDA  = 4;
constexpr int PIN_I2C_SCL  = 5;
constexpr int PIN_LEFT     = 0;   // each button connects its pin to GND
constexpr int PIN_RIGHT    = 1;
constexpr int PIN_CONFIRM  = 8;
constexpr int PIN_BUZZER   = 20;  // passive buzzer (use a transistor if needed)
constexpr int PIN_MOTOR    = 21;  // gate/base of the motor driver, not the motor itself

Adafruit_ST7789 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
bool mpuFound = false;
uint8_t mpuAddress = 0x68;
int currentSDA = PIN_I2C_SDA;
int currentSCL = PIN_I2C_SCL;

bool initMPU(uint8_t addr) {
  // Test if device responds on I2C
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0) return false;

  // Wake up MPU (PWR_MGMT_1 = 0x00)
  Wire.beginTransmission(addr);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Wake up
  if (Wire.endTransmission() != 0) return false;
  delay(15);

  // Set clock source to PLL with X axis gyroscope reference (PWR_MGMT_1 = 0x01)
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(10);

  // Set Gyro full scale range to ±500 deg/s (GYRO_CONFIG = 0x08)
  Wire.beginTransmission(addr);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Set Accel full scale range to ±8g (ACCEL_CONFIG = 0x10)
  Wire.beginTransmission(addr);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Set DLPF to ~21Hz (CONFIG = 0x03)
  Wire.beginTransmission(addr);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  mpuAddress = addr;
  return true;
}

bool initMPUOnPins(int sda, int scl) {
  Wire.end();
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  Wire.begin(sda, scl, 50000); // 50kHz for high signal tolerance
  Wire.setTimeOut(20);
  delay(20);

  // Check standard MPU6050/6500 addresses first
  if (initMPU(0x68)) { currentSDA = sda; currentSCL = scl; return true; }
  if (initMPU(0x69)) { currentSDA = sda; currentSCL = scl; return true; }

  // Scan all addresses
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (initMPU(addr)) {
        currentSDA = sda;
        currentSCL = scl;
        return true;
      }
    }
  }
  return false;
}

bool scanAllPinsForMPU() {
  // Only probe hardware I2C pins (GPIO 4 and GPIO 5)
  if (initMPUOnPins(PIN_I2C_SDA, PIN_I2C_SCL)) return true;
  if (initMPUOnPins(PIN_I2C_SCL, PIN_I2C_SDA)) return true;
  return false;
}

bool readMPU(sensors_event_t *accel, sensors_event_t *gyro) {
  if (!mpuFound) return false;

  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3B); // starting register ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)mpuAddress, 14) != 14) return false;

  int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t az = (int16_t)((Wire.read() << 8) | Wire.read());
  (void)((Wire.read() << 8) | Wire.read()); // temp
  int16_t gx = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t gy = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t gz = (int16_t)((Wire.read() << 8) | Wire.read());

  // Accel ±8g: 4096 LSB/g -> m/s^2
  accel->acceleration.x = (ax / 4096.0f) * 9.80665f;
  accel->acceleration.y = (ay / 4096.0f) * 9.80665f;
  accel->acceleration.z = (az / 4096.0f) * 9.80665f;

  // Gyro ±500 dps: 65.5 LSB/(deg/s) -> rad/s
  gyro->gyro.x = (gx / 65.5f) * (3.14159265f / 180.0f);
  gyro->gyro.y = (gy / 65.5f) * (3.14159265f / 180.0f);
  gyro->gyro.z = (gz / 65.5f) * (3.14159265f / 180.0f);

  return true;
}

constexpr int SCREEN_W = 240;
constexpr int SCREEN_H = 135;
constexpr uint16_t SKY    = 0x0012;
constexpr uint16_t STAR   = 0xFFE0;
constexpr uint16_t PINK   = 0xF99F;
constexpr uint16_t MINT   = 0x7FF5;
constexpr uint16_t SPACE  = 0x000D;
constexpr uint16_t NEBULA = 0x181D;
constexpr uint16_t GLOW   = 0xF9BE;

// ─── Theme palettes ──────────────────────────────────────────────────────────
// 0 = Twinkle (Cute Pink Star), 1 = Blink (Neon Cyan Cyber Eyes), 2 = BMO (Adventure Time Beemo)
uint8_t currentTheme = 0;

// Twinkle Theme (Pink Star)
constexpr uint16_t STAR_PINK    = 0xFDF2; // Bright pastel star pink (#FFAEC9)
constexpr uint16_t STAR_ROSE    = 0xFBEF; // Soft blossom pink
constexpr uint16_t STAR_BERRY   = 0xC20F; // Deep berry outline/border
constexpr uint16_t STAR_BG_CARD = 0x3067; // Dark rose card fill
constexpr uint16_t STAR_GOLD    = 0xFFE0; // Star gold
constexpr uint16_t STAR_MUTED   = 0xD555; // Muted lavender-pink text

// Blink Theme (Cyan Cyber Eyes - matching user reference photo)
constexpr uint16_t BLINK_CYAN       = 0x07FF; // Vibrant neon cyan (#00FFFF)
constexpr uint16_t BLINK_GLOW       = 0x0375; // Glowing cyan halo / border
constexpr uint16_t BLINK_DARK_TEAL  = 0x0188; // Deep obsidian teal card fill
constexpr uint16_t BLINK_TEXT_MUTED = 0x7E39; // Cyber aqua muted text

// BMO Theme (Beemo - exact pastel mint green matching reference photo)
constexpr uint16_t BMO_MINT_BG     = 0xC758; // Pastel mint green (#C3EAC7)
constexpr uint16_t BMO_BLACK       = 0x0000; // Pure black eyes & mouth perimeter
constexpr uint16_t BMO_MOUTH_DARK  = 0x13C7; // Dark teal-green mouth cavity (#14783B)
constexpr uint16_t BMO_TONGUE      = 0x4E8D; // Bright green tongue arch (#4CD368)
constexpr uint16_t BMO_TEETH_WHITE = 0xFFFF; // Pure white upper teeth bar
constexpr uint16_t BMO_DARK_CARD   = 0x0A2B; // Console dark teal card fill
constexpr uint16_t BMO_TEXT_MUTED  = 0x9E75; // Mint muted text
constexpr uint16_t BMO_YELLOW      = 0xFEA0; // BMO D-pad gold / yellow accent

// Dynamic theme color getters
inline uint16_t themeAccent()    { return (currentTheme == 2) ? BMO_YELLOW : ((currentTheme == 1) ? BLINK_CYAN : STAR_PINK); }
inline uint16_t themeSecondary() { return (currentTheme == 2) ? BMO_TONGUE : ((currentTheme == 1) ? 0x57BF     : STAR_ROSE); }
inline uint16_t themeBorder()    { return (currentTheme == 2) ? 0x24C9     : ((currentTheme == 1) ? BLINK_GLOW : STAR_BERRY); }
inline uint16_t themeCardBg()    { return (currentTheme == 2) ? BMO_DARK_CARD : ((currentTheme == 1) ? BLINK_DARK_TEAL : STAR_BG_CARD); }
inline uint16_t themeMuted()     { return (currentTheme == 2) ? BMO_TEXT_MUTED : ((currentTheme == 1) ? BLINK_TEXT_MUTED : STAR_MUTED); }
inline const char* themeTitle()  { return (currentTheme == 2) ? "* BMO BOT *" : ((currentTheme == 1) ? "* BLINK BOT *" : "* TWINKLE *"); }

struct Sparkle {
  float x, y, vx, vy;
  uint16_t color;
  unsigned long born;
};
constexpr int SPARKLE_COUNT = 18;
Sparkle sparkles[SPARKLE_COUNT];

float x = SCREEN_W / 2.0f;
float y = SCREEN_H / 2.0f;
float vx = 0, vy = 0;
float smoothX = 0, smoothY = 0;
uint16_t twinkleColor        = PINK;
unsigned long lastFrame      = 0;
unsigned long lastButton     = 0;
unsigned long motorUntil     = 0;
unsigned long confirmPressedAt = 0;
bool confirmHandled          = false;

enum class TwinkleMode { PLAY, MENU, THEME_SELECT, AIR_MOUSE, TUNE, GAMES, GAME_PLAYING, CUSTOM_DISPLAY, SETTINGS };
TwinkleMode mode    = TwinkleMode::PLAY;
uint8_t menuChoice  = 0;
uint8_t displayStyle = 0; // 0 = still image, 1 = animated image

WebServer web(80);
DNSServer dns;
bool portalActive     = false;
bool portalScreenDrawn = false;
File uploadFile;

struct ButtonState {
  uint8_t pin;
  bool lastReading;
  bool state;
  unsigned long lastDebounceTime;
  bool justPressed;
  bool justReleased;
  unsigned long pressedAt;
  bool longPressHandled;
};

ButtonState btnLeft    = { (uint8_t)PIN_LEFT,    HIGH, HIGH, 0, false, false, 0, false };
ButtonState btnRight   = { (uint8_t)PIN_RIGHT,   HIGH, HIGH, 0, false, false, 0, false };
ButtonState btnConfirm = { (uint8_t)PIN_CONFIRM, HIGH, HIGH, 0, false, false, 0, false };

BLEHIDDevice      *bleHid;
BLECharacteristic *mouseReport;
bool bleConnected = false;

class TwinkleSecurityCallbacks : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override { return 0; }
  void onPassKeyNotify(uint32_t pass_key) override {}
  bool onSecurityRequest() override { return true; }
  bool onConfirmPIN(uint32_t pin) override { return true; }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {
    if (cmpl.success) Serial.println("BLE Pairing / Bonding SUCCESS");
    else Serial.printf("BLE Pairing FAIL: 0x%02x\n", cmpl.fail_reason);
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onAuthenticationComplete(ble_gap_conn_desc *desc) override {
    if (desc != nullptr && desc->sec_state.bonded) Serial.println("BLE Pairing / Bonding SUCCESS");
  }
#endif
};

class TwinkleBleCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    bleConnected = true;
    Serial.println("BLE Client Connected");
  }
  void onDisconnect(BLEServer *pServer) override {
    bleConnected = false;
    Serial.println("BLE Client Disconnected, restarting advertising");
    pServer->getAdvertising()->start();
  }
};

// Standard Bluetooth LE HID descriptor: three mouse buttons + relative X/Y/wheel.
const uint8_t mouseHidDescriptor[] = {
  0x05,0x01,0x09,0x02,0xA1,0x01,0x85,0x01,0x09,0x01,0xA1,0x00,0x05,0x09,
  0x19,0x01,0x29,0x03,0x15,0x00,0x25,0x01,0x95,0x03,0x75,0x01,0x81,0x02,
  0x95,0x01,0x75,0x05,0x81,0x03,0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,
  0x15,0x81,0x25,0x7F,0x75,0x08,0x95,0x03,0x81,0x06,0xC0,0xC0
};

// ─────────────────────────────────────────────────────────────────────────────
// Captive-portal HTML (stored in flash)
// ─────────────────────────────────────────────────────────────────────────────
static const char PORTAL_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Twinkle Studio</title>
<style>
  :root {
    --bg: #0b0817;
    --card: #16102b;
    --border: #7c4dff;
    --accent: #e040fb;
    --accent2: #7c4dff;
    --title: #ffe46b;
    --text: #f0e6ff;
    --sub: #b39ddb;
    --drop: #1e143d;
    --drop-hover: #2a1b54;
    --btn-opt: #2a1b54;
    --btn-opt-border: #512da8;
    --btn-active: #7c4dff;
  }
  body.theme-blink {
    --bg: #02090f;
    --card: #061824;
    --border: #00e5ff;
    --accent: #00f0ff;
    --accent2: #0077b6;
    --title: #00f0ff;
    --text: #e0f7fa;
    --sub: #4dd0e1;
    --drop: #0a2336;
    --drop-hover: #10334c;
    --btn-opt: #0a2336;
    --btn-opt-border: #0077b6;
    --btn-active: #00b4d8;
  }
  body.theme-bmo {
    --bg: #0b261a;
    --card: #123827;
    --border: #7ee8a8;
    --accent: #ffe135;
    --accent2: #2ec4b6;
    --title: #7ee8a8;
    --text: #e8f9ee;
    --sub: #9fe2b9;
    --drop: #194b34;
    --drop-hover: #205e42;
    --btn-opt: #194b34;
    --btn-opt-border: #7ee8a8;
    --btn-active: #ffe135;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif;min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:16px;transition:0.3s}
  .card{background:var(--card);border:1px solid var(--border);border-radius:16px;max-width:440px;width:100%;padding:20px;box-shadow:0 8px 32px rgba(0,0,0,0.6);text-align:center;transition:0.3s}
  h1{color:var(--title);font-size:1.4rem;margin-bottom:6px;display:flex;align-items:center;justify-content:center;gap:6px}
  p.sub{color:var(--sub);font-size:0.85rem;margin-bottom:14px}
  .preview-box{display:flex;justify-content:center;margin:10px 0}
  canvas{width:240px;height:135px;background:#000;border:3px solid var(--border);border-radius:8px;image-rendering:pixelated;box-shadow:0 0 16px rgba(0,229,255,0.2)}
  .drop-zone{position:relative;border:2px dashed var(--border);border-radius:12px;padding:16px;margin:12px 0;cursor:pointer;background:var(--drop);transition:0.2s;overflow:hidden}
  .drop-zone:hover{background:var(--drop-hover);border-color:var(--accent)}
  .drop-zone input{position:absolute;top:0;left:0;width:100%;height:100%;opacity:0;cursor:pointer;z-index:10;display:block}
  .drop-text{font-size:0.95rem;font-weight:bold;color:var(--text);pointer-events:none}
  .controls{display:flex;gap:8px;justify-content:center;margin:10px 0}
  .btn-opt{background:var(--btn-opt);border:1px solid var(--btn-opt-border);color:var(--text);padding:6px 14px;border-radius:8px;font-size:0.8rem;cursor:pointer;transition:0.2s}
  .btn-opt.active{background:var(--btn-active);color:#fff;border-color:var(--accent)}
  .btn-upload{background:linear-gradient(135deg,var(--accent2),var(--accent));color:#fff;border:none;width:100%;padding:14px;border-radius:10px;font-size:1rem;font-weight:bold;cursor:pointer;margin-top:8px;box-shadow:0 4px 16px rgba(0,240,255,0.3);transition:0.2s}
  .btn-upload:disabled{opacity:0.5;cursor:not-allowed}
  #status{margin-top:12px;font-size:0.9rem;min-height:22px;font-weight:500}
  .status-ok{color:#00ff9d}
  .status-err{color:#ff5252}
  .status-info{color:#80d8ff}
  .info-bar{font-size:0.75rem;color:var(--sub);margin-top:6px}
  .tabs{display:flex;gap:8px;justify-content:center;margin-bottom:14px}
  .tab-btn{background:var(--btn-opt);border:1px solid var(--btn-opt-border);color:var(--text);padding:7px 16px;border-radius:20px;font-size:0.85rem;font-weight:bold;cursor:pointer;transition:0.2s}
  .tab-btn.active{background:var(--btn-active);color:#fff;border-color:var(--accent);box-shadow:0 0 12px rgba(0,240,255,0.35)}
  .tab-pane{display:none}
  .tab-pane.active{display:block}
</style>
</head>
<body class="THEME_CLASS">
<div class="card">
  <div class="tabs">
    <button class="tab-btn active" id="tabImgBtn" onclick="switchTab('img')">&#x1F5BC; Photo &amp; GIF</button>
    <button class="tab-btn" id="tabTuneBtn" onclick="switchTab('tune')">&#x1F3B5; MP3 to Tune</button>
  </div>

  <div id="tabImgPane" class="tab-pane active">
    <h1>HEADER_TITLE</h1>
    <p class="sub">HEADER_SUB</p>
    
    <div class="preview-box">
      <canvas id="cv" width="240" height="135"></canvas>
    </div>
    
    <div class="info-bar" id="imgInfo">Display: 240 x 135 Landscape</div>

    <div class="controls">
      <button class="btn-opt active" id="btnCover" onclick="setFit('cover')">Fill (Crop)</button>
      <button class="btn-opt" id="btnContain" onclick="setFit('contain')">Fit (Bars)</button>
      <button class="btn-opt" id="btnStretch" onclick="setFit('stretch')">Stretch</button>
    </div>

    <div class="drop-zone" id="dropZone">
      <input type="file" id="fi" accept="image/*,.bin" onchange="onFileSelected(this)">
      <div class="drop-text" id="dropLabel">&#x1F4C1; Choose Photo or GIF</div>
    </div>

    <button class="btn-upload" id="uploadBtn" onclick="upload()" disabled>SEND_BTN_TEXT</button>
    <div id="status"></div>
  </div>

  <div id="tabTunePane" class="tab-pane">
    <h1>&#x1F3B5; MP3 Tune Studio</h1>
    <p class="sub">Upload any MP3 or Audio file &mdash; auto-converts to buzzer chiptune!</p>

    <div class="drop-zone" id="tuneDropZone">
      <input type="file" id="tuneFi" accept="audio/*,.mp3,.wav,.ogg,.m4a,.midi,.bin" onchange="onTuneFileSelected(this)">
      <div class="drop-text" id="tuneDropLabel">&#x1F3B6; Choose MP3 / WAV Audio</div>
    </div>

    <div class="info-bar" id="tuneInfo">Extracts notes &amp; pitches for 8-bit buzzer</div>

    <div class="controls" style="margin-top:10px">
      <button class="btn-opt" id="btnPlayTune" onclick="toggleTunePreview()" disabled>&#x25B6; Listen (Buzzer Preview)</button>
    </div>

    <button class="btn-upload" id="uploadTuneBtn" onclick="uploadTune()" disabled>Upload Custom Tune to Bot</button>
    <div id="tuneStatus"></div>
  </div>
</div>

<script>
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
const fi = document.getElementById('fi');
const uploadBtn = document.getElementById('uploadBtn');
const statusDiv = document.getElementById('status');
const imgInfo = document.getElementById('imgInfo');

let currentFit = 'cover';
let loadedImg = null;
let rawFrames = [];
let gifTimer = null;
let rawPayload = null;

// Initial splash on canvas
ctx.fillStyle = '#05020d';
ctx.fillRect(0, 0, 240, 135);
ctx.fillStyle = '#7c4dff';
ctx.font = '14px sans-serif';
ctx.textAlign = 'center';
ctx.fillText('Select image or GIF', 120, 72);

function setFit(mode) {
  currentFit = mode;
  document.getElementById('btnCover').classList.toggle('active', mode === 'cover');
  document.getElementById('btnContain').classList.toggle('active', mode === 'contain');
  document.getElementById('btnStretch').classList.toggle('active', mode === 'stretch');
  if (rawFrames.length > 0) {
    processGifFrames();
  } else if (loadedImg) {
    renderStillImage();
  }
}

async function onFileSelected(input) {
  const file = input.files && input.files[0];
  if (!file) return;
  document.getElementById('dropLabel').textContent = file.name;
  statusDiv.className = 'status-info';
  statusDiv.textContent = 'Processing ' + file.name + '...';
  uploadBtn.disabled = true;

  if (file.name.endsWith('.bin')) {
    const ab = await file.arrayBuffer();
    rawPayload = new Uint8Array(ab);
    uploadBtn.disabled = false;
    statusDiv.className = 'status-ok';
    statusDiv.textContent = 'Binary loaded (' + rawPayload.length + ' bytes)';
    return;
  }

  const isGif = file.type === 'image/gif' || file.name.toLowerCase().endsWith('.gif');

  if (isGif) {
    try {
      const buffer = await file.arrayBuffer();
      rawFrames = parseGifBuffer(buffer);
      if (rawFrames.length > 0) {
        processGifFrames();
        return;
      }
    } catch (e) {
      console.warn('Pure-JS GIF parser error, fallback:', e);
    }
  }

  // Still photo fallback
  clearInterval(gifTimer);
  rawFrames = [];
  const url = URL.createObjectURL(file);
  loadedImg = new Image();
  loadedImg.onload = () => {
    renderStillImage();
    uploadBtn.disabled = false;
    statusDiv.className = 'status-ok';
    statusDiv.textContent = 'Photo ready to send!';
  };
  loadedImg.src = url;
}

function drawScaled(source, targetCtx) {
  targetCtx.fillStyle = '#000000';
  targetCtx.fillRect(0, 0, 240, 135);
  const sw = source.width || 240;
  const sh = source.height || 135;
  let dx = 0, dy = 0, dw = 240, dh = 135;

  if (currentFit === 'contain') {
    const scale = Math.min(240 / sw, 135 / sh);
    dw = Math.round(sw * scale); dh = Math.round(sh * scale);
    dx = Math.round((240 - dw) / 2); dy = Math.round((135 - dh) / 2);
  } else if (currentFit === 'cover') {
    const scale = Math.max(240 / sw, 135 / sh);
    dw = Math.round(sw * scale); dh = Math.round(sh * scale);
    dx = Math.round((240 - dw) / 2); dy = Math.round((135 - dh) / 2);
  }
  targetCtx.drawImage(source, dx, dy, dw, dh);
}

function renderStillImage() {
  drawScaled(loadedImg, ctx);
  imgInfo.textContent = `Photo: ${loadedImg.width}x${loadedImg.height}`;
  
  // Build RGB565 buffer with "TWIN" 1-frame header (240x135)
  const imgData = ctx.getImageData(0, 0, 240, 135).data;
  const buf = new Uint8Array(8 + 240 * 135 * 2);
  buf[0] = 84; buf[1] = 87; buf[2] = 73; buf[3] = 78; // 'T','W','I','N'
  buf[4] = 1;  buf[5] = 0;   // 1 frame
  buf[6] = 100; buf[7] = 0; // delay 100ms
  let p = 8;
  for (let i = 0; i < imgData.length; i += 4) {
    const r = imgData[i], g = imgData[i+1], b = imgData[i+2];
    const rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    buf[p++] = (rgb565 >> 8) & 0xFF;
    buf[p++] = rgb565 & 0xFF;
  }
  rawPayload = buf;
}

// ─── Pure-JS GIF Decoder ─────────────────────────────────────────────────────
function parseGifBuffer(buffer) {
  const bytes = new Uint8Array(buffer);
  if (bytes[0] !== 0x47 || bytes[1] !== 0x49 || bytes[2] !== 0x46) {
    throw new Error('Not a valid GIF');
  }
  let pos = 6;
  const gWidth = bytes[pos] | (bytes[pos+1] << 8);
  const gHeight = bytes[pos+2] | (bytes[pos+3] << 8);
  const gFlags = bytes[pos+4];
  const gctFlag = (gFlags & 0x80) !== 0;
  const gctSize = 1 << ((gFlags & 0x07) + 1);
  pos += 7;

  let globalPalette = [];
  if (gctFlag) {
    for (let i = 0; i < gctSize; i++) {
      globalPalette.push([bytes[pos], bytes[pos+1], bytes[pos+2]]);
      pos += 3;
    }
  }

  const frames = [];
  let gceDelay = 100;
  let gceTrans = -1;

  // Composite canvas for GIF disposal accumulation
  const compCanvas = document.createElement('canvas');
  compCanvas.width = gWidth;
  compCanvas.height = gHeight;
  const compCtx = compCanvas.getContext('2d');

  while (pos < bytes.length) {
    const block = bytes[pos++];
    if (block === 0x3B) break; // Trailer
    if (block === 0x21) { // Extension
      const extType = bytes[pos++];
      if (extType === 0xF9) { // Graphic Control Extension
        const blockSize = bytes[pos++];
        const ef = bytes[pos];
        const hasTrans = (ef & 1) !== 0;
        const delay = (bytes[pos+1] | (bytes[pos+2] << 8)) * 10;
        gceDelay = delay >= 20 ? delay : 100;
        gceTrans = hasTrans ? bytes[pos+3] : -1;
        pos += blockSize;
        while (bytes[pos] !== 0) pos += bytes[pos] + 1;
        pos++;
      } else {
        while (bytes[pos] !== 0) pos += bytes[pos] + 1;
        pos++;
      }
    } else if (block === 0x2C) { // Image Descriptor
      const left = bytes[pos] | (bytes[pos+1] << 8);
      const top = bytes[pos+2] | (bytes[pos+3] << 8);
      const w = bytes[pos+4] | (bytes[pos+5] << 8);
      const h = bytes[pos+6] | (bytes[pos+7] << 8);
      const iflags = bytes[pos+8];
      pos += 9;

      const lctFlag = (iflags & 0x80) !== 0;
      const lctSize = 1 << ((iflags & 0x07) + 1);
      let palette = globalPalette;
      if (lctFlag) {
        palette = [];
        for (let i = 0; i < lctSize; i++) {
          palette.push([bytes[pos], bytes[pos+1], bytes[pos+2]]);
          pos += 3;
        }
      }

      const lzwMinCodeSize = bytes[pos++];
      const pixelRes = decompressLzw(bytes, pos, lzwMinCodeSize, w * h);
      pos = pixelRes.newPos;

      const frameCanvas = document.createElement('canvas');
      frameCanvas.width = w;
      frameCanvas.height = h;
      const frameCtx = frameCanvas.getContext('2d');
      const imgData = frameCtx.createImageData(w, h);
      const d = imgData.data;

      for (let i = 0; i < pixelRes.pixels.length; i++) {
        const cIdx = pixelRes.pixels[i];
        const oIdx = i * 4;
        if (cIdx === gceTrans) {
          d[oIdx + 3] = 0;
        } else if (palette[cIdx]) {
          const [r, g, b] = palette[cIdx];
          d[oIdx] = r; d[oIdx + 1] = g; d[oIdx + 2] = b; d[oIdx + 3] = 255;
        }
      }
      frameCtx.putImageData(imgData, 0, 0);

      // Draw onto composite canvas
      compCtx.drawImage(frameCanvas, left, top);

      // Copy current composite frame
      const snapCanvas = document.createElement('canvas');
      snapCanvas.width = gWidth;
      snapCanvas.height = gHeight;
      snapCanvas.getContext('2d').drawImage(compCanvas, 0, 0);

      frames.push({ canvas: snapCanvas, duration: gceDelay });
    }
  }
  return frames;
}

function decompressLzw(bytes, pos, minCodeSize, totalPixels) {
  const clearCode = 1 << minCodeSize;
  const eoiCode = clearCode + 1;
  let codeSize = minCodeSize + 1;
  let maxCode = (1 << codeSize) - 1;

  let dict = [];
  function resetDict() {
    dict = [];
    for (let i = 0; i < clearCode; i++) dict[i] = [i];
    dict[clearCode] = [];
    dict[eoiCode] = null;
    codeSize = minCodeSize + 1;
    maxCode = (1 << codeSize) - 1;
  }
  resetDict();

  const pixels = [];
  let bitBuf = 0;
  let bitCount = 0;
  let prevCode = null;

  while (pos < bytes.length && pixels.length < totalPixels) {
    const blockSize = bytes[pos++];
    if (blockSize === 0) break;
    const blockEnd = pos + blockSize;

    while (pos < blockEnd || bitCount >= codeSize) {
      while (bitCount < codeSize && pos < blockEnd) {
        bitBuf |= bytes[pos++] << bitCount;
        bitCount += 8;
      }
      if (bitCount < codeSize) break;

      const code = bitBuf & ((1 << codeSize) - 1);
      bitBuf >>= codeSize;
      bitCount -= codeSize;

      if (code === clearCode) {
        resetDict();
        prevCode = null;
        continue;
      }
      if (code === eoiCode) break;

      let entry;
      if (code < dict.length) {
        entry = dict[code];
      } else if (code === dict.length && prevCode !== null) {
        entry = [...dict[prevCode], dict[prevCode][0]];
      } else {
        break;
      }

      for (let k = 0; k < entry.length; k++) {
        pixels.push(entry[k]);
        if (pixels.length >= totalPixels) break;
      }

      if (prevCode !== null && dict.length < 4096) {
        dict.push([...dict[prevCode], entry[0]]);
        if (dict.length > maxCode && codeSize < 12) {
          codeSize++;
          maxCode = (1 << codeSize) - 1;
        }
      }
      prevCode = code;
    }
  }
  while (pos < bytes.length && bytes[pos] !== 0) {
    pos += bytes[pos] + 1;
  }
  if (pos < bytes.length && bytes[pos] === 0) pos++;

  return { pixels, newPos: pos };
}

// ─── Auto-adjust & Anti-Lag Optimization ──────────────────────────────────────
function processGifFrames() {
  clearInterval(gifTimer);
  if (!rawFrames || rawFrames.length === 0) return;

  const totalRaw = rawFrames.length;
  const MAX_FRAMES = 8; // Optimal count: fits flash and guarantees 0 lag
  let sampled = [];

  if (totalRaw <= MAX_FRAMES) {
    sampled = rawFrames;
  } else {
    // Sub-sample evenly across entire animation duration
    const step = totalRaw / MAX_FRAMES;
    for (let i = 0; i < MAX_FRAMES; i++) {
      sampled.push(rawFrames[Math.floor(i * step)]);
    }
  }

  let totalDuration = 0;
  for (let f of rawFrames) totalDuration += f.duration;
  const avgDelay = Math.max(80, Math.min(250, Math.round(totalDuration / sampled.length)));
  const numFrames = sampled.length;

  const buf = new Uint8Array(8 + numFrames * 240 * 135 * 2);
  buf[0] = 84; buf[1] = 87; buf[2] = 73; buf[3] = 78; // 'T','W','I','N'
  buf[4] = numFrames & 0xFF; buf[5] = (numFrames >> 8) & 0xFF;
  buf[6] = avgDelay & 0xFF;  buf[7] = (avgDelay >> 8) & 0xFF;

  let p = 8;
  const previewCanvases = [];

  for (let f = 0; f < numFrames; f++) {
    const sc = document.createElement('canvas');
    sc.width = 240; sc.height = 135;
    const sctx = sc.getContext('2d');
    drawScaled(sampled[f].canvas, sctx);
    previewCanvases.push(sc);

    const imgData = sctx.getImageData(0, 0, 240, 135).data;
    for (let i = 0; i < imgData.length; i += 4) {
      const r = imgData[i], g = imgData[i+1], b = imgData[i+2];
      const rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
      buf[p++] = (rgb565 >> 8) & 0xFF;
      buf[p++] = rgb565 & 0xFF;
    }
  }
  rawPayload = buf;

  // Live browser preview
  let curr = 0;
  gifTimer = setInterval(() => {
    ctx.drawImage(previewCanvases[curr], 0, 0);
    curr = (curr + 1) % previewCanvases.length;
  }, avgDelay);

  imgInfo.textContent = `Optimized GIF: ${numFrames} frames @ ${avgDelay}ms (${Math.round(rawPayload.length / 1024)} KB)`;
  uploadBtn.disabled = false;
  statusDiv.className = 'status-ok';
  statusDiv.textContent = `Auto-adjusted! Ready to send (${numFrames} frames)`;
}

async function upload() {
  if (!rawPayload) return;
  uploadBtn.disabled = true;
  statusDiv.className = 'status-info';
  statusDiv.textContent = 'Uploading to Twinkle (' + Math.round(rawPayload.length / 1024) + ' KB)...';

  try {
    const blob = new Blob([rawPayload], { type: 'application/octet-stream' });
    const fd = new FormData();
    fd.append('file', blob, 'image.bin');

    const res = await fetch('/upload', { method: 'POST', body: fd });
    const text = await res.text();
    statusDiv.className = 'status-ok';
    statusDiv.textContent = text || 'Uploaded successfully!';
    uploadBtn.disabled = false;
  } catch (err) {
    statusDiv.className = 'status-err';
    statusDiv.textContent = 'Upload failed: ' + err.message;
    uploadBtn.disabled = false;
  }
}

// ─── Custom Tune Studio Logic ────────────────────────────────────────────────
function switchTab(t) {
  document.getElementById('tabImgBtn').classList.toggle('active', t === 'img');
  document.getElementById('tabTuneBtn').classList.toggle('active', t === 'tune');
  document.getElementById('tabImgPane').classList.toggle('active', t === 'img');
  document.getElementById('tabTunePane').classList.toggle('active', t === 'tune');
  if (t === 'img') stopTunePreview();
}

let audioCtx = null;
let parsedTuneNotes = [];
let previewTimer = null;
let isPreviewing = false;

async function onTuneFileSelected(input) {
  const file = input.files && input.files[0];
  if (!file) return;
  document.getElementById('tuneDropLabel').textContent = file.name;
  const status = document.getElementById('tuneStatus');
  const uploadBtn = document.getElementById('uploadTuneBtn');
  const playBtn = document.getElementById('btnPlayTune');
  const info = document.getElementById('tuneInfo');
  
  stopTunePreview();
  status.className = 'status-info';
  status.textContent = 'Decoding audio waveform...';
  uploadBtn.disabled = true;
  playBtn.disabled = true;

  if (file.name.endsWith('.bin')) {
    const ab = await file.arrayBuffer();
    const bytes = new Uint8Array(ab);
    if (bytes.length >= 6 && bytes[0]===0x54 && bytes[1]===0x55 && bytes[2]===0x4E && bytes[3]===0x45) {
      const count = bytes[4] | (bytes[5] << 8);
      parsedTuneNotes = [];
      let off = 6;
      for (let i=0; i<count && off + 4 <= bytes.length; i++) {
        const freq = (bytes[off] << 8) | bytes[off+1];
        const dur = (bytes[off+2] << 8) | bytes[off+3];
        off += 4;
        parsedTuneNotes.push({ freq, dur });
      }
      info.textContent = `Binary Tune: ${parsedTuneNotes.length} notes`;
      status.className = 'status-ok';
      status.textContent = 'Tune binary ready to send!';
      uploadBtn.disabled = false;
      playBtn.disabled = false;
      return;
    }
  }

  try {
    if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    if (audioCtx.state === 'suspended') await audioCtx.resume();
    
    const arrayBuffer = await file.arrayBuffer();
    const audioBuffer = await audioCtx.decodeAudioData(arrayBuffer);
    
    status.textContent = 'Analyzing pitch & melody frequencies...';
    parsedTuneNotes = convertAudioToTuneNotes(audioBuffer);
    
    if (parsedTuneNotes.length === 0) {
      status.className = 'status-err';
      status.textContent = 'No melody detected. Try an audio file with clear melody.';
      return;
    }

    let totalMs = parsedTuneNotes.reduce((acc, n) => acc + n.dur, 0);
    info.textContent = `Converted: ${parsedTuneNotes.length} notes (${(totalMs/1000).toFixed(1)}s duration)`;
    status.className = 'status-ok';
    status.textContent = 'Auto-converted successfully! Test preview or upload.';
    uploadBtn.disabled = false;
    playBtn.disabled = false;
  } catch(e) {
    console.error(e);
    status.className = 'status-err';
    status.textContent = 'Audio conversion error: ' + (e.message || e);
  }
}

function convertAudioToTuneNotes(audioBuffer) {
  const sampleRate = audioBuffer.sampleRate;
  const pcm = audioBuffer.getChannelData(0);
  const hopSize = Math.floor(sampleRate * 0.045); // 45ms slices
  const windowSize = Math.floor(sampleRate * 0.090); // 90ms analysis window
  
  const rawNotes = [];
  const maxWindows = Math.min(Math.floor((pcm.length - windowSize) / hopSize), 600); // up to ~27 sec

  for (let i = 0; i < maxWindows; i++) {
    const start = i * hopSize;
    let rms = 0;
    for (let j = 0; j < windowSize; j++) {
      const v = pcm[start + j];
      rms += v * v;
    }
    rms = Math.sqrt(rms / windowSize);

    if (rms < 0.015) {
      rawNotes.push(0); // Rest
      continue;
    }

    let bestOffset = -1;
    let bestCorr = 0;
    const minPeriod = Math.floor(sampleRate / 2200);
    const maxPeriod = Math.floor(sampleRate / 80);

    for (let offset = minPeriod; offset <= maxPeriod; offset += 2) {
      let corr = 0;
      for (let j = 0; j < windowSize - offset; j += 4) {
        corr += pcm[start + j] * pcm[start + j + offset];
      }
      if (corr > bestCorr) {
        bestCorr = corr;
        bestOffset = offset;
      }
    }

    if (bestOffset > 0 && bestCorr > 0.01) {
      const freq = sampleRate / bestOffset;
      if (freq >= 80 && freq <= 2200) {
        const midi = Math.round(69 + 12 * Math.log2(freq / 440));
        const snappedFreq = Math.round(440 * Math.pow(2, (midi - 69) / 12));
        rawNotes.push(snappedFreq);
      } else {
        rawNotes.push(0);
      }
    } else {
      rawNotes.push(0);
    }
  }

  const notes = [];
  let curFreq = rawNotes[0] || 0;
  let curCount = 1;

  for (let i = 1; i < rawNotes.length; i++) {
    if (rawNotes[i] === curFreq) {
      curCount++;
    } else {
      let dur = curCount * 45;
      if (dur >= 40) {
        notes.push({ freq: curFreq, dur: dur });
      }
      curFreq = rawNotes[i];
      curCount = 1;
    }
  }
  if (curCount > 0) {
    notes.push({ freq: curFreq, dur: curCount * 45 });
  }

  return notes.slice(0, 300);
}

function toggleTunePreview() {
  if (isPreviewing) {
    stopTunePreview();
  } else {
    startTunePreview();
  }
}

function startTunePreview() {
  if (!parsedTuneNotes || parsedTuneNotes.length === 0) return;
  if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
  if (audioCtx.state === 'suspended') audioCtx.resume();
  
  isPreviewing = true;
  document.getElementById('btnPlayTune').textContent = '⏹ Stop Preview';
  
  let noteIdx = 0;
  function playNext() {
    if (!isPreviewing || noteIdx >= parsedTuneNotes.length) {
      stopTunePreview();
      return;
    }
    const n = parsedTuneNotes[noteIdx++];
    if (n.freq > 0) {
      const osc = audioCtx.createOscillator();
      const gain = audioCtx.createGain();
      osc.type = 'square';
      osc.frequency.setValueAtTime(n.freq, audioCtx.currentTime);
      gain.gain.setValueAtTime(0.08, audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + (n.dur / 1000) * 0.95);
      osc.connect(gain);
      gain.connect(audioCtx.destination);
      osc.start();
      osc.stop(audioCtx.currentTime + (n.dur / 1000) * 0.95);
    }
    previewTimer = setTimeout(playNext, n.dur);
  }
  playNext();
}

function stopTunePreview() {
  isPreviewing = false;
  if (previewTimer) clearTimeout(previewTimer);
  const btn = document.getElementById('btnPlayTune');
  if (btn) btn.textContent = '▶ Listen (Buzzer Preview)';
}

async function uploadTune() {
  if (!parsedTuneNotes || parsedTuneNotes.length === 0) return;
  stopTunePreview();
  
  const status = document.getElementById('tuneStatus');
  const uploadBtn = document.getElementById('uploadTuneBtn');
  status.className = 'status-info';
  status.textContent = 'Sending custom tune to bot...';
  uploadBtn.disabled = true;

  const count = parsedTuneNotes.length;
  const buf = new Uint8Array(6 + count * 4);
  buf[0] = 0x54; // 'T'
  buf[1] = 0x55; // 'U'
  buf[2] = 0x4E; // 'N'
  buf[3] = 0x45; // 'E'
  buf[4] = count & 0xFF;
  buf[5] = (count >> 8) & 0xFF;

  let offset = 6;
  for (let i = 0; i < count; i++) {
    const f = parsedTuneNotes[i].freq;
    const d = parsedTuneNotes[i].dur;
    buf[offset++] = (f >> 8) & 0xFF;
    buf[offset++] = f & 0xFF;
    buf[offset++] = (d >> 8) & 0xFF;
    buf[offset++] = d & 0xFF;
  }

  try {
    const blob = new Blob([buf], { type: 'application/octet-stream' });
    const fd = new FormData();
    fd.append('file', blob, 'custom_tune.bin');

    const res = await fetch('/upload_tune', { method: 'POST', body: fd });
    const text = await res.text();
    status.className = 'status-ok';
    status.textContent = text || 'Tune uploaded successfully!';
    uploadBtn.disabled = false;
  } catch(e) {
    status.className = 'status-err';
    status.textContent = 'Upload failed: ' + e.message;
    uploadBtn.disabled = false;
  }
}
</script>
</body>
</html>
)HTML";

// ─────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────────────────────────────────────
void enterMode(TwinkleMode m);
void drawMenu();
void drawThemeSelect();
void drawSettings();
void drawAirMouseScreen();
void drawCustomDisplay();
void updateCustomDisplay();
void reloadCustomDisplay();
void setupPortal();
void teardownPortal();
void drawStarfield();
void applyBrightness(uint8_t level);
void updateBlinkEyes(float dt, sensors_event_t &accel, sensors_event_t &gyro);
void drawBlinkEyesFrame();

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

Preferences prefs;

/** Non-blocking set tone on buzzer with duty cycle volume scaling. */
void setTone(int freq) {
  extern uint8_t soundVolume;
  if (freq <= 0 || soundVolume == 0) {
    ledcWrite(PIN_BUZZER, 0);
    ledcWriteTone(PIN_BUZZER, 0);
    return;
  }
  ledcWriteTone(PIN_BUZZER, freq);
  uint32_t maxDuty = 512;
  uint32_t duty = (soundVolume >= 10) ? maxDuty : (uint32_t)(maxDuty * powf((float)soundVolume / 10.0f, 1.8f));
  if (duty < 6 && soundVolume > 0) duty = 6;
  ledcWrite(PIN_BUZZER, duty);
}

/** Stop any active buzzer tone immediately (non-blocking). */
void stopTone() {
  ledcWrite(PIN_BUZZER, 0);
  ledcWriteTone(PIN_BUZZER, 0);
}

/** Play a tone on the buzzer with volume scaling (synchronous for short UI beeps). */
void buzz(int freq, int ms) {
  if (freq <= 0) return;
  setTone(freq);
  delay(ms);
  stopTone();
}

/** Set screen brightness level (0-10) using ST7789 commands and PWM backlight if connected. */
void applyBrightness(uint8_t level) {
  if (level > 10) level = 10;
  if (PIN_TFT_BL >= 0) {
    uint32_t duty = (level == 0) ? 0 : (uint32_t)map(level, 1, 10, 20, 255);
    analogWrite(PIN_TFT_BL, duty);
  }
  // ST7789 controller brightness registers:
  // WRCTRLD (0x53): Enable Display Dimming and Brightness Control Block (0x24: BCTRL=1, DD=1)
  uint8_t ctrl = (level == 0) ? 0x00 : 0x24;
  tft.sendCommand(0x53, &ctrl, 1);
  // WRDISBV (0x51): Display Brightness Value (0x00 - 0xFF)
  uint8_t brightVal = (uint8_t)map(level, 0, 10, 0, 255);
  tft.sendCommand(0x51, &brightVal, 1);
}

/** Pulse the vibration motor for ms milliseconds (non-blocking). */
void vibrate(int ms) {
  digitalWrite(PIN_MOTOR, HIGH);
  motorUntil = millis() + ms;
}

/** Spawn a sparkle particle near (sx, sy), reusing the oldest slot. */
void spawnSparkle(float sx, float sy) {
  int oldest = 0;
  unsigned long minT = sparkles[0].born;
  for (int i = 1; i < SPARKLE_COUNT; i++)
    if (sparkles[i].born < minT) { minT = sparkles[i].born; oldest = i; }
  Sparkle &s = sparkles[oldest];
  s.x = sx + random(-10, 10);
  s.y = sy + random(-10, 10);
  float angle = random(0, 628) / 100.0f;    // 0 .. 2*pi
  float speed = 0.3f + random(0, 100) / 150.0f;
  s.vx = cosf(angle) * speed;
  s.vy = sinf(angle) * speed;
  static const uint16_t pal[] = { PINK, MINT, STAR, GLOW, 0xFFFF };
  s.color = pal[random(0, 5)];
  s.born  = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// PLAY mode
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t starBgX[60], starBgY[60]; // static starfield positions
static int playPrevDrawX = -100, playPrevDrawY = -100;
static const uint16_t* playPrevSprite = nullptr;
static unsigned long dizzyUntil = 0;

/** Draw the deep-space background once (called on PLAY entry). */
void drawStarfield() {
  tft.fillScreen(SPACE);
  playPrevDrawX = -100;
  playPrevDrawY = -100;
  playPrevSprite = nullptr;
  // Faint nebula rings
  for (int i = 0; i < 6; i++)
    tft.drawCircle(random(15, SCREEN_W-15), random(20, SCREEN_H-20),
                   random(8, 22), NEBULA);
  // Fixed stars – positions stored so we can restore them after erasure
  for (int i = 0; i < 60; i++) {
    starBgX[i] = random(0, SCREEN_W);
    starBgY[i] = random(0, SCREEN_H);
    tft.drawPixel(starBgX[i], starBgY[i], (random(0,3)==0) ? 0xFFFF : 0xAD75);
  }
}

void updatePlay(float dt, sensors_event_t &accel, sensors_event_t &gyro) {
  // Physical alignment for Landscape orientation (240W x 135H, Rotation 3):
  // Bot held horizontally with screen facing user.
  //
  // Tilting RIGHT (right side down):  star rolls RIGHT (+X)
  // Tilting LEFT (left side down):    star rolls LEFT (-X)
  // Tilting FORWARD (bottom side down): star rolls DOWN (+Y)
  // Tilting BACK (top side down):     star rolls UP (-Y)
  float targetAX = -accel.acceleration.y;
  float targetAY = accel.acceleration.x;

  // Low-pass filter on tilt
  smoothX += (targetAX - smoothX) * 0.18f;
  smoothY += (targetAY - smoothY) * 0.18f;

  // Velocity integration
  vx += smoothX * dt * 38.0f;
  vy += smoothY * dt * 38.0f;
  vx *= 0.91f;
  vy *= 0.91f;

  x += vx;
  y += vy;

  // Screen bounds with subtle bounce based on star sprite dimensions
  int halfW = STAR_SPRITE_W / 2;
  int halfH = STAR_SPRITE_H / 2;
  if (x <= halfW) { x = halfW; vx = -vx * 0.4f; }
  else if (x >= SCREEN_W - halfW) { x = SCREEN_W - halfW; vx = -vx * 0.4f; }

  if (y <= halfH) { y = halfH; vy = -vy * 0.4f; }
  else if (y >= SCREEN_H - halfH) { y = SCREEN_H - halfH; vy = -vy * 0.4f; }

  // Shake detection (deviation from 1g = 9.8 m/s^2) - Lowered sensitivity
  float gyroRate = fabsf(gyro.gyro.x) + fabsf(gyro.gyro.y) + fabsf(gyro.gyro.z);
  float totalA = sqrtf(accel.acceleration.x * accel.acceleration.x +
                       accel.acceleration.y * accel.acceleration.y +
                       accel.acceleration.z * accel.acceleration.z);
  if (fabsf(totalA - 9.8f) > 28.0f || (gyroRate > 15.0f && fabsf(totalA - 9.8f) > 16.0f)) {
    unsigned long curMs = millis();
    if (curMs >= dizzyUntil) {
      dizzyUntil = curMs + 2500;
      vibrate(70);
      buzz(880, 25);
      for (int i = 0; i < 5; i++) spawnSparkle(x, y);
    }
  }

  // Periodic sparkle spawn
  static unsigned long lastSpawn = 0;
  if (millis() - lastSpawn > 110) { spawnSparkle(x, y); lastSpawn = millis(); }

  // Age / move sparkles
  unsigned long now = millis();
  for (int i = 0; i < SPARKLE_COUNT; i++) {
    sparkles[i].x += sparkles[i].vx;
    sparkles[i].y += sparkles[i].vy;
    if (now - sparkles[i].born > 1200) {    // recycle old sparkle
      sparkles[i].x = x;  sparkles[i].y = y;
      sparkles[i].born = now;
    }
  }
}

void drawPlayFrame() {
  static float prevSX[SPARKLE_COUNT], prevSY[SPARKLE_COUNT];

  const uint16_t* curSprite = (millis() < dizzyUntil) ? star_dizzy_sprite : star_normal_sprite;
  int curDrawX = (int)x - STAR_SPRITE_W / 2;
  int curDrawY = (int)y - STAR_SPRITE_H / 2;

  // Compute dirty bounding box enclosing both the previous and current star sprite positions
  int minX, maxX, minY, maxY;
  if (playPrevSprite != nullptr) {
    minX = max(0, min(playPrevDrawX, curDrawX));
    maxX = min(SCREEN_W - 1, max(playPrevDrawX + STAR_SPRITE_W - 1, curDrawX + STAR_SPRITE_W - 1));
    minY = max(0, min(playPrevDrawY, curDrawY));
    maxY = min(SCREEN_H - 1, max(playPrevDrawY + STAR_SPRITE_H - 1, curDrawY + STAR_SPRITE_H - 1));
  } else {
    minX = max(0, curDrawX);
    maxX = min(SCREEN_W - 1, curDrawX + STAR_SPRITE_W - 1);
    minY = max(0, curDrawY);
    maxY = min(SCREEN_H - 1, curDrawY + STAR_SPRITE_H - 1);
  }

  // --- Erase previous sparkles that are OUTSIDE the dirty rectangle ---
  for (int i = 0; i < SPARKLE_COUNT; i++) {
    int px = (int)prevSX[i];
    int py = (int)prevSY[i];
    if (px >= 0 && px < SCREEN_W && py >= 0 && py < SCREEN_H) {
      if (px < minX || px > maxX || py < minY || py > maxY) {
        tft.drawPixel(px, py, SPACE);
        // Restore background star if one was at this position
        for (int j = 0; j < 60; j++) {
          if (starBgX[j] == (uint16_t)px && starBgY[j] == (uint16_t)py) {
            tft.drawPixel(px, py, (j % 3 == 0) ? 0xFFFF : 0xAD75);
            break;
          }
        }
      }
    }
  }

  // --- Render dirty rectangle scanline by scanline into a line buffer ---
  int widthSpan = maxX - minX + 1;
  uint16_t lineBuf[SCREEN_W];
  unsigned long now = millis();

  tft.startWrite();
  for (int py = minY; py <= maxY; py++) {
    // 1. Fill scanline with background SPACE
    for (int col = 0; col < widthSpan; col++) {
      lineBuf[col] = SPACE;
    }

    // 2. Overlay static background stars on this line
    for (int j = 0; j < 60; j++) {
      if (starBgY[j] == (uint16_t)py && starBgX[j] >= (uint16_t)minX && starBgX[j] <= (uint16_t)maxX) {
        lineBuf[starBgX[j] - minX] = (j % 3 == 0) ? 0xFFFF : 0xAD75;
      }
    }

    // 3. Overlay sparkles on this line
    for (int i = 0; i < SPARKLE_COUNT; i++) {
      int sx = (int)sparkles[i].x;
      int sy = (int)sparkles[i].y;
      if (sy == py && sx >= minX && sx <= maxX) {
        uint16_t c = (now - sparkles[i].born < 600) ? sparkles[i].color : 0x2104;
        lineBuf[sx - minX] = c;
      }
    }

    // 4. Overlay the current star sprite on this line
    int spriteRow = py - curDrawY;
    if (spriteRow >= 0 && spriteRow < STAR_SPRITE_H) {
      const uint16_t* rowPtr = &curSprite[spriteRow * STAR_SPRITE_W];
      for (int sx = 0; sx < STAR_SPRITE_W; sx++) {
        int screenX = curDrawX + sx;
        if (screenX >= minX && screenX <= maxX) {
          uint16_t c = pgm_read_word(&rowPtr[sx]);
          if (c != 0) {
            lineBuf[screenX - minX] = c;
          }
        }
      }
    }

    // 5. Blast the scanline directly to ST7789 in a single SPI write
    tft.setAddrWindow(minX, py, widthSpan, 1);
    tft.writePixels(lineBuf, widthSpan);
  }
  tft.endWrite();

  // --- Draw sparkles that are OUTSIDE the dirty rectangle ---
  for (int i = 0; i < SPARKLE_COUNT; i++) {
    int sx = (int)sparkles[i].x;
    int sy = (int)sparkles[i].y;
    if (sx >= 0 && sx < SCREEN_W && sy >= 0 && sy < SCREEN_H) {
      if (sx < minX || sx > maxX || sy < minY || sy > maxY) {
        uint16_t c = (now - sparkles[i].born < 600) ? sparkles[i].color : 0x2104;
        tft.drawPixel(sx, sy, c);
      }
    }
    prevSX[i] = sparkles[i].x;
    prevSY[i] = sparkles[i].y;
  }

  playPrevDrawX = curDrawX;
  playPrevDrawY = curDrawY;
  playPrevSprite = curSprite;
}

// ─────────────────────────────────────────────────────────────────────────────
// BLINK (Robot Cyber Eyes) Theme Implementation
// ─────────────────────────────────────────────────────────────────────────────
static const uint16_t BLINK_PALETTE[] = {
  0x07FF, // Neon Cyan (default photo match)
  0x07E0, // Emerald Matrix Green
  0x981F, // Cyber Purple
  0xFD20, // Solar Orange
  0xFFE0, // Electric Gold
  0x57BF, // Ice Blue
  0xF800, // Crimson Red
  0xF99F  // Neon Pink
};
constexpr int BLINK_PALETTE_COUNT = sizeof(BLINK_PALETTE) / sizeof(BLINK_PALETTE[0]);
static uint8_t blinkColorIndex = 0;
uint16_t blinkEyeColor = BLINK_PALETTE[0];

enum class BlinkEmote { NORMAL, BLINK, HAPPY, WINK, LOOK_AROUND };
static BlinkEmote blinkCurrentEmote = BlinkEmote::NORMAL;
static unsigned long blinkEmoteUntil = 0;
static unsigned long blinkNextEmoteTime = 0;
static unsigned long blinkLastMotionTime = 0;
static unsigned long blinkDizzyUntil = 0;

static float smoothEyeX = 0.0f;
static float smoothEyeY = 0.0f;
static float eyeGazeX = 0.0f;
static float eyeGazeY = 0.0f;
static float eyeDizzyAngle = 0.0f;

static int prevEyeLX = -100, prevEyeLY = -100, prevEyeLW = 0, prevEyeLH = 0;
static int prevEyeRX = -100, prevEyeRY = -100, prevEyeRW = 0, prevEyeRH = 0;
static bool blinkPrevDrawn = false;

void updateBlinkEyes(float dt, sensors_event_t &accel, sensors_event_t &gyro) {
  unsigned long now = millis();

  // Target gaze from tilt
  float targetX = -accel.acceleration.y * 3.6f; // Tilt Right -> look Right (+X), Tilt Left -> look Left (-X)
  float targetY = accel.acceleration.x * 2.8f;  // Tilt Down -> look Down (+Y), Tilt Up -> look Up (-Y)

  targetX = constrain(targetX, -26.0f, 26.0f);
  targetY = constrain(targetY, -16.0f, 16.0f);

  // Motion detection for idle tracking
  float gyroRate = fabsf(gyro.gyro.x) + fabsf(gyro.gyro.y) + fabsf(gyro.gyro.z);
  if (gyroRate > 0.45f || fabsf(accel.acceleration.y) > 2.2f) {
    blinkLastMotionTime = now;
  }

  // Shake detection -> Dizzy mode (lowered sensitivity: requires strong deliberate shake)
  float totalA = sqrtf(accel.acceleration.x * accel.acceleration.x +
                       accel.acceleration.y * accel.acceleration.y +
                       accel.acceleration.z * accel.acceleration.z);
  if (fabsf(totalA - 9.8f) > 28.0f || (gyroRate > 15.0f && fabsf(totalA - 9.8f) > 16.0f)) {
    if (now >= blinkDizzyUntil) {
      blinkDizzyUntil = now + 2600;
      vibrate(60);
      buzz(440, 40); delay(20); buzz(330, 40);
    }
    blinkLastMotionTime = now;
  }

  // Dizzy animation wobble
  if (now < blinkDizzyUntil) {
    eyeDizzyAngle += dt * 16.0f;
    targetX = cosf(eyeDizzyAngle) * 22.0f;
    targetY = sinf(eyeDizzyAngle) * 15.0f;
  } else {
    // Idle Emote Handler (if idle for >= 3 seconds)
    if (now - blinkLastMotionTime >= 3000) {
      if (now >= blinkNextEmoteTime) {
        // Pick random emote: Blink (50%), Happy (20%), Wink (15%), Look around (15%)
        int r = random(0, 100);
        if (r < 50) {
          blinkCurrentEmote = BlinkEmote::BLINK;
          blinkEmoteUntil = now + 200; // 200ms quick natural blink
        } else if (r < 70) {
          blinkCurrentEmote = BlinkEmote::HAPPY;
          blinkEmoteUntil = now + 1200;
          buzz(880, 25); delay(20); buzz(1175, 35);
        } else if (r < 85) {
          blinkCurrentEmote = BlinkEmote::WINK;
          blinkEmoteUntil = now + 350;
          buzz(988, 30);
        } else {
          blinkCurrentEmote = BlinkEmote::LOOK_AROUND;
          blinkEmoteUntil = now + 1400;
        }
        blinkNextEmoteTime = now + 2500 + random(0, 2500); // next emote in 2.5 - 5.0s
      }
    } else {
      if (blinkCurrentEmote != BlinkEmote::NORMAL && now >= blinkEmoteUntil) {
        blinkCurrentEmote = BlinkEmote::NORMAL;
      }
    }
  }

  // Smooth gaze interpolation
  smoothEyeX += (targetX - smoothEyeX) * 0.22f;
  smoothEyeY += (targetY - smoothEyeY) * 0.22f;

  eyeGazeX = smoothEyeX;
  eyeGazeY = smoothEyeY;
}

void drawBlinkEyesFrame() {
  unsigned long now = millis();

  // Base eye geometry relative to 240x135 LCD with small margins
  constexpr int EYE_BASE_W = 74;
  constexpr int EYE_BASE_H = 96;
  constexpr int EYE_BASE_R = 26;
  constexpr int EYE_GAP = 18;

  int leftCenterRest  = (SCREEN_W / 2) - (EYE_BASE_W / 2 + EYE_GAP / 2);  // ~74
  int rightCenterRest = (SCREEN_W / 2) + (EYE_BASE_W / 2 + EYE_GAP / 2);  // ~166
  int eyeCenterYRest  = SCREEN_H / 2;                                     // ~67

  int curLH = EYE_BASE_H;
  int curRH = EYE_BASE_H;
  int curLW = EYE_BASE_W;
  int curRW = EYE_BASE_W;
  int curLR = EYE_BASE_R;
  int curRR = EYE_BASE_R;

  bool isHappy = false;
  bool isDizzy = (now < blinkDizzyUntil);

  if (isDizzy) {
    // Wobbly height oscillation during dizzy
    float wob = sinf(eyeDizzyAngle * 0.8f);
    curLH = constrain((int)(EYE_BASE_H + wob * 14.0f), 40, 108);
    curRH = constrain((int)(EYE_BASE_H - wob * 14.0f), 40, 108);
  } else if (blinkCurrentEmote == BlinkEmote::BLINK) {
    float p = (blinkEmoteUntil > now) ? (float)(blinkEmoteUntil - now) / 200.0f : 0.0f;
    float scale = fabsf(sinf(p * 3.14159f));
    curLH = max(4, (int)(EYE_BASE_H * (1.0f - scale * 0.94f)));
    curRH = curLH;
    curLR = min(EYE_BASE_R, curLH / 2);
    curRR = curLR;
  } else if (blinkCurrentEmote == BlinkEmote::WINK) {
    float p = (blinkEmoteUntil > now) ? (float)(blinkEmoteUntil - now) / 350.0f : 0.0f;
    float scale = fabsf(sinf(p * 3.14159f));
    curLH = max(4, (int)(EYE_BASE_H * (1.0f - scale * 0.94f)));
    curLR = min(EYE_BASE_R, curLH / 2);
  } else if (blinkCurrentEmote == BlinkEmote::HAPPY) {
    isHappy = true;
    curLH = 70;
    curRH = 70;
  }

  // Compute bounding box for Left Eye
  int drawLX = leftCenterRest - curLW / 2 + (int)eyeGazeX;
  int drawLY = eyeCenterYRest - curLH / 2 + (int)eyeGazeY;
  drawLX = constrain(drawLX, 4, SCREEN_W / 2 - EYE_GAP / 2 - curLW);
  drawLY = constrain(drawLY, 4, SCREEN_H - curLH - 4);

  // Compute bounding box for Right Eye
  int drawRX = rightCenterRest - curRW / 2 + (int)eyeGazeX;
  int drawRY = eyeCenterYRest - curRH / 2 + (int)eyeGazeY;
  drawRX = constrain(drawRX, SCREEN_W / 2 + EYE_GAP / 2, SCREEN_W - curRW - 4);
  drawRY = constrain(drawRY, 4, SCREEN_H - curRH - 4);

  // If first time drawing, clear to black
  if (!blinkPrevDrawn) {
    tft.fillScreen(0x0000);
    prevEyeLX = drawLX; prevEyeLY = drawLY; prevEyeLW = curLW; prevEyeLH = curLH;
    prevEyeRX = drawRX; prevEyeRY = drawRY; prevEyeRW = curRW; prevEyeRH = curRH;
    blinkPrevDrawn = true;
  }

  // Calculate dirty rectangle enclosing both previous and current eyes with margin
  int minDirtyX = min(min(prevEyeLX, drawLX) - 2, min(prevEyeRX, drawRX) - 2);
  int maxDirtyX = max(max(prevEyeLX + prevEyeLW, drawLX + curLW) + 2, max(prevEyeRX + prevEyeRW, drawRX + curRW) + 2);
  int minDirtyY = min(min(prevEyeLY, drawLY) - 2, min(prevEyeRY, drawRY) - 2);
  int maxDirtyY = max(max(prevEyeLY + prevEyeLH, drawLY + curLH) + 2, max(prevEyeRY + prevEyeRH, drawRY + curRH) + 2);

  minDirtyX = max(0, minDirtyX);
  maxDirtyX = min(SCREEN_W - 1, maxDirtyX);
  minDirtyY = max(0, minDirtyY);
  maxDirtyY = min(SCREEN_H - 1, maxDirtyY);
  int widthSpan = maxDirtyX - minDirtyX + 1;

  int spirX = (int)(cosf(eyeDizzyAngle * 2.0f) * 12.0f);
  int spirY = (int)(sinf(eyeDizzyAngle * 2.0f) * 12.0f);
  uint16_t lineBuf[SCREEN_W];

  tft.startWrite();
  for (int py = minDirtyY; py <= maxDirtyY; py++) {
    // 1. Fill scanline with pure black background
    for (int c = 0; c < widthSpan; c++) {
      lineBuf[c] = 0x0000;
    }

    // 2. Render Left Eye onto scanline
    if (py >= drawLY && py < drawLY + curLH) {
      int x_start, x_end;
      if (py < drawLY + curLR) {
        int dy = (drawLY + curLR) - py;
        int dx = (int)sqrtf((float)max(0, curLR * curLR - dy * dy));
        x_start = (drawLX + curLR) - dx;
        x_end   = (drawLX + curLW - curLR) + dx - 1;
      } else if (py >= drawLY + curLH - curLR) {
        int dy = py - (drawLY + curLH - 1 - curLR);
        int dx = (int)sqrtf((float)max(0, curLR * curLR - dy * dy));
        x_start = (drawLX + curLR) - dx;
        x_end   = (drawLX + curLW - curLR) + dx - 1;
      } else {
        x_start = drawLX;
        x_end   = drawLX + curLW - 1;
      }
      x_start = max(drawLX, min(drawLX + curLW - 1, x_start));
      x_end   = max(drawLX, min(drawLX + curLW - 1, x_end));

      for (int px = x_start; px <= x_end; px++) {
        if (px < minDirtyX || px > maxDirtyX) continue;
        int colIdx = px - minDirtyX;

        // Outer glow border
        if (px == x_start || px == x_end || py == drawLY || py == drawLY + curLH - 1) {
          lineBuf[colIdx] = themeBorder();
        } else if (((px - drawLX) % 4 == 0) || ((py - drawLY) % 4 == 0)) {
          // LED matrix grid lines matching reference photo
          lineBuf[colIdx] = (blinkEyeColor == 0x07FF) ? 0x0375 : ((blinkEyeColor >> 1) & 0x7BEF);
        } else {
          lineBuf[colIdx] = blinkEyeColor;
        }

        // Happy emote cutout (crescent smiling arc ^ _ ^)
        if (isHappy) {
          int hx = px - (drawLX + curLW / 2);
          int hy = py - (drawLY + curLH + 14);
          if (hx * hx + hy * hy <= (curLW / 2 + 6) * (curLW / 2 + 6)) {
            lineBuf[colIdx] = 0x0000;
          }
        }

        // Dizzy spiral pupil cutout
        if (isDizzy && curLH > 30) {
          int dpx = px - (drawLX + curLW / 2 + spirX);
          int dpy = py - (drawLY + curLH / 2 + spirY);
          if (dpx * dpx + dpy * dpy <= 7 * 7) {
            lineBuf[colIdx] = 0x0000;
          }
        }
      }
    }

    // 3. Render Right Eye onto scanline
    if (py >= drawRY && py < drawRY + curRH) {
      int x_start, x_end;
      if (py < drawRY + curRR) {
        int dy = (drawRY + curRR) - py;
        int dx = (int)sqrtf((float)max(0, curRR * curRR - dy * dy));
        x_start = (drawRX + curRR) - dx;
        x_end   = (drawRX + curRW - curRR) + dx - 1;
      } else if (py >= drawRY + curRH - curRR) {
        int dy = py - (drawRY + curRH - 1 - curRR);
        int dx = (int)sqrtf((float)max(0, curRR * curRR - dy * dy));
        x_start = (drawRX + curRR) - dx;
        x_end   = (drawRX + curRW - curRR) + dx - 1;
      } else {
        x_start = drawRX;
        x_end   = drawRX + curRW - 1;
      }
      x_start = max(drawRX, min(drawRX + curRW - 1, x_start));
      x_end   = max(drawRX, min(drawRX + curRW - 1, x_end));

      for (int px = x_start; px <= x_end; px++) {
        if (px < minDirtyX || px > maxDirtyX) continue;
        int colIdx = px - minDirtyX;

        // Outer glow border
        if (px == x_start || px == x_end || py == drawRY || py == drawRY + curRH - 1) {
          lineBuf[colIdx] = themeBorder();
        } else if (((px - drawRX) % 4 == 0) || ((py - drawRY) % 4 == 0)) {
          // LED matrix grid lines matching reference photo
          lineBuf[colIdx] = (blinkEyeColor == 0x07FF) ? 0x0375 : ((blinkEyeColor >> 1) & 0x7BEF);
        } else {
          lineBuf[colIdx] = blinkEyeColor;
        }

        // Happy emote cutout
        if (isHappy) {
          int hx = px - (drawRX + curRW / 2);
          int hy = py - (drawRY + curRH + 14);
          if (hx * hx + hy * hy <= (curRW / 2 + 6) * (curRW / 2 + 6)) {
            lineBuf[colIdx] = 0x0000;
          }
        }

        // Dizzy spiral pupil cutout
        if (isDizzy && curRH > 30) {
          int dpx = px - (drawRX + curRW / 2 - spirX);
          int dpy = py - (drawRY + curRH / 2 - spirY);
          if (dpx * dpx + dpy * dpy <= 7 * 7) {
            lineBuf[colIdx] = 0x0000;
          }
        }
      }
    }

    // 4. Blast scanline directly to ST7789
    tft.setAddrWindow(minDirtyX, py, widthSpan, 1);
    tft.writePixels(lineBuf, widthSpan);
  }
  tft.endWrite();

  prevEyeLX = drawLX; prevEyeLY = drawLY; prevEyeLW = curLW; prevEyeLH = curLH;
  prevEyeRX = drawRX; prevEyeRY = drawRY; prevEyeRW = curRW; prevEyeRH = curRH;
}

// ─────────────────────────────────────────────────────────────────────────────
// BMO (Beemo) Theme Implementation - Matching User Reference Photo
// ─────────────────────────────────────────────────────────────────────────────
enum class BmoEmote { NORMAL, BLINK, HAPPY, WINK, TALK, SURPRISED };
static BmoEmote bmoCurrentEmote = BmoEmote::NORMAL;
static unsigned long bmoEmoteUntil = 0;
static unsigned long bmoNextEmoteTime = 0;
static unsigned long bmoLastMotionTime = 0;
static unsigned long bmoDizzyUntil = 0;

static float smoothBmoX = 0.0f;
static float smoothBmoY = 0.0f;
static float bmoGazeX = 0.0f;
static float bmoGazeY = 0.0f;
static float bmoDizzyAngle = 0.0f;

static int prevBmoLX = -100, prevBmoLY = -100, prevBmoLR = 0;
static int prevBmoRX = -100, prevBmoRY = -100, prevBmoRR = 0;
static int prevBmoMX = -100, prevBmoMY = -100, prevBmoMW = 0, prevBmoMH = 0;
static bool bmoPrevDrawn = false;

void triggerBmoEmote(BmoEmote em, unsigned long durMs) {
  bmoCurrentEmote = em;
  bmoEmoteUntil = millis() + durMs;
}

void updateBMO(float dt, sensors_event_t &accel, sensors_event_t &gyro) {
  unsigned long now = millis();

  // Target gaze from tilt physics
  float targetX = -accel.acceleration.y * 3.4f;
  float targetY = accel.acceleration.x * 2.6f;

  targetX = constrain(targetX, -22.0f, 22.0f);
  targetY = constrain(targetY, -14.0f, 14.0f);

  // Motion detection for idle tracking
  float gyroRate = fabsf(gyro.gyro.x) + fabsf(gyro.gyro.y) + fabsf(gyro.gyro.z);
  if (gyroRate > 0.45f || fabsf(accel.acceleration.y) > 2.2f) {
    bmoLastMotionTime = now;
  }

  // Robust lower sensitivity shake detection -> Dizzy mode
  float totalA = sqrtf(accel.acceleration.x * accel.acceleration.x +
                       accel.acceleration.y * accel.acceleration.y +
                       accel.acceleration.z * accel.acceleration.z);
  if (fabsf(totalA - 9.8f) > 28.0f || (gyroRate > 15.0f && fabsf(totalA - 9.8f) > 16.0f)) {
    if (now >= bmoDizzyUntil) {
      bmoDizzyUntil = now + 2600;
      vibrate(70);
      buzz(587, 40); delay(20); buzz(440, 40); delay(20); buzz(330, 60);
    }
    bmoLastMotionTime = now;
  }

  // Dizzy animation wobble
  if (now < bmoDizzyUntil) {
    bmoDizzyAngle += dt * 15.0f;
    targetX = cosf(bmoDizzyAngle) * 20.0f;
    targetY = sinf(bmoDizzyAngle) * 12.0f;
  } else {
    // Idle Emotes after 3 seconds of stillness
    if (now - bmoLastMotionTime >= 3000) {
      if (now >= bmoNextEmoteTime) {
        int r = random(0, 100);
        if (r < 45) {
          // Blink
          bmoCurrentEmote = BmoEmote::BLINK;
          bmoEmoteUntil = now + 180;
        } else if (r < 65) {
          // Happy cheer
          bmoCurrentEmote = BmoEmote::HAPPY;
          bmoEmoteUntil = now + 1400;
          buzz(587, 30); delay(20); buzz(880, 40); delay(20); buzz(1175, 70);
        } else if (r < 80) {
          // Playful wink
          bmoCurrentEmote = BmoEmote::WINK;
          bmoEmoteUntil = now + 380;
          buzz(784, 30); delay(15); buzz(988, 40);
        } else if (r < 92) {
          // Talk / sing retro bleeps
          bmoCurrentEmote = BmoEmote::TALK;
          bmoEmoteUntil = now + 1200;
          buzz(523, 25); delay(35); buzz(659, 25); delay(35); buzz(784, 35);
        } else {
          // Surprised O face
          bmoCurrentEmote = BmoEmote::SURPRISED;
          bmoEmoteUntil = now + 1000;
          buzz(988, 30);
        }
        bmoNextEmoteTime = now + 2500 + random(0, 2500);
      }
    } else {
      if (bmoCurrentEmote != BmoEmote::NORMAL && now >= bmoEmoteUntil) {
        bmoCurrentEmote = BmoEmote::NORMAL;
      }
    }
  }

  smoothBmoX += (targetX - smoothBmoX) * 0.22f;
  smoothBmoY += (targetY - smoothBmoY) * 0.22f;
  bmoGazeX = smoothBmoX;
  bmoGazeY = smoothBmoY;
}

void drawBMOFrame() {
  unsigned long now = millis();

  // Face baseline positions (matching user reference photo)
  // Screen: 240 x 135
  constexpr int EYE_BASE_LX = 64;   // Left Eye center
  constexpr int EYE_BASE_RX = 176;  // Right Eye center
  constexpr int EYE_BASE_Y  = 54;   // Eye vertical level
  constexpr int MOUTH_BASE_X = 120; // Mouth center
  constexpr int MOUTH_BASE_Y = 88;  // Mouth vertical level

  bool isDizzy     = (now < bmoDizzyUntil);
  bool isBlink     = (!isDizzy && bmoCurrentEmote == BmoEmote::BLINK);
  bool isWink      = (!isDizzy && bmoCurrentEmote == BmoEmote::WINK);
  bool isHappy     = (!isDizzy && bmoCurrentEmote == BmoEmote::HAPPY);
  bool isSurprised = (!isDizzy && bmoCurrentEmote == BmoEmote::SURPRISED);
  bool isTalking   = (!isDizzy && bmoCurrentEmote == BmoEmote::TALK);

  int curEyeLX = constrain((int)(EYE_BASE_LX + bmoGazeX), 18, 110);
  int curEyeRX = constrain((int)(EYE_BASE_RX + bmoGazeX), 130, SCREEN_W - 18);
  int curEyeY  = constrain((int)(EYE_BASE_Y + bmoGazeY), 16, SCREEN_H - 45);

  int curMouthX = constrain((int)(MOUTH_BASE_X + bmoGazeX * 0.85f), 45, SCREEN_W - 45);
  int curMouthY = constrain((int)(MOUTH_BASE_Y + bmoGazeY * 0.85f), 45, SCREEN_H - 18);

  int mouthW = 72;
  int mouthH = 24;

  if (isTalking) {
    float p = sinf((float)now * 0.025f);
    mouthH = 16 + (int)(p * 8.0f);
  } else if (isSurprised) {
    mouthW = 26;
    mouthH = 26;
  } else if (isHappy) {
    mouthW = 78;
    mouthH = 28;
  }

  // If first time, fill entire screen with BMO mint background
  if (!bmoPrevDrawn) {
    tft.fillScreen(BMO_MINT_BG);
    prevBmoLX = curEyeLX; prevBmoLY = curEyeY; prevBmoLR = 8;
    prevBmoRX = curEyeRX; prevBmoRY = curEyeY; prevBmoRR = 8;
    prevBmoMX = curMouthX; prevBmoMY = curMouthY; prevBmoMW = mouthW; prevBmoMH = mouthH;
    bmoPrevDrawn = true;
  }

  // Dirty rect bounding box covering eyes and mouth previous & current
  int minDirtyX = min(min(min(prevBmoLX - 12, curEyeLX - 12), min(prevBmoRX - 12, curEyeRX - 12)),
                      min(prevBmoMX - prevBmoMW / 2 - 4, curMouthX - mouthW / 2 - 4));
  int maxDirtyX = max(max(max(prevBmoLX + 12, curEyeLX + 12), max(prevBmoRX + 12, curEyeRX + 12)),
                      max(prevBmoMX + prevBmoMW / 2 + 4, curMouthX + mouthW / 2 + 4));
  int minDirtyY = min(min(prevBmoLY - 12, curEyeY - 12), min(prevBmoMY - prevBmoMH / 2 - 4, curMouthY - mouthH / 2 - 4));
  int maxDirtyY = max(max(prevBmoLY + 12, curEyeY + 12), max(prevBmoMY + prevBmoMH / 2 + 4, curMouthY + mouthH / 2 + 4));

  minDirtyX = max(0, minDirtyX);
  maxDirtyX = min(SCREEN_W - 1, maxDirtyX);
  minDirtyY = max(0, minDirtyY);
  maxDirtyY = min(SCREEN_H - 1, maxDirtyY);
  int widthSpan = maxDirtyX - minDirtyX + 1;

  float spirX = cosf(bmoDizzyAngle * 2.0f) * 4.0f;
  float spirY = sinf(bmoDizzyAngle * 2.0f) * 4.0f;

  uint16_t lineBuf[SCREEN_W];

  tft.startWrite();
  for (int py = minDirtyY; py <= maxDirtyY; py++) {
    // 1. Fill entire span with BMO mint green
    for (int col = 0; col < widthSpan; col++) {
      lineBuf[col] = BMO_MINT_BG;
    }

    // 2. Left Eye
    if (py >= curEyeY - 9 && py <= curEyeY + 9) {
      for (int px = curEyeLX - 9; px <= curEyeLX + 9; px++) {
        if (px < minDirtyX || px > maxDirtyX) continue;
        int colIdx = px - minDirtyX;
        float dx = (float)(px - curEyeLX);
        float dy = (float)(py - curEyeY);

        if (isBlink || isWink) {
          if (fabsf(dy) <= 1.5f && fabsf(dx) <= 7.0f) {
            lineBuf[colIdx] = BMO_BLACK;
          }
        } else if (isHappy) {
          float arcY = 3.0f - (dx * dx) * 0.12f;
          if (fabsf(dy - arcY) <= 1.5f && fabsf(dx) <= 7.0f) {
            lineBuf[colIdx] = BMO_BLACK;
          }
        } else if (isDizzy) {
          float ddx = dx - spirX;
          float ddy = dy - spirY;
          if (ddx * ddx + ddy * ddy <= 6.5f * 6.5f) {
            if (fabsf(ddx) <= 1.2f || fabsf(ddy) <= 1.2f) lineBuf[colIdx] = BMO_BLACK;
            else lineBuf[colIdx] = BMO_MINT_BG;
          }
        } else {
          float rad = isSurprised ? 8.5f : 6.8f;
          if (dx * dx + dy * dy <= rad * rad) {
            lineBuf[colIdx] = BMO_BLACK;
          }
        }
      }
    }

    // 3. Right Eye
    if (py >= curEyeY - 9 && py <= curEyeY + 9) {
      for (int px = curEyeRX - 9; px <= curEyeRX + 9; px++) {
        if (px < minDirtyX || px > maxDirtyX) continue;
        int colIdx = px - minDirtyX;
        float dx = (float)(px - curEyeRX);
        float dy = (float)(py - curEyeY);

        if (isBlink) {
          if (fabsf(dy) <= 1.5f && fabsf(dx) <= 7.0f) {
            lineBuf[colIdx] = BMO_BLACK;
          }
        } else if (isHappy) {
          float arcY = 3.0f - (dx * dx) * 0.12f;
          if (fabsf(dy - arcY) <= 1.5f && fabsf(dx) <= 7.0f) {
            lineBuf[colIdx] = BMO_BLACK;
          }
        } else if (isDizzy) {
          float ddx = dx + spirX;
          float ddy = dy + spirY;
          if (ddx * ddx + ddy * ddy <= 6.5f * 6.5f) {
            if (fabsf(ddx) <= 1.2f || fabsf(ddy) <= 1.2f) lineBuf[colIdx] = BMO_BLACK;
            else lineBuf[colIdx] = BMO_MINT_BG;
          }
        } else {
          float rad = isSurprised ? 8.5f : 6.8f;
          if (dx * dx + dy * dy <= rad * rad) {
            lineBuf[colIdx] = BMO_BLACK;
          }
        }
      }
    }

    // 4. BMO Iconic Smile Mouth
    float halfMW = mouthW / 2.0f;
    float halfMH = mouthH / 2.0f;
    if (py >= curMouthY - (int)halfMH - 2 && py <= curMouthY + (int)halfMH + 2) {
      float normY = (py - curMouthY) / halfMH;
      if (normY >= -1.05f && normY <= 1.05f) {
        float spanFactor = sqrtf(max(0.0f, 1.0f - normY * normY));
        float mouthLeft  = curMouthX - halfMW * spanFactor;
        float mouthRight = curMouthX + halfMW * spanFactor;

        for (int px = (int)mouthLeft - 1; px <= (int)mouthRight + 1; px++) {
          if (px < minDirtyX || px > maxDirtyX) continue;
          int colIdx = px - minDirtyX;
          float normX = (px - curMouthX) / halfMW;

          float distSq = normX * normX + normY * normY;
          if (distSq <= 1.08f) {
            bool isOutline = (distSq >= 0.72f) || (px <= (int)mouthLeft + 1) || (px >= (int)mouthRight - 1) || (normY <= -0.82f) || (normY >= 0.82f);

            if (isOutline) {
              lineBuf[colIdx] = BMO_BLACK;
            } else {
              if (isSurprised) {
                lineBuf[colIdx] = BMO_MOUTH_DARK;
              } else if (normY <= -0.28f) {
                lineBuf[colIdx] = BMO_TEETH_WHITE;
              } else {
                float tdx = (px - curMouthX) / (halfMW * 0.60f);
                float tdy = (normY - 0.75f) / 0.70f;
                if (tdx * tdx + tdy * tdy <= 1.0f) {
                  lineBuf[colIdx] = BMO_TONGUE;
                } else {
                  lineBuf[colIdx] = BMO_MOUTH_DARK;
                }
              }
            }
          }
        }
      }
    }

    // 5. Blast line directly to ST7789
    tft.setAddrWindow(minDirtyX, py, widthSpan, 1);
    tft.writePixels(lineBuf, widthSpan);
  }
  tft.endWrite();

  prevBmoLX = curEyeLX; prevBmoLY = curEyeY; prevBmoLR = 8;
  prevBmoRX = curEyeRX; prevBmoRY = curEyeY; prevBmoRR = 8;
  prevBmoMX = curMouthX; prevBmoMY = curMouthY; prevBmoMW = mouthW; prevBmoMH = mouthH;
}

// ─────────────────────────────────────────────────────────────────────────────
// MENU mode
// ─────────────────────────────────────────────────────────────────────────────
const char *menuItems[] = { "Theme", "Air Mouse", "Tune", "Games", "Settings" };
constexpr int MENU_COUNT = 5;

void drawMenu() {
  tft.fillScreen(SPACE);
  tft.setTextWrap(false);

  // Faint decorative corner sparkles
  tft.drawPixel(8, 8, themeSecondary());
  tft.drawPixel(SCREEN_W - 12, 8, themeAccent());
  tft.drawPixel(SCREEN_W - 20, 18, 0xFFFF);
  tft.drawPixel(6, 126, themeSecondary());
  tft.drawPixel(SCREEN_W - 10, 126, themeBorder());

  // Title with dynamic theme style
  tft.setTextSize(2);
  tft.setTextColor(themeAccent());
  tft.setCursor(20, 5);
  tft.print(themeTitle());

  // Divider bar
  tft.drawFastHLine(0, 24, SCREEN_W, themeBorder());

  for (int i = 0; i < MENU_COUNT; i++) {
    int yy = 27 + i * 21;
    if (i == menuChoice) {
      // Selected bubble: theme card fill + accent outline
      tft.fillRoundRect(8, yy - 1, SCREEN_W - 16, 19, 4, themeCardBg());
      tft.drawRoundRect(8, yy - 1, SCREEN_W - 16, 19, 4, themeAccent());
      tft.setTextColor(0xFFFF);
    } else {
      tft.setTextColor(themeMuted());
    }
    tft.setTextSize(1);
    tft.setCursor(22, yy + 4);
    tft.print(menuItems[i]);

    if (i == menuChoice) {
      tft.setTextColor(themeAccent());
      tft.setCursor(SCREEN_W - 26, yy + 4);
      tft.print(">");
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// THEME SELECT mode
// ─────────────────────────────────────────────────────────────────────────────
const char *themeItems[] = { "Twinkle (Star)", "Blink (Eyes)", "BMO (Beemo)", "Custom Display", "Back" };
constexpr int THEME_COUNT = 5;
static uint8_t themeChoice = 0;

void drawThemeSelect() {
  tft.fillScreen(SPACE);
  tft.setTextWrap(false);

  // Faint decorative corner sparkles
  tft.drawPixel(8, 8, themeSecondary());
  tft.drawPixel(SCREEN_W - 12, 8, themeAccent());
  tft.drawPixel(SCREEN_W - 20, 18, 0xFFFF);
  tft.drawPixel(6, 126, themeSecondary());
  tft.drawPixel(SCREEN_W - 10, 126, themeBorder());

  // Title
  tft.setTextSize(2);
  tft.setTextColor(themeAccent());
  tft.setCursor(20, 5);
  tft.print("* THEMES *");

  // Divider bar
  tft.drawFastHLine(0, 24, SCREEN_W, themeBorder());

  for (int i = 0; i < THEME_COUNT; i++) {
    int yy = 27 + i * 21;
    if (i == themeChoice) {
      tft.fillRoundRect(8, yy - 1, SCREEN_W - 16, 20, 4, themeCardBg());
      tft.drawRoundRect(8, yy - 1, SCREEN_W - 16, 20, 4, themeAccent());
      tft.setTextColor(0xFFFF);
    } else {
      tft.setTextColor(themeMuted());
    }
    tft.setTextSize(1);
    tft.setCursor(20, yy + 4);
    tft.print(themeItems[i]);

    if (i == themeChoice) {
      tft.setTextColor(themeAccent());
      tft.setCursor(SCREEN_W - 24, yy + 4);
      tft.print(">");
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// AIR MOUSE mode
// ─────────────────────────────────────────────────────────────────────────────
void drawAirMouseScreen() {
  tft.fillScreen(SPACE);
  tft.setTextWrap(false);

  // Decorative corner sparkles
  tft.drawPixel(8, 8, themeSecondary());
  tft.drawPixel(SCREEN_W - 12, 8, themeAccent());
  tft.drawPixel(6, 126, themeBorder());

  tft.setTextSize(2); tft.setTextColor(themeAccent());
  tft.setCursor(20, 6); tft.print("* Air Mouse *");
  tft.drawFastHLine(0, 26, SCREEN_W, themeBorder());

  // BLE status bubble
  bool conn = bleConnected;
  tft.fillRoundRect(10, 30, SCREEN_W - 20, 20, 4, themeCardBg());
  tft.drawRoundRect(10, 30, SCREEN_W - 20, 20, 4, conn ? themeAccent() : themeBorder());
  tft.setTextSize(1); tft.setTextColor(conn ? (uint16_t)0xFFFF : themeMuted());
  tft.setCursor(18, 37);
  tft.print(conn ? "BLE: Connected" : "BLE: Searching..");

  tft.setTextColor(themeMuted());
  tft.setCursor(16, 58);  tft.print("L Button  : Left Click");
  tft.setCursor(16, 74);  tft.print("R Button  : Right Click");
  tft.setTextColor(themeSecondary());
  tft.setCursor(16, 96);  tft.print("Hold OK (1.5s) -> Menu");
}


static float smoothMouseX = 0.0f;
static float smoothMouseY = 0.0f;
static float accumX = 0.0f;
static float accumY = 0.0f;
static unsigned long lastMouseReportTime = 0;
static unsigned long clickStabilizeUntil = 0;
static uint8_t lastSentButtons = 0;

void updateAirMouse(sensors_event_t &accel, sensors_event_t &gyro) {
  unsigned long now = millis();

  // Read button states directly with debounce fallback for instant response
  bool leftPressed  = (digitalRead(PIN_LEFT) == LOW)  || (btnLeft.state == LOW);
  bool rightPressed = (digitalRead(PIN_RIGHT) == LOW) || (btnRight.state == LOW);

  uint8_t buttons = 0;
  if (leftPressed)  buttons |= 0x01; // Left click / drag
  if (rightPressed) buttons |= 0x02; // Right click / drag

  // Click stabilization: prevent thumb press shake from moving cursor off target
  bool buttonChanged = (buttons != lastSentButtons);
  if (buttons != 0 && lastSentButtons == 0) {
    clickStabilizeUntil = now + 120; // 120ms stable click window
    vibrate(25);
  }

  // Rate-limit BLE reports to ~12ms (approx 80Hz) to prevent BLE packet congestion,
  // but allow immediate dispatch on button state changes.
  if (!buttonChanged && (now - lastMouseReportTime < 12)) {
    return;
  }
  float dt = (lastMouseReportTime == 0) ? 0.012f : (now - lastMouseReportTime) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.05f);
  lastMouseReportTime = now;

  float rawX = 0.0f;
  float rawY = 0.0f;

  if (mpuFound) {
    // Gyro readings (rad/s)
    float gx = gyro.gyro.x; // pitch up/down when upright, yaw when flat
    float gy = gyro.gyro.y; // pitch up/down when tilted, roll
    float gz = gyro.gyro.z; // yaw left/right when held horizontally

    // Deadzone to filter out hand tremors / sensor noise
    if (fabsf(gx) < 0.05f) gx = 0.0f;
    if (fabsf(gy) < 0.05f) gy = 0.0f;
    if (fabsf(gz) < 0.05f) gz = 0.0f;

    // Weighting for horizontal yaw based on device tilt angle (upright vs flat)
    float axWeight = fabsf(accel.acceleration.x);
    float azWeight = fabsf(accel.acceleration.z);
    float totalWeight = axWeight + azWeight;
    if (totalWeight < 0.1f) totalWeight = 0.1f;

    // Horizontal yaw: smoothly blend gx (when upright) and gz (when flat/pointing forward)
    float horizRate = (gx * axWeight + gz * azWeight) / totalWeight;

    // Vertical pitch: pure rotation around the horizontal screen axis (-gy) with ZERO cross-talk
    float vertRate = -gy;

    // Sensitivity scaling factor (1..10 -> 0.2x .. 2.2x, default 5 = 1.0x)
    extern uint8_t mouseSensitivity;
    float sensMultiplier = (mouseSensitivity <= 5)
      ? (0.20f + (mouseSensitivity - 1) * 0.20f)
      : (1.00f + (mouseSensitivity - 5) * 0.24f);

    // Balanced base speed multiplier tuned for desktop navigation
    float baseScale = 4.2f * sensMultiplier;

    float speedH = fabsf(horizRate);
    float speedV = fabsf(vertRate);

    // Gentle progressive acceleration (slow movements stay accurate, flicks move across screen)
    float accelFactorH = baseScale * (1.0f + constrain(speedH * 0.25f, 0.0f, 1.5f));
    float accelFactorV = baseScale * (1.0f + constrain(speedV * 0.25f, 0.0f, 1.5f));

    rawX = horizRate * accelFactorH;
    rawY = vertRate  * accelFactorV;

    // Suppress movement jitter while clicking a button
    if (now < clickStabilizeUntil) {
      rawX *= 0.15f;
      rawY *= 0.15f;
    }
  }

  // Physics low-pass filter (smooth glide)
  smoothMouseX += (rawX - smoothMouseX) * 0.35f;
  smoothMouseY += (rawY - smoothMouseY) * 0.35f;

  // Sub-pixel fractional accumulator (enables pixel-perfect slow movements)
  accumX += smoothMouseX * (dt * 60.0f);
  accumY += smoothMouseY * (dt * 60.0f);

  int8_t dx = (int8_t)constrain((int)accumX, -127, 127);
  int8_t dy = (int8_t)constrain((int)accumY, -127, 127);
  accumX -= dx;
  accumY -= dy;

  if (bleConnected) {
    // Send report if there is movement, button change, or button held down
    if (dx != 0 || dy != 0 || buttonChanged) {
      uint8_t rep[4] = { buttons, (uint8_t)dx, (uint8_t)dy, 0 };
      mouseReport->setValue(rep, 4);
      mouseReport->notify();
      lastSentButtons = buttons;
    }
  }

  // Refresh BLE status bubble
  static unsigned long lastStatus = 0;
  if (now - lastStatus > 500) {
    bool conn2 = bleConnected;
    tft.fillRoundRect(10, 30, SCREEN_W - 20, 20, 4, STAR_BG_CARD);
    tft.drawRoundRect(10, 30, SCREEN_W - 20, 20, 4, conn2 ? STAR_PINK : STAR_BERRY);
    tft.setTextSize(1);
    tft.setTextColor(conn2 ? (uint16_t)0xFFFF : STAR_MUTED);
    tft.setCursor(18, 37);
    tft.print(conn2 ? "BLE: Connected  " : "BLE: Searching..");
    lastStatus = now;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// TUNE PLAYER mode
// ─────────────────────────────────────────────────────────────────────────────

struct Note {
  uint16_t freq;
  uint16_t duration;
};

#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
#define NOTE_C6  1047

const Note PROGMEM TUNE_HAPPY_BIRTHDAY[] = {
  {NOTE_G4, 250}, {0, 30}, {NOTE_G4, 250}, {0, 30}, {NOTE_A4, 500}, {0, 30}, {NOTE_G4, 500}, {0, 30}, {NOTE_C5, 500}, {0, 30}, {NOTE_B4, 1000}, {0, 60},
  {NOTE_G4, 250}, {0, 30}, {NOTE_G4, 250}, {0, 30}, {NOTE_A4, 500}, {0, 30}, {NOTE_G4, 500}, {0, 30}, {NOTE_D5, 500}, {0, 30}, {NOTE_C5, 1000}, {0, 60},
  {NOTE_G4, 250}, {0, 30}, {NOTE_G4, 250}, {0, 30}, {NOTE_G5, 500}, {0, 30}, {NOTE_E5, 500}, {0, 30}, {NOTE_C5, 500}, {0, 30}, {NOTE_B4, 500}, {0, 30}, {NOTE_A4, 800}, {0, 60},
  {NOTE_F5, 250}, {0, 30}, {NOTE_F5, 250}, {0, 30}, {NOTE_E5, 500}, {0, 30}, {NOTE_C5, 500}, {0, 30}, {NOTE_D5, 500}, {0, 30}, {NOTE_C5, 1200}, {0, 300}
};
constexpr int TUNE_HAPPY_BIRTHDAY_LEN = sizeof(TUNE_HAPPY_BIRTHDAY) / sizeof(Note);

const Note PROGMEM TUNE_TWINKLE[] = {
  {NOTE_C4, 380}, {0, 40}, {NOTE_C4, 380}, {0, 40}, {NOTE_G4, 380}, {0, 40}, {NOTE_G4, 380}, {0, 40},
  {NOTE_A4, 380}, {0, 40}, {NOTE_A4, 380}, {0, 40}, {NOTE_G4, 760}, {0, 80},
  {NOTE_F4, 380}, {0, 40}, {NOTE_F4, 380}, {0, 40}, {NOTE_E4, 380}, {0, 40}, {NOTE_E4, 380}, {0, 40},
  {NOTE_D4, 380}, {0, 40}, {NOTE_D4, 380}, {0, 40}, {NOTE_C4, 760}, {0, 80},
  {NOTE_G4, 380}, {0, 40}, {NOTE_G4, 380}, {0, 40}, {NOTE_F4, 380}, {0, 40}, {NOTE_F4, 380}, {0, 40},
  {NOTE_E4, 380}, {0, 40}, {NOTE_E4, 380}, {0, 40}, {NOTE_D4, 760}, {0, 80},
  {NOTE_G4, 380}, {0, 40}, {NOTE_G4, 380}, {0, 40}, {NOTE_F4, 380}, {0, 40}, {NOTE_F4, 380}, {0, 40},
  {NOTE_E4, 380}, {0, 40}, {NOTE_E4, 380}, {0, 40}, {NOTE_D4, 760}, {0, 80},
  {NOTE_C4, 380}, {0, 40}, {NOTE_C4, 380}, {0, 40}, {NOTE_G4, 380}, {0, 40}, {NOTE_G4, 380}, {0, 40},
  {NOTE_A4, 380}, {0, 40}, {NOTE_A4, 380}, {0, 40}, {NOTE_G4, 760}, {0, 80},
  {NOTE_F4, 380}, {0, 40}, {NOTE_F4, 380}, {0, 40}, {NOTE_E4, 380}, {0, 40}, {NOTE_E4, 380}, {0, 40},
  {NOTE_D4, 380}, {0, 40}, {NOTE_D4, 380}, {0, 40}, {NOTE_C4, 950}, {0, 300}
};
constexpr int TUNE_TWINKLE_LEN = sizeof(TUNE_TWINKLE) / sizeof(Note);

const Note PROGMEM TUNE_MARIO[] = {
  {NOTE_E5, 130}, {0, 20}, {NOTE_E5, 130}, {0, 130}, {NOTE_E5, 130}, {0, 130}, {NOTE_C5, 130}, {0, 20}, {NOTE_E5, 200}, {0, 100},
  {NOTE_G5, 300}, {0, 250}, {NOTE_G4, 300}, {0, 250},
  {NOTE_C5, 250}, {0, 100}, {NOTE_G4, 250}, {0, 100}, {NOTE_E4, 250}, {0, 100},
  {NOTE_A4, 200}, {0, 40}, {NOTE_B4, 200}, {0, 40}, {NOTE_AS4, 150}, {0, 20}, {NOTE_A4, 200}, {0, 40},
  {NOTE_G4, 150}, {0, 20}, {NOTE_E5, 180}, {0, 20}, {NOTE_G5, 180}, {0, 20}, {NOTE_A5, 250}, {0, 40}, {NOTE_F5, 150}, {0, 20}, {NOTE_G5, 200}, {0, 60},
  {NOTE_E5, 200}, {0, 40}, {NOTE_C5, 150}, {0, 20}, {NOTE_D5, 150}, {0, 20}, {NOTE_B4, 300}, {0, 300}
};
constexpr int TUNE_MARIO_LEN = sizeof(TUNE_MARIO) / sizeof(Note);

const Note PROGMEM TUNE_TETRIS[] = {
  {NOTE_E5, 300}, {0, 20}, {NOTE_B4, 150}, {0, 20}, {NOTE_C5, 150}, {0, 20}, {NOTE_D5, 300}, {0, 20}, {NOTE_C5, 150}, {0, 20}, {NOTE_B4, 150}, {0, 20},
  {NOTE_A4, 300}, {0, 20}, {NOTE_A4, 150}, {0, 20}, {NOTE_C5, 150}, {0, 20}, {NOTE_E5, 300}, {0, 20}, {NOTE_D5, 150}, {0, 20}, {NOTE_C5, 150}, {0, 20},
  {NOTE_B4, 450}, {0, 20}, {NOTE_C5, 150}, {0, 20}, {NOTE_D5, 300}, {0, 20}, {NOTE_E5, 300}, {0, 20},
  {NOTE_C5, 300}, {0, 20}, {NOTE_A4, 300}, {0, 20}, {NOTE_A4, 450}, {0, 150},
  {NOTE_D5, 350}, {0, 20}, {NOTE_F5, 150}, {0, 20}, {NOTE_A5, 300}, {0, 20}, {NOTE_G5, 150}, {0, 20}, {NOTE_F5, 150}, {0, 20},
  {NOTE_E5, 450}, {0, 20}, {NOTE_C5, 150}, {0, 20}, {NOTE_E5, 300}, {0, 20}, {NOTE_D5, 150}, {0, 20}, {NOTE_C5, 150}, {0, 20},
  {NOTE_B4, 300}, {0, 20}, {NOTE_B4, 150}, {0, 20}, {NOTE_C5, 150}, {0, 20}, {NOTE_D5, 300}, {0, 20}, {NOTE_E5, 300}, {0, 20},
  {NOTE_C5, 300}, {0, 20}, {NOTE_A4, 300}, {0, 20}, {NOTE_A4, 450}, {0, 300}
};
constexpr int TUNE_TETRIS_LEN = sizeof(TUNE_TETRIS) / sizeof(Note);

const char* const tuneNames[] = {
  "1. Happy Birthday",
  "2. Twinkle Star",
  "3. Super Mario",
  "4. Tetris Chiptune",
  "5. Custom MP3 Tune"
};
constexpr int TUNE_PLAYLIST_COUNT = 5;
static uint8_t currentTuneIdx = 0;

constexpr int MAX_CUSTOM_NOTES = 400;
static Note customTuneNotes[MAX_CUSTOM_NOTES];
static uint16_t customTuneNoteCount = 0;

static bool tuneIsPlaying = false;
static uint16_t tuneNoteIdx = 0;
static unsigned long tuneNoteStartTime = 0;
static uint16_t tuneNoteDuration = 0;
static uint16_t currentFreqPlaying = 0;

void loadCustomTune() {
  customTuneNoteCount = 0;
  if (!LittleFS.exists("/custom_tune.bin")) return;
  File f = LittleFS.open("/custom_tune.bin", "r");
  if (!f) return;
  if (f.size() < 6) { f.close(); return; }
  uint8_t hdr[6];
  if (f.read(hdr, 6) != 6) { f.close(); return; }
  if (hdr[0] == 'T' && hdr[1] == 'U' && hdr[2] == 'N' && hdr[3] == 'E') {
    uint16_t count = hdr[4] | (hdr[5] << 8);
    if (count > MAX_CUSTOM_NOTES) count = MAX_CUSTOM_NOTES;
    for (uint16_t i = 0; i < count; i++) {
      uint8_t raw[4];
      if (f.read(raw, 4) != 4) break;
      customTuneNotes[i].freq = ((uint16_t)raw[0] << 8) | raw[1];
      customTuneNotes[i].duration = ((uint16_t)raw[2] << 8) | raw[3];
      customTuneNoteCount++;
    }
  }
  f.close();
}

bool getTuneNote(uint8_t tuneIdx, uint16_t noteIdx, Note &outNote) {
  if (tuneIdx == 0) {
    if (noteIdx >= TUNE_HAPPY_BIRTHDAY_LEN) return false;
    outNote.freq = pgm_read_word(&TUNE_HAPPY_BIRTHDAY[noteIdx].freq);
    outNote.duration = pgm_read_word(&TUNE_HAPPY_BIRTHDAY[noteIdx].duration);
    return true;
  } else if (tuneIdx == 1) {
    if (noteIdx >= TUNE_TWINKLE_LEN) return false;
    outNote.freq = pgm_read_word(&TUNE_TWINKLE[noteIdx].freq);
    outNote.duration = pgm_read_word(&TUNE_TWINKLE[noteIdx].duration);
    return true;
  } else if (tuneIdx == 2) {
    if (noteIdx >= TUNE_MARIO_LEN) return false;
    outNote.freq = pgm_read_word(&TUNE_MARIO[noteIdx].freq);
    outNote.duration = pgm_read_word(&TUNE_MARIO[noteIdx].duration);
    return true;
  } else if (tuneIdx == 3) {
    if (noteIdx >= TUNE_TETRIS_LEN) return false;
    outNote.freq = pgm_read_word(&TUNE_TETRIS[noteIdx].freq);
    outNote.duration = pgm_read_word(&TUNE_TETRIS[noteIdx].duration);
    return true;
  } else if (tuneIdx == 4) {
    if (noteIdx >= customTuneNoteCount) return false;
    outNote = customTuneNotes[noteIdx];
    return true;
  }
  return false;
}

void startTunePlayback(uint8_t tuneIdx) {
  tuneIsPlaying = true;
  tuneNoteIdx = 0;
  Note n;
  if (getTuneNote(tuneIdx, 0, n)) {
    tuneNoteDuration = n.duration;
    tuneNoteStartTime = millis();
    currentFreqPlaying = n.freq;
    setTone(n.freq);
  } else {
    tuneIsPlaying = false;
    currentFreqPlaying = 0;
    stopTone();
  }
}

void stopTunePlayback() {
  tuneIsPlaying = false;
  currentFreqPlaying = 0;
  stopTone();
}

void toggleTunePlayback() {
  if (tuneIsPlaying) {
    stopTunePlayback();
  } else {
    startTunePlayback(currentTuneIdx);
  }
}

void drawTunePlayerScreen() {
  tft.fillScreen(SPACE);
  tft.setTextWrap(false);

  // Decorative corner sparkles
  tft.drawPixel(8, 8, themeSecondary());
  tft.drawPixel(SCREEN_W - 12, 8, themeAccent());
  tft.drawPixel(6, 126, themeBorder());

  // Title
  tft.setTextSize(2);
  tft.setTextColor(themeAccent());
  tft.setCursor(20, 5);
  tft.print("* TUNE PLAYER *");
  tft.drawFastHLine(0, 24, SCREEN_W, themeBorder());

  // Selected Tune Card
  tft.fillRoundRect(8, 28, SCREEN_W - 16, 26, 4, themeCardBg());
  tft.drawRoundRect(8, 28, SCREEN_W - 16, 26, 4, themeAccent());
  tft.setTextSize(1);
  tft.setTextColor(0xFFFF);
  tft.setCursor(16, 37);
  tft.print(tuneNames[currentTuneIdx]);

  // Status Play/Pause badge
  tft.setCursor(SCREEN_W - 74, 37);
  if (tuneIsPlaying) {
    tft.setTextColor(0x07E0); // Bright Green
    tft.print("[ PLAY ]");
  } else {
    tft.setTextColor(STAR_GOLD); // Gold
    tft.print("[ PAUSE ]");
  }

  // Footer instructions
  tft.setTextColor(themeMuted());
  tft.setCursor(12, 110);
  if (currentTuneIdx == 4 && customTuneNoteCount == 0) {
    tft.setTextColor(themeSecondary());
    tft.print("No MP3 yet! Connect WiFi: 192.168.4.1");
  } else {
    tft.print("L/R: Switch Tune | OK: Play/Pause");
  }
  tft.setTextColor(themeSecondary());
  tft.setCursor(12, 122);
  tft.print("Hold OK (1.0s) -> Menu");
}

void drawTunePlayerVisualizer() {
  constexpr int NUM_BARS = 14;
  constexpr int VIS_X = 14;
  constexpr int VIS_Y = 58;
  constexpr int VIS_W = SCREEN_W - 28; // 212 px
  constexpr int VIS_H = 46;
  constexpr int BAR_W = (VIS_W / NUM_BARS) - 3; // ~12 px per bar

  unsigned long now = millis();

  for (int i = 0; i < NUM_BARS; i++) {
    int bx = VIS_X + i * (BAR_W + 3);
    int h = 4;
    if (tuneIsPlaying) {
      float phase = (float)now * 0.009f + i * 0.55f;
      float freqFactor = (currentFreqPlaying > 0) ? constrain(currentFreqPlaying / 1200.0f, 0.4f, 1.6f) : 0.3f;
      float dynamicWave = (sinf(phase) + 1.0f) * 0.5f; // 0..1
      h = 6 + (int)(dynamicWave * (VIS_H - 10) * freqFactor);
      h = constrain(h, 4, VIS_H);
    } else {
      h = 4;
    }

    int by = VIS_Y + (VIS_H - h);

    // Clear top empty part of column
    tft.fillRect(bx, VIS_Y, BAR_W, VIS_H - h, SPACE);

    // Draw active colored bar
    uint16_t barCol = (i % 2 == 0) ? themeAccent() : themeSecondary();
    if (tuneIsPlaying && h > VIS_H * 0.7f) barCol = 0xFFFF;
    tft.fillRoundRect(bx, by, BAR_W, h, 2, barCol);
  }
}

void updateTunePlayer() {
  unsigned long now = millis();
  if (tuneIsPlaying) {
    if (now - tuneNoteStartTime >= tuneNoteDuration) {
      tuneNoteIdx++;
      Note n;
      if (getTuneNote(currentTuneIdx, tuneNoteIdx, n)) {
        tuneNoteDuration = n.duration;
        tuneNoteStartTime = now;
        currentFreqPlaying = n.freq;
        setTone(n.freq);
      } else {
        // Tune finished -> loop from start!
        tuneNoteIdx = 0;
        if (getTuneNote(currentTuneIdx, 0, n)) {
          tuneNoteDuration = n.duration;
          tuneNoteStartTime = now;
          currentFreqPlaying = n.freq;
          setTone(n.freq);
        } else {
          stopTunePlayback();
        }
      }
    }
  }

  // Smooth 30 FPS visualizer animation
  static unsigned long lastVis = 0;
  if (now - lastVis >= 33) {
    drawTunePlayerVisualizer();
    lastVis = now;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// GAMES SUITE (Space Dodge, Cyber Snake, Brick Breaker)
// ─────────────────────────────────────────────────────────────────────────────

enum class GameState { PLAYING, GAMEOVER, VICTORY };
static GameState gameState = GameState::PLAYING;

const char *gamesItems[] = { "1. Space Dodge", "2. Cyber Snake", "3. Brick Breaker", "4. Geometry Dash", "Back to Menu" };
constexpr int GAMES_COUNT = 5;
static uint8_t gamesChoice = 0;
static uint8_t currentGameIdx = 0; // 0=Space Dodge, 1=Snake, 2=Breakout, 3=Geometry Dash

static uint16_t gameScore = 0;
static uint16_t gameHighScore[4] = { 0, 0, 0, 0 };
static bool newHighScoreEarned = false;

void loadHighScores() {
  gameHighScore[0] = prefs.getUShort("hs_dodge", 0);
  gameHighScore[1] = prefs.getUShort("hs_snake", 0);
  gameHighScore[2] = prefs.getUShort("hs_break", 0);
  gameHighScore[3] = prefs.getUShort("hs_geodash", 0);
}

void saveHighScore(uint8_t gameIdx, uint16_t score) {
  if (score > gameHighScore[gameIdx]) {
    gameHighScore[gameIdx] = score;
    newHighScoreEarned = true;
    if (gameIdx == 0) prefs.putUShort("hs_dodge", score);
    else if (gameIdx == 1) prefs.putUShort("hs_snake", score);
    else if (gameIdx == 2) prefs.putUShort("hs_break", score);
    else if (gameIdx == 3) prefs.putUShort("hs_geodash", score);
  }
}

void drawGamesMenu() {
  tft.fillScreen(SPACE);
  tft.setTextWrap(false);

  // Decorative corner sparkles
  tft.drawPixel(8, 8, themeSecondary());
  tft.drawPixel(SCREEN_W - 12, 8, themeAccent());
  tft.drawPixel(SCREEN_W - 20, 18, 0xFFFF);
  tft.drawPixel(6, 126, themeSecondary());
  tft.drawPixel(SCREEN_W - 10, 126, themeBorder());

  // Title with dynamic theme style
  tft.setTextSize(2);
  tft.setTextColor(themeAccent());
  tft.setCursor(20, 5);
  tft.print("* ARCADE *");

  // Divider bar
  tft.drawFastHLine(0, 24, SCREEN_W, themeBorder());

  for (int i = 0; i < GAMES_COUNT; i++) {
    int yy = 27 + i * 21;
    if (i == gamesChoice) {
      tft.fillRoundRect(8, yy - 1, SCREEN_W - 16, 19, 4, themeCardBg());
      tft.drawRoundRect(8, yy - 1, SCREEN_W - 16, 19, 4, themeAccent());
      tft.setTextColor(0xFFFF);
    } else {
      tft.setTextColor(themeMuted());
    }
    tft.setTextSize(1);
    tft.setCursor(20, yy + 4);
    tft.print(gamesItems[i]);

    if (i == gamesChoice) {
      tft.setTextColor(themeAccent());
      tft.setCursor(SCREEN_W - 24, yy + 4);
      tft.print(">");
    }
  }
}

void drawGameOverDialog(const char* title, uint16_t score, uint16_t high) {
  int cw = 194, ch = 86;
  int cx = (SCREEN_W - cw) / 2;
  int cy = (SCREEN_H - ch) / 2;

  tft.fillRoundRect(cx, cy, cw, ch, 6, themeCardBg());
  tft.drawRoundRect(cx, cy, cw, ch, 6, themeAccent());
  tft.drawRoundRect(cx + 1, cy + 1, cw - 2, ch - 2, 5, themeBorder());

  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setTextColor(themeAccent());
  int titleLen = strlen(title);
  tft.setCursor(cx + (cw - titleLen * 12) / 2, cy + 8);
  tft.print(title);

  tft.setTextSize(1);
  tft.setTextColor(0xFFFF);
  tft.setCursor(cx + 20, cy + 32);
  tft.printf("Score : %d", score);

  tft.setTextColor(newHighScoreEarned ? STAR_GOLD : themeMuted());
  tft.setCursor(cx + 104, cy + 32);
  tft.printf("High: %d", high);

  if (newHighScoreEarned) {
    tft.setTextColor(STAR_GOLD);
    tft.setCursor(cx + (cw - 19 * 6) / 2, cy + 46);
    tft.print("* NEW HIGH SCORE! *");
  }

  tft.setTextColor(themeSecondary());
  tft.setCursor(cx + (cw - 22 * 6) / 2, cy + 64);
  tft.print("[OK] Retry  [L/R] Exit");
}

// ─────────────────────────────────────────────────────────────────────────────
// Game 1: Space Dodge (Tilt / Buttons)
// ─────────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Game 1: Space Dodge (Rich Pixel Art & Tilt / Buttons)
// ─────────────────────────────────────────────────────────────────────────────
struct DodgeObstacle {
  float x, y, vy;
  float prevX, prevY;
  uint8_t size;
  uint8_t type; // 0 = Craggy Asteroid, 1 = Fiery Comet, 2 = Cyber Drone
  uint16_t color;
};

struct DodgeStarDust {
  float x, y, vy;
  float prevX, prevY;
  uint16_t color;
};

constexpr int NUM_ASTEROIDS = 4;
constexpr int NUM_DUST = 10;
static DodgeObstacle asteroids[NUM_ASTEROIDS];
static DodgeStarDust spaceDust[NUM_DUST];

static float dodgePlayerX = 120.0f;
static float dodgePlayerPrevX = 120.0f;
static float dodgePlayerVx = 0.0f;
static float dodgeGemX = 0, dodgeGemY = -30, dodgeGemPrevX = 0, dodgeGemPrevY = -30;
static bool dodgeGemActive = false;
static unsigned long dodgeNextGemTime = 0;
static unsigned long dodgeLastScoreTime = 0;

void drawDodgePlayer(int px, int py, bool flameAnim) {
  // 1. Wings & Cannons
  tft.fillRect(px - 7, py + 2, 15, 3, themeAccent());
  tft.drawPixel(px - 7, py + 1, themeSecondary());
  tft.drawPixel(px + 7, py + 1, themeSecondary());
  tft.drawFastVLine(px - 6, py - 2, 4, 0x07FF); // Left laser blaster
  tft.drawFastVLine(px + 6, py - 2, 4, 0x07FF); // Right laser blaster

  // 2. Fuselage & Nose
  tft.fillRect(px - 2, py - 5, 5, 8, 0xFFFF);
  tft.fillRect(px - 1, py - 7, 3, 3, themeAccent());
  tft.drawPixel(px, py - 8, 0xFFFF); // Nose tip

  // 3. Cockpit Glass
  tft.fillRect(px - 1, py - 4, 3, 3, 0x07FF);
  tft.drawPixel(px, py - 3, 0xFFFF); // Canopy glint

  // 4. Animated Dual Plasma Thruster Flames
  if (flameAnim) {
    tft.drawPixel(px - 3, py + 5, STAR_GOLD);
    tft.drawPixel(px + 3, py + 5, STAR_GOLD);
    tft.drawPixel(px - 3, py + 6, 0xFD20);
    tft.drawPixel(px + 3, py + 6, 0xFD20);
    tft.drawPixel(px, py + 5, 0x07FF);
    tft.drawPixel(px, py + 6, 0x07FF);
  } else {
    tft.drawPixel(px - 3, py + 5, 0xFD20);
    tft.drawPixel(px + 3, py + 5, 0xFD20);
    tft.drawPixel(px - 3, py + 6, 0xF800);
    tft.drawPixel(px + 3, py + 6, 0xF800);
    tft.drawPixel(px, py + 5, STAR_GOLD);
  }
}

void drawDodgeObstacle(int ox, int oy, uint8_t type, uint8_t size) {
  if (type == 0) {
    // Craggy 3D Rock Asteroid with Craters
    tft.fillCircle(ox, oy, size, 0x738E); // Slate rock body
    tft.drawCircle(ox, oy, size, 0x39E7); // Darker shadow outline
    tft.drawCircle(ox - 2, oy - 1, 1, 0x2104); // Crater 1
    tft.drawPixel(ox + 2, oy + 1, 0x2104);     // Crater 2
    tft.drawPixel(ox - size + 2, oy - size + 2, 0xDEFB); // Specular rock highlight
  } else if (type == 1) {
    // Blazing Fiery Comet / Plasma Meteor
    tft.fillCircle(ox, oy, size, 0xFD20); // Fire orange
    tft.fillCircle(ox, oy, max(1, size - 2), STAR_GOLD); // Core
    tft.drawPixel(ox - 1, oy - 1, 0xFFFF);
    // Fiery tail
    tft.drawPixel(ox, oy - size - 1, 0xF800);
    tft.drawPixel(ox - 1, oy - size, 0xFD20);
    tft.drawPixel(ox + 1, oy - size, 0xFD20);
  } else {
    // Cyber Alien Scout Drone
    tft.fillRoundRect(ox - size, oy - size + 1, size * 2 + 1, size * 2 - 2, 2, 0x31A6);
    tft.drawRoundRect(ox - size, oy - size + 1, size * 2 + 1, size * 2 - 2, 2, 0x07FF);
    // Glowing red menacing sensor eye
    tft.fillCircle(ox, oy, 2, 0xF800);
    tft.drawPixel(ox, oy, 0xFFFF);
    // Wing blasters
    tft.drawPixel(ox - size, oy - size, 0xF800);
    tft.drawPixel(ox + size, oy - size, 0xF800);
  }
}

void initSpaceDodge() {
  gameState = GameState::PLAYING;
  gameScore = 0;
  newHighScoreEarned = false;
  dodgePlayerX = SCREEN_W / 2.0f;
  dodgePlayerPrevX = dodgePlayerX;
  dodgePlayerVx = 0.0f;
  dodgeGemActive = false;
  dodgeNextGemTime = millis() + random(2500, 5000);
  dodgeLastScoreTime = millis();

  // Background stars
  for (int i = 0; i < NUM_DUST; i++) {
    spaceDust[i].x = random(4, SCREEN_W - 4);
    spaceDust[i].y = random(18, SCREEN_H);
    spaceDust[i].prevX = spaceDust[i].x;
    spaceDust[i].prevY = spaceDust[i].y;
    spaceDust[i].vy = random(25, 75);
    spaceDust[i].color = (i % 3 == 0) ? 0xFFFF : ((i % 2 == 0) ? 0x8C71 : 0x52AA);
  }

  // Asteroids
  for (int i = 0; i < NUM_ASTEROIDS; i++) {
    asteroids[i].x = random(16, SCREEN_W - 16);
    asteroids[i].y = -random(20, 100) - i * 35;
    asteroids[i].prevX = asteroids[i].x;
    asteroids[i].prevY = asteroids[i].y;
    asteroids[i].vy = random(55, 95);
    asteroids[i].size = random(4, 7);
    asteroids[i].type = i % 3;
    asteroids[i].color = (i % 2 == 0) ? 0xFD20 : 0xF800;
  }

  tft.fillScreen(SPACE);
  // Header bar
  tft.drawFastHLine(0, 16, SCREEN_W, themeBorder());
  tft.setTextSize(1);
  tft.setTextColor(themeAccent());
  tft.setCursor(6, 4);
  tft.print("SPACE DODGE");
  tft.setTextColor(0xFFFF);
  tft.setCursor(140, 4);
  tft.printf("HI: %d", gameHighScore[0]);

  // Boot sound
  buzz(587, 40); delay(20); buzz(880, 50);
}

void updateSpaceDodge(float dt, sensors_event_t &accel, sensors_event_t &gyro) {
  if (gameState != GameState::PLAYING) return;
  unsigned long now = millis();

  // Score increases +1 every 200ms survived
  if (now - dodgeLastScoreTime >= 200) {
    gameScore++;
    dodgeLastScoreTime = now;
  }

  // Motion Tilt: Horizontal roll in Landscape is -accel.acceleration.y
  float tiltRoll = -accel.acceleration.y;
  if (fabs(tiltRoll) > 0.5f) {
    dodgePlayerVx += tiltRoll * 20.0f * (dt * 60.0f);
  }
  // Button control
  if (digitalRead(PIN_LEFT) == LOW) {
    dodgePlayerVx -= 160.0f * dt;
  }
  if (digitalRead(PIN_RIGHT) == LOW) {
    dodgePlayerVx += 160.0f * dt;
  }

  // Friction damping
  dodgePlayerVx *= 0.88f;
  dodgePlayerX += dodgePlayerVx * dt;
  dodgePlayerX = constrain(dodgePlayerX, 12.0f, (float)(SCREEN_W - 12));

  // 1. Drifting Background Stars
  for (int i = 0; i < NUM_DUST; i++) {
    if (spaceDust[i].prevY >= 18 && spaceDust[i].prevY <= SCREEN_H - 1) {
      tft.drawPixel((int)spaceDust[i].prevX, (int)spaceDust[i].prevY, SPACE);
    }
    spaceDust[i].y += spaceDust[i].vy * dt;
    if (spaceDust[i].y >= SCREEN_H) {
      spaceDust[i].y = 18;
      spaceDust[i].x = random(4, SCREEN_W - 4);
      spaceDust[i].vy = random(25, 75);
    }
    if (spaceDust[i].y >= 18 && spaceDust[i].y <= SCREEN_H - 1) {
      tft.drawPixel((int)spaceDust[i].x, (int)spaceDust[i].y, spaceDust[i].color);
    }
    spaceDust[i].prevX = spaceDust[i].x;
    spaceDust[i].prevY = spaceDust[i].y;
  }

  // 2. Erase previous player spaceship
  if ((int)dodgePlayerPrevX != (int)dodgePlayerX) {
    tft.fillRect((int)dodgePlayerPrevX - 9, 114, 19, 17, SPACE);
  }

  // 3. Draw Player Spaceship with animated engine fire
  bool flameToggle = ((now / 60) % 2 == 0);
  drawDodgePlayer((int)dodgePlayerX, 122, flameToggle);
  dodgePlayerPrevX = dodgePlayerX;

  // 4. Update Obstacles / Asteroids
  float speedMultiplier = 1.0f + min(1.2f, gameScore * 0.005f);
  for (int i = 0; i < NUM_ASTEROIDS; i++) {
    // Erase previous obstacle
    if (asteroids[i].prevY >= 12 && asteroids[i].prevY <= SCREEN_H + 12) {
      tft.fillRect((int)asteroids[i].prevX - asteroids[i].size - 2,
                   (int)asteroids[i].prevY - asteroids[i].size - 3,
                   (asteroids[i].size + 2) * 2 + 2,
                   (asteroids[i].size + 3) * 2, SPACE);
    }

    asteroids[i].y += asteroids[i].vy * speedMultiplier * dt;

    if (asteroids[i].y > SCREEN_H + 10) {
      asteroids[i].y = -random(15, 60);
      asteroids[i].x = random(16, SCREEN_W - 16);
      asteroids[i].vy = random(55, 95);
      asteroids[i].size = random(4, 7);
      asteroids[i].type = random(0, 3);
    }

    // Draw active obstacle
    if (asteroids[i].y >= 18 && asteroids[i].y <= SCREEN_H) {
      drawDodgeObstacle((int)asteroids[i].x, (int)asteroids[i].y, asteroids[i].type, asteroids[i].size);
    }

    asteroids[i].prevX = asteroids[i].x;
    asteroids[i].prevY = asteroids[i].y;

    // Collision detection with spaceship
    float dx = asteroids[i].x - dodgePlayerX;
    float dy = asteroids[i].y - 122;
    float hitDist = asteroids[i].size + 5.5f;
    if (dx * dx + dy * dy < hitDist * hitDist) {
      gameState = GameState::GAMEOVER;
      saveHighScore(0, gameScore);
      buzz(180, 120); delay(40); buzz(120, 180);
      vibrate(80);
      drawGameOverDialog("GAME OVER", gameScore, gameHighScore[0]);
      return;
    }
  }

  // 5. Energy Crystal / Gem logic
  if (!dodgeGemActive && now >= dodgeNextGemTime) {
    dodgeGemActive = true;
    dodgeGemX = random(20, SCREEN_W - 20);
    dodgeGemY = 16;
    dodgeGemPrevX = dodgeGemX;
    dodgeGemPrevY = dodgeGemY;
  }

  if (dodgeGemActive) {
    if (dodgeGemPrevY >= 14) {
      tft.fillRect((int)dodgeGemPrevX - 5, (int)dodgeGemPrevY - 5, 11, 11, SPACE);
    }
    dodgeGemY += 45.0f * dt;
    if (dodgeGemY > SCREEN_H + 5) {
      dodgeGemActive = false;
      dodgeNextGemTime = now + random(3000, 7000);
    } else if (dodgeGemY >= 18) {
      int gx = (int)dodgeGemX, gy = (int)dodgeGemY;
      // 8-bit glowing diamond crystal
      tft.fillTriangle(gx, gy - 4, gx - 4, gy, gx + 4, gy, STAR_GOLD);
      tft.fillTriangle(gx, gy + 4, gx - 4, gy, gx + 4, gy, 0xFD20);
      tft.drawPixel(gx, gy - 1, 0xFFFF); // Specular glint
    }
    dodgeGemPrevX = dodgeGemX;
    dodgeGemPrevY = dodgeGemY;

    // Gem collection
    float gdx = dodgeGemX - dodgePlayerX;
    float gdy = dodgeGemY - 122;
    if (gdx * gdx + gdy * gdy < 12.0f * 12.0f) {
      dodgeGemActive = false;
      dodgeNextGemTime = now + random(3000, 6000);
      gameScore += 20;
      tft.fillRect((int)dodgeGemPrevX - 5, (int)dodgeGemPrevY - 5, 11, 11, SPACE);
      buzz(988, 25); delay(15); buzz(1318, 35);
      vibrate(20);
    }
  }

  // 6. Update Score in Header
  static uint16_t lastDrawnScore = 9999;
  if (gameScore != lastDrawnScore) {
    tft.fillRect(78, 4, 55, 10, SPACE);
    tft.setTextSize(1);
    tft.setTextColor(0xFFFF);
    tft.setCursor(78, 4);
    tft.printf("SC: %d", gameScore);
    lastDrawnScore = gameScore;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Game 2: Cyber Snake (Fixed Controls & Pure Responsiveness)
// ─────────────────────────────────────────────────────────────────────────────
constexpr int SNAKE_GRID_W = 20;
constexpr int SNAKE_GRID_H = 10;
constexpr int SNAKE_CELL = 11;
constexpr int SNAKE_OX = 10;
constexpr int SNAKE_OY = 18;

struct SnakePoint {
  int8_t x, y;
};

static SnakePoint snakeBody[80];
static uint8_t snakeLength = 3;
static int8_t snakeDir = 0;     // 0=Right, 1=Down, 2=Left, 3=Up
static int8_t snakeNextDir = 0;
static SnakePoint snakeFood;
static unsigned long lastSnakeMove = 0;
static uint16_t snakeInterval = 170;

void spawnSnakeFood() {
  bool valid = false;
  while (!valid) {
    snakeFood.x = random(0, SNAKE_GRID_W);
    snakeFood.y = random(0, SNAKE_GRID_H);
    valid = true;
    for (int i = 0; i < snakeLength; i++) {
      if (snakeBody[i].x == snakeFood.x && snakeBody[i].y == snakeFood.y) {
        valid = false;
        break;
      }
    }
  }
  int fx = SNAKE_OX + snakeFood.x * SNAKE_CELL;
  int fy = SNAKE_OY + snakeFood.y * SNAKE_CELL;
  // Glowing Star Fruit
  tft.fillRoundRect(fx + 1, fy + 1, SNAKE_CELL - 2, SNAKE_CELL - 2, 3, STAR_GOLD);
  tft.drawPixel(fx + SNAKE_CELL / 2, fy + SNAKE_CELL / 2, 0xFFFF);
}

void initCyberSnake() {
  gameState = GameState::PLAYING;
  gameScore = 0;
  newHighScoreEarned = false;
  snakeLength = 3;
  snakeDir = 0;
  snakeNextDir = 0;
  snakeInterval = 170;
  lastSnakeMove = millis();

  snakeBody[0] = { 6, 5 };
  snakeBody[1] = { 5, 5 };
  snakeBody[2] = { 4, 5 };

  tft.fillScreen(SPACE);
  // Header bar
  tft.drawFastHLine(0, 16, SCREEN_W, themeBorder());
  tft.setTextSize(1);
  tft.setTextColor(themeAccent());
  tft.setCursor(6, 4);
  tft.print("CYBER SNAKE");
  tft.setTextColor(0xFFFF);
  tft.setCursor(78, 4);
  tft.printf("SC: 0");
  tft.setCursor(140, 4);
  tft.printf("HI: %d", gameHighScore[1]);

  // Arena Border
  tft.drawRect(SNAKE_OX - 1, SNAKE_OY - 1, SNAKE_GRID_W * SNAKE_CELL + 2, SNAKE_GRID_H * SNAKE_CELL + 2, themeBorder());

  // Draw initial snake
  for (int i = 0; i < snakeLength; i++) {
    int sx = SNAKE_OX + snakeBody[i].x * SNAKE_CELL;
    int sy = SNAKE_OY + snakeBody[i].y * SNAKE_CELL;
    tft.fillRoundRect(sx + 1, sy + 1, SNAKE_CELL - 2, SNAKE_CELL - 2, 2, (i == 0) ? 0xFFFF : themeAccent());
  }

  spawnSnakeFood();
  buzz(659, 40); delay(20); buzz(880, 50);
}

void updateCyberSnake(sensors_event_t &accel) {
  if (gameState != GameState::PLAYING) return;
  unsigned long now = millis();

  // Motion Tilt in Landscape:
  // Roll (-accel.acceleration.y): Positive = Right, Negative = Left
  // Pitch (accel.acceleration.x):  Positive = Down,  Negative = Up
  float tiltRoll  = -accel.acceleration.y;
  float tiltPitch = accel.acceleration.x;

  if (tiltRoll > 2.2f && snakeDir != 2) snakeNextDir = 0;       // Tilt Right -> Move Right
  else if (tiltRoll < -2.2f && snakeDir != 0) snakeNextDir = 2; // Tilt Left  -> Move Left
  else if (tiltPitch > 2.2f && snakeDir != 3) snakeNextDir = 1; // Tilt Down  -> Move Down
  else if (tiltPitch < -2.2f && snakeDir != 1) snakeNextDir = 3; // Tilt Up    -> Move Up

  if (now - lastSnakeMove < snakeInterval) return;
  lastSnakeMove = now;

  snakeDir = snakeNextDir;
  int8_t nextX = snakeBody[0].x;
  int8_t nextY = snakeBody[0].y;

  if (snakeDir == 0) nextX++;
  else if (snakeDir == 1) nextY++;
  else if (snakeDir == 2) nextX--;
  else if (snakeDir == 3) nextY--;

  // Check wall collision
  if (nextX < 0 || nextX >= SNAKE_GRID_W || nextY < 0 || nextY >= SNAKE_GRID_H) {
    gameState = GameState::GAMEOVER;
    saveHighScore(1, gameScore);
    buzz(180, 120); delay(30); buzz(120, 180);
    vibrate(70);
    drawGameOverDialog("GAME OVER", gameScore, gameHighScore[1]);
    return;
  }

  // Check self collision
  for (int i = 0; i < snakeLength - 1; i++) {
    if (snakeBody[i].x == nextX && snakeBody[i].y == nextY) {
      gameState = GameState::GAMEOVER;
      saveHighScore(1, gameScore);
      buzz(180, 120); delay(30); buzz(120, 180);
      vibrate(70);
      drawGameOverDialog("GAME OVER", gameScore, gameHighScore[1]);
      return;
    }
  }

  bool ateFood = (nextX == snakeFood.x && nextY == snakeFood.y);

  if (!ateFood) {
    // Erase tail
    int tx = SNAKE_OX + snakeBody[snakeLength - 1].x * SNAKE_CELL;
    int ty = SNAKE_OY + snakeBody[snakeLength - 1].y * SNAKE_CELL;
    tft.fillRect(tx + 1, ty + 1, SNAKE_CELL - 2, SNAKE_CELL - 2, SPACE);
  } else {
    if (snakeLength < 79) snakeLength++;
    gameScore += 10;
    snakeInterval = max(65, snakeInterval - 3);
    buzz(880, 20); delay(10); buzz(1175, 30);
    vibrate(18);

    // Update Score
    tft.fillRect(78, 4, 55, 10, SPACE);
    tft.setTextSize(1);
    tft.setTextColor(0xFFFF);
    tft.setCursor(78, 4);
    tft.printf("SC: %d", gameScore);
  }

  // Shift body
  for (int i = snakeLength - 1; i > 0; i--) {
    snakeBody[i] = snakeBody[i - 1];
  }
  snakeBody[0] = { nextX, nextY };

  // Redraw previous head as body color
  if (snakeLength > 1) {
    int bx = SNAKE_OX + snakeBody[1].x * SNAKE_CELL;
    int by = SNAKE_OY + snakeBody[1].y * SNAKE_CELL;
    tft.fillRoundRect(bx + 1, by + 1, SNAKE_CELL - 2, SNAKE_CELL - 2, 2, themeAccent());
  }

  // Draw new head
  int hx = SNAKE_OX + snakeBody[0].x * SNAKE_CELL;
  int hy = SNAKE_OY + snakeBody[0].y * SNAKE_CELL;
  tft.fillRoundRect(hx + 1, hy + 1, SNAKE_CELL - 2, SNAKE_CELL - 2, 3, 0xFFFF);
  tft.drawPixel(hx + SNAKE_CELL / 2, hy + SNAKE_CELL / 2, themeAccent());

  if (ateFood) {
    spawnSnakeFood();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Game 3: Brick Breaker (Calibrated Platform Sensitivity & Physics)
// ─────────────────────────────────────────────────────────────────────────────
constexpr int BRICK_ROWS = 3;
constexpr int BRICK_COLS = 8;
constexpr int BRICK_W = 26;
constexpr int BRICK_H = 8;
constexpr int BRICK_OX = 8;
constexpr int BRICK_OY = 24;

static bool breakBricks[BRICK_ROWS][BRICK_COLS];
static uint8_t breakBricksRemaining = 24;
static float breakPaddleX = 100.0f;
static float breakPaddlePrevX = 100.0f;
static float breakPaddleSmoothVx = 0.0f;
static float breakPaddleW = 38.0f;
constexpr float BREAK_PADDLE_Y = 126.0f;
static float breakBallX = 120.0f, breakBallY = 100.0f;
static float breakBallPrevX = 120.0f, breakBallPrevY = 100.0f;
static float breakBallVx = 75.0f, breakBallVy = -75.0f;
static uint8_t breakLives = 3;

void initBrickBreaker() {
  gameState = GameState::PLAYING;
  gameScore = 0;
  breakLives = 3;
  breakBricksRemaining = BRICK_ROWS * BRICK_COLS;
  newHighScoreEarned = false;

  breakPaddleX = (SCREEN_W - breakPaddleW) / 2.0f;
  breakPaddlePrevX = breakPaddleX;
  breakPaddleSmoothVx = 0.0f;
  breakBallX = breakPaddleX + breakPaddleW / 2.0f;
  breakBallY = BREAK_PADDLE_Y - 8.0f;
  breakBallPrevX = breakBallX;
  breakBallPrevY = breakBallY;
  breakBallVx = 70.0f;
  breakBallVy = -70.0f;

  for (int r = 0; r < BRICK_ROWS; r++) {
    for (int c = 0; c < BRICK_COLS; c++) {
      breakBricks[r][c] = true;
    }
  }

  tft.fillScreen(SPACE);
  // Header bar
  tft.drawFastHLine(0, 16, SCREEN_W, themeBorder());
  tft.setTextSize(1);
  tft.setTextColor(themeAccent());
  tft.setCursor(6, 4);
  tft.print("BRICK BREAKER");
  tft.setTextColor(0xFFFF);
  tft.setCursor(95, 4);
  tft.printf("SC:0");
  tft.setCursor(136, 4);
  tft.printf("HP:%d", breakLives);
  tft.setCursor(175, 4);
  tft.printf("HI:%d", gameHighScore[2]);

  // Draw Bricks with 3D bevel
  for (int r = 0; r < BRICK_ROWS; r++) {
    uint16_t rowColor = (r == 0) ? themeAccent() : ((r == 1) ? themeSecondary() : themeBorder());
    for (int c = 0; c < BRICK_COLS; c++) {
      int bx = BRICK_OX + c * (BRICK_W + 2);
      int by = BRICK_OY + r * (BRICK_H + 3);
      tft.fillRoundRect(bx, by, BRICK_W, BRICK_H, 2, rowColor);
      tft.drawPixel(bx + 1, by + 1, 0xFFFF);
    }
  }

  // Draw Paddle
  tft.fillRoundRect((int)breakPaddleX, (int)BREAK_PADDLE_Y, (int)breakPaddleW, 5, 2, themeAccent());
  // Draw Ball
  tft.fillCircle((int)breakBallX, (int)breakBallY, 3, 0xFFFF);

  buzz(523, 40); delay(20); buzz(784, 50);
}

void updateBrickBreaker(float dt, sensors_event_t &accel) {
  if (gameState != GameState::PLAYING) return;

  // Calibrated Low-Sensitivity Paddle Tilt Control
  float tiltRoll = -accel.acceleration.y;
  float targetPaddleVx = 0.0f;
  if (fabs(tiltRoll) > 0.4f) {
    targetPaddleVx = tiltRoll * 28.0f; // Lowered sensitivity for high precision
  }
  // Fine button controls
  if (digitalRead(PIN_LEFT) == LOW) {
    targetPaddleVx -= 95.0f;
  }
  if (digitalRead(PIN_RIGHT) == LOW) {
    targetPaddleVx += 95.0f;
  }

  // Low-pass smooth interpolation
  breakPaddleSmoothVx += (targetPaddleVx - breakPaddleSmoothVx) * 0.22f;
  breakPaddleX += breakPaddleSmoothVx * (dt * 60.0f) * 0.45f;
  breakPaddleX = constrain(breakPaddleX, 4.0f, (float)(SCREEN_W - breakPaddleW - 4));

  // Redraw paddle if moved
  if ((int)breakPaddlePrevX != (int)breakPaddleX) {
    tft.fillRect((int)breakPaddlePrevX, (int)BREAK_PADDLE_Y, (int)breakPaddleW, 5, SPACE);
    tft.fillRoundRect((int)breakPaddleX, (int)BREAK_PADDLE_Y, (int)breakPaddleW, 5, 2, themeAccent());
    breakPaddlePrevX = breakPaddleX;
  }

  // Ball movement
  breakBallX += breakBallVx * dt;
  breakBallY += breakBallVy * dt;

  // Wall collisions
  if (breakBallX <= 5.0f) {
    breakBallX = 5.0f;
    breakBallVx = fabs(breakBallVx);
    buzz(440, 10);
  } else if (breakBallX >= SCREEN_W - 5.0f) {
    breakBallX = SCREEN_W - 5.0f;
    breakBallVx = -fabs(breakBallVx);
    buzz(440, 10);
  }

  if (breakBallY <= 20.0f) {
    breakBallY = 20.0f;
    breakBallVy = fabs(breakBallVy);
    buzz(440, 10);
  }

  // Paddle collision
  if (breakBallY >= BREAK_PADDLE_Y - 4.0f && breakBallY <= BREAK_PADDLE_Y + 4.0f && breakBallVy > 0) {
    if (breakBallX >= breakPaddleX - 3.0f && breakBallX <= breakPaddleX + breakPaddleW + 3.0f) {
      breakBallVy = -fabs(breakBallVy);
      float hitOffset = (breakBallX - (breakPaddleX + breakPaddleW / 2.0f)) / (breakPaddleW / 2.0f);
      breakBallVx = hitOffset * 95.0f;
      buzz(600, 15);
      vibrate(12);
    }
  }

  // Brick collisions
  for (int r = 0; r < BRICK_ROWS; r++) {
    for (int c = 0; c < BRICK_COLS; c++) {
      if (!breakBricks[r][c]) continue;
      int bx = BRICK_OX + c * (BRICK_W + 2);
      int by = BRICK_OY + r * (BRICK_H + 3);

      if (breakBallX >= bx - 3 && breakBallX <= bx + BRICK_W + 3 &&
          breakBallY >= by - 3 && breakBallY <= by + BRICK_H + 3) {
        breakBricks[r][c] = false;
        breakBricksRemaining--;
        tft.fillRect(bx, by, BRICK_W, BRICK_H, SPACE);

        breakBallVy = -breakBallVy;
        gameScore += (3 - r) * 10;
        buzz(520 + (2 - r) * 140, 15);
        vibrate(15);

        // Update score
        tft.fillRect(95, 4, 38, 10, SPACE);
        tft.setTextSize(1);
        tft.setTextColor(0xFFFF);
        tft.setCursor(95, 4);
        tft.printf("SC:%d", gameScore);

        if (breakBricksRemaining == 0) {
          gameState = GameState::VICTORY;
          saveHighScore(2, gameScore);
          buzz(523, 60); delay(20); buzz(659, 60); delay(20); buzz(784, 60); delay(20); buzz(1046, 120);
          drawGameOverDialog("VICTORY!", gameScore, gameHighScore[2]);
          return;
        }
        break;
      }
    }
  }

  // Ball lost below paddle
  if (breakBallY > SCREEN_H + 4) {
    breakLives--;
    tft.fillRect((int)breakBallPrevX - 3, (int)breakBallPrevY - 3, 7, 7, SPACE);

    if (breakLives == 0) {
      gameState = GameState::GAMEOVER;
      saveHighScore(2, gameScore);
      buzz(180, 120); delay(30); buzz(120, 180);
      vibrate(70);
      drawGameOverDialog("GAME OVER", gameScore, gameHighScore[2]);
      return;
    } else {
      buzz(260, 60);
      vibrate(30);
      tft.fillRect(136, 4, 35, 10, SPACE);
      tft.setTextSize(1);
      tft.setTextColor(0xFFFF);
      tft.setCursor(136, 4);
      tft.printf("HP:%d", breakLives);

      breakBallX = breakPaddleX + breakPaddleW / 2.0f;
      breakBallY = BREAK_PADDLE_Y - 8.0f;
      breakBallVx = 70.0f;
      breakBallVy = -70.0f;
      breakBallPrevX = breakBallX;
      breakBallPrevY = breakBallY;
      delay(400);
      return;
    }
  }

  // Redraw Ball
  tft.fillRect((int)breakBallPrevX - 3, (int)breakBallPrevY - 3, 7, 7, SPACE);
  tft.fillCircle((int)breakBallX, (int)breakBallY, 3, 0xFFFF);
  breakBallPrevX = breakBallX;
  breakBallPrevY = breakBallY;
}

// ─────────────────────────────────────────────────────────────────────────────
// Game 4: Geometry Dash (Rhythm Platformer)
// ─────────────────────────────────────────────────────────────────────────────
constexpr float GD_FLOOR_Y      = 114.0f;
constexpr float GD_GRAVITY      = 720.0f;
constexpr float GD_JUMP_VEL     = -240.0f;
constexpr float GD_PAD_VEL      = -310.0f;
constexpr float GD_SPEED        = 130.0f;
constexpr int   GD_CUBE_W       = 14;
constexpr int   GD_CUBE_H       = 14;
constexpr float GD_PLAYER_X     = 42.0f;

enum class GdType : uint8_t {
  SPIKE_1,        // Single ground spike
  SPIKE_2,        // Double spike
  SPIKE_3,        // Triple spike
  BLOCK_1,        // 1-tile block on ground
  BLOCK_2_HIGH,   // 1-tile block in air
  STEP_PLATFORM,  // Step platform (3 blocks wide)
  PAD_JUMP,       // Yellow jump pad
  PORTAL_END      // Victory portal
};

struct GdObject {
  float wx;       // World X position
  float wy;       // World Y position
  GdType type;
  uint8_t w;      // Bounding box width
  uint8_t h;      // Bounding box height
};

const GdObject PROGMEM GD_LEVEL[] = {
  { 220, GD_FLOOR_Y, GdType::SPIKE_1, 12, 14 },
  { 340, GD_FLOOR_Y, GdType::SPIKE_1, 12, 14 },
  { 460, GD_FLOOR_Y, GdType::BLOCK_1, 16, 14 },
  { 580, GD_FLOOR_Y, GdType::SPIKE_2, 24, 14 },
  { 700, GD_FLOOR_Y, GdType::STEP_PLATFORM, 44, 14 },
  { 720, GD_FLOOR_Y - 14, GdType::SPIKE_1, 12, 14 },
  { 840, GD_FLOOR_Y, GdType::PAD_JUMP, 16, 6 },
  { 910, GD_FLOOR_Y, GdType::SPIKE_3, 36, 14 },
  { 1040, GD_FLOOR_Y, GdType::BLOCK_1, 16, 14 },
  { 1080, GD_FLOOR_Y - 20, GdType::BLOCK_2_HIGH, 16, 14 },
  { 1120, GD_FLOOR_Y - 38, GdType::BLOCK_2_HIGH, 32, 14 },
  { 1240, GD_FLOOR_Y, GdType::SPIKE_2, 24, 14 },
  { 1360, GD_FLOOR_Y, GdType::PAD_JUMP, 16, 6 },
  { 1430, GD_FLOOR_Y, GdType::SPIKE_3, 36, 14 },
  { 1490, GD_FLOOR_Y - 30, GdType::BLOCK_2_HIGH, 32, 14 },
  { 1620, GD_FLOOR_Y, GdType::SPIKE_2, 24, 14 },
  { 1740, GD_FLOOR_Y, GdType::STEP_PLATFORM, 48, 14 },
  { 1880, GD_FLOOR_Y, GdType::SPIKE_3, 36, 14 },
  { 2020, GD_FLOOR_Y, GdType::PAD_JUMP, 16, 6 },
  { 2180, GD_FLOOR_Y, GdType::PORTAL_END, 16, 32 }
};
constexpr int GD_LEVEL_COUNT = sizeof(GD_LEVEL) / sizeof(GdObject);
constexpr float GD_TOTAL_DISTANCE = 2200.0f;

static float gdWorldX = 0.0f;
static float gdPlayerY = GD_FLOOR_Y - GD_CUBE_H;
static float gdPlayerPrevY = gdPlayerY;
static float gdPlayerVy = 0.0f;
static float gdRotation = 0.0f;
static bool  gdOnGround = true;
static bool  gdJumpRequested = false;
static uint16_t gdAttempts = 1;
static unsigned long gdLastBeatTime = 0;
static uint8_t gdBeatIdx = 0;
static float gdObsPrevScreenX[GD_LEVEL_COUNT];

void drawGdCube(int cx, int cy, float rot, bool isDead) {
  if (isDead) return;

  if (currentTheme == 0) {
    // === Twinkle Star Character (14x14 pixel-art Star with cute eyes & blush) ===
    static const uint16_t STAR_14X14[14][14] PROGMEM = {
      { 0,0,0,0,0,0,STAR_GOLD,STAR_GOLD,0,0,0,0,0,0 },
      { 0,0,0,0,0,STAR_GOLD,0xFFFF,0xFFFF,STAR_GOLD,0,0,0,0,0 },
      { 0,0,0,0,STAR_GOLD,STAR_GOLD,0xFFFF,0xFFFF,STAR_GOLD,STAR_GOLD,0,0,0,0 },
      { 0,0,0,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,0,0,0 },
      { STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD },
      { 0,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,0 },
      { 0,0,STAR_GOLD,STAR_GOLD,0x0000,0xFFFF,STAR_GOLD,STAR_GOLD,0x0000,0xFFFF,STAR_GOLD,STAR_GOLD,0,0 },
      { 0,0,STAR_GOLD,STAR_GOLD,0x0000,0x0000,STAR_GOLD,STAR_GOLD,0x0000,0x0000,STAR_GOLD,STAR_GOLD,0,0 },
      { 0,0,0,STAR_GOLD,STAR_PINK,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_PINK,STAR_GOLD,0,0,0 },
      { 0,0,0,STAR_GOLD,STAR_GOLD,0x0000,0x0000,0x0000,0x0000,STAR_GOLD,STAR_GOLD,0,0,0 },
      { 0,0,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,0,0,STAR_GOLD,STAR_GOLD,STAR_GOLD,STAR_GOLD,0,0 },
      { 0,STAR_GOLD,STAR_GOLD,STAR_GOLD,0,0,0,0,0,0,STAR_GOLD,STAR_GOLD,STAR_GOLD,0 },
      { STAR_GOLD,STAR_GOLD,STAR_GOLD,0,0,0,0,0,0,0,0,STAR_GOLD,STAR_GOLD,STAR_GOLD },
      { STAR_GOLD,STAR_GOLD,0,0,0,0,0,0,0,0,0,0,STAR_GOLD,STAR_GOLD }
    };

    for (int r = 0; r < 14; r++) {
      for (int c = 0; c < 14; c++) {
        uint16_t col = pgm_read_word(&STAR_14X14[r][c]);
        if (col != 0) {
          tft.drawPixel(cx + c, cy + r, col);
        }
      }
    }
  } else if (currentTheme == 1) {
    // === Blink Cyber Robot Cube ===
    tft.fillRect(cx, cy, GD_CUBE_W, GD_CUBE_H, BLINK_DARK_TEAL);
    tft.drawRect(cx, cy, GD_CUBE_W, GD_CUBE_H, BLINK_CYAN);
    tft.drawRect(cx + 1, cy + 1, GD_CUBE_W - 2, GD_CUBE_H - 2, 0x0000);
    // Glowing Cyber Eye in center
    tft.fillRect(cx + 3, cy + 4, 8, 6, 0x0000);
    tft.fillRect(cx + 4, cy + 5, 6, 4, BLINK_CYAN);
    tft.fillRect(cx + 6, cy + 6, 2, 2, 0xFFFF);
  } else {
    // === BMO Cube ===
    tft.fillRect(cx, cy, GD_CUBE_W, GD_CUBE_H, BMO_TONGUE);
    tft.drawRect(cx, cy, GD_CUBE_W, GD_CUBE_H, BMO_BLACK);
    // BMO Face
    tft.fillCircle(cx + 4, cy + 5, 1, BMO_BLACK);
    tft.fillCircle(cx + 9, cy + 5, 1, BMO_BLACK);
    tft.drawFastHLine(cx + 4, cy + 9, 6, BMO_BLACK);
    tft.fillRect(cx + 5, cy + 8, 4, 2, BMO_TEETH_WHITE);
  }
}

void initGeometryDash() {
  gameState = GameState::PLAYING;
  gameScore = 0;
  newHighScoreEarned = false;
  gdWorldX = 0.0f;
  gdPlayerY = GD_FLOOR_Y - GD_CUBE_H;
  gdPlayerPrevY = gdPlayerY;
  gdPlayerVy = 0.0f;
  gdRotation = 0.0f;
  gdOnGround = true;
  gdJumpRequested = false;
  gdLastBeatTime = millis();
  gdBeatIdx = 0;

  for (int i = 0; i < GD_LEVEL_COUNT; i++) {
    gdObsPrevScreenX[i] = -999.0f;
  }

  tft.fillScreen(SPACE);

  // Neon Floor Bar
  tft.drawFastHLine(0, (int)GD_FLOOR_Y, SCREEN_W, themeAccent());
  tft.fillRect(0, (int)GD_FLOOR_Y + 1, SCREEN_W, SCREEN_H - (int)GD_FLOOR_Y, themeCardBg());

  // Header Bar
  tft.drawFastHLine(0, 16, SCREEN_W, themeBorder());
  tft.setTextSize(1);
  tft.setTextColor(themeAccent());
  tft.setCursor(6, 4);
  tft.print("GEOMETRY DASH");

  tft.setTextColor(0xFFFF);
  tft.setCursor(102, 4);
  tft.printf("ATT:%d", gdAttempts);

  tft.setTextColor(STAR_GOLD);
  tft.setCursor(162, 4);
  tft.printf("BEST:%d%%", gameHighScore[3]);

  // Initial draw player
  drawGdCube((int)GD_PLAYER_X, (int)gdPlayerY, 0, false);

  // Upbeat arcade start sound
  buzz(659, 40); delay(20); buzz(880, 50); delay(20); buzz(1175, 70);
}

void updateGeometryDash(float dt, sensors_event_t &accel, sensors_event_t &gyro) {
  if (gameState != GameState::PLAYING) return;
  unsigned long now = millis();

  // 1. Driving Rhythm Beats (Electronic Chiptune Bassline)
  if (now - gdLastBeatTime >= 240) {
    gdLastBeatTime = now;
    gdBeatIdx++;
    static const uint16_t GD_BASS_PITCH[] = { 196, 220, 262, 294, 220, 262, 330, 392 };
    buzz(GD_BASS_PITCH[gdBeatIdx % 8], 15);
  }

  // 2. Physics & Jump Input (Strict 1 Jump per Right Button Press)
  if (gdJumpRequested && gdOnGround) {
    gdPlayerVy = GD_JUMP_VEL;
    gdOnGround = false;
    gdJumpRequested = false;
    buzz(1046, 15);
    vibrate(12);
  }
  gdJumpRequested = false;

  // If above floor level, assume in-air unless confirmed supported by platform
  if (gdPlayerY < GD_FLOOR_Y - GD_CUBE_H) {
    gdOnGround = false;
  }

  // Apply gravity
  if (!gdOnGround) {
    gdPlayerVy += GD_GRAVITY * dt;
    gdRotation += 360.0f * 1.6f * dt;
  } else {
    gdRotation = 0.0f;
  }

  gdPlayerY += gdPlayerVy * dt;

  // Floor collision
  if (gdPlayerY >= GD_FLOOR_Y - GD_CUBE_H) {
    gdPlayerY = GD_FLOOR_Y - GD_CUBE_H;
    gdPlayerVy = 0.0f;
    gdOnGround = true;
  }

  // Advance level world scroll
  gdWorldX += GD_SPEED * dt;

  // Calculate percentage
  int currentPct = constrain((int)((gdWorldX / GD_TOTAL_DISTANCE) * 100.0f), 0, 100);
  gameScore = currentPct;

  // Update progress in header
  static int lastDrawnPct = -1;
  if (currentPct != lastDrawnPct) {
    tft.fillRect(162, 4, 76, 10, SPACE);
    tft.setTextSize(1);
    tft.setTextColor(STAR_GOLD);
    tft.setCursor(162, 4);
    tft.printf("BEST:%d%%", max((int)gameHighScore[3], currentPct));
    lastDrawnPct = currentPct;
  }

  // 3. Clear Previous Frame Artifacts for Player
  tft.fillRect((int)GD_PLAYER_X - 2, (int)gdPlayerPrevY - 2, GD_CUBE_W + 4, GD_CUBE_H + 4, SPACE);

  // 4. Render and Collision Check Obstacles (With Bounds-Safe Cleanup)
  bool playerCollision = false;

  for (int i = 0; i < GD_LEVEL_COUNT; i++) {
    GdObject obj;
    obj.wx = pgm_read_float(&GD_LEVEL[i].wx);
    obj.wy = pgm_read_float(&GD_LEVEL[i].wy);
    obj.type = (GdType)pgm_read_byte(&GD_LEVEL[i].type);
    obj.w = pgm_read_byte(&GD_LEVEL[i].w);
    obj.h = pgm_read_byte(&GD_LEVEL[i].h);

    float screenX = obj.wx - gdWorldX + GD_PLAYER_X;

    // Erase old obstacle footprint from previous frame with safe screen bounds clipping
    if (gdObsPrevScreenX[i] > -100.0f && gdObsPrevScreenX[i] < (float)(SCREEN_W + 40)) {
      int prevSX = (int)gdObsPrevScreenX[i] - 2;
      int prevSY = (int)obj.wy - obj.h - 2;
      int prevSW = obj.w + 4;
      int prevSH = obj.h + 3;

      int clLeft = max(0, prevSX);
      int clRight = min(SCREEN_W, prevSX + prevSW);
      int clTop = max(17, prevSY);
      int clBottom = min((int)GD_FLOOR_Y, prevSY + prevSH);
      if (clRight > clLeft && clBottom > clTop) {
        tft.fillRect(clLeft, clTop, clRight - clLeft, clBottom - clTop, SPACE);
      }
    }

    if (screenX > -50 && screenX < SCREEN_W + 20) {
      int sx = (int)screenX;
      int sy = (int)obj.wy;

      switch (obj.type) {
        case GdType::SPIKE_1:
          tft.fillTriangle(sx, sy, sx + 6, sy - 14, sx + 12, sy, 0xF800);
          tft.drawTriangle(sx, sy, sx + 6, sy - 14, sx + 12, sy, 0xFFFF);
          break;

        case GdType::SPIKE_2:
          tft.fillTriangle(sx, sy, sx + 6, sy - 14, sx + 12, sy, 0xF800);
          tft.drawTriangle(sx, sy, sx + 6, sy - 14, sx + 12, sy, 0xFFFF);
          tft.fillTriangle(sx + 12, sy, sx + 18, sy - 14, sx + 24, sy, 0xF800);
          tft.drawTriangle(sx + 12, sy, sx + 18, sy - 14, sx + 24, sy, 0xFFFF);
          break;

        case GdType::SPIKE_3:
          for (int sp = 0; sp < 3; sp++) {
            int spx = sx + sp * 12;
            tft.fillTriangle(spx, sy, spx + 6, sy - 14, spx + 12, sy, 0xF800);
            tft.drawTriangle(spx, sy, spx + 6, sy - 14, spx + 12, sy, 0xFFFF);
          }
          break;

        case GdType::BLOCK_1:
        case GdType::BLOCK_2_HIGH:
        case GdType::STEP_PLATFORM:
          tft.fillRoundRect(sx, sy - obj.h, obj.w, obj.h, 2, themeCardBg());
          tft.drawRoundRect(sx, sy - obj.h, obj.w, obj.h, 2, themeAccent());
          tft.drawFastHLine(sx + 2, sy - obj.h + 2, obj.w - 4, 0xFFFF);
          break;

        case GdType::PAD_JUMP:
          tft.fillRoundRect(sx, sy - 5, obj.w, 5, 2, STAR_GOLD);
          tft.drawRoundRect(sx, sy - 5, obj.w, 5, 2, 0xFFFF);
          break;

        case GdType::PORTAL_END:
          tft.fillRoundRect(sx, sy - obj.h, obj.w, obj.h, 4, 0x07FF);
          tft.drawRoundRect(sx, sy - obj.h, obj.w, obj.h, 4, 0xFFFF);
          break;

        default: break;
      }

      gdObsPrevScreenX[i] = screenX;

      // Hitbox Check
      float px1 = GD_PLAYER_X + 2;
      float px2 = GD_PLAYER_X + GD_CUBE_W - 2;
      float py1 = gdPlayerY + 2;
      float py2 = gdPlayerY + GD_CUBE_H;

      float ox1 = screenX;
      float ox2 = screenX + obj.w;
      float oy1 = obj.wy - obj.h;
      float oy2 = obj.wy;

      if (px2 > ox1 && px1 < ox2 && py2 > oy1 && py1 < oy2) {
        if (obj.type == GdType::PAD_JUMP) {
          gdPlayerVy = GD_PAD_VEL;
          gdOnGround = false;
          buzz(1318, 20);
          vibrate(20);
        } else if (obj.type == GdType::PORTAL_END) {
          gameState = GameState::VICTORY;
          saveHighScore(3, 100);
          buzz(523, 70); delay(20); buzz(659, 70); delay(20); buzz(784, 70); delay(20); buzz(1046, 160);
          drawGameOverDialog("100% COMPLETE!", 100, 100);
          return;
        } else if (obj.type == GdType::BLOCK_1 || obj.type == GdType::BLOCK_2_HIGH || obj.type == GdType::STEP_PLATFORM) {
          if (gdPlayerPrevY + GD_CUBE_H <= oy1 + 5.0f && gdPlayerVy >= 0) {
            gdPlayerY = oy1 - GD_CUBE_H;
            gdPlayerVy = 0.0f;
            gdOnGround = true;
          } else {
            playerCollision = true;
          }
        } else {
          playerCollision = true;
        }
      }
    } else {
      gdObsPrevScreenX[i] = -999.0f;
    }
  }

  // Redraw solid floor line and moving grid markers
  tft.drawFastHLine(0, (int)GD_FLOOR_Y, SCREEN_W, themeAccent());
  int floorOffset = ((int)gdWorldX) % 24;
  for (int fx = 0; fx < SCREEN_W; fx += 24) {
    int lineX = fx - floorOffset;
    if (lineX >= 0 && lineX < SCREEN_W) {
      tft.drawFastVLine(lineX, (int)GD_FLOOR_Y + 1, 6, themeBorder());
    }
  }

  // 5. Draw Player Character
  drawGdCube((int)GD_PLAYER_X, (int)gdPlayerY, gdRotation, false);
  gdPlayerPrevY = gdPlayerY;

  // 6. Handle Crash / Shatter Explosion
  if (playerCollision) {
    gameState = GameState::GAMEOVER;
    saveHighScore(3, currentPct);
    gdAttempts++;

    tft.fillRect((int)GD_PLAYER_X - 2, (int)gdPlayerY - 2, GD_CUBE_W + 4, GD_CUBE_H + 4, SPACE);
    for (int p = 0; p < 8; p++) {
      int ox = (int)GD_PLAYER_X + random(-12, 18);
      int oy = (int)gdPlayerY + random(-12, 18);
      tft.fillRect(ox, oy, 3, 3, (p % 2 == 0) ? themeAccent() : STAR_GOLD);
    }

    buzz(150, 160); delay(30); buzz(100, 180);
    vibrate(90);

    drawGameOverDialog("SHATTERED!", currentPct, gameHighScore[3]);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Game Master Controllers
// ─────────────────────────────────────────────────────────────────────────────
void initCurrentGame(uint8_t gameIdx) {
  loadHighScores();
  if (gameIdx == 0) initSpaceDodge();
  else if (gameIdx == 1) initCyberSnake();
  else if (gameIdx == 2) initBrickBreaker();
  else if (gameIdx == 3) initGeometryDash();
}

void updateCurrentGame(float dt, sensors_event_t &accel, sensors_event_t &gyro) {
  if (currentGameIdx == 0) updateSpaceDodge(dt, accel, gyro);
  else if (currentGameIdx == 1) updateCyberSnake(accel);
  else if (currentGameIdx == 2) updateBrickBreaker(dt, accel);
  else if (currentGameIdx == 3) updateGeometryDash(dt, accel, gyro);
}

void handleGameInput(ButtonState &btnL, ButtonState &btnR, ButtonState &btnC, bool rightLong) {
  if (currentGameIdx == 1) { // Cyber Snake intuitive button steering
    if (btnL.justPressed || rightLong) {
      if (snakeDir == 0 || snakeDir == 2) { // Moving Horizontally -> Turn UP
        snakeNextDir = 3;
      } else { // Moving Vertically -> Turn LEFT
        snakeNextDir = 2;
      }
      buzz(659, 15);
      vibrate(10);
    }
    if (btnR.justPressed) {
      if (snakeDir == 0 || snakeDir == 2) { // Moving Horizontally -> Turn DOWN
        snakeNextDir = 1;
      } else { // Moving Vertically -> Turn RIGHT
        snakeNextDir = 0;
      }
      buzz(784, 15);
      vibrate(10);
    }
    if (btnC.justReleased && !btnC.longPressHandled) {
      // Confirm cycles direction (Right -> Down -> Left -> Up)
      snakeNextDir = (snakeDir + 1) % 4;
      buzz(880, 15);
    }
  } else if (currentGameIdx == 3) { // Geometry Dash: RIGHT button to jump
    if (btnR.justPressed) {
      gdJumpRequested = true;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// CUSTOM DISPLAY mode
// ─────────────────────────────────────────────────────────────────────────────

struct CustomDisplayState {
  bool hasImage = false;
  bool isAnimated = false;
  uint16_t numFrames = 1;
  uint16_t frameDelayMs = 100;
  uint16_t currentFrame = 0;
  unsigned long lastFrameSwitchTime = 0;
};
static CustomDisplayState customDisp;

static File persistentImageFile;

void closePersistentImage() {
  if (persistentImageFile) {
    persistentImageFile.close();
  }
}

void reloadCustomDisplay() {
  closePersistentImage();
  customDisp.hasImage = false;
  customDisp.isAnimated = false;
  customDisp.numFrames = 1;
  customDisp.frameDelayMs = 100;
  customDisp.currentFrame = 0;
  customDisp.lastFrameSwitchTime = 0;

  if (!LittleFS.exists("/image.bin")) return;

  persistentImageFile = LittleFS.open("/image.bin", "r");
  if (!persistentImageFile) return;

  size_t sz = persistentImageFile.size();
  if (sz < 16) { persistentImageFile.close(); return; }

  uint8_t hdr[8];
  if (persistentImageFile.read(hdr, 8) == 8 && hdr[0] == 'T' && hdr[1] == 'W' && hdr[2] == 'I' && hdr[3] == 'N') {
    customDisp.numFrames = hdr[4] | (hdr[5] << 8);
    customDisp.frameDelayMs = hdr[6] | (hdr[7] << 8);
    if (customDisp.frameDelayMs < 40) customDisp.frameDelayMs = 80;
    if (customDisp.numFrames > 1) {
      customDisp.isAnimated = true;
    }
    customDisp.hasImage = true;
  } else if (sz >= SCREEN_W * SCREEN_H * 2) {
    customDisp.numFrames = 1;
    customDisp.isAnimated = false;
    customDisp.hasImage = true;
  }
}

void drawCustomDisplayFrame(uint16_t frameIdx) {
  if (!customDisp.hasImage) return;
  if (!persistentImageFile) {
    persistentImageFile = LittleFS.open("/image.bin", "r");
    if (!persistentImageFile) return;
  }

  constexpr uint32_t frameBytes = SCREEN_W * SCREEN_H * 2; // 64,800 bytes
  uint32_t offset = (customDisp.isAnimated || persistentImageFile.size() >= frameBytes + 8) 
                    ? (8 + (uint32_t)frameIdx * frameBytes) 
                    : 0;

  if (persistentImageFile.size() < offset + frameBytes) {
    offset = (persistentImageFile.size() >= frameBytes + 8) ? 8 : 0;
  }

  persistentImageFile.seek(offset);

  uint8_t rowRaw[SCREEN_W * 2]; // 480 bytes per scanline
  uint16_t rowPixels[SCREEN_W]; // 240 pixels per scanline

  tft.startWrite();
  for (int py = 0; py < SCREEN_H; py++) {
    int n = persistentImageFile.read(rowRaw, sizeof(rowRaw));
    if (n < (int)sizeof(rowRaw)) break;

    for (int px = 0; px < SCREEN_W; px++) {
      rowPixels[px] = ((uint16_t)rowRaw[px * 2] << 8) | rowRaw[px * 2 + 1];
    }

    tft.setAddrWindow(0, py, SCREEN_W, 1);
    tft.writePixels(rowPixels, SCREEN_W);
  }
  tft.endWrite();
}

void drawCustomDisplayPortalScreen() {
  tft.fillScreen(SPACE);
  tft.setTextWrap(false);

  // Decorative corner sparkles
  tft.drawPixel(8, 8, themeSecondary());
  tft.drawPixel(SCREEN_W - 12, 8, themeAccent());
  tft.drawPixel(6, 126, themeBorder());

  tft.setTextSize(2); tft.setTextColor(themeAccent());
  tft.setCursor(10, 6); tft.print("* Custom Display");
  tft.drawFastHLine(0, 26, SCREEN_W, themeBorder());

  // Info card
  tft.fillRoundRect(8, 30, SCREEN_W - 16, 58, 5, themeCardBg());
  tft.drawRoundRect(8, 30, SCREEN_W - 16, 58, 5, themeAccent());
  tft.setTextSize(1); tft.setTextColor(themeMuted());
  tft.setCursor(16, 37); tft.print("WiFi AP  : Twinkle-Bot");
  tft.setCursor(16, 51); tft.print("Browser  : 192.168.4.1");
  tft.setCursor(16, 65); tft.print("Upload   : Photo or GIF");
  tft.setTextColor(themeSecondary());
  tft.setCursor(16, 77); tft.print("No password required");

  tft.setTextColor(themeSecondary());
  tft.setCursor(16, 98); tft.print("Hold OK (1.0s) -> Menu");
}

void drawCustomDisplay() {
  reloadCustomDisplay();
  if (customDisp.hasImage) {
    drawCustomDisplayFrame(0);
    customDisp.currentFrame = (customDisp.numFrames > 1) ? 1 : 0;
    customDisp.lastFrameSwitchTime = millis();
  } else {
    drawCustomDisplayPortalScreen();
  }
  if (!portalActive) setupPortal();
}

void updateCustomDisplay() {
  if (!customDisp.hasImage || !customDisp.isAnimated) return;
  unsigned long now = millis();
  if (now - customDisp.lastFrameSwitchTime >= customDisp.frameDelayMs) {
    drawCustomDisplayFrame(customDisp.currentFrame);
    customDisp.currentFrame = (customDisp.currentFrame + 1) % customDisp.numFrames;
    customDisp.lastFrameSwitchTime = now;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// ─── Settings state ───────────────────────────────────────────────────────────
// Row indices: 0=Brightness, 1=Volume, 2=Mouse Sens, 3=Clear Image, 4=Back to Menu
uint8_t screenBrightness = 8;  // 0-10
uint8_t soundVolume      = 8;  // 0-10
uint8_t mouseSensitivity = 5;  // 1-10 (default 5)
constexpr int SETTINGS_COUNT = 5;
static uint8_t settingsChoice = 0;

// Draw a horizontal bar slider at (x,y) with value 0-10
static void drawSlider(int sx, int sy, uint8_t val, bool selected) {
  constexpr int BAR_W = 90;
  constexpr int BAR_H = 8;
  uint16_t bgCol   = selected ? themeCardBg() : (uint16_t)NEBULA;
  uint16_t fillCol = selected ? themeAccent() : themeMuted();
  tft.fillRect(sx, sy, BAR_W, BAR_H, bgCol);
  int filled = (val * BAR_W) / 10;
  if (filled > 0) tft.fillRect(sx, sy, filled, BAR_H, fillCol);
  tft.drawRect(sx, sy, BAR_W, BAR_H, selected ? themeAccent() : themeBorder());
  // Value label
  tft.setTextColor(selected ? themeAccent() : themeMuted());
  tft.setCursor(sx + BAR_W + 6, sy);
  tft.print(val);
}

void drawSettings() {
  tft.fillScreen(SPACE);
  tft.setTextWrap(false);

  // Decorative corner sparkles
  tft.drawPixel(8, 8, themeSecondary());
  tft.drawPixel(SCREEN_W - 12, 8, themeAccent());
  tft.drawPixel(6, 126, themeBorder());

  // Title
  tft.setTextSize(2);
  tft.setTextColor(themeAccent());
  tft.setCursor(20, 5);
  tft.print("* Settings *");
  tft.drawFastHLine(0, 24, SCREEN_W, themeBorder());

  // Row definitions
  const char *labels[]    = { "Brightness", "Sound Vol", "Mouse Sens", "Clear Image", "Back" };
  const bool  isSlider[]  = { true, true, true, false, false };
  const uint8_t sliderVal[] = { screenBrightness, soundVolume, mouseSensitivity, 0, 0 };

  for (int i = 0; i < SETTINGS_COUNT; i++) {
    int yy  = 28 + i * 21;
    bool sel = (i == settingsChoice);

    if (sel) {
      tft.fillRoundRect(8, yy - 1, SCREEN_W - 16, 20, 4, themeCardBg());
      tft.drawRoundRect(8, yy - 1, SCREEN_W - 16, 20, 4, themeAccent());
    }

    tft.setTextSize(1);
    tft.setTextColor(sel ? (uint16_t)0xFFFF : themeMuted());
    tft.setCursor(16, yy + 3);
    tft.print(labels[i]);

    if (isSlider[i]) {
      drawSlider(105, yy + 3, sliderVal[i], sel);
    } else {
      tft.setTextColor(sel ? themeAccent() : themeBorder());
      tft.setCursor(105, yy + 3);
      tft.print(sel ? "[ Press OK ]" : "- - - - -");
    }
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// WiFi captive portal
// ─────────────────────────────────────────────────────────────────────────────
void setupPortal() {
  WiFi.softAP("Twinkle-Bot");
  delay(200);
  dns.start(53, "*", WiFi.softAPIP());

  web.on("/", HTTP_GET, []() {
    String html = FPSTR(PORTAL_HTML);
    if (currentTheme == 2) {
      html.replace("THEME_CLASS", "theme-bmo");
      html.replace("HEADER_TITLE", "&#x1F916; BMO Pixel Studio");
      html.replace("HEADER_SUB", "Upload any Photo or Animated GIF for BMO");
      html.replace("SEND_BTN_TEXT", "Send to BMO");
    } else if (currentTheme == 1) {
      html.replace("THEME_CLASS", "theme-blink");
      html.replace("HEADER_TITLE", "&#x1F916; Blink Cyber Studio");
      html.replace("HEADER_SUB", "Upload any Photo or Matrix GIF for Blink Bot");
      html.replace("SEND_BTN_TEXT", "Send to Blink Bot");
    } else {
      html.replace("THEME_CLASS", "theme-twinkle");
      html.replace("HEADER_TITLE", "<span>&#x2728;</span> Twinkle Art Studio");
      html.replace("HEADER_SUB", "Upload any Photo or Animated GIF for Twinkle Star");
      html.replace("SEND_BTN_TEXT", "Send to Twinkle");
    }
    web.send(200, "text/html", html);
  });
  // Captive-portal redirects (Android / iOS / Windows)
  auto redir = []() {
    web.sendHeader("Location", "http://192.168.4.1/");
    web.send(302);
  };
  web.on("/generate_204",        HTTP_GET, redir);
  web.on("/hotspot-detect.html", HTTP_GET, redir);
  web.on("/connecttest.txt",     HTTP_GET, redir);
  web.on("/ncsi.txt",            HTTP_GET, redir);

  web.on("/upload", HTTP_POST,
    // completion handler
    []() {
      if (uploadFile) uploadFile.close();
      bool ok = LittleFS.exists("/image.bin") && (LittleFS.open("/image.bin", "r").size() > 8);
      if (ok) {
        reloadCustomDisplay();
        if (mode == TwinkleMode::CUSTOM_DISPLAY) {
          drawCustomDisplayFrame(0);
          customDisp.currentFrame = (customDisp.numFrames > 1) ? 1 : 0;
          customDisp.lastFrameSwitchTime = millis();
        }
      }
      web.send(200, "text/plain",
               ok ? "Saved successfully! Displaying on screen now."
                  : "Upload failed or file empty. Try again.");
    },
    // chunk handler
    []() {
      HTTPUpload &up = web.upload();
      if (up.status == UPLOAD_FILE_START) {
        closePersistentImage();
        if (uploadFile) uploadFile.close();
        LittleFS.remove("/image.bin");
        uploadFile = LittleFS.open("/image.bin", "w");
      } else if (up.status == UPLOAD_FILE_WRITE && uploadFile) {
        uploadFile.write(up.buf, up.currentSize);
      } else if (up.status == UPLOAD_FILE_END && uploadFile) {
        uploadFile.close();
      }
    }
  );

  web.on("/upload_tune", HTTP_POST,
    // completion handler
    []() {
      if (uploadFile) uploadFile.close();
      bool ok = LittleFS.exists("/custom_tune.bin") && (LittleFS.open("/custom_tune.bin", "r").size() >= 6);
      if (ok) {
        loadCustomTune();
        if (mode == TwinkleMode::TUNE) {
          currentTuneIdx = 4; // Switch to Custom Tune
          drawTunePlayerScreen();
          startTunePlayback(currentTuneIdx);
        }
      }
      web.send(200, "text/plain",
               ok ? "Custom tune saved successfully! Playing now."
                  : "Upload failed or empty tune file. Try again.");
    },
    // chunk handler
    []() {
      HTTPUpload &up = web.upload();
      if (up.status == UPLOAD_FILE_START) {
        if (tuneIsPlaying) stopTunePlayback();
        if (uploadFile) uploadFile.close();
        LittleFS.remove("/custom_tune.bin");
        uploadFile = LittleFS.open("/custom_tune.bin", "w");
      } else if (up.status == UPLOAD_FILE_WRITE && uploadFile) {
        uploadFile.write(up.buf, up.currentSize);
      } else if (up.status == UPLOAD_FILE_END && uploadFile) {
        uploadFile.close();
      }
    }
  );

  web.onNotFound([]() {
    web.sendHeader("Location", "http://192.168.4.1/");
    web.send(302);
  });

  web.begin();
  portalActive = true;
}

void teardownPortal() {
  closePersistentImage();
  web.stop();
  dns.stop();
  WiFi.softAPdisconnect(true);
  portalActive = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// BLE HID setup
// ─────────────────────────────────────────────────────────────────────────────
void setupBle() {
  BLEDevice::init("Twinkle Mouse");
  BLEDevice::setSecurityCallbacks(new TwinkleSecurityCallbacks());

  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new TwinkleBleCallbacks());

  bleHid = new BLEHIDDevice(srv);
  mouseReport = bleHid->inputReport(1);           // reportID = 1 matches descriptor
  bleHid->manufacturer()->setValue("Twinkle");
  bleHid->pnp(0x02, 0x045E, 0x0000, 0x0110);
  bleHid->hidInfo(0x00, 0x03);                   // 0x03 = RemoteWake + NormallyConnectable
  bleHid->reportMap((uint8_t *)mouseHidDescriptor, sizeof(mouseHidDescriptor));
  bleHid->startServices();
  bleHid->setBatteryLevel(100);

  // Proper BLE Security for persistent bonding without Windows/iOS dropouts
  BLESecurity *sec = new BLESecurity();
  sec->setAuthenticationMode(ESP_LE_AUTH_BOND);
  sec->setCapability(ESP_IO_CAP_NONE);
  sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sec->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sec->setKeySize(16);

  // Standard 31-byte compliant advertising data
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  
  BLEAdvertisementData advData;
  advData.setFlags(0x06); // General Discoverable + BR_EDR_NOT_SUPPORTED
  advData.setAppearance(0x03C2); // HID Mouse
  advData.setCompleteServices(BLEUUID((uint16_t)0x1812)); // HID Service UUID
  adv->setAdvertisementData(advData);

  BLEAdvertisementData scanData;
  scanData.setName("Twinkle Mouse");
  adv->setScanResponseData(scanData);

  adv->setMinPreferred(0x0C); // 15ms
  adv->setMaxPreferred(0x18); // 30ms
  adv->start();
}

// ─────────────────────────────────────────────────────────────────────────────
// Mode transitions
// ─────────────────────────────────────────────────────────────────────────────
void enterMode(TwinkleMode m) {
  // Stop tune tone when leaving TUNE mode
  if (mode == TwinkleMode::TUNE) stopTunePlayback();

  // Tear down portal when leaving CUSTOM_DISPLAY or TUNE
  if ((mode == TwinkleMode::CUSTOM_DISPLAY || mode == TwinkleMode::TUNE) &&
      m != TwinkleMode::CUSTOM_DISPLAY && m != TwinkleMode::TUNE && portalActive) {
    teardownPortal();
  }

  mode = m;
  confirmPressedAt = 0;
  confirmHandled   = false;

  switch (m) {
    case TwinkleMode::PLAY:
      if (currentTheme == 2) {
        tft.fillScreen(BMO_MINT_BG);
        bmoPrevDrawn = false;
        buzz(523, 40); delay(20); buzz(659, 40); delay(20); buzz(784, 60);
      } else if (currentTheme == 1) {
        tft.fillScreen(0x0000);
        blinkPrevDrawn = false;
        buzz(659, 60); delay(30); buzz(880, 60);
      } else {
        drawStarfield();
        buzz(523, 60); delay(30); buzz(659, 60);
      }
      break;
    case TwinkleMode::MENU:
      drawMenu();
      buzz(440, 40);
      break;
    case TwinkleMode::THEME_SELECT:
      themeChoice = currentTheme;
      drawThemeSelect();
      buzz(440, 40);
      break;
    case TwinkleMode::AIR_MOUSE:
      WiFi.mode(WIFI_OFF);
      smoothMouseX = 0.0f;
      smoothMouseY = 0.0f;
      accumX = 0.0f;
      accumY = 0.0f;
      lastSentButtons = 0;
      lastMouseReportTime = millis();
      drawAirMouseScreen();
      buzz(660, 40); delay(20); buzz(880, 40);
      break;
    case TwinkleMode::TUNE:
      currentTuneIdx = 0;
      loadCustomTune();
      drawTunePlayerScreen();
      startTunePlayback(currentTuneIdx);
      if (!portalActive) setupPortal();
      break;
    case TwinkleMode::GAMES:
      gamesChoice = 0;
      loadHighScores();
      drawGamesMenu();
      buzz(523, 40);
      break;
    case TwinkleMode::GAME_PLAYING:
      initCurrentGame(currentGameIdx);
      break;
    case TwinkleMode::CUSTOM_DISPLAY:
      drawCustomDisplay();
      break;
    case TwinkleMode::SETTINGS:
      settingsChoice = 0;
      drawSettings();
      buzz(330, 40);
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Button handling (edge-detected, debounced, hold detection)
// ─────────────────────────────────────────────────────────────────────────────
void updateButton(ButtonState &btn, unsigned long now) {
  btn.justPressed  = false;
  btn.justReleased = false;

  bool reading = digitalRead(btn.pin);
  if (reading != btn.lastReading) {
    btn.lastDebounceTime = now;
    btn.lastReading = reading;
  }
  if ((now - btn.lastDebounceTime) > 25) {
    if (reading != btn.state) {
      btn.state = reading;
      if (btn.state == LOW) {
        btn.justPressed = true;
        btn.pressedAt = now;
        btn.longPressHandled = false;
      } else {
        btn.justReleased = true;
      }
    }
  }
}

void handleButtons() {
  unsigned long now = millis();
  updateButton(btnLeft, now);
  updateButton(btnRight, now);
  updateButton(btnConfirm, now);

  // Debug GPIO 0 raw state once per second
  static unsigned long lastDbg = 0;
  if (now - lastDbg > 1000) {
    Serial.printf("GPIO0(L)=%d GPIO1(R)=%d GPIO8(OK)=%d\n",
      digitalRead(PIN_LEFT), digitalRead(PIN_RIGHT), digitalRead(PIN_CONFIRM));
    lastDbg = now;
  }

  // Long-press RIGHT (>=600ms) acts as LEFT for one-handed / GPIO-0-broken fallback
  bool rightLongPressed = false;
  if (btnRight.state == LOW && !btnRight.longPressHandled) {
    if ((now - btnRight.pressedAt) >= 600) {
      btnRight.longPressHandled = true;
      rightLongPressed = true;
      vibrate(15);
    }
  }

  // --- Hold CONFIRM (1.0 s) opens MENU from any mode ---
  if (btnConfirm.state == LOW && !btnConfirm.longPressHandled) {
    if ((now - btnConfirm.pressedAt) >= 1000) {
      btnConfirm.longPressHandled = true;
      if (mode != TwinkleMode::MENU) {
        vibrate(35);
        buzz(523, 40);
        enterMode(TwinkleMode::MENU);
      }
      return;
    }
  }

  // --- Per-mode button logic ---
  switch (mode) {

    case TwinkleMode::MENU:
      if (btnLeft.justPressed || rightLongPressed) {
        menuChoice = (menuChoice + MENU_COUNT - 1) % MENU_COUNT;
        drawMenu();
        vibrate(20);
        buzz(659, 20);
      }
      if (btnRight.justPressed) {
        menuChoice = (menuChoice + 1) % MENU_COUNT;
        drawMenu();
        vibrate(20);
        buzz(784, 20);
      }
      if (btnConfirm.justReleased && !btnConfirm.longPressHandled) {
        buzz(523, 30);
        const TwinkleMode targets[] = {
          TwinkleMode::THEME_SELECT, TwinkleMode::AIR_MOUSE, TwinkleMode::TUNE, TwinkleMode::GAMES, TwinkleMode::SETTINGS
        };
        enterMode(targets[menuChoice]);
      }
      break;

    case TwinkleMode::THEME_SELECT:
      if (btnLeft.justPressed || rightLongPressed) {
        themeChoice = (themeChoice + THEME_COUNT - 1) % THEME_COUNT;
        drawThemeSelect();
        vibrate(20);
        buzz(659, 20);
      }
      if (btnRight.justPressed) {
        themeChoice = (themeChoice + 1) % THEME_COUNT;
        drawThemeSelect();
        vibrate(20);
        buzz(784, 20);
      }
      if (btnConfirm.justReleased && !btnConfirm.longPressHandled) {
        buzz(523, 30);
        if (themeChoice == 0) {
          currentTheme = 0;
          prefs.putUChar("theme", currentTheme);
          enterMode(TwinkleMode::PLAY);
        } else if (themeChoice == 1) {
          currentTheme = 1;
          prefs.putUChar("theme", currentTheme);
          enterMode(TwinkleMode::PLAY);
        } else if (themeChoice == 2) {
          currentTheme = 2;
          prefs.putUChar("theme", currentTheme);
          enterMode(TwinkleMode::PLAY);
        } else if (themeChoice == 3) {
          enterMode(TwinkleMode::CUSTOM_DISPLAY);
        } else {
          enterMode(TwinkleMode::MENU);
        }
      }
      break;

    case TwinkleMode::PLAY: {
      if (currentTheme == 2) {
        // BMO Theme: Left / Right button triggers cute BMO reaction emote
        if (btnLeft.justPressed || rightLongPressed) {
          triggerBmoEmote(BmoEmote::HAPPY, 1400);
          buzz(587, 30); delay(20); buzz(880, 40); delay(20); buzz(1175, 70);
          vibrate(30);
        }
        if (btnRight.justPressed) {
          triggerBmoEmote(BmoEmote::TALK, 1200);
          buzz(523, 25); delay(35); buzz(659, 25); delay(35); buzz(784, 35);
          vibrate(25);
        }
      } else if (currentTheme == 1) {
        // Blink Eyes: Left/Right cycles eye color palette
        if (btnLeft.justPressed || rightLongPressed) {
          blinkColorIndex = (blinkColorIndex + BLINK_PALETTE_COUNT - 1) % BLINK_PALETTE_COUNT;
          blinkEyeColor = BLINK_PALETTE[blinkColorIndex];
          buzz(523, 20);
          vibrate(20);
        }
        if (btnRight.justPressed) {
          blinkColorIndex = (blinkColorIndex + 1) % BLINK_PALETTE_COUNT;
          blinkEyeColor = BLINK_PALETTE[blinkColorIndex];
          buzz(659, 20);
          vibrate(20);
        }
      } else {
        // Star Theme: Left / Right cycles the star colour
        static const uint16_t cols[] = {
          PINK, MINT, STAR, GLOW, 0xFFFF, 0xF800, 0x07E0, 0x001F, 0xFD20, 0x780F
        };
        constexpr int NCOL = sizeof(cols) / sizeof(cols[0]);
        static uint8_t ci = 0;

        if (btnLeft.justPressed || rightLongPressed) {
          ci = (ci + NCOL - 1) % NCOL;
          twinkleColor = cols[ci];
          buzz(523, 20);
          vibrate(20);
          for (int i = 0; i < 6; i++) spawnSparkle(x, y);
        }
        if (btnRight.justPressed) {
          ci = (ci + 1) % NCOL;
          twinkleColor = cols[ci];
          buzz(659, 20);
          vibrate(20);
          for (int i = 0; i < 6; i++) spawnSparkle(x, y);
        }
      }
      // Tap Confirm in Play mode opens Menu
      if (btnConfirm.justReleased && !btnConfirm.longPressHandled) {
        buzz(523, 30);
        enterMode(TwinkleMode::MENU);
      }
      break;
    }

    case TwinkleMode::SETTINGS:
      // LEFT pressed (or long-press RIGHT as fallback)
      if (btnLeft.justPressed || rightLongPressed) {
        if (settingsChoice == 0) {
          if (screenBrightness > 0) {
            screenBrightness--;
            applyBrightness(screenBrightness);
            prefs.putUChar("bright", screenBrightness);
            vibrate(15);
            buzz(440, 15);
          }
          drawSettings();
        } else if (settingsChoice == 1) {
          if (soundVolume > 0) {
            soundVolume--;
            prefs.putUChar("volume", soundVolume);
            vibrate(15);
          }
          if (soundVolume > 0) buzz(440, 20);
          drawSettings();
        } else if (settingsChoice == 2) {
          if (mouseSensitivity > 1) {
            mouseSensitivity--;
            prefs.putUChar("msens", mouseSensitivity);
            vibrate(15);
            buzz(440, 15);
          }
          drawSettings();
        } else {
          settingsChoice = (settingsChoice + SETTINGS_COUNT - 1) % SETTINGS_COUNT;
          drawSettings();
          vibrate(20);
        }
      }
      // RIGHT short-press
      if (btnRight.justPressed) {
        if (settingsChoice == 0) {
          if (screenBrightness < 10) {
            screenBrightness++;
            applyBrightness(screenBrightness);
            prefs.putUChar("bright", screenBrightness);
            vibrate(15);
            buzz(660, 15);
          }
          drawSettings();
        } else if (settingsChoice == 1) {
          if (soundVolume < 10) {
            soundVolume++;
            prefs.putUChar("volume", soundVolume);
            vibrate(15);
          }
          if (soundVolume > 0) buzz(660, 20);
          drawSettings();
        } else if (settingsChoice == 2) {
          if (mouseSensitivity < 10) {
            mouseSensitivity++;
            prefs.putUChar("msens", mouseSensitivity);
            vibrate(15);
            buzz(660, 15);
          }
          drawSettings();
        } else {
          settingsChoice = (settingsChoice + 1) % SETTINGS_COUNT;
          drawSettings();
          vibrate(20);
        }
      }
      // CONFIRM pressed
      if (btnConfirm.justReleased && !btnConfirm.longPressHandled) {
        if (settingsChoice >= 0 && settingsChoice <= 2) {
          settingsChoice = (settingsChoice + 1) % SETTINGS_COUNT;
          drawSettings();
          vibrate(20);
        } else if (settingsChoice == 3) {
          closePersistentImage();
          LittleFS.remove("/image.bin");
          reloadCustomDisplay();
          buzz(220, 120);
          drawSettings();
        } else {
          enterMode(TwinkleMode::MENU);
        }
      }
      break;

    case TwinkleMode::TUNE:
      if (btnLeft.justPressed || rightLongPressed) {
        currentTuneIdx = (currentTuneIdx + TUNE_PLAYLIST_COUNT - 1) % TUNE_PLAYLIST_COUNT;
        drawTunePlayerScreen();
        startTunePlayback(currentTuneIdx);
        vibrate(20);
      }
      if (btnRight.justPressed) {
        currentTuneIdx = (currentTuneIdx + 1) % TUNE_PLAYLIST_COUNT;
        drawTunePlayerScreen();
        startTunePlayback(currentTuneIdx);
        vibrate(20);
      }
      if (btnConfirm.justReleased && !btnConfirm.longPressHandled) {
        toggleTunePlayback();
        drawTunePlayerScreen();
        vibrate(20);
      }
      break;

    case TwinkleMode::GAMES:
      if (btnLeft.justPressed || rightLongPressed) {
        gamesChoice = (gamesChoice + GAMES_COUNT - 1) % GAMES_COUNT;
        drawGamesMenu();
        vibrate(20);
        buzz(659, 20);
      }
      if (btnRight.justPressed) {
        gamesChoice = (gamesChoice + 1) % GAMES_COUNT;
        drawGamesMenu();
        vibrate(20);
        buzz(784, 20);
      }
      if (btnConfirm.justReleased && !btnConfirm.longPressHandled) {
        buzz(523, 30);
        if (gamesChoice < 4) {
          currentGameIdx = gamesChoice;
          enterMode(TwinkleMode::GAME_PLAYING);
        } else {
          enterMode(TwinkleMode::MENU);
        }
      }
      break;

    case TwinkleMode::GAME_PLAYING:
      if (gameState == GameState::GAMEOVER || gameState == GameState::VICTORY) {
        if (btnConfirm.justReleased && !btnConfirm.longPressHandled) {
          initCurrentGame(currentGameIdx);
        } else if (btnLeft.justPressed || btnRight.justPressed || rightLongPressed) {
          enterMode(TwinkleMode::GAMES);
        }
      } else {
        handleGameInput(btnLeft, btnRight, btnConfirm, rightLongPressed);
      }
      break;

    case TwinkleMode::CUSTOM_DISPLAY:
      if (btnLeft.justPressed || btnRight.justPressed) {
        drawCustomDisplay();
        vibrate(20);
      }
      if (btnConfirm.justReleased && !btnConfirm.longPressHandled) {
        enterMode(TwinkleMode::MENU);
      }
      break;

    default: break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Buttons (active-low with internal pull-up)
  gpio_reset_pin(GPIO_NUM_0);
  pinMode(PIN_LEFT,    INPUT_PULLUP);
  pinMode(PIN_RIGHT,   INPUT_PULLUP);
  pinMode(PIN_CONFIRM, INPUT_PULLUP);

  // Motor driver
  pinMode(PIN_MOTOR, OUTPUT);
  digitalWrite(PIN_MOTOR, LOW);

  // Buzzer via LEDC (arduino-esp32 v3.x API)
  ledcAttach(PIN_BUZZER, 2000, 10);

  // TFT: configure custom SPI pins before calling tft.init()
  SPI.begin(PIN_TFT_SCLK, /*MISO*/ -1, PIN_TFT_MOSI, /*SS*/ -1);
  tft.init(135, 240);
  tft.setRotation(3);  // Landscape (240x135, flipped)

  // Load saved settings from Preferences & apply
  prefs.begin("twinkle", false);
  screenBrightness = prefs.getUChar("bright", 8);
  soundVolume      = prefs.getUChar("volume", 8);
  mouseSensitivity = prefs.getUChar("msens", 5);
  currentTheme     = prefs.getUChar("theme", 0);
  if (screenBrightness > 10) screenBrightness = 8;
  if (soundVolume > 10) soundVolume = 8;
  if (mouseSensitivity < 1 || mouseSensitivity > 10) mouseSensitivity = 5;
  if (currentTheme > 2) currentTheme = 0;

  if (PIN_TFT_BL >= 0) {
    pinMode(PIN_TFT_BL, OUTPUT);
  }
  applyBrightness(screenBrightness);

  // --- Dynamic Boot Splash & IMU Detection per Theme ---
  if (currentTheme == 2) {
    // BMO (Beemo) Boot Splash
    tft.fillScreen(BMO_MINT_BG);

    // Draw cute BMO eyes & smile
    tft.fillCircle(64, 45, 6, BMO_BLACK);
    tft.fillCircle(176, 45, 6, BMO_BLACK);
    tft.fillRoundRect(88, 70, 64, 18, 9, BMO_BLACK);
    tft.fillRoundRect(90, 72, 60, 14, 7, BMO_MOUTH_DARK);
    tft.fillRect(94, 72, 52, 4, BMO_TEETH_WHITE);
    tft.fillCircle(120, 84, 8, BMO_TONGUE);

    tft.setTextColor(BMO_BLACK);  tft.setTextSize(2);
    tft.setCursor(76, 16);   tft.print("BMO BOT");
    tft.setTextColor(0x0A2B);  tft.setTextSize(1);
    tft.setCursor(68, 96);   tft.print("Beemo is booting...");

    if (scanAllPinsForMPU()) {
      mpuFound = true;
      Serial.printf("MPU initialized at 0x%02X (SDA:%d, SCL:%d)\n", mpuAddress, currentSDA, currentSCL);
      tft.fillRoundRect(20, 108, SCREEN_W-40, 20, 4, 0xFFFF);
      tft.drawRoundRect(20, 108, SCREEN_W-40, 20, 4, BMO_BLACK);
      tft.setTextColor(BMO_BLACK);
      tft.setCursor(28, 114);
      tft.printf("IMU: 0x%02X (%d,%d) ONLINE", mpuAddress, currentSDA, currentSCL);
      delay(900);
    } else {
      Serial.println("MPU6050 not detected on any tested pins.");
      tft.fillRoundRect(20, 108, SCREEN_W-40, 20, 4, 0xFFFF);
      tft.drawRoundRect(20, 108, SCREEN_W-40, 20, 4, 0xF800);
      tft.setTextColor(0xF800);
      tft.setCursor(34, 114);
      tft.print("IMU: DEMO FLOAT MODE");
      delay(1200);
    }

    // LittleFS (auto-format on first boot)
    LittleFS.begin(/*formatOnFail*/ true);
    setupBle();
    randomSeed(esp_random());

    // BMO classic upbeat 8-bit game arpeggio
    buzz(392, 50); delay(20);
    buzz(523, 50); delay(20);
    buzz(659, 50); delay(20);
    buzz(784, 70); delay(20);
    buzz(1046, 120);

  } else if (currentTheme == 1) {
    // Cyber Robot Eye (Blink) Boot Splash
    tft.fillScreen(0x0000);
    // Outer cyber corner brackets
    tft.drawFastHLine(8, 8, 24, BLINK_CYAN);
    tft.drawFastVLine(8, 8, 24, BLINK_CYAN);
    tft.drawFastHLine(SCREEN_W - 32, 8, 24, BLINK_CYAN);
    tft.drawFastVLine(SCREEN_W - 9, 8, 24, BLINK_CYAN);
    tft.drawFastHLine(8, SCREEN_H - 9, 24, BLINK_CYAN);
    tft.drawFastVLine(8, SCREEN_H - 32, 24, BLINK_CYAN);
    tft.drawFastHLine(SCREEN_W - 32, SCREEN_H - 9, 24, BLINK_CYAN);
    tft.drawFastVLine(SCREEN_W - 9, SCREEN_H - 32, 24, BLINK_CYAN);

    tft.setTextColor(BLINK_CYAN);  tft.setTextSize(2);
    tft.setCursor(46, 20);   tft.print("[ BLINK BOT ]");
    tft.setTextColor(BLINK_GLOW);  tft.setTextSize(1);
    tft.setCursor(48, 48);   tft.print("> SYSTEM INITIALIZING...");

    tft.setTextColor(0x57BF); tft.setTextSize(1);
    tft.setCursor(56, 75);  tft.print("> SCANNING IMU BUS...");

    if (scanAllPinsForMPU()) {
      mpuFound = true;
      Serial.printf("MPU initialized at 0x%02X (SDA:%d, SCL:%d)\n", mpuAddress, currentSDA, currentSCL);
      tft.fillRoundRect(20, 95, SCREEN_W-40, 24, 4, BLINK_DARK_TEAL);
      tft.drawRoundRect(20, 95, SCREEN_W-40, 24, 4, BLINK_CYAN);
      tft.setTextColor(BLINK_CYAN);
      tft.setCursor(30, 103);
      tft.printf("IMU: 0x%02X (SDA:%d, SCL:%d) OK", mpuAddress, currentSDA, currentSCL);
      delay(900);
    } else {
      Serial.println("MPU6050 not detected on any tested pins.");
      tft.fillRoundRect(20, 95, SCREEN_W-40, 24, 4, BLINK_DARK_TEAL);
      tft.drawRoundRect(20, 95, SCREEN_W-40, 24, 4, 0xF800);
      tft.setTextColor(0xF800);
      tft.setCursor(38, 103);
      tft.print("IMU: DEMO MODE (FALLBACK)");
      delay(1200);
    }

    // LittleFS (auto-format on first boot)
    LittleFS.begin(/*formatOnFail*/ true);
    setupBle();
    randomSeed(esp_random());

    // Futuristic cyber boot chimes
    buzz(440, 40); delay(20);
    buzz(659, 40); delay(20);
    buzz(880, 60); delay(25);
    buzz(1175, 100);

  } else {
    // Twinkle Star Boot Splash
    tft.fillScreen(SPACE);
    tft.drawPixel(8, 8, STAR_ROSE);
    tft.drawPixel(SCREEN_W - 12, 8, STAR_PINK);
    tft.drawPixel(6, 126, STAR_BERRY);
    tft.drawPixel(SCREEN_W - 10, 126, STAR_GOLD);

    tft.setTextColor(STAR_GOLD);  tft.setTextSize(2);
    tft.setCursor(68, 20);   tft.print("* Twinkle *");
    tft.setTextColor(STAR_PINK);  tft.setTextSize(1);
    tft.setCursor(86, 48);   tft.print("loading system...");

    tft.setTextColor(MINT);  tft.setTextSize(1);
    tft.setCursor(68, 75);  tft.print("detecting IMU sensor...");

    if (scanAllPinsForMPU()) {
      mpuFound = true;
      Serial.printf("MPU initialized at 0x%02X (SDA:%d, SCL:%d)\n", mpuAddress, currentSDA, currentSCL);
      tft.fillRoundRect(20, 95, SCREEN_W-40, 24, 4, STAR_BG_CARD);
      tft.drawRoundRect(20, 95, SCREEN_W-40, 24, 4, STAR_PINK);
      tft.setTextColor(STAR_GOLD);
      tft.setCursor(28, 103);
      tft.printf("IMU: 0x%02X (%d,%d) ONLINE", mpuAddress, currentSDA, currentSCL);
      delay(900);
    } else {
      Serial.println("MPU6050 not detected on any tested pins.");
      tft.fillRoundRect(20, 95, SCREEN_W-40, 24, 4, STAR_BG_CARD);
      tft.drawRoundRect(20, 95, SCREEN_W-40, 24, 4, STAR_BERRY);
      tft.setTextColor(STAR_PINK);
      tft.setCursor(34, 103);
      tft.print("IMU: DEMO FLOAT MODE");
      delay(1200);
    }

    // LittleFS (auto-format on first boot)
    LittleFS.begin(/*formatOnFail*/ true);
    setupBle();
    randomSeed(esp_random());

    for (int i = 0; i < SPARKLE_COUNT; i++)
      sparkles[i] = { (float)SCREEN_W/2, (float)SCREEN_H/2, 0, 0, PINK, 0UL };

    // Twinkle joyful boot jingle
    buzz(262, 60); delay(20);
    buzz(330, 60); delay(20);
    buzz(392, 60); delay(20);
    buzz(523, 110);
  }

  enterMode(TwinkleMode::PLAY);
  lastFrame = millis();
  Serial.println("Twinkle ready!");
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  float dt = constrain((now - lastFrame) / 1000.0f, 0.001f, 0.1f);
  lastFrame = now;

  // Auto-stop motor
  if (now >= motorUntil) digitalWrite(PIN_MOTOR, LOW);

  // Read IMU if available, or simulate organic motion
  sensors_event_t accel = {}, gyro = {}, temp = {};
  if (mpuFound && readMPU(&accel, &gyro)) {
    // Successfully read real hardware sensor
  } else {
    // Gentle demo floating motion
    float t = now * 0.002f;
    accel.acceleration.x = sinf(t * 0.8f) * 2.8f;
    accel.acceleration.y = cosf(t * 1.1f) * 2.8f;
    accel.acceleration.z = 9.8f;
    gyro.gyro.x = cosf(t) * 0.5f;
    gyro.gyro.y = sinf(t) * 0.5f;
    gyro.gyro.z = 0;
  }

  handleButtons();

  // Service captive portal if active
  if (portalActive) {
    dns.processNextRequest();
    web.handleClient();
  }

  // Per-mode update / draw
  switch (mode) {
    case TwinkleMode::PLAY:
      if (currentTheme == 2) {
        updateBMO(dt, accel, gyro);
        drawBMOFrame();
      } else if (currentTheme == 1) {
        updateBlinkEyes(dt, accel, gyro);
        drawBlinkEyesFrame();
      } else {
        updatePlay(dt, accel, gyro);
        drawPlayFrame();
      }
      break;

    case TwinkleMode::AIR_MOUSE:
      updateAirMouse(accel, gyro);
      break;

    case TwinkleMode::TUNE:
      updateTunePlayer();
      break;

    case TwinkleMode::GAME_PLAYING:
      updateCurrentGame(dt, accel, gyro);
      break;

    case TwinkleMode::CUSTOM_DISPLAY:
      updateCustomDisplay();
      break;

    default:
      break;  // MENU and SETTINGS are static; redrawn only on button press
  }
}
