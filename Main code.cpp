#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>

// Integrierter ESP32 BLE-Stack
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>

// --- DUAL-WLAN KONFIGURATION ---
struct WifiCred {
  const char* ssid;
  const char* pass;
};

const WifiCred KNOWN_WIFIS[] = {
  { "YOUR_WIFI_SSID_1", "YOUR_WIFI_PASSWORD_1" },
  { "YOUR_WIFI_SSID_2", "YOUR_WIFI_PASSWORD_2" }
};
const int NUM_WIFIS = 2;

// Offizielle deutsche Zeitzone (MEZ / MESZ automatisch)
const char* TZ_GERMANY = "CET-1CEST,M3.5.0,M10.5.0/3";

// Optionales Sekunden-Offset (0 = exakte Atomuhrzeit)
#define CLOCK_OFFSET_SEC   0

// --- PIN-BELEGUNG ---
#define LED_PIN         2
#define ENC_CLK         3
#define ENC_DT          4
#define ENC_SW          5

#define TFT_SCLK        6   // SCL
#define TFT_MOSI        7   // SDA
#define TFT_BL          8   // Backlight PWM
#define TFT_CS         10
#define TFT_DC          1
#define TFT_RES         0

// --- DISPLAY-KONFIGURATION (2,01" JD9853 240x296) ---
#define PANEL_WIDTH   240
#define PANEL_HEIGHT  296
#define JD9853_MADCTL 0xE0
#define TFT_OFFSET_X   24
#define TFT_OFFSET_Y    0

// --- LED-RING & STANDBY-KONFIGURATION ---
#define NUM_LEDS             24
#define LED_START_OFFSET     11      // Startpunkt um 11 LEDs im Uhrzeigersinn verschoben
#define STANDBY_TIMEOUT_MS   20000UL // 20 Sekunden bis Uhrzeit-Standby

// --- HID CODES ---
#define MEDIA_BIT_VOL_UP      (1 << 0)
#define MEDIA_BIT_VOL_DOWN    (1 << 1)
#define MEDIA_BIT_MUTE        (1 << 2)
#define MEDIA_BIT_PLAY_PAUSE  (1 << 3)
#define MEDIA_BIT_NEXT        (1 << 4)
#define MEDIA_BIT_PREV        (1 << 5)
#define MEDIA_BIT_BRI_UP      (1 << 6)
#define MEDIA_BIT_BRI_DOWN    (1 << 7)

#define KEY_MOD_LCTRL         0x01
#define KEY_MOD_LGUI          0x08
#define KEY_L                 0x0F
#define KEY_RIGHT_ARROW       0x4F
#define KEY_LEFT_ARROW        0x50
#define KEY_DOWN_ARROW        0x51

static const uint8_t WINDOWS_HID_REPORT_MAP[] = {
  // Collection 1: Consumer Control (Report ID 1 -> 1 Byte)
  0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x01,
  0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08,
  0x09, 0xE9, 0x09, 0xEA, 0x09, 0xE2, 0x09, 0xCD,
  0x09, 0xB5, 0x09, 0xB6, 0x09, 0x6F, 0x09, 0x70,
  0x81, 0x02, 0xC0,

  // Collection 2: Standard Keyboard (Report ID 2 -> 8 Bytes)
  0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x02,
  0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00,
  0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
  0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x06,
  0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07,
  0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0,

  // Collection 3: Standard Mouse mit Scrollrad (Report ID 3 -> 4 Bytes)
  0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x03,
  0x09, 0x01, 0xA1, 0x00, 0x05, 0x09, 0x19, 0x01,
  0x29, 0x03, 0x15, 0x00, 0x25, 0x01, 0x95, 0x03,
  0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05,
  0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
  0x09, 0x38, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08,
  0x95, 0x03, 0x81, 0x06, 0xC0, 0xC0
};

// --- DISPLAY-KLASSE ---
class JD9853_TFT : public Adafruit_ST7789 {
public:
  JD9853_TFT(SPIClass *spiClass, int8_t cs, int8_t dc, int8_t rst)
    : Adafruit_ST7789(spiClass, cs, dc, rst) {}

  void beginLandscape() {
    init(PANEL_WIDTH, PANEL_HEIGHT);
    _rowstart  = TFT_OFFSET_X;
    _rowstart2 = TFT_OFFSET_X;
    _colstart  = TFT_OFFSET_Y;
    _colstart2 = TFT_OFFSET_Y;
    _xstart    = TFT_OFFSET_X;
    _ystart    = TFT_OFFSET_Y;
    _width     = PANEL_HEIGHT; // 296 px
    _height    = PANEL_WIDTH;  // 240 px
    uint8_t madctl = JD9853_MADCTL;
    sendCommand(0x36, &madctl, 1);
    setSPISpeed(40000000);
  }
};

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
JD9853_TFT tft(&SPI, TFT_CS, TFT_DC, TFT_RES);
Preferences prefs;

// --- DAUERHAFT GESPEICHERTE EINSTELLUNGEN ---
uint8_t cfgTheme     = 0;
uint8_t cfgTftBri    = 10;
uint8_t cfgLedBri    = 6;
uint8_t cfgLedCol    = 0;
int     volPercent   = 50;
int     ambientBri   = 85;
uint8_t ambientMode  = 0;
const int TOTAL_AMBIENT_MODES = 4;

// --- UHRZEIT-SYSTEM ---
unsigned long clockBaseMillis = 0;
long clockBaseSeconds = 0;
bool timeSyncedViaWifi = false;

void setInternalClock(int hours, int minutes, int seconds = 0) {
  hours   = (hours % 24 + 24) % 24;
  minutes = (minutes % 60 + 60) % 60;
  seconds = (seconds % 60 + 60) % 60;
  clockBaseSeconds = (long)hours * 3600L + (long)minutes * 60L + seconds;
  clockBaseMillis  = millis();
}

void getCurrentTime(int &hours, int &minutes, int &seconds) {
  if (timeSyncedViaWifi) {
    time_t now = time(nullptr) + CLOCK_OFFSET_SEC;
    struct tm ti;
    localtime_r(&now, &ti);
    hours   = ti.tm_hour;
    minutes = ti.tm_min;
    seconds = ti.tm_sec;
    return;
  }

  unsigned long elapsedSec = (millis() - clockBaseMillis) / 1000UL;
  long totalSec = (clockBaseSeconds + elapsedSec + CLOCK_OFFSET_SEC + 86400L) % 86400L;
  hours   = totalSec / 3600L;
  minutes = (totalSec % 3600L) / 60L;
  seconds = totalSec % 60L;
}

void parseCompileTime(int &h, int &m, int &s) {
  const char* t = __TIME__;
  h = (t[0] - '0') * 10 + (t[1] - '0');
  m = (t[3] - '0') * 10 + (t[4] - '0');
  s = (t[6] - '0') * 10 + (t[7] - '0');
  long total = (long)h * 3600L + (long)m * 60L + s + 8;
  total %= 86400L;
  h = total / 3600L;
  m = (total % 3600L) / 60L;
  s = total % 60L;
}

void printTwoDigits(int val) {
  if (val < 10) tft.print('0');
  tft.print(val);
}

// --- FARBSCHEMA ---
struct ThemePalette {
  uint16_t bg;
  uint16_t surface;
  uint16_t border;
  uint16_t fg;
  uint16_t muted;
  uint16_t accent;
};

ThemePalette theme;

void applyTheme() {
  if (cfgTheme == 0) {
    theme = { 0x0000, 0x1082, 0x2945, 0xFFFF, 0x6B4D, 0x073F };
  } else if (cfgTheme == 1) {
    theme = { 0x0000, 0x18E3, 0x528A, 0xFFFF, 0x7BEF, 0xFFFF };
  } else {
    theme = { 0xFFFF, 0xE71C, 0x9492, 0x0000, 0x7BEF, 0x0000 };
  }
}

void applyHardwareBrightness() {
  int pwm = map(cfgTftBri, 1, 10, 20, 255);
  analogWrite(TFT_BL, pwm);

  int neoBri = map(cfgLedBri, 1, 10, 8, 110);
  strip.setBrightness(neoBri);
}

void loadSettings() {
  prefs.begin("dial-cfg", true);
  cfgTheme    = constrain(prefs.getUChar("theme", 0), 0, 2);
  cfgTftBri   = constrain(prefs.getUChar("tftBri", 10), 1, 10);
  cfgLedBri   = constrain(prefs.getUChar("ledBri", 6), 1, 10);
  cfgLedCol   = constrain(prefs.getUChar("ledCol", 0), 0, 7);
  volPercent  = constrain(prefs.getUChar("volPct", 50), 0, 100);
  volPercent  = ((volPercent + 2) / 5) * 5;
  ambientMode = constrain(prefs.getUChar("ambMod", 0), 0, TOTAL_AMBIENT_MODES - 1);
  ambientBri  = constrain(prefs.getUChar("ambBri", 85), 15, 140);

  int compH, compM, compS;
  parseCompileTime(compH, compM, compS);
  int h = prefs.getUChar("clkH", compH);
  int m = prefs.getUChar("clkM", compM);
  int s = prefs.getUChar("clkS", compS);
  prefs.end();

  setInternalClock(h, m, s);
  applyTheme();
  applyHardwareBrightness();
}

void saveSettings() {
  int h, m, s;
  getCurrentTime(h, m, s);

  prefs.begin("dial-cfg", false);
  prefs.putUChar("theme", cfgTheme);
  prefs.putUChar("tftBri", cfgTftBri);
  prefs.putUChar("ledBri", cfgLedBri);
  prefs.putUChar("ledCol", cfgLedCol);
  prefs.putUChar("volPct", (uint8_t)volPercent);
  prefs.putUChar("ambMod", ambientMode);
  prefs.putUChar("ambBri", (uint8_t)ambientBri);
  prefs.putUChar("clkH", (uint8_t)h);
  prefs.putUChar("clkM", (uint8_t)m);
  prefs.putUChar("clkS", (uint8_t)s);
  prefs.end();
}

void setRingPixel(int logicalIndex, uint32_t color) {
  int physicalIndex = (logicalIndex + LED_START_OFFSET + NUM_LEDS) % NUM_LEDS;
  strip.setPixelColor(physicalIndex, color);
}

uint32_t getRingColor(int logicalIndex, uint8_t intensity = 255) {
  uint8_t r = 255, g = 255, b = 255;
  switch (cfgLedCol) {
    case 0: r = 255; g = 255; b = 255; break;
    case 1: r = 0;   g = 240; b = 255; break;
    case 2: r = 0;   g = 80;  b = 255; break;
    case 3: r = 180; g = 0;   b = 255; break;
    case 4: r = 255; g = 15;  b = 0;   break;
    case 5: r = 255; g = 140; b = 0;   break;
    case 6: r = 0;   g = 255; b = 60;  break;
    case 7: {
      uint16_t hue = (logicalIndex * 65536L) / NUM_LEDS;
      return strip.gamma32(strip.ColorHSV(hue, 255, intensity));
    }
  }
  r = (r * intensity) >> 8;
  g = (g * intensity) >> 8;
  b = (b * intensity) >> 8;
  return strip.Color(r, g, b);
}

// ============================================================================
// ABSTURZSICHERER BLE-STACK (3 ISOLIERTE SERVICES GEGEN SCROLL-CRASH!)
// ============================================================================
BLEServer* pServer = nullptr;
BLEHIDDevice* hid = nullptr;
BLECharacteristic* inputConsumer = nullptr;
BLECharacteristic* inputKeyboard = nullptr;
BLECharacteristic* inputMouse    = nullptr;

bool bleInitialized = false;
volatile bool shouldRestartAdv = false;
unsigned long disconnectTime = 0;
bool bleConnected = false;
bool lastBleState = false;

class BleCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    shouldRestartAdv = false;
  }

  void onDisconnect(BLEServer* s) override {
    if (s->getConnectedCount() == 0) {
      shouldRestartAdv = true;
      disconnectTime = millis();
    }
  }
};

// Hilfsfunktion: Erstellt einen eigenen isolierten HID-Service für eine Report-ID
// (Verhindert den ESP32-Bug, bei dem zwei 0x2A4D-Charakteristiken im selben Service abstürzen!)
BLECharacteristic* createIsolatedHidReport(uint8_t instanceId, uint8_t reportId) {
  BLEService* svc = pServer->createService(BLEUUID((uint16_t)0x1812), 20, instanceId);

  BLECharacteristic* info = svc->createCharacteristic((uint16_t)0x2A4A, BLECharacteristic::PROPERTY_READ);
  const uint8_t infoVal[] = { 0x11, 0x01, 0x00, 0x02 };
  info->setValue((uint8_t*)infoVal, 4);

  svc->createCharacteristic((uint16_t)0x2A4C, BLECharacteristic::PROPERTY_WRITE_NR);

  BLECharacteristic* proto = svc->createCharacteristic((uint16_t)0x2A4E, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR);
  uint8_t protoVal = 1;
  proto->setValue(&protoVal, 1);

  BLECharacteristic* rMap = svc->createCharacteristic((uint16_t)0x2A4B, BLECharacteristic::PROPERTY_READ);
  rMap->setValue((uint8_t*)WINDOWS_HID_REPORT_MAP, sizeof(WINDOWS_HID_REPORT_MAP));

  BLECharacteristic* repChar = svc->createCharacteristic((uint16_t)0x2A4D, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  repChar->addDescriptor(new BLE2902());

  BLEDescriptor* refDesc = new BLEDescriptor(BLEUUID((uint16_t)0x2908));
  uint8_t refVal[2] = { reportId, 0x01 }; // Input Report
  refDesc->setValue(refVal, 2);
  repChar->addDescriptor(refDesc);

  svc->start();
  return repChar;
}

void initBluetooth() {
  if (bleInitialized) return;
  bleInitialized = true;

  BLEDevice::init("DialDisp");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BleCallbacks());

  // 1. Service-Instanz 0: Consumer Control (Report ID 1 -> Audio/Media/Helligkeit)
  hid = new BLEHIDDevice(pServer);
  inputConsumer = hid->inputReport(1);
  hid->manufacturer()->setValue("Maker");
  hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->hidInfo(0x00, 0x01);
  hid->reportMap((uint8_t*)WINDOWS_HID_REPORT_MAP, sizeof(WINDOWS_HID_REPORT_MAP));

  // 2. Service-Instanz 1: Keyboard (Report ID 2 -> Workspace, Lock, Pfeil runter)
  inputKeyboard = createIsolatedHidReport(1, 0x02);

  // 3. Service-Instanz 2: Mouse Wheel (Report ID 3 -> Mausrad-Scrollen ohne Absturz!)
  inputMouse = createIsolatedHidReport(2, 0x03);

  BLESecurity* pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);
  pSecurity->setCapability(ESP_IO_CAP_NONE);
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  hid->startServices();
  hid->setBatteryLevel(100);

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setAppearance(HID_KEYBOARD);
  pAdvertising->addServiceUUID(hid->hidService()->getUUID());
  pAdvertising->setScanResponse(false);
  pAdvertising->start();
}

bool isBleActive() {
  return (bleInitialized && pServer != nullptr && pServer->getConnectedCount() > 0);
}

// ============================================================================
// RUCKELFREIER HINTERGRUND-WLAN-SYNC (NACH DER BOOT-ANIMATION)
// ============================================================================
enum BgWifiState {
  BG_WIFI_START = 0,
  BG_WIFI_WAIT_CONN,
  BG_WIFI_WAIT_NTP,
  BG_WIFI_DONE
};

BgWifiState bgWifiState = BG_WIFI_START;
int bgApIdx = 0;
unsigned long bgTimer = 0;

void finishWifiAndStartBle() {
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  bgWifiState = BG_WIFI_DONE;
  initBluetooth();
}

void pollBackgroundWifi() {
  if (bgWifiState == BG_WIFI_DONE) return;

  if (bgWifiState == BG_WIFI_START) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_11dBm);
    configTzTime(TZ_GERMANY, "ptbtime1.ptb.de", "de.pool.ntp.org", "time.google.com");
    WiFi.begin(KNOWN_WIFIS[bgApIdx].ssid, KNOWN_WIFIS[bgApIdx].pass);
    bgTimer = millis();
    bgWifiState = BG_WIFI_WAIT_CONN;
  } else if (bgWifiState == BG_WIFI_WAIT_CONN) {
    if (WiFi.status() == WL_CONNECTED) {
      configTzTime(TZ_GERMANY, "ptbtime1.ptb.de", "de.pool.ntp.org", "time.google.com");
      bgTimer = millis();
      bgWifiState = BG_WIFI_WAIT_NTP;
    } else if (millis() - bgTimer > 5000UL) {
      bgApIdx++;
      if (bgApIdx < NUM_WIFIS) {
        WiFi.disconnect(true, false);
        WiFi.begin(KNOWN_WIFIS[bgApIdx].ssid, KNOWN_WIFIS[bgApIdx].pass);
        bgTimer = millis();
      } else {
        finishWifiAndStartBle();
      }
    }
  } else if (bgWifiState == BG_WIFI_WAIT_NTP) {
    time_t now = time(nullptr);
    if (now > 1700000000L) {
      struct tm ti;
      localtime_r(&now, &ti);
      if (ti.tm_year >= (2025 - 1900)) {
        setInternalClock(ti.tm_hour, ti.tm_min, ti.tm_sec);
        timeSyncedViaWifi = true;
        saveSettings();
        finishWifiAndStartBle();
      }
    } else if (millis() - bgTimer > 5500UL) {
      finishWifiAndStartBle();
    }
  }
}

void sendMediaBit(uint8_t bitMask, int repeats = 1) {
  if (!isBleActive() || inputConsumer == nullptr) return;
  uint8_t release = 0;

  for (int i = 0; i < repeats; i++) {
    inputConsumer->setValue(&bitMask, 1);
    inputConsumer->notify();
    delay(8);

    inputConsumer->setValue(&release, 1);
    inputConsumer->notify();
    if (i + 1 < repeats) delay(8);
  }
}

void sendKeyboardShortcut(uint8_t modifiers, uint8_t keycode) {
  if (!isBleActive() || inputKeyboard == nullptr) return;
  uint8_t report[8] = { modifiers, 0, keycode, 0, 0, 0, 0, 0 };
  inputKeyboard->setValue(report, 8);
  inputKeyboard->notify();
  delay(16);

  uint8_t release[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  inputKeyboard->setValue(release, 8);
  inputKeyboard->notify();
}

void sendMouseScroll(int8_t wheelDelta) {
  if (!isBleActive() || inputMouse == nullptr) return;
  uint8_t report[4] = { 0, 0, 0, (uint8_t)wheelDelta };
  inputMouse->setValue(report, 4);
  inputMouse->notify();
}

void syncPcAudioTo50() {
  if (!isBleActive()) return;
  sendMediaBit(MEDIA_BIT_VOL_DOWN, 50);
  delay(20);
  sendMediaBit(MEDIA_BIT_VOL_UP, 25);
  volPercent = 50;
  saveSettings();
}

// --- APP-STATUS & NAVIGATION ---
enum ScreenState {
  SCREEN_MENU = 0,
  SCREEN_VOLUME,
  SCREEN_BRIGHTNESS,
  SCREEN_MEDIA,
  SCREEN_SCROLL,
  SCREEN_SYSTEM,
  SCREEN_AMBIENT,
  SCREEN_SETTINGS
};

const int TOTAL_MENU_ITEMS = 7;
ScreenState currentScreen = SCREEN_MENU;
int menuIndex = 0;

bool isMuted = false;
int briPercent = 70;
bool isPlaying = true;
int mediaPulse = 0;
int sysPulse = 0;
int scrollPulse = 0;
int scrollRingPos = 0;
unsigned long actionTimestamp = 0;

unsigned long lastActivityTime = 0;
bool isStandby = false;
int lastDrawnHour = -1;
int lastDrawnMinute = -1;
bool lastDrawnColon = false;

const int TOTAL_SET_ITEMS = 7;
int setIndex = 0;
bool setEditing = false;

uint8_t prevEncState = 0;
int subSteps = 0;
const int8_t ENC_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ============================================================================
// 1. DIE 4 ANIMIERTEN AMBIENT-WELTEN
// ============================================================================
float animPhase1 = 0.0f;
float animPhase2 = 2.1f;
uint8_t sparkLevel[NUM_LEDS] = {0};
unsigned long lastAmbientTick = 0;

void updateAmbientAnimation() {
  if (millis() - lastAmbientTick < 28) return;
  lastAmbientTick = millis();

  strip.setBrightness(ambientBri);

  if (ambientMode == 0) {
    animPhase1 += 0.09f;
    animPhase2 -= 0.06f;
    if (random(100) < 28) sparkLevel[random(NUM_LEDS)] = random(140, 255);

    for (int i = 0; i < NUM_LEDS; i++) {
      if (sparkLevel[i] > 14) sparkLevel[i] -= 14;
      else sparkLevel[i] = 0;

      float angle = (i * 2.0f * PI) / NUM_LEDS;
      float wave1 = (sinf(angle * 2.0f + animPhase1) + 1.0f) * 0.5f;
      float wave2 = (sinf(angle * 3.0f + animPhase2) + 1.0f) * 0.5f;
      float flame = (wave1 * 0.65f + wave2 * 0.35f);
      flame = flame * flame;

      int heat = (int)(flame * 195.0f) + (sparkLevel[i] / 2) + random(-10, 12);
      heat = constrain(heat, 8, 255);

      uint8_t r, g, b = 0;
      if (heat < 70) {
        r = heat * 2; g = 0;
      } else if (heat < 175) {
        r = 255; g = (uint8_t)((heat - 70) * 1.3f);
      } else {
        r = 255;
        g = constrain(136 + (heat - 175) * 1.4f, 136, 245);
        b = (heat - 175) / 3;
      }
      setRingPixel(i, strip.Color(r, g, b));
    }
  } else if (ambientMode == 1) {
    animPhase1 += 0.05f;
    animPhase2 -= 0.035f;
    if (random(100) < 12) sparkLevel[random(NUM_LEDS)] = 200;

    for (int i = 0; i < NUM_LEDS; i++) {
      if (sparkLevel[i] > 10) sparkLevel[i] -= 10;
      else sparkLevel[i] = 0;

      float angle = (i * 2.0f * PI) / NUM_LEDS;
      float w1 = (sinf(angle * 2.0f + animPhase1) + 1.0f) * 0.5f;
      float w2 = (cosf(angle * 1.0f + animPhase2) + 1.0f) * 0.5f;
      float wave = (w1 * 0.6f + w2 * 0.4f);

      uint8_t r = (uint8_t)(sparkLevel[i] * 0.6f);
      uint8_t g = constrain((int)(wave * 190.0f) + (sparkLevel[i] / 2), 10, 255);
      uint8_t b = constrain((int)(60 + wave * 195.0f), 70, 255);
      setRingPixel(i, strip.Color(r, g, b));
    }
  } else if (ambientMode == 2) {
    animPhase1 += 0.04f;
    animPhase2 += 0.025f;

    for (int i = 0; i < NUM_LEDS; i++) {
      float angle = (i * 2.0f * PI) / NUM_LEDS;
      float ribbon = (sinf(angle * 2.0f + animPhase1) + 1.0f) * 0.5f;
      float curtain = (sinf(angle * 3.0f - animPhase2) + 1.0f) * 0.5f;
      uint16_t hue = 20000 + (uint16_t)(ribbon * 28000.0f);
      uint8_t val  = constrain((int)(35 + curtain * 220.0f), 25, 255);
      setRingPixel(i, strip.gamma32(strip.ColorHSV(hue, 240, val)));
    }
  } else if (ambientMode == 3) {
    animPhase1 += 0.22f;
    if (animPhase1 >= NUM_LEDS) animPhase1 -= NUM_LEDS;

    int headCyan = (int)animPhase1;
    int headPink = (headCyan + (NUM_LEDS / 2)) % NUM_LEDS;

    for (int i = 0; i < NUM_LEDS; i++) setRingPixel(i, strip.Color(12, 0, 28));

    for (int tail = 0; tail < 7; tail++) {
      int idx = (headCyan - tail + NUM_LEDS) % NUM_LEDS;
      uint8_t b = 255 >> tail;
      setRingPixel(idx, strip.Color(0, (b * 220) >> 8, b));
    }
    for (int tail = 0; tail < 7; tail++) {
      int idx = (headPink - tail + NUM_LEDS) % NUM_LEDS;
      uint8_t b = 255 >> tail;
      setRingPixel(idx, strip.Color(b, 0, (b * 180) >> 8));
    }
  }

  strip.show();
}

// ============================================================================
// 2. STANDBY-UHRZEIT-SCREEN
// ============================================================================
void drawStandbyClock(bool forceFull) {
  int h, m, s;
  getCurrentTime(h, m, s);
  bool colonOn = ((millis() / 1000) % 2 == 0);

  if (forceFull) {
    tft.fillScreen(theme.bg);
    lastDrawnHour = -1;
    lastDrawnMinute = -1;
  }

  if (forceFull || h != lastDrawnHour || m != lastDrawnMinute) {
    lastDrawnHour = h;
    lastDrawnMinute = m;
    tft.fillRect(28, 72, 240, 84, theme.bg);

    tft.setTextSize(6);
    tft.setTextColor(theme.fg);
    tft.setCursor(46, 92);
    printTwoDigits(h);

    tft.setCursor(164, 92);
    printTwoDigits(m);

    tft.fillRect(60, 184, 176, 16, theme.bg);
    tft.setTextSize(1);
    tft.setTextColor(theme.muted);
    if (timeSyncedViaWifi) {
      tft.setCursor(104, 188);
      tft.print("NTP SYNC AKTIV");
    } else {
      tft.setCursor(122, 188);
      tft.print("STANDBY");
    }
  }

  if (forceFull || colonOn != lastDrawnColon) {
    lastDrawnColon = colonOn;
    uint16_t col = colonOn ? theme.accent : theme.border;
    tft.fillCircle(148, 104, 4, col);
    tft.fillCircle(148, 126, 4, col);
  }
}

void enterStandby() {
  isStandby = true;
  strip.clear();
  strip.show();
  analogWrite(TFT_BL, 25);
  drawStandbyClock(true);
}

void wakeFromStandby() {
  isStandby = false;
  lastActivityTime = millis();
  applyHardwareBrightness();
}

// ============================================================================
// 3. BUTTERWEICHE 3-SEKUNDEN BOOT-ANIMATION (100% OHNE WLAN-LAGS!)
// ============================================================================
void playBootAnimation() {
  tft.fillScreen(theme.bg);

  int cx = 148;
  int cy = 96;
  tft.drawCircle(cx, cy, 28, theme.border);
  tft.drawCircle(cx, cy, 27, theme.border);
  tft.drawCircle(cx, cy, 18, theme.fg);
  tft.fillCircle(cx, cy, 6, theme.accent);

  const int barWidth = 164;
  const int barX = (296 - barWidth) / 2;
  const int barY = 162;

  tft.fillRoundRect(barX, barY, barWidth, 4, 2, theme.surface);

  unsigned long startTime = millis();
  const unsigned long duration = 3000UL;
  const float k = 4.0f;

  int lastFillW = 0;
  float cometPos = 0.0f;

  // Läuft komplett ungestört mit ~70 FPS durch (kein WLAN-Blockieren während der Animation!)
  while (millis() - startTime < duration) {
    float t = (float)(millis() - startTime) / (float)duration;
    float expProgress = (expf(k * t) - 1.0f) / (expf(k) - 1.0f);

    int fillW = (int)(expProgress * barWidth);
    if (fillW > lastFillW) {
      tft.fillRect(barX + lastFillW, barY, fillW - lastFillW, 4, theme.accent);
      lastFillW = fillW;
    }

    cometPos += 0.18f + (expProgress * 0.75f);
    if (cometPos >= NUM_LEDS) cometPos -= NUM_LEDS;
    int head = (int)cometPos;

    strip.clear();
    for (int tail = 0; tail < 7; tail++) {
      int idx = (head - tail + NUM_LEDS) % NUM_LEDS;
      uint8_t intensity = 255 >> tail;
      setRingPixel(idx, getRingColor(idx, intensity));
    }
    strip.show();
    delay(14);
  }

  tft.fillRoundRect(barX, barY, barWidth, 4, 2, theme.fg);
  for (int i = 0; i < NUM_LEDS; i++) setRingPixel(i, getRingColor(i, 180));
  strip.show();
  delay(120);
}

// ============================================================================
// 4. SKALIERBARE VEKTOR-ICONS
// ============================================================================
void drawThickCircle(int cx, int cy, int r, uint16_t col, int thickness) {
  for (int i = 0; i < thickness; i++) {
    tft.drawCircle(cx, cy, r - i, col);
  }
}

void drawIconSpeaker(int cx, int cy, uint16_t col, bool muted, int s) {
  tft.fillRoundRect(cx - 11 * s, cy - 5 * s, 6 * s, 10 * s, 1 * s, col);
  tft.fillTriangle(cx - 6 * s, cy - 5 * s, cx + 3 * s, cy - 12 * s, cx + 3 * s, cy + 12 * s, col);
  if (muted) {
    for (int t = 0; t < s; t++) {
      tft.drawLine(cx + 6 * s + t, cy - 5 * s, cx + 13 * s + t, cy + 5 * s, theme.accent);
      tft.drawLine(cx + 13 * s + t, cy - 5 * s, cx + 6 * s + t, cy + 5 * s, theme.accent);
    }
  } else {
    tft.drawCircleHelper(cx + 4 * s, cy, 6 * s, 0x06, col);
    tft.drawCircleHelper(cx + 4 * s, cy, 10 * s, 0x06, col);
    if (s > 1) {
      tft.drawCircleHelper(cx + 4 * s, cy, 6 * s - 1, 0x06, col);
      tft.drawCircleHelper(cx + 4 * s, cy, 10 * s - 1, 0x06, col);
    }
  }
}

void drawIconSun(int cx, int cy, uint16_t col, int s) {
  tft.fillCircle(cx, cy, 6 * s, col);
  for (int i = 0; i < 8; i++) {
    float a = i * PI / 4.0f;
    int x1 = cx + cosf(a) * (9 * s);
    int y1 = cy + sinf(a) * (9 * s);
    int x2 = cx + cosf(a) * (14 * s);
    int y2 = cy + sinf(a) * (14 * s);
    tft.drawLine(x1, y1, x2, y2, col);
    if (s > 1) tft.drawLine(x1 + 1, y1, x2 + 1, y2, col);
  }
}

void drawIconMedia(int cx, int cy, uint16_t col, int s) {
  drawThickCircle(cx, cy, 13 * s, col, s);
  tft.fillTriangle(cx - 4 * s, cy - 6 * s, cx - 4 * s, cy + 6 * s, cx + 6 * s, cy, col);
}

void drawIconScroll(int cx, int cy, uint16_t col, int s) {
  for (int t = 0; t < s; t++) {
    tft.drawRoundRect(cx - 8 * s + t, cy - 13 * s + t, 16 * s - 2 * t, 26 * s - 2 * t, 7 * s, col);
  }
  tft.fillTriangle(cx, cy - 9 * s, cx - 3 * s, cy - 5 * s, cx + 3 * s, cy - 5 * s, col);
  tft.fillRoundRect(cx - s, cy - 2 * s, 2 * s + 1, 5 * s, s, theme.accent);
  tft.fillTriangle(cx, cy + 9 * s, cx - 3 * s, cy + 5 * s, cx + 3 * s, cy + 5 * s, col);
}

void drawIconSystem(int cx, int cy, uint16_t col, int s) {
  for (int t = 0; t < s; t++) {
    tft.drawRoundRect(cx - 13 * s + t, cy - 9 * s + t, 26 * s - 2 * t, 17 * s - 2 * t, 3 * s, col);
    tft.drawFastVLine(cx + t, cy - 8 * s, 15 * s, col);
    tft.drawFastHLine(cx - 6 * s, cy + 11 * s + t, 12 * s, col);
  }
  tft.fillRect(cx - s, cy + 8 * s, 2 * s, 3 * s, col);
}

void drawIconFlame(int cx, int cy, uint16_t col, int s) {
  tft.fillCircle(cx, cy + 4 * s, 9 * s, col);
  tft.fillTriangle(cx - 9 * s, cy + 2 * s, cx + 9 * s, cy + 2 * s, cx + 1 * s, cy - 14 * s, col);
  tft.fillTriangle(cx - 8 * s, cy + 4 * s, cx, cy + 4 * s, cx - 5 * s, cy - 8 * s, col);
  uint16_t innerCol = (s > 1) ? theme.surface : theme.bg;
  tft.fillCircle(cx, cy + 6 * s, 4 * s, innerCol);
  tft.fillTriangle(cx - 4 * s, cy + 5 * s, cx + 4 * s, cy + 5 * s, cx, cy - 4 * s, innerCol);
}

void drawIconSettings(int cx, int cy, uint16_t col, int s) {
  for (int i = 0; i < 8; i++) {
    float a = i * PI / 4.0f;
    int tx = cx + cosf(a) * (10 * s);
    int ty = cy + sinf(a) * (10 * s);
    tft.fillCircle(tx, ty, 2 * s, col);
  }
  tft.fillCircle(cx, cy, 9 * s, col);
  tft.fillCircle(cx, cy, 4 * s, theme.surface);
}

void drawMenuIconByIndex(int idx, int cx, int cy, uint16_t col, int scale) {
  switch (idx) {
    case 0: drawIconSpeaker(cx, cy, col, false, scale); break;
    case 1: drawIconSun(cx, cy, col, scale); break;
    case 2: drawIconMedia(cx, cy, col, scale); break;
    case 3: drawIconScroll(cx, cy, col, scale); break;
    case 4: drawIconSystem(cx, cy, col, scale); break;
    case 5: drawIconFlame(cx, cy, col, scale); break;
    case 6: drawIconSettings(cx, cy, col, scale); break;
  }
}

// ============================================================================
// 5. CORNER-SAFE UI & SCREENS
// ============================================================================

void drawTopPill() {
  if (isStandby || currentScreen == SCREEN_AMBIENT) return;
  int pillW = 96;
  int pillX = (296 - pillW) / 2;
  int pillY = 8;

  tft.fillRoundRect(pillX, pillY, pillW, 18, 9, theme.surface);
  tft.drawRoundRect(pillX, pillY, pillW, 18, 9, theme.border);

  uint16_t dotCol = bleConnected ? theme.accent : theme.muted;
  tft.fillCircle(pillX + 14, pillY + 9, 3, dotCol);

  tft.setTextSize(1);
  tft.setTextColor(bleConnected ? theme.fg : theme.muted);
  tft.setCursor(pillX + 26, pillY + 5);
  tft.print(bleConnected ? "CONNECTED" : "PAIRING..");
}

void playViewTransition(bool entering) {
  int cy = 122;
  for (int step = 1; step <= 4; step++) {
    int w = entering ? (step * 55) : ((5 - step) * 55);
    tft.drawFastHLine(148 - w / 2, cy, w, entering ? theme.accent : theme.border);
    delay(7);
  }
  tft.fillRect(14, 30, 268, 204, theme.bg);
}

void updateLeds() {
  if (isStandby) {
    strip.clear();
    strip.show();
    return;
  }
  if (currentScreen == SCREEN_AMBIENT) {
    return;
  }

  applyHardwareBrightness();
  strip.clear();

  if (currentScreen == SCREEN_MENU) {
    int center = (menuIndex * NUM_LEDS) / TOTAL_MENU_ITEMS;
    for (int offset = -1; offset <= 1; offset++) {
      int idx = (center + offset + NUM_LEDS) % NUM_LEDS;
      uint8_t intensity = (offset == 0) ? 255 : 70;
      setRingPixel(idx, getRingColor(idx, intensity));
    }
  } else if (currentScreen == SCREEN_VOLUME) {
    if (isMuted) {
      for (int i = 0; i < NUM_LEDS; i += 6) setRingPixel(i, getRingColor(i, 120));
    } else {
      int totalSub = (volPercent * NUM_LEDS * 255) / 100;
      for (int i = 0; i < NUM_LEDS; i++) {
        int rem = totalSub - (i * 255);
        if (rem > 0) {
          uint8_t b = (rem >= 255) ? 255 : (uint8_t)rem;
          setRingPixel(i, getRingColor(i, b));
        }
      }
    }
  } else if (currentScreen == SCREEN_BRIGHTNESS) {
    int totalSub = (briPercent * NUM_LEDS * 255) / 100;
    for (int i = 0; i < NUM_LEDS; i++) {
      int rem = totalSub - (i * 255);
      if (rem > 0) {
        uint8_t b = (rem >= 255) ? 255 : (uint8_t)rem;
        setRingPixel(i, getRingColor(i, b));
      }
    }
  } else if (currentScreen == SCREEN_MEDIA) {
    if (mediaPulse == -1) {
      for (int i = 15; i <= 21; i++) setRingPixel(i, getRingColor(i, 255));
    } else if (mediaPulse == 1) {
      for (int i = 3; i <= 9; i++) setRingPixel(i, getRingColor(i, 255));
    } else {
      int step = isPlaying ? 2 : 6;
      for (int i = 0; i < NUM_LEDS; i += step) setRingPixel(i, getRingColor(i, 170));
    }
  } else if (currentScreen == SCREEN_SCROLL) {
    if (scrollPulse == 2) {
      for (int i = 8; i <= 16; i++) setRingPixel(i, getRingColor(i, 255));
    } else {
      for (int offset = -1; offset <= 1; offset++) {
        int idx = (scrollRingPos + offset + NUM_LEDS) % NUM_LEDS;
        uint8_t b = (offset == 0) ? 255 : 85;
        setRingPixel(idx, getRingColor(idx, b));
      }
    }
  } else if (currentScreen == SCREEN_SYSTEM) {
    if (sysPulse == -1) {
      for (int i = 16; i <= 20; i++) setRingPixel(i, getRingColor(i, 255));
    } else if (sysPulse == 1) {
      for (int i = 4; i <= 8; i++) setRingPixel(i, getRingColor(i, 255));
    } else {
      for (int i = 0; i < NUM_LEDS; i += 6) setRingPixel(i, getRingColor(i, 200));
    }
  } else if (currentScreen == SCREEN_SETTINGS) {
    for (int i = 0; i < NUM_LEDS; i++) {
      setRingPixel(i, getRingColor(i, 255));
    }
  }

  strip.show();
}

void drawMenuScreen() {
  tft.fillRect(16, 36, 264, 196, theme.bg);

  int prevIdx = (menuIndex - 1 + TOTAL_MENU_ITEMS) % TOTAL_MENU_ITEMS;
  int nextIdx = (menuIndex + 1) % TOTAL_MENU_ITEMS;

  tft.fillRoundRect(26, 68, 56, 56, 12, theme.bg);
  tft.drawRoundRect(26, 68, 56, 56, 12, theme.border);
  drawMenuIconByIndex(prevIdx, 54, 96, theme.muted, 1);

  tft.fillRoundRect(214, 68, 56, 56, 12, theme.bg);
  tft.drawRoundRect(214, 68, 56, 56, 12, theme.border);
  drawMenuIconByIndex(nextIdx, 242, 96, theme.muted, 1);

  int mainW = 92;
  int mainX = (296 - mainW) / 2;
  int mainY = 48;
  tft.fillRoundRect(mainX, mainY, mainW, mainW, 20, theme.surface);
  tft.drawRoundRect(mainX, mainY, mainW, mainW, 20, theme.accent);
  tft.drawRoundRect(mainX + 1, mainY + 1, mainW - 2, mainW - 2, 19, theme.accent);
  drawMenuIconByIndex(menuIndex, 148, mainY + mainW / 2, theme.fg, 2);

  tft.fillRoundRect(124, 150, 48, 4, 2, theme.accent);

  const char* labels[7] = { "AUDIO", "PC LICHT", "MEDIA", "SCROLL", "WORKSPACE", "AMBIENT", "SETUP" };
  tft.setTextSize(2);
  tft.setTextColor(theme.fg);
  int textW = strlen(labels[menuIndex]) * 12;
  tft.setCursor((296 - textW) / 2, 168);
  tft.print(labels[menuIndex]);

  for (int i = 0; i < TOTAL_MENU_ITEMS; i++) {
    int dotX = 100 + i * 16;
    if (i == menuIndex) {
      tft.fillRoundRect(dotX - 4, 208, 8, 5, 2, theme.accent);
    } else {
      tft.fillCircle(dotX, 210, 2, theme.border);
    }
  }
}

void drawLevelScreen(bool isVolume, bool fullRedraw) {
  int pct = isVolume ? volPercent : briPercent;

  if (fullRedraw) {
    tft.setTextColor(theme.muted);
    tft.setTextSize(1);
    tft.setCursor(116, 212);
    tft.print("HOLD: BACK");
  }

  tft.fillRect(42, 60, 72, 72, theme.bg);
  if (isVolume) {
    drawIconSpeaker(76, 96, isMuted ? theme.muted : theme.fg, isMuted, 2);
  } else {
    drawIconSun(76, 96, theme.fg, 2);
  }

  tft.fillRect(122, 72, 145, 48, theme.bg);
  tft.setTextSize(4);
  if (isVolume && isMuted) {
    tft.setTextColor(theme.muted);
    tft.setCursor(126, 82);
    tft.print("MUTE");
  } else {
    tft.setTextColor(theme.fg);
    tft.setCursor(126, 82);
    printTwoDigits(pct);
    tft.setTextSize(2);
    tft.setTextColor(theme.accent);
    tft.print("%");
  }

  int barX = 48;
  int barY = 158;
  int barW = 200;
  int fillW = (pct * barW) / 100;

  tft.fillRect(barX - 8, barY - 6, barW + 16, 20, theme.bg);
  tft.fillRoundRect(barX, barY, barW, 8, 4, theme.surface);
  if (fillW > 0) {
    uint16_t barCol = (isVolume && isMuted) ? theme.muted : theme.accent;
    tft.fillRoundRect(barX, barY, fillW, 8, 4, barCol);
  }
  tft.fillCircle(barX + fillW, barY + 4, 6, (isVolume && isMuted) ? theme.muted : theme.fg);
}

void drawMediaScreen(bool fullRedraw) {
  if (fullRedraw) {
    tft.setTextColor(theme.muted);
    tft.setTextSize(1);
    tft.setCursor(116, 212);
    tft.print("HOLD: BACK");
  }

  int cy = 118;

  uint16_t colPrev = (mediaPulse == -1) ? theme.accent : theme.muted;
  tft.fillCircle(62, cy, 26, (mediaPulse == -1) ? theme.surface : theme.bg);
  tft.drawCircle(62, cy, 26, theme.border);
  tft.fillRect(50, cy - 9, 4, 18, colPrev);
  tft.fillTriangle(55, cy, 70, cy - 9, 70, cy + 9, colPrev);

  uint16_t ringCol = (mediaPulse == 2) ? theme.fg : theme.accent;
  tft.fillCircle(148, cy, 40, theme.surface);
  drawThickCircle(148, cy, 40, ringCol, 2);

  if (isPlaying) {
    tft.fillRoundRect(136, cy - 14, 8, 28, 3, theme.fg);
    tft.fillRoundRect(152, cy - 14, 8, 28, 3, theme.fg);
  } else {
    tft.fillTriangle(140, cy - 15, 140, cy + 15, 164, cy, theme.fg);
  }

  uint16_t colNext = (mediaPulse == 1) ? theme.accent : theme.muted;
  tft.fillCircle(234, cy, 26, (mediaPulse == 1) ? theme.surface : theme.bg);
  tft.drawCircle(234, cy, 26, theme.border);
  tft.fillTriangle(226, cy - 9, 226, cy + 9, 241, cy, colNext);
  tft.fillRect(242, cy - 9, 4, 18, colNext);
}

void drawScrollScreen(bool fullRedraw) {
  if (fullRedraw) {
    tft.setTextColor(theme.muted);
    tft.setTextSize(1);
    tft.setCursor(116, 212);
    tft.print("HOLD: BACK");
  }

  int cx = 148;
  int cy = 106;

  uint16_t colUp = (scrollPulse == -1) ? theme.accent : theme.muted;
  tft.fillTriangle(62, cy - 12, 48, cy + 6, 76, cy + 6, colUp);

  uint16_t colDn = (scrollPulse == 1 || scrollPulse == 2) ? theme.accent : theme.muted;
  tft.fillTriangle(234, cy + 12, 220, cy - 6, 248, cy - 6, colDn);

  uint16_t borderCol = (scrollPulse != 0) ? theme.accent : theme.border;
  tft.fillRoundRect(cx - 44, cy - 40, 88, 80, 18, theme.surface);
  tft.drawRoundRect(cx - 44, cy - 40, 88, 80, 18, borderCol);

  drawIconScroll(cx, cy, (scrollPulse == 2) ? theme.accent : theme.fg, 2);

  tft.fillRect(40, 160, 216, 20, theme.bg);
  tft.setTextSize(1);
  tft.setTextColor((scrollPulse != 0) ? theme.accent : theme.muted);
  if (scrollPulse == -1) {
    tft.setCursor(102, 166);
    tft.print("SCROLL HOCH ^^");
  } else if (scrollPulse == 1) {
    tft.setCursor(96, 166);
    tft.print("SCROLL RUNTER vv");
  } else if (scrollPulse == 2) {
    tft.setCursor(92, 166);
    tft.print("PFEIL UNTEN [v]");
  } else {
    tft.setCursor(62, 166);
    tft.print("TURN: SCROLL | CLICK: DOWN");
  }
}

void drawSystemScreen(bool fullRedraw) {
  if (fullRedraw) {
    tft.setTextColor(theme.muted);
    tft.setTextSize(1);
    tft.setCursor(116, 212);
    tft.print("HOLD: BACK");
  }

  int cx = 148;
  int cy = 108;

  uint16_t colL = (sysPulse == -1) ? theme.accent : theme.muted;
  tft.fillTriangle(46, cy, 64, cy - 14, 64, cy + 14, colL);

  uint16_t colR = (sysPulse == 1) ? theme.accent : theme.muted;
  tft.fillTriangle(250, cy, 232, cy - 14, 232, cy + 14, colR);

  uint16_t borderCol = (sysPulse == 2) ? theme.accent : theme.border;
  tft.fillRoundRect(cx - 48, cy - 40, 96, 80, 18, theme.surface);
  tft.drawRoundRect(cx - 48, cy - 40, 96, 80, 18, borderCol);

  drawIconSystem(cx, cy - 4, (sysPulse == 2) ? theme.accent : theme.fg, 2);

  tft.fillRect(50, 160, 196, 20, theme.bg);
  tft.setTextSize(1);
  tft.setTextColor((sysPulse != 0) ? theme.accent : theme.muted);
  if (sysPulse == -1) {
    tft.setCursor(96, 166);
    tft.print("<< DESKTOP LINKS");
  } else if (sysPulse == 1) {
    tft.setCursor(94, 166);
    tft.print("DESKTOP RECHTS >>");
  } else if (sysPulse == 2) {
    tft.setCursor(108, 166);
    tft.print("PC GESPERRT");
  } else {
    tft.setCursor(76, 166);
    tft.print("TURN: DESK  |  CLICK: LOCK");
  }
}

void drawSettingsScreen() {
  const char* titles[7] = {
    "FARBSCHEMA",
    "DISPLAY LICHT",
    "LED HELLIGKEIT",
    "LED FARBE",
    "UHRZEIT (DE)",
    "PC AUDIO SYNC",
    "<< HAUPTMENUE"
  };

  const char* themeNames[3] = { "DARK CYAN", "SCHWARZ/WEISS", "INVERTIERT" };
  const char* colorNames[8] = { "WEISS", "CYAN", "BLAU", "VIOLETT", "ROT", "GOLD", "GRUEN", "RAINBOW" };

  tft.fillRect(18, 30, 260, 204, theme.bg);

  int h, m, s;
  getCurrentTime(h, m, s);

  for (int i = 0; i < TOTAL_SET_ITEMS; i++) {
    int y = 32 + i * 25;
    bool sel = (i == setIndex);

    if (sel) {
      uint16_t boxBorder = setEditing ? theme.accent : theme.fg;
      tft.fillRoundRect(24, y, 248, 22, 6, theme.surface);
      tft.drawRoundRect(24, y, 248, 22, 6, boxBorder);
    }

    tft.setTextSize(1);
    tft.setTextColor(sel ? theme.fg : theme.muted);
    tft.setCursor(34, y + 7);
    tft.print(titles[i]);

    tft.setTextColor((sel && setEditing) ? theme.accent : (sel ? theme.fg : theme.muted));
    tft.setCursor(172, y + 7);

    if (i == 0) {
      tft.print(themeNames[cfgTheme]);
    } else if (i == 1) {
      tft.print(cfgTftBri * 10);
      tft.print(" %");
    } else if (i == 2) {
      tft.print(cfgLedBri * 10);
      tft.print(" %");
    } else if (i == 3) {
      tft.print(colorNames[cfgLedCol]);
    } else if (i == 4) {
      printTwoDigits(h);
      tft.print(':');
      printTwoDigits(m);
      tft.print(timeSyncedViaWifi ? " NTP" : " MAN");
    } else if (i == 5) {
      tft.print("AUF 50% SYNC");
    } else if (i == 6) {
      tft.print("SPEICHERN");
    }
  }

  tft.setTextSize(1);
  tft.setTextColor(theme.muted);
  tft.setCursor(54, 212);
  tft.print(setEditing ? "Drehen: Aendern | Klick: OK" : "Drehen: Wahl | Klick: Edit");
}

void renderUI(bool fullRedraw) {
  if (currentScreen == SCREEN_AMBIENT) {
    tft.fillScreen(0x0000);
    analogWrite(TFT_BL, 0);
    return;
  }

  applyHardwareBrightness();

  if (fullRedraw) {
    tft.fillScreen(theme.bg);
    drawTopPill();
  }

  switch (currentScreen) {
    case SCREEN_MENU:       drawMenuScreen(); break;
    case SCREEN_VOLUME:     drawLevelScreen(true, fullRedraw); break;
    case SCREEN_BRIGHTNESS: drawLevelScreen(false, fullRedraw); break;
    case SCREEN_MEDIA:      drawMediaScreen(fullRedraw); break;
    case SCREEN_SCROLL:     drawScrollScreen(fullRedraw); break;
    case SCREEN_SYSTEM:     drawSystemScreen(fullRedraw); break;
    case SCREEN_SETTINGS:   drawSettingsScreen(); break;
    default: break;
  }
  updateLeds();
}

void setup() {
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(TFT_BL, OUTPUT);

  prevEncState = (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT);

  strip.begin();
  loadSettings();
  strip.show();

  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.beginLandscape();

  // 1. Zuerst die 3-Sekunden Startup-Animation komplett ohne WLAN-Funkstörungen abspielen!
  playBootAnimation();

  // 2. Hauptmenü sofort anzeigen
  lastActivityTime = millis();
  renderUI(true);

  // 3. Im Hintergrund synchronisiert jetzt pollBackgroundWifi() die deutsche Uhrzeit
  // und schaltet danach automatisch Bluetooth ein!
}

void loop() {
  // 1. Nicht-blockierender WLAN-Zeitabgleich im Hintergrund (startet danach Bluetooth)
  pollBackgroundWifi();

  bleConnected = isBleActive();
  if (bleConnected != lastBleState) {
    lastBleState = bleConnected;
    if (!isStandby && currentScreen != SCREEN_AMBIENT) {
      drawTopPill();
    }
  }

  if (bleInitialized && shouldRestartAdv && !bleConnected && (millis() - disconnectTime > 150)) {
    shouldRestartAdv = false;
    BLEDevice::startAdvertising();
  }

  static unsigned long lastTimeAutoSave = 0;
  if (millis() - lastTimeAutoSave > 300000UL) {
    lastTimeAutoSave = millis();
    saveSettings();
  }

  if (currentScreen == SCREEN_AMBIENT && !isStandby) {
    updateAmbientAnimation();
  }

  if (!isStandby && currentScreen != SCREEN_AMBIENT) {
    if (millis() - lastActivityTime >= STANDBY_TIMEOUT_MS) {
      enterStandby();
    }
  } else if (isStandby) {
    static unsigned long lastClockTick = 0;
    if (millis() - lastClockTick >= 250) {
      lastClockTick = millis();
      drawStandbyClock(false);
    }
  }

  int delta = 0;
  uint8_t currState = (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DT);
  if (currState != prevEncState) {
    subSteps += ENC_TABLE[(prevEncState << 2) | currState];
    prevEncState = currState;

    if (subSteps >= 4) {
      delta = 1;
      subSteps = 0;
    } else if (subSteps <= -4) {
      delta = -1;
      subSteps = 0;
    }
  }

  if (delta != 0) {
    lastActivityTime = millis();

    if (isStandby) {
      wakeFromStandby();
      renderUI(true);
      return;
    }

    switch (currentScreen) {
      case SCREEN_MENU:
        menuIndex = (menuIndex + delta + TOTAL_MENU_ITEMS) % TOTAL_MENU_ITEMS;
        renderUI(false);
        break;

      case SCREEN_VOLUME: {
        isMuted = false;
        int oldHwSteps = (volPercent + 1) / 2;
        volPercent = constrain(volPercent + (delta * 5), 0, 100);
        int newHwSteps = (volPercent + 1) / 2;
        int pulses = abs(newHwSteps - oldHwSteps);
        if (pulses == 0) pulses = 2;

        sendMediaBit(delta > 0 ? MEDIA_BIT_VOL_UP : MEDIA_BIT_VOL_DOWN, pulses);
        renderUI(false);
        break;
      }

      case SCREEN_BRIGHTNESS:
        briPercent = constrain(briPercent + (delta * 5), 0, 100);
        sendMediaBit(delta > 0 ? MEDIA_BIT_BRI_UP : MEDIA_BIT_BRI_DOWN, 1);
        renderUI(false);
        break;

      case SCREEN_MEDIA:
        if (millis() - actionTimestamp > 220) {
          actionTimestamp = millis();
          mediaPulse = (delta > 0) ? 1 : -1;
          sendMediaBit(delta > 0 ? MEDIA_BIT_NEXT : MEDIA_BIT_PREV, 1);
          renderUI(false);
        }
        break;

      case SCREEN_SCROLL:
        scrollRingPos = (scrollRingPos + delta + NUM_LEDS) % NUM_LEDS;
        scrollPulse = (delta > 0) ? 1 : -1;
        actionTimestamp = millis();
        sendMouseScroll(delta > 0 ? -2 : 2);
        renderUI(false);
        break;

      case SCREEN_SYSTEM:
        if (millis() - actionTimestamp > 250) {
          actionTimestamp = millis();
          sysPulse = (delta > 0) ? 1 : -1;
          sendKeyboardShortcut(KEY_MOD_LGUI | KEY_MOD_LCTRL,
                               (delta > 0) ? KEY_RIGHT_ARROW : KEY_LEFT_ARROW);
          renderUI(false);
        }
        break;

      case SCREEN_AMBIENT:
        ambientBri = constrain(ambientBri + (delta * 8), 15, 140);
        break;

      case SCREEN_SETTINGS:
        if (!setEditing) {
          setIndex = (setIndex + delta + TOTAL_SET_ITEMS) % TOTAL_SET_ITEMS;
          renderUI(false);
        } else {
          bool themeChanged = false;
          if (setIndex == 0) {
            cfgTheme = (cfgTheme + delta + 3) % 3;
            applyTheme();
            themeChanged = true;
          } else if (setIndex == 1) {
            cfgTftBri = constrain((int)cfgTftBri + delta, 1, 10);
            applyHardwareBrightness();
          } else if (setIndex == 2) {
            cfgLedBri = constrain((int)cfgLedBri + delta, 1, 10);
            applyHardwareBrightness();
          } else if (setIndex == 3) {
            cfgLedCol = (cfgLedCol + delta + 8) % 8;
          } else if (setIndex == 4) {
            int h, m, s;
            getCurrentTime(h, m, s);
            timeSyncedViaWifi = false;
            int totalMin = (h * 60 + m + delta + 1440) % 1440;
            long newBase = (long)totalMin * 60L - CLOCK_OFFSET_SEC + 86400L;
            clockBaseSeconds = newBase % 86400L;
            clockBaseMillis  = millis();
          } else if (setIndex == 5) {
            volPercent = constrain(volPercent + (delta * 5), 0, 100);
          }
          renderUI(themeChanged);
        }
        break;
    }
  }

  if (!isStandby && (mediaPulse != 0 || sysPulse != 0 || scrollPulse != 0) && (millis() - actionTimestamp > 450)) {
    mediaPulse = 0;
    sysPulse = 0;
    scrollPulse = 0;
    if (currentScreen == SCREEN_MEDIA || currentScreen == SCREEN_SYSTEM || currentScreen == SCREEN_SCROLL) {
      renderUI(false);
    }
  }

  static bool btnWasDown = false;
  static bool longPressHandled = false;
  static unsigned long btnDownTime = 0;

  bool btnDown = (digitalRead(ENC_SW) == LOW);

  if (btnDown && !btnWasDown) {
    btnWasDown = true;
    longPressHandled = false;
    btnDownTime = millis();
    lastActivityTime = millis();
  } else if (btnDown && btnWasDown && !longPressHandled) {
    lastActivityTime = millis();
    if (millis() - btnDownTime > 550 && currentScreen != SCREEN_MENU && !isStandby) {
      longPressHandled = true;
      setEditing = false;
      saveSettings();
      if (currentScreen != SCREEN_AMBIENT) playViewTransition(false);
      currentScreen = SCREEN_MENU;
      renderUI(true);
    }
  } else if (!btnDown && btnWasDown) {
    btnWasDown = false;
    lastActivityTime = millis();

    if (isStandby) {
      wakeFromStandby();
      renderUI(true);
      return;
    }

    if (!longPressHandled && (millis() - btnDownTime > 35)) {
      switch (currentScreen) {
        case SCREEN_MENU:
          if (menuIndex != 5) playViewTransition(true);
          if (menuIndex == 0) currentScreen = SCREEN_VOLUME;
          if (menuIndex == 1) currentScreen = SCREEN_BRIGHTNESS;
          if (menuIndex == 2) currentScreen = SCREEN_MEDIA;
          if (menuIndex == 3) currentScreen = SCREEN_SCROLL;
          if (menuIndex == 4) currentScreen = SCREEN_SYSTEM;
          if (menuIndex == 5) currentScreen = SCREEN_AMBIENT;
          if (menuIndex == 6) {
            currentScreen = SCREEN_SETTINGS;
            setIndex = 0;
            setEditing = false;
          }
          renderUI(true);
          break;

        case SCREEN_VOLUME:
          isMuted = !isMuted;
          sendMediaBit(MEDIA_BIT_MUTE, 1);
          renderUI(false);
          break;

        case SCREEN_BRIGHTNESS:
          playViewTransition(false);
          currentScreen = SCREEN_MENU;
          renderUI(true);
          break;

        case SCREEN_MEDIA:
          isPlaying = !isPlaying;
          mediaPulse = 2;
          actionTimestamp = millis();
          sendMediaBit(MEDIA_BIT_PLAY_PAUSE, 1);
          renderUI(false);
          break;

        case SCREEN_SCROLL:
          scrollPulse = 2;
          actionTimestamp = millis();
          sendKeyboardShortcut(0, KEY_DOWN_ARROW);
          renderUI(false);
          break;

        case SCREEN_SYSTEM:
          sysPulse = 2;
          actionTimestamp = millis();
          sendKeyboardShortcut(KEY_MOD_LGUI, KEY_L);
          renderUI(false);
          break;

        case SCREEN_AMBIENT:
          ambientMode = (ambientMode + 1) % TOTAL_AMBIENT_MODES;
          for (int i = 0; i < NUM_LEDS; i++) sparkLevel[i] = 0;
          break;

        case SCREEN_SETTINGS:
          if (setIndex == 6) {
            setEditing = false;
            saveSettings();
            playViewTransition(false);
            currentScreen = SCREEN_MENU;
            renderUI(true);
          } else if (setIndex == 5) {
            syncPcAudioTo50();
            renderUI(false);
          } else {
            setEditing = !setEditing;
            if (!setEditing) {
              saveSettings();
            }
            renderUI(false);
          }
          break;
      }
    }
  }
}