// Guardband firmware: UI (clock, HR/SpO2) + fall-detection FSM.
// Target: ESP32-S3 SuperMini. Single cooperative loop, no RTOS.

#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <MPU9250_asukiaaa.h>
#include <max30102.h>
#include <algorithm_by_RF.h>
#include <max30102_settings.h>
#include "RTClib.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// [CFG] pins
#define SDA_PIN     8
#define SCL_PIN     9
#define BUTTON_PIN  4        // menu
#define CANCEL_PIN  5        // cancel alert
#define BUZZER_PIN  6
#define LED_PIN     48
#define NUM_LEDS    1
#define HAS_OLED    1         // [CFG] 0 = serial stands in for the display


// [CFG] UI timing
#define DEBOUNCE_MS       50
#define CLOCK_REFRESH_MS  1000
#define HR_REFRESH_MS     250
#define CLOCK_TIMEOUT_MS  5000UL
#define HR_TIMEOUT_MS     15000UL   // no valid reading for this long
#define HR_MAX_MS         60000UL   // hard ceiling on HR screen time
#define HOLD_MS           10000UL   // show "--" if no valid block for this long

// [CFG] HR sensor
#define BUFFER_SIZE       100       // RF lib window (~4 s at 25 Hz)
#define SLIDE_STEP        25        // samples dropped per recompute (~1 s)
#define LED_AMP           0x5F
#define IR_FINGER_MIN     30000UL   // below = no finger; tune

// [CFG] fall FSM (g, ms)
#define SVM_FREEFALL_MAX  0.2f
#define SVM_IMPACT_MIN    2.5f
#define SVM_REST_MIN      0.85f
#define SVM_REST_MAX      1.15f
#define T_FREEFALL_WINDOW 1000UL
#define T_INACTIVITY_HOLD 5000UL
#define T_IMPACT_WINDOW   15000UL   // abandon if not still by then
#define T_ALERT_HOLD      5000UL
#define MPU_PERIOD_MS     10        // 100 Hz
#define LOG_INTERVAL_MS   200
#define ACC_OFF_X         0.0f      // g; Z measured by flip test, X/Y not yet
#define ACC_OFF_Y         0.0f
#define ACC_OFF_Z         0.087f

// [CFG] serial logging (1 = on)
#define LOG_MPU           1
#define LOG_DBG           1

// [CFG] BLE (Nordic UART UUIDs, common in App Inventor examples)
#define BLE_NAME          "Guardband"
#define BLE_SVC_UUID      "0000ffe0-0000-1000-8000-00805f9b34fb"
#define BLE_TX_UUID       "0000ffe1-0000-1000-8000-00805f9b34fb"   // notify to phone

// [CFG] alert flow
#define T_CANCEL_WINDOW   15000UL   // countdown before alert is sent
#define T_ALERT_MAX       60000UL   // stop retrying the alert after this
#define BEEP_MS           250

// [IN] debounced press detector, one per button
struct Button {
  uint8_t       pin;
  bool          lastRaw;
  bool          stable;
  unsigned long tChange;
};

// [HW] devices
RTC_DS3231 rtc;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
MPU9250_asukiaaa mpu(0x69);
Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);

// [STATE] screen
enum ScreenState { SCREEN_OFF, SCREEN_CLOCK, SCREEN_HR };
ScreenState   screen      = SCREEN_OFF;
unsigned long screenSince = 0;     // reset on entry and on valid HR reading
unsigned long hrOpenedMs  = 0;     // HR screen entry, for the ceiling
bool          forceDraw   = true;
unsigned long lastDraw    = 0;
bool rtcOk = false;   // set in setup() if the DS3231 answers; checked before reading the clock

// [STATE] HR sensor
uint8_t  skipBlocks  = 3;          // windows discarded while finger settles
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
uint16_t bufferIndex = 0;
bool     fingerOn    = false;
bool     haveReading = false;
int32_t  lastHr      = 0;
float    lastSpo2    = 0;
unsigned long lastValidMs = 0;

// [STATE] fall FSM
enum FallState { NORMAL, FREEFALL, IMPACT, CONFIRMED, ALERTED };
bool          alertSent   = false;
unsigned long alertSentMs = 0;
FallState     fsmState        = NORMAL;
unsigned long fsmSince        = 0;
unsigned long inactivityStart = 0;
unsigned long lastMpuMs       = 0;
unsigned long lastLogMs       = 0;

// [IN] button instances
Button btnMenu   = { BUTTON_PIN, HIGH, HIGH, 0 };
Button btnCancel = { CANCEL_PIN, HIGH, HIGH, 0 };

// [BLE] state
BLEServer*         bleServer    = nullptr;
BLECharacteristic* bleTx        = nullptr;
volatile bool      bleConnected = false;

class BleCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override    { bleConnected = true; }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    BLEDevice::startAdvertising();       // stay discoverable
  }
};

// [BLE] init: one service, one notify characteristic
void bleBegin() {
  BLEDevice::init(BLE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BleCb());
  BLEService* svc = bleServer->createService(BLE_SVC_UUID);
  bleTx = svc->createCharacteristic(BLE_TX_UUID,
          BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// [BLE] send text; false if no phone connected
bool bleNotify(const char* msg) {
  if (!bleConnected || !bleTx) return false;
  bleTx->setValue((uint8_t*)msg, strlen(msg));
  bleTx->notify();
  Serial.printf("[BLE] notify tx payload=%s\n", msg);
  return true;
}

// [UI] assets
static const unsigned char image_bluetooth_connected_bits[] U8X8_PROGMEM = {
  0x80,0x00,0x40,0x01,0x40,0x02,0x44,0x04,0x48,0x04,0x52,0x12,0x64,0x09,0xcc,0x0c,
  0x64,0x09,0x52,0x12,0x48,0x04,0x44,0x04,0x40,0x02,0x40,0x01,0x80,0x00,0x00,0x00
};

const char* daysOfTheWeek[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
const char* months[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};

// [UI] clock screen
void drawClockScreen(const char* timeStr, const char* dateStr) {
  #if !HAS_OLED
    Serial.printf("[UI] clock %s %s\n", timeStr, dateStr);
    return;
  #endif
  u8g2.clearBuffer();
  u8g2.setFontMode(1);
  u8g2.setBitmapMode(1);
  u8g2.setFont(u8g2_font_timR24_tr);
  u8g2.drawStr(12, 36, timeStr);
  u8g2.setFont(u8g2_font_timR14_tr);
  u8g2.drawStr(13, 52, dateStr);
  if (bleConnected) u8g2.drawXBMP(109, 6, 14, 16, image_bluetooth_connected_bits);
  u8g2.sendBuffer();
}

// [UI] HR/SpO2 screen
void drawHrScreen(int bpmVal, int spo2Val, bool validReading) {
  char bpmStr[8];
  char spo2Str[8];

  if (validReading) {
    sprintf(bpmStr, "%d", bpmVal);
    sprintf(spo2Str, "%d", spo2Val);
  } else {
    sprintf(bpmStr, "--");
    sprintf(spo2Str, "--");
  }
  #if !HAS_OLED
    Serial.printf("[UI] hr %s spo2 %s\n", bpmStr, spo2Str);
    return;
  #endif

  u8g2.clearBuffer();
  u8g2.setFontMode(1);
  u8g2.setBitmapMode(1);

  u8g2.setFont(u8g2_font_t0_22_tr);
  u8g2.drawStr(12, 26, bpmStr);
  u8g2.drawStr(12, 51, spo2Str);

  u8g2.setFont(u8g2_font_t0_14b_tr);
  u8g2.drawUTF8(90, 51, "SpO2");
  u8g2.drawStr(97, 26, "BPM");

  u8g2.sendBuffer();
}

// [UI] clock update
void updateClockScreen() {
  if (!rtcOk) {
    Serial.println("[UI] clock: no RTC");
    return;
  }
  DateTime now = rtc.now();

  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", now.hour(), now.minute());

  char dateStr[12];
  sprintf(dateStr, "%s:%02d:%s",
          daysOfTheWeek[now.dayOfTheWeek()],
          now.day(),
          months[now.month() - 1]);

  drawClockScreen(timeStr, dateStr);
}

// [UI] HR update: last valid reading, "--" if stale
void updateHrScreen() {
  bool fresh = haveReading && (millis() - lastValidMs) < HOLD_MS;
  drawHrScreen(lastHr, (int)(lastSpo2 + 0.5f), fresh);
}

// [SENS] start: reset, configure, clear window
void sensorStart() {
  uint8_t dummy;
  maxim_max30102_reset();
  delay(10);
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &dummy);
  maxim_max30102_init();
  setLED1PulseAmplitude(LED_AMP);
  setLED2PulseAmplitude(LED_AMP);
  maxim_max30102_write_reg(REG_SPO2_CONFIG, 0x67);  // ADC 16384 nA, 100 Hz, 411 us
  bufferIndex = 0;
  skipBlocks  = 3;
  fingerOn    = false;
  haveReading = false;
}

// [SENS] stop: reset leaves the part in low-power idle
void sensorStop() {
  maxim_max30102_reset();
}

// [SENS] slide window: drop oldest SLIDE_STEP samples, keep the rest
void slideWindow() {
  memmove(irBuffer,  irBuffer  + SLIDE_STEP, (BUFFER_SIZE - SLIDE_STEP) * sizeof(uint32_t));
  memmove(redBuffer, redBuffer + SLIDE_STEP, (BUFFER_SIZE - SLIDE_STEP) * sizeof(uint32_t));
  bufferIndex = BUFFER_SIZE - SLIDE_STEP;
}

// [SENS] non-blocking poll; one sample per data-ready event
void sensorPoll() {
  uint8_t  st;
  uint32_t red, ir;

  maxim_max30102_read_reg(REG_INTR_STATUS_1, &st);
  if (!(st & 0x40)) return;  // PPG_RDY not set

  maxim_max30102_read_fifo(&red, &ir);

  // [SENS] finger lost: restart window
  fingerOn = (ir > IR_FINGER_MIN);
  if (!fingerOn) {
    bufferIndex = 0;
    haveReading = false;
    skipBlocks  = 3;
    return;
  }

  redBuffer[bufferIndex] = red;
  irBuffer[bufferIndex]  = ir;
  if (++bufferIndex < BUFFER_SIZE) return;

#if LOG_DBG
  // [DBG] window min/max
  uint32_t irMin = irBuffer[0],  irMax = irBuffer[0];
  uint32_t rdMin = redBuffer[0], rdMax = redBuffer[0];
  for (uint16_t i = 1; i < BUFFER_SIZE; i++) {
    if (irBuffer[i]  < irMin) irMin = irBuffer[i];
    if (irBuffer[i]  > irMax) irMax = irBuffer[i];
    if (redBuffer[i] < rdMin) rdMin = redBuffer[i];
    if (redBuffer[i] > rdMax) rdMax = redBuffer[i];
  }
  Serial.printf("[DBG] ir=%lu..%lu red=%lu..%lu\n",
                (unsigned long)irMin, (unsigned long)irMax,
                (unsigned long)rdMin, (unsigned long)rdMax);
#endif

  // [SENS] settling: skip window, otherwise run RF algorithm
  if (skipBlocks) {
    skipBlocks--;
  } else {
    int32_t hr;
    int8_t  hrValid, spo2Valid;
    float   spo2, ratio, correl;
    rf_heart_rate_and_oxygen_saturation(
      irBuffer, BUFFER_SIZE, redBuffer,
      &spo2, &spo2Valid, &hr, &hrValid, &ratio, &correl);

    if (hrValid && spo2Valid && hr >= 40 && hr <= 180 && spo2 >= 85 && spo2 <= 100) {
      lastHr      = haveReading ? (lastHr * 2 + hr) / 3 : hr;
      lastSpo2    = haveReading ? (lastSpo2 * 2 + spo2) / 3 : spo2;
      lastValidMs = millis();
      screenSince = millis();   // [STATE] extend HR timeout on valid reading
      haveReading = true;
    }
    Serial.printf("[SENS] hr=%ld(%d) spo2=%.1f(%d) correl=%.3f\n",
                  (long)hr, hrValid, spo2, spo2Valid, correl);
  }

  slideWindow();  // keep newest samples for the next window
}

// [IN] returns true once per debounced press
bool buttonPressed(Button &b) {
  bool raw = digitalRead(b.pin);
  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.tChange = millis();
  }
  if ((millis() - b.tChange) > DEBOUNCE_MS && raw != b.stable) {
    b.stable = raw;
    if (raw == LOW) return true;
  }
  return false;
}

// [STATE] enter a screen; HR sensor runs only on HR screen
void setScreen(ScreenState s) {
  if (screen == SCREEN_HR && s != SCREEN_HR) sensorStop();
  if (screen != SCREEN_HR && s == SCREEN_HR) {
    sensorStart();
    hrOpenedMs = millis();
  }
#if HAS_OLED
  if (s == SCREEN_OFF) {
    u8g2.setPowerSave(1);
  } else if (screen == SCREEN_OFF) {
    u8g2.initDisplay();       // [HW] resend panel setup on wake
    u8g2.setFlipMode(0);      // [HW] force normal orientation
    u8g2.setPowerSave(0);
  }
#endif
  screen      = s;
  screenSince = millis();
  forceDraw   = true;
}

// [IN] menu press: off -> clock -> HR -> clock ...
void menuPress() {
  Serial.println("[IN] menu pressed");
  if (screen == SCREEN_CLOCK) setScreen(SCREEN_HR);
  else                        setScreen(SCREEN_CLOCK);  // from OFF or HR
}

// [FSM] state change: log, set LED
void fsmSet(FallState s, uint32_t colour, const char* name, const char* cond) {
  fsmState = s;
  fsmSince = millis();
  led.setPixelColor(0, colour);
  led.show();
  Serial.printf("[FSM] state=%s cond=%s\n", name, cond);
}

// [FSM] one step per MPU sample
void fsmStep(float svm, unsigned long now) {
  switch (fsmState) {
    case NORMAL:
      if (svm < SVM_FREEFALL_MAX) {
        fsmSet(FREEFALL, led.Color(255, 255, 0), "FREEFALL", "svm below freefall threshold");
      }
      break;

    case FREEFALL:
      if (svm > SVM_IMPACT_MIN) {
        inactivityStart = now;
        fsmSet(IMPACT, led.Color(255, 64, 0), "IMPACT", "svm above impact threshold");
      } else if (now - fsmSince > T_FREEFALL_WINDOW) {
        fsmSet(NORMAL, led.Color(0, 0, 0), "NORMAL", "freefall window expired, no impact");
      }
      break;

    case IMPACT:
      if (svm < SVM_REST_MIN || svm > SVM_REST_MAX) inactivityStart = now;  // still moving

      if (now - inactivityStart > T_INACTIVITY_HOLD) {
        fsmSet(CONFIRMED, led.Color(255, 0, 0), "CONFIRMED", "motionless; cancel window open");
      } else if (now - fsmSince > T_IMPACT_WINDOW) {
        fsmSet(NORMAL, led.Color(0, 0, 0), "NORMAL", "still moving after impact, no fall");
      }
      break;

    case CONFIRMED:
      digitalWrite(BUZZER_PIN, ((now / BEEP_MS) & 1) ? HIGH : LOW);  // beep during countdown
      if (now - fsmSince > T_CANCEL_WINDOW) {
        alertSent = false;
        fsmSet(ALERTED, led.Color(255, 0, 0), "ALERTED", "cancel window expired");
        digitalWrite(BUZZER_PIN, HIGH);
      }
      break;

    case ALERTED:
      if (!alertSent) {                       // retry until a phone is connected
        alertSent = bleNotify("FALL_ALERT");
        if (alertSent) alertSentMs = now;
      }
      if ((alertSent && now - alertSentMs > T_ALERT_HOLD) || now - fsmSince > T_ALERT_MAX) {
        if (!alertSent) Serial.println("[ERR] alert not delivered");
        digitalWrite(BUZZER_PIN, LOW);
        fsmSet(NORMAL, led.Color(0, 0, 0), "NORMAL", "alert complete, reset");
      }
      break;
  }
}

// [FSM] cancel button: abort countdown or silence alert
void fsmCancel() {
  if (fsmState != CONFIRMED && fsmState != ALERTED) return;
  digitalWrite(BUZZER_PIN, LOW);
  if (fsmState == ALERTED && alertSent) bleNotify("FALL_CANCELLED");
  fsmSet(NORMAL, led.Color(0, 0, 0), "NORMAL", "cancelled by button");
}

// [MPU] fixed-rate sample, log, feed FSM
void mpuStep() {
  unsigned long now = millis();
  if (now - lastMpuMs < MPU_PERIOD_MS) return;
  lastMpuMs = now;

  if (mpu.accelUpdate() != 0) {
    static unsigned long lastErrMs = 0;
    if (now - lastErrMs > 1000) {
      Serial.println("[ERR] MPU read failed");
      lastErrMs = now;
    }
    return;
  }


  float ax  = mpu.accelX() - ACC_OFF_X;
  float ay  = mpu.accelY() - ACC_OFF_Y;
  float az  = mpu.accelZ() - ACC_OFF_Z;
  float svm = sqrtf(ax * ax + ay * ay + az * az);

#if LOG_MPU
  if (now - lastLogMs > LOG_INTERVAL_MS) {
    lastLogMs = now;
    Serial.printf("[MPU] ax=%.3f ay=%.3f az=%.3f svm=%.3f\n", ax, ay, az, svm);
  }
#endif

  fsmStep(svm, now);
}

void setup() {
  setCpuFrequencyMhz(80);   // [PWR] 80 MHz is plenty for this load
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);   // [TEMP] test RTC at 100 kHz

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(CANCEL_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  rtcOk = rtc.begin();
  if (!rtcOk) Serial.println("[ERR] DS3231 not found, continuing");

  #if HAS_OLED
    u8g2.begin();
    u8g2.setPowerSave(1);
  #endif

  mpu.setWire(&Wire);
  mpu.beginAccel();

  led.begin();
  led.show();
  bleBegin();

  // [SENS] idle until HR screen is opened
  sensorStop();

  Serial.println("[SYS] started");
}

void loop() {
  delay(1);                 // [PWR] yield so the core can idle between passes
  // [MPU] fall detection first; always runs, screen on or off
  mpuStep();

  // [IN] buttons
  if (buttonPressed(btnMenu))   menuPress();
  if (buttonPressed(btnCancel)) fsmCancel();

  // [STATE] timeouts
  unsigned long held = millis() - screenSince;
  if (screen == SCREEN_CLOCK && held >= CLOCK_TIMEOUT_MS) setScreen(SCREEN_OFF);
  if (screen == SCREEN_HR && (held >= HR_TIMEOUT_MS ||
                              (millis() - hrOpenedMs) >= HR_MAX_MS)) setScreen(SCREEN_OFF);

  // [SENS] HR sensor
  if (screen == SCREEN_HR) sensorPoll();

  // [UI] redraw; held off in the freefall window so the bus stays free for the impact
  if (screen == SCREEN_OFF || fsmState == FREEFALL) return;
  unsigned long period = (screen == SCREEN_CLOCK) ? CLOCK_REFRESH_MS : HR_REFRESH_MS;
  if (forceDraw || (millis() - lastDraw) >= period) {
    lastDraw  = millis();
    forceDraw = false;
    if (screen == SCREEN_CLOCK) updateClockScreen();
    else                        updateHrScreen();
  }
}
