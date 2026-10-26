#include <Adafruit_NeoPixel.h>
#include <MPU9250_asukiaaa.h>
#include <Wire.h>

MPU9250_asukiaaa mpu(0x69);

#define LED_PIN     48
#define NUM_LEDS    1
#define BUZZER_PIN  6

Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);

// Detection thresholds (units: g)
const float SVM_FREEFALL_MAX   = 0.2;
const float SVM_IMPACT_MIN     = 7.5;
const float SVM_REST_MIN       = 0.85;
const float SVM_REST_MAX       = 1.15;

// Timing (ms)
const unsigned long T_FREEFALL_WINDOW  = 1000;
const unsigned long T_INACTIVITY_HOLD  = 5000;
const unsigned long T_ALERT_HOLD       = 5000;
const unsigned long T_LOG_INTERVAL     = 200;

enum FallState { NORMAL, FREEFALL, IMPACT, CONFIRMED };
FallState state = NORMAL;

unsigned long stateTimestamp   = 0;
unsigned long inactivityStart  = 0;
unsigned long lastLogTime      = 0;

void logData(float ax, float ay, float az, float svm) {
  Serial.print("MPU6500  ax=");   Serial.print(ax, 3);
  Serial.print("  ay=");          Serial.print(ay, 3);
  Serial.print("  az=");          Serial.print(az, 3);
  Serial.print("  svm=");         Serial.println(svm, 3);
}

void logTransition(const char* stateName, const char* condition) {
  Serial.print("FSM      state=");
  Serial.print(stateName);
  Serial.print("  cond=");
  Serial.println(condition);
}

void logAlert(const char* payload) {
  Serial.print("BLE      notify tx  payload=");
  Serial.println(payload);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(8, 9);
  Wire.setClock(400000);

  mpu.setWire(&Wire);
  mpu.beginAccel();     

  led.begin();
  led.show();
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.println("Guardband fall-detection module initialized.");
  Serial.print("Thresholds: freefall<");
  Serial.print(SVM_FREEFALL_MAX, 2);
  Serial.print("g  impact>");
  Serial.print(SVM_IMPACT_MIN, 2);
  Serial.println("g");
}

void loop() {
  if (mpu.accelUpdate() != 0) {
    Serial.println("MPU6500  error: read failed");
    delay(10);
    return;
  }

  float ax = mpu.accelX();
  float ay = mpu.accelY();
  float az = mpu.accelZ();
  float svm = sqrt(ax * ax + ay * ay + az * az);

  unsigned long now = millis();

  if (now - lastLogTime > T_LOG_INTERVAL) {
    logData(ax, ay, az, svm);
    lastLogTime = now;
  }

  switch (state) {
    case NORMAL:
      if (svm < SVM_FREEFALL_MAX) {
        state = FREEFALL;
        stateTimestamp = now;
        logTransition("FREEFALL", "svm below freefall threshold");
        led.setPixelColor(0, led.Color(255, 255, 0)); // yellow
        led.show();
      }
      break;

    case FREEFALL:
      if (svm > SVM_IMPACT_MIN) {
        state = IMPACT;
        stateTimestamp = now;
        inactivityStart = now;
        logTransition("IMPACT", "svm above impact threshold");
        led.setPixelColor(0, led.Color(255, 64, 0));  // orange
        led.show();
      } 
      else if (now - stateTimestamp > T_FREEFALL_WINDOW) {
        state = NORMAL;
        logTransition("NORMAL", "freefall window expired, no impact");
        led.setPixelColor(0, led.Color(0, 0, 0));
        led.show();
      }
      break;

    case IMPACT:
      if (svm < SVM_REST_MIN || svm > SVM_REST_MAX) {
        // i suppose i need to somehow reset the timer rn
        inactivityStart = now;   // still moving → reset timer
      }

      if (now - inactivityStart > T_INACTIVITY_HOLD) {
        state = CONFIRMED;
        stateTimestamp = now;
        logTransition("CONFIRMED", "motionless for inactivity hold period");
        led.setPixelColor(0, led.Color(255, 0, 0));   // red
        led.show();
        digitalWrite(BUZZER_PIN, HIGH);
        logAlert("FALL_ALERT");
      }
      break;

    case CONFIRMED:
      if (now - stateTimestamp > T_ALERT_HOLD) {
        digitalWrite(BUZZER_PIN, LOW);
        state = NORMAL;
        logTransition("NORMAL", "alert hold expired, reset");
        led.setPixelColor(0, led.Color(0, 0, 0));
        led.show();
      }
      break;
  }

  delay(10);
}