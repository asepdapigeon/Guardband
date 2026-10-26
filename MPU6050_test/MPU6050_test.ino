#include <Adafruit_NeoPixel.h>
#include <MPU9250_asukiaaa.h>
#include <Wire.h>

MPU9250_asukiaaa mpu;

#define LED_PIN 48
#define NUM_LEDS 1
#define BUZZER_PIN 6 // adjust to your actual buzzer pin

Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);

// Thresholds — tune these based on your testing
const float FREEFALL_THRESHOLD = 0.2;
const float IMPACT_THRESHOLD = 7.5;
const float INACTIVITY_LOW = 0.65;
const float INACTIVITY_HIGH = 1.35;

const unsigned long FREEFALL_TIMEOUT = 1000;   // ms to detect impact after free-fall
const unsigned long INACTIVITY_DURATION = 2000; // ms of stillness required to confirm fall

enum FallState { NORMAL, FREEFALL_DETECTED, IMPACT_DETECTED, FALL_CONFIRMED };
FallState state = NORMAL;

unsigned long stateTimestamp = 0;
unsigned long inactivityStart = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);
  mpu.setWire(&Wire);
  led.begin();
  led.show();
  mpu.beginAccel();
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  uint8_t result = mpu.accelUpdate();
  if (result != 0) {
    Serial.println("Accel update failed!");
    delay(10);
    return;
  }

  float ax = mpu.accelX();
  float ay = mpu.accelY();
  float az = mpu.accelZ();
  float mag = sqrt(ax * ax + ay * ay + az * az);

  unsigned long now = millis();

  switch (state) {

    case NORMAL:
      if (mag < FREEFALL_THRESHOLD) {
        state = FREEFALL_DETECTED;
        stateTimestamp = now;
        Serial.println(">>> Free-fall detected, watching for impact...");
        led.setPixelColor(0, led.Color(0, 255, 0)); // green = watching
        led.show();
      }
      break;

    case FREEFALL_DETECTED:
      if (mag > IMPACT_THRESHOLD) {
        led.setPixelColor(0, led.Color(0, 0, 0));
        state = IMPACT_DETECTED;
        stateTimestamp = now;
        inactivityStart = now;
        Serial.println(">>> Impact detected, watching for inactivity...");
        led.setPixelColor(0, led.Color(255, 255, 0)); // Yellow = impact
        led.show();
      } else if (now - stateTimestamp > FREEFALL_TIMEOUT) {
        // no impact happened in time, false alarm
        state = NORMAL;
        led.setPixelColor(0, led.Color(0, 0, 0));
        led.show();
      }
      break;

    case IMPACT_DETECTED:
      if (mag < INACTIVITY_LOW || mag > INACTIVITY_HIGH) {
        led.setPixelColor(0, led.Color(0, 0, 0));
        // still moving significantly, reset the inactivity timer
        inactivityStart = now;
        led.setPixelColor(0, led.Color(255, 64, 0)); // orange = impact
      }

      if (now - inactivityStart > INACTIVITY_DURATION) {
        state = FALL_CONFIRMED;
        Serial.println(">>> FALL CONFIRMED! Triggering alert...");
        led.setPixelColor(0, led.Color(255, 0, 0)); // red = confirmed
        led.show();
        digitalWrite(BUZZER_PIN, HIGH);
      }
      break;

    case FALL_CONFIRMED:
      // stay in this state until manually reset (e.g. button press)
      // for now, auto-reset after 5 seconds for testing purposes
      if (now - stateTimestamp > 5000) {
        digitalWrite(BUZZER_PIN, LOW);
        state = NORMAL;
        led.setPixelColor(0, led.Color(0, 0, 0));
        led.show();
        Serial.println("Reset to normal.");
      }
      break;
  }

  delay(10); // ~100Hz sampling
}