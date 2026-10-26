#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(8, 9);
  Wire.setClock(400000);

  if (!rtc.begin()) {
    Serial.println("Couldn't find DS3231!");
    while (1) delay(10);
  }

  // Format: DateTime(year, month, day, hour, minute, second)
  rtc.adjust(DateTime(2026, 10, 1, 20, 10, 0)); //current time

  Serial.println("Time set!");
}

void loop() {
  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print("  ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if (now.minute() < 10) Serial.print('0');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if (now.second() < 10) Serial.print('0');
  Serial.println(now.second(), DEC);

  delay(1000);
}