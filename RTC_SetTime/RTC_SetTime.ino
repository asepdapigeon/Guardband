#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(8, 9);
  Wire.setClock(100000);

  if (!rtc.begin()) {
    Serial.println("[ERR] RTC not found");
    while (1) delay(10);
  }

  rtc.adjust(DateTime(2026, 10, 9, 14, 15, 0));   // 06:05:00, 9 Oct 2026
  Serial.println("[RTC] time set");
}

void loop() {
  DateTime n = rtc.now();
  Serial.printf("[RTC] %04d-%02d-%02d %02d:%02d:%02d\n",
                n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second());
  delay(1000);
}