#include <Wire.h>
#include <max30102.h>
#include <algorithm_by_RF.h>
#include <max30102_settings.h>

// Your pins on ESP32-S3 SuperMini
#define SDA_PIN 8
#define SCL_PIN 9

// Buffer size from the RF library (usually 100)
#define BUFFER_SIZE 100

uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

int32_t heartRate;
int8_t  validHeartRate;
float   spo2;
int8_t  validSPO2;
float   ratio, correl;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("MAX30102 RF Algorithm - ESP32-S3");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Initialise the sensor (this library has its own init)
  maxim_max30102_reset();
  delay(100);
  
  uint8_t uch_dummy;
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy);  // clear interrupt

  /
/ Recommended settings for better signal
  maxim_max30102_init();
  setLED1PulseAmplitude(0x5F);   // Red – try this first
  setLED2PulseAmplitude(0x5F);   // IR  – same value
  
  Serial.println("Sensor ready. Place finger firmly and stay still...");
}

void loop() {
  // Collect BUFFER_SIZE samples by polling
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Wait until a new sample is available in the FIFO
    uint8_t uch_temp;
    do {
      maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
    } while ((uch_temp & 0x40) == 0);   // wait for data ready bit

    maxim_max30102_read_fifo(&redBuffer[i], &irBuffer[i]);
  }

  // Run the improved RF algorithm
  rf_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_SIZE,
    redBuffer,
    &spo2, &validSPO2,
    &heartRate, &validHeartRate,
    &ratio, &correl
  );

  // Print results
  Serial.print("HR=");
  Serial.print(heartRate);
  Serial.print(" valid=");
  Serial.print(validHeartRate);
  Serial.print("  SpO2=");
  Serial.print(spo2, 1);
  Serial.print(" valid=");
  Serial.print(validSPO2);
  Serial.print("  correl=");
  Serial.println(correl, 3);

  delay(50);
}