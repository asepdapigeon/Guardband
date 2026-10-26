#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define BUTTON_PIN 4

// Standard UUIDs - you can generate your own at uuidgenerator.net, but these work fine for testing
#define SERVICE_UUID        "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("App connected!");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("App disconnected.");
    pServer->startAdvertising(); // restart advertising so it can reconnect
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  BLEDevice::init("Guardband"); // this name will show up in your app's device scan
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902()); // required for notifications

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE advertising started. Waiting for connection...");
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) { // button pressed (active LOW with INPUT_PULLUP)
    Serial.println("Button pressed! Sending FALL_ALERT...");

    pCharacteristic->setValue("FALL_ALERT");
    pCharacteristic->notify();

    delay(1000); // simple debounce - prevent spamming while held
  }
}