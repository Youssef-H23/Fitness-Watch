#include <Wire.h>
#include "DFRobot_BloodOxygen_S.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// --- BLE Libraries ---
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Sensor Objects
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire, 0x57);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// --- Accelerometer Variables ---
#define WINDOW_SIZE 15
float readings[WINDOW_SIZE];
int readIndex = 0, stepCount = 0;
float total = 0, threshold = 12.0; 
bool stepDetected = false;
unsigned long lastStepTime = 0;

// --- Health Data Variables ---
int currentSPO2 = 0;
int currentBPM = 0;

// --- BLE Variables ---
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

// See the following for generating UUIDs: https://www.uuidgenerator.net/
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// BLE Connection Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected via BLE!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected. Restarting advertising...");
      pServer->startAdvertising(); // Restart advertising so you can reconnect
    }
};

void setup() {
  Serial.begin(115200);
  
  // Start I2C for ESP32-C3
  Wire.begin(8, 9); 

  // Init ADXL345
  if(!accel.begin()) {
    Serial.println("ADXL345 not found!");
    while(1);
  }
  accel.setRange(ADXL345_RANGE_2_G);

  // Init MAX30102 V2.0
  while (false == MAX30102.begin()) {
    Serial.println("MAX30102 Init Fail! Check RST pin is at 3.3V.");
    delay(1000);
  }
  
  // --- BLE Setup ---
  BLEDevice::init("ESP32_Health_Monitor"); // Name seen in nRF Connect
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create Characteristic with Read and Notify properties
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Add Descriptor to allow Notifications
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  
  BLEDevice::startAdvertising();

  Serial.println("System Initialized! Waiting for BLE connection...");
  MAX30102.sensorStartCollect();
}

void loop() {
  // 1. Heart Rate & SPO2 (Updates every 4 seconds)
  static unsigned long lastOxyTime = 0;
  if (millis() - lastOxyTime > 4000) {
    MAX30102.getHeartbeatSPO2();
    currentSPO2 = MAX30102._sHeartbeatSPO2.SPO2;
    currentBPM = MAX30102._sHeartbeatSPO2.Heartbeat;
    
    Serial.print("SPO2: "); Serial.print(currentSPO2); Serial.print("% ");
    Serial.print("BPM: "); Serial.println(currentBPM);
    lastOxyTime = millis();
  }

  // 2. Accelerometer / Step Counting
  sensors_event_t event;
  accel.getEvent(&event);
  float magnitude = sqrt(sq(event.acceleration.x) + sq(event.acceleration.y) + sq(event.acceleration.z));

  total = total - readings[readIndex];
  readings[readIndex] = magnitude;
  total = total + readings[readIndex];
  readIndex = (readIndex + 1) % WINDOW_SIZE;
  float smoothed = total / WINDOW_SIZE;

  if (smoothed > threshold && !stepDetected && (millis() - lastStepTime > 300)) {
    stepCount++;
    stepDetected = true;
    lastStepTime = millis();
    Serial.print("Steps: "); Serial.println(stepCount);
  }
  if (smoothed < threshold) stepDetected = false;

  // 3. Send Data via BLE
  static unsigned long lastBleTime = 0;
  // Send data every 1 second, but only if a device is connected
  if (deviceConnected && (millis() - lastBleTime > 1000)) {
    char bleString[50]; // Buffer to hold our formatted string
    
    // Format: "HR: 75 | SpO2: 98% | Steps: 10"
    snprintf(bleString, sizeof(bleString), "HR:%d | SpO2:%d%% | Steps:%d", currentBPM, currentSPO2, stepCount);
    
    pCharacteristic->setValue(bleString);
    pCharacteristic->notify(); // Push the update to nRF Connect
    
    lastBleTime = millis();
  }

  delay(20); // Small delay for stability
}