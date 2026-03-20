#include <Wire.h>
#include "DFRobot_BloodOxygen_S.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// --- WiFi & MQTT Libraries ---
#include <WiFi.h>
#include <PubSubClient.h>

// --- BLE Libraries ---
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- Sensor Objects ---
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

// --- WiFi & MQTT Variables ---
const char* ssid = "Wokwi-GUEST"; // اسم شبكة الواي فاي
const char* password = "";        // باسوورد الواي فاي
const char* mqttServer = "test.mosquitto.org";
const char* clientID = "ESP32-Health-Wokwi"; // Client ID
const char* publishTopic = "HealthMonitor/SensorsData"; // التوبيك اللي هنبعت عليه القرايات

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMqttReconnectAttempt = 0;

// --- BLE Variables ---
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- BLE Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected via BLE!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected. Restarting advertising...");
      pServer->startAdvertising(); 
    }
};

// --- WiFi Setup Function ---
void setup_wifi() {
  delay(10);
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed initially. Will keep trying in background.");
  }
}

void setup() {
  Serial.begin(115200);

  // Setup WiFi & MQTT
  setup_wifi();
  client.setServer(mqttServer, 1883);

  // Start I2C for ESP32-C3
  Wire.begin(8, 9); 

  // Init ADXL345
  if(!accel.begin()) {
    Serial.println("ADXL345 not found!");
  } else {
    accel.setRange(ADXL345_RANGE_2_G);
  }

  // Init MAX30102 V2.0
  if (false == MAX30102.begin()) {
    Serial.println("MAX30102 Init Fail! Check connections.");
  } else {
    MAX30102.sensorStartCollect();
  }
  
  // --- BLE Setup ---
  BLEDevice::init("ESP32_Health_Monitor");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  
  BLEDevice::startAdvertising();

  Serial.println("System Initialized! Running...");
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. MQTT Non-Blocking Reconnect & Loop
  if (!client.connected()) {
    if (currentMillis - lastMqttReconnectAttempt > 5000) {
      lastMqttReconnectAttempt = currentMillis;
      if (WiFi.status() == WL_CONNECTED) { 
        Serial.print("Attempting MQTT connection...");
        if (client.connect(clientID)) {
          Serial.println("connected");
          // شيلنا الـ subscribe من هنا لأن مفيش أوامر هنستقبلها للـ LED
        } else {
          Serial.print("failed, rc=");
          Serial.println(client.state());
        }
      }
    }
  } else {
    client.loop(); 
  }

  // 2. Heart Rate & SPO2 (Updates every 4 seconds)
  static unsigned long lastOxyTime = 0;
  if (currentMillis - lastOxyTime > 4000) {
    MAX30102.getHeartbeatSPO2();
    currentSPO2 = MAX30102._sHeartbeatSPO2.SPO2;
    currentBPM = MAX30102._sHeartbeatSPO2.Heartbeat;
    
    lastOxyTime = currentMillis;
  }

  // 3. Accelerometer / Step Counting
  sensors_event_t event;
  accel.getEvent(&event);
  float magnitude = sqrt(sq(event.acceleration.x) + sq(event.acceleration.y) + sq(event.acceleration.z));

  total = total - readings[readIndex];
  readings[readIndex] = magnitude;
  total = total + readings[readIndex];
  readIndex = (readIndex + 1) % WINDOW_SIZE;
  float smoothed = total / WINDOW_SIZE;

  if (smoothed > threshold && !stepDetected && (currentMillis - lastStepTime > 300)) {
    stepCount++;
    stepDetected = true;
    lastStepTime = currentMillis;
    Serial.print("Steps: "); Serial.println(stepCount);
  }
  if (smoothed < threshold) stepDetected = false;

  // 4. Send Data via BLE (Every 1 second)
  static unsigned long lastBleTime = 0;
  if (deviceConnected && (currentMillis - lastBleTime > 1000)) {
    char bleString[50];
    snprintf(bleString, sizeof(bleString), "HR:%d | SpO2:%d%% | Steps:%d", currentBPM, currentSPO2, stepCount);
    pCharacteristic->setValue(bleString);
    pCharacteristic->notify();
    lastBleTime = currentMillis;
  }

  // 5. Send Data via MQTT to Node-RED (Every 5 seconds)
  static unsigned long lastMqttPublishTime = 0;
  if (client.connected() && (currentMillis - lastMqttPublishTime > 5000)) {
    // تنسيق الداتا: "نبض القلب,الأكسجين,الخطوات" 
    String mqttPayload = String(currentBPM) + "," + String(currentSPO2) + "," + String(stepCount);
    
    Serial.print("MQTT PUBLISH -> Topic: ");
    Serial.print(publishTopic);
    Serial.print(" | Data: ");
    Serial.println(mqttPayload);
    
    client.publish(publishTopic, mqttPayload.c_str());
    lastMqttPublishTime = currentMillis;
  }

  delay(20); 
}