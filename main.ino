#include <Wire.h>
#include "DFRobot_BloodOxygen_S.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <WiFi.h>
#include <PubSubClient.h>
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

// Setting WIFI
const char* ssid = "Wokwi-GUEST"; // Setting your AP SSID
const char* password = ""; // Setting your AP PSK
const char* mqttServer = "test.mosquitto.org";
const char* clientID = "ESP32-wokwi"; // Client ID username+0001
const char* publishTopic = "HealthData"; // Publish topic

// Setting up WiFi and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMqttReconnectAttempt = 0;

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

// Parameters for using non-blocking delay
unsigned long previousMillis = 0;
const long interval = 5000;
String msgStr = "";      // MQTT message buffer

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


 // Subscribe callback
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  String data = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    data += (char)payload[i];
  }
  Serial.println();
  Serial.print("Message size: ");
  Serial.println(length);
  Serial.println();
  Serial.println("-----------------------");
  Serial.println(data);
}




void setup() {
  Serial.begin(115200);
  
  // Start I2C for ESP32-C3
  Wire.begin(8, 9); 
setup_wifi();
  client.setServer(mqttServer, 1883); // Setting MQTT server
  client.setCallback(callback); // Define function which will be called when a message is received.
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
  unsigned long currentMillis = millis(); 

  // --- 1. Non-Blocking MQTT Reconnect ---
  if (!client.connected()) { 
    if (currentMillis - lastMqttReconnectAttempt > 5000) { // بيحاول كل 5 ثواني من غير ما يوقف الكود
      lastMqttReconnectAttempt = currentMillis;
      Serial.print("Attempting MQTT connection...");
      if (client.connect(clientID)) {
        Serial.println("connected");
        // مش محتاجين نعمل subscribe هنا طالما مش هنستقبل حاجة
      } else {
        Serial.print("failed, rc=");
        Serial.println(client.state());
      }
    }
  } else {
    client.loop();
  }

  // --- 2. Heart Rate & SPO2 (Updates every 4 seconds) ---
  static unsigned long lastOxyTime = 0;
  if (currentMillis - lastOxyTime > 4000) {
    MAX30102.getHeartbeatSPO2();
    currentSPO2 = MAX30102._sHeartbeatSPO2.SPO2;
    currentBPM = MAX30102._sHeartbeatSPO2.Heartbeat;
    
    Serial.print("SPO2: "); Serial.print(currentSPO2); Serial.print("% ");
    Serial.print("BPM: "); Serial.println(currentBPM);
    lastOxyTime = currentMillis;
  }

  // --- 3. Accelerometer / Step Counting ---
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

  // --- 4. Send Data via BLE (Every 1 second) ---
  static unsigned long lastBleTime = 0;
  if (deviceConnected && (currentMillis - lastBleTime > 1000)) {
    char bleString[50]; 
    snprintf(bleString, sizeof(bleString), "HR:%d | SpO2:%d%% | Steps:%d", currentBPM, currentSPO2, stepCount);
    pCharacteristic->setValue(bleString);
    pCharacteristic->notify(); 
    lastBleTime = currentMillis;
  }

  // --- 5. Send Data via MQTT (Every 5 seconds) ---
  static unsigned long lastMqttTime = 0;
  if (client.connected() && (currentMillis - lastMqttTime > 5000)) {
    msgStr = String(currentBPM) + "," + String(currentSPO2) + "," + String(stepCount);
    
    Serial.print("PUBLISH DATA: ");
    Serial.println(msgStr);
    
    // سطر واحد بسيط بيبعت الداتا من غير وجع دماغ الـ char array
    client.publish(publishTopic, msgStr.c_str()); 
    
    lastMqttTime = currentMillis;
  }

  delay(20); // Small delay for stability
}