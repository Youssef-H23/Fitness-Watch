#include <time.h>
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_ADXL345_U.h>
#include "DFRobot_MAX30102.h"
#include "secrets.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
DFRobot_MAX30102 MAX30102;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char *mqttBroker = "192.168.1.3";
char stateText[20] = "None";
unsigned long lastMqttReconnect = 0;
bool adxlReady = false;
bool hrReady = false;
int sensorHRValue = 0;

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.printf("MQTT [%s]: %s\n", topic, msg.c_str());
  strncpy(stateText, msg.c_str(), sizeof(stateText) - 1);
  stateText[sizeof(stateText) - 1] = '\0';
}

void tryReconnectMQTT() {
  if (mqttClient.connected()) return;
  Serial.print("Connecting to MQTT...");
  if (mqttClient.connect("ESP32-Fitness-Watch")) {
    Serial.println(" connected");
    mqttClient.subscribe("control/state");
  } else {
    Serial.printf(" failed, rc=%d\n", mqttClient.state());
  }
}

bool timeSynced = false;

void getTimeString(char *buf, size_t len) {
  if (timeSynced) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      int hours = timeinfo.tm_hour;
      const char *ampm = hours >= 12 ? "PM" : "AM";
      hours = hours % 12;
      if (hours == 0) hours = 12;
      snprintf(buf, len, "%d:%02d %s", hours, timeinfo.tm_min, ampm);
      return;
    }
  }
  unsigned long totalSeconds = (millis() / 1000) % (12 * 3600);
  int hours = totalSeconds / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  const char *ampm = (millis() / 1000) < (12 * 3600) ? "AM" : "PM";
  if (hours == 0) hours = 12;
  snprintf(buf, len, "%d:%02d %s", hours, minutes, ampm);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Fitness Watch...");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!display.begin(0x3C, true)) {
    Serial.println("SH1106 allocation failed!");
    for (;;);
  }
  Serial.println("Display initialized");

  Serial.print("Initializing ADXL345...");
  if (adxl.begin()) {
    adxl.setRange(ADXL345_RANGE_16_G);
    adxlReady = true;
    Serial.println(" ready");
  } else {
    Serial.println(" failed");
  }

  Serial.print("Initializing MAX30102...");
  if (MAX30102.begin()) {
    MAX30102.sensorConfiguration(0x1F, SAMPLEAVG_4, MODE_RED_IR, SAMPLERATE_400, PULSEWIDTH_411, ADCRANGE_4096);
    hrReady = true;
    Serial.println(" ready");
  } else {
    Serial.println(" failed");
  }

  Serial.printf("Connecting to WiFi %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected, IP: %s\n", WiFi.localIP().toString().c_str());

  Serial.print("Syncing time from NTP...");
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 10000)) {
    timeSynced = true;
    Serial.println(" synced");
  } else {
    Serial.println(" failed, using boot time");
  }

  mqttClient.setServer(mqttBroker, 1883);
  mqttClient.setCallback(mqttCallback);
  tryReconnectMQTT();
}

void loop() {
  static unsigned long steps = 0;
  static unsigned long lastPulseUpdate = 0;
  static int pulseRate = random(60, 101);
  static unsigned long lastStepUpdate = 0;
  static unsigned long lastMqttTest = 0;
  static int16_t ax = 0, ay = 0, az = 0;

  if (!mqttClient.connected() && millis() - lastMqttReconnect > 5000) {
    tryReconnectMQTT();
    lastMqttReconnect = millis();
  }
  mqttClient.loop();

  if (millis() - lastPulseUpdate > 3000) {
    pulseRate = random(60, 101);
    lastPulseUpdate = millis();
  }

  if (millis() - lastStepUpdate > 1500) {
    steps++;
    lastStepUpdate = millis();
  }

  if (mqttClient.connected() && millis() - lastMqttTest > 5000) {
    char payload[20];
    snprintf(payload, sizeof(payload), "%lu", random(0, 99999));
    mqttClient.publish("test/whatever", payload);
    Serial.printf("Published test/whatever: %s\n", payload);
    lastMqttTest = millis();
  }

  if (hrReady) {
    int32_t spo2 = 0;
    int8_t spo2Valid = 0;
    int32_t heartRate = 0;
    int8_t heartRateValid = 0;
    MAX30102.heartrateAndOxygenSaturation(&spo2, &spo2Valid, &heartRate, &heartRateValid);
    if (heartRateValid) {
      sensorHRValue = (int)heartRate;
    }
  }

  if (adxlReady) {
    sensors_event_t event;
    adxl.getEvent(&event);
    ax = (int16_t)(event.acceleration.x);
    ay = (int16_t)(event.acceleration.y);
    az = (int16_t)(event.acceleration.z);
  }

  char timeBuf[12];
  getTimeString(timeBuf, sizeof(timeBuf));

  char stepsLine[28];
  snprintf(stepsLine, sizeof(stepsLine), "Steps %lu|%d %d %d", steps, ax, ay, az);

  char pulseLine[26];
  if (hrReady && sensorHRValue > 0) {
    snprintf(pulseLine, sizeof(pulseLine), "Pulse %d BPM| %dbpm", pulseRate, sensorHRValue);
  } else {
    snprintf(pulseLine, sizeof(pulseLine), "Pulse %d BPM| --bpm", pulseRate);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(0, 0);
  display.println(timeBuf);
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SH110X_WHITE);

  display.setCursor(0, 14);
  display.println(stepsLine);

  display.setCursor(0, 24);
  display.println(pulseLine);

  display.setCursor(0, 34);
  display.print("State ");
  display.println(stateText);

  display.drawFastHLine(0, 44, SCREEN_WIDTH, SH110X_WHITE);
  display.setCursor(0, 48);
  if (WiFi.status() == WL_CONNECTED) {
    display.println("WiFi: Connected");
  } else {
    display.println("WiFi: Disconnected");
  }
  if (mqttClient.connected()) {
    display.println("MQTT: Connected");
  } else {
    display.println("MQTT: Disconnected");
  }

  display.display();
  delay(100);
}
