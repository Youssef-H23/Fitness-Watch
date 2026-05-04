#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "secrets.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char *mqttBroker = "192.168.1.3";
char stateText[20] = "None";
unsigned long lastMqttReconnect = 0;

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

void getTimeString(char *buf, size_t len) {
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

  if (!display.begin(0x3C, true)) {
    Serial.println("SH1106 allocation failed!");
    for (;;);
  }
  Serial.println("Display initialized");

  Serial.printf("Connecting to WiFi %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected, IP: %s\n", WiFi.localIP().toString().c_str());

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

  char timeBuf[12];
  getTimeString(timeBuf, sizeof(timeBuf));

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(0, 0);
  display.println(timeBuf);
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SH110X_WHITE);

  display.setCursor(0, 14);
  display.print("Steps ");
  display.println(steps);

  display.setCursor(0, 24);
  display.print("Pulse ");
  display.print(pulseRate);
  display.println(" BPM");

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
