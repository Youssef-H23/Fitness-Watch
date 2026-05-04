#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

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
}

void loop() {
  static unsigned long steps = 0;
  static unsigned long lastPulseUpdate = 0;
  static int pulseRate = random(60, 101);
  static unsigned long lastStepUpdate = 0;

  if (millis() - lastPulseUpdate > 3000) {
    pulseRate = random(60, 101);
    lastPulseUpdate = millis();
  }

  if (millis() - lastStepUpdate > 1500) {
    steps++;
    lastStepUpdate = millis();
  }

  char timeBuf[12];
  getTimeString(timeBuf, sizeof(timeBuf));

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(0, 0);
  display.println(timeBuf);
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SH110X_WHITE);

  display.setCursor(0, 16);
  display.println("Steps");
  display.setTextSize(2);
  display.setCursor(0, 26);
  display.println(steps);

  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print("Pulse ");
  display.print(pulseRate);
  display.println(" BPM");

  display.display();

  Serial.printf("Steps: %lu | Pulse: %d BPM | Time: %s\n", steps, pulseRate, timeBuf);
  delay(100);
}
