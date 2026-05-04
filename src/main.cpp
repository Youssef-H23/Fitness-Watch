#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting OLED I2C Test...");

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("I2C initialized on SDA 8 / SCL 9");

  if (!display.begin(0x3C, true)) {
    Serial.println("SH1106 allocation failed!");
    for (;;);
  }
  Serial.println("Display initialized successfully");
}

void loop() {
  static unsigned long counter = 0;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 10);
  display.println("OLED Test 1.3\"");
  display.setCursor(10, 30);
  display.println("ESP32-C3 Mini");
  display.setTextSize(2);
  display.setCursor(10, 50);
  display.print(counter);
  display.display();

  Serial.println(counter);
  counter++;
  delay(500);
}
