#include "oled.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Screen dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// If your OLED is connected to SDA2/SCL2 on Teensy, try using Wire1 (alternate I2C)
// If your core provides Wire2 instead, change the pointer to &Wire2.
TwoWire *oledWire = &Wire1;

// Create display object using the chosen TwoWire instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, oledWire, -1);

// Default pins for SDA2/SCL2 are board/platform dependent; if you need to
// explicitly set pins on the TwoWire instance, call oledWire->begin(sdaPin, sclPin)
// before display.begin().

void initOLED() {
  // initialize TwoWire for OLED (if needed you can call begin with pins)
  // Example (uncomment and set pins if you want explicit pins):
  // oledWire->begin(OLED_SDA_PIN, OLED_SCL_PIN);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("OLED initialized");
  display.display();
}

void oledPrintLine(int row, const char* msg) {
  display.setCursor(0, row * 8);
  display.println(msg);
  display.display();
}

void oledDisplayStatus(float yaw, float error, int leftSpeed, int rightSpeed) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Yaw: ");
  display.println(yaw);
  display.print("Err: ");
  display.println(error);
  display.print("L:");
  display.print(leftSpeed);
  display.print(" R:");
  display.println(rightSpeed);
  display.display();
}
