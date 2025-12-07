#include "oled.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "sensors.h"

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
  display.setCursor(0, 0);
  
  // Raw IR array
  display.print("Raw:");
  for (int i = 0; i < 8; i++) {
    if (rawIrReadings[i] < 1000) display.print(" ");
    display.print(rawIrReadings[i]);
  }
  
  // Digital IR array (with spacing)
  display.setCursor(0, 18);
  display.print("Dig:");
  for (int i = 0; i < 8; i++) {
    display.print(irReadings[i]);
  }
  
  // Motor speeds
  display.setCursor(0, 32);
  display.print("L:");
  display.print(leftSpeed);
  display.print(" R:");
  display.println(rightSpeed);
  
  // Yaw and Error
  display.setCursor(0, 48);
  display.print("Yaw:");
  display.print((int)yaw);
  display.print(" Err:");
  display.print((int)error);
  
  display.display();
}

void oledDisplayIRReadings(uint16_t irReadings[8]) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Digital IR:");
  
  // Display digital readings as single row
  display.setCursor(0, 10);
  for (int i = 0; i < 8; i++) {
    display.print(irReadings[i]);
  }
  
  display.display();
}

void oledDisplayRawIRReadings(uint16_t rawIrReadings[8]) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Raw IR:");
  
  // Display raw values in compact format
  display.setCursor(0, 10);
  for (int i = 0; i < 8; i++) {
    display.print(rawIrReadings[i]);
    if (i < 7) display.print(" ");
  }
  
  display.display();
}

// ============================================================================
// MAZE MODE DISPLAY FUNCTIONS
// ============================================================================

void oledDisplayMazeIdle() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  display.println("MAZE SOLVER");
  display.println("===========");
  display.println();
  display.println("Button 1: DRY RUN");
  display.println("Button 2: FINAL RUN");
  display.println();
  display.println("Press button to start");
  
  display.display();
}

void oledDisplayDryRunStatus(int junctionCount, int dryRunStep, float distance) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  display.println("DRY RUN - EXPLORING");
  display.println("===================");
  display.print("Junctions: ");
  display.println(junctionCount);
  display.print("Step: ");
  display.println(dryRunStep);
  display.print("Distance: ");
  display.print((int)distance);
  display.println(" mm");
  display.println();
  display.println("Seeking endpoint...");
  
  display.display();
}

void oledDisplayFinalRunStatus(int currentStep, int totalSteps, float distance) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  display.println("FINAL RUN - OPTIMIZED");
  display.println("====================");
  display.print("Progress: ");
  display.print(currentStep);
  display.print("/");
  display.println(totalSteps);
  
  // Progress bar
  display.setCursor(0, 28);
  int barLength = (currentStep * 120) / totalSteps;
  display.print("[");
  for (int i = 0; i < barLength; i++) display.print("=");
  for (int i = barLength; i < 120; i++) display.print(" ");
  display.println("]");
  
  display.print("Distance: ");
  display.print((int)distance);
  display.println(" mm");
  
  display.display();
}

void oledDisplayMazeComplete(float dryRunDist, float finalRunDist) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  display.println("MAZE COMPLETE!");
  display.println("==============");
  display.print("Dry Run:  ");
  display.print((int)dryRunDist);
  display.println(" mm");
  display.print("Final Run: ");
  display.print((int)finalRunDist);
  display.println(" mm");
  display.println();
  
  float ratio = (finalRunDist > 0) ? (dryRunDist / finalRunDist) : 1.0;
  display.print("Efficiency: ");
  display.print(ratio);
  display.println("x");
  
  display.display();
}
