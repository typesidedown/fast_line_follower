#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>

// Button pin definitions
#define BUTTON_MODE_1_PIN 20    // Pin for button 1 (mode selection)
#define BUTTON_MODE_2_PIN 22    // Pin for button 2 (mode selection)

// Debounce configuration
#define BUTTON_DEBOUNCE_MS 50

// Button state tracking
struct ButtonState {
  bool button1Pressed;
  bool button2Pressed;
  unsigned long button1LastPressTime;
  unsigned long button2LastPressTime;
  bool button1Triggered;
  bool button2Triggered;
};

extern ButtonState buttonState;

void initButtons();
void updateButtonStates();
bool wasButton1Pressed();
bool wasButton2Pressed();

#endif // BUTTONS_H
