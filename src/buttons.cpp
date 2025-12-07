#include "buttons.h"
#include <Arduino.h>

ButtonState buttonState;

void initButtons() {
  pinMode(BUTTON_MODE_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MODE_2_PIN, INPUT_PULLUP);
  
  buttonState.button1Pressed = false;
  buttonState.button2Pressed = false;
  buttonState.button1LastPressTime = 0;
  buttonState.button2LastPressTime = 0;
  buttonState.button1Triggered = false;
  buttonState.button2Triggered = false;
  
  Serial.println("[BUTTONS] Initialized");
}

void updateButtonStates() {
  unsigned long now = millis();
  
  // Button 1 handling
  bool button1Current = (digitalRead(BUTTON_MODE_1_PIN) == LOW);  // Active low (pullup)
  
  if (button1Current && !buttonState.button1Pressed) {
    // Button pressed (transition from high to low)
    if (now - buttonState.button1LastPressTime > BUTTON_DEBOUNCE_MS) {
      buttonState.button1Pressed = true;
      buttonState.button1Triggered = true;
      buttonState.button1LastPressTime = now;
      Serial.println("[BUTTONS] Button 1 pressed");
    }
  } else if (!button1Current && buttonState.button1Pressed) {
    // Button released
    buttonState.button1Pressed = false;
  }
  
  // Button 2 handling
  bool button2Current = (digitalRead(BUTTON_MODE_2_PIN) == LOW);  // Active low (pullup)
  
  if (button2Current && !buttonState.button2Pressed) {
    // Button pressed (transition from high to low)
    if (now - buttonState.button2LastPressTime > BUTTON_DEBOUNCE_MS) {
      buttonState.button2Pressed = true;
      buttonState.button2Triggered = true;
      buttonState.button2LastPressTime = now;
      Serial.println("[BUTTONS] Button 2 pressed");
    }
  } else if (!button2Current && buttonState.button2Pressed) {
    // Button released
    buttonState.button2Pressed = false;
  }
}

bool wasButton1Pressed() {
  if (buttonState.button1Triggered) {
    buttonState.button1Triggered = false;
    return true;
  }
  return false;
}

bool wasButton2Pressed() {
  if (buttonState.button2Triggered) {
    buttonState.button2Triggered = false;
    return true;
  }
  return false;
}
