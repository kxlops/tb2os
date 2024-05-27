#include <Arduino.h>

// Define the pins for the buttons
const int button1Pin = T2; // Button 1
const int button2Pin = T3; // Button 2

// Debounce settings
unsigned long lastDebounceTime = 0; 
unsigned long debounceDelay = 50; 

int lastButton1State = HIGH;
int lastButton2State = HIGH;
int button1State;
int button2State;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize buttons
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);

  Serial.println("Button test started.");
}

void loop() {
  // Read the state of the buttons
  int reading1 = digitalRead(button1Pin);
  int reading2 = digitalRead(button2Pin);

  // Check for button state changes and debounce
  if (reading1 != lastButton1State) {
    lastDebounceTime = millis();
  }

  if (reading2 != lastButton2State) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Update the button state if the reading has been stable for the debounce delay
    if (reading1 != button1State) {
      button1State = reading1;
      if (button1State == LOW) {
        Serial.println("Button 1: Pressed");
      } else {
        Serial.println("Button 1: Released");
      }
    }
    if (reading2 != button2State) {
      button2State = reading2;
      if (button2State == LOW) {
        Serial.println("Button 2: Pressed");
      } else {
        Serial.println("Button 2: Released");
      }
    }
  }

  // Save the reading for the next loop
  lastButton1State = reading1;
  lastButton2State = reading2;

  // Small delay to make the serial output readable
  delay(50);
}
