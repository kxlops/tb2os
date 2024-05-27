#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "pins_arduino.h"
#include <TaskScheduler.h>

// Define the pins for the L9110S motor driver
const int motor1PWM = T4;  // PWM pin for Motor 1 speed control
const int motor1Dir = T5;  // Direction pin for Motor 1
const int motor2PWM = T6;  // PWM pin for Motor 2 speed control
const int motor2Dir = T7;  // Direction pin for Motor 2

// Define the pins for the buttons
const int button1Pin = T2; // Button 1
const int button2Pin = T3; // Button 2
const int button3Pin = T1; // Button 3

// PWM channel definitions
const int motor1PWM_channel = 0;
const int motor2PWM_channel = 1;

// PWM frequency and resolution
const int pwmFrequency = 5000;
const int pwmResolution = 8; // 8-bit resolution: 0-255

// Motor speed
const int mediumSpeed = 200; // Increase duty cycle for higher speed

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// TaskScheduler
Scheduler runner;

// Task declarations
void readButtons();
void updateMotorState();
void updateDisplay();

Task taskReadButtons(50, TASK_FOREVER, &readButtons);
Task taskUpdateMotorState(50, TASK_FOREVER, &updateMotorState);
Task taskUpdateDisplay(500, TASK_FOREVER, &updateDisplay);

enum MotorState {
  OFF,
  FORWARD,
  BACKWARD,
  STOPPING
};

MotorState motorState = OFF;
unsigned long motorStartTime = 0;
unsigned long motorDuration = 2000;
int currentSpeed = 0;
bool currentDirection = HIGH; // HIGH for forward, LOW for backward

// Button states and debounce variables
int lastButton1State = HIGH;
int lastButton2State = HIGH;
int lastButton3State = HIGH;
int button1State;
int button2State;
int button3State;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;
const unsigned long debounceDelay = 50;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize buttons
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);

  // Configure the PWM channels with the specified properties
  ledcSetup(motor1PWM_channel, pwmFrequency, pwmResolution);
  ledcSetup(motor2PWM_channel, pwmFrequency, pwmResolution);

  // Attach the PWM channels to the specified GPIO pins
  ledcAttachPin(motor1PWM, motor1PWM_channel);
  ledcAttachPin(motor2PWM, motor2PWM_channel);

  // Set the direction pins as output
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  // Initialize OLED display
  display.init();
  display.clear();
  display.display();

  // Show welcome message
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Welcome to tb2os");
  display.drawString(0, 10, "Use buttons to navigate");
  display.display();
  delay(2000);
  display.clear();

  Serial.println("Motor control started.");

  // Add tasks to the scheduler
  runner.addTask(taskReadButtons);
  runner.addTask(taskUpdateMotorState);
  runner.addTask(taskUpdateDisplay);

  // Enable tasks
  taskReadButtons.enable();
  taskUpdateMotorState.enable();
  taskUpdateDisplay.enable();
}

void stopMotors() {
  ledcWrite(motor1PWM_channel, 0);
  ledcWrite(motor2PWM_channel, 0);
  currentSpeed = 0;
  Serial.println("Motors stopped.");
}

void easeMotorsToStop(unsigned long startTime, unsigned long duration, bool direction) {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;
  if (elapsedTime < duration) {
    float progress = (float)elapsedTime / duration;
    int value = currentSpeed - (progress * currentSpeed);
    ledcWrite(motor1PWM_channel, value);
    ledcWrite(motor2PWM_channel, value);
    Serial.print("Easing Motor PWM Value: ");
    Serial.println(value);
  } else {
    stopMotors();
    motorState = OFF;
  }

  // Maintain the motor direction during stopping
  digitalWrite(motor1Dir, direction); 
  digitalWrite(motor2Dir, direction);
}

void readButtons() {
  // Read the state of the buttons
  int reading1 = digitalRead(button1Pin);
  int reading2 = digitalRead(button2Pin);
  int reading3 = digitalRead(button3Pin);

  // Check for button state changes and debounce
  if (reading1 != lastButton1State) {
    lastDebounceTime1 = millis();
  }
  if (reading2 != lastButton2State) {
    lastDebounceTime2 = millis();
  }
  if (reading3 != lastButton3State) {
    lastDebounceTime3 = millis();
  }

  if ((millis() - lastDebounceTime1) > debounceDelay) {
    if (reading1 != button1State) {
      button1State = reading1;
      if (button1State == LOW) {
        Serial.println("Button 1: Pressed");
        display.clear();
        display.drawString(0, 0, "Forward selected");
        display.display();
        motorState = FORWARD;
        // Set motor directions
        digitalWrite(motor1Dir, HIGH); // Set Motor 1 direction forward
        digitalWrite(motor2Dir, HIGH); // Set Motor 2 direction forward
        ledcWrite(motor1PWM_channel, mediumSpeed);
        ledcWrite(motor2PWM_channel, mediumSpeed);
        currentSpeed = mediumSpeed;
        currentDirection = HIGH;
      }
    }
  }

  if ((millis() - lastDebounceTime2) > debounceDelay) {
    if (reading2 != button2State) {
      button2State = reading2;
      if (button2State == LOW) {
        Serial.println("Button 2: Pressed");
        display.clear();
        display.drawString(0, 0, "Backward selected");
        display.display();
        motorState = BACKWARD;
        // Set motor directions
        digitalWrite(motor1Dir, LOW);  // Set Motor 1 direction backward
        digitalWrite(motor2Dir, LOW);  // Set Motor 2 direction backward
        ledcWrite(motor1PWM_channel, mediumSpeed);
        ledcWrite(motor2PWM_channel, mediumSpeed);
        currentSpeed = mediumSpeed;
        currentDirection = LOW;
      }
    }
  }

  if ((millis() - lastDebounceTime3) > debounceDelay) {
    if (reading3 != button3State) {
      button3State = reading3;
      if (button3State == LOW) {
        Serial.println("Button 3: Pressed");
        display.clear();
        display.drawString(0, 0, "Stopping motors");
        display.display();
        motorState = STOPPING;
        motorStartTime = millis();
      }
    }
  }

  // Save the reading for the next loop
  lastButton1State = reading1;
  lastButton2State = reading2;
  lastButton3State = reading3;
}

void updateMotorState() {
  // State machine for motor control
  switch (motorState) {
    case OFF:
      stopMotors();
      break;
    case FORWARD:
      // Motors running forward at medium speed
      break;
    case BACKWARD:
      // Motors running backward at medium speed
      break;
    case STOPPING:
      easeMotorsToStop(motorStartTime, motorDuration, currentDirection);
      break;
  }
}

void updateDisplay() {
  int motor1Direction = digitalRead(motor1Dir);
  int motor2Direction = digitalRead(motor2Dir);
  int motor1PWMValue = ledcRead(motor1PWM_channel);
  int motor2PWMValue = ledcRead(motor2PWM_channel);

  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Motor States:");
  display.drawString(0, 10, String("Motor 1 - Dir: ") + (motor1Direction ? "Forward" : "Backward"));
  display.drawString(0, 20, String("PWM: ") + motor1PWMValue);
  display.drawString(0, 30, String("Motor 2 - Dir: ") + (motor2Direction ? "Forward" : "Backward"));
  display.drawString(0, 40, String("PWM: ") + motor2PWMValue);
  display.display();

  Serial.print("Motor 1 - Direction: ");
  Serial.print(motor1Direction ? "Forward" : "Backward");
  Serial.print(" | PWM: ");
  Serial.println(motor1PWMValue);

  Serial.print("Motor 2 - Direction: ");
  Serial.print(motor2Direction ? "Forward" : "Backward");
  Serial.print(" | PWM: ");
  Serial.println(motor2PWMValue);
}

void loop() {
  runner.execute();
}
