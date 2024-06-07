#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "pins_arduino.h"
#include <TaskScheduler.h>

// Define the motor control pins
#define MOTOR1_IN1_PIN T4
#define MOTOR1_IN2_PIN T5
#define MOTOR2_IN1_PIN T6
#define MOTOR2_IN2_PIN T7

// Define LEDC channel and frequency
#define MOTOR1_PWM_CHANNEL 0
#define MOTOR2_PWM_CHANNEL 1
#define LEDC_TIMER_BIT 8  // 8-bit timer resolution
#define LEDC_BASE_FREQ 5000  // 5 kHz PWM frequency

// Define the pins for the buttons
const int button1Pin = T1; // Button 1
const int button2Pin = T2; // Button 2
const int button3Pin = T3; // Button 3

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// TaskScheduler
Scheduler runner;

// Task declarations
void readButtons();
void updateMotorState();
void updateDisplay();
void runMotorSequence();
void stopMotorSequence();

Task taskReadButtons(50, TASK_FOREVER, &readButtons);
Task taskUpdateMotorState(50, TASK_FOREVER, &updateMotorState);
Task taskUpdateDisplay(500, TASK_FOREVER, &updateDisplay);
Task taskRunMotorSequence(50, TASK_FOREVER, &runMotorSequence);

enum MotorState {
  OFF,
  FORWARD,
  BACKWARD,
  STOPPING,
  EASING_IN,
  EASING_OUT
};

enum EasingType {
  NONE,
  EASE_IN,
  EASE_OUT
};

// Define motor sequence step structure
struct MotorSequenceStep {
  bool direction; // true for forward, false for reverse
  int speed;
  unsigned long duration; // duration of the step in milliseconds
  EasingType easing;
};

// Define motor sequences
MotorSequenceStep sequence1[] = {
  {false, 100, 1000, NONE},
  {true, 100, 1000, NONE},
  {true, 100, 10000, EASE_IN},    // Ease in forward at medium speed for 10 seconds
  {true, 100, 10000, NONE},       // Forward at medium speed for 10 seconds
  {true, 100, 30000, EASE_OUT},   // Ease out forward to stop in 30 seconds
  // {false, 128, 2000, EASE_IN},   // Ease in reverse at medium speed for 2 seconds
  // {false, 128, 2000, NONE},      // Reverse at medium speed for 2 seconds
  // {false, 0, 1000, EASE_OUT}     // Ease out reverse to stop in 1 second
};

// Sequence state variables
MotorSequenceStep* currentSequence = sequence1;
int currentStep = 0;
unsigned long stepStartTime = 0;
bool sequenceRunning = false;
String currentSequenceName = "Sequence 1";

MotorState motorState = OFF;
unsigned long motorStartTime = 0;
unsigned long motorDuration = 2000;
int currentSpeed = 0;
const int maxForwardSpeed = 200; // Maximum speed for forward direction
const int maxReverseSpeed = 60; // Maximum speed for reverse direction
bool currentDirection = HIGH; // HIGH for forward, LOW for backward

// Button states and debounce variables
int button1State;
int button2State;
int button3State;
int lastButton1State = HIGH;
int lastButton2State = HIGH;
int lastButton3State = HIGH;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;
const unsigned long debounceDelay = 50;

// Track last values to print only when changes occur
bool lastDirection = !currentDirection;
int lastSpeed = -1;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize buttons
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);

  // Configure the PWM channels with the specified properties
  ledcSetup(MOTOR1_PWM_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(MOTOR2_PWM_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);

  // Set the direction pins as output
  pinMode(MOTOR1_IN1_PIN, OUTPUT);
  pinMode(MOTOR1_IN2_PIN, OUTPUT);
  pinMode(MOTOR2_IN1_PIN, OUTPUT);
  pinMode(MOTOR2_IN2_PIN, OUTPUT);

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
  runner.addTask(taskRunMotorSequence);

  // Enable tasks
  taskReadButtons.enable();
  taskUpdateMotorState.enable();
  taskUpdateDisplay.enable();
}

void runMotor(bool forward, int speed) {
  if (forward) {
    setupMotorDirection(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, true);
    Serial.println("Direction: Forward");
    currentDirection = true;
  } else {
    setupMotorDirection(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, false);
    Serial.println("Direction: Reverse");
    currentDirection = false;
  }

  setMotorSpeed(speed);
}

void setupMotorDirection(int in1Pin, int in2Pin, int in3Pin, int in4Pin, bool forward) {
  if (forward) {
    ledcDetachPin(in2Pin);                      // Detach PWM from IN2_PIN
    ledcAttachPin(in1Pin, MOTOR1_PWM_CHANNEL);  // Attach PWM to IN1_PIN for forward
    ledcDetachPin(in4Pin);                      // Detach PWM from IN2_PIN
    ledcAttachPin(in3Pin, MOTOR2_PWM_CHANNEL);  // Attach PWM to IN1_PIN for forward
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW);
  } else {
    ledcDetachPin(in1Pin);                      // Detach PWM from IN1_PIN
    ledcAttachPin(in2Pin, MOTOR1_PWM_CHANNEL);  // Attach PWM to IN2_PIN for reverse
    ledcDetachPin(in3Pin);                      // Detach PWM from IN1_PIN
    ledcAttachPin(in4Pin, MOTOR2_PWM_CHANNEL);  // Attach PWM to IN2_PIN for reverse
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, HIGH);
  }
}

void setMotorSpeed(int speed) {
  ledcWrite(MOTOR1_PWM_CHANNEL, speed);
  ledcWrite(MOTOR2_PWM_CHANNEL, speed);
  Serial.print("PWM applied: ");
  Serial.println(speed);
}

void stopMotor() {
  setMotorSpeed(0);
  digitalWrite(MOTOR1_IN1_PIN, LOW);  // Ensure both IN1 and IN2 are LOW to stop the motor
  digitalWrite(MOTOR1_IN2_PIN, LOW);
  digitalWrite(MOTOR2_IN1_PIN, LOW);  // Ensure both IN1 and IN2 are LOW to stop the motor
  digitalWrite(MOTOR2_IN2_PIN, LOW);
  Serial.println("Motor stopped");
  currentSpeed = 0;
  lastSpeed = -1;
}

void easeMotorSpeed(unsigned long startTime, unsigned long duration, int startSpeed, int endSpeed) {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;
  if (elapsedTime < duration) {
    float progress = (float)elapsedTime / duration;
    int value = startSpeed + (progress * (endSpeed - startSpeed));
    setMotorSpeed(value);
    Serial.print("Easing Motor PWM Value: ");
    Serial.println(value);
  } else {
    setMotorSpeed(endSpeed);
  }
}

void readButtons() {
  // Read the state of the buttons
  int reading1 = digitalRead(button1Pin);
  int reading2 = digitalRead(button2Pin);
  int reading3 = digitalRead(button3Pin);

  // Check for button state changes and debounce
  if (reading1 != lastButton1State) lastDebounceTime1 = millis();
  if (reading2 != lastButton2State) lastDebounceTime2 = millis();
  if (reading3 != lastButton3State) lastDebounceTime3 = millis();

  if ((millis() - lastDebounceTime1) > debounceDelay) {
    if (reading1 != button1State) {
      button1State = reading1;
      if (button1State == LOW) {
        Serial.println("Button 1: Pressed");
        currentSequence = sequence1;         // Select sequence 1
        currentSequenceName = "Sequence 1";  // Update sequence name
        currentStep = 0;
        sequenceRunning = true;
        taskRunMotorSequence.enable();
        motorState = FORWARD;
        stepStartTime = millis();  // Initialize step start time
      }
    }
  }

  if ((millis() - lastDebounceTime2) > debounceDelay) {
    if (reading2 != button2State) {
      button2State = reading2;
      if (button2State == LOW) {
        Serial.println("Button 2: Pressed");
        // Additional sequences can be added here
        motorState = STOPPING;
        motorStartTime = millis();
      }
    }
  }

  if ((millis() - lastDebounceTime3) > debounceDelay) {
    if (reading3 != button3State) {
      button3State = reading3;
      if (button3State == LOW) {
        Serial.println("Button 3: Pressed");
        sequenceRunning = false;
        stopMotorSequence();
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
      stopMotor();
      break;
    case FORWARD:
      // Motors running forward at medium speed
      break;
    case BACKWARD:
      // Motors running backward at medium speed
      break;
    case STOPPING:
      easeMotorSpeed(motorStartTime, motorDuration, currentSpeed, 0);
      break;
    case EASING_IN:
    case EASING_OUT:
      // Easing logic is handled in runMotorSequence
      break;
  }
}

void runMotorSequence() {
  if (sequenceRunning) {
    if (currentStep < sizeof(sequence1) / sizeof(sequence1[0])) {
      MotorSequenceStep step = currentSequence[currentStep];
      Serial.print("Executing step ");
      Serial.print(currentStep);
      Serial.print(": Direction ");
      Serial.print(step.direction ? "Forward" : "Reverse");
      Serial.print(", Speed ");
      Serial.print(step.speed);
      Serial.print(", Duration ");
      Serial.print(step.duration);
      Serial.print(", Easing ");
      Serial.println(step.easing == EASE_IN ? "EASE_IN" : step.easing == EASE_OUT ? "EASE_OUT" : "NONE");

      if (step.easing == EASE_IN) {
        motorState = EASING_IN;
        easeMotorSpeed(stepStartTime, step.duration, 0, step.speed);
      } else if (step.easing == EASE_OUT) {
        motorState = EASING_OUT;
        easeMotorSpeed(stepStartTime, step.duration, step.speed, 0);
      } else {
        motorState = step.direction ? FORWARD : BACKWARD;
        runMotor(step.direction, step.speed);
      }

      if (millis() - stepStartTime >= step.duration) {
        currentStep++;
        stepStartTime = millis();
      }

      if (currentStep >= sizeof(sequence1) / sizeof(sequence1[0])) {
        sequenceRunning = false;
        stopMotorSequence();
      }
    }
  }
}

void stopMotorSequence() {
  stopMotor();
  motorState = OFF;  // Ensure the motor state is updated
  taskRunMotorSequence.disable();
}

void updateDisplay() {
  int motor1PWMValue = ledcRead(MOTOR1_PWM_CHANNEL);
  int motor2PWMValue = ledcRead(MOTOR2_PWM_CHANNEL);
  unsigned long timeRemaining = sequenceRunning ? max(0UL, stepStartTime + currentSequence[currentStep].duration - millis()) : 0;
  String direction = sequenceRunning ? (currentSequence[currentStep].direction ? "Fwd" : "Rev") : "N/A";
  int speed = sequenceRunning ? currentSequence[currentStep].speed : 0;
  unsigned long duration = sequenceRunning ? currentSequence[currentStep].duration : 0;
  String easing = sequenceRunning ? (currentSequence[currentStep].easing == EASE_IN ? "In" : currentSequence[currentStep].easing == EASE_OUT ? "Out" : "None") : "None";
  
  String stateString;
  switch (motorState) {
    case OFF: stateString = "OFF"; break;
    case FORWARD: stateString = "FORWARD"; break;
    case BACKWARD: stateString = "BACKWARD"; break;
    case STOPPING: stateString = "STOPPING"; break;
    case EASING_IN: stateString = "EASING_IN"; break;
    case EASING_OUT: stateString = "EASING_OUT"; break;
  }

  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Stp: " + String(currentStep) + " Sp: " + String(speed) + " Dr: " + direction + " E: " + easing);
  display.drawString(0, 10, "Dur: " + String(duration));
  display.drawString(0, 20, "M1 PWM: " + String(motor1PWMValue) + " Dir: " + (currentDirection ? "Fwd" : "Rev"));
  display.drawString(0, 30, "M2 PWM: " + String(motor2PWMValue) + " Dir: " + (currentDirection ? "Fwd" : "Rev"));
  display.drawString(0, 40, "Time Left: " + String(timeRemaining) + "ms");
  display.drawString(0, 50, "State: " + stateString);
  display.display();

  Serial.print("Motor 1 - Direction: ");
  Serial.print(currentDirection ? "Forward" : "Backward");
  Serial.print(" | PWM: ");
  Serial.println(motor1PWMValue);

  Serial.print("Motor 2 - Direction: ");
  Serial.print(currentDirection ? "Forward" : "Backward");
  Serial.print(" | PWM: ");
  Serial.println(motor2PWMValue);

  Serial.print("Current Step: ");
  Serial.println(currentStep);

  Serial.print("Step Direction: ");
  Serial.print(direction);
  Serial.print(" | Speed: ");
  Serial.print(speed);
  Serial.print(" | Duration: ");
  Serial.print(duration);
  Serial.print(" | Easing: ");
  Serial.println(easing);

  Serial.print("Time Remaining: ");
  Serial.println(timeRemaining);

  Serial.print("State: ");
  Serial.println(stateString);
}

void loop() {
  runner.execute();
}
