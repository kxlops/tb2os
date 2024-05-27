#include <Arduino.h>

// Define the pins for the L9110S motor driver
const int motor1PWM = T4;  // PWM pin for Motor 1 speed control
const int motor1Dir = T5;  // Direction pin for Motor 1
const int motor2PWM = T6;  // PWM pin for Motor 2 speed control
const int motor2Dir = T7;  // Direction pin for Motor 2

// PWM channel definitions
const int motor1PWM_channel = 0;
const int motor2PWM_channel = 1;

// PWM frequency and resolution
const int pwmFrequency = 5000;
const int pwmResolution = 8; // 8-bit resolution: 0-255

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Configure the PWM channels with the specified properties
  ledcSetup(motor1PWM_channel, pwmFrequency, pwmResolution);
  ledcSetup(motor2PWM_channel, pwmFrequency, pwmResolution);

  // Attach the PWM channels to the specified GPIO pins
  ledcAttachPin(motor1PWM, motor1PWM_channel);
  ledcAttachPin(motor2PWM, motor2PWM_channel);

  // Set the direction pins as output
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  Serial.println("PWM test with easing functions started.");
}

void easeMotor(int pwmChannel, int startValue, int endValue, int duration) {
  int stepCount = 100;
  int stepDelay = duration / stepCount;
  float stepIncrement = (endValue - startValue) / (float)stepCount;

  for (int i = 0; i <= stepCount; i++) {
    int value = startValue + stepIncrement * i;
    ledcWrite(pwmChannel, value);
    delay(stepDelay);
  }
}

void loop() {
  // Set direction for both motors
  digitalWrite(motor1Dir, HIGH); // Motor 1 forward
  digitalWrite(motor2Dir, LOW);  // Motor 2 backward

  // Accelerate both motors smoothly
  Serial.println("Accelerate both motors");
  easeMotor(motor1PWM_channel, 0, 128, 2000); // 0 to 50% speed in 2 seconds
  easeMotor(motor2PWM_channel, 0, 128, 2000); // 0 to 50% speed in 2 seconds
  delay(2000);

  // Decelerate both motors smoothly
  Serial.println("Decelerate both motors");
  easeMotor(motor1PWM_channel, 128, 0, 2000); // 50% to 0 speed in 2 seconds
  easeMotor(motor2PWM_channel, 128, 0, 2000); // 50% to 0 speed in 2 seconds
  delay(2000);

  // Change direction
  digitalWrite(motor1Dir, LOW);  // Motor 1 backward
  digitalWrite(motor2Dir, HIGH); // Motor 2 forward

  // Accelerate both motors smoothly
  Serial.println("Accelerate both motors");
  easeMotor(motor1PWM_channel, 0, 128, 2000); // 0 to 50% speed in 2 seconds
  easeMotor(motor2PWM_channel, 0, 128, 2000); // 0 to 50% speed in 2 seconds
  delay(2000);

  // Decelerate both motors smoothly
  Serial.println("Decelerate both motors");
  easeMotor(motor1PWM_channel, 128, 0, 2000); // 50% to 0 speed in 2 seconds
  easeMotor(motor2PWM_channel, 128, 0, 2000); // 50% to 0 speed in 2 seconds
  delay(2000);
}
