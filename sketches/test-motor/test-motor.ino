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

// Reduced PWM duty cycle for lower power input
const int reducedDutyCycle = 128; // 50% duty cycle

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

  Serial.println("PWM test for two motors started.");
}

void loop() {
  // Set Motor 1 forward, Motor 2 backward
  Serial.println("Motor 1 Forward, Motor 2 Backward");
  digitalWrite(motor1Dir, HIGH); // Set Motor 1 direction forward
  digitalWrite(motor2Dir, LOW);  // Set Motor 2 direction backward
  ledcWrite(motor1PWM_channel, reducedDutyCycle); // Reduced speed for Motor 1
  ledcWrite(motor2PWM_channel, reducedDutyCycle); // Reduced speed for Motor 2
  delay(2000);

  // Stop both motors
  Serial.println("Stop both motors");
  ledcWrite(motor1PWM_channel, 0);   // Stop Motor 1
  ledcWrite(motor2PWM_channel, 0);   // Stop Motor 2
  delay(1000);

  // Set Motor 1 backward, Motor 2 forward
  Serial.println("Motor 1 Backward, Motor 2 Forward");
  digitalWrite(motor1Dir, LOW);  // Set Motor 1 direction backward
  digitalWrite(motor2Dir, HIGH); // Set Motor 2 direction forward
  ledcWrite(motor1PWM_channel, reducedDutyCycle); // Reduced speed for Motor 1
  ledcWrite(motor2PWM_channel, reducedDutyCycle); // Reduced speed for Motor 2
  delay(2000);

  // Stop both motors
  Serial.println("Stop both motors");
  ledcWrite(motor1PWM_channel, 0);   // Stop Motor 1
  ledcWrite(motor2PWM_channel, 0);   // Stop Motor 2
  delay(1000);
}

