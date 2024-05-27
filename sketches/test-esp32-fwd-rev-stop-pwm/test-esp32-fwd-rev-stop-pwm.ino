// Define the motor control pins
#define IN1_PIN 14  // IN1 pin
#define IN2_PIN 15  // IN2 pin

// Define LEDC channel and frequency
#define LEDC_CHANNEL 0
#define LEDC_TIMER_BIT 8  // 8-bit timer resolution
#define LEDC_BASE_FREQ 5000  // 5 kHz PWM frequency

void setup() {
  // Initialize motor control pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Configure LEDC PWM settings
  ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcAttachPin(IN1_PIN, LEDC_CHANNEL); // Attach PWM to IN1_PIN

  // Start Serial communication for debugging
  Serial.begin(115200);
  Serial.println("Motor Test Started");
}

void loop() {
  // Run motor forward
  setMotorDirection(true);
  runMotor(128);
  delay(5000);  // Run for 5 seconds

  // Stop motor
  stopMotor();
  delay(2000);  // Stop for 2 seconds

  // Run motor reverse
  setMotorDirection(false);
  runMotor(128);
  delay(5000);  // Run for 5 seconds

  // Stop motor
  stopMotor();
  delay(2000);  // Stop for 2 seconds
}

// Function to set motor direction
void setMotorDirection(bool forward) {
  if (forward) {
    digitalWrite(IN2_PIN, LOW);
    Serial.println("Direction: Forward");
  } else {
    digitalWrite(IN2_PIN, HIGH);
    Serial.println("Direction: Reverse");
  }
}

// Function to run motor at specified speed
void runMotor(int speed) {
  ledcWrite(LEDC_CHANNEL, speed);
  Serial.print("Speed: ");
  Serial.println(speed);
}

// Function to stop the motor
void stopMotor() {
  digitalWrite(IN1_PIN, LOW);  
  digitalWrite(IN2_PIN, LOW);
  ledcWrite(LEDC_CHANNEL, 0);
  Serial.println("Motor stopped");
}
