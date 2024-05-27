// Define the motor control pins
#define IN1_PIN 4  // IN1 pin
#define IN2_PIN 5  // IN2 pin

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

  // Start Serial communication for debugging
  Serial.begin(115200);
  Serial.println("Motor Test Started");

  // Reverse sequence
  runMotor(false);

  // Stop the motor
  stopMotor();

  // Forward sequence
  runMotor(true);

  // Stop the motor
  stopMotor();
}

void loop() {
  // No operation in loop
}

// Function to run the motor
void runMotor(bool forward) {
  if (forward) {
    ledcDetachPin(IN2_PIN);  // Detach PWM from IN2_PIN
    ledcAttachPin(IN1_PIN, LEDC_CHANNEL);  // Attach PWM to IN1_PIN for forward
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    Serial.println("Direction: Forward");
  } else {
    ledcDetachPin(IN1_PIN);  // Detach PWM from IN1_PIN
    ledcAttachPin(IN2_PIN, LEDC_CHANNEL);  // Attach PWM to IN2_PIN for reverse
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    Serial.println("Direction: Reverse");
  }

  // Manually set speed values
  for (int speed = 0; speed <= 128; speed += 32) {
    ledcWrite(LEDC_CHANNEL, speed);
    Serial.print("PWM applied: ");
    Serial.println(speed);
    delay(1000); // Delay to observe behavior at each speed
  }
}

// Function to stop the motor
void stopMotor() {
  ledcWrite(LEDC_CHANNEL, 0);  // Set motor speed to 0
  delay(100); // Small delay to ensure the motor stops completely
  digitalWrite(IN1_PIN, LOW);  // Ensure both IN1 and IN2 are LOW to stop the motor
  digitalWrite(IN2_PIN, LOW);
  Serial.println("Motor stopped");
}
