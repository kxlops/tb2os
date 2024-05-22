#include <Wire.h>
#include "SSD1306Wire.h"
#include "pins_arduino.h"
#include "frame_data.h" // Include the frame data

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED);  // ADDRESS, SDA, SCL

#define FRAME_DELAY 50  // Delay between frames in milliseconds

const int numFrames = sizeof(frames) / sizeof(frames[0]);

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

void displayReset(void) {
  // Send a reset
  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, HIGH);
  delay(10);  // Increase delay to ensure proper reset
  digitalWrite(RST_OLED, LOW);
  delay(10);  // Increase delay to ensure proper reset
  digitalWrite(RST_OLED, HIGH);
  delay(10);
}

void displayFrame(const unsigned char* frame) {
  display.clear();
  display.drawXbm(0, 0, 128, 64, frame);
  display.display();  // Ensure buffer is sent to display
  Serial.println("Frame displayed");
}

void showLoadingScreen() {
  displayFrame(frames[0]);  // Display the first frame
  delay(5000);  // Display for 5 seconds
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println();
  Serial.println("Starting setup...");

  // This turns on and resets the OLED on the Heltec boards
  Serial.println("Turning on Vext...");
  VextON();
  Serial.println("Vext turned on");

  Serial.println("Resetting display...");
  displayReset();
  Serial.println("Display reset");

  // Initialising the UI will init the display too.
  Serial.println("Initializing display...");
  display.init();
  if (display.init()) {
    Serial.println("Display initialized successfully");
  } else {
    Serial.println("Display initialization failed");
  }

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // Show the first frame for 5 seconds
  showLoadingScreen();
}

void loop() {
  for (int i = 0; i < numFrames; i++) {
    Serial.print("Displaying frame: ");
    Serial.println(i + 1);

    // Debug: Print the first few bytes of the frame data
    for (int j = 0; j < 64; j++) {
      Serial.print(frames[i][j], HEX);
      Serial.print(" ");
    }
    Serial.println();

    displayFrame(frames[i]);
    delay(FRAME_DELAY);
  }
}
