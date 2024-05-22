#include <Wire.h>
#include "SSD1306Wire.h"
#include "pins_arduino.h"
#include "frame_data.h" // Include the frame data

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED);  // ADDRESS, SDA, SCL

const int numFrames = sizeof(frames) / sizeof(frames[0]);
const int frameDelay = 50;  // Fixed delay between frames in milliseconds

// Grid settings
int gridSpacing = 16;  // Default spacing for grid lines
int gridDepth = 2;     // Default depth factor for grid lines
int verticalRange = 4; // Default range for vertical lines
int horizontalRange = 2; // Default range for horizontal lines
bool debugLogs = false; // Default debug logging setting
bool dottedLines = false; // Default line type setting (false for solid, true for dotted)
int dottedLineGap = 4; // Default gap for dotted lines

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

void drawDottedLine(int x0, int y0, int x1, int y1, int gap) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = (dx > dy ? dx : -dy) / 2, e2;
  int gapCounter = 0;

  while (true) {
    if (gapCounter % gap == 0) display.setPixel(x0, y0);
    e2 = err;
    if (e2 > -dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
    if (x0 == x1 && y0 == y1) break;
    gapCounter++;
  }
}

void draw3DGrid() {
  int centerX = 64;  // Center of the screen (128/2)
  int centerY = 32;  // Center of the screen (64/2)

  // Draw vertical lines
  for (int i = -verticalRange; i <= verticalRange; i++) {
    int x = centerX + i * gridSpacing;
    if (dottedLines) {
      drawDottedLine(x, 0, centerX + i * (gridSpacing / gridDepth), centerY, dottedLineGap);
      drawDottedLine(centerX + i * (gridSpacing / gridDepth), centerY, x, 64, dottedLineGap);
    } else {
      display.drawLine(x, 0, centerX + i * (gridSpacing / gridDepth), centerY);
      display.drawLine(centerX + i * (gridSpacing / gridDepth), centerY, x, 64);
    }
  }

  // Draw horizontal lines
  for (int i = -horizontalRange; i <= horizontalRange; i++) {
    int y = centerY + i * gridSpacing;
    if (dottedLines) {
      drawDottedLine(0, y, centerX, centerY + i * (gridSpacing / gridDepth), dottedLineGap);
      drawDottedLine(128, y, centerX, centerY + i * (gridSpacing / gridDepth), dottedLineGap);
    } else {
      display.drawLine(0, y, centerX, centerY + i * (gridSpacing / gridDepth));
      display.drawLine(128, y, centerX, centerY + i * (gridSpacing / gridDepth));
    }
  }

  // Add lines at the borders to fill the gaps
  if (dottedLines) {
    drawDottedLine(0, 0, centerX, centerY - (centerY / gridDepth), dottedLineGap);
    drawDottedLine(128, 0, centerX, centerY - (centerY / gridDepth), dottedLineGap);
    drawDottedLine(0, 64, centerX, centerY + (centerY / gridDepth), dottedLineGap);
    drawDottedLine(128, 64, centerX, centerY + (centerY / gridDepth), dottedLineGap);
  } else {
    display.drawLine(0, 0, centerX, centerY - (centerY / gridDepth));
    display.drawLine(128, 0, centerX, centerY - (centerY / gridDepth));
    display.drawLine(0, 64, centerX, centerY + (centerY / gridDepth));
    display.drawLine(128, 64, centerX, centerY + (centerY / gridDepth));
  }
}

void displayFrame(const unsigned char* frame) {
  display.clear();
  display.drawXbm(0, 0, 128, 64, frame);
  draw3DGrid();
  display.display();  // Ensure buffer is sent to display

  if (debugLogs) {
    Serial.println("Frame displayed");
  }
}

void processSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove any leading or trailing whitespace

    if (input.length() > 1) {
      char command = input.charAt(0);
      int value = input.substring(1).toInt();

      switch (command) {
        case 'S':
          gridSpacing = value;
          Serial.println("Grid spacing set to: " + String(gridSpacing));
          break;
        case 'D':
          gridDepth = value;
          Serial.println("Grid depth set to: " + String(gridDepth));
          break;
        case 'V':
          verticalRange = value;
          Serial.println("Vertical range set to: " + String(verticalRange));
          break;
        case 'H':
          horizontalRange = value;
          Serial.println("Horizontal range set to: " + String(horizontalRange));
          break;
        case 'L':
          debugLogs = (value == 1);
          Serial.println("Debug logs " + String(debugLogs ? "enabled" : "disabled"));
          break;
        case 'T':
          dottedLines = (value == 1);
          Serial.println("Dotted lines " + String(dottedLines ? "enabled" : "disabled"));
          break;
        case 'G':
          dottedLineGap = value;
          Serial.println("Dotted line gap set to: " + String(dottedLineGap));
          break;
        default:
          Serial.println("Unknown command");
          break;
      }
    }
  }
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

  Serial.println("Enter 'S' followed by a number to set grid spacing (e.g., S16)");
  Serial.println("Enter 'D' followed by a number to set grid depth (e.g., D2)");
  Serial.println("Enter 'V' followed by a number to set vertical range (e.g., V4)");
  Serial.println("Enter 'H' followed by a number to set horizontal range (e.g., H2)");
  Serial.println("Enter 'L' followed by '1' to enable debug logs or '0' to disable (e.g., L1 or L0)");
  Serial.println("Enter 'T' followed by '1' for dotted lines or '0' for solid lines (e.g., T1 or T0)");
  Serial.println("Enter 'G' followed by a number to set dotted line gap (e.g., G4)");
}

void loop() {
  processSerialInput();

  for (int i = 0; i < numFrames; i++) {
    displayFrame(frames[i]);
    delay(frameDelay);
  }
}
