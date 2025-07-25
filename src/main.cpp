/*
 * Developed by Omar Florez
 * cv.omarflorez.me
 * 2025-07-25
 * 
 * ESP32 Pan-Tilt System with Joystick Control
 * Features: Laser pointer, smooth servo movement, auto-calibration
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// Pin definitions
#define JOYSTICK_VRX_PIN 32  // Analog pin for joystick X axis (ADC1_CH4)
#define JOYSTICK_VRY_PIN 33  // Analog pin for joystick Y axis (ADC1_CH5)
#define JOYSTICK_SW_PIN  25  // Digital pin for joystick button
#define SERVO_PAN_PIN    13  // Pin for horizontal servo (Pan)
#define SERVO_TILT_PIN   14  // Pin for vertical servo (Tilt)
#define LASER_PIN        26  // Pin for laser module

// Servo configuration
#define SERVO_MIN_ANGLE  0
#define SERVO_MAX_ANGLE  180
#define SERVO_CENTER_ANGLE 90

// Joystick configuration
#define JOYSTICK_DEADZONE 300  // Dead zone for better response
#define JOYSTICK_SMOOTHING 0.08  // Smoothing factor for fluid movement
#define JOYSTICK_MIN_CHANGE 0.5   // Minimum change threshold

// Servo inversion settings
#define INVERT_PAN  true   // true = invert horizontal servo (Pan)
#define INVERT_TILT false  // true = invert vertical servo (Tilt)

// Global variables
Servo servoPan;   // Horizontal servo
Servo servoTilt;  // Vertical servo

float currentPanAngle = SERVO_CENTER_ANGLE;
float currentTiltAngle = SERVO_CENTER_ANGLE;
float targetPanAngle = SERVO_CENTER_ANGLE;
float targetTiltAngle = SERVO_CENTER_ANGLE;

// Calibration variables
int joystickMinX = 204, joystickMaxX = 3891;
int joystickMinY = 204, joystickMaxY = 3891;
int joystickCenterX = 1922, joystickCenterY = 1718;
bool isCalibrated = true; // Pre-calibrated with optimized values

// Smoothing variables
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 20; // 20ms = 50Hz for smooth movement

// Position tracking variables
float lastValidPanAngle = SERVO_CENTER_ANGLE;
float lastValidTiltAngle = SERVO_CENTER_ANGLE;

void calibrateJoystick();
void updateServos();

void setup() {
  Serial.begin(115200);
  delay(2000); // Wait for serial monitor to be ready
  
  Serial.println("=== PAN-TILT SYSTEM WITH JOYSTICK ===");
  Serial.println("Initializing...");
  
  // Configure joystick pins
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  
  // Configure laser module
  pinMode(LASER_PIN, OUTPUT);
  
  // Configure servos
  servoPan.attach(SERVO_PAN_PIN);
  servoTilt.attach(SERVO_TILT_PIN);
  
  // Set initial servo positions
  servoPan.write(SERVO_CENTER_ANGLE);
  servoTilt.write(SERVO_CENTER_ANGLE);
  
  // Turn on laser
  digitalWrite(LASER_PIN, HIGH);
  
  delay(1000); // Wait for servos to position
  
  Serial.println("Servos configured at center position (90°)");
  Serial.println("Laser automatically turned on");
  Serial.println("System using optimized calibration values!");
  Serial.println("System ready to use!");
  Serial.println("Move joystick to control servos");
  Serial.println("Press button to recalibrate if needed");
  Serial.println("================================================");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check if button is pressed for recalibration
  if (digitalRead(JOYSTICK_SW_PIN) == LOW) {
    delay(50); // Debounce
    if (digitalRead(JOYSTICK_SW_PIN) == LOW) {
      Serial.println("Button pressed - Starting calibration...");
      calibrateJoystick();
    }
  }
  
  // Update servos at regular intervals for smooth movement
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    updateServos();
    lastUpdateTime = currentTime;
  }
}

void calibrateJoystick() {
  Serial.println("=== JOYSTICK CALIBRATION ===");
  Serial.println("Move joystick in all directions for 5 seconds");
  Serial.println("Make sure to reach all extremes");
  
  // Save current position before calibration
  float currentPanPos = currentPanAngle;
  float currentTiltPos = currentTiltAngle;
  
  int minX = 4095, maxX = 0;
  int minY = 4095, maxY = 0;
  int centerX = 0, centerY = 0;
  int sampleCount = 0;
  
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    int x = analogRead(JOYSTICK_VRX_PIN);
    int y = analogRead(JOYSTICK_VRY_PIN);
    
    minX = min(minX, x);
    maxX = max(maxX, x);
    minY = min(minY, y);
    maxY = max(maxY, y);
    
    centerX += x;
    centerY += y;
    sampleCount++;
    
    delay(10);
  }
  
  // Calculate average center
  centerX /= sampleCount;
  centerY /= sampleCount;
  
  // Apply optimized safety margins
  int marginX = (maxX - minX) * 0.05; // 5% margin
  int marginY = (maxY - minY) * 0.05;
  
  joystickMinX = minX + marginX;
  joystickMaxX = maxX - marginX;
  joystickMinY = minY + marginY;
  joystickMaxY = maxY - marginY;
  joystickCenterX = centerX;
  joystickCenterY = centerY;
  
  // Restore current position after calibration
  currentPanAngle = currentPanPos;
  currentTiltAngle = currentTiltPos;
  targetPanAngle = currentPanPos;
  targetTiltAngle = currentTiltPos;
  lastValidPanAngle = currentPanPos;
  lastValidTiltAngle = currentTiltPos;
  
  isCalibrated = true;
  
  Serial.println("=== CALIBRATION COMPLETED ===");
  Serial.printf("X Axis: %d - %d (Center: %d)\n", joystickMinX, joystickMaxX, joystickCenterX);
  Serial.printf("Y Axis: %d - %d (Center: %d)\n", joystickMinY, joystickMaxY, joystickCenterY);
  Serial.printf("Position restored: Pan %.1f°, Tilt %.1f°\n", currentPanPos, currentTiltPos);
  Serial.println("System ready to use!");
  Serial.println("================================================");
}

void updateServos() {
  if (!isCalibrated) {
    return;
  }
  
  // Read joystick values
  int joystickX = analogRead(JOYSTICK_VRX_PIN);
  int joystickY = analogRead(JOYSTICK_VRY_PIN);
  
  // Constrain values to calibrated range
  joystickX = constrain(joystickX, joystickMinX, joystickMaxX);
  joystickY = constrain(joystickY, joystickMinY, joystickMaxY);
  
  // Check if in dead zone
  bool inDeadzoneX = abs(joystickX - joystickCenterX) < JOYSTICK_DEADZONE;
  bool inDeadzoneY = abs(joystickY - joystickCenterY) < JOYSTICK_DEADZONE;
  
  // Force update if joystick is at extremes
  bool atExtremeX = (joystickX <= joystickMinX + 100) || (joystickX >= joystickMaxX - 100);
  bool atExtremeY = (joystickY <= joystickMinY + 100) || (joystickY >= joystickMaxY - 100);
  
  // Update target angles with improved logic
  if (!inDeadzoneX || atExtremeX) {
    // Map joystick to servo angles for Pan with balanced range
    float mappedPan = map(joystickX, joystickMinX, joystickMaxX, SERVO_MIN_ANGLE + 10, SERVO_MAX_ANGLE - 10);
    if (INVERT_PAN) {
      mappedPan = SERVO_MAX_ANGLE - 10 - (mappedPan - (SERVO_MIN_ANGLE + 10));
    }
    float newPanAngle = constrain(mappedPan, SERVO_MIN_ANGLE + 10, SERVO_MAX_ANGLE - 10);
    
    // Update if change is significant or at extreme
    if (abs(newPanAngle - lastValidPanAngle) > JOYSTICK_MIN_CHANGE || atExtremeX) {
      targetPanAngle = newPanAngle;
      lastValidPanAngle = newPanAngle;
    }
  }
  
  if (!inDeadzoneY || atExtremeY) {
    // Map joystick to servo angles for Tilt with balanced range
    float mappedTilt = map(joystickY, joystickMinY, joystickMaxY, SERVO_MIN_ANGLE + 10, SERVO_MAX_ANGLE - 10);
    if (INVERT_TILT) {
      mappedTilt = SERVO_MAX_ANGLE - 10 - (mappedTilt - (SERVO_MIN_ANGLE + 10));
    }
    float newTiltAngle = constrain(mappedTilt, SERVO_MIN_ANGLE + 10, SERVO_MAX_ANGLE - 10);
    
    // Update if change is significant or at extreme
    if (abs(newTiltAngle - lastValidTiltAngle) > JOYSTICK_MIN_CHANGE || atExtremeY) {
      targetTiltAngle = newTiltAngle;
      lastValidTiltAngle = newTiltAngle;
    }
  }
  
  // Apply smooth interpolation
  if (!inDeadzoneX || atExtremeX) {
    float panDiff = targetPanAngle - currentPanAngle;
    currentPanAngle += panDiff * JOYSTICK_SMOOTHING;
  }
  
  if (!inDeadzoneY || atExtremeY) {
    float tiltDiff = targetTiltAngle - currentTiltAngle;
    currentTiltAngle += tiltDiff * JOYSTICK_SMOOTHING;
  }
  
  // Constrain angles
  currentPanAngle = constrain(currentPanAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  currentTiltAngle = constrain(currentTiltAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  // Move servos
  servoPan.write(currentPanAngle);
  servoTilt.write(currentTiltAngle);
}