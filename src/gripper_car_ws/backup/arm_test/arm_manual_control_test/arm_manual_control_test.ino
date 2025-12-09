/*
 * Base Rotation Servo Continuous Control Test
 *
 * Hardware:
 * - Arduino Uno
 * - PCA9685 16-Channel PWM Servo Driver (I2C Address: 0x40)
 * - 1x Servo Motor on Channel 0 (Base rotation)
 *
 * Wiring:
 * - PCA9685 VCC → 5V (external power recommended for servos)
 * - PCA9685 GND → GND
 * - PCA9685 SDA → A4
 * - PCA9685 SCL → A5
 * - Servo connected to PCA9685 Channel 0
 *
 * Controls via Serial Monitor (115200 baud):
 * - Press 'w' (hold) : Rotate from 0° to 180° continuously
 * - Press 's' (hold) : Rotate from 180° to 0° continuously
 * - Press any other key to stop movement
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Base rotation servo on Channel 0
#define BASE_SERVO_CH 2

// Servo calibration for base rotation (from your existing setup)
#define MIN_PULSE 150
#define MAX_PULSE 600

// Current servo position (in degrees, 0-180)
int currentAngle = 0;

// Movement parameters
#define STEP_SIZE 10          // Degrees to move per step
#define STEP_DELAY 20        // Milliseconds between steps (slower = smoother)

// Movement state
bool movingForward = false;  // Moving 0 -> 180
bool movingBackward = false; // Moving 180 -> 0

void setup() {
  Serial.begin(115200);

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz for servos

  delay(100);

  // Initialize servo to 0 degrees
  currentAngle = 0;
  moveServo(0);

  delay(500);

  // Print welcome message
  Serial.println(F("\n========================================"));
  Serial.println(F("  Base Rotation Continuous Control"));
  Serial.println(F("========================================"));
  Serial.println(F("Servo initialized to 0 degrees"));
  Serial.println(F("\nControls:"));
  Serial.println(F("  w : Move forward (0° -> 180°)"));
  Serial.println(F("  s : Move backward (180° -> 0°)"));
  Serial.println(F("  Any other key : STOP"));
  Serial.println(F("========================================\n"));
  Serial.print(F("Current angle: "));
  Serial.print(currentAngle);
  Serial.println(F("°"));
}

void loop() {
  // Check for serial input to control movement
  if (Serial.available() > 0) {
    char key = Serial.read();

    // Clear any remaining characters in buffer
    while (Serial.available() > 0) {
      Serial.read();
    }

    if (key == 'w' || key == 'W') {
      // Move forward (0 -> 180)
      movingForward = true;
      movingBackward = false;
      Serial.println(F("Moving FORWARD (0° -> 180°)"));
    }
    else if (key == 's' || key == 'S') {
      // Move backward (180 -> 0)
      movingBackward = true;
      movingForward = false;
      Serial.println(F("Moving BACKWARD (180° -> 0°)"));
    }
    else {
      // Stop movement
      if (movingForward || movingBackward) {
        movingForward = false;
        movingBackward = false;
        Serial.print(F("STOPPED at "));
        Serial.print(currentAngle);
        Serial.println(F("°"));
      }
    }
  }

  // Handle continuous movement
  if (movingForward) {
    if (currentAngle < 180) {
      currentAngle += STEP_SIZE;
      if (currentAngle > 180) {
        currentAngle = 180;
      }
      moveServo(currentAngle);

      // Print progress every 10 degrees
      if (currentAngle % 10 == 0) {
        Serial.print(F("Angle: "));
        Serial.print(currentAngle);
        Serial.println(F("°"));
      }

      delay(STEP_DELAY);
    } else {
      // Reached 180 degrees
      Serial.println(F("Reached 180° - STOPPED"));
      movingForward = false;
    }
  }

  if (movingBackward) {
    if (currentAngle > 0) {
      currentAngle -= STEP_SIZE;
      if (currentAngle < 0) {
        currentAngle = 0;
      }
      moveServo(currentAngle);

      // Print progress every 10 degrees
      if (currentAngle % 10 == 0) {
        Serial.print(F("Angle: "));
        Serial.print(currentAngle);
        Serial.println(F("°"));
      }

      delay(STEP_DELAY);
    } else {
      // Reached 0 degrees
      Serial.println(F("Reached 0° - STOPPED"));
      movingBackward = false;
    }
  }
}

void moveServo(int angle) {
  // Constrain angle to 0-180
  angle = constrain(angle, 0, 180);

  // Convert angle to pulse value
  uint16_t pulse = map(angle, 0, 180, MIN_PULSE, MAX_PULSE);

  // Set PWM on channel 0
  pwm.setPWM(BASE_SERVO_CH, 0, pulse);

  // Update current position
  currentAngle = angle;
}
