/*
 * Minimal Mobile Manipulator Firmware (Optimized)
 * Sends SERVO_POS ONLY when angles actually change.
 * Minimal Mobile Manipulator Firmware
 * Arduino + PCA9685 + ROS2 Serial Communication
 * ----------------------------------------------
 * FEATURES:
 * - Control 6 servos via PCA9685
 * - Receive commands from ROS2:
 *      SERVO,<index>,<angle>
 *      SET_ALL_SERVOS,<a1>,<a2>,<a3>,<a4>,<a5>,<a6>
 * - Smooth servo motion
 * - Publish joint positions (commanded positions) to ROS2:
 *      SERVO_POS,<6 values>
 * - Heartbeat every 1s
 *
 * NOTES:
 * - No physical feedback; positions = commanded angles
 * - No sensors; no digital/analog pins used
 
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca(0x40);

// PCA9685 Channels
const uint8_t SERVO_CH[6] = {0, 1, 2, 4, 5, 6};

// Timing
const float SERVO_FREQ_HZ = 50.0;
const uint16_t SERVO_MIN_US = 600;
const uint16_t SERVO_MAX_US = 2400;

const float STEP_DEG = 1.0;
const int STEP_DELAY_MS = 30;

// Joint state
float currentAngle[6] = {90, 90, 90, 90, 90, 90};
float targetAngle[6]  = {90, 90, 90, 90, 90, 90};

// Last published joint state
float lastSentAngle[6] = {90, 90, 90, 90, 90, 90};

String inputString = "";
bool stringComplete = false;
unsigned long lastHeartbeat = 0;

// Write servo
inline void goAngle(uint8_t ch, float angle) {
  angle = constrain(angle, 0, 180);
  float us = SERVO_MIN_US + (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  pca.writeMicroseconds(ch, (uint16_t)us);
}

// Smooth stepping
inline bool updateStep() {
  bool changed = false;

  for (int i = 0; i < 6; i++) {
    float before = currentAngle[i];

    if (currentAngle[i] < targetAngle[i]) {
      currentAngle[i] += STEP_DEG;
      if (currentAngle[i] > targetAngle[i]) currentAngle[i] = targetAngle[i];
    }
    else if (currentAngle[i] > targetAngle[i]) {
      currentAngle[i] -= STEP_DEG;
      if (currentAngle[i] < targetAngle[i]) currentAngle[i] = targetAngle[i];
    }

    // If servo moved, mark for SERVO_POS update
    if (currentAngle[i] != before) changed = true;

    goAngle(SERVO_CH[i], currentAngle[i]);
  }

  return changed;  // true means servo updated
}

// Send positions (only when changed)
void sendServoPositions() {
  Serial.print("SERVO_POS,");
  for (int i = 0; i < 6; i++) {
    Serial.print(currentAngle[i]);
    if (i < 5) Serial.print(",");
    lastSentAngle[i] = currentAngle[i];
  }
  Serial.println();
}

// Heartbeat
void sendHeartbeat() {
  Serial.print("HEARTBEAT ");
  Serial.println(millis());
}

// Process incoming commands
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  // SERVO,<index>,<angle>
  if (cmd.startsWith("SERVO,")) {
    int c1 = cmd.indexOf(',');
    int c2 = cmd.indexOf(',', c1 + 1);

    int idx = cmd.substring(c1 + 1, c2).toInt();
    float ang = cmd.substring(c2 + 1).toFloat();

    if (idx >= 0 && idx < 6)
      targetAngle[idx] = constrain(ang, 0, 180);

    Serial.println("OK,SERVO," + String(idx) + "," + String(ang));
  }

  // SET_ALL_SERVOS,<6 angles>
  else if (cmd.startsWith("SET_ALL_SERVOS,")) {
    String t = cmd.substring(cmd.indexOf(',') + 1);
    float vals[6];
    int index = 0, last = 0;

    for (int i = 0; i <= t.length(); i++) {
      if (i == t.length() || t.charAt(i) == ',') {
        vals[index++] = t.substring(last, i).toFloat();
        last = i + 1;
        if (index >= 6) break;
      }
    }

    if (index == 6) {
      for (int i = 0; i < 6; i++)
        targetAngle[i] = constrain(vals[i], 0, 180);

      Serial.println("OK,SET_ALL_SERVOS\n");
    }
  }

  else {
    Serial.println("ERROR,UNKNOWN_COMMAND\n");
  }
}

// Serial handler
void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') stringComplete = true;
    else inputString += c;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ_HZ);
  delay(500);
  Serial.println("ARM READY");
}

void loop() {
  // Parse incoming command
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Update servos smoothly
  bool moved = updateStep();

  // Only publish if any servo actually changed
  if (moved) {
    sendServoPositions();
  }

  // Heartbeat every 1 second
  if (millis() - lastHeartbeat > 1000) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }

  delay(STEP_DELAY_MS);
}
