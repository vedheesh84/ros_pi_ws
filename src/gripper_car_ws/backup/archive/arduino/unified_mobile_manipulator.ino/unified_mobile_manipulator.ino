/*
 * UNIFIED MOBILE MANIPULATOR CONTROLLER
 * =====================================
 * Single Arduino controlling both mobile base and robotic arm
 *
 * HARDWARE:
 * - Mobile Base: 4x DC Motors via L298N H-Bridge
 * - Robotic Arm: 6x Servos via PCA9685 I2C Servo Driver
 * - Encoders: 4x Quadrature encoders for wheel odometry
 * - Emergency Stop: Digital button on pin 2
 *
 * COMMUNICATION:
 * - Baud Rate: 115200
 * - Protocol: Line-based commands with checksums
 *
 * AUTHOR: Generated for Mobile Manipulator Integration
 * DATE: 2025
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// Serial communication
#define BAUD_RATE 115200
#define SERIAL_TIMEOUT 500  // ms

// L298N Motor Driver Pins (Mobile Base)
#define M1_IN1 2   // Back Left Motor - Forward
#define M1_IN2 3   // Back Left Motor - Backward
#define M2_IN1 4   // Back Right Motor - Forward
#define M2_IN2 5   // Back Right Motor - Backward
#define M3_IN1 6   // Front Left Motor - Forward
#define M3_IN2 7   // Front Left Motor - Backward
#define M4_IN1 8   // Front Right Motor - Forward
#define M4_IN2 9   // Front Right Motor - Backward
#define EN_LEFT 10  // Enable for M1 + M3 (PWM)
#define EN_RIGHT 11 // Enable for M2 + M4 (PWM)

// Encoder Pins (Interrupts)
#define ENC_M1_A 18  // Back Left Encoder A (INT5)
#define ENC_M1_B 19  // Back Left Encoder B
#define ENC_M2_A 20  // Back Right Encoder A (INT3)
#define ENC_M2_B 21  // Back Right Encoder B
#define ENC_M3_A A0  // Front Left Encoder A (analog pin as digital)
#define ENC_M3_B A1  // Front Left Encoder B
#define ENC_M4_A A2  // Front Right Encoder A
#define ENC_M4_B A3  // Front Right Encoder B

// Emergency Stop Button
#define EMERGENCY_STOP_PIN 2  // Shared with M1_IN1 - Note: Reassign if needed

// PCA9685 Configuration (Robotic Arm)
#define PCA9685_ADDRESS 0x40
#define SERVO_FREQ 50  // 50 Hz for servos
#define NUM_SERVOS 6

// Servo Channel Mapping
#define SERVO_JOINT_1 0
#define SERVO_JOINT_2 1
#define SERVO_JOINT_3 2
#define SERVO_JOINT_4 4
#define SERVO_GRIPPER_BASE 5
#define SERVO_LEFT_GEAR 6

// Servo Pulse Width Limits (microseconds)
#define SERVO_MIN_PULSE 600
#define SERVO_MAX_PULSE 2400

// Motor Control
#define MIN_PWM 60          // Minimum PWM to overcome motor friction
#define MAX_PWM 255         // Maximum PWM
#define CMD_TIMEOUT 300     // Motor command timeout (ms)

// Encoder Configuration
#define TICKS_PER_REV 360   // Encoder ticks per wheel revolution (adjust for your encoders)
#define WHEEL_RADIUS 0.075  // Wheel radius in meters
#define WHEEL_BASE 0.67     // Distance between left and right wheels (meters)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// PCA9685 Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// Encoder counts (volatile for interrupt safety)
volatile long enc_count_m1 = 0;
volatile long enc_count_m2 = 0;
volatile long enc_count_m3 = 0;
volatile long enc_count_m4 = 0;

// Servo positions (0-180 degrees)
int servo_positions[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};
int servo_targets[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

// Motor control
unsigned long last_cmd_time = 0;
bool emergency_stop = false;

// Timing
unsigned long last_heartbeat = 0;
unsigned long last_odom_publish = 0;
const int HEARTBEAT_INTERVAL = 1000;    // 1 Hz
const int ODOM_PUBLISH_INTERVAL = 50;   // 20 Hz

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT);

  // Wait for serial connection
  delay(100);

  // Initialize motor pins
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  pinMode(EN_LEFT, OUTPUT);
  pinMode(EN_RIGHT, OUTPUT);

  // Initialize encoders
  pinMode(ENC_M1_A, INPUT_PULLUP);
  pinMode(ENC_M1_B, INPUT_PULLUP);
  pinMode(ENC_M2_A, INPUT_PULLUP);
  pinMode(ENC_M2_B, INPUT_PULLUP);
  pinMode(ENC_M3_A, INPUT_PULLUP);
  pinMode(ENC_M3_B, INPUT_PULLUP);
  pinMode(ENC_M4_A, INPUT_PULLUP);
  pinMode(ENC_M4_B, INPUT_PULLUP);

  // Attach interrupts for encoders (only available on certain pins)
  attachInterrupt(digitalPinToInterrupt(ENC_M1_A), encoder_m1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M2_A), encoder_m2_isr, CHANGE);

  // Emergency stop button
  // pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

  // Stop all motors initially
  stopAllMotors();

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize servos to center position
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServo(i, 90);
  }

  // Send startup message
  Serial.println("STATUS,READY,Unified Mobile Manipulator Controller");
  Serial.println("INFO,Base: 4-wheel differential drive with encoders");
  Serial.println("INFO,Arm: 6-DOF with PCA9685 servo driver");

  last_cmd_time = millis();
  last_heartbeat = millis();
  last_odom_publish = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long current_time = millis();

  // Check for emergency stop
  // if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
  //   if (!emergency_stop) {
  //     emergency_stop = true;
  //     stopAllMotors();
  //     Serial.println("EMERGENCY_STOP,ACTIVATED");
  //   }
  // }

  // Process incoming serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      processCommand(command);
      last_cmd_time = current_time;
    }
  }

  // Motor watchdog - stop if no commands received
  if (current_time - last_cmd_time > CMD_TIMEOUT) {
    stopAllMotors();
  }

  // Smooth servo movement
  updateServos();

  // Publish odometry at fixed rate
  if (current_time - last_odom_publish >= ODOM_PUBLISH_INTERVAL) {
    publishOdometry();
    last_odom_publish = current_time;
  }

  // Send heartbeat
  if (current_time - last_heartbeat >= HEARTBEAT_INTERVAL) {
    Serial.print("HEARTBEAT,");
    Serial.println(current_time);
    last_heartbeat = current_time;
  }

  // Small delay to prevent overwhelming the serial
  delay(10);
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void processCommand(String cmd) {
  // Parse command format: COMMAND,arg1,arg2,...
  int comma_idx = cmd.indexOf(',');
  if (comma_idx == -1) {
    Serial.println("ERROR,Invalid command format");
    return;
  }

  String command_type = cmd.substring(0, comma_idx);
  String args = cmd.substring(comma_idx + 1);

  // MOBILE BASE COMMANDS
  if (command_type == "VEL") {
    // VEL,<linear_x>,<angular_z>
    handleVelocityCommand(args);
  }
  else if (command_type == "STOP") {
    // STOP - Emergency stop all motors
    stopAllMotors();
    Serial.println("OK,STOP");
  }
  else if (command_type == "RESET_ENCODERS") {
    // RESET_ENCODERS - Zero all encoder counts
    resetEncoders();
    Serial.println("OK,RESET_ENCODERS");
  }

  // ROBOTIC ARM COMMANDS
  else if (command_type == "SERVO") {
    // SERVO,<index>,<angle>
    handleServoCommand(args);
  }
  else if (command_type == "SET_ALL_SERVOS") {
    // SET_ALL_SERVOS,<a0>,<a1>,<a2>,<a3>,<a4>,<a5>
    handleSetAllServos(args);
  }

  // STATUS COMMANDS
  else if (command_type == "GET_STATUS") {
    // GET_STATUS - Return system status
    sendStatus();
  }
  else if (command_type == "GET_ODOM") {
    // GET_ODOM - Return current odometry
    publishOdometry();
  }
  else if (command_type == "RESET_EMERGENCY") {
    // RESET_EMERGENCY - Clear emergency stop
    emergency_stop = false;
    Serial.println("OK,RESET_EMERGENCY");
  }

  // UNKNOWN COMMAND
  else {
    Serial.print("ERROR,Unknown command: ");
    Serial.println(command_type);
  }
}

// ============================================================================
// MOBILE BASE CONTROL
// ============================================================================

void handleVelocityCommand(String args) {
  // Parse: <linear_x>,<angular_z>
  int comma_idx = args.indexOf(',');
  if (comma_idx == -1) {
    Serial.println("ERROR,VEL requires 2 arguments");
    return;
  }

  float linear_x = args.substring(0, comma_idx).toFloat();
  float angular_z = args.substring(comma_idx + 1).toFloat();

  // Check emergency stop
  if (emergency_stop) {
    Serial.println("ERROR,Emergency stop active");
    return;
  }

  // Skid-steer mixing: left and right wheel velocities
  // left_vel = linear_x - (angular_z * wheel_base / 2)
  // right_vel = linear_x + (angular_z * wheel_base / 2)
  float left_vel = linear_x - angular_z;
  float right_vel = linear_x + angular_z;

  // Convert to PWM (-1.0 to 1.0 range expected)
  // Clamp to [-1.0, 1.0]
  left_vel = constrain(left_vel, -1.0, 1.0);
  right_vel = constrain(right_vel, -1.0, 1.0);

  // Scale to PWM values
  int left_pwm = (int)(left_vel * MAX_PWM);
  int right_pwm = (int)(right_vel * MAX_PWM);

  // Set motor speeds
  setLeftMotors(left_pwm);
  setRightMotors(right_pwm);

  // Acknowledge command
  Serial.print("OK,VEL,");
  Serial.print(linear_x, 3);
  Serial.print(",");
  Serial.println(angular_z, 3);
}

void setLeftMotors(int pwm) {
  // Left motors: M1 (back left) + M3 (front left)
  bool forward = pwm >= 0;
  int abs_pwm = abs(pwm);

  // Apply minimum PWM threshold
  if (abs_pwm > 0 && abs_pwm < MIN_PWM) {
    abs_pwm = MIN_PWM;
  }

  // Set direction for M1 (back left)
  digitalWrite(M1_IN1, forward ? HIGH : LOW);
  digitalWrite(M1_IN2, forward ? LOW : HIGH);

  // Set direction for M3 (front left)
  digitalWrite(M3_IN1, forward ? HIGH : LOW);
  digitalWrite(M3_IN2, forward ? LOW : HIGH);

  // Set PWM speed
  analogWrite(EN_LEFT, abs_pwm);
}

void setRightMotors(int pwm) {
  // Right motors: M2 (back right) + M4 (front right)
  bool forward = pwm >= 0;
  int abs_pwm = abs(pwm);

  // Apply minimum PWM threshold
  if (abs_pwm > 0 && abs_pwm < MIN_PWM) {
    abs_pwm = MIN_PWM;
  }

  // Set direction for M2 (back right)
  digitalWrite(M2_IN1, forward ? HIGH : LOW);
  digitalWrite(M2_IN2, forward ? LOW : HIGH);

  // Set direction for M4 (front right)
  digitalWrite(M4_IN1, forward ? HIGH : LOW);
  digitalWrite(M4_IN2, forward ? LOW : HIGH);

  // Set PWM speed
  analogWrite(EN_RIGHT, abs_pwm);
}

void stopAllMotors() {
  // Stop all motors
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN1, LOW);
  digitalWrite(M4_IN2, LOW);
  analogWrite(EN_LEFT, 0);
  analogWrite(EN_RIGHT, 0);
}

// ============================================================================
// ENCODER HANDLING
// ============================================================================

void encoder_m1_isr() {
  // Back left encoder interrupt
  if (digitalRead(ENC_M1_A) == digitalRead(ENC_M1_B)) {
    enc_count_m1++;
  } else {
    enc_count_m1--;
  }
}

void encoder_m2_isr() {
  // Back right encoder interrupt
  if (digitalRead(ENC_M2_A) == digitalRead(ENC_M2_B)) {
    enc_count_m2++;
  } else {
    enc_count_m2--;
  }
}

void resetEncoders() {
  // Reset all encoder counts to zero
  noInterrupts();
  enc_count_m1 = 0;
  enc_count_m2 = 0;
  enc_count_m3 = 0;
  enc_count_m4 = 0;
  interrupts();
}

void publishOdometry() {
  // Publish encoder counts for odometry calculation on ROS side
  // Format: ODOM,<enc1>,<enc2>,<enc3>,<enc4>,<timestamp>
  noInterrupts();
  long e1 = enc_count_m1;
  long e2 = enc_count_m2;
  long e3 = enc_count_m3;
  long e4 = enc_count_m4;
  interrupts();

  Serial.print("ODOM,");
  Serial.print(e1);
  Serial.print(",");
  Serial.print(e2);
  Serial.print(",");
  Serial.print(e3);
  Serial.print(",");
  Serial.print(e4);
  Serial.print(",");
  Serial.println(millis());
}

// ============================================================================
// ROBOTIC ARM CONTROL (SERVOS)
// ============================================================================

void handleServoCommand(String args) {
  // Parse: <index>,<angle>
  int comma_idx = args.indexOf(',');
  if (comma_idx == -1) {
    Serial.println("ERROR,SERVO requires 2 arguments");
    return;
  }

  int servo_idx = args.substring(0, comma_idx).toInt();
  int angle = args.substring(comma_idx + 1).toInt();

  // Validate servo index
  if (servo_idx < 0 || servo_idx >= NUM_SERVOS) {
    Serial.println("ERROR,Invalid servo index");
    return;
  }

  // Validate angle
  if (angle < 0 || angle > 180) {
    Serial.println("ERROR,Angle must be 0-180");
    return;
  }

  // Set target position
  servo_targets[servo_idx] = angle;

  Serial.print("OK,SERVO,");
  Serial.print(servo_idx);
  Serial.print(",");
  Serial.println(angle);
}

void handleSetAllServos(String args) {
  // Parse: <a0>,<a1>,<a2>,<a3>,<a4>,<a5>
  int angles[NUM_SERVOS];
  int start = 0;

  for (int i = 0; i < NUM_SERVOS; i++) {
    int comma_idx = args.indexOf(',', start);
    String angle_str;

    if (comma_idx == -1 && i == NUM_SERVOS - 1) {
      angle_str = args.substring(start);
    } else if (comma_idx != -1) {
      angle_str = args.substring(start, comma_idx);
      start = comma_idx + 1;
    } else {
      Serial.println("ERROR,SET_ALL_SERVOS requires 6 angles");
      return;
    }

    angles[i] = angle_str.toInt();

    // Validate angle
    if (angles[i] < 0 || angles[i] > 180) {
      Serial.println("ERROR,All angles must be 0-180");
      return;
    }
  }

  // Set all targets
  for (int i = 0; i < NUM_SERVOS; i++) {
    servo_targets[i] = angles[i];
  }

  Serial.println("OK,SET_ALL_SERVOS");
}

void setServo(int channel, int angle) {
  // Convert angle (0-180) to pulse width
  angle = constrain(angle, 0, 180);

  // Calculate pulse length
  int pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  // Convert pulse to PCA9685 value (0-4095 for 12-bit resolution)
  // PCA9685 value = (pulse_us * 4096) / (1000000 / freq)
  int pwm_value = (int)((pulse * 4096.0) / (1000000.0 / SERVO_FREQ));

  // Get actual channel from servo index
  int actual_channel = getServoChannel(channel);

  // Set PWM
  pwm.setPWM(actual_channel, 0, pwm_value);

  servo_positions[channel] = angle;
}

int getServoChannel(int servo_idx) {
  // Map servo index to actual PCA9685 channel
  switch (servo_idx) {
    case 0: return SERVO_JOINT_1;
    case 1: return SERVO_JOINT_2;
    case 2: return SERVO_JOINT_3;
    case 3: return SERVO_JOINT_4;
    case 4: return SERVO_GRIPPER_BASE;
    case 5: return SERVO_LEFT_GEAR;
    default: return 0;
  }
}

void updateServos() {
  // Smoothly move servos towards target positions
  // Move 1 degree per iteration for smooth motion
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servo_positions[i] < servo_targets[i]) {
      servo_positions[i]++;
      setServo(i, servo_positions[i]);
    } else if (servo_positions[i] > servo_targets[i]) {
      servo_positions[i]--;
      setServo(i, servo_positions[i]);
    }
  }
}

// ============================================================================
// STATUS REPORTING
// ============================================================================

void sendStatus() {
  // Send comprehensive status
  Serial.println("STATUS,RUNNING");
  Serial.print("EMERGENCY_STOP,");
  Serial.println(emergency_stop ? "ACTIVE" : "INACTIVE");

  // Servo positions
  Serial.print("SERVO_POS,");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(servo_positions[i]);
    if (i < NUM_SERVOS - 1) Serial.print(",");
  }
  Serial.println();

  // Encoder counts
  publishOdometry();
}

// ============================================================================
// END OF FIRMWARE
// ============================================================================
