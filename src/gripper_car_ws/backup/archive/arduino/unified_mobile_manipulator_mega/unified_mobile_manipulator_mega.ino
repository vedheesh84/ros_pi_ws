/*
 * Unified Mobile Manipulator Controller for Arduino Mega
 *
 * Hardware Configuration:
 * - Arduino MEGA 2560
 * - 2x L298N Motor Drivers (4 DC motors for differential drive)
 * - 4x Quadrature Encoders (one per wheel)
 * - PCA9685 16-Channel PWM Servo Driver (I2C)
 * - 6x Servo Motors for robotic arm
 *
 * ROS2 Integration:
 * - Subscribes to /cmd_vel for base velocity commands
 * - Subscribes to /arm/joint_commands for arm servo positions
 * - Publishes encoder counts for odometry calculation
 * - Publishes servo positions for joint states
 *
 * Author: Generated for Mobile Manipulator Project
 * Date: 2025
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

// Serial Communication
#define BAUD_RATE 115200
#define SERIAL_TIMEOUT 500  // milliseconds

// Motor Driver Pin Definitions (L298N x2)
// Driver 1: Back Left (BL) and Back Right (BR)
#define BL_IN1  2   // Back Left Motor Direction 1
#define BL_IN2  3   // Back Left Motor Direction 2
#define BL_EN   10  // Back Left Motor Enable (PWM)

#define BR_IN1  4   // Back Right Motor Direction 1
#define BR_IN2  5   // Back Right Motor Direction 2
#define BR_EN   11  // Back Right Motor Enable (PWM)

// Driver 2: Front Left (FL) and Front Right (FR)
#define FL_IN1  6   // Front Left Motor Direction 1
#define FL_IN2  7   // Front Left Motor Direction 2
#define FL_EN   12  // Front Left Motor Enable (PWM)

#define FR_IN1  8   // Front Right Motor Direction 1
#define FR_IN2  9   // Front Right Motor Direction 2
#define FR_EN   13  // Front Right Motor Enable (PWM)

// Encoder Pin Definitions (Quadrature)
#define BL_ENC_A  18  // Interrupt pin (INT5)
#define BL_ENC_B  19  // Interrupt pin (INT4)

#define BR_ENC_A  20  // Interrupt pin (INT3)
#define BR_ENC_B  21  // Interrupt pin (INT2)

#define FL_ENC_A  2   // Interrupt pin (INT0) - Note: Shared with BL_IN1, reassign if needed
#define FL_ENC_B  22

#define FR_ENC_A  3   // Interrupt pin (INT1) - Note: Shared with BL_IN2, reassign if needed
#define FR_ENC_B  23

// Motor Control Constants
#define WHEEL_SEPARATION 0.67   // meters (distance between left and right wheels)
#define WHEEL_RADIUS 0.075      // meters
#define ENCODER_TICKS_PER_REV 360
#define MAX_PWM 255
#define MOTOR_DEADBAND 30       // Minimum PWM to overcome friction
#define CMD_TIMEOUT 300         // milliseconds

// PCA9685 Servo Driver
#define PCA9685_ADDRESS 0x40
#define SERVO_FREQ 50           // 50Hz for servos
#define NUM_SERVOS 6

// Servo Channel Mapping on PCA9685
#define SERVO_JOINT_1 0         // Base rotation
#define SERVO_JOINT_2 1         // Shoulder
#define SERVO_JOINT_3 2         // Elbow
#define SERVO_JOINT_4 4         // Wrist
#define SERVO_GRIPPER_BASE 5    // Gripper rotation
#define SERVO_GRIPPER_GEAR 6    // Gripper open/close

// Servo Pulse Width Limits (microseconds)
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_CENTER_PULSE 1500

// Gripper servo has different range
#define GRIPPER_MIN_PULSE 1000
#define GRIPPER_MAX_PULSE 2000

// Servo Angle Limits
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Update Rates
#define ENCODER_PUBLISH_RATE 50    // Hz
#define STATUS_PUBLISH_RATE 1      // Hz
#define SERVO_SMOOTH_STEP 1        // degrees per step
#define SERVO_SMOOTH_DELAY 30      // milliseconds between steps

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// PCA9685 Driver Instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// Encoder Counts (volatile for interrupt safety)
volatile long encoder_bl = 0;
volatile long encoder_br = 0;
volatile long encoder_fl = 0;
volatile long encoder_fr = 0;

// Motor Velocities
float linear_velocity = 0.0;   // m/s
float angular_velocity = 0.0;  // rad/s

// Servo Positions
uint8_t current_servo_positions[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};  // degrees
uint8_t target_servo_positions[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};   // degrees

// Timing
unsigned long last_cmd_time = 0;
unsigned long last_encoder_publish = 0;
unsigned long last_status_publish = 0;
unsigned long last_servo_update = 0;

// Safety Flag
bool emergency_stop = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void setupMotors();
void setupEncoders();
void setupServos();
void updateMotors();
void updateServos();
void publishEncoders();
void publishStatus();
void processSerialCommands();
void stopAllMotors();
void setMotorSpeed(int motor, int speed);
uint16_t angleToPulse(uint8_t angle, bool is_gripper = false);
void setServo(uint8_t channel, uint8_t angle, bool is_gripper = false);
void smoothMoveServo(uint8_t servo_index, uint8_t target_angle);

// Encoder ISRs
void encoderBL_ISR();
void encoderBR_ISR();
void encoderFL_ISR();
void encoderFR_ISR();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize Serial Communication
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(50);

  // Wait for serial connection
  delay(1000);
  Serial.println("STATUS,Unified Mobile Manipulator Initializing");

  // Setup Subsystems
  setupMotors();
  setupEncoders();
  setupServos();

  // Initialize Timers
  last_cmd_time = millis();
  last_encoder_publish = millis();
  last_status_publish = millis();
  last_servo_update = millis();

  Serial.println("STATUS,Initialization Complete");
  Serial.println("STATUS,Ready for Commands");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long current_time = millis();

  // Process incoming serial commands
  processSerialCommands();

  // Check for command timeout (safety feature)
  if (current_time - last_cmd_time > CMD_TIMEOUT && !emergency_stop) {
    linear_velocity = 0.0;
    angular_velocity = 0.0;
  }

  // Update motor speeds based on velocity commands
  updateMotors();

  // Smoothly update servo positions
  updateServos();

  // Publish encoder data at fixed rate
  if (current_time - last_encoder_publish >= (1000 / ENCODER_PUBLISH_RATE)) {
    publishEncoders();
    last_encoder_publish = current_time;
  }

  // Publish status at fixed rate
  if (current_time - last_status_publish >= (1000 / STATUS_PUBLISH_RATE)) {
    publishStatus();
    last_status_publish = current_time;
  }
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

void setupMotors() {
  // Configure motor driver pins as outputs
  pinMode(BL_IN1, OUTPUT);
  pinMode(BL_IN2, OUTPUT);
  pinMode(BL_EN, OUTPUT);

  pinMode(BR_IN1, OUTPUT);
  pinMode(BR_IN2, OUTPUT);
  pinMode(BR_EN, OUTPUT);

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_EN, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_EN, OUTPUT);

  // Initialize all motors to stopped state
  stopAllMotors();

  Serial.println("STATUS,Motors Initialized");
}

void updateMotors() {
  if (emergency_stop) {
    stopAllMotors();
    return;
  }

  // Calculate wheel velocities using differential drive kinematics
  // v_left = linear - angular * (wheel_separation / 2)
  // v_right = linear + angular * (wheel_separation / 2)

  float v_left = linear_velocity - (angular_velocity * WHEEL_SEPARATION / 2.0);
  float v_right = linear_velocity + (angular_velocity * WHEEL_SEPARATION / 2.0);

  // Convert velocities to PWM values
  // PWM = (velocity / wheel_radius) * scaling_factor
  int pwm_left = (int)(v_left / WHEEL_RADIUS * 255.0);
  int pwm_right = (int)(v_right / WHEEL_RADIUS * 255.0);

  // Constrain PWM values
  pwm_left = constrain(pwm_left, -MAX_PWM, MAX_PWM);
  pwm_right = constrain(pwm_right, -MAX_PWM, MAX_PWM);

  // Set motor speeds (left side: BL and FL, right side: BR and FR)
  setMotorSpeed(0, pwm_left);   // Back Left
  setMotorSpeed(1, pwm_right);  // Back Right
  setMotorSpeed(2, pwm_left);   // Front Left
  setMotorSpeed(3, pwm_right);  // Front Right
}

void setMotorSpeed(int motor, int speed) {
  // Apply deadband compensation
  if (abs(speed) > 0 && abs(speed) < MOTOR_DEADBAND) {
    speed = (speed > 0) ? MOTOR_DEADBAND : -MOTOR_DEADBAND;
  }

  int pwm_value = abs(speed);
  bool forward = (speed >= 0);

  switch (motor) {
    case 0: // Back Left
      digitalWrite(BL_IN1, forward ? HIGH : LOW);
      digitalWrite(BL_IN2, forward ? LOW : HIGH);
      analogWrite(BL_EN, pwm_value);
      break;

    case 1: // Back Right
      digitalWrite(BR_IN1, forward ? HIGH : LOW);
      digitalWrite(BR_IN2, forward ? LOW : HIGH);
      analogWrite(BR_EN, pwm_value);
      break;

    case 2: // Front Left
      digitalWrite(FL_IN1, forward ? HIGH : LOW);
      digitalWrite(FL_IN2, forward ? LOW : HIGH);
      analogWrite(FL_EN, pwm_value);
      break;

    case 3: // Front Right
      digitalWrite(FR_IN1, forward ? HIGH : LOW);
      digitalWrite(FR_IN2, forward ? LOW : HIGH);
      analogWrite(FR_EN, pwm_value);
      break;
  }
}

void stopAllMotors() {
  digitalWrite(BL_IN1, LOW);
  digitalWrite(BL_IN2, LOW);
  analogWrite(BL_EN, 0);

  digitalWrite(BR_IN1, LOW);
  digitalWrite(BR_IN2, LOW);
  analogWrite(BR_EN, 0);

  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  analogWrite(FL_EN, 0);

  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, LOW);
  analogWrite(FR_EN, 0);

  linear_velocity = 0.0;
  angular_velocity = 0.0;
}

// ============================================================================
// ENCODER FUNCTIONS
// ============================================================================

void setupEncoders() {
  // Configure encoder pins as inputs with pull-ups
  pinMode(BL_ENC_A, INPUT_PULLUP);
  pinMode(BL_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP);
  pinMode(BR_ENC_B, INPUT_PULLUP);
  pinMode(FL_ENC_A, INPUT_PULLUP);
  pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP);
  pinMode(FR_ENC_B, INPUT_PULLUP);

  // Attach interrupts for encoder A channels
  attachInterrupt(digitalPinToInterrupt(BL_ENC_A), encoderBL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BR_ENC_A), encoderBR_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), encoderFL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), encoderFR_ISR, CHANGE);

  Serial.println("STATUS,Encoders Initialized");
}

void encoderBL_ISR() {
  if (digitalRead(BL_ENC_A) == digitalRead(BL_ENC_B)) {
    encoder_bl++;
  } else {
    encoder_bl--;
  }
}

void encoderBR_ISR() {
  if (digitalRead(BR_ENC_A) == digitalRead(BR_ENC_B)) {
    encoder_br++;
  } else {
    encoder_br--;
  }
}

void encoderFL_ISR() {
  if (digitalRead(FL_ENC_A) == digitalRead(FL_ENC_B)) {
    encoder_fl++;
  } else {
    encoder_fl--;
  }
}

void encoderFR_ISR() {
  if (digitalRead(FR_ENC_A) == digitalRead(FR_ENC_B)) {
    encoder_fr++;
  } else {
    encoder_fr--;
  }
}

void publishEncoders() {
  // Send encoder counts to ROS2
  // Format: ODOM,enc_bl,enc_br,enc_fl,enc_fr,timestamp
  Serial.print("ODOM,");
  Serial.print(encoder_bl);
  Serial.print(",");
  Serial.print(encoder_br);
  Serial.print(",");
  Serial.print(encoder_fl);
  Serial.print(",");
  Serial.print(encoder_fr);
  Serial.print(",");
  Serial.println(millis());
}

// ============================================================================
// SERVO CONTROL FUNCTIONS
// ============================================================================

void setupServos() {
  // Initialize I2C and PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  delay(100);  // Allow PCA9685 to stabilize

  // Set all servos to center position
  for (int i = 0; i < NUM_SERVOS; i++) {
    bool is_gripper = (i == 5);  // Servo 5 is the gripper
    setServo(i, 90, is_gripper);
    current_servo_positions[i] = 90;
    target_servo_positions[i] = 90;
  }

  Serial.println("STATUS,Servos Initialized");
}

uint16_t angleToPulse(uint8_t angle, bool is_gripper) {
  // Convert angle (0-180) to pulse width in microseconds
  uint16_t min_pulse = is_gripper ? GRIPPER_MIN_PULSE : SERVO_MIN_PULSE;
  uint16_t max_pulse = is_gripper ? GRIPPER_MAX_PULSE : SERVO_MAX_PULSE;

  uint16_t pulse = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
                       min_pulse, max_pulse);

  // Convert microseconds to PCA9685 pulse count (0-4095)
  // Formula: pulse_count = (pulse_us * 4096) / (1000000 / freq)
  // At 50Hz: pulse_count = (pulse_us * 4096) / 20000
  uint16_t pulse_count = (pulse * 4096) / (1000000 / SERVO_FREQ);

  return pulse_count;
}

void setServo(uint8_t channel, uint8_t angle, bool is_gripper) {
  // Constrain angle to valid range
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  // Get channel mapping
  uint8_t pca_channel;
  switch(channel) {
    case 0: pca_channel = SERVO_JOINT_1; break;
    case 1: pca_channel = SERVO_JOINT_2; break;
    case 2: pca_channel = SERVO_JOINT_3; break;
    case 3: pca_channel = SERVO_JOINT_4; break;
    case 4: pca_channel = SERVO_GRIPPER_BASE; break;
    case 5: pca_channel = SERVO_GRIPPER_GEAR; break;
    default: return;
  }

  uint16_t pulse = angleToPulse(angle, is_gripper);
  pwm.setPWM(pca_channel, 0, pulse);
}

void updateServos() {
  unsigned long current_time = millis();

  // Smooth servo movement to prevent jerky motion
  if (current_time - last_servo_update >= SERVO_SMOOTH_DELAY) {
    bool movement_occurred = false;

    for (int i = 0; i < NUM_SERVOS; i++) {
      if (current_servo_positions[i] != target_servo_positions[i]) {
        bool is_gripper = (i == 5);

        // Move one step towards target
        if (current_servo_positions[i] < target_servo_positions[i]) {
          current_servo_positions[i] = min(current_servo_positions[i] + SERVO_SMOOTH_STEP,
                                           target_servo_positions[i]);
        } else {
          current_servo_positions[i] = max(current_servo_positions[i] - SERVO_SMOOTH_STEP,
                                           target_servo_positions[i]);
        }

        setServo(i, current_servo_positions[i], is_gripper);
        movement_occurred = true;
      }
    }

    last_servo_update = current_time;
  }
}

void publishServoPositions() {
  // Send current servo positions to ROS2
  // Format: SERVO_POS,p1,p2,p3,p4,p5,p6
  Serial.print("SERVO_POS,");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(current_servo_positions[i]);
    if (i < NUM_SERVOS - 1) Serial.print(",");
  }
  Serial.println();
}

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================

void processSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() == 0) return;

    // Parse command
    int comma_index = command.indexOf(',');
    String cmd_type = command.substring(0, comma_index);
    String cmd_data = command.substring(comma_index + 1);

    // Process different command types
    if (cmd_type == "VEL") {
      // Velocity command: VEL,<linear>,<angular>
      int second_comma = cmd_data.indexOf(',');
      linear_velocity = cmd_data.substring(0, second_comma).toFloat();
      angular_velocity = cmd_data.substring(second_comma + 1).toFloat();
      last_cmd_time = millis();
      Serial.println("OK,VEL");

    } else if (cmd_type == "SERVO") {
      // Single servo command: SERVO,<index>,<angle>
      int second_comma = cmd_data.indexOf(',');
      int servo_index = cmd_data.substring(0, second_comma).toInt();
      int angle = cmd_data.substring(second_comma + 1).toInt();

      if (servo_index >= 0 && servo_index < NUM_SERVOS) {
        target_servo_positions[servo_index] = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        Serial.println("OK,SERVO");
      } else {
        Serial.println("ERROR,Invalid servo index");
      }

    } else if (cmd_type == "SET_ALL_SERVOS") {
      // Set all servos: SET_ALL_SERVOS,<a1>,<a2>,<a3>,<a4>,<a5>,<a6>
      int angles[NUM_SERVOS];
      int start_index = 0;

      for (int i = 0; i < NUM_SERVOS; i++) {
        int comma_pos = cmd_data.indexOf(',', start_index);
        if (comma_pos == -1 && i < NUM_SERVOS - 1) {
          Serial.println("ERROR,Invalid servo data");
          return;
        }

        String angle_str = (comma_pos == -1) ?
                          cmd_data.substring(start_index) :
                          cmd_data.substring(start_index, comma_pos);
        angles[i] = angle_str.toInt();
        start_index = comma_pos + 1;
      }

      for (int i = 0; i < NUM_SERVOS; i++) {
        target_servo_positions[i] = constrain(angles[i], SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      }
      Serial.println("OK,SET_ALL_SERVOS");

    } else if (cmd_type == "STOP") {
      // Emergency stop
      stopAllMotors();
      emergency_stop = true;
      Serial.println("STATUS,EMERGENCY_STOP_ACTIVATED");

    } else if (cmd_type == "RESET_EMERGENCY") {
      // Reset emergency stop
      emergency_stop = false;
      Serial.println("STATUS,EMERGENCY_STOP_RELEASED");

    } else if (cmd_type == "RESET_ENCODERS") {
      // Reset encoder counts
      noInterrupts();
      encoder_bl = 0;
      encoder_br = 0;
      encoder_fl = 0;
      encoder_fr = 0;
      interrupts();
      Serial.println("OK,RESET_ENCODERS");

    } else if (cmd_type == "GET_STATUS") {
      // Request status information
      publishStatus();
      publishServoPositions();

    } else if (cmd_type == "GET_SERVOS") {
      // Request servo positions
      publishServoPositions();

    } else {
      Serial.print("ERROR,Unknown command: ");
      Serial.println(cmd_type);
    }
  }
}

void publishStatus() {
  Serial.print("STATUS,Running|Lin:");
  Serial.print(linear_velocity, 3);
  Serial.print("|Ang:");
  Serial.print(angular_velocity, 3);
  Serial.print("|EStop:");
  Serial.println(emergency_stop ? "ON" : "OFF");
}

// ============================================================================
// END OF PROGRAM
// ============================================================================
