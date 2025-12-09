/*
 * MOBILE BASE CONTROLLER - Arduino Mega 2560
 * ============================================
 *
 * Four-wheel differential drive controller with:
 * - PID velocity control per wheel
 * - Quadrature encoder feedback
 * - Serial communication with ROS2
 * - TEST mode for simulated encoder feedback (no hardware required)
 *
 * Hardware:
 * - Arduino Mega 2560
 * - 2x L298N motor drivers
 * - 4x DC motors with quadrature encoders
 *
 * Serial Protocol:
 *   ROS -> Arduino:
 *     VEL,<linear_x>,<angular_z>  - Velocity command (m/s, rad/s)
 *     STOP                        - Emergency stop
 *     PID,<kp>,<ki>,<kd>         - Update PID gains
 *     RST                         - Reset encoder counts
 *     TEST,ON                     - Enable test mode (simulate encoders)
 *     TEST,OFF                    - Disable test mode (use real encoders)
 *
 *   Arduino -> ROS:
 *     ENC,<fl>,<fr>,<bl>,<br>    - Cumulative encoder ticks
 *     STATUS,<message>           - Status messages
 *     ERROR,<message>            - Error messages
 */

// =====================================================
// CONFIGURATION
// =====================================================

// Serial
#define SERIAL_BAUD 115200

// Robot parameters
#define WHEEL_RADIUS 0.075  // meters
#define WHEEL_BASE 0.35     // meters (left-right wheel distance = 2 * 0.175)
#define ENCODER_CPR 360     // Counts per revolution (after decoding)

// Timing
#define PID_SAMPLE_TIME 20   // milliseconds
#define ENCODER_PUB_RATE 50  // Hz
#define CMD_TIMEOUT 2000     // milliseconds - stop if no command received

// PID gains (can be tuned via serial)
float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

// Encoder debounce window (can be tuned via serial: DEBOUNCE,<us>)
// Filters electrical noise and crosstalk from motor currents
// Real encoder transitions at 150 RPM ≈ 1.1ms apart; noise is <100us
unsigned long encoder_debounce_us = 200;  // microseconds (0.2 ms default)

// Per-motor speed calibration factors
// Use these to normalize wheel speeds and prevent differential slip
// Typical range: 0.8 - 1.2 (multiplicative factors applied to encoder counts)
// Per-motor PWM scaling factors
// Use these to compensate for motor/wheel differences by scaling the
// controller output (PWM-like) per motor. Typical range: 0.8 - 1.2
// Set via CAL command: CAL,<s0>,<s1>,<s2>,<s3>
float motor_cal[4] = { 0.902, 0.750, 1.00, 0.80 };  // FL, FR, BL, BR (1.0 = no scaling)

// NOTE: On this hardware the motor driver ENABLE (EN/PWM) pins control speed
// Motor drivers are driven using IN1/IN2 for direction + EN pin for speed (0-255 PWM).
// The controller uses PID to compute a signed velocity error, then outputs
// a PWM magnitude that is applied via the EN pin while IN1/IN2 select direction.
// This enables true closed-loop speed control with encoder feedback per wheel.

// =====================================================
// PIN DEFINITIONS - Arduino Mega 2560 + 2x L298N
// =====================================================

// Motor Driver 1 (L298N #1) - Front motors
#define M_FL_IN1 22  // Front Left Direction A
#define M_FL_IN2 23  // Front Left Direction B
#define M_FL_EN 12   // Front Left Speed (PWM)
#define M_FR_IN1 24  // Front Right Direction A
#define M_FR_IN2 25  // Front Right Direction B
#define M_FR_EN 11   // Front Right Speed (PWM)

// Motor Driver 2 (L298N #2) - Back motors
#define M_BL_IN1 26  // Back Left Direction A
#define M_BL_IN2 27  // Back Left Direction B
#define M_BL_EN 10   // Back Left Speed (PWM)
#define M_BR_IN1 28  // Back Right Direction A
#define M_BR_IN2 29  // Back Right Direction B
#define M_BR_EN 9    // Back Right Speed (PWM)

// Quadrature Encoders (using interrupt-capable pins)
#define ENC_FL_A 18  // Front Left Channel A (INT.3)
#define ENC_FL_B 31  // Front Left Channel B
#define ENC_FR_A 19  // Front Right Channel A (INT.2)
#define ENC_FR_B 33  // Front Right Channel B
#define ENC_BL_A 20  // Back Left Channel A (INT.1)
#define ENC_BL_B 35  // Back Left Channel B
#define ENC_BR_A 21  // Back Right Channel A (INT.0)
#define ENC_BR_B 37  // Back Right Channel B

// =====================================================
// GLOBAL VARIABLES
// =====================================================

// Encoder counts (volatile for ISR access)
volatile long enc_counts[4] = { 0, 0, 0, 0 };  // FL, FR, BL, BR
long prev_enc_counts[4] = { 0, 0, 0, 0 };

// Debounce tracking for encoder ISRs (volatile for ISR access)
// Ignores ISR triggers within ENCODER_DEBOUNCE_MS of the previous one
// This filters electrical noise and crosstalk while passing real encoder transitions
#define ENCODER_DEBOUNCE_MS 2  // milliseconds
volatile unsigned long last_isr_time[4] = { 0, 0, 0, 0 };

// Per-encoder sign mapping: set to +1 if the encoder count increases when
// the wheel rotates in the controller's "forward" direction, or -1 if
// the encoder wiring/polarity is inverted. Adjust if you rewire encoders.
const int encoder_dir[4] = { 1, -1, 1, -1 };  // FL, FR, BL, BR

// Target velocities (rad/s for each wheel)
float target_vel[4] = { 0, 0, 0, 0 };  // FL, FR, BL, BR
float current_vel[4] = { 0, 0, 0, 0 };

// PID state for each wheel
float pid_integral[4] = { 0, 0, 0, 0 };
float pid_prev_error[4] = { 0, 0, 0, 0 };
// motor_state: -1 = reverse, 0 = stopped, 1 = forward
int motor_state[4] = { 0, 0, 0, 0 };
int prev_motor_state[4] = { 0, 0, 0, 0 };
// If a motor experienced an unexpected direction flip, lock it for diagnosis
bool motor_locked[4] = { false, false, false, false };

// Timing
unsigned long last_pid_time = 0;
unsigned long last_enc_pub_time = 0;
unsigned long last_cmd_time = 0;
unsigned long last_test_update_time = 0;

// Command buffer
String cmd_buffer = "";

// TEST MODE - Simulates encoder feedback without real hardware
bool test_mode = false;
float sim_enc_accumulator[4] = { 0.0, 0.0, 0.0, 0.0 };  // Fractional tick accumulator

// Manual MOT command tracking: when a direct MOT command is used we mark
// the motor as manually driven and honor the same CMD_TIMEOUT behavior as
// velocity commands. This prevents MOT from leaving motors running
// indefinitely if the host stops sending commands.
bool manual_active[4] = { false, false, false, false };

// Last published encoder counts (for change detection)
long last_published_counts[4] = { 0, 0, 0, 0 };

// =====================================================
// MOTOR CONTROL
// =====================================================

// Motor control pin arrays
const int motor_in1_pins[4] = { M_FL_IN1, M_FR_IN1, M_BL_IN1, M_BR_IN1 };
const int motor_in2_pins[4] = { M_FL_IN2, M_FR_IN2, M_BL_IN2, M_BR_IN2 };
const int motor_en_pins[4] = { M_FL_EN, M_FR_EN, M_BL_EN, M_BR_EN };  // PWM enable pins

// Motor direction multipliers (adjust if motors spin wrong way)
// 1 = normal, -1 = reversed
// NOTE: set to all +1 for now so direction inversion is handled by wiring
// only. This avoids double-inversion with encoder_dir and prevents
// control sign conflicts while tuning PID logic.
const int motor_direction[4] = { 1, 1, 1, 1 };  // FL, FR, BL, BR

void setupMotors() {
  for (int i = 0; i < 4; i++) {
    pinMode(motor_in1_pins[i], OUTPUT);
    pinMode(motor_in2_pins[i], OUTPUT);
    pinMode(motor_en_pins[i], OUTPUT);

    // Initialize to stopped (all pins LOW)
    digitalWrite(motor_in1_pins[i], LOW);
    digitalWrite(motor_in2_pins[i], LOW);
    analogWrite(motor_en_pins[i], 0);  // PWM to 0 (stopped)
  }
}

// Set motor speed and direction using PWM + IN pins
// pwm_like: -255 (full reverse) to +255 (full forward), 0 = stop
// Applies per-motor calibration and respects direction mapping
void setMotorOutput(int motor_idx, int pwm_like) {
  // Respect motor lock for diagnosis
  if (motor_locked[motor_idx] && pwm_like != 0) {
    Serial.print("HW,LOCKED,");
    Serial.print(motor_idx);
    Serial.println(",ignoring command");
    return;
  }
  // Apply per-motor scaling calibration
  float scaled_f = pwm_like * motor_cal[motor_idx];
  int scaled_pwm = (int)constrain(scaled_f, -255, 255);

  // Apply direction mapping
  int final_pwm = scaled_pwm * motor_direction[motor_idx];

  // Determine direction and speed magnitude
  int pwm_magnitude = abs(final_pwm);
  int direction = (final_pwm > 0) ? 1 : (final_pwm < 0) ? -1
                                                        : 0;

  // Set direction pins
  if (direction > 0) {
    // Forward
    digitalWrite(motor_in1_pins[motor_idx], HIGH);
    digitalWrite(motor_in2_pins[motor_idx], LOW);
    motor_state[motor_idx] = 1;
  } else if (direction < 0) {
    // Reverse
    digitalWrite(motor_in1_pins[motor_idx], LOW);
    digitalWrite(motor_in2_pins[motor_idx], HIGH);
    motor_state[motor_idx] = -1;
  } else {
    // Stop
    digitalWrite(motor_in1_pins[motor_idx], LOW);
    digitalWrite(motor_in2_pins[motor_idx], LOW);
    motor_state[motor_idx] = 0;
  }

  // Apply speed via PWM (0-255)
  analogWrite(motor_en_pins[motor_idx], pwm_magnitude);

  // Detect a direction flip (forward <-> reverse). If this occurs during
  // normal operation (test_mode == false) we lock and stop the motor so
  // the user can diagnose which motor flips unexpectedly.
  if (motor_state[motor_idx] != prev_motor_state[motor_idx]) {
    // Determine if this is a sign flip (forward <-> reverse)
    if ((prev_motor_state[motor_idx] == 1 && motor_state[motor_idx] == -1) || (prev_motor_state[motor_idx] == -1 && motor_state[motor_idx] == 1)) {
      // Print diagnostic and lock motor if not in test mode
      Serial.print("HW,DIR_FLIP,");
      Serial.print(motor_idx);
      Serial.print(",from=");
      Serial.print(prev_motor_state[motor_idx]);
      Serial.print(",to=");
      Serial.print(motor_state[motor_idx]);
      long counts[4];
      getEncoderCounts(counts);
      Serial.print(",ENC=");
      Serial.print(counts[0]);
      Serial.print(",");
      Serial.print(counts[1]);
      Serial.print(",");
      Serial.print(counts[2]);
      Serial.print(",");
      Serial.println(counts[3]);

      if (!test_mode) {
        // Stop and lock the motor for diagnosis
        analogWrite(motor_en_pins[motor_idx], 0);
        digitalWrite(motor_in1_pins[motor_idx], LOW);
        digitalWrite(motor_in2_pins[motor_idx], LOW);
        motor_locked[motor_idx] = true;
        motor_state[motor_idx] = 0;
      }
    }

    // If we locked the motor above, the EN pin was set to 0 and
    // motor_state may have been changed to 0. Compute the actual
    // EN value and final pwm that were applied for accurate logging.
    int actual_en = motor_locked[motor_idx] ? 0 : pwm_magnitude;
    int actual_final_pwm = motor_locked[motor_idx] ? 0 : final_pwm;

    // Always update prev state for future comparisons and print a concise log
    prev_motor_state[motor_idx] = motor_state[motor_idx];
    Serial.print("HW,MOTOR,");
    Serial.print(motor_idx);
    Serial.print(",state=");
    Serial.print(motor_state[motor_idx]);
    Serial.print(",raw_pwm=");
    Serial.print(pwm_like);
    Serial.print(",final_pwm=");
    Serial.print(actual_final_pwm);
    Serial.print(",EN=");
    Serial.println(actual_en);
  }
}


// Print hardware pin mapping and current pin levels for diagnosis
void printHardwareStatus() {
  Serial.println("HW,STATUS,Pin mapping and current levels:");
  for (int i = 0; i < 4; i++) {
    Serial.print("HW,MOTOR,");
    Serial.print(i);
    Serial.print(",IN1_pin=");
    Serial.print(motor_in1_pins[i]);
    Serial.print(",IN2_pin=");
    Serial.print(motor_in2_pins[i]);
    Serial.print(",IN1_level=");
    Serial.print(digitalRead(motor_in1_pins[i]));
    Serial.print(",IN2_level=");
    Serial.print(digitalRead(motor_in2_pins[i]));
    Serial.print(",state=");
    Serial.print(motor_state[i]);
    Serial.print(",locked=");
    Serial.println(motor_locked[i] ? 1 : 0);
  }

  // Encoder pins
  Serial.print("HW,ENC,FL_A=");
  Serial.print(digitalRead(ENC_FL_A));
  Serial.print(",FL_B=");
  Serial.print(digitalRead(ENC_FL_B));
  Serial.print(",FR_A=");
  Serial.print(digitalRead(ENC_FR_A));
  Serial.print(",FR_B=");
  Serial.print(digitalRead(ENC_FR_B));
  Serial.print(",BL_A=");
  Serial.print(digitalRead(ENC_BL_A));
  Serial.print(",BL_B=");
  Serial.print(digitalRead(ENC_BL_B));
  Serial.print(",BR_A=");
  Serial.print(digitalRead(ENC_BR_A));
  Serial.print(",BR_B=");
  Serial.println(digitalRead(ENC_BR_B));

  long counts[4];
  getEncoderCounts(counts);
  Serial.print("HW,ENC_COUNTS,");
  Serial.print(counts[0]);
  Serial.print(",");
  Serial.print(counts[1]);
  Serial.print(",");
  Serial.print(counts[2]);
  Serial.print(",");
  Serial.println(counts[3]);
}
void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    setMotorOutput(i, 0);
    target_vel[i] = 0;
    pid_integral[i] = 0;
    pid_prev_error[i] = 0;
    manual_active[i] = false;
  }
}

// =====================================================
// ENCODER HANDLING
// =====================================================

void setupEncoders() {
  // Set encoder pins as inputs with pullups
  pinMode(ENC_FL_A, INPUT_PULLUP);
  pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_BL_A, INPUT_PULLUP);
  pinMode(ENC_BL_B, INPUT_PULLUP);
  pinMode(ENC_BR_A, INPUT_PULLUP);
  pinMode(ENC_BR_B, INPUT_PULLUP);

  // Attach interrupts (2x decoding - rising edge on channel A)
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isr_enc_fl, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isr_enc_fr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), isr_enc_bl, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), isr_enc_br, RISING);
}

// Encoder ISRs - 2x decoding (count on rising edge of A)
// With debouncing to filter noise/crosstalk from motor currents
void isr_enc_fl() {
  unsigned long now = micros();
  if (now - last_isr_time[0] < encoder_debounce_us) {
    return;  // Ignore spurious pulses within debounce window
  }
  last_isr_time[0] = now;

  if (digitalRead(ENC_FL_B)) {
    enc_counts[0]--;
  } else {
    enc_counts[0]++;
  }
}

void isr_enc_fr() {
  unsigned long now = micros();
  if (now - last_isr_time[1] < encoder_debounce_us) {
    return;  // Ignore spurious pulses within debounce window
  }
  last_isr_time[1] = now;

  if (digitalRead(ENC_FR_B)) {
    enc_counts[1]++;
  } else {
    enc_counts[1]--;
  }
}

void isr_enc_bl() {
  unsigned long now = micros();
  if (now - last_isr_time[2] < encoder_debounce_us) {
    return;  // Ignore spurious pulses within debounce window
  }
  last_isr_time[2] = now;

  if (digitalRead(ENC_BL_B)) {
    enc_counts[2]--;
  } else {
    enc_counts[2]++;
  }
}

void isr_enc_br() {
  unsigned long now = micros();
  if (now - last_isr_time[3] < encoder_debounce_us) {
    return;  // Ignore spurious pulses within debounce window
  }
  last_isr_time[3] = now;

  if (digitalRead(ENC_BR_B)) {
    enc_counts[3]++;
  } else {
    enc_counts[3]--;
  }
}

void getEncoderCounts(long* counts) {
  // Disable interrupts for atomic read
  noInterrupts();
  for (int i = 0; i < 4; i++) {
    counts[i] = enc_counts[i];
  }
  interrupts();
}

void resetEncoders() {
  noInterrupts();
  for (int i = 0; i < 4; i++) {
    enc_counts[i] = 0;
    prev_enc_counts[i] = 0;
  }
  interrupts();

  // Also reset simulated encoder accumulators
  for (int i = 0; i < 4; i++) {
    sim_enc_accumulator[i] = 0.0;
  }
}

// =====================================================
// TEST MODE - Simulated Encoder Update
// =====================================================

void updateSimulatedEncoders() {
  // Only run in test mode
  if (!test_mode) return;

  unsigned long now = millis();
  float dt = (now - last_test_update_time) / 1000.0;  // Convert to seconds

  // Limit dt to reasonable values
  if (dt <= 0 || dt > 0.1) {
    last_test_update_time = now;
    return;
  }

  // Simulate encoder ticks based on target wheel velocities
  // ticks = (omega * dt) / (2 * PI) * CPR
  // where omega is wheel angular velocity in rad/s

  for (int i = 0; i < 4; i++) {
    // Calculate expected ticks from target velocity
    float delta_ticks = (target_vel[i] * dt * ENCODER_CPR) / (2.0 * PI);

    // Accumulate fractional ticks
    sim_enc_accumulator[i] += delta_ticks;

    // Extract whole ticks and add to encoder count
    long whole_ticks = (long)sim_enc_accumulator[i];
    if (whole_ticks != 0) {
      noInterrupts();
      enc_counts[i] += whole_ticks;
      interrupts();
      sim_enc_accumulator[i] -= whole_ticks;
    }
  }

  last_test_update_time = now;
}

// =====================================================
// PID CONTROLLER
// =====================================================

float computePID(int motor_idx, float target, float current, float dt) {
  float error = target - current;

  // Proportional term
  float p_term = Kp * error;

  // Integral term with anti-windup
  pid_integral[motor_idx] += error * dt;
  pid_integral[motor_idx] = constrain(pid_integral[motor_idx], -100, 100);
  float i_term = Ki * pid_integral[motor_idx];

  // Derivative term
  float d_term = 0;
  if (dt > 0) {
    d_term = Kd * (error - pid_prev_error[motor_idx]) / dt;
  }
  pid_prev_error[motor_idx] = error;

  // Compute output
  float output = p_term + i_term + d_term;

  // Scale PID output into PWM range and return magnitude only (0..255).
  // We intentionally return magnitude so direction is controlled only
  // by the sign of the target velocity elsewhere.
  float output_scale = 25.5;  // 255 / 10
  output = output * output_scale;

  return constrain(fabs(output), 0, 255);
}

void updatePID() {
  unsigned long now = millis();
  float dt = (now - last_pid_time) / 1000.0;  // Convert to seconds

  if (dt < (PID_SAMPLE_TIME / 1000.0)) {
    return;  // Not time yet
  }

  // Get current encoder counts
  long counts[4];
  getEncoderCounts(counts);

  // Calculate wheel velocities (rad/s)
  for (int i = 0; i < 4; i++) {
    // Normalize encoder delta by encoder_dir so that a positive delta
    // always corresponds to the controller's "forward" direction.
    long raw_delta = counts[i] - prev_enc_counts[i];
    long delta = raw_delta * encoder_dir[i];
    prev_enc_counts[i] = counts[i];

    // Convert encoder ticks to radians
    // velocity = (delta_ticks / CPR) * 2*PI / dt
    current_vel[i] = (delta * 2.0 * PI) / (ENCODER_CPR * dt);
  }

  // Update PID for each wheel
  for (int i = 0; i < 4; i++) {
    // Deadband: if velocity error is small, treat as reached and stop
    float vel_error = target_vel[i] - current_vel[i];
    const float VEL_DEADBAND = 0.3; // rad/s
    if (fabs(vel_error) < VEL_DEADBAND) {
      setMotorOutput(i, 0);
      pid_integral[i] = 0;
      continue;
    }

    // Compute magnitude-only PID using absolute speeds
    int pwm_mag = (int)computePID(i, fabs(target_vel[i]), fabs(current_vel[i]), dt);

    // If target is effectively zero, ensure we don't drive
    if (fabs(target_vel[i]) < 0.01) {
      pwm_mag = 0;
      pid_integral[i] = 0;
    }

    // Apply direction based on target sign only
    int pwm = (target_vel[i] >= 0) ? pwm_mag : -pwm_mag;

    // Now call setMotorOutput with signed PWM
    setMotorOutput(i, pwm);
  }

  last_pid_time = now;
}

// =====================================================
// DIFFERENTIAL DRIVE KINEMATICS
// =====================================================

void setVelocityCommand(float linear_x, float angular_z) {
  // Differential drive inverse kinematics
  // v_left = linear_x - angular_z * (wheel_base / 2)
  // v_right = linear_x + angular_z * (wheel_base / 2)

  float v_left = linear_x - (angular_z * WHEEL_BASE / 2.0);
  float v_right = linear_x + (angular_z * WHEEL_BASE / 2.0);

  // Convert linear velocity to wheel angular velocity (rad/s)
  float omega_left = v_left / WHEEL_RADIUS;
  float omega_right = v_right / WHEEL_RADIUS;

  // Set target velocities for all 4 wheels
  // FL and BL get left velocity, FR and BR get right velocity
  // Set wheel angular velocity targets. Keep sign mapping consistent
  // by applying motor_direction only in `setMotorOutput()` so the
  // controller and PID operate on physical wheel velocities.
  target_vel[0] = omega_left;   // Front Left
  target_vel[1] = omega_right;  // Front Right
  target_vel[2] = omega_left;   // Back Left
  target_vel[3] = omega_right;  // Back Right

  // Update last command time
  last_cmd_time = millis();
}

// =====================================================
// SERIAL COMMUNICATION
// =====================================================

void processCommand(String cmd) {
  cmd.trim();

  if (cmd.startsWith("VEL,")) {
    // Parse velocity command: VEL,linear_x,angular_z
    int idx1 = cmd.indexOf(',');
    int idx2 = cmd.indexOf(',', idx1 + 1);

    if (idx2 > idx1) {
      float linear_x = cmd.substring(idx1 + 1, idx2).toFloat();
      float angular_z = cmd.substring(idx2 + 1).toFloat();
      setVelocityCommand(linear_x, angular_z);
    }
  } else if (cmd == "STOP") {
    stopAllMotors();
    Serial.println("STATUS,Stopped");
  } else if (cmd.startsWith("PID,")) {
    // Parse PID gains: PID,kp,ki,kd
    int idx1 = cmd.indexOf(',');
    int idx2 = cmd.indexOf(',', idx1 + 1);
    int idx3 = cmd.indexOf(',', idx2 + 1);

    if (idx3 > idx2 && idx2 > idx1) {
      Kp = cmd.substring(idx1 + 1, idx2).toFloat();
      Ki = cmd.substring(idx2 + 1, idx3).toFloat();
      Kd = cmd.substring(idx3 + 1).toFloat();
      Serial.print("STATUS,PID updated: Kp=");
      Serial.print(Kp);
      Serial.print(" Ki=");
      Serial.print(Ki);
      Serial.print(" Kd=");
      Serial.println(Kd);
    }
  } else if (cmd == "RST") {
    resetEncoders();
    Serial.println("STATUS,Encoders reset");
  } else if (cmd == "TEST,ON") {
    test_mode = true;
    last_test_update_time = millis();
    resetEncoders();
    Serial.println("STATUS,Test mode ENABLED - simulating encoders");
  } else if (cmd == "TEST,OFF") {
    test_mode = false;
    resetEncoders();
    Serial.println("STATUS,Test mode DISABLED - using real encoders");
  } else if (cmd == "STATUS") {
    // Return current status
    Serial.print("STATUS,Mode:");
    Serial.print(test_mode ? "TEST" : "NORMAL");
    Serial.print(",Vel:[");
    Serial.print(target_vel[0], 2);
    Serial.print(",");
    Serial.print(target_vel[1], 2);
    Serial.print(",");
    Serial.print(target_vel[2], 2);
    Serial.print(",");
    Serial.print(target_vel[3], 2);
    Serial.println("]");
  } else if (cmd == "HWCHK") {
    // Print hardware pin mapping and live levels for debugging
    printHardwareStatus();
  } else if (cmd.startsWith("DEBOUNCE,")) {
    // Parse debounce window: DEBOUNCE,<microseconds>
    int idx = cmd.indexOf(',');
    if (idx > 0) {
      unsigned long new_debounce = cmd.substring(idx + 1).toInt();
      if (new_debounce >= 0 && new_debounce <= 10000) {
        encoder_debounce_us = new_debounce;
        Serial.print("STATUS,Encoder debounce set to ");
        Serial.print(encoder_debounce_us);
        Serial.println(" microseconds");
      } else {
        Serial.println("ERROR,Debounce value out of range (0-10000 microseconds)");
      }
    }
  } else if (cmd.startsWith("CAL,")) {
    // Parse motor PWM scaling factors: CAL,<s0>,<s1>,<s2>,<s3>
    // Example: CAL,1.0,1.15,0.95,1.05 (scale controller output per motor)
    int i1 = cmd.indexOf(',');
    int i2 = cmd.indexOf(',', i1 + 1);
    int i3 = cmd.indexOf(',', i2 + 1);
    int i4 = cmd.indexOf(',', i3 + 1);

    if (i4 > i3 && i3 > i2 && i2 > i1) {
      motor_cal[0] = cmd.substring(i1 + 1, i2).toFloat();
      motor_cal[1] = cmd.substring(i2 + 1, i3).toFloat();
      motor_cal[2] = cmd.substring(i3 + 1, i4).toFloat();
      motor_cal[3] = cmd.substring(i4 + 1).toFloat();

      Serial.print("STATUS,Motor calibration updated: ");
      Serial.print(motor_cal[0], 3);
      Serial.print(",");
      Serial.print(motor_cal[1], 3);
      Serial.print(",");
      Serial.print(motor_cal[2], 3);
      Serial.print(",");
      Serial.println(motor_cal[3], 3);
    } else {
      Serial.println("ERROR,CAL format: CAL,<s0>,<s1>,<s2>,<s3>");
    }
  } else if (cmd.startsWith("MOT,")) {
    int i1 = cmd.indexOf(',');
    int i2 = cmd.indexOf(',', i1 + 1);
    if (i2 > i1) {
      int m_idx = cmd.substring(i1 + 1, i2).toInt();
      int pwm = cmd.substring(i2 + 1).toInt();
      if (m_idx >= 0 && m_idx < 4) {
        setMotorOutput(m_idx, pwm);
        manual_active[m_idx] = true;
        last_cmd_time = millis();
        Serial.print("STATUS,MOT,");
        Serial.print(m_idx);
        Serial.print(",");
        Serial.println(pwm);
      }
    }
  } else if (cmd == "ENCHECK") {
    // Run forward/reverse encoder check for each motor and print deltas
    Serial.println("TEST,ENCHECK,START");
    long base_counts[4];
    getEncoderCounts(base_counts);

    for (int m = 0; m < 4; m++) {
      if (motor_locked[m]) {
        Serial.print("TEST,ENCHECK,SKIP_LOCKED,");
        Serial.println(m);
        continue;
      }

      // Forward pulse
      long before_fw[4];
      getEncoderCounts(before_fw);
      setMotorOutput(m, 150);
      delay(500);
      setMotorOutput(m, 0);
      long after_fw[4];
      getEncoderCounts(after_fw);
      long delta_fw = after_fw[m] - before_fw[m];

      // Reverse pulse
      long before_rev[4];
      getEncoderCounts(before_rev);
      setMotorOutput(m, -150);
      delay(500);
      setMotorOutput(m, 0);
      long after_rev[4];
      getEncoderCounts(after_rev);
      long delta_rev = after_rev[m] - before_rev[m];

      Serial.print("TEST,ENCHECK,EDGES,");
      Serial.print(m);
      Serial.print(",fw=");
      Serial.print(delta_fw);
      Serial.print(",rev=");
      Serial.println(delta_rev);
      delay(200);
    }

    Serial.println("TEST,ENCHECK,DONE");
  } else if (cmd.startsWith("UNLOCK,")) {
    int idx = cmd.indexOf(',');
    String arg = cmd.substring(idx + 1);
    if (arg == "ALL") {
      for (int i = 0; i < 4; i++) motor_locked[i] = false;
      Serial.println("STATUS,UNLOCK,ALL");
    } else {
      int m = arg.toInt();
      if (m >= 0 && m < 4) {
        motor_locked[m] = false;
        Serial.print("STATUS,UNLOCK,");
        Serial.println(m);
      }
    }
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (cmd_buffer.length() > 0) {
        processCommand(cmd_buffer);
        cmd_buffer = "";
      }
    } else if (c != '\r') {
      cmd_buffer += c;
    }
  }
}

void publishEncoderCounts() {
  unsigned long now = millis();
  unsigned long pub_interval = 1000 / ENCODER_PUB_RATE;

  if (now - last_enc_pub_time >= pub_interval) {
    long counts[4];
    getEncoderCounts(counts);

    // Only publish if any encoder value has changed
    bool changed = false;
    for (int i = 0; i < 4; i++) {
      if (counts[i] != last_published_counts[i]) {
        changed = true;
        break;
      }
    }

    if (changed) {
      Serial.print("ENC,");
      Serial.print(counts[0]);
      Serial.print(",");
      Serial.print(counts[1]);
      Serial.print(",");
      Serial.print(counts[2]);
      Serial.print(",");
      Serial.println(counts[3]);

      // Update last published values
      for (int i = 0; i < 4; i++) {
        last_published_counts[i] = counts[i];
      }
    }

    last_enc_pub_time = now;
  }
}

/*
void checkCommandTimeout() {
    if (millis() - last_cmd_time > CMD_TIMEOUT) {
        // No command received for too long, stop motors
        if (target_vel[0] != 0 || target_vel[1] != 0 ||
            target_vel[2] != 0 || target_vel[3] != 0) {
            stopAllMotors();
            // Only print once
            Serial.println("STATUS,Command timeout - stopped");
        }
    }
}
*/

void checkCommandTimeout() {
  unsigned long dt = millis() - last_cmd_time;

  if (dt > CMD_TIMEOUT) {
    bool any_target = (target_vel[0] != 0 || target_vel[1] != 0 || target_vel[2] != 0 || target_vel[3] != 0);
    bool any_manual = (manual_active[0] || manual_active[1] || manual_active[2] || manual_active[3]);

    if (any_target || any_manual) {
      Serial.print("TIME SINCE LAST CMD: ");
      Serial.println(dt);

      stopAllMotors();
      Serial.println("STATUS,Command timeout - stopped");
    }
  }
}

// =====================================================
// MAIN
// =====================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ;  // Wait for serial port to connect
  }

  setupMotors();
  setupEncoders();

  last_pid_time = millis();
  last_enc_pub_time = millis();
  last_cmd_time = millis();
  last_test_update_time = millis();

  Serial.println("STATUS,Mobile Base Controller Ready - PWM Speed Control Enabled");
  Serial.print("STATUS,Wheel Radius: ");
  Serial.print(WHEEL_RADIUS);
  Serial.print("m, Wheel Base: ");
  Serial.print(WHEEL_BASE);
  Serial.println("m");
  Serial.println("STATUS,Motor Control: IN1/IN2 direction + PWM enable pins for speed");
  Serial.println("STATUS,Available commands:");
  Serial.println("STATUS,  VEL,<linear_x>,<angular_z> - Velocity command (closed-loop PID)");
  Serial.println("STATUS,  MOT,<idx>,<pwm> - Direct motor test (-255..255 PWM)");
  Serial.println("STATUS,  MOTOR,<idx>,FWD/REV,<ms> - Run motor for duration");
  Serial.println("STATUS,  MOTTEST - Run forward/reverse test on all motors");
  Serial.println("STATUS,  DEBOUNCE,<us> - Set encoder debounce (default 200 us)");
  Serial.println("STATUS,  CAL,<s0>,<s1>,<s2>,<s3> - Per-motor speed scaling factors");
  Serial.println("STATUS,  PID,<kp>,<ki>,<kd> - Update PID gains");
  Serial.println("STATUS,  HWCHK - Print hardware pin mapping");
  Serial.println("STATUS,  ENCHECK - Run forward/reverse encoder check per motor");
  Serial.println("STATUS,  UNLOCK,<idx|ALL> - Unlock motor(s) after DIR_FLIP detection");
  Serial.println("STATUS,  RST - Reset encoder counts");
  Serial.println("STATUS,  TEST,ON/OFF - Enable/disable encoder simulation");
  Serial.print("COMPILED CMD_TIMEOUT = ");
  Serial.println(CMD_TIMEOUT);

  // Print initial hardware mapping and pin levels to help diagnose wiring
  printHardwareStatus();
}

void loop() {
  // Read and process serial commands
  readSerial();

  // Check for command timeout
  checkCommandTimeout();

  // Update simulated encoders (only active in TEST mode)
  updateSimulatedEncoders();

  // Update PID controllers (skipped in TEST mode since no real motors)
  if (!test_mode) {
    updatePID();
  }

  // Publish encoder counts
  publishEncoderCounts();
}
