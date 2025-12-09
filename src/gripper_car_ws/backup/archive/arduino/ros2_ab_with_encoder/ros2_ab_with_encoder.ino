/*********************************************************************
 * Four-Wheel Motor + Encoder Firmware for Arduino MEGA
 * Matches user wiring EXACTLY
 *
 * Sends:  ENC,LF,RF,LB,RB
 * Expects: VEL,linear,angular
 *          STOP
 *********************************************************************/

/////////////////////////// PIN MAP /////////////////////////////////

// ---------------------- MOTOR PINS -------------------------------
// Driver 1 (M1 = LF, M2 = RB)
#define LF_IN1 2
#define LF_IN2 3
#define RB_IN1 4
#define RB_IN2 5

// Driver 2 (M3 = LB, M4 = RF)
#define LB_IN1 6
#define LB_IN2 7
#define RF_IN1 8
#define RF_IN2 9

// --------------- ENCODER PINS (quadrature) ------------------------
// LF encoder → pins 10, 11
#define LF_ENC_A 10
#define LF_ENC_B 11

// RB encoder → pins 12, 13
#define RB_ENC_A 12
#define RB_ENC_B 13

// LB encoder → pins 24, 25
#define LB_ENC_A 24
#define LB_ENC_B 25

// RF encoder → pins 22, 23
#define RF_ENC_A 22
#define RF_ENC_B 23

/////////////////////////////////////////////////////////////////////

/////////////////////// ENCODER COUNTERS ////////////////////////////
volatile long lf_count = 0;
volatile long rf_count = 0;
volatile long lb_count = 0;
volatile long rb_count = 0;

/////////////////////// ENCODER ISRs ////////////////////////////////

void encLF() { lf_count += (digitalRead(LF_ENC_A) == digitalRead(LF_ENC_B)) ? 1 : -1; }
void encRB() { rb_count += (digitalRead(RB_ENC_A) == digitalRead(RB_ENC_B)) ? 1 : -1; }
void encLB() { lb_count += (digitalRead(LB_ENC_A) == digitalRead(LB_ENC_B)) ? 1 : -1; }
void encRF() { rf_count += (digitalRead(RF_ENC_A) == digitalRead(RF_ENC_B)) ? 1 : -1; }

/////////////////////////////////////////////////////////////////////

//////////////////////// CONTROL CONSTANTS //////////////////////////
float lin_cmd = 0;
float ang_cmd = 0;

const float wheel_separation = 0.30;  // meters
const int max_pwm = 255;

unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 500; // ms

/////////////////////////////////////////////////////////////////////

//////////////////// MOTOR CONTROL HELPERS //////////////////////////
void driveMotor(int in1, int in2, int pwm)
{
    pwm = constrain(pwm, -255, 255);

    if (pwm > 0) {
        analogWrite(in1, pwm);
        analogWrite(in2, 0);
    } else if (pwm < 0) {
        analogWrite(in1, 0);
        analogWrite(in2, -pwm);
    } else {
        analogWrite(in1, 0);
        analogWrite(in2, 0);
    }
}

void stopAll()
{
    driveMotor(LF_IN1, LF_IN2, 0);
    driveMotor(RB_IN1, RB_IN2, 0);
    driveMotor(LB_IN1, LB_IN2, 0);
    driveMotor(RF_IN1, RF_IN2, 0);
}

/////////////////////////////////////////////////////////////////////

//////////////////////////// SETUP //////////////////////////////////
void setup()
{
    Serial.begin(57600);

    // Motor output pins
    pinMode(LF_IN1, OUTPUT);
    pinMode(LF_IN2, OUTPUT);
    pinMode(RB_IN1, OUTPUT);
    pinMode(RB_IN2, OUTPUT);
    pinMode(LB_IN1, OUTPUT);
    pinMode(LB_IN2, OUTPUT);
    pinMode(RF_IN1, OUTPUT);
    pinMode(RF_IN2, OUTPUT);

    // Encoder pins
    pinMode(LF_ENC_A, INPUT_PULLUP);
    pinMode(LF_ENC_B, INPUT_PULLUP);

    pinMode(RB_ENC_A, INPUT_PULLUP);
    pinMode(RB_ENC_B, INPUT_PULLUP);

    pinMode(LB_ENC_A, INPUT_PULLUP);
    pinMode(LB_ENC_B, INPUT_PULLUP);

    pinMode(RF_ENC_A, INPUT_PULLUP);
    pinMode(RF_ENC_B, INPUT_PULLUP);

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(LF_ENC_A), encLF, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RB_ENC_A), encRB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LB_ENC_A), encLB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RF_ENC_A), encRF, CHANGE);

    last_cmd_time = millis();
}

/////////////////////////////////////////////////////////////////////

///////////////////////// MAIN LOOP //////////////////////////////////
void loop()
{
    // ----------------- Handle Serial commands ----------------------
    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        line.trim();

        if (line.startsWith("VEL"))
        {
            int c1 = line.indexOf(',');
            int c2 = line.indexOf(',', c1 + 1);

            lin_cmd = line.substring(c1 + 1, c2).toFloat();
            ang_cmd = line.substring(c2 + 1).toFloat();

            last_cmd_time = millis();
        }
        else if (line.startsWith("STOP"))
        {
            lin_cmd = 0;
            ang_cmd = 0;
            stopAll();
        }
    }

    // --------- Watchdog: if no cmd, stop motors ----------
    if (millis() - last_cmd_time > CMD_TIMEOUT) {
        lin_cmd = 0;
        ang_cmd = 0;
    }

    // ---------------- Kinematics → wheel speeds ---------------------
    float v_left  = lin_cmd - ang_cmd * (wheel_separation / 2.0);
    float v_right = lin_cmd + ang_cmd * (wheel_separation / 2.0);

    int pwm_left  = constrain(v_left  * 400, -255, 255); // scale for your motors
    int pwm_right = constrain(v_right * 400, -255, 255);

    // ---------------- Drive all 4 wheels -----------------------------
    driveMotor(LF_IN1, LF_IN2, pwm_left);
    driveMotor(LB_IN1, LB_IN2, pwm_left);
    driveMotor(RF_IN1, RF_IN2, pwm_right);
    driveMotor(RB_IN1, RB_IN2, pwm_right);

    // ---------------- Send encoder data every 20ms -------------------
    static unsigned long t_last = 0;
    if (millis() - t_last > 20)
    {
        noInterrupts();
        long lf = lf_count;
        long rf = rf_count;
        long lb = lb_count;
        long rb = rb_count;
        interrupts();

        Serial.print("ENC,");
        Serial.print(lf); Serial.print(",");
        Serial.print(rf); Serial.print(",");
        Serial.print(lb); Serial.print(",");
        Serial.println(rb);

        t_last = millis();
    }
}
