/*
  Motor Direction & Encoder Test Sketch
  ------------------------------------
  Use this sketch to individually test forward/reverse wiring of each motor
  and to read encoder counts.

  Serial commands (send as lines):
    HELP                       - show this menu
    MOTOR,<idx>,FWD,<ms>       - run motor <idx> forward for <ms> milliseconds
    MOTOR,<idx>,REV,<ms>       - run motor <idx> reverse for <ms> milliseconds
    MOTORALL,FWD,<ms>          - run all motors forward for <ms>
    MOTORALL,REV,<ms>          - run all motors reverse for <ms>
    ENC,<idx>                  - print encoder count for motor <idx>
    ENC,ALL                    - print all encoder counts
    RST                        - reset encoder counts

  Motor index mapping: 0=FL, 1=FR, 2=BL, 3=BR
*/

// Pin definitions - match main controller
#define M_FL_IN1  22
#define M_FL_IN2  23
#define M_FR_IN1  24
#define M_FR_IN2  25
#define M_BL_IN1  26
#define M_BL_IN2  27
#define M_BR_IN1  28
#define M_BR_IN2  29

#define ENC_FL_A  18
#define ENC_FL_B  31
#define ENC_FR_A  19
#define ENC_FR_B  33
#define ENC_BL_A  20
#define ENC_BL_B  35
#define ENC_BR_A  21
#define ENC_BR_B  37

volatile long enc_counts[4] = {0,0,0,0};
long prev_enc_counts[4] = {0,0,0,0};

const int in1_pins[4] = {M_FL_IN1, M_FR_IN1, M_BL_IN1, M_BR_IN1};
const int in2_pins[4] = {M_FL_IN2, M_FR_IN2, M_BL_IN2, M_BR_IN2};

void setup() {
  Serial.begin(115200);
  while (!Serial) ;

  Serial.println("Motor Direction & Encoder Test Ready");
  Serial.println("Type HELP for commands");

  for (int i = 0; i < 4; ++i) {
    pinMode(in1_pins[i], OUTPUT);
    pinMode(in2_pins[i], OUTPUT);
    digitalWrite(in1_pins[i], LOW);
    digitalWrite(in2_pins[i], LOW);
  }

  pinMode(ENC_FL_A, INPUT_PULLUP);
  pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_BL_A, INPUT_PULLUP);
  pinMode(ENC_BL_B, INPUT_PULLUP);
  pinMode(ENC_BR_A, INPUT_PULLUP);
  pinMode(ENC_BR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isr_enc_fl, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isr_enc_fr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), isr_enc_bl, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), isr_enc_br, RISING);
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() == 0) return;
    handleCommand(s);
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.equalsIgnoreCase("HELP")) {
    Serial.println(F("Commands: HELP, MOTOR,<idx>,FWD,<ms>, MTOR,<idx>,REV,<ms>, MOTORALL,FWD,<ms>, MOTORALL,REV,<ms>, ENC,<idx>|ALL, RST"));
    return;
  }

  // Split on commas
  int parts[8];
  int pcount = 0;
  char buf[128];
  cmd.toCharArray(buf, sizeof(buf));
  char *tok = strtok(buf, ",");
  String tokens[8];
  while (tok && pcount < 8) {
    tokens[pcount++] = String(tok);
    tok = strtok(NULL, ",");
  }

  if (pcount == 0) return;
  String t0 = tokens[0];
  if (t0.equalsIgnoreCase("MOTOR") && pcount >= 4) {
    int idx = tokens[1].toInt();
    String dir = tokens[2];
    int ms = tokens[3].toInt();
    if (idx < 0 || idx > 3) { Serial.println("Invalid motor index"); return; }
    if (dir.equalsIgnoreCase("FWD")) runMotor(idx, true, ms);
    else if (dir.equalsIgnoreCase("REV")) runMotor(idx, false, ms);
    else Serial.println("Invalid direction");
    return;
  }

  if (t0.equalsIgnoreCase("MOTORALL") && pcount >= 3) {
    String dir = tokens[1];
    int ms = tokens[2].toInt();
    bool fwd = dir.equalsIgnoreCase("FWD");
    for (int i=0;i<4;i++) setMotorDir(i, fwd ? 1 : -1);
    delay(ms);
    stopAll();
    Serial.println("MOTORALL done");
    return;
  }

  if (t0.equalsIgnoreCase("ENC") && pcount >= 2) {
    if (tokens[1].equalsIgnoreCase("ALL")) {
      long c[4]; getEncoderCounts(c);
      Serial.print("ENCALL,");
      Serial.print(c[0]); Serial.print(","); Serial.print(c[1]); Serial.print(","); Serial.print(c[2]); Serial.print(","); Serial.println(c[3]);
      return;
    } else {
      int idx = tokens[1].toInt();
      if (idx < 0 || idx > 3) { Serial.println("Invalid encoder index"); return; }
      long c[4]; getEncoderCounts(c);
      Serial.print("ENC,"); Serial.print(idx); Serial.print(","); Serial.println(c[idx]);
      return;
    }
  }

  if (t0.equalsIgnoreCase("RST")) { resetEncoders(); Serial.println("ENC reset"); return; }

  Serial.println("Unknown command");
}

void runMotor(int idx, bool forward, int ms) {
  setMotorDir(idx, forward ? 1 : -1);
  delay(ms);
  setMotorDir(idx, 0);
  Serial.print("MOTOR "); Serial.print(idx); Serial.println(" done");
}

void setMotorDir(int idx, int dir) {
  if (dir > 0) {
    digitalWrite(in1_pins[idx], HIGH);
    digitalWrite(in2_pins[idx], LOW);
  } else if (dir < 0) {
    digitalWrite(in1_pins[idx], LOW);
    digitalWrite(in2_pins[idx], HIGH);
  } else {
    digitalWrite(in1_pins[idx], LOW);
    digitalWrite(in2_pins[idx], LOW);
  }
}

void stopAll() {
  for (int i=0;i<4;i++) setMotorDir(i, 0);
}

void getEncoderCounts(long *counts) {
  noInterrupts();
  for (int i=0;i<4;i++) counts[i] = enc_counts[i];
  interrupts();
}

void resetEncoders() {
  noInterrupts(); for (int i=0;i<4;i++) { enc_counts[i]=0; prev_enc_counts[i]=0; } interrupts();
}

// Encoder ISRs (2x decoding on rising edge of A)
void isr_enc_fl() { if (digitalRead(ENC_FL_B)) enc_counts[0]--; else enc_counts[0]++; }
void isr_enc_fr() { if (digitalRead(ENC_FR_B)) enc_counts[1]++; else enc_counts[1]--; }
void isr_enc_bl() { if (digitalRead(ENC_BL_B)) enc_counts[2]--; else enc_counts[2]++; }
void isr_enc_br() { if (digitalRead(ENC_BR_B)) enc_counts[3]++; else enc_counts[3]--; }
