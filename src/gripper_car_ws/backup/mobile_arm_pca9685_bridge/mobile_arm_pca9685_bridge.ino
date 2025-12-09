#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const uint8_t SERVO1_CH = 0;
const uint8_t SERVO2_CH = 1;
const uint8_t SERVO3_CH = 2;
const uint8_t SERVO4_CH = 4;
const uint8_t SERVO5_CH = 5;
const uint8_t SERVO6_CH = 6;

struct ServoCal
{
  uint16_t minPulse;
  uint16_t maxPulse;
};

ServoCal servoCal[16];

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  servoCal[SERVO1_CH] = {150, 600};
  servoCal[SERVO2_CH] = {160, 610};
  servoCal[SERVO3_CH] = {150, 590};
  servoCal[SERVO4_CH] = {155, 600};
  servoCal[SERVO5_CH] = {200, 560};
  servoCal[SERVO6_CH] = {210, 580};

  Serial.println("#READY");
}

void loop()
{
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      return;
    }
    if (line.charAt(0) != '#') {
      return;
    }
    line.remove(0, 1);
    int colonIndex = line.indexOf(':');
    if (colonIndex < 0) {
      return;
    }
    int channel = line.substring(0, colonIndex).toInt();
    int value = line.substring(colonIndex + 1).toInt();
    if (channel < 0 || channel > 15) {
      return;
    }
    const ServoCal & cal = servoCal[channel];
    uint16_t pulse = constrain(value, cal.minPulse, cal.maxPulse);
    pwm.setPWM(channel, 0, pulse);
    Serial.print("#ACK:");
    Serial.print(channel);
    Serial.print(':');
    Serial.println(pulse);
  }
}
