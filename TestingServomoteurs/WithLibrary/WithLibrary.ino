#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);   // servo = 50 Hz
  delay(10);

  // center servo (CH0)
  pwm.setPWM(0, 0, 307);
}

void loop() {
  // left
  pwm.setPWM(0, 0, 205);
  delay(1000);

  // center
  pwm.setPWM(0, 0, 307);
  delay(1000);

  // right
  pwm.setPWM(0, 0, 410);
  delay(1000);
}