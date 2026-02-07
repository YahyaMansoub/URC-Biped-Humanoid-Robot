#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// 🔴 CHANGE ONLY THESE TWO VALUES
#define SERVO_CHANNEL 6
#define SERVO_PWM     290  // try 205 → 410

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(50);   // 50 Hz for servos
  delay(10);

  // Move selected servo to selected position
  pwm.setPWM(SERVO_CHANNEL, 0, SERVO_PWM);
}

void loop() {
  // intentionally empty
}
