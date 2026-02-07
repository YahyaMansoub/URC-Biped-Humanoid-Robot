#include <Servo.h>

Servo servo;

// 🔴 CHANGE ONLY THIS VALUE
int angle = 100;

void setup() {
  servo.attach(2);   // 🔴 change pin if needed
  servo.write(angle);
}

void loop() {
  // nothing here on purpose
}

