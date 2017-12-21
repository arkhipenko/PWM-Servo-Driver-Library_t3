#include <PWMServoDriver_t3.h>

#define Tilt  0
#define Yaw   3

// create servo object to control a servo
PWMServoDriverT3 pwm = PWMServoDriverT3(0x40, 2);

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println("16 channel Servo test!");

  pwm.begin();

  pwm.setPWMFreq(60);
}

void loop() {


  Serial.println("Test 1");
  pwm.setPWM(Tilt, 0, 120);
  delay(40);
  pwm.setPWM(Yaw, 0, 140);
  delay(2000);

  Serial.println("Test 2");
  pwm.setPWM(Tilt, 0, 580);
  delay(40);
  pwm.setPWM(Yaw, 0, 460);
  delay(2000);

  Serial.println("Test 3");
  pwm.setPWM(Tilt, 0, 350);
  delay(40);
  pwm.setPWM(Yaw, 0, 300);
  delay(2000);
  
  while (1);
}

