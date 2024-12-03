#include <Servo.h>

#define SERVO_1_PIN 3
#define SERVO_2_PIN 5
#define SERVO_3_PIN 6
#define SERVO_4_PIN 9
#define SERVO_CENTER_1 90
#define SERVO_CENTER_2 22
#define SERVO_CENTER_3 40
#define SERVO_CENTER_4 90
#define DELAY_TIME 1000

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int jointAngles[4] = {0, 19, 106, 0};

void setup() {

  Serial.begin(9600);
  Serial.println("Initializing Servos");
  servo1.attach(SERVO_1_PIN);
//  servo1.write(SERVO_CENTER_1);
//  servo1.write(90);
  servo2.attach(SERVO_2_PIN);
//  servo2.write(SERVO_CENTER_2);
//  servo2.write(41);
  servo3.attach(SERVO_3_PIN);
//  servo3.write(SERVO_CENTER_3);
//  servo3.write(146);
  servo4.attach(SERVO_4_PIN);
//  servo4.write(SERVO_CENTER_4);
//  servo4.write(90);
  Serial.println("Initialization Complete");
//  delay(5000);
//  Serial.println("Setting Servo Angles");
  setServoAngles(jointAngles);
  delay(1000);
}

void loop() {
//  Serial.println("HERE");
//  setServoAngles(jointAngles);
//  delay(DELAY_TIME);
//  servo1.write(SERVO_CENTER_1);
//  servo2.write(SERVO_CENTER_2);
//  servo3.write(SERVO_CENTER_3);
//  servo4.write(SERVO_CENTER_4);
}

void setServoAngles(int jointAngles[4]) {
  int writtenAngle1 = SERVO_CENTER_1 + jointAngles[0];
  int writtenAngle2 = SERVO_CENTER_2 + jointAngles[1];
  int writtenAngle3 = SERVO_CENTER_3 + jointAngles[2];
  int writtenAngle4 = SERVO_CENTER_4 + jointAngles[3];
//  servo1.write(90);
//  servo2.write(41);
//  servo3.write(146);
//  servo4.write(90);
  if(0 <= writtenAngle1 <= 180) {
    servo1.write(SERVO_CENTER_1 + jointAngles[0]);
    Serial.println("Writing " + String(writtenAngle1) + " to Servo 1");
  }
  if(0 <= writtenAngle2 <= 180) {
    servo1.write(SERVO_CENTER_2 + jointAngles[1]);
    Serial.println("Writing " + String(writtenAngle2) + " to Servo 2");
  }
  if(0 <= writtenAngle3 <= 180) {
    servo1.write(SERVO_CENTER_3 + jointAngles[2]);
    Serial.println("Writing " + String(writtenAngle3) + " to Servo 3");
  }
  if(0 <= writtenAngle4 <= 180) {
    servo1.write(SERVO_CENTER_4 + jointAngles[3]);
    Serial.println("Writing " + String(writtenAngle4) + " to Servo 4");
  }
}
