#include "sbus.h"
#include <ESP32Servo.h>

/* SBUS object, reading SBUS */
bfs::SbusRx sbus(&Serial, 6, 5, true);
/* SBUS data */
bfs::SbusData data;

/* Motor controller pinouts.
    uses one PWM signal for forward and another for reverse on each motor. 
*/
const int MOTOR_1_FORWARD = 1;
const int MOTOR_1_REVERSE = 2;
const int MOTOR_2_FORWARD = 3;
const int MOTOR_2_REVERSE = 4;

const int SERVO_PIN = 8;
const int SERVO_MAX_RATE = 2250;
const int SERVO_MIN_RATE = 750;

const int DEADBAND = 20;

Servo flipperServo = Servo();

void setup() {
  /* Begin the SBUS communication */
  sbus.Begin();
  pinMode(MOTOR_1_FORWARD, OUTPUT);
  pinMode(MOTOR_2_FORWARD, OUTPUT);
  pinMode(MOTOR_1_REVERSE, OUTPUT);
  pinMode(MOTOR_2_REVERSE, OUTPUT);
  flipperServo.attach(SERVO_PIN);
}

void loop () {
  if(sbus.Read()){
    data = sbus.data();

    // Check for failsafe

    if(data.failsafe){
      // Failsafe tripped, set all outputs low
      analogWrite(MOTOR_1_FORWARD, 0);
      analogWrite(MOTOR_1_REVERSE, 0);
      analogWrite(MOTOR_2_FORWARD, 0);
      analogWrite(MOTOR_2_REVERSE, 0);
      analogWrite(SERVO_PIN, 750);
    } else {
      // Failsafe passed, run robot code
      double rawAxial = 0;
      rawAxial = data.ch[0];
      // default channel range is 172-1811, map to +- 250
      int axial = map(rawAxial, 172, 1811, -250, 250);

      double rawYaw = 0;
      rawYaw = data.ch[1];
      // default channel range is 172-1811, map to +- 250
      int yaw = map(rawYaw, 172, 1811, -250, 250);

      // Apply deadbands
      if(abs(axial) < DEADBAND){
        axial = 0;
      }
      if(abs(yaw) < DEADBAND){
        yaw = 0;
      }

      int leftMotorPower = axial+yaw;
      int rightMotorPower = axial-yaw;

      int rawServo = data.ch[2];
      int servoPos = map(rawServo, 172, 1811, SERVO_MIN_RATE, SERVO_MAX_RATE);
      flipperServo.writeMicroseconds(servoPos);

      // Write to the motor channels
      // Use max(a,b) to make sure that we only feed positive numbers to each channel
      analogWrite(MOTOR_1_FORWARD, max(0, leftMotorPower*-1));
      analogWrite(MOTOR_1_REVERSE, max(0, leftMotorPower));
      analogWrite(MOTOR_2_FORWARD, max(0, rightMotorPower));
      analogWrite(MOTOR_2_REVERSE, max(0, rightMotorPower*-1));
    }
  }
  
}