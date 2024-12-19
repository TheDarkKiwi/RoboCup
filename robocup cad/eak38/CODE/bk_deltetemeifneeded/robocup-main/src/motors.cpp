#include "motors.h"
#include "Arduino.h"
#include <Servo.h>

const float KP = 1;
const float KI = 0.000;
const float KD = 0;
const float ENCODER_COUNTS_PER_METER = 663.0 / (3.14159 * 62.5);

// Encoder positions
volatile int32_t encoderPosLeft = 0;
volatile int32_t encoderPosRight = 0;

// Speed calculation variables
int64_t leftMotorSpeed = 0;
int64_t rightMotorSpeed = 0;


int targetLeftSpeed = 0;
int targetRightSpeed = 0;


boolean A_set_left = false;
boolean B_set_left = false;
boolean A_set_right = false;
boolean B_set_right = false;



Servo RobotServo;  // create servo object to control robot servo
Servo LeftDrive;  // create servo object to control left drive motor
Servo RightDrive;  // create servo object to control right drive motor

#define SMOOTHING_FACTOR 0.01 // Alpha value for EMA, adjust as needed

void motors_init(void) {
  motors_encoder_init();
  RobotServo.attach(ROBOT_SERVO_PIN,ROBOT_SERVO_MIN,ROBOT_SERVO_MAX);
  LeftDrive.attach(LEFT_MOTOR_PIN,DRIVE_MOTOR_MAX_REVERSE,DRIVE_MOTOR_MAX);
  RightDrive.attach(RIGHT_MOTOR_PIN,DRIVE_MOTOR_MAX_REVERSE,DRIVE_MOTOR_MAX);
  Serial.println("Motors initialized \n");
}

void motors_encoder_init(void) {
  pinMode(encoder1PinA, INPUT);       //Set encoder pins as inputs
  pinMode(encoder1PinB, INPUT); 
  pinMode(encoder2PinA, INPUT); 
  pinMode(encoder2PinB, INPUT); 

  attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, CHANGE);  //Set up an interrupt for each encoder
  attachInterrupt(digitalPinToInterrupt(4), doEncoder2A, CHANGE);


  Serial.println("Encoders initialized \n");
}

void motors_print(void) {
  Serial7.printf("Left :%6lld->%6d Right :%6lld->%6d", leftMotorSpeed, targetLeftSpeed, rightMotorSpeed, targetRightSpeed);
  Serial7.println();

}


/* In this section, the motor speeds should be updated/written.
 *It is also a good idea to check whether value to write is valid.
 *It is also a good idea to do so atomically!
 */
void motors_drive(MotorSide motor, int16_t speed) {
  speed = map(speed, -100, 100, DRIVE_MOTOR_MAX_REVERSE, DRIVE_MOTOR_MAX);
  if (speed < DRIVE_MOTOR_MAX_REVERSE) {
    speed = DRIVE_MOTOR_MAX_REVERSE;
  } else if (speed > DRIVE_MOTOR_MAX) {
    speed = DRIVE_MOTOR_MAX;
  }
  if (motor == LEFT) {
    LeftDrive.writeMicroseconds(speed);
  } else if (motor == RIGHT) {
    RightDrive.writeMicroseconds(speed);
  }
}

void motors_PID_drive(void){
  static int32_t lastPosLeft = 0;
  static int32_t lastPosRight = 0;
  static int32_t outputLeft = 0;
  static int32_t outputRight = 0;

  // Calculate the speed of the motors
  int64_t rawLeftMotorSpeed = (encoderPosLeft - lastPosLeft) * MOTOR_PID_FREQUENCY;
  int64_t rawRightMotorSpeed = (encoderPosRight - lastPosRight) * MOTOR_PID_FREQUENCY;

  applyEMA(leftMotorSpeed, rawLeftMotorSpeed);
  applyEMA(rightMotorSpeed, rawRightMotorSpeed);

  leftMotorSpeed = rawLeftMotorSpeed;
  rightMotorSpeed = rawRightMotorSpeed;

  lastPosLeft = encoderPosLeft;
  lastPosRight = encoderPosRight;

  int32_t errorLeft = targetLeftSpeed - leftMotorSpeed;
  int32_t errorRight = targetRightSpeed - rightMotorSpeed;

  // Scale the constants based on the frequency factor
  float scaledKP = KP / MOTOR_PID_FREQUENCY;
  float scaledKI = KI / MOTOR_PID_FREQUENCY;
  float scaledKD = KD * MOTOR_PID_FREQUENCY;

  // Calculate the proportional term of the PID controller
  int32_t proportionalTermLeft = scaledKP * errorLeft;
  int32_t proportionalTermRight = scaledKP * errorRight;

  // Calculate the integral term of the PID controller
  static int32_t integralTermLeft = 0;
  static int32_t integralTermRight = 0;
  integralTermLeft += scaledKI * errorLeft;
  integralTermRight += scaledKI * errorRight;

  // Calculate the derivative term of the PID controller
  static int32_t previousErrorLeft = 0;
  static int32_t previousErrorRight = 0;
  int32_t derivativeTermLeft = scaledKD * (errorLeft - previousErrorLeft);
  int32_t derivativeTermRight = scaledKD * (errorRight - previousErrorRight);
  previousErrorLeft = errorLeft;
  previousErrorRight = errorRight;

  // Calculate the output of the PID controller
  outputLeft += proportionalTermLeft + integralTermLeft + derivativeTermLeft;
  outputRight -= proportionalTermRight + integralTermRight + derivativeTermRight;


  // Limit the output to the valid range for the motor driver
  outputLeft = constrain(outputLeft, -100, 100);
  outputRight = constrain(outputRight, -100, 100);


  if (outputLeft == 0 ){
    leftMotorSpeed = 0;
  }
  if (outputRight == 0 ){
    rightMotorSpeed = 0;
  }

  // Set the motor speeds based on the calculated outputs
  motors_drive(LEFT, outputLeft);
  motors_drive(RIGHT, outputRight);
}



void applyEMA(int64_t &smoothedSpeed, int64_t rawSpeed) {
    smoothedSpeed = SMOOTHING_FACTOR * rawSpeed + (1.0 - SMOOTHING_FACTOR) * smoothedSpeed;
}

void doEncoder1A() {

    A_set_left = digitalRead(encoder1PinA) == HIGH;
    B_set_left = digitalRead(encoder1PinB) == HIGH;

    if (A_set_left != B_set_left) {
        encoderPosLeft++;
    } else {
        encoderPosLeft--;
    }
}

void doEncoder2A() {
    A_set_right = digitalRead(encoder2PinA) == HIGH;
    B_set_right = digitalRead(encoder2PinB) == HIGH;

    if (A_set_right != B_set_right) {
        encoderPosRight++;
    } else {
        encoderPosRight--;
    }

}
