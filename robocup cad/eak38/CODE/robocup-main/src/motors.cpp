#include "motors.h"
#include "Arduino.h"
#include <Servo.h>
#include "sensors.h"
#include <SoftwareSerial.h>
#include <HerkulexServo.h>

const float KP = 0.08;
const float KI = 2;
const float KD = 0;
const float ENCODER_COUNTS_PER_METER = 663.0 / (3.14159 * 62.5);
const int MAX_INTEGRAL = 450;

HerkulexServoBus herkulex_bus(Serial3);
HerkulexServo smartServo(herkulex_bus, 4);

// Encoder positions
volatile int32_t encoderPosLeft = 0;
volatile int32_t encoderPosRight = 0;


boolean A_set_left = false;
boolean B_set_left = false;
boolean A_set_right = false;
boolean B_set_right = false;

// Speed calculation variables
int64_t leftMotorSpeed = 0;
int64_t rightMotorSpeed = 0;

int targetLeftSpeed = 0;
int targetRightSpeed = 0;


int RobotServoGoal = BYPASS;
bool RobotServoGoalReached = false;

Servo RobotServo;  // create servo object to control robot servo
Servo LeftDrive;  // create servo object to control left drive motor
Servo RightDrive;  // create servo object to control right drive motor

void motors_init(void) {
  motors_encoder_init();
  motors_smart_servo_init();
  RobotServo.attach(ROBOT_SERVO_PIN, ROBOT_SERVO_MIN, ROBOT_SERVO_MAX);
  LeftDrive.attach(LEFT_MOTOR_PIN, DRIVE_MOTOR_MAX_REVERSE, DRIVE_MOTOR_MAX);
  RightDrive.attach(RIGHT_MOTOR_PIN, DRIVE_MOTOR_MAX_REVERSE, DRIVE_MOTOR_MAX);
}

void motors_test(void) {
  RightDrive.writeMicroseconds(1070);
  LeftDrive.writeMicroseconds(1920);
}

void motors_encoder_init(void) {
  pinMode(encoder1PinA, INPUT);       // Set encoder pins as inputs
  pinMode(encoder1PinB, INPUT); 
  pinMode(encoder2PinA, INPUT); 
  pinMode(encoder2PinB, INPUT); 

  attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, CHANGE);  // Set up an interrupt for each encoder
  attachInterrupt(digitalPinToInterrupt(4), doEncoder2A, CHANGE);

}

void motors_print(void) {
  Serial7.printf("Left :%6lld->%6d Right :%6lld->%6d", leftMotorSpeed, targetLeftSpeed, rightMotorSpeed, targetRightSpeed);
  Serial7.println();
}

/* In this section, the motor speeds should be updated/written.
 * It is also a good idea to check whether the value to write is valid.
 * It is also a good idea to do so atomically!
 */
void motors_drive(MotorSide motor, int16_t speed) {
  if (!blueButtonState){
    speed = 0;
  }
  static int16_t currentLeftSpeed = DRIVE_MOTOR_STOP;
  static int16_t currentRightSpeed = DRIVE_MOTOR_STOP;

  speed += DRIVE_MOTOR_STOP;
  speed = constrain(speed, DRIVE_MOTOR_MAX_REVERSE, DRIVE_MOTOR_MAX);

  if (motor == LEFT) {
    if (speed != currentLeftSpeed) {
      //noInterrupts(); // Disable interrupts
      LeftDrive.writeMicroseconds(speed);
      //interrupts(); // Enable interrupts
      currentLeftSpeed = speed;
    }
  } else if (motor == RIGHT) {
    if (speed != currentRightSpeed) {
      //noInterrupts(); // Disable interrupts
      RightDrive.writeMicroseconds(speed);
      //interrupts(); // Enable interrupts
      currentRightSpeed = speed;
    }
  }
}

void motors_PID_drive(void) {
  static int32_t lastPosLeft = 0;
  static int32_t lastPosRight = 0;
  static int32_t outputLeft = 0;
  static int32_t outputRight = 0;

  // Calculate the speed of the motors
  int32_t rawLeftMotorSpeed = (encoderPosLeft - lastPosLeft) * MOTOR_PID_FREQUENCY;
  int32_t rawRightMotorSpeed = (encoderPosRight - lastPosRight) * MOTOR_PID_FREQUENCY;

  //applyEMA(leftMotorSpeed, rawLeftMotorSpeed);
  //applyEMA(rightMotorSpeed, rawRightMotorSpeed);


  leftMotorSpeed = rawLeftMotorSpeed;
  rightMotorSpeed = rawRightMotorSpeed;

  lastPosLeft = encoderPosLeft;
  lastPosRight = encoderPosRight;

  int32_t errorLeft = targetLeftSpeed - leftMotorSpeed;
  int32_t errorRight = targetRightSpeed - rightMotorSpeed;

  // Scale the constants based on the frequency factor
  float scaledKP = KP;
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
  integralTermLeft = constrain(integralTermLeft, -MAX_INTEGRAL, MAX_INTEGRAL);
  integralTermRight = constrain(integralTermRight, -MAX_INTEGRAL, MAX_INTEGRAL);

  // Calculate the derivative term of the PID controller
  static int32_t previousErrorLeft = 0;
  static int32_t previousErrorRight = 0;
  int32_t derivativeTermLeft = scaledKD * (errorLeft - previousErrorLeft);
  int32_t derivativeTermRight = scaledKD * (errorRight - previousErrorRight);
  previousErrorLeft = errorLeft;
  previousErrorRight = errorRight;

  // Calculate the output of the PID controller
  outputLeft = proportionalTermLeft + integralTermLeft + derivativeTermLeft;
  outputRight = proportionalTermRight + integralTermRight + derivativeTermRight;

  constrain(outputLeft, DRIVE_MOTOR_MAX_REVERSE - DRIVE_MOTOR_STOP, DRIVE_MOTOR_MAX - DRIVE_MOTOR_STOP);
  constrain(outputRight, DRIVE_MOTOR_MAX_REVERSE - DRIVE_MOTOR_STOP, DRIVE_MOTOR_MAX - DRIVE_MOTOR_STOP);

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



void motors_robot_servo_controller() {
  static int servoPos = BYPASS;
  if (RobotServoGoal != servoPos) {
    RobotServoGoalReached = false;
    if (RobotServoGoal < servoPos - 50) {
      servoPos = servoPos - 50;
      RobotServo.writeMicroseconds(servoPos);
    } else if (RobotServoGoal > servoPos + 50) {
      servoPos = servoPos + 50;
      RobotServo.writeMicroseconds(servoPos);
    } else {
      RobotServo.writeMicroseconds(RobotServoGoal);
      RobotServoGoalReached = true;
    }
  } else {
    RobotServo.writeMicroseconds(RobotServoGoal);
    RobotServoGoalReached = true;
  }
}

void motors_smart_servo_init() {
  Serial3.begin(115200);         //reboot motor
  delay(500);
}

bool motors_smart_servo_controller(int pos = OPEN) {
  const int TOLERANCE = 5; // Define a tolerance value
  if (abs(smartServo.getPosition() - pos) <= TOLERANCE) {
    smartServo.setTorqueOff();
    herkulex_bus.update();
    return true;
  } else {
    smartServo.setTorqueOn();
    herkulex_bus.prepareIndividualMove();
    smartServo.setPosition(pos, 50);
    herkulex_bus.executeMove();
    herkulex_bus.update();
    return false;
  }
}
