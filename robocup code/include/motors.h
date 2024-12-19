//************************************
//         motors.h    
//************************************

#ifndef MOTORS_H_
#define MOTORS_H_

#include <Servo.h>
#include <stdint.h>

#define OPEN 360
#define CLOSED 180


extern int targetLeftSpeed;
extern int targetRightSpeed;

extern int RobotServoGoal;
extern bool RobotServoGoalReached;


// Pin assignments and initializations
#define ROBOT_SERVO_PIN 8    //Pin corresponding to the robot servo
#define LEFT_MOTOR_PIN 0      //Pin corresponding to the left dc motor
#define RIGHT_MOTOR_PIN 1     //Pin corresponding to the right dc motor
#define ROBOT_SERVO_MIN 544   //Minimum value for the robot servo
#define ROBOT_SERVO_MAX 1920  //Maximum value for the robot servo
#define DRIVE_MOTOR_MAX_REVERSE 1010    //Minimum value for the left dc motor
#define DRIVE_MOTOR_STOP 1500  //Minimum value for the left dc motor
#define DRIVE_MOTOR_MAX 1990   //Maximum value for the left dc motor


#define DRIVE_SPEED 2400


// PID constants and other motor control variables

#define MOTOR_PID_FREQUENCY 50
#define MIN_UPDATE_TIME 0 //30000000
#define SMOOTHING_FACTOR 0.01 // Alpha value for EMA, adjust as needed

enum PinAssignments {
  encoder1PinA = 2,
  encoder1PinB = 3,
  
  encoder2PinA = 4,
  encoder2PinB = 5,
};

enum MotorSide {
  LEFT,
  RIGHT
};

enum RobotServoGoal {
  RELEASE = ROBOT_SERVO_MIN,
  CAPTURE = 1000,
  BYPASS = 1610,
  COLLECT = ROBOT_SERVO_MAX
};

void motors_init(void);

void motors_test(void);

void motors_encoder_init(void);

void motors_drive(MotorSide motor, int16_t speed);

void motors_PID_drive(void);

void doEncoder1A();

void doEncoder2A();

void applyEMA(int64_t &smoothedSpeed, int64_t rawSpeed);

void motors_print(void);

void motors_robot_servo_controller();

void motors_smart_servo_init();

bool motors_smart_servo_controller(int servoPos);





#endif /* MOTORS_H_ */
