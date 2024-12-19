#include "sensors.h"
#include "motors.h"
#include "navigate.h"

bool lastWallDir = LEFT;
uint8_t weightsQT = 0;
bool collection = false;

void navigate_logic()
{   
    if (weightsQT > 0 && sensors_colour_read() == HOME) {
        navigate_drop_weight();
    } else if (weightsQT == 3) {
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
    } else if((target_weight[0][0] == 1 || target_weight[1][0] == 1  ||target_weight[2][0] == 1 || collection || RobotServoGoal != BYPASS || inductiveState) && weightsQT < 3 && sensors_colour_read() == ARENA) {
        //navigate_to_weight();
        navigate_wall_follow(LEFT);
    } else if (tof_sensor_values[0][0][0] < WALL_FOLLOW_DISTANCE ||tof_sensor_values[0][1][0] < WALL_FOLLOW_DISTANCE || tof_sensor_values[0][2][0] < WALL_FOLLOW_DISTANCE) {
        if(lastWallDir == LEFT) {
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = -DRIVE_SPEED;
        } else {
            targetLeftSpeed = -DRIVE_SPEED;
            targetRightSpeed = DRIVE_SPEED;
        }
    } else if(ultraSonicFrontLeft < START_WALL_FOLLOW_DISTANCE || ultraSonicFrontRight < START_WALL_FOLLOW_DISTANCE || ultraSonicBackLeft < START_WALL_FOLLOW_DISTANCE || ultraSonicBackRight < START_WALL_FOLLOW_DISTANCE) {
        if(ultraSonicFrontLeft < ultraSonicFrontRight || ultraSonicBackLeft < ultraSonicBackRight) {
            lastWallDir = LEFT;
            navigate_wall_follow(LEFT);
        } else {
            lastWallDir = RIGHT;
            navigate_wall_follow(LEFT);
        }
    } else {
        targetLeftSpeed = DRIVE_SPEED;
        targetRightSpeed = DRIVE_SPEED;
    }
}


void navigate_to_weight()
{
    static uint32_t collectionTimePastSensor = 0;
    static uint32_t pastSensorThreshold = 100;
    static uint32_t collectionTimePastProx = 0;
    static uint32_t pastProxThreshold = 80;
    int error = 0;
    int distance_to_weight = 0;
    if (target_weight[0][0] == 1 && target_weight[1][0] == 1) {
        error = -1;
        distance_to_weight = (target_weight[0][1] + target_weight[1][1]) / 2;
    } else if (target_weight[0][0] == 1) {
        error = -2;
        distance_to_weight = target_weight[0][1];
    } else if (target_weight[2][0] ==1 && target_weight[1][0] == 1) {
        error = 1;
        distance_to_weight = (target_weight[2][1] + target_weight[1][1]) / 2;
    } else if (target_weight[2][0] == 1) {
        error = 2;
        distance_to_weight = target_weight[2][1];
    } else if (target_weight[1][0] == 1) {
        error = 0;
        distance_to_weight = target_weight[1][1];
    } else {
        distance_to_weight = 4000;
    }
    // PID signal to be removed from inside wheel
    // PID controller constants
    const u_int16_t Kp = 1000;  // Proportional gain 10
    const u_int16_t Ki = 0;  // Integral gain
    const u_int16_t Kd = 0;  // Derivative gain

    // PID controller variables
    static int32_t previous_error = 0;
    static int32_t integral = 0;

    // Calculate the PID signal
    integral += error;  // Update the integral term
    int32_t derivative = error - previous_error;  // Calculate the derivative term
    int32_t pid_signal = Kp * error + Ki * integral + Kd * derivative;  // Calculate the PID signal

    // Update the previous error for the next iteration
    previous_error = error;

    if (distance_to_weight < 150) {
        collection = true;
    } 
    if (RobotServoGoal == CAPTURE) {
        Serial7.println("CAPTURE");
        targetLeftSpeed = 200;
        targetRightSpeed = 200;
        if (RobotServoGoalReached || weightsQT >= 2) {
            RobotServoGoal = BYPASS;
            RobotServoGoalReached = false;
            collection = false;
            weightsQT += 1;
        }
    } else if (collectionTimePastProx > pastProxThreshold) {
        Serial7.println("COLLECT");
        RobotServoGoal = CAPTURE;
        RobotServoGoalReached = false;
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
        collectionTimePastProx = 0;

    } else if (RobotServoGoal == COLLECT) {
        Serial7.println("PRECOLLECT");
        if (RobotServoGoalReached) {
            collectionTimePastProx += 1;
            targetLeftSpeed = 600;
            targetRightSpeed = 600;

        } else {
            targetLeftSpeed = 0;
            targetRightSpeed = 0;

        }

    } else if (inductiveState) {
        Serial7.println("INDUCTIVE");
        RobotServoGoal = COLLECT;
        RobotServoGoalReached = false;
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
        collectionTimePastProx = 0;

    } else if (collection) {   
        Serial7.println("PREINDUCTIVE");
        // Apply the PID signal to the motors
        if (collectionTimePastSensor > pastSensorThreshold) {
            collection = false;
            collectionTimePastSensor = 0;

        } else { 
            targetLeftSpeed = 1100;
            targetRightSpeed = 1100;
            collectionTimePastSensor += 1;

        }
    } else {
        if (pid_signal > 0) {
            Serial7.println("PID LEFT");
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = DRIVE_SPEED - pid_signal;

        } else {
            Serial7.println("PID RIGHT");
            targetLeftSpeed = DRIVE_SPEED + pid_signal;
            targetRightSpeed = DRIVE_SPEED;
        }
    }
}

void navigate_wall_follow(bool left = LEFT)
{
    uint16_t ultraSonicFrontValue = 0;
    uint16_t ultraSonicBackValue = 0;
    if (left) {
        ultraSonicFrontValue = ultraSonicFrontLeft;
        ultraSonicBackValue = ultraSonicBackLeft;
    } else {
        ultraSonicFrontValue = ultraSonicFrontRight;
        ultraSonicBackValue = ultraSonicBackRight;
    }
    ultraSonicFrontValue =  constrain(ultraSonicFrontValue, 0, 600);
    ultraSonicBackValue = constrain(ultraSonicBackValue, 0, 600);

    // PID controller constants
    const u_int16_t KpDistance = 15;  // Proportional gain
    const u_int16_t KiDistance = 0;  // Integral gain
    const u_int16_t KdDistance = 0;  // Derivative gain

    const u_int16_t KpAngle = 18;  // Proportional gain
    const u_int16_t KiAngle = 0;  // Integral gain
    const u_int16_t KdAngle = 0;  // Derivative gain

    int distanceError = WALL_FOLLOW_DISTANCE - min(ultraSonicFrontValue, ultraSonicBackValue);

    static int32_t previousDistanceError = 0;
    static int32_t integralDistance = 0;

    integralDistance += distanceError;  // Update the integral term
    int32_t distanceDerivative = distanceError - previousDistanceError;  // Calculate the derivative term
    
    // Calculate the PID signal
    int distancePID = KpDistance * distanceError + KiDistance * integralDistance + KdDistance * distanceDerivative;  // Calculate the PID signal


    // Update the previous error for the next iteration
    previousDistanceError = distanceError;

    int angleError = ultraSonicFrontValue - ultraSonicBackValue;
    
    // PID controller variables
    static int32_t previousAngleError = 0;
    static int32_t integralAngle = 0;

    integralAngle += angleError;  // Update the integral term
    int32_t angleDerivative = angleError - previousAngleError;  // Calculate the derivative term

    // Calculate the PID signal
    int anglePID = KpAngle * angleError + KiAngle * integralAngle + KdAngle * angleDerivative;  // Calculate the PID signal

    // Update the previous error for the next iteration
    previousAngleError = angleError;
    
    anglePID = constrain(anglePID, -DRIVE_SPEED, DRIVE_SPEED);
    anglePID = constrain(anglePID - distancePID, - DRIVE_SPEED, DRIVE_SPEED);
    //Serial7.printf("Front: %d, Back: %d, Angle: %d PID: %d\n", ultraSonicFrontValue, ultraSonicBackValue, angleError, anglePID);
    if (left) {
        if (anglePID > 0) {
            targetLeftSpeed = DRIVE_SPEED - anglePID;
            targetRightSpeed = DRIVE_SPEED;
        } else {
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = DRIVE_SPEED + anglePID;
        }
    } else {
        if (anglePID > 0) {
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = DRIVE_SPEED - anglePID;
        } else {
            targetLeftSpeed = DRIVE_SPEED + anglePID;
            targetRightSpeed = DRIVE_SPEED;
        }
    }
}

void navigate_drop_weight()
{
    static int time = 0;

    if (!motors_smart_servo_controller(OPEN)) {
        return;
    }
    if (RobotServoGoal != RELEASE) {
        RobotServoGoal = RELEASE;
        RobotServoGoalReached = false;
    } else if (RobotServoGoalReached && time < 50) {
        time += 1;
    } else if (RobotServoGoalReached) {
        RobotServoGoal = BYPASS;
        RobotServoGoalReached = false;
        time = 0;
        motors_smart_servo_controller(CLOSED);
        weightsQT = 0;
    }
}