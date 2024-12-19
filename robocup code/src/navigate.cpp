#include "sensors.h"
#include "motors.h"
#include "navigate.h"
#include "IMU.h"
#include "watchdog.h"
#include "telem.h"

int lastWallDir = WALL_FOLLOW_LEFT;
uint8_t weightsQT = 0;
bool collection = false;
bool ifwatchdog = false;

Watchdog navigateSearchWatchdog = Watchdog(8000);
Watchdog homeWatchdog = Watchdog(20000);

void navigate_logic()
{
    static ramp_t lastRampState = OFF;
    if (rampState == OFF || rampState != lastRampState) {
        RampWatchdog.reset();
    } else if (RampWatchdog.isExpired()) {
        ifwatchdog = true;
        rampState = OFF;
        RampWatchdog.reset();
    }
    lastRampState = rampState;
    
    colour_t colour = sensors_colour_read();

    if (weightsQT < 3 || colour == HOME) {
        homeWatchdog.reset();
    }
    if(IMUWatchdog.isExpired()|| ifwatchdog) {
        if (debug) {
            Serial7.println("IMU Watchdog Triggered");
        }
        navigate_IMU_watchdog();
        navigateSearchWatchdog.reset();
    } else if (colour == HOME && weightsQT > 0) {
        navigate_drop_weight();
        navigateSearchWatchdog.reset();
    } else if(rampState != OFF && !collection) {
        navigate_ramp();
        navigateSearchWatchdog.reset();
        IMUWatchdog.reset();
    } else if(homeWatchdog.isExpired()) {
        navigate_heading(180);
    } else if((target_weight[0][0] == 1 || target_weight[1][0] == 1  ||target_weight[2][0] == 1 || collection || RobotServoGoal != BYPASS || inductiveState) && weightsQT < 3 && colour == ARENA) {
        navigateSearchWatchdog.reset();
        navigate_to_weight();

    } else if(navigateSearchWatchdog.isExpired()) {
        if (debug) {
            Serial7.println("Search Watchdog Triggered");
        }
        navigate_search_watchdog();

    } else if (tof_sensor_values[0][0][0] < WALL_FOLLOW_DISTANCE ||tof_sensor_values[0][1][0] < WALL_FOLLOW_DISTANCE || tof_sensor_values[0][2][0] < WALL_FOLLOW_DISTANCE) {
        if(lastWallDir == WALL_FOLLOW_LEFT) {
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = -DRIVE_SPEED;
        } else {
            targetLeftSpeed = -DRIVE_SPEED;
            targetRightSpeed = DRIVE_SPEED;
        }

    } else if(ultraSonicFrontLeft < START_WALL_FOLLOW_DISTANCE || ultraSonicFrontRight < START_WALL_FOLLOW_DISTANCE || ultraSonicBackLeft < START_WALL_FOLLOW_DISTANCE || ultraSonicBackRight < START_WALL_FOLLOW_DISTANCE) {
        if(ultraSonicFrontLeft < ultraSonicFrontRight || ultraSonicBackLeft < ultraSonicBackRight) {
            lastWallDir = WALL_FOLLOW_LEFT;
            navigate_wall_follow(WALL_FOLLOW_LEFT);
        } else {
            lastWallDir = WALL_FOLLOW_RIGHT;
            navigate_wall_follow(WALL_FOLLOW_RIGHT);
        }

    } else {
        targetLeftSpeed = DRIVE_SPEED;
        targetRightSpeed = DRIVE_SPEED;
    }
    if (weightsQT >= 3 || colour != ARENA) {
        navigateSearchWatchdog.reset();
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
    RampWatchdog.reset();
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
        if (debug) {
            Serial7.println("CAPTURE");
        }
        targetLeftSpeed = 200;
        targetRightSpeed = 200;
        if (RobotServoGoalReached || weightsQT >= 2) {
            RobotServoGoal = BYPASS;
            RobotServoGoalReached = false;
            collection = false;
            weightsQT += 1;
        }
    } else if (collectionTimePastProx > pastProxThreshold) {
        if (debug) {
            Serial7.println("COLLECT");
        }
        RobotServoGoal = CAPTURE;
        RobotServoGoalReached = false;
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
        collectionTimePastProx = 0;

    } else if (RobotServoGoal == COLLECT) {
        if (debug) {
            Serial7.println("PRECOLLECT");
        }
        if (RobotServoGoalReached) {
            collectionTimePastProx += 1;
            targetLeftSpeed = 600;
            targetRightSpeed = 600;

        } else {
            targetLeftSpeed = 0;
            targetRightSpeed = 0;

        }

    } else if (inductiveState) {
        if (debug) {
            Serial7.println("INDUCTIVE");
        }
        IMUWatchdog.reset();
        RobotServoGoal = COLLECT;
        RobotServoGoalReached = false;
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
        collectionTimePastProx = 0;

    } else if (collection) {
        if (debug) {
            Serial7.println("PREINDUCTIVE");
        }
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
            if (debug) {
            Serial7.println("PID Weight Left");
            }
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = DRIVE_SPEED - pid_signal;

        } else {
            if (debug) {
            Serial7.println("PID Weight Right");
            }
            targetLeftSpeed = DRIVE_SPEED + pid_signal;
            targetRightSpeed = DRIVE_SPEED;
        }
    }
}

void navigate_wall_follow(int wallFollowDir = WALL_FOLLOW_LEFT)
{
    uint16_t ultraSonicFrontValue = 0;
    uint16_t ultraSonicBackValue = 0;
    if (wallFollowDir == WALL_FOLLOW_LEFT) {
        ultraSonicFrontValue = ultraSonicFrontLeft;
        ultraSonicBackValue = ultraSonicBackLeft;
    } else {
        ultraSonicFrontValue = ultraSonicFrontRight;
        ultraSonicBackValue = ultraSonicBackRight;
    }
    ultraSonicFrontValue =  constrain(ultraSonicFrontValue, 0, START_WALL_FOLLOW_DISTANCE);
    ultraSonicBackValue = constrain(ultraSonicBackValue, 0, START_WALL_FOLLOW_DISTANCE);

    // PID controller constants
    const u_int16_t KpDistance = 15;  // Proportional gain 15, 11
    const u_int16_t KiDistance = 0;  // Integral gain
    const u_int16_t KdDistance = 0;  // Derivative gain

    const u_int16_t KpAngle = 18;  // Proportional gain 18, 7
    const u_int16_t KiAngle = 0;  // Integral gain
    const u_int16_t KdAngle = 0;  // Derivative gain

    //int distanceError = WALL_FOLLOW_DISTANCE - min(ultraSonicFrontValue, ultraSonicBackValue);
    int distanceError = WALL_FOLLOW_DISTANCE - ((ultraSonicFrontValue + ultraSonicBackValue)/2);

    static int32_t previousDistanceError = 0;
    static int32_t integralDistance = 0;

    integralDistance += distanceError;  // Update the integral term
    int32_t distanceDerivative = distanceError - previousDistanceError;  // Calculate the derivative term
    
    // Calculate the PID signal
    int distancePID = KpDistance * distanceError + KiDistance * integralDistance + KdDistance * distanceDerivative;  // Calculate the PID signal


    // Update the previous error for the next iteration
    previousDistanceError = distanceError;

    int angleError = ultraSonicBackValue - ultraSonicFrontValue;
    
    // PID controller variables
    static int32_t previousAngleError = 0;
    static int32_t integralAngle = 0;

    integralAngle += angleError;  // Update the integral term
    int32_t angleDerivative = angleError - previousAngleError;  // Calculate the derivative term

    // Calculate the PID signal
    int anglePID = KpAngle * angleError + KiAngle * integralAngle + KdAngle * angleDerivative;  // Calculate the PID signal

    // Update the previous error for the next iteration
    previousAngleError = angleError;
    if (debug) {
        Serial7.printf("Front: %d, Back: %d, Angle PID: %d Distance PID: %d\n", ultraSonicFrontValue, ultraSonicBackValue, anglePID, distancePID);
    }
    anglePID = constrain(anglePID, -DRIVE_SPEED, DRIVE_SPEED);
    distancePID = constrain(distancePID, -DRIVE_SPEED, DRIVE_SPEED);
    int totalPID = constrain(anglePID + distancePID, int(-1.3*DRIVE_SPEED), int(1.3*DRIVE_SPEED));
    if (debug) {
        Serial7.printf("Front: %d, Back: %d, Angle PID: %d Distance PID: %d total: %d constrained\n", ultraSonicFrontValue, ultraSonicBackValue, anglePID, distancePID, totalPID);
    }
    if (wallFollowDir == WALL_FOLLOW_RIGHT) {
        if (debug) {
            Serial7.println("Wall Follow Right");
        }
        if (totalPID > 0) {
            targetLeftSpeed = DRIVE_SPEED - totalPID;
            targetRightSpeed = DRIVE_SPEED;
        } else {
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = DRIVE_SPEED + totalPID;
        }
    } else {
        if (debug) {
            Serial7.println("Wall Follow Left");
        }
        if (totalPID > 0) {
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = DRIVE_SPEED - totalPID;
        } else {
            targetLeftSpeed = DRIVE_SPEED + totalPID;
            targetRightSpeed = DRIVE_SPEED;
        }
    }
}

void navigate_drop_weight()
{
    static bool INHOME = false; 
    static bool DROPPING = false;
    static int time = 0;
    int distance_to_wall_front = min(tof_sensor_values[0][0][0], tof_sensor_values[0][1][0]);
    distance_to_wall_front = min(distance_to_wall_front, tof_sensor_values[0][2][0]);
    if (DROPPING) {
        if (!motors_smart_servo_controller(OPEN)) {
            return;
        }
        if (RobotServoGoal != RELEASE) {
            RobotServoGoal = RELEASE;
            RobotServoGoalReached = false;
        } else if (RobotServoGoalReached && time < 100) {
            time += 1;
        } else if (RobotServoGoalReached) {
            RobotServoGoal = BYPASS;
            RobotServoGoalReached = false;
            time = 0;
            motors_smart_servo_controller(CLOSED);
            weightsQT = 0;
            DROPPING = false;
        }
    } else {
        if (!INHOME) {
            if (distance_to_wall_front > 200) {
                targetLeftSpeed = DRIVE_SPEED;
                targetRightSpeed = DRIVE_SPEED;
            } else {
                targetLeftSpeed = 0;
                targetRightSpeed = 0;
                INHOME = true; 
            } 
        } else {
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = -DRIVE_SPEED;
            if (distance_to_wall_front > 600) {
                targetLeftSpeed = 0;
                targetRightSpeed = 0;
                DROPPING = true;
                INHOME = false;
            }
        }
    }   
}

void navigate_IMU_watchdog()
{
    ifwatchdog = true;
    static unsigned long lastToggleTime = 0;
    static int state = 0; // 0: reversing, 1: turning, 2: driving forward

    unsigned long currentTime = millis();
    if (currentTime - lastToggleTime >= 2000) {
        state = (state + 1) % 3;
        lastToggleTime = currentTime;
    }

    switch (state) {
        case 0: // reversing
            targetLeftSpeed = -DRIVE_SPEED;
            targetRightSpeed = -DRIVE_SPEED;
            break;
        case 1: // turning
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = -DRIVE_SPEED;
            break;
        case 2:
            ifwatchdog = false;
            lastToggleTime = 0;
            break;
    }
}

void navigate_search_watchdog()
{
    static unsigned long lastToggleTime = 0;
    static int state = 0; // 0: turning in one direction, 1: turning in the opposite direction, 2: reset

    unsigned long currentTime = millis();
    if (currentTime - lastToggleTime >= 4000) {
        state = (state + 1) % 3; // Change to % 2 to ensure only two states
        lastToggleTime = currentTime;
    }


    switch (state) {
        case 0: // turning in one direction
            if (lastWallDir == WALL_FOLLOW_LEFT) {
                targetLeftSpeed = DRIVE_SPEED / 2;
                targetRightSpeed = -DRIVE_SPEED / 2;
            } else {
                targetLeftSpeed = -DRIVE_SPEED / 2;
                targetRightSpeed = DRIVE_SPEED / 2;
            }
            break;
        case 1: // turning in the opposite direction
            if (lastWallDir == WALL_FOLLOW_LEFT) {
                targetLeftSpeed = -DRIVE_SPEED / 2;
                targetRightSpeed = DRIVE_SPEED / 2;
            } else {
                targetLeftSpeed = DRIVE_SPEED / 2;
                targetRightSpeed = -DRIVE_SPEED / 2;
            }
            break;
        case 2: // reset
            navigateSearchWatchdog.reset();
            break;
    }
}

void navigate_wall_follow_test() {
    if (tof_sensor_values[0][0][0] < WALL_FOLLOW_DISTANCE ||tof_sensor_values[0][1][0] < WALL_FOLLOW_DISTANCE || tof_sensor_values[0][2][0] < WALL_FOLLOW_DISTANCE) {
        if(lastWallDir == WALL_FOLLOW_LEFT) {
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = -DRIVE_SPEED;
            if (debug) {
                Serial7.println("Avoid Left");
            }
        } else {
            targetLeftSpeed = -DRIVE_SPEED;
            targetRightSpeed = DRIVE_SPEED;
            if (debug) {
                Serial7.println("Avoid Right");
            }
        }
    } else if(ultraSonicFrontLeft < START_WALL_FOLLOW_DISTANCE || ultraSonicFrontRight < START_WALL_FOLLOW_DISTANCE || ultraSonicBackLeft < START_WALL_FOLLOW_DISTANCE || ultraSonicBackRight < START_WALL_FOLLOW_DISTANCE) {
        if(ultraSonicFrontLeft < ultraSonicFrontRight || ultraSonicBackLeft < ultraSonicBackRight) {
            lastWallDir = WALL_FOLLOW_LEFT;
            navigate_wall_follow(WALL_FOLLOW_LEFT);
        } else {
            lastWallDir = WALL_FOLLOW_RIGHT;
            navigate_wall_follow(WALL_FOLLOW_RIGHT);
        }
    } else {
        if (debug) {
            Serial7.println("Driving Forward");
        }
        targetLeftSpeed = 500;
        targetRightSpeed = 500;
    }
}

void navigate_ramp(void){
    int error = 0;
    if (inductiveState && weightsQT < 3) {
        RobotServoGoal = COLLECT;
        RobotServoGoalReached = false;
        collection = true;
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
        return;
    }
    switch (rampState) {
        case UP:
            if (error > 1)
            {
                targetLeftSpeed = DRIVE_SPEED/2;
                targetRightSpeed = -300;
            }
            else if (error < -1)
            {
                targetLeftSpeed = -300;
                targetRightSpeed = DRIVE_SPEED/2;
            } else {
                targetLeftSpeed = DRIVE_SPEED/2;
                targetRightSpeed = DRIVE_SPEED/2;
            }
        case DOWN:
            targetLeftSpeed = DRIVE_SPEED/2;
            targetRightSpeed = DRIVE_SPEED/2;
            break;
        case FLAT:
            error = IMU_get_roll();
            if (error > 1)
            {
                targetLeftSpeed = -DRIVE_SPEED/2;
                targetRightSpeed = DRIVE_SPEED/2;
            }
            else if (error < -1)
            {
                targetLeftSpeed = DRIVE_SPEED/2;
                targetRightSpeed = -DRIVE_SPEED/2;
            } else {
                targetLeftSpeed = DRIVE_SPEED/2;
                targetRightSpeed = DRIVE_SPEED/2;
            }
            break;
        case OFF:
            break;
    }
}

void navigate_heading(float goalHeading) {
    float heading = IMU_get_heading();

    if (goalHeading > 360) {
        goalHeading = goalHeading - 360;
    } else if (goalHeading < 0) {
        goalHeading = goalHeading + 360;
    }
    float error = goalHeading - heading;

    if (error >= 180 && error < 360) {
        error = error - 360;
    } else if (error < -180 && error >= -360) {
        error = error + 360;
    }
    if (error > 0) {
        targetLeftSpeed = DRIVE_SPEED/2;
        targetRightSpeed = -DRIVE_SPEED/2;
    } else {
        targetLeftSpeed = -DRIVE_SPEED/2;
        targetRightSpeed = DRIVE_SPEED/2;
    }
    if (abs(error) < 5) {
        homeWatchdog.reset();
    }
    
}

void navigate_home() {
    float heading = IMU_get_heading();
    int headingTolerance = 10;
    int headingGoal = 180;
    int wallFollowDir = 0;
    if (homeGreen) {
        wallFollowDir = WALL_FOLLOW_LEFT;
    } else {
        wallFollowDir = WALL_FOLLOW_RIGHT;
    }
    if (tof_sensor_values[0][0][0] < WALL_FOLLOW_DISTANCE ||tof_sensor_values[0][1][0] < WALL_FOLLOW_DISTANCE || tof_sensor_values[0][2][0] < WALL_FOLLOW_DISTANCE) {
        if(wallFollowDir == WALL_FOLLOW_LEFT && ultraSonicBackRight > START_WALL_FOLLOW_DISTANCE || wallFollowDir == WALL_FOLLOW_RIGHT && ultraSonicBackLeft < START_WALL_FOLLOW_DISTANCE) {
            targetLeftSpeed = DRIVE_SPEED;
            targetRightSpeed = -DRIVE_SPEED;
        } else {
            targetLeftSpeed = -DRIVE_SPEED;
            targetRightSpeed = DRIVE_SPEED;
        }
    } else if (ultraSonicFrontLeft < START_WALL_FOLLOW_DISTANCE && ultraSonicBackLeft < START_WALL_FOLLOW_DISTANCE && ultraSonicBackRight < START_WALL_FOLLOW_DISTANCE) {
        lastWallDir = WALL_FOLLOW_RIGHT;
        navigate_wall_follow(WALL_FOLLOW_RIGHT);
    } else if (ultraSonicFrontRight < START_WALL_FOLLOW_DISTANCE && ultraSonicBackLeft < START_WALL_FOLLOW_DISTANCE && ultraSonicBackRight < START_WALL_FOLLOW_DISTANCE) {
        lastWallDir = WALL_FOLLOW_LEFT;
        navigate_wall_follow(WALL_FOLLOW_LEFT);

    } else if ((heading < 180 - headingTolerance && ultraSonicBackRight > START_WALL_FOLLOW_DISTANCE && ultraSonicFrontRight > START_WALL_FOLLOW_DISTANCE) || (heading > 180 + headingTolerance && ultraSonicBackLeft > START_WALL_FOLLOW_DISTANCE && ultraSonicFrontLeft > START_WALL_FOLLOW_DISTANCE)) {
        navigate_heading(180);
    } else if(ultraSonicFrontLeft < START_WALL_FOLLOW_DISTANCE || ultraSonicFrontRight < START_WALL_FOLLOW_DISTANCE || ultraSonicBackLeft < START_WALL_FOLLOW_DISTANCE || ultraSonicBackRight < START_WALL_FOLLOW_DISTANCE) {
        if(ultraSonicFrontLeft < ultraSonicFrontRight || ultraSonicBackLeft < ultraSonicBackRight) {
            lastWallDir = WALL_FOLLOW_LEFT;
            navigate_wall_follow(WALL_FOLLOW_LEFT);
        } else {
            lastWallDir = WALL_FOLLOW_RIGHT;
            navigate_wall_follow(WALL_FOLLOW_RIGHT);
        }
    } else if (heading < headingGoal - headingTolerance && heading > headingGoal + headingTolerance) {
        navigate_heading(headingGoal);
    } else {
        targetLeftSpeed = DRIVE_SPEED;
        targetRightSpeed = DRIVE_SPEED; 
            
    }
}