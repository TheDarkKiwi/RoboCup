#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "watchdog.h"

extern Watchdog IMUWatchdog;
extern Watchdog RampWatchdog;


enum ramp_t {
    OFF,
    UP,
    DOWN,
    FLAT
};

extern ramp_t rampState;
void IMU_init(void);

void IMU_read(void);

float IMU_get_heading(void);

void IMU_update_watchdog(void);

void IMU_save_calibration(void);

void displaySensorDetails(void);

void displaySensorStatus(void);

void displayCalStatus(void);

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);

void IMU_check_ramp(void);


float IMU_get_roll(void);



#endif // IMU_H