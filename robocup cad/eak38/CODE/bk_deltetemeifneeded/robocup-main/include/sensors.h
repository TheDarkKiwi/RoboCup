//************************************
//         sensors.h     
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

#include <Wire.h>
#include <SPI.h>
#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <VL53L1X.h>

const byte SX1509_ADDRESS = 0x3F;
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35


// The number of sensors in your system.
const uint8_t sensorCount = 3;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL0[3] = {2,3,4};
const uint8_t xshutPinsL1[3] = {5,6,7};

const uint8_t weight_spad_horizontal = 1;
const uint8_t weight_spad_vertical = 2;
const uint8_t weight_spad_array[weight_spad_vertical] = {195,60};


void sensors_init(void);

void sensors_tof_init(void);

void sensors_tof_read(void);

// Read ultrasonic value
void read_ultrasonic(/* Parameters */);

// Read infrared value
void read_infrared(/* Parameters */);

void read_colour(/* Parameters */);

// Pass in data and average the lot
void sensor_average(/* Parameters */);

#endif /* SENSORS_H_ */
