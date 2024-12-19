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

const byte TOF_SX1509_ADDRESS = 0x3F;
const byte AIO_SX1509_ADDRESS = 0x3E;
#define VL53L1X_ADDRESS_START 0x35
#define VL53L0X_ADDRESS_START 0x30

#define INDUCTIVE_PIN 25
#define BLUE_BUTTON_PIN 24


// The number of sensors in your system.
const uint8_t horizontalSensorCount = 3;
const uint8_t verticalSensorCount = 2;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL1[verticalSensorCount][horizontalSensorCount] = {{2,3,4},{5,6,7}};
const uint8_t xshutPinsL0[1] = {1};

const uint8_t weight_spad_horizontal = 1;
const uint8_t weight_spad_vertical = 2;
const uint8_t weight_spad_array[weight_spad_vertical] = {195,60};


// target weight is given first dimension is sensor number, second dimension is distance.
extern uint16_t target_weight[3][2];
extern volatile bool inductiveState;
extern volatile bool blueButtonState;
extern volatile uint16_t ultraSonicFrontLeft;
extern volatile uint16_t ultraSonicFrontRight;
extern volatile uint16_t ultraSonicBackLeft;
extern volatile uint16_t ultraSonicBackRight;
extern uint16_t TOFFrontValue;
extern uint16_t TOFBackValue;
extern uint16_t tof_sensor_values[verticalSensorCount][horizontalSensorCount][2];
extern bool homeGreen;

#define TARGET_WEIGHT_FILTER_SIZE 1

typedef enum {
    HOME = 1,
    ARENA,
    ENEMY
} colour_t;
#define COLOUR_THRESHOLD 0.8

#define AtrigPin 32
#define AechoPin 33

#define BtrigPin 30
#define BechoPin 31

#define CtrigPin 22
#define CechoPin 23

#define DtrigPin 20
#define DechoPin 21

void sensors_init(void);

void sensors_tof_init(void);

void sensors_tof_read(void);

void sensors_tof_print(void);

void sensors_inductive_init(void);

void sensors_blue_init(void);

void sensors_inductive_ISR(void);

void sensors_blue_ISR(void);

void sensors_ultrasonic_init(void);

void sensors_ultrasonic_read(void);

void sensors_navigation_print(void);

void sensors_ultrasonic_A_ISR(void);

void sensors_ultrasonic_B_ISR(void);

void sensors_ultrasonic_C_ISR(void);

void sensors_ultrasonic_D_ISR(void);

colour_t sensors_colour_read();

colour_t sensors_colour_read_auto_colour();

void sensors_colour_init();

#endif /* SENSORS_H_ */
