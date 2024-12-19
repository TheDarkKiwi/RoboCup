#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "IMU.h"
#include "watchdog.h"
#include <EEPROM.h>
#include "telem.h"

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
Watchdog IMUWatchdog= Watchdog(8000);
Watchdog RampWatchdog = Watchdog(5000);


ramp_t rampState = OFF;

void IMU_init() {
{
    if (debug) {
        Serial7.println("Orientation Sensor Test");
        Serial7.println("");
    }

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        if (debug) {
            Serial7.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        }
        while (1);
    }

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        if (debug) {
            Serial7.println("\nNo Calibration Data for this sensor exists in EEPROM");
        }
        delay(500);
    }
    else
    {
        if (debug) {
            Serial7.println("\nFound Calibration for this sensor in EEPROM.");
        }
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);
        calibrationData.accel_offset_x = 0;
        calibrationData.gyro_offset_x = 0;
        displaySensorOffsets(calibrationData);

        if (debug) {
            Serial7.println("\n\nRestoring Calibration data to the BNO055...");
        }
        bno.setSensorOffsets(calibrationData);

        if (debug) {
            Serial7.println("\n\nCalibration data loaded into BNO055");
        }
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

   /* Crystal must be configured AFTER loading calibration data into BNO055. */
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    /* always recal the mag as It goes out of calibration very often */
    if (!foundCalib){
        if (debug) {
            Serial7.println("Please Calibrate Sensor: ");
        }
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            if (debug) {
                Serial7.print("X: ");
                Serial7.print(event.orientation.x, 4);
                Serial7.print("\tY: ");
                Serial7.print(event.orientation.y, 4);
                Serial7.print("\tZ: ");
                Serial7.print(event.orientation.z, 4);
            }

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            if (debug) {
                Serial7.println("");
            }

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    if (debug) {
        Serial7.println("\nFully calibrated!");
        Serial7.println("--------------------------------");
        Serial7.println("Calibration Results: ");
    }
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    if (debug) {
        Serial7.println("\n\nStoring calibration data to EEPROM...");
    }

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    if (debug) {
        Serial7.println("Data stored to EEPROM.");
        Serial7.println("\n--------------------------------\n");
    }
    delay(500);
    bno.setMode(OPERATION_MODE_NDOF_FMC_OFF);
}

}

void IMU_read() {
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    if (debug) {
        Serial7.print("accel: ");
        Serial7.print(accelerometerData.acceleration.x);
        Serial7.print(" ");
        Serial7.print(accelerometerData.acceleration.y);
        Serial7.print(" ");
        Serial7.print(accelerometerData.acceleration.z);
        Serial7.println();
    }
}

float IMU_get_heading() {
    static bool first = true;
    static float offset = 0;
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    float heading = orientationData.orientation.x;
    if (first) {
        offset = heading;
        first = false;
    }
    if (heading - offset < 0) {
        return heading - offset + 360;
    }
    return heading - offset;
}

void IMU_update_watchdog() {
    float heading = IMU_get_heading();
    static float lastHeading = heading;
    if (abs(heading - lastHeading) > 3) {
        if (debug) {
            Serial7.println("IMU Watchdog Reset");
        }
        IMUWatchdog.reset();
    }
    lastHeading = heading;
}

void IMU_save_calibration() {
    uint8_t calibData[22];
    bno.getSensorOffsets(calibData);
    if (debug) {
        for (int i = 0; i < 22; i++) {
            Serial7.print(calibData[i], HEX);
            Serial7.print(" ");
        }
        Serial7.println();
    }
}

void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    if (debug) {
        Serial7.println("------------------------------------");
        Serial7.print("Sensor:       "); Serial7.println(sensor.name);
        Serial7.print("Driver Ver:   "); Serial7.println(sensor.version);
        Serial7.print("Unique ID:    "); Serial7.println(sensor.sensor_id);
        Serial7.print("Max Value:    "); Serial7.print(sensor.max_value); Serial7.println(" xxx");
        Serial7.print("Min Value:    "); Serial7.print(sensor.min_value); Serial7.println(" xxx");
        Serial7.print("Resolution:   "); Serial7.print(sensor.resolution); Serial7.println(" xxx");
        Serial7.println("------------------------------------");
        Serial7.println("");
    }
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial7 Monitor */
    if (debug) {
        Serial7.println("");
        Serial7.print("System Status: 0x");
        Serial7.println(system_status, HEX);
        Serial7.print("Self Test:     0x");
        Serial7.println(self_test_results, HEX);
        Serial7.print("System Error:  0x");
        Serial7.println(system_error, HEX);
        Serial7.println("");
    }
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    if (debug) {
        Serial7.print("\t");
        if (!system)
        {
            Serial7.print("! ");
        }

        /* Display the individual values */
        Serial7.print("Sys:");
        Serial7.print(system, DEC);
        Serial7.print(" G:");
        Serial7.print(gyro, DEC);
        Serial7.print(" A:");
        Serial7.print(accel, DEC);
        Serial7.print(" M:");
        Serial7.print(mag, DEC);
    }
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    if (debug) {
        Serial7.print("Accelerometer: ");
        Serial7.print(calibData.accel_offset_x); Serial7.print(" ");
        Serial7.print(calibData.accel_offset_y); Serial7.print(" ");
        Serial7.print(calibData.accel_offset_z); Serial7.print(" ");

        Serial7.print("\nGyro: ");
        Serial7.print(calibData.gyro_offset_x); Serial7.print(" ");
        Serial7.print(calibData.gyro_offset_y); Serial7.print(" ");
        Serial7.print(calibData.gyro_offset_z); Serial7.print(" ");

        Serial7.print("\nMag: ");
        Serial7.print(calibData.mag_offset_x); Serial7.print(" ");
        Serial7.print(calibData.mag_offset_y); Serial7.print(" ");
        Serial7.print(calibData.mag_offset_z); Serial7.print(" ");

        Serial7.print("\nAccel Radius: ");
        Serial7.print(calibData.accel_radius);

        Serial7.print("\nMag Radius: ");
        Serial7.print(calibData.mag_radius);
    }
}

void IMU_check_ramp() {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    if (orientationData.orientation.y > 7) {
        rampState = UP;
    } else if (orientationData.orientation.y < -5) {
        rampState = DOWN;
    } else {
        if (rampState == UP || rampState == FLAT) {
            rampState = FLAT;


        } else {
            rampState = OFF;
        }
    }
}

float IMU_get_roll() {
    static bool first = true;
    static float offset = 0;
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    float roll = orientationData.orientation.z;
    Serial7.printf("Roll: %.2f offset: %.2f\n", roll, offset);
    
    if (roll < 0) {
        roll += 360;
    }
    if (first) {
        offset = roll;
        first = false;
    }

    return roll - offset;
}
