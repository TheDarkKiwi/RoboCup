//************************************
//         sensors.cpp       
//************************************

 // This file contains functions used to read and average
 // the sensors.


#include "sensors.h"
#include <Wire.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

// Local definitions
//#define 

SX1509 io; // Create an SX1509 object to be used throughout
VL53L0X sensorsL0[sensorCount];
VL53L1X sensorsL1[sensorCount];

uint16_t tof_sensor_values[sensorCount][weight_spad_vertical][2] = {0};


void sensors_init(void){

  io.begin(SX1509_ADDRESS);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensors_tof_init();
}

void sensors_tof_init(void){
  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }

  // L0 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL0[i], HIGH);
    delay(10);

    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L0 ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);

    sensorsL0[i].startContinuous(50);
  }

  // L1 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);

    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);
    sensorsL1[i].setDistanceMode(VL53L1X::Long);
    sensorsL1[i].setMeasurementTimingBudget(20000);
    sensorsL1[i].setROISize(16,8);
    sensorsL1[i].setROICenter(weight_spad_array[0]);
    sensorsL1[i].readRangeSingleMillimeters(false);

    sensorsL1[i].startContinuous(50);
  }
}

void sensors_tof_read(void){
  static int vertical = 1;

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    if (sensorsL1[i].dataReady()){
      sensorsL1[i].read(false);
      if (sensorsL1[i].ranging_data.range_mm > 10000) {
        tof_sensor_values[i][vertical][0] = 2000;
      } else {
        tof_sensor_values[i][vertical][0] = sensorsL1[i].ranging_data.range_mm; 
      }
      tof_sensor_values[i][vertical][1] = sensorsL1[i].ranging_data.range_status;
      // if (i == 1 && vertical == 1 && horizontal == 1) {
      //   Serial7.print(sensorsL1[i].ranging_data.peak_signal_count_rate_MCPS);
      //   Serial7.print(" ");
      //   Serial7.print(sensorsL1[i].ranging_data.ambient_count_rate_MCPS);
      //   Serial7.print(" ");
      //   Serial7.print(sensorsL1[i].ranging_data.range_mm);
      //   Serial7.print(" ");
      //   Serial7.print(sensorsL1[i].ranging_data.range_status);
      //   Serial7.println();
      // }
    }
    sensorsL1[i].setROICenter(weight_spad_array[vertical]);

    sensorsL1[i].readRangeSingleMillimeters(false);
  }
  if (++vertical >= weight_spad_vertical) { vertical = 0;}
}

void sensors_tof_print(void){
  static bool printRaw = true;

  if (printRaw)
  {
    Serial7.print("Sensor ");
    Serial7.print(tof_sensor_values[1][0][0]);
    Serial7.print(" ");
    Serial7.print(tof_sensor_values[1][1][0]);
    Serial7.println();
  } else {
    Serial7.println("Sensors L1");
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        if ((tof_sensor_values[i][0][0] - tof_sensor_values[i][1][0]) < 300) {
          Serial7.print(0);
          Serial7.print(" ");
        } else {
          Serial7.print(1);
          Serial7.print(" ");
        }
    }
    Serial7.println();
  }
}

// Read ultrasonic value
void read_ultrasonic(/* Parameters */){
  Serial.println("Ultrasonic value \n");
}

// Read infrared value
void read_infrared(/* Parameters */){
  Serial.println("Infrared value \n");  
}

// Read colour sensor value
void read_colour(/* Parameters */){
  Serial.println("colour value \n");  
}

// Pass in data and average the lot
void sensor_average(/* Parameters */){
  Serial.println("Averaging the sensors \n");
}

