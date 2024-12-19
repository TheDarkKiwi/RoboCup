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
#include <Adafruit_TCS34725.h>

SX1509 TOF_io; // Create an SX1509 object to be used throughout
VL53L1X sensorsL1[verticalSensorCount][horizontalSensorCount];
VL53L0X sensorsL0[1];
uint16_t TOFBackValue = 0;

uint16_t tof_sensor_values[verticalSensorCount][horizontalSensorCount][2] = {0};
float tof_sensor_amb_peak[verticalSensorCount][horizontalSensorCount][2] = {0};

uint16_t target_weight[3][2] = {0};
uint8_t target_weight_buffer[3] = {0};

uint16_t ultraSonicFrontLeft = 0;
uint16_t ultraSonicFrontRight = 0;
uint16_t ultraSonicBackLeft = 0;
uint16_t ultraSonicBackRight = 0;

//SX1509 AIO_io;
bool inductiveState = false;
bool blueButtonState = false;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void sensors_init(void){
  //AIO_io.begin(AIO_SX1509_ADDRESS);
  TOF_io.begin(TOF_SX1509_ADDRESS);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  Wire1.begin();
  Wire1.setClock(400000); // use 400 kHz I2C

  sensors_tof_init();
  sensors_inductive_init();
  sensors_blue_init();
  sensors_ultrasonic_init();
  sensors_colour_init();
}

void sensors_tof_init(void){
  // Disable/reset all sensors by driving their XSHUT pins low.

  // L0 XSHUT pins
  for (uint8_t i = 0; i < 1; i++)
  {
    TOF_io.pinMode(xshutPinsL0[i], OUTPUT);
    TOF_io.digitalWrite(xshutPinsL0[i], LOW);
  }

  // L1 XSHUT pins
  for (uint8_t i = 0; i < verticalSensorCount; i++)
  {
    for (uint8_t j = 0; j < horizontalSensorCount; j++)
    {
      TOF_io.pinMode(xshutPinsL1[i][j], OUTPUT);
      TOF_io.digitalWrite(xshutPinsL1[i][j], LOW);
    }
  }
  // L0 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < 1; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    TOF_io.digitalWrite(xshutPinsL0[i], HIGH);
    delay(10);

    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init())
    {
      Serial7.print("Failed to detect and initialize sensor L0 ");
      Serial7.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);
    sensorsL0[i].setMeasurementTimingBudget(20000);
    sensorsL0[i].startContinuous();
  }

  // L1 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < verticalSensorCount; i++)
  {
    for (uint8_t j = 0; j < horizontalSensorCount; j++)
    {
      // Stop driving this sensor's XSHUT low. This should allow the carrier
      // board to pull it high. (We do NOT want to drive XSHUT high since it is
      // not level shifted.) Then wait a bit for the sensor to start up.
      //pinMode(xshutPins[i], INPUT);
      TOF_io.digitalWrite(xshutPinsL1[i][j], HIGH);
      delay(10);

      sensorsL1[i][j].setTimeout(500);
      if (!sensorsL1[i][j].init())
      {
        Serial7.print("Failed to detect and initialize sensor L1 ");
        Serial7.println(i,j);
        while (1);
      }

      // Each sensor must have its address changed to a unique value other than
      // the default of 0x29 (except for the last one, which could be left at
      // the default). To make it simple, we'll just count up from 0x2A.
      sensorsL1[i][j].setAddress(VL53L1X_ADDRESS_START + i * horizontalSensorCount + j);
      sensorsL1[i][j].setDistanceMode(VL53L1X::Short);
      sensorsL1[i][j].setMeasurementTimingBudget(50000);
      if (i == 0) {
        //sensorsL1[i][j].setROISize(16,4);
        //sensorsL1[i][j].setROICenter(193);
      } else {
      sensorsL1[i][j].setROISize(10,16);
      sensorsL1[i][j].setROICenter(199);
      }
      sensorsL1[i][j].readRangeSingleMillimeters(false);
    }
  }
}

void sensors_tof_read(void){
  // Read new values
  TOFBackValue = sensorsL0[0].readRangeContinuousMillimeters();

  for (uint8_t i = 0; i < verticalSensorCount; i++)
  {
    for (uint8_t j = 0; j < horizontalSensorCount; j++)
    {
      if (sensorsL1[i][j].dataReady()) {
        sensorsL1[i][j].read(false);
        tof_sensor_values[i][j][0] = sensorsL1[i][j].ranging_data.range_mm;
        tof_sensor_values[i][j][1] = sensorsL1[i][j].ranging_data.range_status;
        tof_sensor_amb_peak[i][j][0] = sensorsL1[i][j].ranging_data.ambient_count_rate_MCPS;
        tof_sensor_amb_peak[i][j][1] = sensorsL1[i][j].ranging_data.peak_signal_count_rate_MCPS;
        if ((sensorsL1[i][j].ranging_data.peak_signal_count_rate_MCPS < 0.45 && i == 1))
        {
          tof_sensor_values[i][j][0] = 4000;
        }

      } else {
        tof_sensor_values[i][j][1] = 20;
        Serial7.println("Data not ready");
      }
    }
  }
  for (uint8_t i = 0; i < verticalSensorCount; i++)
  {
    for (uint8_t j = 0; j < horizontalSensorCount; j++)
    {
      sensorsL1[i][j].readRangeSingleMillimeters(false);
    }
  }

  for (uint8_t i = 0; i < horizontalSensorCount; i++) {
    if (((tof_sensor_values[0][i][0] - tof_sensor_values[1][i][0]) > 200)) {
      target_weight_buffer[i] += 1;
      if (target_weight_buffer[i] >= TARGET_WEIGHT_FILTER_SIZE) {
        target_weight[i][0] = 1;
        target_weight[i][1] = tof_sensor_values[1][i][0];

        target_weight_buffer[i] = TARGET_WEIGHT_FILTER_SIZE;
      }
    } else {
      target_weight[i][0] = 0;
      target_weight[i][1] = 0;
      if (target_weight_buffer[i] > 0) {
        target_weight_buffer[i] = 0;
      }
    }
  }
}

void sensors_tof_print(void){
  static bool printRaw = true;

  if (printRaw)
  {
    for (uint8_t i = 0; i < verticalSensorCount; i++)
    {
      for (uint8_t j = 0; j < horizontalSensorCount; j++)
      {
        Serial7.printf("%5d,", tof_sensor_values[i][j][0]);
        Serial7.printf("%5d ", tof_sensor_values[i][j][1]);
        Serial7.printf("%5.2f,", tof_sensor_amb_peak[i][j][0]);
        Serial7.printf("%5.2f ", tof_sensor_amb_peak[i][j][1]);
      }
      Serial7.println();
    }
  } else {
    for (uint8_t i = 0; i < horizontalSensorCount; i++)
    {
      if (target_weight[i][0] == 1) {
        Serial7.printf("Sensor %d: %d\n", i, target_weight[i][1]);
        Serial7.println();
    }
    }
  }
}
void sensors_inductive_init(){
  pinMode(INDUCTIVE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INDUCTIVE_PIN),sensors_inductive_ISR, CHANGE);
}
void sensors_inductive_ISR(){
  inductiveState = (!digitalRead(INDUCTIVE_PIN));
}
void sensors_blue_init(){
  pinMode(BLUE_BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BLUE_BUTTON_PIN), sensors_blue_ISR, CHANGE);
}

void sensors_blue_ISR(){
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 500) {
    blueButtonState = !blueButtonState;
  }

  last_interrupt_time = interrupt_time;
}

void sensors_ultrasonic_init(){
  pinMode(AtrigPin, OUTPUT);
  pinMode(AechoPin, INPUT);
  pinMode(BtrigPin, OUTPUT);
  pinMode(BechoPin, INPUT);
  pinMode(CtrigPin, OUTPUT);
  pinMode(CechoPin, INPUT);
  pinMode(DtrigPin, OUTPUT);
  pinMode(DechoPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(AechoPin),sensors_ultrasonic_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BechoPin),sensors_ultrasonic_B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CechoPin),sensors_ultrasonic_C_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DechoPin),sensors_ultrasonic_D_ISR, CHANGE);
}

void sensors_ultrasonic_A_ISR(){
  static uint32_t start = 0;
  if (digitalRead(AechoPin) == HIGH) {
    start = micros();
  } else {
    uint32_t duration = micros() - start;
    if (duration != 0) {
      ultraSonicFrontLeft = duration/6;
    } else {
      ultraSonicFrontLeft = 0; // Handle division by zero
    }
  }
}
void sensors_ultrasonic_B_ISR(){
  static uint32_t start = 0;
  if (digitalRead(BechoPin) == HIGH) {
    start = micros();
  } else {
    uint32_t duration = micros() - start;
    if (duration != 0) {
      ultraSonicFrontRight = duration/6;
    } else {
      ultraSonicFrontRight = 0; // Handle division by zero
    }
  }
}

void sensors_ultrasonic_C_ISR(){
  static uint32_t start = 0;
  if (digitalRead(CechoPin) == HIGH) {
    start = micros();
  } else {
    uint32_t duration = micros() - start;
    if (duration != 0) {
      ultraSonicBackLeft = duration/6;
    } else {
      ultraSonicBackLeft = 0; // Handle division by zero
    }
  }
}

void sensors_ultrasonic_D_ISR(){
  static uint32_t start = 0;
  if (digitalRead(DechoPin) == HIGH) {
    start = micros();
  } else {
    uint32_t duration = micros() - start;
    if (duration != 0) {
      ultraSonicBackRight = duration/6;
    } else {
      ultraSonicBackRight = 0; // Handle division by zero
    }
  }
}

// Read ultrasonic value
void sensors_ultrasonic_read(){
  // Trigger the ultrasonic sensors
  // Trigger all ultrasonic sensors at once
  static uint8_t sensorIndex = 0;

  // Ensure all trigger pins are low
  delayMicroseconds(2);

  // Trigger the current sensor
  switch (sensorIndex) {
    case 0:
      digitalWrite(AtrigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(AtrigPin, LOW);
      break;
    case 1:
      digitalWrite(BtrigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(BtrigPin, LOW);
      break;
    case 2:
      digitalWrite(CtrigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(CtrigPin, LOW);
      break;
    case 3:
      digitalWrite(DtrigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(DtrigPin, LOW);
      break;
  }

  // Move to the next sensor for the next call
  sensorIndex = (sensorIndex + 1) % 4;
}

void sensors_navigation_print(){
  Serial7.printf("Front L: %4d, R: %4d\nBack L: %4d, R: %4d\n", ultraSonicFrontLeft, ultraSonicFrontRight, ultraSonicBackLeft, ultraSonicBackRight);
}


void sensors_colour_init(){
  if (tcs.begin(TCS34725_ADDRESS,&Wire1)) {
    Serial7.println("Found sensor");
  } else {
    Serial7.println("No TCS34725 found ... check your connections");
  }


}

void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

colour_t sensors_colour_read(){
  static bool first = true;
  static bool second = false;
  static float Fr = -1, Fg = -1, Fb = -1, Sr = -1, Sg = -1, Sb = -1;
  float r, g, b;
  static colour_t colour = HOME;
  uint16_t clear, red, green, blue;
  static int time;
  const int intergtationTime = 80/20; // 80ms integration time / 20ms loop time

  if (time < intergtationTime) {
    time++;
    return colour;
  } else {
    time = 0;
  }

  getRawData_noDelay(&red, &green, &blue, &clear);
  uint32_t sum = clear;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  

  if (first) {
    if (Fr == -1 && Fg == -1 && Fb == -1){
      Fr = r;
      Fg = g;
      Fb = b;
    }
    if (r < Fr + COLOUR_THRESHOLD && r > Fr - COLOUR_THRESHOLD && g < Fg + COLOUR_THRESHOLD && g > Fg - COLOUR_THRESHOLD && b < Fb + COLOUR_THRESHOLD && b > Fb - COLOUR_THRESHOLD) {
      colour = HOME;
      return HOME;
    } else {
      first = false;
      second = true;
    }
  }
  if (second) {
    if (Sr == -1 && Sg == -1 && Sb == -1){
      Sr = r;
      Sg = g;
      Sb = b;
    }
    if (r < Sr + COLOUR_THRESHOLD && r > Sr - COLOUR_THRESHOLD && g < Sg + COLOUR_THRESHOLD && g > Sg - COLOUR_THRESHOLD && b < Sb + COLOUR_THRESHOLD && b > Sb - COLOUR_THRESHOLD) {
      colour = ARENA;
      return ARENA;
    } else {
      second = false;
    }
  }

  if (!first && !second) {
    if (r < Fr + COLOUR_THRESHOLD && r > Fr - COLOUR_THRESHOLD && g < Fg + COLOUR_THRESHOLD && g > Fg - COLOUR_THRESHOLD && b < Fb + COLOUR_THRESHOLD && b > Fb - COLOUR_THRESHOLD) {
      colour = HOME;
      return HOME;
    } else if (r < Sr + COLOUR_THRESHOLD && r > Sr - COLOUR_THRESHOLD && g < Sg + COLOUR_THRESHOLD && g > Sg - COLOUR_THRESHOLD && b < Sb + COLOUR_THRESHOLD && b > Sb - COLOUR_THRESHOLD) {
      colour = ARENA;
      return ARENA;
    } else {
      colour = ENEMY;
      return ENEMY;
    }
  }
  return colour;
}