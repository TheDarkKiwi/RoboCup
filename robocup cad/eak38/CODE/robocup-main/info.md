servo locations
544 = far back (push off)
1610 = flush / bypass


microseconds

without colour sensor
Task Timings:
sensors_tof_read: Min=6832 Max=11658 Avg=6874
sensors_tof_print: Min=9019 Max=9541 Avg=9024
motors_PID_drive: Min=0 Max=1 Avg=0
telem_read: Min=0 Max=1 Avg=0
motors_robot_servo_controller: Min=0 Max=1 Avg=0
sensors_ultrasonic_read: Min=12 Max=13 Avg=12
navigate_logic: Min=853 Max=1027 Avg=1007

// with colour sensor
Task Timings:
sensors_tof_read: Min=6833 Max=11657 Avg=6876
sensors_tof_print: Min=9020 Max=9024 Avg=9022
motors_PID_drive: Min=0 Max=1 Avg=0
telem_read: Min=0 Max=11 Avg=0
motors_robot_servo_controller: Min=0 Max=0 Avg=0
sensors_ultrasonic_read: Min=12 Max=13 Avg=12
navigate_logic: Min=52954 Max=52956 Avg=52955



fix

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
const int interruptPin = 2;
volatile boolean state = false;

const byte timeServoInterval = 6;
int timeSimUserInterval; // Set "do something" interval programmatically
unsigned long timeNowServo = 0;
unsigned long timeNowUser = 0;

int angleCurrent;
int angleTarget;

Servo ovres;

void isr()
{
  state = true;
}

/* tcs.getRawData() does a delay(Integration_Time) after the sensor readout.
  We don't need to wait for the next integration cycle because we receive an interrupt when the integration cycle is complete*/
void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

void setup()
{
  pinMode(interruptPin, INPUT_PULLUP); //TCS interrupt output is Active-LOW and Open-Drain
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr, FALLING);

  Serial.begin(19200);
  
  tcs.begin();
  tcs.write8(TCS34725_PERS, TCS34725_PERS_NONE); // Set persistence filter to generate an interrupt for every RGB Cycle, regardless of the integration limits
  tcs.setInterrupt(true);

  randomSeed(analogRead(0));
  ovres.attach(3);
  ovres.write(0);
  angleCurrent = ovres.read(); // Seed with initial value to start
  angleTarget = random(0, 180); // Seed with initial value to start
}

void loop()
{
  readSensor();
  simulateUserInput(); // Values (°) randomly generated within a range
  rotateServo();
}

void readSensor()
{
  if (state)
  {
    uint16_t r, g, b, c;
    getRawData_noDelay(&r, &g, &b, &c);

    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    Serial.println(" ");

    tcs.clearInterrupt();
    state = false;
  }
}

void rotateServo()
{
  if (millis() - timeNowServo >= timeServoInterval) // Check if it is time to rotate
  {
    timeNowServo = millis(); // Record the current time
    if (angleCurrent != angleTarget) // Don't write to servo if target angle is reached
    {
      if (angleCurrent <= angleTarget)
      {
        angleCurrent ++;
        ovres.write(angleCurrent);
      }
      else
      {
        if (angleCurrent >= angleTarget)
        {
          angleCurrent --;
          ovres.write(angleCurrent);
        }
      }
    }
  }
}

void simulateUserInput() // Set target angle programmatically (later through user input)
{
  if (millis() - timeNowUser > timeSimUserInterval)
  {
    timeNowUser = millis();
    timeSimUserInterval = random(1000, 8000);
    angleTarget = random(0, 180);
  }
}