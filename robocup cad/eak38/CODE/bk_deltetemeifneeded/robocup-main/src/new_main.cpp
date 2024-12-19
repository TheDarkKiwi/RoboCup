/*
This example shows how to set up and read multiple VL53L1X sensors connected to
the same I2C bus. Each sensor needs to have its XSHUT pin connected to a
different Arduino pin, and you should change sensorCount and the xshutPins array
below to match your setup.

For more information, see ST's application note AN4846 ("Using multiple VL53L0X
in a single design"). The principles described there apply to the VL53L1X as
well.
*/

#include <Wire.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>
#include "sensors.h"
#include "telem.h"
#include "motors.h"
#include <TaskScheduler.h>  





Scheduler taskManager;

Task Tsensors_tof_read(TASK_IMMEDIATE, TASK_FOREVER, &sensors_tof_read);
Task Tmotors_PID_drive(5, TASK_FOREVER, &motors_PID_drive);
Task Tmotors_print(50, TASK_FOREVER, &motors_print);
Task Ttelem_read(20, TASK_FOREVER, &telem_read);

void setup()
{
    telem_serial_init();
    sensors_init();
    motors_init();
    taskManager.init();
    //taskManager.addTask(Tsensors_tof_read);
    taskManager.addTask(Tmotors_PID_drive);
    taskManager.addTask(Tmotors_print);
    taskManager.addTask(Ttelem_read);
    taskManager.enableAll();

    
}

void loop()
{
    taskManager.execute();
}
