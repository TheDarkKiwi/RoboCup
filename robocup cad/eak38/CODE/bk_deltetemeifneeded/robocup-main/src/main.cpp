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
#include <navigate.h>
#include <limits.h> // Include this header for ULONG_MAX


Scheduler taskManager;

Task Tsensors_tof_read(80, TASK_FOREVER, &sensors_tof_read);
Task Tsensors_tof_print(80, TASK_FOREVER, &sensors_tof_print);
Task Tmotors_PID_drive(5, TASK_FOREVER, &motors_PID_drive);
Task Tmotors_print(50, TASK_FOREVER, &motors_print);
Task Ttelem_read(20, TASK_FOREVER, &telem_read);
Task Tmotors_robot_servo_controller(100, TASK_FOREVER, &motors_robot_servo_controller);
Task Tsensors_ultrasonic_read(20, TASK_FOREVER, &sensors_ultrasonic_read);
Task Tsensors_navigation_print(50, TASK_FOREVER, &sensors_navigation_print);
Task Tnavigate_logic(20, TASK_FOREVER, &navigate_logic);
Task Tnavigate_wall_follow(20, TASK_FOREVER, &navigate_wall_follow);


void setup()
{
    telem_serial_init();
    sensors_init();
    motors_init();
    taskManager.init();
    taskManager.addTask(Tsensors_tof_read);
    //taskManager.addTask(Tsensors_tof_print);
    taskManager.addTask(Tmotors_PID_drive);
    //taskManager.addTask(Tmotors_print);
    taskManager.addTask(Ttelem_read);
    taskManager.addTask(Tmotors_robot_servo_controller);
    taskManager.addTask(Tnavigate_logic);
    taskManager.addTask(Tsensors_ultrasonic_read);
    //taskManager.addTask(Tsensors_navigation_print);
    taskManager.enableAll();
    //motors_test();
    while (!motors_smart_servo_controller(CLOSED))
    {
       continue;
    }
}

void loop()
{
    taskManager.execute();
}


// task timings

struct TaskTiming {
    const char* name;
    unsigned long minTime;
    unsigned long maxTime;
    unsigned long totalTime;
    unsigned long count;
};



TaskTiming taskTimings[] = {
    {"sensors_tof_read", ULONG_MAX, 0, 0, 0},
    {"sensors_tof_print", ULONG_MAX, 0, 0, 0},
    {"motors_PID_drive", ULONG_MAX, 0, 0, 0},
    {"telem_read", ULONG_MAX, 0, 0, 0},
    {"motors_robot_servo_controller", ULONG_MAX, 0, 0, 0},
    {"sensors_ultrasonic_read", ULONG_MAX, 0, 0, 0},
    {"navigate_logic", ULONG_MAX, 0, 0, 0}
};

void updateTaskTiming(TaskTiming& timing, unsigned long duration) {
    if (duration < timing.minTime) timing.minTime = duration;
    if (duration > timing.maxTime) timing.maxTime = duration;
    timing.totalTime += duration;
    timing.count++;
}

void printTaskTimings() {
    Serial.println("Task Timings:");
    for (const auto& timing : taskTimings) {
        if (timing.count > 0) {
            Serial.print(timing.name);
            Serial.print(": Min=");
            Serial.print(timing.minTime);
            Serial.print(" Max=");
            Serial.print(timing.maxTime);
            Serial.print(" Avg=");
            Serial.println(timing.totalTime / timing.count);
        }
    }
}

void executeTask(void (*taskFunction)(), TaskTiming& timing) {
    unsigned long start = micros();
    taskFunction();
    unsigned long duration = micros() - start;
    updateTaskTiming(timing, duration);
}
/*
void loop() {
    executeTask(sensors_tof_read, taskTimings[0]);
    executeTask(sensors_tof_print, taskTimings[1]);
    executeTask(motors_PID_drive, taskTimings[2]);
    executeTask(telem_read, taskTimings[3]);
    executeTask(motors_robot_servo_controller, taskTimings[4]);
    executeTask(sensors_ultrasonic_read, taskTimings[5]);
    executeTask(navigate_logic, taskTimings[6]);
    printTaskTimings();
    delay(1000); // Adjust delay as needed
}
*/
