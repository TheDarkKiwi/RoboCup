#include "telem.h"
#include <Arduino.h>
#include <motors.h>

bool debug = false;

char cmd[100];
int cmdIndex;
int action;
int goal;


void telem_serial_init(void)
{
  Serial7.begin(115200);
  delay(500);
}



void telem_read(void)
{
    while (Serial7.available())
    {
    char c = (char)Serial7.read();
    if(c=='\r' || c=='\n') {
      cmd[cmdIndex] = 0;
      cmdIndex = 0;
      execute(cmd);
    } else {      
      cmd[cmdIndex++] = c;
    }
  }
}

void execute(char* cmd) {
  
  // cmd example: "R 2200" => Slider id = "R", value = 2200

  if(cmd[0] == 'L') action = 0;
  else if(cmd[0] == 'R') action = 1;
  else if(cmd[0] == 'W') action = 2;
  else return; // unknown command

  if(cmd[1] != ' ') return; // unknown command

  // get integer number after first 2 characters:
  goal = atoi(cmd+2); 

  if (action == 0) {
    targetLeftSpeed = goal;
  } else if (action == 1) {
    targetRightSpeed = goal;
  } else if (action == 2) {
    if (goal == 0) {
      RobotServoGoal = BYPASS;
    } else if (goal == 2) {
      RobotServoGoal = CAPTURE;
    } else if (goal == 1) {
      RobotServoGoal = COLLECT;
    } else if (goal == 3) {
    RobotServoGoal = RELEASE;
    }
  }
}