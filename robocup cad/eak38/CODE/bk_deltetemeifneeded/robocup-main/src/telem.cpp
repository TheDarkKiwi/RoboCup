#include "telem.h"
#include <Arduino.h>
#include <motors.h>

char cmd[100];
int cmdIndex;
int motor;
int speed;


void telem_serial_init(void)
{
    while (!Serial7) {}
    Serial7.begin(115200);
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

  if(cmd[0] == 'L') motor = 0;
  else if(cmd[0] == 'R') motor = 1;
  else return; // unknown command

  if(cmd[1] != ' ') return; // unknown command

  // get integer number after first 2 characters:
  speed = atoi(cmd+2); // Changed from cmd+4 to cmd+2

  // -2200 to 2200
  if (motor == 0) {
    targetLeftSpeed = speed;
  } else {
    targetRightSpeed = speed;
  }
}