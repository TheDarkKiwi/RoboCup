#ifndef NAVIGATE_H
#define NAVIGATE_H

#define WALL_FOLLOW_DISTANCE 200
#define WALL_FOLLOW_LEFT 0
#define WALL_FOLLOW_RIGHT 1

#define START_WALL_FOLLOW_DISTANCE 600
#define FRONT_AVOID_DISTANCE 300

void navigate_to_weight();

void navigate_wall_follow(int wallFollowDir);

void navigate_logic();

void navigate_drop_weight();

void navigate_IMU_watchdog();

void navigate_search_watchdog();

void navigate_wall_follow_test();

void navigate_ramp();

void navigate_home();

void navigate_heading(float goalHeading);

#endif // NAVIGATE_H