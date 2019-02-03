#pragma once

#include "../../gantry/include/Robot.h"
#include "../../gantry/include/RobotPosition.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath> // sqrt(), atan()

using namespace std;

typedef enum {
    X = 0,
    Y = 1,
    Z = 2
} Dimension;

typedef enum {
    POSITIVE = 0,
    NEGATIVE = 1
} Direction;

// Initialize robot
ofstream robot_log("robot.log");
Robot robot("/dev/ttyUSB0", "home/rcvlab/Robot/robot_limits.dat", robot_log);

bool move_robot_home();
bool move_robot_to_pos(const float x_pos, const float y_pos);
bool move_robot_continuously(const Dimension& dim, const Direction& dir);