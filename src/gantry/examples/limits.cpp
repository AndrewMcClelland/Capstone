#include <string>
#include <iostream>
#include <fstream>

#include <cmath>

#include "RobotLimits.h"

// *** Change this into a replacement for gantry_limits

int main (int argc, char *argv[]) {
	using namespace std;

	string limit_file = "/var/qpool/robot_limits.dat";

	cout << "This program tests the RobotLimits class." << endl;

	RobotLimits l = RobotLimits(limit_file);

	cout << "Here are the current robot limits:" << endl << l;

	l.toFile(limit_file);
}
