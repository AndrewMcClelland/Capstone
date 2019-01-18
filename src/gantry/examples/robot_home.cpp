#include <Robot.h>
#include <RobotPosition.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>	// sqrt(), atan()

using namespace std;

int main (int argc, char* argv[]) {
	cout << "Move robot home" << endl;

	// Initialize robot
	ofstream robot_log("./build/robot.log");
	Robot robot("/dev/ttyS1", "/var/qpool/robot_limits.dat", robot_log);

	// Move robot out of the way
	robot.home();
	robot.raise();
	robot.moveTo(RobotPosition(0, 0, 0, 0, 0, -90), 1.0);

	return 0;
}
