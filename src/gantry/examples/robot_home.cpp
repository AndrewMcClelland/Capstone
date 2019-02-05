#include <Robot.h>
#include <RobotPosition.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath> // sqrt(), atan()

using namespace std;

int main(int argc, char *argv[])
{
	cout << "Move robot home" << endl;

	// Initialize robot
	ofstream robot_log("robot.log");
	Robot robot("/dev/ttyUSB0", "/home/gantryrobot/Desktop/Capstone/src/gantry/robot_limits.dat", robot_log);

	// Move robot out of the way
	// robot.home();
	// std::cout << "the robot is at home position ANDREW" << std::endl;
	// robot.raise();
	// std::cout << "the robot is raised at z-axis." << std::endl;
	robot.moveTo(RobotPosition(3000, 0, 0, 180, 35, -90), 1.0);
	// robot.moveTo(RobotPosition( 0, -1795, -290, -180, -35, -90), 1.0);
	std::cout << "the robot is minimum position" << std::endl;
	// robot.moveTo(RobotPosition(2000, -5, -100, 180, 35, -90), 1.0);
	// std::cout << "the robot is at max position" << std::endl;

	return 0;
}

// int main (int argc, char* argv[]) {
// 	cout << "Move robot home" << endl;

// 	// Initialize robot
// 	ofstream robot_log("robot.log");
// 	Robot robot("/dev/ttyUSB0", "home/rcvlab/Robot/robot_limits.dat", robot_log);

// 	// Move robot out of the way
// 	robot.home();
// 	robot.raise();
// 	robot.moveTo(RobotPosition(0, 0, 0, 0, 0, -90), 1.0);

// 	return 0;
// }
