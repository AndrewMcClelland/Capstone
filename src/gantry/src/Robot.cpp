#include "Robot.h"

#include <sstream>

extern "C" {
#include <unistd.h>
#include <math.h> // changed from cmath to math.h by Marc cmath was causing problems with compiler (version too old)
}

#define LOG (*log)	// Convenient for accesing log as a normal ostream
#define MOVING_CHECK_TIME 100000 // Time in microseconds between samples to check for current movement (< 1,000,000)

using namespace std;
using namespace RobotManipulators;

/* Constants */

const double Robot::Resolution[] = {
	-0.0150150,	// x
	-0.0150000,	// y
	-0.0150150,	// z
	-0.00712871,	// j4
	-0.00360002,	// j5
	-0.007128711	// j6
};

const double Robot::RaiseSpeed		= 1.0;	// Speed of raise
const double Robot::MoveSpeed		= 1.0;	// Speed of horiz. move
const double Robot::ApproachSpeed	= 0.1;	// Speed of approach
const double Robot::DefaultSpeed	= 0.1;	// Default speed when none is specified

const unsigned int Robot::ApproachLength = 60;	// Length of approach in mm

/* Methods */

Robot::Robot (const std::string& _serialport, const std::string& _limit_file, std::ostream& _log) :
	log(&_log), limits(RobotLimits(_limit_file)), activated(false), homed(false), controller(_serialport, LOG)
{
	cout	<< "Robot::Robot(): Please make sure the teach pendant is connected to the controller." << endl
		<< "                Start the controller and transfer control to the host computer by pressing" << endl
		<< "                <ESC> followed by <F2> on the teach pendant. Press <Enter> when ready to continue.";
	waitForEnter();
	
	if (!connected()) LOG << "Robot::Robot(): Cannot establish robot connection." << endl;

	update(true);		// Initialize robot position and other state parameters

	// controller << "ENABLE ARM" << finish;
}

Robot::~Robot () {
}

const RobotPosition Robot::getPos () {
	// Ask for current position
	controller << "W2" << block;

		/* Typical answer to W2:
		 * >>W2
		 * 
		 * CURRENT ACTUAL POSITION :
		 * NAME       AX#1/6      AX#2/7      AX#3/8      AX#4        AX#5
		 * PULSES   -0000000001 -0000000001 -0000000001 -0000000001 -0000000001
		 *          -0000000114
		 * NAME       JT#1/6    JT#2/7    JT#3/8    JT#4      JT#5
		 * JOINTS   +000.0150 +000.0150 +000.0150 +000.0071 +000.0036
		 *          +000.8126
		 * WORLD     +000.0198 +000.0149 -076.1849 +000.0000 +089.9964 +000.8198
		 * .
		 */

	// Skip forward to the interesting part of the answer
	if (!controller.skipTo("JOINTS")) {
		LOG << "Robot::getPos(): Could not find 'JOINTS' in 'W2' response! Aborting." << endl;
		return RobotPosition();
	}

	string reply;
	controller >> reply;
	istringstream replystream(reply);
	
	double _x, _y, _z, _j4, _j5, _j6;
	string dummy;
	replystream >> dummy >> _x >> _y >> _z >> _j4 >> _j5 >> _j6;

	// Return position
	return RobotPosition(_x, _y, _z, _j4, _j5, _j6);
}

void Robot::waitForEnter () {
	char Enter = 0x0a;
	char c = 0;
	do cin.get(c); while (c != Enter);
}

void Robot::update (const bool& thorough) {
	if (!connected()) {
		LOG << "Robot::update(): No connection to robot! Aborting." << endl;
		return;
	}

	RobotPosition prev = pos;
	pos = getPos();

	// cout << "Printing from Robot.cpp file in update() function...\n Current position is = " << pos << endl;
	
	// If there has been no movement since last update(). Assume we can skip checking status flags
	if (pos == prev && !thorough) return;
	

	// Ask for status information
	controller << "STATUS" << block;

		/* Typical answer to STATUS:
		 * >>STATUS
		 * RAPL-II Version 2.60     G3000 Robot
		 * LAST ERROR CODE: 012-COMMAND ERROR.
		 * SPEED (% OF FULL) :    +50
		 * MAX LINEAR SPEED: +990.6000
		 * CURRENT PROGRAM EXECUTING:
		 * [00]AUTOEXEC#00005 :019-LINE NOT FOUND.
		 * JOINT LIMIT CHECK IS OFF
		 * CONTROLLER IS NOT HOMED
		 * TOOL TRANSFORM
		 * NULL      +000.0000 +000.0000 +000.0000 +000.0000 +000.0000 +000.0000
		 * GRIPPER TYPE:  SERVO
		 * AXIS#        1 2 3 4 5 6
		 * LOCK         N N N N N N
		 * DONE         Y Y Y Y Y Y
		 * LIMP         Y Y Y Y Y Y
		 * HOMED        N N N N N N
		 * REQUESTED ROBOT CONFIG : UPRIGHT/REACH FORWARD/ELBOW UP/WRIST FREE
		 * CURRENT ROBOT CONFIG : UPRIGHT/REACH FORWARD/ELBOW DOWN/WRIST FREE
		 * MANUAL MODE STATUS IS OFF
		 * METRIC UNITS.
		 */
	
	// Skip forward to the interesting part of the answer
	if (!controller.skipTo("AXIS#")) {
		LOG << "Robot::update(): Could not find 'AXIS#' in 'STATUS' response! Aborting." << endl;
		return;
	}
	if (!controller.skipTo("LIMP")) {
		LOG << "Robot::update(): Could not find 'LIMP' in 'STATUS' response! Aborting." << endl;
		return;
	}
	
	string reply;
	controller >> reply;
	istringstream replystream(reply);
	
	string dummy, _limp, _homed;
	replystream >> dummy >> _limp >> dummy >> dummy >> dummy >> dummy >> dummy >> dummy >> _homed;

	activated = _limp == "N";
	homed = _homed == "Y";
}

void Robot::activate () {
	if (!connected()) {
		LOG << "Robot::activate(): No connection to robot! Aborting." << endl;
		return;
	}

	if (activated) return;

	// controller << "ARM ON" << finish;
	controller << "ENABLE ARM" << finish;

	cout << "Robot::activate(): Please make sure that the ARM POWER switch is on before pressing <Enter>.";
	waitForEnter();
	
	activated = true;
}

void Robot::deactivate () {
	if (!connected()) {
		LOG << "Robot::deactivate(): No connection to robot! Aborting." << endl;
		return;
	}

	// controller << "ARM OFF" << finish;
	
	activated = false;
}

void Robot::home (const bool& redo) {
	if (!connected()) {
		LOG << "Robot::home(): No connection to robot! Aborting." << endl;
		return;
	}

	if (!activated) activate();

	if (homed && !redo) return;
	
	// Run HOME program on controller and turn joint 6 by -90 degrees to make joint 5 tilt cue in right direction
	controller << "RUN HOME" << finish;
	
	// The following is now included in the controller HOME program
	// controller << "JOINT 6 -90" << finish;
	
	homed = true;
}

const RobotPosition Robot::currentPos () {
	update();
	return pos;
}

const RobotLimits Robot::currentLimits () {
	return limits;
}

void Robot::moveTo (const RobotPosition& _dest, const double& speed) {
	// Move directly to the given position at the given speed
	
#define MOVE_PROG_NAME	"QPOOL"		// Name of move program in controller memory
#define MOVE_POINT_NAME	"#QPOOL"	// Name of endpoint in controller memory
	
	update();
	
	if (!connected()) {
		LOG << "Robot::moveTo(): No connection to robot! Aborting." << endl;
		return;
	}
	if (!activated) activate();
	if (!homed) {
		LOG << "Robot::moveTo(): Robot not homed! Aborting." << endl;
		return;
	}

	RobotPosition destination = _dest;
	// Make sure destination is within limits. If not, move destination within limits
	if (!limits.posWithin(destination)) { // destination not within limits
		LOG << "Robot::moveTo(): Destination not within limits: " << destination << endl;
		limits.movePosWithin(destination);
		LOG << "                                    Reassigned: " << destination << endl;
	}

	// Check if movement is necessary at all
	if (pos == destination) {	// movement not necessary
		LOG << "Robot::moveTo(): Movement not necessary. Aborting." << endl;
		return;
	}

	// Convert given speed (portion of limits.speed()) to real speed (% of full robot speed)
	unsigned int realSpeed = (unsigned int) nearbyint(limits.speed() * speed);

	// Write some useful info to the log
	LOG	<< "Robot::moveTo(): Moving from " << pos << endl
		<< "         with speed = " << realSpeed << "% to " << destination << endl;

	// Convert destination units from mm/degrees to motor pulses
	int destpulses[6]; // holds destination in units of motor pulses
	destpulses[0] = (int) nearbyint(destination.x  / Resolution[0]);
	destpulses[1] = (int) nearbyint(destination.y  / Resolution[1]);
	destpulses[2] = (int) nearbyint(destination.z  / Resolution[2]);
	destpulses[3] = (int) nearbyint(destination.j4 / Resolution[3]);
	destpulses[4] = (int) nearbyint(destination.j5 / Resolution[4]);
	destpulses[5] = (int) nearbyint(destination.j6 / Resolution[5]);

	// Create controller commands for setting up destination point, speed, and move robot
	std::ostringstream pointCmd, speedCmd, moveCmd;

	pointCmd << "POINT " << MOVE_POINT_NAME << " "
		 << destpulses[0] << ", " << destpulses[1] << ", " << destpulses[2] << ", "
		 << destpulses[3] << ", " << destpulses[4] << ", " << destpulses[5];
	speedCmd << "SPEED " << realSpeed;
	moveCmd  << "RUN " << MOVE_PROG_NAME;

	// Send commands to controller
	controller << pointCmd.str() << finish << speedCmd.str() << finish << moveCmd.str() << finish;
#undef MOVE_PROG_NAME
#undef MOVE_POINT_NAME
}

void Robot::moveBy (const RobotPosition& delta, const double& speed) {
	// Add the given relative position onto the current position and move there
	update();
	moveTo(pos + delta, speed);
}

void Robot::moveToLoop(const RobotPosition& _dest, const double& speed) {
	RobotPosition currPos;
	int error_x = 30, error_y = 30;

	while(true) {
		currPos = getPos();
		
		if(abs(currPos.x - _dest.x) < error_x && abs(currPos.y - _dest.y) < error_y) {
			cout << "Reached destination " << _dest << endl;
			return;
		}
		cout << "Still moving... current position = " << currPos << endl;
		moveBy((_dest - currPos), 1.0);
	}
}

void Robot::raise () {
	// Raise the z-axis to top/max limit
	update();

	RobotPosition destination = pos;
	destination.z = limits.max().z;
	moveTo(destination, RaiseSpeed);
}

void Robot::move3step (const RobotPosition& destination) {
	// Raise the z-axis to top/max limit and then move to destination + approach, From there, approach to destination
	raise();
	RobotPosition approach = RobotPosition(0, 0, ApproachLength);
	moveTo(destination+approach, MoveSpeed);
	moveTo(destination, ApproachSpeed);
}

void Robot::ready (const ReadyLocation& r) {
	double j4 = (limits.min().j4 + limits.max().j4) / 2;
	double j5 = (limits.min().j5 + limits.max().j5) / 2;
	double j6 = (limits.min().j6 + limits.max().j6) / 2;

	RobotPosition ReadyLocations[5] = {
		// CENTER
		RobotPosition(
			(limits.min().x + limits.max().x) / 2,
			(limits.min().y + limits.max().y) / 2,
			limits.max().z,
			j4, j5, j6
		),

		RobotPosition(limits.min().x, limits.min().y, limits.max().z, j4, j5, j6), // NW
		RobotPosition(limits.min().x, limits.max().y, limits.max().z, j4, j5, j6), // NE
		RobotPosition(limits.max().x, limits.max().y, limits.max().z, j4, j5, j6), // SE
		RobotPosition(limits.max().x, limits.min().y, limits.max().z, j4, j5, j6), // SW
	};

	RobotPosition destination = ReadyLocations[r];

	if (!limits.posWithin(destination)) {	// This should never happen
		LOG << "Robot::ready(): *** Ready location not within limits!!! Aborting." << endl;
		return;
	}

	// perform move to ready position
	raise();
	moveTo(destination, MoveSpeed);
}

#undef LOG
#undef MOVING_CHECK_TIME
