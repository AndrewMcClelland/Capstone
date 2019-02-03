#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>

#include "RobotPosition.h"
#include "RobotLimits.h"
#include "RobotController.h"

// Declaration of the ready locations
typedef enum {
	CENTER = 0,
	NW,
	NE,
	SE,
	SW
} ReadyLocation;


class Robot {

private:

	// The following array holds pulses-to-mm/degrees multipliers for each joint
	// These are found by dividing the joint values (in mm/degrees) by the number of motor pulses
	static const double Resolution[];
	
	// The following values are given as a portion of the current speed limit (0 <= x <= 1).
	static const double RaiseSpeed;		// Speed of raise
	static const double MoveSpeed;		// Speed of horiz. move
	static const double ApproachSpeed;	// Speed of approach
	static const double DefaultSpeed;	// Default speed when none is specified

	static const unsigned int ApproachLength;	// Length of approach in mm
	
	std::ostream*	log;		// Pointer to where debug/status information should be sent

	RobotLimits	limits;		// Current robot limits. Robot cannot move outside these
	RobotPosition	pos;		// Robot position (x, y, z) in mm and (j4, j5, j6) in degrees

	bool	activated;		// 0 if robot joints are disabled, 1 if joints are enabled
	bool	homed;			// 0 if not homed, 1 if homed

	const RobotPosition getPos();

	// Return when <Enter> (ASCII newline (LF)) appears on stdin
	void waitForEnter ();

public:
	
	RobotController	controller;	// Controller communications handle
	
	// Constructor
	Robot (
		const std::string& _serialport	= "/dev/ttyUSB0",
		const std::string& _limit_file	= "/home/rcvlab/Robot/robot_limits.dat",
		std::ostream& _log		= std::cerr);

	// Destructor
	~Robot ();

	// return true if connected to robot controller, false otherwise
	bool connected () { return controller.connected(); }
	
	// get current state (including position) from robot controller.
	// If thorough is false and position has not changed: skip some time-consuming tests
	void update (const bool& thorough = false);

	// Activates the robot joints. Run this before trying to move
	void activate ();

	// Deactivates the robot joints. Run this before trying to push the robot manually
	void deactivate ();

	// Homing routine. Necessary on controller startup to initialize position etc.
	// If already homed, do not re-home unless redo is true
	void home (const bool& redo = false);

	// Return current robot position
	const RobotPosition currentPos ();

	// Return current robot limits
	const RobotLimits currentLimits ();
	
	// Move the robot directly to the given position
	void moveTo (const RobotPosition& _dest, const double& speed = DefaultSpeed);

	// Move the robot directly to the given relative position (relative to the current position)
	void moveBy (const RobotPosition& delta, const double& speed = DefaultSpeed);
	
	// Raise the robot to top Z-coordinate
	void raise ();

	// Move the robot to the given position in a 3 step process: Raise -> Move -> Approach
	void move3step (const RobotPosition& _dest);
	
	// Move the robot to a ready location
	void ready (const ReadyLocation& r = CENTER);
};

#endif // ROBOT_H
