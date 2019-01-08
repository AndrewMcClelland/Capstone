#ifndef ROBOT_LIMITS_H
#define ROBOT_LIMITS_H

#include <string>
#include <iostream>

#include "RobotPosition.h"

// These are the internal maximum positional limits. Any limits in the limit file must be within these.
// Make sure that all MAX-components have a numeric value higher than their corresponding
// MIN-components
//						 X      Y     Z    J4   J5   J6
#define _DEFAULT_MIN		RobotPosition(   0, -1795, -290, -180, -35, -90)
#define _DEFAULT_MAX		RobotPosition(3000,     0,    0,  180,  35,  90)

// Maximum speed limit given as percentage of full speed
#define _DEFAULT_MAX_SPEED 	50

// Hardcoded maximum difference between a coordinate and its closest limit.
#define _MAX_LIM_FUDGE		0.1

class RobotLimits {

private:
	RobotPosition minPos, maxPos;	// Minimum and maximum positional limits
	unsigned int maxSpeed;		// Maximum speed limit in percentage of full speed
	double fudge;			// Maximum difference between two "equal" values

public:
	// Constructor with default file name containing limits.
	// Pass an empty string to get hardcoded default limits.
	RobotLimits (
		const std::string& limit_file = "/home/kevin/Documents/ECEGantry/RobotV2/robot_limits.dat",
		const double& _fudge = 0.05);

	// Accessor methods
	const RobotPosition	min () const	{ return minPos;   }
	const RobotPosition	max () const	{ return maxPos;   }
	const unsigned int	speed () const	{ return maxSpeed; }
	double			getFudge ()	{ return fudge;    }

	// Set the default limits. The default limits are those declared towards the top of this file
	void setDefault ();

	// Specify the maximum allowable difference between two "equal values"
	// There is only one fudge value for all coordinates (x, y, z, j4, j5, j6), and its unit
	// is mm for x, y and z, and degrees for j4, j5 and j6.
	void setFudge (const double& f);

	// Return true if the given RobotPosition is within limits. False otherwise
	bool posWithin (const RobotPosition& pos) const;

	// Make sure the given RobotPosition is within these limits by moving to the closest point
	// that is within limits
	void movePosWithin (RobotPosition& pos) const;

	// Read in limits from the given file
	bool fromFile (const std::string& filename);

	// Write out limits to the given file
	bool toFile (const std::string& filename);

	// Facilitates output of robot limits to an ostream object
	friend std::ostream& operator<< (std::ostream& os, const RobotLimits& l);
};

#endif // ROBOT_LIMITS_H
