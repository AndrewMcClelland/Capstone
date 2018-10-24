#ifndef ROBOT_POSITION_H
#define ROBOT_POSITION_H

#include <iostream>	// ostream

// Hardcoded maximum difference in any coordinate between two "equal" positions
#define _MAX_POS_FUDGE 0.1

class RobotPosition {

private:
	// Maximum difference between two "equal" values
	double fudge;

public:
	// components of a robot position. x, y and z are in mm and j4, j5 and j6 are in degrees
	double x, y, z, j4, j5, j6;

	// Constructor
	RobotPosition (
		const double& _x  = 0.0,
		const double& _y  = 0.0,
		const double& _z  = 0.0,
		const double& _j4 = 0.0,
		const double& _j5 = 0.0,
		const double& _j6 = 0.0,
		const double& _fudge = 0.05);

	// Read in position from the given file
	bool fromFile (const std::string& filename);

	// Write out position to the given file
	bool toFile (const std::string& filename);

	// Specify the maximum allowable difference between two "equal" values
	// There is only one fudge value for all coordinates (x, y, z, j4, j5, j6), and its unit
	// is mm for x, y and z, and degrees for j4, j5 and j6.
	void setFudge (const double& f);

	// Addition of positions (component-wise)
	const RobotPosition operator+ (const RobotPosition& rv) const;

	// Subtraction of positions (component-wise)
	const RobotPosition operator- (const RobotPosition& rv) const;

	// Assign another RobotPosition to this
	RobotPosition& operator= (const RobotPosition& rv);

	// Add another RobotPosition to this
	RobotPosition& operator+= (const RobotPosition& rv);

	// Compare two robot positions. return true if equal
	bool operator== (const RobotPosition& rv) const;

	// Compare two robot positions. return true if not equal
	bool operator!= (const RobotPosition& rv) const { return !operator==(rv); }

	// Facilitates output of a robot position to an ostream object
	friend std::ostream& operator<< (std::ostream& os, const RobotPosition& p);
};

#endif // ROBOT_POSITION_H
