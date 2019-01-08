#include "RobotLimits.h"

#include <fstream>

RobotLimits::RobotLimits (const std::string& limit_file, const double& _fudge) {
	setFudge(_fudge);
	if (limit_file.length() <= 0 || !fromFile(limit_file)) setDefault();
}

void RobotLimits::setDefault () {
	minPos = _DEFAULT_MIN;
	maxPos = _DEFAULT_MAX;
	maxSpeed = _DEFAULT_MAX_SPEED;
}

void RobotLimits::setFudge (const double& f) {
	if (f < 0)			fudge = 0;
	else if (f < _MAX_LIM_FUDGE)	fudge = f;
	else				fudge = _MAX_LIM_FUDGE;
}

bool RobotLimits::posWithin (const RobotPosition& pos) const {
	if ((pos.x  > max().x  + fudge) || ((pos.x  < min().x  - fudge))) return false;
	if ((pos.y  > max().y  + fudge) || ((pos.y  < min().y  - fudge))) return false;
	if ((pos.z  > max().z  + fudge) || ((pos.z  < min().z  - fudge))) return false;
	if ((pos.j4 > max().j4 + fudge) || ((pos.j4 < min().j4 - fudge))) return false;
	if ((pos.j5 > max().j5 + fudge) || ((pos.j5 < min().j5 - fudge))) return false;
	if ((pos.j6 > max().j6 + fudge) || ((pos.j6 < min().j6 - fudge))) return false;
	return true;
}

void RobotLimits::movePosWithin (RobotPosition& pos) const {
	if (posWithin(pos)) return;	// Nothing to do

	if (pos.x  > max().x  + fudge) pos.x  = max().x;
	if (pos.x  < min().x  - fudge) pos.x  = min().x;
	if (pos.y  > max().y  + fudge) pos.y  = max().y;
	if (pos.y  < min().y  - fudge) pos.y  = min().y;
	if (pos.z  > max().z  + fudge) pos.z  = max().z;
	if (pos.z  < min().z  - fudge) pos.z  = min().z;
	if (pos.j4 > max().j4 + fudge) pos.j4 = max().j4;
	if (pos.j4 < min().j4 - fudge) pos.j4 = min().j4;
	if (pos.j5 > max().j5 + fudge) pos.j5 = max().j5;
	if (pos.j5 < min().j5 - fudge) pos.j5 = min().j5;
	if (pos.j6 > max().j6 + fudge) pos.j6 = max().j6;
	if (pos.j6 < min().j6 - fudge) pos.j6 = min().j6;
}

bool RobotLimits::fromFile (const std::string& filename) {
	using namespace std;

	ifstream infile(filename.c_str());
	if (infile) {
		infile	>> minPos.x  >> minPos.y  >> minPos.z
			>> minPos.j4 >> minPos.j5 >> minPos.j6
			>> maxPos.x  >> maxPos.y  >> maxPos.z
			>> maxPos.j4 >> maxPos.j5 >> maxPos.j6
			>> maxSpeed;

		// Now, make sure these limits are wihin the default limits
		RobotLimits defaultLimits = RobotLimits("");
		defaultLimits.movePosWithin(minPos);
		defaultLimits.movePosWithin(maxPos);
		if (maxSpeed > _DEFAULT_MAX_SPEED) maxSpeed = _DEFAULT_MAX_SPEED;

		return true;
	}
	else return false;
}

bool RobotLimits::toFile (const std::string& filename) {
	using namespace std;

	ofstream outfile(filename.c_str());
	if (outfile) {
		outfile	<< minPos.x  << " " << minPos.y  << " " << minPos.z  << " "
			<< minPos.j4 << " " << minPos.j5 << " " << minPos.j6 << " "
			<< maxPos.x  << " " << maxPos.y  << " " << maxPos.z  << " "
			<< maxPos.j4 << " " << maxPos.j5 << " " << maxPos.j6 << " "
			<< maxSpeed  << endl;
		return true;
	}
	else return false;
}

std::ostream& operator<< (std::ostream& os, const RobotLimits& l) {
	using std::endl;
	os	<< "Min.: "	<< l.min()		<< endl
		<< "Max.: "	<< l.max()		<< endl
		<< "Speed.: "	<< l.speed() << "%"	<< endl;
	return os;
}

