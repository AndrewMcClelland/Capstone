#include "RobotPosition.h"

#include <fstream>	// ifstream, ofstream
#include <cmath>	// fabs() // was cmath > changed to math by Marc

RobotPosition::RobotPosition (
	const double& _x, const double& _y, const double& _z, const double& _j4, const double& _j5, const double& _j6,
	const double& _fudge) :

	x(_x), y(_y), z(_z), j4(_j4), j5(_j5), j6(_j6)
{
	setFudge(_fudge);
}

bool RobotPosition::fromFile (const std::string& filename) {
	using namespace std;

	ifstream infile(filename.c_str());
	if (infile) {
		infile	>> x >> y >> z >> j4 >> j5 >> j6;
		return true;
	}
	else return false;
}

bool RobotPosition::toFile (const std::string& filename) {
	using namespace std;

	ofstream outfile(filename.c_str());
	if (outfile) {
		outfile	<< x << " " << y << " " << z << " " << j4 << " " << j5 << " " << j6 << endl;
		return true;
	}
	else return false;
}

void RobotPosition::setFudge (const double& f) {
	if (f < 0)			fudge = 0;
	else if (f < _MAX_POS_FUDGE)	fudge = f;
	else				fudge = _MAX_POS_FUDGE;
}

const RobotPosition RobotPosition::operator+ (const RobotPosition& rv) const {
	return RobotPosition(x + rv.x, y + rv.y, z + rv.z, j4 + rv.j4, j5 + rv.j5, j6 + rv.j6);
}

const RobotPosition RobotPosition::operator- (const RobotPosition& rv) const {
	return RobotPosition(x - rv.x, y - rv.y, z - rv.z, j4 - rv.j4, j5 - rv.j5, j6 - rv.j6);
}

RobotPosition& RobotPosition::operator= (const RobotPosition& rv) {
	if (this == &rv) return *this;
	x  = rv.x;
	y  = rv.y;
	z  = rv.z;
	j4 = rv.j4;
	j5 = rv.j5;
	j6 = rv.j6;
	return *this;
}

RobotPosition& RobotPosition::operator+= (const RobotPosition& rv) {
	x  += rv.x;
	y  += rv.y;
	z  += rv.z;
	j4 += rv.j4;
	j5 += rv.j5;
	j6 += rv.j6;
	return *this;
}

bool RobotPosition::operator== (const RobotPosition& rv) const {
	return	fabs(x  - rv.x ) <= fudge &&
		fabs(y  - rv.y ) <= fudge &&
		fabs(z  - rv.z ) <= fudge &&
		fabs(j4 - rv.j4) <= fudge &&
		fabs(j5 - rv.j5) <= fudge &&
		fabs(j6 - rv.j6) <= fudge;
}

std::ostream& operator<< (std::ostream& os, const RobotPosition& p) {
	os	<<    "x=" << p.x  << " mm"
		<<  ", y=" << p.y  << " mm"
		<<  ", z=" << p.z  << " mm"
		<< ", j4=" << p.j4 << " degrees"
		<< ", j5=" << p.j5 << " degrees"
		<< ", j6=" << p.j6 << " degrees";
	return os;
}
