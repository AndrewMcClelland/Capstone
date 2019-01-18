#include "Robot.h"

#include <iostream>

int main (int argc, char *argv[]) {
	using namespace std;

	Robot r = Robot("/dev/ttyS1", "/var/qpool/robot_limits.dat");
	
	cout << "Homing..." << flush;
	r.home();
	cout << "done" << endl;
	
// ***	cout	<< "Current location: " << r.currentPos() << endl << endl;
	
	cout << endl << "Cycling through ready locations..." << endl;
	r.ready();
/* ***	r.ready(NW);
	r.ready(NE);
	r.ready(SE);
	r.ready(SW);
	r.ready(CENTER);
*** */	r.ready(NE);

/* ***
	cout << "Deactivating the robot..." << flush;
	r.deactivate();
	cout << " ok." << endl;
*** */
}
