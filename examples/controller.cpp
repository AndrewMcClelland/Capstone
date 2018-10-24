#include "RobotController.h"

#include <iostream>
#include <string>

using namespace std;
using namespace RobotManipulators;

int main (int argc, char* argv[]) {
	RobotController r;
	if (r.connected())	cout << "Connected to Robot controller" << endl;
	else			cout << "Not connected to Robot controller!" << endl;
	cout << "----------------------------------" << endl;

	r << "JOINT 6 +90" << finish << "JOINT 6 -90" << block;

	string result;
	r >> result;

	cout << "Command result:" << endl << result << endl;

	r << "STATUS" << block;
	r.skipTo("AXIS#");
	r >> result;
	
	cout << "Command result:" << endl << result << endl;
	
	return 0;
}
