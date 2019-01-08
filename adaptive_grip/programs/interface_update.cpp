#include "Robot.h"

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
using namespace std;

void test_move(Robot &r);
void oneLine(Robot& r); 
void multiLine(Robot& r);


void send_and_poll(Robot &r);

/*
	Function		: 		main
	Description	: 		Main Method
	Return Value: 		0 for Successful Run. Non-zero otherwise.
*/
int main (int argc, char *argv[]) {
	char choose;
	cout << "Connecting..." << endl;
///Robot r = Robot("/dev/gantry", "/home/kevin/Documents/ECEGantry/RobotV2/robot_limits.dat");
   Robot r = Robot("/dev/cu.usbserial");
	cout << "Connected to Gantry" << endl;
	while (true) {
		do {
				cout << "\nOne line or Multi line command? [o/m]" << endl;
				cin >> choose;
				choose = toupper(choose);
				cin.ignore(100, '\n');
		} while ((choose != 'O') && (choose != 'M') && (choose != 'P') && (choose != 'D') && (choose != 'Z'));
		if (choose == 'O')
			oneLine(r);
		else if (choose == 'M')
			multiLine(r);
		else if (choose == 'P')
			send_and_poll(r);
		else if (choose == 'Z')
			test_move(r);
		else if (choose == 'D')
			return 0;
		
	} // while (true)
}

void send_and_poll(Robot &r)
{
	// Send a a command; poll the result.
	string cmd;
	string reply;

	// Receive line from standard input
	cout << "\nSend and poll: input a single-line RAPL command." << endl;

	getline(cin, cmd);

	cout << "Sending line: " << cmd << endl;

	// Uppercase
	transform(cmd.begin(), cmd.end(), cmd.begin(), ::toupper);

	// Send the command; block until it's done
	// e.g. until the the controller sends back a prompt.
	r.controller << cmd << RobotManipulators::block;

	// Now poll the buffer.
	r.controller >> reply;
	/*
		RobotContoller::receiveData();
			CRScomm::CRS_receive()
	*/
	cout << "REPLY:" << endl << reply << endl;
}


void test_move(Robot &r){
	cout << "Entering test_move()..." << endl;
	r.moveTo(RobotPosition(0, 0, 0, 0, 0, -90), 1.0);
	cout << "Finished." << endl;
}


/*
	Function		: 		oneLine
	Arguments	:		
		r - Robot instance to execude commands on
	Description	: 		
		Enter a one-line RAPL command.
	Return Value: 		None
*/
void oneLine(Robot& r) {
	string cmd;
		cout << "\nWelcome to one line, input RAPL commands in sequence." << endl
		 << "Case does not matter, type /done to exit." << endl;
		do {
			cout << "\nPlease enter a command." << endl;
			getline(cin, cmd);
			transform(cmd.begin(), cmd.end(), cmd.begin(), ::toupper);
			if (cmd == "/DONE")
				break;
			r.runCmd(cmd);
		} while (true);
	cout << "\none line commands test aborted."<< endl;
	return;
}

void multiLine(Robot& r)  {
	vector<string> commands;
	string cmd;
	bool keepRun = true;
	char ynchr;
	int counter = 1;
	
	cout << "\nWelcome to multi line, input RAPL commands in sequence." << endl
		 << "Case does not matter, type /del to delete previous line, /show to display commands and /done to execute commands." << endl;
	do {
		keepRun = true;
		do {
			cout << "\nPlease enter command " << counter << endl;
			getline(cin, cmd);
			transform(cmd.begin(), cmd.end(), cmd.begin(), ::toupper);
			if (cmd == "/DONE") {
				break;
			}
			else if ((cmd == "/DEL") && (commands.size() >= 1)) {
				commands.pop_back();
				counter--;
			}
			else if (cmd == "/SHOW"){
				cout << "\nCURRENT COMMANDS" << endl;
				for (int i = 0; i < commands.size(); i++)
					cout << (i+1) << ": " << commands[i] << endl;
			}
			else {
				commands.push_back(cmd);
				counter++;
			}
			
		} while (true);
		cout << "\nCOMMANDS" << endl;
		for (int i = 0; i < commands.size(); i++)
			cout << (i+1) << ": " << commands[i] << endl;
		r.multiRunCmd(commands);
		do {
			cout << "Would you like to run more? [y/n]" << endl;
			cin >> ynchr;
			ynchr = toupper(ynchr);
		} while ((ynchr != 'Y') && (ynchr != 'N'));
		if (ynchr == 'N')
			keepRun = false;
		cin.ignore((cmd.size() + 1), '\n');
		counter = 1;
		commands.clear();
	} while (keepRun == true);
	cout << "\nmulti line commands test aborted."<< endl;
}
