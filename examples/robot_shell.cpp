#include <string>	// string container
#include <iostream>	// Console I/O
#include <fstream>	// File I/O (Log)
#include <sstream>	// String I/O (istringstream)
#include <list>		// list container (used in command completion)

#include <cstdlib>	// malloc(), free()
#include <cctype>	// isspace()
#include <cstring>	// strlen()

extern "C" {
#include <stdio.h>		//
#include <readline/readline.h>	// Readline functionality
#include <readline/history.h>	//
}

// *** #include "Robot.h"
// *** #include <poolcue.h>

using namespace std;

/* * * * * * * * * *
 * Type definitions
 * * * * * * * * * */

typedef void ComFunc (list<string>& args);

typedef struct {
	string name;	// User printable name of the function.
	ComFunc* func;	// Pointer to function for executing this command.
	string doc;	// Documentation for this function.
} COMMAND;

/* The following commands must be accepted
	- help			- Display online help.
	- help <Command>	- Display specific help on a command.
	- where			- Print the current robot position.
	- ready <ReadyPos>	- Move to the given ready location.
	- moveto <RobotPos>	- Move the robot to the given absolute position.
	- moveby <RobotPos>	- Move the robot to the given position relative to the current.
	- move <Joint> <Dist>	- Move the specified Joint by the given distance [mm/degrees].
	- move3step <RobotPos>	- Move the robot to the given absolute position using 3 steps.
	- raise			- Raise the robot Z-axis to its top position.
	- home			- Run the homing routine.
	- activate		- Activate the robot joints.
	- deactivate		- Deactivate the robot joints.
	- limits		- Display the current robot software limits
	- editlimits		- Edit the current robot software limits
	- cue_home		- Move pool cue to home position.
	- cue_where		- Print the current pool cue position.
	- cue_shoot <Speed> -t<Time>	- Perform time-based shot with the given parameters.
	- cue_shoot <Speed> -l<CuePos>	- Perform position-based shot with the given parameters.
   Possible future commands:
	- cue_lock		- Lock the cue in the current position.
	- cue_unlock		- Unlock/disable/deactivate the cue motor.
	- cue_moveto <CuePos>	- Move the pool cue to the given position (0 < CuePos < 0.28)
	- cue_moveby <CuePos>	- Move the pool cue by the given distance (may be negative)

   Explanation of wildcards:
	<Command>	- Any of the commands described above.
	<ReadyPos>	- Any one of the following: {NW, NE, SW, SE, CENTER}.
	<RobotPos>	- A list of 1 to 6 elements. Each element is a joint value [mm/degrees].
	<Joint>		- Any one of the following: {X, Y, Z, J4, J5, J6}.
	<Dist>		- Joint distance. Floating-point value of mm (X/Y/Z) or degrees (J4/5/6).
	<Speed>		- Desired cue speed. Floating-poitn value in m/s between 0 and 3.
	<Time>		- Floating-point time value in seconds [s].
	<CuePos>	- Floating-point pool cue position in meters [m]. Between 0 and 0.28.
*/


/* * * * * * * * * * * * * * * * * * * * *
 * Forward declaration of command handlers
 * * * * * * * * * * * * * * * * * * * * */

ComFunc cmdHelp, cmdQuit, cmdNotImpl, cmdUnknown;


/* * * * * * * * * * * * * * * * *
 * Command name -> handler mapping
 * * * * * * * * * * * * * * * * */

COMMAND commands[] = {
	{ "help",	cmdHelp,	"Display help on the given command." },
	{ "?",		cmdHelp,	"Display help on the given command." },
	{ "quit",	cmdQuit,	"Exit this program." },
	{ "exit",	cmdQuit,	"Exit this program." },
	{ "where",	cmdNotImpl,	"Display the current robot position." },
	{ "ready", 	cmdNotImpl,	"Move to the given ready location." },
	{ "moveto", 	cmdNotImpl,	"Move the robot to the given absolute position." },
	{ "moveby", 	cmdNotImpl,	"Move the robot to the given position relative to the current." },
	{ "move", 	cmdNotImpl,	"Move the specified Joint by the given distance [mm/degrees]." },
	{ "move3step",	cmdNotImpl,	"Move the robot to the given absolute position using 3 steps." },
	{ "raise",	cmdNotImpl,	"Raise the robot Z-axis to its top position." },
	{ "home",	cmdNotImpl,	"Run the homing routine." },
	{ "activate",	cmdNotImpl,	"Activate the robot joints." },
	{ "deactivate",	cmdNotImpl,	"Deactivate the robot joints." },
	{ "limits",	cmdNotImpl,	"Display the current robot software limits" },
	{ "editlimits",	cmdNotImpl,	"Edit the current robot software limits" },
	{ "cue_home",	cmdNotImpl,	"Move pool cue to home position." },
	{ "cue_where",	cmdNotImpl,	"Print the current pool cue position." },
	{ "cue_shoot",	cmdNotImpl,	"Perform time-based shot with the given parameters." },
	{ "cue_shoot",	cmdNotImpl,	"Perform position-based shot with the given parameters." },
	{ "",		cmdUnknown,	"Unknown command." },	// Empty string signals end of list.
};


/* * * * * * * * * *
 * Global variables
 * * * * * * * * * */

ofstream Log("shell.log");	// Log messages go into this file.
// *** Robot robot;		// Robot object
string cmdline;			// Holds current command-line input from user
list<string> ComplMatches;	// Queue of current matches in the completion algorithm
string Prompt = ">>> ";		// User prompt
bool quit = false;		// Set to true when program should quit


/* * * * * * * * * * * * * * *
 * Misc. forward declarations
 * * * * * * * * * * * * * * */

char* commandCompletion (const char* text, int state);
string trim (char* s);
list<string> split (const string& s);
void execute (const string& cmd, list<string>& words);


/* * * * * * * * * * * * * * * * * * * *
 * Main function. Execution starts here
 * * * * * * * * * * * * * * * * * * * */

int main (int argc, char *argv[]) {

	// Initialization
// ***	robot = Robot("/dev/ttyS1", "/var/qpool/robot_limits.dat", Log);
// ***	poolcue_open()

	// Set up custom command-completion function
	rl_completion_entry_function = commandCompletion;

	while (!quit) {
		char* input = readline(Prompt.c_str());
		if (input) {
			cmdline = trim(input);		// remove leading and trailing whitespace
			free(input);
			input = (char*) NULL;
		}
		else {					// user typed in Ctrl-D: Quit.
			cmdline = "exit";
			cout << "exit" << endl;
		}

		if (cmdline.size() <= 0) continue;	// skip blank lines

		add_history(cmdline.c_str());
		list<string> words = split(cmdline);
		string cmd = words.front();		// separate command from arguments
		words.pop_front();
		Log << "Executing \"" << cmd << "\" with " << words.size() << " parameters." << endl;
		execute(cmd, words);
	}

	cout << "Bye." << endl;
	Log.close();
}


/* * * * * * * * *
 * Misc. functions
 * * * * * * * * */

// Perform command completion on the given text. For more information on this, see readline.info.
char* commandCompletion (const char* text, int state) {
	if (state == 0) {	// New match. Generate new queue of matches
		ComplMatches.clear();
		for (unsigned int i = 0; commands[i].name.length() > 0; i++) {
			if (commands[i].name.compare(0, strlen(text), text) == 0) {
				// Current command starts with contents of 'text': Add command to queue
				ComplMatches.push_back(commands[i].name);
			}
		}
	}

	// Pop the next string off the queue and return it. Return NULL only if queue is empty
	if (ComplMatches.empty()) return (char*) NULL;
	else {
		string match = ComplMatches.front();
		ComplMatches.pop_front();
		char* buffer = (char*) malloc((match.length() + 1) * sizeof(char));
		buffer[match.copy(buffer, string::npos)] = '\0';
		return buffer;
	}
}

// Strip whitespace from the start and end of s. Return result as a string object.
string trim (char* s) {
	char *start, *end;

	// Increment start pos while whitespace is found
	for (start = s; isspace((int) *start); start++);

	if (*start != 0) {	// Repeat for trailing whitespace only if there is anything left
		end = start + strlen(start) - 1;
		while (end > start && isspace((int) *end)) end--;
		*++end = '\0';	// Set new terminator
	}

	return string(start);
}

// Split the given string into a list of words.
list<string> split (const string& s) {
	istringstream line(s);
	string word;
	list<string> words;
	while (line >> word) words.push_back(word);
	return words;
}

// Execute the given command with the given arguments
void execute (const string& cmd, list<string>& args) {
	unsigned int i = 0;
	while (commands[i].name.length() > 0) {
		if (commands[i].name == cmd) break;
		i++;
	}
	(*(commands[i].func))(args);
}


/* * * * * * * * * *
 * Command Handlers
 * * * * * * * * * */

void cmdHelp (list<string>& args) {
	cout << "This is the help function which is not implemented yet..." << endl;
}

void cmdQuit (list<string>& args) {
	quit = true;
}

void cmdNotImpl (list<string>& args) {
	cout << "This command is not implemented." << endl;
	cout << "Arguments:" << endl;
	string arg;
	while (args.size() > 0) {
		arg = args.front();
		cout << "\t[" << arg << "]" << endl;
		args.pop_front();
	}
}

void cmdUnknown (list<string>& args) {
	cout << "Unknown command." << endl;
}
