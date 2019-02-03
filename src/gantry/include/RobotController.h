#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "CRScomm.h"

#include <iostream>	// Standard C++ I/O facilities
#include <string>	// Standard C++ string container

/* Goal: The following code should work:
 *	RobotController r;
 *	if (!r.connected()) ...
 *	...
 *	r << "STATUS" << block;	// Send "STATUS" command to controller and wait for command to finish
 *				// A command is finished iff the prompt has been returned from the controller
 *				// AND all movement has finished
 *
 *	r << "STATUS";		// Send "STATUS" command to controller and return immediately
 *	r << block;		// Wait for previous command to finish. If already finished, return immediately
 * Output (<<) modifiers:
 *	- block		Wait until command is finished executing. Do not empty receiveBuffer
 *	- finish	Wait until command is finished executing. Empty receiveBuffer
 */

class RobotController {

public:
	std::ostream*	log;		// Pointer to where debug/status information should be sent
					// Must be public to allow access from functions in RobotManipulators namespace
	
	// Constructor
	RobotController (
		const std::string& _serialport	= "/dev/ttyUSB0",
		std::ostream& _log		= std::cerr);

	// Destructor
	~RobotController ();

	// Return true if connection to robot is establisher, false otherwise
	const bool connected () { return connection; }
	
	// Send a command to the robot controller
	RobotController& operator<< (const std::string& command);
	
	// Accept a void static function pointer as part of a robot controller command
	RobotController& operator<< (void (*f)(RobotController&)) { f(*this); return *this; }
	
	// Receive output from the robot controller
	RobotController& operator>> (std::string& result);
	
	// Wait until the currently executing command is finished. Leave controller output untouched in buffer
	bool block ();

	// Wait until the currently executing command is finished. Empty buffer.
	bool finish ();
	
	// Return true if the given string equals the start of the receive buffer.
	// Wait automatically until enough input is available to do comparison
	bool expect (const std::string& s);

	// Consume input from controller until given string appears in input. All input up to and including given string is removed.
	bool skipTo (const std::string& s);

private:
	static const unsigned int WAIT_TIMEOUT = 120; // Do not wait more than this number of seconds for anything from controller

	std::string	serialport;	// Filename of serial port where robot controller is connected
	
	// False if no connection to robot, true if connection is established connection is established iff terminalMode is true
	bool	connection;
	
	// Low-level communication handle
	CRScomm	controller;
	
	// This string holds the characters received from the robot controller.
	std::string receiveBuffer;

	// Routine for triggering controller to change from ACI mode to Terminal mode
	void ACI_to_Term ();
	
	// Connect to robot controller and initialize robot
	void connect ();
	
	// De-initialize robot and disconnect from robot controller
	void disconnect ();

	// Send the given command string to the the robot controller and return immediately.
	// Return true on success, false on failure
	bool sendCommand (const std::string& command);

	// Receive data currently available from the controller and append to receiveBuffer
	void receiveData ();

	// throw away all unread bytes that have arrived from the controller return true if anything was actually thrown away
	bool emptyBuffer ();
};

namespace RobotManipulators {
	void block(RobotController& r);		// static version of block() member for use in '<<' notation
	void finish(RobotController& r);	// static version of finish() member for use in '<<' notation
}

#endif // ROBOT_CONTROLLER_H
