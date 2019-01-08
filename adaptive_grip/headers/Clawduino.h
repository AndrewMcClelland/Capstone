#ifndef CLAWDUINO__H
#define CLAWDUINO__H

#include <stdio.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
// for now..
#include "Logger.h"
#include "common_defs.h"

#define ARDUINO_REACT_MS 50
using namespace std;

class Clawduino {

public:

	Clawduino(Logger * log, std::string port="/dev/ttyUSB1", int baud = 9600);

	// Writes 'input' to the serial monitor.
	// Waits `ARDUINO_REACT_MS` * microseconds (by default), and reads the serial monitor.
	// This should trigger whatever function needs to be triggered
	// and get some response back indicating successful function entry.
	int send_message(std::string input, std::string & output, int milliseconds = ARDUINO_REACT_MS) const;

private :

	Logger * logger;

	bool 		valid;

	int		fd;

	float		resistors[3];
	

};

#endif
