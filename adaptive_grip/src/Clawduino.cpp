
#include "Clawduino.h"


Clawduino::Clawduino(Logger * log, std::string port, int baud) :
	valid(false),
	logger(log) {

	struct termios toptions;

	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		logger->log("Clawduino::Clawduino() - the port could not be opened.");
		return;
	}

	if (tcgetattr(fd, &toptions) < 0) {
		logger->log("clawduino::Clawduino() - couldn't get term options.");
		return;
	}

	speed_t brate;
	if (baud != 9600) {
		logger->log("Clawduino::Clawduino() - only a baud rate of 9600 supported.");
		return;
	} else {
		brate = B9600;
	}

	cfsetispeed(&toptions, brate);
	cfsetospeed(&toptions, brate);

	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	// no flow control
	toptions.c_cflag &= ~CRTSCTS;

	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw

	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	toptions.c_cc[VMIN]  = 0;
	toptions.c_cc[VTIME] = 20;

	if (tcsetattr(fd, TCSANOW, &toptions) < 0) {
		logger->log("clawduino::Clawduino() - couldn't get term attributes.");
		return;
	}

	valid = true; // done.
}

// send a message. Poll the reply.
int 
Clawduino::send_message(std::string input, std::string & output, int milliseconds) 
const {

	// The "sending" portion.
	const char * toWrite = input.c_str();
	int 	 sendLen = strlen(toWrite);

	int 	 wrResult = write(fd, toWrite, sendLen);

	if (!valid) {
		logger->log("Clawduino::send_message() - object is corrupt (!valid). Returning.");
		return FAILURE;
	}

	stringstream dMsg;
	dMsg << "Send the string <" << input << "> [" << sendLen << "] with a return code of " << wrResult << ".";
	logger->log(dMsg);

	// Wait for the Arduino to react.
	usleep( milliseconds * 1000 );

	// Now read from the serial port
	{
		output = "";
		char readChar[1];
		int  i = 0;
		do {
			int n = read(fd, readChar, 1);
///			cout << "Got: <" << readChar << ">" << endl;
			if (readChar[0] != '\n') {
				output += readChar[0];
			}
			if (n == -1) break;
			if (n == 0) {
				usleep (30 * 1000);
				continue;
			}
		} while (readChar[0] != '\n');

	}
///output = reply.str();
///output += "aaron";
   logger->log("OUTPUT: " + output);
	return SUCCESS;
}
