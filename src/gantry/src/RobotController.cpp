#include "RobotController.h"

extern "C" {
#include <cctype>		// toupper()
#include <unistd.h>		// usleep()
#include <sys/time.h>		// struct timeval, gettimeofday()
}

#define LOG (*log)		// Convenient for accesing log as a normal ostream
#define SLEEP_A_LITTLE (usleep(500000))

using namespace std;

RobotController::RobotController (const string& _serialport, ostream& _log) :
	log(&_log), serialport(_serialport), connection(false), receiveBuffer("")
{
	connect();	// Initialize connection to robot controller
	if (!connected()) LOG << "RobotController::RobotController(): Cannot establish robot connection." << endl;
}

RobotController::~RobotController () {
	disconnect();
}

RobotController& RobotController::operator<< (const string& command) {
	if (!connected()) {
		LOG << "RobotController::operator<<(): Not connected! Aborting." << endl;
		return *this;
	}
	if (!sendCommand(command))
		LOG << "RobotController::operator<<(): Failed to send given command \"" << command << "\"!" << endl;
	return *this;
}

RobotController& RobotController::operator>> (string& result) {
	if (!connected()) {
		LOG << "RobotController::operator>>(): Not connected! Aborting." << endl;
		return *this;
	}

	receiveData();
	result = receiveBuffer;
	receiveBuffer.clear();
	return *this;
}

bool RobotController::block () {
	// Keep waiting until prompt is found at end of buffer
	
	// There are the valid prompts
	string prompt1 = "\r\n>>";
	string prompt2 = "\r\nO>";
	
	struct timeval start, now, diff;
	gettimeofday(&start, NULL);		// capture start time
#define max(a,b) ((a) > (b) ? (a) : (b))
	while (receiveBuffer.length() < max(prompt1.length(), prompt2.length()) || (
		receiveBuffer.compare(receiveBuffer.length() - prompt1.length(), prompt1.length(), prompt1) &&
		receiveBuffer.compare(receiveBuffer.length() - prompt2.length(), prompt2.length(), prompt2))) {
		
		// no prompt matches
		receiveData();					// try to get more data
		gettimeofday(&now, NULL);			// capture current time
		timediff(&start, &now, &diff);			// calculate elapsed time
		if (diff.tv_sec >= (int) WAIT_TIMEOUT) {	// compare to timeout
			LOG << "RobotController::block(): Timed out waiting for data!" << endl;
			return false;
		}
	}

	return true;
}

bool RobotController::finish () {
	bool retval = block();
	emptyBuffer();
	return retval;
}

bool RobotController::expect (const string&  s) {
	// Wait until enough data is available
	struct timeval start, now, diff;
	gettimeofday(&start, NULL);		// capture start time
	while (s.length() > receiveBuffer.length()) {
		gettimeofday(&now, NULL);			// capture current time
		timediff(&start, &now, &diff);			// calculate elapsed time
		if (diff.tv_sec >= (int) WAIT_TIMEOUT) {	// compare to timeout
			LOG << "RobotController::expect(): Timed out waiting for data!" << endl;
			return false;
		}
		receiveData();		// try to get more data
	}
	
	// Compare start of receiveBuffer to given string.
	// If match: Return true and remove matching string from receiveBuffer
	// Else: Return false;
	if (receiveBuffer.compare(0, s.length(), s)) { // Strings do NOT match
		return false;
	}
	receiveBuffer.erase(0, s.length());
	return true;
}

bool RobotController::skipTo (const string& s) {
	// Wait until enough data is available
	struct timeval start, now, diff;
	gettimeofday(&start, NULL);		// capture start time
	
	// Try to find s in receiveBuffer
	string::size_type i;
	while ((i = receiveBuffer.find(s)) == string::npos) {	// while s not found in buffer
		gettimeofday(&now, NULL);			// capture current time
		timediff(&start, &now, &diff);			// calculate elapsed time
		if (diff.tv_sec >= (int) WAIT_TIMEOUT) {	// compare to timeout
			LOG << "RobotController::skipTo(): Timed out waiting for data!" << endl;
			return false;
		}
		receiveBuffer.clear();	// remove everything that didn't match
		receiveData();		// try to get more data
	}
	// assume s is found in buffer
	receiveBuffer.erase(0,i);
	return true;
}

/* Private methods: */

void RobotController::ACI_to_Term () {
	// The following constants are used in preparing ACI packets for sending to the controller
	enum {
		// Special character used to start ACI communications
		ACI_START = 'R',

		// When transmitting IDs add this value to the master/slave ID:
		ADD_TO_ID = 0x20,

		// Master ID is always 0x01 in a single-master system
		MASTER_ID = 0x01 + ADD_TO_ID,
		SLAVE_ID = 0x01 + ADD_TO_ID,
		
		// Control characters/bytes used in the ACI protocol:
		SOH = 0x01,
		STX = 0x02,
		ETX = 0x03,
		EOT = 0x04,
		ENQ = 0x05,
		ACK = 0x06,
		NAK = 0x15,
		ETB = 0x17,

		// Action specifiers (byte 4 in header packet):
		ACI_WRITE = 0x00,
		ACI_READ =  0x01,

		// Special WRITE codes (byte 5 in header packet):
		ACI_TO_TERM = 0x48,
	};

	const Byte enq[] = { ACI_START, SLAVE_ID, ENQ }; const Size enq_l = 3;
	const Byte enq_r[] = { ACI_START, SLAVE_ID, ACK }; const Size enq_r_l = 3;
	const Byte header[] = { SOH, SLAVE_ID, MASTER_ID, ACI_WRITE, ACI_TO_TERM, 0, 0, 0, 0, 0, 0, ETX, 0x8a }; const Size header_l = 13;
	const Byte header_r[] = { ACK, STX, ETX, 0x00 }; const Size header_r_l = 4;
	const Byte ack[] = { ACK }; const Size ack_l = 1;
	const Byte eot[] = { EOT }; const Size eot_l = 1;
	Byte dummy;

	CRS_send(&controller, enq, enq_l);
	CRS_expect(&controller, enq_r, enq_r_l);
	CRS_send(&controller, header, header_l);
	CRS_expect(&controller, header_r, header_r_l);
	CRS_send(&controller, ack, ack_l);
	CRS_expect(&controller, eot, eot_l);
	CRS_send(&controller, eot, eot_l);
	CRS_receive(&controller, &dummy, 1);
}

void RobotController::connect () {
	if (connected()) {
		LOG << "RobotController::connect(): Already connected! Aborting." << endl;
		return;
	}

	controller.serialport = serialport.c_str();

	// Initialize low-level communication handlers
	if (CRS_init(&controller)) {
		LOG << "RobotController::connect(): CRS_init() failed! Aborting." << endl;
		return;
	}

	// switch to terminal mode (ignore errors in case we are already in terminal mode)
	ACI_to_Term();
	SLEEP_A_LITTLE;		// wait until controller has made the switch
	
	// Turn off automatic command completion
	if (!sendCommand("NOHELP")) {
		LOG << "RobotController::connect(): Failed to turn off command completion! Aborting." << endl;
		return;
	}

	emptyBuffer();		// empty receive buffer
	connection = true;	// set connection to true
}

void RobotController::disconnect () {
	if (!connected()) {
		LOG << "RobotController::disconnect(): Not connected! Aborting." << endl;
		return;
	}

	// Switch back to ACI mode
	const Byte packet[] = { 0x1a, 0x1a };
	const Size length = 2;
	int bytes_sent;
	if ((bytes_sent = CRS_send(&controller, packet, length)) != (int) length)
		LOG	<< "RobotController::disconnect(): Failed to switch to ACI mode!" << endl;
	SLEEP_A_LITTLE;	// Wait until controller has made the switch

	// Close low-level communication handlers
	CRS_close(&controller);
	
	connection = false;
}

bool RobotController::sendCommand (const string& command) {
	receiveData();
	if (receiveBuffer.length() > 0)	// First make sure that there are no unread controller messages
		LOG << "RobotController::sendCommand(): Warning: controller output discarded: " << receiveBuffer << endl;
	emptyBuffer();

	Size buflen = command.length() + 1;	// Number of chars in command + terminating CR
	Byte buffer[buflen];			// Make room for all chars in command + terminating CR
	
	string s = command;			// copy command into mutable string
	for (unsigned int i = 0; i < s.length(); i++) s[i] = toupper(s[i]);	// convert command to upper case
	s += "\r";				// add terminating CR
	s.copy((char*) buffer, buflen);		// copy characters from s to buffer

	int bytes_sent;
	if ((bytes_sent = CRS_send(&controller, buffer, buflen)) != (int) buflen) {
		LOG	<< "RobotController::sendCommand(): Failed to send command \""
			<< command << "\"! CRS_send() only sent " << bytes_sent << " out of "
			<< buflen << " bytes." << endl;
		return false;
	}
	expect(command);
	// Command sent successfully
	return true;
}

void RobotController::receiveData () {
#define BUFFER_SIZE 512
	Byte buffer[BUFFER_SIZE+1];	// leave enough space for terminating \0
	Size bytes;			// number of bytes actually read
	
	do {
		bytes = CRS_receive(&controller, buffer, BUFFER_SIZE);	// Receive bytes from controller
/* ***
LOG << "\t\treceiveData(): \"";
for (unsigned int i = 0; i < bytes; i++) {
	if (buffer[i] > 31 && buffer[i] < 127)	LOG << (char) buffer[i];
	else					LOG << "{" << (int) buffer[i] << "}";
}
LOG << "\"" << endl;
*** */
		buffer[bytes] = '\0';			// set last byte to '\0'
		receiveBuffer += (char*) buffer;	// append received byte to internal buffer
	} while (bytes == BUFFER_SIZE);			// repeat as long as buffer comes back full
#undef BUFFER_SIZE
}

bool RobotController::emptyBuffer () {
	receiveData();
	bool retval = receiveBuffer.length() > 0;
	receiveBuffer.clear();
	return retval;
}


/* Non-member function */

void RobotManipulators::block(RobotController& r) {
	if (!r.block()) *r.log << "RobotController::block(): Returned false in manipulator context!" << endl;
}

void RobotManipulators::finish(RobotController& r) {
	if (!r.finish()) *r.log << "RobotController::finish(): Returned false in manipulator context!" << endl;
}

#undef LOG
#undef SLEEP_A_LITTLE
