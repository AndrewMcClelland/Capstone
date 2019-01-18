#include "CRScomm.h"

int CRS_init(CRScomm *controller) {
	struct termios backup, current;
	int fd, retval;

	fd = open(controller->serialport, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) { perror("CRS_init(): Unable to open serial port"); return -1; }

	if (fcntl(fd, F_SETFL, 0)) { perror("CRS_init(): Unable to lock serial port"); return -1; }

	if (tcgetattr(fd, &backup)) { perror("CRS_init(): Unable to retrieve serial port settings"); return -1; }

	retval = tcgetattr(fd, &current);		// start with old settings
	cfmakeraw(&current);				// enable raw mode
	current.c_iflag |= (IXON|IXOFF);		// enable XON/XOFF flow control
	current.c_cc[VMIN] = 0;				// set # chars to satisfy read
	current.c_cc[VTIME] = 0;			// set no timeout. Decisecond resolution is too coarse for our purposes
	retval += cfsetispeed(&current, B38400);	// set baudrates
	retval += cfsetospeed(&current, B38400);
	retval += tcflush(fd, TCIOFLUSH);		// flush buffers
	retval += tcsetattr(fd, TCSANOW, &current);	// activate new settings
	if (retval) { perror("CRS_init(): Unable to set serial port settings"); return -1; }

	controller->_fd = fd;
	controller->_backup = backup;
	controller->_valid = true;
	return 0;
} // end CRS_init()

void CRS_close (CRScomm *controller) {
	assert(controller->_valid);
	controller->_valid = false;

	if (tcflush(controller->_fd, TCIOFLUSH)) perror("CRS_close(): Unable to flush serial port");
	if (tcsetattr(controller->_fd, TCSANOW, &controller->_backup))
		perror("CRS_close(): Unable to restore old serial port settings");
	if (close(controller->_fd)) perror("CRS_close(): Unable to close serial port");
} // end CRS_close()

int CRS_send (const CRScomm *controller, const Byte data[], const Size length) {
	int numbytes;

	assert(controller->_valid);
	CRS_DEBUG Bytes_print(data, length, 0);
	numbytes = write(controller->_fd, data, length);
	CRS_DEBUG printf(" - %i bytes written.\n", numbytes);
	return numbytes;
} // end CRS_send()

Size CRS_receive (const CRScomm *controller, Byte data[], const Size length) {
	Size numbytes = 0;
	int retval;
	struct timeval start, now, diff;

	assert(controller->_valid);
	gettimeofday(&start, NULL);	// reset timer
	while (length > numbytes) {	// still room for more stuff in buffer
		retval = read(controller->_fd, &data[numbytes], (length - numbytes));
		if (retval == -1) { // read failed
			perror("CRS_receive(): read() failed");
			break;
		}
		else if (retval == 0) { // read nothing
			// Check: Have we timed out?
			gettimeofday(&now, NULL); // capture current time
			timediff(&start, &now, &diff); // calculate elapsed time
			if (tv_to_usec(&diff) >= RECEIVE_TIMEOUT) { // compare to timeout
				CRS_DEBUG fprintf(stderr, "CRS_receive(): timed out.\n");
				break;
			}
		}
		else { // read something
			assert(retval > 0);
			numbytes += retval;		// increment byte counter
		}
	} // end while
	CRS_DEBUG Bytes_print(data, numbytes, 0);
	CRS_DEBUG printf(" - %i bytes received.\n", numbytes);
	return numbytes;
} // end CRS_receive()

int CRS_expect (const CRScomm *controller, const Byte expected[], const Size length) {
	Byte data[length];
	Size i, retval;
	
	assert(controller->_valid);
	retval = CRS_receive(controller, data, length);
	if (retval != length) {
		fprintf(stderr, "CRS_expect(): Unexpected number of bytes returned from CRS_receive()! ");
		fprintf(stderr, "Expected %i, but got %i\n", length, retval);
		return -1;
	}
	assert(retval == length);
	for (i=0; i<length; i++) if (data[i] != expected[i]) return 1; // search for mismatches
	return 0;
} // end CRS_expect()

void Bytes_print (const Byte data[], const Size length, const int newline) {
	Size i;

	printf("[ ");
	for (i=0; i<length; i++) printf("%02hhx ", data[i]);
	printf("]");
	if (newline) printf("\n");
} // end Bytes_print()

void timediff (const struct timeval *first, const struct timeval *last, struct timeval *result) {
	if (first->tv_usec > last->tv_usec) {
		result->tv_usec = 1000000 + last->tv_usec - first->tv_usec;
		result->tv_sec = last->tv_sec - first->tv_sec - 1;
	}
	else {
		result->tv_usec = last->tv_usec - first->tv_usec;
		result->tv_sec = last->tv_sec - first->tv_sec;
	}
} // end timediff()

long tv_to_usec (const struct timeval *t) {
	long us = t->tv_sec * 1000000;
	us += t->tv_usec;
	return us;
} // end tv_to_usec
