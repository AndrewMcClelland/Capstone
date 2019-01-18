/* CRScomm.h
 *
 * Author: Johan Herland, <9jh33@qlink.queensu.ca>
 *
 * This file contains definitions regarding the CRS communications link
 */
#ifndef CRSCOMM_H
#define CRSCOMM_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <assert.h>

#define CRS_DEBUG if (false)

#define RECEIVE_TIMEOUT (100*1000)	// Number of microseconds (usec) before CRS_receive() times out

typedef unsigned char Byte;
typedef unsigned int Size;

typedef struct CRScomm {
	const char *serialport;	// Filename of serial port device
	int _fd;		// FILE * to open serial port
	struct termios _backup;	// Backup of serial port settings
	int _valid;		// valid flag (turned on by init, turned off by close)
} CRScomm;


/* Open and initialize the serial port for communication with a CRS robot controller
 * Preconditions:
 *	- controller->serialport must be set to the filename of the serial port device
 *	- serialport must be readable and writable
 * Postconditions:
 *	- If successful, sets controller->file, controller->_backup and controller->_valid
 * Returns: 0 on success, -1 on error
 */
int CRS_init(CRScomm *controller);

/* Restore old serial port settings and close the serial port
 * Preconditions:
 *	- controller->file is a valid FILE * for the serialport
 *	- controller->_backup contains valid serial port settings
 * Postconditions:
 *	- If successful, controller->file is no longer a valid FILE *
 * Returns: 0 on success, -1 on error
 */
void CRS_close(CRScomm *controller);

/* Send the 'length' first bytes of 'data[]' to 'controller'.
 * Preconditions:
 *	- controller must be properly initialized using CRS_init()
 *	- length <= # elements in data[]
 * Postconditions:
 *	- none
 * Returns: Number of bytes actually sent, -1 on error
 */
int CRS_send (const CRScomm *controller, const Byte data[], const Size length);

/* Receive 'length' bytes from 'controller' and deposit into 'data[]'.
 * Aborts before 'length' bytes is read if:
 *	- read operations fails
 *	- no bytes are received in 1 second
 * Preconditions:
 *	- controller must be properly initialized using CRS_init()
 *	- 'data[]' must have room for at least 'length' bytes
 * Postconditions:
 *	- none
 * Returns: Number of bytes actually retrieved
 */
Size CRS_receive (const CRScomm *controller, Byte data[], const Size length);

/* Receive 'length' bytes from 'controller' and verify that they correspond to the 'length' first bytes in 'expected[]'.
 * Aborts before 'length' bytes is read if:
 *	- read operations fails
 *	- no bytes are received in 1 second
 * Preconditions:
 *	- controller must be properly initialized using CRS_init()
 *	- length <= # elements in expected[]
 * Postconditions:
 *	- none
 * Returns: 0 if received bytes matched 'expected[]', 1 if mismatch between received bytes and 'expected[]', -1 on error
 */
int CRS_expect (const CRScomm *controller, const Byte expected[], const Size length);

/* Print the first 'length' bytes of 'data[]' in hex to stdout ending with an optional newline.
 * Adds a newline at the end if 'newline' is true (non-zero)
 * Preconditions:
 *	- length <= # elements in data[]
 * Postconditions:
 *	- none
 */
void Bytes_print (const Byte data[], const Size length, const int newline);

/* Compute the elapsed time between 'first' and 'last' and place the result in 'result'.
 * Preconditions:
 *	- 'first' represents an earlier time than 'last'
 * Postconditions:
 *	- 'result' contains the elapsed time between 'first' and 'last'
 */
void timediff (const struct timeval *first, const struct timeval *last, struct timeval *result);

/* Return the number of microseconds corresponding to the given timeval
 * Preconditions:
 *	- given timeval != NULL
 * Postconditions:
 *	- none
 * Returns: a double representing the time (in usecs) in 't'
 */
long tv_to_usec (const struct timeval *t);

#endif // CRSCOMM_H
