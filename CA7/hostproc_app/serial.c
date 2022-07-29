// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>

#include "serial.h"

struct serial_dev {
    int port;
    struct termios tty;
};



int serial_setup(serial_dev_t * serial) {


    //open feedback serial channel
	serial->port = open("/dev/ttyRPMSG0", O_RDWR);

	// Check for errors
	if (serial->port < 0) {
	    printf("Error %i from open: %s\n", errno, strerror(errno));
        return EXIT_FAILURE;
	}



	if(tcgetattr(serial->port, &serial->tty) != 0) {
	    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return EXIT_FAILURE;
    }

    serial->tty.c_cflag &= ~PARENB; //clear parity bit
    serial->tty.c_cflag &= ~CSTOPB; //only one stop bit

    serial->tty.c_cflag &= ~CSIZE;
    serial->tty.c_cflag |= CS8; //8 bits characters

    serial->tty.c_cflag &= ~CRTSCTS; //no flow control

    serial->tty.c_cflag |= CREAD | CLOCAL;

    serial->tty.c_lflag &= ~ICANON; //characters processed all the time
    serial->tty.c_lflag &= ~ECHO; // Disable echo
    serial->tty.c_lflag &= ~ECHOE; // Disable erasure
    serial->tty.c_lflag &= ~ECHONL; // Disable new-line echo

    serial->tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    serial->tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

    serial->tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    serial->tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    serial->tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed


	//non blocking mode, return immediately
    serial->tty.c_cc[VTIME] = 0;
    serial->tty.c_cc[VMIN] = 0;

	//not sure if relevant as it is a virtual uart
	cfsetospeed(&serial->tty, 115200);

	if (tcsetattr(serial->port, TCSANOW, &serial->tty) != 0) {
	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return EXIT_FAILURE;
	}

    return EXIT_SUCCESS;
}
