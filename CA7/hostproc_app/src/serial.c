// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "serial.h"



int serial_setup(serial_dev_t * serial, const char * file) {


    //open feedback serial channel
	serial->port = open(file, O_RDWR);

	// Check for errors
	if (serial->port < 0) {
	    printf("Error %i from open: %s\n", errno, strerror(errno));
        exit(EXIT_FAILURE);
	}

    return EXIT_SUCCESS;
}

int serial_send(serial_dev_t * serial, uint8_t * msg, uint32_t len) {
    write(serial->port, msg, len);
    return EXIT_SUCCESS;
}

int serial_recv(serial_dev_t * serial, uint8_t * msg, uint32_t * len) {
    *len = read(serial->port, msg, *len);
    return EXIT_SUCCESS;
}

int serial_get_count(serial_dev_t * dev) {
    int bytes;
    ioctl(dev->port, FIONREAD, &bytes);
    return bytes;
}


