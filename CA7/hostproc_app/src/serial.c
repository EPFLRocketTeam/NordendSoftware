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
#include "protocol/msv2.h"



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


/**
 * @brief 	initialize communicator device
 * @detail
 */
void comunicator_init(	comunicator_t * com,
						serial_dev_t serial,
						void (*cb)(uint8_t, uint16_t, uint8_t *)) {
	msv2_init(&com->msv2);
	com->cb = cb;
	com->serial = serial;
}

void comunicator_recv(comunicator_t * com) {
	uint32_t len = 1;
	for(;;) {
		uint8_t data;
		len  = 1;
		serial_recv(&com->serial, &data, &len);
		if(len == 1) {
			MSV2_ERROR_t ret = msv2_decode_fragment(&com->msv2, data);
			if(ret == MSV2_SUCCESS) {
				com->cb(com->msv2.rx.opcode, com->msv2.rx.data_len*2, com->msv2.rx.data);
			}
		}
	}
}

void comunicator_send(	comunicator_t * com,
								uint8_t opcode,
								uint16_t len,
								uint8_t * data) {

	uint16_t bin_len = msv2_create_frame(&com->msv2, opcode, len/2, data);
	serial_send(&com->serial, com->msv2.tx.data, bin_len);
}



