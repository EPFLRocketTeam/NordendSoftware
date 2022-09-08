#ifndef HOSTPROC_APP_SERIAL_H
#define HOSTPROC_APP_SERIAL_H

#include <termios.h>
#include <stdint.h>
#include "protocol/msv2.h"

typedef struct serial_dev {
    int port;
    struct termios tty;
}serial_dev_t;

typedef struct comunicator {
	MSV2_INST_t msv2;
	serial_dev_t serial;
	//opcode, len, data
	void (*cb)(uint8_t, uint16_t, uint8_t *);

}comunicator_t;



#ifdef __cplusplus
extern "C"{
#endif

int serial_setup(serial_dev_t *serial, const char *file);

int serial_send(serial_dev_t *serial, uint8_t *msg, uint32_t len);

int serial_recv(serial_dev_t *serial, uint8_t *msg, uint32_t *len);

int serial_get_count(serial_dev_t *dev);

void comunicator_init(	comunicator_t * com,
						serial_dev_t serial,
						void (*cb)(uint8_t, uint16_t, uint8_t *));

void comunicator_recv(comunicator_t * com);

void comunicator_send(comunicator_t * com, uint8_t opcode, uint16_t len, uint8_t * data);

#ifdef __cplusplus
} // extern c
#endif

#endif //HOSTPROC_APP_SERIAL_H
