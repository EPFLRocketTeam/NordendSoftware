#ifndef HOSTPROC_APP_SERIAL_H
#define HOSTPROC_APP_SERIAL_H

#include <termios.h>
#include <stdint.h>

typedef struct serial_dev {
    int port;
    struct termios tty;
}serial_dev_t;


#ifdef __cplusplus
extern "C"{
#endif

int serial_setup(serial_dev_t *serial, const char *file);

int serial_send(serial_dev_t *serial, uint8_t *msg, uint32_t len);

int serial_recv(serial_dev_t *serial, uint8_t *msg, uint32_t *len);

int serial_get_count(serial_dev_t *dev);

#ifdef __cplusplus
} // extern c
#endif

#endif //HOSTPROC_APP_SERIAL_H
