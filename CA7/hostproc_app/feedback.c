
#include <stdlib.h>
#include <stdio.h>

#include <unistd.h>


#include "serial.h"
#include "feedback.h"

void * feedback_entry(void * ptr) {
    //initialization
    serial_dev_t feedback_device;

    serial_setup(&feedback_device, "/dev/ttyRPMSG0");

    //first message for setup
    const char * msg = "hello";
    serial_send(&feedback_device, msg, 6);

    for (;;) {
        uint32_t length = serial_get_count(&feedback_device);
        if(length > 0) {
            char * msg_buffer = malloc(length);

            serial_recv(&feedback_device, msg_buffer, &length);

            fflush(stdout);
            fwrite(msg_buffer, sizeof(char), length, stdout);
            fflush(stdout);

            free(msg_buffer);
        }
        sleep(1);
    }
}