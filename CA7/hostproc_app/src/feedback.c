
#include <stdlib.h>
#include <stdio.h>

#include <unistd.h>
#include <sys/stat.h>


#include "serial.h"
#include "feedback.h"

#define MS2US(ms) ((ms)*1000)

void * feedback_entry(void * ptr) {
    //initialization
    serial_dev_t feedback_device;

    serial_setup(&feedback_device, "/dev/ttyRPMSG0");

    //first message for setup
    static char * msg = "hello";
    serial_send(&feedback_device, msg, 6);

    printf("setup feedback channel\n");
    //handle new file creation
    //First check exitance of file -> then increment names until file does not exist
    //maybe use the mkdir and check error code.
    static char fname[64];
    static uint16_t num = 0;
    do{
    	snprintf(fname, 64, "av_fbak%d.log", num);
    	num++;
    }while((access(fname, F_OK) == 0));
    FILE * fp = fopen(fname, "w+");

    for (;;) {
        uint32_t length = serial_get_count(&feedback_device);
        if(length > 0) {
            char * msg_buffer = malloc(length);

            serial_recv(&feedback_device, msg_buffer, &length);

            fwrite(msg_buffer, sizeof(char), length, fp);
#ifdef VERBOSE
            fflush(stdout);
            fwrite(msg_buffer, sizeof(char), length, stdout);
            fflush(stdout);
#endif
            free(msg_buffer);
        }
        //sleep for 500ms
        usleep(MS2US(500));
    }
}
