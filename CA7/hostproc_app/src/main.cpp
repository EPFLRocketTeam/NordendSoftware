// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>

#include "kalman.h"
#include "feedback.h"
#include "serial.h"



int main(void) {
    //spawn feedback thread

    pthread_t feedback_thread;
    pthread_t kalman_thread;

    pthread_create(&feedback_thread, NULL, feedback_entry, NULL);
    pthread_create(&kalman_thread, NULL, matrix_test, NULL);

    for(;;) {
        sleep(1);
    }

    //wait for threads to finish
    pthread_join(feedback_thread, NULL);


}