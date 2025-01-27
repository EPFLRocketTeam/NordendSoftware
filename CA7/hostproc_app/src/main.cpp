/**
 * \file main.cpp
 * \brief Main function of the Host processor application
 *
 * \date 18.08.2022
 * \author Iacopo Sprenger
 *
 *
 * \defgroup host_main Host processor linux app
 * \{
 */


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
#include "sync.h"
#include "serial.h"



int main(void) {
    //spawn feedback thread

	printf("Host Processor App\n");
	printf("Iacopo Sprenger\n");

    pthread_t feedback_thread;
    pthread_t sync_thread;
    pthread_t kalman_thread;

    pthread_create(&feedback_thread, NULL, feedback_entry, NULL);
    pthread_create(&sync_thread, NULL, sync_entry, NULL);
    pthread_create(&kalman_thread, NULL, kalman_entry, NULL);

    for(;;) {
        sleep(1);
    }

    //wait for threads to finish
    pthread_join(feedback_thread, NULL);
    pthread_join(kalman_thread, NULL);
    pthread_join(sync_thread, NULL);


}
/**
*   \}
*/
