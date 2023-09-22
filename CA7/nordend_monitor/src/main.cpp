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
#include "queue.h"

#include "ui.h"

#include "dummy.h"

//#define DUMMY



int main(int argc, char ** argv) {
    //spawn feedback thread

	printf("Nordend Monitor App\n");
	printf("Iacopo Sprenger\n");

    pthread_t feedback_thread;
    pthread_t sync_thread;

    ui_param_t ui_param;
    feedback_data_t fb_data;
    sync_data_t sc_data;

#ifndef DUMMY
    pthread_create(&feedback_thread, NULL, feedback_entry, NULL);
    pthread_create(&sync_thread, NULL, sync_entry, NULL);
#else
    pthread_create(&feedback_thread, NULL, dummy_feedback_entry, NULL);
    pthread_create(&sync_thread, NULL, dummy_sync_entry, NULL);
#endif

    //start the UI
    ui_start(argc, argv, &ui_param);


    //wait for threads to finish
    pthread_join(feedback_thread, NULL);
    pthread_join(sync_thread, NULL);


}
/**
*   \}
*/
