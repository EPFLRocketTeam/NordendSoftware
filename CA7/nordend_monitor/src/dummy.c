/*
 * this dummy file makes fake sync and feedback threads for testing on the computer
 *
 */
#include "dummy.h"
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#define MS2US(ms) ((ms)*1000)







void * dummy_feedback_entry(void * ptr) {



	for(;;) {

		sleep(1);

	}

}


void * dummy_sync_entry(void * ptr) {

	for(;;) {



		sleep(1);

	}

}
