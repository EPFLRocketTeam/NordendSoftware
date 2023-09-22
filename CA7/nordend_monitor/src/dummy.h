#ifndef DUMMY_H
#define DUMMY_H

#include "queue.h"


typedef struct feedback_data {
	queue_t * q;
}feedback_data_t;

typedef struct sync_data {
	queue_t * q;
}sync_data_t;


#ifdef __cplusplus
extern "C"{
#endif

void * dummy_feedback_entry(void * ptr);
void * dummy_sync_entry(void * ptr);

#ifdef __cplusplus
} //extern c
#endif

#endif /* DUMMY_H */
