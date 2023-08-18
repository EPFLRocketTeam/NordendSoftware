#include <stdlib.h>
#include <pthread.h>
#include <string.h>

#include "queue.h"



struct queue {
	void * buffer;
	unsigned len;
	unsigned size;

	unsigned last_i;
	unsigned next_i;
};




queue_t * queue_init(unsigned size, unsigned len) {
	queue_t * q = malloc(sizeof(queue_t));
	q->buffer = malloc(len*size);

	q->len = len;
	q->size = size;

	q->last_i = 0;
	q->next_i = 0;

	return q;
}


int queue_push(queue_t * q, void * elem) {
	//check for collisions
	memcpy(q->buffer + (q->next_i*q->size), elem, q->size);
	q->next_i += q->size;
	if(q->next_i >= q->size*q->len) { //wrap around the circular buffer
		q->next_i = 0;
	}
}

int queue_pop(queue_t * q, void * elem) {
	//check for collisions
	memcpy(elem, q->buffer + (q->last_i*q->size), q->size);
	q->last_i += q->size;
	if(q->last_i >= q->size*q->len) { //wrap around the circular buffer
		q->last_i = 0;
	}
}

int queue_is_empty(queue_t * q) {
	if(q->next_i == q->last_i) {
		return 1;
	} else {
		return 0;
	}
}

unsigned queue_get_size(queue_t * q) {
	return q->size;
}



int queue_get_elem(queue_t * q, int i) {
	int idx = q->next_i - i*q->size;
	if(idx < 0) {
		idx += q->size*q->len;
	}
}




/**
 * ring buffer for text rollback
 */








