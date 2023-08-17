#ifndef QUEUE_H
#define QUEUE_H

#include <stdlib.h>
#include <stddef.h>


typedef struct queue queue_t;

#ifdef __cplusplus
extern "C"{
#endif


queue_t * queue_init(unsigned size, unsigned len);

unsigned queue_get_size(queue_t * q);

int queue_push(queue_t * q, void * elem);
int queue_pop(queue_t * q, void * elem);
int queue_is_empty(queue_t * q);



#ifdef __cplusplus
} //extern c
#endif


#endif /* QUEUE_H */
