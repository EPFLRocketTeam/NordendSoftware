#ifndef UI_H
#define UI_H


#include "queue.h"

typedef struct ui_param {
	queue_t * q_fb;
	queue_t * q_sc;
}ui_param_t;



#ifdef __cplusplus
extern "C"{
#endif


int ui_start(int argc, char ** argv, ui_param_t * param);

#ifdef __cplusplus
} //extern c
#endif

#endif /* UI_H */
