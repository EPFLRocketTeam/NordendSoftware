/*  Title       : Threads
 *  Filename    : threads.h
 *  Author      : iacopo sprenger
 *  Date        : 20.02.2022
 *  Version     : 0.1
 *  Description : threads startup code
 */

#ifndef INIT_H
#define INIT_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>

/**********************
 *  CONSTANTS
 **********************/


/**********************
 *  MACROS
 **********************/

/**
 * @brief 	macro to declare a static thread in FreeRTOS
 * @details	This macros make the necessary funtion calls to setup a stack and
 * 			working area for the declaration of a static FreeRTOS thread.
 *
 * @param	handle	A @p TaskHandle_t object to reference the created Thread.
 * @param	name	A name for thread.
 * @param 	func	The entry point for the thread.
 * @param 	cont	The context for the thread.
 * @param 	sz		The desired size for the thread stack.
 * @param	prio	The priority for the thread.
 */
#define INIT_THREAD_CREATE(handle, name, func, cont, sz, prio) \
	static StaticTask_t name##_buffer; \
	static StackType_t name##_stack[ sz ]; \
	handle = xTaskCreateStatic( \
			func, \
	        #name, \
			sz, \
			( void * ) cont, \
			prio, \
			name##_stack, \
			&name##_buffer)

/**********************
 *  TYPEDEFS
 **********************/


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif


void init(void);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* INIT_H */

/* END */
