/*  Title       : Debug
 *  Filename    : debug.h
 *  Author      : iacopo sprenger
 *  Date        : 04.09.2022
 *  Version     : 0.1
 *  Description : debugging
 */


#ifndef DEBUG_H
#define DEBUG_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>

/**********************
 *  CONSTANTS
 **********************/

typedef enum log_prio {
	LOG_CRITICAL,
	LOG_ERROR,
	LOG_WARNING,
	LOG_IMPORTANT,
	LOG_INFO,
	LOG_DEBUG
}log_prio_t;


/**********************
 *  MACROS
 **********************/


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


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

void debug_log(log_prio_t prio, const char * lotte, ...);

#endif /* DEBUG_H */

/* END */
