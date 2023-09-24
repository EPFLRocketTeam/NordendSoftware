/*  Title       : Debug
 *  Filename    : debug.h
 *  Author      : iacopo sprenger
 *  Date        : 04.09.2022
 *  Version     : 0.1
 *  Description : debugging
 */

/**********************
 *	INCLUDES
 **********************/

#include <main.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <device/device.h>
#include <driver/hostproc.h>
#include <feedback/debug.h>

/**********************
 *	CONSTANTS
 **********************/



/**********************
 *	MACROS
 **********************/



/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/


/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/


void debug_log(log_prio_t prio, const char * lotte, ...) {
	if(prio > MAX_PRIO) return;
	char buffer[128];
	va_list args;
	va_start(args, lotte);
	//add timestamp but without \0 char
	static char prio_name[6][4] = {
			"CRI",
			"ERR",
			"WAR",
			"IMP",
			"INF",
			"DBG"
	};

	uint16_t p_len = snprintf(buffer, 128, "[ %05lu ] %s: ", HAL_GetTick(), prio_name[prio]) - 1;
	uint16_t len = vsnprintf(buffer+p_len, 128-p_len, lotte, args);
	device_interface_send(hostproc_get_feedback_interface(), (uint8_t *) buffer, len+p_len);
	va_end(args);
}





/* END */
