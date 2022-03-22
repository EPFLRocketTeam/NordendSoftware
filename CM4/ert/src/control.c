/*  Title		: Control
 *  Filename	: control.c
 *	Author		: iacopo sprenger
 *	Date		: 20.01.2022
 *	Version		: 0.1
 *	Description	: control
 */

/**********************
 *	INCLUDES
 **********************/

#include <main.h>
#include <cmsis_os.h>
#include <control.h>
#include <led.h>

/**********************
 *	CONSTANTS
 **********************/

#define CONTROL_HEART_BEAT	100


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

void control_thread(void * arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(CONTROL_HEART_BEAT);

	led_init();

	led_set_color(LED_BLUE);

	last_wake_time = xTaskGetTickCount();

	for(;;) {


		vTaskDelayUntil( &last_wake_time, period );
	}
}


/* END */
