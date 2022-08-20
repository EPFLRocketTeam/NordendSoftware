/**
 * @file 		control.c
 * @brief 		Main control of the program through the global state machine
 *
 * @date 		20.01.2022
 * @author 		Iacopo Sprenger
 *
 * @defgroup 	control Control
 * @{
 */

/**********************
 *	INCLUDES
 **********************/

#include <main.h>
#include <gpio.h>
#include <usart.h>


#include <cmsis_os.h>

#include <driver/serial.h>
#include <device/device.h>
#include <device/i2c_sensor.h>
#include <driver/hostproc.h>

#include <control.h>
#include <feedback/led.h>
#include <feedback/buzzer.h>

#include <abstraction/gpio.h>


/**********************
 *	CONSTANTS
 **********************/

#define CONTROL_HEART_BEAT	200


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/

/**
 * @brief State of the control FSM
 */
typedef enum control_state {
	/** Wait for arming or calibration */
	CONTROL_IDLE = 0,
	/** Calibrate sensors and actuators */
	CONTROL_CALIBRATION = 1,
	/** Armed, wait for liftoff */
	CONTROL_ARMED = 2,
	/** Powered ascent */
	CONTROL_POWERED = 3,
	/** Supersonic flight */
	CONTROL_SUPERSONIC = 4,
	/** Subsonic, coast flight */
	CONTROL_COAST = 5,
	/** Apogee reached, trigger first event */
	CONTROL_APOGEE = 6,
	/** Drogue chute descent, wait for second event */
	CONTROL_DROGUE = 7,
	/** Low alt reached, trigger second event */
	CONTROL_EVENT = 8,
	/** Main chute descent, wait for touchdown */
	CONTROL_MAIN = 9,
	/** Touchdown detected, end of the flight */
	CONTROL_TOUCHDOWN = 10,
	/** Ballistic flight detected */
	CONTROL_BALLISTIC = 11,
	/** Auto triggered error */
	CONTROL_ERROR = 12,
	/** User triggered error */
	CONTROL_ABORT = 13
}control_state_t;




typedef struct control {
	control_state_t state;
}control_t;


/**********************
 *	VARIABLES
 **********************/

control_t control;

/**********************
 *	PROTOTYPES
 **********************/

void control_idle_start(void);
void control_idle_run(void);

void control_calibration_start(void);
void control_calibration_run(void);

void control_armed_start(void);
void control_armed_run(void);

void control_powered_start(void);
void control_powered_run(void);

void control_supersonic_start(void);
void control_supersonic_run(void);

void control_coast_start(void);
void control_coast_run(void);

void control_apogee_start(void);
void control_apogee_run(void);

void control_drogue_start(void);
void control_drogue_run(void);

void control_event_start(void);
void control_event_run(void);

void control_main_start(void);
void control_main_run(void);

void control_touchdown_start(void);
void control_touchdown_run(void);

void control_ballistic_start(void);
void control_ballistic_run(void);

void control_error_start(void);
void control_error_run(void);

void control_abort_start(void);
void control_abort_run(void);

void (*control_fcn[])(void) = {
        control_idle_run,
        control_calibration_run,
        control_armed_run,
        control_powered_run,
        control_supersonic_run,
        control_coast_run,
        control_apogee_run,
        control_drogue_run,
        control_event_run,
        control_main_run,
        control_touchdown_run,
        control_ballistic_run,
        control_error_run,
        control_abort_run
};


/**********************
 *	DECLARATIONS
 **********************/



/**
 * @brief 	Control thread entry point
 * @details This thread holds the main state machine of the WildhornAV software. It will be
 * 			the main decision point for actions to be taken with respect to real world events.
 *
 *
 * @param	arg	freertos thread entry point context (unused)
 *
 */
void control_thread(__attribute__((unused)) void * arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(CONTROL_HEART_BEAT);
	last_wake_time = xTaskGetTickCount();

    control_idle_start();

	for(;;) {


        control_fcn[control.state]();

		vTaskDelayUntil( &last_wake_time, period );
	}
}


/**
 * @brief	Idle state entry
 * @details This function resets the state machine into it's (default) Idle
 * 			state.
 */
void control_idle_start(void) {
	control.state = CONTROL_IDLE;

}

/**
 * @brief	Idle state runtime
 * @details The Idle state will simply wait for the arming command/action to
 * 			happen. It will also wait for the calibration command/action to
 * 			happen.
 */
void control_idle_run(void) {

}

/**
 * @brief	Calibration state entry
 * @details This function will setup and initiate the calibration sequence.
 */
void control_calibration_start(void) {
	control.state = CONTROL_CALIBRATION;
}

/**
 * @brief	Calibration state runtime
 * @details This state will wait for the calibration sequence to finish and jump
 * 			back to Idle.
 */
void control_calibration_run(void) {

}

/**
 * @brief	Armed state entry
 * @details This function will setup and initiate the Armed state.
 */
void control_armed_start(void) {
	control.state = CONTROL_ARMED;

}

/**
 * @brief	Armed state runtime
 * @details This state will wait for the liftoff detection (high Z acceleration)
 * 			and transition to the Powered state.
 */
void control_armed_run(void) {

}

/**
 * @brief	Powered state entry
 * @details
 */
void control_powered_start(void) {
	control.state = CONTROL_POWERED;

}

/**
 * @brief	Powered state runtime
 * @details
 */
void control_powered_run(void) {

}

/**
 * @brief	Supersonic state entry
 * @details
 */
void control_supersonic_start(void) {
	control.state = CONTROL_SUPERSONIC;

}

/**
 * @brief	Supersonic state runtime
 * @details
 */
void control_supersonic_run(void) {

}

/**
 * @brief	Coast state entry
 * @details
 */
void control_coast_start(void) {
	control.state = CONTROL_COAST;

}

/**
 * @brief	Coast state runtime
 * @details
 */
void control_coast_run(void) {

}

/**
 * @brief	Apogee state entry
 * @details
 */
void control_apogee_start(void) {
	control.state = CONTROL_APOGEE;

}

/**
 * @brief	Apogee state runtime
 * @details
 */
void control_apogee_run(void) {

}

/**
 * @brief	Drogue state entry
 * @details
 */
void control_drogue_start(void) {
	control.state = CONTROL_DROGUE;

}

/**
 * @brief	Drogue state runtime
 * @details
 */
void control_drogue_run(void) {

}

/**
 * @brief	Event state entry
 * @details
 */
void control_event_start(void) {
	control.state = CONTROL_EVENT;

}

/**
 * @brief	Event state runtime
 * @details
 */
void control_event_run(void) {

}

/**
 * @brief	Main state entry
 * @details
 */
void control_main_start(void) {
	control.state = CONTROL_MAIN;

}

/**
 * @brief	Main state runtime
 * @details
 */
void control_main_run(void) {

}

/**
 * @brief	Touchdown state entry
 * @details
 */
void control_touchdown_start(void) {
	control.state = CONTROL_TOUCHDOWN;

}

/**
 * @brief	Touchdown state runtime
 * @details
 */
void control_touchdown_run(void) {

}

/**
 * @brief	Ballistic state entry
 * @details
 */
void control_ballistic_start(void) {
	control.state = CONTROL_BALLISTIC;

}

/**
 * @brief	Ballistic state runtime
 * @details
 */
void control_ballistic_run(void) {

}

/**
 * @brief	Error state entry
 * @details
 */
void control_error_start(void) {
	control.state = CONTROL_ERROR;

}

/**
 * @brief	Error state runtime
 * @details
 */
void control_error_run(void) {

}

/**
 * @brief	Abort state entry
 * @details
 */
void control_abort_start(void) {
	control.state = CONTROL_ABORT;

}

/**
 * @brief	Abort state runtime
 * @details
 */
void control_abort_run(void) {

}


/* END */
