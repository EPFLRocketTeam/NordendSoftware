/**
 * @file 		engine_control.c
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

#include <adc.h>

#include <od/od.h>

#include <cmsis_os.h>

#include <driver/serial.h>
#include <device/device.h>
#include <device/i2c_sensor.h>
#include <driver/hostproc.h>

#include <feedback/led.h>
#include <feedback/debug.h>

#include <abstraction/gpio.h>
#include <engine_control.h>

/**********************
 *	CONSTANTS
 **********************/

#define CONTROL_HEART_BEAT	200
#define FINAL_COUNTDOWN 999 //must be changed by wanted value

/**********************
 *	MACROS
 **********************/

/**********************
 *	TYPEDEFS
 **********************/

/**
 * @brief State of the control FSM
 * 
 * the description of the states don't make sense -> must be reviewed
 */
typedef enum control_state {
	/** Wait for arming or calibration */
	CONTROL_IDLE = 0,
	/** Calibrate sensors and actuators */
	CONTROL_CALIBRATION = 1,
	/** Armed, wait for liftoff */
	CONTROL_VENTING = 2,
	/** Powered ascent */
	CONTROL_PRESSURISATION = 3,
	/** Supersonic flight */
	CONTROL_COUNTDOWN = 4,
	/** Subsonic, coast flight */
	CONTROL_GLIDE = 5,
	/** Apogee reached, trigger first event */
	CONTROL_IGNITION = 6,
	/** Drogue chute descent, wait for second event */
	CONTROL_POWERED = 7,
	/** Low alt reached, trigger second event */
	CONTROL_THRUST = 8,
	/** Main chute descent, wait for touchdown */
	CONTROL_SHUTDOWN = 9,
	/** Touchdown detected, end of the flight */ //-> should be end of propulsion only
	CONTROL_APOGEE = 10,
	/** Ballistic flight detected */
	CONTROL_DEPRESSURISATION = 11,
	/** Auto triggered error */
	CONTROL_ERROR = 12,
	/** User triggered error */
	CONTROL_ABORT = 13
} control_state_t;

typedef struct control {
	control_state_t state;
	control_state_t prev_state;
} control_t;

/**********************
 *	VARIABLES
 **********************/
//shoudn't it be static ?
control_t control;

/**********************
 *	PROTOTYPES
 **********************/

void control_idle_start(void);
void control_idle_run(void);

void control_calibration_start(void);
void control_calibration_run(void);

void control_venting_start(void);
void control_venting_run(void);

void control_pressurisation_start(void);
void control_pressurisation_run(void);

void control_countdown_start(void);
void control_countdown_run(void);

void control_glide_start(void);
void control_glide_run(void);

void control_ignition_start(void);
void control_ignition_run(void);

void control_powered_start(void);
void control_powered_run(void);

void control_thrust_start(void);
void control_thrust_run(void);

void control_shutdown_start(void);
void control_shutdown_run(void);

void control_apogee_start(void);
void control_apogee_run(void);

void control_depressurisation_start(void);
void control_depressurisation_run(void);

void control_error_start(void);
void control_error_run(void);

void control_abort_start(void);
void control_abort_run(void);

void (*control_fcn[])(void) = {
	control_idle_run,
	control_calibration_run,
	control_venting_run,
	control_pressurisation_run,
	control_countdown_run,
	control_glide_run,
	control_ignition_run,
	control_powered_run,
	control_thrust_run,
	control_shutdown_run,
	control_apogee_run,
	control_depressurisation_run,
	control_error_run,
	control_abort_run
};

static void prev_state_start(void);

/**********************
 *	DECLARATIONS
 **********************/

/**
 * @brief 	Control thread entry point
 * @details This thread holds the main state machine of the Nordend software. It will be
 * 			the main decision point for actions to be taken with respect to real world events.
 *
 *
 * @param	arg	freertos thread entry point context (unused)
 *
 */
void engine_control_thread(__attribute__((unused)) void *arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(CONTROL_HEART_BEAT);
	last_wake_time = xTaskGetTickCount();

	control_idle_start();
	uint16_t checkpoint = led_add_checkpoint(led_blue);
	debug_log("Control start\n");

	for (;;) {

		//read battery TODO
//		HAL_ADC_Start(&hadc1);
//
//		HAL_ADC_PollForConversion(&hadc1, 10);
//
//		uint32_t bat1 = HAL_ADC_GetValue(&hadc1);
//
//		//convert to millivolt
//
//		od_write_BATTERY_A(&bat1);
//
//		HAL_ADC_Start(&hadc1);
//
//		HAL_ADC_PollForConversion(&hadc1, 10);
//
//		uint32_t bat2 = HAL_ADC_GetValue(&hadc1);
//
//		//convert to millivolt
//
//		od_write_BATTERY_B(&bat2);

		//how are radio instructions implemented ?

		led_checkpoint(checkpoint);
		//debug_log("Control loop | state: %d\n", control.state);

		// Call the function associated with the current state.
		control_fcn[control.state]();

		vTaskDelayUntil(&last_wake_time, period);
	}
}

/**
 * @brief	Idle state entry (aka Ground state)
 * @details This function resets the state machine into it's (default) Idle
 * 			state. The idle state is used as 'default' state before liftoff and after touchdown
 * 			It can also be accessed by the error and glide state.
 */
void control_idle_start(void) {
	control.state = CONTROL_IDLE;
}

/**
 * @brief	Idle state runtime
 * @details The Idle state will simply wait for the calibration, venting, pressurisation and countdown
 * 			command/action to happen.
 */
void control_idle_run(void) {

	// TODO Check battery state, if charge disconnected scream!
	if (/* charge_disconnected */ ) {
		// Log("charge disconnected")
		control_error_start();
	}

	// Does nothing, control_thread will loop until further instructions
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
 * 			back to Idle or go to Error.
 */
void control_calibration_run(void) {
	uint8_t error_calibration = 0; //calibration()

	//Run the calibration subroutines and log for errors

	if (error_calibration) {
		control_error_start();
		return;
	}

	control_idle_start();
}

/**
 * @brief Venting state entry
 * @details This function will initiate the venting sequence.
 * 			This function can be called as many times as needed and is initiated by radio/remotely.
 * 			It can also be initiated by glide.
 */
void control_venting_start(void) {
	control.state = CONTROL_VENTING;
}

/**
 * @brief Venting state runtime
 * @details This state will wait for the venting sequence to finish and jump back to its previous state.
 * 			This function will open/close the venting valves.
 * 			---->Should we be making an vent-open and vent-close state?
 */
void control_venting_run(void) {
	uint8_t error_venting = 0;

	//Open or close the venting valves while logging for errors
	//Venting valves are controlled by solenoids on 2 pins from 3,4,5,6 (needs crosscheck)

	if (error_venting) {
		control_error_start();
		return;
	}

	// TODO define venting valve behavior

	//must correct so that it goes back to glide if called from there
	// TODO remove
	//	switch (control.prev_state) {
	//		case CONTROL_IDLE:
	//			control_idle_start();
	//			break;
	//		case CONTROL_GLIDE:
	//			control_glide_start();
	//			break;
	//		default:
	//			// avoids undefined behavior
	//			control_idle_start();
	//			 break;
	//	}

	// Return to previous state
	control.state = control.prev_state;

}

/**
 * @brief Pressurisation state entry.
 * @details This function will initiate the pressurisation sequence.
 * 			This function can be called as many times as needed and is initiated by radio/remotely.
 * 			It could also be initiated by glide (tbd).
 */
void control_pressurisation_start(void) {
	control.state = CONTROL_PRESSURISATION;
}

/**
 * @brief Pressurisation state runtime
 * @details This state will wait for the pressurisation sequence to finish and jump back to its previous state.
 * 			This state will open/close the N20 pressurisation valve.
 */
void control_pressurisation_run(void) {
	uint8_t error_pressurisation = 0; //pressurisation()

	//Open/close the pressurisation valve while logging for errors
	//Press valve controlled by solenoid on one of the 3-6 pins

	if (error_pressurisation) {
		control_error_start();
		return;
	}

	control_idle_start();
}

/**
 * @brief	Countdown state entry
 * @details	This function will initiate countdown and will launch the engine sequence altogether.
 * 			This state is initiated by radio/remotely.
 */
void control_countdown_start(void) {
	control.state = CONTROL_COUNTDOWN;
}

/**
 * @brief	countdown state runtime
 * @details	This state will wait for the countdown to end and will jump to ignition.
 */
void control_countdown_run(void) {
	uint8_t countdown = FINAL_COUNTDOWN;
	do {
		//delay
		osDelay(CONTROL_HEART_BEAT); // TODO define desired countdown incrementation delay
		//check 

		if (/*error_detected*/) {
			control_error_start();
			return;
		} else if (/*abort needed*/) {
			control_abort_start();
			return;
		}

		countdown--;
	} while (countdown);
	control_ignition_start();
}

/**
 * @brief	Glide state entry
 * @details This function resets the state machine into it's (default) Glide.
 * 			state. The glide state is used as 'default' state before during flight.
 * 			It can also be accessed and access the venting/pressurisation and abort states.
 */
void control_glide_start(void) {
	control.state = CONTROL_GLIDE;
}

/**
 * @brief	Glide state runtime
 * @details The glide state will simply wait for the depressurisation action to start and finish
 *			It will also wait until touchdown then return to idle.
 */
void control_glide_run(void) {
	if (/*depressurisation_needed()*/) {
		control_start_depressurisation_start();
		return;
	}
	if (/* landing() */) {	//-> landing() gives a 1 once it has landed, else 0
		control_idle_start();
		return;
	};

}

/**
 * @brief	Ignition state entry
 * @details	This function will initiate the ignition sequence
 */
void control_ignition_start(void) {
	control.state = CONTROL_IGNITION;

}

/**
 * @brief Ignition state runtime
 * @details	Ignition will send the appropriate current for the igniter to turn on.
 * 			This state will wait for ignition to end and will jump to powered.
 */
void control_ignition_run(void) {
	uint8_t error_ignition = 0; //ignition() -> will turn the igniter on (solenoids)

	//Set ethanol and N2O servos (pins 13 and 14) to partially open
	//Activate ignition

	//Check if good engine start (pressure and temp?), if too many failed abort

	if (/*error_ignition == IGNITION_ABORT*/) {
		control_abort_start();
		return;
	} else if (/*error_ignition == IGNITION_ERROR*/) {
		//turn off solenoids()
		control_error_start(); //or new function ?
		return;
	}
	control_powered_start();
}

/**
 * @brief	Powered state entry
 * @details	This function will initiate the powered sequence.
 */
void control_powered_start(void) {
	control.state = CONTROL_POWERED;
}

/**
 * @brief	powered state runtime
 * @details	This state will open the servos to their 'partially open' state.
 * 			After a delay, it will jump to the thrust state.
 */
void control_powered_run(void) {
	uint8_t error_powering = 0; //powering() -> partial open state of servos
	if (error_powering) {
		control_abort_start();
		return;
	}
	control_thrust_start();
}

/**
 * @brief	Thrust state entry.
 * @details This function will initiate the thrust sequence.
 */
void control_thrust_start(void) {
	control.state = CONTROL_THRUST;
}

/**
 * @brief	Thrust state runtime
 * @details	This state will open the servos to their 'fully open' position.
 * 			After a delay, it will jump to the shutdown state.
 */
void control_thrust_run(void) {
	uint8_t error_thrust = 0; //thrust() -> full open state of servos
	if (error_thrust) {
		control_abort_start();
		return;
	}
	control_shutdown_start();
}

/**
 * @brief	Shutdown state entry
 * @details	This function will initiate the shutdown sequence.
 */
void control_shutdown_start(void) {
	control.state = CONTROL_SHUTDOWN;

}

/**
 * @brief	Shutdown state runtime
 * @details	This function will stop the engine, depending on which algorithm is chosen (before or during the apogee, tbd).
 * 			After a delay, it will jump to the apogee state.
 * 			Since we do not know if the engine has enough power to reach apogee without a full combustion, shutdown() is tbd
 */
void control_shutdown_run(void) {
	uint8_t error_shutdown = 0; //shutdown()
	if (error_shutdown) {
		control_abort_start();
		return;
	}
	control_apogee_start();
}

/**
 * @brief	Apogee state entry
 * @details	This function will initiate the apogee state/sequence.
 */
void control_apogee_start(void) {
	control.state = CONTROL_APOGEE;

}

/**
 * @brief	Apogee state runtime
 * @details	This function will open the venting valves (N20 and ethanol).
 * 			After the end of the sequence, it will jump to the depressurisation state.
 */
void control_apogee_run(void) {
	uint8_t error_start_fall = 0; //start_fall() -> venting
	if (error_start_fall) {
		control_abort_start();
		return;
	}
	control_depressurisation_start();
}

/**
 * @brief	Depressurisation state entry
 * @details	This function will initiate the depressurisation state/sequence.
 */
void control_depressurisation_start(void) {
	control.state = CONTROL_DEPRESSURISATION;

}

/**
 * @brief	Depressurisation state runtime
 * @details	This function will open the pressurisation valve (N20).
 * 			After the end of the sequence, it will jump to the glide state.
 */
void control_depressurisation_run(void) {
	uint8_t error_depressurisation = 0; //depressurisation() -> open valve
	if (error_depressurisation) {
		control_abort_start();
		return;
	}
	control_glide_start();
}

/**
 * @brief	Error state entry
 * @details	This function will initiate the error sequence.
 * 			It can be accessed by the calibration, venting, pressurisation, countdown,
 * 			ignition and powered state.
 * 			It can access the calibration state.
 */
void control_error_start(void) {
	control.state = CONTROL_ERROR;

}

/**
 * @brief	Error state runtime
 * @details	The error state will try to fix the issue and return to the previous state
 * 			in the case of calibration or idle if else.
 * 			It will send the maximum information to the GS, and have different sequences
 * 			depending on the error to try to ix the problem.
 */
void control_error_run(void) {
	//memorize_what_went_wrong_for_next_time()
	if (/* venting|pressurisation|countdown|ignition went wrong */) {
		control_calibration_start();

	}
	control_idle_start();
}

/**
 * @brief	Abort state entry
 * @details	This function will initiate the abort sequence.
 * 			This state can be accessed by the states ignition to depressurisation.
 * 			It can be triggered by radio or by other functions (like the ones giving pressure and temperature values)
 */
void control_abort_start(void) {
	control.state = CONTROL_ABORT;

}

/**
 * @brief	Abort state runtime
 * @details	This state will go to glide or idle (depending on the state of the rocket)
 * 			so that the depressurisation/venting sequences can be triggered
 * 			manually or automatically (tdb depending on the previous state or the cause of the abort?).
 */
void control_abort_run(void) {
	uint8_t state_rocket = 0; //dead_or_not_too_dead()

	if (state_rocket = 0 /*not too dead*/) {
		control_glide_start();
		return;
	}
	control_idle_start(); /*dead*/
}

/* END */
