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
#include <queue.h>

#include <driver/serial.h>
#include <device/device.h>
#include <device/i2c_sensor.h>
#include <driver/hostproc.h>

#include <feedback/led.h>
#include <feedback/debug.h>

#include <abstraction/gpio.h>
#include <engine_control.h>

#include <propulsion/servo.h>
#include <driver/pwm.h>


#include <sensor/engine_pressure.h>
#include <sensor/temperature.h>

/**********************
 *	CONSTANTS
 **********************/



/**
 * ENGINE CONTROL SEQUENCE
 * 
 * The engine control sequence is managed by a state machine. 
 * Here is a description of the state machine
 * 
 * IDLE
 * - The engine is not ready for fire and is not doing anything.
 * - Can move to ARMED with the ARM command.
 * 
 * CALIBRATION
 * - Calibration of engine's sensors.
 * 
 * MANUAL OPERATION
 * - Parking state for when the engine is manually controlled.
 * - Whenever the engine receives a manual operation command, it gets locked to this state.
 * - This is meant to avoid messing up the sequence, when manual valve operation is used.
 * - Can move back to IDLE by issuing the RECOVER command.
 * 
 * ARMED
 * - From this state, the system can be safely pressured.
 * - Can move to PRESSURED with the PRESSURE command.
 * 
 * PRESSURED
 * - The engine is pressured and ready for ignition.
 * - Can move to IGNITER with the IGNITE command.
 * 
 * IGNITER
 * - The igniter is fired.
 * - After IGNITER_COUNTER is elapsed, we move to IGNITION.
 * 
 * IGNITION
 * - Fuel and Oxydizer valves are partially opened.
 * - After IGNITION_COUNTER is elapsed, we move to THRUST.
 * 
 * THRUST
 * - Fuel and Oxydizer valves are fully opened.
 * - After THRUST_COUNTER is elapsed, we move to SHUTDOWN.
 * 
 * SHUTDOWN
 * - Fuel Valve is closed.
 * - After SHUTDOWN_COUNTER is elapsed, we move to GLIDE.
 * 
 * GLIDE
 * - Engine is depressured (sol_press closed and sol_purge open).
 * - Recovery is signaled that the burn is finished.
 * 
 * ERROR
 * - Automatically triggered error state.
 * - Can move to IDLE with the RECOVER command.
 * 
 * ABORT
 * - Manually triggered error state.
 * - Can move to IDLE with the RECOVER command.
 * 
 * 
 * Accepted commands for the engine control system:
 * ARM 			-> Arm the system
 * PRESSURE 	-> Pressure the system
 * IGNITE		-> Ignite the system
 * RECOVER		-> Recover from parking state
 * VENT_N2O		-> open/close the N2O venting valve
 * VENT_ETH		-> open/close the ETH venting valve
 * MAN_PRESS	-> open/close the Pressurant valve
 * MAN_PURGE	-> open/close the Purge valve
 * VALVE_N2O	-> open/close the N2O main valve
 * VALVE_ETH	-> open/close the ETH main valve
 */


#define COMMAND_QUEUE_LENGTH 	16

#define IGNITER_COUNTER 		3000  //ms
#define IGNITION_COUNTER		2000  //ms
#define THRUST_COUNTER		    10000 //ms
#define SHUTDOWN_COUNTER		0     //ms






#define CONTROL_HEART_BEAT 500



#define CONTROL_ONE_SECOND pdMS_TO_TICKS(1000)

//TODO Find IGNITER GPIO port and pin
//Igniter pin definition
#define IGNITER_PORT GPIOG
#define IGNITER_PIN  1


//#define USE_CHECKPOINT




/**********************
 *	MACROS
 **********************/



/**********************
 *	TYPEDEFS
 **********************/


typedef struct command {
	control_command_t cmd; 
	int32_t parameter;
}command_t;

typedef struct control
{
	control_state_t state;

	engine_control_data_t ec_data;

	int32_t igniter_time;
	int32_t ignition_time;
	int32_t thrust_time;
	int32_t shutdown_time;


	solenoid_t solenoid_n2o;
	solenoid_t solenoid_ethanol;
	solenoid_t solenoid_press;
	solenoid_t solenoid_purge;

	device_t *i2c_engine_press;
	device_t *i2c_engine_temp;

	servo_t servo_ethanol;
	servo_t servo_n2o;

	StaticQueue_t command_queue_params;
	QueueHandle_t command_queue;
	uint8_t command_queue_storage[sizeof(command_t)*COMMAND_QUEUE_LENGTH];

	int32_t counter;
	uint32_t time;
	uint32_t last_time;
} control_t;

/**********************
 *	VARIABLES
 **********************/

/**
 * Main control structure, with all items
 */
static control_t control;

/**
 * Last wake time for the timer
 */
static TickType_t last_wake_time;
static const TickType_t period = pdMS_TO_TICKS(CONTROL_HEART_BEAT);

/*
 * For Status of vent pins -- use OD
 */

//Servo stuff

// Specific to the SB2290SG Monster Torque Brushless Servo
#define min_pulse 800
#define max_pulse 2200
#define degrees_per_usec 0.114f


/**********************
 *	PROTOTYPES
 **********************/

void control_idle_start(void);
void control_calibration_start(void);
void control_pressured_start(void);
void control_armed_start(void);
void control_igniter_start(void);
void control_ignition_start(void);
void control_thrust_start(void);
void control_shutdown_start(void);
void control_glide_start(void);
void control_error_start(void);
void control_abort_start(void);

void control_idle_run(void);
void control_calibration_run(void);
void control_pressured_run(void);
void control_armed_run(void);
void control_igniter_run(void);
void control_ignition_run(void);
void control_thrust_run(void);
void control_shutdown_run(void);
void control_glide_run(void);
void control_error_run(void);
void control_abort_run(void);

void control_man_press(int32_t param);
void control_man_purge(int32_t param);
void control_valve_eth(int32_t param);
void control_valve_n2o(int32_t param);
void control_vent_eth(int32_t param);
void control_vent_n2o(int32_t param);

util_error_t init_engine_control();

int engine_control_command_pop(control_command_t * cmd, int32_t * parameter);

/**********************
 *	DECLARATIONS
 **********************/

///////////////////////////////////////////////////////////////////////////////////////////////////
//CONTROL THREAD INITIALIZATION
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 	Control thread entry point
 * @details This thread holds the main state machine of the Nordend propulsion software. It will be
 * 			the main decision point for actions to be taken with respect to real world events.
 *
 *
 * @param	arg	freertos thread entry point context (unused)
 *
 */

void engine_control_thread(__attribute__((unused)) void *arg) {
	init_engine_control();

	uint16_t checkpoint = led_add_checkpoint(led_blue);
	debug_log(LOG_INFO, "Control start\n");

	// Timer things
	last_wake_time = xTaskGetTickCount();

	
	for (;;) {
		led_checkpoint(checkpoint);
		debug_log(LOG_INFO, "Current state : %d\n", control.state);



		// Call the function associated with the current state.
		switch (control.state) {
		case CONTROL_IDLE:
			control_idle_run();
			break;
		case CONTROL_CALIBRATION:
			control_calibration_run();
			break;
		case CONTROL_ARMED:
			control_armed_run();
			break;
		case CONTROL_PRESSURED:
			control_pressured_run();
			break;
		case CONTROL_IGNITER:
			control_igniter_run();
			break;
		case CONTROL_IGNITION:
			control_ignition_run();
			break;
		case CONTROL_THRUST:
			control_thrust_run();
			break;
		case CONTROL_SHUTDOWN:
			control_shutdown_run();
			break;
		case CONTROL_GLIDE:
			control_glide_run();
			break;
		case CONTROL_ERROR:
			control_error_run();
			break;
		case CONTROL_ABORT:
			control_abort_run();
			break;
		default:
			// Undefined behavior / ghost state -> abort
			control_error_start();
			break;
		}


		control.ec_data.state = control.state;
		control.ec_data.time = HAL_GetTick();

		debug_log(LOG_INFO, "update EC od: %d, %d, %d, %d\n",
					control.ec_data.state,
					control.ec_data.last_cmd,
					control.ec_data.last_parameter,
					control.ec_data.time);
		od_write_ENGINE_CONTROL_DATA(&control.ec_data);

		vTaskDelayUntil(&last_wake_time, period);
		
	}
}


/**
 * @fn util_error_t init(void)
 * @brief Initializes all peripherals and sets up the control object.
 *
 * @return ER_SUCCESS if everything went well, a non-zero error code otherwise.
 */

util_error_t init_engine_control(void) {
	// Initialize the value of control
	control_idle_start();

	//init command queue
	control.command_queue = xQueueCreateStatic( 
								COMMAND_QUEUE_LENGTH,
                                sizeof(command_t),
                                control.command_queue_storage,
                                &control.command_queue_params);

	control.igniter_time = IGNITER_COUNTER;
	control.ignition_time = IGNITION_COUNTER;
	control.thrust_time = THRUST_COUNTER;
	control.shutdown_time = SHUTDOWN_COUNTER;

	// Assign Ethanol servo to pin 13 (TIM4, CH2) and N2O servo to pin 14 (TIM4, CH3)
	util_error_t ethanol_err = servo_init(
			&control.servo_ethanol,
			PWM_SELECT_CH1,
			min_pulse,
			max_pulse,
			SERVO_ETHANOL_OFFSET,
			degrees_per_usec,
			SERVO_ETHANOL_OPEN,
			SERVO_ETHANOL_IGNITION,
			SERVO_ETHANOL_CLOSED);

	util_error_t n2o_err = servo_init(
			&control.servo_n2o,
			PWM_SELECT_CH2,
			min_pulse,
			max_pulse,
			SERVO_N2O_OFFSET,
			degrees_per_usec,
			SERVO_N2O_OPEN,
			SERVO_N2O_IGNITION,
			SERVO_N2O_CLOSED);

	//init the solenoids
	solenoid_init(&control.solenoid_ethanol, 	1, GPIOC, GPIO_PIN_5);
	solenoid_init(&control.solenoid_n2o, 		1, GPIOC, GPIO_PIN_6);
	solenoid_init(&control.solenoid_purge, 		0, GPIOC, GPIO_PIN_3);
	solenoid_init(&control.solenoid_press, 		0, GPIOC, GPIO_PIN_4);

	// Using channels 1 and 2 -- initialize the PWM channel
	pwm_init();

	// Initialize servo states
	util_error_t servo_err = ER_SUCCESS;
	servo_err |= servo_set_state(&control.servo_n2o, SERVO_CLOSED);
	servo_err |= servo_set_state(&control.servo_ethanol, SERVO_CLOSED);

	// Initialize solenoids
	util_error_t sol_err = ER_SUCCESS;
	sol_err |= solenoid_inactive(&control.solenoid_ethanol);
	sol_err |= solenoid_inactive(&control.solenoid_n2o);
	sol_err |= solenoid_inactive(&control.solenoid_purge);
	sol_err |= solenoid_inactive(&control.solenoid_press);


	return ethanol_err | n2o_err | servo_err | sol_err;
}


void engine_control_command_push(control_command_t cmd, int32_t parameter) {
	command_t command;
	command.cmd = cmd;
	command.parameter = parameter;

	//post to the queue without timeout
	xQueueSend(control.command_queue, (void*) &command, 0);

	//we dont care if posted successfully or not for now...
}

int engine_control_command_pop(control_command_t * cmd, int32_t * parameter) {
	command_t command;
	//receive from queue without timeout
	if(xQueueReceive(control.command_queue, &command, 0) == pdPASS) {
		*cmd = command.cmd;
		if(parameter) {
			*parameter = command.parameter;
		}
		control.ec_data.last_parameter = command.parameter;
		control.ec_data.last_cmd = command.cmd;
		debug_log(LOG_INFO, "cmd popcat : %d (%d)\n", command.cmd, command.parameter);
		//return 1 if a command has been received
		return 1;
	} else {
		//return 0 if no commands have been received
		*cmd = COMMAND_NONE;
		*parameter = 0;
		return 0;
	}
}


control_command_t control_read_commands(control_command_t * expected_cmd, size_t expected_cmd_len, int32_t * param) {
	control_command_t cmd;
	while(engine_control_command_pop(&cmd, param) != 0) {
		for(int i = 0; i < expected_cmd_len; i++) {
			if(cmd == expected_cmd[i]) {
				return cmd;
			} else if (cmd == COMMAND_ABORT) {
				control_abort_start();
				return COMMAND_NONE;
			}
		}
	}
	return COMMAND_NONE;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//ACTUATORS MANUAL OPERATION
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Manually open/close the pressurant valve
 * param: 
 * 	1: open
 * 	0: close
*/
void control_man_press(int32_t param) {

	if(param == 1) {
		solenoid_open(&control.solenoid_press);
	} else if(param == 0) {
		solenoid_close(&control.solenoid_press);
	}

}

/**
 * Manually open/close the purge valve
 * param: 
 * 	1: open
 * 	0: close
*/
void control_man_purge(int32_t param) {

	if(param == 1) {
		solenoid_open(&control.solenoid_purge);
	} else if(param == 0) {
		solenoid_close(&control.solenoid_purge);
	}
}

/**
 * Manually move the ethanol valve
 * param: angle in deg [0 - 90]
*/
void control_valve_eth(int32_t param) {

	if(param > 0 && param < 90) {
		servo_set_rotation(&control.servo_ethanol, (float) param);
	}

}

/**
 * Manually move the n2o valve
 * param: angle in deg
*/
void control_valve_n2o(int32_t param) {

	if(param > 0 && param < 90) {
		servo_set_rotation(&control.servo_n2o, (float) param);
	}

}

/**
 * Manually open/close the eth vent valve
 * 	1: open
 * 	0: close
*/
void control_vent_eth(int32_t param) {

	if(param == 1) {
		solenoid_open(&control.solenoid_ethanol);
	} else if(param == 0) {
		solenoid_close(&control.solenoid_ethanol);
	}

}

/**
 * Manually open/close the eth vent valve
 * 	1: open
 * 	0: close
*/
void control_vent_n2o(int32_t param) {

	if(param == 1) {
		solenoid_open(&control.solenoid_n2o);
	} else if(param == 0) {
		solenoid_close(&control.solenoid_n2o);
	}

}



///////////////////////////////////////////////////////////////////////////////////////////////////
// MANUAL STATE RUNTIME FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

void control_idle_start(void) {
	control.state = CONTROL_IDLE;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_green);
#endif
}

void control_idle_run(void) {
	//check if we want to arm
	control_command_t expected_cmds[] = {
		COMMAND_ARM,
		COMMAND_MAN_PRESS,
		COMMAND_MAN_PURGE,
		COMMAND_VALVE_ETH,
		COMMAND_VALVE_N2O,
		COMMAND_VENT_ETH,
		COMMAND_VENT_N2O
	};

	control_command_t received_command;

	int32_t parameter;
	received_command = control_read_commands(expected_cmds, 7, &parameter);

	switch(received_command) {
		case COMMAND_ARM:
		control_armed_start();
		break;
		case COMMAND_MAN_PRESS:
		control_man_press(parameter);
		break;
		case COMMAND_MAN_PURGE:
		control_man_purge(parameter);
		break;
		case COMMAND_VALVE_ETH:
		control_valve_eth(parameter);
		break;
		case COMMAND_VALVE_N2O:
		control_valve_n2o(parameter);
		break;
		case COMMAND_VENT_ETH:
		control_vent_eth(parameter);
		break;
		case COMMAND_VENT_N2O:
		control_vent_n2o(parameter);
		break;
		default:
		break;
	}
}



void control_calibration_start(void) {
	control.state = CONTROL_CALIBRATION;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_lime);
#endif
}

void control_calibration_run(void) {
	//for now just go back to idle

	//check for abort
	control_command_t expected_cmds[] = {
		COMMAND_NONE
	};
	control_read_commands(expected_cmds, 1, NULL);

	control_idle_start();
}

void control_armed_start(void) {
	control.state = CONTROL_ARMED;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_yellow);
#endif
}

void control_armed_run(void) {
	//wait for ignition command

	control_command_t expected_cmds[] = {
		COMMAND_PRESSURE, 
		COMMAND_DISARM 
	};

	control_command_t received_command;

	received_command = control_read_commands(expected_cmds, 2, NULL);

	if(received_command == COMMAND_PRESSURE) { //ignition command
		control_pressured_start();
	} else if(received_command == COMMAND_DISARM) {
		control_idle_start();
	}
}


void control_pressured_start(void) {
	control.state = CONTROL_PRESSURED;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_orange);
#endif

	solenoid_open(&control.solenoid_press);
}

void control_pressured_run(void) {
	//wait for ignition command
	control_command_t expected_cmds[] = {
		COMMAND_IGNITE
	};

	control_command_t received_command;

	received_command = control_read_commands(expected_cmds, 1, NULL);

	if(received_command == COMMAND_IGNITE) { //ignition command
		control_igniter_start();
	}

}


void control_igniter_start(void) {
	control.state = CONTROL_IGNITER;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_red);
#endif

	control.counter = control.igniter_time;
	//fire igniter
	gpio_set(IGNITER_PORT, IGNITER_PIN);
}

/**
 * @brief Igniter state runtime
 * @details	Ignition will send the appropriate current for the igniter to turn on.
 * 			This state will wait for ignition to end and will jump to powered.
 */
void control_igniter_run(void) {
	control.last_time = control.time;

	//check for abort
	control_command_t expected_cmds[] = {
		COMMAND_NONE
	};

	control_read_commands(expected_cmds, 1, NULL);

	control.last_time = control.time;
	control.time = HAL_GetTick();
	control.counter -= (control.time - control.last_time);
	if (control.counter > 0) {
		return;
	}

	//shutdown igniter
	gpio_clr(IGNITER_PORT, IGNITER_PIN);

	control_ignition_start();
}

void control_ignition_start(void) {
	control.state = CONTROL_IGNITION;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_teal);
#endif

	servo_set_state(&control.servo_n2o, SERVO_PARTIALLY_OPEN);
	servo_set_state(&control.servo_ethanol, SERVO_PARTIALLY_OPEN);
	control.counter = control.ignition_time;
}


void control_ignition_run(void) {

	//check for abort
	control_command_t expected_cmds[] = {
		COMMAND_NONE
	};

	control_read_commands(expected_cmds, 1, NULL);

	control.last_time = control.time;
	control.time = HAL_GetTick();
	control.counter -= (control.time - control.last_time);
	if (control.counter > 0) {
		return;
	}


	control_thrust_start();
}

void control_thrust_start(void) {
	control.state = CONTROL_THRUST;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_pink);
#endif

	servo_set_state(&control.servo_n2o, SERVO_OPEN);
	servo_set_state(&control.servo_ethanol, SERVO_OPEN);
	control.counter = control.thrust_time;
}

/**
 * @brief	Thrust state runtime

 * @details	This state will open the servos to their 'fully open' position.
 * 			After a delay, it will jump to the shutdown state.
 */
void control_thrust_run(void) {

	//check for abort
	control_command_t expected_cmds[] = {
		COMMAND_NONE
	};

	control_read_commands(expected_cmds, 1, NULL);

	control.last_time = control.time;
	control.time = HAL_GetTick();
	control.counter -= (control.time - control.last_time);
	if (control.counter > 0) {
		return;
	}

	control_shutdown_start();
}

void control_shutdown_start(void) {
	control.state = CONTROL_SHUTDOWN;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_teal);
#endif

	servo_set_state(&control.servo_ethanol, SERVO_CLOSED);
	control.counter = control.shutdown_time;
}

void control_shutdown_run(void) {

	//check for abort
	control_command_t expected_cmds[] = {
		COMMAND_NONE
	};
	control_read_commands(expected_cmds, 1, NULL);

	control.last_time = control.time;
	control.time = HAL_GetTick();
	control.counter -= (control.time - control.last_time);
	if (control.counter > 0) {
		return;
	}

	control_glide_start();
}

void control_glide_start(void) {
	control.state = CONTROL_GLIDE;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_blue);
#endif
}

void control_glide_run(void) {

	control_command_t expected_cmds[] = {
		COMMAND_RECOVER
	};
	control_command_t received_command;
	received_command = control_read_commands(expected_cmds, 1, NULL);
	if(received_command == COMMAND_RECOVER) {
		control_idle_start();
	}



	//do nothing
}

void control_error_start(void) {
	control.state = CONTROL_ERROR;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_white);
#endif
}

/**
 * @brief	Error state runtime
 * @details	The error state will try to fix the issue and return to the previous state
 * 			in the case of calibration or idle if else.
 * 			It will send the maximum information to the GS, and have different sequences
 * 			depending on the error to try to fix the problem.
 */
void control_error_run(void) {
	//Will attempt to recover from error

	control_command_t expected_cmds[] = {
		COMMAND_RECOVER
	};
	control_command_t received_command;
	received_command = control_read_commands(expected_cmds, 1, NULL);
	if(received_command == COMMAND_RECOVER) {
		control_idle_start();
	}

}

void control_abort_start(void) {
	control.state = CONTROL_ABORT;
#ifndef USE_CHECKPOINT
	led_rgb_set_color(led_white);
#endif
}

/**
 * @brief	Abort state runtime
 * @details	This state will go to glide so that the depressurisation/venting sequences can be triggered
 * 			manually or automatically .
 */
void control_abort_run(void) {

	control_command_t expected_cmds[] = {
		COMMAND_RECOVER
	};
	control_command_t received_command;
	received_command = control_read_commands(expected_cmds, 1, NULL);
	if(received_command == COMMAND_RECOVER) {
		control_idle_start();
	}

	//do nothing
}
/* END */
