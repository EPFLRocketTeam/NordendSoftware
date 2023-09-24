/**
 * @file 		recovery_control.c
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
#include <recovery_control.h>

#include <propulsion/servo.h>
#include <driver/pwm.h>


#include <sensor/engine_pressure.h>
#include <sensor/temperature.h>

/**********************
 *	CONSTANTS
 **********************/


#define COMMAND_QUEUE_LENGTH 16

#define RECOVERY_HEART_BEAT 50



/**********************
 *	MACROS
 **********************/

#define RETURN_UNTIL_COUNTER_ZERO ({control.last_time = control.time;\
	control.time = HAL_GetTick();\
	control.counter -= control.time - control.last_time;\
	if (control.counter > 0) return;})

/**********************
 *	TYPEDEFS
 **********************/


typedef struct recovery_control
{
	recovery_state_t state;

	recovery_control_data_t rc_data;

	StaticQueue_t command_queue_params;
	QueueHandle_t command_queue;
	uint8_t command_queue_storage[sizeof(recovery_cmd_t)*COMMAND_QUEUE_LENGTH];

	int32_t counter;
	uint32_t time;
	uint32_t last_time;
} recovery_control_t;

/**********************
 *	VARIABLES
 **********************/

/**
 * Main control structure, with all items
 */
static recovery_control_t control;

/**
 * Last wake time for the timer
 */
static TickType_t last_wake_time;
static const TickType_t period = pdMS_TO_TICKS(RECOVERY_HEART_BEAT);



/**********************
 *	PROTOTYPES
 **********************/

void recovery_idle_start(void);
void recovery_calibration_start(void);
void recovery_armed_start(void);
void recovery_powered_start(void);
void recovery_glide_start(void);
void recovery_first_event_start(void);
void recovery_drogue_start(void);
void recovery_second_event_start(void);
void recovery_main_start(void);
void recovery_touchdown_start(void);
void recovery_error_start(void);
void recovery_abort_start(void);

void recovery_idle_run(void);
void recovery_calibration_run(void);
void recovery_armed_run(void);
void recovery_powered_run(void);
void recovery_glide_run(void);
void recovery_first_event_run(void);
void recovery_drogue_run(void);
void recovery_second_event_run(void);
void recovery_main_run(void);
void recovery_touchdown_run(void);
void recovery_error_run(void);
void recovery_abort_run(void);



util_error_t init_recovery_control();

int recovery_control_command_pop(recovery_cmd_t * cmd);

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

void recovery_control_thread(__attribute__((unused)) void *arg) {
	init_recovery_control();

	uint16_t checkpoint = led_add_checkpoint(led_blue);
	debug_log(LOG_INFO, "Recovery start\n");

	// Timer things
	last_wake_time = xTaskGetTickCount();

	
	for (;;) {
		led_checkpoint(checkpoint);
		debug_log(LOG_INFO, "Current state : %d\n", control.state);

		// Call the function associated with the current state.
		switch (control.state) {
		case RECOVERY_IDLE:
			recovery_idle_run();
			break;
		case RECOVERY_CALIBRATION:
			recovery_calibration_run();
			break;
		case RECOVERY_ARMED:
			recovery_armed_run();
			break;
		case RECOVERY_POWERED:
			recovery_powered_run();
			break;
		case RECOVERY_GLIDE:
			recovery_glide_run();
			break;
		case RECOVERY_FIRST_EVENT:
			recovery_first_event_run();
			break;
		case RECOVERY_DROGUE:
			recovery_drogue_run();
			break;
		case RECOVERY_SECOND_EVENT:
			recovery_second_event_run();
			break;
		case RECOVERY_MAIN:
			recovery_main_run();
			break;
		case RECOVERY_TOUCHDOWN:
			recovery_touchdown_run();
			break;
		case RECOVERY_ERROR:
			recovery_error_run();
			break;
		case RECOVERY_ABORT:
			recovery_abort_run();
			break;
		default:
			// Undefined behavior / ghost state -> abort
			recovery_error_start();
			break;
		}

		control.rc_data.state = control.state;
		control.rc_data.time = HAL_GetTick();

		od_write_RECOVERY_CONTROL_DATA(&control.rc_data);

		vTaskDelayUntil(&last_wake_time, period);
		
	}
}


/**
 * @fn util_error_t init(void)
 * @brief Initializes all peripherals and sets up the control object.
 *
 * @return ER_SUCCESS if everything went well, a non-zero error code otherwise.
 */

util_error_t init_recovery_control(void) {
	// Initialize the value of control
	control.state = RECOVERY_IDLE;

	//init command queue
	control.command_queue = xQueueCreateStatic( 
								COMMAND_QUEUE_LENGTH,
                                sizeof(recovery_cmd_t),
                                control.command_queue_storage,
                                &control.command_queue_params);




	return ER_SUCCESS;
}


void recovery_control_command_push(recovery_cmd_t cmd) {
	recovery_cmd_t command = cmd;

	//post to the queue without timeout
	xQueueSend(control.command_queue, (void*) &command, 0);

	//we dont care if posted successfully or not for now...
}

int recovery_control_command_pop(recovery_cmd_t * cmd) {
	recovery_cmd_t command;
	//receive from queue without timeout
	if(xQueueReceive(control.command_queue, &command, 0) == pdPASS) {
		*cmd = command;
		//return 1 if a command has been received
		return 1;
	} else {
		//return 0 if no commands have been received
		*cmd = RECOVERY_CMD_NONE;
		return 0;
	}
}


recovery_cmd_t recovery_read_commands(recovery_cmd_t * expected_cmd, size_t expected_cmd_len) {
	recovery_cmd_t cmd;
	while(recovery_control_command_pop(&cmd) != 0) {
		for(size_t i = 0; i < expected_cmd_len; i++) {
			if(cmd == expected_cmd[i]) {
				return cmd;
			} else if (cmd == RECOVERY_CMD_ABORT) {
				recovery_abort_start();
				return RECOVERY_CMD_NONE;
			}
		}
	}
	return RECOVERY_CMD_NONE;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//ACTUATORS MANUAL OPERATION
///////////////////////////////////////////////////////////////////////////////////////////////////





///////////////////////////////////////////////////////////////////////////////////////////////////
// MANUAL STATE RUNTIME FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

void recovery_idle_start(void) {
	control.state = RECOVERY_IDLE;
}

void recovery_idle_run(void) {

	recovery_cmd_t expected_cmds[] = {
		RECOVERY_CMD_ARM
	};

	recovery_cmd_t received_command;

	received_command = recovery_read_commands(expected_cmds, 1);

	if(received_command == RECOVERY_CMD_ARM) {
		recovery_armed_start();
	}
}

void recovery_calibration_start(void) {
	control.state = RECOVERY_CALIBRATION;

}

void recovery_calibration_run(void) {

}

void recovery_armed_start(void) {
	control.state = RECOVERY_ARMED;

}

void recovery_armed_run(void) {
	recovery_cmd_t expected_cmds[] = {
		RECOVERY_CMD_LIFTOFF,
		RECOVERY_CMD_DISARM,
	};

	recovery_cmd_t received_command;

	received_command = recovery_read_commands(expected_cmds, 2);

	if(received_command == RECOVERY_CMD_DISARM) {
		recovery_idle_start();
	} else if(received_command == RECOVERY_CMD_LIFTOFF) {
		recovery_powered_start();
	}
}

void recovery_powered_start(void) {
	control.state = RECOVERY_POWERED;

}

void recovery_powered_run(void) {
	recovery_cmd_t expected_cmds[] = {
		RECOVERY_CMD_MECO,
	};

	recovery_cmd_t received_command;

	received_command = recovery_read_commands(expected_cmds, 1);

	if(received_command == RECOVERY_CMD_MECO) {
		recovery_glide_start();
	}
}

void recovery_glide_start(void) {
	control.state = RECOVERY_GLIDE;

}

void recovery_glide_run(void) {

}

void recovery_first_event_start(void) {
	control.state = RECOVERY_FIRST_EVENT;

}

void recovery_first_event_run(void) {

}

void recovery_drogue_start(void) {
	control.state = RECOVERY_DROGUE;

}

void recovery_drogue_run(void) {

}

void recovery_second_event_start(void) {
	control.state = RECOVERY_SECOND_EVENT;

}

void recovery_second_event_run(void) {

}

void recovery_main_start(void){
	control.state = RECOVERY_MAIN;

}

void recovery_main_run(void){

}

void recovery_touchdown_start(void) {
	control.state = RECOVERY_TOUCHDOWN;

}

void recovery_touchdown_run(void) {

}

void recovery_error_start(void) {
	control.state = RECOVERY_ERROR;

}

void recovery_error_run(void) {

}

void recovery_abort_start(void) {
	control.state = RECOVERY_ABORT;

}

void recovery_abort_run(void) {

}




/* END */
