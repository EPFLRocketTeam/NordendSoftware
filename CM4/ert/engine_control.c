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

#include <propulsion/servo.h>
#include <driver/pwm.h>


#include <sensor/engine_pressure.h>
#include <sensor/temperature.h>

/**********************
 *	CONSTANTS
 **********************/

#define CONTROL_HEART_BEAT 200
#define CONTROL_TAKEOFF_THRESH 0 // TODO The tolerance for taking off, in milliseconds
#define FINAL_COUNTDOWN 10000 // TODO must be changed by wanted countdown duration, in milliseconds
#define IGNITION_COUNTER_THRESHOLD 10000 // TODO must be changed by wanted nb attempts
#define SHUTDOWN_TO_APOGEE_TIME 6000

#define CONTROL_ONE_SECOND pdMS_TO_TICKS(1000)

//TODO Find IGNITER GPIO port and pin
//Igniter pin definition
#define IGNITER_PORT GPIOG
#define IGNITER_PIN  1

#define IGNITER_ON_TIME 1000 // Time in ms to keep the igniter pin ON
#define IGNITER_OFF_TIME 3000 // Time in ms to wait after turning the igniter pin OFF before

#define N2O_SERVO_CLOSE_TIME 13000 // Time in ms
#define ETHANOL_SERVO_OPEN_TIME 30000 // Time in ms

#define DEPRESSURISATION_TIME 5000 // Time in ms

/**
 * @enum od_engine_state
 * @brief  Represents the possible values of an engine state OD entry
 */


/**********************
 *	MACROS
 **********************/

#define IDLE_UNTIL_COUNTER_ZERO ({control.last_time = control.time;\
	control.time = HAL_GetTick();\
	control.counter -= control.time - control.last_time;\
	if (control.counter > 0) return;})

/**********************
 *	TYPEDEFS
 **********************/

typedef struct control
{
	control_state_t state;

	solenoid_t solenoid_n2o;
	solenoid_t solenoid_ethanol;
	solenoid_t solenoid_press;
	solenoid_t solenoid_purge;

	device_t *i2c_engine_press;
	device_t *i2c_engine_temp;

	servo_t servo_ethanol;
	servo_t servo_n2o;

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
uint32_t min_pulse = 800;
uint32_t max_pulse = 2200;
float degrees_per_usec = 0.114;


/**********************
 *	PROTOTYPES
 **********************/
util_error_t init_engine_control();

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
	util_error_t init_err = init_engine_control();

	uint16_t checkpoint = led_add_checkpoint(led_blue);
	debug_log("Control start\n");

	// Timer things
	last_wake_time = xTaskGetTickCount();

	
	for (;;) {

		debug_log("Current state : %d\n", control.state);

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
	control.state = CONTROL_IDLE;

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


///////////////////////////////////////////////////////////////////////////////////////////////////
//CONTROL STATE CHANGING AND SCHEDULING
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
// MANUAL STATE RUNTIME FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

void control_idle_start(void) {
	control.state = CONTROL_IDLE;
}

void control_idle_run(void) {
	//check if we want to arm

	//check if we want to manually move


}



void control_calibration_start(void) {
	control.state = CONTROL_CALIBRATION;
}

void control_calibration_run(void) {
	//for now just go back to idle

	control_idle_start();
}


void control_pressured_start(void) {
	control.state = CONTROL_PRESSURED;

	solenoid_open(&control.solenoid_press);
}

void control_pressured_run(void) {
	//wait for ignition command


	if(0) { //ignition command
		control_igniter_start();
	}
}


void control_igniter_start(void) {
	//set counter to igniter time
	control.counter = IGNITER_ON_TIME + IGNITER_OFF_TIME;
	control.state = CONTROL_IGNITER;
}

/**
 * @brief Igniter state runtime
 * @details	Ignition will send the appropriate current for the igniter to turn on.
 * 			This state will wait for ignition to end and will jump to powered.
 */
void control_igniter_run(void) {
	control.last_time = control.time;

	if (control.counter >= IGNITER_OFF_TIME) {
		HAL_GPIO_WritePin(IGNITER_PORT, IGNITER_PIN, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(IGNITER_PORT, IGNITER_PIN, GPIO_PIN_RESET);
	}

	//TIMER WAIT

	control_ignition_start();
}

void control_ignition_start(void) {
	control.state = CONTROL_IGNITION;
	servo_set_state(&control.servo_n2o, SERVO_PARTIALLY_OPEN);
	servo_set_state(&control.servo_ethanol, SERVO_PARTIALLY_OPEN);
	control.counter = 100;
}


void control_ignition_run(void) {

	IDLE_UNTIL_COUNTER_ZERO;


	control_thrust_start();
}

void control_thrust_start(void) {
	control.state = CONTROL_THRUST;
	servo_set_state(&control.servo_n2o, SERVO_OPEN);
	servo_set_state(&control.servo_ethanol, SERVO_OPEN);
	control.counter = CONTROL_ONE_SECOND * 10; // 10 second wait before switching state
}

/**
 * @brief	Thrust state runtime

 * @details	This state will open the servos to their 'fully open' position.
 * 			After a delay, it will jump to the shutdown state.
 */
void control_thrust_run(void) {
	IDLE_UNTIL_COUNTER_ZERO;

	control_shutdown_start();
}

void control_shutdown_start(void) {
	control.state = CONTROL_SHUTDOWN;
	servo_set_state(&control.servo_ethanol, SERVO_CLOSED);
	control.counter = 100;
}

void control_shutdown_run(void) {
	util_error_t error_shutdown = ER_SUCCESS; // shutdown()

	//signal for recovery
	control_start_glide();
}

void control_glide_start(void) {
	control.state = CONTROL_GLIDE;
}

void control_glide_run(void) {

	//do nothing
}

void control_error_start(void) {
	control.state = CONTROL_ERROR;
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
}

void control_abort_start(void) {
	control.state = CONTROL_ABORT;
}

/**
 * @brief	Abort state runtime
 * @details	This state will go to glide so that the depressurisation/venting sequences can be triggered
 * 			manually or automatically .
 */
void control_abort_run(void) {

	//do nothing
}
/* END */
