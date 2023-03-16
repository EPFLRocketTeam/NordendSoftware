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
#include <solenoide.h>

#include <sensor/engine_pressure.h>
#include <sensor/temperature.h>

/**********************
 *	CONSTANTS
 **********************/

#define CONTROL_HEART_BEAT	200
#define FINAL_COUNTDOWN 10 //must be changed by wanted value, in seconds

#define CONTROL_ONE_SECOND pdMS_TO_TICKS(1000)

// TODO must define !
#define SOLENOID_N2O SOLENOID_1
#define SOLENOID_ETHANOL SOLENOID_2
#define SOLENOID_PRESSURISATION SOLENOID_3
#define SOLENOID_PURGE SOLENOID_4

/**
 * Origin offset (in microseconds), for computing the pulse width. Default is 1500.
 */
static const float SERVO_ETHANOL_OFFSET = 1500;

/**
 * Origin offset (in microseconds), for computing the pulse width. Default is 1500.
 */
static const float SERVO_N2O_OFFSET = 1500;


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
	/** Control venting 1 */
	CONTROL_VENT_N20 = 2,
	/** Control venting 2 */
	CONTROL_VENT_ETHANOL = 3,
	/** Control the opening of the purge */
	CONTROL_VENT_PURGE = 4,
	/** Control the N2O servo */
	CONTROL_N2O = 5,
	/** Control the ethanol servo */
	CONTROL_ETHANOL = 6,
	/** Powered ascent */
	CONTROL_PRESSURISATION = 7,
	/** Subsonic, coast flight */
	CONTROL_GLIDE = 8,
	/** Supersonic flight */
	CONTROL_COUNTDOWN = 9,
	/** Apogee reached, trigger first event */
	CONTROL_IGNITER = 10,
	/** Drogue chute descent, wait for second event */
	CONTROL_IGNITION = 11,
	/** Low alt reached, trigger second event */
	CONTROL_THRUST = 12,
	/** Main chute descent, wait for touchdown */
	CONTROL_SHUTDOWN = 13,
	/** Touchdown detected, end of the flight */ //-> should be end of propulsion only
	CONTROL_APOGEE = 14,
	/** Ballistic flight detected */
	CONTROL_DEPRESSURISATION = 15,
	/** Ground (automatic) error */
	CONTROL_ERROR = 16,
	/** Flight (radio-triggered) error */
	CONTROL_ABORT = 17
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

static device_t * i2c_engine_press;
static device_t * i2c_engine_temp;

/**
 * Instance of Ethanol servo.
 */
static servo_t ethanol_servo_inst;
static servo_t * servo_ethanol = &ethanol_servo_inst;

/**
 * Instance of N2O servo
 */
static servo_t servo_n2o_inst;
static servo_t * servo_n2o = &servo_n2o_inst;

/*
* Status of vent pins
*/
static uint8_t vent_n2o_pins = 0;
static uint8_t vent_ethanol_pins = 0;
static uint8_t vent_purge_pins = 0;
static uint8_t vent_pressurization_pins = 0;


/**********************
 *	PROTOTYPES
 **********************/

void schedule_next_state(control_state_t next_state);

void control_idle_run(void);
void control_calibration_run(void);
void control_vent_n2o_run(void);
void control_vent_ethanol_run(void);
void control_vent_purge_run(void);
void control_n2o_run(void);
void control_ethanol_run(void);
void control_pressurisation_run(void);
void control_glide_run(void);
void control_countdown_run(void);
void control_igniter_run(void);
void control_ignition_run(void);
void control_thrust_run(void);
void control_shutdown_run(void);
void control_apogee_run(void);
void control_depressurisation_run(void);
void control_error_run(void);
void control_abort_run(void);

void (*control_fcn[])(void) = {
	control_idle_run,
	control_calibration_run,
	control_vent_n2o_run,
	control_vent_ethanol_run,
	control_vent_purge_run,
	control_ethanol_run,
	control_pressurisation_run,
	control_glide_run,
	control_countdown_run,
	control_igniter_run,
	control_igniter_run,
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

void schedule_next_state(control_state_t next_state) {
	control.prev_state = control.state;
	control.state = next_state;
}

uint8_t error_can_be_recalibrated(){
	uint8_t boule = 0;
	switch (control.prev_state)
	{
		case CONTROL_VENT_ETHANOL : 
			boule=1;
			break;
		case CONTROL_VENT_N20: 
			boule=1;
			break;
		case CONTROL_VENT_PURGE: 
			boule=1;
			break;
		case CONTROL_PRESSURISATION : 
			boule=1;
			break;
		case CONTROL_COUNTDOWN : 
			boule=1;
			break;
		case CONTROL_IGNITION : 
			boule=1;
			break;
		default :
			break;
	}
	return boule;
}


/**
 * @fn void control_isr_thread(void*)
 * @brief Control ISR thread entry
 * @details Interrupt service routing thread that polls radio every second to check if a new command is sent.
 * 			If the command "makes sense" the main control thread is notified, and the state of the propulsion machine
 * 			is changed accordingly.
 *
 * @param arg FreeRTOS thread entry point context (unused) (prevents compiler warning)
 */
void enginge_control_isr_thread(__attribute__((unused)) void *arg) {
	for(;;) {
		// every XXX ticks, poll radio

		if (/* radio event received AND manual change is requested */) {
			// Notify engine control thread
			// TODO maybe we actually don't want to notify thread unless necessary ?
			// TODO need access to task pointer...
			// xTaskNotifyGive();
		} else {
			// Pause task for 1 ms
			vTaskDelay(CONTROL_ONE_SECOND);
		}
	}
}

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
	//Initialize the value of control
	control.state = CONTROL_IDLE;

	//Timer things
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(CONTROL_HEART_BEAT);
	last_wake_time = xTaskGetTickCount();

	uint16_t checkpoint = led_add_checkpoint(led_blue);
	debug_log("Control start\n");

	// Initialize servos – TODO perhaps relegate to another state/function

	pwm_data_t pwm_data_inst;
	pwm_data_t * pwm_data = &pwm_data_inst;

	// Specific to the SB2290SG Monster Torque Brushless Servo
	uint32_t min_pulse = 800;
	uint32_t max_pulse = 2200;
	float degrees_per_usec = 0.114;

	//Get sensor devices
	i2c_engine_press = i2c_sensor_get_ADC();
	i2c_engine_temp = i2c_sensor_get_ADC();

	//Initialize sensors
	util_error_t engine_press_err = engine_pressure_init(i2c_engine_press);
	util_error_t engine_temp_err = temperature_sensor_init(i2c_engine_temp);

	//Sensor initialisation checkpoints
	uint16_t checkpoint_engpress;
	if(engine_press_err == ER_SUCCESS) {
		checkpoint_engpress = led_add_checkpoint(led_green);
	} else {
		checkpoint_engpress = led_add_checkpoint(led_red);
	}

	uint16_t checkpoint_engtemp;
	if(engine_temp_err == ER_SUCCESS) {
		checkpoint_engtemp = led_add_checkpoint(led_green);
	} else {
		checkpoint_engtemp = led_add_checkpoint(led_red);
	}

	// Assign Ethanol servo to pin 13 (TIM4, CH2) and N2O servo to pin 14 (TIM4, CH3)
	servo_init(
			servo_ethanol,
			pwm_data,
			PWM_SELECT_CH2,
			min_pulse,
			max_pulse,
			SERVO_ETHANOL_OFFSET,
			degrees_per_usec,
			SERVO_ETHANOL_OPEN,
			SERVO_ETHANOL_IGNITION,
			SERVO_ETHANOL_CLOSED);

	servo_init(
			servo_n2o,
			pwm_data,
			PWM_SELECT_CH3,
			min_pulse,
			max_pulse,
			SERVO_N2O_OFFSET,
			degrees_per_usec,
			SERVO_N2O_OPEN,
			SERVO_N2O_IGNITION,
			SERVO_N2O_CLOSED);

	// Using channels 1 and 2 -- initialize the PWM channel
	pwm_init(pwm_data, PWM_TIM4, servo_ethanol->pwm_channel | servo_n2o->pwm_channel);


	for (;;) {
		//read battery TODO

		//how are radio instructions implemented ?
		

		//LED Debug checkpoints
		led_checkpoint(checkpoint);
		led_checkpoint(checkpoint_engpress);
		led_checkpoint(checkpoint_engtemp);
		//debug_log("Control loop | state: %d\n", control.state);

		// Call the function associated with the current state.
		control_fcn[control.state]();

		vTaskDelayUntil(&last_wake_time, period);
	}
}

/**
 * @fn void prev_state_start(void)
 * @brief Returns to the previous saved state.
 * @details Used for automatic switching back from certain states.
 */
void prev_state_start(void) {
	control_state_t new_prev = control.state;
	control.state = control.prev_state;
	control.prev_state = new_prev;
}

/**
 * @brief	Idle state runtime
 * @details The Idle state will simply wait for the calibration, venting, pressurisation and countdown
 * 			command/action to happen.
 */
void control_idle_run(void) {

	// TODO Check battery state, if charge disconnected scream!

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
	


	
	if (/* charge_disconnected */ ) {
		// Log("charge disconnected")
		schedule_next_state(CONTROL_ERROR);
	}

	// Does nothing, control_thread will loop until further instructions
}


/**
 * @brief	Calibration state runtime
 * @details This state will wait for the calibration sequence to finish and jump
 * 			back to Idle or go to Error.
 */
void control_calibration_run(void) {
	util_error_t error_calibration = 0;

	error_calibration |= engine_pressure_calibrate(i2c_engine_press);
	error_calibration |= engine_temperature_calibrate(i2c_engine_temp);

	//TODO: Is any type of calibration necessary for servos etc?

	if (error_calibration) {
		schedule_next_state(CONTROL_ERROR);
		return;
	}

	schedule_next_state(CONTROL_IDLE);
}


/**
 * @brief Venting state runtime
 * @details This state will wait for the venting sequence to finish and jump back to its previous state.
 * 			This function will open/close the venting valves.
 * 			---->Should we be making an vent-open and vent-close state?
 */
void control_vent_n2o_run(void) {
	uint8_t error_venting = 0;

	if (vent_n2o_pins) {
		solenoid_off(SOLENOID_N2O);
		solenoid_off(SOLENOID_N2O);
		vent_n2o_pins = 0;
	}
	else {
		solenoid_on(SOLENOID_N2O);
		solenoid_on(SOLENOID_N2O);
		vent_n2o_pins = 1;
	}

	if (error_venting) {
		schedule_next_state(CONTROL_ERROR);
		return;
	}

	// Return to previous state
	prev_state_start();
}

/**
 * @brief Venting state 2 runtime
 * @details This state will wait for the venting sequence to finish and jump back to its previous state.
 * 			This function will open/close the venting valves.
 * 			---->Should we be making an vent-open and vent-close state?
 */
void control_vent_ethanol_run(void) {

	if (vent_ethanol_pins) {
		solenoid_off(SOLENOID_ETHANOL);
		solenoid_off(SOLENOID_ETHANOL);
		vent_ethanol_pins = 0;
	}
	else {
		solenoid_on(SOLENOID_ETHANOL);
		solenoid_on(SOLENOID_ETHANOL);
		vent_ethanol_pins = 1;
	}


	prev_state_start();
}

/**
 * @fn void control_vent_purge_run(void)
 * @brief Purge state runtime
 * @details
 */
void control_vent_purge_run(void) {

	if (vent_purge_pins) {
		solenoid_off(SOLENOID_PURGE);//TODO FIND WHICH SOLENOIDS ARE WHICH
		solenoid_off(SOLENOID_PURGE);
		vent_purge_pins = 0;
	}
	else {
		solenoid_on(SOLENOID_PURGE);//TODO FIND WHICH SOLENOIDS ARE WHICH
		solenoid_on(SOLENOID_PURGE);
		vent_purge_pins = 1;
	}

	//TODO Return to proper state after runnning
	prev_state_start();

}


/**
 * @fn void control_n2o_run(void)
 * @brief N2O state runtime
 * @details
 */
void control_n2o_run(void) {
	// TODO don't we need an open, close, and partially open state ?
	// TODO review how to implement this, I'm not comfortable comparing floats

	switch (servo_get_state(servo_n2o)) {
		case SERVO_OPEN:
			servo_set_state(servo_n2o, SERVO_CLOSED);
			break;
		case SERVO_PARTIALLY_OPEN:
			servo_set_state(servo_n2o, SERVO_OPEN);
			break;
		case SERVO_CLOSED:
			servo_set_state(servo_n2o, SERVO_PARTIALLY_OPEN);
			break;
		default:
			servo_set_state(servo_n2o, SERVO_CLOSED);
			break;
	}

	// Go back to prev state
	prev_state_start();
}

/**
 * @fn void control_ethanol_run(void)
 * @brief Ethanol state runtime
 */
void control_ethanol_run(void) {

	switch (servo_get_state(servo_ethanol)) {
		case SERVO_OPEN:
			servo_set_state(servo_ethanol, SERVO_CLOSED);
			break;
		case SERVO_PARTIALLY_OPEN:
			servo_set_state(servo_ethanol, SERVO_OPEN);
			break;
		case SERVO_CLOSED:
			servo_set_state(servo_ethanol, SERVO_PARTIALLY_OPEN);
			break;
		default:
			servo_set_state(servo_ethanol, SERVO_CLOSED);
			break;
	}

	// Go back to prev state
	prev_state_start();
}


/**
 * @brief Pressurisation state runtime
 * @details This state will wait for the pressurisation sequence to finish and jump back to its previous state.
 * 			This state will open/close the N20 pressurisation valve.
 */
void control_pressurisation_run(void) {

	if (vent_pressurization_pins) {
		solenoid_off(SOLENOID_PRESSURISATION);//TODO FIND WHICH SOLENOIDS ARE WHICH
		solenoid_off(SOLENOID_PRESSURISATION);
		vent_pressurization_pins = 0;
	}
	else {
		solenoid_on(SOLENOID_PRESSURISATION);//TODO FIND WHICH SOLENOIDS ARE WHICH
		solenoid_on(SOLENOID_PRESSURISATION);
		vent_pressurization_pins = 1;
	}

	//TODO Return to proper state after runnning
	prev_state_start();

}

/**
 * @brief	Glide state runtime
 * @details The glide state will simply wait for the depressurisation action to start and finish
 *			It will also wait until touchdown then return to idle.
 */
void control_glide_run(void) {
	//TODO check for depressurisation
	if (/*depressurisation_needed()*/) {
		schedule_next_state(CONTROL_DEPRESSURISATION);
		return;
	}
	//TODO check for landing
	if (/* landing() */) {	//-> landing() gives a 1 once it has landed, else 0
		schedule_next_state(CONTROL_IDLE);
		return;
	};

}

/**
 * @brief	countdown state runtime
 * @details	This state will wait for the countdown to end and will jump to ignition.
 */
void control_countdown_run(void) {
	uint8_t countdown = FINAL_COUNTDOWN;

	do {
		//delay
		vTaskDelay(CONTROL_ONE_SECOND); // wait 1 second

		// TODO leave error handling to the main loop
//		if (/*error_detected*/) {
//			schedule_next_state(CONTROL_ERROR);
//			return;
//		} else if (/*abort needed*/) {
//			schedule_next_state(CONTROL_ABORT);
//			return;
//		}

		countdown--;
	} while (countdown);

	schedule_next_state(CONTROL_IGNITER);
}


/**
 * @brief Igniter state runtime
 * @details	Ignition will send the appropriate current for the igniter to turn on.
 * 			This state will wait for ignition to end and will jump to powered.
 */
void control_igniter_run(void) {
	uint8_t error_ignition = 0; //ignition() -> will turn the igniter on (solenoids)

	// Activate ignition
	// TODO define behavior !

	//Check if good engine start (pressure and temp?), if too many failed abort

	if (/*error_ignition == IGNITION_ABORT*/) {
		schedule_next_state(CONTROL_ABORT);
		return;
	} else if (/*error_ignition == IGNITION_ERROR*/) {
		//turn off solenoids()

		schedule_next_state(CONTROL_ERROR);
		return;
	}

	schedule_next_state(CONTROL_IGNITION);
}

/**
 * @brief	Ignition state runtime
 * @details	This state will open the servos to their 'partially open' state.
 * 			After a delay, it will jump to the thrust state.
 */
void control_ignition_run(void) {
	util_error_t error_ignition = ER_SUCCESS; //powering() -> partial open state of servos

	vTaskDelay(pdMS_TO_TICKS(100)); // pause for 100 ms

	//Set ethanol and N2O servos (pins 13 and 14) to partially open
	error_ignition |= servo_set_state(servo_n2o, SERVO_PARTIALLY_OPEN);
	error_ignition |= servo_set_state(servo_ethanol, SERVO_PARTIALLY_OPEN);

	if (error_ignition)
		schedule_next_state(CONTROL_ABORT);
	else
		schedule_next_state(CONTROL_THRUST);
}

/**
 * @brief	Thrust state runtime
 * @details	This state will open the servos to their 'fully open' position.
 * 			After a delay, it will jump to the shutdown state.
 */
void control_thrust_run(void) {
	util_error_t error_thrust = ER_SUCCESS; //thrust() -> full open state of servos


	error_thurst |= servo_set_state(servo_n2o, SERVO_OPEN);
	error_thrust |= servo_set_rotation(servo_ethanol, SERVO_OPEN);

	vTaskDelay(30 * CONTROL_ONE_SECOND); // pause for 30 seconds

	if (error_thrust)
		schedule_next_state(CONTROL_ABORT);
	else
		schedule_next_state(CONTROL_SHUTDOWN);
}

/**
 * @brief	Shutdown state runtime
 * @details	This function will stop the engine, depending on which algorithm is chosen (before or during the apogee, tbd).
 * 			After a delay, it will jump to the apogee state.
 * 			Since we do not know if the engine has enough power to reach apogee without a full combustion, shutdown() is tbd
 */
void control_shutdown_run(void) {
	util_error_t error_shutdown = ER_SUCCESS; //shutdown()

	// TODO define engine shutdown behavior

	if (error_shutdown)
		schedule_next_state(CONTROL_ABORT);
	else
		schedule_next_state(CONTROL_APOGEE);
}

/**
 * @brief	Apogee state runtime
 * @details	This function will open the venting valves (N20 and ethanol).
 * 			After the end of the sequence, it will jump to the depressurisation state.
 */
void control_apogee_run(void) {
	util_error_t error_start_fall = ER_SUCCESS;

	error_start_fall |= solenoid_on(SOLENOID_N2O);     //TODO Check if this is correct, pretty sure solenoid returns nothing and we do not need the error var
	error_start_fall |= solenoid_on(SOLENOID_ETHANOL);

	if (error_start_fall)
		schedule_next_state(CONTROL_ABORT);
	else
		schedule_next_state(CONTROL_DEPRESSURISATION);
}

/**
 * @brief	Depressurisation state runtime
 * @details	This function will open the pressurisation valve (N20).
 * 			After the end of the sequence, it will jump to the glide state.
 */
void control_depressurisation_run(void) {
	uint8_t error_depressurisation = ER_SUCCESS; //depressurisation() -> open valve

	error_depressurisation != solenoid_on(SOLENOID_PRESSURISATION);

	if (error_depressurisation) {
		schedule_next_state(CONTROL_ABORT);
		return;
	}
	control_glide_start();
}

/**
 * @brief	Error state runtime
 * @details	The error state will try to fix the issue and return to the previous state
 * 			in the case of calibration or idle if else.
 * 			It will send the maximum information to the GS, and have different sequences
 * 			depending on the error to try to fix the problem.
 */
void control_error_run(void) {
	//TODO memorize_what_went_wrong_for_next_time()
	if (error_can_be_recalibrated()) {
		schedule_next_state(CONTROL_CALIBRATION);
	}
	control_idle_start();
}

/**
 * @brief	Abort state runtime
 * @details	This state will go to glide so that the depressurisation/venting sequences can be triggered
 * 			manually or automatically .
 */
void control_abort_run(void) {
	//TODO 
	schedule_next_state = CONTROL_GLIDE;
}

/* END */
