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

#define CONTROL_HEART_BEAT 200
#define CONTROL_TAKEOFF_THRESH 0 // TODO The tolerance for taking off, in milliseconds
#define FINAL_COUNTDOWN 10000 // TODO must be changed by wanted countdown duration, in milliseconds
#define IGNITION_COUNTER_THRESHOLD 10000 // TODO must be changed by wanted nb attempts
#define SHUTDOWN_TO_APOGEE_TIME 6000

#define CONTROL_ONE_SECOND pdMS_TO_TICKS(1000)

#define SOLENOID_N2O SOLENOID_1
#define SOLENOID_ETHANOL SOLENOID_2
#define SOLENOID_PRESSURISATION SOLENOID_3
#define SOLENOID_PURGE SOLENOID_4

//TODO Find IGNITER GPIO port and pin
//Igniter pin definition
#define IGNITER_PORT GPIOX
#define IGNITER_PIN  1

/**
 * Origin offset (in microseconds), for computing the pulse width. Default is 1500.
 */
#define SERVO_ETHANOL_OFFSET 1500;

/**
 * Origin offset (in microseconds), for computing the pulse width. Default is 1500.
 */
#define SERVO_N2O_OFFSET = 1500;

/**
 * @enum od_engine_state
 * @brief  Represents the possible values of an engine state OD entry
 */
enum od_engine_state {
	ENGINE_STATE_CLOSED = 0b0000,        /**< ENGINE_STATE_CLOSED */
	ENGINE_STATE_PARTIALLY_OPEN = 0b1010,/**< ENGINE_STATE_PARTIALLY_OPEN */
	ENGINE_STATE_OPEN = 0b1111           /**< ENGINE_STATE_OPEN */
};

// Scheduler related

/**********************
 *	MACROS
 **********************/

#define IDLE_UNTIL_COUNTER_ZERO ({control->last_time = control->time;\
	control->time = HAL_GetTick();\
	control->counter -= countrol->time - control-last_time;\
	if (control->counter > 0) return;})

/**********************
 *	TYPEDEFS
 **********************/

/**********************
 *	VARIABLES
 **********************/

/**
 * Main control structure, with all items
 */
static control_t control;
static uint8_t error_loop_control = 0;
static uint8_t ignition_insufficient_pressure_counter =0;

/**
 * Last wake time for the timer
 */
static TickType_t last_wake_time;
static const TickType_t period = pdMS_TO_TICKS(CONTROL_HEART_BEAT);

/*
 * For Status of vent pins -- use OD
 */
/**********************
 *	PROTOTYPES
 **********************/

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
	util_error_t init_err = init();

	// Timer things
	last_wake_time = xTaskGetTickCount();

	if (init_err) {
		// lol = !lol (PropulsionControl/engine.c:41)
	}

	int adc_resolution = 16;
	float ref_voltage = 3.3;

	// Determine the offset value for each ADC channel
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	uint32_t offset_bat1 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	uint32_t offset_bat2 = HAL_ADC_GetValue(&hadc1);

	// Calculate the fixed offset value for each ADC channel
	float expected_voltage = 0.0; // expected voltage when input voltage is zero
	float measured_voltage_bat1 = (float)offset_bat1 / ((1 << adc_resolution) - 1) * ref_voltage * 1000;
	float measured_voltage_bat2 = (float)offset_bat2 / ((1 << adc_resolution) - 1) * ref_voltage * 1000;
	float fixed_offset_bat1 = expected_voltage - measured_voltage_bat1;
	float fixed_offset_bat2 = expected_voltage - measured_voltage_bat2;

	for (;;) {
	  // Start the ADC conversion sequence
	  HAL_ADC_Start(&hadc1);

	  // Poll for the first conversion completion (Battery 1)
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  uint32_t bat1_value = HAL_ADC_GetValue(&hadc1) + fixed_offset_bat1;
	  uint32_t bat1_voltage = (float)bat1_value / ((1 << adc_resolution) - 1) * ref_voltage * 1000;

	  // Poll for the second conversion completion (Battery 2)
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  uint32_t bat2_value = HAL_ADC_GetValue(&hadc1) + fixed_offset_bat2;
	  uint32_t bat2_voltage = (float)bat2_value / ((1 << adc_resolution) - 1) * ref_voltage * 1000;

	  od_write_BATTERY_A(&bat1_voltage);
	  od_write_BATTERY_B(&bat2_voltage);

		// LED Debug checkpoints
		led_checkpoint(checkpoint);
		led_checkpoint(checkpoint_engpress);
		led_checkpoint(checkpoint_engtemp);
		// debug_log("Control loop | state: %d\n", control.state);

		//Do task scheduler things
		control_sched_t flagged_state = CONTROL_SCHED_NOTHING;
		flagged_state = check_flags_state();

		if (flagged_state != CONTROL_SCHED_NOTHING) {
			reset_flag(flagged_state);
			control_state_t requested_state = correlate_state_sched(flagged_state);
			control_sched_check_next(requested_state);
		}

		// Call the function associated with the current state.
		switch (control.state) {
		case CONTROL_IDLE:
			control_idle_run();
			break;
		case CONTROL_CALIBRATION:
			control_calibration_run();
			break;
		case CONTROL_VENT_ETHANOL:
			control_vent_ethanol_run();
			break;
		case CONTROL_VENT_N2O:
			control_vent_n2o_run();
			break;
		case CONTROL_SERVOS_N2O:
			control_servos_n2o_run();
			break;
		case CONTROL_SERVOS_ETHANOL:
			control_servos_ethanol_run();
			break;
		case CONTROL_PRESSURISATION:
			control_pressurisation_run();
			break;
		case CONTROL_GLIDE:
			control_glide_run();
			break;
		case CONTROL_COUNTDOWN:
			control_countdown_run();
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
		case CONTROL_APOGEE:
			control_apogee_run();
			break;
		case CONTROL_DEPRESSURISATION:
			control_depressurisation_run();
			break;
		case CONTROL_ERROR:
			control_error_run();
			break;
		case CONTROL_ABORT:
			control_abort_run();
			break;
		default:
			// Undefined behavior / ghost state -> abort
			control_abort_run();
			break;
		}

		vTaskDelayUntil(&last_wake_time, period);
		
	}
}

control_sched_t check_flagged_state() {
	rf_cmd_t changes_sched = {0};
	od_read_RF_CMD(&changes_sched);

	uint16_t countdown_requested = 0;
	od_read_COUNTDOWN(&countdown_requested);

	// Check scheduled states in the same order as in the control_sched enum
	if (changes_sched.abort == CMD_ACTIVE) return CONTROL_SCHED_ABORT;
	if (changes_sched.ventEthanol == CMD_ACTIVE) return CONTROL_SCHED_VENT_ETHANOL;
	if (changes_sched.ventN20 == CMD_ACTIVE) return CONTROL_SCHED_VENT_N2O;
	if (changes_sched.calibrate == CMD_ACTIVE) return CONTROL_SCHED_CALIBRATE;
	if (changes_sched.servoEthanol == CMD_ACTIVE) return CONTROL_SCHED_SERVOS_ETHANOL;
	if (changes_sched.servoN20 == CMD_ACTIVE) return CONTROL_SCHED_SERVOS_N2O;
	if (countdown_requested == IGNITION_CODE) return CONTROL_SCHED_COUNTDOWN;
	if (changes_sched.pressurization) return CONTROL_SCHED_PRESSURISATION;
	if (changes_sched.shutdown) return CONTROL_SCHED_SHUTDOWN;

	// Default
	return CONTROL_SCHED_NOTHING;
}

void reset_flag(control_sched_t flagged_state) {
	rf_cmd_t flags;
	od_read_RF_CMD(&flags);

	switch (flagged_state) {
		case CONTROL_SCHED_ABORT:
			flags.abort = CMD_INACTIVE;
			break;
		case CONTROL_SCHED_VENT_ETHANOL:
			flags.ventEthanol = CMD_INACTIVE;
			break;
		case CONTROL_SCHED_VENT_N2O:
			flags.ventN20 = CMD_INACTIVE;
		case CONTROL_SCHED_CALIBRATE:
			flags.calibrate = CMD_INACTIVE;
			break;
		case CONTROL_SCHED_SERVOS:
			break;
		case CONTROL_SCHED_COUNTDOWN:
			break;
		case CONTROL_SCHED_PRESSURISATION:
			break;
		default: // do nothing
	}
	od_write_RF_CMD(&flags);
}

/**
 * @fn util_error_t init(void)
 * @brief Initializes all peripherals and sets up the control object.
 *
 * @return ER_SUCCESS if everything went well, a non-zero error code otherwise.
 */

util_error_t init(void) {
	// Initialize the value of control
	control.state = CONTROL_IDLE;

	led_feedback_init();

	uint16_t checkpoint = led_add_checkpoint(led_blue);
	debug_log("Control start\n");

	// Get sensor devices
	i2c_engine_press = i2c_sensor_get_ADC();
	i2c_engine_temp = i2c_sensor_get_ADC();

	// Initialize sensors
	util_error_t engine_press_err = engine_pressure_init(i2c_engine_press);
	util_error_t engine_temp_err = temperature_sensor_init(i2c_engine_temp);

	// Sensor initialisation checkpoints
	uint16_t checkpoint_engpress;
	if (engine_press_err == ER_SUCCESS) {
		checkpoint_engpress = led_add_checkpoint(led_green);
	} else {
		checkpoint_engpress = led_add_checkpoint(led_red);
	}

	uint16_t checkpoint_engtemp;
	if (engine_temp_err == ER_SUCCESS) {
		checkpoint_engtemp = led_add_checkpoint(led_green);
	} else {
		checkpoint_engtemp = led_add_checkpoint(led_red);
	}

	// Initialize servos
	pwm_data_t pwm_data;

	// Specific to the SB2290SG Monster Torque Brushless Servo
	uint32_t min_pulse = 800;
	uint32_t max_pulse = 2200;
	float degrees_per_usec = 0.114;

	servo_t servo_ethanol;
	servo_t servo_n2o;

	// Assign Ethanol servo to pin 13 (TIM4, CH2) and N2O servo to pin 14 (TIM4, CH3)
	util_error_t ethanol_err = servo_init(
			&servo_ethanol,
			&pwm_data,
			PWM_SELECT_CH2,
			min_pulse,
			max_pulse,
			SERVO_ETHANOL_OFFSET,
			degrees_per_usec,
			SERVO_ETHANOL_OPEN,
			SERVO_ETHANOL_IGNITION,
			SERVO_ETHANOL_CLOSED);

	util_error_t n2o_err = servo_init(
			&servo_n2o,
			&pwm_data,
			PWM_SELECT_CH3,
			min_pulse,
			max_pulse,
			SERVO_N2O_OFFSET,
			degrees_per_usec,
			SERVO_N2O_OPEN,
			SERVO_N2O_IGNITION,
			SERVO_N2O_CLOSED);

	// Using channels 1 and 2 -- initialize the PWM channel
	pwm_init(pwm_data, PWM_TIM4,
			servo_ethanol->pwm_channel | servo_n2o->pwm_channel);

	// Initialize servo states
	util_error_t servo_err = ER_SUCCESS;
	servo_err |= servo_set_state(servo_n2o, SERVO_CLOSED);
	servo_err |= servo_set_state(servo_ethanol, SERVO_CLOSED);

	// Initialize solenoids
	util_error_t sol_err = ER_SUCCESS;
	sol_err |= solenoid_off(SOLENOID_N2O);
	sol_err |= solenoid_off(SOLENOID_ETHANOL);
	sol_err |= solenoid_off(SOLENOID_PRESSURISATION);

	// Initialize engine state for OD – everything 0 (closed)
	rf_cmd_t engine_state_init = {0};
	od_write_ENGINE_STATE(&engine_state_t);

	return led_err | servo_err | sol_err;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//CONTROL STATE CHANGING AND SCHEDULING
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @fn control_state_t correlate_state_sched(control_sched_t)
 * @brief Correlates scheduling states to FSM states
 * @details Used to convert scheduler states to FSM states
 */
control_state_t correlate_state_sched(control_sched_t requested_state) {
	switch (requested_state) {
	case CONTROL_SCHED_ABORT:
		return CONTROL_ABORT;
		break;
	case CONTROL_SCHED_VENT_N2O:
		return CONTROL_VENT_N2O;
		break;
	case CONTROL_SCHED_VENT_ETHANOL:
		return CONTROL_VENT_ETHANOL;
		break;
	case CONTROL_SCHED_CALIBRATE:
		return CONTROL_CALIBRATION;
		break;
	case CONTROL_SCHED_SERVOS_N2O:
		return CONTROL_SERVOS_N2O;
		break;
	case CONTROL_SCHED_SERVOS_ETHANOL:
		return CONTROL_SERVOS_ETHANOL;
		break;
	case CONTROL_SCHED_COUNTDOWN:
		return CONTROL_COUNTDOWN;
		break;
	case CONTROL_SCHED_SHUTDOWN:
		return CONTROL_SHUTDOWN;
		break;
	case CONTROL_SCHED_PRESSURISATION:
		return CONTROL_PRESSURISATION;
		break;
	default:
		return CONTROL_IDLE;
		break;
	}
}

/**
 * @fn void control_sched_check_next(control_state_t)
 * @brief Checks if requested state is valid
 * @details Used to check if the next state requested by GS is valid.
 */
static void control_sched_check_next(control_state_t *requested_state) {
	if (control.state != requested_state) {
		for (uint8_t i = 0; i < SCHED_ALLOWED_WIDTH; i++) {
			if (sched_allowed[control.state][i] == requested_state) {
				schedule_next_state(requested_state);
				return;
			}
		}
	}
}

/**
 * @fn void schedule_next_state(control_state_t)
 * @brief Sets the state for the next execution
 * @details Used to switch to the next desired state. <=====3
 */
void schedule_next_state(control_state_t next_state) {
	control.prev_state = control.state;
	switch (next_state)
	{
	case CONTROL_IDLE:
		control_idle_start();
		break;
	case CONTROL_CALIBRATION:
		control_calibration_start();
		break;
	case CONTROL_VENT_N2O:
		control_vent_n2o_start();
		break;
	case CONTROL_VENT_ETHANOL:
		control_vent_ethanol_start();
		break;
	case CONTROL_SERVOS_N2O:
		control_servos_n2o_start();
		break;
	case CONTROL_SERVOS_ETHANOL:
		control_servos_ethanol_start();
		break;
	case CONTROL_PRESSURISATION:
		control_pressurisation_start();
		break;
	case CONTROL_GLIDE:
		control_glide_start();
		break;
	case CONTROL_COUNTDOWN:
		control_countdown_start();
		break;
	case CONTROL_IGNITER:
		control_igniter_start();
		break;
	case CONTROL_IGNITION:
		control_ignition_start();
		break;
	case CONTROL_THRUST:
		control_thrust_start();
		break;
	case CONTROL_SHUTDOWN:
		control_shutdown_start();
		break;
	case CONTROL_APOGEE:
		control_apogee_start();
		break;
	case CONTROL_DEPRESSURISATION:
		control_depressurisation_start();
		break;
	case CONTROL_ERROR:
		control_error_start();
		break;
	case CONTROL_ABORT:
		control_abort_start();
		break;
	default:
		break;
	}
}

/**
 * @fn void prev_state_start(void)
 * @brief Returns to the previous saved state.
 * @details Used for automatic switching back from certain states.
 */
void prev_state_start(void) {
	schedule_next_state(control.prev_state);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// MANUAL STATE RUNTIME FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

void control_calibration_start(void) {
	control.state = CONTROL_CALIBRATION;
}

/**
 * @brief	Calibration state runtime
 * @details This state will wait for the calibration sequence to finish and jump
 * 			back to Idle or go to Error.
 */
void control_calibration_run(void) {
	util_error_t error_calibration = 0;

	error_calibration |= engine_pressure_calibrate(control->i2c_engine_press);
	error_calibration |= engine_temperature_calibrate(control->i2c_engine_temp);

	if (error_calibration) {
		schedule_next_state(CONTROL_ERROR);
		return;
	}
	error_loop_control = 0;
	schedule_next_state(CONTROL_IDLE);
}

void control_vent_ethanol_start(void) {
	control.state = CONTROL_VENTS;
}

void control_vent_n2o_start(void) {
	control.state = CONTROL_VENTS;
}


/**
 * @brief Venting state runtime
 * @details This state will wait for the venting sequence to finish and jump back to its previous state.
 * 			This function will open/close the venting valves.
 * 			---->Should we be making an vent-open and vent-close state?
 */
void control_vent_ethanol_run(void) {
	util_error_t error_venting = ER_SUCCESS;

	// Read ethanol state from OD (closed == 0) and update accordingly
	rf_cmd_t state;
	od_read_ENGINE_STATE(&state);
	state.ventEthanol != state.ventEthanol;
	
	if (state.ventEthanol) {
		error_venting |= solenoid_on(SOLENOID_ETHANOL);
	} else {
		error_venting |= solenoid_off(SOLENOID_ETHANOL);
	}

	if (error_venting) {
		schedule_next_state(CONTROL_ERROR);
	} else {
		prev_state_start();
	}

}

/**
 * @brief Venting state runtime
 * @details This state will wait for the venting sequence to finish and jump back to its previous state.
 * 			This function will open/close the venting valves.
 * 			---->Should we be making an vent-open and vent-close state?
 */
void control_vent_n2o_run(void) {
	util_error_t error_venting = ER_SUCCESS;

	// Read n2o state from OD (closed == 0) and update accordingly
	rf_cmd_t state;
	od_read_ENGINE_STATE(&state);
	state.ventN20 != state.ventN20;
	
	if (state.ventN20) {
		error_venting |= solenoid_on(SOLENOID_N2O);
	} else {
		error_venting |= solenoid_off(SOLENOID_N2O);
	}

	if (error_venting) {
		schedule_next_state(CONTROL_ERROR);
	} else {
		prev_state_start();
	}

}

void control_servos_n2o_start(void) {
	control.state = CONTROL_SERVOS_N2O;
}


void control_servos_ethanol_start(void) {
	control.state = CONTROL_SERVOS_ETHANOL;
}

static uint8_t servo_n2o_state_od(rf_cmd_t *state) {
	od_read_ENGINE_STATE(&state);
	return state->servoN20;
}

static uint8_t servo_ethanol_state_od(rf_cmd_t *state) {
	od_read_ENGINE_STATE(&state);
	return state->servoEthanol;
}

/**
 * @fn void control_n2o_run(void)
 * @brief N2O state runtime
 * @details
 */
//TODO Turn on enable_servo_pin
void control_servos_n2o_run(void) {
	util_error_t servo_err = ER_SUCCESS;
	rf_cmd_t eng_state;

	// Check if changes were requested
	rf_cmd_t requested_changes;
	od_read_RF_CMD(&requested_changes);

	// If change requested, update ethanol servo
	if (requested_changes->servoEthanol)
		switch (servo_ethanol_state_od(&eng_state)) {
			case ENGINE_STATE_CLOSED:
				servo_err |= servo_set_state(servo_ethanol, SERVO_OPEN);
				eng_state->servoEthanol = ENGINE_STATE_OPEN;
				break;
			case ENGINE_STATE_OPEN:
				servo_err |= servo_set_state(servo_ethanol, SERVO_CLOSED);
				eng_state->servoEthanol = ENGINE_STATE_CLOSED;
				break;
			default:
				servo_err |= servo_set_state(servo_ethanol, SERVO_CLOSED);
				break;
		}

	if (!servo_err) od_write_ENGINE_STATE(&state);
	servo_err = ER_SUCCESS;

	// If change requested, update n2o servo
	if (requested_changes->servoN20)
		switch (servo_n2o_state_od(&eng_state)) {
			case ENGINE_STATE_CLOSED:
				servo_err |= servo_set_state(servo_n2o, SERVO_OPEN);
				eng_state->servoN20 = ENGINE_STATE_OPEN;
				break;
			case ENGINE_STATE_OPEN:
				servo_err |= servo_set_state(servo_n2o, SERVO_CLOSED);
				eng_state->servoN20 = ENGINE_STATE_CLOSED;
				break;
			default:
				servo_err |= servo_set_state(servo_n2o, SERVO_CLOSED);
				break;
		}

	if (servo_err)
		control_error_start();
	else {
		// Update OD
		od_write_ENGINE_STATE(&state);
		// Go back to prev state
		prev_state_start();
	}
}
//TODO turn on enable pin
void control_servos_ethanol_run(void){
	//TODO Write things here
}

void control_pressurisation_start(void) {
	control.state = CONTROL_PRESSURISATION;
}

/**
 * @brief Pressurisation state runtime
 * @details This state will wait for the pressurisation sequence to finish and jump back to its previous state.
 * 			This state will open/close the N2O pressurisation valve.
 */
void control_pressurisation_run(void) {
	rf_cmd_t eng_state;
	od_read_ENGINE_CONTROL(&eng_state);

	if (eng_state.pressurization != ENGINE_STATE_CLOSED) {
		solenoid_off(SOLENOID_PRESSURISATION);
		eng_state.pressurization = ENGINE_STATE_CLOSED;
	} else {
		solenoid_on(SOLENOID_PRESSURISATION);
		vent_pressurization_pins = 1;
		eng_state.pressurization = ENGINE_STATE_OPEN;
	}

	// Update OD entry
	od_write_ENGINE_CONTROL(&eng_state);

	prev_state_start();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// AUTOMATIC & SPECIAL STATE RUNTIME FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

void control_glide_start(void) {
	control.state = CONTROL_GLIDE;
	counter.counter = 120 * CONTROL_ONE_SECOND;
}

/**
 * @brief	Glide state runtime
 * @details The glide state will simply wait for the depressurisation action to start and finish
 *			It will also wait until touchdown then return to idle.
 */
void control_glide_run(void) {
	// Wait for 2 minutes before going to IDLE
	// In the future this will be synchronized with the AV state machine

	IDLE_UNTIL_COUNTER_ZERO;
	//TODO Turn off servo_enable pin, reset solenoids to off
	control_idle_start();
}

/**
 * @details	Can only be called from Idle and has lowest priority (just in case :D)
 */
void control_countdown_start(void) {
	control->counter = FINAL_COUNTDOWN;
	schedule_next_state(CONTROL_COUNTDOWN);
}

/**
 * @brief	countdown state runtime
 * @details	This state will wait for the countdown to end and will jump to ignition.
 */
void control_countdown_run(void) {
	// Compute time elapsed since last time, in milliseconds.
	control->last_time = control->time;
	control->time = HAL_GetTick();

	control->counter -= control->time - control->last_time;

	// Check if we're ready to take off!
	if (control->counter <= CONTROL_TAKEOFF_THRESH) {
		control_igniter_start();
	}
}

void control_igniter_start(void) {
	control.state = CONTROL_IGNITER;
}

/**
 * @brief Igniter state runtime
 * @details	Ignition will send the appropriate current for the igniter to turn on.
 * 			This state will wait for ignition to end and will jump to powered.
 */
void control_igniter_run(void) {
	uint8_t error_ignition = 0; // ignition() -> will turn the igniter on (solenoids)

	// Activate ignition
	// TODO when to put igniter's bit to 0 again (loop for 1 second ?)
	HAL_GPIO_WritePin(IGNITER_PORT, IGNITER_PIN, GPIO_PIN_SET);
	// Check if good engine start (pressure and temp?), if too many failed abort

	if (error_ignition) {
		control_abort_start();
		return;
	}

	control_powered_start();

}

void control_ignition_start(void) {
	control.counter = 100; // init counter to 100ms
	control.state = CONTROL_IGNITION;
}

/**
 * @brief	Ignition state runtime
 * @details	This state will open the servos to their 'partially open' state.
 * 			After a delay, it will jump to the thrust state.
 */
void control_ignition_run(void) {
	util_error_t error_ignition = ER_SUCCESS; // powering() -> partial open state of servos

	IDLE_UNTIL_COUNTER_ZERO;

	// Set ethanol and N2O servos (pins 13 and 14) to partially open
	error_ignition |= servo_set_state(servo_n2o, SERVO_PARTIALLY_OPEN);
	error_ignition |= servo_set_state(servo_ethanol, SERVO_PARTIALLY_OPEN);

	//TODO Check for pressure in combustion chamber and define behavior
	double engine_pressure;
	error_ignition != engine_pressure_read(control.i2c_engine_press,&engine_pressure);
	if(engine_pressure</*wanted_pressure*/){
		++ignition_insufficient_pressure_counter;
		if(ignition_insufficient_pressure_counter == IGNITION_COUNTER_THRESHOLD){
			control_abort_start();
			return;
		}
	}
		
	if (error_ignition)
		control_abort_start();
	else {
		// Update OD state
		rf_cmd_t state;
		od_read_ENGINE_CONTROL(&state);
		state->servoN20 = ENGINE_STATE_PARTIALLY_OPEN;
		state->servo_ethanol = ENGINE_STATE_PARTIALLY_OPEN;
		od_write_ENGINE_CONTROL(&state);
		if(engine_pressure</*wanted_pressure*/){
			++ignition_insufficient_counter;
			if(ignition_insufficient_counter == IGNITION_COUNTER_THRESHOLD){
				control_abort_start();
			}
			return;
		}
		control_thrust_start();
	}
}

void control_thrust_start(void) {
	control.state = CONTROL_THRUST;
	control.counter = CONTROL_ONE_SECOND * 10; // 10 second wait before switching state
}

/**
 * @brief	Thrust state runtime
 * @details	This state will open the servos to their 'fully open' position.
 * 			After a delay, it will jump to the shutdown state.
 */
void control_thrust_run(void) {
	util_error_t error_thrust = ER_SUCCESS; // thrust() -> full open state of servos

	error_thurst |= servo_set_state(servo_n2o, SERVO_OPEN);
	error_thrust |= servo_set_rotation(servo_ethanol, SERVO_OPEN);

	// Idle until the counter reaches zero (delay defined in start)
	IDLE_UTIL_COUNTER_ZERO;

	if (error_thrust)
		control_abort_start();
	else {
		// Update OD state
		rf_cmd_t state;
		od_read_ENGINE_CONTROL(&state);
		state->servoN20 = ENGINE_STATE_OPEN;
		state->servo_ethanol = ENGINE_STATE_OPEN;
		od_write_ENGINE_CONTROL(&state);

		control_shutdown_start();
	}
}

void control_shutdown_start(void) {
	control.state = CONTROL_SHUTDOWN;
	control.counter = SHUTDOWN_TO_APOGEE_TIME;
}

/**
 * @brief	Shutdown state runtime
 * @details	This function will stop the engine, depending on which algorithm is chosen (before or during the apogee, tbd).
 * 			After a delay, it will jump to the apogee state.
 * 			Nordend (2023) : engine doesn't have enough power to reach apogee without a full combustion so shutdown goes to glide() 
 */
void control_shutdown_run(void) {
	util_error_t error_shutdown = ER_SUCCESS; // shutdown()

	//TODO define engine shutdown behavior
	//Wait for APOGEE in other FSM

	if (error_shutdown)
		control_abort_start();
	else {
		// Compute time elapsed since last time, in milliseconds.
		control->last_time = control->time;
		control->time = HAL_GetTick();

		control->counter -= control->time - control->last_time;

		// Check if we're ready to reach the top of the sky !
		if (control->counter <= CONTROL_APOGEE_THRESH) {
			control_apogee_start();
		}
	}

}

void control_apogee_start(void) {
	control.state = CONTROL_APOGEE;
}

/**
 * @brief	Apogee state runtime
 * @details	This function will open the venting valves (N2O and ethanol).
 * 			After the end of the sequence, it will jump to the depressurisation state.
 */
void control_apogee_run(void) {
	util_error_t error_start_fall = ER_SUCCESS;

	error_start_fall |= solenoid_on(SOLENOID_N2O);
	error_start_fall |= solenoid_on(SOLENOID_ETHANOL);

	if (error_start_fall)
		control_abort_start();
	else
		control_depressurisation_start();
}

void control_depressurisation_start(void) {
	control.state = CONTROL_DEPRESSURISATION;
}

/**
 * @brief	Depressurisation state runtime
 * @details	This function will open the pressurisation valve (N2O).
 * 			After the end of the sequence, it will jump to the glide state.
 */
void control_depressurisation_run(void) {
	uint8_t error_depressurisation = ER_SUCCESS; // depressurisation() -> open valve

	// TODO put a timer to close the servo again once the tank is definitely empty
	solenoid_on(SOLENOID_PRESSURISATION);

	if (error_depressurisation) {
		control_abort_start();
	} else {
		control_glide_start();
	}
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
	// TODO memorize_in_object_dictionnary_what_went_wrong_for_next_time()

	if (control.prev_state == CONTROL_CALIBRATION)
		if (error_loop_control != 4) {
			++error_loop_control;
			control_calibration_start();
		} else {
			error_loop_control = 0;
			control_idle_start();
		}
	else
		control_idle_start();
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
	// TODO

	control_glide_start();
}

/* END */
