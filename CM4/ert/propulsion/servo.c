/*  Title		: Template
 *  Filename	: template.c
 *	Author		: Julian Marmier
 *	Date		: 01.12.2022
 *	Version		: 0.1
 *	Description	: Interface for the SB2290SG Monster Torque
 *				  Brushless Servo. Can be generalized to work with different servos.
 */

/**********************
 *	INCLUDES
 **********************/

#include "servo.h"
#include <math.h>
#include <tim.h>

/**********************
 *	CONSTANTS
 **********************/

/**
 * Instance of Ethanol servo.
 */
static servo_t ethanol_servo_inst;
servo_t * servo_ethanol = &ethanol_servo_inst;
/**
 * Origin offset (in microseconds), for computing the pulse width. Default is 1500.
 */
static const float SERVO_ETHANOL_OFFSET = 1500;



/**
 * Instance of N2O servo
 */
static servo_t servo_n2o_inst;
servo_t * servo_n2o = &servo_n2o_inst;
/**
 * Origin offset (in microseconds), for computing the pulse width. Default is 1500.
 */
static const float SERVO_N2O_OFFSET = 1500;

const uint32_t DEFAULT_OFFSET = 1500;


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
static uint32_t degrees_to_usec(float degrees, float degrees_per_usec) {
	// TODO use offset cleverly here.
	return (uint32_t) degrees / degrees_per_usec;
}


util_error_t servo_init(
	servo_t * servo,
	pwm_data_t * pwm,
	PWM_Channel_Selection_t pwm_channel,
	uint32_t min_pulse,
	uint32_t max_pulse,
	uint32_t origin,
	float degrees_per_usec
) {
	servo->pwm_data = pwm;
	servo->pwm_channel = pwm_channel;
	servo->min_pulse = min_pulse;
	servo->max_pulse = max_pulse;
	// Default origin should be 1500.
	servo->origin = origin;
	servo->degrees_per_usec = degrees_per_usec;

	return ER_SUCCESS;
}

util_error_t servo_set_rotation(servo_t * servo, float newRotation) {
	servo->rotation = newRotation;
	uint32_t us = degrees_to_usec(newRotation, servo->degrees_per_usec) + servo->origin;
	us = clamp_u32(servo->min_pulse, us, servo->max_pulse);

	pwm_set_microseconds(servo->pwm_data, us, servo->pwm_channel);

	return ER_SUCCESS;
}

float servo_get_rotation(servo_t * servo) {
	return servo->rotation;
}

void servo_thread(__attribute__((unused)) void * arg) {
	// TODO remove when servos have been correctly added to engine control thread.

	// Initialize servos -- empty structs
	pwm_data_t pwm_data_inst;
	pwm_data_t * pwm_data = &pwm_data_inst;

	// Specific to the SB2290SG Monster Torque Brushless Servo
	uint32_t min_pulse = 800;
	uint32_t max_pulse = 2200;
	float degrees_per_usec = 0.114;

	servo_init(servo_ethanol, pwm_data, PWM_SELECT_CH1, min_pulse, max_pulse, SERVO_ETHANOL_OFFSET, degrees_per_usec);
	servo_init(servo_n2o, pwm_data, PWM_SELECT_CH2, min_pulse, max_pulse, SERVO_N2O_OFFSET, degrees_per_usec);

	// Using channels 1 and 2 -- initialize the PWM channel
	pwm_init(pwm_data, PWM_TIM5, servo_ethanol->pwm_channel | servo_n2o->pwm_channel);
}




/* END */
