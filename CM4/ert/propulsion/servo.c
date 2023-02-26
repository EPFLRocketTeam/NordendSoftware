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

/// TODO - change the time used here?

/**
 * Instance of first servo
 */
servo_t servo1_inst;
servo_t * servo1 = &servo1_inst;
/**
 * Open/closed constants for the valves of first servo.
 */
const float SERVO1_OPEN = 0;
const float SERVO1_CLOSED = 90.0;
/**
 * Origin offset (in microseconds), for computing the pulse width. Default is 1500.
 */
const float SERVO1_OFFSET = 1500;



/**
 * Instance of second servo
 */
servo_t servo2_inst;
servo_t * servo2 = &servo2_inst;
/**
 * Open/closed constants for the valves of second servo.
 */
const float SERVO2_OPEN = 0;
const float SERVO2_CLOSED = 90.0;
/**
 * Origin offset (in microseconds), for computing the pulse width. Default is 1500.
 */
const float SERVO2_OFFSET = 1500;

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

util_error_t set_rotation(servo_t * servo, float newRotation) {
	servo->rotation = newRotation;
	uint32_t us = degrees_to_usec(newRotation, servo->degrees_per_usec) + servo->origin;
	us = clamp_u32(servo->min_pulse, us, servo->max_pulse);

	pwm_set_microseconds(servo->pwm_data, us, servo->pwm_channel);

	return ER_SUCCESS;
}

float get_rotation(servo_t * servo) {
	return servo->rotation;
}

void servo_thread(__attribute__((unused)) void * arg) {
	// Initialize servos -- empty structs
	pwm_data_t pwm_data_inst;
	pwm_data_t * pwm_data = &pwm_data_inst;

	// Specific to the SB2290SG Monster Torque Brushless Servo
	uint32_t min_pulse = 800;
	uint32_t max_pulse = 2200;
	float degrees_per_usec = 0.114;

	servo_init(servo1, pwm_data, PWM_SELECT_CH1, min_pulse, max_pulse, SERVO1_OFFSET, degrees_per_usec);
	servo_init(servo2, pwm_data, PWM_SELECT_CH2, min_pulse, max_pulse, SERVO2_OFFSET, degrees_per_usec);

	// Using channels 1 and 2 -- initialize the PWM channel
	pwm_init(pwm_data, PWM_TIM5, servo1->pwm_channel | servo2->pwm_channel);
}




/* END */
