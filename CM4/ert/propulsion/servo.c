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

#if 0

#include "servo.h"
#include <math.h>
#include <tim.h>
#include <feedback/led.h>

/**********************
 *	CONSTANTS
 **********************/

/**********************
 *	MACROS
 **********************/

#define DEG_TO_USEC(degrees, degrees_per_usec) ((uint32_t) degrees / degrees_per_usec)

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

util_error_t servo_init(
	servo_t * servo,
	PWM_Channel_Selection_t pwm_channel,
	uint32_t min_pulse,
	uint32_t max_pulse,
	uint32_t origin,
	float degrees_per_usec,
	float open_rotation,
	float partially_open_rotation,
	float closed_rotation
) {
	servo->pwm_channel = pwm_channel;
	servo->min_pulse = min_pulse;
	servo->max_pulse = max_pulse;

	// Default origin should be 1500.
	servo->origin = origin;
	servo->degrees_per_usec = degrees_per_usec;

	servo->open_rotation = open_rotation;
	servo->partially_open_rotation = partially_open_rotation;
	servo->closed_rotation = closed_rotation;

	servo->state = SERVO_CLOSED;

	return ER_SUCCESS;
}

util_error_t servo_set_rotation(servo_t *servo, float newRotation) {
	servo->rotation = newRotation;

	uint32_t us = DEG_TO_USEC(newRotation, servo->degrees_per_usec) + servo->origin;
	us = clamp_u32(servo->min_pulse, us, servo->max_pulse);

	pwm_set_microseconds(us, servo->pwm_channel);

	return ER_SUCCESS;
}

float servo_get_rotation(servo_t *servo) {
	return servo->rotation;
}

util_error_t servo_set_state(servo_t *servo, servo_state_t new_state) {
	servo->state = new_state;
	switch (new_state) {
		case SERVO_OPEN:
			servo_set_rotation(servo, servo->open_rotation);
			break;
		case SERVO_PARTIALLY_OPEN:
			servo_set_rotation(servo, servo->partially_open_rotation);
			break;
		case SERVO_CLOSED:
			servo_set_rotation(servo, servo->closed_rotation);
			break;
		default:
			servo_set_rotation(servo, servo->closed_rotation);
	}

	return ER_SUCCESS;
}

servo_state_t servo_get_state(servo_t *servo) {
	return servo->state;
}

void servo_thread(__attribute__((unused)) void * arg) {return;}


#endif

/* END */
