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

#include <tim.h>

/**********************
 *	CONSTANTS
 **********************/

//// TODO - change the time used here.
//#define SERVO_TIM			PWM_TIM5 // Using timer 5

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
	return (uint32_t) degrees / degrees_per_usec;
}


util_error_t servo_init(
	servo_t * servo,
	pwm_data_t * pwm,
	PWM_Channel_Selection_t pwm_channel,
	uint32_t min_pulse,
	uint32_t max_pulse,
	float degrees_per_usec
) {
	servo->pwm_data = pwm;
	servo->pwm_channel = pwm_channel;
	servo->min_pulse = min_pulse;
	servo->max_pulse = max_pulse;
	servo->degrees_per_usec = degrees_per_usec;

	return ER_SUCCESS;
}

util_error_t set_rotation(servo_t * servo, float newRotation) {
	servo->rotation = newRotation;
	uint32_t us = degrees_to_usec(newRotation, servo->degrees_per_usec);

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

	servo_t servo1_inst;
	servo_t * servo1 = &servo1_inst;
	servo_init(servo1, pwm_data, PWM_SELECT_CH1, min_pulse, max_pulse, degrees_per_usec);

	servo_t servo2_inst;
	servo_t * servo2 = &servo2_inst;
	servo_init(servo2, pwm_data, PWM_SELECT_CH2, min_pulse, max_pulse, degrees_per_usec);

	// Using channels 1 and 2 -- initialize the PWM channel
	pwm_init(pwm_data, PWM_TIM5, servo1->pwm_channel | servo2->pwm_channel);
}




/* END */
