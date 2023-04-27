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
#include <feedback/led.h>

/**********************
 *	CONSTANTS
 **********************/

const uint32_t DEFAULT_OFFSET = 1500;
const uint32_t SERVO_ETHANOL_OFFSET = 1500;

//static const led_color_t led_red = {
//		.r = 0xff,
//		.g = 0x00,
//		.b = 0x00
//};
//
//static const led_color_t led_green = {
//		.r = 0x00,
//		.g = 0xff,
//		.b = 0x00
//};
//
//static const led_color_t led_blue = {
//		.r = 0x00,
//		.g = 0x00,
//		.b = 0xff
//};

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

servo_t servo_ethanol = {0};

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

void servo_thread(__attribute__((unused)) void * arg) {
	// TODO remove when servos have been correctly added to engine control thread.

	// Initialize servos -- empty structs
	pwm_data_t pwm_data_inst;
	pwm_data_t * pwm_data = &pwm_data_inst;

	// Specific to the SB2290SG Monster Torque Brushless Servo
	uint32_t min_pulse = 800;
	uint32_t max_pulse = 2200;
	float degrees_per_usec = 0.114;

	//Not working servo_init(&servo_ethanol, pwm_data, PWM_SELECT_CH1, min_pulse, max_pulse, SERVO_ETHANOL_OFFSET, degrees_per_usec, 0, 45, 90);

	// Using channels 1 and 2 -- initialize the PWM channel
	//pwm_init(pwm_data, PWM_TIM5, servo_ethanol.pwm_channel);

	// Debug code
//	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
//
//	htim5.Instance->ARR = 1000000;

	for (;;) {

		osDelay(1000);
		led_rgb_set_color(led_red);
		htim5.Instance->CCR1 = 100000;
		osDelay(1000);
		led_rgb_set_color(led_blue);
		htim5.Instance->CCR1 = 500000;
		/*
		// servo_set_state(&servo_ethanol, SERVO_CLOSED);
		pwm_set_microseconds(pwm_data, 800, PWM_SELECT_CH1);
		debug_log("Hello\n");
		led_rgb_set_color(led_red);
		osDelay(1000);
		pwm_set_microseconds(pwm_data, 1500, PWM_SELECT_CH1);
		led_rgb_set_color(led_green);
//		servo_set_state(&servo_ethanol, SERVO_PARTIALLY_OPEN);
		osDelay(1000);
		pwm_set_microseconds(pwm_data, 2100, PWM_SELECT_CH1);
		led_rgb_set_color(led_blue);
		*/
//		servo_set_state(&servo_ethanol, SERVO_OPEN);
	}
}




/* END */
