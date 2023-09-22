/*
 * servo.c
 *
 *  Created on: Dec 11, 2022
 *      Author: Iacopo Sprenger
 */

#include "servo.h"
#include <tim.h>


#define SERVO_TIM			htim5


#define SERVO_BASE_FREQ		(200e6)
#define SERVO_PSC			(200)

#define SERVO_FREQ		(50) //Hz

#define SERVO_MAX			((SERVO_BASE_FREQ/SERVO_PSC)/SERVO_FREQ)



/*
 * Only supports two channels lol
 * pulses on us
 */
util_error_t servo_init(servo_t * servo, uint8_t channel, uint32_t pulse_min, uint32_t pulse_max, uint16_t angle_max) {
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	servo->pulse_min = pulse_min;
	servo->pulse_max = pulse_max;
	servo->angle_max = angle_max;

	SERVO_TIM.Instance->PSC = SERVO_PSC-1;
	SERVO_TIM.Instance->ARR = SERVO_MAX;

	if(channel == SERVO_CHANNEL_GP0) {
		SERVO_TIM.Instance->CCR1 = pulse_min;
		servo->pwm_channel = TIM_CHANNEL_1;
		HAL_TIM_PWM_Start(&SERVO_TIM, TIM_CHANNEL_1);
	} else if (channel == SERVO_CHANNEL_GP1) {
		SERVO_TIM.Instance->CCR2 = pulse_min;
		servo->pwm_channel = TIM_CHANNEL_2;
		HAL_TIM_PWM_Start(&SERVO_TIM, TIM_CHANNEL_2);
	} else {
		return ER_OUT_OF_RANGE;
	}
	return ER_SUCCESS;
}

util_error_t servo_set_angle(servo_t * servo, uint16_t angle) {
	uint32_t new_pulse;
	uint32_t delta_t = servo->pulse_max - servo->pulse_min;

	new_pulse = delta_t*angle/servo->angle_max + servo->pulse_min;

	if(servo->pwm_channel == TIM_CHANNEL_1) {
		SERVO_TIM.Instance->CCR1 = new_pulse;
	} else if(servo->pwm_channel == TIM_CHANNEL_2) {
		SERVO_TIM.Instance->CCR2 = new_pulse;
	}
	return ER_SUCCESS;
}


//
//util_error_t servo_open(servo_t * servo) {
//	if(servo->pwm_channel == TIM_CHANNEL_1) {
//		SERVO_TIM.Instance->CCR1 = servo->pulse_open;
//	} else if(servo->pwm_channel == TIM_CHANNEL_2) {
//		SERVO_TIM.Instance->CCR2 = servo->pulse_open;
//	}
//	return ER_SUCCESS;
//}
//
//util_error_t servo_half(servo_t * servo) {
//	if(servo->pwm_channel == TIM_CHANNEL_1) {
//		SERVO_TIM.Instance->CCR1 = servo->pulse_half;
//	} else if(servo->pwm_channel == TIM_CHANNEL_2) {
//		SERVO_TIM.Instance->CCR2 = servo->pulse_half;
//	}
//	return ER_SUCCESS;
//}
//
//
//util_error_t servo_close(servo_t * servo) {
//	if(servo->pwm_channel == TIM_CHANNEL_1) {
//		SERVO_TIM.Instance->CCR1 = servo->pulse_closed;
//	} else if(servo->pwm_channel == TIM_CHANNEL_2) {
//		SERVO_TIM.Instance->CCR2 = servo->pulse_closed;
//	}
//	return ER_SUCCESS;
//}
