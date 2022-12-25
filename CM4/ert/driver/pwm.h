/*  Title       : PWM
 *  Filename    : pwm.h
 *  Author      : Julian Marmier
 *  Date        : 14.12.2022
 *  Version     : 0.1
 *  Description : Pulse width modulation (PWM) driver for PWM clocks (TIM2…7 and TIM12…14 only)
 */

#ifndef DRIVER_PWM_H
#define DRIVER_PWM_H


/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include "tim.h"
#include "util.h"

/**********************
 *  CONSTANTS
 **********************/

/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

/**
 * Timers that are available for use.
 * Should reflect the current board pin configuration.
 */
typedef enum PWM_Timer {
	PWM_TIM3,
	PWM_TIM4,
	PWM_TIM5
} PWM_Timer_t;

typedef enum PWM_Channel_Selection {
	PWM_SELECT_CH1 = 0b1,
	PWM_SELECT_CH2 = 0b10,
	PWM_SELECT_CH3 = 0b100,
	PWM_SELECT_CH4 = 0b1000
} PWM_Channel_Selection_t;

typedef struct pwm_data {
	TIM_HandleTypeDef htim;			/*!< One of htim2…htim7 */
	float usec;						/*!< The current microseconds */
	uint8_t channels; 				/*!< One-hot encoding to represent which channels to activate for this particular instance. */
} pwm_data_t;

/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

util_error_t pwm_init(pwm_data_t * data, PWM_Timer_t timer, uint8_t channel_sel);
util_error_t pwm_init_arr(pwm_data_t * data, PWM_Timer_t timer, uint8_t channel_sel, uint32_t arr_val);

util_error_t pwm_set_microseconds(pwm_data_t * data, uint32_t us, uint8_t channel_sel);
uint32_t pwm_read_microseconds(pwm_data_t * data, uint8_t channel_index);

util_error_t pwm_set_duty(pwm_data_t * data, uint32_t new_duty, uint8_t channel_sel);
uint32_t read_duty(pwm_data_t * data, uint8_t channel_index);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* DRIVER_PWM_H_ */
