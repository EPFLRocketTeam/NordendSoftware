/*  Title       : PWM
 *  Filename    : pwm.h
 *  Author      : Julian Marmier
 *  Date        : 14.12.2022
 *  Version     : 0.1
 *  Description : Pulse width modulation (PWM) driver for TIM5
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

typedef enum PWM_Channel_Selection {
	PWM_SELECT_CH1 = 0b1,
	PWM_SELECT_CH2 = 0b10,
} PWM_Channel_Selection_t;

//typedef struct pwm_data {
//	TIM_HandleTypeDef htim;			/*!< One of htim2…htim7 */
//	float usec;						/*!< The current microseconds */
//	uint8_t channels; 				/*!< One-hot encoding to represent which channels to activate for this particular instance. */
//} pwm_data_t;

/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @fn util_error_t pwm_init(void)
 * @brief Initializes the PWM.
 * @return ER_SUCCESS on success, otherwise an error code.
 */
util_error_t pwm_init(void);

/**
 * @fn util_error_t pwm_set_period(uint16_t)
 * @param us the period in microseconds.
 * @return ER_SUCCESS on success, otherwise an error code.
 */
util_error_t pwm_set_period(uint16_t us);

/**
 * @fn util_error_t pwm_set_microseconds(uint32_t, uint8_t)
 * @brief Sets the given channel value to the given pulse width.
 *
 * @param us The pulse width (in microseconds)
 * @param channel_sel
 * @return ER_SUCCESS on success, otherwise an error code.
 */
util_error_t pwm_set_microseconds(uint32_t us, uint8_t channel_sel);

/**
 * @fn uint32_t pwm_read_microseconds(pwm_data_t*, uint8_t)
 * @brief Gets the pulse width from the given channel
 *
 * @param channel_index the channel index (1 or 2) to read from
 * @return ER_SUCCESS on success, otherwise an error code.
 */
uint32_t pwm_read_microseconds(uint8_t channel_index);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* DRIVER_PWM_H_ */
