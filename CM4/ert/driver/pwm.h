/*  Title       : PWM
 *  Filename    : pwm.h
 *  Author      : Julian Marmier
 *  Date        : 14.12.2022
 *  Version     : 0.1
 *  Description : Pulse width modulation (PWM) driver
 */

#ifndef DRIVER_PWM_H
#define DRIVER_PWM_H


/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include "tim.h"

/**********************
 *  CONSTANTS
 **********************/


/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

typedef struct pwm_init_structure {
	TIM_HandleTypeDef htim;			/*!< One of htim1…htim8 */
	uint32_t alternate_function;
	uint16_t gpio_pin;				/*!< GPIO pins associated with the timer channels. @see */
	GPIO_TypeDef gpio_bank; 		/*!< One of GPIOA...K */
	uint8_t channels; 				/*!< number of timer channels to use */
} pwm_init_structure_t;

/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

util_error_t pwm_init(pwm_init_structure_t * data);
util_error_t setMicroseconds(float us);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* DRIVER_PWM_H_ */
