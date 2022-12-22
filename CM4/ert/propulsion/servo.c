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

#include <stdint.h>
#include <main.h>
#include <math.h>
#include <tim.h>
#include <util.h>
#include <cmsis_os.h>
#include <abstraction/gpio.h>

/**********************
 *	CONSTANTS
 **********************/

// TODO - change the time used here.
#define SERVO_TIM			htim5.Instance // Using timer 5
#define PRESCALER 			SERVO_TIM->PSC // TODO define prescaler used here
#define F_CLK				100 // TODO define internal timer clock frequency (in Hz)
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

// Compute prescaler value
static uint8_t usecToCCR(uint8_t usec){
	// TODO Check if resolution and calculation holds
	uint8_t val = (SERVO_TIM->ARR * usec * F_CLK) / ((SERVO_TIM->PSC + 1) * (SERVO_TIM->ARR + 1));
	return val;
}

util_error_t servo_init(servo_t * data) {
	// Initialize GPIO pin
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Alternate = GPIO_AF2_TIM5;
	// TODO – run this through Charlotte + Recovery team !
	GPIO_InitStructure.Pin = data->gpio_pin;
	HAL_GPIO_Init()

	SERVO_TIM->CCR1 = data->min_pulse;
	HAL_TIM_PWM_Start(&SERVO_TIM, data->tim_channel); // Not sure if I should be using channel 1 or 4 ??

	return ER_SUCCESS;
}

util_error_t setRotation(servo_t * servo, float newRotation) {

	return ER_SUCCESS;
}

float getRotation(servo_t * servo) {
	return servo->rotation;
}

void servo_thread(__attribute__((unused)) void * arg) {
	// Initialize auto-reload register
	SERVO_TIM->ARR = 1;

	// Initialize servos -- empty structs
	servo_t servo1;
	servo1->gpio_pin = GPIO_PIN_1;
	servo1->tim_channel = TIM_CHANNEL_1;

	servo_t servo2;
	servo2->gpio_pin = GPIO_PIN_2;
	servo2->tim_channel = TIM_CHANNEL_2;

	init(servo1);
	init(servo2);
}




/* END */
