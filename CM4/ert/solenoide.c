/*
 * solenoide.c
 *
 *  Created on: Dec 11, 2022
 *      Author: julianruizrodriguez
 */

#include "solenoide.h"
#include <abstraction/gpio.h>

#define SOLENOID_PORT	GPIOC

//TODO Check if this is still needed (code below)
////////////////////////////////////////////////////
util_error_t solenoid_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pin =  SOLENOID_1 | SOLENOID_2 | SOLENOID_3 | SOLENOID_4;

	HAL_GPIO_Init(SOLENOID_PORT, &GPIO_InitStructure);

	return ER_SUCCESS;
}
///////////////////////////////////////////////////

//Set a solenoid's pin to 1
util_error_t solenoid_on(Solenoids_t solenoid){
	HAL_GPIO_WritePin(SOLENOID_PORT, solenoid, GPIO_PIN_SET);
	return ER_SUCCESS;
}

//Set a solenoid's pin to 0
util_error_t solenoid_off(Solenoids_t solenoid){
	HAL_GPIO_WritePin(SOLENOID_PORT, solenoid, GPIO_PIN_RESET);
	return ER_SUCCESS;
}
