/*
 * solenoide.c
 *
 *  Created on: Dec 11, 2022
 *      Author: julianruizrodriguez
 */

#include "solenoide.h"
#include <abstraction/gpio.h>

#define SOLENOID_PIN	GPIO_PIN_3
#define SOLENOID_PORT	GPIOC


void solenoid_init(void) {
	//GPIO init solenoid

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pin =  SOLENOID_PIN;

}

void solenoid_on(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
}

void solenoid_off(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
}
