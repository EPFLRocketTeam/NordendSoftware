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

//Associates Solenoids and their pins
typedef enum Solenoids {
	SOLENOID_1 = GPIO_PIN_6;
	SOLENOID_2 = GPIO_PIN_5;
	SOLENOID_3 = GPIO_PIN_4;
	SOLENOID_4 = GPIO_PIN_3;
}Solenoids_t;

////////////////////////////////////////////////////
//Maybe not necessary?
void solenoid_init(void) {
	//GPIO init solenoid

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pin =  SOLENOID_PIN;

}
///////////////////////////////////////////////////



//Set a solenoid's pin to 1
void solenoid_on(Solenoids_t solenoid){
	HAL_GPIO_WritePin(GPIOC, solenoid, GPIO_PIN_SET);
}

//Set a solenoid's pin to 0
void solenoid_off(Solenoids_t solenoid){
	HAL_GPIO_WritePin(GPIOC, solenoid, GPIO_PIN_RESET);
}
