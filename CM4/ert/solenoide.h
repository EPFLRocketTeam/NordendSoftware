/*
 * solenoide.h
 *
 *  Created on: Dec 11, 2022
 *      Author: julianruizrodriguez
 */

#include <util.h>
#include <device/device.h>

//Associates Solenoids and their pins
typedef enum Solenoids {
	SOLENOID_1 = GPIO_PIN_6;
	SOLENOID_2 = GPIO_PIN_5;
	SOLENOID_3 = GPIO_PIN_4;
	SOLENOID_4 = GPIO_PIN_3;
}Solenoids_t;

void solenoid_init(void);

void solenoid_on(Solenoids_t pin);
void solenoid_off(Solenoids_t pin);
