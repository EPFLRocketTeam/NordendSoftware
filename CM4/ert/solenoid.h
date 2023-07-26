/*
 * solenoide.h
 *
 *  Created on: Dec 11, 2022
 *      Author: julianruizrodriguez
 */

#include <util.h>
#include <device/device.h>

typedef struct solenoid {
	uint8_t norm_open;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	uint8_t open; //1 = open
}solenoid_t;


util_error_t solenoid_init(solenoid_t * valve, uint8_t norm_open, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


util_error_t solenoid_open(solenoid_t * valve);


util_error_t solenoid_close(solenoid_t * valve);

util_error_t solenoid_active(solenoid_t * valve);


util_error_t solenoid_inactive(solenoid_t * valve);
