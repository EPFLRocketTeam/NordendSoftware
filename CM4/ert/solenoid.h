/*
 * solenoide.h
 *
 *  Created on: Dec 11, 2022
 *      Author: julianruizrodriguez
 */

#ifndef SOLENOID_H
#define SOLENOID_H

#include <util.h>
#include <device/device.h>

typedef struct solenoid {
	uint8_t norm_open;
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	uint8_t open; //1 = open
}solenoid_t;


#ifdef __cplusplus
extern "C"{
#endif

util_error_t solenoid_init(solenoid_t * valve, uint8_t norm_open, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


util_error_t solenoid_open(solenoid_t * valve);


util_error_t solenoid_close(solenoid_t * valve);

util_error_t solenoid_active(solenoid_t * valve);


util_error_t solenoid_inactive(solenoid_t * valve);

uint8_t solenoid_get_active(solenoid_t * valve);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */



#endif /* SOLENOID_H */
