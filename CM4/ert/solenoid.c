/*
 * solenoide.c
 *
 *  Created on: Dec 11, 2022
 *      Author: julianruizrodriguez
 */

#include "solenoid.h"
#include <abstraction/gpio.h>

util_error_t solenoid_init(solenoid_t * valve, uint8_t norm_open, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	valve->norm_open = norm_open;
	valve->GPIOx = GPIOx;
	valve->GPIO_Pin = GPIO_Pin;
	gpio_config_t conf = {0};
	conf.bias = GPIO_BIAS_LOW;
	conf.drive = GPIO_DRIVE_PP;
	conf.mode = GPIO_MODE_OUT;
	conf.speed = 0;
	gpio_cfg(GPIOx, GPIO_Pin, conf);
	gpio_clr(GPIOx, GPIO_Pin);
	return ER_SUCCESS;
}


util_error_t solenoid_open(solenoid_t * valve) {
	if(valve->norm_open) {
		gpio_clr(valve->GPIOx, valve->GPIO_Pin);
	} else {
		gpio_set(valve->GPIOx, valve->GPIO_Pin);
	}
	valve->open = 1;
	return ER_SUCCESS;
}


util_error_t solenoid_close(solenoid_t * valve) {
	if(valve->norm_open) {
		gpio_set(valve->GPIOx, valve->GPIO_Pin);
	} else {
		gpio_clr(valve->GPIOx, valve->GPIO_Pin);
	}
	valve->open = 0;
	return ER_SUCCESS;
}

util_error_t solenoid_active(solenoid_t * valve) {
	gpio_set(valve->GPIOx, valve->GPIO_Pin);
	if(valve->norm_open) {
		valve->open = 0;
	} else {
		valve->open = 1;
	}
	return ER_SUCCESS;
}


util_error_t solenoid_inactive(solenoid_t * valve) {
	gpio_clr(valve->GPIOx, valve->GPIO_Pin);
	if(valve->norm_open) {
		valve->open = 1;
	} else {
		valve->open = 0;
	}
	return ER_SUCCESS;
}

uint8_t solenoid_get_active(solenoid_t * valve) {
	return gpio_get(valve->GPIOx, valve->GPIO_Pin);
}
