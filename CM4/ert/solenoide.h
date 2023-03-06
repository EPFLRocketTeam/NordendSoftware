/*
 * solenoide.h
 *
 *  Created on: Dec 11, 2022
 *      Author: julianruizrodriguez
 */

#include <util.h>
#include <device/device.h>

void solenoid_init(void);

void solenoid_on(uint8_t & pin);
void solenoid_off(uint8_t & pin);
