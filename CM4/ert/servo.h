/*
 * solenoide.h
 *
 *  Created on: Dec 11, 2022
 *      Author: Iacopo
 */

#ifndef SERVO_H
#define SERVO_H

#include <util.h>

#define SERVO_CHANNEL_GP1 0
#define SERVO_CHANNEL_GP2 1

typedef struct servo {
	uint16_t pwm_channel;
	uint32_t pulse_min;
	uint32_t pulse_max;
	uint16_t angle_max;
}servo_t;


#ifdef __cplusplus
extern "C"{
#endif

util_error_t servo_init(servo_t * servo, uint8_t channel, uint32_t pulse_min, uint32_t pulse_max, uint16_t angle_max);
util_error_t servo_set_angle(servo_t * servo, uint16_t angle);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */



#endif /* SERVO_H */
