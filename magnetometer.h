/*  Title		: magnetometer.h
*  Filename	: magnetometer.h
*	Author		: Julian Ruiz Rodriguez
*	Date		: 28.10.2022
*	Version	: 0.1
*/

#ifndef Magnetometer_h
#define Magnetometer_h

#include <stdio.h>
#include <stdbool.h>
#include "util.h"
#include "device.h"
#include <math.h>

#define MAGNETO_AXIS_COUNT 3

typedef struct magnetometer_data {
    uint16_t magneto_x_axis;
    uint16_t magneto_y_axis;
    uint16_t magneto_z_axis;
}magnetometer_data_t;

util_error_t magnetometer_read_X_axis(device_t * magneto, magnetometer_data_t * data);
util_error_t magnetometer_read_Y_axis(device_t * magneto, magnetometer_data_t * data);
util_error_t magnetometer_read_Z_axis(device_t * magneto, magnetometer_data_t * data);

bool new_set_of_data(device_t * magneto);

enum CONFIG_MODE {
    NO_T_COMPENSATION_NO_REBOOT = 0,
    T_COMPENSATION_NO_REBOOT,
    T_COMPENSATION_REBOOT,
};


#endif /* Magnetometer_h */
