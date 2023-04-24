/*
 * temperature.h
 *
 *  Created on: 24 nov. 2022
 *      Author: julianruizrodriguez
 */

#ifndef SENSOR_TEMPERATURE_H_
#define SENSOR_TEMPERATURE_H_

#include <sensor/ADC.h>
#include <util.h>

typedef struct temperature_data {
    double temperature;
}temperature_data_t;

util_error_t temperature_sensor_init(device_t * temp_sensor);
util_error_t temperature_sensor_read(device_t * temp_snsor, temperature_data_t * data);
util_error_t temperature_sensor_calibrate(device_t * temp_sensor);

#endif /* SENSOR_TEMPERATURE_H_ */
