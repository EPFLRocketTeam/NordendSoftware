/*
 * temperature.c
 *
 *  Created on: 24 nov. 2022
 *      Author: julianruizrodriguez
 */

#include "temperature.h"

static const double MIN_TEMP = -70;
#define MAX_TEMP 550;
static const double  LINEAR_COEFFICIENT = 0.003850;
static const double R0 =1000.0;
static const double CURRENT =0.0001; //recommended in A
static const double CORRECTION_TO_SELF_HEATING =CURRENT*CURRENT*0.2;

util_error_t temperature_sensor_init(device_t * temp_sensor){
    util_error_t error = ER_SUCCESS;
    error |= adc_init(temp_sensor);
    return error;
}

util_error_t  temperature_sensor_read(device_t * temp_sensor, temperature_data_t * data) {
	util_error_t error = ER_SUCCESS;
	double voltage;

	error |= adc_read_voltage(temp_sensor, & voltage,  1);

	double resistance = voltage/CURRENT;

	data->temperature = LINEAR_COEFFICIENT*(resistance-R0);

	return error;
}

util_error_t temperature_sensor_calibrate(device_t * temp_sensor) {
	util_error_t error = ER_SUCCESS;

	//TODO write code to calibrate the temperature sensor if necessary

	return error;
}
