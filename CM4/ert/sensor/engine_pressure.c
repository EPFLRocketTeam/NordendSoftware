/*  Title		: engine_pressure.c
 *  Filename	: engine_pressure.c
 *	Author		: Thilo Chalas, Luciano Calcoen
 *	Date		: 17.11.2022
 *	Version		: 0.1
 *	Description	: Engine nozzle pressure sensor setup and acquisition
 */

/**********************
 *	INCLUDES
 **********************/

#include <sensor/engine_pressure.h>

/*
 *  DEFINES
*/

#define PRESS_SENS_ADC_CHANNEL 1

//in MPa
#define MIN_PRESSURE 0
#define MAX_PRESSURE 10

//in mA
#define MIN_CURRENT 4
#define MAX_CURRENT 20

//in Ohm
#define DIFFERENTIAL_RESISTOR 100

//static double const VREF = 2.048;
static double LIN_RATIO = 0;

void engine_sensor_convert_pres(double * data);

util_error_t engine_pressure_init(device_t * eng_press){
    util_error_t error = ER_SUCCESS;
    error |= adc_init(eng_press);
    LIN_RATIO = (double)((MAX_PRESSURE-MIN_PRESSURE)/(
        (DIFFERENTIAL_RESISTOR*MAX_CURRENT)-(DIFFERENTIAL_RESISTOR*MIN_CURRENT)));
    return error;
}

/*to-do once we have the hardware values (Vref, Vmin, Vmax)*/
void engine_sensor_convert_pres(double * data) {
	*data = LIN_RATIO * (*data);
}

util_error_t engine_pressure_read(device_t * eng_press, double * data) {
	util_error_t error = ER_SUCCESS;
    error |= adc_read_voltage(eng_press, data, PRESS_SENS_ADC_CHANNEL);
    engine_sensor_convert_pres(data);
	return error;
}

util_error_t engine_pressure_calibrate(device_t * eng_press) {
    util_error_t error = ER_SUCCESS;
    //TODO: write some sort of calibration code for the sensor
    return error;
}