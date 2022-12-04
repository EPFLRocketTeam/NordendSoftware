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

//in MPa
#define MIN_PRESSURE 0
#define MAX_PRESSURE 10

//in mA
#define MIN_CURRENT 4
#define MAX_CURRENT 20

//in Ohm
#define DIFFERENTIAL_RESISTOR 100

static double const VREF = 2.048;
static double LIN_RATIO = 0;

util_error_t engine_sensor_convert_pres(double * data);

util_error_t engine_pressure_init(device_t * eng_pres){
    util_error_t error = ER_SUCCESS;
    LIN_RATIO = 
    error |= adc_init(eng_pres);
    LIN_RATIO = (double)((MAX_PRESSURE-MIN_PRESSURE)/(
        (DIFFERENTIAL_RESISTOR*MAX_CURRENT)-(DIFFERENTIAL_RESISTOR*MIN_CURRENT)));
    return error;
}

/*to-do once we have the hardware values (Vref, Vmin, Vmax)*/
util_error_t engine_sensor_convert_pres(double * data) {
	*data = LIN_RATIO * (*data);
	return error;
}

util_error_t engine_pressure_read(device_t * eng_press, double * data) {
	util_error_t error = ER_SUCCESS;
    error |= adc_read_data(eng_press, data, VREF, DIFFERENTIAL_MODE);
    engine_sensor_convert_pres(data);
	return error;
}