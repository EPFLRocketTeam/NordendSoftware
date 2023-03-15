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

//in bar
#define MIN_PRESSURE 0
#define MAX_PRESSURE 10

//in V (must be defined with the type of sensor)
#define AA_Vdc 5
#define CH_Vdc 8

//static double const VREF = 2.048;
const static double LIN_RATIO = (MAX_PRESSURE-MIN_PRESSURE)/(0.8*AA_Vdc);

void engine_sensor_convert_pres(double * data);

util_error_t engine_pressure_init(device_t * eng_press){
    util_error_t error = ER_SUCCESS;
    error |= adc_init(eng_press);
    return error;
}

/*to-do once we have the hardware values (Vref, Vmin, Vmax)*/
void engine_sensor_convert_pres(double * data) {
	*data = LIN_RATIO * (*data - 0.1*AA_Vdc) + MIN_PRESSURE;
}

util_error_t engine_pressure_read(device_t * eng_press, double * data) {
	util_error_t error = ER_SUCCESS;
    error |= adc_read_voltage(eng_press, data, PRESS_SENS_ADC_CHANNEL);
    engine_sensor_convert_pres(data);
	return error;
}

util_error_t engine_pressure_calibrate(device_t * eng_press) {
    util_error_t error = ER_SUCCESS;
    //no calibration necessary as the pressure_sensor "will" stay in the range of Pressure min and pressure max 
    return error;
}