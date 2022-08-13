/*  Title		: barometer
 *  Filename	: barometer.c
 *	Author		: iacopo sprenger
 *	Date		: 10.06.2022
 *	Version		: 0.1
 *	Description	: barometer setup and acquisition
 */

/**********************
 *	INCLUDES
 **********************/

#include <sensor/barometer.h>
#include <util.h>

/**********************
 *	CONSTANTS
 **********************/

#define DATA_READ	0x00
#define PROM_READ	0xA0
#define BARO_RESET	0xE1

#define CONVERT_D1	0x48
#define CONVERT_D2 	0x58


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/






/**********************
 *	VARIABLES
 **********************/



/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/



/*
 * Temporary read temperature
 * needs to send the temperature conversion command before.
 */

util_error_t barometer_read_prom(device_t * baro, barometer_meta_t * meta) {
	util_error_t error = device_read(baro, PROM_READ, (uint8_t *) meta->baro_coeffs, 16);
	return error;
}

util_error_t barometer_convert_temp(device_t * baro) {
	util_error_t error = device_write_u8(baro, CONVERT_D2, 0);
	return error;
}

util_error_t barometer_read_temp(device_t * baro, barometer_meta_t * meta) {
	uint8_t data[3];
	util_error_t error = device_read(baro, DATA_READ, data, 3);
	meta->baro_d2 = (data[0] << 16) + (data[1] << 8) + data[2];

	meta->baro_dt = meta->baro_d2 - (meta->baro_coeffs[5] << 8);
	meta->baro_temp = 2000 + (((int64_t)meta->baro_dt * meta->baro_coeffs[6]) >> 23);
	return error;
}

util_error_t barometer_convert_pres(device_t * baro) {
	util_error_t error = device_write_u8(baro, CONVERT_D1, 0);
	return error;
}


util_error_t barometer_read_pres(device_t * baro, barometer_meta_t * meta) {
	uint8_t data[3];
	util_error_t error = device_read(baro, DATA_READ, data, 3);
	meta->baro_d1 = (data[0] << 16) + (data[1] << 8) + data[2];

	meta->baro_offset = ((int64_t)meta->baro_coeffs[2] << 16) + (((int64_t)meta->baro_dt * meta->baro_coeffs[4]) >> 7);
	meta->baro_sensitivity = (meta->baro_coeffs[1] << 15) + (((int64_t)meta->baro_dt * meta->baro_coeffs[3]) >> 8);
	meta->baro_pressure = ((((meta->baro_d1 * meta->baro_sensitivity) >> 21) - meta->baro_offset) >> 15);

	return error;
}


//dumb convert
//-> we could read other sensors while waiting for conversion to end!!
util_error_t barometer_read(device_t * baro, barometer_meta_t * meta) {
	util_error_t error = ER_SUCCESS;
	//launch temp read
	error |= barometer_convert_temp(baro);
	//yield for 10ms
	osDelay(pdMS_TO_TICKS(10));
	//read temp
	error |= barometer_read_temp(baro, meta);
	//lanch press read
	error |= barometer_convert_pres(baro);
	//yield for 10ms
	osDelay(pdMS_TO_TICKS(10));
	//read temp
	error |= barometer_read_pres(baro, meta);

	return error;
}

void barometer_convert(barometer_meta_t * meta, barometer_data_t * data) {
	data->pressure = meta->baro_pressure;
	data->temperature = meta->baro_temp;
	//data->altitude = ((t0+21575.0)/100.0)/A*(pow(((float)p)/((float)p0), -A*R/G0) - 1);
}

/**
 * @brief Initialize barometers
 */
util_error_t barometer_init(device_t * baro, barometer_meta_t * meta) {

	barometer_read_prom(baro, meta);

	return ER_SUCCESS;
}

/* END */
