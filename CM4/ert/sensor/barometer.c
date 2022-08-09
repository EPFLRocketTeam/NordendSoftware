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


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/

typedef struct barometer_meta {
	   uint16_t baro_coeffs[8];
	   uint32_t baro_d1;
	   uint32_t baro_d2;
	   int32_t baro_dt;
	   int32_t baro_temp;
	   int64_t baro_offset;
	   int64_t baro_sensitivity;
	   int32_t baro_pressure;
}barometer_meta_t;

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

util_error_t read_prom(device_t * baro, barometer_meta_t * meta) {

}

util_error_t read_temp(device_t * baro, barometer_meta_t * meta) {
	uint8_t data[3];
	util_error_t error = device_read(baro, BARO_READ, &data, 3)
	meta->baro_d2 = (data[0] << 16) + (data[1] << 8) + data[2];

	meta->baro_dt = meta->baro_d2 - (meta->baro_coeffs[5] << 8);
	meta->baro_temp = 2000 + (((int64_t)meta->baro_dt * meta->baro_coeffs[6]) >> 23);
	return error;
}

util_error_t read_pres(device_t * baro, barometer_meta_t * meta) {
	uint8_t data[3];
	util_error_t error device_read(baro, BARO_READ, &data, 3)
	meta->baro_d1 = (data[0] << 16) + (data[1] << 8) + data[2];

	meta->baro_offset = ((int64_t)meta->baro_coeffs[2] << 16) + (((int64_t)meta->baro_dt * meta->baro_coeffs[4]) >> 7);
	meta->baro_sensitivity = (meta->baro_coeffs[1] << 15) + (((int64_t)meta->baro_dt * meta->baro_coeffs[3]) >> 8);
	meta->baro_pressure = ((((meta->baro_d1 * meta->baro_sensitivity) >> 21) - meta->baro_offset) >> 15);

	return error;
}


util_error_t barometer_read(device_t * baro) {

}

/**
 * @brief Initialize barometers
 */
util_error_t barometer_init(device_t * baro) {



	//Barometer does not have an ID

	//initialize sensor

	return ER_SUCCESS;
}

/* END */
