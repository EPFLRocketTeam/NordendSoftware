/*  Title		: Barometer
 *  Filename	: barometer.h
 *	Author		: Julian Marmier, adapted from Iacopo Sprenger
 *	Date		: 19.11.2022
 *	Version		: 0.1
 *	Description	: Interface for the Bosch BMP390 barometric pressure sensor
 */

#ifndef BAROMETER_H
#define BAROMETER_H

/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <util.h>
#include <device/device.h>

/**********************
 *  CONSTANTS
 **********************/

/**
 * Total number of bytes to read from the sensor data registers
 */
#define DATA_TOTAL_BYTES 6

/**********************
 *  MACROS
 **********************/

/**********************
 *  TYPEDEFS
 **********************/

/**
 * The calculated data
 */
typedef struct barometer_data {
	uint8_t uncompensated[DATA_TOTAL_BYTES];
	float pressure;
	float temperature;
	float altitude;
	uint32_t timestamp;
} barometer_data_t;

/**
 * Barometer calibration data
 */
typedef struct barometer_meta {
	float par_t1;
	float par_t2;
	float par_t3;
	float t_lin; // variable - this represents the temperature
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
	float pressure_sea_level; // variable - will have to be set during runtime.
} barometer_meta_t;

/**********************
 *  VARIABLES
 **********************/

/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C" {
#endif

util_error_t barometer_read(device_t *baro, barometer_meta_t *meta, barometer_data_t *data);
util_error_t barometer_init(device_t *baro, barometer_meta_t *meta);
void barometer_set_alt(float alt);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* BAROMETER_H */

/* END */
