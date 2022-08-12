/*  Title		: Barometer
 *  Filename	: barometer.h
 *	Author		: iacopo sprenger
 *	Date		: 10.06.2022
 *	Version		: 0.1
 *	Description	: barometer setup and acquisition
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


/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/



typedef struct barometer_data {
	int32_t pressure;
	int32_t temperature;
	int32_t altitude;
}barometer_data_t;

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
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

util_error_t barometer_read_prom(device_t * baro, barometer_meta_t * meta);
util_error_t barometer_convert_temp(device_t * baro);
util_error_t barometer_read_temp(device_t * baro, barometer_meta_t * meta);
util_error_t barometer_convert_pres(device_t * baro);
util_error_t barometer_read_pres(device_t * baro, barometer_meta_t * meta);
void barometer_convert(barometer_meta_t * meta, barometer_data_t * data);
util_error_t barometer_init(device_t * baro, barometer_meta_t * meta);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* BAROMETER_H */

/* END */
