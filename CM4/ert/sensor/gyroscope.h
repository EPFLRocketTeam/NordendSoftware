/*  Title		: Gyroscope
 *  Filename	: gyroscope.h
 *	Author		: iacopo sprenger
 *	Date		: 10.06.2022
 *	Version		: 0.1
 *	Description	: gyroscope setup and acquisition
 */

#ifndef GYROSCOPE_H
#define GYROSCOPE_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <util.h>
#include <device/device.h>

/**********************
 *  CONSTANTS
 **********************/

#define GYRO_AXIS_COUNT 3


/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

typedef struct gyroscope_data {
	int16_t raw[GYRO_AXIS_COUNT];
	int16_t	processed[GYRO_AXIS_COUNT];
}gyroscope_data_t;


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

util_error_t gyroscope_init(device_t * gyro);

void gyroscope_process_data(gyroscope_data_t * data, uint16_t range);
util_error_t gyroscope_read_data(device_t * acc, gyroscope_data_t * data);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* GYROSCOPE_H */

/* END */
