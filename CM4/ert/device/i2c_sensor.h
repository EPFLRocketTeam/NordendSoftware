/*  Title		: i2c_sensor
 *  Filename	: i2c_sensor.c
 *	Author		: iacopo sprenger
 *	Date		: 22.04.2022
 *	Version		: 0.1
 *	Description	: device driver for i2c sensors
 */

#ifndef I2C_SENSOR_H
#define I2C_SENSOR_H



/**********************
 *  INCLUDES
 **********************/

#include <device/device.h>
#include <stdint.h>


/**********************
 *  CONSTANTS
 **********************/


/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

// pas sur pour les données de capteur ici...
typedef struct i2c_sensor_context {
	uint8_t device_address;
	void * context;
}i2c_sensor_context_t;


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

util_error_t i2c_sensor_init(void);
util_error_t i2c_engine_sensor_init(void);

device_t * i2c_sensor_get_bmi088_gyr(uint8_t num);
device_t * i2c_sensor_get_bmi088_acc(uint8_t num);
device_t * i2c_sensor_get_bmp390_baro(uint8_t num);
device_t * i2c_sensor_get_adxl375_acc(uint8_t num);

device_t * i2c_engine_sensor_get_mcp3426_up(void);
device_t * i2c_engine_sensor_get_mcp3426_dn(void);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* I2C_SENSOR_H */

/* END */
