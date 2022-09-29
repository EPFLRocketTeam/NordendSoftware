/*  Title		: I2C
 *  Filename	: i2c.c
 *	Author		: iacopo sprenger
 *	Date		: 22.04.2022
 *	Version		: 0.1
 *	Description	: i2c interface for the generic device driver
 */


#ifndef I2C_H
#define I2C_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <device/device.h>
#include <semphr.h>

/**********************
 *  CONSTANTS
 **********************/

#define I2C_TIMEOUT	pdMS_TO_TICKS(10)

/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

/**
 * @brief I2C interface context structure
 * @details this only contains the HAL I2C handle pointer.
 */
typedef struct i2c_interface_context {
	I2C_HandleTypeDef * i2c;
	SemaphoreHandle_t sem;
	StaticSemaphore_t sem_buffer;
	util_error_t error;

}i2c_interface_context_t;


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

device_interface_t ** i2c_get_interfaces(void);
uint32_t i2c_get_interfaces_count(void);

void i2c_init(void);
void i2c_spi_guard(void);
device_interface_t * i2c_get_sensor_interface(void);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* I2C_H */

/* END */
