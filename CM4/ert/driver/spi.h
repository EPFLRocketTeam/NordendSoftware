/*  Title		: SPI
 *  Filename	: spi.c
 *	Author		: iacopo sprenger
 *	Date		: 22.04.2022
 *	Version		: 0.1
 *	Description	: spi interface for the generic device driver
 */


#ifndef SPI_H
#define SPI_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <device/device.h>
#include <semphr.h>

/**********************
 *  CONSTANTS
 **********************/


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
typedef struct spi_interface_context {
	SPI_HandleTypeDef * spi;
	SemaphoreHandle_t sem;
	StaticSemaphore_t sem_buffer;

}spi_interface_context_t;


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

device_interface_t ** spi_get_interfaces(void);
uint32_t spi_get_interfaces_count(void);

void spi_init(void);
void spi_spi_guard(void);
device_interface_t * spi_get_sensor_interface(void);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* SPI_H */

/* END */
