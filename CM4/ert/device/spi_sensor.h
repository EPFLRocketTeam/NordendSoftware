/*  Title		: spi_sensor
 *  Filename	: spi_sensor.h
 *	Author		: iacopo sprenger
 *	Date		: 22.04.2022
 *	Version		: 0.1
 *	Description	: device driver for spi sensors
 */

#ifndef SPI_SENSOR_H
#define SPI_SENSOR_H



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
typedef struct spi_sensor_context {
	union data {
	} data;
}spi_sensor_context_t;


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif


util_error_t spi_sensor_init(void);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* SPI_SENSOR_H */

/* END */
