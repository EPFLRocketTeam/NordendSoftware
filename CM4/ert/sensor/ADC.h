/*  Title		: ADC.h
 *  Filename	: ADC.h
 *	Author		: Thilo Chalas, Luciano Calcoen
 *	Date		: 19.11.2022
 *	Version		: 0.1
 *	Description	: ADC driver
 */

#ifndef ADC_H
#define ADC_H



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


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

util_error_t adc_read_voltage(device_t * adc, double * data, uint8_t channel);
util_error_t adc_init(device_t * adc);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* ACCELEROMETER_H */

/* END */
