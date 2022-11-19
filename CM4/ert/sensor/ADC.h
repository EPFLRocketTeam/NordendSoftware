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

typedef uint16_t adc_data_t

/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

util_error_t adc_read_data(device_t * adc, adc_data_t * data);
util_error_t adc_select_channel(device_t * adc, bool channel);
util_error_t adc_init(device_t * adc);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* ACCELEROMETER_H */

/* END */
