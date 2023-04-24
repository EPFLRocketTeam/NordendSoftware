/*  Title		: engine_pressure.h
*  Filename	: engine_pressure.h
*	Author		: Thilo Chalas, Luciano Calcoen
*	Date		: 17.11.2022
*	Version		: 0.1
*	Description	: Engine nozzle pressure sensor setup and acquisition
*/

#ifndef ADC_ENGINE_H
#define ADC_ENGINE_H

#include <sensor/ADC.h>

util_error_t engine_pressure_init(device_t * eng_press);
util_error_t engine_pressure_read(device_t * eng_press, double * data);
util_error_t engine_pressure_calibrate(device_t * eng_press);

#ifdef __cplusplus
extern "C"{
#endif


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif