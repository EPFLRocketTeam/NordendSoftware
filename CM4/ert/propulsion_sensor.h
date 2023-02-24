/*  Title		: Propulsion I2C sensors acquisition
 *  Filename	: propulsion_sensor.h
 *	Author		: Thilo CHALAS, Luciano CALCOEN
 *	Date		: 28.11.2022
 *	Version		: 0.1
 *	Description	: Thread grouping all the propulsion sensors on the same I2C bus.
 */

#ifndef I2C_PROP_SENS_ACQ_H
#define I2C_PROP_SENS_ACQ_H



/**********************
 *  INCLUDES
 **********************/

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


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

void prop_sensor_i2c_thread(__attribute__((unused)) void * arg);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* I2C_ACQ_H */

/* END */
