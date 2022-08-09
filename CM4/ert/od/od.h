/*  Title       : Object dictionary
 *  Filename    : od.h
 *  Author      : Oliver Facklam
 *  Date        : 28.05.2022
 *  Version     : 0.1
 *  Description : Object dictionary implementation
 */

#ifndef OD_H
#define OD_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>

#include "sensor/accelerometer.h"
#include "sensor/barometer.h"
#include "sensor/gyroscope.h"
#include "sensor/gnss.h"

/**********************
 *  CONSTANTS
 **********************/

#define OD_FRAME_MAX_SIZE   (64)    // 64 bytes for CAN frame size
#define OD_MAX_DATAID       (256U)   // 8-bit identifiers

/**********************
 *  MACROS
 **********************/

#define DECLARE_OD_ENTRY(NAME, TYPE) \
    void od_read_ ## NAME  (TYPE *dst); \
    void od_write_ ## NAME (TYPE *src);

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

void od_init();
void od_update_task(void *argument);

DECLARE_OD_ENTRY(ACC_I2C_A, accelerometer_data_t);
DECLARE_OD_ENTRY(ACC_SPI_A, accelerometer_data_t);
DECLARE_OD_ENTRY(ACC_I2C_B, accelerometer_data_t);
DECLARE_OD_ENTRY(ACC_SPI_B, accelerometer_data_t);
DECLARE_OD_ENTRY(GYRO_I2C_A, gyroscope_data_t);
DECLARE_OD_ENTRY(GYRO_SPI_A, gyroscope_data_t);
DECLARE_OD_ENTRY(GYRO_I2C_B, gyroscope_data_t);
DECLARE_OD_ENTRY(GYRO_SPI_B, gyroscope_data_t);
DECLARE_OD_ENTRY(BARO_I2C_A, barometer_data_t);
DECLARE_OD_ENTRY(BARO_SPI_A, barometer_data_t);
DECLARE_OD_ENTRY(BARO_I2C_B, barometer_data_t);
DECLARE_OD_ENTRY(BARO_SPI_B, barometer_data_t);
DECLARE_OD_ENTRY(GNSS, gnss_data_t);
DECLARE_OD_ENTRY(BATTERY_A, uint32_t);
DECLARE_OD_ENTRY(BATTERY_B, uint32_t);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* OD_H */

/* END */
