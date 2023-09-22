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

#include "data_types.h"


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


typedef struct {
    uint8_t data_id;
    uint8_t size;
    uint8_t *data;
} od_entry_t;

typedef struct {
    uint8_t data_id;
    uint8_t size;
    uint8_t data[OD_FRAME_MAX_SIZE];
} od_frame_t;


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

void od_sync_handler(uint8_t opcode, uint16_t len, uint8_t * data);
void od_can_handler(uint8_t opcode, uint16_t len, uint8_t * data);

void od_init();
void od_update_task(void *argument);
void od_broadcast_task(void *argument);

void od_handle_can_frame(uint8_t src, od_frame_t *frame);
void od_push_to_out_q(od_frame_t *frame);
void od_pop_from_in_q(od_frame_t *frame);
void od_push_to_in_q(od_frame_t *frame);
void od_pop_from_out_q(od_frame_t *frame);

DECLARE_OD_ENTRY(KALMAN_DATA_A, 			kalman_data_t);
DECLARE_OD_ENTRY(KALMAN_DATA_B, 			kalman_data_t);
DECLARE_OD_ENTRY(GNSS_DATA_A, 				gnss_data_t);
DECLARE_OD_ENTRY(GNSS_DATA_B, 				gnss_data_t);
DECLARE_OD_ENTRY(BATTERY_A, 				battery_data_t);
DECLARE_OD_ENTRY(BATTERY_B, 				battery_data_t);
DECLARE_OD_ENTRY(ENGINE_CONTROL_DATA, 		engine_control_data_t);
DECLARE_OD_ENTRY(RECOVERY_CONTROL_DATA, 	recovery_control_data_t);
DECLARE_OD_ENTRY(SENSOR_BARO_A_0, 			sensor_baro_data_t);
DECLARE_OD_ENTRY(SENSOR_BARO_A_1, 			sensor_baro_data_t);
DECLARE_OD_ENTRY(SENSOR_BARO_B_0, 			sensor_baro_data_t);
DECLARE_OD_ENTRY(SENSOR_BARO_B_1, 			sensor_baro_data_t);
DECLARE_OD_ENTRY(SENSOR_IMU_A_0, 			sensor_imu_data_t);
DECLARE_OD_ENTRY(SENSOR_IMU_A_1, 			sensor_imu_data_t);
DECLARE_OD_ENTRY(SENSOR_IMU_B_0, 			sensor_imu_data_t);
DECLARE_OD_ENTRY(SENSOR_IMU_B_1, 			sensor_imu_data_t);
DECLARE_OD_ENTRY(SENSOR_MAG_A, 				sensor_mag_data_t);
DECLARE_OD_ENTRY(SENSOR_MAG_B, 				sensor_mag_data_t);
DECLARE_OD_ENTRY(SENSOR_ACC_A_0, 			sensor_acc_data_t);
DECLARE_OD_ENTRY(SENSOR_ACC_A_1, 			sensor_acc_data_t);
DECLARE_OD_ENTRY(SENSOR_ACC_B_0, 			sensor_acc_data_t);
DECLARE_OD_ENTRY(SENSOR_ACC_B_1,			sensor_acc_data_t);
DECLARE_OD_ENTRY(ENGINE_SENSORS_DATA, 		sensor_eng_data_t);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* OD_H */

/* END */
