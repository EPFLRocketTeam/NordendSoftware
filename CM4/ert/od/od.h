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

#include "sensor/gnss.h"
#include <protocol/structures.h>
#include <RadioPacket/RadioPacket.h>

#include "sensor/accelerometer.h"
#include "sensor/magnetometer.h"
#include "sensor/temperature.h"
#include "sensor/barometer.h"

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

DECLARE_OD_ENTRY(KALMAN_DATA_A, transfer_data_res_t);
DECLARE_OD_ENTRY(KALMAN_DATA_B, transfer_data_res_t);
DECLARE_OD_ENTRY(GNSS, gnss_data_t);
DECLARE_OD_ENTRY(BATTERY_A, uint32_t);
DECLARE_OD_ENTRY(BATTERY_B, uint32_t);
DECLARE_OD_ENTRY(RF_CMD, rf_cmd_t);
DECLARE_OD_ENTRY(COUNTDOWN, uint16_t);
DECLARE_OD_ENTRY(ENGINE_STATE, rf_cmd_t);
DECLARE_OD_ENTRY(ENGINE_FSM_STATE, uint8_t);
DECLARE_OD_ENTRY(ACC_I2C_A, accelerometer_data_t);
DECLARE_OD_ENTRY(ACC_I2C_B, accelerometer_data_t);
DECLARE_OD_ENTRY(BARO_A, barometer_data_t);
DECLARE_OD_ENTRY(BARO_B, barometer_data_t);
DECLARE_OD_ENTRY(MAG_I2C_A, magnetometer_data_t);
DECLARE_OD_ENTRY(MAG_I2C_B, magnetometer_data_t);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* OD_H */

/* END */
