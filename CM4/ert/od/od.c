/*  Title		: Object dictionary
 *  Filename	: od.c
 *	Author		: Oliver Facklam
 *	Date		: 28.05.2022
 *	Version		: 0.1
 *	Description	: Object dictionary implementation
 */

/**********************
 *	INCLUDES
 **********************/

#include "od.h"
#include <cmsis_os2.h>
#include <FreeRTOS.h>
#include <device/comunicator.h>
#include <hostcom.h>
#include <driver/serial.h>
#include <feedback/debug.h>
#include <protocol/structures.h>

#include <string.h>
#include <assert.h>

/**********************
 *	CONSTANTS
 **********************/

#define OD_MSGQ_SIZE (32)
//#define DEBUG_NO_CAN 1      // set to 1 to turn off CAN to test OD functionality only

/**********************
 *	MACROS
 **********************/

#define ALLOCATE_OD_ENTRY(NAME, ID, TYPE) \
    enum { NAME = ID }; \
    static_assert((NAME) < OD_MAX_DATAID); \
    static_assert(sizeof(TYPE) < OD_FRAME_MAX_SIZE); \
    static TYPE (NAME##_var); \
    static const od_entry_t (NAME##_entry) = { .data_id=(NAME), .size=sizeof(TYPE), .data=(uint8_t*)&(NAME##_var) }; \
    \
    void od_read_##NAME  (TYPE *dst) { od_unsafe_read ((NAME), (uint8_t*) dst); } \
    void od_write_##NAME (TYPE *src) { od_unsafe_write((NAME), (uint8_t*) src); }


#define LINK_OD_ENTRY(NAME) [(NAME)] = (NAME##_entry)

/**********************
 *	TYPEDEFS
 **********************/



/**********************
 *  PROTOTYPES
 **********************/

static void od_unsafe_read (uint8_t data_id, uint8_t *dst);
static void od_unsafe_write(uint8_t data_id, uint8_t *src);

/**********************
 *	VARIABLES
 **********************/

/**
 * Object dictionary entries
 */
ALLOCATE_OD_ENTRY(ACC_I2C_A, 0, accelerometer_data_t);
ALLOCATE_OD_ENTRY(ACC_SPI_A, 1, accelerometer_data_t);
ALLOCATE_OD_ENTRY(ACC_I2C_B, 2, accelerometer_data_t);
ALLOCATE_OD_ENTRY(ACC_SPI_B, 3, accelerometer_data_t);
ALLOCATE_OD_ENTRY(GYRO_I2C_A, 4, gyroscope_data_t);
ALLOCATE_OD_ENTRY(GYRO_SPI_A, 5, gyroscope_data_t);
ALLOCATE_OD_ENTRY(GYRO_I2C_B, 6, gyroscope_data_t);
ALLOCATE_OD_ENTRY(GYRO_SPI_B, 7, gyroscope_data_t);
ALLOCATE_OD_ENTRY(BARO_I2C_A, 8, barometer_data_t);
ALLOCATE_OD_ENTRY(BARO_SPI_A, 9, barometer_data_t);
ALLOCATE_OD_ENTRY(BARO_I2C_B, 10, barometer_data_t);
ALLOCATE_OD_ENTRY(BARO_SPI_B, 11, barometer_data_t);
ALLOCATE_OD_ENTRY(KALMAN_DATA_A, 12, transfer_data_res_t);
ALLOCATE_OD_ENTRY(KALMAN_DATA_B, 13, transfer_data_res_t);
ALLOCATE_OD_ENTRY(GNSS, 14, gnss_data_t);
ALLOCATE_OD_ENTRY(BATTERY_A, 15, uint32_t);
ALLOCATE_OD_ENTRY(BATTERY_B, 16, uint32_t);
ALLOCATE_OD_ENTRY(MAG_I2C_A, 17, magnetometer_data_t);
ALLOCATE_OD_ENTRY(MAG_I2C_B, 18, magnetometer_data_t);


/**
 * The object dictionary
 */
static const od_entry_t od_entries[OD_MAX_DATAID] = {
    LINK_OD_ENTRY(ACC_I2C_A),
	LINK_OD_ENTRY(ACC_SPI_A),
	LINK_OD_ENTRY(ACC_I2C_B),
	LINK_OD_ENTRY(ACC_SPI_B),
	LINK_OD_ENTRY(GYRO_I2C_A),
	LINK_OD_ENTRY(GYRO_SPI_A),
	LINK_OD_ENTRY(GYRO_I2C_B),
	LINK_OD_ENTRY(GYRO_SPI_B),
	LINK_OD_ENTRY(BARO_I2C_A),
	LINK_OD_ENTRY(BARO_SPI_A),
	LINK_OD_ENTRY(BARO_I2C_B),
	LINK_OD_ENTRY(BARO_SPI_B),
	LINK_OD_ENTRY(KALMAN_DATA_A),
	LINK_OD_ENTRY(KALMAN_DATA_B),
	LINK_OD_ENTRY(GNSS),
	LINK_OD_ENTRY(BATTERY_A),
	LINK_OD_ENTRY(BATTERY_B),
    LINK_OD_ENTRY(MAG_I2C_A),
    LINK_OD_ENTRY(MAG_I2C_B)
};

/**
 * Synchronization primitives
 */
static osMessageQueueId_t out_q;
static osMessageQueueId_t in_q;


static comunicator_t od_can_comunicator;
static comunicator_t * od_hostcom_comunicator;

/**********************
 *	DECLARATIONS
 **********************/





void od_init() {
    // Initialize queues
	static StaticQueue_t out_cb;
	static od_frame_t out_mem[OD_MSGQ_SIZE];
	osMessageQueueAttr_t out_attr = {
			.name = "OD outgoing queue",
			.cb_mem = &out_cb,
			.cb_size = sizeof(out_cb),
			.mq_mem = &out_mem,
			.mq_size = sizeof(out_mem)
	};
	out_q = osMessageQueueNew(OD_MSGQ_SIZE, sizeof(od_frame_t), &out_attr);

	static StaticQueue_t in_cb;
	static od_frame_t in_mem[OD_MSGQ_SIZE];
	osMessageQueueAttr_t in_attr = {
			.name = "OD incoming queue",
			.cb_mem = &in_cb,
			.cb_size = sizeof(in_cb),
			.mq_mem = &in_mem,
			.mq_size = sizeof(in_mem)
	};
	in_q = osMessageQueueNew(OD_MSGQ_SIZE, sizeof(od_frame_t), &in_attr);

	comunicator_init(&od_can_comunicator, serial_get_s3_interface(), od_can_handler);
	serial_register_handler(serial_get_s3_interface(), communicator_handler, &od_can_comunicator);

}


void od_sync_handler(uint8_t opcode, uint16_t len, uint8_t * data) {

	debug_log("received: %d\n", opcode);

	if(opcode <= BATTERY_B) { //check against last entry to see validity
		//valid data
		if(len == od_entries[opcode].size) {
			debug_log("processed: %d\n", opcode);
			int32_t lock = osKernelLock();
			od_frame_t inbound;
			inbound.data_id = od_entries[opcode].data_id;
			inbound.size = od_entries[opcode].size;
			memcpy(inbound.data, data, inbound.size);
			osKernelRestoreLock(lock);

			osMessageQueuePut(in_q, &inbound, 0U, 100);

		}
	}
}

void od_can_handler(uint8_t opcode, uint16_t len, uint8_t * data) {

	debug_log("receivedCAN: %d\n", opcode);

	if(opcode <= BATTERY_B) { //check against last entry to see validity

		if(opcode == GNSS) {
			gnss_data_t _data;
			memcpy(&_data, data, sizeof(gnss_data_t));
			hostcom_data_gnss_send(HAL_GetTick(), (int32_t) _data.altitude);
			static uint8_t first = 1;
			if(first) {
				barometer_set_alt(_data.altitude);
				first = 0;
			}
		}
		//valid data
		if(len == od_entries[opcode].size) {
			debug_log("processed: %d\n", opcode);
			int32_t lock = osKernelLock();
			od_frame_t inbound;
			inbound.data_id = od_entries[opcode].data_id;
			inbound.size = od_entries[opcode].size;
			memcpy(inbound.data, data, inbound.size);
			osKernelRestoreLock(lock);

			osMessageQueuePut(in_q, &inbound, 0U, 100);

		}
	}
}


void od_handle_can_frame(uint8_t src, od_frame_t *frame) {
	UNUSED(src);
	osMessageQueuePut(in_q, frame, 0U, 100);
}


void od_push_to_out_q(od_frame_t *frame) {
	osMessageQueuePut(out_q, frame, 0U, 100);
}

void od_pop_from_in_q(od_frame_t *frame) {
	osMessageQueueGet(in_q, frame, NULL, osWaitForever);
}

void od_push_to_in_q(od_frame_t *frame) {
	osMessageQueuePut(in_q, frame, 0U, 100);
}

void od_pop_from_out_q(od_frame_t *frame) {
	osMessageQueueGet(out_q, frame, NULL, osWaitForever);
}

/**
 * Read/write interface
 */
static void od_unsafe_read(uint8_t data_id, uint8_t *dst) {
    int32_t lock = osKernelLock();

    od_entry_t entry = od_entries[data_id];
    memcpy(dst, entry.data, entry.size);

    osKernelRestoreLock(lock);
}

static void od_unsafe_write(uint8_t data_id, uint8_t *src) {
    od_frame_t to_send;
    to_send.data_id = od_entries[data_id].data_id;
    to_send.size = od_entries[data_id].size;
    memcpy(to_send.data, src, to_send.size);

    osMessageQueuePut(out_q, &to_send, 0U, 100);
}

/**
 * Task definition
 */
void od_update_task(__attribute__((unused)) void *argument) {
    while(1) {
        // Get latest incoming frame (blocking)
        od_frame_t to_receive;

#if DEBUG_NO_CAN
        #warning "[DEBUG]: CAN turned off!"
        osMessageQueueGet(out_q, &to_receive, NULL, osWaitForever);
#else
        osMessageQueueGet(in_q, &to_receive, NULL, osWaitForever);
#endif


        debug_log("added to OD: %d\n", to_receive.data_id);


        // Update field atomically
        int32_t lock = osKernelLock();

        od_entry_t entry = od_entries[to_receive.data_id];
        memcpy(entry.data, to_receive.data, entry.size);

        osKernelRestoreLock(lock);
    }
}




void od_broadcast_task(__attribute__((unused)) void *argument) {

	od_hostcom_comunicator = hostcom_get_sync_comunicator();

	while(1) {
		od_frame_t to_send;

		od_pop_from_out_q(&to_send);
		//also store the data locally
		od_push_to_in_q(&to_send);

		debug_log("sending_frame: %d\n", to_send.data_id);

		//TODO: comunicator data could be generated only once!!

		comunicator_send(od_hostcom_comunicator, to_send.data_id, to_send.size, to_send.data);
		comunicator_send(&od_can_comunicator, to_send.data_id, to_send.size, to_send.data);

		debug_log("frame sent!: %d\n", to_send.data_id);


	}
}

/* END */
