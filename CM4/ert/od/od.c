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

#include <string.h>
#include <assert.h>

/**********************
 *	CONSTANTS
 **********************/

#define OD_MSGQ_SIZE (16)
#define DEBUG_NO_CAN 0      // set to 1 to turn off CAN to test OD functionality only

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
ALLOCATE_OD_ENTRY(GNSS, 12, gnss_data_t);
ALLOCATE_OD_ENTRY(BATTERY_A, 13, uint32_t);
ALLOCATE_OD_ENTRY(BATTERY_B, 14, uint32_t);


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
	LINK_OD_ENTRY(GNSS),
	LINK_OD_ENTRY(BATTERY_A),
	LINK_OD_ENTRY(BATTERY_B)
};

/**
 * Synchronization primitives
 */
osMessageQueueId_t out_q;
osMessageQueueId_t in_q;

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

    osMessageQueuePut(out_q, &to_send, 0U, osWaitForever);
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


        // Update field atomically
        int32_t lock = osKernelLock();

        od_entry_t entry = od_entries[to_receive.data_id];
        memcpy(entry.data, to_receive.data, entry.size);

        osKernelRestoreLock(lock);
    }
}

/* END */
