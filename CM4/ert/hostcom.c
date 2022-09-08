/*  Title		: hostcom
 *  Filename	: hostcom.c
 *	Author		: iacopo sprenger
 *	Date		: 20.01.2021
 *	Version		: 0.1
 *	Description	: communication channels with the host processor
 */

/**********************
 *	INCLUDES
 **********************/

#include <cmsis_os.h>
#include <hostcom.h>
#include <protocol/structures.h>
#include <semphr.h>
#include <device/device.h>
#include <driver/hostproc.h>

/**********************
 *	CONSTANTS
 **********************/


#define HOSTCOM_SYNC_HEART_BEAT 200


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/


SemaphoreHandle_t hostcom_data_rdy = NULL;
StaticSemaphore_t hostcom_data_rdy_buffer;





/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/


/**
 * One thread for OD sync --> in the OD
 * One thread for data send -->
 * One thread for cmd
 */



void hostcom_data_acc_send(uint32_t timestamp, int32_t acc) {
	device_interface_t * data_if = hostproc_get_data_interface();
	static transfer_data_t hostcom_data;
	hostcom_data.data = acc;
	hostcom_data.time = timestamp;
	hostcom_data.type = TRANSFER_DATA_ACC;
	device_interface_send(data_if, (uint8_t *)&hostcom_data, sizeof(hostcom_data));
}

void hostcom_data_baro_send(uint32_t timestamp, int32_t pres) {
	device_interface_t * data_if = hostproc_get_data_interface();
	static transfer_data_t hostcom_data;
	hostcom_data.data = pres;
	hostcom_data.time = timestamp;
	hostcom_data.type = TRANSFER_DATA_BARO;
	device_interface_send(data_if, (uint8_t *)&hostcom_data, sizeof(hostcom_data));
}

void hostcom_data_gnss_send(uint32_t timestamp, int32_t alt) {
	device_interface_t * data_if = hostproc_get_data_interface();
	static transfer_data_t hostcom_data;
	hostcom_data.data = alt;
	hostcom_data.time = timestamp;
	hostcom_data.type = TRANSFER_DATA_GNSS;
	device_interface_send(data_if, (uint8_t *)&hostcom_data, sizeof(hostcom_data));
}




void hostcom_sync_thread(void * arg) {
    static TickType_t last_wake_time;
    static const TickType_t period = pdMS_TO_TICKS(HOSTCOM_SYNC_HEART_BEAT);
    last_wake_time = xTaskGetTickCount();


    for(;;) {

        vTaskDelayUntil( &last_wake_time, period );
    }
}








/* END */
