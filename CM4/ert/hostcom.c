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


transfer_data_t hostcom_data;


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
	hostcom_data.data = acc;
	hostcom_data.time = timestamp;
	hostcom_data.type = TRANSFER_DATA_ACC;
	xSemaphoreGive(hostcom_data_rdy);
}

void hostcom_data_baro_send(uint32_t timestamp, int32_t pres) {
	hostcom_data.data = pres;
	hostcom_data.time = timestamp;
	hostcom_data.type = TRANSFER_DATA_BARO;
	xSemaphoreGive(hostcom_data_rdy);
}

void hostcom_data_gnss_send(uint32_t timestamp, int32_t alt) {
	hostcom_data.data = alt;
	hostcom_data.time = timestamp;
	hostcom_data.type = TRANSFER_DATA_GNSS;
	xSemaphoreGive(hostcom_data_rdy);
}




void hostcom_sync_thread(void * arg) {
    static TickType_t last_wake_time;
    static const TickType_t period = pdMS_TO_TICKS(HOSTCOM_SYNC_HEART_BEAT);
    last_wake_time = xTaskGetTickCount();


    for(;;) {

        vTaskDelayUntil( &last_wake_time, period );
    }
}

void hostcom_data_thread(void * arg) {

	device_interface_t * data_if = hostproc_get_data_interface();


    for(;;) {

    	//semaphore
    	xSemaphoreTake(hostcom_data_rdy, osWaitForever);
    	device_interface_send(data_if, (uint8_t *)&hostcom_data, sizeof(hostcom_data));


    }
}







/* END */
