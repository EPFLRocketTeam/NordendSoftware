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
#include <device/comunicator.h>
#include <od/od.h>
#include <wildhorn.h>

/**********************
 *	CONSTANTS
 **********************/


#define HOSTCOM_SYNC_HEART_BEAT 200
#define HOSTCOM_DATA_HEART_BEAT 500


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/



static comunicator_t data_com;
static comunicator_t sync_com;


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
	static transfer_data_t hostcom_data;
	hostcom_data.data = acc;
	hostcom_data.time = timestamp;
	comunicator_send(&data_com, TRANSFER_DATA_ACC, sizeof(hostcom_data), (uint8_t *)&hostcom_data);
}

void hostcom_data_baro_send(uint32_t timestamp, int32_t pres) {
	static transfer_data_t hostcom_data;
	hostcom_data.data = pres;
	hostcom_data.time = timestamp;
	comunicator_send(&data_com, TRANSFER_DATA_BARO, sizeof(hostcom_data), (uint8_t *)&hostcom_data);
}

void hostcom_data_gnss_send(uint32_t timestamp, int32_t alt) {
	static transfer_data_t hostcom_data;
	hostcom_data.data = alt;
	hostcom_data.time = timestamp;
	comunicator_send(&data_com, TRANSFER_DATA_GNSS, sizeof(hostcom_data), (uint8_t *)&hostcom_data);
}


void hostcom_handle_data(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(opcode == TRANSFER_DATA_RES) {
		if(len == sizeof(transfer_data_res_t)) {
			transfer_data_res_t data;
			memcpy(&data, _data, sizeof(transfer_data_res_t));
#if WH_COMPUTER == WH_A
			od_write_KALMAN_DATA_A(&data);
#else
			od_write_KALMAN_DATA_A(&data);
#endif
		}
	}
	//handle kalman inbound data!!
}



void hostcom_handle_sync(uint8_t opcode, uint16_t len, uint8_t * data) {
	UNUSED(opcode);
	UNUSED(len);
	UNUSED(data);
	//handle sync inbound data!!
}


comunicator_t * hostcom_get_sync_comunicator() {
	return &sync_com;
}



/**
 * @brief 	Initialize hostcom comunicator
 * @details	This handles the MSV2 sending and reception of packets
 * 			with the main processor for data and sync channels.
 *
 * @note	MSV2 is used to ensure synchronization of the data frames between
 * 			processors
 */
void hostcom_thread(__attribute__((unused)) void * arg) {
    static TickType_t last_wake_time;
    static const TickType_t period = pdMS_TO_TICKS(HOSTCOM_DATA_HEART_BEAT);



	comunicator_init_lone(&data_com, hostproc_get_data_interface(), hostcom_handle_data);
	comunicator_init_lone(&sync_com, hostproc_get_sync_interface(), od_sync_handler);


	for(;;) {

		comunicator_recv(&data_com);
		comunicator_recv(&sync_com);

		//send and receive sync frames

		vTaskDelayUntil( &last_wake_time, period );
	}


}






/* END */
