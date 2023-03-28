/**
 * @file 		miaou.c
 * @brief 		Communication with the Miaou Radio modem
 *
 * @date 		13.08.2021
 * @author 		Iacopo Sprenger
 *
 * @defgroup 	miaou Miaou
 * @{
 */

/**********************
 *	INCLUDES
 **********************/

#include <miaou.h>
#include <device/comunicator.h>
#include <od/od.h>
#include <RadioPacket/RadioPacket.h>
#include <sensor/gnss.h>
#include <driver/serial.h>
#include <device/device.h>
#include <feedback/led.h>
#include <feedback/debug.h>
#include <string.h>

/**********************
 *	CONSTANTS
 **********************/

#define MIAOU_HEART_BEAT 1200


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/

static comunicator_t miaou_comunicator;
static radio_packet_t miaou_packet;


/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/

//TODO :
//- write miaou handler (different opcodes, different boards, write in od, radio & gnss)
//- for now only one packet, will add 3/5 later

void miaou_handler(uint8_t opcode, uint16_t len, uint8_t * _data) {

	if(opcode == RF_PREFIX) {
		if(len == sizeof(PacketAV_uplink_t)) {
			PacketAV_uplink_t data;
			memcpy(&data, _data, sizeof(PacketAV_uplink_t));
			rf_cmd_t cmd = data.cmd;
			//uint8_t cmd_counter = data.cmd_counter;
			uint16_t cmd_countdown = data.cmd_countdown;

#if ND_COMPUTER == ND_A
			od_write_RF_CMD(&cmd);
			od_write_COUNTDOWN(&cmd_countdown);

#else
			od_write_RF_CMD(&cmd);
			od_write_COUNTDOWN(&cmd_countdown);
#endif

	}
}

/*
	if(opcode = TRANSFER_DATA_GNSS_RMC) {  //utliliser le gnss data type ou en créer un autre parce que le code de décodage tourne sur miaou??
		if(len == sizeof(transfer_data_gnss_rmc_t)) {
			transfer_data_gnss_rmc_t data;
			memcpy(&data, _data, sizeof(transfer_data_gnss_rmc_t));
			od_write_(&data);
	}
}
	if(opcode = TRANSFER_DATA_GNSS_OTHER) {
		if(len == sizeof()) {

	}
}
*/
}


void miaou_thread(__attribute__((unused)) void * arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(MIAOU_HEART_BEAT);
	last_wake_time = xTaskGetTickCount();

	// initialise la réception de données par serial, géré après par le thread serial

	device_interface_t * miaou_interface = serial_get_s1_interface();

	comunicator_init(&miaou_comunicator, miaou_interface, miaou_handler);

	serial_register_handler(miaou_interface, communicator_handler, &miaou_comunicator);

	uint16_t checkpoint = led_add_checkpoint(led_orange);

	static int32_t packet_number = 0;

	// transission données

	for(;;) {

		led_checkpoint(checkpoint);

		packet_number += 1;

	//	miaou_packet.prefix = 'R'<<24 | 'F'<<16 | 'B'<<8 | 'G';


		gnss_data_t gnss_data;
		od_read_GNSS(&gnss_data);
		miaou_packet.gnss_lat = gnss_data.latitude;
		miaou_packet.gnss_lon = gnss_data.longitude;
		miaou_packet.gnss_alt = gnss_data.altitude;

		transfer_data_res_t kalman_data;
		od_read_KALMAN_DATA_A(&kalman_data);
		miaou_packet.kalman_z = kalman_data.alt;
		miaou_packet.kalman_v = kalman_data.vel;

		miaou_packet.packet_nbr = packet_number;
		miaou_packet.timestamp = HAL_GetTick();

		comunicator_send(	&miaou_comunicator,
							0x00, //radio_packet_opcode,
							sizeof(radio_packet_t), //radio_packet_size,
							(uint8_t *) &miaou_packet);


		debug_log("mioau packet sent!\n");


		vTaskDelayUntil( &last_wake_time, period );
	}
}





/** @} */

/* END */
