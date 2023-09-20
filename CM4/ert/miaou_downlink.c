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

#include <device/comunicator.h>
#include <od/od.h>
#include <ERT_RF_Protocol_Interface/PacketDefinition.h>
#include <od/data_types.h>
#include <driver/serial.h>
#include <device/device.h>
#include <feedback/led.h>
#include <feedback/debug.h>
#include <miaou_downlink.h>
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

static comunicator_t miaou_downlink_comunicator;
static av_downlink_t miaou_packet;


/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/


void miaou_downlink_handler(uint8_t opcode, uint16_t len, uint8_t * _data) {

}


void miaou_downlink_thread(__attribute__((unused)) void * arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(MIAOU_HEART_BEAT);
	last_wake_time = xTaskGetTickCount();

	debug_log(LOG_INFO, "setup Miaou downlink\n");



	device_interface_t * miaou_interface = serial_get_s1_interface();

	comunicator_init(&miaou_downlink_comunicator, miaou_interface, miaou_downlink_handler);

	serial_register_handler(miaou_interface, communicator_handler, &miaou_downlink_comunicator);

	uint16_t checkpoint = led_add_checkpoint(led_orange);

	static int32_t packet_number = 0;

	// transission donnees

	for(;;) {

		led_checkpoint(checkpoint);
		led_rgb_set_color(led_orange);

		packet_number += 1;

		gnss_data_t gnss_data;
		od_read_GNSS_DATA_A(&gnss_data);
		miaou_packet.gnss_lat = gnss_data.latitude;
		miaou_packet.gnss_lon = gnss_data.longitude;
		miaou_packet.gnss_alt = gnss_data.altitude;

		kalman_data_t kalman_data;
		od_read_KALMAN_DATA_A(&kalman_data);
		miaou_packet.kalman_z = kalman_data.alt;
		miaou_packet.kalman_v = kalman_data.vel;

		miaou_packet.packet_nbr = packet_number;
		miaou_packet.timestamp = HAL_GetTick();

		comunicator_send(	&miaou_downlink_comunicator,
							0x65, //radio_packet_opcode,
							sizeof(av_downlink_t), //radio_packet_size,
							(uint8_t *) &miaou_packet);


		debug_log(LOG_INFO, "mioau packet sent!\n");


		vTaskDelayUntil( &last_wake_time, period );
	}
}





/** @} */

/* END */
