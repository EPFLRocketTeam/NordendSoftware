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
#include <sensor/accelerometer.h>
#include <sensor/barometer.h>
#include <sensor/gnss.h>
#include <driver/serial.h>
#include <device/device.h>
#include <feedback/led.h>
#include <feedback/debug.h>

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



void miaou_handler(uint8_t opcode, uint16_t len, uint8_t * data) {
	UNUSED(opcode);
	UNUSED(len);
	UNUSED(data);
}


void miaou_thread(__attribute__((unused)) void * arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(MIAOU_HEART_BEAT);
	last_wake_time = xTaskGetTickCount();


	//setup Miaou reception
	device_interface_t * miaou_interface = serial_get_s1_interface();

	comunicator_init(&miaou_comunicator, miaou_interface, miaou_handler);

	serial_register_handler(miaou_interface, communicator_handler, &miaou_comunicator);

	uint16_t checkpoint = led_add_checkpoint(led_orange);

	static int32_t packet_number = 0;

	for(;;) {

		led_checkpoint(checkpoint);

		packet_number += 1;

		miaou_packet.prefix = 'R'<<24 | 'F'<<16 | 'B'<<8 | 'G';

		accelerometer_data_t acc_data;
		od_read_ACC_I2C_A(&acc_data);
		miaou_packet.acc_z = acc_data.processed[ACC_X];

		barometer_data_t baro_data;
		od_read_BARO_I2C_A(&baro_data);
		miaou_packet.baro_press = baro_data.pressure;
		miaou_packet.baro_temp = baro_data.temperature;
		miaou_packet.baro_alt = baro_data.altitude;

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
//
//		comunicator_send(	&miaou_comunicator,
//							radio_packet_opcode,
//							radio_packet_size,
//							(uint8_t *) &miaou_packet);


		debug_log("mioau packet sent!\n");


		vTaskDelayUntil( &last_wake_time, period );
	}
}





/** @} */

/* END */
