/*  Title		: miaou
 *  Filename	: miaou.c
 *	Author		: iacopo sprenger
 *	Date		: 13.08.2021
 *	Version		: 0.1
 *	Description	: Communication with the Miaou Radio modem
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

/**********************
 *	CONSTANTS
 **********************/

#define MIAOU_HEART_BEAT 500


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

}


void miaou_thread(__attribute__((unused)) void * arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(MIAOU_HEART_BEAT);
	last_wake_time = xTaskGetTickCount();

	device_interface_t * miaou_interface = serial_get_s3_interface();

	comunicator_init(&miaou_comunicator, miaou_interface, miaou_handler);



	for(;;) {

		miaou_packet.preamble = 0xAE;

		accelerometer_data_t acc_data;
		od_read_ACC_I2C_A(&acc_data);
		miaou_packet.acc_z = acc_data.processed[ACC_X];

		barometer_data_t baro_data;
		od_read_BARO_I2C_A(&baro_data);
		miaou_packet.baro_press = baro_data.pressure;
		miaou_packet.baro_temp = baro_data.temperature;

		gnss_data_t gnss_data;
		od_read_GNSS(&gnss_data);
		miaou_packet.gnss_lat = gnss_data.lat;
		miaou_packet.gnss_lon = gnss_data.lon;
		miaou_packet.gnss_alt = gnss_data.alt;

		comunicator_send(	&miaou_comunicator,
							0x00,
							sizeof(radio_packet_t),
							(uint8_t *) &miaou_packet);


		vTaskDelayUntil( &last_wake_time, period );
	}
}







/* END */
