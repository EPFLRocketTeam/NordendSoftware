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
	UNUSED(opcode);
	UNUSED(len);
	UNUSED(_data);
}


void miaou_downlink_thread(__attribute__((unused)) void * arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(MIAOU_HEART_BEAT);
	last_wake_time = xTaskGetTickCount();

	debug_log(LOG_INFO, "setup Miaou downlink\n");



	device_interface_t * miaou_interface = serial_get_s1_interface();

	comunicator_init(&miaou_downlink_comunicator, miaou_interface, miaou_downlink_handler);

	serial_register_handler(miaou_interface, communicator_handler, &miaou_downlink_comunicator);

	//uint16_t checkpoint = led_add_checkpoint(led_orange);

	static uint32_t packet_number = 0;

	// transission donnees

	for(;;) {

		led_rgb_set_color(led_orange);


		engine_control_data_t ec_data;
		sensor_acc_data_t acc_data0, acc_data1;
		sensor_baro_data_t baro_data0, baro_data1;
		sensor_imu_data_t imu_data0, imu_data1;

		od_read_ENGINE_CONTROL_DATA(&ec_data);
		od_read_SENSOR_ACC_B_0(&acc_data0);
		od_read_SENSOR_ACC_B_1(&acc_data1);
		od_read_SENSOR_BARO_B_0(&baro_data0);
		od_read_SENSOR_BARO_B_1(&baro_data1);
		od_read_SENSOR_IMU_B_0(&imu_data0);
		od_read_SENSOR_IMU_B_1(&imu_data1);


		packet_number += 1;

		miaou_packet.acc_hg_z = (acc_data0.acc[AXIS_Z] + acc_data1.acc[AXIS_Z]) / 2;
		miaou_packet.acc_z = (imu_data0.acc[AXIS_Z] + imu_data1.acc[AXIS_Z]) / 2;


		miaou_packet.baro_press = (baro_data0.pressure + baro_data1.pressure) / 2;
		miaou_packet.baro_temp = (baro_data0.temperature + baro_data1.temperature) / 2;



		miaou_packet.engine_state.pressurize = ec_data.press;
		miaou_packet.engine_state.purge = ec_data.purge;
		miaou_packet.engine_state.vent_N2O = ec_data.vent_n2o;
		miaou_packet.engine_state.vent_fuel = ec_data.vent_eth;
		miaou_packet.engine_state.servo_N2O = ec_data.servo_n2o == 180 ? 1 : 0;
		miaou_packet.engine_state.servo_fuel = ec_data.servo_eth == 180 ? 1 : 0;

		miaou_packet.av_state = ec_data.state;


		miaou_packet.packet_nbr = packet_number;
		miaou_packet.timestamp = util_get_time();

		comunicator_send(	&miaou_downlink_comunicator,
							0x65, //radio_packet_opcode,
							sizeof(av_downlink_t), //radio_packet_size,
							(uint8_t *) &miaou_packet);


		debug_log(LOG_INFO, "miaou packet sent: %d %d\n", miaou_packet.packet_nbr, miaou_packet.timestamp);


		vTaskDelayUntil( &last_wake_time, period );
	}
}





/** @} */

/* END */
