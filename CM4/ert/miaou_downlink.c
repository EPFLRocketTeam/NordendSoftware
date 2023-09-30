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
	 if(opcode == MIAOU_GNSS) {
		debug_log(LOG_INFO, "Received GNSS packet\n");
		if(1 || len == sizeof(av_miaou_gnss_t)) {
			av_miaou_gnss_t data;
			memcpy(&data, _data, sizeof(av_miaou_gnss_t));
			//handle gnss frame from miaou
			gnss_data_t gnss_data;

			gnss_data.altitude = data.altitude;
			gnss_data.latitude = data.latitude;
			gnss_data.longitude = data.longitude;
			gnss_data.speed = data.speed;
			gnss_data.hdop = data.hdop;
			gnss_data.time = data.time;
			od_write_GNSS_DATA_B(&gnss_data);

		}
	}
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

		//led_rgb_set_color(led_orange);

		engine_control_data_t ec_data;
		sensor_acc_data_t acc_data0, acc_data1;
		sensor_baro_data_t baro_data0, baro_data1, baro_data_r0, baro_data_r1;
		sensor_imu_data_t imu_data0, imu_data1;
		gnss_data_t gnss_data, gnss_data_r;
		sensor_eng_data_t eng_data;

		od_read_ENGINE_CONTROL_DATA(&ec_data);
		od_read_SENSOR_ACC_B_0(&acc_data0);
		od_read_SENSOR_ACC_B_1(&acc_data1);
		od_read_SENSOR_BARO_B_0(&baro_data0);
		od_read_SENSOR_BARO_B_1(&baro_data1);
		od_read_SENSOR_BARO_A_0(&baro_data_r0);
		od_read_SENSOR_BARO_A_1(&baro_data_r1);
		od_read_SENSOR_IMU_B_0(&imu_data0);
		od_read_SENSOR_IMU_B_1(&imu_data1);

		od_read_GNSS_DATA_B(&gnss_data);
		od_read_GNSS_DATA_A(&gnss_data_r);

		od_read_ENGINE_SENSORS_DATA(&eng_data);


		packet_number += 1;

		miaou_packet.prefix = ERT_PREFIX;

		miaou_packet.acc_hg_z = (acc_data0.acc[AXIS_Z] + acc_data1.acc[AXIS_Z]) / 2;
		miaou_packet.acc_z = (imu_data0.acc[AXIS_Z] + imu_data1.acc[AXIS_Z]) / 2;


		miaou_packet.baro_press = (baro_data0.press + baro_data1.press) / 2;
		miaou_packet.baro_temp = (baro_data0.temp + baro_data1.temp) / 2;

		miaou_packet.baro_press_r = (baro_data_r0.press + baro_data_r1.press) / 2;

		miaou_packet.gnss_lon = gnss_data.longitude;
		miaou_packet.gnss_lat = gnss_data.latitude;
		miaou_packet.gnss_alt = gnss_data.altitude;

		miaou_packet.gnss_lon_r = gnss_data_r.longitude;
		miaou_packet.gnss_lat_r = gnss_data_r.latitude;
		miaou_packet.gnss_alt_r = gnss_data_r.altitude;

		miaou_packet.engine_state.pressurize = ec_data.press;
		miaou_packet.engine_state.purge = ec_data.purge;
		miaou_packet.engine_state.vent_N2O = ec_data.vent_n2o;
		miaou_packet.engine_state.vent_fuel = ec_data.vent_eth;
		miaou_packet.engine_state.servo_N2O = ec_data.servo_n2o == 180 ? 1 : 0;
		miaou_packet.engine_state.servo_fuel = ec_data.servo_eth == 180 ? 1 : 0;

		miaou_packet.av_state = ec_data.state;

		miaou_packet.N2O_pressure = eng_data.press_n2o;
		miaou_packet.fuel_pressure = eng_data.press_eth;
		miaou_packet.tank_temp = eng_data.temp_tank;
		miaou_packet.chamber_pressure = eng_data.press_eng;


		miaou_packet.packet_nbr = packet_number;
		miaou_packet.timestamp = util_get_time();

		comunicator_send(	&miaou_downlink_comunicator,
							MIAOU_RF, //radio_packet_opcode,
							sizeof(av_downlink_t), //radio_packet_size,
							(uint8_t *) &miaou_packet);


		debug_log(LOG_INFO, "miaou packet sent: %d %d\n", miaou_packet.packet_nbr, miaou_packet.timestamp);

		vTaskDelayUntil( &last_wake_time, period );
	}
}





/** @} */

/* END */
