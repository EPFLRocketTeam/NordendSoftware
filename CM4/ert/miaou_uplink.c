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
#include <sensor/gnss.h>
#include <driver/serial.h>
#include <device/device.h>
#include <feedback/led.h>
#include <feedback/debug.h>
#include <miaou_uplink.h>
#include <engine_control.h>
#include <string.h>

/**********************
 *	CONSTANTS
 **********************/



/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/

static comunicator_t miaou_uplink_comunicator;


/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/

//TODO :
//- write miaou handler (different opcodes, different boards, write in od, radio & gnss)
//- for now only one packet, will add 3/5 later

void miaou_uplink_handler(uint8_t opcode, uint16_t len, uint8_t * _data) {

	debug_log(LOG_INFO, "Handling miaou packet op=%u, l=%u\n", opcode, len);
	if(opcode == MIAOU_RF) {
		if(len == sizeof(av_uplink_t)) {
			av_uplink_t data;
			memcpy(&data, _data, sizeof(av_uplink_t));
			debug_log(LOG_INFO, "received: id=%u, v=%u\n", data.order_id, data.order_value);
			//handle command frame from miaou


			switch(data.order_id) {
			case AV_CMD_SERVO_N2O:
				if(data.order_value == ACTIVE) {
					engine_control_command_push(COMMAND_VALVE_N2O, 180);
				} else if(data.order_value == INACTIVE){
					engine_control_command_push(COMMAND_VALVE_N2O, 0);
				}
				break;
			case AV_CMD_SERVO_FUEL:
				if(data.order_value == ACTIVE) {
					engine_control_command_push(COMMAND_VALVE_ETH, 180);
				} else if(data.order_value == INACTIVE){
					engine_control_command_push(COMMAND_VALVE_ETH, 0);
				}
				break;
			case AV_CMD_VENT_N2O:
				if(data.order_value == ACTIVE) {
					engine_control_command_push(COMMAND_VENT_N2O, 1);
				} else if(data.order_value == INACTIVE){
					engine_control_command_push(COMMAND_VENT_N2O, 0);
				}
				break;
			case AV_CMD_VENT_FUEL:
				if(data.order_value == ACTIVE) {
					engine_control_command_push(COMMAND_VENT_ETH, 1);
				} else if(data.order_value == INACTIVE){
					engine_control_command_push(COMMAND_VENT_ETH, 0);
				}
				break;
			case AV_CMD_ARM:
				if(data.order_value == ACTIVE) {
					engine_control_command_push(COMMAND_ARM, 0);
				} else if(data.order_value == INACTIVE){
					engine_control_command_push(COMMAND_DISARM, 0);
				}
				break;
			case AV_CMD_PRESSURIZE:
				engine_control_command_push(COMMAND_PRESSURE, 0);
				break;
			case AV_CMD_ABORT:
				engine_control_command_push(COMMAND_ABORT, 0);
				break;
			case AV_CMD_IGNITION:
				engine_control_command_push(COMMAND_IGNITE, 0);
				break;
			case AV_CMD_RECOVER:
				engine_control_command_push(COMMAND_RECOVER, 0);
				break;
			default:
				break;
			}


		}
	} else if(opcode == MIAOU_GNSS) {
		if(len == sizeof(av_miaou_gnss_t)) {
			av_miaou_gnss_t data;
			memcpy(&data, _data, sizeof(av_miaou_gnss_t));
			//handle gnss frame from miaou
		}
	}
}


void miaou_uplink_init(void) {
	//init communicator
	device_interface_t * miaou_interface = serial_get_s1_interface();

	comunicator_init(&miaou_uplink_comunicator, miaou_interface, miaou_uplink_handler);

	serial_register_handler(miaou_interface, communicator_handler, &miaou_uplink_comunicator);

}







/** @} */

/* END */
