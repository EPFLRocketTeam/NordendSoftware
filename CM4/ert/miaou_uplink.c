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

	if(opcode == MIAOU_RF) {
		if(len == sizeof(av_uplink_t)) {
			av_uplink_t data;
			memcpy(&data, _data, sizeof(av_uplink_t));
			//handle command frame from miaou

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

}







/** @} */

/* END */
