/**
 * @file 		comunicator.c
 * @brief 		serial msv2 communicator
 *
 * @date 		12.08.2022
 * @author 		Iacopo Sprenger
 *
 * @defgroup 	comunicator Comunicator
 * @{
 */

/**********************
 *	INCLUDES
 **********************/

#include <device/comunicator.h>
#include <driver/serial.h>
#include <init.h>


/**********************
 *	CONSTANTS
 **********************/


/**********************
 *	MACROS
 **********************/


#define COMUNICATOR_MAX	16


/**********************
 *	TYPEDEFS
 **********************/




/**********************
 *	VARIABLES
 **********************/

static comunicator_t * comunicators[COMUNICATOR_MAX];

static uint16_t comunicator_count = 0;



/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/


/**
 * @brief 	initialize communicator device
 * @detail
 */
util_error_t comunicator_init(	comunicator_t * com,
								device_interface_t * channel,
								void (*cb)(uint8_t, uint16_t, uint8_t *)) {
	if(comunicator_count >= COMUNICATOR_MAX) {
		return ER_OUT_OF_RANGE;
	}
	msv2_init(&com->msv2);
	com->cb = cb;
	com->interface = channel;
	device_interface_register_handle_data(channel, comunicator_handle_data);
	comunicators[comunicator_count++] = com;
	return ER_SUCCESS;
}

/**
 * @brief 	initialize communicator device
 * @detail
 */
util_error_t comunicator_init_lone(	comunicator_t * com,
									device_interface_t * channel,
									void (*cb)(uint8_t, uint16_t, uint8_t *)) {
	msv2_init(&com->msv2);
	com->cb = cb;
	com->interface = channel;
	return ER_SUCCESS;
}

util_error_t comunicator_recv(comunicator_t * com) {
	uint32_t len = 1;
	util_error_t error;
	for(;;) {
		uint8_t data;
		len  = 1;
		error = device_interface_recv(com->interface, &data, &len);
		if(error) {
			return error;
		}
		if(len == 1) {
			MSV2_ERROR_t ret = msv2_decode_fragment(&com->msv2, data);
			if(ret == MSV2_SUCCESS) {
				com->cb(com->msv2.rx.opcode, com->msv2.rx.data_len*2, com->msv2.rx.data);
			}
		} else {
			return ER_SUCCESS;
		}
	}
}

util_error_t comunicator_send(	comunicator_t * com,
								uint8_t opcode,
								uint16_t len,
								uint8_t * data) {

	uint16_t bin_len = msv2_create_frame(&com->msv2, opcode, len/2, data);
	util_error_t error = device_interface_send(	com->interface,
												com->msv2.tx.data,
												bin_len);
	return error;
}


util_error_t comunicator_handle_data(void* if_ctx, void* dem_ctx) {
	for(uint16_t i = 0; i < comunicator_count; i++) {
		if(comunicators[i]->interface->context == if_ctx) {
			return comunicator_recv(comunicators[i]);
		}
	}
}

/**
 * @brief 	comunicator data handling thread
 * @brief	For now only works with serial -> interrupts data rdy comes
 * 			from the serial driver.
 * 			only one thread -> later maybe one thread per comunicator!
 */

/*
void comunicator_thread(__attribute__((unused)) void * arg) {

	for(;;) {
		if(serial_data_ready() == ER_SUCCESS) {
			//iterate over all interfaces in deamon
			for(uint16_t i = 0; i < comunicator_count; i++) {
				comunicator_recv(comunicators[i]);
			}
		}
	}
}
*/



/** @} */


/* END */
