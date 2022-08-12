/*  Title       : Communicator
 *  Filename    : communicator.c
 *  Author      : iacopo sprenger
 *  Date        : 12.08.2022
 *  Version     : 0.1
 *  Description : serial msv2 communicator
 */

/**********************
 *	INCLUDES
 **********************/

#include <device/comunicator.h>
#include <protocol/msv2.h>

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


/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/


util_error_t communicator_init(device_t * com, void (*cb)(uint8_t, uint16_t, uint8_t *)) {

}

util_error_t communicator_send(device_t * com, uint8_t opcode, uint16_t len, uint8_t * data) {

}








/* END */
