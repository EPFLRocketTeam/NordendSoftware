/*  Title       : Communicator
 *  Filename    : communicator.h
 *  Author      : iacopo sprenger
 *  Date        : 12.08.2022
 *  Version     : 0.1
 *  Description : serial msv2 communicator
 */

#ifndef COMUNICATOR_H
#define COMUNICATOR_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <device/device.h>
#include <util.h>

#include <protocol/msv2.h>

/**********************
 *  CONSTANTS
 **********************/


/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

typedef struct comunicator {
	MSV2_INST_t msv2;
	//opcode, len, data
	void (*cb)(uint8_t, uint16_t, uint8_t *);
	device_interface_t * interface;

}comunicator_t;


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif


util_error_t comunicator_init(	comunicator_t * com,
								device_interface_t* channel,
								void (*cb)(uint8_t, uint16_t, uint8_t *));

util_error_t comunicator_send(comunicator_t * com, uint8_t opcode, uint16_t len, uint8_t * data);

void comunicator_thread(__attribute__((unused)) void * com);





#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* COMUNICATOR_H */

/* END */
