/*  Title       : Communicator
 *  Filename    : communicator.h
 *  Author      : iacopo sprenger
 *  Date        : 12.08.2022
 *  Version     : 0.1
 *  Description : serial msv2 communicator
 */

#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <device/device.h>
#include <util.h>

/**********************
 *  CONSTANTS
 **********************/


/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/


/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif


util_error_t communicator_init(device_t * com, void (*cb)(uint8_t, uint16_t, uint8_t *));

util_error_t communicator_send(device_t * com, uint8_t opcode, uint16_t len, uint8_t * data);




#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* COMMUNICATOR_H */

/* END */
