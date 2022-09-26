/*  Title       : Object dictionary
 *  Filename    : od.h
 *  Author      : Oliver Facklam
 *  Date        : 28.05.2022
 *  Version     : 0.1
 *  Description : Object dictionary implementation
 */

#ifndef CAN_H
#define CAN_H



/**********************
 *  INCLUDES
 **********************/



/**********************
 *  CONSTANTS
 **********************/


#define CAN_DATAID_OFFSET 3U 	// 8-bit data ID | 3-bit board ID
#define CAN_BOARDID_MASK 0x07U


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

void can_transmit_thread(__attribute__((unused))  void *arg);
void can_receive_thread(__attribute__((unused))  void *arg);

void can_init(uint8_t board_id);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* CAN_H */

/* END */
