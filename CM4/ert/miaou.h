/**
 * @file 		miaou.h
 * @brief 		Communication with the Miaou Radio modem
 *
 * @date 		13.08.2021
 * @author 		Iacopo Sprenger
 *
 * @ingroup 	miaou
 * @{
 */

#ifndef MIAOU_H
#define MIAOU_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>

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



void miaou_thread(__attribute__((unused)) void * arg);
//void miaou_handler(uint8_t opcode, uint16_t len, uint8_t * _data);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* TEMPLATE_H */

/** @} */

/* END */
