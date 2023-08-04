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

#ifndef MIAOU_DOWNLINK_H
#define MIAOU_DOWNLINK_H



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

void miaou_downlink_thread(__attribute__((unused)) void * arg);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* MIAOU_DOWNLINK_H */

/** @} */

/* END */
