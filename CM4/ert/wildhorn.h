/*  Title       : Wildhorn
 *  Filename    : wildhorn.h
 *  Author      : iacopo sprenger
 *  Date        : 22.04.2022
 *  Version     : 0.1
 *  Description : wildhorn rocket parameters
 */

#ifndef WILDHORN_H
#define WILDHORN_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>

/**********************
 *  CONSTANTS
 **********************/


//present sockets

#define WH_TRUE		1
#define WH_FALSE 	0
#define WH_A			0
#define WH_B			1


/**
 * COMPUTER TYPE HERE !!
 */
#define WH_COMPUTER	WH_B

#if WH_COMPUTER == WH_A
	#define WH_HAS_SENSORS	WH_TRUE
	#define WH_HAS_FEEDBACK WH_FALSE
	#define WH_HAS_RADIO	WH_TRUE
	#define WH_HAS_GNSS		WH_FALSE
	#define WH_HAS_KRTEK	WH_TRUE
#else
	#define WH_HAS_SENSORS	WH_TRUE
	#define WH_HAS_FEEDBACK WH_TRUE
	#define WH_HAS_RADIO	WH_FALSE
	#define WH_HAS_GNSS		WH_TRUE
	#define WH_HAS_KRTEK	WH_FALSE
#endif


#define WH_USE_BUZZER	WH_FALSE


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


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* WILDHORN_H */

/* END */
