/*  Title       : Wildhorn
 *  Filename    : wildhorn.h
 *  Author      : iacopo sprenger
 *  Date        : 22.04.2022
 *  Version     : 0.1
 *  Description : wildhorn rocket parameters
 */

#ifndef WILDHORN_H
#define WILDHORN_H

/*MUST BE CHANGE IN 2023*/

/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>

/**********************
 *  CONSTANTS
 **********************/


//present sockets

#define ND_TRUE		1
#define ND_FALSE 	0
#define ND_A			0
#define ND_B			1


/**
 * COMPUTER TYPE HERE !!
 */
#define ND_COMPUTER	ND_B

#if ND_COMPUTER == ND_A
	#define ND_HAS_SENSORS			ND_TRUE
	#define ND_HAS_RADIO			ND_TRUE
	#define ND_HAS_FEEDBACK 		ND_FALSE
	#define ND_HAS_PROPULSION		ND_FALSE
	#define ND_HAS_KRTEK			ND_TRUE
	#define ND_HAS_PROP_SENSORS 	ND_FALSE
#else
	#define ND_HAS_SENSORS			ND_TRUE
	#define ND_HAS_RADIO			ND_TRUE
	#define ND_HAS_FEEDBACK 		ND_TRUE
	#define ND_HAS_PROPULSION		ND_TRUE
	#define ND_HAS_KRTEK			ND_FALSE
	#define ND_HAS_PROP_SENSORS 	ND_TRUE


#define ND_USE_BUZZER	ND_FALSE


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