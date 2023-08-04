/*  Title       : Wildhorn
 *  Filename    : wildhorn.h
 *  Author      : iacopo sprenger
 *  Date        : 22.04.2022
 *  Version     : 0.1
 *  Description : nordend rocket parameters
 */

#ifndef NORDEND_H
#define NORDEND_H

/*MUST BE CHANGE IN 2023*/

/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>

/**********************
 *  CONSTANTS
 **********************/


//present sockets

#define ND_TRUE		    1
#define ND_FALSE 	    0
#define ND_A			0
#define ND_B			1


/**
 * COMPUTER TYPE HERE !!
 */
#define ND_COMPUTER	ND_B

#if ND_COMPUTER == ND_A
/* S1 */
	#define ND_HAS_UPLINK			ND_TRUE
	#define ND_HAS_DOWNLINK			ND_FALSE
/* S2 */
	#define ND_HAS_SENSORS			ND_TRUE
/* S3 */
	#define ND_HAS_PROPULSION		ND_TRUE
	#define ND_HAS_RECOVERY			ND_FALSE
#else
/* S1 */
	#define ND_HAS_UPLINK			ND_FALSE
	#define ND_HAS_DOWNLINK			ND_TRUE
/* S2 */
	#define ND_HAS_SENSORS			ND_TRUE
/* S3 */
	#define ND_HAS_PROPULSION		ND_FALSE
	#define ND_HAS_RECOVERY			ND_TRUE
#endif


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

#endif /* NORDEND_H */

/* END */
