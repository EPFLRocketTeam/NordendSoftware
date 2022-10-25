/*  Title       : Template
 *  Filename    : template.h
 *  Author      : iacopo sprenger
 *  Date        : 20.01.2021
 *  Version     : 0.1
 *  Description : template for code files
 */

#ifndef HOSTCOM_H
#define HOSTCOM_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <device/comunicator.h>

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


void hostcom_data_acc_send(uint32_t timestamp, int32_t acc);

void hostcom_data_baro_send(uint32_t timestamp, int32_t pres);

void hostcom_data_gnss_send(uint32_t timestamp, int32_t alt);

comunicator_t * hostcom_get_sync_comunicator();


void hostcom_thread(void * arg);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* HOSTCOM_H */

/* END */
