/**
 * @file 		serial.h
 * @brief 		serial interface for device driver
 *
 * @date 		02.04.2022
 * @author 		Iacopo Sprenger
 *
 * @ingroup 	serial
 * @{
 */

#ifndef SERIAL_H
#define SERIAL_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <device/device.h>
#include <util.h>
#include <semphr.h>

/**********************
 *  CONSTANTS
 **********************/

#define SERIAL_BUFFER_LEN 256


/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

typedef struct serial_deamon_context {
	SemaphoreHandle_t rx_sem;
	StaticSemaphore_t rx_sem_buffer;

}serial_deamon_context_t;




/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

util_error_t serial_init(void);

device_interface_t * serial_get_s3_interface(void);

device_interface_t * serial_get_s2_interface(void);

device_interface_t * serial_get_s1_interface(void);

util_error_t serial_data_ready(void * dem_ctx);

util_error_t serial_send(void * context, uint8_t* data, uint32_t len);

util_error_t serial_recv(void * context, uint8_t * data, uint32_t * len);





#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* SERIAL_H */


/** @} */
/* END */
