/**
 * @file 		serial.c
 * @brief 		serial interface for device driver
 *
 * @date 		02.04.2022
 * @author 		Iacopo Sprenger
 *
 * @defgroup 	serial Serial
 * @{
 */

/**********************
 *	INCLUDES
 **********************/

#include <util.h>
#include <usart.h>
#include <main.h>

#include "serial.h"
#include <device/device.h>
#include <feedback/led.h>


/**********************
 *	CONSTANTS
 **********************/

#define S1_UART 			huart2
#define S2_UART 			huart3
#define S3_UART 			huart6

#define SERIAL_DMA_LEN 32


#define SERIAL_MAX_INTERFACES	8


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/

typedef struct serial_interface_context {
	UART_HandleTypeDef * uart;
	util_buffer_u8_t rx_buffer;
	uint8_t rx_data[SERIAL_BUFFER_LEN];
	uint32_t rx_data_len;
	uint8_t rx_fragment;
	uint8_t tx_data[SERIAL_BUFFER_LEN];
	util_error_t (*serial_handler)(device_interface_t *, void *);
	void * handler_context;
}serial_interface_context_t;



/**********************
 *	VARIABLES
 **********************/



static device_interface_t s3_interface;

__attribute__((unused))
static device_interface_t s2_interface;

static device_interface_t s1_interface;

static serial_interface_context_t s3_interface_context = {
		.uart = &S3_UART
};

__attribute__((unused))
static serial_interface_context_t s2_interface_context = {
		.uart = &S2_UART
};

static serial_interface_context_t s1_interface_context = {
		.uart = &S1_UART
};


static SemaphoreHandle_t serial_rx_sem;
static StaticSemaphore_t serial_rx_sem_buffer;

static device_interface_t * serial_interfaces[SERIAL_MAX_INTERFACES];
static uint16_t serial_interfaces_count;


/**********************
 *	PROTOTYPES
 **********************/


util_error_t serial_send(void * context, uint8_t* data, uint32_t len);

util_error_t serial_recv(void * context, uint8_t* data, uint32_t * len);

util_error_t serial_handle_data(void * if_context, void * dem_context);

util_error_t serial_interface_init(	device_interface_t * serial_if,
									serial_interface_context_t * serial_ctx);

util_error_t serial_interface_init_d(	device_interface_t * serial_if,
										serial_interface_context_t * serial_ctx);

util_error_t serial_setup_reception(serial_interface_context_t * interface_context);


/**********************
 *	DECLARATIONS
 **********************/

//Interrupt receive handler for serial
/**
 * @brief       Serial reception interrupt callback.
 * @param huart Serial UART device on which the bytes have arrived.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	for(uint8_t i = 0; i < serial_interfaces_count; i++) {
		serial_interface_context_t * serial_context = (serial_interface_context_t *) serial_interfaces[i]->context;
		if(serial_context->uart == huart) {
			util_buffer_u8_add(&serial_context->rx_buffer, serial_context->rx_fragment);

			//request next byte
			HAL_UART_Receive_IT(huart, &serial_context->rx_fragment, 1);
			xSemaphoreGiveFromISR( serial_rx_sem, &xHigherPriorityTaskWoken );

			break;
		}
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}




/**
 * @brief   Returns a pointer to the s3 serial interface.
 */
device_interface_t * serial_get_s3_interface(void)
{
	return &s3_interface;
}
/**
 * @brief   Returns a pointer to the s1 serial interface.
 */
device_interface_t * serial_get_s1_interface(void)
{
	return &s1_interface;
}

/**
 * @brief   Initialize the serial driver.
 */
util_error_t serial_init(void)
{
	util_error_t error = ER_SUCCESS;

	//initialize serial semaphore
	serial_rx_sem = xSemaphoreCreateBinaryStatic(&serial_rx_sem_buffer);

	serial_interfaces_count = 0;

	//miaou or gnss
	error |= serial_interface_init(&s1_interface, &s1_interface_context);

	//feedback or krtek
	error |= serial_interface_init(&s3_interface, &s3_interface_context);




	return error;
}

/**
 * @brief   Initialize a serial interface.
 */
util_error_t serial_interface_init(	device_interface_t * serial_if,
									serial_interface_context_t * serial_ctx) {
	if(serial_interfaces_count >= SERIAL_MAX_INTERFACES) {
		return ER_RESSOURCE_ERROR;
	}
	util_error_t error = ER_SUCCESS;
	error |= device_interface_create(	serial_if,
										(void*) serial_ctx,
										NULL,
										serial_send,
										serial_recv,
										NULL);

	serial_interfaces[serial_interfaces_count++] = serial_if;

	serial_setup_reception((serial_interface_context_t * )serial_if->context);

	return error;
}


util_error_t serial_register_handler(
		device_interface_t * serial_if,
		util_error_t (*serial_handler)(device_interface_t *, void *),
		void * handler_context ) {
	serial_interface_context_t * ctx = (serial_interface_context_t *) serial_if->context;
	ctx->handler_context = handler_context;
	ctx->serial_handler = serial_handler;
	return ER_SUCCESS;
}

/**
 * @brief   Blocking function, waiting for data to be ready.
 */
util_error_t serial_data_ready()
{
	if( xSemaphoreTake(serial_rx_sem, osWaitForever) == pdTRUE ) {
		return ER_SUCCESS;
	} else {
		return ER_TIMEOUT;
	}
}

/**
 * @brief   Setup reception on the serial interface.
 */
util_error_t serial_setup_reception(serial_interface_context_t * interface_context)
{
	//setup Interrupt reception byte per byte
	util_buffer_u8_init(&interface_context->rx_buffer, interface_context->rx_data, SERIAL_BUFFER_LEN);

	if(HAL_UART_Receive_IT(	interface_context->uart,
							&interface_context->rx_fragment,
							1) != HAL_OK) {
		return ER_RESSOURCE_ERROR;
	}
	return ER_SUCCESS;

}

util_error_t serial_send(void * context, uint8_t* data, uint32_t len)
{
	serial_interface_context_t * interface_context = (serial_interface_context_t *) context;

	if(HAL_UART_Transmit_IT(interface_context->uart, data, len) != HAL_OK) {
		return ER_RESSOURCE_ERROR;
	}

	return ER_SUCCESS;
}

util_error_t serial_recv(void * context, uint8_t * data, uint32_t * len)
{
	serial_interface_context_t * interface_context = (serial_interface_context_t *) context;
	uint16_t i = 0;
	while(!util_buffer_u8_isempty(&interface_context->rx_buffer) && i < *len) {
		data[i++] = util_buffer_u8_get(&interface_context->rx_buffer);
	}
	*len = i;
	return ER_SUCCESS;
}



void serial_thread(__attribute__((unused)) void * arg) {



	for(;;) {
		if(serial_data_ready() == ER_SUCCESS) {
			//iterate over all interfaces in deamon
			for(uint16_t i = 0; i < serial_interfaces_count; i++) {
				serial_interface_context_t * ctx = (serial_interface_context_t *) serial_interfaces[i]->context;
				if(ctx->serial_handler) {
					led_set(3);
					ctx->serial_handler(serial_interfaces[i], ctx->handler_context);
				}
			}
		}
	}
}




/** @} */
/* END */
