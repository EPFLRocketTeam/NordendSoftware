/*  Title		: SPI
 *  Filename	: spi.c
 *	Author		: iacopo sprenger
 *	Date		: 22.04.2022
 *	Version		: 0.1
 *	Description	: spi interface for the generic device driver
 */

/**********************
 *	INCLUDES
 **********************/

#include <main.h>
#include <driver/spi.h>
#include <device/device.h>
#include <spi.h>
#include <util.h>

/**********************
 *	CONSTANTS
 **********************/

#define S1_SPI		hspi1
#define S2_SPI		hspi2
#define S3_SPI		hspi3


#define TEMP_DATA_LEN 32

/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/



/**********************
 *	VARIABLES
 **********************/

/**
 * @brief This is the SPI Interface associated with the sensors
 */
static device_interface_t sensor_interface;

static spi_interface_context_t sensor_interface_context = {
		.spi = &S2_SPI

};

static device_interface_t * spi_interfaces[] = {
		&sensor_interface
};

static uint32_t spi_interfaces_count = sizeof(spi_interfaces)/sizeof(device_interface_t *);


/**********************
 *	PROTOTYPES
 **********************/

util_error_t spi_send(void * context, uint8_t * data, uint32_t len);
util_error_t spi_recv(void * context, uint8_t * data, uint32_t * len);


/**********************
 *	DECLARATIONS
 **********************/

device_interface_t ** spi_get_interfaces(void) {
	return spi_interfaces;
}

uint32_t spi_get_interfaces_count(void) {
	return spi_interfaces_count;
}



/**
 * @brief 	Getter for the interface associated with the sensors
 * 
 * @return	The pointer to the sensor interface object
 */
device_interface_t * spi_get_sensor_interface(void) {
	return &sensor_interface;
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	for(uint32_t i = 0; i < spi_interfaces_count; i++) {
		spi_interface_context_t * if_ctx = (spi_interface_context_t *) spi_interfaces[i]->context;
		if(if_ctx->spi == hspi) {
			xSemaphoreGiveFromISR(if_ctx->sem, &xHigherPriorityTaskWoken);
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	for(uint32_t i = 0; i < spi_interfaces_count; i++) {
		spi_interface_context_t * if_ctx = (spi_interface_context_t *) spi_interfaces[i]->context;
		if(if_ctx->spi == hspi) {
			xSemaphoreGiveFromISR(if_ctx->sem, &xHigherPriorityTaskWoken);
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


util_error_t spi_acquire_bus(device_t * dev) {

}

util_error_t spi_release_bus(device_t * dev) {

}


/**
 * @brief SPI send
 *
 * @param context	The device interface context generic pointer.
 * @param data		Pointer to the data to be transmitted.
 * @param len		length of the data array
 */
util_error_t spi_send(void * context, uint8_t * data, uint32_t len) {
	spi_interface_context_t * ctx = (spi_interface_context_t *) context;
	if(len > TEMP_DATA_LEN) {
		return ER_OUT_OF_RANGE;
	}
	HAL_SPI_Transmit_IT(ctx->spi, data, len);
	if( xSemaphoreTake(ctx->sem, 0xffff) == pdTRUE ) {
		return ER_SUCCESS;
	} else {
		return ER_TIMEOUT;
	}
}

/**
 * @brief SPI recv
 *
 * @param context	The device interface context generic pointer.
 * @param data		Pointer to store the received data.
 * @param len		length of the data array
 *
 * @note 	The received data will start at data+1 as the first byte is the I2C
 * 			device address.
 */
util_error_t spi_recv(void * context, uint8_t * data, uint32_t * len) {
	spi_interface_context_t * ctx = (spi_interface_context_t *) context;
	HAL_SPI_Receive_IT(ctx->spi, data, *len);
	if( xSemaphoreTake(ctx->sem, 0xffff) == pdTRUE ) {
		return ER_SUCCESS;
	} else {
		return ER_TIMEOUT;
	}
}

/**
 * @brief	Initializer for the spi subsystem.
 * @deails	This function initializes the three spi interfaces which are present on the hostboards.
 *
 */
void spi_init(void) {

	//transfer done semaphores
	sensor_interface_context.sem = xSemaphoreCreateBinaryStatic(&sensor_interface_context.sem_buffer);

	device_interface_create(&sensor_interface,
							(void *) &sensor_interface_context,
							NULL,
							spi_send,
							spi_recv,
							NULL);

}

/* END */
