/*  Title		: I2C
 *  Filename	: i2c.c
 *	Author		: iacopo sprenger
 *	Date		: 22.04.2022
 *	Version		: 0.1
 *	Description	: i2c interface for the generic device driver
 */

/**********************
 *	INCLUDES
 **********************/

#include <main.h>
#include <driver/i2c.h>
#include <device/device.h>
#include <i2c.h>
#include <util.h>

#include <feedback/debug.h>

/**********************
 *	CONSTANTS
 **********************/

#define S1_I2C		hi2c1
#define S2_I2C		hi2c2
#define S3_I2C		hi2c5



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
 * @brief This is the I2C Interface associated with the sensors
 */
static device_interface_t s1_interface;
static device_interface_t s2_interface;
static device_interface_t s3_interface;

static i2c_interface_context_t s1_interface_context = {
		.i2c = &S1_I2C
};

static i2c_interface_context_t s2_interface_context = {
		.i2c = &S2_I2C
};

static i2c_interface_context_t s3_interface_context = {
		.i2c = &S3_I2C
};

static device_interface_t * i2c_interfaces[] = {
		&s1_interface,
		&s2_interface,
		&s3_interface
};

static uint32_t i2c_interfaces_count = sizeof(i2c_interfaces)/sizeof(device_interface_t *);


/**********************
 *	PROTOTYPES
 **********************/

util_error_t i2c_send(void * context, uint8_t * data, uint32_t len);
util_error_t i2c_recv(void * context, uint8_t * data, uint32_t * len);


/**********************
 *	DECLARATIONS
 **********************/

device_interface_t ** i2c_get_interfaces(void) {
	return i2c_interfaces;
}

uint32_t i2c_get_interfaces_count(void) {
	return i2c_interfaces_count;
}

/**
 * @brief This funcion disables completely the SPI for hardware saveguard.
 */
void i2c_spi_guard(void) {
	//disable spi on S2 due to broken sensor board...
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
 * @brief 	Getter for the interface associated with the sensors
 * 
 * @return	The pointer to the sensor interface object
 */
device_interface_t * i2c_get_s2_interface(void) {
	return &s2_interface;
}

device_interface_t * i2c_get_s1_interface(void) {
	return &s1_interface;
}

device_interface_t * i2c_get_s3_interface(void) {
	return &s3_interface;
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	for(uint32_t i = 0; i < i2c_interfaces_count; i++) {
		i2c_interface_context_t * if_ctx = (i2c_interface_context_t *) i2c_interfaces[i]->context;
		if(if_ctx->i2c == hi2c) {
			xSemaphoreGiveFromISR(if_ctx->sem, &xHigherPriorityTaskWoken);
			if_ctx->error = ER_SUCCESS;
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	for(uint32_t i = 0; i < i2c_interfaces_count; i++) {
		i2c_interface_context_t * if_ctx = (i2c_interface_context_t *) i2c_interfaces[i]->context;
		if(if_ctx->i2c == hi2c) {
			xSemaphoreGiveFromISR(if_ctx->sem, &xHigherPriorityTaskWoken);
			if_ctx->error = ER_SUCCESS;
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



/**
 * @brief I2C send
 *
 * @param context	The device interface context generic pointer.
 * @param data		Pointer to the data to be transmitted (first byte is the
 * 					I2C device address.)
 * @param len		length of the data array
 */
util_error_t i2c_send(void * context, uint8_t * data, uint32_t len) {
	i2c_interface_context_t * ctx = (i2c_interface_context_t *) context;
	debug_log(LOG_ERROR, "sending %x ...\n", data[0]);
	HAL_I2C_Master_Transmit_IT(ctx->i2c, data[0], &(data[1]), len-1);
	if( xSemaphoreTake(ctx->sem, I2C_TIMEOUT) == pdTRUE ) {
		debug_log(LOG_ERROR, "sent, with error code %d\n", ctx->error);
		return ctx->error;
	} else {
		debug_log(LOG_ERROR, "timeout\n");
		return ER_TIMEOUT;
	}
}

/**
 * @brief I2C recv
 *
 * @param context	The device interface context generic pointer.
 * @param data		Pointer to store the received data (first byte is the
 * 					I2C device address.)
 * @param len		length of the data array
 *
 * @note 	The received data will start at data+1 as the first byte is the I2C
 * 			device address.
 */
util_error_t i2c_recv(void * context, uint8_t * data, uint32_t * len) {
	i2c_interface_context_t * ctx = (i2c_interface_context_t *) context;
	HAL_I2C_Master_Receive_IT(ctx->i2c, data[0], &(data[1]), (*len)-1);
	if( xSemaphoreTake(ctx->sem, I2C_TIMEOUT) == pdTRUE ) {
		return ctx->error;
	} else {
		return ER_TIMEOUT;
	}
}

/**
 * @brief	Initializer for the i2c subsystem.
 * @deails	This function initializes the three i2c interfaces which are present on the hostboards.
 *
 */
void i2c_s1_init(void) {

	//transfer done semaphores
	s1_interface_context.sem = xSemaphoreCreateBinaryStatic(&s1_interface_context.sem_buffer);

	device_interface_create(&s1_interface,
							(void *) &s1_interface_context,
							NULL,
							i2c_send,
							i2c_recv,
							NULL);

}

void i2c_s2_init(void) {

	//transfer done semaphores
	s2_interface_context.sem = xSemaphoreCreateBinaryStatic(&s2_interface_context.sem_buffer);

	device_interface_create(&s2_interface,
							(void *) &s2_interface_context,
							NULL,
							i2c_send,
							i2c_recv,
							NULL);

}

void i2c_s3_init(void) {

	//transfer done semaphores
	s3_interface_context.sem = xSemaphoreCreateBinaryStatic(&s3_interface_context.sem_buffer);

	device_interface_create(&s3_interface,
							(void *) &s3_interface_context,
							NULL,
							i2c_send,
							i2c_recv,
							NULL);

}

/* END */
