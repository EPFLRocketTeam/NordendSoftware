/*  Title		: i2c_sensor
 *  Filename	: i2c_sensor.c
 *	Author		: iacopo sprenger
 *	Date		: 22.04.2022
 *	Version		: 0.1
 *	Description	: device driver for i2c sensors
 *
 *	@note  	We use semaphores to create a "blocking function" which does not
 *			slow doen the RTOS with useless polling.
 */

/**********************
 *	INCLUDES
 **********************/
#include <main.h>
#include <device/device.h>
#include <device/i2c_sensor.h>
#include <driver/i2c.h>
#include <util.h>
#include <string.h>
#include <abstraction/gpio.h>
/**********************
 *	CONSTANTS
 **********************/

#define TEMP_DATA_LEN	32



/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/




/**********************
 *	VARIABLES
 **********************/

static device_t bmi088_acc_device0;
static device_t bmi088_gyr_device0;

static device_t bmi088_acc_device1;
static device_t bmi088_gyr_device1;

static device_t bmp390_baro_device0;
static device_t bmp390_baro_device1;

static device_t adxl375_acc_device0;
static device_t adxl375_acc_device1;


static i2c_sensor_context_t bmi088_acc_context0 = {
		.device_address = 0x18
};

static i2c_sensor_context_t bmi088_gyr_context0 = {
		.device_address = 0x68
};

static i2c_sensor_context_t bmi088_acc_context1 = {
		.device_address = 0x19
};

static i2c_sensor_context_t bmi088_gyr_context1 = {
		.device_address = 0x69
};

static i2c_sensor_context_t bmp390_baro_context0 = {
		.device_address = 0x76
};

static i2c_sensor_context_t bmp390_baro_context1 = {
		.device_address = 0x77
};

static i2c_sensor_context_t adxl375_acc_context0 = {
		.device_address = 0x1d
};

static i2c_sensor_context_t adxl375_acc_context1 = {
		.device_address = 0x53
};



static device_t mcp3426_adc_device_up;
static device_t mcp3426_adc_device_dn;
static device_t mcp3426_adc_device_st;

static i2c_sensor_context_t mcp3426_adc_context_up = {
		.device_address = 0x69
};

static i2c_sensor_context_t mcp3426_adc_context_dn = {
		.device_address = 0x68
};

static i2c_sensor_context_t mcp3426_adc_context_st = {
		.device_address = 0xff
};

/**********************
 *	PROTOTYPES
 **********************/

util_error_t i2c_sensor_read_reg(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len);
util_error_t i2c_sensor_write_reg(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len);

util_error_t i2c_sensor_read_reg_HAL(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len);
util_error_t i2c_sensor_write_reg_HAL(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len);

/**********************
 *	DECLARATIONS
 **********************/


util_error_t i2c_engine_sensor_init(void) {
	device_interface_t * i2c_engine_sensor_interface = i2c_get_s3_interface();

	device_create(&mcp3426_adc_device_up, &mcp3426_adc_context_up, i2c_engine_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create(&mcp3426_adc_device_dn, &mcp3426_adc_context_dn, i2c_engine_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create(&mcp3426_adc_device_st, &mcp3426_adc_context_st, i2c_engine_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);

	gpio_config_t conf = {0};
	conf.bias = GPIO_BIAS_HIGH;
	conf.drive = GPIO_DRIVE_PP;
	conf.mode = GPIO_MODE_OUT;
	conf.speed = 0;
	gpio_cfg(GPIOA, GPIO_PIN_2, conf);

	//set the i2c-can to master mode
	gpio_set(GPIOA, GPIO_PIN_2);

	return ER_SUCCESS;
}


util_error_t i2c_sensor_init(void) {

	device_interface_t * i2c_sensor_interface = i2c_get_s2_interface();

	device_create(&bmi088_acc_device0, &bmi088_acc_context0, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create(&bmi088_gyr_device0, &bmi088_gyr_context0, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create(&bmi088_acc_device1, &bmi088_acc_context1, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create(&bmi088_gyr_device1, &bmi088_gyr_context1, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create(&bmp390_baro_device0, &bmp390_baro_context0, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create(&bmp390_baro_device1, &bmp390_baro_context1, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create(&adxl375_acc_device0, &adxl375_acc_context0, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create(&adxl375_acc_device1, &adxl375_acc_context1, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);

	return ER_SUCCESS;

}

device_t * i2c_engine_sensor_get_mcp3426_up(void) {
	return &mcp3426_adc_device_up;
}

device_t * i2c_engine_sensor_get_mcp3426_dn(void) {
	return &mcp3426_adc_device_dn;
}


device_t * i2c_sensor_get_bmi088_acc(uint8_t num) {
	if(num == 0) {
		return &bmi088_acc_device0;
	} else if(num == 1) {
		return &bmi088_acc_device1;
	}
	return NULL;
}

device_t * i2c_sensor_get_bmi088_gyr(uint8_t num) {
	if(num == 0) {
		return &bmi088_gyr_device0;
	} else if(num == 1) {
		return &bmi088_gyr_device1;
	}
	return NULL;
}

device_t * i2c_sensor_get_bmp390_baro(uint8_t num) {
	if(num == 0) {
		return &bmp390_baro_device0;
	} else if(num == 1) {
		return &bmp390_baro_device1;
	}
	return NULL;
}

device_t * i2c_sensor_get_adxl375_acc(uint8_t num) {
	if(num == 0) {
		return &adxl375_acc_device0;
	} else if(num == 1) {
		return &adxl375_acc_device1;
	}
	return NULL;
}



void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	device_interface_t ** i2c_interfaces = i2c_get_interfaces();
	for(uint32_t i = 0; i < i2c_get_interfaces_count(); i++) {
		i2c_interface_context_t * if_ctx = (i2c_interface_context_t *) i2c_interfaces[i]->context;
		if(if_ctx->i2c == hi2c) {
			xSemaphoreGiveFromISR(if_ctx->sem, &xHigherPriorityTaskWoken);
			if_ctx->error = ER_SUCCESS;
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	device_interface_t ** i2c_interfaces = i2c_get_interfaces();
	for(uint32_t i = 0; i < i2c_get_interfaces_count(); i++) {
		i2c_interface_context_t * if_ctx = (i2c_interface_context_t *) i2c_interfaces[i]->context;
		if(if_ctx->i2c == hi2c) {
			xSemaphoreGiveFromISR(if_ctx->sem, &xHigherPriorityTaskWoken);
			if_ctx->error = ER_SUCCESS;
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	device_interface_t ** i2c_interfaces = i2c_get_interfaces();
	for(uint32_t i = 0; i < i2c_get_interfaces_count(); i++) {
		i2c_interface_context_t * if_ctx = (i2c_interface_context_t *) i2c_interfaces[i]->context;
		if(if_ctx->i2c == hi2c) {
			xSemaphoreGiveFromISR(if_ctx->sem, &xHigherPriorityTaskWoken);
			if_ctx->error = ER_RESSOURCE_ERROR;
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Function used to receive register data from a sensor
 *
 * @param context	Pointer to a device context
 * @param interface	Pointer to the interface associated with the device
 * @param addr		Register address
 * @param data		Data to read in the register
 * @param len		Length of the data to be read
 *
 */
util_error_t i2c_sensor_read_reg(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len) {
	i2c_sensor_context_t * ctx = (i2c_sensor_context_t *) context;
	util_error_t error = ER_SUCCESS;
	uint8_t i2c_addr = ctx->device_address;
	uint8_t temp_data[TEMP_DATA_LEN] = {0};
	temp_data[0] = i2c_addr;
	len += 1;
	temp_data[1] = addr & 0xff; //register address
	error |= interface->send(interface->context, temp_data, len);
	error |= interface->recv(interface->context, temp_data, &len);
	memcpy(data, temp_data+1, len-1); //copy only the received data (w/o dev addr)
	return error;
}

/**
 * @brief Function used to send register data to a sensor
 *
 * @param context	Pointer to a device context
 * @param interface	Pointer to the interface associated with the device
 * @param addr		Register address
 * @param data		Data to write in the register
 * @param len		Length of the data to be written
 *
 */
util_error_t i2c_sensor_write_reg(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len) {
	i2c_sensor_context_t * ctx = (i2c_sensor_context_t *) context;
	util_error_t error = ER_SUCCESS;
	uint8_t i2c_addr = ctx->device_address;
	uint8_t temp_data[TEMP_DATA_LEN] = {0};
	temp_data[0] = i2c_addr;
	len += 1;
	temp_data[1] = addr & 0xff; //register address
	error |= interface->send(interface->context, temp_data, len);
	memcpy(temp_data+1, data, len-1); //copy data to temp array
	error |= interface->send(interface->context, temp_data, len);
	return error;
}

/**
 * @brief Function used to receive register data from a sensor using direct HAL
 *
 * @param context	Pointer to a device context
 * @param interface	Pointer to the interface associated with the device
 * @param addr		Register address
 * @param data		Data to read in the register
 * @param len		Length of the data to be read
 *
 */
util_error_t i2c_sensor_read_reg_HAL(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len) {
	i2c_sensor_context_t * ctx = (i2c_sensor_context_t *) context;
	i2c_interface_context_t * if_ctx = (i2c_interface_context_t *) interface->context;
	HAL_I2C_Mem_Read_IT(if_ctx->i2c, (ctx->device_address << 1), addr, sizeof(uint8_t), data, len);
	if( xSemaphoreTake(if_ctx->sem, I2C_TIMEOUT) == pdTRUE ) {
		return if_ctx->error;
	} else {
		return ER_TIMEOUT;
	}
}

/**
 * @brief Function used to send register data to a sensor using direct HAL
 *
 * @param context	Pointer to a device context
 * @param interface	Pointer to the interface associated with the device
 * @param addr		Register address
 * @param data		Data to write in the register
 * @param len		Length of the data to be written
 *
 */
util_error_t i2c_sensor_write_reg_HAL(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len) {
	i2c_sensor_context_t * ctx = (i2c_sensor_context_t *) context;
	i2c_interface_context_t * if_ctx = (i2c_interface_context_t *) interface->context;
	HAL_I2C_Mem_Write_IT(if_ctx->i2c, (ctx->device_address << 1), addr, sizeof(uint8_t), data, len);
	if( xSemaphoreTake(if_ctx->sem, I2C_TIMEOUT) == pdTRUE ) {
		return if_ctx->error;
	} else {
		return ER_TIMEOUT;
	}
}



/* END */
