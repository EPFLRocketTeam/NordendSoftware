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

static device_t i2c_accelerometer_device;
//static device_t i2c_gyroscope_device;
static device_t i2c_barometer_device;
static device_t i2c_ADC_device;
static device_t i2c_magnetometer_device;

static i2c_sensor_context_t i2c_accelerometer_device_context = {
		.device_address = 0x30
};

// static i2c_sensor_context_t i2c_gyroscope_device_context = {
// 		.device_address = 0xD0
// };

static i2c_sensor_context_t i2c_magnetometer_device_context = {
		.device_address = 0xD0
};

static i2c_sensor_context_t i2c_barometer_device_context = {
		.device_address = 0xee
};

static i2c_sensor_context_t i2c_ADC_device_context = {
		.device_address = 0xD0 //TODO see which address this truly is based on sensor board config
};

// Propulsion sensors
static device_t i2c_engine_pressure_device;

static i2c_sensor_context_t i2c_engine_pressure_device_context = {
		.device_address = 0xD0 // TODO no clue
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

device_t * i2c_sensor_get_accelerometer(void) {
	return &i2c_accelerometer_device;
}

device_t * i2c_sensor_get_engine_pressure(void) {
	return &i2c_engine_pressure_device;
}

//  device_t * i2c_sensor_get_gyroscope(void) {
// 	return &i2c_gyroscope_device;
//  }

device_t* i2c_sensor_get_magnetometer(void) {
	return &i2c_magnetometer_device;
}

device_t * i2c_sensor_get_barometer(void) {
	return &i2c_barometer_device;
}

device_t * i2c_sensor_get_ADC(void) {
	return &i2c_ADC_device;
}

util_error_t i2c_sensor_init(void) {

	device_interface_t * i2c_sensor_interface = i2c_get_sensor_interface();

	//device_create((void*) &i2c_gyroscope_device, &i2c_gyroscope_device_context, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create((void*) &i2c_accelerometer_device, &i2c_accelerometer_device_context, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create((void*) &i2c_barometer_device, &i2c_barometer_device_context, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create((void*) &i2c_ADC_device, &i2c_ADC_device_context, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create((void*) &i2c_magnetometer_device, &i2c_magnetometer_device_context, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);
	device_create((void*) &i2c_engine_pressure_device, &i2c_engine_pressure_device_context, i2c_sensor_interface, i2c_sensor_read_reg_HAL, i2c_sensor_write_reg_HAL);

	return ER_SUCCESS;

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
	HAL_I2C_Mem_Read_IT(if_ctx->i2c, ctx->device_address, addr, sizeof(uint8_t), data, len);
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
	HAL_I2C_Mem_Write_IT(if_ctx->i2c, ctx->device_address, addr, sizeof(uint8_t), data, len);
	if( xSemaphoreTake(if_ctx->sem, I2C_TIMEOUT) == pdTRUE ) {
		return if_ctx->error;
	} else {
		return ER_TIMEOUT;
	}
}



/* END */
