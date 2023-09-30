
#include "sensor_mcp3426.h"
#include <device/i2c_sensor.h>
#include <driver/i2c.h>
#include <feedback/debug.h>
#include <string.h>
#include <util.h>


util_error_t mcp3425_write_wrapper(device_t * dev, uint8_t data) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	i2c_interface_context_t * if_ctx = (i2c_interface_context_t *) dev->interface->context;
	uint8_t _data[2];
//	_data[0] = context->device_address;
//	_data[1] = data;
//	return device_interface_send(dev->interface, _data, 2);

	_data[0] = data;

	if(HAL_OK == HAL_I2C_Master_Transmit(if_ctx->i2c, (context->device_address << 1), &_data[0], 1, 10)) {
		return ER_SUCCESS;
	} else {
		return ER_TIMEOUT;
	}
}

util_error_t mcp3425_read_wrapper(device_t * dev, uint8_t * data, uint16_t len) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	i2c_interface_context_t * if_ctx = (i2c_interface_context_t *) dev->interface->context;

	if(len > 4) {
		return ER_OUT_OF_RANGE;
	}
	//uint8_t _data[4];

	if(HAL_OK == HAL_I2C_Master_Receive(if_ctx->i2c, (context->device_address << 1), &data[0], len, 10)) {
		return ER_SUCCESS;
	} else {
		return ER_TIMEOUT;
	}
}

util_error_t mcp3425_adc_init(device_t * dev, mcp3426_adc_context_t * ctx) {

	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	context->context = ctx;

	//discover HW not sure if possible ??

	//check if write is possible
	if(mcp3425_write_wrapper(dev, 0) == ER_SUCCESS) {
		ctx->hw_available = 1;
		debug_log(LOG_WARNING, "%s: detected board\n", __func__);
	} else {
		ctx->hw_available = 0;
		debug_log(LOG_WARNING, "%s: not detected board\n", __func__);
		return ER_RESSOURCE_ERROR;
	}


	return ER_SUCCESS;
}


//output data in mv
util_error_t mcp3425_adc_read(device_t * dev, uint8_t channel, float * data) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	mcp3426_adc_context_t * ctx = (mcp3426_adc_context_t *) context->context;

	if(ctx->hw_available) {
		ctx->config = 	MCP3422_CHANNEL_VALUE(channel) |
						MCP3422_SAMPLE_RATE_VALUE(MCP3422_SRATE_240) |
						MCP3422_PGA_VALUE(MCP3422_PGA_1) |
						MCP3422_CONT_SAMPLING;

		ctx->pga = 1;
		ctx->selected_sstvt = 1.0; // mV / LSB

		//write only one byte
		mcp3425_write_wrapper(dev, ctx->config);

		//wait for a sample to be acquired (240sps)
		osDelay(HB_MS2TICK(1000/200));

		//read data
		uint8_t _data[4] = {0};
		mcp3425_read_wrapper(dev, &_data[0], 2);
		uint32_t tmp_data = (_data[1]) + (_data[0] << 8);

		*data = (float)tmp_data / ((float) ctx->pga) * ctx->selected_sstvt;



		return ER_SUCCESS;
	} else {
		return ER_RESSOURCE_ERROR;
	}
}


uint8_t mcp3425_adc_is_available(device_t * dev) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	mcp3426_adc_context_t * ctx = (mcp3426_adc_context_t *) context->context;
	return ctx->hw_available;
}




