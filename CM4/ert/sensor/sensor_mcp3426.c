
#include "sensor_mcp3426.h"
#include <device/i2c_sensor.h>
#include <feedback/debug.h>
#include <string.h>
#include <util.h>


util_error_t mcp3425_write_wrapper(device_t * dev, uint8_t data) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	uint8_t _data[2];
	_data[0] = context->device_address;
	_data[1] = data;
	return device_interface_send(dev->interface, _data, 2);
}

util_error_t mcp3425_read_wrapper(device_t * dev, uint8_t * data, uint16_t len) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	if(len > 4) {
		return ER_OUT_OF_RANGE;
	}
	uint8_t _data[5];
	uint32_t _len = len+1;
	_data[0] = context->device_address;
	memcpy(&_data[1], data, len);
	util_error_t err = device_interface_recv(dev->interface, _data, &_len);
	memcpy(&data[0], &_data[1], len);
	return err;
}

util_error_t mcp3425_adc_init(device_t * dev, mcp3426_adc_context_t * ctx) {

	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	context->context = ctx;

	//discover HW not sure if possible ??

	//check if write is possible
	if(mcp3425_write_wrapper(dev, 0) == ER_SUCCESS) {
		ctx->hw_available = 1;
	} else {
		ctx->hw_available = 0;
		return ER_RESSOURCE_ERROR;
	}


	return ER_SUCCESS;
}

util_error_t mcp3425_adc_read(device_t * dev, uint8_t channel, uint32_t * data) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	mcp3426_adc_context_t * ctx = (mcp3426_adc_context_t *) context->context;

	if(ctx->hw_available) {
		ctx->config = 	MCP3422_CHANNEL_VALUE(channel) |
						MCP3422_SAMPLE_RATE_VALUE(MCP3422_SRATE_240) |
						MCP3422_PGA_VALUE(MCP3422_PGA_1) |
						MCP3422_CONT_SAMPLING;

		//write only one byte
		mcp3425_write_wrapper(dev, ctx->config);

		//wait for a sample to be acquired (240sps)
		osDelay(HB_MS2TICK(1000/240));

		//read data
		uint8_t _data[4];
		mcp3425_read_wrapper(dev, &_data[0], 3);
		*data = util_decode_i16(_data);

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




