
#include "sensor_adxl375.h"
#include <device/device.h>
#include <device/i2c_sensor.h>




util_error_t adxl375_acc_init(device_t * dev, adxl375_acc_context_t * ctx) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	context->context = ctx;

	uint8_t data[1];

	device_read_u8(dev, ADXL375_REG_DEVID, &data[0]);

	if(data[0] == ADXL375_DEVID_VAL) {
		ctx->hw_available = 1;
	} else {
		ctx->hw_available = 0;
		return ER_RESSOURCE_ERROR;
	}

	//set bw rate
	// 200Hz -> 0b1011
	device_write_u8(dev, ADXL375_REG_BWRATE, 0b00001011);

	//set measure in pwrctl
	device_write_u8(dev, ADXL375_REG_PWRCTL, 0b00001000);

	ctx->selected_sstvt = 1.0/20.5;

	return ER_SUCCESS;
}

util_error_t adxl375_acc_read(device_t * dev, sensor_acc_data_t * data) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	adxl375_acc_context_t * ctx = (adxl375_acc_context_t *) context->context;
	if(ctx->hw_available) {
		uint8_t _data[6];
		device_read(dev, ADXL375_REG_DATAX0, &_data[0], 6);
		data->raw_acc[AXIS_X] = util_decode_i16(&_data[0]);
		data->raw_acc[AXIS_Y] = util_decode_i16(&_data[2]);
		data->raw_acc[AXIS_Z] = util_decode_i16(&_data[4]);
		data->acc[AXIS_X] = data->raw_acc[AXIS_X]*ctx->selected_sstvt;
		data->acc[AXIS_Y] = data->raw_acc[AXIS_Y]*ctx->selected_sstvt;
		data->acc[AXIS_Z] = data->raw_acc[AXIS_Z]*ctx->selected_sstvt;

		return ER_SUCCESS;
	} else {
		return ER_RESSOURCE_ERROR;
	}
}


uint8_t adx375_acc_is_available(device_t * dev) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	adxl375_acc_context_t * ctx = (adxl375_acc_context_t *) context->context;

	return ctx->hw_available;
}



