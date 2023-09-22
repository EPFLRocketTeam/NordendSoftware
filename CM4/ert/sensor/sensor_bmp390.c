

#include "sensor_bmp390.h"
#include <device/i2c_sensor.h>
#include <feedback/debug.h>



util_error_t bmp390_baro_init(device_t * dev, bmp390_baro_context_t * ctx) {
	 i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	 context->context = ctx;
	 uint8_t buffer[1];

	 device_read_u8(dev, BMP390_REG_BARO_CHIPID, &buffer[0]);
	 debug_log(LOG_WARNING, "discovering.. %x\n", buffer[0]);
	 if(buffer[0] == BMP390_BARO_CHIPID_VAL) {
		 ctx->hw_available = 1;
	 } else {
		 ctx->hw_available = 0;
		 return ER_RESSOURCE_ERROR;
	 }

	 //enable pressure and temperature in normal mode
	 device_write_u8(dev, BMP390_REG_BARO_PWR_CTRL, (0b11 << 4) | 0b11);

	 return ER_SUCCESS;
}

util_error_t bmp390_baro_read(device_t * dev, sensor_baro_data_t * data) {
	 i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	 bmp390_baro_context_t * ctx = context->context;

	 if(ctx->hw_available) {
		 uint8_t buffer[6];
		 device_read(dev, BMP390_REG_BARO_DATA0, buffer, 6);
		 data->pressure = buffer[0] | buffer[1] << 8 | buffer[2] << 16;
		 data->temperature = buffer[3] | buffer[4] << 8 | buffer[5] << 16;

		 data->time = util_get_time();
		 debug_log(LOG_WARNING, "baro data: %d, %d\n", data->pressure, data->temperature);

		 return ER_SUCCESS;
	 } else {
		 return ER_RESSOURCE_ERROR;
	 }
}




