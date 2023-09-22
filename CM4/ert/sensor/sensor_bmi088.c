
#include <sensor/sensor_bmi088.h>
#include <device/i2c_sensor.h>
#include <feedback/debug.h>



util_error_t bmi088_acc_init(device_t * dev, bmi088_acc_context_t * ctx) {
	 i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	 context->context = ctx;
	 uint8_t buffer[1];

	 buffer[0] = 0x00;
	 device_read_u8(dev, BMI088_REG_ACC_CHIPID, &buffer[0]);
	 debug_log(LOG_WARNING, "discovering.. %x\n", buffer[0]);
	 if(buffer[0] == BMI088_ACC_CHIPID_VAL) {
		 ctx->hw_available = 1;
	 } else {
		 ctx->hw_available = 0;
		 return ER_RESSOURCE_ERROR;
	 }

	 ctx->selected_range = 6; //6g is default
	 ctx->selected_sstvt = ctx->selected_range / (float)INT16_MAX;

	 device_write_u8(dev, BMI088_REG_ACC_PWRCTRL, 0x04);

	 device_write_u8(dev, BMI088_REG_ACC_PWRCONF, 0x00);


	 return ER_SUCCESS;
}

util_error_t bmi088_gyr_init(device_t * dev, bmi088_gyr_context_t * ctx) {
	 i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	 context->context = ctx;
	 uint8_t buffer[1];

	 buffer[0] = 0x00;
	 device_read_u8(dev, BMI088_REG_GYR_CHIPID, &buffer[0]);
	 debug_log(LOG_WARNING, "discovering.. %x\n", buffer[0]);
	 if(buffer[0] == BMI088_GYR_CHIPID_VAL) {
		 ctx->hw_available = 1;
	 } else {
		 ctx->hw_available = 0;
		 return ER_RESSOURCE_ERROR;
	 }

	 ctx->selected_range = 2000; //2000 dsp is default
	 ctx->selected_sstvt = 2000 / (float)INT16_MAX;

	 return ER_SUCCESS;
}

util_error_t bmi088_acc_read(device_t * dev, sensor_imu_data_t * data) {
	 i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	 bmi088_acc_context_t * ctx = context->context;

	 if(ctx->hw_available) {
		 uint8_t buffer[6];
		 device_read(dev, BMI088_REG_ACC_X_L, buffer, 6);
		 //debug_log(LOG_WARNING, "acc data: %x, %x, %x\n", buffer[0], buffer[2], buffer[4]);
		 data->raw_acc[AXIS_X] = buffer[0] | buffer[1] << 8;
		 data->raw_acc[AXIS_Y] = buffer[2] | buffer[3] << 8;
		 data->raw_acc[AXIS_Z] = buffer[4] | buffer[5] << 8;
		 data->acc[AXIS_X] = data->raw_acc[AXIS_X]*ctx->selected_sstvt;
		 data->acc[AXIS_Y] = data->raw_acc[AXIS_Y]*ctx->selected_sstvt;
		 data->acc[AXIS_Z] = data->raw_acc[AXIS_Z]*ctx->selected_sstvt;
		 data->acc_time = util_get_time();
		 debug_log(LOG_WARNING, "acc data: %d, %d, %d\n", data->raw_acc[AXIS_X], data->raw_acc[AXIS_Y], data->raw_acc[AXIS_Z]);

		 return ER_SUCCESS;
	 } else {
		 return ER_RESSOURCE_ERROR;
	 }
}

util_error_t bmi088_gyr_read(device_t * dev, sensor_imu_data_t * data) {
	 i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	 bmi088_gyr_context_t * ctx = context->context;

	 if(ctx->hw_available) {
		 uint8_t buffer[6];
		 device_read(dev, BMI088_REG_GYR_X_L, buffer, 6);
		 //debug_log(LOG_WARNING, "gyr data: %x, %x, %x\n", buffer[0], buffer[2], buffer[4]);
		 data->raw_gyro[AXIS_X] = (buffer[0] | buffer[1] << 8);
		 data->raw_gyro[AXIS_Y] = (buffer[2] | buffer[3] << 8);
		 data->raw_gyro[AXIS_Z] = (buffer[4] | buffer[5] << 8);
		 data->gyro[AXIS_X] = data->raw_gyro[AXIS_X]*ctx->selected_sstvt;
		 data->gyro[AXIS_Y] = data->raw_gyro[AXIS_Y]*ctx->selected_sstvt;
		 data->gyro[AXIS_Z] = data->raw_gyro[AXIS_Z]*ctx->selected_sstvt;
		 data->gyro_time = util_get_time();
		 debug_log(LOG_WARNING, "gyro data: %d, %d, %d\n", data->raw_gyro[AXIS_X], data->raw_gyro[AXIS_Y], data->raw_gyro[AXIS_Z]);

		 return ER_SUCCESS;
	 } else {
		 return ER_RESSOURCE_ERROR;
	 }
}





