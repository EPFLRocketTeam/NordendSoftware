

#include "sensor_bmp390.h"
#include <device/i2c_sensor.h>
#include <feedback/debug.h>


float bmp390_compensate_temperature(device_t * dev, uint32_t uncomp_temp);
float bmp390_compensate_pressure(device_t * dev, uint32_t uncomp_press) ;



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

	 //read nv memory for trimming parameters
	 uint8_t data[21];

	 device_read(dev, BMP390_REG_TRIM, &data[0], 21);

	 ctx->par_t1 = util_decode_u16(&data[ 0]);
	 ctx->par_t2 = util_decode_u16(&data[ 2]);
	 ctx->par_t3 = util_decode_i8 (&data[ 4]);
	 ctx->par_p1 = util_decode_i16(&data[ 5]);
	 ctx->par_p2 = util_decode_i16(&data[ 7]);
	 ctx->par_p3 = util_decode_i8 (&data[ 9]);
	 ctx->par_p4 = util_decode_i8 (&data[10]);
	 ctx->par_p5 = util_decode_u16(&data[11]);
	 ctx->par_p6 = util_decode_u16(&data[13]);
	 ctx->par_p7 = util_decode_i8 (&data[15]);
	 ctx->par_p8 = util_decode_i8 (&data[16]);
	 ctx->par_p9 = util_decode_i16(&data[17]);
	 ctx->par_p10= util_decode_i8 (&data[19]);
	 ctx->par_p11= util_decode_i8 (&data[20]);

	 debug_log(LOG_WARNING, "coeffs: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
			 ctx->par_t1,
			 ctx->par_t2,
			 ctx->par_t3,
			 ctx->par_p1,
			 ctx->par_p2,
			 ctx->par_p3,
			 ctx->par_p4,
			 ctx->par_p5,
			 ctx->par_p6,
			 ctx->par_p7,
			 ctx->par_p8,
			 ctx->par_p9,
			 ctx->par_p10,
			 ctx->par_p11);


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
		 data->raw_press = buffer[0] | buffer[1] << 8 | buffer[2] << 16;
		 data->raw_temp = buffer[3] | buffer[4] << 8 | buffer[5] << 16;

		 data->time = util_get_time();

		 data->temp = bmp390_compensate_temperature(dev, data->raw_temp);
		 data->press = bmp390_compensate_pressure(dev, data->raw_press);

		 debug_log(LOG_WARNING, "baro data: %d, %d\n", data->raw_press, data->raw_temp);

		 return ER_SUCCESS;
	 } else {
		 return ER_RESSOURCE_ERROR;
	 }
}

uint8_t bmp390_baro_is_available(device_t * dev) {
	 i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	 bmp390_baro_context_t * ctx = context->context;
	 return ctx->hw_available;
}

float bmp390_compensate_temperature(device_t * dev, uint32_t uncomp_temp) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	bmp390_baro_context_t * calib_data = context->context;

	float partial_data1;
	float partial_data2;
	partial_data1 = (float)(uncomp_temp - calib_data->par_t1);
	partial_data2 = (float)(partial_data1 * calib_data->par_t2);
	/* Update the compensated temperature in calib structure since this is
	* needed for pressure calculation */
	calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;
	/* Returns compensated raw_temp */
	return calib_data->t_lin;
}


float bmp390_compensate_pressure(device_t * dev, uint32_t uncomp_press) {
	i2c_sensor_context_t * context = (i2c_sensor_context_t *) dev->context;
	bmp390_baro_context_t * calib_data = context->context;

	/* Variable to store the compensated raw_press */
	float comp_press;
	/* Temporary variables used for compensation */
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;

	/* Calibration data */
	partial_data1 = calib_data->par_p6 * calib_data->t_lin;
	partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = calib_data->par_p2 * calib_data->t_lin;
	partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out2 = (float)uncomp_press *
			(calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;

	comp_press = partial_out1 + partial_out2 + partial_data4;

	return comp_press;
}




