/*  Title		: Sensor Acquisition
 *  Filename	: sensor.c
 *	Author		: iacopo sprenger
 *	Date		: 13.07.2021
 *	Version		: 0.1
 *	Description	: Thread grouping all the sensors on the same I2C bus.
 */


/**********************
 *	INCLUDES
 **********************/

#include <sensor.h>
#include <od/od.h>


#include <main.h>
#include <cmsis_os.h>

#include <device/i2c_sensor.h>
<<<<<<< HEAD
=======
#include <sensor/accelerometer.h>
#include <sensor/gyroscope.h>
#include <sensor/barometer.h>
#include <sensor/engine_pressure.h>
>>>>>>> Thilo_Chalas
#include <od/od.h>
#include <driver/hostproc.h>
#include <hostcom.h>
#include <feedback/led.h>
#include <feedback/debug.h>
/**********************
 *	CONSTANTS
 **********************/

//TODO: check if this is short/long enough
#define I2C_SENSOR_HEART_BEAT	100


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/


<<<<<<< HEAD
=======
static device_t * i2c_acc;
static device_t * i2c_gyro;
static device_t * i2c_baro;
static device_t * i2c_engine_press;


static uint8_t i2c_calib;

//data

static accelerometer_data_t i2c_acc_data;
static gyroscope_data_t i2c_gyro_data;
static barometer_data_t i2c_baro_data;
static barometer_meta_t i2c_baro_meta;
static double i2c_engine_press_data;



>>>>>>> Thilo_Chalas
/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/



/**
 * @brief 	i2c sensor aqcuisition thread
 * @details	this thread will acquire data from all the sensors connected on the
 * 			i2c bus. The data will then be dispatched for delivery.
 */
void sensor_i2c_thread(__attribute__((unused)) void * arg) {
<<<<<<< HEAD
//	static TickType_t last_wake_time;
//	static const TickType_t period = pdMS_TO_TICKS(I2C_SENSOR_HEART_BEAT);
//	static const TickType_t baro_delay = pdMS_TO_TICKS(10);
//	last_wake_time = xTaskGetTickCount();
//
//	uint16_t checkpoint = led_add_checkpoint(led_green);
//	debug_log("Sensor i2c start\n");
//	//get devices
//
//
//	//init
//
//	//manual calibration only:
//	i2c_calib = 0;
//
//	//mainloop
//	for(;;) {
//		led_checkpoint(checkpoint);
//		led_checkpoint(checkpoint_baro);
//		led_checkpoint(checkpoint_acc);
//
//
//		if(1) {
//			if(baro_err == ER_SUCCESS) {
//				//baro start temp
//				barometer_convert_temp(i2c_baro);
//				//TickType_t baro_temp_time = xTaskGetTickCount();
//			}
//
//			if(acc_err == ER_SUCCESS) {
//				//acc read
//				accelerometer_read_data(i2c_acc, &i2c_acc_data);
//				accelerometer_process_data(&i2c_acc_data, 10000);
//				//hostcom_data_acc_send(HAL_GetTick(), i2c_acc_data.processed[ACC_Z]);
//			}
//
//			vTaskDelay(baro_delay);
//
//			if(baro_err == ER_SUCCESS) {
//				//baro read & start pres
//				barometer_read_temp(i2c_baro, &i2c_baro_meta);
//				barometer_convert_pres(i2c_baro);
//				//TickType_t baro_pres_time = xTaskGetTickCount();
//			}
//
//			vTaskDelay(baro_delay);
//
//			if(baro_err == ER_SUCCESS) {
//				//baro read
//				barometer_read_pres(i2c_baro, &i2c_baro_meta);
//				barometer_convert(&i2c_baro_meta, &i2c_baro_data);
//				hostcom_data_baro_send(HAL_GetTick(), i2c_baro_data.pressure);
//			}
//
//			//store everything
//
//
//
//
//
//
//#if WH_COMPUTER == A
//			od_write_ACC_I2C_A(&i2c_acc_data);
//			od_write_BARO_I2C_A(&i2c_baro_data);
//#else
//			od_write_ACC_I2C_B(&i2c_acc_data);
//			od_write_BARO_I2C_B(&i2c_baro_data);
//#endif
//
//
//		} else { /* Calibration */
//			//calibration steps
//
//			//normally not necessary...
//
//
//		}
//
//
//
//
//
//		//send data to hostproc for verif
//
//
////		debug_log(	"time: %ld\nacc: %d, %d, %d\npress: %ld, temp: %ld\n",
////					HAL_GetTick(),
////					i2c_acc_data.processed[0], i2c_acc_data.processed[1],
////					i2c_acc_data.processed[2], i2c_baro_data.pressure,
////					i2c_baro_data.temperature);
//
//
//		vTaskDelayUntil( &last_wake_time, period );
//
//	}
=======
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(I2C_SENSOR_HEART_BEAT);
	static const TickType_t baro_delay = pdMS_TO_TICKS(10);
	last_wake_time = xTaskGetTickCount();

	uint16_t checkpoint = led_add_checkpoint(led_green);
	debug_log("Sensor i2c start\n");
	//get devices
	i2c_acc = i2c_sensor_get_accelerometer();
	i2c_gyro = i2c_sensor_get_gyroscope();
	i2c_baro = i2c_sensor_get_barometer();




	//init
	util_error_t acc_err = accelerometer_init(i2c_acc);
	util_error_t gyro_err = gyroscope_init(i2c_gyro);
	util_error_t baro_err = barometer_init(i2c_baro, &i2c_baro_meta);
	util_error_t engine_pressure_err = engine_pressure_init(i2c_engine_press);

	uint16_t checkpoint_acc;
	if(acc_err == ER_SUCCESS) {
		checkpoint_acc = led_add_checkpoint(led_green);
	} else {
		checkpoint_acc = led_add_checkpoint(led_red);
	}
	uint16_t checkpoint_gyro;
	if(gyro_err == ER_SUCCESS) {
		checkpoint_gyro = led_add_checkpoint(led_green);
	} else {
		checkpoint_gyro = led_add_checkpoint(led_red);
	}
	uint16_t checkpoint_baro;
	if(baro_err == ER_SUCCESS) {
		checkpoint_baro = led_add_checkpoint(led_green);
	} else {
		checkpoint_baro = led_add_checkpoint(led_red);
	}

	//manual calibration only:
	i2c_calib = 0;

	//mainloop
	for(;;) {
		led_checkpoint(checkpoint);
		led_checkpoint(checkpoint_baro);
		led_checkpoint(checkpoint_gyro);
		led_checkpoint(checkpoint_acc);


		if(1) {
			if(baro_err == ER_SUCCESS) {
				//baro start temp
				barometer_convert_temp(i2c_baro);
				//TickType_t baro_temp_time = xTaskGetTickCount();
			}

			if(acc_err == ER_SUCCESS) {
				//acc read
				accelerometer_read_data(i2c_acc, &i2c_acc_data);
				accelerometer_process_data(&i2c_acc_data, 10000);
				//hostcom_data_acc_send(HAL_GetTick(), i2c_acc_data.processed[ACC_Z]);
			}

			vTaskDelay(baro_delay);

			if(baro_err == ER_SUCCESS) {
				//baro read & start pres
				barometer_read_temp(i2c_baro, &i2c_baro_meta);
				barometer_convert_pres(i2c_baro);
				//TickType_t baro_pres_time = xTaskGetTickCount();
			}

			if(gyro_err == ER_SUCCESS) {
				//gyro read or acc read second time
				gyroscope_read_data(i2c_gyro, &i2c_gyro_data);
				gyroscope_process_data(&i2c_gyro_data, 10000);
			}

			vTaskDelay(baro_delay);

			if(baro_err == ER_SUCCESS) {
				//baro read
				barometer_read_pres(i2c_baro, &i2c_baro_meta);
				barometer_convert(&i2c_baro_meta, &i2c_baro_data);
				hostcom_data_baro_send(HAL_GetTick(), i2c_baro_data.pressure);
			}

			//store everything






#if WH_COMPUTER == A
			od_write_ACC_I2C_A(&i2c_acc_data);
			od_write_GYRO_I2C_A(&i2c_gyro_data);
			od_write_BARO_I2C_A(&i2c_baro_data);
#else
			od_write_ACC_I2C_B(&i2c_acc_data);
			od_write_GYRO_I2C_B(&i2c_gyro_data);
			od_write_BARO_I2C_B(&i2c_baro_data);
#endif


		} else { /* Calibration */
			//calibration steps

			//normally not necessary...


		}





		//send data to hostproc for verif


//		debug_log(	"time: %ld\nacc: %d, %d, %d\npress: %ld, temp: %ld\n",
//					HAL_GetTick(),
//					i2c_acc_data.processed[0], i2c_acc_data.processed[1],
//					i2c_acc_data.processed[2], i2c_baro_data.pressure,
//					i2c_baro_data.temperature);


		vTaskDelayUntil( &last_wake_time, period );

	}
>>>>>>> Thilo_Chalas

}

/* END */
