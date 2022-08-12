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
#include <sensor/accelerometer.h>
#include <sensor/gyroscope.h>
#include <sensor/barometer.h>
#include <od/od.h>
/**********************
 *	CONSTANTS
 **********************/

//TODO: check if this is short/long enough
#define I2C_SENSOR_HEART_BEAT	80


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/


static device_t * i2c_acc;
static device_t * i2c_gyro;
static device_t * i2c_baro;

static device_interface_t * hostproc_feedback;


static uint8_t i2c_calib;

//data

static accelerometer_data_t i2c_acc_data;
static gyroscope_data_t i2c_gyro_data;
static barometer_data_t i2c_baro_data;
static barometer_meta_t i2c_baro_meta;



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
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(I2C_SENSOR_HEART_BEAT);
	last_wake_time = xTaskGetTickCount();

	//get devices
	i2c_acc = i2c_sensor_get_accelerometer();
	i2c_gyro = i2c_sensor_get_gyroscope();
	i2c_baro = i2c_sensor_get_barometer();

	hostproc_feedback = hostproc_get_feedback_interface();

	//init
	accelerometer_init(i2c_acc);
	gyroscope_init(i2c_gyro);
	barometer_init(i2c_baro, &i2c_baro_meta);

	//manual calibration only:
	i2c_calib = 0;

	//mainloop
	for(;;) {
		if(!i2c_calib) { /* Normal operation */
			TickType_t baro_delay = pdMS_TO_TICKS(10);

			//baro start temp
			barometer_convert_temp(i2c_baro);
			TickType_t baro_temp_time = xTaskGetTickCount();

			//acc read
			accelerometer_read_data(i2c_acc, &i2c_acc_data);
			accelerometer_process_data(&i2c_acc_data, 10000);

			vTaskDelayUntil(&baro_temp_time, baro_delay);

			//baro read & start pres
			barometer_read_temp(i2c_baro, &i2c_baro_meta);
			barometer_convert_pres(i2c_baro);
			TickType_t baro_pres_time = xTaskGetTickCount();

			//gyro read or acc read second time
			gyroscope_read_data(i2c_gyro, &i2c_gyro_data);
			gyroscope_process_data(&i2c_gyro_data, 10000);

			vTaskDelayUntil(&baro_pres_time, baro_delay);

			//baro read
			barometer_read_pres(i2c_baro, &i2c_baro_meta);
			barometer_convert(&i2c_baro_meta, &i2c_baro_data);

			//store everything
			od_write_ACC_I2C_A(&i2c_acc_data);
			od_write_GYRO_I2C_A(&i2c_gyro_data);
			od_write_BARO_I2C_A(&i2c_baro_data);


		} else { /* Calibration */
			//calibration steps

			//normally not necessary...


		}

		//send data to hostproc for verif


		vTaskDelayUntil( &last_wake_time, period );

	}

}

/* END */
