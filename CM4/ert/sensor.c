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
#include <sensor/sensor_bmi088.h>
#include <sensor/sensor_bmp390.h>
#include <od/od.h>
#include <driver/hostproc.h>
#include <hostcom.h>
#include <feedback/led.h>
#include <feedback/debug.h>
#include <nordend.h>
/**********************
 *	CONSTANTS
 **********************/

//TODO: check if this is short/long enough
#define I2C_SENSOR_HEART_BEAT	50


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/

static device_t * bmi088_acc[2];
static device_t * bmi088_gyr[2];
static device_t * bmp390_baro[2];

static bmi088_acc_context_t bmi088_acc_ctx[2];
static bmi088_gyr_context_t bmi088_gyr_ctx[2];
static bmp390_baro_context_t bmp390_baro_ctx[2];


static sensor_imu_data_t imu_data[2];
static sensor_baro_data_t baro_data[2];

/**********************
 *	PROTOTYPES
 **********************/



/**********************
 *	DECLARATIONS
 **********************/



/**
 * @brief 	i2c sensor acquisition thread
 * @details	this thread will acquire data from all the sensors connected on the
 * 			i2c bus. The data will then be dispatched for delivery.
 */
void sensor_i2c_thread(__attribute__((unused)) void * arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(I2C_SENSOR_HEART_BEAT);

	last_wake_time = xTaskGetTickCount();

	//init and discover sensors
	bmi088_acc[0] = i2c_sensor_get_bmi088_acc(0);
	bmi088_gyr[0] = i2c_sensor_get_bmi088_gyr(0);
	bmi088_acc[1] = i2c_sensor_get_bmi088_acc(1);
	bmi088_gyr[1] = i2c_sensor_get_bmi088_gyr(1);

	bmp390_baro[0] = i2c_sensor_get_bmp390_baro(0);
	bmp390_baro[1] = i2c_sensor_get_bmp390_baro(1);

	bmi088_acc_init(bmi088_acc[0], &bmi088_acc_ctx[0]);
	bmi088_gyr_init(bmi088_gyr[0], &bmi088_gyr_ctx[0]);
	bmi088_acc_init(bmi088_acc[1], &bmi088_acc_ctx[1]);
	bmi088_gyr_init(bmi088_gyr[1], &bmi088_gyr_ctx[1]);

	bmp390_baro_init(bmp390_baro[0], &bmp390_baro_ctx[0]);
	bmp390_baro_init(bmp390_baro[1], &bmp390_baro_ctx[1]);




	// main loop
	for(;;) {
		//poll sensors for data

		debug_log(LOG_DEBUG, "reading sensors\n");

		bmi088_acc_read(bmi088_acc[0], &imu_data[0]);
		bmi088_gyr_read(bmi088_gyr[0], &imu_data[0]);
		bmi088_acc_read(bmi088_acc[1], &imu_data[1]);
		bmi088_gyr_read(bmi088_gyr[1], &imu_data[1]);

		bmp390_baro_read(bmp390_baro[0], &baro_data[0]);
		bmp390_baro_read(bmp390_baro[1], &baro_data[1]);

#if ND_COMPUTER == ND_A
		if(bmi088_acc_is_available(bmi088_acc[0]) || bmi088_gyr_is_available(bmi088_gyr[0])) {
			od_write_SENSOR_IMU_A_0(&imu_data[0]);
		}
		if(bmi088_acc_is_available(bmi088_acc[1]) || bmi088_gyr_is_available(bmi088_gyr[1])) {
			od_write_SENSOR_IMU_A_1(&imu_data[1]);
		}
		if(bmp390_baro_is_available(bmp390_baro[0])) {
			od_write_SENSOR_BARO_A_0(&baro_data[0]);
		}
		if(bmp390_baro_is_available(bmp390_baro[1])) {
			od_write_SENSOR_BARO_A_1(&baro_data[1]);
		}
#else
		if(bmi088_acc_is_available(bmi088_acc[0]) || bmi088_gyr_is_available(bmi088_gyr[0])) {
			od_write_SENSOR_IMU_B_0(&imu_data[0]);
		}
		if(bmi088_acc_is_available(bmi088_acc[1]) || bmi088_gyr_is_available(bmi088_gyr[1])) {
			od_write_SENSOR_IMU_B_1(&imu_data[1]);
		}
		if(bmp390_baro_is_available(bmp390_baro[0])) {
			od_write_SENSOR_BARO_B_0(&baro_data[0]);
		}
		if(bmp390_baro_is_available(bmp390_baro[1])) {
			od_write_SENSOR_BARO_B_1(&baro_data[1]);
		}
#endif


		vTaskDelayUntil( &last_wake_time, period );

	}

}

/* END */
