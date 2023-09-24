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
#include <sensor/sensor_mcp3426.h>
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
#define ENGINE_SENSOR_HEART_BEAT	200


/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/

static device_t * mcp3426_adc_up;
static device_t * mcp3426_adc_dn;
//static device_t * mcp3426_adc_st;

static mcp3426_adc_context_t mcp3426_adc_up_ctx;
static mcp3426_adc_context_t mcp3426_adc_dn_ctx;


/**********************
 *	PROTOTYPES
 **********************/


util_error_t engine_sensor_convert_values(sensor_eng_data_t * data);


/**********************
 *	DECLARATIONS
 **********************/





/**
 * @brief 	engine sensor acquisition thread
 * @details	this thread will acquire data from all the sensors connected on the
 * 			i2c bus. The data will then be dispatched for delivery.
 */
void engine_sensor_thread(__attribute__((unused)) void * arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(ENGINE_SENSOR_HEART_BEAT);

	last_wake_time = xTaskGetTickCount();

	mcp3426_adc_up = i2c_engine_sensor_get_mcp3426_up();
	mcp3426_adc_dn = i2c_engine_sensor_get_mcp3426_dn();
	//init and discover sensors


	mcp3425_adc_init(mcp3426_adc_up, &mcp3426_adc_up_ctx);
	mcp3425_adc_init(mcp3426_adc_dn, &mcp3426_adc_dn_ctx);


	// main loop
	for(;;) {
		//poll sensors for data

		debug_log(LOG_DEBUG, "reading sensors\n");

		//read data
		sensor_eng_data_t data;
		uint32_t _data;
		mcp3425_adc_read(mcp3426_adc_up, 0, &_data);
		data.adc_1 = _data;
		data.adc_1_time = util_get_time();
		mcp3425_adc_read(mcp3426_adc_up, 1, &_data);
		data.adc_2 = _data;
		data.adc_2_time = util_get_time();
		mcp3425_adc_read(mcp3426_adc_dn, 0, &_data);
		data.adc_3 = _data;
		data.adc_3_time = util_get_time();
		mcp3425_adc_read(mcp3426_adc_dn, 1, &_data);
		data.adc_4 = _data;
		data.adc_4_time = util_get_time();


		vTaskDelayUntil( &last_wake_time, period );

	}

}



util_error_t engine_sensor_convert_values(sensor_eng_data_t * data) {

}

/* END */
