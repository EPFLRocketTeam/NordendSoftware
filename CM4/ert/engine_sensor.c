/*  Title		: Sensor Acquisition
 *  Filename	: engine_sensor.c
 *	Author		: iacopo sprenger
 *	Date		: 24.09.2023
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
#include <math.h>

#include <device/i2c_sensor.h>
#include <sensor/sensor_mcp3426.h>
#include <od/od.h>
#include <driver/hostproc.h>
#include <hostcom.h>
#include <feedback/led.h>
#include <feedback/debug.h>
#include <nordend.h>
#include <abstraction/gpio.h>
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

	//osDelay(100);




	mcp3425_adc_init(mcp3426_adc_up, &mcp3426_adc_up_ctx);
	mcp3425_adc_init(mcp3426_adc_dn, &mcp3426_adc_dn_ctx);




	// main loop
	for(;;) {
		//poll sensors for data

		debug_log(LOG_INFO, "reading engine sensors\n");

		//read data
		sensor_eng_data_t data;
		float _data;
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

		//if(mcp3425_adc_is_available(mcp3426_adc_up) && mcp3425_adc_is_available(mcp3426_adc_dn)) {
			engine_sensor_convert_values(&data);
			od_write_ENGINE_SENSORS_DATA(&data);
		//}

		vTaskDelayUntil( &last_wake_time, period );

	}

}


#define VOLTAGE_DIVIDER(v) (v)*(14.3 + 10.0) / (10.0)


#define V_SUPPLY 	(5000.0) //mv
#define P_MAX		(60.0)   //bar
#define P_MIN      	(1.0)    //bar


float engine_convert_pressure(float voltage) {

	float delta_p = P_MAX - P_MIN;
	float v_max = 0.8*V_SUPPLY;
	float v_min = 0.1*V_SUPPLY;

	return (delta_p * (voltage - v_min)) / v_max + P_MIN;
}


float engine_convert_temperature(float voltage) {

	float resistance = 5100*voltage / (5000 - voltage);

	float temp = (sqrtf(-0.00232 * resistance + 17.59246) - 3.908) / 0.00116;

	return temp; //pt1000 formula here

}


util_error_t engine_sensor_convert_values(sensor_eng_data_t * data) {

	data->press_eth = engine_convert_pressure(VOLTAGE_DIVIDER(data->adc_2));
	data->press_n2o = engine_convert_pressure(VOLTAGE_DIVIDER(data->adc_1));
	data->press_eng = engine_convert_pressure(VOLTAGE_DIVIDER(data->adc_4));
	data->temp_tank = engine_convert_temperature(data->adc_3); //TODO: find our how to read the temperature sensor..


	return ER_SUCCESS;
}

/* END */
