/*  Title		: Propulsion sensor Acquisition
 *  Filename	: propulsion_sensor.c
 *	Author		: thilo chalas, luciano calcoen
 *	Date		: 28.11.2022
 *	Version		: 0.1
 *	Description	: Thread grouping all the propulsion sensors on the same I2C bus.
 */


/**********************
 *	INCLUDES
 **********************/

#if 0

#include <propulsion_sensor.h>
#include <od/od.h>

#include <main.h>
#include <cmsis_os.h>
#include <od/od.h>

#include <device/i2c_sensor.h>
#include <sensor/engine_pressure.h>
#include <sensor/temperature.h>
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

static device_t * i2c_engine_press_A;
static device_t * i2c_engine_press_B;
static device_t * i2c_engine_press_C;
static device_t * i2c_engine_temp;


static uint8_t i2c_calib;

//data

static double i2c_engine_press_data_A;
static double i2c_engine_press_data_B;
static double i2c_engine_press_data_C;
static temperature_data_t i2c_engine_temp_data;


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
void prop_sensor_i2c_thread(__attribute__((unused)) void * arg) {
	static TickType_t last_wake_time;
	static const TickType_t period = pdMS_TO_TICKS(I2C_SENSOR_HEART_BEAT);
	last_wake_time = xTaskGetTickCount();

	uint16_t checkpoint = led_add_checkpoint(led_blue);
	debug_log(LOG_INFO, "Propulsion sensor i2c start\n");
	// Get sensor devices
	i2c_engine_press_A = i2c_sensor_get_ADC_A();
	i2c_engine_temp = i2c_sensor_get_ADC_A();
	
	i2c_engine_press_B = i2c_sensor_get_ADC_B();
	i2c_engine_press_C = i2c_sensor_get_ADC_B();
	

	// Initialize sensors
	util_error_t engine_press_err_A = engine_pressure_init(i2c_engine_press_A);
	util_error_t engine_temp_err = temperature_sensor_init(i2c_engine_temp);
	
	util_error_t engine_press_err_B = engine_pressure_init(i2c_engine_press_B);
	util_error_t engine_press_err_C = engine_pressure_init(i2c_engine_press_C);
	

	// Sensor initialisation checkpoints
	uint16_t checkpoint_engpress = 0;
	if (engine_press_err_A == ER_SUCCESS) {
		checkpoint_engpress = led_add_checkpoint(led_green);
	} else {
		checkpoint_engpress = led_add_checkpoint(led_red);
	}

	uint16_t checkpoint_engtemp = 0;
	if (engine_temp_err == ER_SUCCESS) {
		checkpoint_engtemp = led_add_checkpoint(led_green);
	} else {
		checkpoint_engtemp = led_add_checkpoint(led_red);
	}

	if (engine_press_err_B == ER_SUCCESS) {
		checkpoint_engpress = led_add_checkpoint(led_green);
	} else {
		checkpoint_engpress = led_add_checkpoint(led_red);
	}

	if (engine_press_err_C == ER_SUCCESS) {
		checkpoint_engpress = led_add_checkpoint(led_green);
	} else {
		checkpoint_engpress = led_add_checkpoint(led_red);
	}
	
	//manual calibration only:
	i2c_calib = 0;

	//Sensor calibration
	util_error_t error_calibration = 0;
	error_calibration |= engine_pressure_calibrate(i2c_engine_press_A);
	error_calibration |= temperature_sensor_calibrate(i2c_engine_temp);
	
	error_calibration |= engine_pressure_calibrate(i2c_engine_press_B);
	error_calibration |= engine_pressure_calibrate(i2c_engine_press_C);
	
	
	uint16_t checkpoint_calib = 0;
	if (error_calibration == ER_SUCCESS) {
		checkpoint_calib = led_add_checkpoint(led_green);
	} else {
		checkpoint_calib = led_add_checkpoint(led_red);
	}

	//mainloop
	for(;;) {
		led_checkpoint(checkpoint);
		led_checkpoint(checkpoint_engpress);
		led_checkpoint(checkpoint_engtemp);
		led_checkpoint(checkpoint_calib);



		if(1) {
			debug_log(LOG_DEBUG, "Getting data\n");
            if(engine_press_err_A == ER_SUCCESS) {
                //read engine pressure
				engine_pressure_read(i2c_engine_press_A, &i2c_engine_press_data_A);
				debug_log(LOG_DEBUG, "Pressure A %d\n");
			}
			if(engine_temp_err == ER_SUCCESS) {
				temperature_sensor_read(i2c_engine_temp, &i2c_engine_temp_data);
			}
			if(engine_press_err_B == ER_SUCCESS) {
				engine_pressure_read(i2c_engine_press_B, &i2c_engine_press_data_B);
			}
			if(engine_press_err_C == ER_SUCCESS) {
				engine_pressure_read(i2c_engine_press_C, &i2c_engine_press_data_C);
			}
			//store everything
			od_write_ENG_PRESS_I2C_A(&i2c_engine_press_data_A);
			od_write_ENG_TEMP_I2C(&i2c_engine_temp_data);
			od_write_ENG_PRESS_I2C_B(&i2c_engine_press_data_B);
			od_write_ENG_PRESS_I2C_C(&i2c_engine_press_data_C);

/*to be determined as to what we're supposed to replace it with*/
			
// #if WH_COMPUTER == A
// 			od_write_ENGPRESS_I2C_A(&i2c_acc_data);
// #else
// 			od_write_ENGPRESS_I2C_B(&i2c_acc_data);
// #endif


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

}

#endif

/* END */
