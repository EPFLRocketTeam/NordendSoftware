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

#include <propulsion_sensor.h>
#include <od/od.h>

#include <main.h>
#include <cmsis_os.h>

#include <device/i2c_sensor.h>
#include <sensor/engine_pressure.h>
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

static device_t * i2c_engine_press;


static uint8_t i2c_calib;

//data

static double i2c_engine_press_data;



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

	uint16_t checkpoint = led_add_checkpoint(led_green);
	debug_log("Propulsion sensor i2c start\n");
	//get devices
	i2c_engine_press = i2c_sensor_get_engine_pressure();

	//init
	util_error_t engine_press_err = engine_pressure_init(i2c_engine_press);

	uint16_t checkpoint_press;
	if(engine_press_err == ER_SUCCESS) {
		checkpoint_press = led_add_checkpoint(led_green);
	} else {
		checkpoint_press = led_add_checkpoint(led_red);
	}
	//manual calibration only:
	i2c_calib = 0;

	//mainloop
	for(;;) {
		led_checkpoint(checkpoint);
		led_checkpoint(checkpoint_press);


		if(1) {
            if(engine_press_err == ER_SUCCESS) {
                //read engine pressure
				engine_pressure_read(i2c_engine_press, &i2c_engine_press_data);
			}
			//store everything

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

/* END */
