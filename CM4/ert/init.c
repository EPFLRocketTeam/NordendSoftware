/*  Title		: Threads
 *  Filename	: threads.c
 *	Author		: iacopo sprenger
 *	Date		: 20.02.2022
 *	Version		: 0.1
 *	Description	: template for code files
 */

/**********************
 *	INCLUDES
 **********************/


#include <cmsis_os.h>
#include <wildhorn.h>
#include <feedback/led.h>
#include <feedback/buzzer.h>
#include <driver/serial.h>
#include <driver/i2c.h>
#include <device/i2c_sensor.h>
#include <control.h>
#include <device/hostproc.h>
#include <init.h>
#include <sensor.h>
#include <od/od.h>


/**********************
 *	CONSTANTS
 **********************/

#define DEFAULT_SZ	(1024)

#define OD_SZ           DEFAULT_SZ
#define OD_PRIO         (6)

#define CONTROL_SZ		DEFAULT_SZ
#define CONTROL_PRIO	(6)

#define LED_RGB_SZ		DEFAULT_SZ
#define LED_RGB_PRIO	(1)

#define SENSOR_I2C_SZ	DEFAULT_SZ
#define SENSOR_I2C_PRIO	(6)


/**********************
 *	MACROS
 **********************/



/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/

static TaskHandle_t od_handle = NULL;
static TaskHandle_t control_handle = NULL;
static TaskHandle_t led_rgb_handle = NULL;
static TaskHandle_t sensor_i2c_handle = NULL;

/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/

/**
 * @brief	Initialize all the threads of Wildhorn AV
 * @details	This is the only function that needs to be called from the ST
 * 			Auto-generated files. This is clever in case the autogeneration
 * 			fails. This will minimize the code to be rewritten.
 *
 */
void init(void) {

	//initialize serial
	serial_init();

	//initialize hostproc comm
	hostproc_init();

	// initialize object dictionary
	od_init();

#if WH_HAS_FEEDBACK == WH_TRUE
#if WH_USE_BUZZER == WH_TRUE
	buzzer_init();
#endif
	led_feedback_init();
#endif

#if WH_HAS_SENSORS == WH_TRUE
	i2c_spi_guard();
	i2c_init();
	i2c_sensor_init();
#endif


	INIT_THREAD_CREATE(od_handle, od, od_update_task, NULL, OD_SZ, OD_PRIO);


	INIT_THREAD_CREATE(led_rgb_handle, led_rgb, led_rgb_thread, NULL, LED_RGB_SZ, LED_RGB_PRIO);


	INIT_THREAD_CREATE(control_handle, control, control_thread, NULL, CONTROL_SZ, CONTROL_PRIO);

#if WH_HAS_SENSORS == WH_TRUE
	INIT_THREAD_CREATE(sensor_i2c_handle, sensor_i2c, sensor_i2c_thread, NULL, SENSOR_I2C_SZ, SENSOR_I2C_PRIO);
#endif

}


/* END */
