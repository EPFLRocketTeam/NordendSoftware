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
#include <driver/spi.h>
#include <device/i2c_sensor.h>
#include <device/spi_sensor.h>
#include <control.h>
#include <driver/hostproc.h>
#include <init.h>
#include <sensor.h>
#include <od/od.h>
#include <device/comunicator.h>
#include <miaou.h>

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


#define COMUNICATOR_SZ	DEFAULT_SZ
#define COMUNICATOR_PRIO		(6)


#define MIAOU_SZ	DEFAULT_SZ
#define MIAOU_PRIO		(6)


#define SENSOR_SZ	DEFAULT_SZ
#define SENSOR_PRIO		(6)


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
static TaskHandle_t communicator_handle = NULL;
static TaskHandle_t miaou_handle = NULL;

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
	//spi_init();
	i2c_init();
	//spi_sensor_init();
	i2c_sensor_init();
#endif


	INIT_THREAD_CREATE(od_handle, od, od_update_task, NULL, OD_SZ, OD_PRIO);

	INIT_THREAD_CREATE(led_rgb_handle, led_rgb, led_rgb_thread, NULL, LED_RGB_SZ, LED_RGB_PRIO);


#if WH_HAS_KRTEK
	INIT_THREAD_CREATE(control_handle, control, control_thread, NULL, CONTROL_SZ, CONTROL_PRIO);
#endif

	INIT_THREAD_CREATE(communicator_handle, comunicator, comunicator_thread, NULL, COMUNICATOR_SZ, COMUNICATOR_PRIO);

#if WH_HAS_RADIO
	INIT_THREAD_CREATE(miaou_handle, miaou, miaou_thread, NULL, MIAOU_SZ, MIAOU_PRIO);
#endif


#if WH_HAS_SENSORS
	INIT_THREAD_CREATE(sensor_i2c_handle, sensor_i2c, sensor_i2c_thread, NULL, SENSOR_SZ, SENSOR_PRIO);
	//INIT_THREAD_CREATE(sensor_spi_handle, sensor_spi, sensor_spi_thread, NULL, SENSOR_SZ, SENSOR_PRIO);
#endif

#if WH_HAS_GNSS

#endif

}


/* END */
