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
#include <nordend.h>
#include <feedback/led.h>
#include <feedback/buzzer.h>
#include <driver/serial.h>
#include <driver/i2c.h>
#include <driver/spi.h>
#include <driver/can.h>
#include <device/i2c_sensor.h>
#include <device/spi_sensor.h>
#include <driver/hostproc.h>
#include <init.h>
#include <sensor.h>

#include <propulsion_sensor.h>

#include <od/od.h>
#include <device/comunicator.h>
#include <engine_control.h>
#include <hostcom.h>
#include <sensor/gnss.h>
#include <propulsion/servo.h>

#include <feedback/debug.h>
#include <miaou_downlink.h>
#include <miaou_uplink.h>


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


#define SERIAL_SZ	DEFAULT_SZ
#define SERIAL_PRIO		(6)

#define HOSTCOM_SZ		DEFAULT_SZ
#define HOSTCOM_PRIO		(6)

#define SERVO_SZ 	DEFAULT_SZ
#define SERVO_PRIO 	(1)

#define MIAOU_SZ	DEFAULT_SZ
#define MIAOU_PRIO		(4)


#define SENSOR_SZ	DEFAULT_SZ
#define SENSOR_PRIO		(3)

#define PROP_SENSOR_SZ	DEFAULT_SZ
#define PROP_SENSOR_PRIO		(1)

#define CAN_RX_SZ	DEFAULT_SZ
#define CAN_RX_PRIO		(3)

#define CAN_TX_SZ	DEFAULT_SZ
#define CAN_TX_PRIO		(3)


/**********************
 *	MACROS
 **********************/



/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/

static TaskHandle_t od_update_handle = NULL;
static TaskHandle_t od_broadcast_handle = NULL;
static TaskHandle_t engine_control_handle = NULL;
static TaskHandle_t led_rgb_handle = NULL;
static TaskHandle_t serial_handle = NULL;
static TaskHandle_t hostcom_handle = NULL;
static TaskHandle_t miaou_handle = NULL;
static TaskHandle_t can_rx_handle = NULL;
static TaskHandle_t can_tx_handle = NULL;
static TaskHandle_t sensor_handle = NULL;

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
	// A + B
	serial_init();

	//initialize hostproc comm
	// A + B
	hostproc_init();

	// initialize object dictionary
	// A + B
	od_init();

	// initialize the can drivers
	// A + B
	can_init(ND_COMPUTER);


#if ND_HAS_SENSORS == ND_TRUE
	/**
	 * Some stuff is wrong in how they handled the sensors.
	 * They initialize only one bus...
	 * We have to init all the used busses and then init the sensors located on those busses.
	 */
	//init the sensors

	//init the i2c s2 interface for the sensors
	i2c_s2_init();

	//create the sensors devices
	i2c_sensor_init();


	INIT_THREAD_CREATE(sensor_handle, sensor, sensor_i2c_thread, NULL, SENSOR_SZ, SENSOR_PRIO);

#endif

	//Object dictionnary threads
	INIT_THREAD_CREATE(od_update_handle, od_update, od_update_task, NULL, OD_SZ, OD_PRIO);

	INIT_THREAD_CREATE(od_broadcast_handle, od_broadcast, od_broadcast_task, NULL, OD_SZ, OD_PRIO);

	// Object dictionnary transmission and reception threads
	INIT_THREAD_CREATE(can_rx_handle, can_rx, can_receive_thread, NULL, CAN_TX_SZ, CAN_TX_PRIO);

	//this is handled by the od broadcast thread
	//INIT_THREAD_CREATE(can_tx_handle, can_tx, can_transmit_thread, NULL, CAN_RX_SZ, CAN_RX_PRIO);
	UNUSED(can_tx_handle);

	// RGB Led control thread
	INIT_THREAD_CREATE(led_rgb_handle, led_rgb, led_rgb_thread, NULL, LED_RGB_SZ, LED_RGB_PRIO);


#if ND_HAS_PROPULSION == ND_TRUE
	INIT_THREAD_CREATE(engine_control_handle, engine_control, engine_control_thread, NULL, CONTROL_SZ, CONTROL_PRIO);
#else
	UNUSED(engine_control_handle);
#endif

#if ND_HAS_RECOVERY == ND_TRUE
	//HERE PLACE THE RECOVERY THREAD INIT @CHARLOTTE
#endif

#if ND_HAS_UPLINK == ND_TRUE
	miaou_uplink_init();
#endif

#if ND_HAS_DOWNLINK == ND_TRUE
	INIT_THREAD_CREATE(miaou_handle, miaou_downlink, miaou_downlink_thread, NULL, MIAOU_SZ, MIAOU_PRIO);
#else
	UNUSED(miaou_handle);
#endif

	INIT_THREAD_CREATE(serial_handle, serial, serial_thread, NULL, SERIAL_SZ, SERIAL_PRIO);

	INIT_THREAD_CREATE(hostcom_handle, hostcom, hostcom_thread, NULL, HOSTCOM_SZ, HOSTCOM_PRIO);









}


/* END */
