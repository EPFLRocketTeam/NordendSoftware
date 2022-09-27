/*  Title		: spi_sensor
 *  Filename	: spi_sensor.c
 *	Author		: iacopo sprenger
 *	Date		: 22.04.2022
 *	Version		: 0.1
 *	Description	: device driver for spi sensors
 *
 *	@note  	We use semaphores to create a "blocking function" which does not
 *			slow doen the RTOS with useless polling.
 */

/**********************
 *	INCLUDES
 **********************/
#include <main.h>
#include <device/device.h>
#include <device/spi_sensor.h>
#include <driver/spi.h>
#include <util.h>
#include <string.h>
/**********************
 *	CONSTANTS
 **********************/

#define TEMP_DATA_LEN	32



/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/




/**********************
 *	VARIABLES
 **********************/




static device_t spi_accelerometer_device;
static device_t spi_gyroscope_device;
static device_t spi_barometer_device;

static spi_sensor_context_t spi_accelerometer_device_context;
static spi_sensor_context_t spi_gyroscope_device_context;
static spi_sensor_context_t spi_barometer_device_context;

/**********************
 *	PROTOTYPES
 **********************/

util_error_t spi_sensor_read_reg(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len);
util_error_t spi_sensor_write_reg(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len);

util_error_t spi_sensor_read_reg_HAL(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len);
util_error_t spi_sensor_write_reg_HAL(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len);

/**********************
 *	DECLARATIONS
 **********************/

device_t * spi_sensor_get_accelerometer(void) {
	return &spi_accelerometer_device;
}

device_t * spi_sensor_get_gyroscope(void) {
	return &spi_gyroscope_device;
}

device_t * spi_sensor_get_barometer(void) {
	return &spi_barometer_device;
}

util_error_t spi_sensor_init(void) {

	device_interface_t * spi_sensor_interface = spi_get_sensor_interface();

	device_create((void*) &spi_accelerometer_device, &spi_accelerometer_device_context, spi_sensor_interface, spi_sensor_read_reg, spi_sensor_write_reg);
	device_create((void*) &spi_gyroscope_device, &spi_gyroscope_device_context, spi_sensor_interface, spi_sensor_read_reg, spi_sensor_write_reg);
	device_create((void*) &spi_barometer_device, &spi_barometer_device_context, spi_sensor_interface, spi_sensor_read_reg, spi_sensor_write_reg);

	return ER_SUCCESS;

}



/**
 * @brief Function used to receive register data from a sensor
 *
 * @param context	Pointer to a device context
 * @param interface	Pointer to the interface associated with the device
 * @param addr		Register address
 * @param data		Data to read in the register
 * @param len		Length of the data to be read
 *
 */
util_error_t spi_sensor_read_reg(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len) {
	UNUSED(context);
	util_error_t error = ER_SUCCESS;
	//send address
	//receive data
	//TODO: Setup SPI transfer Type ??
	//		This should be done with an SPI TAKE function -> to take
	//		ownership of the bus (and assert the ChipSelect)
	error |= interface->send(interface->context, (uint8_t *) &addr, 1);
	error |= interface->recv(interface->context, data, &len);
	return error;
}

/**
 * @brief Function used to send register data to a sensor
 *
 * @param context	Pointer to a device context
 * @param interface	Pointer to the interface associated with the device
 * @param addr		Register address
 * @param data		Data to write in the register
 * @param len		Length of the data to be written
 *
 */
util_error_t spi_sensor_write_reg(void* context, device_interface_t * interface, uint32_t addr, uint8_t * data, uint32_t len) {
	spi_sensor_context_t * ctx = (spi_sensor_context_t *) context;
	UNUSED(ctx);
	util_error_t error = ER_SUCCESS;
	//send address
	//send data
	//TODO: Setup SPI transfer Type ??
	//		This should be done with an SPI TAKE function -> to take
	//		ownership of the bus (and assert the ChipSelect)
	error |= interface->send(interface->context, (uint8_t *) &addr, 1);
	error |= interface->send(interface->context, data, len);
	return error;
}



/* END */
