/**
 * @file 		device.c
 * @brief 		Generic device driver
 *
 * @date 		20.03.2022
 * @author 		Iacopo Sprenger
 *
 * @defgroup 	device Device
 * @{
 */

/**********************
 *	INCLUDES
 **********************/

#include <cmsis_os.h>
#include <util.h>

#include "device.h"


/**********************
 *	CONSTANTS
 **********************/

#define LEN_32 4
#define LEN_16 2
#define LEN_8  1 

/**********************
 *	MACROS
 **********************/


/**********************
 *	TYPEDEFS
 **********************/





/**********************
 *	VARIABLES
 **********************/


/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/

/**
 * @brief 	Initialize a device instance
 * @details This function fills in the fields of a device object. The purpose
 * 			of the context is to provide the read_reg and write_reg some arbitrary
 * 			context on the device that is being accessed.
 * 			The read_reg and write_reg functions should be written by the user
 * 			and tailored the the device. The arguments given by the system are
 * 			the device context, the associated interface, a 32 bits address,
 * 			the data to be written/read and the length of the data
 *
 * @param	dev			Pointer to the @p device_t structure describing this device.
 * @param   context		Generic pointer to a device context.
 * @param	interface	Pointer to the @p device_interface_t associated with this device.
 * @param	read_reg	Pointer to a read register function for this device.
 * @param	write_reg	Pointer to a write register function for this device.
 */
util_error_t device_create(	device_t * dev,
						void * context,
						device_interface_t * interface,
						util_error_t (*read_reg)(void*, device_interface_t *, uint32_t, uint8_t *, uint32_t),
						util_error_t (*write_reg)(void*, device_interface_t *, uint32_t, uint8_t *, uint32_t))
{
	static int32_t count = 0;
	dev->context = context;
	dev->interface = interface;
	dev->read_reg = read_reg;
	dev->write_reg = write_reg;
	dev->id = count++;

	return ER_SUCCESS;
}

/**
 * @brief	Initialize a device interface instance.
 * @details This function fills the fields of a device interface object. The purpose
 * 			of the context is to provide the send, recv and handle data with some
 * 			arbitrary context on the device interface that is being accessed.
 * 			The daemon
 *
 * @param	interface	Pointer to the device_interface_t object describing the interface.
 * @param 	context		Generic pointer to a device interface context.
 * @param	daemon		Pointer to the daemon associated with this interface.
 * 						Can be null.
 * @param	send		Interface specific send function.
 * @param	recv		Interface specific recv function.
 * @param 	handle_data Interface specific handle data function.
 */
util_error_t device_interface_create(   device_interface_t * interface,
                            			void * context,
										void * daemon,
										util_error_t (*send)(void*, uint8_t*, uint32_t),
										util_error_t (*recv)(void*, uint8_t*, uint32_t*),
										util_error_t (*handle_data)(void*, void*))
{
	UNUSED(daemon);
    static int32_t count = 0;
    interface->context = context;
    interface->send = send;
    interface->recv = recv;
    interface->handle_data = handle_data;
    interface->id = count++;
    return ER_SUCCESS;
}

util_error_t device_interface_register_handle_data(	device_interface_t * interface,
													util_error_t (*handle_data)(void*, void*)) {
	interface->handle_data = handle_data;
	return ER_SUCCESS;
}

//interface send/recv functions

/**
 * @brief Send raw data through the interface specific send function.
 * @details
 *
 * @param	interface	The interface through which data should be sent.
 * @param 	data		A point to the data to be sent.
 * @param	len			The length of the data to be sent.
 */
util_error_t device_interface_send(device_interface_t * interface, uint8_t * data, uint32_t len)
{
	if(interface->send) {
		return interface->send(interface->context, data, len);
	} else {
		return ER_RESSOURCE_ERROR;
	}
}

/**
 * @brief Receive raw data through the interface specific recv function.
 * @details
 *
 * @param	interface	The interface through which data should be received.
 * @param 	data		A point to the data to be received.
 * @param	len			A pointer to the length of the data to be received.
 */
util_error_t device_interface_recv(device_interface_t * interface, uint8_t * data, uint32_t * len)
{
	if(interface->send) {
		return interface->recv(interface->context, data, len);
	} else {
		return ER_RESSOURCE_ERROR;
	}
}

// device write functions

/**
 * @brief	Write an int32_t to a device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	The data to be written.
 */
util_error_t device_write_i32(device_t * dev, uint32_t addr, int32_t data)
{
    uint8_t tmp[LEN_32];
    util_encode_i32(tmp, data);
    dev->write_reg(dev->context, dev->interface, addr, tmp, LEN_32);
    return ER_SUCCESS;
}

/**
 * @brief	Write an uint32_t to a device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	The data to be written.
 */
util_error_t device_write_u32(device_t * dev, uint32_t addr, uint32_t data)
{
    uint8_t tmp[LEN_32];
    util_encode_u32(tmp, data);
    dev->write_reg(dev->context, dev->interface, addr, tmp, LEN_32);
    return ER_SUCCESS;
}

/**
 * @brief	Write an int16_t to a device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	The data to be written.
 */
util_error_t device_write_i16(device_t * dev, uint32_t addr, int16_t data)
{
    uint8_t tmp[LEN_16];
    util_encode_i16(tmp, data);
    dev->write_reg(dev->context, dev->interface, addr, tmp, LEN_16);
    return ER_SUCCESS;
}

/**
 * @brief	Write an uint16_t to a device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	The data to be written.
 */
util_error_t device_write_u16(device_t * dev, uint32_t addr, uint16_t data)
{
    uint8_t tmp[LEN_16];
    util_encode_u16(tmp, data);
    dev->write_reg(dev->context, dev->interface, addr, tmp, LEN_16);
    return ER_SUCCESS;
}

/**
 * @brief	Write an int8_t to a device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	The data to be written.
 */
util_error_t device_write_i8(device_t * dev, uint32_t addr, int8_t data)
{
    uint8_t tmp[LEN_8];
    util_encode_i8(tmp, data);
    dev->write_reg(dev->context, dev->interface, addr, tmp, LEN_8);
    return ER_SUCCESS;
}

/**
 * @brief	Write an uint8_t to a device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	The data to be written.
 */
util_error_t device_write_u8(device_t * dev, uint32_t addr, uint8_t data)
{
    uint8_t tmp[LEN_8];
    util_encode_u8(tmp, data);
    dev->write_reg(dev->context, dev->interface, addr, tmp, LEN_8);
    return ER_SUCCESS;
}

/**
 * @brief	Read from an int32_t device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	A pointer to the data to be read.
 */
util_error_t device_read_i32(device_t * dev, uint32_t addr, int32_t* data)
{
    uint8_t tmp[LEN_32];
    dev->read_reg(dev->context, dev->interface, addr, tmp, LEN_32);
    *data = util_decode_i32(tmp);
    return ER_SUCCESS;
}

/**
 * @brief	Read from an uint32_t device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	A pointer to the data to be read.
 */
util_error_t device_read_u32(device_t * dev, uint32_t addr, uint32_t* data)
{
    uint8_t tmp[LEN_32];
    dev->read_reg(dev->context, dev->interface, addr, tmp, LEN_32);
    *data = util_decode_u32(tmp);
    return ER_SUCCESS;
}

/**
 * @brief	Read from an int16_t device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	A pointer to the data to be read.
 */
util_error_t device_read_i16(device_t * dev, uint32_t addr, int16_t* data)
{
    uint8_t tmp[LEN_16];
    dev->read_reg(dev->context, dev->interface, addr, tmp, LEN_16);
    *data = util_decode_i16(tmp);
    return ER_SUCCESS;
}

/**
 * @brief	Read from an uint16_t device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	A pointer to the data to be read.
 */
util_error_t device_read_u16(device_t * dev, uint32_t addr, uint16_t* data)
{
    uint8_t tmp[LEN_16];
    dev->read_reg(dev->context, dev->interface, addr, tmp, LEN_16);
    *data = util_decode_u16(tmp);
    return ER_SUCCESS;
}

/**
 * @brief	Read from an int8_t device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	A pointer to the data to be read.
 */
util_error_t device_read_i8(device_t * dev, uint32_t addr, int8_t* data)
{
    uint8_t tmp[LEN_8];
    dev->read_reg(dev->context, dev->interface, addr, tmp, LEN_8);
    *data = util_decode_i8(tmp);
    return ER_SUCCESS;
}

/**
 * @brief	Read from an uint8_t device register.
 * @details
 *
 * @param	dev		A pointer to the desired device.
 * @param	addr	The address of the register.
 * @param	data	A pointer to the data to be read.
 */
util_error_t device_read_u8(device_t * dev, uint32_t addr, uint8_t* data)
{
    uint8_t tmp[LEN_8];
    dev->read_reg(dev->context, dev->interface, addr, tmp, LEN_8);
    *data = util_decode_u8(tmp);
    return ER_SUCCESS;
}

util_error_t device_read(device_t * dev, uint32_t addr, uint8_t* data, uint32_t len) {
	return dev->read_reg(dev->context, dev->interface, addr, data, len);
}

util_error_t device_write(device_t * dev, uint32_t addr, uint8_t* data, uint32_t len) {
	return dev->write_reg(dev->context, dev->interface, addr, data, len);

}

/** @} */
/* END */
