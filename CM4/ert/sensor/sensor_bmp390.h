#ifndef SENSOR_BMP390_H
#define SENSOR_BMP390_H

#include <util.h>
#include <device/device.h>
#include <od/data_types.h>



#define BMP390_REG_BARO_CMD					(0x7e)
#define BMP390_REG_BARO_CONFIG				(0x1f)
#define BMP390_REG_BARO_ODR				    (0x1d)
#define BMP390_REG_BARO_OSR					(0x1c)
#define BMP390_REG_BARO_PWR_CTRL			(0x1b)
#define BMP390_REG_BARO_IF_CONF				(0x1a)
#define BMP390_REG_BARO_DATA5				(0x09)
#define BMP390_REG_BARO_DATA4				(0x08)
#define BMP390_REG_BARO_DATA3				(0x07)
#define BMP390_REG_BARO_DATA2				(0x06)
#define BMP390_REG_BARO_DATA1				(0x05)
#define BMP390_REG_BARO_DATA0				(0x04)
#define BMP390_REG_BARO_STATUS				(0x03)
#define BMP390_REG_BARO_ERROR				(0x02)
#define BMP390_REG_BARO_REV_ID				(0x01)
#define BMP390_REG_BARO_CHIPID				(0x00)

#define BMP390_BARO_CHIPID_VAL				(0x60)




typedef struct bmp390_baro_context {
	uint8_t hw_available;
	uint16_t selected_range;
	float selected_sstvt;
}bmp390_baro_context_t;




util_error_t bmp390_baro_init(device_t * dev, bmp390_baro_context_t * ctx);

util_error_t bmp390_baro_read(device_t * dev, sensor_baro_data_t * data);

uint8_t bmp390_baro_is_available(device_t * dev);



#endif /* SENSOR_BMP390_H */
