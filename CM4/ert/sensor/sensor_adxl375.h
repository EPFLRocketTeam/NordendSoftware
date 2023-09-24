#ifndef SENSOR_ADXL375_H
#define SENSOR_ADXL375_H


#include <util.h>
#include <device/device.h>

#define ADXL375_REG_DEVID				(0x00)
#define ADXL375_REG_THS_SHK				(0x1D)
#define ADXL375_REG_OFSX				(0x1E)
#define ADXL375_REG_OFSY				(0x1F)
#define ADXL375_REG_OFSZ				(0x20)
#define ADXL375_REG_DUR					(0x21)
#define ADXL375_REG_BWRATE				(0x2C)
#define ADXL375_REG_PWRCTL				(0x2D)
#define ADXL375_REG_DATAFMT				(0x31)
#define ADXL375_REG_DATAX0				(0x32)
#define ADXL375_REG_DATAX1				(0x33)
#define ADXL375_REG_DATAY0				(0x34)
#define ADXL375_REG_DATAY1				(0x35)
#define ADXL375_REG_DATAZ0				(0x36)
#define ADXL375_REG_DATAZ1				(0x37)


#define ADXL375_DEVID_VAL				(0b11100101)


typedef struct adxl375_acc_context {
	uint8_t hw_available;

}adxl375_acc_context_t;


util_error_t adxl375_acc_init(device_t * dev, adxl375_acc_context_t ctx);

util_error_t adxl375_acc_read(device_t * dev, sensor_acc_data_t * data);



#endif /* SENSOR_ADXL375_H */
