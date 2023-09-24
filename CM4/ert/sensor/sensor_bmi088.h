#ifndef SENSOR_BMI088_H
#define SENSOR_BMI088_H

#include <util.h>
#include <device/device.h>
#include <od/data_types.h>


#define BMI088_REG_ACC_SOFTRST						(0x7e)
#define BMI088_REG_ACC_PWRCTRL						(0x7d)
#define BMI088_REG_ACC_PWRCONF						(0x6d)
#define BMI088_REG_ACC_RANGE						(0x41)
#define BMI088_REG_ACC_CONF 						(0x40)
#define BMI088_REG_ACC_Z_H   						(0x17)
#define BMI088_REG_ACC_Z_L							(0x16)
#define BMI088_REG_ACC_Y_H							(0x15)
#define BMI088_REG_ACC_Y_L							(0x14)
#define BMI088_REG_ACC_X_H							(0x13)
#define BMI088_REG_ACC_X_L							(0x12)
#define BMI088_REG_ACC_STATUS						(0x02)
#define BMI088_REG_ACC_ERROR						(0x01)
#define BMI088_REG_ACC_CHIPID						(0x00)

#define BMI088_ACC_CHIPID_VAL						(0x1e)

#define BMI088_REG_GYR_SOFTRST						(0x14)
#define BMI088_REG_GYR_LPM1						    (0x11)
#define BMI088_REG_GYR_BW						    (0x10)
#define BMI088_REG_GYR_RANGE					    (0x0f)
#define BMI088_REG_GYR_Z_H   						(0x07)
#define BMI088_REG_GYR_Z_L							(0x06)
#define BMI088_REG_GYR_Y_H							(0x05)
#define BMI088_REG_GYR_Y_L							(0x04)
#define BMI088_REG_GYR_X_H							(0x03)
#define BMI088_REG_GYR_X_L							(0x02)
#define BMI088_REG_GYR_CHIPID						(0x00)

#define BMI088_GYR_CHIPID_VAL						(0x0f)


typedef struct bmi088_acc_context {
	uint8_t hw_available;
	uint16_t selected_range;
	float selected_sstvt;
}bmi088_acc_context_t;

typedef struct bmi088_gyr_context {
	uint8_t hw_available;
	uint16_t selected_range;
	float selected_sstvt;
}bmi088_gyr_context_t;



util_error_t bmi088_acc_init(device_t * dev, bmi088_acc_context_t * ctx);
util_error_t bmi088_gyr_init(device_t * dev, bmi088_gyr_context_t * ctx);

util_error_t bmi088_acc_read(device_t * dev, sensor_imu_data_t * data);
util_error_t bmi088_gyr_read(device_t * dev, sensor_imu_data_t * data);

uint8_t bmi088_acc_is_available(device_t * dev);
uint8_t bmi088_gyr_is_available(device_t * dev);


#endif /* SENSOR_BMI088_H */
