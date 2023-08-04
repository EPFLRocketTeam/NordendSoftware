/*  Title		: magnetometer.c
*  Filename	: magnetometer.c
*	Author		: Julian Ruiz Rodriguez
*	Date			: 28.10.2022
*	Version	: 0.1
*/

#include "magnetometer.h"

#define WHO_AM_I        0x4F
#define WHO_AM_I_MAGIC  0x40
#define CFG_REG_A       0x60
#define CFG_REG_B       0x61
#define CFG_REG_C       0x62

#define STATUS_REG      0x67

#define OUTX_L_REG      0x68
#define OUTX_H_REG      0x69
#define OUTY_L_REG      0x6A
#define OUTY_H_REG      0x6B
#define OUTZ_L_REG      0x6C
#define OUTZ_H_REG      0x6D
#define TEMP_OUT_L_REG  0x6E
#define TEMP_OUT_H_REG  0x6F

static enum CONFIG_MODE INIT_MODE = NO_T_COMPENSATION_NO_REBOOT;

util_error_t magnetometer_init(device_t * magneto) {
    util_error_t error = ER_SUCCESS;
    uint8_t data; //read sensor magic number
    error |= device_read_u8(magneto, WHO_AM_I, &data);

    if(data != WHO_AM_I_MAGIC) {
        return error | ER_RESSOURCE_ERROR;
    }

    //initialize sensor data rate -> 100Hz
    switch (INIT_MODE) {
        case NO_T_COMPENSATION_NO_REBOOT:
            error |= device_write_u8(magneto, CFG_REG_A, 0b00001111);
            break;

        case T_COMPENSATION_NO_REBOOT:
            error |= device_write_u8(magneto, CFG_REG_A, 0b10001111);
            break;

        case T_COMPENSATION_REBOOT:
            error |= device_write_u8(magneto, CFG_REG_A, 0b11001111);
            break;

        default:
            break;
    }

    return error;
}

util_error_t magnetometer_read_X_axis(device_t * magneto, magnetometer_data_t * meta) {
    uint8_t data[2];
    util_error_t error = device_read(magneto, OUTX_L_REG, data, 2);

    meta->magneto_x_axis = data[1] << 8 | data[0];

    return error;
}

util_error_t magnetometer_read_Y_axis(device_t * magneto, magnetometer_data_t * meta) {
    uint8_t data[2];
    util_error_t error = device_read(magneto, OUTY_L_REG, data, 2);

    meta->magneto_y_axis = data[1] << 8 | data[0];

    return error;
}

util_error_t magnetometer_read_Z_axis(device_t * magneto, magnetometer_data_t * meta) {
    uint8_t data[2];
    util_error_t error = device_read(magneto, OUTZ_L_REG, data, 2);

    meta->magneto_z_axis = data[1] << 8 | data[0];

    return error;
}

bool new_set_of_data(device_t * magneto) {
    uint8_t infoSet;

    util_error_t error = device_read(magneto, STATUS_REG, &infoSet, 1);
    // Error
    if (error) return false;

    if ((infoSet & 00001000) == 00001000) {
        return true;
    } else {
        return false;
    }
}
