//
// Created by iacopo on 8/5/22.
//

#ifndef HOSTPROC_APP_STRUCTURES_H
#define HOSTPROC_APP_STRUCTURES_H


#include <stdint.h>


/*
 * feedback is plaintext
*/

typedef struct transfer_sync {
    int32_t i2c_acc[3];
    int32_t i2c_gyro[3];
    int32_t i2c_baro[2];
    uint32_t i2c_time;

    int32_t spi_acc[3];
    int32_t spi_gyro[3];
    int32_t spi_baro[2];
    uint32_t spi_time;

    int32_t bat_voltage[2];
    uint32_t adc_time;

    float gnss_lat;
    float gnss_lon;
    float gnss_sat_time;
    uint32_t gnss_time;

}transfer_sync_t;





typedef enum transfer_data_type {
    TRANSFER_DATA_ACC,
    TRANSFER_DATA_BARO,
    TRANSFER_DATA_GNSS
}transfer_data_type_t;

//we only send vertical data
// -> low latency for kalman filter
typedef struct transfer_data {
    transfer_data_type_t type;
    int32_t data;
    uint32_t time;
}transfer_data_t;

typedef struct transfer_cmd {

}transfer_cmd_t;





#endif //HOSTPROC_APP_STRUCTURES_H
