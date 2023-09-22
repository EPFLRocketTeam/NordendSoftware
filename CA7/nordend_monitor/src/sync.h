//
// Created by iacopo on 7/29/22.
//

#ifndef HOSTPROC_APP_SYNC_H
#define HOSTPROC_APP_SYNC_H

#include "../../../CM4/ert/od/data_types.h"

#include <stdint.h>


typedef struct sync_store {
	kalman_data_t kalman_data_a;
	kalman_data_t kalman_data_b;
	gnss_data_t gnss_data_a;
	gnss_data_t gnss_data_b;
	battery_data_t battery_a;
	battery_data_t battery_b;
	engine_control_data_t engine_control;
	recovery_control_data_t recovery_control;
	sensor_baro_data_t baro_a[2];
	sensor_baro_data_t baro_b[2];
	sensor_imu_data_t imu_a[2];
	sensor_imu_data_t imu_b[2];
	sensor_mag_data_t mag_a;
	sensor_mag_data_t mag_b;
	sensor_acc_data_t acc_a[2];
	sensor_acc_data_t acc_b[2];
	sensor_eng_data_t sensor_eng;

}sync_store_t;


#ifdef __cplusplus
extern "C"{
#endif

void sync_copy_data(sync_store_t * _data);

void *sync_entry(void *ptr);

#ifdef __cplusplus
} //extern c
#endif

#endif //HOSTPROC_APP_SYNC_H
