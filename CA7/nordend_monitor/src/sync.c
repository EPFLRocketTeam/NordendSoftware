/**
 * Sync is used to log and process data messages
 */


#include <stdlib.h>
#include <stdio.h>

#include <unistd.h>
#include <string.h>
#include <stdint.h>


#include "serial.h"
#include "sync.h"


#define NO_PACKET_SZ_CHECK


#define KALMAN_DATA_A 			    1
#define KALMAN_DATA_B 			    2
#define GNSS_DATA_A				    3
#define GNSS_DATA_B	 			    4
#define BATTERY_A 				    5
#define BATTERY_B 				    6
#define ENGINE_CONTROL_DATA 	    7
#define RECOVERY_CONTROL_DATA 	    8
#define SENSOR_BARO_A 			    9
#define SENSOR_BARO_B 			    10
#define SENSOR_IMU_A 			    11
#define SENSOR_IMU_B 			    12
#define SENSOR_MAG_A 			    13
#define SENSOR_MAG_B 			    14
#define SENSOR_ACC_A 			    15
#define SENSOR_ACC_B 			    16
#define ENGINE_SENSORS_DATA 	    17




static sync_store_t data;



static FILE * fp = NULL;

void sync_copy_data(sync_store_t * _data) {
	memcpy(_data, &data, sizeof(sync_store_t));
}



void sync_handle_gnss(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(gnss_data_t)) {
		if(opcode == GNSS_DATA_A) {
			memcpy(&data.gnss_data_a, _data, sizeof(gnss_data_t));
		} else {
			memcpy(&data.gnss_data_b, _data, sizeof(gnss_data_t));
		}
	}
}

void sync_handle_battery(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(battery_data_t)) {
		if(opcode == BATTERY_A) {
			memcpy(&data.battery_a, _data, sizeof(battery_data_t));
		} else {
			memcpy(&data.battery_b, _data, sizeof(battery_data_t));
		}
	}
}

void sync_handle_engine_control(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(engine_control_data_t)) {
		memcpy(&data.engine_control, _data, sizeof(engine_control_data_t));
		fprintf(fp, "ENGINE_CONTROL,%d,%d,%d,%ld\n",
					data.engine_control.state,
					data.engine_control.last_cmd,
					data.engine_control.last_parameter,
					data.engine_control.time);
	}
}


void sync_handle_recovery_control(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(recovery_control_data_t)) {
		memcpy(&data.recovery_control, _data, sizeof(recovery_control_data_t));
	}
}

void sync_handle_sensor_baro(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(sensor_baro_data_t)) {
		if(opcode == SENSOR_BARO_A) {
			memcpy(&data.baro_a, _data, sizeof(sensor_baro_data_t));
		} else {
			memcpy(&data.baro_b, _data, sizeof(sensor_baro_data_t));
		}
	}
}

void sync_handle_sensor_imu(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(sensor_imu_data_t)) {
		if(opcode == SENSOR_IMU_A) {
			memcpy(&data.imu_a, _data, sizeof(sensor_imu_data_t));
		} else {
			memcpy(&data.imu_b, _data, sizeof(sensor_imu_data_t));
		}
	}
}

void sync_handle_sensor_mag(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(sensor_mag_data_t)) {
		if(opcode == SENSOR_MAG_A) {
			memcpy(&data.mag_a, _data, sizeof(sensor_mag_data_t));
		} else {
			memcpy(&data.mag_b, _data, sizeof(sensor_mag_data_t));
		}
	}
}

void sync_handle_sensor_acc(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(sensor_acc_data_t)) {
		if(opcode == SENSOR_ACC_A) {
			memcpy(&data.acc_a, _data, sizeof(sensor_acc_data_t));
		} else {
			memcpy(&data.acc_b, _data, sizeof(sensor_acc_data_t));
		}
	}
}

void sync_handle_sensor_eng(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(sensor_eng_data_t)) {
		memcpy(&data.sensor_eng, _data, sizeof(sensor_eng_data_t));
	}
}


void sync_handle_data(uint8_t opcode, uint16_t len, uint8_t * _data) {

	//printf("handling: %d | %d\n", opcode, len);
	fprintf(fp, "handling: %d, %d\n", opcode, len);
	switch(opcode) {
	case GNSS_DATA_A:
	case GNSS_DATA_B:
		sync_handle_gnss(opcode, len, _data);
		break;
	case BATTERY_A:
	case BATTERY_B:
		sync_handle_battery(opcode, len, _data);
		break;
	case ENGINE_CONTROL_DATA:
		sync_handle_engine_control(opcode, len, _data);
		break;
	case RECOVERY_CONTROL_DATA:
		sync_handle_recovery_control(opcode, len, _data);
		break;
	case SENSOR_BARO_A:
	case SENSOR_BARO_B:
		sync_handle_sensor_baro(opcode, len, _data);
		break;
	case SENSOR_IMU_A:
	case SENSOR_IMU_B:
		sync_handle_sensor_imu(opcode, len, _data);
		break;
	case SENSOR_MAG_A:
	case SENSOR_MAG_B:
		sync_handle_sensor_mag(opcode, len, _data);
		break;
	case SENSOR_ACC_A:
	case SENSOR_ACC_B:
		sync_handle_sensor_acc(opcode, len, _data);
		break;
	case ENGINE_SENSORS_DATA:
		sync_handle_sensor_eng(opcode, len, _data);
		break;
	default:
		break;
	}

}





void * sync_entry(void * ptr) {
    //initialization
    serial_dev_t sync_device;
    comunicator_t com;

    serial_setup(&sync_device, "/dev/ttyRPMSG1");


    comunicator_init(&com, &sync_device, sync_handle_data);

    //first message for setup
    static char * msg = "hello";
    serial_send(&sync_device, msg, 6);

    printf("setup sync channel\n");
    //handle new file creation
    //First check exitance of file -> then increment names until file does not exist
    //maybe use the mkdir and check error code.
    static char fname[64];
    static uint16_t num = 0;
    do{
    	snprintf(fname, 64, "/home/root/av_sync%d.log", num);
    	num++;
    }while((access(fname, F_OK) == 0));
    fp = fopen(fname, "w+");

    comunicator_recv(&com);


    return NULL;
}

