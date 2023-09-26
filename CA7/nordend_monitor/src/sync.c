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

#define OD_MATCH(NAME, ID) enum{ NAME = ID }

OD_MATCH(KALMAN_DATA_A, 			1  );
OD_MATCH(KALMAN_DATA_B, 			2  );
OD_MATCH(GNSS_DATA_A, 				3  );
OD_MATCH(GNSS_DATA_B, 				4  );
OD_MATCH(BATTERY_A, 				5  );
OD_MATCH(BATTERY_B, 				6  );
OD_MATCH(ENGINE_CONTROL_DATA, 		7  );
OD_MATCH(RECOVERY_CONTROL_DATA, 	8  );
OD_MATCH(SENSOR_BARO_A_0, 			10 );
OD_MATCH(SENSOR_BARO_A_1, 			11 );
OD_MATCH(SENSOR_BARO_B_0, 			12 );
OD_MATCH(SENSOR_BARO_B_1, 			13 );
OD_MATCH(SENSOR_IMU_A_0, 			20 );
OD_MATCH(SENSOR_IMU_A_1, 			21 );
OD_MATCH(SENSOR_IMU_B_0, 			22 );
OD_MATCH(SENSOR_IMU_B_1, 			23 );
OD_MATCH(SENSOR_MAG_A, 			    30 );
OD_MATCH(SENSOR_MAG_B, 			    31 );
OD_MATCH(SENSOR_ACC_A_0, 			40 );
OD_MATCH(SENSOR_ACC_A_1, 			41 );
OD_MATCH(SENSOR_ACC_B_0, 			42 );
OD_MATCH(SENSOR_ACC_B_1, 			43 );
OD_MATCH(ENGINE_SENSORS_DATA, 		50 );




static sync_store_t data;



static FILE * fp = NULL;

void sync_copy_data(sync_store_t * _data) {
	memcpy(_data, &data, sizeof(sync_store_t));
}

/*GNSS_DATA_x,lon,lat,alt,spd,hdop,time*/
void sync_handle_gnss(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(gnss_data_t)) {
		if(opcode == GNSS_DATA_A) {
			memcpy(&data.gnss_data_a, _data, sizeof(gnss_data_t));
			fprintf(fp, "GNSS_DATA_A,%g,%g,%g,%g,%g,%d\n",
					data.gnss_data_a.longitude,
					data.gnss_data_a.latitude,
					data.gnss_data_a.altitude,
					data.gnss_data_a.speed,
					data.gnss_data_a.hdop,
					data.gnss_data_a.time);
		} else {
			memcpy(&data.gnss_data_b, _data, sizeof(gnss_data_t));
			fprintf(fp, "GNSS_DATA_B,%g,%g,%g,%g,%g,%d\n",
					data.gnss_data_b.longitude,
					data.gnss_data_b.latitude,
					data.gnss_data_b.altitude,
					data.gnss_data_b.speed,
					data.gnss_data_b.hdop,
					data.gnss_data_b.time);
		}
	}
}

/* BATTERY_x,voltage,time*/
void sync_handle_battery(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(battery_data_t)) {
		if(opcode == BATTERY_A) {
			memcpy(&data.battery_a, _data, sizeof(battery_data_t));
			fprintf(fp, "BATTERY_A,%d,%d\n",
					data.battery_a.voltage,
					data.battery_a.time);
		} else {
			memcpy(&data.battery_b, _data, sizeof(battery_data_t));
			fprintf(fp, "BATTERY_B,%d,%d\n",
					data.battery_b.voltage,
					data.battery_b.time);
		}
	}
}

/*ENGINE_CONTROL,state,last_cmd,vent_eth,vent_n2o,press,purge,servo_eth,servo_n2o,time*/
void sync_handle_engine_control(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(engine_control_data_t)) {
		memcpy(&data.engine_control, _data, sizeof(engine_control_data_t));
		fprintf(fp, "ENGINE_CONTROL,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
					data.engine_control.state,
					data.engine_control.last_cmd,
					data.engine_control.vent_eth,
					data.engine_control.vent_n2o,
					data.engine_control.press,
					data.engine_control.purge,
					data.engine_control.servo_eth,
					data.engine_control.servo_n2o,
					data.engine_control.time);
	}
}

//UNUSED LOL
void sync_handle_recovery_control(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(recovery_control_data_t)) {
		memcpy(&data.recovery_control, _data, sizeof(recovery_control_data_t));
	}
}

/*SENSOR_BARO_x_x,pressure,temperature,altitude,time*/
void sync_handle_sensor_baro(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(sensor_baro_data_t)) {
		switch(opcode){
		case SENSOR_BARO_A_0:
			memcpy(&data.baro_a[0], _data, sizeof(sensor_baro_data_t));
			fprintf(fp, "SENSOR_BARO_A_0,%d,%d,%g,%d\n",
						data.baro_a[0].press,
						data.baro_a[0].temp,
						data.baro_a[0].alt,
						data.baro_a[0].time);
			break;
		case SENSOR_BARO_A_1:
			memcpy(&data.baro_a[1], _data, sizeof(sensor_baro_data_t));
			fprintf(fp, "SENSOR_BARO_A_1,%d,%d,%g,%d\n",
						data.baro_a[1].press,
						data.baro_a[1].temp,
						data.baro_a[1].alt,
						data.baro_a[1].time);
			break;
		case SENSOR_BARO_B_0:
			memcpy(&data.baro_b[0], _data, sizeof(sensor_baro_data_t));
			fprintf(fp, "SENSOR_BARO_B_0,%d,%d,%g,%d\n",
						data.baro_b[0].press,
						data.baro_b[0].temp,
						data.baro_b[0].alt,
						data.baro_b[0].time);
			break;
		case SENSOR_BARO_B_1:
			memcpy(&data.baro_b[1], _data, sizeof(sensor_baro_data_t));
			fprintf(fp, "SENSOR_BARO_B_1,%d,%d,%g,%d\n",
						data.baro_b[1].press,
						data.baro_b[1].temp,
						data.baro_b[1].alt,
						data.baro_b[1].time);
			break;
		}
	}
}

/*SENSOR_IMU_x_x,acc_x,acc_y,acc_z,acc_time,gyro_x,gyro_y,gyro_z,gyro_time*/
void sync_handle_sensor_imu(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(sensor_imu_data_t)) {
		switch(opcode){
		case SENSOR_IMU_A_0:
			memcpy(&data.imu_a[0], _data, sizeof(sensor_imu_data_t));
			fprintf(fp, "SENSOR_IMU_A_0,%g,%g,%g,%d,%g,%g,%g,%d\n",
						data.imu_a[0].acc[AXIS_X],
						data.imu_a[0].acc[AXIS_Y],
						data.imu_a[0].acc[AXIS_Z],
						data.imu_a[0].acc_time,
						data.imu_a[0].gyro[AXIS_X],
						data.imu_a[0].gyro[AXIS_Y],
						data.imu_a[0].gyro[AXIS_Z],
						data.imu_a[0].gyro_time);
			break;
		case SENSOR_IMU_A_1:
			memcpy(&data.imu_a[1], _data, sizeof(sensor_imu_data_t));
			fprintf(fp, "SENSOR_IMU_A_1,%g,%g,%g,%d,%g,%g,%g,%d\n",
						data.imu_a[1].acc[AXIS_X],
						data.imu_a[1].acc[AXIS_Y],
						data.imu_a[1].acc[AXIS_Z],
						data.imu_a[1].acc_time,
						data.imu_a[1].gyro[AXIS_X],
						data.imu_a[1].gyro[AXIS_Y],
						data.imu_a[1].gyro[AXIS_Z],
						data.imu_a[1].gyro_time);
			break;
		case SENSOR_IMU_B_0:
			memcpy(&data.imu_b[0], _data, sizeof(sensor_imu_data_t));
			fprintf(fp, "SENSOR_IMU_B_0,%g,%g,%g,%d,%g,%g,%g,%d\n",
						data.imu_b[0].acc[AXIS_X],
						data.imu_b[0].acc[AXIS_Y],
						data.imu_b[0].acc[AXIS_Z],
						data.imu_b[0].acc_time,
						data.imu_b[0].gyro[AXIS_X],
						data.imu_b[0].gyro[AXIS_Y],
						data.imu_b[0].gyro[AXIS_Z],
						data.imu_b[0].gyro_time);
			break;
		case SENSOR_IMU_B_1:
			memcpy(&data.imu_b[1], _data, sizeof(sensor_imu_data_t));
			fprintf(fp, "SENSOR_IMU_B_1,%g,%g,%g,%d,%g,%g,%g,%d\n",
						data.imu_b[1].acc[AXIS_X],
						data.imu_b[1].acc[AXIS_Y],
						data.imu_b[1].acc[AXIS_Z],
						data.imu_b[1].acc_time,
						data.imu_b[1].gyro[AXIS_X],
						data.imu_b[1].gyro[AXIS_Y],
						data.imu_b[1].gyro[AXIS_Z],
						data.imu_b[1].gyro_time);
			break;
		}
	}
}

/*UNUSED*/
void sync_handle_sensor_mag(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(sensor_mag_data_t)) {
		if(opcode == SENSOR_MAG_A) {
			memcpy(&data.mag_a, _data, sizeof(sensor_mag_data_t));
		} else {
			memcpy(&data.mag_b, _data, sizeof(sensor_mag_data_t));
		}
	}
}

/*SENSOR_ACC_x_x,acc_x,acc_y,acc_z,time*/
void sync_handle_sensor_acc(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(1 || len == sizeof(sensor_acc_data_t)) {
		switch(opcode){
		case SENSOR_ACC_A_0:
			memcpy(&data.acc_a[0], _data, sizeof(sensor_acc_data_t));
			fprintf(fp, "SENSOR_ACC_A_0,%g,%g,%g,%d\n",
						data.acc_a[0].acc[AXIS_X],
						data.acc_a[0].acc[AXIS_Y],
						data.acc_a[0].acc[AXIS_Z],
						data.acc_a[0].time);
			break;
		case SENSOR_ACC_A_1:
			memcpy(&data.acc_a[1], _data, sizeof(sensor_acc_data_t));
			fprintf(fp, "SENSOR_ACC_A_1,%g,%g,%g,%d\n",
						data.acc_a[1].acc[AXIS_X],
						data.acc_a[1].acc[AXIS_Y],
						data.acc_a[1].acc[AXIS_Z],
						data.acc_a[1].time);
			break;
		case SENSOR_ACC_B_0:
			memcpy(&data.acc_b[0], _data, sizeof(sensor_acc_data_t));
			fprintf(fp, "SENSOR_ACC_B_0,%g,%g,%g,%d\n",
						data.acc_b[0].acc[AXIS_X],
						data.acc_b[0].acc[AXIS_Y],
						data.acc_b[0].acc[AXIS_Z],
						data.acc_b[0].time);
			break;
		case SENSOR_ACC_B_1:
			memcpy(&data.acc_b[1], _data, sizeof(sensor_acc_data_t));
			fprintf(fp, "SENSOR_ACC_B_1,%g,%g,%g,%d\n",
						data.acc_b[1].acc[AXIS_X],
						data.acc_b[1].acc[AXIS_Y],
						data.acc_b[1].acc[AXIS_Z],
						data.acc_b[1].time);
			break;
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
	//fprintf(fp, "handling: %d, %d\n", opcode, len);
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
	case SENSOR_BARO_A_0:
	case SENSOR_BARO_A_1:
	case SENSOR_BARO_B_0:
	case SENSOR_BARO_B_1:
		sync_handle_sensor_baro(opcode, len, _data);
		break;
	case SENSOR_IMU_A_0:
	case SENSOR_IMU_A_1:
	case SENSOR_IMU_B_0:
	case SENSOR_IMU_B_1:
		sync_handle_sensor_imu(opcode, len, _data);
		break;
	case SENSOR_MAG_A:
	case SENSOR_MAG_B:
		sync_handle_sensor_mag(opcode, len, _data);
		break;
	case SENSOR_ACC_A_0:
	case SENSOR_ACC_A_1:
	case SENSOR_ACC_B_0:
	case SENSOR_ACC_B_1:
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
    uint32_t ini_data = 0x69;
    comunicator_send(&com, 0xfe, sizeof(uint32_t), (uint8_t *)&ini_data);

    printf("setup sync channel\n");
    //handle new file creation
    //First check exitance of file -> then increment names until file does not exist
    //maybe use the mkdir and check error code.
    static char fname[64];
    static uint16_t num = 0;
    do{
    	snprintf(fname, 64, "/home/root/sync%d.log", num);
    	num++;
    }while((access(fname, F_OK) == 0));
    fp = fopen(fname, "w+");

    comunicator_recv(&com);


    return NULL;
}

