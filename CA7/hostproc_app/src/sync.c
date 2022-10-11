
#include <stdlib.h>
#include <stdio.h>

#include <unistd.h>
#include <string.h>


#include "serial.h"
#include "sync.h"


#define ACC_AXIS_COUNT	3

typedef struct accelerometer_data {
	int16_t raw[ACC_AXIS_COUNT];
	int16_t	processed[ACC_AXIS_COUNT];
	uint32_t timestamp;
}accelerometer_data_t;

#define GYRO_AXIS_COUNT 3

typedef struct gyroscope_data {
	int16_t raw[GYRO_AXIS_COUNT];
	int16_t	processed[GYRO_AXIS_COUNT];
	uint32_t timestamp;
}gyroscope_data_t;

typedef struct barometer_data {
	int32_t pressure;
	int32_t temperature;
	int32_t altitude;
	uint32_t timestamp;
}barometer_data_t;

typedef struct gnss_data {
    float longitude;
    float latitude;
    float altitude;
    float speed;
    float time;
    float hdop;
}gnss_data_t;

#define ACC_I2C_A 	0       // accelerometer_data_t
#define ACC_SPI_A	1       // accelerometer_data_t
#define ACC_I2C_B	2       // accelerometer_data_t
#define ACC_SPI_B	3       // accelerometer_data_t
#define GYRO_I2C_A	4       // gyroscope_data_t
#define GYRO_SPI_A	5       // gyroscope_data_t
#define GYRO_I2C_B	6       // gyroscope_data_t
#define GYRO_SPI_B	7       // gyroscope_data_t
#define BARO_I2C_A	8       // barometer_data_t
#define BARO_SPI_A	9 		// barometer_data_t
#define BARO_I2C_B	10		// barometer_data_t
#define BARO_SPI_B	11		// barometer_data_t
#define KALMAN_DATA_A 12	//transfer_data_res_t);
#define KALMAN_DATA_B 13    //transfer_data_res_t);
#define GNSS 		14		// gnss_data_t
#define BATTERY_A	15		// uint32_t
#define BATTERY_B	16		// uint32_t




static FILE * fp = NULL;


void sync_handle_acc(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(len == sizeof(accelerometer_data_t)) {
		accelerometer_data_t data;
		memcpy(&data, _data, sizeof(accelerometer_data_t));
		switch(opcode) {
		case ACC_I2C_A:
			fprintf(fp, "ACC_I2C_A,");
			break;
		case ACC_I2C_B:
			fprintf(fp, "ACC_I2C_B,");
			break;
		}
		fprintf(fp, "%d,%d,%d,%d,%d,%d\n",
				data.raw[0],
				data.raw[1],
				data.raw[2],
				data.processed[0],
				data.processed[1],
				data.processed[2],
				data.timestamp);
	}
}

void sync_handle_gyro(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(len == sizeof(gyroscope_data_t)) {
		gyroscope_data_t data;
		memcpy(&data, _data, sizeof(gyroscope_data_t));
		switch(opcode) {
		case GYRO_I2C_A:
			fprintf(fp, "GYRO_I2C_A,");
			break;
		case GYRO_I2C_B:
			fprintf(fp, "GYRO_I2C_B,");
			break;
		}
		fprintf(fp, "%d,%d,%d,%d,%d,%d\n",
				data.raw[0],
				data.raw[1],
				data.raw[2],
				data.processed[0],
				data.processed[1],
				data.processed[2],
				data.timestamp);
	}
}

void sync_handle_baro(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(len == sizeof(barometer_data_t)) {
		barometer_data_t data;
		memcpy(&data, _data, sizeof(barometer_data_t));
		switch(opcode) {
		case BARO_I2C_A:
			fprintf(fp, "BARO_I2C_A,");
			break;
		case BARO_I2C_B:
			fprintf(fp, "BARO_I2C_B,");
			break;
		}
		fprintf(fp, "%d,%d,%d\n",
				data.temperature,
				data.pressure,
				data.altitude,
				data.timestamp);
	}
}

void sync_handle_gnss(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(len == sizeof(gnss_data_t)) {
		gnss_data_t data;
		memcpy(&data, _data, sizeof(gnss_data_t));
		//gnss_data_t * data = (gnss_data_t *) _data;
		fprintf(fp, "GNSS,%f,%f,%f,%f,%f\n",
				data.altitude,
				data.hdop,
				data.latitude,
				data.longitude,
				data.speed,
				data.time);
	}
}

void sync_handle_bat(uint8_t opcode, uint16_t len, uint8_t * _data) {
	if(len == sizeof(float)) {
		float data;
		memcpy(&data, _data, sizeof(float));
		switch(opcode) {
		case BATTERY_A:
			fprintf(fp, "BATTERY_A,");
			break;
		case BATTERY_B:
			fprintf(fp, "BATTERY_B,");
			break;
		}
		fprintf(fp, "%f\n", data);
	}
}


void sync_handle_data(uint8_t opcode, uint16_t len, uint8_t * _data) {

	//printf("handling: %d | %d\n", opcode, len);
	switch(opcode) {
	case	ACC_I2C_A 	:
	case	ACC_I2C_B	:
		sync_handle_acc(opcode, len, _data);
		break;
	case	GYRO_I2C_A	:
	case	GYRO_I2C_B	:
		sync_handle_gyro(opcode, len, _data);
		break;
	case	BARO_I2C_A	:
	case	BARO_I2C_B	:
		sync_handle_baro(opcode, len, _data);
		break;
	case	GNSS 		:
		sync_handle_gnss(opcode, len, _data);
		break;
	case	BATTERY_A	:
	case	BATTERY_B	:
		sync_handle_bat(opcode, len, _data);
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

