#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>


typedef enum cmd_subsystem_target {
	SUBSYSTEM_PROPULSION,
	SUBSYSTEM_RECOVERY
}cmd_subsystem_target_t;

//possible commands
typedef enum control_command {
	COMMAND_NONE,
	COMMAND_ARM,
	COMMAND_DISARM,
	COMMAND_PRESSURE,
	COMMAND_IGNITE,
	COMMAND_RECOVER,
	COMMAND_VENT_N2O,
	COMMAND_VENT_ETH,
	COMMAND_MAN_PRESS,
	COMMAND_MAN_PURGE,
	COMMAND_VALVE_N2O,
	COMMAND_VALVE_ETH,
	COMMAND_ABORT
} control_command_t;

typedef enum control_state
{
	/** Wait for arming or calibration */
	CONTROL_IDLE,
	/** Calibrate sensors and actuators */
	CONTROL_CALIBRATION ,
	/** Manual Servo movement */
	CONTROL_MANUAL_OPERATION,
	/** System is armed and ready to pressure*/
	CONTROL_ARMED,
	/** system is pressured */
	CONTROL_PRESSURED,
	/** fire igniter */
	CONTROL_IGNITER,
	/** partially open valves*/
	CONTROL_IGNITION,
	/** fully open valves */
	CONTROL_THRUST,
	/** close ethanol valve */
	CONTROL_SHUTDOWN,
	/** glide */
	CONTROL_GLIDE,
	/** system error*/
	CONTROL_ERROR,
	/** User triggered abort */
	CONTROL_ABORT
} control_state_t;


typedef struct engine_control_data {
	uint8_t state;
	uint8_t last_cmd;
	int32_t last_parameter;


	uint32_t time;
}engine_control_data_t;

typedef struct recovery_control_data {
	uint8_t state;
	uint8_t last_cmd;


	uint32_t time;
}recovery_control_data_t;

typedef struct cmd_engine_control_order {
	uint8_t cmd;
	int32_t parameter;
}cmd_engine_control_order_t;

typedef struct cmd_recovery_control_order {
	uint8_t cmd;
}cmd_recovery_control_order;


typedef struct gnss_data {
	float	longitude;
	float	latitude;
	float	altitude;
	float   speed;
	float 	hdop;

	uint32_t time;
}gnss_data_t;

typedef struct battery_data {
	uint32_t voltage;
	uint32_t time;
}battery_data_t;

typedef struct sensor_baro_data {
	uint32_t pressure;
	int32_t temperature;
	float alt;

	uint32_t time;
}sensor_baro_data_t;

typedef struct sensor_imu_data {
	int32_t raw_acc[3];
	float acc[3];
	int32_t raw_gyro[3];
	float gyro[3];
	int32_t temperature;

	uint32_t time;
}sensor_imu_data_t;

typedef struct sensor_mag_data {
	int32_t raw_mag[3];
	float mag[3];

	uint32_t time;
}sensor_mag_data_t;

typedef struct sensor_acc_data {
	int32_t raw_acc[3];
	float acc[3];

	uint32_t time;
}sensor_acc_data_t;


typedef struct sensor_eng_data {
	int32_t pressure_1;
	int32_t pressure_2;

	uint32_t time;
}sensor_eng_data_t;


typedef struct kalman_data {
    int32_t alt;
    int32_t vel;

    uint32_t time;
}kalman_data_t;




#endif
