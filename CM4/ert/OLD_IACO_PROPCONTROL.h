/*  Title       : Control
 *  Filename    : control.h
 *  Author      : iacopo sprenger
 *  Date        : 20.01.2021
 *  Version     : 0.1
 *  Description : main program control
 */

#ifndef CONTROL_H
#define CONTROL_H

/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <epos4.h>
#include <servo.h>
#include <can_comm.h>

/**********************
 *  CONSTANTS
 **********************/



/**********************
 *  MACROS
 **********************/





/**********************
 *  TYPEDEFS
 **********************/

typedef enum CONTROL_STATE{
	CS_IDLE = 0x00,
	CS_CALIBRATION = 0x01,
	CS_ARMED = 0x02,
	CS_COUNTDOWN  = 0x03,
	CS_IGNITION  = 0x04,
	CS_THRUST  = 0x05,
	CS_SHUTDOWN  = 0x06,
	CS_GLIDE  = 0x07,
	CS_ABORT  = 0x08,
	CS_ERROR  = 0x09
}CONTROL_STATE_t;

typedef struct CONTROL_PP_PARAMS {
	uint32_t acc;
	uint32_t dec;
	uint32_t speed;
	uint32_t countdown_wait;
	uint32_t half_wait;
	uint32_t full_wait;
	int32_t half_angle;
	int32_t full_angle;
}CONTROL_PP_PARAMS_t;

typedef struct CONTROL_STATUS {
	CONTROL_STATE_t state;
	uint16_t pp_psu_voltage;
	uint16_t pp_error;
	int32_t pp_position;
	uint16_t pp_status;
	uint16_t tvc_psu_voltage;
	int8_t tvc_temperature;
	uint8_t tvc_error;
	int32_t tvc_position;
	int32_t counter;
	uint8_t counter_active;
	uint32_t time;
	uint8_t venting;
}CONTROL_STATUS_t;


//action scheduling
//
//schedule: higher in the list -> higher priority
typedef enum CONTROL_SCHED{
	CONTROL_SCHED_NOTHING = 0x00,
	CONTROL_SCHED_ABORT,
	CONTROL_SCHED_MOVE_PP,
	CONTROL_SCHED_MOVE_TVC,
	CONTROL_SCHED_CALIBRATE,
	CONTROL_SCHED_ARM,
	CONTROL_SCHED_DISARM,
	CONTROL_SCHED_IGNITE,
	CONTROL_SCHED_RECOVER,
	CONTROL_SCHED_N
}CONTROL_SCHED_t;


typedef struct CONTROL_INST{
	CONTROL_STATE_t state;
	uint32_t time;
	uint32_t last_time;
	int32_t counter;
	uint8_t counter_active;
	uint32_t iter;
	EPOS4_INST_t * pp_epos4;
	EPOS4_INST_t * ab_epos4;
	SERVO_INST_t * tvc_servo;
	CONTROL_PP_PARAMS_t pp_params;
	EPOS4_MOV_t pp_mov_type;
	int32_t pp_mov_target;
	uint8_t pp_mov_started;
	int32_t pp_recover_pos;
	int32_t	tvc_mov_target;
	uint8_t tvc_mov_started;
	CONTROL_SCHED_t sched;
	uint8_t pp_motor_prepped;
	uint8_t pp_close_mov_started;
	uint8_t pp_abort_mov_started;
	CAN_msg msg;
	uint8_t venting;
	uint8_t needs_recover;
	uint8_t hang_for_recovery;
}CONTROL_INST_t;




/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

void control_thread(void * arg);

CONTROL_STATE_t control_get_state();

CONTROL_PP_PARAMS_t control_get_pp_params();

void control_set_pp_params(CONTROL_PP_PARAMS_t params);

void control_move_pp(EPOS4_MOV_t mov_type, int32_t target);

void control_move_tvc(int32_t target);

void control_calibrate();

void control_arm();

void control_disarm();

void control_ignite();

void control_abort();

void control_recover();

uint8_t control_open_vent();

uint8_t control_close_vent();

CONTROL_STATUS_t control_get_status();

void control_attempt_recover(CONTROL_STATUS_t last_state);

void control_release();



#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* CONTROL_H */

/* END */












