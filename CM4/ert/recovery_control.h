/**
 * @file 		recovery_control.h
 * @brief 		Main control of the program through the global state machine
 *
 * @date 		20.01.2022
 * @author 		Iacopo Sprenger
 *
 * @ingroup 	engine_control
 * @{
 */

#ifndef RECOVERY_CONTROL_H
#define RECOVERY_CONTROL_H

/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <util.h>


/**********************
 *  CONSTANTS
 **********************/

/**********************
 *  MACROS
 **********************/

/**********************
 *  TYPEDEFS
 **********************/

/**
 * @brief State of the recovery FSM
 *
 */
typedef enum recovery_state
{
	/** Wait for arming or calibration */
	RECOVERY_IDLE,
	/** Calibrate sensors and actuators */
	RECOVERY_CALIBRATION ,
	/** Manual event trigger movement */
	RECOVERY_MANUAL_OPERATION,
	/** System is armed and ready to pressure*/
	RECOVERY_ARMED,
	/** Rocket is in powered ascent */
	RECOVERY_POWERED,
	/** Rocket is gliding */
	RECOVERY_GLIDE,
	/** First recovery event */
	RECOVERY_FIRST_EVENT,
	/** Waiting for second event */
	RECOVERY_DROGUE,
	/** Second recovery event */
	RECOVERY_SECOND_EVENT,
	/** Waiting for touchdown */
	RECOVERY_MAIN,
	/** Rocket has landed */
	RECOVERY_TOUCHDOWN,
	/** system error*/
	RECOVERY_ERROR,
	/** User triggered abort */
	RECOVERY_ABORT
} recovery_state_t;


typedef enum recovery_cmd {
	RECOVERY_CMD_NONE,
	RECOVERY_CMD_ARM,
	RECOVERY_CMD_DISARM,
	RECOVERY_CMD_LIFTOFF,
	RECOVERY_CMD_MECO,
	RECOVERY_CMD_ABORT
}recovery_cmd_t;


/**********************
 *  VARIABLES
 **********************/

/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C" {
#endif

void recovery_control_command_push(recovery_cmd_t cmd);

void recovery_control_thread(void *arg);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* TEMPLATE_H */

/** @} */

/* END */
