/**
 * @file 		control.h
 * @brief 		Main control of the program through the global state machine
 *
 * @date 		20.01.2022
 * @author 		Iacopo Sprenger
 *
 * @ingroup 	engine_control
 * @{
 */

#ifndef ENGINE_CONTROL_H
#define ENGINE_CONTROL_H

/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <util.h>
#include <propulsion/servo.h>
#include "solenoid.h"


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
 * @brief State of the control FSM
 *
 * the description of the states don't make sense -> must be reviewed
 */
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



/**
 * The commands are processed through a message queue so that none can be missed.
 */
typedef enum control_command {
	COMMAND_ARM,
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


/**********************
 *  VARIABLES
 **********************/

/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C" {
#endif

void engine_control_command_push(control_command_t cmd, int32_t parameter);

void engine_control_thread(void *arg);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* TEMPLATE_H */

/** @} */

/* END */
