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







typedef enum control_sched
{
	// In order of priority -- cardinality is implicit
	CONTROL_SCHED_ABORT,
	CONTROL_SCHED_VENT_N2O,
	CONTROL_SCHED_VENT_ETHANOL,
	CONTROL_SCHED_CALIBRATE,
	CONTROL_SCHED_SERVOS_N2O,
	CONTROL_SCHED_SERVOS_ETHANOL,
	CONTROL_SCHED_COUNTDOWN,
	CONTROL_SCHED_SHUTDOWN, // unused in Nordend
	CONTROL_SCHED_PRESSURISATION,
	// CONTROL_SCHED_DEPRESSURISATION, -> if needed to be radio-triggered
	CONTROL_SCHED_NOTHING
} control_sched_t;


/**********************
 *  VARIABLES
 **********************/

/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"
{
#endif

void control_idle_start(void);
void control_calibration_start(void);
void control_pressured_start(void);
void control_armed_start(void);
void control_igniter_start(void);
void control_ignition_start(void);
void control_thrust_start(void);
void control_shutdown_start(void);
void control_glide_start(void);
void control_error_start(void);
void control_abort_start(void);

void control_idle_run(void);
void control_calibration_run(void);
void control_pressured_run(void);
void control_armed_run(void);
void control_igniter_run(void);
void control_ignition_run(void);
void control_thrust_run(void);
void control_shutdown_run(void);
void control_glide_run(void);
void control_error_run(void);
void control_abort_run(void);

void engine_control_thread(void *arg);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* TEMPLATE_H */

/** @} */

/* END */
