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
#include "util.h"

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
typedef enum control_state {
	/** Wait for arming or calibration */
	CONTROL_IDLE = 0,
	/** Calibrate sensors and actuators */
	CONTROL_CALIBRATION = 1,
	/** Control venting 1 */
	CONTROL_VENT1 = 2,
	/** Control venting 2 */
	CONTROL_VENT2 = 3,
	/** Control the opening of the purge */
	CONTROL_PURGE = 4,
	/** Control the N2O servo */
	CONTROL_N2O = 5,
	/** Control the ethanol servo */
	CONTROL_ETHANOL = 6,
	/** Powered ascent */
	CONTROL_PRESSURISATION = 7,
	/** Subsonic, coast flight */
	CONTROL_GLIDE = 8,
	/** Supersonic flight */
	CONTROL_COUNTDOWN = 9,
	/** Apogee reached, trigger first event */
	CONTROL_IGNITER = 10,
	/** Drogue chute descent, wait for second event */
	CONTROL_IGNITION = 11,
	/** Low alt reached, trigger second event */
	CONTROL_THRUST = 12,
	/** Main chute descent, wait for touchdown */
	CONTROL_SHUTDOWN = 13,
	/** Touchdown detected, end of the flight */ //-> should be end of propulsion only
	CONTROL_APOGEE = 14,
	/** Ballistic flight detected */
	CONTROL_DEPRESSURISATION = 15,
	/** Ground (automatic) error */
	CONTROL_ERROR = 16,
	/** Flight (radio-triggered) error */
	CONTROL_ABORT = 17
} control_state_t;

typedef struct control {
	control_state_t state;
	control_state_t prev_state;

	Solenoid_t solenoid_n2o;
	Solenoid_t solenoid_ethanol;
	Solenoid_t solenoid_pressurisation;
	Solenoid_t solenoid_purge;

	device_t i2c_engine_press;
	device_t i2c_engine_temp;

	servo_t servo_ethanol;
	servot_t servo_n2o;

	uint8_t vent1_pins;
	uint8_t vent2_pins;
} control_t;

/**********************
 *  VARIABLES
 **********************/



/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif


void control_idle_start(void);
void control_calibration_start(void);
void control_vent1_start(void);
void control_vent2_start(void);
void control_purge_start(void);
void control_n2o_start(void);
void control_ethanol_start(void);
void control_pressurisation_start(void);
void control_glide_start(void);
void control_countdown_start(void);
void control_igniter_start(void);
void control_ignition_start(void);
void control_thrust_start(void);
void control_shutdown_start(void);
void control_apogee_start(void);
void control_depressurisation_start(void);
void control_error_start(void);
void control_abort_start(void);

void control_idle_run(void);
void control_calibration_run(void);
void control_vent1_run(void);
void control_vent2_run(void);
void control_purge_run(void);
void control_n2o_run(void);
void control_ethanol_run(void);
void control_pressurisation_run(void);
void control_glide_run(void);
void control_countdown_run(void);
void control_igniter_run(void);
void control_ignition_run(void);
void control_thrust_run(void);
void control_shutdown_run(void);
void control_apogee_run(void);
void control_depressurisation_run(void);
void control_error_run(void);
void control_abort_run(void);

void engine_control_thread(void * arg);

util_error_t init(void);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* TEMPLATE_H */

/** @} */

/* END */
