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
typedef enum control_state
{
	/** Wait for arming or calibration */
	CONTROL_IDLE = 0,
	/** Calibrate sensors and actuators */
	CONTROL_CALIBRATION = 1,
	/** Control venting */
	CONTROL_VENTS = 2,
	/** Control the servos */
	CONTROL_SERVOS = 3,
	/** Pressurisation */
	CONTROL_PRESSURISATION = 4,
	/** Stand-by while falling */
	CONTROL_GLIDE = 5,
	/** Countdown until engine ignition */
	CONTROL_COUNTDOWN = 6,
	/** New word */
	CONTROL_IGNITER = 7,
	/** smth smth */
	CONTROL_IGNITION = 8,
	/** Blow up stuff so rocket go ffffffffffiiiiiouuu */
	CONTROL_THRUST = 9,
	/** Stops engine combustion */
	CONTROL_SHUTDOWN = 10,
	/** Reach highest altitud before gliding back down */
	CONTROL_APOGEE = 11,
	/** Engine Depressurisation */
	CONTROL_DEPRESSURISATION = 12,
	/** Ground (automatic) error */
	CONTROL_ERROR = 13,
	/** Flight or radio-triggered) error */
	CONTROL_ABORT = 14
} control_state_t;

typedef struct control
{
	control_state_t state;
	control_state_t prev_state;

	Solenoid_t *solenoid_n2o;
	Solenoid_t *solenoid_ethanol;
	Solenoid_t *solenoid_pressurisation;
	Solenoid_t *solenoid_purge;

	device_t *i2c_engine_press;
	device_t *i2c_engine_temp;

	servo_t *servo_ethanol;
	servot_t *servo_n2o;

	uint32_t counter;
	uint32_t time;
	uint32_t last_time;
} control_t;

typedef enum control_sched
{
	// In order of priority -- cardinality is implicit
	CONTROL_SCHED_ABORT,
	CONTROL_SCHED_VENTS,
	CONTROL_SCHED_CALIBRATE,
	CONTROL_SCHED_SERVOS,
	CONTROL_SCHED_COUNTDOWN,
	CONTROL_SCHED_SHUTDOWN, // unused in Nordend
	CONTROL_SCHED_PRESSURISATION,
	// CONTROL_SCHED_DEPRESSURISATION, -> if needed to be radio-triggered
	CONTROL_SCHED_NOTHING
} control_sched_t;

#define SCHED_ALLOWED_WIDTH (8)

static control_sched_t sched_allowed[][SCHED_ALLOWED_WIDTH] = {
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_CALIBRATE, CONTROL_SCHED_VENTS, CONTROL_SCHED_SERVOS, CONTROL_SCHED_PRESSURISATION, CONTROL_SCHED_COUNTDOWN, CONTROL_SCHED_NOTHING}, // IDLE
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// CALIBRATION
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// VENTS
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// SERVOS
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// PRESSURISATION
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// GLIDE
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// COUNTDOWN
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// IGNITER
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// IGNITION
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// THRUST
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// SHUTDOWN
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// APOGEE
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// DEPRESSURISATION
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// ERROR
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// ABORT
};
//TODO Make access to solenoids and servos possible at all times

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
	void control_vent_start(void);				
	void control_servo_start(void);
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
	void control_vent_run(void);
	void control_servo_start(void);
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

	void schedule_next_state(control_state_t next_state);
	static void prev_state_start(void);

	void engine_control_thread(void *arg);

	util_error_t init(void);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* TEMPLATE_H */

/** @} */

/* END */
