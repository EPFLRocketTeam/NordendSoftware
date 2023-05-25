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
#include <RadioPacket/RadioPacket.h>
#include <solenoide.h>
#include <propulsion/servo.h>


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
	/** Control vent N2O*/
	CONTROL_VENT_N2O = 2,
	/** Control vent ethanol */
	CONTROL_VENT_ETHANOL = 3,
	/** Control the servos */
	CONTROL_SERVOS_N2O = 4,

	CONTROL_SERVOS_ETHANOL = 5,
	/** Pressurisation */
	CONTROL_PRESSURISATION = 6,
	/** Stand-by while falling */
	CONTROL_GLIDE = 7,
	/** Countdown until engine ignition */
	CONTROL_COUNTDOWN = 8,
	/** New word */
	CONTROL_IGNITER = 9,
	/** smth smth */
	CONTROL_IGNITION = 10,
	/** Blow up stuff so rocket go ffffffffffiiiiiouuu */
	CONTROL_THRUST = 11,
	/** Stops engine combustion */
	CONTROL_SHUTDOWN = 12,
	/** Reach highest altitude before gliding back down */
	CONTROL_APOGEE = 13,
	/** Engine Depressurisation */
	CONTROL_DEPRESSURISATION = 14,
	/** Ground (automatic) error */
	CONTROL_ERROR = 15,
	/** Flight or radio-triggered error */
	CONTROL_ABORT = 16
} control_state_t;

typedef struct control
{
	control_state_t state;
	control_state_t prev_state;

	Solenoids_t *solenoid_n2o;
	Solenoids_t *solenoid_ethanol;
	Solenoids_t *solenoid_pressurisation;
	Solenoids_t *solenoid_purge;

	device_t *i2c_engine_press;
	device_t *i2c_engine_temp;

	servo_t *servo_ethanol;
	servo_t *servo_n2o;

	uint32_t counter;
	uint32_t time;
	uint32_t last_time;
} control_t;

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

#define SCHED_ALLOWED_WIDTH (9)

static control_sched_t sched_allowed[][SCHED_ALLOWED_WIDTH] = {
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_CALIBRATE, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_PRESSURISATION, CONTROL_SCHED_COUNTDOWN, CONTROL_SCHED_NOTHING}, // IDLE
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// CALIBRATION
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// VENTS
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// SERVOS
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// PRESSURISATION
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// GLIDE
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// COUNTDOWN
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// IGNITER
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// IGNITION
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// THRUST
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// SHUTDOWN
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// APOGEE
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// DEPRESSURISATION
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// ERROR
	{CONTROL_SCHED_ABORT, CONTROL_SCHED_VENT_N2O, CONTROL_SCHED_VENT_ETHANOL, CONTROL_SCHED_SERVOS_N2O, CONTROL_SCHED_SERVOS_ETHANOL, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING, CONTROL_SCHED_NOTHING},	 	// ABORT
};

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
	void control_vent_n2o_start(void);
	void control_vent_ethanol_start(void);				
	void control_servos_n2o_start(void);
	void control_servos_ethanol_start(void);
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
	void control_vent_n2o_run(void);
	void control_vent_ethanol_run(void);				
	void control_servos_n2o_run(void);
	void control_servos_ethanol_run(void);
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
	control_state_t correlate_state_sched(control_sched_t requested_state);

	void engine_control_thread(void *arg);

	util_error_t init_eng_ctrl(void);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* TEMPLATE_H */

/** @} */

/* END */
