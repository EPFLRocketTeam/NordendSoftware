/**  Title       : Template
 *  Filename    : template.h
 *  Author      : Julian Marmier
 *  Date        : 20.01.2021
 *  Version     : 0.1
 *  Description : template for code files
 */

#if 0

#ifndef SERVO_H
#define SERVO_H

/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include "util.h"
#include "driver/pwm.h"

/**********************
 *  CONSTANTS
 **********************/

/**
 * Origin offset (in microseconds), for computing the pulse width. Default is 1500.
 */
#define SERVO_ETHANOL_OFFSET (1500)
#define SERVO_ETHANOL_OPEN (90)
#define SERVO_ETHANOL_IGNITION (45)
#define SERVO_ETHANOL_CLOSED (0)


/**
 * Origin offset (in microseconds), for computing the pulse width. Default is 1500.
 */
#define SERVO_N2O_OFFSET (1500)
#define SERVO_N2O_OPEN (90)
#define SERVO_N2O_IGNITION (45)
#define SERVO_N2O_CLOSED (0)




/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

/**
 * @struct servo_state
 * @brief Defines the three possible servo states:
 * 	open, partially open and closed.
 *
 */
typedef enum servo_state {
	SERVO_CLOSED,
	SERVO_PARTIALLY_OPEN,
	SERVO_OPEN
} servo_state_t;


/**
 * @struct servo
 * @brief Data about the servo's current position.
 * Please do not modify directly, use the provided Servo API functions.
 */
typedef struct servo {
	/** Rotation in degrees */
	float rotation; // TODO define coordinate system
	uint32_t pulsewidth;					/*!< Current pulse width */
	uint32_t min_pulse;						/*!< Minimum pulse width, in microseconds */
	uint32_t max_pulse;						/*!< Maximum pulse width, in microseconds */
	uint32_t origin;						/*!< Offset for the origin, in microseconds. Usually 1500 us corresponds to 0 degrees */
	float degrees_per_usec;					/*!< Degrees per microsecond */

	servo_state_t state;						/*!< State */
	float open_rotation;					/*!< Open state rotation */
	float partially_open_rotation;			/*!< Partially open state rotation */
	float closed_rotation;					/*!< Closed state rotation */

	PWM_Channel_Selection_t pwm_channel;  	/*!< Associated PWM channel */

} servo_t;

/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @fn util_error_t setRotation(servo_t*, float)
 * @brief Sets the rotation of the servo, in degrees.
 *
 * @param servo The associated servo instance.
 * @param newRotation The new rotation to set, in degrees.
 * @return ER_SUCCESS if everything went well.
 */
util_error_t servo_set_rotation(servo_t *servo, float new_rotation);

/**
 * @fn float getRotation(servo_t*)
 * @brief Gets the current rotation of the servo, in degrees.
 * @note Use of this function is discouraged if using the state mechanism,
 * 	as there are no checks in place to verify that
 * @param servo The associated servo instance.
 * @return the current rotation (in degrees) of the servo
 */
float servo_get_rotation(servo_t *servo);

/**
 * @fn util_error_t servo_set_state(servo_t*, servo_state_t)
 * @brief Sets the state of the servo (open, closed, or partially open)
 *
 * @param servo The associated servo instance.
 * @param new_state The new state to set the servo to.
 * @return ER_SUCCESS if everything went well.
 */
util_error_t servo_set_state(servo_t *servo, servo_state_t new_state);

/**
 * @fn servo_state_t servo_get_state(servo_t*)
 * @brief Gets the state of the servo (open, closed, or partially open)
 *
 * @param servo The associated servo instance.
 * @return ER_SUCCESS if everything went well.
 */
servo_state_t servo_get_state(servo_t *servo);


/**
 * @fn util_error_t servo_init(servo_t*, pwm_data_t*, PWM_Channel_Selection_t, uint32_t, uint32_t, float)
 * @brief Initializes the given servo instance according to the parameters.
 *
 * @pre
 * @post
 * @param servo The servo instance to initialize
 * @param pwm_channel The channel associated with the servo instance
 * @param min_pulse The minimum allowed pulse that can be sent to the servo, in microseconds
 * @param max_pulse The maximum allowed pulse that can be sent to the servo, in microseconds
 * @param origin The origin offset (in microseconds), for computing the pulse width. Default should be 1500.
 * @param degrees_per_usec The degrees-to-microsecond ratio of the servo, used for setting and getting rotation.
 * @param open_rotation The rotation (in degrees) for the OPEN state.
 * @param partially_open_rotation The rotation (in degrees) for the PARTIALLY_OPEN state.
 * @param closed_rotation The rotation (in degrees) for the CLOSED state.
 * @return
 */
util_error_t servo_init(
		servo_t * servo,
		PWM_Channel_Selection_t pwm_channel,
		uint32_t min_pulse,
		uint32_t max_pulse,
		uint32_t origin,
		float degrees_per_usec,
		float open_rotation,
		float partially_open_rotation,
		float closed_rotation
	);


void servo_thread(__attribute__((unused)) void * arg);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* SERVO_H */


#endif

/* END */
