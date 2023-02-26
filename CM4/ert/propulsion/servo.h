/*  Title       : Template
 *  Filename    : template.h
 *  Author      : Julian Marmier
 *  Date        : 20.01.2021
 *  Version     : 0.1
 *  Description : template for code files
 */

#ifndef TEMPLATE_H
#define TEMPLATE_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include "util.h"
#include "driver/pwm.h"

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
 * @struct servo
 * @brief Data about the servo's current position.
 * Please do not modify directly, use the provided Servo API.
 */
typedef struct servo {
	/** Rotation in degrees */
	float rotation; // TODO define coordinate system
	uint32_t pulsewidth; /*!< Current pulse width */
	uint32_t min_pulse; /*!< Minimum pulse width, in microseconds */
	uint32_t max_pulse; /*!< Maximum pulse width, in microseconds */
	uint32_t origin; /*!< Offset for the origin, in microseconds. Usually 1500 us corresponds to 0 degrees */
	float degrees_per_usec; /*!< Degrees per microsecond */

	pwm_data_t * pwm_data; /*!< Attached PWM driver data structure */
	PWM_Channel_Selection_t pwm_channel;  /*!< Associated PWM channel */
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
util_error_t set_rotation(servo_t * servo, float newRotation);

/**
 * @fn float getRotation(servo_t*)
 * @brief Gets the current rotation of the servo, in degrees.
 *
 * @param servo The associated servo instance.
 * @return the current rotation (in degrees) of the servo
 */
float get_rotation(servo_t * servo);


/**
 * @fn util_error_t servo_init(servo_t*, pwm_data_t*, PWM_Channel_Selection_t, uint32_t, uint32_t, float)
 * @brief Initializes the given servo instance according to the parameters.
 *
 * @pre
 * @post
 * @param servo The servo instance to initialize
 * @param pwm The PWM instance to attach to the servo instance. The PWM must be initialized using {@link pwm_init} from driver/pwm.
 * @param pwm_channel The channel associated with the servo instance
 * @param min_pulse The minimum allowed pulse that can be sent to the servo, in microseconds
 * @param max_pulse The maximum allowed pulse that can be sent to the servo, in microseconds
 * @param origin The origin offset (in microseconds), for computing the pulse width. Default should be 1500.
 * @param degrees_per_usec The degrees-to-microsecond ratio of the servo, used for setting and getting rotation.
 * @return
 */
util_error_t servo_init(
		servo_t * servo,
		pwm_data_t * pwm,
		PWM_Channel_Selection_t pwm_channel,
		uint32_t min_pulse,
		uint32_t max_pulse,
		uint32_t origin,
		float degrees_per_usec
	);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* TEMPLATE_H */

/* END */
