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
	/** Current pulse width */
	int pulsewidth;
	/** Minimum pulse width, in microseconds */
	int min_pulse;
	/** Maximum pulse width, in microseconds */
	int max_pulse;
	/** Degrees per microsecond */
	int degrees_per_usec;

	/** GPIO pin of the servo */
	uint16_t gpio_pin;
	/** Timer channel of the servo */
	uint32_t tim_channel;
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
 * @param newRotation
 * @return ER_SUCCESS if everything went well.
 */
util_error_t setRotation(servo_t * servo, float newRotation);

/**
 * @fn float getRotation(servo_t*)
 * @brief Gets the current rotation of the servo, in degrees.
 *
 * @param servo The associated servo instance.
 * @return the current rotation (in degrees) of the servo
 */
float getRotation(servo_t * servo);

/**
 * @fn util_error_t pulse(servo_t*)
 * @brief Sends the current memorized pulse to the servo.
 * Should be called in a loop.
 *
 * @param servo The associated servo instance.
 * @return ER_SUCCESS if everything went well.
 */
util_error_t pulse(servo_t * data);

/**
 * @fn util_error_t init(servo_t*)
 * @brief Initializes the servo
 *
 * @param data
 * @return ER_SUCCESS if everything went well.
 */
util_error_t init(servo_t * data);


#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* TEMPLATE_H */

/* END */
