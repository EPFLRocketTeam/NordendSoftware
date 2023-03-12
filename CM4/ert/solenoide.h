/*
 * solenoide.h
 *
 *  Created on: Dec 11, 2022
 *      Author: julianruizrodriguez
 */

#include <util.h>
#include <device/device.h>

/**
 * @enum 	Solenoids
 * @brief 	Used for Solenoid pin number
 * 			to GPIO pinout conversion
 */
typedef enum Solenoids {
	SOLENOID_1 = GPIO_PIN_6,/**< SOLENOID_1 */
	SOLENOID_2 = GPIO_PIN_5,/**< SOLENOID_2 */
	SOLENOID_3 = GPIO_PIN_4,/**< SOLENOID_3 */
	SOLENOID_4 = GPIO_PIN_3 /**< SOLENOID_4 */
} Solenoids_t;

/**
 * @fn 		void solenoid_init(void)
 * @brief 	Initializes the solenoid pin bank with the
 * 			correct pin numbers
 */
void solenoid_init(void);

/**
 * @fn 			void solenoid_on(Solenoids_t)
 * @brief 		Sets the given solenoid's pin to 1 (active)
 * @param pin 	the solenoid pin to activate
 */
void solenoid_on(Solenoids_t pin);

/**
 * @fn 			void solenoid_off(Solenoids_t)
 * @brief 		Sets the given solenoid's pin to 0 (inactive)
 * @param pin	the solenoid pin to reset
 */
void solenoid_off(Solenoids_t pin);
