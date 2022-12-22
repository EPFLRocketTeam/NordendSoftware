/*  Title		: PWM
 *  Filename	: pwm.c
 *	Author		: Julian Marmier
 *	Date		: 16.12.2022
 *	Version		: 0.1
 *	Description	: Interface for PWM clocks (TIM2…7 and TIM12…14 only)
 */

/**********************
 *	INCLUDES
 **********************/

#include "pwm.h"
#include "stm32mp1xx_hal_rcc.c"

/**********************
 *	CONSTANTS
 **********************/

// TODO Check if this returns the /2 or not !! And if this truly corresponds to APB
// Get CK_INT for the timers (timers 2 to 7, 12 to 14 only)
#define CK_INT  HAL_RCC_GetPCLK1Freq() // 200 MHz

/**********************
 *	MACROS
 **********************/

// Uses macro pasting to "shorten the code length"
// Not sure if such a hack is good here…
#define PWM_INIT_CHANNEL(data, chan) \
	data->htim->Instance->CCR##chan = 0; \
	HAL_TIM_PWM_Start(&data->htim.Instance, TIM_CHANNEL##chan);

/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/


/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/

util_error_t pwm_init(pwm_init_structure_t * data) {

	// Initialize the GPIO first
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructure.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Pin = data->gpio_pin;
	GPIO_InitStructure.Alternate = data->alternate_function;

	HAL_GPIO_Init(data->gpio_bank, &GPIO_InitStructure);

	// Then initialize the Timer
	data->htim->Instance->ARR = 0;

	// Initialize each channel (1…4 depending on initialization)
	PWM_INIT_CHANNEL(data, 1)

	if (data->channels >= 2)
		PWM_INIT_CHANNEL(data, 2)

	if (data->channels >= 3)
		PWM_INIT_CHANNEL(data, 3)

	if (data->channels >= 4)
		PWM_INIT_CHANNEL(data, 4)


	return ER_SUCCESS;
}

static uint8_t usecToCCR(pwm_init_structure_t * data, uint8_t usec){
	// TODO Check if resolution and calculation holds
	uint8_t val = (data->htim->Instance->ARR * usec * CK_INT) / ((data->htim->Instance->PSC + 1) * (data->htim->Instance->ARR + 1));
	return val;
}

util_error_t setMicroseconds(float uses, pwm_init_structure_t * data, uint8_t channel) {
	if (channel > data->channels) return ER_OUT_OF_RANGE;

	TIM_TypeDef * inst = data->htim->Instance;
	uint8_t v = usecToCCR(data, usec);

	switch (channel) {
		case 1:
			inst->CCR1 = v;
			break;
		case 2:
			inst->CCR2 = v;
			break;
		case 3:
			inst->CCR3 = v;
			break;
		case 4:
			inst->CCR4 = v;
			break;

	}

	return ER_SUCCESS;
}




/* END */
