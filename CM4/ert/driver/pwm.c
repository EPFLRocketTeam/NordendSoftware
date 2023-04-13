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
#include "stm32mp1xx_hal_rcc.h"
#include <abstraction/gpio.h>
#include <feedback/debug.h>
#include <tim.h>

/**********************
 *	CONSTANTS
 **********************/

// TODO Update with HAL function when ready
// Get CK_INT for the timers (timers 2 to 7, 12 to 14 only)
// #define CK_INT  HAL_RCC_GetPCLK1Freq() // 200 MHz
#define CK_INT (200) // MHz
#define INSTANCE htim5.Instance

/**********************
 *	MACROS
 **********************/

// Converts microseconds to a clock count value (in terms of the prescaler)
#define USEC_TO_CCR(usec) ((usec) * (CK_INT) / (INSTANCE->PSC + 1))
#define CCR_TO_USEC(ccr) ((ccr) * (INSTANCE->PSC+1) / (CK_INT))

static const uint32_t DEFAULT_ARR_VALUE = 1000000;

/**********************
 *	TYPEDEFS
 **********************/

/**********************
 *	VARIABLES
 **********************/

/**********************
 *	PROTOTYPES
 **********************/

static uint8_t channel_is_selected(uint8_t channel_sel, uint8_t channel_index);


/**********************
 *	DECLARATIONS
 **********************/

// Returns 1 if the given channel number is activated according to channel_sel, 0 if not.
static uint8_t channel_is_selected(uint8_t channel_sel, uint8_t channel_index) {
	return (channel_sel >> (channel_index - 1)) & 1;
}

util_error_t pwm_init(void) {
	// Start the PWM on each channel
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

	htim5.Instance->ARR = DEFAULT_ARR_VALUE;

	return ERR_SUCCESS;
}

util_error_t pwm_set_period(uint16_t us) {
	htim5.Instance->ARR = USEC_TO_CCR(us);
}


util_error_t pwm_set_microseconds(uint32_t usec, uint8_t channel_sel) {

	uint32_t v = USEC_TO_CCR(us);

	if (channel_is_selected(channel_sel, 1))
		INSTANCE->CCR1 = v;

	if (channel_is_selected(channel_sel, 2))
		INSTANCE->CCR2 = v;

	return ERR_SUCCESS;
}


uint32_t pwm_read_microseconds(pwm_data_t *data, uint8_t channel_index) {
	uint32_t ccr_val = 0;

	switch (channel_index) {
	case 1:
		ccr_val = inst->CCR1;
		break;
	case 2:
		ccr_val = inst->CCR2;
		break;
	default:
		return 0;
	}

	return CCR_TO_USEC(ccr_val);
}

/* END */
