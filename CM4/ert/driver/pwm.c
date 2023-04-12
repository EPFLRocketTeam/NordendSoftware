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

/**********************
 *	CONSTANTS
 **********************/

// TODO Update with HAL function when ready
// Get CK_INT for the timers (timers 2 to 7, 12 to 14 only)
// #define CK_INT  HAL_RCC_GetPCLK1Freq() // 200 MHz
#define CK_INT (200) // MHz

/**********************
 *	MACROS
 **********************/

// Uses macro pasting to "shorten the code length"
// Not sure if such a hack is good here…
#define PWM_INIT_CHANNEL(data, chan) \
	data->htim.Instance->CCR##chan = 0; \
	HAL_TIM_PWM_Start(&data->htim, TIM_CHANNEL_##chan);

/**********************
 *	TYPEDEFS
 **********************/

/**********************
 *	VARIABLES
 **********************/

/**********************
 *	PROTOTYPES
 **********************/

static void init_gpio(GPIO_TypeDef * bank, uint32_t pins, uint32_t alternate);
static uint32_t gpio_pin_assign(uint8_t channel_sel,
		uint32_t pin_ch1, uint32_t pin_ch2, uint32_t pin_ch3, uint32_t pin_ch4);
static uint8_t channel_is_active(uint8_t channel_sel, uint8_t channel_index);
static uint32_t usec_to_ccr(pwm_data_t *data, uint32_t usec);
static uint32_t ccr_read_to_usec(pwm_data_t *data, uint32_t ccr);


/**********************
 *	DECLARATIONS
 **********************/

static void init_gpio(GPIO_TypeDef * bank, uint32_t pins, uint32_t alternate) {
	// Initialize the GPIO using the given bank, pin config and alternate function mapping
	GPIO_InitTypeDef init_struct;
	init_struct.Alternate = alternate;
	init_struct.Pull = GPIO_NOPULL;
	init_struct.Mode = GPIO_MODE_AF_PP;
	init_struct.Speed = GPIO_SPEED_FREQ_LOW;
	init_struct.Pin = pins;

//	HAL_GPIO_Init(bank, &init_struct);
}

static uint32_t gpio_pin_assign(uint8_t channel_sel,
		uint32_t pin_ch1, uint32_t pin_ch2, uint32_t pin_ch3, uint32_t pin_ch4) {
	uint32_t pins = 0;

	if (channel_is_active(channel_sel, 1))
		pins |= pin_ch1;

	if (channel_is_active(channel_sel, 2))
		pins |= pin_ch2;

	if (channel_is_active(channel_sel, 3))
		pins |= pin_ch3;

	if (channel_is_active(channel_sel, 4))
		pins |= pin_ch4;

	return pins;
}


// Returns 1 if the given channel number is activated according to channel_sel, 0 if not.
static uint8_t channel_is_active(uint8_t channel_sel, uint8_t channel_index) {
	return (channel_sel >> (channel_index - 1)) & 1;
}

// Converts microseconds to something to put into the CCR register
static uint32_t usec_to_ccr(pwm_data_t *data, uint32_t usec) {
	// TODO Check if resolution and calculation holds
//	debug_log("PSC is %d\n", data->htim.Instance->PSC);
//	debug_log("I want to have %d microseconds\n", usec);

	uint32_t val_ccr =
			usec * (CK_INT) / (data->htim.Instance->PSC + 1);
//			(data->htim.Instance->ARR * usec * CK_INT)
//					/ ((data->htim.Instance->PSC + 1)
//							* (data->htim.Instance->ARR + 1));

//	debug_log("CCR is %d\n", val_ccr);
	return val_ccr;
}

// Given the data structure and channel-specific CCR register, reads the timer registers and returns the microseconds respectively
static uint32_t ccr_read_to_usec(pwm_data_t *data, uint32_t ccr) {
	uint32_t us = (ccr * (data->htim.Instance->ARR + 1)
			* (data->htim.Instance->PSC + 1))
			/ (CK_INT * data->htim.Instance->ARR);
	return us;
}

util_error_t pwm_init(pwm_data_t * data, PWM_Timer_t timer, uint8_t channel_sel) {
	return pwm_init_arr(data, timer, channel_sel, 1000000); // Using 1000000 (1M) as default ARR value
}

util_error_t pwm_init_arr(pwm_data_t *data, PWM_Timer_t timer,
		uint8_t channel_sel, uint32_t arr_val) {

	// Pin configs (Bank, Pin) ////////////////////////////////////////////////////
	// TIM3 (leds)	CH1: (C, 6) R	CH2: (C, 7) G	CH3: (B, 0) B				 //
	// TIM4 		CH1: (B, 6)		CH2: (D, 13)	CH3: (D, 14)  	CH4: (D, 15) 	 //
	// TIM5			CH1: (A, 0) 	CH2: (A, 1)		CH3: (A, 2) 	CH4: (A, 3)  	 //
	///////////////////////////////////////////////////////////////////////////////

	// Then initialize the Timer
	switch (timer) {
	// TODO (for future developers) – update this based on the board pin config.
	case PWM_TIM3:
	{
		data->htim = htim3;

		uint32_t pinsC = gpio_pin_assign(channel_sel, GPIO_PIN_6, GPIO_PIN_7, 0,
				0);
		uint32_t pinsB = gpio_pin_assign(channel_sel, 0, 0, GPIO_PIN_0, 0);

		init_gpio(GPIOC, pinsC, GPIO_AF2_TIM3);
		init_gpio(GPIOB, pinsB, GPIO_AF2_TIM3);
	}
		break;
	case PWM_TIM4:
	{
		// TODO does not exist in tim.c,
		// 	needs to be added via software interface!
		data->htim = htim2; // (Currently using htim2 to avoid compiler errors
		uint32_t pinsB = gpio_pin_assign(channel_sel, GPIO_PIN_6, 0, 0, 0);
		uint32_t pinsD = gpio_pin_assign(channel_sel, 0, GPIO_PIN_13,
		GPIO_PIN_14, GPIO_PIN_15);

		init_gpio(GPIOB, pinsB, GPIO_AF2_TIM4);
		init_gpio(GPIOD, pinsD, GPIO_AF2_TIM4);
	}
		break;
	case PWM_TIM5:
	{
		data->htim = htim5;
		debug_log("In timer 5");
		uint32_t pinsA = gpio_pin_assign(channel_sel, GPIO_PIN_0, GPIO_PIN_1,
		GPIO_PIN_2, GPIO_PIN_3);
		init_gpio(GPIOA, pinsA, GPIO_AF2_TIM5);
	}
		break;
	}

	data->htim.Instance->ARR = arr_val;
	data->channels = channel_sel;

	// Initialize each channel (1…4 depending on initialization)
	// A for loop can't be used here because of the pasting macro :(
	if (channel_is_active(data->channels, 1)) {
		PWM_INIT_CHANNEL(data, 1)
	}

	if (channel_is_active(data->channels, 2)) {
		PWM_INIT_CHANNEL(data, 2)
	}

	if (channel_is_active(data->channels, 3)) {
		PWM_INIT_CHANNEL(data, 3)
	}

	if (channel_is_active(data->channels, 4)) {
		PWM_INIT_CHANNEL(data, 4)
	}

	return ER_SUCCESS;
}

util_error_t pwm_set_microseconds(pwm_data_t *data, uint32_t usec,
		uint8_t channel_sel) {

	uint32_t v = usec_to_ccr(data, usec);
	return pwm_set_duty(data, v, channel_sel);
}

util_error_t pwm_set_duty(pwm_data_t *data, uint32_t new_duty, uint8_t channel_sel) {
	TIM_TypeDef *inst = data->htim.Instance;

	if (channel_is_active(channel_sel, 1))
		inst->CCR1 = new_duty;

	if (channel_is_active(channel_sel, 2))
		inst->CCR2 = new_duty;

	if (channel_is_active(channel_sel, 3))
		inst->CCR3 = new_duty;

	if (channel_is_active(channel_sel, 4))
		inst->CCR4 = new_duty;

	return ER_SUCCESS;
}

//util_error_t pwm_set_width(pwm_data_t *data, uint32_t new_duty, uint8_t channel_sel) {
//	TIM_TypeDef *inst = data->htim.Instance;
//
//	if (channel_is_active(channel_sel, 1))
//		inst->CCR1 = new_duty;
//
//	if (channel_is_active(channel_sel, 2))
//		inst->CCR2 = new_duty;
//
//	if (channel_is_active(channel_sel, 3))
//		inst->CCR3 = new_duty;
//
//	if (channel_is_active(channel_sel, 4))
//		inst->CCR4 = new_duty;
//
//	return ER_SUCCESS;
//}

uint32_t pwm_read_microseconds(pwm_data_t *data, uint8_t channel_index) {
	uint32_t ccr_val = read_duty(data, channel_index);
	return ccr_read_to_usec(data, ccr_val);
}

uint32_t read_duty(pwm_data_t * data, uint8_t channel_index) {
	uint32_t ccr_val;
	TIM_TypeDef *inst = data->htim.Instance;

	switch (channel_index) {
	case 1:
		ccr_val = inst->CCR1;
		break;
	case 2:
		ccr_val = inst->CCR2;
		break;
	case 3:
		ccr_val = inst->CCR3;
		break;
	case 4:
		ccr_val = inst->CCR4;
		break;
	default:
		return 0;
	}

	return ccr_val;
}


/* END */
