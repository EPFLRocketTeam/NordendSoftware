/*  Title		: Led
 *  Filename	: led.c
 *	Author		: iacopo sprenger
 *	Date		: 20.01.2021
 *	Version		: 0.1
 *	Description	: rgb led control
 */

/**********************
 *	INCLUDES
 **********************/

#include "led.h"
#include <main.h>
#include <tim.h>
#include <cmsis_os.h>
#include <abstraction/gpio.h>

#include <feedback/debug.h>
#include <math.h>

/**********************
 *	CONFIGURATION
 **********************/

#define LED_TIM			htim3




/**********************
 *	CONSTANTS
 **********************/

#define LED_MAX			(0xff)


#define LED_MAX_CHECKPOINTS 32



/**********************
 *	MACROS
 **********************/


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

/**
 * @brief 	Initialize the feedback module
 * @details This will initialize a board to accept a feedback board on
 * 			the S3 socket.
 *
 */
void led_feedback_init(void) {
	//GPIO init leds
	//feedback leds located on socket 3

#if 0
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
#endif
}


void led_set(uint8_t num) {
	gpio_set(GPIOA, 1<<num);
}

void led_clear(uint8_t num) {
	gpio_clr(GPIOA, 1<<num);
}

/**
 * @brief 	Initialize the RGB LED.
 * @details Starts the PWM channels connected to the RGB led for user feedback.
 */
void led_rgb_init(void) {

	//make sure GPIO are initialized correctly

	GPIO_InitTypeDef GPIO_InitStruct;
	//coucou:))

	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	LED_TIM.Instance->ARR = LED_MAX;
	LED_TIM.Instance->CCR1 = 0;
	LED_TIM.Instance->CCR2 = 0;
	LED_TIM.Instance->CCR3 = 0;
	HAL_TIM_PWM_Start(&LED_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&LED_TIM, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&LED_TIM, TIM_CHANNEL_3);

//	gpio_config_t cfg = {
//			.drive = GPIO_DRIVE_PP,
//			.mode = GPIO_MODE_OUT,
//			.speed = 0,
//			.bias = GPIO_BIAS_NONE
//	};
//
//	gpio_cfg(GPIOA, GPIO_PIN_15, cfg);
}
/**
 * @brief	Set RBG LED color using r, g, b values.
 *
 * @param	r	Red channel value.
 * @param 	g	Green channel value.
 * @param	b	Blue channel value.
 */
void led_rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
	LED_TIM.Instance->CCR1 = r;
	LED_TIM.Instance->CCR2 = g;
	LED_TIM.Instance->CCR3 = b;
}

/**
 * @brief	Set RBG LED color using color structure.
 *
 * @param	color	Color structure, defines the color to be set.
 */
void led_rgb_set_color(led_color_t color) {
	LED_TIM.Instance->CCR1 = color.r;
	LED_TIM.Instance->CCR2 = color.g;
	LED_TIM.Instance->CCR3 = color.b;
}


/*
 * Thread checker system
 */


typedef struct led_checkpoint {
	led_color_t color;
	uint8_t check;
}led_checkpoint_t;



static led_checkpoint_t checkpoints[LED_MAX_CHECKPOINTS];

static uint16_t checkpoint_count = 0;



uint16_t led_add_checkpoint(led_color_t color) {
	if(checkpoint_count < LED_MAX_CHECKPOINTS) {
		checkpoints[checkpoint_count].color = color;
		checkpoints[checkpoint_count].check = 1;
		uint16_t tmp = checkpoint_count;
		checkpoint_count++;
		return tmp;
	}

	return LED_MAX_CHECKPOINTS;
}


void led_checkpoint(uint16_t point) {
	if(point < checkpoint_count) {
		checkpoints[point].check = 1;
	}
}


led_color_t led_hsv_2_rgb(float h, float s, float v) {
	float r = 0, g = 0, b = 0;

	if (s == 0)
	{
		r = v;
		g = v;
		b = v;
	}
	else
	{
		int i;
		float f, p, q, t;

		if (h == 360)
			h = 0;
		else
			h = h / 60;

		i = (int)truncf(h);
		f = h - i;

		p = v * (1.0 - s);
		q = v * (1.0 - (s * f));
		t = v * (1.0 - (s * (1.0 - f)));

		switch (i)
		{
		case 0:
			r = v;
			g = t;
			b = p;
			break;

		case 1:
			r = q;
			g = v;
			b = p;
			break;

		case 2:
			r = p;
			g = v;
			b = t;
			break;

		case 3:
			r = p;
			g = q;
			b = v;
			break;

		case 4:
			r = t;
			g = p;
			b = v;
			break;

		default:
			r = v;
			g = p;
			b = q;
			break;
		}

	}

	led_color_t rgb;
	rgb.r = r * 255;
	rgb.g = g * 255;
	rgb.b = b * 255;

	return rgb;
}

void rainbow_thread(__attribute__((unused)) void * arg) {

	float hue = 0;
	for(;;) {

		led_rgb_set_color(led_hsv_2_rgb(hue, 1, 1));

		hue += 5;
		if(hue > 360) hue = 0;

		osDelay(100);
	}

}



void led_rgb_thread(__attribute__((unused)) void * arg) {

	led_rgb_init();

	//led_feedback_init();

	//led_rgb_set_color(led_blue);

#ifdef USE_CHECKPOINT
	uint16_t base = led_add_checkpoint(led_white);
	static uint16_t counter = 0;
#endif

	for(;;) {
#ifdef USE_CHECKPOINT
		led_clear(3);
		led_checkpoint(base);
		while(1) {
			if(!(counter < checkpoint_count)) {
				counter = 0;
			}
			if(checkpoints[counter].check) {
				led_rgb_set_color(checkpoints[counter].color);
				checkpoints[counter].check = 0;
				counter++;
				break;
			} else {
				counter++;
			}
		}
		osDelay(500);
		led_rgb_set_color(led_black);
#endif
		osDelay(500);
	}
}


/* END */


