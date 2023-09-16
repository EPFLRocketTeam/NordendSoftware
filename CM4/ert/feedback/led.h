/*  Title       : Led
 *  Filename    : led.h
 *  Author      : iacopo sprenger
 *  Date        : 20.01.2021
 *  Version     : 0.1
 *  Description : rgb led control
 */

#ifndef LED_H
#define LED_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>

/**********************
 *  CONSTANTS
 **********************/

#define LED_RED			0xff, 0x00, 0x00
#define LED_GREEN		0x00, 0xff, 0x00
#define LED_BLUE		0x00, 0x00, 0xff

#define LED_ORANGE		0x7f, 0x0f, 0x07
#define LED_YELLOW		0xff, 0x1f, 0x07
#define LED_TEAL		0x00, 0x7f, 0x7f
#define LED_PINK		0x7f, 0x00, 0x7f
#define LED_LILA		0xff, 0x03, 0x4f

#define LED_BLACK		0x00, 0x00, 0x00
#define LED_WHITE		0xff, 0xff, 0xff





/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

typedef struct led_color {
	uint8_t r;
	uint8_t g;
	uint8_t b;
}led_color_t;



/**********************
 *  VARIABLES
 **********************/

static const led_color_t led_red = {
		.r = 0xff,
		.g = 0x00,
		.b = 0x00
};

static const led_color_t led_green = {
		.r = 0x00,
		.g = 0xff,
		.b = 0x00
};

static const led_color_t led_yellow = {
		.r = 0xff,
		.g = 0x1f,
		.b = 0x07
};

static const led_color_t led_lime = {
		.r = 0x3f,
		.g = 0xff,
		.b = 0x3f
};

static const led_color_t led_blue = {
		.r = 0x00,
		.g = 0x00,
		.b = 0xff
};

static const led_color_t led_black = {
		.r = 0x00,
		.g = 0x00,
		.b = 0x00
};

static const led_color_t led_teal = {
		.r = 0x00,
		.g = 0x7f,
		.b = 0x7f
};

static const led_color_t led_pink = {
		.r = 0x7f,
		.g = 0x00,
		.b = 0x7f
};


static const led_color_t led_white = {
		.r = 0xff,
		.g = 0xff,
		.b = 0xff
};


static const led_color_t led_orange = {
		.r = 0xff,
		.g = 0xff,
		.b = 0x00
};



/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

uint16_t led_add_checkpoint(led_color_t color);

void led_checkpoint(uint16_t point);

void led_rgb_init(void);

void led_rgb_set_color(led_color_t color);

void led_rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b);

void led_feedback_init(void);

void led_rgb_thread(void * arg);

void led_set(uint8_t num);

void led_clear(uint8_t num);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* LED_H */

/* END */
