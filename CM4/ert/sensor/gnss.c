/*  Title		: GNSS
 *  Filename	: gnss.c
 *	Author		: iacopo sprenger
 *	Date		: 10.06.2022
 *	Version		: 0.1
 *	Description	: gnss setup and acquisition
 */

#if 0

/**********************
 *	INCLUDES
 **********************/

#include <sensor/gnss.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <util.h>
#include <device/device.h>
#include <driver/serial.h>
#include <od/od.h>
#include <feedback/debug.h>
#include <feedback/led.h>
#include <hostcom.h>
#include <math.h>
/**********************
 *	CONSTANTS
 **********************/


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

//static gnss_context_t gnss_decoder = {0};

static uint16_t checkpoint;

//static float last_alt = 100;

#define WEIRD_THRESH 100


/**********************
 *	DECLARATIONS
 **********************/


float read_lat(char * string){
	float min = strtof(string+2, NULL);
	string[2] = '\0';
	float deg = strtof(string, NULL);

	return deg + min/60.0f;
}

float read_lon(char * string){
	float min = strtof(string+3, NULL);
	string[3] = '\0';
	float deg = strtof(string, NULL);

	return deg + min/60.0f;
}

void gnss_decode_gga(gnss_context_t * decoder) {

    switch (decoder->word_count) {

        case GNSS_GGA_LATITUDE:
            decoder->data.latitude = read_lat((char *) decoder->accumulator);
            break;

        case GNSS_GGA_LONGITUDE:
            decoder->data.longitude = read_lon((char *) decoder->accumulator);
            break;

        case GNSS_GGA_NS:
            if (decoder->accumulator[0] == 'S') {
                decoder->data.latitude = decoder->data.latitude * (-1);
            }
            break;

        case GNSS_GGA_EW:
            if (decoder->accumulator[0] == 'W') {
                decoder->data.longitude = decoder->data.longitude * (-1);
            }
            break;

        case GNSS_GGA_ALTITUDE:
            decoder->data.altitude = strtof((char *) decoder->accumulator, NULL);
            break;

        case GNSS_GGA_ALT_UNIT:
            if (decoder->accumulator[0] == 'F') {
                decoder->data.altitude = decoder->data.altitude * GNSS_FEET_CONVERSION;
            }
            decoder->done = 1;
            break;

        case GNSS_GGA_TIME:
            decoder->data.time = strtof((char *) decoder->accumulator, NULL);
            break;

        case GNSS_GGA_HDOP:
            decoder->data.hdop = strtof((char *) decoder->accumulator, NULL);
            break;
    }
}

void gnss_decode_rmc(gnss_context_t * decoder) {

    switch (decoder->word_count) {

        case GNSS_RMC_LATITUDE:
        	//debug_log("GNSS: LAT\n");
            decoder->data.latitude = read_lat((char *) decoder->accumulator);
            break;

        case GNSS_RMC_LONGITUDE:
        	//debug_log("GNSS: LON\n");
            decoder->data.longitude = read_lon((char *) decoder->accumulator);
            break;

        case GNSS_RMC_NS:
            if (decoder->accumulator[0] == 'S') {
                decoder->data.latitude = decoder->data.latitude * (-1);
            }
            break;

        case GNSS_RMC_SPEED:
               decoder->data.speed = strtof((char *) decoder->accumulator, NULL)*0.514;
               break;

        case GNSS_RMC_EW:
            if (decoder->accumulator[0] == 'W') {
                decoder->data.longitude = decoder->data.longitude * (-1);
            }
            decoder->done = 1;
            break;

        case GNSS_RMC_TIME:
            decoder->data.time = strtof((char *) decoder->accumulator, NULL);
            decoder->data.altitude = 0;
            decoder->data.hdop = 0;
            break;
    }
}


void gnss_handle_fragment(gnss_context_t * decoder, volatile uint8_t c) {

	//debug_log("gnss handle frag: %c\n", c);

    switch (c) {

        case '$':
            decoder->accu_count = 0;
            decoder->word_count = 0;
            decoder->done = 0;
            break;

        case ',':
            decoder->accumulator[decoder->accu_count] = '\0';
            if(decoder->word_count == 0) {
            	//debug_log("gnss next: %s\n", decoder->accumulator);
                if (strcmp((char*)decoder->accumulator, "GNGGA") == 0) {
                	//debug_log("decoder: GGA\n");
                    decoder->type = GGA;
                }
                else if (strcmp((char*)decoder->accumulator, "GNRMC") == 0) {
                	//debug_log("decoder: RMC\n");
                    decoder->type = RMC;
                } else {
                    decoder->type = OTHER;
                }
            } else {
                switch (decoder->type) {
                    case GGA:
                        gnss_decode_gga(decoder);
                        break;
                    case RMC:
                        gnss_decode_rmc(decoder);
                        break;
                    default:
                        break;
                }
            }
            decoder->word_count += 1;
            decoder->accu_count = 0;
            break;

        default:
            decoder->accumulator[decoder->accu_count] = c;
            decoder->accu_count++;
            break;
    }
    //debug_log("ret from gnss: %d\n", decoder->stat);
}




util_error_t gnss_handle_data(device_interface_t * gnss_interface, void * context) {
	UNUSED(context);
	uint32_t len = 1;
	util_error_t error;
	for(;;) {
		uint8_t data;
		len = 1;
		error = device_interface_recv(gnss_interface, &data, &len);
		if(error) {
			return error;
		}
//		if(len == 1) {
//			led_checkpoint(checkpoint);
//			gnss_handle_fragment(&gnss_decoder, data);
//			if(gnss_decoder.done) {
//				debug_log("done!\n");
//				debug_log("GNSS: %lu | %g, %g\n", (uint32_t)gnss_decoder.data.altitude,
//							gnss_decoder.data.latitude, gnss_decoder.data.longitude);
//				if(gnss_decoder.data.altitude != 0) {
//					if(fabs(gnss_decoder.data.altitude - last_alt) < WEIRD_THRESH) {
//						last_alt = gnss_decoder.data.altitude;
//						hostcom_data_gnss_send(HAL_GetTick(), (int32_t)gnss_decoder.data.altitude);
//						static uint8_t first = 1;
//						if(first) {
//							barometer_set_alt(gnss_decoder.data.altitude);
//							first = 0;
//						}
//					}
//				}
//				od_write_GNSS(&gnss_decoder.data);
//				gnss_decoder.done = 0;
//			}
//		}
		else {
			return ER_SUCCESS;
		}
	}

}


void gnss_pins_guard(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}


util_error_t gnss_init(void) {

	device_interface_t * gnss_interface = serial_get_s1_interface();

	checkpoint = led_add_checkpoint(led_orange);


	serial_register_handler(gnss_interface, gnss_handle_data, NULL);

	return ER_SUCCESS;
}

#endif

/* END */
