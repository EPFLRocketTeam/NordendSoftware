/*  Title		: GNSS
 *  Filename	: gnss.c
 *	Author		: iacopo sprenger
 *	Date		: 10.06.2022
 *	Version		: 0.1
 *	Description	: gnss setup and acquisition
 */

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

static volatile gnss_context_t gnss_decoder = {0};

static uint16_t checkpoint;


/**********************
 *	DECLARATIONS
 **********************/

gnss_return_t gnss_decode_gpgga(gnss_context_t * decoder) {

    switch (decoder->word_count) {

        case GNSS_GPGGA_LATITUDE:
            decoder->data.latitude = strtof((char *) decoder->accumulator, NULL);
            break;

        case GNSS_GPGGA_LONGITUDE:
            decoder->data.longitude = strtof((char *) decoder->accumulator, NULL);
            break;

        case GNSS_GPGGA_NS:
            if (decoder->accumulator[0] == 'S') {
                decoder->data.latitude = decoder->data.latitude * (-1);
            }
            break;

        case GNSS_GPGGA_EW:
            if (decoder->accumulator[0] == 'W') {
                decoder->data.longitude = decoder->data.longitude * (-1);
            }
            break;

        case GNSS_GPGGA_ALTITUDE:
            decoder->data.altitude = strtof((char *) decoder->accumulator, NULL);
            break;

        case GNSS_GPGGA_ALT_UNIT:
            if (decoder->accumulator[0] == 'F') {
                decoder->data.altitude = decoder->data.altitude * GNSS_FEET_CONVERSION;
            }
            decoder->stat = GNSS_SUCCESS;
            break;

        case GNSS_GPGGA_TIME:
            decoder->data.time = strtof((char *) decoder->accumulator, NULL);
            break;

        case GNSS_GPGGA_HDOP:
            decoder->data.hdop = strtof((char *) decoder->accumulator, NULL);
            break;
    }
    return GNSS_PROGRESS;
}

gnss_return_t gnss_decode_gprmc(gnss_context_t * decoder) {

    switch (decoder->word_count) {

        case GNSS_GPRMC_LATITUDE:
        	//debug_log("GNSS: LAT\n");
            decoder->data.latitude = strtof((char *) decoder->accumulator, NULL);
            break;

        case GNSS_GPRMC_LONGITUDE:
        	//debug_log("GNSS: LON\n");
            decoder->data.longitude = strtof((char *) decoder->accumulator, NULL);
            break;

        case GNSS_GPRMC_NS:
            if (decoder->accumulator[0] == 'S') {
                decoder->data.latitude = decoder->data.latitude * (-1);
            }
            break;

        case GNSS_GPRMC_EW:
            if (decoder->accumulator[0] == 'W') {
                decoder->data.longitude = decoder->data.longitude * (-1);
            }
            decoder->stat = GNSS_SUCCESS;
            break;

        case GNSS_GPRMC_TIME:
            decoder->data.time = strtof((char *) decoder->accumulator, NULL);
            decoder->data.altitude = 0;
            decoder->data.hdop = 0;
            break;
    }
    return GNSS_PROGRESS;
}


gnss_return_t gnss_handle_fragment(gnss_context_t * decoder, volatile uint8_t c) {

	//debug_log("gnss handle frag: %c\n", c);

    switch (c) {

        case '$':
            decoder->accu_count = 0 ;
            decoder->word_count = 0 ;
            decoder->stat = GNSS_PROGRESS;
            break;

        case ',':
            decoder->accumulator[decoder->accu_count] = '\0';
            //debug_log("gnss next: %s\n", decoder->accumulator);
            if(decoder->word_count == 0) {
                if (strcmp((char*)decoder->accumulator, "GPGGA") == 0) {
                	//debug_log("decoder: GPGGA\n");
                    decoder->type = GPGGA;
                }
                else if (strcmp((char*)decoder->accumulator, "GPRMC") == 0) {
                	//debug_log("decoder: GPRMC\n");
                    decoder->type = GPRMC;
                } else {
                    decoder->type = OTHER;
                }
            } else {
                switch (decoder->type) {
                    case GPGGA:
                        gnss_decode_gpgga(decoder);
                        break;
                    case GPRMC:
                        gnss_decode_gprmc(decoder);
                        break;
                    default:
                        break;
                }
            }
            decoder->word_count += 1;
            decoder->accu_count = 0;
            break;

        default:
        	decoder->stat = GNSS_PROGRESS;
            decoder->accumulator[decoder->accu_count] = c;
            decoder->accu_count++;
            break;
    }
    //debug_log("ret from gnss: %d\n", decoder->stat);
    return decoder->stat;
}




util_error_t gnss_handle_data(device_interface_t * gnss_interface, void * context) {
	led_set(0);
	uint32_t len = 1;
	util_error_t error;
	for(;;) {
		uint8_t data;
		len = 1;
		error = device_interface_recv(gnss_interface, &data, &len);
		if(error) {
			return error;
		}
		if(len == 1) {
			led_checkpoint(checkpoint);
			if(gnss_handle_fragment(&gnss_decoder, data) == GNSS_SUCCESS) {
				debug_log("GNSS: %lu\n", (uint32_t)gnss_decoder.data.altitude);
				od_write_GNSS(&gnss_decoder.data);
			}
		} else {
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



/* END */
