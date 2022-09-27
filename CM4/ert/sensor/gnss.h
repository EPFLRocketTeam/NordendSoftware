/*  Title		: GNSS
 *  Filename	: gnss.h
 *	Author		: iacopo sprenger
 *	Date		: 10.06.2022
 *	Version		: 0.1
 *	Description	: gnss setup and acquisition
 */

#ifndef GNSS_H
#define GNSS_H



/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <util.h>

/**********************
 *  CONSTANTS
 **********************/

#define ACCU_SIZE 128
#define GNSS_GPGGA_TIME 1
#define GNSS_GPGGA_LATITUDE 2
#define GNSS_GPGGA_NS 3
#define GNSS_GPGGA_LONGITUDE 4
#define GNSS_GPGGA_EW 5
#define GNSS_GPGGA_HDOP 8
#define GNSS_GPGGA_ALTITUDE 9
#define GNSS_GPGGA_ALT_UNIT 10
#define GNSS_FEET_CONVERSION 0.3048f


#define GNSS_GPRMC_TIME 1
#define GNSS_GPRMC_LATITUDE 3
#define GNSS_GPRMC_NS 4
#define GNSS_GPRMC_LONGITUDE 5
#define GNSS_GPRMC_EW 6

/**********************
 *  MACROS
 **********************/


/**********************
 *  TYPEDEFS
 **********************/

/*
typedef struct gnss_data {
	float lat;
	float lon;
	int32_t alt;

}gnss_data_t;
*/

typedef struct gnss_data {
    float longitude;
    float latitude;
    float altitude;
    float time;
    float hdop;
}gnss_data_t;

typedef enum gnss_trame_type {
    GPGGA,
    GPRMC,
    OTHER
}gnss_trame_type_t;

typedef enum gnss_return {
	GNSS_PROGRESS = 0,
	GNSS_SUCCESS = 1
}gnss_return_t;

typedef struct gnss_context {
    gnss_trame_type_t type;
    uint8_t accumulator[ACCU_SIZE];
    uint16_t accu_count;
    uint16_t word_count;
    gnss_data_t data;
    gnss_return_t stat;
} gnss_context_t;




/**********************
 *  VARIABLES
 **********************/


/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C"{
#endif

gnss_return_t gnss_decode_gpgga(gnss_context_t * decoder);
gnss_return_t gnss_decode_gprmc(gnss_context_t * decoder);
gnss_return_t gnss_handle_fragment(gnss_context_t * decoder, uint8_t c);

util_error_t gnss_init(void);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* GNSS_H */

/* END */
