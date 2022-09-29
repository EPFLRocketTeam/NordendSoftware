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
#define GNSS_GGA_TIME 1
#define GNSS_GGA_LATITUDE 2
#define GNSS_GGA_NS 3
#define GNSS_GGA_LONGITUDE 4
#define GNSS_GGA_EW 5
#define GNSS_GGA_HDOP 8
#define GNSS_GGA_ALTITUDE 9
#define GNSS_GGA_ALT_UNIT 10
#define GNSS_FEET_CONVERSION 0.3048f


#define GNSS_RMC_TIME 1
#define GNSS_RMC_LATITUDE 3
#define GNSS_RMC_NS 4
#define GNSS_RMC_LONGITUDE 5
#define GNSS_RMC_EW 6

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
    GGA,
    RMC,
    OTHER
}gnss_trame_type_t;

typedef struct gnss_context {
    gnss_trame_type_t type;
    uint8_t accumulator[ACCU_SIZE];
    uint16_t accu_count;
    uint16_t word_count;
    gnss_data_t data;
    uint8_t done;
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

void gnss_decode_gga(gnss_context_t * decoder);
void gnss_decode_rmc(gnss_context_t * decoder);
void gnss_handle_fragment(gnss_context_t * decoder, uint8_t c);

util_error_t gnss_init(void);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* GNSS_H */

/* END */
