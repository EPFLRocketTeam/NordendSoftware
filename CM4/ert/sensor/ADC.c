/*  Title		: engine_pressure.c
 *  Filename	: engine_pressure.c
 *	Author		: Thilo Chalas, Luciano Calcoen
 *	Date		: 17.11.2022
 *	Version		: 0.1
 *	Description	: Engine nozzle pressure sensor setup and acquisition
 */

/**********************
 *	INCLUDES
 **********************/

#include <sensor/ADC.h>
#include <device/i2c_sensor.h>

/**********************
 *	CONSTANTS
 **********************/

#define A0_GND_ADDRESS 0b0010100
#define A0_VCC_ADDRESS 0b1010100
#define ADC_OUTPUT_BYTES 2

/**********************
 *	MACROS
 **********************/

/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/

static uint8_t adc_setup = 0b10000000;
static uint8_t adc_output[ADC_OUTPUT_BYTES];
static double adc_max_out = 0;

/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/

/**
 * @brief Read the ADC data and convert it to a voltage
 * 
 * @param adc 
 * @param data 
 * @param vref 
 * @param differential 
 * @return util_error_t 
 */

util_error_t adc_read_data(device_t * adc, double * data, const double & vref, const bool & differential){
    util_error_t error = ER_SUCCESS;
    device_interface_recv(adc->interface, adc_output , ADC_OUTPUT_BYTES);
    double data = 0;
    for(uint8_t ii=0; ii < ADC_OUTPUT_BYTES; ++ii){
        for(uint8_t jj=0; jj < 8; ++jj){
            data += (double) (adc[ii] & ((0b00000001)<<jj)) * (2**((ii+1)*jj)); //GOOD LUCK UNDERSTANDING MY SHIT LOL
        }
    }
    data = (data/adc_max_out) * vref;
    if(differential){
        data /= 2;
        data -= vref;
    }
    return error;
}

/**
 * @brief Initialize an ADC
 * 
 * @param adc 
 * @return util_error_t 
 */

util_error_t adc_init(device_t * adc){
    util_error_t error = ER_SUCCESS;
    adc_max_out = (double) ((2**(8*ADC_OUTPUT_BYTES))-1)
    device_interface_send(adc->interface, &adc_setup, 1);
    return error;
}








/* END */
