/*  Title		: ADC.c
 *  Filename	: ADC.c
 *	Author		: Thilo Chalas, Luciano Calcoen
 *	Date		: 17.11.2022
 *	Version		: 0.1
 *	Description	: ADC setup and voltage acquisition
 */

/**********************
 *	INCLUDES
 **********************/

#include <sensor/ADC.h>
#include <device/i2c_sensor.h>
#include <util.h>

/**********************
 *	CONSTANTS
 **********************/

#define ADC_RESOLUTION_BITS 12
#define ADC_OUTPUT_BYTES 3
//ADC PGA GAIN BITS (2 LSB bits)
#define PGA_GAIN_BITS 0b00000000
#define PGA_GAIN 1

/**********************
 *	MACROS
 **********************/

/**********************
 *	TYPEDEFS
 **********************/


/**********************
 *	VARIABLES
 **********************/

static uint8_t adc_setup = 0b10010000 | PGA_GAIN_BITS;
static uint8_t adc_output[ADC_OUTPUT_BYTES];
static uint8_t prev_channel = 0;
const double adc_voltage_range  = 4.096;
const uint32_t ADCmaxvalue = 1<<ADC_RESOLUTION_BITS;
const double adc_lsb_volt = (adc_voltage_range / ADCmaxvalue);
/**********************
 *	PROTOTYPES
 **********************/


/**********************
 *	DECLARATIONS
 **********************/


/**
 * @brief Convert a value from the ADC to a voltage
 * 
 * @param adc_value 
 * @return double 
 */

double convert_adc_to_volt(int16_t adc_value){
    return adc_value * adc_lsb_volt / PGA_GAIN;
}

/**
 * @brief Read the ADC data of a channel and convert it to a voltage
 * 
 * @param adc 
 * @param data 
 * @param channel  
 * @return util_error_t 
 */

util_error_t adc_read_voltage(device_t * adc, double * voltage, uint8_t channel){
    util_error_t error = ER_SUCCESS;
    //Select the proper ADC channel
    if (prev_channel != channel){
        uint8_t adc_channel = adc_setup | (channel ? 0b00100000 : 0b00000000);
        error |= device_interface_send(adc->interface, &adc_channel, 1);
        uint8_t dataready = 0;
        //Wait for the next conversion to be ready
        while (!dataready){
            error |= device_interface_recv(adc->interface, adc_output , ADC_OUTPUT_BYTES);
            if (adc_output[2] == (adc_channel & 0b01111111)){
                dataready = 1;
            }       
        }
    }
    
    //Read the data from the ADC
    error |= device_interface_recv(adc->interface, adc_output , ADC_OUTPUT_BYTES);
    
    //Convert the value to a voltage
    int16_t adc_value = (adc_output[0]<<8 | adc_output[1]);
    *voltage = convert_adc_to_volt(adc_value);

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
    error |= device_interface_send(adc->interface, &adc_setup, 1);
    return error;
}









/* END */
