/*  Title		: Accelerometer
 *  Filename	: accelerometer.c
 *	Author		: iacopo sprenger, Thilo Chalas, Luciano Calcoen
 *	Date		: 08.11.2022
 *	Version		: 0.2
 *	Description	: accelerometer setup and acquisition
 */

/**********************
 *	INCLUDES
 **********************/

#include <sensor/accelerometer.h>
#include <device/i2c_sensor.h>

/**********************
 *	CONSTANTS
 **********************/

#define WHO_AM_I		    0x00 //DEVICE ID
#define WHO_AM_I_MAGIC	    0xe5 //I guess this is supposed to be the 0x00 register's reset value?
#define THRESH_SHOCK        0x1d //SEE ADXL375 datasheet for details
#define OFSX                0x1e //SEE ADXL375 datasheet for details
#define OFSY                0x1f //SEE ADXL375 datasheet for details
#define OFSZ                0x20 //SEE ADXL375 datasheet for details
#define DUR                 0x21 //SEE ADXL375 datasheet for details
#define LATENT              0x22 //SEE ADXL375 datasheet for details
#define WINDOW              0x23 //SEE ADXL375 datasheet for details
#define THRESH_ACT          0x24 //SEE ADXL375 datasheet for details
#define THRESH_INACT        0x25 //SEE ADXL375 datasheet for details
#define TIME_INACT          0x26 //SEE ADXL375 datasheet for details
#define ACT_INACT_CTL       0x27 //SEE ADXL375 datasheet for details
#define SHOCK_AXES          0x2a //SEE ADXL375 datasheet for details
#define ACT_SHOCK_STATUS    0x2b //SEE ADXL375 datasheet for details
#define BW_RATE             0x2c //SEE ADXL375 datasheet for details
#define POWER_CTL           0x2d //SEE ADXL375 datasheet for details
#define INT_ENABLE          0x2e //SEE ADXL375 datasheet for details
#define INT_MAP             0x2f //SEE ADXL375 datasheet for details
#define INT_SOURCE          0x30 //SEE ADXL375 datasheet for details
#define DATA_FORMAT         0x31 //SEE ADXL375 datasheet for details
#define DATAX0              0x32 //SEE ADXL375 datasheet for details
#define DATAX1              0x33 //SEE ADXL375 datasheet for details
#define DATAY0              0x34 //SEE ADXL375 datasheet for details
#define DATAY1              0x35 //SEE ADXL375 datasheet for details
#define DATAZ0              0x36 //SEE ADXL375 datasheet for details
#define DATAZ1              0x37 //SEE ADXL375 datasheet for details
#define FIFO_CTL            0x38 //SEE ADXL375 datasheet for details
#define FIFO_STATUS         0x39 //SEE ADXL375 datasheet for details



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

//acceleromter calibration to be coded later
util_error_t accelerometer_calibrate(device_t * acc, accelerometer_calibration_data_t * dataCalibration, accelerometer_data_t * data) {
	util_error_t error = ER_SUCCESS;
	static int valueCount = 0;

	uint8_t tmp[6];
	error |= device_read(acc, DATAX0, tmp, 6);
	dataCalibration->axeXRaw[valueCount] = tmp[1] | (tmp[0]<<8);
	dataCalibration->axeYRaw[valueCount] = tmp[3] | (tmp[2]<<8);
	dataCalibration->axeZRaw[valueCount] = tmp[5] | (tmp[4]<<8);

	valueCount = valueCount + 1;

	if (valueCount >= CALIBRATION_VALUE_COUNT) {
		double somme = 0, somme2 = 0, somme3 = 0, divid = 49.0;
		for (int j = 0; j < CALIBRATION_VALUE_COUNT; j++) {
		  somme += dataCalibration->axeXRaw[j];
		  somme2 += dataCalibration->axeYRaw[j];
		  somme3 += dataCalibration->axeZRaw[j];
		}
		double offset = -(somme/divid), offset2 = -(somme2/divid), offset3 = 20.5 - (somme3/divid);

		byte intOffset = round(offset)/4;
		byte intOffset2 = round(offset2)/4;
		byte intOffset3 = round(offset3)/4;

		error |= device_write_u8(acc, OFSX, intOffset);
	    error |= device_write_u8(acc, OFSY, intOffset2);
	    error |= device_write_u8(acc, OFSZ, intOffset3);


	    date->state = READY;
	}


	return error;
}

void accelerometer_process_data(accelerometer_data_t * data, uint16_t range) {
	for (uint8_t i = 0; i < ACC_AXIS_COUNT; i++) {
		data->processed[i] = ((int64_t)data->raw[i] * range) >> 15;
	}
}

util_error_t accelerometer_read_data(device_t * acc, accelerometer_data_t * data) {
	//i2c_sensor_context_t * context = (i2c_sensor_context_t *) acc->context;
	util_error_t error = ER_SUCCESS;
	uint8_t tmp[6];
	error |= device_read(acc, DATAX0, tmp, 6);
	data->raw[ACC_X] = tmp[1] | (tmp[0]<<8);
	data->raw[ACC_Y] = tmp[3] | (tmp[2]<<8);
	data->raw[ACC_Z] = tmp[5] | (tmp[4]<<8);
	data->timestamp = HAL_GetTick();

	return error;
}

/**
 * @brief Initialize accelerometers
 */
util_error_t accelerometer_init(device_t * acc) {
	util_error_t error = ER_SUCCESS;
	uint8_t data; //read sensor magic number
	error |= device_read_u8(acc, WHO_AM_I, &data);

	if(data != WHO_AM_I_MAGIC) {
		return error | ER_RESSOURCE_ERROR;
	}

	//initialize sensor
	//Set the accelerometer offset values (set to 0, should be calibrated later? maybe?)
	error |= device_write_u8(acc, OFSX, 0b00000000);
    error |= device_write_u8(acc, OFSY, 0b00000000);
    error |= device_write_u8(acc, OFSZ, 0b00000000);
    //Set the data output rate (for now set to default 100Hz but we might be able to do better?)
    error |= device_write_u8(acc, BW_RATE, 0b00001010);
    //Activate measurement mode
    error |= device_write_u8(acc, POWER_CTL, 0b00001000);
    //Set the data format (same as default)
    error |= device_write_u8(acc, DATA_FORMAT, 0b00001011);
    //Setup the FIFO sampling mode
    error |= device_write_u8(acc, FIFO_CTL, 0b10000000);

	//Maybe add a self test routine in this part

	return error;
}

/* END */
