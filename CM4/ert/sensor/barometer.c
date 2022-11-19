/*  Title		: barometer
 *  Filename	: barometer.c
 *	Author		: Julian Marmier, adopted from Iacopo Sprenger
 *	Date		: 19.11.2022
 *	Version		: 0.1
 *	Description	: Interface for the Bosch BMP390 barometric pressure sensor
 */

/**********************
 *	INCLUDES
 **********************/

#include <sensor/barometer.h>
#include <util.h>
#include <feedback/debug.h>
#include <math.h>

/**********************
 *	CONSTANTS
 **********************/

/**
 * Subsampling rate configuration data register
 */
#define ODR_SEL 0x1D

/**
 * Oversampling rate configuration data register
 */
#define OSR 0x1C

/**
 * IIR filter configuration register address
 */
#define CONFIG 0x1F

/**
 * Power control register address
 */
#define PWR_CTRL 0x1B // Power control to activate the sensor

/**
 * Temperature and pressure data addresses
 */
#define DATA_READ 0x04 // Burst read registers 0x04 to 0x09 (total 6 bytes)

/**
 * Calibration data (stored in NVM) burst read start address
 */
#define NVM_READ 0x31 // Calibration data address

/**
 * Total number of bytes to read from the calibration data
 */
#define NVM_TOTAL_BYTES 21

/**
 * Total number of bytes to read from the sensor data registers
 */
#define DATA_TOTAL_BYTES 6

/**
 * The standard pressure at sea level, in pascals.
 * For much better precision, the GNSS initial altitude data
 * should be used to better reflect current conditions.
 */
static const float PRESSURE_AT_SEA_LEVEL = 101325;

/**********************
 *	MACROS
 **********************/

/**********************
 *	TYPEDEFS
 **********************/

/**********************
 *	VARIABLES
 **********************/

static float baro_initial_altitude = 100.0f; // Default value, this will change.
static uint8_t baro_init_measure = 0; // By default don't create an initial measurement

/**********************
 *	PROTOTYPES
 **********************/

/**********************
 *	DECLARATIONS
 **********************/

/**
 * Helper functions (should either be incorporated into util
 * or replaced by actual functions)
 */

/**
 * Returns an unsigned 16-bit integer from the two given unsigned bytes
 * Note the little-endian format.
 * @param msb the most significant byte
 * @param lsb the least significant byte
 * @return an unsigned 16-bit integer combining the two given bytes.
 */
static uint16_t create_u16(uint8_t msb, uint8_t lsb) {
	return ((uint16_t) msb << 8) | lsb;
}

/**
 * Returns an unsigned 32-bit integer from the three given
 * signed bytes (useful for oversampling).
 * Note the little-endian format.
 * @param msb the most significant byte
 * @param mid the middle byte
 * @param lsb the least significant byte
 * @return an unsigned 32-bit integer combining the three given bytes
 */
static u_int32_t create_u32_from24(uint8_t msb, uint8_t mid, uint8_t lsb) {
	return ((uint32_t) msb << 16) | ((uint32_t) mid << 8) | lsb;
}

/**
 * Returns an signed 16-bit integer from the two given unsigned bytes
 * Note the little-endian format.
 * @param a the most significant bits
 * @param b the least significant bits
 * @return an signed 16-bit integer combining the two given unsigned bytes
 */
static int16_t create_i16(uint8_t msb, uint8_t lsb) {
	return (int16_t) create_u16(msb, lsb);
}

// --------------

/**
 * Computes the calibration coefficient as according
 * to section 8.4 of the datasheet. Mathematically this is equivalent to nvm_data/(2^div).
 * @param nvm_data the binary/signed data received from the device.
 * @param div the power of 2 to divide by.
 * @return nvm_data/(2^div).
 */
static float compute_coefficient(int32_t nvm_data, int32_t div) {
	// TODO this can be replaced with ldexpf
	int divider = div < 0 ? 1 >> -div : 1 << div;

	return (float) nvm_data / (float) divider;
}

/**
 * Calculates the altitude (in m) in terms of the given value.
 * @param pressure the pressure in pascals.
 * @return the corresponding altitude, in meters.
 */
static float calculate_altitude(float pressure, barometer_meta_t *meta) {
	// TODO specify in documentation that barometer_calib_data_t expects a pressure_sea_level value in pascals
	float p0 =
			meta->pressure_sea_level == NULL ?
					PRESSURE_AT_SEA_LEVEL : meta->pressure_sea_level;

	return 44330.0f * (1.0f - powf(pressure / p0, 0.1903));

	// TODO check if we want relative altitude (using barometer_set_alt) or not
}

/**
 * Uses the initial altitude supplied by the object dictionary
 * to calculate the pressure at sea level
 */
static float calculate_initial_pressure(float pressure, barometer_meta_t *meta) {
		meta->pressure_sea_level = pressure
				* powf(1.0f - (baro_initial_altitude / 44330.0f), -5.2548f);
		baro_init_measure -= 1;
}

/**
 * Reads data from the sensor and updates measurements in final_data.
 * Everything can be done in a single go without penalizing measurement cycles.
 * @param baro The device instance associated with the barometer.
 * @param calib The barometer calibration data.
 * @param final_data The data at the end.
 * @return an error status.
 */
util_error_t barometer_read(device_t *baro, barometer_meta_t *meta,
		barometer_data_t *data) {
	uint8_t read_in_data[DATA_TOTAL_BYTES]; // 6 bytes for both pressure and temperature

	// Burst read temp and pressure for a total of 6 bytes, load the result into read_in_data
	util_error_t error = device_read(baro, DATA_READ, read_in_data,
	DATA_TOTAL_BYTES);

	// Load resulting read_in_data into respective final_data locations
	// NOTE even though we don't oversample temperature, it's still necessary to include XLSB for compatibility
	data->temperature = barometer_compensate_temperature(
			create_u32_from24(read_in_data[5], read_in_data[4],
					read_in_data[3]), meta);

	data->pressure = barometer_compensate_pressure(
			create_u32_from24(read_in_data[2], read_in_data[1],
					read_in_data[0]), meta);


	// Check if the initial altitude has been reset
	if (baro_init_measure)
		calculate_initial_pressure(data->pressure, meta);


	debug_log("Pressure: sea level %ld / current %ld\n", meta->pressure_sea_level, data->pressure);

	// Calculate the altitude
	data->altitude = calculate_altitude(data->pressure, meta);

	// Store current clock tick
	data->timestamp = HAL_GetTick();

	return error;
}

/**
 * Set the current altitude of the barometer
 */
void barometer_set_alt(float alt) {
	baro_init_measure = 1;
	baro_initial_altitude = alt;
}

/**
 * Initializes the barometer.
 * @param baro the device associated with the barometer.
 * @param meta the metadata of the barometer.
 * @return an error status.
 */
util_error_t barometer_init(device_t *baro, barometer_meta_t *meta) {
	uint8_t data[NVM_TOTAL_BYTES];

	// Burst read calibration data from non-volatile memory
	util_error_t errCalib = device_read(baro, NVM_READ, data, NVM_TOTAL_BYTES);

	// Wait 10ms
	osDelay(pdMS_TO_TICKS(10));

	// Load data into meta for future data treatment, see datasheet sections 3.11 and 8.4
	// TODO - maybe it's better to send multiple reads ?
	meta->par_t1 = compute_coefficient(create_u16(data[1], data[0]), -8);
	meta->par_t2 = compute_coefficient(create_u16(data[3], data[2]), 30);
	meta->par_t3 = compute_coefficient((int8_t) data[4], 48);
	meta->par_p1 = compute_coefficient(create_i16(data[6], data[5]), 20);
	meta->par_p2 = compute_coefficient(create_i16(data[8], data[7]), 29);
	meta->par_p3 = compute_coefficient((int8_t) data[9], 32);
	meta->par_p4 = compute_coefficient((int8_t) data[10], 37);
	meta->par_p5 = compute_coefficient(create_u16(data[12], data[11]), -3);
	meta->par_p6 = compute_coefficient(create_u16(data[14], data[13]), 6);
	meta->par_p7 = compute_coefficient((int8_t) data[15], 8);
	meta->par_p8 = compute_coefficient((int8_t) data[16], 15);
	meta->par_p9 = compute_coefficient(create_i16(data[19], data[18]), 48);
	meta->par_p7 = compute_coefficient((int8_t) data[20], 48);
	meta->par_p8 = compute_coefficient((int8_t) data[21], 65);

	// Set the IIR filter coefficient to 2
	util_error_t errIIR = device_write_u8(baro, CONFIG, 0x02);

	// Set the prescaler for sampling to 0 (=> 100 Hz sampling) for now
	// TODO change this to 0 later on when the modifications will have been decided in sensor.c
	util_error_t errODR = device_write_u8(baro, ODR_SEL, 0x01);

	// Set pressure oversampling to x2, temp to x1 (no oversampling)
	util_error_t errOSR = device_write_u8(baro, OSR, 0x01);

	// Activate the barometer by setting MODE bits to "11" and activating p_en and t_en
	util_error_t errActivation = device_write_u8(baro, PWR_CTRL, 0b011011);

	// Return total error status
	return errCalib | errIIR | errODR | errOSR | errActivation;
}

/* END */
