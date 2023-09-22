#ifndef SENSOR_MCP3426_H
#define SENSOR_MCP3426_H

#include <util.h>
#include <device/device.h>
#include <od/data_types.h>


typedef struct mcp3426_adc_context {
	uint8_t hw_available;
	uint16_t selected_range;
	float selected_sstvt;
}mcp3426_adc_context_t;




util_error_t mcp3425_adc_init(device_t * dev, mcp3426_adc_context_t * ctx);

util_error_t mcp3425_adc_read(device_t * dev, sensor_baro_data_t * data);


#endif /* SENSOR_MCP3426_H */
