#ifndef SENSOR_MCP3426_H
#define SENSOR_MCP3426_H

#include <util.h>
#include <device/device.h>
#include <od/data_types.h>



#define MCP3422_CHANNEL_MASK	0x60
#define MCP3422_PGA_MASK		0x03
#define MCP3422_SRATE_MASK		0x0C

#define MCP3422_SRATE_240		0x0
#define MCP3422_SRATE_60		0x1
#define MCP3422_SRATE_15		0x2
#define MCP3422_SRATE_3			0x3
#define MCP3422_PGA_1			0
#define MCP3422_PGA_2			1
#define MCP3422_PGA_4			2
#define MCP3422_PGA_8			3

#define MCP3422_CONT_SAMPLING	0x10


typedef struct mcp3426_adc_context {
	uint8_t hw_available;
	uint16_t selected_srate;
	uint8_t num_bits;
	float selected_sstvt;
}mcp3426_adc_context_t;




util_error_t mcp3425_adc_init(device_t * dev, mcp3426_adc_context_t * ctx);

util_error_t mcp3425_adc_read(device_t * dev, sensor_baro_data_t * data);


#endif /* SENSOR_MCP3426_H */
