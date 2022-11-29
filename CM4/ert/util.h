/*  Title       : error codes
 *  Filename    : error.h
 *  Author      : iacopo sprenger
 *  Date        : 21.03.2022
 *  Version     : 0.1
 *  Description : 
 */

#ifndef UTIL_H
#define UTIL_H

//#define UTIL_ALLOW_LIST

/**********************
 *  INCLUDES
 **********************/

#include <stdint.h>
#include <main.h>
#include <cmsis_os.h>

/**********************
 *  CONSTANTS
 **********************/

/**********************
 *  MACROS
 **********************/

/**
 * @brief	Macro to write masked data into a register.
 *
 */
#define WRITE_IN_REG(reg, mask, data) (reg) &= ~(mask); (reg) |= (data)

#ifdef UTIL_ALLOW_LIST

#define util_list_attribute \
    struct util_list_t * next

#define util_list_foreach(head, node) \
    for(util_list_t * node = head; node->next != NULL; node = node->next)

#endif

/**
 * @brief	Macro to enter a critical section
 */
#define ENTER_CRITICAL taskENTER_CRITICAL

/**
 * @brief	Macro to exit a critical section
 */
#define EXIT_CRITICAL taskEXIT_CRITICAL

/**********************
 *  TYPEDEFS
 **********************/

/**
 * @brief Unified error codes for the whole Nordend project.
 * @note  The error codes can be ORed together to create more complex errors.
 */
typedef enum util_error {
	/** Operation completed successfully */
	ER_SUCCESS = 0,
	/** Error due to lack of readiness */
	ER_DATA_NOT_RDY = 1 << 1,
	/** Error due to a generic failure */
	ER_FAILURE = 1 << 2,
	/** Error due to a range issue */
	ER_OUT_OF_RANGE = 1 << 3,
	/** Error due to a timeout */
	ER_TIMEOUT = 1 << 4,
	/** Error due to a resource issue */
	ER_RESOURCE_ERROR = 1 << 5
} util_error_t;

#ifdef UTIL_ALLOW_LIST

typedef struct util_list {
	struct util_list_t * next;
}util_list_t;

#endif

/**
 * @enum util_endianness
 * @brief Determines whether the data buffer input is
 * little endian (data[0] is LSB) or big endian (data[0] is MSB)
 *
 */
typedef enum util_endianness {
	/**
	 * @brief Represents a little endian data format, i.e. bytes are read as LSB first.
	 *
	 */
	UTIL_LITTLE_ENDIAN = 0,/**< UTIL_LITTLE_ENDIAN */
	/**
	 * @brief Represents a big endian format, i.e. bytes are read as MSB first.
	 *
	 */
	UTIL_BIG_ENDIAN = 1 /**< UTIL_BIG_ENDIAN */
} util_endianness_t;
//buffers

/**
 * @struct util_buffer_u8
 * @brief Buffer of 8 bit unsigned integers
 *
 */
typedef struct util_buffer_u8 {
	uint16_t current_write_index;
	uint16_t current_read_index;
	uint16_t bfr_len;
	uint8_t *buffer;
	util_endianness_t endianness;
} util_buffer_u8_t;

/**
 * @struct util_buffer_u16
 * @brief Buffer of 16 bit unsigned integers
 *
 */
typedef struct util_buffer_u16 {
	uint16_t current_write_index;
	uint16_t current_read_index;
	uint16_t bfr_len;
	uint16_t *buffer;
	util_endianness_t endianness;
} util_buffer_u16_t;

/**
 * @struct util_buffer_i16
 * @brief Buffer of 16 bit signed integers
 *
 */
typedef struct util_buffer_i16 {
	uint16_t current_write_index;
	uint16_t current_read_index;
	uint16_t bfr_len;
	int16_t *buffer;
} util_buffer_i16_t;

/**********************
 *  VARIABLES
 **********************/

/**********************
 *  PROTOTYPES
 **********************/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef UTIL_ALLOW_LIST
/** head must be of util_list_t type with the next field as the head
 *  any node with the util_list_attribute can be cast to util_list_t
 *  
 * 
 */
static inline void util_list_append(util_list_t** head, util_list_t* node)
{
    util_list_t** p = (util_list_t**) head;
    node->next = NULL;
    while((*p) != NULL) {
        p = (util_list_t**) &(*p)->next;
    }
    *p = node;
}

/** head must be of util_list_t type with the next field as the head
 *  any node with the util_list_attribute can be cast to util_list_t
 * 
 * 
 */
static inline void util_list_remove(util_list_t** head, util_list_t* node)
{
    util_list_t** p = (util_list_t**) head;
    while((*p) != 0) {
        p = (util_list_t**) &(*p)->next;
        if((*p) == node) break;
    }
    *p = node->next; //this remove the node from the list
}

#endif


/**
 * @fn void util_encode_u8(uint8_t*, uint8_t)
 * @brief Encodes
 *
 * @param data
 * @param value
 */
static inline void util_encode_u8(uint8_t *data, uint8_t value) {
	data[0] = value;
}

static inline void util_encode_u16(uint8_t *data, uint16_t value) {
	data[0] = value;
	data[1] = value >> 8;
}

static inline void util_encode_u32(uint8_t *data, uint32_t value) {
	data[0] = value;
	data[1] = value >> 8;
	data[2] = value >> 16;
	data[3] = value >> 24;
}

static inline void util_encode_i8(uint8_t *data, int8_t value) {
	data[0] = value;
}

static inline void util_encode_i16(uint8_t *data, int16_t value) {
	data[0] = value;
	data[1] = value >> 8;
}

static inline void util_encode_i32(uint8_t *data, int32_t value) {
	data[0] = value;
	data[1] = value >> 8;
	data[2] = value >> 16;
	data[3] = value >> 24;
}

static inline uint8_t util_decode_u8(uint8_t *data) {
	return data[0];
}

static inline uint16_t util_decode_u16(uint8_t *data) {
	return (uint16_t) data[0] | data[1] << 8;
}

static inline uint32_t util_decode_u24(uint8_t *data) {
	return (uint32_t) data[0] | data[1] << 8 | data[2] << 16;
}

static inline uint32_t util_decode_u32(uint8_t *data) {
	return (uint32_t) data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
}

static inline int8_t util_decode_i8(uint8_t *data) {
	return data[0];
}

static inline int16_t util_decode_i16(uint8_t *data) {
	return (int16_t) data[0] | data[1] << 8;
}

static inline int32_t util_decode_i32(uint8_t *data) {
	return (int32_t) data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
}

//U8 BUFFER


static inline void util_buffer_u8_init(util_buffer_u8_t *bfr, uint8_t *buffer,
		uint16_t bfr_len) {
	bfr->current_write_index = 0;
	bfr->current_read_index = 0;
	bfr->bfr_len = bfr_len;
	bfr->buffer = buffer;
}

/**
 * @fn void util_buffer_check_write
 * @brief Checks if the write index of the buffer is correct, otherwise will set it to 0 (beginning).
 *
 * @param bfr The buffer to check.
 */
void util_buffer_check_write(util_buffer_u8_t *bfr) {
	if (bfr->current_write_index >= bfr->bfr_len)
		bfr->current_write_index = 0;
}

/**
 * @fn void util_buffer_check_read(util_buffer_u8_t*)
 * @brief Checks if the read index of the buffer is correct, otherwise will set it to 0 (beginning).
 *
 * @param bfr The buffer to check.
 */
void util_buffer_check_read(util_buffer_u8_t *bfr) {
	if (bfr->current_read_index == bfr->bfr_len)
		bfr->current_read_index = 0;
}

static inline void util_buffer_u8_add(util_buffer_u8_t *bfr, uint8_t d) {
	bfr->buffer[bfr->current_write_index++] = d;
	util_buffer_check_write(bfr);
}


static inline uint8_t util_buffer_u8_get(util_buffer_u8_t *bfr) {
	uint8_t tmp = bfr->buffer[bfr->current_read_index++];
	util_buffer_check_read(bfr);
	return tmp;
}

/**
 * @fn uint16_t util_buffer_u8_get_u16(util_buffer_u8_t*)
 * @brief Gets the next unsigned 16-bit integer from the buffer.
 *
 * @param bfr The buffer to read from.
 * @return an unsigned 16-bit integer decoded with respect to the buffer's endianness.
 */
static inline uint16_t util_buffer_u8_get_u16(util_buffer_u8_t *bfr) {
	uint8_t data[2];

	for (int i = 0; i < 2; i++) {
		data[i] = bfr->buffer[bfr->current_read_index++];
		util_buffer_check_read(bfr);
	}

	return util_decode_u16(data); // TODO implement endianness !
}

/*!
 * @fn uint16_t util_buffer_u8_get_i16(util_buffer_u8_t*)
 * @brief Gets the next signed 16-bit integer from the buffer.
 *
 * @param bfr The buffer to read from.
 * @return a signed 16-bit integer decoded with respect to the buffer's endianness.
 */
static inline int16_t util_buffer_u8_get_i16(util_buffer_u8_t *bfr) {
	uint8_t data[2];

	for (int i = 0; i < 2; i++) {
		data[i] = bfr->buffer[bfr->current_read_index++];
		util_buffer_check_read(bfr);
	}


	return util_decode_i16(data);
}

/**
 * @fn uint32_t util_buffer_u8_get_u24(util_buffer_u8_t*)
 * @brief Gets the next signed 24-bit integer (as 32-bit int) from the buffer.
 *
 * @param bfr The buffer to read from.
 * @return an unsigned 24-bit integer decoded with respect to the buffer's endianness.
 */
static inline uint32_t util_buffer_u8_get_u24(util_buffer_u8_t *bfr) {
	uint8_t data[3];

	for (int i = 0; i < 3; i++) {
		data[i] = bfr->buffer[bfr->current_read_index++];
		util_buffer_check_read(bfr);
	}

	return util_decode_u24(data);
}

/**
 * @fn uint32_t util_buffer_u8_get_u32(util_buffer_u8_t*)
 * @brief Gets the next unsigned 32-bit integer from the buffer.
 *
 * @param bfr The buffer to read from.
 * @return an unsigned 32-bit integer decoded with respect to the buffer's endianness.
 */
static inline uint32_t util_buffer_u8_get_u32(util_buffer_u8_t *bfr) {
	uint8_t data[4];

	for (int i = 0; i < 4; i++) {
		data[i] = bfr->buffer[bfr->current_read_index++];
		util_buffer_check_read(bfr);
	}

	return util_decode_u32(data);
}

/**
 * @fn int32_t util_buffer_u8_get_i32(util_buffer_u8_t*)
 * @brief Gets the next signed 32-bit integer from the buffer.
 *
 * @param bfr The buffer to read from.
 * @return a signed 32-bit integer decoded with respect to the buffer's endianness.
 */
static inline int32_t util_buffer_u8_get_i32(util_buffer_u8_t *bfr) {
	uint8_t data[4];

	for (int i = 0; i < 4; i++) {
		data[i] = bfr->buffer[bfr->current_read_index++];
		util_buffer_check_read(bfr);
	}

	return util_decode_i32(data);
}

//Access from ix-th element back in history from the last insert
static inline uint8_t util_buffer_u8_access(util_buffer_u8_t *bfr, int16_t ix) {
	ix = bfr->current_write_index - ix - 1;
	while (ix < 0)
		ix += bfr->bfr_len;
	return bfr->buffer[ix];
}

static inline uint8_t util_buffer_u8_isempty(util_buffer_u8_t *bfr) {
	return bfr->current_read_index == bfr->current_write_index;
}

//U16 BUFFER
static inline void util_buffer_u16_init(util_buffer_u16_t *bfr,
		uint16_t *buffer, uint16_t bfr_len) {
	bfr->current_write_index = 0;
	bfr->current_read_index = 0;
	bfr->bfr_len = bfr_len;
	bfr->buffer = buffer;
}

static inline void util_buffer_u16_add(util_buffer_u16_t *bfr, uint16_t d) {
	bfr->buffer[bfr->current_write_index++] = d;
	if (bfr->current_write_index == bfr->bfr_len)
		bfr->current_write_index = 0;
}

static inline uint16_t util_buffer_u16_get(util_buffer_u16_t *bfr) {
	uint16_t tmp = bfr->buffer[bfr->current_read_index++];
	if (bfr->current_read_index == bfr->bfr_len)
		bfr->current_read_index = 0;
	return tmp;
}

static inline uint8_t util_buffer_u16_isempty(util_buffer_u16_t *bfr) {
	return bfr->current_read_index == bfr->current_write_index;
}

//I16 BUFFER
static inline void util_buffer_i16_init(util_buffer_i16_t *bfr, int16_t *buffer,
		uint16_t bfr_len) {
	bfr->current_write_index = 0;
	bfr->current_read_index = 0;
	bfr->bfr_len = bfr_len;
	bfr->buffer = buffer;
}

static inline void util_buffer_i16_add(util_buffer_i16_t *bfr, int16_t d) {
	bfr->buffer[bfr->current_write_index++] = d;
	if (bfr->current_write_index == bfr->bfr_len)
		bfr->current_write_index = 0;
}

static inline int16_t util_buffer_i16_get(util_buffer_i16_t *bfr) {
	int16_t tmp = bfr->buffer[bfr->current_read_index++];
	if (bfr->current_read_index == bfr->bfr_len)
		bfr->current_read_index = 0;
	return tmp;
}

static inline uint8_t util_buffer_i16_isempty(util_buffer_i16_t *bfr) {
	return bfr->current_read_index == bfr->current_write_index;
}

/* circular buffer */

#define UTIL_GENERATE_BUFFER(type, name)   																		\
	typedef struct UTIL_BUFFER_##name{   																		\
		uint16_t c_ix;                																			\
		uint16_t l_ix;            																				\
		uint16_t bfr_len;           																			\
		type * buffer;            																				\
	}UTIL_BUFFER_##name##_t;             																		\
static inline void util_buffer_##name##_init(UTIL_BUFFER_##name##_t * bfr, type * buffer, uint16_t bfr_len) { 	\
	bfr->c_ix = 0;                                                                                   			\
	bfr->l_ix = 0;                                                                                   			\
	bfr->bfr_len = bfr_len;                                                                          			\
	bfr->buffer = buffer;                                                                            			\
}                                                                                                    			\
static inline void util_buffer_##name##_add(UTIL_BUFFER_##name##_t * bfr, type d) {                  			\
	bfr->buffer[bfr->c_ix++] = d;                                                                    			\
	if(bfr->c_ix == bfr->bfr_len) bfr->c_ix = 0;                                                     			\
}                                                                                                    			\
static inline type util_buffer_##name##_get(UTIL_BUFFER_##name##_t * bfr) {                          			\
	type tmp = bfr->buffer[bfr->l_ix++];                                                          	 			\
	if(bfr->l_ix == bfr->bfr_len) bfr->l_ix=0;                                                       			\
	return tmp;                                                                                      			\
}                                                                                                    			\
static inline type util_buffer_##name##_access(UTIL_BUFFER_##name##_t * bfr, uint16_t ix) {          			\
	int16_t i = bfr->c_ix - ix - 1;                                                                  			\
	while(i < 0) i += bfr->bfr_len;                                                      			 			\
	return bfr->buffer[i];                                                                           			\
}																									  			\
static inline uint8_t util_buffer_##name##_isempty(UTIL_BUFFER_##name##_t * bfr) {                           	\
	return bfr->l_ix == bfr->c_ix;                                                                   			\
}

#define util_abs(a)	((a)<0?-(a):(a))

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* UTIL_H */

/* END */
