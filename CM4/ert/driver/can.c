/*  Title		: Object dictionary
 *  Filename	: od.c
 *	Author		: Oliver Facklam
 *	Date		: 28.05.2022
 *	Version		: 0.1
 *	Description	: Object dictionary implementation
 */

/**********************
 *	INCLUDES
 **********************/

#include <cmsis_os2.h>
#include <FreeRTOS.h>
#include <fdcan.h>
#include <semphr.h>

#include <driver/can.h>
#include <od/od.h>
#include <device/device.h>
#include <feedback/led.h>

/**********************
 *	CONSTANTS
 **********************/

#define CAN_MAX_INTERFACES 2


/**********************
 *	MACROS
 **********************/



/**********************
 *	TYPEDEFS
 **********************/


typedef struct can_interface_context {
	FDCAN_HandleTypeDef * fdcan;
	uint8_t board_id;
}can_interface_context_t;


/**********************
 *  PROTOTYPES
 **********************/


/**********************
 *	VARIABLES
 **********************/

device_interface_t fdcan1_interface;

can_interface_context_t fdcan1_context = {
	.fdcan = &hfdcan1,
	.board_id = 1
};


static uint16_t checkpoint_tx;
static uint16_t checkpoint_rx;


static SemaphoreHandle_t can_rx_sem;
static StaticSemaphore_t can_rx_sem_buffer;

static device_interface_t * can_interfaces[CAN_MAX_INTERFACES];
static uint16_t can_interfaces_count;


/**********************
 *	DECLARATIONS
 **********************/


void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
	// messages have been transmitted
	UNUSED(hfdcan);
	UNUSED(BufferIndexes);
}

/*
 * careful as this is in interrupt context
 * -> make a bottom half to handle incomming data
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	UNUSED(hfdcan);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
		xSemaphoreGiveFromISR( can_rx_sem, &xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
	// protocol error occurred
	UNUSED(hfdcan);
}


void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
	// bus error occurred
	UNUSED(hfdcan);
	UNUSED(ErrorStatusITs);
}


/**
 * @brief   Blocking function, waiting for data to be ready.
 */
util_error_t can_data_ready()
{
	if( xSemaphoreTake(can_rx_sem, osWaitForever) == pdTRUE ) {
		return ER_SUCCESS;
	} else {
		return ER_TIMEOUT;
	}

}


util_error_t can_recv(void * context, uint8_t * data, uint32_t * len) {
	UNUSED(context);
	UNUSED(data);
	UNUSED(len);
	return ER_RESSOURCE_ERROR;
}


util_error_t can_handle_data(device_interface_t * can_if) {
	can_interface_context_t * can = (can_interface_context_t *) can_if->context;
	// drain the FIFO
	while(HAL_FDCAN_GetRxFifoFillLevel(can->fdcan, FDCAN_RX_FIFO0) > 0) {
		od_frame_t frame;
		FDCAN_RxHeaderTypeDef rxHeader;

		HAL_FDCAN_GetRxMessage(can->fdcan, FDCAN_RX_FIFO0, &rxHeader, frame.data);

		frame.data_id = rxHeader.Identifier >> CAN_DATAID_OFFSET;
		uint8_t source = rxHeader.Identifier & CAN_BOARDID_MASK;

		od_handle_can_frame(source, &frame);

	}
	return ER_SUCCESS;
}





util_error_t can_send(void * context, uint8_t * data, uint32_t len) {
	if(len != sizeof(od_frame_t)) {
		return ER_FAILURE;
	}
	// check free level
	can_interface_context_t * can = (can_interface_context_t *) context;
	od_frame_t * frame = (od_frame_t *) data;

	uint32_t freeLevel = HAL_FDCAN_GetTxFifoFreeLevel(can->fdcan);
	if(freeLevel <= 0)
		return ER_RESSOURCE_ERROR;

	// send frame
	FDCAN_TxHeaderTypeDef txHeader = {
			.Identifier = (frame->data_id << CAN_DATAID_OFFSET) | (can->board_id & CAN_BOARDID_MASK),
			.IdType = FDCAN_STANDARD_ID, // 11 bits
			.TxFrameType = FDCAN_DATA_FRAME,
			.DataLength = FDCAN_DLC_BYTES_64,
			.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
			.BitRateSwitch = FDCAN_BRS_OFF,
			.FDFormat = FDCAN_FD_CAN,
			.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
	};
	HAL_FDCAN_AddMessageToTxFifoQ(can->fdcan, &txHeader, frame->data);


	return ER_SUCCESS;
}


util_error_t can_interface_init(device_interface_t * can_if, can_interface_context_t * can_ctx) {

	if(can_interfaces_count >= CAN_MAX_INTERFACES) {
		return ER_RESSOURCE_ERROR;
	}

	// Configure clock unit --> route kernel clock directly to CAN core
	// Kernel clock & bit timings are determined by settings in CubeMX
	// f_tqck = f_kerck = 32Mhz
	// tq = prescaler * t_tqck = prescaler / f_tqck
	// t_sync = 1 tq; t_bs1 = segment1 * tq; t_bs2 = segment2 * tq
	// t_bit = t_sync + t_bs1 + t_bs2 = tq * (segment1 + segment2 + 1)
	// baudrate = f_tqck / (prescaler * (seg1 + seg2 + 1)) = 1Mbps
	FDCAN_ClkCalUnitTypeDef ccuConfig = {
			.ClockCalibration = FDCAN_CLOCK_CALIBRATION_DISABLE,
			.ClockDivider = FDCAN_CLOCK_DIV1,
	};
	HAL_FDCAN_ConfigClockCalibration(can_ctx->fdcan, &ccuConfig);

	//can callback init for reception
	device_interface_create(can_if, can_ctx, NULL, can_send, can_recv, NULL);

	can_interfaces[can_interfaces_count++] = can_if;

	// Peripheral, clocks & baud rate already initialized

	// Configure RX behavior
	HAL_FDCAN_ConfigGlobalFilter(can_ctx->fdcan,
			FDCAN_ACCEPT_IN_RX_FIFO0,
			FDCAN_ACCEPT_IN_RX_FIFO0,
			FDCAN_REJECT_REMOTE,
			FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigRxFifoOverwrite(can_ctx->fdcan, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);

	// Configure interrupts
	uint32_t interrupts = FDCAN_IT_TX_COMPLETE
			| FDCAN_IT_RX_FIFO0_NEW_MESSAGE
			| FDCAN_IT_ARB_PROTOCOL_ERROR
			| FDCAN_IT_DATA_PROTOCOL_ERROR
			| FDCAN_IT_ERROR_PASSIVE
			| FDCAN_IT_ERROR_WARNING
			| FDCAN_IT_BUS_OFF;
	uint32_t allBuffers = 0xFFFFFFFF;
	HAL_FDCAN_ActivateNotification(can_ctx->fdcan, interrupts, allBuffers);

	// Start FDCAN
	HAL_FDCAN_Start(can_ctx->fdcan);

	return ER_SUCCESS;
}





void can_init(uint8_t board_id) {


	can_rx_sem = xSemaphoreCreateBinaryStatic(&can_rx_sem_buffer);

	fdcan1_context.board_id = board_id;

	can_interface_init(&fdcan1_interface, &fdcan1_context);


	checkpoint_tx = led_add_checkpoint(led_teal);

	checkpoint_rx = led_add_checkpoint(led_pink);


}




void can_transmit_thread(__attribute__((unused))  void *arg) {

	// Infinite loop
	while(1) {
		// retrieve next frame
		od_frame_t frame;
		od_pop_from_out_q(&frame);
		od_push_to_in_q(&frame);

		led_checkpoint(checkpoint_tx);

		// send over CAN
		device_interface_send(&fdcan1_interface, (uint8_t *) &frame, sizeof(od_frame_t));
	}
}

void can_receive_thread(__attribute__((unused))  void *arg) {

	// Infinite loop
	while(1) {
		if(can_data_ready() == ER_SUCCESS) {
			//iterate over all interfaces in deamon
			for(uint16_t i = 0; i < can_interfaces_count; i++) {
				led_checkpoint(checkpoint_rx);
				can_handle_data(can_interfaces[i]);
			}
		}
	}
}




/* END */
