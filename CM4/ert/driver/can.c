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

/**********************
 *	CONSTANTS
 **********************/



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

typedef struct can_daemon_context {
	SemaphoreHandle_t rx_sem;
	StaticSemaphore_t rx_sem_buffer;
}can_daemon_context_t;


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


device_interface_t fdcan2_interface;

can_interface_context_t fdcan2_context = {
	.fdcan = &hfdcan2,
	.board_id = 1
};


device_daemon_t fdcan_daemon;

can_daemon_context_t fdcan_daemon_context;




/**********************
 *	DECLARATIONS
 **********************/


void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
	// messages have been transmitted
}

/*
 * careful as this is in interrupt context
 * -> make a bottom half to handle incomming data
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
		xSemaphoreGiveFromISR( fdcan_daemon_context.rx_sem, &xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
	// protocol error occurred
}


void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
	// bus error occurred
}


/**
 * @brief   Blocking function, waiting for data to be ready.
 */
util_error_t can_data_ready(void * dem_ctx)
{
	can_daemon_context_t * daemon = (can_daemon_context_t *) dem_ctx;
	if( xSemaphoreTake(fdcan_daemon_context.rx_sem, osWaitForever) == pdTRUE ) {
		return ER_SUCCESS;
	} else {
		return ER_TIMEOUT;
	}

}


util_error_t can_recv(void * context, uint8_t * data, uint32_t * len) {
	return ER_RESSOURCE_ERROR;
}


util_error_t can_handle_data(void * if_context, __attribute__((unused)) void * dem_context) {
	can_interface_context_t * can = (can_interface_context_t *) if_context;
	// drain the FIFO
	while(HAL_FDCAN_GetRxFifoFillLevel(can->fdcan, FDCAN_RX_FIFO0) > 0) {
		od_frame_t frame;
		FDCAN_RxHeaderTypeDef rxHeader;

		HAL_FDCAN_GetRxMessage(can->fdcan, FDCAN_RX_FIFO0, &rxHeader, frame.data);

		frame.data_id = rxHeader.Identifier >> CAN_DATAID_OFFSET;
		uint8_t source = rxHeader.Identifier & CAN_BOARDID_MASK;

		//TODO: call rx handler

	}
}


util_error_t can_send(void * context, uint8_t * data, uint32_t len) {
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
	device_interface_create(can_if, can_ctx, &fdcan_daemon, can_send, can_recv, can_handle_data);

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
}



void can_init() {


	fdcan_daemon_context.rx_sem = xSemaphoreCreateBinaryStatic(&fdcan_daemon_context.rx_sem_buffer);

	device_deamon_create(&fdcan_daemon, "fdcan daemon", 6, &fdcan_daemon_context, can_data_ready);

	can_interface_init(&fdcan1_interface, &fdcan1_context);



}




void can_transmit_thread(__attribute__((unused))  void *arg) {

	// Infinite loop
	while(1) {
		// retrieve next frame
		od_frame_t frame;
		od_pop_from_out_q(&frame);

		// send over CAN
		device_interface_send(&fdcan1_interface, (uint8_t *) &frame, sizeof(od_frame_t));
	}
}




/* END */
