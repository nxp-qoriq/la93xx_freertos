/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "common.h"
#include "immap.h"
#include "la9310_main.h"
#include "la9310_irq.h"
#include "core_cm4.h"
#include "la9310_edmaAPI.h"
#include "bbdev_ipc.h"
#include "rfic_api.h"
#include "sync_timing_device.h"
#include "math.h"
#include "rfic_api.h"
#include "phytimer.h"
#include <time.h>
#include "dfe_avi.h"
#include "dfe_app.h"
#include "dfe_host_if.h"
#include "la9310_avi.h"
#include "../drivers/avi/la9310_vspa_dma.h"
#include "../drivers/avi/la9310_avi_ds.h"

/* VSPA timeouts */
#define NO_WAIT                 ~0
#define NO_TIMEOUT              (-1)
#define VSPA_IMM_RESPONSE       0
#define VSPA_ACK_TIMEOUT_MS     20

#define AXIQ_TAIL_LENGTH        8 /* 8 samples */
#define MAX_SYMBOLS             14 /* maximum number of symbols in slot */

#define VSPA_SW_VER_PRODUCTION  0xDFEF0000
bool_t bVspaProductionBinary = pdFALSE;

QueueHandle_t xTimeQueue;
static TimerHandle_t vspa_timer;

/* BBDEV IPC parameters */
static int ipc_up = 0;
static int ipc_dev_id = BBDEV_IPC_DEV_ID_0;
static int ipc_tx_qid = BBDEV_IPC_M2H_QUEUE;
static int ipc_rx_qid = BBDEV_IPC_H2M_QUEUE;

/* default config */
uint32_t uRxAntennaComparator = PHY_TIMER_COMP_CH2_RX_ALLOWED;
uint32_t uTxAntennaComparator = PHY_TIMER_COMP_CH5_TX_ALLOWED;
uint32_t rx_addr = 0xA0000000;
uint32_t tx_addr = 0xA0010000;
uint32_t sym_size_128 = 35;
uint32_t rx_sym_nr = 14/2;
uint32_t tx_sym_nr = 14/2;

tDFEPatternConfig pattern = {
	.scs = SCS_kHz30,
	.p = { .dl_slots = 2,
	       .g1_slots = 5,
	       .ul_slots = 2,
	       .g2_slots = 6,
		   .ul2_slots = 4,
		   .g3_slots = 1,
	     },
};

static TaskHandle_t pxTddConfigTask = NULL;
static bool_t bTddConfigDone = 0;

static uint32_t ulNextTick;
static uint32_t ulCurrentSlot;
static uint32_t ulTotalSlots;
static int32_t iTimeAdvance;
uint32_t ulTotalTicks;
uint32_t ulVspaMsgCnt;
uint32_t ulVspaMsgTxCnt;
uint32_t ulVspaMsgRxCnt;

enum ePhyTimerComparatorTrigger tx_allowed_last_state = ePhyTimerComparatorNoChange;
enum ePhyTimerComparatorTrigger rx_allowed_last_state = ePhyTimerComparatorNoChange;
uint32_t next_tx_allowed = 0;
uint32_t next_rx_allowed = 0;
uint32_t last_tx_allowed_timestamp = 0;
uint32_t last_rx_allowed_timestamp = 0;

const uint32_t ofdm_short_sym_time[SCS_MAX] = {
	[SCS_kHz15]  =  4384, /* 15KHz:  (2048 + 144)*2  */
	[SCS_kHz30]  =  2192, /* 30KHz:  (2048 + 144)    */
};

const uint32_t ofdm_long_sym_time[SCS_MAX] = {
	[SCS_kHz15]  = 4448, /* 15KHz:  (2048 + 176)*2  */
	[SCS_kHz30]  = 2224, /* 30KHz:  (2048 + 176)    */
};

const uint32_t slot_duration[SCS_MAX] = {
	[SCS_kHz15]  = 61440, /* PHY_TIMER_MS_1   */
	[SCS_kHz30]  = 30720, /* PHY_TIMER_MS_0p5 */
};

const uint32_t tick_interval[SCS_MAX] = {
	[SCS_kHz15]  = 30720 * 2,  /* 61.44MHz * 1000us */
	[SCS_kHz30]  = 30720,      /* 61.44MHz * 500us */
};

sTraceEntry app_logging[MAX_TS] = { 0 };
uint32_t debug_ts[MAX_TS] = { 0 };
bool debug = pdTRUE;
bool stop = pdFALSE;

tSlot *slots;

uint32_t log_idx = 0;

const char *event_to_string(uint32_t event)
{
	if ( event < TRACE_MAX )
		return eTraceEventString[event];

	return NULL;
}

static void prvTick( void *pvParameters, long unsigned int param1);
static uint32_t prvSendVspaCmd(struct dfe_mbox *mbox_h2v, uint32_t rsp_type, int32_t vspa_timeout_ms);

/* routine for returning the current timer value */
inline uint32_t uGetPhyTimerTimestamp(void)
{
	return ulPhyTimerCapture( PHY_TIMER_COMPARATOR_COUNT - 1 );
}

/* routine for implementing delay given as timer ticks */
void vPhyTimerDelay(uint32_t sleep_phy_timer_ticks)
{
    uint32_t init_phy_timer_value = uGetPhyTimerTimestamp();
    uint32_t exit_phy_timer_value = init_phy_timer_value + sleep_phy_timer_ticks;

    uint32_t crt_phy_timer_value;

    if (exit_phy_timer_value >= init_phy_timer_value)
    {
        do crt_phy_timer_value = uGetPhyTimerTimestamp();
        while (crt_phy_timer_value < exit_phy_timer_value);
    }
    else
    {
        do crt_phy_timer_value = uGetPhyTimerTimestamp();
        while ((crt_phy_timer_value >= init_phy_timer_value) || (crt_phy_timer_value < exit_phy_timer_value));
    }
}

/* routine for waiting until timer exceeds target value  */
void vPhyTimerWaitComparator(uint32_t target)
{
    uint32_t init_phy_timer_value = uGetPhyTimerTimestamp();
    uint32_t exit_phy_timer_value = target;

    uint32_t crt_phy_timer_value;

    /* Check if the target is actually in the past */
    if ((target - init_phy_timer_value) >= (UINT32_MAX / 2)) {
		return;
	}

	/* the target is in the future, wait... */
    if (exit_phy_timer_value >= init_phy_timer_value)
    {
        do crt_phy_timer_value = uGetPhyTimerTimestamp();
        while (crt_phy_timer_value < exit_phy_timer_value);
    }
    else
    {
        do crt_phy_timer_value = uGetPhyTimerTimestamp();
        while ((crt_phy_timer_value >= init_phy_timer_value) || (crt_phy_timer_value < exit_phy_timer_value));
    }
}

#if 0
uint32_t qec_params[QEC_MAX_CORR][MBOX_IQ_CORR_MAX] = {
	[QEC_TX_CORR] = { 0, 1/*f2*/, 0, 0, 0, 0, 0, 0, 0, 0, 1/*f1*/, 1 /*f4*/, 0 /* DC_I */, 0 /* DC_Q */, 1 /* FDELAY */ },
	[QEC_RX_CORR] = { 0, 1/*f2*/, 0, 0, 0, 0, 0, 0, 0, 0, 1/*f1*/, 1 /*f4*/, 0 /* DC_I */, 0 /* DC_Q */, 1 /* FDELAY */ },
};

int iSetTxRxQecParamIndexValue(uint32_t txrx, uint32_t tap_idx, uint32_t value)
{
	/* check if desired value is already set */
	if (qec_params[txrx][tap_idx] == value)
		return 0;
	else
		qec_params[txrx][tap_idx] = value;

	return 1;
}
#endif

void vSetQecParam(uint32_t txrx, uint32_t mode, uint32_t idx, uint32_t value)
{
	struct dfe_mbox mbox_h2v;

	PRINTF("txrx = %#x, mode = %#x, idx = %d, value = %#x\r\n",
		   txrx, mode, idx, value);

#if 0
	/* if mode is pass-through, do not update VSPA if qec parameter doesn't change */
	if (mode != QEC_IQ_DC_OFFSET_PASSTHROUGH)
		if (!iSetTxRxQecParamIndexValue(txrx, idx, value))
			return;
#endif

	MBOX_CLEAR(mbox_h2v);
	MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_QEC);

	MBOX_SET_QEC_CORR_MODE(mbox_h2v, mode);
	MBOX_SET_QEC_TX_RX_MODE(mbox_h2v, txrx);
	MBOX_SET_QEC_IDX(mbox_h2v, idx);
	MBOX_SET_QEC_VALUE(mbox_h2v, value);

	prvSendVspaCmd(&mbox_h2v, 0xDEAD, NO_TIMEOUT);
}

int iSetRxAntenna(uint32_t antenna)
{
	switch (antenna) {
		case PHY_TIMER_COMP_CH1_RX_ALLOWED:
		case PHY_TIMER_COMP_CH2_RX_ALLOWED:
		case PHY_TIMER_COMP_CH3_RX_ALLOWED:
		case PHY_TIMER_COMP_CH4_RX_ALLOWED:
			uRxAntennaComparator = antenna;
			break;
		default:
			log_err("Rx Antenna invalid config\r\n");
			return DFE_INVALID_PARAM;
	}

	return DFE_NO_ERROR;
}

void vSetTxRxBuffers(uint32_t rxa, uint32_t txa, uint32_t sym_size, uint32_t rxs, uint32_t txs)
{
	rx_addr = rxa;
	tx_addr = txa;
	sym_size_128 = sym_size;
	rx_sym_nr = rxs;
	tx_sym_nr = txs;
}

inline void vTraceEventRecord(eTraceEvent event, uint32_t p1, uint32_t p2)
{
	app_logging[log_idx].timestamp = uGetPhyTimerTimestamp();
	app_logging[log_idx].event = event;
	app_logging[log_idx].param1 = p1;
	app_logging[log_idx].param2 = p2;

	log_idx++;
	log_idx %= MAX_TS;
}

void vTraceEventShow()
{
	uint8_t i = 0;

	PRINTF("App logging:\r\n");
	PRINTF(" _______________________________________________ \r\n");
	PRINTF("|  Timestamp  | Event |   Param1   |   Param2   |\r\n");
	PRINTF("|_____________|_______|____________|____________|\r\n");
	PRINTF("|                                               |\r\n");
	for (i = 0; i < MAX_TS; i++) {
		PRINTF("|  0x%08x |%7s| 0x%08x | 0x%08x |\r\n",
				app_logging[i].timestamp,
				event_to_string(app_logging[i].event),
				app_logging[i].param1,
				app_logging[i].param2);
	}
	PRINTF("|_______________________________________________|\r\n");
}

static void prvTimerCb(TimerHandle_t timer)
{
	uint32_t rsp_type;

	rsp_type = (uint32_t)pvTimerGetTimerID(timer);
	PRINTF("VSPA timer expired, rsp_type = %#x\r\n", rsp_type);

	//TODO: send a notification to host?
}

void vPhyTimerTickConfig()
{
    NVIC_SetPriority( IRQ_PPS_OUT, 1 );
    NVIC_EnableIRQ( IRQ_PPS_OUT );

    ulNextTick = ulPhyTimerCapture( PHY_TIMER_COMP_PPS_OUT );
    ulNextTick += tick_interval[pattern.scs];

    vPhyTimerComparatorConfig( PHY_TIMER_COMP_PPS_OUT,
            PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
            ePhyTimerComparatorOutToggle,
            ulNextTick );
}

void vPhyTimerPPSOUTHandler()
{
    NVIC_ClearPendingIRQ( IRQ_PPS_OUT );

	/* Update next tick's timestamp */
    ulNextTick += tick_interval[pattern.scs];

	/* log tick */
	vTraceEventRecord(TRACE_TICK, 0xFFFFFFFF, ulNextTick);

	vPhyTimerComparatorConfig( PHY_TIMER_COMP_PPS_OUT,
				PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
				ePhyTimerComparatorOutToggle,
				ulNextTick );

	/* Call the tick callback */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTimerPendFunctionCallFromISR( prvTick, NULL, 0, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static char *prvOpcodeToStr(uint8_t opcode)
{
	switch(opcode) {
	case DFE_OP_TEST_MBOX:
		return "Test mailbox";

	default:
		return "N/A";
	}
}

static void prvStartVspaTimer(uint32_t rsp_type, uint32_t timeout_ms)
{
	int ret;

	vTimerSetTimerID(vspa_timer, (void *)rsp_type);
	ret = xTimerChangePeriod(vspa_timer, pdMS_TO_TICKS(timeout_ms), 0);
	if (ret == pdFAIL) {
		PRINTF("Error updating timer period to %dms\n", timeout_ms);
		return;
	}

	PRINTF("Set timer to %dms\r\n", timeout_ms);
	xTimerStart(vspa_timer, 0);
}

static void prvStopVspaTimer(void)
{
	if (xTimerIsTimerActive(vspa_timer))
		xTimerStop(vspa_timer, 0);
	PRINTF("Stopped timer\r\n");
}

static uint32_t prvSendVspaCmd(struct dfe_mbox *mbox_h2v, uint32_t rsp_type,
			       int32_t vspa_timeout_ms)
{
	struct dfe_mbox mbox_v2h = {0};
	uint32_t status = DFE_ERROR_GENERIC;
	int ret;

#if 0
	PRINTF("Send command to VSPA, opcode %d, msb = %#x, lsb = %#x\r\n",
			MBOX_GET_OPCODE(*mbox_h2v), mbox_h2v->msb, mbox_h2v->lsb);
#endif
	if (vspa_timeout_ms > 0)
		prvStartVspaTimer(rsp_type, vspa_timeout_ms);

	ret = vDFEMbxSend(mbox_h2v, DFE_OPS_MBOX_ID);
	ulVspaMsgCnt++;
	/*
	 * If send succeeded, wait for an ACK from VSPA; otherwise just
	 * signal the error back to host
	 */
	if (ret != 0) {
		//PRINTF("Error sending message to VSPA\r\n");
		status = DFE_ERROR_GENERIC;
		goto send_ack;
	}

	if (vspa_timeout_ms == NO_WAIT)
		return status;

	ret = vDFEMbxReceive(&mbox_v2h, DFE_OPS_MBOX_ID, VSPA_ACK_TIMEOUT_MS);

	if (ret != 0) {
		//PRINTF("No ACK from VSPA for opcode %d\r\n", MBOX_GET_OPCODE(*mbox_h2v));
		status = DFE_ERROR_VSPA_TIMEOUT;
		goto send_ack;
	}

#if 0
	PRINTF("[DEBUG]: MSB = %#x, LSB = %#x\r\n", mbox_v2h.msb, mbox_v2h.lsb);
#endif

	return status;

send_ack:
	prvSendMsgToHost(DFE_VSPA_ERROR, status, 0);
	return status;
}

void vVSPADebugBreakPoint()
{
	struct dfe_mbox mbox_h2v;

	MBOX_CLEAR(mbox_h2v);
	MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_DEBUG_BP);
	prvSendVspaCmd(&mbox_h2v, 0xDEAD, NO_TIMEOUT);
}

static void prvVSPAConfig(uint32_t tdd0_fdd1, int resp_mode)
{
	static uint8_t first_operation = 1;
	struct dfe_mbox mbox_h2v;

	/* send semistatic config */
	MBOX_CLEAR(mbox_h2v);
	MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_SEMISTATIC);

	MBOX_SET_RX_TDD_FDD(mbox_h2v, tdd0_fdd1);
	MBOX_SET_TX_TDD_FDD(mbox_h2v, tdd0_fdd1);
	MBOX_SET_CPE_BS_MODE(mbox_h2v, 1); /*ue*/
	MBOX_SET_DCS_MAPPING(mbox_h2v, 1); /* adc 0,1,2,3 */
	/* Rx ADC is 0-based */
	MBOX_SET_RX_IDX(mbox_h2v, uRxAntennaComparator - 1);
	/* la9310 has a single Tx - ch5. VSPA expects 0 */
	MBOX_SET_TX_IDX(mbox_h2v, 0);
	/* IQ swap settings */
	MBOX_SET_RX_IQSWAP(mbox_h2v, 0);
	MBOX_SET_TX_IQSWAP(mbox_h2v, 0);
	/* TDD SCS config */
	MBOX_SET_SCS(mbox_h2v, pattern.scs);

	/* production VSPA requires slightly different handling */
	/* switching between modes requires chan_start updates*/
	if (bVspaProductionBinary) {
		if (!first_operation) {
			MBOX_SET_RX_CHAN_RESTART(mbox_h2v, 1);
			MBOX_SET_TX_CHAN_RESTART(mbox_h2v, 1);
		}
	}

	prvSendVspaCmd(&mbox_h2v, 0xDEAD, resp_mode);

	/* configuration of buffer */
	MBOX_CLEAR(mbox_h2v);
	MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_DDR_CFG_4KB);
	MBOX_SET_RX_ADDR(mbox_h2v, rx_addr >> 12);
	MBOX_SET_TX_ADDR(mbox_h2v, tx_addr >> 12);
	MBOX_SET_SYM_SIZE(mbox_h2v, sym_size_128); /* 35 * 128 bytes*/
	MBOX_SET_RX_SYM_NR(mbox_h2v, rx_sym_nr); /* VSPA reserves double nr of syms */
	MBOX_SET_TX_SYM_NR(mbox_h2v, tx_sym_nr); /* VSPA reserves double nr of syms */
	prvSendVspaCmd(&mbox_h2v, 0xDEAD, resp_mode);

	first_operation = 0;
}

#if 0
static void prvVSPALoop(void *pvParameters)
{
	struct dfe_mbox vspa_ctrl_msg;
	uint8_t opcode;
	uint32_t ret = 0;

	vspa_timer = xTimerCreate("vspa_timer", 1, pdFALSE, 0, prvTimerCb);
	if (vspa_timer == NULL)
		PRINTF("Error creating VSPA timer\r\n");

	PRINTF("Waiting for messages from VSPA...\n\r");

	while (1) {
		/* Check for message */
		ret = vDFEMbxReceive(&vspa_ctrl_msg, DFE_CTRL_MBOX_ID, 0);
		if (ret != 0)
			continue;

		opcode = MBOX_GET_OPCODE(vspa_ctrl_msg);
		PRINTF("Received msg from VSPA: 0x%x (%s)\r\n",
			       opcode, prvOpcodeToStr(opcode));
		PRINTF("MSB = 0x%x, LSB = 0x%x\r\n",
			      vspa_ctrl_msg.msb, vspa_ctrl_msg.lsb);
#if 0
		switch (opcode) {
		case DFE_OP_TEST_MBOX:
			PRINTF("DFE_OP_TEST_MBOX received!\r\n");
			break;

		default:
			PRINTF("Udefined opcode from VSPA: 0x%x\r\n", opcode);
			break;
		}
#endif
	}
}
#endif

static void vGetSymbolsStartStop( tSlot slot, uint32_t *start_offset, uint32_t *stop_offset )
{
	uint32_t start_time = 0;
	uint32_t stop_time = 0;

	/* compute start time for slot */
	if ( slot.start_symbol > 0 ) {
		start_time += ofdm_long_sym_time[pattern.scs];
		start_time += ofdm_short_sym_time[pattern.scs] * (slot.start_symbol - 1);
	}

	/* compute stop time for slot */
	if ( slot.end_symbol < (MAX_SYMBOLS - 1) ) {
		stop_time += ofdm_short_sym_time[pattern.scs] * (MAX_SYMBOLS - slot.end_symbol - 1);
	}

	/* make adjustments for SCS_kHz15 */
	/* SCS-15kHz has 2 long cyclic prefix symbols */
	if ( pattern.scs == SCS_kHz15 ) {
		if ( slot.start_symbol > 7) {
			start_time += ofdm_long_sym_time[pattern.scs];
			start_time -= ofdm_short_sym_time[pattern.scs];
		}

		if ( slot.end_symbol < 7 ) {
			stop_time -= ofdm_short_sym_time[pattern.scs];
			stop_time += ofdm_long_sym_time[pattern.scs];
		}
	}

	*start_offset = start_time;
	*stop_offset = stop_time;
}

inline void vSendTimeAgentMessage(uint32_t target_timestamp, enum ePhyTimerComparatorTrigger tx_state, enum ePhyTimerComparatorTrigger rx_state)
{
	sTimeAgentMessage msg;

	msg.target_timestamp = target_timestamp;
	msg.tx_allowed_state = tx_state;
	msg.rx_allowed_state = rx_state;

#if 1
	/* We have not woken a task at the start of the ISR. */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR( xTimeQueue, ( void * ) &msg, &xHigherPriorityTaskWoken );
#else
	xQueueSend( xTimeQueue,
               ( void * ) &msg,
               ( TickType_t ) 0 );

#endif
}

static void prvTick( void *pvParameters, long unsigned int param1 )
{
	bool_t bIsDownlinkSlot = pdFALSE;
	bool_t bIsUplinkSlot = pdFALSE;
	uint32_t start_offset = 0;
	uint32_t stop_offset = 0;

	/* log current tick */
	vTraceEventRecord(TRACE_SLOT, ulCurrentSlot, ulTotalSlots);

	/* look into pre-computed slot array */
	bIsDownlinkSlot = slots[ulCurrentSlot].is_dl;
	bIsUplinkSlot = slots[ulCurrentSlot].is_ul;


	//vTraceEventRecord(TRACE_SLOT, bIsDownlinkSlot, bIsUplinkSlot);

	vGetSymbolsStartStop(slots[ulCurrentSlot], &start_offset, &stop_offset);
	vTraceEventRecord(TRACE_SLOT, start_offset, stop_offset);

	/* PhyTimer programming */
	if (bIsDownlinkSlot) {
		next_rx_allowed = ulNextTick + iTimeAdvance + start_offset;
		vSendTimeAgentMessage(next_rx_allowed, ePhyTimerComparatorNoChange, ePhyTimerComparatorOut1);
		rx_allowed_last_state = ePhyTimerComparatorOut1;

		//vTraceEventRecord(TRACE_AXIQ, 0x500, ulNextTick);
// 		vPhyTimerComparatorConfig( PHY_TIMER_COMP_CH2_RX_ALLOWED,
// 			PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
// 			ePhyTimerComparatorOut1,
// 			next_rx_allowed );
	} else {
		//vTraceEventRecord(TRACE_AXIQ, 0x501, ulNextTick);
		next_rx_allowed = ulNextTick + iTimeAdvance - stop_offset + AXIQ_TAIL_LENGTH;
		vSendTimeAgentMessage(next_rx_allowed, ePhyTimerComparatorNoChange, ePhyTimerComparatorOut0);
		rx_allowed_last_state = ePhyTimerComparatorOut0;
// 		vPhyTimerComparatorConfig( PHY_TIMER_COMP_CH2_RX_ALLOWED,
// 			PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
// 			ePhyTimerComparatorOut0,
// 			next_rx_allowed );
	}

	if (bIsUplinkSlot) {
		//vTraceEventRecord(TRACE_AXIQ, 0x600, ulNextTick - iTimeAdvance + start_offset);
		next_tx_allowed = ulNextTick - iTimeAdvance + start_offset;
		vSendTimeAgentMessage(next_tx_allowed, ePhyTimerComparatorOut1, ePhyTimerComparatorNoChange);
		tx_allowed_last_state = ePhyTimerComparatorOut1;
// 		vPhyTimerComparatorConfig( PHY_TIMER_COMP_CH5_TX_ALLOWED,
// 			PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
// 			ePhyTimerComparatorOut1,
// 			next_tx_allowed );
	} else {
		//vTraceEventRecord(TRACE_AXIQ, 0x601, ulNextTick - iTimeAdvance - stop_offset + AXIQ_TAIL_LENGTH);
		next_tx_allowed = ulNextTick - iTimeAdvance - stop_offset + AXIQ_TAIL_LENGTH;
		vSendTimeAgentMessage(next_tx_allowed, ePhyTimerComparatorOut0, ePhyTimerComparatorNoChange);
		tx_allowed_last_state = ePhyTimerComparatorOut0;
// 		vPhyTimerComparatorConfig( PHY_TIMER_COMP_CH5_TX_ALLOWED,
// 			PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
// 			ePhyTimerComparatorOut0,
// 			next_tx_allowed );
	}

	if (bIsDownlinkSlot) {
		vTraceEventRecord(TRACE_VSPA, 0x502, 0);
		/* send slot config to VSPA */
		struct dfe_mbox mbox_h2v = {0};

		MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_TDD_RX);
		MBOX_SET_START_SYM(mbox_h2v, slots[ulCurrentSlot].start_symbol);
		MBOX_SET_STOP_SYM(mbox_h2v, slots[ulCurrentSlot].end_symbol);
		MBOX_SET_SLOT_IDX(mbox_h2v, ulCurrentSlot);
		prvSendVspaCmd(&mbox_h2v, 0xDEAD, NO_WAIT);
		ulVspaMsgRxCnt++;
	}

	if (bIsUplinkSlot) {
		vTraceEventRecord(TRACE_VSPA, 0x602, 0);
		/* send slot config to VSPA */
		struct dfe_mbox mbox_h2v = {0};

		MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_TDD_TX);
		MBOX_SET_START_SYM(mbox_h2v, slots[ulCurrentSlot].start_symbol);
		MBOX_SET_STOP_SYM(mbox_h2v, slots[ulCurrentSlot].end_symbol);
		MBOX_SET_SLOT_IDX(mbox_h2v, ulCurrentSlot);
		prvSendVspaCmd(&mbox_h2v, 0xDEAD, NO_WAIT);
		ulVspaMsgTxCnt++;
	}

	/* update current slot */
	ulCurrentSlot++;
	ulCurrentSlot %= ulTotalSlots;
	ulTotalTicks++;

#if 0
	/* DEBUG: add stop condition after end of pattern */
	//if ( ulTotalTicks >= ulTotalSlots )
	if (ulTotalTicks && (ulCurrentSlot == 0))
		stop = pdTRUE;
#endif

	vTraceEventRecord(TRACE_TICK, 0x777, ulNextTick);
		
	if ( stop && (ulCurrentSlot == 0) ) {
		/* disable tick and any other used comparator */
		vPhyTimerComparatorDisable( PHY_TIMER_COMP_PPS_OUT );
		vPhyTimerComparatorDisable( uTxAntennaComparator );
		vPhyTimerComparatorDisable( uRxAntennaComparator );

		/* free memory allocated for keeping slot info */
		vPortFree(slots);
		slots = NULL;

		/* trace the stop event */
		vTraceEventRecord(TRACE_TICK, 0xDEADDEAD, ulNextTick);
		return;
	}
}

void vConfigTddPattern( tDFEPatternConfig pcfg )
{
	/*  __________________________________________________________________
	* |          |          |          |          |           |          |
	* | dl_slots | g1_slots | ul_slots | g2_slots | ul2_slots | g3_slots |
	* |__________|__________|__________|__________|___________|__________|
	*
	*/

	pattern.scs = pcfg.scs;

	pattern.p.dl_slots = pcfg.p.dl_slots;
	pattern.p.g1_slots = pcfg.p.g1_slots;
	pattern.p.ul_slots = pcfg.p.ul_slots;
	pattern.p.g2_slots = pcfg.p.g2_slots;
	pattern.p.ul2_slots = pcfg.p.ul2_slots;
	pattern.p.g3_slots = pcfg.p.g3_slots;
}

void vPrintTddPattern()
{
	PRINTF("\r\nBuffer configuration:\r\n");
	PRINTF("\trx_addr  = 0x%08x, rx_syms_num = %d\r\n", rx_addr, rx_sym_nr);
	PRINTF("\ttx_addr  = 0x%08x, tx_syms_num = %d\r\n", tx_addr, tx_sym_nr);
	PRINTF("\tsym_size = %d (%d * 128 bytes)\r\n", sym_size_128 * 128, sym_size_128);

	PRINTF("\r\nPattern configuration:\r\n");
	PRINTF("\tSCS: %s\r\n", pattern.scs == SCS_kHz15 ? "15 kHz" : "30 kHz");
	PRINTF(" __________________________________________________________________ \r\n");
	PRINTF("|          |          |          |          |           |          |\r\n");
	PRINTF("| dl_slots | g1_slots | ul_slots | g2_slots | ul2_slots | g3_slots |\r\n");
	PRINTF("|          |          |          |          |           |          |\r\n");
	PRINTF("|    %2d    |    %2d    |    %2d    |    %2d    |    %2d     |    %2d    |\r\n",
			pattern.p.dl_slots,
			pattern.p.g1_slots,
			pattern.p.ul_slots,
			pattern.p.g2_slots,
			pattern.p.ul2_slots,
			pattern.p.g3_slots
  		);
	PRINTF("|__________|__________|__________|__________|___________|__________|\r\n");
}

static void prvSetupTdd()
{
	uint32_t k = 0;

	ulTotalSlots = pattern.p.dl_slots +
				pattern.p.g1_slots +
				pattern.p.ul_slots +
				pattern.p.g2_slots +
				pattern.p.ul2_slots +
				pattern.p.g3_slots;

	slots = pvPortMalloc(ulTotalSlots);
	if (!slots) {
		PRINTF("Error allocating memory for slots structure!\n");
		return;
	}
	memset(slots, 0, sizeof(tSlot) * ulTotalSlots);

	for (uint8_t i = 0; i < pattern.p.dl_slots; i++) {
		slots[k].is_dl = 1;
		slots[k].is_ul = 0;
		slots[k].start_symbol = 0;
		slots[k].end_symbol = 13;
		k++;
	}

	for (uint8_t i = 0; i < pattern.p.g1_slots; i++) {
		slots[k].is_dl = 0;
		slots[k].is_ul = 0;
		slots[k].start_symbol = 0;
		slots[k].end_symbol = 13;
		k++;
	}

	for (uint8_t i = 0; i < pattern.p.ul_slots; i++) {
		slots[k].is_dl = 0;
		slots[k].is_ul = 1;
		slots[k].start_symbol = 0;
		slots[k].end_symbol = 13;
		k++;
	}

	for (uint8_t i = 0; i < pattern.p.g2_slots; i++) {
		slots[k].is_dl = 0;
		slots[k].is_ul = 0;
		slots[k].start_symbol = 0;
		slots[k].end_symbol = 13;
		k++;
	}

	for (uint8_t i = 0; i < pattern.p.ul2_slots; i++) {
		slots[k].is_dl = 0;
		slots[k].is_ul = 1;
		slots[k].start_symbol = 0;
		slots[k].end_symbol = 13;
		k++;
	}

	for (uint8_t i = 0; i < pattern.p.g3_slots; i++) {
		slots[k].is_dl = 0;
		slots[k].is_ul = 0;
		slots[k].start_symbol = 0;
		slots[k].end_symbol = 13;
		k++;
	}

	vPrintTddPattern();
}

static void prvTimeAgentTask(void *pvParameters)
{
	sTimeAgentMessage msg;

	/* Create a queue capable of containing 10 elements */
	xTimeQueue = xQueueCreate( 10, sizeof( sTimeAgentMessage ) );

	if( xTimeQueue == NULL )
	{
		/* Queue was not created and must not be used. */
		PRINTF("xQueueCreate error!\r\n");
		return;
    }

	/* task loop */
	for ( ;; ) {
		if( xQueueReceive( xTimeQueue,
							&( msg ),
							( TickType_t ) 1 ) == pdPASS )
		{
			vTraceEventRecord(TRACE_PHYTIMER, msg.target_timestamp, msg.tx_allowed_state | (msg.rx_allowed_state << 16));

			//vPhyTimerWaitComparator(msg.target_timestamp);

			if (msg.tx_allowed_state != ePhyTimerComparatorNoChange) {
				if (last_tx_allowed_timestamp)
					vPhyTimerWaitComparator(last_tx_allowed_timestamp);
				vTraceEventRecord(TRACE_PHYTIMER, 0x600 + msg.tx_allowed_state , ulNextTick);
				vPhyTimerComparatorConfig( uTxAntennaComparator,
				PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
				msg.tx_allowed_state,
				msg.target_timestamp );

				last_tx_allowed_timestamp = msg.target_timestamp;
			}

			if (msg.rx_allowed_state != ePhyTimerComparatorNoChange) {
				if (last_rx_allowed_timestamp)
					vPhyTimerWaitComparator(last_rx_allowed_timestamp);
				vTraceEventRecord(TRACE_PHYTIMER, 0x500 + msg.rx_allowed_state , ulNextTick);
				vPhyTimerComparatorConfig( uRxAntennaComparator,
				PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
				msg.rx_allowed_state,
				msg.target_timestamp );

				last_rx_allowed_timestamp = msg.target_timestamp;
			}

			vTraceEventRecord(TRACE_PHYTIMER, msg.target_timestamp, 0x77777777);
		}

		vTaskDelay(0);
	}
}

void prvConfigTddTask(void *pvParameters)
{
#if 0
	/* PhyTimer API profiling - takes 15 ticks to read the counter */
	PRINTF("MSECONDS_TO_PHY_TIMER_COUNT(1) = %x\r\n", MSECONDS_TO_PHY_TIMER_COUNT(1));

	uint32_t t1 = uGetPhyTimerTimestamp();
	uint32_t t2 = uGetPhyTimerTimestamp();
	PRINTF("t1 = %x\r\nt2 = %x\r\n", t1, t2);
	PRINTF("t2 - t1 = %x (%d)\r\n", t2-t1, t2-t1);
#endif

	vTraceEventRecord(TRACE_PHYTIMER, 0xCCCCCCCC, 0);

	/* set pattern */
	prvSetupTdd();

	/* send VSPA config */
	prvVSPAConfig(0/*tdd*/, NO_TIMEOUT);

	/* reset all counters for easy debugging */
	ulTotalTicks = 0;
	ulVspaMsgCnt = 0;
	ulVspaMsgRxCnt = 0;
	ulVspaMsgTxCnt = 0;

	/* configure tick interval and setup PhyTimer */
	vPhyTimerTickConfig();

	bTddConfigDone = 1;

	while (1) {
		vTaskDelay(1000);
	}
}

void vConfigFddStart()
{
	struct dfe_mbox mbox_h2v = {0};

	prvVSPAConfig(1/*fdd*/, NO_TIMEOUT);

	MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_FDD);
	prvSendVspaCmd(&mbox_h2v, 0xDEAD, NO_TIMEOUT);
}

void vConfigFddStop()
{
	struct dfe_mbox mbox_h2v = {0};

	MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_STOP);
	prvSendVspaCmd(&mbox_h2v, 0xDEAD, NO_TIMEOUT);
}

void vSetTimeAdvance(int32_t iVal)
{
	iTimeAdvance = iVal;
}

int iTddStart(void)
{
	/* check if TDD is running */
	if (!stop && ulTotalTicks) {
		PRINTF("TDD operation already running!\r\n");
		return DFE_OPERATION_RUNNING;
	}

	stop = pdFALSE;
	if(pdPASS != xTaskCreate(prvConfigTddTask, "prvConfigTddTask", 
	configMINIMAL_STACK_SIZE * 4,
			NULL, tskIDLE_PRIORITY + 2, &pxTddConfigTask ))
	{
		PRINTF("Failed to create Config task\r\n");
		return DFE_TASK_CREATE_ERROR;
	}

	while(!bTddConfigDone);
	if (pxTddConfigTask)
		vTaskDelete(pxTddConfigTask);
	pxTddConfigTask = NULL;

	return 0;
}

void vTddStop(void)
{
	stop = pdTRUE;
	vTaskDelay(1000);
	PRINTF("ulTotalTicks = %d\r\n", ulTotalTicks);
	PRINTF("ulVspaMsgCnt = %d\r\n", ulVspaMsgCnt);
	PRINTF("ulVspaMsgTxCnt = %d\r\n", ulVspaMsgTxCnt);
	PRINTF("ulVspaMsgRxCnt = %d\r\n", ulVspaMsgRxCnt);
	PRINTF("tx+rx = %d\r\n", ulVspaMsgTxCnt + ulVspaMsgRxCnt);
}

void vFddStartStop(uint32_t is_on)
{
	uint32_t comparator_value;

	is_on = !!is_on;
	PRINTF("Set TxAllowed(CH5), RxAllowed(CH2) to ");
	if (is_on) {
		comparator_value = ePhyTimerComparatorOut1;
		PRINTF("1\r\n");

		vConfigFddStart();
	}
	else {
		comparator_value = ePhyTimerComparatorOut0;
		PRINTF("0\r\n");
		vConfigFddStop();
	}

	/* delay of 2 symbols */
	vPhyTimerDelay( 2 * ofdm_short_sym_time[SCS_kHz30]);

	vPhyTimerComparatorForce(uTxAntennaComparator, comparator_value);
	vPhyTimerComparatorForce(uRxAntennaComparator, comparator_value);
}

int iBenchmarkVspaTest(uint32_t size, uint32_t mode, uint32_t parallel_dma, uint32_t iterations, uint32_t *gbits, uint32_t *mbits)
{
	struct dfe_mbox mbox_h2v, mbox_v2h;
	uint32_t ret, vspa_cycles = 0;
	uint32_t retries = 0;
	uint32_t opcode = 0;
	float_t result;

	/* return whole part and fractional part via pointers */
	if (!gbits || !mbits)
		goto benchError;

	/* parameters sanity check */
	if ( ( !mode ) || ( mode >= BENCHMARK_CNT ) )
		goto benchError;

	if ( ( !size ) || ( size > 8192 ) )
		goto benchError;

	switch (parallel_dma) {
		case 1:
		case 2:
		case 4:
			break;
		default:
			goto benchError;
	}

	/* config Tx/Rx buffer addresses to VSPA */
	prvVSPAConfig(0, VSPA_IMM_RESPONSE);

	/* configure VSPA message for benchmark */
	MBOX_CLEAR(mbox_h2v);
	MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_BENCHMARK_PCI);
	MBOX_SET_SIZE(mbox_h2v, size);
	MBOX_SET_BENCHMARK_MODE(mbox_h2v, mode);
	MBOX_SET_NUM_PARALLEL_DMAS(mbox_h2v, parallel_dma);
	MBOX_SET_NUM_LOOPS(mbox_h2v, iterations);
	prvSendVspaCmd(&mbox_h2v, 0xDEAD, NO_WAIT);

	/* wait for VSPA to answer to our benchmark request.
	 * while at this, discard any answer from previous commands
	 */
	vTaskDelay(100);
	retries = 20;
	while (--retries && (opcode != MBOX_OPC_BENCHMARK_PCI)) {
		//PRINTF(".\r\n");
		ret = vDFEMbxReceive(&mbox_v2h, DFE_OPS_MBOX_ID, VSPA_IMM_RESPONSE);
		if (ret != 0) {
			PRINTF("No response from VSPA for opcode %d\r\n", MBOX_GET_OPCODE(mbox_h2v));
			goto vspaError;
		}

		opcode = MBOX_GET_OPCODE(mbox_v2h);
		vTaskDelay(100);
		//vTaskDelay(0);
		//PRINTF("retries=%d, opcode = %#x, ret = %d\r\n", retries, MBOX_GET_OPCODE(mbox_h2v), ret);
	}
	/* retreve the number of VSPA cycles */
	vspa_cycles = MBOX_GET_PARAM1(mbox_v2h);
	PRINTF("VSPA cycles: %d\r\n", vspa_cycles);

	/* size * 8 * (491.52 MHz / 2^30) / vspa_cycles */
	if (vspa_cycles) {
		result = (float_t) size * 8 * 0.457763671875 / vspa_cycles;
		/* no float support in printf, thus some basic math to display result in a human-readable form */
		result *= 1000;
		*gbits = (uint32_t) result / 1000;
		*mbits = (uint32_t) result % 1000;
#if 0
		PRINTF("Benchmark result = %d.%d Gb/s\r\n",(uint32_t) result / 1000, (uint32_t) result % 1000);
#endif
	} else {
		PRINTF("Invalid response from VSPA\r\n");
		return 0;
	}
	return vspa_cycles;

benchError:
	PRINTF("Invalid parameter(s)\r\n");
	return 0;

vspaError:
	return 0;
}

/* warm-up VSPA */
static void prvVspaWarmUp()
{
	/* production VSPA requires slightly different handling */
	if (!bVspaProductionBinary)
		vConfigFddStart();

	vPhyTimerDelay( 2 * ofdm_short_sym_time[SCS_kHz30] );
	vPhyTimerComparatorForce(uTxAntennaComparator, ePhyTimerComparatorOut1);
	vPhyTimerComparatorForce(uRxAntennaComparator, ePhyTimerComparatorOut1);
	/* keep FDD running for few symbols */
	vPhyTimerDelay( 2 * ofdm_short_sym_time[SCS_kHz30] );

	if (!bVspaProductionBinary)
		vConfigFddStop();

	/* keep FDD running for ~2 symbols */
	vPhyTimerDelay( 2 * ofdm_short_sym_time[SCS_kHz30] );
	vPhyTimerComparatorForce(uTxAntennaComparator, ePhyTimerComparatorOut0);
	vPhyTimerComparatorForce(uRxAntennaComparator, ePhyTimerComparatorOut0);

	log_err("VSPA warm-up done!\r\n");
}

#define ENQUEUE_RETRIES		100
#define GET_BUF_RETRIES		10
static void *prvGetTxBuf(void)
{
	void *buf;
	uint32_t len;
	int retries = GET_BUF_RETRIES;

	while (retries--) {
		buf = bbdev_ipc_get_next_internal_buf(ipc_dev_id, ipc_tx_qid, &len);
		if (buf)
			return buf;
	}

	PRINTF("Failed to get internal buffer\r\n");
	return NULL;
}
static void prvSendRaw(void *buf, uint32_t len)
{
	struct bbdev_ipc_raw_op_t enq_raw_op;
	int retries = ENQUEUE_RETRIES;
	int ret = 0;

	enq_raw_op.in_len = len;
	enq_raw_op.in_addr = (uint32_t)buf;
	enq_raw_op.out_len = 0;
	enq_raw_op.out_addr = 0;

	while (retries--) {
		ret = bbdev_ipc_enqueue_raw_op(ipc_dev_id, ipc_tx_qid, &enq_raw_op);
		if (ret == 0)
			break;
		if (ret == IPC_CH_FULL)
			continue;
	}
	if (ret < 0)
		PRINTF("bbdev_ipc_enqueue_raw_op failed (%d)\r\n", ret);
}

/* routine used to send a msg to host application */
static void prvSendMsgToHost(uint32_t type, uint32_t status, int args_count, ...)
{
	va_list args;

	struct dfe_msg *msg;

	if (!ipc_up)
		return;

	msg = (struct dfe_msg *)prvGetTxBuf();
	if (!msg)
		return;

	msg->type = type;
	msg->status = status;

	va_start(args, args_count);
	for (int i = 0; i < MIN(args_count, MAX_MSG_PAYLOAD); i++)
		msg->payload[i] = va_arg(args, int);
	
	log_err("Sending message to host: type = %d, status = %d\r\n",
			msg->type, msg->status);

	hexdump((uint8_t *)msg, DFE_MSG_SIZE);
	prvSendRaw((void *)msg, DFE_MSG_SIZE);
}

static int prvBBDEVSetup(void)
{
	struct dev_attr_t *attr;
	int queue_id, ret;

	log_err("Waiting for host to init IPC\r\n");

	/* Wait for bbdev device ready.*/
	while (!bbdev_ipc_is_host_initialized(ipc_dev_id)) {}

	attr = bbdev_ipc_get_dev_attr(ipc_dev_id);
	if (!attr) {
		log_err("Unable to get bbdev attributes\r\n");
		return -1;
	}
	for (queue_id = 0; queue_id < attr->num_queues; queue_id++) {
		ret = bbdev_ipc_queue_configure(ipc_dev_id, queue_id);
		if (ret != IPC_SUCCESS) {
			log_err("queue configure failed (error %d)\r\n", ret);
			return ret;
		}
	}

	/* We're ready to exchange messages with host */
	ipc_up = 1;
	log_err("IPC handshake done\r\n");

	return pdPASS;
}

static void prvDoIPCReset()
{
	log_err("Received an IPC reset notification from host\r\n");

	/* Do local cleanup and prepare for new IPC configuration */

	/* Suspend Rx loops */
	ipc_up = 0;

	/* wait for everything to finish */
	vTaskDelay(1000);

	/* stop and restart IPC */
	bbdev_ipc_close(0, 0);
	if (bbdev_ipc_init(0, 0)) {
		log_err( "BBDEV IPC init failed\r\n" );
		return;
	}

	/* re-issue BBDEV setup */
	if (prvBBDEVSetup() != pdPASS) {
		log_err("BBDEV IPC setup failed\r\n");
		return;
	}

	/* now ready */
	bbdev_ipc_signal_ready(ipc_dev_id);
	log_err("BBDEV IPC reinit done\r\n");
}

static void prvProcessRx(struct dfe_msg *msg)
{
	tDFEPatternConfig pcfg;
	int ret = DFE_NO_ERROR;

	switch (msg->type) {
	case DFE_IPC_RESET:
		prvDoIPCReset();
		break;
	case DFE_TDD_START:
		ret = iTddStart();
		break;
	case DFE_TDD_STOP:
		vTddStop();
		break;
	case DFE_FDD_START:
		vFddStartStop(1);
		break;
	case DFE_FDD_STOP:
		vFddStartStop(0);
		break;
	case DFE_TDD_CFG_PATTERN:
		if (!stop && ulTotalTicks) {
			ret = DFE_OPERATION_RUNNING;
			break;
		}
		pcfg.scs = (msg->payload[0] == 15) ? SCS_kHz15 : SCS_kHz30;
		pcfg.p.dl_slots = msg->payload[1];
		pcfg.p.g1_slots = msg->payload[2];
		pcfg.p.ul_slots = msg->payload[3];
		pcfg.p.g2_slots = msg->payload[4];
		pcfg.p.ul2_slots = msg->payload[5];
		pcfg.p.g3_slots = msg->payload[6];
		vConfigTddPattern(pcfg);
		break;
	case DFE_CFG_RX_ANTENNA:
		ret = iSetRxAntenna(msg->payload[0]);
		break;
	case DFE_CFG_SYM_SIZE:
		sym_size_128 = msg->payload[0];
		break;
	case DFE_CFG_RX_ADDR:
		rx_addr = msg->payload[0];
		break;
	case DFE_CFG_RX_SYM_NUM:
		rx_sym_nr = msg->payload[0];
		break;
	case DFE_CFG_TX_ADDR:
		tx_addr = msg->payload[0];
		break;
	case DFE_CFG_TX_SYM_NUM:
		tx_sym_nr = msg->payload[0];
		break;
	case DFE_CFG_AXIQ_LB_ENABLE:
		vAxiqLoopbackSet(1);
		break;
	case DFE_CFG_AXIQ_LB_DISABLE:
		vAxiqLoopbackSet(0);
		break;
	case DFE_VSPA_DEBUG_BP:
		vVSPADebugBreakPoint();
		break;
	case DFE_CFG_QEC_PARAM:
		vSetQecParam(msg->payload[0], /* tx/rx */
					 msg->payload[1], /* is passthrough ? */
					 msg->payload[2], /* coeff idx */
					 msg->payload[3]  /* coeff val */
					);
		break;
	case DFE_VSPA_DMA_BENCH:
		{
			uint32_t whole = 0, fractional = 0, vspa_cycles = 0;

			vspa_cycles = iBenchmarkVspaTest(msg->payload[0], msg->payload[1], msg->payload[2], msg->payload[3], &whole, &fractional);

			/* send back command's response */
			prvSendMsgToHost(msg->type, ret, 5, whole, fractional, vspa_cycles, msg->payload[2], msg->payload[3]);
			return;
		}
	default:
		prvSendMsgToHost(msg->type, DFE_INVALID_COMMAND, 0);
		break;
	}

	/* ack the command */
	prvSendMsgToHost(msg->type, ret, 0);
}

static void prvProcessHostRx(void)
{
	struct bbdev_ipc_raw_op_t *deq = NULL;
	struct dfe_msg *msg;
	int is_ipc_reset;
	int ret;

	do {
		if (!ipc_up)
			continue;
		deq = bbdev_ipc_dequeue_raw_op(ipc_dev_id, ipc_rx_qid);
	} while (!deq);

	msg = (struct dfe_msg *)deq->in_addr;
	is_ipc_reset = (msg->type == DFE_IPC_RESET);

	prvProcessRx(msg);

	/* If bbdev ipc is in reset state do not dequeue */
	if (is_ipc_reset) {
		return;
	}

	PRINTF("Received message from host: addr = 0x%x, len = %d (msg->type = %#x)\r\n",
		      deq->in_addr, deq->in_len, msg->type);
#if 0
	hexdump((uint8_t *)deq->in_addr, deq->in_len);
#endif

	ret = bbdev_ipc_consume_raw_op(ipc_dev_id, ipc_rx_qid, deq);
	if (ret)
		log_err("bbdev_ipc_consume_raw_op failed(%d)\n\r", ret);
}

static void prvRxLoop(void *pvParameters)
{
	if(prvBBDEVSetup() != pdPASS)
	{
		log_err("Failed to create host polling task\r\n");
		vTaskDelete(NULL);
	}

	/* Signal Host to start sending packets for processing */
	bbdev_ipc_signal_ready(ipc_dev_id);
	log_err("Waiting for messages from host...\n\r");

	while (1) {
		prvProcessHostRx();
	}
}

int vDFEInit(void)
{
	int ret = 0;

	/* stats init */
	ulCurrentSlot = 0;
	ulTotalSlots = 0;
	ulTotalTicks = 0;
	ulVspaMsgCnt = 0;
	ulVspaMsgRxCnt = 0;
	ulVspaMsgTxCnt = 0;

	memset(app_logging, 0, sizeof(app_logging));
	stop = pdFALSE;

	//bVspaProductionBinary = !!((iLa9310AviVspaSwVer() & 0xDFEF0000) == 0xDFEF0000);

	bVspaProductionBinary = 0;

	prvVspaWarmUp();

	/* create a task for host interface */
	ret = xTaskCreate(prvRxLoop, "DFE Host Polling Loop", configMINIMAL_STACK_SIZE * 3,
			  NULL, tskIDLE_PRIORITY + 1, NULL );
	if(ret != pdPASS)
	{
		log_err("Failed to create host polling task\r\n");
		return -1;
	}

	/* TODO: vcxo and misc chips setup */

#if 0 /* not yet used */
	/* create task for VSPA msg receive */
	ret = xTaskCreate(prvVSPALoop, "DFE VSPA Polling Loop", configMINIMAL_STACK_SIZE * 2,
			  NULL, tskIDLE_PRIORITY + 2, NULL );
	if(ret != pdPASS)
	{
		log_err("Failed to create VSPA polling task\r\n");
		return -1;
	}
#endif

	/* create task for Time Agent */
	ret = xTaskCreate(prvTimeAgentTask, "PhyTimer Agent", configMINIMAL_STACK_SIZE * 3,
			  NULL, tskIDLE_PRIORITY + 2, NULL );
	if(ret != pdPASS)
	{
		PRINTF("Failed to create Time Agent task\r\n");
		return -1;
	}

	return ret;
}

void vApplicationMallocFailedHook()
{
	PRINTF("\r\nMemory allocation failed!\r\n");
    prvSendMsgToHost(DFE_MALLOC_FAILED, 0, 0);
	for(;;);
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
	PRINTF("\r\nMemory allocation failed!\r\n");
    prvSendMsgToHost(DFE_STACK_OVERFLOW, 0, 0);
	for(;;);
}