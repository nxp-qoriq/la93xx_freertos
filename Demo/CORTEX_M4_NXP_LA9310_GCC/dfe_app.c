/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "common.h"
#include "immap.h"
#include "la9310_main.h"
#include "la9310_irq.h"
#include "la9310_pinmux.h"
#include "core_cm4.h"
#include "la9310_edmaAPI.h"
#include "bbdev_ipc.h"
#include "sync_timing_device.h"
#include "math.h"
#include "phytimer.h"
#include <time.h>
#include "dfe_avi.h"
#include "dfe_app.h"
#include "dfe_host_if.h"
#include "la9310_avi.h"
#include "../drivers/avi/la9310_vspa_dma.h"
#include "../drivers/avi/la9310_avi_ds.h"
#include "rfnm_rf_ctrl.h"
#include "task_stats.h"

/* VSPA timeouts */
#define NO_WAIT                 ~0
#define NO_TIMEOUT              (-1)
#define VSPA_IMM_RESPONSE       0
#define VSPA_ACK_TIMEOUT_MS     20

#define AXIQ_TAIL_LENGTH        0 /* 0 samples */
#define MAX_SYMBOLS             14 /* maximum number of symbols in slot */
#define MAX_SFNS                1024

#define TDD_SWITCH_DELAY        61 /* 1uS */
#define TDD_SWITCH_TX_ADV       0 /* 0uS - compensated by Host/M7 TDD logic */
#define PHYTIMER_10MS_FRAME     0x96000  /* 61.44MHz * 10000 uS */

#define VSPA_SW_VER_PRODUCTION  0xDFEF0000

#define CM4_DFE_SW_ID           0xDFE00000
#define CM4_DFE_SW_MAJOR        0x00 /* 8-bit */
#define CM4_DFE_SW_MINOR        0x42 /* 8-bit */

#define CM4_DFE_SW_VER          (CM4_DFE_SW_ID | CM4_DFE_SW_MAJOR << 8 | CM4_DFE_SW_MINOR)

bool_t bVspaProductionBinary = pdFALSE;

QueueHandle_t xTimeQueue;
static TimerHandle_t vspa_timer;

/* BBDEV IPC parameters */
static int ipc_up = 0;
static int ipc_host_connected = 0;
static int ipc_dev_id = BBDEV_IPC_DEV_ID_0;
static int ipc_tx_qid = BBDEV_IPC_M2H_QUEUE;
static int ipc_rx_qid = BBDEV_IPC_H2M_QUEUE;

/* default config */
uint32_t uRxAntennaComparator = PHY_TIMER_COMP_CH2_RX_ALLOWED;
uint32_t uRxAntennaComparator2 = PHY_TIMER_COMPARATOR_COUNT; /* not configured */
const uint32_t uTxAntennaComparator = PHY_TIMER_COMP_CH5_TX_ALLOWED;
uint32_t uRxAntennaComparatorMask = PHY_TIMER_COMP_CH2_RX_ALLOWED - 1; /* uRxAntennaComparator-1 -> ADC1 */
uint32_t rx_addr = 0xA0000000;
uint32_t tx_addr = 0xA0010000;
uint32_t sym_size_128 = 35;
uint32_t rx_sym_nr = 14/2;
uint32_t tx_sym_nr = 14/2;
uint8_t  host_bypass_flag_tx_rx = 1;

static uint32_t timestamp_to_start = 0;
static bool bFddIsRunning = pdFALSE;
static volatile uint32_t ulLastPpsInTimestamp = 0;
static volatile uint32_t ulNextTick;
/* SFN/slot keeping */
static volatile uint32_t ulCurrentSlot;
static volatile uint32_t ulCurrentSlotInFrame;
static volatile uint32_t ulCurrentSfn;
static volatile uint32_t ulTotalSlots;

// Intialize Timing Advance to 13us
// This is in PHY TIMER ticks frequency
// Time Advance is to be derived from 2 values
// One is fixed Nta_offset which is 13uS and
// one is Nta_max based on distance between BS and UE
// 3GPP 38.133
static uint32_t iNtaOffset = 13*62; // 25600 *Tc (.509nS) 62 ~= 1uS of PhyTimer
static int32_t iNtaMax = 0;
static int32_t iUplinkTimeAdvance;

static int32_t iTimeOffsetCorr = 0;
static int32_t iTimeOffsetCorrToApply = 0;
/* Application internal counters*/
uint32_t ulTotalTicks;
uint32_t ulVspaMsgCnt;
/* variable used for mainiting the system tick alive after TDD stop */
static volatile bool bKeepTickAlive = pdFALSE;
static volatile bool bTddResume = pdFALSE;
static volatile bool bTddStop = pdFALSE;
static volatile bool bTddStopSentToVSPA = pdFALSE;
static volatile bool bTickConfigured = pdFALSE;
static volatile bool bApplyTimeOffsetCorrection = pdFALSE;
static volatile bool bTimeOffsetCorrectionApplied = pdFALSE;
static volatile bool bTimeOffsetCorrectionTickApply = pdFALSE;
static volatile bool bApplySfnSlotUpdate = pdFALSE;
static volatile bool bSfnSlotUpdateIsDelta = pdFALSE;
static volatile bool bTddIsStopped = pdFALSE;
static volatile bool bIsAllSPattern = pdFALSE;
int32_t sfn_update;
int32_t slot_update;

uint32_t tdd_start1_ts = 0;
uint32_t tdd_start2_ts = 0;
uint32_t tdd_start3_ts = 0;
uint32_t tdd_start4_ts = 0;

enum ePhyTimerComparatorTrigger tx_allowed_last_state = ePhyTimerComparatorNoChange;
enum ePhyTimerComparatorTrigger rx_allowed_last_state = ePhyTimerComparatorNoChange;
uint32_t tx_allowed_start = 0;
uint32_t tx_allowed_stop = 0;
uint32_t rx_allowed_start = 0;
uint32_t rx_allowed_stop = 0;

const uint32_t ofdm_short_sym_time[SCS_MAX] = {
	[SCS_kHz15]  =  4384, /* 15KHz:  (2048 + 144)*2  */
	[SCS_kHz30]  =  2192, /* 30KHz:  (2048 + 144)    */
	[SCS_kHz60]  =  1096, /* 30KHz:  (2048 + 144)/2  */
};

const uint32_t ofdm_long_sym_time[SCS_MAX] = {
	[SCS_kHz15]  = 4416, /* 15KHz:  (2048 + 160)*2  */
	[SCS_kHz30]  = 2224, /* 30KHz:  (2048 + 176)    */
	[SCS_kHz60]  = 1128, /* 60KHz:  (2048 + 208)/2  */
};

const uint32_t slot_duration[SCS_MAX][MAX_SLOT_TYPES] = {
	/* even */
	[SCS_kHz15][0] = 30720 * 2, /* 61.44MHz * 1000us */
	[SCS_kHz30][0] = 30720,     /* 61.44MHz * 500us */
	[SCS_kHz60][0] = 15376,     /* long_sym + short_sym * 13 */
	/* odd */
	[SCS_kHz15][1] = 30720 * 2, /* 61.44MHz * 1000us */
	[SCS_kHz30][1] = 30720,     /* 61.44MHz * 500us */
	[SCS_kHz60][1] = 15344,     /* short_sym * 14 */
};

const uint32_t tick_interval[SCS_MAX] = {
	[SCS_kHz15]  = 30720 * 2,  /* 61.44MHz * 1000us */
	[SCS_kHz30]  = 30720,      /* 61.44MHz * 500us */
	[SCS_kHz60]  = 30720 / 2, /* 61.44MHz * 250us - aprox */
};

const uint32_t max_slots_per_sfn[SCS_MAX] = {
	[SCS_kHz15]  = 10,
#if DFE_TICK_DEBUG
	[SCS_kHz30]  = 4,
#else
	[SCS_kHz30]  = 20,
#endif
	[SCS_kHz60]  = 40,
};

#ifdef LA9310_1SEC_PPS
// 2000 * 500mS = 1Sec
// 1000 * 1ms = 1Sec
// Based on Slot Period
uint32_t ppsOutSkipCount;

const uint32_t ppsSkipCount[SCS_MAX] = {
	[SCS_kHz15]  = (1000 - 1),
	[SCS_kHz30]  = (2000 - 1),
	[SCS_kHz60]  = (4000 - 1),
};
#endif

sTraceEntry app_logging[MAX_TS] = { 0 };
uint32_t debug_ts[MAX_TS] = { 0 };
bool debug = pdTRUE;

tSlot *slots = NULL;
uint8_t scs = SCS_kHz30;

uint32_t log_idx = 0;

const char *event_to_string(uint32_t event)
{
	if ( event < TRACE_MAX )
		return eTraceEventString[event];

	return NULL;
}

static void prvTick( void *pvParameters, long unsigned int param1);
static uint32_t prvSendVspaCmd(struct dfe_mbox *mbox_h2v, uint32_t rsp_type, int32_t vspa_timeout_ms);

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
		vTraceEventRecord(TRACE_PHYTIMER, 0xAAAA1111, target);
		return;
	}
	vTraceEventRecord(TRACE_PHYTIMER, 0xAAAA2222, target);

	/* the target is in the future, wait... */
	if (exit_phy_timer_value >= init_phy_timer_value) {
		do {
			crt_phy_timer_value = uGetPhyTimerTimestamp();
			//vTaskDelay(0);
		} while (crt_phy_timer_value < exit_phy_timer_value);
	}
	else {
		do {
			crt_phy_timer_value = uGetPhyTimerTimestamp();
			//vTaskDelay(0);
		} while ((crt_phy_timer_value >= init_phy_timer_value) || (crt_phy_timer_value < exit_phy_timer_value));
	}
}

/* DFE App TDD Tx/Rx switch */
static inline void switch_txrx(uint32_t mode, uint32_t target_ts, uint32_t stop_tti)
{
	rf_ctrl.mode = mode;
	rf_ctrl.target_phytimer_ts = target_ts - TDD_SWITCH_DELAY;
	rf_ctrl.tti_period_ts = stop_tti; /* 0: do nothing, 1: stop, 2: period aligned to target_ts */

	vPhyTimerComparatorConfig( PHY_TIMER_COMP_RFCTL_5,
							   PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
							   ePhyTimerComparatorOutToggle,
							   (rf_ctrl.issued_phytimer_ts = (uGetPhyTimerTimestamp() + 50)));

	vTraceEventRecord((mode == 0xAAAAAAAA) ? TRACE_RF_TX : TRACE_RF_RX,
						rf_ctrl.issued_phytimer_ts,
						rf_ctrl.target_phytimer_ts);
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

int iSetRxAntenna(uint32_t antenna_mask)
{
	uint8_t iter;
	uint8_t idx = 0;
	uint8_t antenna;

	PRINTF("antenna_mask = %#x\r\n", antenna_mask);
	uRxAntennaComparator = PHY_TIMER_COMPARATOR_COUNT;
	uRxAntennaComparator2 = PHY_TIMER_COMPARATOR_COUNT;

	for (iter = 0; iter < MAX_RX_ANT_SUPPORTED; iter++) {
		/* check if antenna bit is set */
		if (!(antenna_mask & (1 << iter)))
			continue;

		/* compute actual antenna comparator value to be used further */
		antenna = iter + PHY_TIMER_COMP_CH1_RX_ALLOWED;

		if (idx >= MAX_RX_ANT_CONFIG) {
			log_err("Max number of Rx Antennas configured is %d\r\n", MAX_RX_ANT_CONFIG);
			break;
		}

		if (idx == 0) {
			uRxAntennaComparator = antenna;
		} else if (idx == 1) {
			uRxAntennaComparator2 = antenna;
		}

		idx++;
	}

	PRINTF("uRxAntennaComparator  = %d\r\n", uRxAntennaComparator);
	PRINTF("uRxAntennaComparator2 = %d\r\n", uRxAntennaComparator2);

	uRxAntennaComparatorMask = antenna_mask;

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
	uint32_t debug_timestamp = 0;
	uint32_t offset = 0;
	NVIC_SetPriority( IRQ_PPS_OUT, 1 );
	NVIC_EnableIRQ( IRQ_PPS_OUT );

#ifdef LA9310_1SEC_PPS
	setPMUX(0, PMUXCR0_PPSOUT_PIN);
	vPhyTimerComparatorForce(PHY_TIMER_COMP_PPS_OUT, ePhyTimerComparatorOut1);
	ppsOutSkipCount = ppsSkipCount[scs];
#endif

	/* figure out if there's any frame trigger signal */
	ulLastPpsInTimestamp = 0;
	NVIC_SetPriority( IRQ_PPS_IN, 2 );
	NVIC_EnableIRQ( IRQ_PPS_IN );

	/* wait 1200*1ms for frame trigger to happen */
	uint32_t retries = 1100;
	while( (--retries) && (ulLastPpsInTimestamp == 0)) {
		/* wait 1ms betwen retries*/
		vPhyTimerDelay(0xF000); /* 1000uS (1ms) */
	};

	debug_timestamp = uGetPhyTimerTimestamp();

	/* if frame trigger is present, use it */
	if (0 != ulLastPpsInTimestamp) {
		offset = (debug_timestamp - ulLastPpsInTimestamp) % PHYTIMER_10MS_FRAME;
		ulNextTick = debug_timestamp + PHYTIMER_10MS_FRAME - offset;
	} else {
		ulNextTick = debug_timestamp;
	}

	ulNextTick += PHYTIMER_10MS_FRAME;
	ulNextTick -= slot_duration[scs][0];

	vPhyTimerComparatorConfig( PHY_TIMER_COMP_PPS_OUT,
			PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
			ePhyTimerComparatorOutToggle,
			ulNextTick );

	/* given that odd/even slots in SCS 60kHz differ in duration, use a workaround for gerenrating periodic TTIs */
	tti_trigger(ulNextTick, (scs == SCS_kHz60) ? (slot_duration[scs - 1][0] / 2) : slot_duration[scs][0]);

	PRINTF("ulLastPpsInTimestamp = %#x\r\n" \
	       "                 now = %#x\r\n" \
	       "          ulNextTick = %#x\r\n\r\n",
	       ulLastPpsInTimestamp,
	       debug_timestamp,
	       ulNextTick);

	bTickConfigured = pdTRUE;
}

void vPhyTimerPPSINHandler()
{
	NVIC_ClearPendingIRQ( IRQ_PPS_IN );
	ulLastPpsInTimestamp = ulPhyTimerComparatorRead( PHY_TIMER_COMP_PPS_IN );
	NVIC_DisableIRQ( IRQ_PPS_IN );
}

void vPhyTimerPPSOUTHandler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	NVIC_ClearPendingIRQ( IRQ_PPS_OUT );

	if (bFddIsRunning)
	{
		ulNextTick += slot_duration[SCS_kHz30][0] * max_slots_per_sfn[SCS_kHz30]; /* 10ms */
		vPhyTimerComparatorConfig( PHY_TIMER_COMP_PPS_OUT,
				PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
				ePhyTimerComparatorOutToggle,
				ulNextTick );

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		return;
	}
	/* Update next tick's timestamp */
	ulNextTick += slot_duration[scs][ulCurrentSlotInFrame % 2];

	/* check if TO needs to be applied */
	if (bApplyTimeOffsetCorrection) {
		ulNextTick += iTimeOffsetCorr;
		bApplyTimeOffsetCorrection = pdFALSE;
		bTimeOffsetCorrectionApplied = pdTRUE;
		iTimeOffsetCorrToApply = iTimeOffsetCorr;
	}

	/* log tick */
	vTraceEventRecord(TRACE_TICK, 0xBBBBBBBB, ulNextTick);

	vPhyTimerComparatorConfig( PHY_TIMER_COMP_PPS_OUT,
				PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
				ePhyTimerComparatorOutToggle,
				ulNextTick );

	if (!tdd_start1_ts)
			tdd_start1_ts = ulNextTick;

	/* Call the tick callback */
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
	if ((vspa_timeout_ms != NO_WAIT) && (vspa_timeout_ms > 0))
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
#ifdef _NOT_IMPLEMENTED
	static uint8_t first_operation = 1;
#endif
	struct dfe_mbox mbox_h2v;

	/* send semistatic config */
	MBOX_CLEAR(mbox_h2v);
	MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_SEMISTATIC);

	MBOX_SET_RX_TDD_FDD(mbox_h2v, tdd0_fdd1);
	MBOX_SET_TX_TDD_FDD(mbox_h2v, tdd0_fdd1);
	MBOX_SET_CPE_BS_MODE(mbox_h2v, 1); /*ue*/
	MBOX_SET_DCS_MAPPING(mbox_h2v, 1); /* adc 0,1,2,3 */
	/* Rx ADC is 0-based */
	MBOX_SET_RX_IDX(mbox_h2v, uRxAntennaComparatorMask);
	/* la9310 has a single Tx - ch5. VSPA expects 0 */
	MBOX_SET_TX_IDX(mbox_h2v, 0);

	/* running outside fr1_fr2_tool requires these bits */
	MBOX_SET_HOST_BYPASS_TX(mbox_h2v, host_bypass_flag_tx_rx);
	MBOX_SET_HOST_BYPASS_RX(mbox_h2v, host_bypass_flag_tx_rx);

	/* IQ swap settings */
	MBOX_SET_RX_IQSWAP(mbox_h2v, 0);
	MBOX_SET_TX_IQSWAP(mbox_h2v, 0);
	/* TDD SCS config */
	MBOX_SET_SCS(mbox_h2v, scs);

#ifdef _NOT_IMPLEMENTED
	/* production VSPA requires slightly different handling */
	/* switching between modes requires chan_start updates*/
	if (bVspaProductionBinary) {
		if (!first_operation) {
			MBOX_SET_RX_CHAN_RESTART(mbox_h2v, 1);
			MBOX_SET_TX_CHAN_RESTART(mbox_h2v, 1);
		}
	}
#endif

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

#ifdef _NOT_IMPLEMENTED
	first_operation = 0;
#endif
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
		ret = vDFEMbxReceive(&vspa_ctrl_msg, DFE_OPS_MBOX_ID, 0);
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

static void vGetSymbolsStartStop( tSlot *slot, uint32_t is_dl, uint32_t *start_offset, uint32_t *stop_offset, uint32_t slot_idx )
{
	uint32_t start_time = 0;
	uint32_t stop_time = 0;
	uint32_t start_symbol = ((is_dl == 1) ? slot->start_symbol_dl : slot->start_symbol_ul);
	uint32_t end_symbol = ((is_dl == 1) ? slot->end_symbol_dl : slot->end_symbol_ul);

	//vTraceEventRecord( (is_dl == 1) ? TRACE_SLOT_DL : TRACE_SLOT_UL, start_symbol, end_symbol);

	/* compute start time for slot */
	if ( start_symbol > 0 ) {
		if ((scs == SCS_kHz60) && ((slot_idx % MAX_SLOT_TYPES) == 1))
			start_time += ofdm_short_sym_time[scs];
		else
			start_time += ofdm_long_sym_time[scs];
		start_time += ofdm_short_sym_time[scs] * (start_symbol - 1);
	}

	/* compute stop time for slot */
	if ( end_symbol < (MAX_SYMBOLS - 1) ) {
		stop_time += ofdm_short_sym_time[scs] * (MAX_SYMBOLS - end_symbol - 1);
	}

	/* make adjustments for SCS_kHz15 */
	/* SCS-15kHz has 2 long cyclic prefix symbols */
	if ( scs == SCS_kHz15 ) {
		if ( start_symbol > 7) {
			start_time += ofdm_long_sym_time[scs];
			start_time -= ofdm_short_sym_time[scs];
		}

		if ( end_symbol < 7 ) {
			stop_time -= ofdm_short_sym_time[scs];
			stop_time += ofdm_long_sym_time[scs];
		}
	}

	//vTraceEventRecord( (is_dl == 1) ? TRACE_SLOT_DL : TRACE_SLOT_UL, start_time, slot_duration[scs][slot_idx % MAX_SLOT_TYPES] - stop_time);

	*start_offset = start_time;
	*stop_offset = slot_duration[scs][slot_idx % MAX_SLOT_TYPES] - stop_time;
}

inline void vSendTimeAgentMessage(uint32_t target_timestamp,
								  enum ePhyTimerComparatorTrigger tx_state,
								  enum ePhyTimerComparatorTrigger rx_state,
								  uint32_t exec_after_timestamp)
{
	sTimeAgentMessage msg;

	msg.target_timestamp = target_timestamp;
	msg.tx_allowed_state = tx_state;
	msg.rx_allowed_state = rx_state;
	msg.exec_after_timestamp = exec_after_timestamp;

#if 1
	/* We have not woken a task at the start of the ISR. */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if ( errQUEUE_FULL == xQueueSendFromISR( xTimeQueue, ( void * ) &msg, &xHigherPriorityTaskWoken ) )
	{
		PRINTF("xQueueSendFromISR error! queue full!\r\n");
		while(1);;
	}
#else
	xQueueSend( xTimeQueue,
               ( void * ) &msg,
               ( TickType_t ) 0 );

#endif
}

uint32_t tx_allowed_on = 0;
uint32_t tx_allowed_off = 0;
bool_t tx_allowed_on_set = 0;
bool_t tx_allowed_off_set = 0;
uint32_t rx_allowed_on = 0;
uint32_t rx_allowed_off = 0;
bool_t rx_allowed_on_set = 0;
bool_t rx_allowed_off_set = 0;
uint32_t tx_axiq_tail = 0;
bool_t first604 = 0;

static void prvTick( void *pvParameters, long unsigned int param1 )
{
	struct la9310_hif * hif = pLa9310Info->pHif;
	struct dfe_mbox mbox_h2v = {0};
	bool_t bIsDownlinkSlot = 0;
	bool_t bIsUplinkSlot = 0;
	uint32_t start_offset_dl = 0;
	uint32_t stop_offset_dl = 0;
	uint32_t start_offset_ul = 0;
	uint32_t stop_offset_ul = 0;

#ifdef LA9310_1SEC_PPS
  if (ppsOutSkipCount < ppsSkipCount[scs]) {
    if (0 == ppsOutSkipCount) {
      // Set PMUXCR0 to GPIO
      setPMUX(0, PMUXCR0_PPSOUT_PIN);
    }
    ppsOutSkipCount++;
  } else {
    // Set PMUXCR0 to PPS_OUT
    ppsOutSkipCount = 0;
    clearPMUX(0, PMUXCR0_PPSOUT_PIN);
    //PRINTF("%d : PMUX %d  timer 0x%x\r\n",ppsOutSkipCount, readPMUX(0),uGetPhyTimerTimestamp());
  }
  //PRINTF("%d : PMUX %d \r\n",ppsOutSkipCount, readPMUX(0));
#endif

	/* check stop condition */
	if ( bTddStopSentToVSPA && bTddStop ) {
		/* disable tick and any other used comparator */
		if (!bKeepTickAlive) {
			vPhyTimerComparatorDisable( PHY_TIMER_COMP_PPS_OUT );
			bTickConfigured = pdFALSE;
		}

		/* stop RF Tx if on */
		switch_txrx(0xBBBBBBBB, ulNextTick, !bKeepTickAlive /* stop tti*/);

		/* stop Tx Allowed */
		if (tx_allowed_off_set) {
			/* wait for previous event to occur */
			vPhyTimerWaitComparator(tx_allowed_on);
			vPhyTimerComparatorConfig( uTxAntennaComparator,
										PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
										ePhyTimerComparatorOut0,
										tx_allowed_off );
			vTraceEventRecord(TRACE_AXIQ_TX, 0x601, tx_allowed_off);
			tx_allowed_off_set = 0;
			tx_allowed_on_set = 0;
			//PRINTF("\r\n\r\nTX OFF !\r\n\r\n");
		}

		if (rx_allowed_off_set) {
			/* for RX do not wait for previous event to occur given that in cell search (DDD..D)
			 * the rx allowed on was far away in the past that could be considered a future event,
			 * resulting in a wrong behavior.
			 */
			//vPhyTimerWaitComparator(rx_allowed_on);
			vPhyTimerComparatorConfig( uRxAntennaComparator,
										PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
										ePhyTimerComparatorOut0,
										rx_allowed_off );

			if (isConfigured(uRxAntennaComparator2))
				vPhyTimerComparatorConfig( uRxAntennaComparator2,
											PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
											ePhyTimerComparatorOut0,
											rx_allowed_off );


			vTraceEventRecord(TRACE_AXIQ_RX, 0x501, rx_allowed_off);
			rx_allowed_off_set = 0;
			rx_allowed_on_set = 0;
			//PRINTF("\r\n\r\nRX OFF !\r\n\r\n");
		}

		/* reset the flag */
		bTddStopSentToVSPA = pdFALSE;
		bTddIsStopped = pdTRUE;

		//PRINTF("\r\n\r\nISR stop stop stop! bKeepTickAlive = %d, bTddStopSentToVSPA = %d, bTddResume = %d\r\n\r\n",
		//	bKeepTickAlive, bTddStopSentToVSPA, bTddResume);

		/* trace the stop event */
		if (!bKeepTickAlive) {
			vTraceEventRecord(TRACE_TICK, 0xDEADDEAD, ulNextTick);
			return;
		}
	}

	/* notify host about current slot/sfn */
	hif->stats.dbg1 = ulCurrentSfn;
	hif->stats.dbg2 = ulCurrentSlotInFrame;

	if (ipc_host_connected)
		prvSendMsgToHost(DFE_TTI_MESSAGE, 0, 2 /* payload words*/, ulCurrentSlotInFrame, ulCurrentSfn);

	//vTraceEventRecord(TRACE_TICK, 0xFFFF0002, ulNextTick);
#if 1
	if (bTddResume && bKeepTickAlive) {
		vTraceEventRecord(TRACE_TICK, 0xFFFF0003, ulCurrentSlotInFrame);
		/* if tdd start command is issued and keep is alive, start pattern aligned to frame */
		if (ulCurrentSlotInFrame != 0) //(max_slots_per_sfn[scs] - 1) )
			goto update_sfn_slot;

		ulCurrentSlot = 0;

		vTraceEventRecord(TRACE_TICK, 0x55555555, ulCurrentSfn);
		vTraceEventRecord(TRACE_TICK, 0x55555555, ulCurrentSlotInFrame);

		bTddResume = pdFALSE;
		bTddIsStopped = pdFALSE;
		if (!tdd_start2_ts)
			tdd_start2_ts = ulNextTick;
		else if (!tdd_start3_ts)
			tdd_start3_ts = ulNextTick;
		else
			tdd_start4_ts = ulNextTick;
	}
#endif

	/* log current tick */
	vTraceEventRecord(TRACE_SLOT, ulCurrentSlot, ulTotalSlots);

	/* if TDD is stopped but tick must be kept alive, update sfn/slot */
	if (bTddIsStopped && bKeepTickAlive) {
		goto update_sfn_slot;
	}

	/* look into pre-computed slot array */
	bIsDownlinkSlot = slots[ulCurrentSlot].is_dl;
	bIsUplinkSlot = slots[ulCurrentSlot].is_ul;

	if (bIsDownlinkSlot) {
		vGetSymbolsStartStop(&slots[ulCurrentSlot], 1 /*is_dl*/, &start_offset_dl, &stop_offset_dl, ulCurrentSlotInFrame);
		vTraceEventRecord(TRACE_SLOT_DL, start_offset_dl, stop_offset_dl);
	}

	if (bIsUplinkSlot) {
		vGetSymbolsStartStop(&slots[ulCurrentSlot], 0 /*is_dl*/, &start_offset_ul, &stop_offset_ul, ulCurrentSlotInFrame);
		vTraceEventRecord(TRACE_SLOT_UL, start_offset_ul, stop_offset_ul);
	}

#ifdef DFE_CONSECUTIVE_S_SLOTS_WA
	// consecutive S-slots
	if (bIsAllSPattern) {
			rx_allowed_start = ulNextTick + start_offset_dl;
			rx_allowed_stop = ulNextTick + stop_offset_dl - slot_duration[scs][ulCurrentSlotInFrame % 2];
			tx_allowed_start = ulNextTick - iUplinkTimeAdvance + start_offset_ul - slot_duration[scs][ulCurrentSlotInFrame % 2];
			tx_allowed_stop = ulNextTick - iUplinkTimeAdvance + stop_offset_ul + AXIQ_TAIL_LENGTH - slot_duration[scs][ulCurrentSlotInFrame % 2];

	} else {
#endif /* DFE_CONSECUTIVE_S_SLOTS_WA */
		if (rx_allowed_off_set) {
			vPhyTimerComparatorConfig( uRxAntennaComparator,
										PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
										ePhyTimerComparatorOut1,
										rx_allowed_stop );
		}

		if (bIsDownlinkSlot) {
			rx_allowed_start = ulNextTick + start_offset_dl;
			rx_allowed_stop = ulNextTick + stop_offset_dl;
		}

		if (bIsUplinkSlot) {
			tx_allowed_start = ulNextTick - iUplinkTimeAdvance + start_offset_ul;
			tx_allowed_stop = ulNextTick - iUplinkTimeAdvance + stop_offset_ul + AXIQ_TAIL_LENGTH;
		}
#ifdef DFE_CONSECUTIVE_S_SLOTS_WA
	}
#endif /* DFE_CONSECUTIVE_S_SLOTS_WA */

	if (bTimeOffsetCorrectionApplied) {
		tx_allowed_stop -= iTimeOffsetCorrToApply;
		rx_allowed_stop -= iTimeOffsetCorrToApply;
		bTimeOffsetCorrectionApplied = pdFALSE;
		bTimeOffsetCorrectionTickApply = pdTRUE;
	}

#ifdef DFE_CONSECUTIVE_S_SLOTS_WA
	if (bIsAllSPattern) {
		tx_allowed_off = tx_allowed_stop;
		rx_allowed_off = rx_allowed_stop;
		tx_allowed_on = tx_allowed_start;
		rx_allowed_on = rx_allowed_start;

		// At this step RxAllow OFF and TxAllowed ON
		if (rx_allowed_off_set) {
			vPhyTimerComparatorConfig( uRxAntennaComparator,
										PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
										ePhyTimerComparatorOut0,
										rx_allowed_off );

			if (isConfigured(uRxAntennaComparator2))
				vPhyTimerComparatorConfig( uRxAntennaComparator2,
											PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
											ePhyTimerComparatorOut0,
											rx_allowed_off );
			vTraceEventRecord(TRACE_AXIQ_RX, 0x501, rx_allowed_off);
		}

		if (tx_allowed_on_set) {
			vPhyTimerComparatorConfig( uTxAntennaComparator,
										PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
										ePhyTimerComparatorOut1,
										tx_allowed_on );
			vTraceEventRecord(TRACE_AXIQ_TX, 0x602, tx_allowed_on);

			/* avoid potential issue in M7 TDD Tx/Rx switch code*/
			if (rx_allowed_off_set)
				vPhyTimerWaitComparator(rx_allowed_off);

			switch_txrx(0xAAAAAAAA, tx_allowed_on - TDD_SWITCH_TX_ADV, 0);
		}

		rx_allowed_off_set = 1;
		tx_allowed_off_set = 1;
		tx_allowed_on_set = 1;
		rx_allowed_on_set = 1;
	}
	else {
#endif /* DFE_CONSECUTIVE_S_SLOTS_WA */
		if (bIsDownlinkSlot) {
			if (!rx_allowed_on_set) {
				rx_allowed_on = rx_allowed_start;

				vPhyTimerComparatorConfig( uRxAntennaComparator,
											PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
											ePhyTimerComparatorOut1,
											rx_allowed_on );

				if (isConfigured(uRxAntennaComparator2))
					vPhyTimerComparatorConfig( uRxAntennaComparator2,
												PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
												ePhyTimerComparatorOut1,
												rx_allowed_on );
				// wait for prev command to complete before sending new command
				if (tx_allowed_on_set)
					vPhyTimerWaitComparator(tx_allowed_on);

				switch_txrx(0xBBBBBBBB, rx_allowed_on, (bTimeOffsetCorrectionTickApply) ? (tick_interval[scs] - GPT3_CORRECTION_FACTOR_500US): 0);
				vTraceEventRecord(TRACE_AXIQ_RX, 0x502, rx_allowed_on);
				rx_allowed_on_set = 1;
				bTimeOffsetCorrectionTickApply = pdFALSE;
			}

			rx_allowed_off = rx_allowed_stop;
			rx_allowed_off_set = 1;

			if (slots[ulCurrentSlot].end_symbol_dl < (MAX_SYMBOLS - 1) ) {
				rx_allowed_on_set = 0;
			}
		}

		if (bIsUplinkSlot) {
			if (!tx_allowed_on_set) {
				tx_allowed_on = tx_allowed_start;

				vPhyTimerComparatorConfig( uTxAntennaComparator,
											PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
											ePhyTimerComparatorOut1,
											tx_allowed_on );
				switch_txrx(0xAAAAAAAA, tx_allowed_on - TDD_SWITCH_TX_ADV, 0);
				vTraceEventRecord(TRACE_AXIQ_TX, 0x602, tx_allowed_on);
				tx_allowed_on_set = 1;
			}

			tx_allowed_off = tx_allowed_stop;

			/* A-011354: Tx allowed length is expected to be: (TX_window_size_in_bytes+255)/256 * 256) */
			tx_axiq_tail = ((tx_allowed_off - tx_allowed_on + 63)/64) * 64 - tx_allowed_off + tx_allowed_on;
			vTraceEventRecord(TRACE_AXIQ_TX, 0x603, tx_axiq_tail);

			tx_allowed_off += tx_axiq_tail;
			tx_allowed_off_set = 1;

			if (slots[ulCurrentSlot].end_symbol_ul < (MAX_SYMBOLS - 1) ) {
				tx_allowed_on_set = 0;
			}
		}
#ifdef DFE_CONSECUTIVE_S_SLOTS_WA
	}
#endif /* DFE_CONSECUTIVE_S_SLOTS_WA */

	/* VSPA messaging */
	memset(&mbox_h2v, 0, sizeof(struct dfe_mbox));
	/* Msg TDD operation 0x3, sfn, slot */
	MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_TDD);
	MBOX_SET_SLOT_IDX(mbox_h2v, ulCurrentSlotInFrame);

	/* stop TDD bit */
	if ((ulCurrentSlotInFrame == (max_slots_per_sfn[scs] - 1)) && bTddStop) {
		MBOX_SET_TDD_STOP(mbox_h2v, 1);
		bTddStopSentToVSPA = pdTRUE;
	}

	/* Downlink */
	if (bIsDownlinkSlot) {
		MBOX_SET_RX_START_SYM(mbox_h2v, slots[ulCurrentSlot].start_symbol_dl);
		MBOX_SET_RX_NR_SYM(mbox_h2v, slots[ulCurrentSlot].end_symbol_dl - slots[ulCurrentSlot].start_symbol_dl + 1);
	}

	/* Uplink */
	if (bIsUplinkSlot) {
		MBOX_SET_TX_START_SYM(mbox_h2v, slots[ulCurrentSlot].start_symbol_ul);
		MBOX_SET_TX_NR_SYM(mbox_h2v, slots[ulCurrentSlot].end_symbol_ul - slots[ulCurrentSlot].start_symbol_ul + 1);

#ifdef DFE_CONSECUTIVE_S_SLOTS_WA
		// consecutive S slots
		if (bIsAllSPattern) {
			/* exploit this condition to avoid sending the first message*/
			if (first604) {
				MBOX_SET_PARAM2(mbox_h2v, (tx_allowed_on - uGetPhyTimerTimestamp())*491/61);
				vTraceEventRecord(TRACE_AXIQ_TX, 0x604, MBOX_GET_PARAM2(mbox_h2v));
			}
		} else {
#endif /* DFE_CONSECUTIVE_S_SLOTS_WA */
			/* tell VSPA the time when Tx Allowed will be enabled */
			if (tx_allowed_on == tx_allowed_start) {
				MBOX_SET_PARAM2(mbox_h2v, (tx_allowed_on - uGetPhyTimerTimestamp())*491/61);
				vTraceEventRecord(TRACE_AXIQ_TX, 0x604, MBOX_GET_PARAM2(mbox_h2v));
			}
#ifdef DFE_CONSECUTIVE_S_SLOTS_WA
		}
#endif /* DFE_CONSECUTIVE_S_SLOTS_WA */
	}

	prvSendVspaCmd(&mbox_h2v, 0xDEAD, NO_WAIT);

update_sfn_slot:

	if (bApplySfnSlotUpdate) {
		if (!bSfnSlotUpdateIsDelta) {
			ulCurrentSfn = sfn_update;
			ulCurrentSlotInFrame = slot_update;
		} else {
			ulCurrentSfn += sfn_update;
			ulCurrentSlotInFrame += slot_update;

			if (ulCurrentSfn < 0)
				ulCurrentSfn += MAX_SFNS;

			if (ulCurrentSlotInFrame < 0)
				ulCurrentSlotInFrame += max_slots_per_sfn[scs];
		}

		ulCurrentSlot = ulCurrentSlotInFrame % ulTotalSlots;
		bApplySfnSlotUpdate = pdFALSE;
	}

	/* update current slot */
	ulCurrentSlot++;
	ulCurrentSlot %= ulTotalSlots;
	ulTotalTicks++;
	/* pattern may be shorter than frame */
	ulCurrentSlotInFrame++;
	ulCurrentSlotInFrame %= max_slots_per_sfn[scs];
	if (ulCurrentSlotInFrame == 0)
	{
		ulCurrentSfn++;
		ulCurrentSfn %= MAX_SFNS;
	}

	if (bTddIsStopped && bKeepTickAlive)
		goto tick_end;

#ifdef DFE_CONSECUTIVE_S_SLOTS_WA
	if (bIsAllSPattern) {
		// At this step RxAllowed ON and TxAllowed OFF
		if (rx_allowed_on_set) {
			if (rx_allowed_off_set)
				vPhyTimerWaitComparator(rx_allowed_off);

			vPhyTimerComparatorConfig( uRxAntennaComparator,
										PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
										ePhyTimerComparatorOut1,
										rx_allowed_on );

			if (isConfigured(uRxAntennaComparator2))
				vPhyTimerComparatorConfig( uRxAntennaComparator2,
											PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
											ePhyTimerComparatorOut1,
											rx_allowed_on );
			vTraceEventRecord(TRACE_AXIQ_RX, 0x10000502, rx_allowed_on);
		}

		if (tx_allowed_off_set && first604) {
			/* A-011354: Tx allowed length is expected to be: (TX_window_size_in_bytes+255)/256 * 256) */
			tx_axiq_tail = ((tx_allowed_off - tx_allowed_on + 63)/64) * 64 - tx_allowed_off + tx_allowed_on;
			vTraceEventRecord(TRACE_AXIQ_TX, 0x603, tx_axiq_tail);
			tx_allowed_off += tx_axiq_tail;
			if (tx_allowed_on_set)
				vPhyTimerWaitComparator(tx_allowed_on);

			vPhyTimerComparatorConfig( uTxAntennaComparator,
										PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
										ePhyTimerComparatorOut0,
										tx_allowed_off );
			/* RF Tx->Rx here; Also use this command to update TTI boundary */
			switch_txrx(0xBBBBBBBB, tx_allowed_off - tx_axiq_tail, (bTimeOffsetCorrectionTickApply) ? (tick_interval[scs] - GPT3_CORRECTION_FACTOR_500US): 0);
			vTraceEventRecord(TRACE_AXIQ_TX, 0x10000601, tx_allowed_off);
		}

		rx_allowed_on = rx_allowed_start;
		tx_allowed_off = tx_allowed_stop;
		rx_allowed_on_set = 1;
		tx_allowed_off_set = 1;
		first604 = 1;
	} else {
#endif /* DFE_CONSECUTIVE_S_SLOTS_WA */
		if (!bIsDownlinkSlot) {
			if (rx_allowed_off_set) {
				if (rx_allowed_on_set)
					vPhyTimerWaitComparator(rx_allowed_start);

				vPhyTimerComparatorConfig( uRxAntennaComparator,
										PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
										ePhyTimerComparatorOut0,
										rx_allowed_off );

				if (isConfigured(uRxAntennaComparator2))
					vPhyTimerComparatorConfig( uRxAntennaComparator2,
											PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
											ePhyTimerComparatorOut0,
											rx_allowed_off );

				/* do nothing to turn off RF Rx */
				//switch_txrx(0xBBBBBBBB, rx_allowed_off, 0);
				vTraceEventRecord(TRACE_AXIQ_RX, 0x501, rx_allowed_off);
				rx_allowed_off_set = 0;
			}

			rx_allowed_on_set = 0;
		}

		if (!bIsUplinkSlot) {
			if (tx_allowed_off_set) {
				if (tx_allowed_on_set)
					vPhyTimerWaitComparator(tx_allowed_start);

				vPhyTimerComparatorConfig( uTxAntennaComparator,
										PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
										ePhyTimerComparatorOut0,
										tx_allowed_off );
				/* turn Rx on to stop the RF Tx */
				switch_txrx(0xBBBBBBBB, tx_allowed_off - tx_axiq_tail, 0);
				vTraceEventRecord(TRACE_AXIQ_TX, 0x601, tx_allowed_off);
				tx_allowed_off_set = 0;
			}

			tx_allowed_on_set = 0;
		}
#ifdef DFE_CONSECUTIVE_S_SLOTS_WA
	}
#endif /* DFE_CONSECUTIVE_S_SLOTS_WA */

#if DFE_TICK_DEBUG
	/* DEBUG: add stop condition after end of pattern */
	if ( ulTotalTicks > ulTotalSlots )
	if (ulTotalTicks && (ulCurrentSlot == 0))
		bTddStop = pdTRUE;
#endif

tick_end:
	vTraceEventRecord(TRACE_TICK, 0xEEEEEEEE, ulNextTick);
}

void dump_slots()
{
	PRINTF("\r\n");
	for (uint8_t i = 0; i < ulTotalSlots; i++) {
		PRINTF("Slot %d [dl=%d, ul=%d]:\r\n", i, slots[i].is_dl, slots[i].is_ul);
		PRINTF("\t start_sym: dl=%2d, ul=%2d\r\n", slots[i].start_symbol_dl, slots[i].start_symbol_ul);
		PRINTF("\t end_sym:   dl=%2d, ul=%2d\r\n", slots[i].end_symbol_dl, slots[i].end_symbol_ul);
	}
}

void vPrintTddPattern()
{
	PRINTF("\r\nBuffer configuration:\r\n");
	PRINTF("\trx_addr  = 0x%08x, rx_syms_num = %d\r\n", rx_addr, rx_sym_nr);
	PRINTF("\ttx_addr  = 0x%08x, tx_syms_num = %d\r\n", tx_addr, tx_sym_nr);
	PRINTF("\tsym_size = %d (%d * 128 bytes)\r\n", sym_size_128 * 128, sym_size_128);

	PRINTF("bIsAllSPattern = %d\r\n", bIsAllSPattern);
	PRINTF("\r\nPattern configuration: %s\r\n", bIsAllSPattern ? "consecutive S-slots": "");
	PRINTF("\tSCS: %s\r\n", scs == SCS_kHz15 ? "15 kHz" : ((scs == SCS_kHz30) ? "30 kHz": "60 kHz"));

	PRINTF("\tPattern: ");
	for (uint8_t i = 0; i < ulTotalSlots; i++) {
		if (slots[i].is_dl && slots[i].is_ul)
			PRINTF("S");
		else if (!slots[i].is_dl && !slots[i].is_ul)
			PRINTF("G");
		else if (slots[i].is_dl)
			PRINTF("D");
		else
			PRINTF("U");
	}
	PRINTF("\r\n\r\n");

	dump_slots();
}

/* helper API to configure a predefined pattern from console */
void vSetupTddPattern(uint32_t cfg_scs)
{
	uint8_t k = 0;

	scs = cfg_scs;
	memset(slots, 0, MAX_SLOTS * sizeof(tSlot));

#define SETUP_SLOT_DL(field, count, dls, dle)       \
	do {                                            \
		for (uint8_t i = 0; i < (field); i++) {     \
			slots[(count)].is_dl = 1;               \
			slots[(count)].is_ul = 0;               \
			slots[(count)].start_symbol_dl = (dls); \
			slots[(count)].end_symbol_dl = (dle);   \
			slots[(count)].start_symbol_ul = 0;     \
			slots[(count)].end_symbol_ul = 0;       \
			(count)++;                              \
		}                                           \
	} while(0)

#define SETUP_SLOT_UL(field, count, uls, ule)       \
	do {                                            \
		for (uint8_t i = 0; i < (field); i++) {     \
			slots[(count)].is_dl = 0;               \
			slots[(count)].is_ul = 1;               \
			slots[(count)].start_symbol_dl = 0;     \
			slots[(count)].end_symbol_dl = 0;       \
			slots[(count)].start_symbol_ul = (uls); \
			slots[(count)].end_symbol_ul = (ule);   \
			(count)++;                              \
		}                                           \
	} while(0)

#define SETUP_SLOT_MIXED(count, dls, dle, uls, ule) \
	do {                                            \
		slots[(count)].is_dl = 1;                   \
		slots[(count)].is_ul = 1;                   \
		slots[(count)].start_symbol_dl = (dls);     \
		slots[(count)].end_symbol_dl = (dle);       \
		slots[(count)].start_symbol_ul = (uls);     \
		slots[(count)].end_symbol_ul = (ule);       \
		(count)++;                                  \
	} while(0)

#define SETUP_SLOT_G(field, count)                  \
	do {                                            \
		for (uint8_t i = 0; i < (field); i++) {     \
			slots[(count)].is_dl = 0;               \
			slots[(count)].is_ul = 0;               \
			slots[(count)].start_symbol_dl = 0;     \
			slots[(count)].end_symbol_dl = 0;       \
			slots[(count)].start_symbol_ul = 0;     \
			slots[(count)].end_symbol_ul = 0;       \
			(count)++;                              \
		}                                           \
	} while(0)

	SETUP_SLOT_DL(3, k, 0, 13);        //DDD
	SETUP_SLOT_MIXED(k, 0, 5, 10, 13); //S 6:4:4
	SETUP_SLOT_UL(1 , k, 0, 13);       //U

	ulTotalSlots = k;

	vPrintTddPattern();
}

void vTddApplyTimeOffsetCorrection(int time_offset)
{
#if 0
	PRINTF("TO = %d\r\n", time_offset);
#endif
	iTimeOffsetCorr = time_offset;
	bApplyTimeOffsetCorrection = pdTRUE;
}

void vTddApplySfnSlotUpdate(uint32_t is_delta, int32_t sfn, int32_t slot)
{
#if 0
	PRINTF("is_delta = %d\r\n", is_delta);
	PRINTF("     sfn = %d\r\n", sfn);
	PRINTF("    slot = %d\r\n", slot);
#endif
	bSfnSlotUpdateIsDelta = !!is_delta;
	sfn_update = sfn;
	slot_update = slot;

	bApplySfnSlotUpdate = pdTRUE;
}

#if 0
static void prvTimeAgentTask(void *pvParameters)
{
	sTimeAgentMessage msg;
	uint32_t prev_tx_state = ePhyTimerComparatorNoChange;
	uint32_t prev_rx_state = ePhyTimerComparatorNoChange;

	/* Create a queue capable of containing 10 elements */
	xTimeQueue = xQueueCreate( 10, sizeof( sTimeAgentMessage ) );

	if( xTimeQueue == NULL )
	{
		/* Queue was not created and must not be used. */
		PRINTF("xQueueCreate error!\r\n");
		return;
    }

	/* task loop */
	for ( ;; )
	{
		if( xQueueReceive( xTimeQueue,
							&( msg ),
							( TickType_t ) 0 ) == pdPASS )
		{
			vTraceEventRecord(TRACE_PHYTIMER, msg.target_timestamp, msg.tx_allowed_state | (msg.rx_allowed_state << 16));

			if (msg.rx_allowed_state != prev_rx_state)
			{
				vTraceEventRecord(TRACE_PHYTIMER, 0x500 + msg.rx_allowed_state, msg.exec_after_timestamp);
				vPhyTimerWaitComparator(msg.exec_after_timestamp);
				vTraceEventRecord(TRACE_PHYTIMER, 0x500 + msg.rx_allowed_state, msg.target_timestamp);
				vPhyTimerComparatorConfig( uRxAntennaComparator,
											PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
											msg.rx_allowed_state,
											msg.target_timestamp );
				prev_rx_state = msg.rx_allowed_state;
			}

			if (msg.tx_allowed_state != prev_tx_state)
			{
				vTraceEventRecord(TRACE_PHYTIMER, 0x600 + msg.tx_allowed_state, msg.exec_after_timestamp);
				vPhyTimerWaitComparator(msg.exec_after_timestamp);
				vTraceEventRecord(TRACE_PHYTIMER, 0x600 + msg.tx_allowed_state, msg.target_timestamp);
				vPhyTimerComparatorConfig( uTxAntennaComparator,
											PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
											msg.tx_allowed_state,
											msg.target_timestamp );
				prev_tx_state = msg.tx_allowed_state;
			}
		}
		vTaskDelay(1);
	}
}
#endif

void prvConfigTdd()
{
	vTraceEventRecord(TRACE_PHYTIMER, 0xCCCCCCCC, 0);

	if (ulTotalSlots == 0) {
		PRINTF("No pattern configured!\r\n");
		return;
	}

	/* determine a particular consecutive S-slot configuration */
	bIsAllSPattern = 1;
	for (uint8_t i = 0; i < ulTotalSlots; i++) {
		bIsAllSPattern = bIsAllSPattern & (slots[i].is_dl & slots[i].is_ul);
		PRINTF("[%d] bIsAllSPattern = %d\r\n", i, bIsAllSPattern);
	}

	PRINTF("bIsAllSPattern = %d\r\n", bIsAllSPattern);
	vPrintTddPattern();

	PRINTF("debug>>bKeepTickAlive=%d, bTddStop=%d, bTickConfigured=%d\r\n", bKeepTickAlive, bTddStop, bTickConfigured);

	/* do not reset counters or sfn/slot if tick is kept alive */
	if (bKeepTickAlive && !bTddStop && bTickConfigured) {
		PRINTF("debug>>Tick previously configured!\r\n");
		/* update tick time offset */
		tti_trigger(ulNextTick, (scs == SCS_kHz60) ? (slot_duration[scs - 1][0] / 2) : slot_duration[scs][0]);
		goto config_end;
	}
	else PRINTF("debug>> moving fwd wth tdd config\r\n");

	/* send VSPA config */
	prvVSPAConfig(0/*tdd*/, NO_TIMEOUT);

	/* reset all counters for easy debugging */
	ulTotalTicks = 0;
	ulVspaMsgCnt = 0;

	/* next incremnet will turn these into 0 */
	ulCurrentSfn = 1024;
	ulCurrentSlotInFrame = max_slots_per_sfn[scs];

	/* Initialize UL Time Advance */
#ifdef DFE_TICK_DEBUG
	iUplinkTimeAdvance = 0;
#else
	iUplinkTimeAdvance = iNtaOffset + iNtaMax;
#endif /* DFE_TICK_DEBUG */

	/* configure tick interval and setup PhyTimer */
	vPhyTimerTickConfig();

	return;

config_end:
	bTddResume = pdTRUE;
	return;
}

void vConfigFddStart(uint32_t param)
{
	struct dfe_mbox mbox_h2v = {0};

	prvVSPAConfig(1/*fdd*/, NO_TIMEOUT);

	MBOX_SET_OPCODE(mbox_h2v, MBOX_OPC_FDD);
	MBOX_SET_PARAM2(mbox_h2v, (param - uGetPhyTimerTimestamp())*491/61);
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
	iUplinkTimeAdvance = iVal;
}

int iTddStart(void)
{
	PRINTF("bKeepTickAlive = %d\r\n", bKeepTickAlive);

	/* check if TDD is running */
	if (!bTddStop && ulTotalTicks) {
		PRINTF("TDD operation already running!\r\n");
		return DFE_OPERATION_RUNNING;
	}

	bTddStop = pdFALSE;
	prvConfigTdd();
	return 0;
}

void vTddStop(void)
{
	bTddStop = pdTRUE;
	vTaskDelay(1000);
	PRINTF("ulLastPpsInTimestamp = %#x\r\n", ulLastPpsInTimestamp);
	PRINTF("ulTotalTicks = %d\r\n", ulTotalTicks);
	PRINTF("ulVspaMsgCnt = %d\r\n", ulVspaMsgCnt);
	PRINTF("tdd_start1_ts = %#x\r\n", tdd_start1_ts);
	PRINTF("tdd_start2_ts = %#x\r\n", tdd_start2_ts);
	PRINTF("tdd_start3_ts = %#x\r\n", tdd_start3_ts);
	PRINTF("tdd_start4_ts = %#x\r\n", tdd_start4_ts);
}

void vFddStartStop(uint32_t is_on)
{
	uint32_t comparator_value;

	is_on = !!is_on;

	PRINTF("Set TxAllowed(CH%d), RxAllowed(CH%d)", uTxAntennaComparator - 6, uRxAntennaComparator - 0);
	if (isConfigured(uRxAntennaComparator2))
		PRINTF(", RxAllowed(CH%d)", uRxAntennaComparator2 - 0);
	PRINTF(" to %d", is_on);

	NVIC_SetPriority( IRQ_PPS_OUT, 1 );
	NVIC_EnableIRQ( IRQ_PPS_OUT );

	if (is_on) {
		comparator_value = ePhyTimerComparatorOut1;
		timestamp_to_start = uGetPhyTimerTimestamp() + PHYTIMER_10MS_FRAME;
		/* tell VSPA to to FDD start and also the aprox number of VSPA clocks when Tx Allowed will be turned on */
		vPhyTimerWaitComparator(timestamp_to_start - slot_duration[scs][0]); /* send the message closer to the tx_allowed on event */
		vConfigFddStart(timestamp_to_start);
		ulNextTick = timestamp_to_start;
	} else {
		comparator_value = ePhyTimerComparatorOut0;
		vConfigFddStop();
		timestamp_to_start = ulNextTick + PHYTIMER_10MS_FRAME;
	}

	vPhyTimerComparatorConfig( uTxAntennaComparator,
									PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
									comparator_value,
									timestamp_to_start );

	vPhyTimerComparatorConfig( uRxAntennaComparator,
									PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
									comparator_value,
									timestamp_to_start );

	if (isConfigured(uRxAntennaComparator2))
		vPhyTimerComparatorConfig( uRxAntennaComparator2,
										PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
										comparator_value,
										timestamp_to_start );
	if (is_on)
		vPhyTimerComparatorConfig( PHY_TIMER_COMP_PPS_OUT,
			PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
			ePhyTimerComparatorOutToggle,
			timestamp_to_start );
	else
		vPhyTimerComparatorDisable( PHY_TIMER_COMP_PPS_OUT );

	bFddIsRunning = is_on;
}

void vFddStartStopAllPaths(uint32_t is_on, uint32_t delta)
{
	uint32_t comparator_value;

	is_on = !!is_on;

	PRINTF("vFddStartStopAllPaths: on=%d, delta=%d\r\n", is_on, delta);

	NVIC_SetPriority( IRQ_PPS_OUT, 1 );
	NVIC_EnableIRQ( IRQ_PPS_OUT );

	if (is_on) {
		comparator_value = ePhyTimerComparatorOut1;
		timestamp_to_start = uGetPhyTimerTimestamp() + PHYTIMER_10MS_FRAME;
		/* tell VSPA to to FDD start and also the aprox number of VSPA clocks when Tx Allowed will be turned on */
		vPhyTimerWaitComparator(timestamp_to_start - slot_duration[scs][0]); /* send the message closer to the tx_allowed on event */
		vConfigFddStart(timestamp_to_start);
		ulNextTick = timestamp_to_start;
	} else {
		comparator_value = ePhyTimerComparatorOut0;
		vConfigFddStop();
		timestamp_to_start = ulNextTick + PHYTIMER_10MS_FRAME;
	}

	vPhyTimerComparatorConfig( uTxAntennaComparator,
									PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
									comparator_value,
									timestamp_to_start );

	for (uint32_t i = 0; i < 4; i++)
		vPhyTimerComparatorConfig( PHY_TIMER_COMP_CH1_RX_ALLOWED + i,
									PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
									comparator_value,
									timestamp_to_start + delta * (i+1));

	if (is_on)
		vPhyTimerComparatorConfig( PHY_TIMER_COMP_PPS_OUT,
			PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
			ePhyTimerComparatorOutToggle,
			timestamp_to_start );
	else
		vPhyTimerComparatorDisable( PHY_TIMER_COMP_PPS_OUT );

	bFddIsRunning = is_on;
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
		ret = vDFEMbxReceive(&mbox_v2h, DFE_OPS_MBOX_ID, VSPA_IMM_RESPONSE);
		if (ret != 0) {
			PRINTF("No response from VSPA for opcode %d\r\n", MBOX_GET_OPCODE(mbox_h2v));
			goto vspaError;
		}

		opcode = MBOX_GET_OPCODE(mbox_v2h);
		vTaskDelay(100);
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
		vConfigFddStart(0);

	vPhyTimerDelay( 2 * ofdm_short_sym_time[SCS_kHz30] );
	for (int i = 0; i < 4; i ++)
		vPhyTimerComparatorForce(PHY_TIMER_COMP_CH1_RX_ALLOWED + i, ePhyTimerComparatorOut1);
	vPhyTimerComparatorForce(uTxAntennaComparator, ePhyTimerComparatorOut1);

	/* keep FDD running for few symbols */
	vPhyTimerDelay( 2 * ofdm_short_sym_time[SCS_kHz30] );

	if (!bVspaProductionBinary)
		vConfigFddStop();

	/* keep FDD running for ~2 symbols */
	vPhyTimerDelay( 2 * ofdm_short_sym_time[SCS_kHz30] );
	vPhyTimerComparatorForce(uTxAntennaComparator, ePhyTimerComparatorOut0);
	for (int i = 0; i < 4; i ++)
		vPhyTimerComparatorForce(PHY_TIMER_COMP_CH1_RX_ALLOWED + i, ePhyTimerComparatorOut0);

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
	
#if 0 /* debug purpouses */
	log_err("Sending message to host: type = %d, status = %d\r\n",
			msg->type, msg->status);

	hexdump((uint8_t *)msg, DFE_MSG_SIZE);
#endif
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
	int ret = DFE_NO_ERROR;

	switch (msg->type) {
	case DFE_IPC_RESET:
		ipc_host_connected = 0;
		prvDoIPCReset();
		break;
	case DFE_IPC_HOST_CONNECT:
		ipc_host_connected = 1;
		break;
	case DFE_IPC_HOST_DISCONNECT:
		ipc_host_connected = 0;
		break;
	case DFE_TDD_START:
		PRINTF("ipc_host_connected = %d\r\n", ipc_host_connected);
		ret = iTddStart();
		break;
	case DFE_TDD_STOP:
		vTddStop();
		break;
	case DFE_TDD_SWITCH_TX:
		switch_rf(0xAAAAAAAA);
		break;
	case DFE_TDD_SWITCH_RX:
		switch_rf(0xBBBBBBBB);
		break;
	case DFE_FDD_START:
		vFddStartStop(1);
		break;
	case DFE_FDD_STOP:
		vFddStartStop(0);
		break;
	case DFE_FDD_ALL_PATHS_START:
		vFddStartStopAllPaths(1, msg->payload[0]);
		break;
	case DFE_FDD_ALL_PATHS_STOP:
		vFddStartStopAllPaths(0, msg->payload[0]);
		break;
	case DFE_CFG_SCS:
		if (!bTddStop && ulTotalTicks) {
			ret = DFE_OPERATION_RUNNING;
			break;
		}

		if (msg->payload[0] == 15)
		 	scs = SCS_kHz15;
		else if (msg->payload[0] == 30)
		 	scs = SCS_kHz30;
		else if (msg->payload[0] == 60) 
			scs = SCS_kHz60;
		else
		 	scs = SCS_kHz30; // 30Khz is default
		break;
	case DFE_TDD_CFG_PATTERN_NEW:
		if (!bTddStop && ulTotalTicks) {
			ret = DFE_OPERATION_RUNNING;
			break;
		}

		if (msg->payload[0] >= MAX_SLOTS) {
			ret = DFE_INVALID_PARAM;
			break;
		}

		uint8_t slot_idx = msg->payload[0];

		slots[slot_idx].is_dl = msg->payload[1];
		slots[slot_idx].is_ul = msg->payload[2];
		slots[slot_idx].start_symbol_dl = msg->payload[3];
		slots[slot_idx].end_symbol_dl = msg->payload[4];
		slots[slot_idx].start_symbol_ul = msg->payload[5];
		slots[slot_idx].end_symbol_ul = msg->payload[6];
#if 0
		PRINTF("slot[%d]: is_dl=%d, is_ul=%d, start_dl=%d, end_dl=%d, start_ul=%d, end_ul=%d\r\n",
				slot_idx,
				slots[slot_idx].is_dl, slots[slot_idx].is_ul,
				slots[slot_idx].start_symbol_dl, slots[slot_idx].end_symbol_dl,
				slots[slot_idx].start_symbol_ul, slots[slot_idx].end_symbol_ul
		);
#endif

		/* slot configs must arrive in order! */
		ulTotalSlots = slot_idx + 1;
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
		/* shift the mask to left, thus matching DCFG_DCSR_DBGGENCR1 reg mapping */
		if (msg->payload[0] == 0) {
			/* special case where all paths will be enabled; keep backward compatibility with old logic */
			vAxiqLoopbackSet(1, 0xF << 1);
		} else {
			vAxiqLoopbackSet(1, msg->payload[0] << 1);
		}
		break;
	case DFE_CFG_AXIQ_LB_DISABLE:
		vAxiqLoopbackSet(0, 0);
		break;
	case DFE_DEBUG_CMD:
		switch (msg->payload[0]) {
			case 1:
				dump_slots();
				vTraceEventShow();
				break;
			case 2:
				vStatsUsageCommand();
				break;
			default:
				vVSPADebugBreakPoint();
			break;
		}
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
	case DFE_VSPA_PROD_HOST_BYPASS:
		host_bypass_flag_tx_rx = !!msg->payload[0];
		break;

	case DFE_CELL_SEARCH_START:
		//vCellSearchStart(msg->payload[0]);
		break;
	case DFE_CELL_SEARCH_STOP:
		//vCellSearchStop();
		break;
	case DFE_CELL_ATTACH:
		break;
	case DFE_TDD_TICK_KEEPALIVE:
		bKeepTickAlive = !!msg->payload[0];
		break;
	case DFE_TDD_UL_TIME_ADVANCE:
		vSetTimeAdvance(msg->payload[0]);
		break;
	case DFE_TDD_TIME_OFFSET_CORR:
		vTddApplyTimeOffsetCorrection(msg->payload[0]);
		break;
	case DFE_TDD_SFN_SLOT_SET:
		vTddApplySfnSlotUpdate(0, msg->payload[0], msg->payload[1]);
		break;
	case DFE_TDD_SFN_SLOT_DELTA:
		vTddApplySfnSlotUpdate(1, msg->payload[0], msg->payload[1]);
		break;
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
	struct la9310_hif * pxHif = pLa9310Info->pHif;
	int ret = 0;

	/* set ver field in HIF */
	pxHif->ver = CM4_DFE_SW_VER;

	/* stats init */
	ulCurrentSlot = 0;
	ulCurrentSlotInFrame = 0;
	ulTotalSlots = 0;
	ulTotalTicks = 0;
	ulVspaMsgCnt = 0;

	/* keep slots[] in heap */
	slots = pvPortMalloc(MAX_SLOTS * sizeof(tSlot));
	memset(slots, 0, MAX_SLOTS * sizeof(tSlot));

	memset(app_logging, 0, sizeof(app_logging));
	bTddStop = pdFALSE;
	bVspaProductionBinary = !!((iLa9310AviVspaSwVer() & VSPA_SW_VER_PRODUCTION) == VSPA_SW_VER_PRODUCTION);

	prvVspaWarmUp();

	/* create a task for host interface */
	ret = xTaskCreate(prvRxLoop, "DFE Host Polling Loop", configMINIMAL_STACK_SIZE * 4,
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

#if 0
	/* create task for Time Agent */
	ret = xTaskCreate(prvTimeAgentTask, "PhyTimer Agent", configMINIMAL_STACK_SIZE * 4,
			  NULL, tskIDLE_PRIORITY + 2, NULL );
	if(ret != pdPASS)
	{
		PRINTF("Failed to create Time Agent task\r\n");
		return -1;
	}
#endif
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
