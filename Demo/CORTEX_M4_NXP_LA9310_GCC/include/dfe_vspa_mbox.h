/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#ifndef __DFE_VSPA_MBOX_H__
#define __DFE_VSPA_MBOX_H__

/* Mailboxes */
#define DFE_OPS_MBOX_ID			1

/* Mailbox interface opcodes (DFE & debug operations) */
#define DFE_OP_TEST_MBOX		0

typedef enum {
	BENCHMARK_MODE_NONE = 0,
	BENCHMARK_MODE_RD = 1,    
	BENCHMARK_MODE_WR = 2,
	BENCHMARK_MODE_RW = 3,
	BENCHMARK_CNT
} benchmark_mode_e;

typedef enum {
	MBOX_OPC_STOP = 0,
	MBOX_OPC_SEMISTATIC = 0xF1,
	MBOX_OPC_FDD = 2,
	MBOX_OPC_TDD = 3,
	MBOX_OPC_DDR_CFG_4KB = 0xF5,
	MBOX_OPC_DDR_CFG_128B = 0xF6,
	MBOX_OPC_DEBUG_BP = 0xF7,
	MBOX_OPC_BENCHMARK_PCI = 0xF8,
	MBOX_OPC_QEC = 0xF9,
	/* production VSPA */
	MBOX_OPC_PROD_SEMISTATIC_0E = 0x0E,
	MBOX_OPC_PROD_DDR_CFG_4KB = 0x10,
	MBOX_OPC_PROD_DDR_CFG_128B = 0x20,
	MBOX_OPC_CNT,
} mbox_opc_e;

struct dfe_mbox {
	/* MSB [opcode | param 1] */
	union {
		unsigned int msb;
		struct dfe_mbox_ctrl {
			unsigned int param1:24;
			unsigned int opcode:8;
		} ctrl;
		struct dfe_mbox_cmd {
			unsigned int unused:16;
			unsigned int num_sym:4;
			unsigned int start_sym:4;
			unsigned int opcode:8;
		} cmd;
	};
	/* LSB [ param2   ] */
	/* LSB [ addr ] */
	/* LSB [ ack | err_code ] */
	union {
		unsigned int lsb;
		unsigned int param2;
		unsigned int addr;
		struct dfe_mbox_status {
			unsigned int ack:1;
			unsigned int err_code:8;
			unsigned int unused:23;
		} status;
	};
}__attribute__((packed));

/* Accessors for mailbox fields */
#define MBOX_MASK(size)					\
	((~(unsigned int)0) >> (32 - (size)))
#define MBOX_SET_FIELD(word, val, size, shift)		\
	(word |= ((val) & MBOX_MASK(size)) << shift)
#define MBOX_GET_FIELD(word, size, shift)		\
	((word >> shift) & MBOX_MASK(size))

#define MBOX_CLEAR(mbox)                        {(mbox).msb = 0; (mbox).lsb = 0;}

#define MBOX_SET_OPCODE(mbox, opcode)           MBOX_SET_FIELD((mbox).msb, (opcode), 8, 24)
#define MBOX_GET_OPCODE(mbox)                   MBOX_GET_FIELD((mbox).msb, 8, 24)

#define MBOX_SET_TX_SYM_NR(mbox, nr)            MBOX_SET_FIELD((mbox).msb, (nr), 4, 20)
#define MBOX_GET_TX_SYM_NR(mbox)                MBOX_GET_FIELD((mbox).msb, 4, 20)
#define MBOX_SET_TX_ADDR(mbox, addr)            MBOX_SET_FIELD((mbox).msb, (addr), 20, 0)
#define MBOX_GET_TX_ADDR(mbox)                  MBOX_GET_FIELD((mbox).msb, 20, 0)
#define MBOX_SET_SYM_SIZE(mbox, size)           MBOX_SET_FIELD((mbox).lsb, (size), 8, 24)
#define MBOX_GET_SYM_SIZE(mbox)                 MBOX_GET_FIELD((mbox).lsb, 8, 24)
#define MBOX_SET_RX_SYM_NR(mbox, nr)            MBOX_SET_FIELD((mbox).lsb, (nr), 4, 20)
#define MBOX_GET_RX_SYM_NR(mbox)                MBOX_GET_FIELD((mbox).lsb, 4, 20)
#define MBOX_SET_RX_ADDR(mbox, addr)            MBOX_SET_FIELD((mbox).lsb, (addr), 20, 0)
#define MBOX_GET_RX_ADDR(mbox)                  MBOX_GET_FIELD((mbox).lsb, 20, 0)

#define MBOX_SET_SIZE(mbox, size)               MBOX_SET_PARAM1(mbox, (size))
#define MBOX_GET_SIZE(mbox)                     MBOX_GET_PARAM1(mbox)
 
#define MBOX_SET_RX_START_SYM(mbox, sym)        MBOX_SET_FIELD((mbox).msb, (sym), 4, 20)
#define MBOX_GET_RX_START_SYM(mbox)             MBOX_GET_FIELD((mbox).msb, 4, 20)
#define MBOX_SET_RX_NR_SYM(mbox, nr)            MBOX_SET_FIELD((mbox).msb, (nr), 4, 16)
#define MBOX_GET_RX_NR_SYM(mbox)                MBOX_GET_FIELD((mbox).msb, 4, 16)

#define MBOX_SET_TX_START_SYM(mbox, sym)        MBOX_SET_FIELD((mbox).msb, (sym), 4, 12)
#define MBOX_GET_TX_START_SYM(mbox)             MBOX_GET_FIELD((mbox).msb, 4, 12)
#define MBOX_SET_TX_NR_SYM(mbox, nr)            MBOX_SET_FIELD((mbox).msb, (nr), 4, 8)
#define MBOX_GET_TX_NR_SYM(mbox)                MBOX_GET_FIELD((mbox).msb, 4, 8)

#define MBOX_SET_TDD_STOP(mbox, stop)           MBOX_SET_FIELD((mbox).msb, (stop), 1, 7)
#define MBOX_GET_TDD_STOP(mbox)                 MBOX_GET_FIELD((mbox).msb, 1, 7)

#define MBOX_SET_SLOT_IDX(mbox, slot)           MBOX_SET_FIELD((mbox).msb, (slot), 5, 0)
#define MBOX_GET_SLOT_IDX(mbox)                 MBOX_GET_FIELD((mbox).msb, 5, 0)

#define MBOX_SET_PARAM1(mbox, p)                MBOX_SET_FIELD((mbox).msb, (p), 24, 0)
#define MBOX_GET_PARAM1(mbox)                   MBOX_GET_FIELD((mbox).msb, 24, 0)
#define MBOX_GET_PARAM1_SIGNED(mbox)            (((int)MBOX_GET_PARAM1(mbox) << 8) >> 8)

#define MBOX_SET_ACK(mbox, ack)                 MBOX_SET_FIELD((mbox).lsb, (ack), 1, 0)
#define MBOX_GET_ACK(mbox)                      MBOX_GET_FIELD((mbox).lsb, 1, 0)
#define MBOX_SET_ERR_CODE(mbox, err)            MBOX_SET_FIELD((mbox).lsb, (err), 8, 1)
#define MBOX_GET_ERR_CODE(mbox)                 MBOX_GET_FIELD((mbox).lsb, 8, 1)

#define MBOX_SET_PARAM2(mbox, p)                MBOX_SET_FIELD((mbox).lsb, (p), 32, 0)
#define MBOX_GET_PARAM2(mbox)                   MBOX_GET_FIELD((mbox).lsb, 32, 0)

#define MBOX_SET_ADDR(mbox, addr)               MBOX_SET_PARAM2((mbox), (addr))
#define MBOX_GET_ADDR(mbox)                     MBOX_GET_PARAM2((mbox))

#define MBOX_GET_BENCHMARK_MODE(mbox)           MBOX_GET_FIELD((mbox).lsb, 4, 28)
#define MBOX_SET_BENCHMARK_MODE(mbox, mode)     MBOX_SET_FIELD((mbox).lsb, (mode), 4, 28)
#define MBOX_GET_NUM_PARALLEL_DMAS(mbox)        MBOX_GET_FIELD((mbox).lsb, 4, 24)
#define MBOX_SET_NUM_PARALLEL_DMAS(mbox, num)   MBOX_SET_FIELD((mbox).lsb, (num), 4, 24)
#define MBOX_GET_NUM_LOOPS(mbox)                MBOX_GET_FIELD((mbox).lsb, 4, 20)
#define MBOX_SET_NUM_LOOPS(mbox, num)           MBOX_SET_FIELD((mbox).lsb, (num), 4, 20)

#if 0 /* moved in host interface header */
/* QEC-related */
typedef enum {
	MBOX_IQ_CORR_FTAP0 = 0x00,
	MBOX_IQ_CORR_FTAP1,  // 0x01
	MBOX_IQ_CORR_FTAP2,  // 0x02
	MBOX_IQ_CORR_FTAP3,  // 0x03
	MBOX_IQ_CORR_FTAP4,  // 0x04
	MBOX_IQ_CORR_FTAP5,  // 0x05
	MBOX_IQ_CORR_FTAP6,  // 0x06
	MBOX_IQ_CORR_FTAP7,  // 0x07
	MBOX_IQ_CORR_FTAP8,  // 0x08
	MBOX_IQ_CORR_FTAP9,  // 0x09
	MBOX_IQ_CORR_FTAP10, // 0x0A
	MBOX_IQ_CORR_FTAP11, // 0x0B
	MBOX_IQ_CORR_DC_I,   // 0x0C
	MBOX_IQ_CORR_DC_Q,   // 0x0D
	MBOX_IQ_CORR_FDELAY, // 0x0E
	MBOX_IQ_CORR_MAX,    // 0x0F
} mbox_qec_factor_e;
 
typedef enum {
	QEC_IQ_DC_OFFSET_CORR = 0,
	QEC_IQ_DC_OFFSET_PASSTHROUGH = 1,
} qec_corr_mode_e;
 
typedef enum {
	QEC_TX_CORR = 0,
	QEC_RX_CORR = 1,
	QEC_MAX_CORR
} qec_tx_rx_mode_e;
#endif

/* RX/TX QEC + DC offset */
#define MBOX_GET_QEC_CORR_MODE(mbox)            MBOX_GET_FIELD((mbox).msb, 1, 23)
#define MBOX_SET_QEC_CORR_MODE(mbox, mode)      MBOX_SET_FIELD((mbox).msb, mode, 1, 23)
#define MBOX_GET_QEC_TX_RX_MODE(mbox)           MBOX_GET_FIELD((mbox).msb, 1, 22)
#define MBOX_SET_QEC_TX_RX_MODE(mbox, tx_rx)    MBOX_SET_FIELD((mbox).msb, tx_rx, 1, 22)
#define MBOX_GET_QEC_IDX(mbox)                  MBOX_GET_FIELD((mbox).msb, 5, 17)
#define MBOX_SET_QEC_IDX(mbox, idx)             MBOX_SET_FIELD((mbox).msb, idx, 5, 17)
#define MBOX_GET_QEC_VALUE(mbox)                MBOX_GET_PARAM2(mbox)
#define MBOX_SET_QEC_VALUE(mbox, value)         MBOX_SET_PARAM2(mbox, value)
	
/* specific to production VSPA */
/* Semistatic CFG - 0E */
/*MSB*/
#define MBOX_SET_RX_CHAN_RESTART(mbox, cfg)     MBOX_SET_FIELD((mbox).msb, cfg, 1, 21)
#define MBOX_GET_RX_CHAN_RESTART(mbox)          MBOX_GET_FIELD((mbox).msb, 1, 21)
#define MBOX_SET_TX_CHAN_RESTART(mbox, cfg)     MBOX_SET_FIELD((mbox).msb, cfg, 1, 20)
#define MBOX_GET_TX_CHAN_RESTART(mbox)          MBOX_GET_FIELD((mbox).msb, 1, 20)
#define MBOX_SET_RX_TDD_FDD(mbox, cfg)          MBOX_SET_FIELD((mbox).msb, cfg, 1, 13)
#define MBOX_GET_RX_TDD_FDD(mbox)               MBOX_GET_FIELD((mbox).msb, 1, 13)
#define MBOX_SET_TX_TDD_FDD(mbox, cfg)          MBOX_SET_FIELD((mbox).msb, cfg, 1, 12)
#define MBOX_GET_TX_TDD_FDD(mbox)               MBOX_GET_FIELD((mbox).msb, 1, 12)
#define MBOX_SET_CPE_BS_MODE(mbox, mode)        MBOX_SET_FIELD((mbox).msb, mode, 1, 10)
#define MBOX_GET_CPE_BS_MODE(mbox)              MBOX_GET_FIELD((mbox).msb, 1, 10)
#define MBOX_SET_HOST_BYPASS_TX(mbox, cfg)      MBOX_SET_FIELD((mbox).msb, cfg, 1, 14)
#define MBOX_GET_HOST_BYPASS_TX(mbox)           MBOX_GET_FIELD((mbox).msb, 1, 14)
#define MBOX_SET_HOST_BYPASS_RX(mbox, cfg)      MBOX_SET_FIELD((mbox).msb, cfg, 1, 15)
#define MBOX_GET_HOST_BYPASS_RX(mbox)           MBOX_GET_FIELD((mbox).msb, 1, 15)

/*LSB*/
#define MBOX_SET_DCS_MAPPING(mbox, map)         MBOX_SET_FIELD((mbox).lsb, map, 1, 23)
#define MBOX_GET_DCS_MAPPING(mbox)              MBOX_GET_FIELD((mbox).lsb, 1, 23)
#define MBOX_SET_RX_IDX(mbox, idx)              MBOX_SET_FIELD((mbox).lsb, idx, 3, 20)
#define MBOX_GET_RX_IDX(mbox)                   MBOX_GET_FIELD((mbox).lsb, 3, 20)
#define MBOX_SET_TX_IDX(mbox, idx)              MBOX_SET_FIELD((mbox).lsb, idx, 3, 16)
#define MBOX_GET_TX_IDX(mbox)                   MBOX_GET_FIELD((mbox).lsb, 3, 16)
#define MBOX_SET_SCS(mbox, scs)                 MBOX_SET_FIELD((mbox).lsb, scs, 2, 6)
#define MBOX_GET_SCS(mbox)                      MBOX_GET_FIELD((mbox).lsb, 2, 6)
#define MBOX_SET_RX_IQSWAP(mbox, swap)          MBOX_SET_FIELD((mbox).lsb, swap, 1, 5)
#define MBOX_GET_RX_IQSWAP(mbox)                MBOX_GET_FIELD((mbox).lsb, 1, 5)
#define MBOX_SET_TX_IQSWAP(mbox, swap)          MBOX_SET_FIELD((mbox).lsb, swap, 1, 4)
#define MBOX_GET_TX_IQSWAP(mbox)                MBOX_GET_FIELD((mbox).lsb, 1, 4)

#endif /* __DFE_VSPA_MBOX_H__ */
