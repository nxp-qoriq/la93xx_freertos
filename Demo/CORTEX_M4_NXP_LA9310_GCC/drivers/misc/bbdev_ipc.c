/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2021, 2024 NXP
 */

#include <string.h>
#include "FreeRTOS.h"
#include "config.h"
#include "task.h"
#include "queue.h"
#include "debug_console.h"
#include "la9310_irq.h"
#include "io.h"
#include "bbdev_ipc.h"
#include "la93xx_bbdev_ipc.h"

/* IPC area start on TCML */
#define IPC_CTRL_AREA_BASE_ADDR		(TCML_PHY_ADDR + LA9310_EP_IPC_OFFSET)

#define MAX_BD_ENTRY_DATA_SIZE		(64)
#define BUFFER_ENTRY_SIZE		(64)	/* Required to store buf_entry_t */
#define TOTAL_ENTRY_SIZE		(BUFFER_ENTRY_SIZE + MAX_BD_ENTRY_DATA_SIZE + \
					 IPC_MAX_INTERNAL_BUFFER_SIZE)
#define QUEUE_MEMPOOL_SIZE		(TOTAL_ENTRY_SIZE * IPC_MAX_DEPTH + 128)
#define MODEM_IPC_APP_MEMPOOL_SIZE	(QUEUE_MEMPOOL_SIZE * IPC_MAX_CHANNEL_COUNT)

#define UNUSED(x)	(void)(x)

#define IPC_DEBUG 0

#if IPC_DEBUG
#define ipc_log_debug log_info
#else
#define ipc_log_debug(...)
#endif

typedef uint32_t phys_addr_t;

struct md_priv {
	uint32_t pi[IPC_MAX_CHANNEL_COUNT];
	uint32_t ci[IPC_MAX_CHANNEL_COUNT];
	struct bbdev_ipc_raw_op_t *op[IPC_MAX_DEPTH];
	void *internal_bufs[IPC_MAX_CHANNEL_COUNT][IPC_MAX_DEPTH];
};

struct md_priv md_priv_queue;

typedef void * ipc_t;

struct dev_attr_t dev_attr;

uint32_t ipc_mempool_area;

/** IPC memory pool */
typedef struct ipc_mem_pool {
	uint64_t mod_phys;      /**< modem address of the buffer */
	uint64_t modem_cptr;    /**< Points to the start of the freespace in mem pool */
	uint32_t size;  /**< size of the memory pool */
} __attribute__((packed)) ipc_mem_pool_t;

ipc_mem_pool_t ipc_mem_pool[IPC_MAX_CHANNEL_COUNT];

ipc_t ipc_handle[IPC_MAX_INSTANCE_COUNT];

struct buf_entry_t {
	uint32_t addr;
	struct buf_entry_t *next;
	uint16_t queue_id;
	/* To align to 64 */
	uint8_t rsvd[54];
};

struct buf_pool_t {
	uint32_t num_bufs;
	struct buf_entry_t *buf_entry;
};

struct buf_pool_t buf_pool[IPC_MAX_INSTANCE_COUNT][IPC_MAX_CHANNEL_COUNT];

static inline struct buf_entry_t *
get_buf(uint8_t dev_id, uint16_t queue_id)
{
	struct buf_pool_t *q_buf_pool = &buf_pool[dev_id][queue_id];
	struct buf_entry_t *buf_entry = q_buf_pool->buf_entry;

	if (q_buf_pool->num_bufs == 0)
		return NULL;

	q_buf_pool->buf_entry = buf_entry->next;
	q_buf_pool->num_bufs--;

	return buf_entry;
}

static inline void
put_buf(uint8_t dev_id, struct buf_entry_t *buf_entry)
{
	uint16_t queue_id = buf_entry->queue_id;
	struct buf_pool_t *q_buf_pool = &buf_pool[dev_id][queue_id];

	if (q_buf_pool->num_bufs != 0)
		buf_entry->next = q_buf_pool->buf_entry;

	q_buf_pool->buf_entry = buf_entry;
	q_buf_pool->num_bufs++;
}

int bbdev_ipc_is_host_initialized(uint8_t dev_id)
{
	UNUSED(dev_id);

	/* TODO: Try reducing to one check for LIB */
	/* Wait for Host LIB and APP ready */
	if (!CHK_HIF_HOST_RDY(pLa9310Info->pHif, LA9310_HIF_STATUS_IPC_LIB_READY)) {
		ipc_log_debug("%s: HOST LIB not ready.",__func__);
		return 0;
	}
	if(!CHK_HIF_HOST_RDY(pLa9310Info->pHif, LA9310_HIF_STATUS_IPC_APP_READY)) {
		ipc_log_debug("%s: HOST APP not ready.",__func__);
		return 0;
	}

	return 1;
}

void bbdev_ipc_signal_ready(uint8_t dev_id)
{
	UNUSED(dev_id);

	SET_HIF_MOD_RDY(pLa9310Info->pHif, LA9310_HIF_MOD_READY_IPC_APP);
}

struct dev_attr_t *
bbdev_ipc_get_dev_attr(uint8_t dev_id)
{
	ipc_instance_t *ipc_instance = ipc_handle[dev_id];
	ipc_ch_t *ch;
	int i;

	memset(&dev_attr, 0, sizeof(dev_attr));

	for (i = 0; i < IPC_MAX_CHANNEL_COUNT; i++) {
		ch = &(ipc_instance->ch_list[i]);
		if (ch->op_type) {
			/* Configured queues are in sequence.
			 * So it is safe to assume i = num_queues at
			 * when ch->op_type is valid.
			 */
			dev_attr.num_queues++;
			dev_attr.qattr[i].queue_id = i;
			dev_attr.qattr[i].op_type = ch->op_type;
			dev_attr.qattr[i].depth = ch->depth;
			dev_attr.qattr[i].conf_enable = ch->conf_enable;
		}
	}

	return &dev_attr;
}

static phys_addr_t
bbdev_ipc_malloc(ipc_mem_pool_t* pool, uint32_t size, uint32_t align, int *err)
{
	uint32_t waste = 0;
	uint32_t align_off = 0;

	/* Calculate hole due to alignment requirement */
	if (align) {
		align_off = (uint32_t)(pool->modem_cptr % align);
		waste = align - align_off;
	}

	/* Is enough memory available in mempool? */
	if ((pool->modem_cptr + waste + size) >=
	    (pool->mod_phys + pool->size)) {
		*err = IPC_MEM_INVALID;
		log_err("mempool don't have enough memory\r\n");
		return 0;
	}

	/* Update memory pointers in mempool after allocation */
	pool->modem_cptr += waste;
	phys_addr_t addr = (phys_addr_t) pool->modem_cptr;
	pool->modem_cptr += size;

	/* Zero out allocated memory */
	memset((void *)addr, 0, size);

	/* Fill errocode */
	*err = IPC_SUCCESS;

	return addr;
}

static inline int
is_bd_ring_full(uint32_t ci, uint32_t pi, uint32_t ring_size)
{
	if (((pi + 1) % ring_size) == ci)
		return 1; /* Ring is Full */

	return 0;
}

static inline int
is_bd_ring_empty(uint32_t ci, uint32_t pi)
{
	if (ci == pi)
		return 1; /* No more Buffer */
	return 0;
}

void *
bbdev_ipc_get_next_internal_buf(uint16_t dev_id, uint16_t queue_id,
		uint32_t *length)
{
	ipc_instance_t *ipc_instance = ipc_handle[dev_id];
	ipc_ch_t *ch = &(ipc_instance->ch_list[queue_id]);
	ipc_br_md_t *md = &(ch->md);
	uint32_t pi, ci;

	if (ch->conf_enable)
		ci = md_priv_queue.ci[queue_id];
	else
		ci = md->ci;
	pi = md_priv_queue.pi[queue_id];

	if (is_bd_ring_full(ci, pi, md->ring_size))
		return NULL;

	*length = IPC_MAX_INTERNAL_BUFFER_SIZE;

	return md_priv_queue.internal_bufs[queue_id][pi];
}
int bbdev_ipc_enqueue_raw_op(uint16_t dev_id, uint16_t queue_id,
				  struct bbdev_ipc_raw_op_t *op)
{
	ipc_instance_t *ipc_instance = ipc_handle[dev_id];
	ipc_ch_t *ch = &(ipc_instance->ch_list[queue_id]);
	ipc_br_md_t *md = &(ch->md);
	host_ipc_params_t *hp = (host_ipc_params_t *)ch->host_ipc_params;
	struct bbdev_ipc_raw_op_t *host_mem;
	uint32_t pi, ci, ring_size;
	ipc_bd_t *bdr;
	ipc_bd_t *bd;

	/**
	 * In case of confirmation mode, local consumer index is incremented
	 * after receiving the output data(dequeue). Hence, before enqueuing the
	 * raw operation, we need to compare this local consumer index and
	 * producer index to check if bd ring is full.
	 * But, in case of non confirmation mode, since we will not receive the
	 * output data(dequeue function will not be called), local consumer
	 * index will not be updated. Hence, to check if bd ring is full, we
	 * will rely on the shared consumer index, which will be incrememnted by
	 * other side after consuming the packet.
	 */
	if (ch->conf_enable)
		ci = md_priv_queue.ci[queue_id];
	else
		ci = md->ci;
	pi = md_priv_queue.pi[queue_id];
	ring_size = md->ring_size;

	if (is_bd_ring_full(ci, pi, ring_size))
		return IPC_CH_FULL;

	ipc_log_debug("%s enter: pi: %d, ring size: %d\r\n", __func__,
		 pi, md->ring_size);

	bdr = ch->bd_h;
	bd = &bdr[pi];

	host_mem = (struct bbdev_ipc_raw_op_t *)(bd->modem_ptr);
	if (op->out_addr)
		host_mem->out_addr = op->out_addr - TCML_PHY_ADDR;
	else
		host_mem->out_addr = 0;
	host_mem->out_len = op->out_len;
	host_mem->in_addr = op->in_addr - TCML_PHY_ADDR;
	host_mem->in_len = op->in_len;

	md_priv_queue.op[pi] = op;

	/* Move Producer Index forward */
	pi++;
	/* Reset PI, if wrapping */
	if (pi == ring_size)
		pi = 0;
	md_priv_queue.pi[queue_id] = pi;

	/* Wait for all updates and then update PI */
	dmb();
	hp->pi = pi;

	ipc_log_debug("%s exit: pi: %d, ring size: %d\r\n", __func__,
		 pi, md->ring_size);

	return IPC_SUCCESS;
}

struct bbdev_ipc_raw_op_t *bbdev_ipc_dequeue_raw_op(uint16_t dev_id,
		uint16_t queue_id)
{
	ipc_instance_t *ipc_instance = ipc_handle[dev_id];
	ipc_ch_t *ch = &(ipc_instance->ch_list[queue_id]);
	ipc_br_md_t *md = &(ch->md);
	uint32_t ci, pi, temp_ci;
	struct bbdev_ipc_raw_op_t *op;
	uint32_t md_priv_op_index;

	if (ch->is_host_to_modem) {
		ci = md_priv_queue.ci[queue_id];
		pi = md->pi;

		if (is_bd_ring_empty(ci, pi))
			return NULL;

		ipc_log_debug("%s enter: pi: %d, ci: %d, ring size: %d\r\n",
			__func__, pi, ci, md->ring_size);

		ipc_bd_t *bdr = ch->bd_m;
		ipc_bd_t *bd = &bdr[ci];
		op = (struct bbdev_ipc_raw_op_t *)
			(bd->modem_ptr + TCML_PHY_ADDR);

	} else {
		temp_ci = md->ci;
		ci = md_priv_queue.ci[queue_id];
		if (temp_ci == ci)
			return NULL;
		pi = md_priv_queue.pi[queue_id];

		ipc_log_debug("%s enter: pi: %d, ci: %d, ring size: %d\r\n",
			 __func__, pi, ci, md->ring_size);

		ipc_bd_t *bdr = ch->bd_m;
		ipc_bd_t *bd = &bdr[ci];

		op = (struct bbdev_ipc_raw_op_t *)
			(bd->modem_ptr + TCML_PHY_ADDR);
		md_priv_queue.op[ci]->status = op->status;
		md_priv_queue.op[ci]->out_len = op->out_len;
		md_priv_op_index = ci;

		/* Move Consumer Index forward */
		ci++;
		/* Reset CI, if wrapping */
		if (ci == md->ring_size)
			ci = 0;
		md_priv_queue.ci[queue_id] = ci;

		ipc_log_debug("%s exit: pi: %d, ci: %d, ring size: %d\r\n",
			 __func__, pi, ci, md->ring_size);

		op = md_priv_queue.op[md_priv_op_index];
	}

	return op;
}

uint16_t bbdev_ipc_consume_raw_op(uint16_t dev_id, uint16_t queue_id,
				  struct bbdev_ipc_raw_op_t *op)
{
	ipc_instance_t *ipc_instance = ipc_handle[dev_id];
	ipc_ch_t *ch = &(ipc_instance->ch_list[queue_id]);
	ipc_br_md_t *md = &(ch->md);
	host_ipc_params_t *hp = (host_ipc_params_t *)ch->host_ipc_params;
	struct bbdev_ipc_raw_op_t *host_mem;
	ipc_bd_t *bdr;
	ipc_bd_t *bd;
	uint32_t ci;

	ci = md_priv_queue.ci[queue_id];

	ipc_log_debug("%s enter: ci: %d, ring size: %d\r\n",
		__func__, ci, md->ring_size);

	bdr = ch->bd_h;
	bd = &bdr[ci];

	host_mem = (struct bbdev_ipc_raw_op_t *)(bd->modem_ptr);
	host_mem->status = op->status;
	host_mem->out_len = op->out_len;

	/* Move Consumer Index forward */
	ci++;
	/* Reset CI, if wrapping */
	if (ci == md->ring_size)
		ci = 0;
	md_priv_queue.ci[queue_id] = ci;

	/* Wait for all updates and then update PI */
	dmb();
	hp->ci = ci;

	ipc_log_debug("%s exit: ci: %d, ring size: %d\r\n",
		__func__, ci, md->ring_size);

	return 0;
}

int
bbdev_ipc_queue_configure(uint8_t dev_id, uint16_t queue_id)
{
	ipc_instance_t *ipc_instance = ipc_handle[dev_id];
	uint32_t depth, i;
	struct buf_entry_t *buf_entry;
	host_ipc_params_t *hp;
	ipc_ch_t *ch;
	phys_addr_t mem;
	int code;

	while (!(in_le32(&(ipc_instance->initialized)))) { }

	ch = &(ipc_instance->ch_list[queue_id]);
	hp = (host_ipc_params_t *)ch->host_ipc_params;

	ch->ch_id = queue_id;
	depth = ch->depth;

	if (depth > IPC_MAX_DEPTH) {
		log_err("depth: %d is more than IPC_MAX_DEPTH: %d\n",
			depth, IPC_MAX_DEPTH);
		return IPC_INPUT_INVALID;
	}

	ch->md.ring_size = depth;
	ch->md.pi = 0;
	ch->md.ci = 0;

	ch->md.msg_size = MAX_BD_ENTRY_DATA_SIZE;

	ipc_log_debug("%s: queue: %d, depth: %d, msg size: %d\r\n",
		 __func__, queue_id, depth, MAX_BD_ENTRY_DATA_SIZE);

	/* We have 64 bytes of buf_entry, followed by remaining memory for
	 * the BD and internal memory.
	 */
	mem = bbdev_ipc_malloc(&(ipc_mem_pool[queue_id]),
			 depth * TOTAL_ENTRY_SIZE, 0, &code);
	if (!mem)
		return code;

	for (i = 0; i < depth; i++) {
		buf_entry = (struct buf_entry_t *)(mem);
		mem += sizeof(struct buf_entry_t);
		buf_entry->addr = mem;
		buf_entry->queue_id = queue_id;

		put_buf(dev_id, buf_entry);
		mem += MAX_BD_ENTRY_DATA_SIZE;
		md_priv_queue.internal_bufs[queue_id][i] =
			(void *)mem;
		mem += IPC_MAX_INTERNAL_BUFFER_SIZE;
	}

	for (i = 0; i < depth; i++) {
		buf_entry = get_buf(dev_id, queue_id);
		ch->bd_m[i].modem_ptr = buf_entry->addr -
			TCML_PHY_ADDR;
		hp->bd_m_modem_ptr[i] = buf_entry->addr -
			TCML_PHY_ADDR;
	}

	return IPC_SUCCESS;
}

int
bbdev_ipc_init(uint8_t dev_id, uint8_t core_id)
{
	ipc_metadata_t * ipc_md;
	ipc_instance_t *instance;
	int i;

	ipc_md = ( ipc_metadata_t * ) IPC_CTRL_AREA_BASE_ADDR;
	memset((void *)ipc_md, 0x0, sizeof(ipc_metadata_t));
	memset(&md_priv_queue, 0, sizeof(md_priv_queue));

	ipc_md->ipc_modem_signature = IPC_MODEM_SIGNATURE;
	ipc_handle[dev_id] = (ipc_t)(&ipc_md->instance_list[dev_id]);

	/* Update IPC md offset and size in HIF */
	pLa9310Info->pHif->ipc_regs.ipc_mdata_size =
			(uint32_t)sizeof(ipc_metadata_t);
	pLa9310Info->pHif->ipc_regs.ipc_mdata_offset = LA9310_EP_IPC_OFFSET;

	/* 64 byte aligned mempool area */
	ipc_mempool_area = ((int)(((uint32_t)ipc_md +
		sizeof(ipc_metadata_t) + 64) / 64) * 64);

	instance = &(ipc_md->instance_list[dev_id]);
	for (i = 0; i < IPC_MAX_CHANNEL_COUNT; i++) {
		/* Create mem pools.*/
		/* Get contiguous block for IPC Rx buffers. */
		ipc_mem_pool[i].mod_phys = (uint32_t)ipc_mempool_area +
				(i * QUEUE_MEMPOOL_SIZE);
		ipc_mem_pool[i].modem_cptr = ipc_mem_pool[i].mod_phys;
		ipc_mem_pool[i].size = QUEUE_MEMPOOL_SIZE;
	}

	/* Initialize the IPC channel id in metadata */
	for(i = 0; i < IPC_MAX_CHANNEL_COUNT; i++)
		instance->ch_list[i].ch_id = i;

	instance->instance_id = dev_id;
	instance->initialized = i;

	SET_HIF_MOD_RDY( pLa9310Info->pHif, LA9310_HIF_MOD_READY_IPC_LIB );
	log_info( "IPC on modem ready\n\r" );

	return 0;
}

void
bbdev_ipc_close(uint8_t dev_id, uint8_t core_id)
{
	CLEAR_HIF_MOD_RDY( pLa9310Info->pHif );
}
