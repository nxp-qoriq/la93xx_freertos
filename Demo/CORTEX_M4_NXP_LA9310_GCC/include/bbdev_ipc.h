/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2017, 2021-2024 NXP
 */

#ifndef _BBDEV_IPC_H_
#define _BBDEV_IPC_H_

/**
 * @file        bbdev_ipc.h
 * @brief       This file contains the BBDEV IPC related information
 * @addtogroup  BBDEV_IPC_API
 * @{
 */

#include <stdint.h>
#include "la9310_error_codes.h"
#include "la93xx_bbdev_ipc.h"

/** Attributes of a queue */
struct queue_attr_t {
	/** Queue ID */
	uint32_t queue_id;
	/** Type of operation supported on this queue */
	uint32_t op_type;
	/** Queue depth */
	uint32_t depth;
	/** conf mode enabled */
	uint32_t conf_enable;
};

/** Attributes of the device */
struct dev_attr_t {
	/** Total number of Queues configured */
	int num_queues;
	/** Per queue configuration */
	struct queue_attr_t qattr[IPC_MAX_CHANNEL_COUNT];
};

/**
 * Init function to initialize the BBDEV IPC subsystem. This API will
 * also internally wait on the BBDEV to be initialized on the host (DPDK).
 *
 * @param dev_id
 *   The identifier of the device. Currently only one device id is
 *   supported - 'BBDEV_IPC_DEV_ID_0'
 * @param core_id
 *   Core ID from where this API is being called
 *
 * @return
 *   - 0  on success, error code otherwise
 */
int bbdev_ipc_init(uint8_t dev_id, uint8_t core_id);
void bbdev_ipc_close(uint8_t dev_id, uint8_t core_id);

/**
 * Checks if device on host has been initialized yet or not. As FreeRTOS
 * runs on multi-core, this API is used to determine if HOST LIB has
 * been completed or not. 'bbdev_ipc_queue_configure' API should be called
 * before this API.
 *
 * @param dev_id
 *   The identifier of the device. Currently only one device id is
 *   supported - 'BBDEV_IPC_DEV_ID_0'
 *
 * @return
 *   - 1 if device has been initialized
 *   - 0 if device has not been initialized yet
 */
int bbdev_ipc_is_host_initialized(uint8_t dev_id);

/**
 * Get attributes of the device
 *
 * @param dev_id
 *   The identifier of the device. Currently only one device id is
 *   supported - 'BBDEV_IPC_DEV_ID_0'
 *
 * @return
 *   - pointer to device attributes
 */
struct dev_attr_t *
bbdev_ipc_get_dev_attr(uint8_t dev_id);

/**
 * Configures a queue on BBDEV IPC device. This API should be called from
 * where I/O needs to be done.
 *
 * @param dev_id
 *   The identifier of the device. Currently only one device id is
 *   supported - 'BBDEV_IPC_DEV_ID_0'
 * @param queue_id
 *   The index of the queue.
 *
 * @return
 *   - 0 on success, error code otherwise
 */
int bbdev_ipc_queue_configure(uint8_t dev_id, uint16_t queue_id);

/**
 * Signal the Host device that BBDEV is now ready to process packets.
 * This will signal the Host only when all the cores have been initialized.
 *
 * @param dev_id
 *   The identifier of the device. Currently only one device id is
 *   supported - 'BBDEV_IPC_DEV_ID_0'
 *
 */
void bbdev_ipc_signal_ready(uint8_t dev_id);

/** Get the buffer pointer from internal pool.
 *  This API would return buffer associated with next enqueue operation.
 *  Note: Each enqueue operation has only one associated internal buffer.
 *  This memory shall be passed in the 'in_addr' pointer in bbdev_ipc_raw_op_t.
 *
 * @param dev_id
 *   The identifier of the device.
 * @param queue_id
 *   The index of the queue.
 * @param length
 *   returns max length of the memory in case 'length' is specified.
 *
 * @return
 *    Memory address of internal buffer.
 *    NULL - in case memory for next enqueue operation is not free
 *    and still in use.
 */
void *
bbdev_ipc_get_next_internal_buf(uint16_t dev_id, uint16_t queue_id,
		uint32_t *length);

/**
 * Enqueue a RAW operation to a queue of the device.
 *
 * If confirmation is required then the memory for the 'op' structure should
 * be allocated from heap/mempool and should be freed only after confirmation.
 * Otherwise, it shall be on stack or if on heap, should be freed after enqueue
 * operation.
 *
 * @param dev_id
 *   The identifier of the device. Currently only one device id is
 *   supported - 'BBDEV_IPC_DEV_ID_0'
 * @param queue_id
 *   The index of the queue.
 * @param op
 *   Pointer containing operation to be enqueued.
 *
 * @return
 *   Status of enqueue operation.
 */
int bbdev_ipc_enqueue_raw_op(uint16_t dev_id, uint16_t queue_id,
		struct bbdev_ipc_raw_op_t *op);

/**
 * Dequeue a raw operation.
 * For MODEM->HOST queues, this would provide RAW op which had 'conf_required'
 * set in the 'rte_bbdev_raw_op' structure.
 * For HOST->MODEM queues, this would provide RAW op which are sent from MODEM.
 * 'op' memory would be internally allocated
 *
 * @param dev_id
 *   The identifier of the device. Currently only one device id is
 *   supported - 'BBDEV_IPC_DEV_ID_0'
 * @param queue_id
 *   The index of the queue.
 *
 * @return
 *   Pointer containing dequeued operation.
 */
struct bbdev_ipc_raw_op_t *bbdev_ipc_dequeue_raw_op(uint16_t dev_id,
		uint16_t queue_id);

/**
 * Consume the RAW operation. Valid for Modem -> Host queues.
 * This will mark previous internal BD as free.
 *
 * @param dev_id
 *   The identifier of the device. Currently only one device id is
 *   supported - 'BBDEV_IPC_DEV_ID_0'
 * @param queue_id
 *   The index of the queue.
 * @param op
 *   Pointer containing operation to be enqueued.
 *
 * @return
 *   Status of enqueue operation.
 */
uint16_t bbdev_ipc_consume_raw_op(uint16_t dev_id, uint16_t queue_id,
		struct bbdev_ipc_raw_op_t *op);

/** @} */

#endif /* _BBDEV_IPC_H_ */
