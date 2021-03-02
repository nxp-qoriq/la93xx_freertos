/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2021 NXP
 */

#include <string.h>
#include "FreeRTOS.h"
#include "debug_console.h"
#include "io.h"
#include "bbdev_ipc.h"

#define BBDEV_IPC_DEV_ID_0	0
#define TEST_REPETITIONS 	10000
#define TEST_BUFFER_INPUT_VAL 	0x01020304

uint32_t *input_data;
uint32_t *output_data;

static void
host_to_modem_test(int queue_id, int conf_mode, int latency_test)
{
	struct bbdev_ipc_raw_op_t *deq_raw_op;
	int ret, i, j, len;
	uint32_t *in_data, *out_data;

	i = 0;
	while (i < TEST_REPETITIONS) {

		deq_raw_op = bbdev_ipc_dequeue_raw_op(BBDEV_IPC_DEV_ID_0,
						      queue_id);
		if (deq_raw_op == NULL)
			continue;

		if (!latency_test && deq_raw_op->out_addr) {
			in_data = (uint32_t *)deq_raw_op->in_addr;
			out_data = (uint32_t *)deq_raw_op->out_addr;
			len = deq_raw_op->in_len / sizeof(uint32_t);

			for (j = 0; j < len; j++)
				out_data[j] = in_data[j];

			deq_raw_op->out_len = len * sizeof(uint32_t);
		}

		ret = bbdev_ipc_consume_raw_op(BBDEV_IPC_DEV_ID_0, queue_id,
					       deq_raw_op);
		if (ret) {
			log_err("rte_bbdev_consume_raw_op failed (%d)\n\r", ret);
			log_err("\n\r============================================================================================\n\r");
			if (conf_mode)
				log_err("HOST->MODEM test failed for confirmation mode.\n\r");
			else
				log_err("HOST->MODEM test failed for non confirmation mode.\n\r");
			log_err("============================================================================================\n\r");
			return;
		}

		i++;
	}

	log_info("\n\r============================================================================================\n\r");
	if (conf_mode) {
		log_info("HOST->MODEM test completed for confirmation mode. Check application logs for results.\n\r");
	} else {
		log_info("Received %d operations for HOST->MODEM\n\r",
			 TEST_REPETITIONS);
		log_info("HOST->MODEM test successful for non confirmation mode\n\r");
	}
	log_info("============================================================================================\n\r");
}

static void
modem_to_host_test(int queue_id, int conf_mode, int latency_test)
{
	struct bbdev_ipc_raw_op_t enq_raw_op, *deq_raw_op;
	uint32_t length, input_len;
	int ret, i, j;

	i = 0;
	while (i < TEST_REPETITIONS) {

		input_data = bbdev_ipc_get_next_internal_buf(
			BBDEV_IPC_DEV_ID_0, queue_id, &length);
		input_len = length/2;
		output_data = (void *)input_data + length/2;

		for (j = 0; j < input_len/sizeof(uint32_t); j++) {
			input_data[j] = TEST_BUFFER_INPUT_VAL;
			output_data[j] = 0;
		}

		enq_raw_op.in_len = input_len;
		enq_raw_op.in_addr = (uint32_t)input_data;
		enq_raw_op.out_len = 0;
		enq_raw_op.out_addr = (uint32_t)output_data;

		ret = bbdev_ipc_enqueue_raw_op(BBDEV_IPC_DEV_ID_0, queue_id,
					       &enq_raw_op);
		if (ret < 0)
			continue;

		if (conf_mode) {
			do {
				deq_raw_op = bbdev_ipc_dequeue_raw_op(
							BBDEV_IPC_DEV_ID_0,
							queue_id);
			} while (!deq_raw_op);

			if (!latency_test && deq_raw_op->out_addr) {
				for (j = 0; j < input_len/sizeof(uint32_t); j++) {
					if (output_data[j] !=
					    TEST_BUFFER_INPUT_VAL) {
						log_err("output %x does not match expected output %x",
							output_data[j],
							TEST_BUFFER_INPUT_VAL);
						log_err("\n\r============================================================================================\n\r");
						log_err("MODEM->HOST test failed for confirmation mode\n\r");
						log_err("============================================================================================\n\r");
						return;
					}
				}
			}
		}

		i++;
	}

	log_info("\n\r============================================================================================\n\r");
	if (conf_mode) {
		log_info("Validated %d operations for MODEM->HOST\n\r",
			 TEST_REPETITIONS);
		log_info("MODEM->HOST test successful for confirmation mode\n\r");
	} else {
		log_info("MODEM->HOST test completed for non confirmation mode. Check application logs for results.\n\r");
	}
	log_info("============================================================================================\n\r");
}

static void
round_trip_latency_test(int h2m_queue_id, int m2h_queue_id)
{
	struct bbdev_ipc_raw_op_t enq_raw_op, *deq_raw_op;
	uint32_t length, input_len;
	int ret, i;

	i = 0;
	while (i < TEST_REPETITIONS) {

		input_data = bbdev_ipc_get_next_internal_buf(
			BBDEV_IPC_DEV_ID_0, m2h_queue_id, &length);
		input_len = length/2;
		output_data = (void *)input_data + length/2;

		enq_raw_op.in_len = input_len;
		enq_raw_op.in_addr = (uint32_t)input_data;
		enq_raw_op.out_len = 0;
		enq_raw_op.out_addr = (uint32_t)output_data;

		deq_raw_op = bbdev_ipc_dequeue_raw_op(BBDEV_IPC_DEV_ID_0,
						      h2m_queue_id);
		if (deq_raw_op == NULL)
			continue;

		i++;

		ret = bbdev_ipc_consume_raw_op(BBDEV_IPC_DEV_ID_0, h2m_queue_id,
					       deq_raw_op);
		if (ret < 0) {
			log_err("rte_bbdev_consume_raw_op failed (%d)", ret);
			log_err("\n\r============================================================================================\n\r");
			log_err("Round trip latency test failed.\n\r");
			log_err("============================================================================================\n\r");
			return;
		}

retry_enqueue:

		ret = bbdev_ipc_enqueue_raw_op(BBDEV_IPC_DEV_ID_0, m2h_queue_id,
					       &enq_raw_op);
		if (ret < 0)
			goto retry_enqueue;
	}

	log_info("\n\r============================================================================================\n\r");
	log_info("Round trip latency test completed with %d test cases. Check application logs for results.\n\r",
		 i);
	log_info("============================================================================================\n\r");
}

int vLa9310DemoIpcRawTest(bool latency_test)
{
	struct dev_attr_t *attr;
	int queue_id, ret;

	log_info("\n\rExecuting BBDEV RAW ops test\n\r\r");

	/* Wait for bbdev device ready.*/
	while (!bbdev_ipc_is_host_initialized(BBDEV_IPC_DEV_ID_0)) {}

	attr = bbdev_ipc_get_dev_attr(BBDEV_IPC_DEV_ID_0);

	for (queue_id = 0; queue_id < attr->num_queues; queue_id++) {
		ret = bbdev_ipc_queue_configure(BBDEV_IPC_DEV_ID_0, queue_id);
		if (ret != IPC_SUCCESS) {
			log_err("queue configure failed for queue %d error %d \n\r\r",
				queue_id, ret);
			return ret;
		}
	}

	log_info("\n\r%s: ==> Processing queues [", __func__);
	for (queue_id = 0; queue_id < attr->num_queues; queue_id++) {
		if (queue_id == 0)
			log_info("%d", queue_id);
		else
			log_info(", %d", queue_id);
	}
	log_info("]\n\r\r");

	/* Signal Host to start sending packets for processing */
	bbdev_ipc_signal_ready(BBDEV_IPC_DEV_ID_0);

	queue_id = 0;
	if (!latency_test) {
		if (attr->qattr[0].conf_enable) {
			host_to_modem_test(queue_id, 1, 0);
			modem_to_host_test(queue_id + 1, 1, 0);
		} else {
			host_to_modem_test(queue_id, 0, 0);
			modem_to_host_test(queue_id + 1, 0, 0);
		}
	} else {
		if (attr->qattr[0].conf_enable) {
			host_to_modem_test(queue_id, 1, 1);
			modem_to_host_test(queue_id + 1, 1, 1);
		} else {
			round_trip_latency_test(0, 1);
		}
	}

	return 0;
}

void vLa9310DemoIpcMsgTest()
{
	struct bbdev_ipc_raw_op_t *deq_raw_op = NULL;
	int ret;

	deq_raw_op = bbdev_ipc_dequeue_raw_op(0, 0);
	if (deq_raw_op == NULL)
		return;

	if (deq_raw_op->out_addr) {
		uint32_t *in_data, *out_data, len, i;

		in_data = (uint32_t *)deq_raw_op->in_addr;
		out_data = (uint32_t *)deq_raw_op->out_addr;
		len = deq_raw_op->in_len / sizeof(uint32_t);

		for (i = 0; i < len; i++)
			out_data[i] = SWAP_32(in_data[i]);

		deq_raw_op->out_len = len * sizeof(uint32_t);
	}
retry:
	ret = bbdev_ipc_consume_raw_op(0, 0, deq_raw_op);
	if (ret)
		goto retry;
}
