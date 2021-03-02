/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */


/* error codes */
#define FAILURE                           -1
#define SUCCESS                           0
#define EDMA_SOURCE_UNALIGNED             1
#define EDMA_DESTINATION_UNALIGNED        2
#define EDMA_SIZE_UNALIGNED               3
#define EDMA_ZERO_BYTE_COUNT              4
#define EDMA_FREE                         5
#define EDMA_BUSY                         6
#define EDMA_TX_COMPLETION_QUEUE_EMPTY    7
#define EINVALID_MSG_LEN                  8
#define EINVALID_MSG_ADDRESS              9
#define EIPC_CHANNEL_FULL                 10
#define EIPC_CHANNEL_EMPTY                11
#define EIPC_NOTIFICATION_QUEUE_EMPTY     12
#define AVI_IPC_INIT_NOT_DONE             13
#define AVI_INVALID_MBOX_INDEX            14
#define AVI_MBOX_NOT_AVAILABLE            15
#define GPIO_PIN_NOT_SUPPORTED            16
#define GPIO_TYPE_NOT_SET                 17
#define HOST_NOT_READY                    18
#define I2C_TIMEOUT                       19
#define I2C_RESTART                       20
#define I2C_NODEV                         21
#define I2C_NOT_IDLE                      22
#define I2C_NOT_BUSY                      23
#define I2C_INVALID_OFFSET                24
#define I2C_NO_WAKEUP_INIT                25
#define I2C_NO_WAKEUP_READ                26
#define I2C_NOACK                         27
#define I2C_READ_TIMEOUT                  28
#define I2C_SLAVE_ADDR_TIMEOUT            29
#define I2C_MEM_ADDR_TIMEOUT              30
#define EDMA_RX_COMPLETION_QUEUE_EMPTY    31
#define EDMA_IPC_ERROR                    32
#define EINVALID_IPC_CHANNEL              33
#define EDMA_INVALID_CHANNEL              34
#define AVI_MBOX_RCV_FAIL                 35
