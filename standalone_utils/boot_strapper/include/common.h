/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#ifndef COMMON_H
#define COMMON_H

#include "types.h"
#include "config.h"
#include "immap.h"
#include "utils.h"
#include "log.h"
#include "io.h"
#include "error_codes.h"
#include "platform.h"

#define MAX_DRIV_BUFF_SZ	64

/* Boot header structure */
struct header {
#define PREAMBLE		0xaa55aa55
	uint32_t preamble;
	uint32_t plugin_size;
	uint32_t plugin_offset;
	uint32_t bl_size;
	uint32_t bl_src_offset;
	uint32_t bl_dest;
	uint32_t bl_entry;
#define PCIE_EDMA_DIS_MASK	0x00000001
#define RESET_HS_DIS_MASK	0x00010000
	uint32_t flags;
};

/*################### Extended Boot Header #############*/

struct ext_header {
	uint32_t page_write_size;
	uint32_t addr_shift;
	uint32_t freertos_image_offset;
	uint32_t vspa_bin_location;
	uint32_t vspa_table_location;
	uint32_t reserved1;
	uint32_t reserved2;
	uint32_t reserved3;
};
#endif /* ifndef COMMON_H */
