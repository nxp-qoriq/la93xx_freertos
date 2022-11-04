/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#include "i2c.h"
//#define BOOTSTRAP_DEBUG
int main_init(void){
	struct header hdr;
	struct ext_header ext_hdr;
	__attribute__((__noreturn__)) void (*jump_bl_entry)(void) = NULL;
	uint32_t ret = FAILURE;
	uint8_t tmp[MAX_DRIV_BUFF_SZ];
	log_init();
	rom_memset((void *)TCM_LOG_BUFF_START, 0, LOG_BUFF_SIZE);
	log_str("boot strapper main called\n");
	rom_memset(&hdr, 0, sizeof(struct header));
	
	/*inialize i2c controller*/
	ret = i2c_init((void *)tmp);
	if (ret != SUCCESS)
		error_handler(ret);
	/*
	 * Read the extended header values
	 */
	ret = i2c_read(I2C_EXT_HDR_OFFSET, (uint8_t *)&ext_hdr,
			sizeof(struct ext_header), (void *)tmp);
	if (ret != SUCCESS)
		error_handler(ret);
#ifdef BOOTSTRAP_DEBUG
	log_str("ext_header data\n");
	uint32_t *val = (uint32_t *)&ext_hdr;
	for(int i = 0; i < sizeof(struct ext_header)/sizeof(int); i++)
		log_uint((uint32_t)val[i]);

	log_str("boot_strapper header offset");
	log_uint((uint32_t)ext_hdr.freertos_image_offset);
#endif

	/*
	 * Read the boot_strapper header
	 */
	ret = i2c_read(ext_hdr.freertos_image_offset, (uint8_t *)&hdr,
			sizeof(struct header), (void *)tmp);
	if (ret != SUCCESS)
		error_handler(ret);
#ifdef BOOTSTRAP_DEBUG
	log_str("header data\n");
	val = (uint32_t *)&hdr;
	for(int i = 0; i < sizeof(struct header)/sizeof(int); i++)
		log_uint((uint32_t)val[i]);
	log_str("reading freeRTOS img data\n");
#endif		
	
	uint32_t src_offset = hdr.bl_src_offset;
	uint32_t dst_addr = hdr.bl_dest;
	uint32_t size = hdr.bl_size, size1 = 0, read_size = 0;

	while(size != 0){
		if((src_offset + (size - 1)) > I2C_MAX_READ_SIZE){
			src_offset+= size1;
			dst_addr+= size1;
			size1 = (I2C_MAX_READ_SIZE - src_offset) + 1;
			read_size = (I2C_MAX_READ_SIZE - src_offset);
			size = size - size1;
		}

		else{
			src_offset+= size1;
			dst_addr+= size1;
			read_size = size;
			size -= read_size;

		}
#ifdef BOOTSTRAP_DEBUG
		log_str("read_size");
		log_uint((uint32_t)read_size);
#endif
		ret = i2c_read(src_offset,
				(uint8_t *)dst_addr,
				read_size, (void *)tmp);
		if (ret != SUCCESS)
			error_handler(ret);
	}
	
	jump_bl_entry = (void *)(hdr.bl_entry | M4_THUMB_BIT);

	log_str("jumping to freertos image\n");
	jump_bl_entry();

	/*
	 * Control should never reach here
	 */
	while(1);
	return 0;
}

