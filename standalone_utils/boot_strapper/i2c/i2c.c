/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#include "i2c.h"

static int slave_addr = SLAVE_ADDR;
#ifdef I2C_CALCULATE_CLOCK
/* Clock rate and Frequency Divider Register settings
const uint16_t fsl_i2c_speed_map[] = {
	20, 22, 24, 26, 28, 30, 34, 40, 28, 32, 36, 40, 44, 48, 56, 68,
	48, 56, 64, 72, 80, 88, 104, 128, 80, 96, 112, 128, 144, 160, 192,
	240, 160, 192, 224, 256, 288, 320, 384, 480, 320, 384, 448, 512, 576,
	640, 768, 960, 640, 768, 896, 1024, 1152, 1280, 1536, 1920, 1280,
	1536, 1792, 2048, 2304, 2560, 3072, 3840, 40, 44, 48, 52, 56, 60, 68,
	80, 56, 64, 72, 80, 88, 96, 112, 136, 96, 112, 128, 144, 160, 176, 208,
	256, 160, 192, 224, 256, 288, 320, 384, 480, 320, 384, 448, 512, 576,
	640, 768, 960, 640, 768, 896, 1024, 1152, 1280, 1536, 1920, 1280, 1536,
	1792, 2048, 2304, 2560, 3072, 3840, 2560, 3072, 3584, 4096, 4608, 5120,
	6144, 7680, 80, 88, 96, 104, 112, 120, 136, 160, 112, 128, 144, 160,
	176, 192, 224, 272, 192, 224, 256, 288, 320, 352, 416, 512, 320, 384,
	448, 512, 576, 640, 768, 960, 640, 768, 896, 1024, 1152, 1280, 1536,
	1920, 1280, 1536, 1792, 2048, 2304, 2560, 3072, 3840, 2560, 3072, 3584,
	4096, 4608, 5120, 6144, 7680, 5120, 6144, 7168, 8192, 9216, 10240,
	12288, 15360 };

**************************************************************************
 * Functon	:	set_i2c_bus_speed
 * Arguments	:	i2c bus speed, i2c_regs address
 * Return	:	void
 * Description	:	calculates the value of divider and writes the
			appropriate value of FDR in IBFD register
 *************************************************************************

 * to calculate the FDR value the speed map commented above is used.
 * Divider value is calculated as get_sysclock()/i2c_speed. get_sysclock
 * returns the system which is 100MHz in this case and i2c speed used is
 * 100KHz.
 * Value of divider is 1000 and the value in speed map > divider value is
 * 1024 at offset 51. So FDR is 51.
 */
#endif

static void set_i2c_bus_speed(i2c_regs_t *i2c_regs, uint32_t i2c_speed)
{
	uint8_t fdr = 0;
#ifdef I2C_CALCULATE_CLOCK
	/* divider = get_sysclock()/i2c_speed;
	 * uint32_t divider = 1000;
	 * Find the First Divider value freater than or equal to the
	 * required Divider
	 * for (fdr = 0; fdr < I2C_MAX_FDR; fdr++) {
	 *	if (fsl_i2c_speed_map[fdr] >= (uint16_t)divider)
	 *		break;
	 * }
	 */
#else
#ifdef	CONFIG_I2C_FAST_MODE 
	fdr = 76;
#else
	fdr = 51;
#endif
	//log_str((char *)"FDR written in IBFD register\n");
	OUT_8(&i2c_regs->ibfd, fdr);/* Writes FDR value to IBFD register */
#endif
}

/***************************************************************************
 * Function	:	enable_digital_filter
 * Arguments	:	i2c_regs address
 * Return	:	void
 * Description	:	set the GLFLT_EN bit in Debug register
			Fix corresponding to Ticket TKT256758 - DesignPDM
 ***************************************************************************/
static void enable_digital_filter(i2c_regs_t *i2c_regs)
{
	/* Enable Digital Filter - In I2C Debug Regiter */
	OUT_8(&i2c_regs->ibdbg, I2C_DBG_GLITCH_EN);
}

/***************************************************************************
 * Function	:	clear_status_flags
 * Arguments	:	i2c_regs address, flag
 * Return	:	void
 * Description	:	Writes the value of flag in IBSR register
 ***************************************************************************/
static void clear_status_flags(i2c_regs_t *i2c_regs, uint8_t flag)
{
	OUT_8(&i2c_regs->ibsr, flag);
}

/***************************************************************************
 * Function	:	i2c_init
 * Arguments	:	void pointer
 * Return	:	result
 * Description	:	Initializes I2C.
 ***************************************************************************/
uint32_t i2c_init(void *x)
{
	uint32_t timer_wrap;
	i2c_regs_t *i2c_regs = NULL;
	i2c_regs = (i2c_regs_t *)CCSR_I2C1_BASE_ADDR;

	/* Stop I2C controller */
	OUT_8(&i2c_regs->ibcr, I2C_CR_MDIS);

	set_i2c_bus_speed(i2c_regs, CONFIG_I2C_SPEED);
	enable_digital_filter(i2c_regs); /* Need to ensure this ?? */

	/* Clear Status Register */
	clear_status_flags(i2c_regs, I2C_SR_IBIF | I2C_SR_IBAL);

	/*Start I2C controller */
	if (IN_8(&i2c_regs->ibcr) & I2C_CR_MDIS)
		OUT_8(&i2c_regs->ibcr, I2C_CR_MENA);

	timer_init(&timer_wrap);
	do {
		if (!(IN_8(&i2c_regs->ibcr) & I2C_CR_MDIS)) {
			//log_str((char *)"I2C module Initialized\n");
			timer_stop();
			return SUCCESS;
		}
	} while (!(get_timeout(CONFIG_I2C_TIMEOUT, &timer_wrap)));

	timer_stop();
//	log_str((char *)"No I2C module wakeup\n");

	return ERROR_I2C_NO_WAKEUP_INIT;
}

/***************************************************************************
 * Function	:	wait_for_bus_free
 * Inputs	:	i2c_regs address, current bus master
 * Return	:	Success or error based on bus is free or not.
 * Description	:	This function checks if the bus is free returns
			SUCCESS else return error.
 ***************************************************************************/
static int wait_for_bus_free(i2c_regs_t *i2c_regs, bool current_bus_master)
{
	uint8_t status = 0;
	uint8_t flag = 0;
	uint32_t timer_wrap;
	timer_init(&timer_wrap);
	do {
		status = IN_8(&i2c_regs->ibsr);
		if (current_bus_master == IS_CURRENT_MASTER) {
			if (status & I2C_SR_IBAL) {
				timer_stop();
				flag = (I2C_SR_IBIF | I2C_SR_IBAL);
				clear_status_flags(i2c_regs, flag);
				return SUCCESS;
			}
		}
		if (!(status & I2C_SR_IBB_BUSY)) {
			timer_stop();
			return SUCCESS;
		}
	} while (!(get_timeout(CONFIG_I2C_TIMEOUT, &timer_wrap)));

	timer_stop();
	//log_str((char *)"Bus not idle\n");
	return ERROR_I2C_NOT_IDLE;
}

/***************************************************************************
 * Function	:	wait_for_status_change
 * Inputs	:	i2c_regs address, state
 * Returns	;	SUCCESS or error
 * Descripotion	:	This function returns success if the state input is
			same as the IBSR register only when the arbitration
			is not lost else logs the error.
 ***************************************************************************/
static int wait_for_status_change(i2c_regs_t *i2c_regs, uint8_t state)
{
	uint8_t status;
	uint32_t timer_wrap;
	timer_init(&timer_wrap);
	do {
		status = IN_8(&i2c_regs->ibsr);
		if (status & I2C_SR_IBAL) {
				timer_stop();
				clear_status_flags(i2c_regs, I2C_SR_IBIF |
								I2C_SR_IBAL);
				//log_str((char *)"Arbitration lost status\n");
				//log_uint(status);
				return ERROR_I2C_RESTART;
			}
			if ((status & state) == state) {
				timer_stop();
				return SUCCESS;
			}
		} while (!(get_timeout(CONFIG_I2C_TIMEOUT, &timer_wrap)));

	timer_stop();
	//log_str((char *)"I2C timeout with status\n");
	//log_uint(status);
	return ERROR_I2C_TIMEOUT;
}

/***************************************************************************
 * Function	:	transmit_byte
 * Inputs	:	i2c_regs, Byte
 * Return	:	Success or error based on the operations.
 * Description	:	Writes the byte on to the IBDR register and sends the
			response based on the acknowledge recieve.
 ***************************************************************************/
static int transmit_byte(i2c_regs_t *i2c_regs, uint8_t byte)
{
	int ret = 0;
	uint8_t temp = 0;

	/* Clear Interrupt flags */
	clear_status_flags(i2c_regs, I2C_SR_IBIF);
	/* Write the byte to be transmitted on IBDR register */
	OUT_8(&i2c_regs->ibdr, byte);
	ret = wait_for_status_change(i2c_regs, I2C_SR_IBIF | I2C_SR_TCF);
	if (ret != SUCCESS)
		return ret;
	temp = IN_8(&i2c_regs->ibsr);
	if (temp & I2C_SR_NO_RXAK) {
		clear_status_flags(i2c_regs, I2C_SR_IBIF);
		//log_str((char *)"No Slave acknowledge\n");
		return ERROR_I2C_NOACK;
	}
	//log_str((char *)"Bytes transfer done");
	//log_uint(byte);

	return SUCCESS;
}

/***************************************************************************
 * Function	:	stop_i2c_cntrl
 * Inputs	:	i2c_regs address
 * Return	:	void
 * Description	:	sets the I2C module in slave and reciever mode
 ***************************************************************************/
static void stop_i2c_cntrl(i2c_regs_t *i2c_regs)
{
	int ret = 0;
	uint8_t temp = 0;
	temp = IN_8(&i2c_regs->ibcr);
	/* Set the I2C module in the slave and receiver mode */
	temp &= ~(I2C_CR_MSSL | I2C_CR_TXRX);
	OUT_8(&i2c_regs->ibcr, temp);

	ret = wait_for_bus_free(i2c_regs, IS_CURRENT_MASTER);
	if (ret != SUCCESS);
		//log_str((char *)"STOP failed");
}

/***************************************************************************
 * Function	:	read_data
 * Inputs	:	i2c_regs, destination, size
 * return	:	Success if data read is successfull else return error
 ***************************************************************************/
static int read_data(i2c_regs_t *i2c_regs, uint8_t *des, uint32_t size)
{
	uint8_t temp = 0, flag = 0;
	uint32_t i = 0;
#ifdef CONFIG_I2C_WORD_COPY
	uint32_t word = 0, byte_loop = 0, words_loop = 0, *dest;
	uint32_t j = 0, k = 0, dest_unalign_chk = 0, size_rem_loop = 0;
#endif
	int ret;

	temp = IN_8(&i2c_regs->ibcr);
	temp &= ~(I2C_CR_TXRX | I2C_CR_NOACK);
	if (size == 1)
		temp |= I2C_CR_NOACK;
	OUT_8(&i2c_regs->ibcr, temp);

	clear_status_flags(i2c_regs, I2C_SR_IBIF);

	IN_8(&i2c_regs->ibdr); /* Dummy read */
#ifdef CONFIG_I2C_WORD_COPY
	/* dest_unalign_chk checks if the destination is unaligned */
	dest_unalign_chk = (uint32_t)des % 4;
	/* byte loop stores number of 1 byte transfer to be performed */
	if (dest_unalign_chk)
		byte_loop = 4 - dest_unalign_chk;
	else
		byte_loop = 0;
	/* Dest pointer which is poiting to next word boundary */
	dest = (uint32_t *)(des + byte_loop);
	/* Number of word copies that will be done */
	words_loop = (size - byte_loop) / 4;
	/* Size remaining to be copied after word right*/
	size_rem_loop = (size - byte_loop) % 4;
#endif
	for (i = 0; i < size; i++) {
		flag = I2C_SR_IBIF | I2C_SR_TCF;
		ret = wait_for_status_change(i2c_regs, flag);
		if (ret != SUCCESS) {
			if (ret == ERROR_I2C_TIMEOUT)
				return ERROR_READ_TIMEOUT;
			else
				return ret;
		}
		if (i == (size - 1)) {
			stop_i2c_cntrl(i2c_regs);
		} else if (i == (size - 2)) {
			temp = IN_8(&i2c_regs->ibcr);
			temp |= I2C_CR_NOACK;
			OUT_8(&i2c_regs->ibcr, temp);
		}
		clear_status_flags(i2c_regs, I2C_SR_IBIF);
		temp = IN_8(&i2c_regs->ibdr);

		/* WORD_COPY macro enabled if needed to perform word write */
#ifdef CONFIG_I2C_WORD_COPY
		/*
		 * byte_loop performs byte wise write and word loop
		 * performs word write size_rem_loop performs transfer of
		 * bytes that are left unalligned in last.
		 */
		if (byte_loop) {
			des[i] = temp;
#ifdef CONFIG_DEBUG
			//log_str((char *)"Byte Read is\n");
			//log_uint((uint8_t)des[i]);
#endif
			byte_loop--;
		} else if (words_loop) {
			word =  temp << (8 * j) | word;
			j++;
			if (j == 4) {
				j = 0;
				dest[k] = word;
#ifdef CONFIG_DEBUG
			//	log_str((char *)"Word Read is\n");
			//	log_uint((uint32_t)dest[k]);
#endif
				k++;
				word = 0;
				words_loop--;
			}
		} else if (size_rem_loop) {
			des[i] = temp;
#ifdef CONFIG_DEBUG
	//		log_str((char *)"Byte Read is\n");
	//		log_uint((uint8_t)des[i]);
#endif
			size_rem_loop--;
		}
#else
		des[i] = temp;
#ifdef CONFIG_DEBUG
	//	log_str((char *)"Byte Read is\n");
	//	log_uint((uint8_t)des[i]);
#endif
#endif
	}

	return SUCCESS;
}


/***************************************************************************
 * Function	:	attempt_send_addr
 * Inputs	:	i2c_regs, src_offset, alen
 * Return	:	Success or Error code
 * Description	:	Sends the slave address and the source offset
 ***************************************************************************/
static int attempt_send_addr(i2c_regs_t *i2c_regs, uint32_t src_offset,
				uint8_t alen)
{
	uint8_t temp = 0;
	int ret = 0;
	uint8_t control = 0;
	uint32_t timer_wrap;
	control = IN_8(&i2c_regs->ibcr);

	/* Enable I2C controller */
	if (control & I2C_CR_MDIS) {
		control = ((control | I2C_CR_NOACK) & ~I2C_CR_MDIS);
		OUT_8(&i2c_regs->ibcr, control);
	}

	/* Check if the I2C module is enabled else return error*/
	timer_init(&timer_wrap);
	do {
		if (!(IN_8(&i2c_regs->ibcr) & I2C_CR_MDIS))
			break;
	} while	(!(get_timeout(CONFIG_I2C_TIMEOUT, &timer_wrap)));
	timer_stop();
	if (IN_8(&i2c_regs->ibcr) & I2C_CR_MDIS) {
	//	log_str((char *)"No I2C wakeup\n");
		return ERROR_I2C_NO_WAKEUP_READ;
	}

	/* Check for the slave address in the IBAD register */
	if (IN_8(&i2c_regs->ibad) == (slave_addr << 1))
		OUT_8(&i2c_regs->ibad, (I2C_RESERVED_ID<<1));//TODO: MK why we are update ibad register with reserve id?

	/* Clear status flag*/
	clear_status_flags(i2c_regs, I2C_SR_IBIF | I2C_SR_IBAL);

	ret = wait_for_bus_free(i2c_regs, ISNOT_CURRENT_MASTER);
	if (ret != SUCCESS)
		return ret;

	/* Set Master Mode*/
	temp = IN_8(&i2c_regs->ibcr);
	temp |= I2C_CR_MSSL;
	OUT_8(&i2c_regs->ibcr, temp);

	ret = wait_for_status_change(i2c_regs, I2C_SR_IBB_BUSY);
	if (ret != SUCCESS) {
	//	log_str((char *)"Bus not busy\n");
		return ERROR_I2C_NOT_BUSY;
	}

	/* Set transmit mode */
	temp = IN_8(&i2c_regs->ibcr);
	temp |= I2C_CR_TXRX | I2C_CR_NOACK;
	OUT_8(&i2c_regs->ibcr, temp);

	/* Write slave address in IBDR register */
	ret = transmit_byte(i2c_regs, slave_addr << 1);
	if (ret != SUCCESS) {
		if (ret == ERROR_I2C_NOACK) {
	//		log_str((char *)"No slave ACK for SLAVE ID\n");
			return ERROR_I2C_NODEV;
		} else if (ret == ERROR_I2C_TIMEOUT) {
				return ERROR_SLAVE_ADDR_TIMEOUT;
			} else {
				return ret;
			}
	}

	/* Transmit source offset */
	while (alen--) {
		ret = transmit_byte(i2c_regs,
			(src_offset >> (alen * 8)) & 0xff);
		if (ret != SUCCESS) {
			if (ret == ERROR_I2C_TIMEOUT)
				return ERROR_MEM_ADDR_TIMEOUT;
			else
				return ret;
		}
	}
	return SUCCESS;
}

/***************************************************************************
 * Function	:	init_source_addr
 * Inputs	:	i2c_regs address, Source offset, Alen
 * Return	:	Return Success or the error recieved
 * Description	:	Initializes the transfer
 ***************************************************************************/
static int init_source_addr(i2c_regs_t *i2c_regs, uint32_t src_offset,
				uint8_t alen)
{
	int ret = ERROR_I2C_TIMEOUT;
	int retry = 0;
	uint32_t timer_wrap;
	for (retry = 0; retry < 3; retry++) {
		ret = attempt_send_addr(i2c_regs, src_offset, alen);
		if (ret == SUCCESS) {
	//		log_str((char *)"Src_offset INIT done");
			return SUCCESS;
		}
		stop_i2c_cntrl(i2c_regs);
		if (ret == ERROR_I2C_NODEV)
			return ret;

	//	log_str((char *)"Retry failed during iteration");
	//	log_uint(retry);

		/* Disable I2C module */
		OUT_8(&i2c_regs->ibcr, I2C_CR_MDIS);
		timer_init(&timer_wrap);

		/* WHY THIMEOUT IS NEEDED HERE?? */
		do {
			if (IN_8(&i2c_regs->ibcr) & I2C_CR_MDIS)
				break;
		} while (!(get_timeout(CONFIG_I2C_TIMEOUT, &timer_wrap)));
		timer_stop();
	}
//	log_str((char *)"Failed to initialize transfer");
	return ret;
}

/***************************************************************************
 * Function	:	i2c_read
 * Arguments	:	src_offset - Source offset
 *			dst - destination pointer
 *			size - Size to be read
 *			void pointer
 * Return	:	result
 * Description	:	Read data from I2C.
 ***************************************************************************/
uint32_t i2c_read(uint32_t src_offset, uint8_t *dst, uint32_t size, void *x)
{
	i2c_regs_t *i2c_regs = NULL;
	i2c_regs = (i2c_regs_t *)CCSR_I2C1_BASE_ADDR;
	int ret = 0;
	uint8_t temp = 0;
	
	slave_addr += (src_offset >> 16) & 0x0F;
	src_offset &= 0xFFFF;
	/*
	 * 3 condition when address can be larger than 16 bits
	 * Starting address > 16bit (Ending address migth be
	 * less than 16bit if rollover on u32 boundary)
	 * Ending Address > 16bt
	 * Size > 16bit (Starting and ending address < 16bit
	 * via rolloff on u32 boundary)
	 */
	bool ovf_cond1 = (src_offset > I2C_MAX_SRC_OFFSET);
	bool ovf_cond2 = ((src_offset + size - 1) > I2C_MAX_SRC_OFFSET);
	bool ovf_cond3 = (size > I2C_MAX_READ_SIZE);
	if (ovf_cond1 || ovf_cond2 || ovf_cond3) {
//		log_str((char *)"Offset address greater than 16-bit");
		return ERROR_I2C_INVALID_OFFSET;
	}

	/* set alen = 2 for 2 byte read (exteded addressing) */
	uint8_t alen = 2;
	ret = init_source_addr(i2c_regs, src_offset, alen);
	if (ret != SUCCESS)
		return ret;

	/* Repeat START */
	temp = IN_8(&i2c_regs->ibcr);
	temp |= I2C_CR_RSTA;
	OUT_8(&i2c_regs->ibcr, temp);

	/* Set READ bit for read operation */
	ret = transmit_byte(i2c_regs, (slave_addr << 1) | 0x1);
	if (ret != SUCCESS) {
		/* Send STOP to controller */
		stop_i2c_cntrl(i2c_regs);
		if (ret == ERROR_I2C_TIMEOUT)
			return ERROR_SLAVE_ADDR_TIMEOUT;
		else
			return ret;
		}
	ret = read_data(i2c_regs, dst, size);
	if (ret != SUCCESS) {
		stop_i2c_cntrl(i2c_regs);
		return ret;
	}
	stop_i2c_cntrl(i2c_regs);

	return SUCCESS;
}
