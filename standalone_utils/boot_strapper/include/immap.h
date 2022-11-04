/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#ifndef IMMAP_H
#define IMMAP_H

#define M4_VTOR_BASE		0xE000ED08	/* Vector Table Offset Reg */
#define M4_THUMB_BIT		0x1		/* M4 Thumb bit */
#define M4_SYST_CSR		0xE000E010	/* SysTick Ctl and Status */
#define M4_SYST_RVR		0xE000E014	/* SysTick Reload Value */
#define M4_SYST_CVR		0xE000E018	/* SysTick Current Value */
#define M4_SYST_CALIB		0xE000E01C	/* SysTick Calibration value */

#define CCSR_BASE_ADDR		0x40000000	/* CCSR base addr */
#define DCSR_BASE_ADDR		0xE0000000	/* DCSR base addr */
#define TCML_BASE_ADDR		0x1f800000	/* TCML base addr */
#define TCML_MEM_SIZE		0x20000		/* TCML memory size */
#define TCMU_BASE_ADDR		0x20000000	/* TCMU base addr */
#define TCMU_MEM_SIZE		0x10000		/* TCMU memory size */
#ifdef CONFIG_PCIE_EMU_TESTING
#define PCIE_HOST_BASE_ADDR	TCMU_BASE_ADDR	/* PCIe host mem base */
#else
#define PCIE_HOST_BASE_ADDR	0xA0000000	/* PCIe host mem base */
#endif
#define PCIE_HOST_MEM_SIZE	0x40000000	/* PCIe host mem size */

/* Log buffer on SRAM (TCMU), size: 1k */
#define TCM_LOG_BUFF_START	(TCMU_BASE_ADDR + 0xF000)
#define LOG_BUFF_SIZE		0x400

/* Stack on SRAM (TCMU), size: 3k */
#define TCM_STACK_START		(TCMU_BASE_ADDR + 0xF400)
#define STACK_SIZE		0xC00
#define STACK_INIT		(TCM_STACK_START + STACK_SIZE - 4)

/* Boot flow specific defines */
#define I2C_BOOT_HDR_OFFSET	0x2000		/* I2C Boot header offset */
#define I2C_EXT_HDR_OFFSET	0x20		/* I2C Boot header offset */
#define DIRECT_BL_DEST		TCML_BASE_ADDR	/* Direct flow bl dest */
#define PLUGIN_START		TCML_BASE_ADDR	/* Plugin routine address */

/* CCSR space DCFG base */
#define CCSR_DCFG_BASE_ADDR	(CCSR_BASE_ADDR + 0x1e00000)
#define DCFG_PORSR2_OFFSET	0x4		/* DCFG PORSR2 offset */
#define	PORSR2_BOOT_SRC_MASK	0x00c00000	/* BOOT SRC mask */
#define PORSR2_BOOT_SRC_SHIFT	22		/* BOOT SRC shift */
#define BOOT_SRC_MASK		0b11		/* BOOT SRC mask */
#define BOOT_SRC_DIRECT		0b00		/* BOOT SRC: Direct */
#define BOOT_SRC_RESERVED	0b01		/* BOOT SRC: Reserved */
#define BOOT_SRC_I2C		0b10		/* BOOT SRC: I2C */
#define BOOT_SRC_PCIE		0b11		/* BOOT SRC: PCIe */
#define	PORSR2_SYSRDY_EVT_MASK	0x00080000	/* CFG_SYSRDY_EVT mask */
#define	PORSR2_WRM_RSTB_MASK	0x80000000	/* CFG_WRM_RSTB mask */
#define	PORSR2_WRM_RSTB_SHIFT	31		/* CFG_WRM_RSTB shift */
#define DCFG_SVR_OFFSET		0xA4		/* DCFG SVR offset */
#define DCFG_SCRATCH1_OFFSET	0x200		/* DCFG SCRATCH1 offset */

/* I2C controller base addr */
#define CCSR_I2C1_BASE_ADDR	(CCSR_BASE_ADDR + 0x2000000)

/* PCIe controller base addr */
#define CCSR_PCIE_BASE_ADDR	(CCSR_BASE_ADDR + 0x3400000)

/* EDMA controller base addr */
#define CCSR_EDMA_BASE_ADDR	(CCSR_BASE_ADDR + 0x22c0000)

/* CCSR Reset base addr */
#define CCSR_RST_BASE_ADDRESS	(CCSR_BASE_ADDR + 0x1E60000)
#define RCW_REQR_OFFSET		0x100		/* RCW request reg off */
#define RCW_COMPLETIONR_OFFSET	0x104		/* RCW comp reg off */
#define PBI_COMPLETIONR_OFFSET	0x114		/* PBI comp reg off */
#define RSTSR_OFFSET		0x0C		/* Reset ready reg off */
#define RCW_COMPLETION_DONE	0x1		/* RCW comp bit */
#define PBI_COMPLETION_DONE	0x1		/* PBI comp bit */
#define RCW_REQR_RQ		0x1		/* RCW request bit */
#define RST_READY		0x1		/* Reset ready bit */

#define RCW_REQR		(CCSR_RST_BASE_ADDRESS + RCW_REQR_OFFSET)
#define RCW_COMPLETIONR		(CCSR_RST_BASE_ADDRESS + RCW_COMPLETIONR_OFFSET)
#define PBI_COMPLETIONR		(CCSR_RST_BASE_ADDRESS + PBI_COMPLETIONR_OFFSET)
#define RSTSR			(CCSR_RST_BASE_ADDRESS + RSTSR_OFFSET)

/* Run contol block base address */
#define DCSR_RCPM_BASE_ADDR	(DCSR_BASE_ADDR + 0x41000)
#define RCPM_CGACRE0_OFFSET	0x900

/* EPU block base address */
#define DCSR_EPU_BASE_ADDR	(DCSR_BASE_ADDR + 0x45000)
#define EPU_EPSMCR0_OFFSET	0x200
#define EPU_EPECR0_OFFSET	0x300

#endif /* ifndef IMMAP_H */
