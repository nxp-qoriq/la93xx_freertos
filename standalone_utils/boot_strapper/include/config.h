/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#ifndef CONFIG_H
#define CONFIG_H

#define CONFIG_EDMA_LE
#define CONFIG_I2C_WORD_COPY
#define CONFIG_ADDRESS_ALIGNMENT_CHECK
#define CONFIG_SIZE_ALIGNMENT_CHECK

/*
 * Macro for workaround on RTL 15 release
 */
/* #define CONFIG_RTL15_WORKAROUND */
#ifdef CONFIG_RTL15_WORKAROUND
/*
 * PCIE BAR's mask size addresses need not to be programmed by
 * Rattler Boot-ROM. But for RTL15 we need to program these as
 * RTL15 doesn't have correct BAR size configuration. Expected
 * to be corrected in next RTL release.
 */
#define CONFIG_PCIE_BARS
#endif

/*
 * For building ROM images for verification, some portions of
 * code are stubbed out.
 */
/* #define CONFIG_MINIMAL_ROM */
#ifdef CONFIG_MINIMAL_ROM
/*
 * As verification environment is slow, so skip TCM initialization.
 */
#define CONFIG_SKIP_TCM_INIT
/*
 * As verification environment is slow, PCIe equalization takes much
 * longer time to complete. So to shorten up the time added following
 * macro for additional configuration.
 */
#define CONFIG_PCIE_EQU_PROG
/*
 * As in verification environment the I2C with the IBFD value as 51
 * is very slow, so to change the I2C to fast mode IBFD value is
 * changed to 76 (as asked by the verification team). This changes makes the
 * divider value to 88 in place of 1024.
 */
#define CONFIG_I2C_FAST_MODE
#endif

/* Used for testing PCIe boot flow on emulator */
/* #define CONFIG_PCIE_EMU_TESTING */

#endif /* ifndef CONFIG_H */
