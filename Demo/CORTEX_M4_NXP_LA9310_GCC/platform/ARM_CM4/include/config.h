/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021-2022 NXP
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define PCIE_PHY_ADDR           0xa0000000
#define TCMU_PHY_ADDR           0x20000000

#ifdef TURN_ON_STANDALONE_MODE
#define TCML_PHY_ADDR 0x1f801000
#else
#define TCML_PHY_ADDR 0x1f800000
#endif
#define NXP_ERRATUM_A010650     1

#define NXP_ERRATUM_A_009410    ( 1 )
#define NXP_ERRATUM_A_009531    ( 1 )

#define ARM_ERRATUM_838869      ( 1 )
#endif /* CONFIG_H */
