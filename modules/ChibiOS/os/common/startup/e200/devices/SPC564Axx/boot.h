/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    boot.h
 * @brief   Boot parameters for the SPC564Axx.
 * @{
 */

#ifndef BOOT_H
#define BOOT_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/**
 * @name   MASx registers definitions
 * @{
 */
#define MAS0_TBLMAS_TBL         0x10000000
#define MAS0_ESEL_MASK          0x000F0000
#define MAS0_ESEL(n)            ((n) << 16)

#define MAS1_VALID              0x80000000
#define MAS1_IPROT              0x40000000
#define MAS1_TID_MASK           0x00FF0000
#define MAS1_TS                 0x00001000
#define MAS1_TSISE_MASK         0x00000F80
#define MAS1_TSISE_1K           0x00000000
#define MAS1_TSISE_2K           0x00000080
#define MAS1_TSISE_4K           0x00000100
#define MAS1_TSISE_8K           0x00000180
#define MAS1_TSISE_16K          0x00000200
#define MAS1_TSISE_32K          0x00000280
#define MAS1_TSISE_64K          0x00000300
#define MAS1_TSISE_128K         0x00000380
#define MAS1_TSISE_256K         0x00000400
#define MAS1_TSISE_512K         0x00000480
#define MAS1_TSISE_1M           0x00000500
#define MAS1_TSISE_2M           0x00000580
#define MAS1_TSISE_4M           0x00000600
#define MAS1_TSISE_8M           0x00000680
#define MAS1_TSISE_16M          0x00000700
#define MAS1_TSISE_32M          0x00000780
#define MAS1_TSISE_64M          0x00000800
#define MAS1_TSISE_128M         0x00000880
#define MAS1_TSISE_256M         0x00000900
#define MAS1_TSISE_512M         0x00000980
#define MAS1_TSISE_1G           0x00000A00
#define MAS1_TSISE_2G           0x00000A80
#define MAS1_TSISE_4G           0x00000B00

#define MAS2_EPN_MASK           0xFFFFFC00
#define MAS2_EPN(n)             ((n) & MAS2_EPN_MASK)
#define MAS2_EBOOK              0x00000000
#define MAS2_VLE                0x00000020
#define MAS2_W                  0x00000010
#define MAS2_I                  0x00000008
#define MAS2_M                  0x00000004
#define MAS2_G                  0x00000002
#define MAS2_E                  0x00000001

#define MAS3_RPN_MASK           0xFFFFFC00
#define MAS3_RPN(n)             ((n) & MAS3_RPN_MASK)
#define MAS3_U0                 0x00000200
#define MAS3_U1                 0x00000100
#define MAS3_U2                 0x00000080
#define MAS3_U3                 0x00000040
#define MAS3_UX                 0x00000020
#define MAS3_SX                 0x00000010
#define MAS3_UW                 0x00000008
#define MAS3_SW                 0x00000004
#define MAS3_UR                 0x00000002
#define MAS3_SR                 0x00000001
/** @} */

/**
 * @name    BUCSR registers definitions
 * @{
 */
#define BUCSR_BPEN              0x00000001
#define BUCSR_BPRED_MASK        0x00000006
#define BUCSR_BPRED_0           0x00000000
#define BUCSR_BPRED_1           0x00000002
#define BUCSR_BPRED_2           0x00000004
#define BUCSR_BPRED_3           0x00000006
#define BUCSR_BALLOC_MASK       0x00000030
#define BUCSR_BALLOC_0          0x00000000
#define BUCSR_BALLOC_1          0x00000010
#define BUCSR_BALLOC_2          0x00000020
#define BUCSR_BALLOC_3          0x00000030
#define BUCSR_BALLOC_BFI        0x00000200
/** @} */

/**
 * @name    LICSR1 registers definitions
 * @{
 */
#define LICSR1_ICE              0x00000001
#define LICSR1_ICINV            0x00000002
#define LICSR1_ICORG            0x00000010
/** @} */

/**
 * @name   MSR register definitions
 * @{
 */
#define MSR_UCLE                0x04000000
#define MSR_SPE                 0x02000000
#define MSR_WE                  0x00040000
#define MSR_CE                  0x00020000
#define MSR_EE                  0x00008000
#define MSR_PR                  0x00004000
#define MSR_FP                  0x00002000
#define MSR_ME                  0x00001000
#define MSR_FE0                 0x00000800
#define MSR_DE                  0x00000200
#define MSR_FE1                 0x00000100
#define MSR_IS                  0x00000020
#define MSR_DS                  0x00000010
#define MSR_RI                  0x00000002
/** @} */

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*
 * TLB default settings.
 */
#define TLB0_MAS0               (MAS0_TBLMAS_TBL | MAS0_ESEL(0))
#define TLB0_MAS1               (MAS1_VALID | MAS1_IPROT | MAS1_TSISE_256K)
#define TLB0_MAS2               (MAS2_EPN(0x40000000) | MAS2_VLE)
#define TLB0_MAS3               (MAS3_RPN(0x40000000) |                     \
                                 MAS3_UX | MAS3_SX | MAS3_UW | MAS3_SW |    \
                                 MAS3_UR | MAS3_SR)

#define TLB1_MAS0               (MAS0_TBLMAS_TBL | MAS0_ESEL(1))
#define TLB1_MAS1               (MAS1_VALID | MAS1_IPROT | MAS1_TSISE_4M)
#define TLB1_MAS2               (MAS2_EPN(0x00000000) | MAS2_VLE)
#define TLB1_MAS3               (MAS3_RPN(0x00000000) |                     \
                                 MAS3_UX | MAS3_SX | MAS3_UW | MAS3_SW |    \
                                 MAS3_UR | MAS3_SR)

#define TLB2_MAS0               (MAS0_TBLMAS_TBL | MAS0_ESEL(2))
#define TLB2_MAS1               (MAS1_VALID | MAS1_IPROT | MAS1_TSISE_1M)
#define TLB2_MAS2               (MAS2_EPN(0xC3F00000) | MAS2_I)
#define TLB2_MAS3               (MAS3_RPN(0xC3F00000) |                     \
                                 MAS3_UW | MAS3_SW | MAS3_UR | MAS3_SR)

#define TLB3_MAS0               (MAS0_TBLMAS_TBL | MAS0_ESEL(3))
#define TLB3_MAS1               (MAS1_VALID | MAS1_IPROT | MAS1_TSISE_1M)
#define TLB3_MAS2               (MAS2_EPN(0xFFE00000) | MAS2_I)
#define TLB3_MAS3               (MAS3_RPN(0xFFE00000) |                     \
                                 MAS3_UW | MAS3_SW | MAS3_UR | MAS3_SR)

#define TLB4_MAS0               (MAS0_TBLMAS_TBL | MAS0_ESEL(4))
#define TLB4_MAS1               (MAS1_VALID | MAS1_IPROT | MAS1_TSISE_1M)
#define TLB4_MAS2               (MAS2_EPN(0xFFF00000) | MAS2_I)
#define TLB4_MAS3               (MAS3_RPN(0xFFF00000) |                     \
                                 MAS3_UW | MAS3_SW | MAS3_UR | MAS3_SR)

/*
 * BUCSR default settings.
 */
#if !defined(BOOT_BUCSR_DEFAULT) || defined(__DOXYGEN__)
#define BOOT_BUCSR_DEFAULT      (BUCSR_BPEN | BUCSR_BPRED_0 |               \
                                 BUCSR_BALLOC_0 | BUCSR_BALLOC_BFI)
#endif

/*
 * LICSR1 default settings.
 */
#if !defined(BOOT_LICSR1_DEFAULT) || defined(__DOXYGEN__)
#define BOOT_LICSR1_DEFAULT     (LICSR1_ICE | LICSR1_ICORG)
#endif

/*
 * MSR default settings.
 */
#if !defined(BOOT_MSR_DEFAULT) || defined(__DOXYGEN__)
#define BOOT_MSR_DEFAULT        (MSR_SPE | MSR_WE | MSR_CE | MSR_ME)
#endif

/*
 * Boot default settings.
 */
#if !defined(BOOT_PERFORM_CORE_INIT) || defined(__DOXYGEN__)
#define BOOT_PERFORM_CORE_INIT  1
#endif

/*
 * VLE mode default settings.
 */
#if !defined(BOOT_USE_VLE) || defined(__DOXYGEN__)
#define BOOT_USE_VLE            1
#endif

/*
 * RAM relocation flag.
 */
#if !defined(BOOT_RELOCATE_IN_RAM) || defined(__DOXYGEN__)
#define BOOT_RELOCATE_IN_RAM    0
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* BOOT_H */

/** @} */
