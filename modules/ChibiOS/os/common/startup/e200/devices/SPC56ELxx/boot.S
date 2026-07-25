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
 * @file    SPC56ELxx/boot.s
 * @brief   SPC56ELxx boot-related code.
 *
 * @addtogroup PPC_BOOT
 * @{
 */

#include "boot.h"

#if defined(__HIGHTEC__)
#define se_bge bge
#endif

#if !defined(__DOXYGEN__)

        /* BAM record.*/
        .section    .boot, "ax"

#if BOOT_USE_VLE
        .long       0x015A0000
#else
        .long       0x005A0000
#endif
        .long       _reset_address

        .align      2
        .globl      _reset_address
        .type       _reset_address, @function
_reset_address:
        e_bl        _coreinit
        e_bl        _ivinit

#if BOOT_RELOCATE_IN_RAM
        /*
         * Image relocation in RAM.
         */
        e_lis       r4, __ram_reloc_start__@h
        e_or2i      r4, __ram_reloc_start__@l
        e_lis       r5, __ram_reloc_dest__@h
        e_or2i      r5, __ram_reloc_dest__@l
        e_lis       r6, __ram_reloc_end__@h
        e_or2i      r6, r6, __ram_reloc_end__@l
.relloop:
        se_cmpl     r4, r6
        se_bge      .relend
        se_lwz      r7, 0(r4)
        se_addi     r4, 4
        se_stw      r7, 0(r5)
        se_addi     r5, 4
        se_b        .relloop
.relend:
        e_lis       r3, _boot_address@h
        e_or2i      r3, _boot_address@l
        mtctr       r3
        se_bctrl
#else
        e_b         _boot_address
#endif

#if BOOT_PERFORM_CORE_INIT
        .align      2
_ramcode:
        tlbwe
        se_isync
        se_blr
#endif /* BOOT_PERFORM_CORE_INIT */

        .align      2
_coreinit:
#if BOOT_PERFORM_CORE_INIT
        /*
         * Invalidating all TLBs except TLB0.
         */
        e_lis       r3, 0
        mtspr       625, r3        /* MAS1 */
        mtspr       626, r3        /* MAS2 */
        mtspr       627, r3        /* MAS3 */
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(1))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(2))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(3))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(4))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(5))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(6))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(7))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(8))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(9))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(10))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(11))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(12))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(13))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(14))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe
        e_lis       r3, (MAS0_TBLMAS_TBL | MAS0_ESEL(15))@h
        mtspr       624, r3        /* MAS0 */
        tlbwe

        /*
         * TLB1 allocated to internal RAM.
         */
        e_lis       r3, TLB1_MAS0@h
        mtspr       624, r3        /* MAS0 */
        e_lis       r3, TLB1_MAS1@h
        e_or2i      r3, TLB1_MAS1@l
        mtspr       625, r3        /* MAS1 */
        e_lis       r3, TLB1_MAS2@h
        e_or2i      r3, TLB1_MAS2@l
        mtspr       626, r3        /* MAS2 */
        e_lis       r3, TLB1_MAS3@h
        e_or2i      r3, TLB1_MAS3@l
        mtspr       627, r3        /* MAS3 */
        tlbwe

        /*
         * TLB2 allocated to internal Peripherals Bridge A.
         */
        e_lis       r3, TLB2_MAS0@h
        mtspr       624, r3        /* MAS0 */
        e_lis       r3, TLB2_MAS1@h
        e_or2i      r3, TLB2_MAS1@l
        mtspr       625, r3        /* MAS1 */
        e_lis       r3, TLB2_MAS2@h
        e_or2i      r3, TLB2_MAS2@l
        mtspr       626, r3        /* MAS2 */
        e_lis       r3, TLB2_MAS3@h
        e_or2i      r3, TLB2_MAS3@l
        mtspr       627, r3        /* MAS3 */
        tlbwe

        /*
         * TLB3 allocated to internal Peripherals Bridge B.
         */
        e_lis       r3, TLB3_MAS0@h
        mtspr       624, r3        /* MAS0 */
        e_lis       r3, TLB3_MAS1@h
        e_or2i      r3, TLB3_MAS1@l
        mtspr       625, r3        /* MAS1 */
        e_lis       r3, TLB3_MAS2@h
        e_or2i      r3, TLB3_MAS2@l
        mtspr       626, r3        /* MAS2 */
        e_lis       r3, TLB3_MAS3@h
        e_or2i      r3, TLB3_MAS3@l
        mtspr       627, r3        /* MAS3 */
        tlbwe

        /*
         * TLB4 allocated to on-platform peripherals.
         */
        e_lis       r3, TLB4_MAS0@h
        mtspr       624, r3        /* MAS0 */
        e_lis       r3, TLB4_MAS1@h
        e_or2i      r3, TLB4_MAS1@l
        mtspr       625, r3        /* MAS1 */
        e_lis       r3, TLB4_MAS2@h
        e_or2i      r3, TLB4_MAS2@l
        mtspr       626, r3        /* MAS2 */
        e_lis       r3, TLB4_MAS3@h
        e_or2i      r3, TLB4_MAS3@l
        mtspr       627, r3        /* MAS3 */
        tlbwe

        /*
         * TLB5 allocated to on-platform peripherals.
         */
        e_lis       r3, TLB5_MAS0@h
        mtspr       624, r3        /* MAS0 */
        e_lis       r3, TLB5_MAS1@h
        e_or2i      r3, TLB5_MAS1@l
        mtspr       625, r3        /* MAS1 */
        e_lis       r3, TLB5_MAS2@h
        e_or2i      r3, TLB5_MAS2@l
        mtspr       626, r3        /* MAS2 */
        e_lis       r3, TLB5_MAS3@h
        e_or2i      r3, TLB5_MAS3@l
        mtspr       627, r3        /* MAS3 */
        tlbwe

        /*
         * RAM clearing, this device requires a write to all RAM location in
         * order to initialize the ECC detection hardware, this is going to
         * slow down the startup but there is no way around.
         */
        xor         r0, r0, r0
        xor         r1, r1, r1
        xor         r2, r2, r2
        xor         r3, r3, r3
        xor         r4, r4, r4
        xor         r5, r5, r5
        xor         r6, r6, r6
        xor         r7, r7, r7
        xor         r8, r8, r8
        xor         r9, r9, r9
        xor         r10, r10, r10
        xor         r11, r11, r11
        xor         r12, r12, r12
        xor         r13, r13, r13
        xor         r14, r14, r14
        xor         r15, r15, r15
        xor         r16, r16, r16
        xor         r17, r17, r17
        xor         r18, r18, r18
        xor         r19, r19, r19
        xor         r20, r20, r20
        xor         r21, r21, r21
        xor         r22, r22, r22
        xor         r23, r23, r23
        xor         r24, r24, r24
        xor         r25, r25, r25
        xor         r26, r26, r26
        xor         r27, r27, r27
        xor         r28, r28, r28
        xor         r29, r29, r29
        xor         r30, r30, r30
        xor         r31, r31, r31
        e_lis       r4, __ram_start__@h
        e_or2i      r4, __ram_start__@l
        e_lis       r5, __ram_end__@h
        e_or2i      r5, __ram_end__@l
.cleareccloop:
        se_cmpl     r4, r5
        se_bge      .cleareccend
        e_stmw      r16, 0(r4)
        e_addi      r4, r4, 64
        se_b        .cleareccloop
.cleareccend:
#endif /* BOOT_PERFORM_CORE_INIT */

        /*
         * Special function registers clearing, required in order to avoid
         * possible problems with lockstep mode.
         */
        mtcrf       0xFF, r31
        mtspr       9, r31         /* CTR     */
        mtspr       22, r31        /* DEC     */
        mtspr       26, r31        /* SRR0-1  */
        mtspr       27, r31
        mtspr       54, r31        /* DECAR   */
        mtspr       58, r31        /* CSRR0-1 */
        mtspr       59, r31
        mtspr       61, r31        /* DEAR    */
        mtspr       256, r31       /* USPRG0  */
        mtspr       272, r31       /* SPRG1-7 */
        mtspr       273, r31
        mtspr       274, r31
        mtspr       275, r31
        mtspr       276, r31
        mtspr       277, r31
        mtspr       278, r31
        mtspr       279, r31
        mtspr       285, r31       /* TBU     */
        mtspr       284, r31       /* TBL     */
#if 0
        mtspr       318, r31       /* DVC1-2  */
        mtspr       319, r31
#endif
        mtspr       562, r31       /* DBCNT */
        mtspr       570, r31       /* MCSRR0  */
        mtspr       571, r31       /* MCSRR1  */
        mtspr       604, r31       /* SPRG8-9 */
        mtspr       605, r31

#if BOOT_PERFORM_CORE_INIT
        /*
         * *Finally* the TLB0 is re-allocated to flash, note, the final phase
         * is executed from RAM.
         */
        e_lis       r3, TLB0_MAS0@h
        mtspr       624, r3        /* MAS0 */
        e_lis       r3, TLB0_MAS1@h
        e_or2i      r3, TLB0_MAS1@l
        mtspr       625, r3        /* MAS1 */
        e_lis       r3, TLB0_MAS2@h
        e_or2i      r3, TLB0_MAS2@l
        mtspr       626, r3        /* MAS2 */
        e_lis       r3, TLB0_MAS3@h
        e_or2i      r3, TLB0_MAS3@l
        mtspr       627, r3        /* MAS3 */
        mflr        r4
        e_lis       r6, _ramcode@h
        e_or2i      r6, _ramcode@l
        e_lis       r7, 0x40010000@h
        mtctr       r7
        se_lwz      r3, 0(r6)
        se_stw      r3, 0(r7)
        se_lwz      r3, 4(r6)
        se_stw      r3, 4(r7)
        se_lwz      r3, 8(r6)
        se_stw      r3, 8(r7)
        se_bctrl
        mtlr        r4
#endif /* BOOT_PERFORM_CORE_INIT */

        /*
         * Branch prediction enabled.
         */
        e_li        r3, BOOT_BUCSR_DEFAULT
        mtspr       1013, r3       /* BUCSR */

        /*
         * Cache invalidated and then enabled.
         */
        e_li        r3, LICSR1_ICINV
        mtspr       1011, r3       /* LICSR1 */
.inv:   mfspr       r3, 1011       /* LICSR1 */
        e_and2i.    r3, LICSR1_ICINV
        se_bne      .inv
        e_lis       r3, BOOT_LICSR1_DEFAULT@h
        e_or2i      r3, BOOT_LICSR1_DEFAULT@l
        mtspr       1011, r3       /* LICSR1 */

        se_blr

        /*
         * Exception vectors initialization.
         */
        .align      2
_ivinit:
        /* MSR initialization.*/
        e_lis       r3, BOOT_MSR_DEFAULT@h
        e_or2i      r3, BOOT_MSR_DEFAULT@l
        mtMSR       r3

        /* IVPR initialization.*/
        e_lis       r3, __ivpr_base__@h
        e_or2i      r3, __ivpr_base__@l
        mtIVPR      r3

        /* IVORs initialization.*/
        e_lis       r3, _unhandled_exception@h
        e_or2i      r3, _unhandled_exception@l

        mtspr       400, r3        /* IVOR0-15 */
        mtspr       401, r3
        mtspr       402, r3
        mtspr       403, r3
        mtspr       404, r3
        mtspr       405, r3
        mtspr       406, r3
        mtspr       407, r3
        mtspr       408, r3
        mtspr       409, r3
        mtspr       410, r3
        mtspr       411, r3
        mtspr       412, r3
        mtspr       413, r3
        mtspr       414, r3
        mtspr       415, r3
        mtspr       528, r3        /* IVOR32-34 */
        mtspr       529, r3
        mtspr       530, r3

        se_blr

        .section    .handlers, "ax"

        /*
         * Unhandled exceptions handler.
         */
        .weak       _unhandled_exception
        .type       _unhandled_exception, @function
_unhandled_exception:
        se_b        _unhandled_exception

#endif /* !defined(__DOXYGEN__) */

/** @} */
