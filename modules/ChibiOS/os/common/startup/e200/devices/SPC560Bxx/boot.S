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
 * @file    SPC560Bxx/boot.s
 * @brief   SPC560Bxx boot-related code.
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

        .long       0x015A0000
        .long       _reset_address

        .align      2
        .globl      _reset_address
        .type       _reset_address, @function
_reset_address:
#if BOOT_PERFORM_CORE_INIT
        e_bl        _coreinit
#endif
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
_coreinit:
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

        /*
         * Branch prediction enabled.
         */
        e_li        r3, BOOT_BUCSR_DEFAULT
        mtspr       1013, r3       /* BUCSR */

        se_blr
#endif /* BOOT_PERFORM_CORE_INIT */

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

        se_blr

        .section    .ivors, "ax"

        .globl      IVORS
IVORS:
        e_b         _IVOR0
        .align      4
        e_b         _IVOR1
        .align      4
        e_b         _IVOR2
        .align      4
        e_b         _IVOR3
        .align      4
        e_b         _IVOR4
        .align      4
        e_b         _IVOR5
        .align      4
        e_b         _IVOR6
        .align      4
        e_b         _IVOR7
        .align      4
        e_b         _IVOR8
        .align      4
        e_b         _IVOR9
        .align      4
        e_b         _IVOR10
        .align      4
        e_b         _IVOR11
        .align      4
        e_b         _IVOR12
        .align      4
        e_b         _IVOR13
        .align      4
        e_b         _IVOR14
        .align      4
        e_b         _IVOR15

        .section    .handlers, "ax"

        /*
         * Default IVOR handlers.
         */
        .align      2
        .weak       _IVOR0,  _IVOR1,  _IVOR2,  _IVOR3,  _IVOR4,  _IVOR5
        .weak       _IVOR6,  _IVOR7,  _IVOR8,  _IVOR9,  _IVOR10, _IVOR11
        .weak       _IVOR12, _IVOR13, _IVOR14, _IVOR15
_IVOR0:
_IVOR1:
_IVOR2:
_IVOR3:
_IVOR5:
_IVOR6:
_IVOR7:
_IVOR8:
_IVOR9:
_IVOR11:
_IVOR12:
_IVOR13:
_IVOR14:
_IVOR15:
        .global     _unhandled_exception
_unhandled_exception:
        se_b        _unhandled_exception

#endif /* !defined(__DOXYGEN__) */

/** @} */
