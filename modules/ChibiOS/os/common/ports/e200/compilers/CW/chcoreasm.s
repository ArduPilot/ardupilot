/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    e200/compilers/GCC/chcoreasm.s
 * @brief   Power Architecture port low level code.
 *
 * @addtogroup PPC_GCC_CORE
 * @{
 */

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#if !defined(FALSE) || defined(__DOXYGEN__)
#define FALSE                               0
#endif

#if !defined(TRUE) || defined(__DOXYGEN__)
#define TRUE                                1
#endif

/*===========================================================================*/
/* Code section.                                                             */
/*===========================================================================*/

/*
 * Imports the PPC configuration headers.
 */
#define _FROM_ASM_
#include "chlicense.h"
#include "chconf.h"
#include "chcore.h"

#if !defined(__DOXYGEN__)

/*
 * RTOS-specific context offset.
 */
#if defined(_CHIBIOS_RT_CONF_)
#define CONTEXT_OFFSET  12

#elif defined(_CHIBIOS_NIL_CONF_)
#define CONTEXT_OFFSET  0

#else
#error "invalid chconf.h"
#endif

#if defined(_CHIBIOS_RT_CONF_)
        .extern     chThdExit
#endif

#if PPC_USE_VLE == TRUE
        .section    .text_vle, 16

        .align      2
        .globl      _port_switch
        .type       _port_switch, @function
_port_switch:
        e_subi      r1, r1, 80
        se_mflr     r0
        e_stw       r0, 84(r1)
        mfcr        r0
        se_stw      r0, 0(r1)
        e_stmw      r14, 4(r1)

        se_stw      r1, CONTEXT_OFFSET(r4)
        se_lwz      r1, CONTEXT_OFFSET(r3)

        e_lmw       r14, 4(r1)
        se_lwz      r0, 0(r1)
        mtcr        r0
        e_lwz       r0, 84(r1)
        se_mtlr     r0
        e_addi      r1, r1, 80
        se_blr

        .align      2
        .globl      _port_thread_start
        .type       _port_thread_start, @function
_port_thread_start:
#if CH_DBG_SYSTEM_STATE_CHECK
        bl          _dbg_check_unlock
#endif
#if CH_DBG_STATISTICS
        bl          _stats_stop_measure_crit_thd
#endif
        wrteei      1
        mr          r3, r31
        mtctr       r30
        se_bctrl
        se_li       r0, 0
        e_bl        chThdExit

#else /* PPC_USE_VLE == FALSE */

#error "non-VLE mode not yet implemented"

#endif /* PPC_USE_VLE == FALSE */

#endif /* !defined(__DOXYGEN__) */

/** @} */
