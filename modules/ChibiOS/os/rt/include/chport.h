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
 * @file    rt/include/chport.h
 * @brief   Port wrapper header.
 *
 * @addtogroup port_wrapper
 * @details This module performs checks on the information exported by
 *          the port layer. The port layer is checked at compile time
 *          in order to make sure that it exports all the required macros
 *          and definitions.
 * @note    This module does not export any functionality.
 * @{
 */

#ifndef CHPORT_H
#define CHPORT_H

/* Inclusion of the port layer.*/
#include "chcore.h"

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Required macros checks.*/
#if !defined(PORT_COMPILER_NAME)
#error "PORT_COMPILER_NAME not defined in chcore.h"
#endif

#if !defined(PORT_IDLE_THREAD_STACK_SIZE)
#error "PORT_IDLE_THREAD_STACK_SIZE not defined in chcore.h"
#endif

#if !defined(PORT_INT_REQUIRED_STACK)
#error "PORT_INT_REQUIRED_STACK not defined in chcore.h"
#endif

#if !defined(PORT_SUPPORTS_RT)
#error "PORT_SUPPORTS_RT not defined in chcore.h"
#endif

#if !defined(PORT_NATURAL_ALIGN)
#error "PORT_NATURAL_ALIGN not defined in chcore.h"
#endif

#if !defined(PORT_STACK_ALIGN)
#error "PORT_STACK_ALIGN not defined in chcore.h"
#endif

#if !defined(PORT_WORKING_AREA_ALIGN)
#error "PORT_WORKING_AREA_ALIGN not defined in chcore.h"
#endif

#if !defined(PORT_ARCHITECTURE_NAME)
#error "PORT_ARCHITECTURE_NAME not defined in chcore.h"
#endif

#if !defined(PORT_CORE_VARIANT_NAME)
#error "PORT_CORE_VARIANT_NAME not defined in chcore.h"
#endif

#if !defined(PORT_INFO)
#error "PORT_INFO not defined in chcore.h"
#endif

#if !defined(PORT_IRQ_IS_VALID_PRIORITY)
#error "PORT_IRQ_IS_VALID_PRIORITY not defined in chcore.h"
#endif

#if !defined(PORT_IRQ_IS_VALID_KERNEL_PRIORITY)
#error "PORT_IRQ_IS_VALID_KERNEL_PRIORITY not defined in chcore.h"
#endif

#if !defined(PORT_SETUP_CONTEXT)
#error "PORT_SETUP_CONTEXT not defined in chcore.h"
#endif

#if !defined(PORT_WA_SIZE)
#error "PORT_WA_SIZE not defined in chcore.h"
#endif

#if !defined(PORT_IRQ_PROLOGUE)
#error "PORT_IRQ_PROLOGUE not defined in chcore.h"
#endif

#if !defined(PORT_IRQ_EPILOGUE)
#error "PORT_IRQ_EPILOGUE not defined in chcore.h"
#endif

#if !defined(PORT_IRQ_HANDLER)
#error "PORT_IRQ_HANDLER not defined in chcore.h"
#endif

#if !defined(PORT_FAST_IRQ_HANDLER)
#error "PORT_FAST_IRQ_HANDLER not defined in chcore.h"
#endif

/* Just in case the port layer does not export the following definitions.*/
#if !defined(PORT_CORES_NUMBER)
#define PORT_CORES_NUMBER                   1
#endif

#if (PORT_CORES_NUMBER < 1) || (PORT_CORES_NUMBER > 64)
#error "invalid PORT_CORES_NUMBER value"
#endif

#if PORT_CORES_NUMBER == 1
#if CH_CFG_SMP_MODE != FALSE
#error "this port does not support SMP"
#endif
#endif

/* Recursive locks port capability assessed.*/
#if defined(port_get_lock_status) && defined(port_is_locked)
#define CH_PORT_SUPPORTS_RECURSIVE_LOCKS    TRUE
#else
#define CH_PORT_SUPPORTS_RECURSIVE_LOCKS    FALSE
#endif

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

#endif /* CHPORT_H */

/** @} */
