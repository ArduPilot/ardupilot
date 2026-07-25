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
 * @file    nil/templates/chconf.h
 * @brief   Configuration file template.
 * @details A copy of this file must be placed in each project directory, it
 *          contains the application specific kernel settings.
 *
 * @addtogroup NIL_CONFIG
 * @details Kernel related settings and hooks.
 * @{
 */

#ifndef CHCONF_H
#define CHCONF_H

#define _CHIBIOS_NIL_CONF_
#define _CHIBIOS_NIL_CONF_VER_4_0_

/*===========================================================================*/
/**
 * @name Kernel parameters and options
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Maximum number of user threads in the application.
 * @note    This number is not inclusive of the idle thread which is
 *          implicitly handled.
 * @note    Set this value to be exactly equal to the number of threads you
 *          will use or you would be wasting RAM and cycles.
 * @note    This values also defines the number of available priorities
 *          (0..CH_CFG_MAX_THREADS-1).
 */
#if !defined(CH_CFG_MAX_THREADS)
#define CH_CFG_MAX_THREADS                  8
#endif

/**
 * @brief   Auto starts threads when @p chSysInit() is invoked.
 */
#if !defined(CH_CFG_AUTOSTART_THREADS)
#define CH_CFG_AUTOSTART_THREADS            TRUE
#endif

/** @} */

/*===========================================================================*/
/**
 * @name System timer settings
 * @{
 */
/*===========================================================================*/

/**
 * @brief   System time counter resolution.
 * @note    Allowed values are 16 or 32 bits.
 */
#if !defined(CH_CFG_ST_RESOLUTION)
#define CH_CFG_ST_RESOLUTION                32
#endif

/**
 * @brief   System tick frequency.
 * @note    This value together with the @p CH_CFG_ST_RESOLUTION
 *          option defines the maximum amount of time allowed for
 *          timeouts.
 */
#if !defined(CH_CFG_ST_FREQUENCY)
#define CH_CFG_ST_FREQUENCY                 1000
#endif

/**
 * @brief   Time delta constant for the tick-less mode.
 * @note    If this value is zero then the system uses the classic
 *          periodic tick. This value represents the minimum number
 *          of ticks that is safe to specify in a timeout directive.
 *          The value one is not valid, timeouts are rounded up to
 *          this value.
 */
#if !defined(CH_CFG_ST_TIMEDELTA)
#define CH_CFG_ST_TIMEDELTA                 2
#endif

/** @} */

/*===========================================================================*/
/**
 * @name Subsystem options
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Threads synchronization APIs.
 * @details If enabled then the @p chThdWait() function is included in
 *          the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_WAITEXIT)
#define CH_CFG_USE_WAITEXIT                 TRUE
#endif

/**
 * @brief   Semaphores APIs.
 * @details If enabled then the Semaphores APIs are included in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_SEMAPHORES)
#define CH_CFG_USE_SEMAPHORES               TRUE
#endif

/**
 * @brief   Mutexes APIs.
 * @details If enabled then the mutexes APIs are included in the kernel.
 *
 * @note    Feature not currently implemented.
 * @note    The default is @p FALSE.
 */
#if !defined(CH_CFG_USE_MUTEXES)
#define CH_CFG_USE_MUTEXES                  FALSE
#endif

/**
 * @brief   Events Flags APIs.
 * @details If enabled then the event flags APIs are included in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_EVENTS)
#define CH_CFG_USE_EVENTS                   TRUE
#endif

/**
 * @brief   Synchronous Messages APIs.
 * @details If enabled then the synchronous messages APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_MESSAGES)
#define CH_CFG_USE_MESSAGES                 TRUE
#endif

/** @} */

/*===========================================================================*/
/**
 * @name OSLIB options
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Mailboxes APIs.
 * @details If enabled then the asynchronous messages (mailboxes) APIs are
 *          included in the kernel.
 *
 * @note    The default is @p TRUE.
 * @note    Requires @p CH_CFG_USE_SEMAPHORES.
 */
#if !defined(CH_CFG_USE_MAILBOXES)
#define CH_CFG_USE_MAILBOXES                TRUE
#endif

/**
 * @brief   Memory checks APIs.
 * @details If enabled then the memory checks APIs are included in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_MEMCHECKS)
#define CH_CFG_USE_MEMCHECKS                TRUE
#endif

/**
 * @brief   Core Memory Manager APIs.
 * @details If enabled then the core memory manager APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_MEMCORE)
#define CH_CFG_USE_MEMCORE                  TRUE
#endif

/**
 * @brief   Managed RAM size.
 * @details Size of the RAM area to be managed by the OS. If set to zero
 *          then the whole available RAM is used. The core memory is made
 *          available to the heap allocator and/or can be used directly through
 *          the simplified core memory allocator.
 *
 * @note    In order to let the OS manage the whole RAM the linker script must
 *          provide the @p __heap_base__ and @p __heap_end__ symbols.
 * @note    Requires @p CH_CFG_USE_MEMCORE.
 */
#if !defined(CH_CFG_MEMCORE_SIZE)
#define CH_CFG_MEMCORE_SIZE                 0
#endif

/**
 * @brief   Heap Allocator APIs.
 * @details If enabled then the memory heap allocator APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_HEAP)
#define CH_CFG_USE_HEAP                     TRUE
#endif

/**
 * @brief   Memory Pools Allocator APIs.
 * @details If enabled then the memory pools allocator APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_MEMPOOLS)
#define CH_CFG_USE_MEMPOOLS                 TRUE
#endif

/**
 * @brief  Objects FIFOs APIs.
 * @details If enabled then the objects FIFOs APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_OBJ_FIFOS)
#define CH_CFG_USE_OBJ_FIFOS                TRUE
#endif

/**
 * @brief   Pipes APIs.
 * @details If enabled then the pipes APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_PIPES)
#define CH_CFG_USE_PIPES                    TRUE
#endif

/**
 * @brief   Objects Caches APIs.
 * @details If enabled then the objects caches APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_OBJ_CACHES)
#define CH_CFG_USE_OBJ_CACHES               TRUE
#endif

/**
 * @brief   Delegate threads APIs.
 * @details If enabled then the delegate threads APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_DELEGATES)
#define CH_CFG_USE_DELEGATES                TRUE
#endif

/**
 * @brief   Jobs Queues APIs.
 * @details If enabled then the jobs queues APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_CFG_USE_JOBS)
#define CH_CFG_USE_JOBS                     TRUE
#endif

/** @} */

/*===========================================================================*/
/**
 * @name Objects factory options
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Objects Factory APIs.
 * @details If enabled then the objects factory APIs are included in the
 *          kernel.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(CH_CFG_USE_FACTORY)
#define CH_CFG_USE_FACTORY                  TRUE
#endif

/**
 * @brief   Maximum length for object names.
 * @details If the specified length is zero then the name is stored by
 *          pointer but this could have unintended side effects.
 */
#if !defined(CH_CFG_FACTORY_MAX_NAMES_LENGTH)
#define CH_CFG_FACTORY_MAX_NAMES_LENGTH     8
#endif

/**
 * @brief   Enables the registry of generic objects.
 */
#if !defined(CH_CFG_FACTORY_OBJECTS_REGISTRY)
#define CH_CFG_FACTORY_OBJECTS_REGISTRY     TRUE
#endif

/**
 * @brief   Enables factory for generic buffers.
 */
#if !defined(CH_CFG_FACTORY_GENERIC_BUFFERS)
#define CH_CFG_FACTORY_GENERIC_BUFFERS      TRUE
#endif

/**
 * @brief   Enables factory for semaphores.
 */
#if !defined(CH_CFG_FACTORY_SEMAPHORES)
#define CH_CFG_FACTORY_SEMAPHORES           TRUE
#endif

/**
 * @brief   Enables factory for mailboxes.
 */
#if !defined(CH_CFG_FACTORY_MAILBOXES)
#define CH_CFG_FACTORY_MAILBOXES            TRUE
#endif

/**
 * @brief   Enables factory for objects FIFOs.
 */
#if !defined(CH_CFG_FACTORY_OBJ_FIFOS)
#define CH_CFG_FACTORY_OBJ_FIFOS            TRUE
#endif

/**
 * @brief   Enables factory for Pipes.
 */
#if !defined(CH_CFG_FACTORY_PIPES)
#define CH_CFG_FACTORY_PIPES                TRUE
#endif

/** @} */

/*===========================================================================*/
/**
 * @name Debug options
 * @{
 */
/*===========================================================================*/

/**
 * @brief   Debug option, kernel statistics.
 *
 * @note    Feature not currently implemented.
 * @note    The default is @p FALSE.
 */
#if !defined(CH_DBG_STATISTICS)
#define CH_DBG_STATISTICS                   FALSE
#endif

/**
 * @brief   Debug option, system state check.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(CH_DBG_SYSTEM_STATE_CHECK)
#define CH_DBG_SYSTEM_STATE_CHECK           FALSE
#endif

/**
 * @brief   Debug option, parameters checks.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(CH_DBG_ENABLE_CHECKS)
#define CH_DBG_ENABLE_CHECKS                FALSE
#endif

/**
 * @brief   System assertions.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(CH_DBG_ENABLE_ASSERTS)
#define CH_DBG_ENABLE_ASSERTS               FALSE
#endif

/**
 * @brief   Stack check.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(CH_DBG_ENABLE_STACK_CHECK)
#define CH_DBG_ENABLE_STACK_CHECK           FALSE
#endif

/** @} */

/*===========================================================================*/
/**
 * @name Kernel hooks
 * @{
 */
/*===========================================================================*/

/**
 * @brief   System initialization hook.
 */
#define CH_CFG_SYSTEM_INIT_HOOK() {                                         \
}

/**
 * @brief   Threads descriptor structure extension.
 * @details User fields added to the end of the @p thread_t structure.
 */
#define CH_CFG_THREAD_EXT_FIELDS                                            \
  /* Add threads custom fields here.*/

/**
 * @brief   Threads initialization hook.
 */
#define CH_CFG_THREAD_EXT_INIT_HOOK(tr) {                                   \
  /* Add custom threads initialization code here.*/                         \
}

/**
 * @brief   Threads finalization hook.
 * @details User finalization code added to the @p chThdExit() API.
 */
#define CH_CFG_THREAD_EXIT_HOOK(tp) {}

/**
 * @brief   Idle thread enter hook.
 * @note    This hook is invoked within a critical zone, no OS functions
 *          should be invoked from here.
 * @note    This macro can be used to activate a power saving mode.
 */
#define CH_CFG_IDLE_ENTER_HOOK() {                                          \
}

/**
 * @brief   Idle thread leave hook.
 * @note    This hook is invoked within a critical zone, no OS functions
 *          should be invoked from here.
 * @note    This macro can be used to deactivate a power saving mode.
 */
#define CH_CFG_IDLE_LEAVE_HOOK() {                                          \
}

/**
 * @brief   System halt hook.
 */
#define CH_CFG_SYSTEM_HALT_HOOK(reason) {                                   \
}

/** @} */

/*===========================================================================*/
/* Port-specific settings (override port settings defaulted in nilcore.h).   */
/*===========================================================================*/

#endif  /* CHCONF_H */

/** @} */
