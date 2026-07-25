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
 * @file    common/ARMCMx/nvic.h
 * @brief   Cortex-Mx NVIC support macros and structures.
 *
 * @addtogroup COMMON_ARMCMx_NVIC
 * @{
 */

#ifndef NVIC_H
#define NVIC_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name System vectors numbers
 * @{
 */
#define HANDLER_MEM_MANAGE      0       /**< MEM MANAGE vector id.          */
#define HANDLER_BUS_FAULT       1       /**< BUS FAULT vector id.           */
#define HANDLER_USAGE_FAULT     2       /**< USAGE FAULT vector id.         */
#define HANDLER_RESERVED_3      3
#define HANDLER_RESERVED_4      4
#define HANDLER_RESERVED_5      5
#define HANDLER_RESERVED_6      6
#define HANDLER_SVCALL          7       /**< SVCALL vector id.              */
#define HANDLER_DEBUG_MONITOR   8       /**< DEBUG MONITOR vector id.       */
#define HANDLER_RESERVED_9      9
#define HANDLER_PENDSV          10      /**< PENDSV vector id.              */
#define HANDLER_SYSTICK         11      /**< SYS TCK vector id.             */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Priority level to priority mask conversion macro.
 */
#define NVIC_PRIORITY_MASK(prio) ((prio) << (8U - (unsigned)__NVIC_PRIO_BITS))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void nvicInit(void);
  void nvicEnableVector(uint32_t n, uint32_t prio);
  void nvicDisableVector(uint32_t n);
  void nvicSetSystemHandlerPriority(uint32_t handler, uint32_t prio);
  void nvicClearPending(uint32_t n);
  void nvicSetPending(uint32_t n);
#ifdef __cplusplus
}
#endif

#endif /* NVIC_H */

/** @} */
