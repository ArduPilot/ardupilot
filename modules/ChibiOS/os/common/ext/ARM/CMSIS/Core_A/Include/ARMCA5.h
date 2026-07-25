/******************************************************************************
 * @file     ARMCA5.h
 * @brief    CMSIS Cortex-A5 Core Peripheral Access Layer Header File 
 * @version  V1.00
 * @data     16 Mar 2017
 *
 * @note
 *
 ******************************************************************************/
/*
 * Copyright (c) 2009-2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __ARMCA5_H__
#define __ARMCA5_H__

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum IRQn
{
/******  SGI Interrupts Numbers                 ****************************************/
  SGI0_IRQn            =  0,
  SGI1_IRQn            =  1,
  SGI2_IRQn            =  2,
  SGI3_IRQn            =  3,
  SGI4_IRQn            =  4,
  SGI5_IRQn            =  5,
  SGI6_IRQn            =  6,
  SGI7_IRQn            =  7,
  SGI8_IRQn            =  8,
  SGI9_IRQn            =  9,
  SGI10_IRQn           = 10,
  SGI11_IRQn           = 11,
  SGI12_IRQn           = 12,
  SGI13_IRQn           = 13,
  SGI14_IRQn           = 14,
  SGI15_IRQn           = 15,

/******  Cortex-A5 Processor Exceptions Numbers ****************************************/
  GlobalTimer_IRQn     = 27,        /*!< Global Timer Interrupt                        */
  PrivTimer_IRQn       = 29,        /*!< Private Timer Interrupt                       */
  PrivWatchdog_IRQn    = 30,        /*!< Private Watchdog Interrupt                    */

/******  Platform Exceptions Numbers ***************************************************/
  Watchdog_IRQn        = 32,        /*!< SP805 Interrupt        */
  Timer0_IRQn          = 34,        /*!< SP804 Interrupt        */
  Timer1_IRQn          = 35,        /*!< SP804 Interrupt        */
  RTClock_IRQn         = 36,        /*!< PL031 Interrupt        */
  UART0_IRQn           = 37,        /*!< PL011 Interrupt        */
  UART1_IRQn           = 38,        /*!< PL011 Interrupt        */
  UART2_IRQn           = 39,        /*!< PL011 Interrupt        */
  UART3_IRQn           = 40,        /*!< PL011 Interrupt        */
  MCI0_IRQn            = 41,        /*!< PL180 Interrupt (1st)  */
  MCI1_IRQn            = 42,        /*!< PL180 Interrupt (2nd)  */
  AACI_IRQn            = 43,        /*!< PL041 Interrupt        */
  Keyboard_IRQn        = 44,        /*!< PL050 Interrupt        */
  Mouse_IRQn           = 45,        /*!< PL050 Interrupt        */
  CLCD_IRQn            = 46,        /*!< PL111 Interrupt        */
  Ethernet_IRQn        = 47,        /*!< SMSC_91C111 Interrupt  */
  VFS2_IRQn            = 73,        /*!< VFS2 Interrupt         */
} IRQn_Type;

/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/

/* Peripheral and RAM base address */
#define VE_A5_MP_FLASH_BASE0                  (0x00000000UL)                        /*!< (FLASH0    ) Base Address */
#define VE_A5_MP_FLASH_BASE1                  (0x08000000UL)                        /*!< (FLASH1    ) Base Address */
#define VE_A5_MP_PERIPH_BASE                  (0x18000000UL)                        /*!< (Peripheral) Base Address */
#define VE_A5_MP_SRAM_BASE                    (0x2E000000UL)                        /*!< (SRAM      ) Base Address */
#define VE_A5_MP_DRAM_BASE                    (0x80000000UL)                        /*!< (DRAM      ) Base Address */
#define VE_A5_MP_VRAM_BASE                    (0x18000000UL)                        /*!< (VRAM      ) Base Address */
#define VE_A5_MP_ETHERNET_BASE                (0x02000000UL + VE_A5_MP_PERIPH_BASE) /*!< (ETHERNET  ) Base Address */
#define VE_A5_MP_USB_BASE                     (0x03000000UL + VE_A5_MP_PERIPH_BASE) /*!< (USB       ) Base Address */
#define VE_A5_MP_DAP_BASE                     (0x1C000000UL)                        /*!< (DAP       ) Base Address */
#define VE_A5_MP_SYSTEM_REG_BASE              (0x00010000UL + 0x1C000000UL)         /*!< (SYSTEM REG) Base Address */
#define VE_A5_MP_SERIAL_BASE                  (0x00030000UL + 0x1C000000UL)         /*!< (SERIAL    ) Base Address */
#define VE_A5_MP_AACI_BASE                    (0x00040000UL + 0x1C000000UL)         /*!< (AACI      ) Base Address */
#define VE_A5_MP_MMCI_BASE                    (0x00050000UL + 0x1C000000UL)         /*!< (MMCI      ) Base Address */
#define VE_A5_MP_KMI0_BASE                    (0x00060000UL + 0x1C000000UL)         /*!< (KMI0      ) Base Address */
#define VE_A5_MP_UART_BASE                    (0x00090000UL + 0x1C000000UL)         /*!< (UART      ) Base Address */
#define VE_A5_MP_WDT_BASE                     (0x000F0000UL + 0x1C000000UL)         /*!< (WDT       ) Base Address */
#define VE_A5_MP_TIMER_BASE                   (0x00110000UL + 0x1C000000UL)         /*!< (TIMER     ) Base Address */
#define VE_A5_MP_DVI_BASE                     (0x00160000UL + 0x1C000000UL)         /*!< (DVI       ) Base Address */
#define VE_A5_MP_RTC_BASE                     (0x00170000UL + 0x1C000000UL)         /*!< (RTC       ) Base Address */
#define VE_A5_MP_UART4_BASE                   (0x001B0000UL + 0x1C000000UL)         /*!< (UART4     ) Base Address */
#define VE_A5_MP_CLCD_BASE                    (0x001F0000UL + 0x1C000000UL)         /*!< (CLCD      ) Base Address */
#define VE_A5_MP_GIC_DISTRIBUTOR_BASE         (0x00001000UL + 0x2C000000UL)         /*!< (GIC DIST  ) Base Address */
#define VE_A5_MP_GIC_INTERFACE_BASE           (0x00000100UL + 0x2C000000UL)         /*!< (GIC CPU IF) Base Address */
#define VE_A5_MP_PRIVATE_TIMER                (0x00000600UL + 0x2C000000UL)         /*!< (PTIM      ) Base Address */
#define GIC_DISTRIBUTOR_BASE                  VE_A5_MP_GIC_DISTRIBUTOR_BASE
#define GIC_INTERFACE_BASE                    VE_A5_MP_GIC_INTERFACE_BASE
#define TIMER_BASE                            VE_A5_MP_PRIVATE_TIMER

//The VE-A5 model implements L1 cache as architecturally defined, but does not implement L2 cache.
//Do not enable the L2 cache if you are running RTX on a VE-A5 model as it may cause a data abort.
#define VE_A5_MP_PL310_BASE                   (0x00A00000UL)                        /*!< (L2C-310   ) Base Address */
#define L2C_310_BASE                          VE_A5_MP_PL310_BASE

/* --------  Configuration of the Cortex-A5 Processor and Core Peripherals  ------- */
#define __CA_REV        0x0000U    /* Core revision r0p0                            */
#define __CORTEX_A           5U    /* Cortex-A5 Core                                */
#define __FPU_PRESENT        1U    /* FPU present                                   */
#define __GIC_PRESENT        1U    /* GIC present                                   */
#define __TIM_PRESENT        1U    /* TIM present                                   */
#define __L2C_PRESENT        1U    /* L2C present                                   */

#include "core_ca.h"
#include <system_ARMCA5.h>

#ifdef __cplusplus
}
#endif

#endif  // __ARMCA5_H__
