/**************************************************************************//**
 * @file     ADUCM363.h
 * @brief    CMSIS Cortex-M3 Core Peripheral Access Layer Header File for
 *           Device ADUCM360
 * @version  V3.10
 * @date     23. November 2012
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.

   Portions Copyright (c) 2017-2019 Analog Devices, Inc.
   ---------------------------------------------------------------------------*/

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup ADUCM363
  * @{
  */

#ifndef __ADUCM363_H__
#define __ADUCM363_H__

#ifndef __NO_MMR_STRUCTS__
// The new style CMSIS structure definitions for MMRs clash with
// the old style defs. If the old style are required for compilation
// then set __NO_MMR_STRUCTS__ to 0x1
#define __NO_MMR_STRUCTS__ 0x0
#endif

#ifdef __cplusplus
extern "C" {
#endif 



/********************************************
** Start of section using anonymous unions **
*********************************************/

#if defined(__ARMCC_VERSION)
  #pragma push
  #pragma anon_unions
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif


 /* Interrupt Number Definition */

typedef enum {
// -------------------------  Cortex-M3 Processor Exceptions Numbers  -----------------------------
  Reset_IRQn                        = -15,  /*!<   1  Reset Vector, invoked on Power up and warm reset */
  NonMaskableInt_IRQn               = -14,  /*!<   2  Non maskable Interrupt, cannot be stopped or preempted */
  HardFault_IRQn                    = -13,  /*!<   3  Hard Fault, all classes of Fault */
  MemoryManagement_IRQn             = -12,  /*!<   4  Memory Management, MPU mismatch, including Access Violation and No Match */
  BusFault_IRQn                     = -11,  /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault */
  UsageFault_IRQn                   = -10,  /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition */
  SVCall_IRQn                       = -5,   /*!<  11  System Service Call via SVC instruction */
  DebugMonitor_IRQn                 = -4,   /*!<  12  Debug Monitor                    */
  PendSV_IRQn                       = -2,   /*!<  14  Pendable request for system service */
  SysTick_IRQn                      = -1,   /*!<  15  System Tick Timer                */
// --------------------------  ADUCM363 Specific Interrupt Numbers  ------------------------------
  WUT_IRQn                          = 0,    /*!<   0  WUT                              */
  EINT0_IRQn                        = 1,    /*!<   1  EINT0                            */
  EINT1_IRQn                        = 2,    /*!<   2  EINT1                            */
  EINT2_IRQn                        = 3,    /*!<   3  EINT2                            */
  EINT3_IRQn                        = 4,    /*!<   4  EINT3                            */
  EINT4_IRQn                        = 5,    /*!<   5  EINT4                            */
  EINT5_IRQn                        = 6,    /*!<   6  EINT5                            */
  EINT6_IRQn                        = 7,    /*!<   7  EINT6                            */
  EINT7_IRQn                        = 8,    /*!<   8  EINT7                            */
  WDT_IRQn                          = 9,    /*!<   9  WDT                              */
  TIMER0_IRQn                       = 11,   /*!<  11  TIMER0                           */
  TIMER1_IRQn                       = 12,   /*!<  12  TIMER1                           */
  ADC1_IRQn                         = 14,   /*!<  14  ADC1                             */
  SINC2_IRQn                        = 15,   /*!<  15  SINC2                            */
  FLASH_IRQn                        = 16,   /*!<  16  FLASH                            */
  UART_IRQn                        = 17,   /*!<  17  UART                             */
  SPI0_IRQn                         = 18,   /*!<  18  SPI0                             */
  SPI1_IRQn                         = 19,   /*!<  19  SPI1                             */
  I2CS_IRQn                         = 20,   /*!<  20  I2CS                             */
  I2CM_IRQn                         = 21,   /*!<  21  I2CM                             */
  DMA_ERR_IRQn                      = 22,   /*!<  22  DMA_ERR                          */
  DMA_SPI1_TX_IRQn                  = 23,   /*!<  23  DMA_SPI1_TX                      */
  DMA_SPI1_RX_IRQn                  = 24,   /*!<  24  DMA_SPI1_RX                      */
  DMA_UART_TX_IRQn                 = 25,   /*!<  25  DMA_UART_TX                      */
  DMA_UART_RX_IRQn                 = 26,   /*!<  26  DMA_UART_RX                      */
  DMA_I2CS_TX_IRQn                  = 27,   /*!<  27  DMA_I2CS_TX                      */
  DMA_I2CS_RX_IRQn                  = 28,   /*!<  28  DMA_I2CS_RX                      */
  DMA_I2CM_TX_IRQn                  = 29,   /*!<  29  DMA_I2CM_TX                      */
  DMA_I2CM_RX_IRQn                  = 30,   /*!<  30  DMA_I2CM_RX                      */
  DMA_DAC_IRQn                      = 31,   /*!<  31  DMA_DAC                          */
  DMA_ADC1_IRQn                     = 33,   /*!<  33  DMA_ADC1                         */
  DMA_SINC2_IRQn                    = 34,   /*!<  34  DMA_SINC2                        */
  DMA_SPI0_TX_IRQn                  = 35,   /*!<  35  DMA_SPI0_TX                      */
  DMA_SPI0_RX_IRQn                  = 36,   /*!<  36  DMA_SPI0_RX                      */
  DMA_UART1_TX_IRQn                 = 37,   /*!<  37  DMA_UART1_TX                      */
  DMA_UART1_RX_IRQn                 = 38,   /*!<  38  DMA_UART1_RX                      */
  DMA_UART2_TX_IRQn                 = 39,   /*!<  39  DMA_UART2_TX                      */
  DMA_UART2_RX_IRQn                 = 40,   /*!<  40  DMA_UART2_RX                      */
  PWM_TRIP_IRQn                     = 41,   /*!<  35  PWM_TRIP                         */
  PWM_PAIR0_IRQn                    = 42,   /*!<  36  PWM_PAIR0                        */
  PWM_PAIR1_IRQn                    = 43,   /*!<  37  PWM_PAIR1                        */
  PWM_PAIR2_IRQn                    = 44,   /*!<  38  PWM_PAIR2                        */
  UART1_IRQn                        = 47,   /*!<  47  UART1                             */
  UART2_IRQn                        = 48,   /*!<  48  UART2                             */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */

/* Processor and Core Peripheral Section */ /* Configuration of the Cortex-M3 Processor and Core Peripherals */

#define __CM3_REV              0x0201       /*!< Cortex-M3 Core Revision r2p0          */
#define __MPU_PRESENT             0         /*!< MPU present or not                    */
#define __NVIC_PRIO_BITS          3         /*!< Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used */
/** @} */ /* End of group Configuration_of_CMSIS */

#include <core_cm3.h>                       /*!< Cortex-M3 processor and core peripherals */
#include "system_ADuCM363.h"               /*!< ADUCM363 System                      */

/** @addtogroup Device_Peripheral_Registers
  * @{
  */



/* ADCCON[ADCEN] - Enable Bit */
#define ADCCON_ADCEN_MSK               (0x1   << 19 )
#define ADCCON_ADCEN                   (0x1   << 19 )
#define ADCCON_ADCEN_DIS               (0x0   << 19 ) /* DIS                      */
#define ADCCON_ADCEN_EN                (0x1   << 19 ) /* EN                       */

/* ADCCON[ADCCODE] - ADC Output Coding bits */
#define ADCCON_ADCCODE_MSK             (0x1   << 18 )
#define ADCCON_ADCCODE                 (0x1   << 18 )
#define ADCCON_ADCCODE_INT             (0x0   << 18 ) /* INT                      */
#define ADCCON_ADCCODE_UINT            (0x1   << 18 ) /* UINT                     */

/* ADCCON[BUFPOWN] - Negative buffer power down */
#define ADCCON_BUFPOWN_MSK             (0x1   << 17 )
#define ADCCON_BUFPOWN                 (0x1   << 17 )
#define ADCCON_BUFPOWN_DIS             (0x0   << 17 ) /* Disable powerdown - Negative buffer is enabled */
#define ADCCON_BUFPOWN_EN              (0x1   << 17 ) /* Enable powerdown - Negative buffer is disabled */

/* ADCCON[BUFPOWP] - Positive buffer power down */
#define ADCCON_BUFPOWP_MSK             (0x1   << 16 )
#define ADCCON_BUFPOWP                 (0x1   << 16 )
#define ADCCON_BUFPOWP_DIS             (0x0   << 16 ) /* Disable powerdown - Positive buffer is enabled */
#define ADCCON_BUFPOWP_EN              (0x1   << 16 ) /* Enable powerdown - Positive buffer is disabled */

/* ADCCON[BUFBYPP] - Positive buffer bypass */
#define ADCCON_BUFBYPP_MSK             (0x1   << 15 )
#define ADCCON_BUFBYPP                 (0x1   << 15 )
#define ADCCON_BUFBYPP_DIS             (0x0   << 15 ) /* DIS                      */
#define ADCCON_BUFBYPP_EN              (0x1   << 15 ) /* EN                       */

/* ADCCON[BUFBYPN] - Negative buffer bypass */
#define ADCCON_BUFBYPN_MSK             (0x1   << 14 )
#define ADCCON_BUFBYPN                 (0x1   << 14 )
#define ADCCON_BUFBYPN_DIS             (0x0   << 14 ) /* DIS                      */
#define ADCCON_BUFBYPN_EN              (0x1   << 14 ) /* EN                       */

/* ADCCON[ADCREF] - Reference selection */
#define ADCCON_ADCREF_MSK              (0x3   << 12 )
#define ADCCON_ADCREF_INTREF           (0x0   << 12 ) /* INTREF                   */
#define ADCCON_ADCREF_EXTREF           (0x1   << 12 ) /* EXTREF                   */
#define ADCCON_ADCREF_EXTREF2          (0x2   << 12 ) /* EXTREF2                  */
#define ADCCON_ADCREF_AVDDREF          (0x3   << 12 ) /* AVDDREF                  */

/* ADCCON[ADCDIAG] - Diagnostic Current bits bits */
#define ADCCON_ADCDIAG_MSK             (0x3   << 10 )
#define ADCCON_ADCDIAG_DIAG_OFF        (0x0   << 10 ) /* DIAG_OFF                 */
#define ADCCON_ADCDIAG_DIAG_POS        (0x1   << 10 ) /* DIAG_POS                 */
#define ADCCON_ADCDIAG_DIAG_NEG        (0x2   << 10 ) /* DIAG_NEG                 */
#define ADCCON_ADCDIAG_DIAG_ALL        (0x3   << 10 ) /* DIAG_ALL                 */

/* ADCCON[ADCCP] - AIN+ bits */
#define ADCCON_ADCCP_MSK               (0x1F  << 5  )
#define ADCCON_ADCCP_AIN0              (0x0   << 5  ) /* AIN0                     */
#define ADCCON_ADCCP_AIN1              (0x1   << 5  ) /* AIN1                     */
#define ADCCON_ADCCP_AIN2              (0x2   << 5  ) /* AIN2                     */
#define ADCCON_ADCCP_AIN3              (0x3   << 5  ) /* AIN3                     */
#define ADCCON_ADCCP_AIN4              (0x4   << 5  ) /* AIN4                     */
#define ADCCON_ADCCP_AIN5              (0x5   << 5  ) /* AIN5                     */
#define ADCCON_ADCCP_AIN6              (0x6   << 5  ) /* AIN6                     */
#define ADCCON_ADCCP_AIN7              (0x7   << 5  ) /* AIN7                     */
#define ADCCON_ADCCP_AIN8              (0x8   << 5  ) /* AIN8                     */
#define ADCCON_ADCCP_AIN9              (0x9   << 5  ) /* AIN9                     */
#define ADCCON_ADCCP_AIN10             (0xA   << 5  ) /* AIN10                    */
#define ADCCON_ADCCP_AIN11             (0xB   << 5  ) /* AIN11                    */
#define ADCCON_ADCCP_DAC               (0xC   << 5  ) /* DAC                      */
#define ADCCON_ADCCP_AVDD4             (0xD   << 5  ) /* AVDD4                    */
#define ADCCON_ADCCP_IOVDD4            (0xE   << 5  ) /* IOVDD4                   */
#define ADCCON_ADCCP_AGND              (0xF   << 5  ) /* AGND                     */
#define ADCCON_ADCCP_TEMP              (0x10  << 5  ) /* TEMP                     */

/* ADCCON[ADCCN] - AIN- bits */
#define ADCCON_ADCCN_MSK               (0x1F  << 0  )
#define ADCCON_ADCCN_AIN0              (0x0   << 0  ) /* AIN0                     */
#define ADCCON_ADCCN_AIN1              (0x1   << 0  ) /* AIN1                     */
#define ADCCON_ADCCN_AIN2              (0x2   << 0  ) /* AIN2                     */
#define ADCCON_ADCCN_AIN3              (0x3   << 0  ) /* AIN3                     */
#define ADCCON_ADCCN_AIN4              (0x4   << 0  ) /* AIN4                     */
#define ADCCON_ADCCN_AIN5              (0x5   << 0  ) /* AIN5                     */
#define ADCCON_ADCCN_AIN6              (0x6   << 0  ) /* AIN6                     */
#define ADCCON_ADCCN_AIN7              (0x7   << 0  ) /* AIN7                     */
#define ADCCON_ADCCN_AIN8              (0x8   << 0  ) /* AIN8                     */
#define ADCCON_ADCCN_AIN9              (0x9   << 0  ) /* AIN9                     */
#define ADCCON_ADCCN_AIN10             (0xA   << 0  ) /* AIN10                    */
#define ADCCON_ADCCN_AIN11             (0xB   << 0  ) /* AIN11                    */
#define ADCCON_ADCCN_DAC               (0xC   << 0  ) /* DAC                      */
#define ADCCON_ADCCN_AGND              (0xF   << 0  ) /* AGND                     */
#define ADCCON_ADCCN_TEMP              (0x11  << 0  ) /* TEMP                     */

/* ADCMDE[PGA] - PGA Gain Select bit */
#define ADCMDE_PGA_MSK                 (0xF   << 4  )
#define ADCMDE_PGA_G1                  (0x0   << 4  ) /* G1                       */
#define ADCMDE_PGA_G2                  (0x1   << 4  ) /* G2                       */
#define ADCMDE_PGA_G4                  (0x2   << 4  ) /* G4                       */
#define ADCMDE_PGA_G8                  (0x3   << 4  ) /* G8                       */
#define ADCMDE_PGA_G16                 (0x4   << 4  ) /* G16                      */
#define ADCMDE_PGA_G32                 (0x5   << 4  ) /* G32                      */
#define ADCMDE_PGA_G64                 (0x6   << 4  ) /* G64                      */
#define ADCMDE_PGA_G128                (0x7   << 4  ) /* G128                     */

/* ADCMDE[ADCMOD2] - ADC modulator gain of 2 control bits */
#define ADCMDE_ADCMOD2_MSK             (0x1   << 3  )
#define ADCMDE_ADCMOD2                 (0x1   << 3  )
#define ADCMDE_ADCMOD2_MOD2OFF         (0x0   << 3  ) /* MOD2OFF                  */
#define ADCMDE_ADCMOD2_MOD2ON          (0x1   << 3  ) /* MOD2ON                   */

/* ADCMDE[ADCMD] - ADC Mode bits */
#define ADCMDE_ADCMD_MSK               (0x7   << 0  )
#define ADCMDE_ADCMD_OFF               (0x0   << 0  ) /* OFF                      */
#define ADCMDE_ADCMD_CONT              (0x1   << 0  ) /* CONT                     */
#define ADCMDE_ADCMD_SINGLE            (0x2   << 0  ) /* SINGLE                   */
#define ADCMDE_ADCMD_IDLE              (0x3   << 0  ) /* IDLE                     */
#define ADCMDE_ADCMD_INTOCAL           (0x4   << 0  ) /* INTOCAL                  */
#define ADCMDE_ADCMD_INTGCAL           (0x5   << 0  ) /* INTGCAL                  */
#define ADCMDE_ADCMD_SYSOCAL           (0x6   << 0  ) /* SYSOCAL                  */
#define ADCMDE_ADCMD_SYSGCAL           (0x7   << 0  ) /* SYSGCAL                  */

/* ADCMSKI[ATHEX] - ADC Accumulator Comparator Threshold status bit mask */
#define ADCMSKI_ATHEX_MSK              (0x1   << 3  )
#define ADCMSKI_ATHEX                  (0x1   << 3  )
#define ADCMSKI_ATHEX_DIS              (0x0   << 3  ) /* DIS                      */
#define ADCMSKI_ATHEX_EN               (0x1   << 3  ) /* EN                       */

/* ADCMSKI[THEX] - ADC comparator threshold mask */
#define ADCMSKI_THEX_MSK               (0x1   << 2  )
#define ADCMSKI_THEX                   (0x1   << 2  )
#define ADCMSKI_THEX_DIS               (0x0   << 2  ) /* DIS                      */
#define ADCMSKI_THEX_EN                (0x1   << 2  ) /* EN                       */

/* ADCMSKI[OVR] - ADC overrange bit mask. */
#define ADCMSKI_OVR_MSK                (0x1   << 1  )
#define ADCMSKI_OVR                    (0x1   << 1  )
#define ADCMSKI_OVR_DIS                (0x0   << 1  ) /* DIS                      */
#define ADCMSKI_OVR_EN                 (0x1   << 1  ) /* EN                       */

/* ADCMSKI[RDY] - valid conversion result mask */
#define ADCMSKI_RDY_MSK                (0x1   << 0  )
#define ADCMSKI_RDY                    (0x1   << 0  )
#define ADCMSKI_RDY_DIS                (0x0   << 0  ) /* DIS                      */
#define ADCMSKI_RDY_EN                 (0x1   << 0  ) /* EN                       */

/* ADCFLT[CHOP] - Enables System-Chopping bits */
#define ADCFLT_CHOP_MSK                (0x1   << 15 )
#define ADCFLT_CHOP                    (0x1   << 15 )
#define ADCFLT_CHOP_OFF                (0x0   << 15 ) /* OFF                      */
#define ADCFLT_CHOP_ON                 (0x1   << 15 ) /* ON                       */

/* ADCFLT[RAVG2] - Enables a running Average-By-2 bits */
#define ADCFLT_RAVG2_MSK               (0x1   << 14 )
#define ADCFLT_RAVG2                   (0x1   << 14 )
#define ADCFLT_RAVG2_OFF               (0x0   << 14 ) /* OFF                      */
#define ADCFLT_RAVG2_ON                (0x1   << 14 ) /* ON                       */

/* ADCFLT[SINC4EN] - Enable the Sinc4 filter instead of Sinc3 filter. */
#define ADCFLT_SINC4EN_MSK             (0x1   << 12 )
#define ADCFLT_SINC4EN                 (0x1   << 12 )
#define ADCFLT_SINC4EN_DIS             (0x0   << 12 ) /* DIS                      */
#define ADCFLT_SINC4EN_EN              (0x1   << 12 ) /* EN                       */

/* ADCFLT[AF] - Averaging filter */
#define ADCFLT_AF_MSK                  (0xF   << 8  )

/* ADCFLT[NOTCH2] - Inserts a notch at FNOTCH2 */
#define ADCFLT_NOTCH2_MSK              (0x1   << 7  )
#define ADCFLT_NOTCH2                  (0x1   << 7  )
#define ADCFLT_NOTCH2_DIS              (0x0   << 7  ) /* DIS                      */
#define ADCFLT_NOTCH2_EN               (0x1   << 7  ) /* EN                       */

/* ADCFLT[SF] - SINC Filter value */
#define ADCFLT_SF_MSK                  (0x7F  << 0  )

/* TCON[EVENTEN] - Enable time capture of an event */
#define TCON_EVENTEN_MSK               (0x1   << 12 )
#define TCON_EVENTEN                   (0x1   << 12 )
#define TCON_EVENTEN_DIS               (0x0   << 12 ) /* DIS                      */
#define TCON_EVENTEN_EN                (0x1   << 12 ) /* EN                       */

/* TCON[EVENT] - Event Select, selects 1 of the available events. */
#define TCON_EVENT_MSK                 (0xF   << 8  )

/* TCON[RLD] - Timer reload on write to clear register */
#define TCON_RLD_MSK                   (0x1   << 7  )
#define TCON_RLD                       (0x1   << 7  )
#define TCON_RLD_DIS                   (0x0   << 7  ) /* DIS                      */
#define TCON_RLD_EN                    (0x1   << 7  ) /* EN                       */

/* TCON[CLK] - Clock Select */
#define TCON_CLK_MSK                   (0x3   << 5  )
#define TCON_CLK_UCLK                  (0x0   << 5  ) /* UCLK - System Clock      */
#define TCON_CLK_PCLK                  (0x1   << 5  ) /* PCLK - Peripheral clock  */
#define TCON_CLK_LFOSC                 (0x2   << 5  ) /* LFOSC - Internal 32 kHz Oscillator */
#define TCON_CLK_LFXTAL                (0x3   << 5  ) /* LFXTAL - External 32 kHz crystal */

/* TCON[ENABLE] - Enable */
#define TCON_ENABLE_MSK                (0x1   << 4  )
#define TCON_ENABLE                    (0x1   << 4  )
#define TCON_ENABLE_DIS                (0x0   << 4  ) /* DIS                      */
#define TCON_ENABLE_EN                 (0x1   << 4  ) /* EN                       */

/* TCON[MOD] - Mode */
#define TCON_MOD_MSK                   (0x1   << 3  )
#define TCON_MOD                       (0x1   << 3  )
#define TCON_MOD_FREERUN               (0x0   << 3  ) /* FREERUN                  */
#define TCON_MOD_PERIODIC              (0x1   << 3  ) /* PERIODIC                 */

/* TCON[UP] - Count-up */
#define TCON_UP_MSK                    (0x1   << 2  )
#define TCON_UP                        (0x1   << 2  )
#define TCON_UP_DIS                    (0x0   << 2  ) /* DIS                      */
#define TCON_UP_EN                     (0x1   << 2  ) /* EN                       */

/* TCON[PRE] - Prescaler */
#define TCON_PRE_MSK                   (0x3   << 0  )
#define TCON_PRE_DIV1                  (0x0   << 0  ) /* DIV1                     */
#define TCON_PRE_DIV16                 (0x1   << 0  ) /* DIV16                    */
#define TCON_PRE_DIV256                (0x2   << 0  ) /* DIV256                   */
#define TCON_PRE_DIV32768              (0x3   << 0  ) /* DIV32768 - If the selected clock source is UCLK then this setting results in a prescaler of 4. */

/* TCLRI[CAP] - Clear captured event interrupt */
#define TCLRI_CAP_MSK                  (0x1   << 1  )
#define TCLRI_CAP                      (0x1   << 1  )
#define TCLRI_CAP_CLR                  (0x1   << 1  ) /* CLR                      */

/* TCLRI[TMOUT] - Clear timeout interrupt */
#define TCLRI_TMOUT_MSK                (0x1   << 0  )
#define TCLRI_TMOUT                    (0x1   << 0  )
#define TCLRI_TMOUT_CLR                (0x1   << 0  ) /* CLR                      */

/* TSTA[CLRI] - Value updated in the timer clock domain */
#define TSTA_CLRI_MSK                  (0x1   << 7  )
#define TSTA_CLRI                      (0x1   << 7  )
#define TSTA_CLRI_CLR                  (0x0   << 7  ) /* CLR                      */
#define TSTA_CLRI_SET                  (0x1   << 7  ) /* SET                      */

/* TSTA[CON] - Ready to receive commands */
#define TSTA_CON_MSK                   (0x1   << 6  )
#define TSTA_CON                       (0x1   << 6  )
#define TSTA_CON_CLR                   (0x0   << 6  ) /* CLR                      */
#define TSTA_CON_SET                   (0x1   << 6  ) /* SET                      */

/* TSTA[CAP] - Capture event pending */
#define TSTA_CAP_MSK                   (0x1   << 1  )
#define TSTA_CAP                       (0x1   << 1  )
#define TSTA_CAP_CLR                   (0x0   << 1  ) /* CLR                      */
#define TSTA_CAP_SET                   (0x1   << 1  ) /* SET                      */

/* TSTA[TMOUT] - Time out event occurred */
#define TSTA_TMOUT_MSK                 (0x1   << 0  )
#define TSTA_TMOUT                     (0x1   << 0  )
#define TSTA_TMOUT_CLR                 (0x0   << 0  ) /* CLR                      */
#define TSTA_TMOUT_SET                 (0x1   << 0  ) /* SET                      */

/* GPCON[CON7] - Configuration bits for P0.7 */
#define GPCON_CON7_MSK                 (0x3   << 14 )

/* GPCON[CON6] - Configuration bits for P0.6 */
#define GPCON_CON6_MSK                 (0x3   << 12 )

/* GPCON[CON5] - Configuration bits for P0.5 */
#define GPCON_CON5_MSK                 (0x3   << 10 )

/* GPCON[CON4] - Configuration bits for P0.4 */
#define GPCON_CON4_MSK                 (0x3   << 8  )

/* GPCON[CON3] - Configuration bits for P0.3 */
#define GPCON_CON3_MSK                 (0x3   << 6  )

/* GPCON[CON2] - Configuration bits for P0.2 */
#define GPCON_CON2_MSK                 (0x3   << 4  )

/* GPCON[CON1] - Configuration bits for P0.1 */
#define GPCON_CON1_MSK                 (0x3   << 2  )

/* GPCON[CON0] - Configuration bits for P0.0 */
#define GPCON_CON0_MSK                 (0x3   << 0  )

/* GPOEN[OEN7] - Direction for port pin */
#define GPOEN_OEN7_MSK                 (0x1   << 7  )
#define GPOEN_OEN7                     (0x1   << 7  )
#define GPOEN_OEN7_IN                  (0x0   << 7  ) /* IN                       */
#define GPOEN_OEN7_OUT                 (0x1   << 7  ) /* OUT                      */

/* GPOEN[OEN6] - Direction for port pin */
#define GPOEN_OEN6_MSK                 (0x1   << 6  )
#define GPOEN_OEN6                     (0x1   << 6  )
#define GPOEN_OEN6_IN                  (0x0   << 6  ) /* IN                       */
#define GPOEN_OEN6_OUT                 (0x1   << 6  ) /* OUT                      */

/* GPOEN[OEN5] - Direction for port pin */
#define GPOEN_OEN5_MSK                 (0x1   << 5  )
#define GPOEN_OEN5                     (0x1   << 5  )
#define GPOEN_OEN5_IN                  (0x0   << 5  ) /* IN                       */
#define GPOEN_OEN5_OUT                 (0x1   << 5  ) /* OUT                      */

/* GPOEN[OEN4] - Direction for port pin */
#define GPOEN_OEN4_MSK                 (0x1   << 4  )
#define GPOEN_OEN4                     (0x1   << 4  )
#define GPOEN_OEN4_IN                  (0x0   << 4  ) /* IN                       */
#define GPOEN_OEN4_OUT                 (0x1   << 4  ) /* OUT                      */

/* GPOEN[OEN3] - Direction for port pin */
#define GPOEN_OEN3_MSK                 (0x1   << 3  )
#define GPOEN_OEN3                     (0x1   << 3  )
#define GPOEN_OEN3_IN                  (0x0   << 3  ) /* IN                       */
#define GPOEN_OEN3_OUT                 (0x1   << 3  ) /* OUT                      */

/* GPOEN[OEN2] - Direction for port pin */
#define GPOEN_OEN2_MSK                 (0x1   << 2  )
#define GPOEN_OEN2                     (0x1   << 2  )
#define GPOEN_OEN2_IN                  (0x0   << 2  ) /* IN                       */
#define GPOEN_OEN2_OUT                 (0x1   << 2  ) /* OUT                      */

/* GPOEN[OEN1] - Direction for port pin */
#define GPOEN_OEN1_MSK                 (0x1   << 1  )
#define GPOEN_OEN1                     (0x1   << 1  )
#define GPOEN_OEN1_IN                  (0x0   << 1  ) /* IN                       */
#define GPOEN_OEN1_OUT                 (0x1   << 1  ) /* OUT                      */

/* GPOEN[OEN0] - Direction for port pin */
#define GPOEN_OEN0_MSK                 (0x1   << 0  )
#define GPOEN_OEN0                     (0x1   << 0  )
#define GPOEN_OEN0_IN                  (0x0   << 0  ) /* IN                       */
#define GPOEN_OEN0_OUT                 (0x1   << 0  ) /* OUT                      */

/* GPIN[IN7] - Input for port pin */
#define GPIN_IN7_MSK                   (0x1   << 7  )
#define GPIN_IN7                       (0x1   << 7  )
#define GPIN_IN7_LOW                   (0x0   << 7  ) /* LOW                      */
#define GPIN_IN7_HIGH                  (0x1   << 7  ) /* HIGH                     */

/* GPIN[IN6] - Input for port pin */
#define GPIN_IN6_MSK                   (0x1   << 6  )
#define GPIN_IN6                       (0x1   << 6  )
#define GPIN_IN6_LOW                   (0x0   << 6  ) /* LOW                      */
#define GPIN_IN6_HIGH                  (0x1   << 6  ) /* HIGH                     */

/* GPIN[IN5] - Input for port pin */
#define GPIN_IN5_MSK                   (0x1   << 5  )
#define GPIN_IN5                       (0x1   << 5  )
#define GPIN_IN5_LOW                   (0x0   << 5  ) /* LOW                      */
#define GPIN_IN5_HIGH                  (0x1   << 5  ) /* HIGH                     */

/* GPIN[IN4] - Input for port pin */
#define GPIN_IN4_MSK                   (0x1   << 4  )
#define GPIN_IN4                       (0x1   << 4  )
#define GPIN_IN4_LOW                   (0x0   << 4  ) /* LOW                      */
#define GPIN_IN4_HIGH                  (0x1   << 4  ) /* HIGH                     */

/* GPIN[IN3] - Input for port pin */
#define GPIN_IN3_MSK                   (0x1   << 3  )
#define GPIN_IN3                       (0x1   << 3  )
#define GPIN_IN3_LOW                   (0x0   << 3  ) /* LOW                      */
#define GPIN_IN3_HIGH                  (0x1   << 3  ) /* HIGH                     */

/* GPIN[IN2] - Input for port pin */
#define GPIN_IN2_MSK                   (0x1   << 2  )
#define GPIN_IN2                       (0x1   << 2  )
#define GPIN_IN2_LOW                   (0x0   << 2  ) /* LOW                      */
#define GPIN_IN2_HIGH                  (0x1   << 2  ) /* HIGH                     */

/* GPIN[IN1] - Input for port pin */
#define GPIN_IN1_MSK                   (0x1   << 1  )
#define GPIN_IN1                       (0x1   << 1  )
#define GPIN_IN1_LOW                   (0x0   << 1  ) /* LOW                      */
#define GPIN_IN1_HIGH                  (0x1   << 1  ) /* HIGH                     */

/* GPIN[IN0] - Input for port pin */
#define GPIN_IN0_MSK                   (0x1   << 0  )
#define GPIN_IN0                       (0x1   << 0  )
#define GPIN_IN0_LOW                   (0x0   << 0  ) /* LOW                      */
#define GPIN_IN0_HIGH                  (0x1   << 0  ) /* HIGH                     */

/* GPOUT[OUT7] - Output for port pin */
#define GPOUT_OUT7_MSK                 (0x1   << 7  )
#define GPOUT_OUT7                     (0x1   << 7  )
#define GPOUT_OUT7_LOW                 (0x0   << 7  ) /* LOW                      */
#define GPOUT_OUT7_HIGH                (0x1   << 7  ) /* HIGH                     */

/* GPOUT[OUT6] - Output for port pin */
#define GPOUT_OUT6_MSK                 (0x1   << 6  )
#define GPOUT_OUT6                     (0x1   << 6  )
#define GPOUT_OUT6_LOW                 (0x0   << 6  ) /* LOW                      */
#define GPOUT_OUT6_HIGH                (0x1   << 6  ) /* HIGH                     */

/* GPOUT[OUT5] - Output for port pin */
#define GPOUT_OUT5_MSK                 (0x1   << 5  )
#define GPOUT_OUT5                     (0x1   << 5  )
#define GPOUT_OUT5_LOW                 (0x0   << 5  ) /* LOW                      */
#define GPOUT_OUT5_HIGH                (0x1   << 5  ) /* HIGH                     */

/* GPOUT[OUT4] - Output for port pin */
#define GPOUT_OUT4_MSK                 (0x1   << 4  )
#define GPOUT_OUT4                     (0x1   << 4  )
#define GPOUT_OUT4_LOW                 (0x0   << 4  ) /* LOW                      */
#define GPOUT_OUT4_HIGH                (0x1   << 4  ) /* HIGH                     */

/* GPOUT[OUT3] - Output for port pin */
#define GPOUT_OUT3_MSK                 (0x1   << 3  )
#define GPOUT_OUT3                     (0x1   << 3  )
#define GPOUT_OUT3_LOW                 (0x0   << 3  ) /* LOW                      */
#define GPOUT_OUT3_HIGH                (0x1   << 3  ) /* HIGH                     */

/* GPOUT[OUT2] - Output for port pin */
#define GPOUT_OUT2_MSK                 (0x1   << 2  )
#define GPOUT_OUT2                     (0x1   << 2  )
#define GPOUT_OUT2_LOW                 (0x0   << 2  ) /* LOW                      */
#define GPOUT_OUT2_HIGH                (0x1   << 2  ) /* HIGH                     */

/* GPOUT[OUT1] - Output for port pin */
#define GPOUT_OUT1_MSK                 (0x1   << 1  )
#define GPOUT_OUT1                     (0x1   << 1  )
#define GPOUT_OUT1_LOW                 (0x0   << 1  ) /* LOW                      */
#define GPOUT_OUT1_HIGH                (0x1   << 1  ) /* HIGH                     */

/* GPOUT[OUT0] - Output for port pin */
#define GPOUT_OUT0_MSK                 (0x1   << 0  )
#define GPOUT_OUT0                     (0x1   << 0  )
#define GPOUT_OUT0_LOW                 (0x0   << 0  ) /* LOW                      */
#define GPOUT_OUT0_HIGH                (0x1   << 0  ) /* HIGH                     */

/* GPSET[SET7] - Set Output High for port pin */
#define GPSET_SET7_MSK                 (0x1   << 7  )
#define GPSET_SET7                     (0x1   << 7  )
#define GPSET_SET7_SET                 (0x1   << 7  ) /* SET                      */

/* GPSET[SET6] - Set Output High for port pin */
#define GPSET_SET6_MSK                 (0x1   << 6  )
#define GPSET_SET6                     (0x1   << 6  )
#define GPSET_SET6_SET                 (0x1   << 6  ) /* SET                      */

/* GPSET[SET5] - Set Output High for port pin */
#define GPSET_SET5_MSK                 (0x1   << 5  )
#define GPSET_SET5                     (0x1   << 5  )
#define GPSET_SET5_SET                 (0x1   << 5  ) /* SET                      */

/* GPSET[SET4] - Set Output High for port pin */
#define GPSET_SET4_MSK                 (0x1   << 4  )
#define GPSET_SET4                     (0x1   << 4  )
#define GPSET_SET4_SET                 (0x1   << 4  ) /* SET                      */

/* GPSET[SET3] - Set Output High for port pin */
#define GPSET_SET3_MSK                 (0x1   << 3  )
#define GPSET_SET3                     (0x1   << 3  )
#define GPSET_SET3_SET                 (0x1   << 3  ) /* SET                      */

/* GPSET[SET2] - Set Output High for port pin */
#define GPSET_SET2_MSK                 (0x1   << 2  )
#define GPSET_SET2                     (0x1   << 2  )
#define GPSET_SET2_SET                 (0x1   << 2  ) /* SET                      */

/* GPSET[SET1] - Set Output High for port pin */
#define GPSET_SET1_MSK                 (0x1   << 1  )
#define GPSET_SET1                     (0x1   << 1  )
#define GPSET_SET1_SET                 (0x1   << 1  ) /* SET                      */

/* GPSET[SET0] - Set Output High for port pin */
#define GPSET_SET0_MSK                 (0x1   << 0  )
#define GPSET_SET0                     (0x1   << 0  )
#define GPSET_SET0_SET                 (0x1   << 0  ) /* SET                      */

/* GPCLR[CLR7] - Set Output Low for port pin */
#define GPCLR_CLR7_MSK                 (0x1   << 7  )
#define GPCLR_CLR7                     (0x1   << 7  )
#define GPCLR_CLR7_CLR                 (0x1   << 7  ) /* CLR                      */

/* GPCLR[CLR6] - Set Output Low for port pin */
#define GPCLR_CLR6_MSK                 (0x1   << 6  )
#define GPCLR_CLR6                     (0x1   << 6  )
#define GPCLR_CLR6_CLR                 (0x1   << 6  ) /* CLR                      */

/* GPCLR[CLR5] - Set Output Low for port pin */
#define GPCLR_CLR5_MSK                 (0x1   << 5  )
#define GPCLR_CLR5                     (0x1   << 5  )
#define GPCLR_CLR5_CLR                 (0x1   << 5  ) /* CLR                      */

/* GPCLR[CLR4] - Set Output Low for port pin */
#define GPCLR_CLR4_MSK                 (0x1   << 4  )
#define GPCLR_CLR4                     (0x1   << 4  )
#define GPCLR_CLR4_CLR                 (0x1   << 4  ) /* CLR                      */

/* GPCLR[CLR3] - Set Output Low for port pin */
#define GPCLR_CLR3_MSK                 (0x1   << 3  )
#define GPCLR_CLR3                     (0x1   << 3  )
#define GPCLR_CLR3_CLR                 (0x1   << 3  ) /* CLR                      */

/* GPCLR[CLR2] - Set Output Low for port pin */
#define GPCLR_CLR2_MSK                 (0x1   << 2  )
#define GPCLR_CLR2                     (0x1   << 2  )
#define GPCLR_CLR2_CLR                 (0x1   << 2  ) /* CLR                      */

/* GPCLR[CLR1] - Set Output Low for port pin */
#define GPCLR_CLR1_MSK                 (0x1   << 1  )
#define GPCLR_CLR1                     (0x1   << 1  )
#define GPCLR_CLR1_CLR                 (0x1   << 1  ) /* CLR                      */

/* GPCLR[CLR0] - Set Output Low for port pin */
#define GPCLR_CLR0_MSK                 (0x1   << 0  )
#define GPCLR_CLR0                     (0x1   << 0  )
#define GPCLR_CLR0_CLR                 (0x1   << 0  ) /* CLR                      */

/* GPTGL[TGL7] - Toggle Output for port pin */
#define GPTGL_TGL7_MSK                 (0x1   << 7  )
#define GPTGL_TGL7                     (0x1   << 7  )
#define GPTGL_TGL7_TGL                 (0x1   << 7  ) /* TGL                      */

/* GPTGL[TGL6] - Toggle Output for port pin */
#define GPTGL_TGL6_MSK                 (0x1   << 6  )
#define GPTGL_TGL6                     (0x1   << 6  )
#define GPTGL_TGL6_TGL                 (0x1   << 6  ) /* TGL                      */

/* GPTGL[TGL5] - Toggle Output for port pin */
#define GPTGL_TGL5_MSK                 (0x1   << 5  )
#define GPTGL_TGL5                     (0x1   << 5  )
#define GPTGL_TGL5_TGL                 (0x1   << 5  ) /* TGL                      */

/* GPTGL[TGL4] - Toggle Output for port pin */
#define GPTGL_TGL4_MSK                 (0x1   << 4  )
#define GPTGL_TGL4                     (0x1   << 4  )
#define GPTGL_TGL4_TGL                 (0x1   << 4  ) /* TGL                      */

/* GPTGL[TGL3] - Toggle Output for port pin */
#define GPTGL_TGL3_MSK                 (0x1   << 3  )
#define GPTGL_TGL3                     (0x1   << 3  )
#define GPTGL_TGL3_TGL                 (0x1   << 3  ) /* TGL                      */

/* GPTGL[TGL2] - Toggle Output for port pin */
#define GPTGL_TGL2_MSK                 (0x1   << 2  )
#define GPTGL_TGL2                     (0x1   << 2  )
#define GPTGL_TGL2_TGL                 (0x1   << 2  ) /* TGL                      */

/* GPTGL[TGL1] - Toggle Output for port pin */
#define GPTGL_TGL1_MSK                 (0x1   << 1  )
#define GPTGL_TGL1                     (0x1   << 1  )
#define GPTGL_TGL1_TGL                 (0x1   << 1  ) /* TGL                      */

/* GPTGL[TGL0] - Toggle Output for port pin */
#define GPTGL_TGL0_MSK                 (0x1   << 0  )
#define GPTGL_TGL0                     (0x1   << 0  )
#define GPTGL_TGL0_TGL                 (0x1   << 0  ) /* TGL                      */

/* SPIDIV[BCRST] - Bit counter reset */
#define SPIDIV_BCRST_MSK               (0x1   << 7  )
#define SPIDIV_BCRST                   (0x1   << 7  )
#define SPIDIV_BCRST_DIS               (0x0   << 7  ) /* DIS                      */
#define SPIDIV_BCRST_EN                (0x1   << 7  ) /* EN                       */

/* SPIDIV[DIV] - Factor used to divide UCLK to generate the serial clock */
#define SPIDIV_DIV_MSK                 (0x3F  << 0  )

/* SPICON[MOD] - SPI IRQ Mode bits */
#define SPICON_MOD_MSK                 (0x3   << 14 )
#define SPICON_MOD_TX1RX1              (0x0   << 14 ) /* TX1RX1                   */
#define SPICON_MOD_TX2RX2              (0x1   << 14 ) /* TX2RX2                   */
#define SPICON_MOD_TX3RX3              (0x2   << 14 ) /* TX3RX3                   */
#define SPICON_MOD_TX4RX4              (0x3   << 14 ) /* TX4RX4                   */

/* SPICON[TFLUSH] - TX FIFO Flush Enable bit */
#define SPICON_TFLUSH_MSK              (0x1   << 13 )
#define SPICON_TFLUSH                  (0x1   << 13 )
#define SPICON_TFLUSH_DIS              (0x0   << 13 ) /* DIS                      */
#define SPICON_TFLUSH_EN               (0x1   << 13 ) /* EN                       */

/* SPICON[RFLUSH] - RX FIFO Flush Enable bit */
#define SPICON_RFLUSH_MSK              (0x1   << 12 )
#define SPICON_RFLUSH                  (0x1   << 12 )
#define SPICON_RFLUSH_DIS              (0x0   << 12 ) /* DIS                      */
#define SPICON_RFLUSH_EN               (0x1   << 12 ) /* EN                       */

/* SPICON[CON] - Continuous transfer enable */
#define SPICON_CON_MSK                 (0x1   << 11 )
#define SPICON_CON                     (0x1   << 11 )
#define SPICON_CON_DIS                 (0x0   << 11 ) /* DIS                      */
#define SPICON_CON_EN                  (0x1   << 11 ) /* EN                       */

/* SPICON[LOOPBACK] - Loopback enable bit */
#define SPICON_LOOPBACK_MSK            (0x1   << 10 )
#define SPICON_LOOPBACK                (0x1   << 10 )
#define SPICON_LOOPBACK_DIS            (0x0   << 10 ) /* DIS                      */
#define SPICON_LOOPBACK_EN             (0x1   << 10 ) /* EN                       */

/* SPICON[SOEN] - Slave MISO output enable bit */
#define SPICON_SOEN_MSK                (0x1   << 9  )
#define SPICON_SOEN                    (0x1   << 9  )
#define SPICON_SOEN_DIS                (0x0   << 9  ) /* DIS                      */
#define SPICON_SOEN_EN                 (0x1   << 9  ) /* EN                       */

/* SPICON[RXOF] - RX Oveflow Overwrite enable */
#define SPICON_RXOF_MSK                (0x1   << 8  )
#define SPICON_RXOF                    (0x1   << 8  )
#define SPICON_RXOF_DIS                (0x0   << 8  ) /* DIS                      */
#define SPICON_RXOF_EN                 (0x1   << 8  ) /* EN                       */

/* SPICON[ZEN] - Transmit zeros when empty */
#define SPICON_ZEN_MSK                 (0x1   << 7  )
#define SPICON_ZEN                     (0x1   << 7  )
#define SPICON_ZEN_DIS                 (0x0   << 7  ) /* DIS                      */
#define SPICON_ZEN_EN                  (0x1   << 7  ) /* EN                       */

/* SPICON[TIM] - Transfer and interrupt mode */
#define SPICON_TIM_MSK                 (0x1   << 6  )
#define SPICON_TIM                     (0x1   << 6  )
#define SPICON_TIM_RXRD                (0x0   << 6  ) /* RXRD - Cleared by user to initiate transfer with a read of the SPIRX register */
#define SPICON_TIM_TXWR                (0x1   << 6  ) /* TXWR - Set by user to initiate transfer with a write to the SPITX register. */

/* SPICON[LSB] - LSB First Transfer enable */
#define SPICON_LSB_MSK                 (0x1   << 5  )
#define SPICON_LSB                     (0x1   << 5  )
#define SPICON_LSB_DIS                 (0x0   << 5  ) /* DIS                      */
#define SPICON_LSB_EN                  (0x1   << 5  ) /* EN                       */

/* SPICON[WOM] - Wired OR enable */
#define SPICON_WOM_MSK                 (0x1   << 4  )
#define SPICON_WOM                     (0x1   << 4  )
#define SPICON_WOM_DIS                 (0x0   << 4  ) /* DIS                      */
#define SPICON_WOM_EN                  (0x1   << 4  ) /* EN                       */

/* SPICON[CPOL] - Clock polarity mode */
#define SPICON_CPOL_MSK                (0x1   << 3  )
#define SPICON_CPOL                    (0x1   << 3  )
#define SPICON_CPOL_LOW                (0x0   << 3  ) /* LOW                      */
#define SPICON_CPOL_HIGH               (0x1   << 3  ) /* HIGH                     */

/* SPICON[CPHA] - Clock phase mode */
#define SPICON_CPHA_MSK                (0x1   << 2  )
#define SPICON_CPHA                    (0x1   << 2  )
#define SPICON_CPHA_SAMPLELEADING      (0x0   << 2  ) /* SAMPLELEADING            */
#define SPICON_CPHA_SAMPLETRAILING     (0x1   << 2  ) /* SAMPLETRAILING           */

/* SPICON[MASEN] - Master enable */
#define SPICON_MASEN_MSK               (0x1   << 1  )
#define SPICON_MASEN                   (0x1   << 1  )
#define SPICON_MASEN_DIS               (0x0   << 1  ) /* DIS                      */
#define SPICON_MASEN_EN                (0x1   << 1  ) /* EN                       */

/* SPICON[ENABLE] - SPI Enable bit */
#define SPICON_ENABLE_MSK              (0x1   << 0  )
#define SPICON_ENABLE                  (0x1   << 0  )
#define SPICON_ENABLE_DIS              (0x0   << 0  ) /* DIS                      */
#define SPICON_ENABLE_EN               (0x1   << 0  ) /* EN                       */

/* SPIDMA[IENRXDMA] - Enable receive DMA request */
#define SPIDMA_IENRXDMA_MSK            (0x1   << 2  )
#define SPIDMA_IENRXDMA                (0x1   << 2  )
#define SPIDMA_IENRXDMA_DIS            (0x0   << 2  ) /* DIS                      */
#define SPIDMA_IENRXDMA_EN             (0x1   << 2  ) /* EN                       */

/* SPIDMA[IENTXDMA] - Enable transmit DMA request */
#define SPIDMA_IENTXDMA_MSK            (0x1   << 1  )
#define SPIDMA_IENTXDMA                (0x1   << 1  )
#define SPIDMA_IENTXDMA_DIS            (0x0   << 1  ) /* DIS                      */
#define SPIDMA_IENTXDMA_EN             (0x1   << 1  ) /* EN                       */

/* SPIDMA[ENABLE] - Enable DMA for data transfer */
#define SPIDMA_ENABLE_MSK              (0x1   << 0  )
#define SPIDMA_ENABLE                  (0x1   << 0  )
#define SPIDMA_ENABLE_DIS              (0x0   << 0  ) /* DIS                      */
#define SPIDMA_ENABLE_EN               (0x1   << 0  ) /* EN                       */

/* SPISTA[CSERR] - Detected an abrupt CS deassertion */
#define SPISTA_CSERR_MSK               (0x1   << 12 )
#define SPISTA_CSERR                   (0x1   << 12 )
#define SPISTA_CSERR_CLR               (0x0   << 12 ) /* CLR                      */
#define SPISTA_CSERR_SET               (0x1   << 12 ) /* SET                      */

/* SPISTA[CSRSG] - Detected an abrupt CS deassertion */
#define SPISTA_CSRSG_MSK               (0x1   << 14 )
#define SPISTA_CSRSG                   (0x1   << 14 )
#define SPISTA_CSRSG_CLR               (0x0   << 14 ) /* CLR                      */
#define SPISTA_CSRSG_SET               (0x1   << 14 ) /* SET                      */

/* SPISTA[RXS] - Set when there are more bytes in the RX FIFO than the TIM bit says */
#define SPISTA_RXS_MSK                 (0x1   << 11 )
#define SPISTA_RXS                     (0x1   << 11 )
#define SPISTA_RXS_CLR                 (0x0   << 11 ) /* CLR                      */
#define SPISTA_RXS_SET                 (0x1   << 11 ) /* SET                      */

/* SPISTA[RXFSTA] - Receive FIFO Status */
#define SPISTA_RXFSTA_MSK              (0x7   << 8  )
#define SPISTA_RXFSTA_EMPTY            (0x0   << 8  ) /* EMPTY                    */
#define SPISTA_RXFSTA_ONEBYTE          (0x1   << 8  ) /* ONEBYTE                  */
#define SPISTA_RXFSTA_TWOBYTES         (0x2   << 8  ) /* TWOBYTES                 */
#define SPISTA_RXFSTA_THREEBYTES       (0x3   << 8  ) /* THREEBYTES               */
#define SPISTA_RXFSTA_FOURBYTES        (0x4   << 8  ) /* FOURBYTES                */

/* SPISTA[RXOF] - Receive FIFO overflow */
#define SPISTA_RXOF_MSK                (0x1   << 7  )
#define SPISTA_RXOF                    (0x1   << 7  )
#define SPISTA_RXOF_CLR                (0x0   << 7  ) /* CLR                      */
#define SPISTA_RXOF_SET                (0x1   << 7  ) /* SET                      */

/* SPISTA[RX] - Set when a receive interrupt occurs */
#define SPISTA_RX_MSK                  (0x1   << 6  )
#define SPISTA_RX                      (0x1   << 6  )
#define SPISTA_RX_CLR                  (0x0   << 6  ) /* CLR                      */
#define SPISTA_RX_SET                  (0x1   << 6  ) /* SET                      */

/* SPISTA[TX] - Set when a transmit interrupt occurs */
#define SPISTA_TX_MSK                  (0x1   << 5  )
#define SPISTA_TX                      (0x1   << 5  )
#define SPISTA_TX_CLR                  (0x0   << 5  ) /* CLR                      */
#define SPISTA_TX_SET                  (0x1   << 5  ) /* SET                      */

/* SPISTA[TXUR] - Transmit FIFO underflow */
#define SPISTA_TXUR_MSK                (0x1   << 4  )
#define SPISTA_TXUR                    (0x1   << 4  )
#define SPISTA_TXUR_CLR                (0x0   << 4  ) /* CLR                      */
#define SPISTA_TXUR_SET                (0x1   << 4  ) /* SET                      */

/* SPISTA[TXFSTA] - transmit FIFO Status */
#define SPISTA_TXFSTA_MSK              (0x7   << 1  )
#define SPISTA_TXFSTA_EMPTY            (0x0   << 1  ) /* EMPTY                    */
#define SPISTA_TXFSTA_ONEBYTE          (0x1   << 1  ) /* ONEBYTE                  */
#define SPISTA_TXFSTA_TWOBYTES         (0x2   << 1  ) /* TWOBYTES                 */
#define SPISTA_TXFSTA_THREEBYTES       (0x3   << 1  ) /* THREEBYTES               */
#define SPISTA_TXFSTA_FOURBYTES        (0x4   << 1  ) /* FOURBYTES                */

/* SPISTA[IRQ] - Interrupt status bit */
#define SPISTA_IRQ_MSK                 (0x1   << 0  )
#define SPISTA_IRQ                     (0x1   << 0  )
#define SPISTA_IRQ_CLR                 (0x0   << 0  ) /* CLR                      */
#define SPISTA_IRQ_SET                 (0x1   << 0  ) /* SET                      */

/* SPIDIV[BCRST] - Bit counter reset */
#define SPIDIV_BCRST_MSK               (0x1   << 7  )
#define SPIDIV_BCRST                   (0x1   << 7  )
#define SPIDIV_BCRST_DIS               (0x0   << 7  ) /* DIS                      */
#define SPIDIV_BCRST_EN                (0x1   << 7  ) /* EN                       */

/* SPIDIV[DIV] - Factor used to divide UCLK to generate the serial clock */
#define SPIDIV_DIV_MSK                 (0x3F  << 0  )
// ------------------------------------------------------------------------------------------------
// -----                                        T0                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Timer 0 (pADI_TM0)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_TM0 Structure                     */
  __IO uint16_t  LD;                        /*!< 16-bit load value                     */
  __I  uint16_t  RESERVED0;
  __IO uint16_t  VAL;                       /*!< "16-bit timer value, read only."      */
  __I  uint16_t  RESERVED1;
  __IO uint16_t  CON;                       /*!< Control Register                      */
  __I  uint16_t  RESERVED2;
  __IO uint16_t  CLRI;                      /*!< Clear interrupt register              */
  __I  uint16_t  RESERVED3;
  __IO uint16_t  CAP;                       /*!< Capture Register                      */
  __I  uint16_t  RESERVED4[5];
  __IO uint16_t  STA;                       /*!< Status Register                       */
} ADI_TIMER_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          T0LD                                       (*(volatile unsigned short int *) 0x40000000)
#define          T0VAL                                      (*(volatile unsigned short int *) 0x40000004)
#define          T0CON                                      (*(volatile unsigned short int *) 0x40000008)
#define          T0CLRI                                     (*(volatile unsigned short int *) 0x4000000C)
#define          T0CAP                                      (*(volatile unsigned short int *) 0x40000010)
#define          T0STA                                      (*(volatile unsigned short int *) 0x4000001C)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for T0LD*/
#define T0LD_RVAL                      0x0 

/* T0LD[VALUE] - Load value */
#define T0LD_VALUE_MSK                 (0xFFFF << 0  )

/* Reset Value for T0VAL*/
#define T0VAL_RVAL                     0x0 

/* T0VAL[VALUE] - Current value */
#define T0VAL_VALUE_MSK                (0xFFFF << 0  )

/* Reset Value for T0CON*/
#define T0CON_RVAL                     0xA 

/* T0CON[EVENTEN] - Enable time capture of an event */
#define T0CON_EVENTEN_BBA              (*(volatile unsigned long *) 0x42000130)
#define T0CON_EVENTEN_MSK              (0x1   << 12 )
#define T0CON_EVENTEN                  (0x1   << 12 )
#define T0CON_EVENTEN_DIS              (0x0   << 12 ) /* DIS                      */
#define T0CON_EVENTEN_EN               (0x1   << 12 ) /* EN                       */

/* T0CON[EVENT] - Event Select, selects 1 of the available events. */
#define T0CON_EVENT_MSK                (0xF   << 8  )
#define T0CON_EVENT_T2                 (0x0   << 8  ) /* T2 - Wakeup Timer        */
#define T0CON_EVENT_EXT0               (0x1   << 8  ) /* EXT0 - External interrupt 0 */
#define T0CON_EVENT_EXT1               (0x2   << 8  ) /* EXT1 - External interrupt 1 */
#define T0CON_EVENT_EXT2               (0x3   << 8  ) /* EXT2 - External interrupt 2 */
#define T0CON_EVENT_EXT3               (0x4   << 8  ) /* EXT3 - External interrupt 3 */
#define T0CON_EVENT_EXT4               (0x5   << 8  ) /* EXT4 - External interrupt 4 */
#define T0CON_EVENT_EXT5               (0x6   << 8  ) /* EXT5 - External interrupt 5 */
#define T0CON_EVENT_EXT6               (0x7   << 8  ) /* EXT6 - External interrupt 6 */
#define T0CON_EVENT_EXT7               (0x8   << 8  ) /* EXT7 - External interrupt 7 */
#define T0CON_EVENT_T3                 (0x9   << 8  ) /* T3 - Watchdog timer      */
#define T0CON_EVENT_T1                 (0xA   << 8  ) /* T1 - Timer1              */
#define T0CON_EVENT_ADC1               (0xC   << 8  ) /* ADC1 - ADC1              */
#define T0CON_EVENT_STEP               (0xD   << 8  ) /* STEP - STEP              */
#define T0CON_EVENT_DMADONE            (0xE   << 8  ) /* DMADONE                  */
#define T0CON_EVENT_FEE                (0xF   << 8  ) /* FEE - Flash controller   */

/* T0CON[RLD] - Timer reload on write to clear register */
#define T0CON_RLD_BBA                  (*(volatile unsigned long *) 0x4200011C)
#define T0CON_RLD_MSK                  (0x1   << 7  )
#define T0CON_RLD                      (0x1   << 7  )
#define T0CON_RLD_DIS                  (0x0   << 7  ) /* DIS                      */
#define T0CON_RLD_EN                   (0x1   << 7  ) /* EN                       */

/* T0CON[CLK] - Clock Select */
#define T0CON_CLK_MSK                  (0x3   << 5  )
#define T0CON_CLK_UCLK                 (0x0   << 5  ) /* UCLK - System Clock      */
#define T0CON_CLK_PCLK                 (0x1   << 5  ) /* PCLK - Peripheral clock  */
#define T0CON_CLK_LFOSC                (0x2   << 5  ) /* LFOSC - Internal 32 kHz Oscillator */
#define T0CON_CLK_LFXTAL               (0x3   << 5  ) /* LFXTAL - External 32 kHz crystal */

/* T0CON[ENABLE] - Enable */
#define T0CON_ENABLE_BBA               (*(volatile unsigned long *) 0x42000110)
#define T0CON_ENABLE_MSK               (0x1   << 4  )
#define T0CON_ENABLE                   (0x1   << 4  )
#define T0CON_ENABLE_DIS               (0x0   << 4  ) /* DIS                      */
#define T0CON_ENABLE_EN                (0x1   << 4  ) /* EN                       */

/* T0CON[MOD] - Mode */
#define T0CON_MOD_BBA                  (*(volatile unsigned long *) 0x4200010C)
#define T0CON_MOD_MSK                  (0x1   << 3  )
#define T0CON_MOD                      (0x1   << 3  )
#define T0CON_MOD_FREERUN              (0x0   << 3  ) /* FREERUN                  */
#define T0CON_MOD_PERIODIC             (0x1   << 3  ) /* PERIODIC                 */

/* T0CON[UP] - Count-up */
#define T0CON_UP_BBA                   (*(volatile unsigned long *) 0x42000108)
#define T0CON_UP_MSK                   (0x1   << 2  )
#define T0CON_UP                       (0x1   << 2  )
#define T0CON_UP_DIS                   (0x0   << 2  ) /* DIS                      */
#define T0CON_UP_EN                    (0x1   << 2  ) /* EN                       */

/* T0CON[PRE] - Prescaler */
#define T0CON_PRE_MSK                  (0x3   << 0  )
#define T0CON_PRE_DIV1                 (0x0   << 0  ) /* DIV1                     */
#define T0CON_PRE_DIV16                (0x1   << 0  ) /* DIV16                    */
#define T0CON_PRE_DIV256               (0x2   << 0  ) /* DIV256                   */
#define T0CON_PRE_DIV32768             (0x3   << 0  ) /* DIV32768 - If the selected clock source is UCLK then this setting results in a prescaler of 4. */

/* Reset Value for T0CLRI*/
#define T0CLRI_RVAL                    0x0 

/* T0CLRI[CAP] - Clear captured event interrupt */
#define T0CLRI_CAP_BBA                 (*(volatile unsigned long *) 0x42000184)
#define T0CLRI_CAP_MSK                 (0x1   << 1  )
#define T0CLRI_CAP                     (0x1   << 1  )
#define T0CLRI_CAP_CLR                 (0x1   << 1  ) /* CLR                      */

/* T0CLRI[TMOUT] - Clear timeout interrupt */
#define T0CLRI_TMOUT_BBA               (*(volatile unsigned long *) 0x42000180)
#define T0CLRI_TMOUT_MSK               (0x1   << 0  )
#define T0CLRI_TMOUT                   (0x1   << 0  )
#define T0CLRI_TMOUT_CLR               (0x1   << 0  ) /* CLR                      */

/* Reset Value for T0CAP*/
#define T0CAP_RVAL                     0x0 

/* T0CAP[VALUE] - Capture value */
#define T0CAP_VALUE_MSK                (0xFFFF << 0  )

/* Reset Value for T0STA*/
#define T0STA_RVAL                     0x0 

/* T0STA[CLRI] - Value updated in the timer clock domain */
#define T0STA_CLRI_BBA                 (*(volatile unsigned long *) 0x4200039C)
#define T0STA_CLRI_MSK                 (0x1   << 7  )
#define T0STA_CLRI                     (0x1   << 7  )
#define T0STA_CLRI_CLR                 (0x0   << 7  ) /* CLR                      */
#define T0STA_CLRI_SET                 (0x1   << 7  ) /* SET                      */

/* T0STA[CON] - Ready to receive commands */
#define T0STA_CON_BBA                  (*(volatile unsigned long *) 0x42000398)
#define T0STA_CON_MSK                  (0x1   << 6  )
#define T0STA_CON                      (0x1   << 6  )
#define T0STA_CON_CLR                  (0x0   << 6  ) /* CLR                      */
#define T0STA_CON_SET                  (0x1   << 6  ) /* SET                      */

/* T0STA[CAP] - Capture event pending */
#define T0STA_CAP_BBA                  (*(volatile unsigned long *) 0x42000384)
#define T0STA_CAP_MSK                  (0x1   << 1  )
#define T0STA_CAP                      (0x1   << 1  )
#define T0STA_CAP_CLR                  (0x0   << 1  ) /* CLR                      */
#define T0STA_CAP_SET                  (0x1   << 1  ) /* SET                      */

/* T0STA[TMOUT] - Time out event occurred */
#define T0STA_TMOUT_BBA                (*(volatile unsigned long *) 0x42000380)
#define T0STA_TMOUT_MSK                (0x1   << 0  )
#define T0STA_TMOUT                    (0x1   << 0  )
#define T0STA_TMOUT_CLR                (0x0   << 0  ) /* CLR                      */
#define T0STA_TMOUT_SET                (0x1   << 0  ) /* SET                      */
#if (__NO_MMR_STRUCTS__==1)

#define          T1LD                                       (*(volatile unsigned short int *) 0x40000400)
#define          T1VAL                                      (*(volatile unsigned short int *) 0x40000404)
#define          T1CON                                      (*(volatile unsigned short int *) 0x40000408)
#define          T1CLRI                                     (*(volatile unsigned short int *) 0x4000040C)
#define          T1CAP                                      (*(volatile unsigned short int *) 0x40000410)
#define          T1STA                                      (*(volatile unsigned short int *) 0x4000041C)
#endif // (__NO_MMR_STRUCTS__==1)

/* Reset Value for T1LD*/
#define T1LD_RVAL                      0x0 

/* T1LD[VALUE] - Load value */
#define T1LD_VALUE_MSK                 (0xFFFF << 0  )

/* Reset Value for T1VAL*/
#define T1VAL_RVAL                     0x0 

/* T1VAL[VALUE] - Current value */
#define T1VAL_VALUE_MSK                (0xFFFF << 0  )

/* Reset Value for T1CON*/
#define T1CON_RVAL                     0xA 

/* T1CON[EVENTEN] - Enable time capture of an event */
#define T1CON_EVENTEN_BBA              (*(volatile unsigned long *) 0x42008130)
#define T1CON_EVENTEN_MSK              (0x1   << 12 )
#define T1CON_EVENTEN                  (0x1   << 12 )
#define T1CON_EVENTEN_DIS              (0x0   << 12 ) /* DIS                      */
#define T1CON_EVENTEN_EN               (0x1   << 12 ) /* EN                       */

/* T1CON[EVENT] - Event Select, selects 1 of the available events. */
#define T1CON_EVENT_MSK                (0xF   << 8  )
#define T1CON_EVENT_COM                (0x0   << 8  ) /* COM                      */
#define T1CON_EVENT_T0                 (0x1   << 8  ) /* T0 - Timer0              */
#define T1CON_EVENT_SPI0               (0x2   << 8  ) /* SPI0                     */
#define T1CON_EVENT_SPI1               (0x3   << 8  ) /* SPI1                     */
#define T1CON_EVENT_I2CS               (0x4   << 8  ) /* I2CS                     */
#define T1CON_EVENT_I2CM               (0x5   << 8  ) /* I2CM                     */
#define T1CON_EVENT_DMAERR             (0x6   << 8  ) /* DMAERR                   */
#define T1CON_EVENT_DMADONE            (0x7   << 8  ) /* DMADONE                  */
#define T1CON_EVENT_EXT1               (0x8   << 8  ) /* EXT1                     */
#define T1CON_EVENT_EXT2               (0x9   << 8  ) /* EXT2                     */
#define T1CON_EVENT_EXT3               (0xA   << 8  ) /* EXT3                     */
#define T1CON_EVENT_PWMTRIP            (0xB   << 8  ) /* PWMTRIP                  */
#define T1CON_EVENT_PWM0               (0xC   << 8  ) /* PWM0                     */
#define T1CON_EVENT_PWM1               (0xD   << 8  ) /* PWM1                     */
#define T1CON_EVENT_PWM2               (0xE   << 8  ) /* PWM2                     */

/* T1CON[RLD] - Timer reload on write to clear register */
#define T1CON_RLD_BBA                  (*(volatile unsigned long *) 0x4200811C)
#define T1CON_RLD_MSK                  (0x1   << 7  )
#define T1CON_RLD                      (0x1   << 7  )
#define T1CON_RLD_DIS                  (0x0   << 7  ) /* DIS                      */
#define T1CON_RLD_EN                   (0x1   << 7  ) /* EN                       */

/* T1CON[CLK] - Clock Select */
#define T1CON_CLK_MSK                  (0x3   << 5  )
#define T1CON_CLK_UCLK                 (0x0   << 5  ) /* UCLK - System Clock      */
#define T1CON_CLK_PCLK                 (0x1   << 5  ) /* PCLK - Peripheral clock  */
#define T1CON_CLK_LFOSC                (0x2   << 5  ) /* LFOSC - Internal 32 kHz Oscillator */
#define T1CON_CLK_LFXTAL               (0x3   << 5  ) /* LFXTAL - External 32 kHz crystal */

/* T1CON[ENABLE] - Enable */
#define T1CON_ENABLE_BBA               (*(volatile unsigned long *) 0x42008110)
#define T1CON_ENABLE_MSK               (0x1   << 4  )
#define T1CON_ENABLE                   (0x1   << 4  )
#define T1CON_ENABLE_DIS               (0x0   << 4  ) /* DIS                      */
#define T1CON_ENABLE_EN                (0x1   << 4  ) /* EN                       */

/* T1CON[MOD] - Mode */
#define T1CON_MOD_BBA                  (*(volatile unsigned long *) 0x4200810C)
#define T1CON_MOD_MSK                  (0x1   << 3  )
#define T1CON_MOD                      (0x1   << 3  )
#define T1CON_MOD_FREERUN              (0x0   << 3  ) /* FREERUN                  */
#define T1CON_MOD_PERIODIC             (0x1   << 3  ) /* PERIODIC                 */

/* T1CON[UP] - Count-up */
#define T1CON_UP_BBA                   (*(volatile unsigned long *) 0x42008108)
#define T1CON_UP_MSK                   (0x1   << 2  )
#define T1CON_UP                       (0x1   << 2  )
#define T1CON_UP_DIS                   (0x0   << 2  ) /* DIS                      */
#define T1CON_UP_EN                    (0x1   << 2  ) /* EN                       */

/* T1CON[PRE] - Prescaler */
#define T1CON_PRE_MSK                  (0x3   << 0  )
#define T1CON_PRE_DIV1                 (0x0   << 0  ) /* DIV1                     */
#define T1CON_PRE_DIV16                (0x1   << 0  ) /* DIV16                    */
#define T1CON_PRE_DIV256               (0x2   << 0  ) /* DIV256                   */
#define T1CON_PRE_DIV32768             (0x3   << 0  ) /* DIV32768 - If the selected clock source is UCLK then this setting results in a prescaler of 4. */

/* Reset Value for T1CLRI*/
#define T1CLRI_RVAL                    0x0 

/* T1CLRI[CAP] - Clear captured event interrupt */
#define T1CLRI_CAP_BBA                 (*(volatile unsigned long *) 0x42008184)
#define T1CLRI_CAP_MSK                 (0x1   << 1  )
#define T1CLRI_CAP                     (0x1   << 1  )
#define T1CLRI_CAP_CLR                 (0x1   << 1  ) /* CLR                      */

/* T1CLRI[TMOUT] - Clear timeout interrupt */
#define T1CLRI_TMOUT_BBA               (*(volatile unsigned long *) 0x42008180)
#define T1CLRI_TMOUT_MSK               (0x1   << 0  )
#define T1CLRI_TMOUT                   (0x1   << 0  )
#define T1CLRI_TMOUT_CLR               (0x1   << 0  ) /* CLR                      */

/* Reset Value for T1CAP*/
#define T1CAP_RVAL                     0x0 

/* T1CAP[VALUE] - Capture value */
#define T1CAP_VALUE_MSK                (0xFFFF << 0  )

/* Reset Value for T1STA*/
#define T1STA_RVAL                     0x0 

/* T1STA[CLRI] - Value updated in the timer clock domain */
#define T1STA_CLRI_BBA                 (*(volatile unsigned long *) 0x4200839C)
#define T1STA_CLRI_MSK                 (0x1   << 7  )
#define T1STA_CLRI                     (0x1   << 7  )
#define T1STA_CLRI_CLR                 (0x0   << 7  ) /* CLR                      */
#define T1STA_CLRI_SET                 (0x1   << 7  ) /* SET                      */

/* T1STA[CON] - Ready to receive commands */
#define T1STA_CON_BBA                  (*(volatile unsigned long *) 0x42008398)
#define T1STA_CON_MSK                  (0x1   << 6  )
#define T1STA_CON                      (0x1   << 6  )
#define T1STA_CON_CLR                  (0x0   << 6  ) /* CLR                      */
#define T1STA_CON_SET                  (0x1   << 6  ) /* SET                      */

/* T1STA[CAP] - Capture event pending */
#define T1STA_CAP_BBA                  (*(volatile unsigned long *) 0x42008384)
#define T1STA_CAP_MSK                  (0x1   << 1  )
#define T1STA_CAP                      (0x1   << 1  )
#define T1STA_CAP_CLR                  (0x0   << 1  ) /* CLR                      */
#define T1STA_CAP_SET                  (0x1   << 1  ) /* SET                      */

/* T1STA[TMOUT] - Time out event occurred */
#define T1STA_TMOUT_BBA                (*(volatile unsigned long *) 0x42008380)
#define T1STA_TMOUT_MSK                (0x1   << 0  )
#define T1STA_TMOUT                    (0x1   << 0  )
#define T1STA_TMOUT_CLR                (0x0   << 0  ) /* CLR                      */
#define T1STA_TMOUT_SET                (0x1   << 0  ) /* SET                      */
// ------------------------------------------------------------------------------------------------
// -----                                        PWM                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Pulse Width Modulation (pADI_PWM)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_PWM Structure                     */
  __IO uint16_t  PWMCON0;                   /*!< PWM Control register                  */
  __I  uint16_t  RESERVED0;
  __IO uint8_t   PWMCON1;                   /*!< Trip control register                 */
  __I  uint8_t   RESERVED1[3];
  __IO uint16_t  PWMCLRI;                   /*!< PWM interrupt clear. Write to this register clears the latched PWM interrupt. */
  __I  uint16_t  RESERVED2[3];
  __IO uint16_t  PWM0COM0;                  /*!< Compare Register 0 for PWM0 and PWM1  */
  __I  uint16_t  RESERVED3;
  __IO uint16_t  PWM0COM1;                  /*!< Compare Register 1 for PWM0 and PWM1  */
  __I  uint16_t  RESERVED4;
  __IO uint16_t  PWM0COM2;                  /*!< Compare Register 2 for PWM0 and PWM1  */
  __I  uint16_t  RESERVED5;
  __IO uint16_t  PWM0LEN;                   /*!< Period Value register for PWM0 and PWM1 */
  __I  uint16_t  RESERVED6;
  __IO uint16_t  PWM1COM0;                  /*!< Compare Register 0 for PWM2 and PWM3  */
  __I  uint16_t  RESERVED7;
  __IO uint16_t  PWM1COM1;                  /*!< Compare Register 1 for PWM2 and PWM3  */
  __I  uint16_t  RESERVED8;
  __IO uint16_t  PWM1COM2;                  /*!< Compare Register 2 for PWM2 and PWM3  */
  __I  uint16_t  RESERVED9;
  __IO uint16_t  PWM1LEN;                   /*!< Period Value register for PWM2 and PWM3 */
  __I  uint16_t  RESERVED10;
  __IO uint16_t  PWM2COM0;                  /*!< Compare Register 0 for PWM4 and PWM5  */
  __I  uint16_t  RESERVED11;
  __IO uint16_t  PWM2COM1;                  /*!< Compare Register 1 for PWM4 and PWM5  */
  __I  uint16_t  RESERVED12;
  __IO uint16_t  PWM2COM2;                  /*!< Compare Register 2 for PWM4 and PWM5  */
  __I  uint16_t  RESERVED13;
  __IO uint16_t  PWM2LEN;                   /*!< Period Value register for PWM4 and PWM5 */
} ADI_PWM_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          PWMCON0                                    (*(volatile unsigned short int *) 0x40001000)
#define          PWMCON1                                    (*(volatile unsigned char      *) 0x40001004)
#define          PWMCLRI                                    (*(volatile unsigned short int *) 0x40001008)
#define          PWM0COM0                                   (*(volatile unsigned short int *) 0x40001010)
#define          PWM0COM1                                   (*(volatile unsigned short int *) 0x40001014)
#define          PWM0COM2                                   (*(volatile unsigned short int *) 0x40001018)
#define          PWM0LEN                                    (*(volatile unsigned short int *) 0x4000101C)
#define          PWM1COM0                                   (*(volatile unsigned short int *) 0x40001020)
#define          PWM1COM1                                   (*(volatile unsigned short int *) 0x40001024)
#define          PWM1COM2                                   (*(volatile unsigned short int *) 0x40001028)
#define          PWM1LEN                                    (*(volatile unsigned short int *) 0x4000102C)
#define          PWM2COM0                                   (*(volatile unsigned short int *) 0x40001030)
#define          PWM2COM1                                   (*(volatile unsigned short int *) 0x40001034)
#define          PWM2COM2                                   (*(volatile unsigned short int *) 0x40001038)
#define          PWM2LEN                                    (*(volatile unsigned short int *) 0x4000103C)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for PWMCON0*/
#define PWMCON0_RVAL                   0x12 

/* PWMCON0[SYNC] - PWM Synchronization */
#define PWMCON0_SYNC_BBA               (*(volatile unsigned long *) 0x4202003C)
#define PWMCON0_SYNC_MSK               (0x1   << 15 )
#define PWMCON0_SYNC                   (0x1   << 15 )
#define PWMCON0_SYNC_DIS               (0x0   << 15 ) /* DIS                      */
#define PWMCON0_SYNC_EN                (0x1   << 15 ) /* EN                       */

/* PWMCON0[PWM5INV] - Inversion of PWM output */
#define PWMCON0_PWM5INV_BBA            (*(volatile unsigned long *) 0x42020034)
#define PWMCON0_PWM5INV_MSK            (0x1   << 13 )
#define PWMCON0_PWM5INV                (0x1   << 13 )
#define PWMCON0_PWM5INV_DIS            (0x0   << 13 ) /* DIS                      */
#define PWMCON0_PWM5INV_EN             (0x1   << 13 ) /* EN                       */

/* PWMCON0[PWM3INV] - Inversion of PWM output */
#define PWMCON0_PWM3INV_BBA            (*(volatile unsigned long *) 0x42020030)
#define PWMCON0_PWM3INV_MSK            (0x1   << 12 )
#define PWMCON0_PWM3INV                (0x1   << 12 )
#define PWMCON0_PWM3INV_DIS            (0x0   << 12 ) /* DIS                      */
#define PWMCON0_PWM3INV_EN             (0x1   << 12 ) /* EN                       */

/* PWMCON0[PWM1INV] - Inversion of PWM output */
#define PWMCON0_PWM1INV_BBA            (*(volatile unsigned long *) 0x4202002C)
#define PWMCON0_PWM1INV_MSK            (0x1   << 11 )
#define PWMCON0_PWM1INV                (0x1   << 11 )
#define PWMCON0_PWM1INV_DIS            (0x0   << 11 ) /* DIS                      */
#define PWMCON0_PWM1INV_EN             (0x1   << 11 ) /* EN                       */

/* PWMCON0[PWMIEN] - Enables PWM interrupts */
#define PWMCON0_PWMIEN_BBA             (*(volatile unsigned long *) 0x42020028)
#define PWMCON0_PWMIEN_MSK             (0x1   << 10 )
#define PWMCON0_PWMIEN                 (0x1   << 10 )
#define PWMCON0_PWMIEN_DIS             (0x0   << 10 ) /* DIS                      */
#define PWMCON0_PWMIEN_EN              (0x1   << 10 ) /* EN                       */

/* PWMCON0[ENA] - enable PWM outputs */
#define PWMCON0_ENA_BBA                (*(volatile unsigned long *) 0x42020024)
#define PWMCON0_ENA_MSK                (0x1   << 9  )
#define PWMCON0_ENA                    (0x1   << 9  )
#define PWMCON0_ENA_DIS                (0x0   << 9  ) /* DIS                      */
#define PWMCON0_ENA_EN                 (0x1   << 9  ) /* EN                       */

/* PWMCON0[PRE] - PWM Clock Prescaler */
#define PWMCON0_PRE_MSK                (0x7   << 6  )

/* PWMCON0[POINV] - Invert all PWM outputs */
#define PWMCON0_POINV_BBA              (*(volatile unsigned long *) 0x42020014)
#define PWMCON0_POINV_MSK              (0x1   << 5  )
#define PWMCON0_POINV                  (0x1   << 5  )
#define PWMCON0_POINV_DIS              (0x0   << 5  ) /* DIS                      */
#define PWMCON0_POINV_EN               (0x1   << 5  ) /* EN                       */

/* PWMCON0[HOFF] - High Side Off */
#define PWMCON0_HOFF_BBA               (*(volatile unsigned long *) 0x42020010)
#define PWMCON0_HOFF_MSK               (0x1   << 4  )
#define PWMCON0_HOFF                   (0x1   << 4  )
#define PWMCON0_HOFF_DIS               (0x0   << 4  ) /* DIS                      */
#define PWMCON0_HOFF_EN                (0x1   << 4  ) /* EN                       */

/* PWMCON0[LCOMP] - Load Compare Registers */
#define PWMCON0_LCOMP_BBA              (*(volatile unsigned long *) 0x4202000C)
#define PWMCON0_LCOMP_MSK              (0x1   << 3  )
#define PWMCON0_LCOMP                  (0x1   << 3  )
#define PWMCON0_LCOMP_DIS              (0x0   << 3  ) /* DIS                      */
#define PWMCON0_LCOMP_EN               (0x1   << 3  ) /* EN                       */

/* PWMCON0[DIR] - Direction Control */
#define PWMCON0_DIR_BBA                (*(volatile unsigned long *) 0x42020008)
#define PWMCON0_DIR_MSK                (0x1   << 2  )
#define PWMCON0_DIR                    (0x1   << 2  )
#define PWMCON0_DIR_DIS                (0x0   << 2  ) /* DIS                      */
#define PWMCON0_DIR_EN                 (0x1   << 2  ) /* EN                       */

/* PWMCON0[MOD] - Enables H-Bridge Mode */
#define PWMCON0_MOD_BBA                (*(volatile unsigned long *) 0x42020004)
#define PWMCON0_MOD_MSK                (0x1   << 1  )
#define PWMCON0_MOD                    (0x1   << 1  )
#define PWMCON0_MOD_DIS                (0x0   << 1  ) /* DIS                      */
#define PWMCON0_MOD_EN                 (0x1   << 1  ) /* EN                       */

/* PWMCON0[ENABLE] - Enables all PWM outputs */
#define PWMCON0_ENABLE_BBA             (*(volatile unsigned long *) 0x42020000)
#define PWMCON0_ENABLE_MSK             (0x1   << 0  )
#define PWMCON0_ENABLE                 (0x1   << 0  )
#define PWMCON0_ENABLE_DIS             (0x0   << 0  ) /* DIS                      */
#define PWMCON0_ENABLE_EN              (0x1   << 0  ) /* EN                       */

/* Reset Value for PWMCON1*/
#define PWMCON1_RVAL                   0x0 

/* PWMCON1[CONVSTART] - Enable adc conversion start from pwm */
#define PWMCON1_CONVSTART_BBA          (*(volatile unsigned long *) 0x4202009C)
#define PWMCON1_CONVSTART_MSK          (0x1   << 7  )
#define PWMCON1_CONVSTART              (0x1   << 7  )
#define PWMCON1_CONVSTART_DIS          (0x0   << 7  ) /* DIS                      */
#define PWMCON1_CONVSTART_EN           (0x1   << 7  ) /* EN                       */

/* PWMCON1[TRIPEN] - Enable PWM trip functionality */
#define PWMCON1_TRIPEN_BBA             (*(volatile unsigned long *) 0x42020098)
#define PWMCON1_TRIPEN_MSK             (0x1   << 6  )
#define PWMCON1_TRIPEN                 (0x1   << 6  )
#define PWMCON1_TRIPEN_DIS             (0x0   << 6  ) /* DIS                      */
#define PWMCON1_TRIPEN_EN              (0x1   << 6  ) /* EN                       */

/* PWMCON1[CONVSTARTDELAY] - ADC conversion start delay configuration */
#define PWMCON1_CONVSTARTDELAY_MSK     (0xF   << 0  )

/* Reset Value for PWMCLRI*/
#define PWMCLRI_RVAL                   0x0 

/* PWMCLRI[TRIP] - Clear the latched trip interrupt */
#define PWMCLRI_TRIP_BBA               (*(volatile unsigned long *) 0x42020110)
#define PWMCLRI_TRIP_MSK               (0x1   << 4  )
#define PWMCLRI_TRIP                   (0x1   << 4  )
#define PWMCLRI_TRIP_DIS               (0x0   << 4  ) /* DIS                      */
#define PWMCLRI_TRIP_EN                (0x1   << 4  ) /* EN                       */

/* PWMCLRI[PWM2] - Clear the latched PWM2 interrupt */
#define PWMCLRI_PWM2_BBA               (*(volatile unsigned long *) 0x42020108)
#define PWMCLRI_PWM2_MSK               (0x1   << 2  )
#define PWMCLRI_PWM2                   (0x1   << 2  )
#define PWMCLRI_PWM2_DIS               (0x0   << 2  ) /* DIS                      */
#define PWMCLRI_PWM2_EN                (0x1   << 2  ) /* EN                       */

/* PWMCLRI[PWM1] - Clear the latched PWM1 interrupt */
#define PWMCLRI_PWM1_BBA               (*(volatile unsigned long *) 0x42020104)
#define PWMCLRI_PWM1_MSK               (0x1   << 1  )
#define PWMCLRI_PWM1                   (0x1   << 1  )
#define PWMCLRI_PWM1_DIS               (0x0   << 1  ) /* DIS                      */
#define PWMCLRI_PWM1_EN                (0x1   << 1  ) /* EN                       */

/* PWMCLRI[PWM0] - Clear the latched PWM0 interrupt */
#define PWMCLRI_PWM0_BBA               (*(volatile unsigned long *) 0x42020100)
#define PWMCLRI_PWM0_MSK               (0x1   << 0  )
#define PWMCLRI_PWM0                   (0x1   << 0  )
#define PWMCLRI_PWM0_DIS               (0x0   << 0  ) /* DIS                      */
#define PWMCLRI_PWM0_EN                (0x1   << 0  ) /* EN                       */

/* Reset Value for PWM0COM0*/
#define PWM0COM0_RVAL                  0x0 

/* Reset Value for PWM0COM1*/
#define PWM0COM1_RVAL                  0x0 

/* Reset Value for PWM0COM2*/
#define PWM0COM2_RVAL                  0x0 

/* Reset Value for PWM0LEN*/
#define PWM0LEN_RVAL                   0x0 

/* Reset Value for PWM1COM0*/
#define PWM1COM0_RVAL                  0x0 

/* Reset Value for PWM1COM1*/
#define PWM1COM1_RVAL                  0x0 

/* Reset Value for PWM1COM2*/
#define PWM1COM2_RVAL                  0x0 

/* Reset Value for PWM1LEN*/
#define PWM1LEN_RVAL                   0x0 

/* Reset Value for PWM2COM0*/
#define PWM2COM0_RVAL                  0x0 

/* Reset Value for PWM2COM1*/
#define PWM2COM1_RVAL                  0x0 

/* Reset Value for PWM2COM2*/
#define PWM2COM2_RVAL                  0x0 

/* Reset Value for PWM2LEN*/
#define PWM2LEN_RVAL                   0x0 
// ------------------------------------------------------------------------------------------------
// -----                                        PWRCTL                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Power Management Unit (pADI_PWRCTL)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_PWRCTL Structure                  */
  __IO uint8_t   PWRMOD;                    /*!< Power modes register                  */
  __I  uint8_t   RESERVED0[3];
  __IO uint16_t  PWRKEY;                    /*!< Key protection for the PWRMOD register. */
} ADI_PWRCTL_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          PWRMOD                                     (*(volatile unsigned char      *) 0x40002400)
#define          PWRKEY                                     (*(volatile unsigned short int *) 0x40002404)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for PWRMOD*/
#define PWRMOD_RVAL                    0x40 

/* PWRMOD[WICENACK] - For Deepsleep mode only */
#define PWRMOD_WICENACK_BBA            (*(volatile unsigned long *) 0x4204800C)
#define PWRMOD_WICENACK_MSK            (0x1   << 3  )
#define PWRMOD_WICENACK                (0x1   << 3  )
#define PWRMOD_WICENACK_DIS            (0x0   << 3  ) /* DIS                      */
#define PWRMOD_WICENACK_EN             (0x1   << 3  ) /* EN                       */

/* PWRMOD[MOD] - Power Mode */
#define PWRMOD_MOD_MSK                 (0x7   << 0  )
#define PWRMOD_MOD_FULLACTIVE          (0x0   << 0  ) /* FULLACTIVE               */
#define PWRMOD_MOD_MCUHALT             (0x1   << 0  ) /* MCUHALT                  */
#define PWRMOD_MOD_PERHALT             (0x2   << 0  ) /* PERHALT                  */
#define PWRMOD_MOD_SYSHALT             (0x3   << 0  ) /* SYSHALT                  */
#define PWRMOD_MOD_TOTALHALT           (0x4   << 0  ) /* TOTALHALT                */
#define PWRMOD_MOD_HIBERNATE           (0x5   << 0  ) /* HIBERNATE                */

/* Reset Value for PWRKEY*/
#define PWRKEY_RVAL                    0x0 

/* PWRKEY[VALUE] - Key value */
#define PWRKEY_VALUE_MSK               (0xFFFF << 0  )
#define PWRKEY_VALUE_KEY1              (0x4859 << 0  ) /* KEY1                     */
#define PWRKEY_VALUE_KEY2              (0xF27B << 0  ) /* KEY2                     */
// ------------------------------------------------------------------------------------------------
// -----                                        RESET                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Reset (pADI_RESET)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_RESET Structure                   */
  
  union {
    __IO uint8_t   RSTSTA;                  /*!< Reset Status                          */
    __IO uint8_t   RSTCLR;                  /*!< Reset Status Clear                    */
  } ;
} ADI_RESET_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          RSTSTA                                     (*(volatile unsigned char      *) 0x40002440)
#define          RSTCLR                                     (*(volatile unsigned char      *) 0x40002440)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for RSTSTA*/
#define RSTSTA_RVAL                    0x1 

/* RSTSTA[SWRST] - Software reset status bit */
#define RSTSTA_SWRST_BBA               (*(volatile unsigned long *) 0x4204880C)
#define RSTSTA_SWRST_MSK               (0x1   << 3  )
#define RSTSTA_SWRST                   (0x1   << 3  )
#define RSTSTA_SWRST_CLR               (0x0   << 3  ) /* CLR                      */
#define RSTSTA_SWRST_SET               (0x1   << 3  ) /* SET                      */

/* RSTSTA[WDRST] - Watchdog reset status bit */
#define RSTSTA_WDRST_BBA               (*(volatile unsigned long *) 0x42048808)
#define RSTSTA_WDRST_MSK               (0x1   << 2  )
#define RSTSTA_WDRST                   (0x1   << 2  )
#define RSTSTA_WDRST_CLR               (0x0   << 2  ) /* CLR                      */
#define RSTSTA_WDRST_SET               (0x1   << 2  ) /* SET                      */

/* RSTSTA[EXTRST] - External reset status bit */
#define RSTSTA_EXTRST_BBA              (*(volatile unsigned long *) 0x42048804)
#define RSTSTA_EXTRST_MSK              (0x1   << 1  )
#define RSTSTA_EXTRST                  (0x1   << 1  )
#define RSTSTA_EXTRST_CLR              (0x0   << 1  ) /* CLR                      */
#define RSTSTA_EXTRST_SET              (0x1   << 1  ) /* SET                      */

/* RSTSTA[POR] - Power-on reset status bit */
#define RSTSTA_POR_BBA                 (*(volatile unsigned long *) 0x42048800)
#define RSTSTA_POR_MSK                 (0x1   << 0  )
#define RSTSTA_POR                     (0x1   << 0  )
#define RSTSTA_POR_CLR                 (0x0   << 0  ) /* CLR                      */
#define RSTSTA_POR_SET                 (0x1   << 0  ) /* SET                      */

/* Reset Value for RSTCLR*/
#define RSTCLR_RVAL                    0x1 

/* RSTCLR[SWRST] - Software reset status bit */
#define RSTCLR_SWRST_BBA               (*(volatile unsigned long *) 0x4204880C)
#define RSTCLR_SWRST_MSK               (0x1   << 3  )
#define RSTCLR_SWRST                   (0x1   << 3  )
#define RSTCLR_SWRST_DIS               (0x0   << 3  ) /* DIS                      */
#define RSTCLR_SWRST_EN                (0x1   << 3  ) /* EN                       */

/* RSTCLR[WDRST] - Watchdog reset status bit */
#define RSTCLR_WDRST_BBA               (*(volatile unsigned long *) 0x42048808)
#define RSTCLR_WDRST_MSK               (0x1   << 2  )
#define RSTCLR_WDRST                   (0x1   << 2  )
#define RSTCLR_WDRST_DIS               (0x0   << 2  ) /* DIS                      */
#define RSTCLR_WDRST_EN                (0x1   << 2  ) /* EN                       */

/* RSTCLR[EXTRST] - External reset status bit */
#define RSTCLR_EXTRST_BBA              (*(volatile unsigned long *) 0x42048804)
#define RSTCLR_EXTRST_MSK              (0x1   << 1  )
#define RSTCLR_EXTRST                  (0x1   << 1  )
#define RSTCLR_EXTRST_DIS              (0x0   << 1  ) /* DIS                      */
#define RSTCLR_EXTRST_EN               (0x1   << 1  ) /* EN                       */

/* RSTCLR[POR] - Power-on reset status bit */
#define RSTCLR_POR_BBA                 (*(volatile unsigned long *) 0x42048800)
#define RSTCLR_POR_MSK                 (0x1   << 0  )
#define RSTCLR_POR                     (0x1   << 0  )
#define RSTCLR_POR_DIS                 (0x0   << 0  ) /* DIS                      */
#define RSTCLR_POR_EN                  (0x1   << 0  ) /* EN                       */
// ------------------------------------------------------------------------------------------------
// -----                                        INTERRUPT                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Interrupts (pADI_INTERRUPT)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_INTERRUPT Structure               */
  __IO uint16_t  EI0CFG;                    /*!< External Interrupt configuration register 0 */
  __I  uint16_t  RESERVED0;
  __IO uint16_t  EI1CFG;                    /*!< External Interrupt configuration register 1 */
  __I  uint16_t  RESERVED1[5];
  __IO uint16_t  EICLR;                     /*!< External Interrupts Clear register    */
} ADI_INTERRUPT_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          EI0CFG                                     (*(volatile unsigned short int *) 0x40002420)
#define          EI1CFG                                     (*(volatile unsigned short int *) 0x40002424)
#define          EICLR                                      (*(volatile unsigned short int *) 0x40002430)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for EI0CFG*/
#define EI0CFG_RVAL                    0x0 

/* EI0CFG[IRQ3EN] - External Interrupt 3 Enable */
#define EI0CFG_IRQ3EN_BBA              (*(volatile unsigned long *) 0x4204843C)
#define EI0CFG_IRQ3EN_MSK              (0x1   << 15 )
#define EI0CFG_IRQ3EN                  (0x1   << 15 )
#define EI0CFG_IRQ3EN_DIS              (0x0   << 15 ) /* DIS                      */
#define EI0CFG_IRQ3EN_EN               (0x1   << 15 ) /* EN                       */

/* EI0CFG[IRQ3MDE] - External Interrupt 0 Mode */
#define EI0CFG_IRQ3MDE_MSK             (0x7   << 12 )
#define EI0CFG_IRQ3MDE_RISE            (0x0   << 12 ) /* RISE                     */
#define EI0CFG_IRQ3MDE_FALL            (0x1   << 12 ) /* FALL                     */
#define EI0CFG_IRQ3MDE_RISEORFALL      (0x2   << 12 ) /* RISEORFALL               */
#define EI0CFG_IRQ3MDE_HIGHLEVEL       (0x3   << 12 ) /* HIGHLEVEL                */
#define EI0CFG_IRQ3MDE_LOWLEVEL        (0x4   << 12 ) /* LOWLEVEL                 */

/* EI0CFG[IRQ2EN] - External Interrupt 2 Enable */
#define EI0CFG_IRQ2EN_BBA              (*(volatile unsigned long *) 0x4204842C)
#define EI0CFG_IRQ2EN_MSK              (0x1   << 11 )
#define EI0CFG_IRQ2EN                  (0x1   << 11 )
#define EI0CFG_IRQ2EN_DIS              (0x0   << 11 ) /* DIS                      */
#define EI0CFG_IRQ2EN_EN               (0x1   << 11 ) /* EN                       */

/* EI0CFG[IRQ2MDE] - External Interrupt 2 Mode */
#define EI0CFG_IRQ2MDE_MSK             (0x7   << 8  )
#define EI0CFG_IRQ2MDE_RISE            (0x0   << 8  ) /* RISE                     */
#define EI0CFG_IRQ2MDE_FALL            (0x1   << 8  ) /* FALL                     */
#define EI0CFG_IRQ2MDE_RISEORFALL      (0x2   << 8  ) /* RISEORFALL               */
#define EI0CFG_IRQ2MDE_HIGHLEVEL       (0x3   << 8  ) /* HIGHLEVEL                */
#define EI0CFG_IRQ2MDE_LOWLEVEL        (0x4   << 8  ) /* LOWLEVEL                 */

/* EI0CFG[IRQ1EN] - External Interrupt 1 Enable */
#define EI0CFG_IRQ1EN_BBA              (*(volatile unsigned long *) 0x4204841C)
#define EI0CFG_IRQ1EN_MSK              (0x1   << 7  )
#define EI0CFG_IRQ1EN                  (0x1   << 7  )
#define EI0CFG_IRQ1EN_DIS              (0x0   << 7  ) /* DIS                      */
#define EI0CFG_IRQ1EN_EN               (0x1   << 7  ) /* EN                       */

/* EI0CFG[IRQ1MDE] - External Interrupt 1 Mode */
#define EI0CFG_IRQ1MDE_MSK             (0x7   << 4  )
#define EI0CFG_IRQ1MDE_RISE            (0x0   << 4  ) /* RISE                     */
#define EI0CFG_IRQ1MDE_FALL            (0x1   << 4  ) /* FALL                     */
#define EI0CFG_IRQ1MDE_RISEORFALL      (0x2   << 4  ) /* RISEORFALL               */
#define EI0CFG_IRQ1MDE_HIGHLEVEL       (0x3   << 4  ) /* HIGHLEVEL                */
#define EI0CFG_IRQ1MDE_LOWLEVEL        (0x4   << 4  ) /* LOWLEVEL                 */

/* EI0CFG[IRQ0EN] - External Interrupt 0 Enable */
#define EI0CFG_IRQ0EN_BBA              (*(volatile unsigned long *) 0x4204840C)
#define EI0CFG_IRQ0EN_MSK              (0x1   << 3  )
#define EI0CFG_IRQ0EN                  (0x1   << 3  )
#define EI0CFG_IRQ0EN_DIS              (0x0   << 3  ) /* DIS                      */
#define EI0CFG_IRQ0EN_EN               (0x1   << 3  ) /* EN                       */

/* EI0CFG[IRQ0MDE] - External Interrupt 0 Mode */
#define EI0CFG_IRQ0MDE_MSK             (0x7   << 0  )
#define EI0CFG_IRQ0MDE_RISE            (0x0   << 0  ) /* RISE                     */
#define EI0CFG_IRQ0MDE_FALL            (0x1   << 0  ) /* FALL                     */
#define EI0CFG_IRQ0MDE_RISEORFALL      (0x2   << 0  ) /* RISEORFALL               */
#define EI0CFG_IRQ0MDE_HIGHLEVEL       (0x3   << 0  ) /* HIGHLEVEL                */
#define EI0CFG_IRQ0MDE_LOWLEVEL        (0x4   << 0  ) /* LOWLEVEL                 */

/* Reset Value for EI1CFG*/
#define EI1CFG_RVAL                    0x0 

/* EI1CFG[IRQ7EN] - External Interrupt 7 Enable */
#define EI1CFG_IRQ7EN_BBA              (*(volatile unsigned long *) 0x420484BC)
#define EI1CFG_IRQ7EN_MSK              (0x1   << 15 )
#define EI1CFG_IRQ7EN                  (0x1   << 15 )
#define EI1CFG_IRQ7EN_DIS              (0x0   << 15 ) /* DIS                      */
#define EI1CFG_IRQ7EN_EN               (0x1   << 15 ) /* EN                       */

/* EI1CFG[IRQ7MDE] - External Interrupt 7 Mode */
#define EI1CFG_IRQ7MDE_MSK             (0x7   << 12 )
#define EI1CFG_IRQ7MDE_RISE            (0x0   << 12 ) /* RISE                     */
#define EI1CFG_IRQ7MDE_FALL            (0x1   << 12 ) /* FALL                     */
#define EI1CFG_IRQ7MDE_RISEORFALL      (0x2   << 12 ) /* RISEORFALL               */
#define EI1CFG_IRQ7MDE_HIGHLEVEL       (0x3   << 12 ) /* HIGHLEVEL                */
#define EI1CFG_IRQ7MDE_LOWLEVEL        (0x4   << 12 ) /* LOWLEVEL                 */

/* EI1CFG[IRQ6EN] - External Interrupt 6 Enable */
#define EI1CFG_IRQ6EN_BBA              (*(volatile unsigned long *) 0x420484AC)
#define EI1CFG_IRQ6EN_MSK              (0x1   << 11 )
#define EI1CFG_IRQ6EN                  (0x1   << 11 )
#define EI1CFG_IRQ6EN_DIS              (0x0   << 11 ) /* DIS                      */
#define EI1CFG_IRQ6EN_EN               (0x1   << 11 ) /* EN                       */

/* EI1CFG[IRQ6MDE] - External Interrupt 6 Mode */
#define EI1CFG_IRQ6MDE_MSK             (0x7   << 8  )
#define EI1CFG_IRQ6MDE_RISE            (0x0   << 8  ) /* RISE                     */
#define EI1CFG_IRQ6MDE_FALL            (0x1   << 8  ) /* FALL                     */
#define EI1CFG_IRQ6MDE_RISEORFALL      (0x2   << 8  ) /* RISEORFALL               */
#define EI1CFG_IRQ6MDE_HIGHLEVEL       (0x3   << 8  ) /* HIGHLEVEL                */
#define EI1CFG_IRQ6MDE_LOWLEVEL        (0x4   << 8  ) /* LOWLEVEL                 */

/* EI1CFG[IRQ5EN] - External Interrupt 5 Enable */
#define EI1CFG_IRQ5EN_BBA              (*(volatile unsigned long *) 0x4204849C)
#define EI1CFG_IRQ5EN_MSK              (0x1   << 7  )
#define EI1CFG_IRQ5EN                  (0x1   << 7  )
#define EI1CFG_IRQ5EN_DIS              (0x0   << 7  ) /* DIS                      */
#define EI1CFG_IRQ5EN_EN               (0x1   << 7  ) /* EN                       */

/* EI1CFG[IRQ5MDE] - External Interrupt 5 Mode */
#define EI1CFG_IRQ5MDE_MSK             (0x7   << 4  )
#define EI1CFG_IRQ5MDE_RISE            (0x0   << 4  ) /* RISE                     */
#define EI1CFG_IRQ5MDE_FALL            (0x1   << 4  ) /* FALL                     */
#define EI1CFG_IRQ5MDE_RISEORFALL      (0x2   << 4  ) /* RISEORFALL               */
#define EI1CFG_IRQ5MDE_HIGHLEVEL       (0x3   << 4  ) /* HIGHLEVEL                */
#define EI1CFG_IRQ5MDE_LOWLEVEL        (0x4   << 4  ) /* LOWLEVEL                 */

/* EI1CFG[IRQ4EN] - External Interrupt 4 Enable */
#define EI1CFG_IRQ4EN_BBA              (*(volatile unsigned long *) 0x4204848C)
#define EI1CFG_IRQ4EN_MSK              (0x1   << 3  )
#define EI1CFG_IRQ4EN                  (0x1   << 3  )
#define EI1CFG_IRQ4EN_DIS              (0x0   << 3  ) /* DIS                      */
#define EI1CFG_IRQ4EN_EN               (0x1   << 3  ) /* EN                       */

/* EI1CFG[IRQ4MDE] - External Interrupt 4 Mode */
#define EI1CFG_IRQ4MDE_MSK             (0x7   << 0  )
#define EI1CFG_IRQ4MDE_RISE            (0x0   << 0  ) /* RISE                     */
#define EI1CFG_IRQ4MDE_FALL            (0x1   << 0  ) /* FALL                     */
#define EI1CFG_IRQ4MDE_RISEORFALL      (0x2   << 0  ) /* RISEORFALL               */
#define EI1CFG_IRQ4MDE_HIGHLEVEL       (0x3   << 0  ) /* HIGHLEVEL                */
#define EI1CFG_IRQ4MDE_LOWLEVEL        (0x4   << 0  ) /* LOWLEVEL                 */

/* Reset Value for EICLR*/
#define EICLR_RVAL                     0x0 

/* EICLR[IRQ7] - Clears External interrupt 7 internal flag */
#define EICLR_IRQ7_BBA                 (*(volatile unsigned long *) 0x4204861C)
#define EICLR_IRQ7_MSK                 (0x1   << 7  )
#define EICLR_IRQ7                     (0x1   << 7  )
#define EICLR_IRQ7_CLR                 (0x1   << 7  ) /* CLR                      */

/* EICLR[IRQ6] - Clears External interrupt 6 internal flag */
#define EICLR_IRQ6_BBA                 (*(volatile unsigned long *) 0x42048618)
#define EICLR_IRQ6_MSK                 (0x1   << 6  )
#define EICLR_IRQ6                     (0x1   << 6  )
#define EICLR_IRQ6_CLR                 (0x1   << 6  ) /* CLR                      */

/* EICLR[IRQ5] - Clears External interrupt 5 internal flag */
#define EICLR_IRQ5_BBA                 (*(volatile unsigned long *) 0x42048614)
#define EICLR_IRQ5_MSK                 (0x1   << 5  )
#define EICLR_IRQ5                     (0x1   << 5  )
#define EICLR_IRQ5_CLR                 (0x1   << 5  ) /* CLR                      */

/* EICLR[IRQ4] - Clears External interrupt 4 internal flag */
#define EICLR_IRQ4_BBA                 (*(volatile unsigned long *) 0x42048610)
#define EICLR_IRQ4_MSK                 (0x1   << 4  )
#define EICLR_IRQ4                     (0x1   << 4  )
#define EICLR_IRQ4_CLR                 (0x1   << 4  ) /* CLR                      */

/* EICLR[IRQ3] - Clears External interrupt 3 internal flag */
#define EICLR_IRQ3_BBA                 (*(volatile unsigned long *) 0x4204860C)
#define EICLR_IRQ3_MSK                 (0x1   << 3  )
#define EICLR_IRQ3                     (0x1   << 3  )
#define EICLR_IRQ3_CLR                 (0x1   << 3  ) /* CLR                      */

/* EICLR[IRQ2] - Clears External interrupt 2 internal flag */
#define EICLR_IRQ2_BBA                 (*(volatile unsigned long *) 0x42048608)
#define EICLR_IRQ2_MSK                 (0x1   << 2  )
#define EICLR_IRQ2                     (0x1   << 2  )
#define EICLR_IRQ2_CLR                 (0x1   << 2  ) /* CLR                      */

/* EICLR[IRQ1] - Clears External interrupt 1 internal flag */
#define EICLR_IRQ1_BBA                 (*(volatile unsigned long *) 0x42048604)
#define EICLR_IRQ1_MSK                 (0x1   << 1  )
#define EICLR_IRQ1                     (0x1   << 1  )
#define EICLR_IRQ1_CLR                 (0x1   << 1  ) /* CLR                      */

/* EICLR[IRQ0] - Clears External interrupt 0 internal flag */
#define EICLR_IRQ0_BBA                 (*(volatile unsigned long *) 0x42048600)
#define EICLR_IRQ0_MSK                 (0x1   << 0  )
#define EICLR_IRQ0                     (0x1   << 0  )
#define EICLR_IRQ0_CLR                 (0x1   << 0  ) /* CLR                      */
// ------------------------------------------------------------------------------------------------
// -----                                        WDT                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Watchdog Timer (pADI_WDT)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_WDT Structure                     */
  __IO uint16_t  T3LD;                      /*!< Load value.                           */
  __I  uint16_t  RESERVED0;
  __IO uint16_t  T3VAL;                     /*!< "Current count value, read only."     */
  __I  uint16_t  RESERVED1;
  __IO uint16_t  T3CON;                     /*!< Control Register                      */
  __I  uint16_t  RESERVED2;
  __IO uint16_t  T3CLRI;                    /*!< "Clear interrupt, write only."        */
  __I  uint16_t  RESERVED3[5];
  __IO uint16_t  T3STA;                     /*!< "Status register, read only."         */
} ADI_WDT_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          T3LD                                       (*(volatile unsigned short int *) 0x40002580)
#define          T3VAL                                      (*(volatile unsigned short int *) 0x40002584)
#define          T3CON                                      (*(volatile unsigned short int *) 0x40002588)
#define          T3CLRI                                     (*(volatile unsigned short int *) 0x4000258C)
#define          T3STA                                      (*(volatile unsigned short int *) 0x40002598)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for T3LD*/
#define T3LD_RVAL                      0x1000 

/* T3LD[VALUE] - Current Value */
#define T3LD_VALUE_MSK                 (0xFFFF << 0  )

/* Reset Value for T3VAL*/
#define T3VAL_RVAL                     0x1000 

/* T3VAL[VALUE] - Current Value */
#define T3VAL_VALUE_MSK                (0xFFFF << 0  )

/* Reset Value for T3CON*/
#define T3CON_RVAL                     0xE9 

/* T3CON[MOD] - Mode */
#define T3CON_MOD_BBA                  (*(volatile unsigned long *) 0x4204B118)
#define T3CON_MOD_MSK                  (0x1   << 6  )
#define T3CON_MOD                      (0x1   << 6  )
#define T3CON_MOD_FREERUN              (0x0   << 6  ) /* FREERUN                  */
#define T3CON_MOD_PERIODIC             (0x1   << 6  ) /* PERIODIC                 */

/* T3CON[ENABLE] - Enable */
#define T3CON_ENABLE_BBA               (*(volatile unsigned long *) 0x4204B114)
#define T3CON_ENABLE_MSK               (0x1   << 5  )
#define T3CON_ENABLE                   (0x1   << 5  )
#define T3CON_ENABLE_DIS               (0x0   << 5  ) /* DIS                      */
#define T3CON_ENABLE_EN                (0x1   << 5  ) /* EN                       */

/* T3CON[PRE] - Prescaler */
#define T3CON_PRE_MSK                  (0x3   << 2  )
#define T3CON_PRE_DIV1                 (0x0   << 2  ) /* DIV1                     */
#define T3CON_PRE_DIV16                (0x1   << 2  ) /* DIV16                    */
#define T3CON_PRE_DIV256               (0x2   << 2  ) /* DIV256                   */
#define T3CON_PRE_DIV4096              (0x3   << 2  ) /* DIV4096                  */

/* T3CON[IRQ] - Timer Interrupt , */
#define T3CON_IRQ_BBA                  (*(volatile unsigned long *) 0x4204B104)
#define T3CON_IRQ_MSK                  (0x1   << 1  )
#define T3CON_IRQ                      (0x1   << 1  )
#define T3CON_IRQ_DIS                  (0x0   << 1  ) /* DIS                      */
#define T3CON_IRQ_EN                   (0x1   << 1  ) /* EN                       */

/* T3CON[PD] - Power down clear */
#define T3CON_PD_BBA                   (*(volatile unsigned long *) 0x4204B100)
#define T3CON_PD_MSK                   (0x1   << 0  )
#define T3CON_PD                       (0x1   << 0  )
#define T3CON_PD_DIS                   (0x0   << 0  ) /* DIS                      */
#define T3CON_PD_EN                    (0x1   << 0  ) /* EN                       */

/* Reset Value for T3CLRI*/
#define T3CLRI_RVAL                    0x0 

/* T3CLRI[VALUE] - Clear Watchdog */
#define T3CLRI_VALUE_MSK               (0xFFFF << 0  )
#define T3CLRI_VALUE_CLR               (0xCCCC << 0  ) /* CLR                      */

/* Reset Value for T3STA*/
#define T3STA_RVAL                     0x20 

/* T3STA[LOCK] - Lock status bit */
#define T3STA_LOCK_BBA                 (*(volatile unsigned long *) 0x4204B310)
#define T3STA_LOCK_MSK                 (0x1   << 4  )
#define T3STA_LOCK                     (0x1   << 4  )
#define T3STA_LOCK_CLR                 (0x0   << 4  ) /* CLR                      */
#define T3STA_LOCK_SET                 (0x1   << 4  ) /* SET                      */

/* T3STA[CON] - T3CON write sync in progress */
#define T3STA_CON_BBA                  (*(volatile unsigned long *) 0x4204B30C)
#define T3STA_CON_MSK                  (0x1   << 3  )
#define T3STA_CON                      (0x1   << 3  )
#define T3STA_CON_CLR                  (0x0   << 3  ) /* CLR                      */
#define T3STA_CON_SET                  (0x1   << 3  ) /* SET                      */

/* T3STA[LD] - T3LD write sync in progress */
#define T3STA_LD_BBA                   (*(volatile unsigned long *) 0x4204B308)
#define T3STA_LD_MSK                   (0x1   << 2  )
#define T3STA_LD                       (0x1   << 2  )
#define T3STA_LD_CLR                   (0x0   << 2  ) /* CLR                      */
#define T3STA_LD_SET                   (0x1   << 2  ) /* SET                      */

/* T3STA[CLRI] - T3CLRI write sync in progress */
#define T3STA_CLRI_BBA                 (*(volatile unsigned long *) 0x4204B304)
#define T3STA_CLRI_MSK                 (0x1   << 1  )
#define T3STA_CLRI                     (0x1   << 1  )
#define T3STA_CLRI_CLR                 (0x0   << 1  ) /* CLR                      */
#define T3STA_CLRI_SET                 (0x1   << 1  ) /* SET                      */

/* T3STA[IRQ] - Interrupt Pending */
#define T3STA_IRQ_BBA                  (*(volatile unsigned long *) 0x4204B300)
#define T3STA_IRQ_MSK                  (0x1   << 0  )
#define T3STA_IRQ                      (0x1   << 0  )
#define T3STA_IRQ_CLR                  (0x0   << 0  ) /* CLR                      */
#define T3STA_IRQ_SET                  (0x1   << 0  ) /* SET                      */
// ------------------------------------------------------------------------------------------------
// -----                                        WUT                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief WakeUp Timer (pADI_WUT)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_WUT Structure                     */
  __IO uint16_t  T2VAL0;                    /*!< Current count value LSB               */
  __I  uint16_t  RESERVED0;
  __IO uint16_t  T2VAL1;                    /*!< Current count value MSB               */
  __I  uint16_t  RESERVED1;
  __IO uint16_t  T2CON;                     /*!< Control Register                      */
  __I  uint16_t  RESERVED2;
  __IO uint16_t  T2INC;                     /*!< 12-bit register. Wake up field A      */
  __I  uint16_t  RESERVED3;
  __IO uint16_t  T2WUFB0;                   /*!< Wake up field B  LSB                  */
  __I  uint16_t  RESERVED4;
  __IO uint16_t  T2WUFB1;                   /*!< Wake up field B  MSB                  */
  __I  uint16_t  RESERVED5;
  __IO uint16_t  T2WUFC0;                   /*!< Wake up field C  LSB                  */
  __I  uint16_t  RESERVED6;
  __IO uint16_t  T2WUFC1;                   /*!< Wake up field C  MSB                  */
  __I  uint16_t  RESERVED7;
  __IO uint16_t  T2WUFD0;                   /*!< Wake up field D  LSB                  */
  __I  uint16_t  RESERVED8;
  __IO uint16_t  T2WUFD1;                   /*!< Wake up field D  MSB                  */
  __I  uint16_t  RESERVED9;
  __IO uint16_t  T2IEN;                     /*!< Interrupt enable                      */
  __I  uint16_t  RESERVED10;
  __IO uint16_t  T2STA;                     /*!< Status                                */
  __I  uint16_t  RESERVED11;
  __IO uint16_t  T2CLRI;                    /*!< Clear interrupts. Write only.         */
  __I  uint16_t  RESERVED12[5];
  __IO uint16_t  T2WUFA0;                   /*!< Wake up field A  LSB.                 */
  __I  uint16_t  RESERVED13;
  __IO uint16_t  T2WUFA1;                   /*!< Wake up field A MSB.                  */
} ADI_WUT_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          T2VAL0                                     (*(volatile unsigned short int *) 0x40002500)
#define          T2VAL1                                     (*(volatile unsigned short int *) 0x40002504)
#define          T2CON                                      (*(volatile unsigned short int *) 0x40002508)
#define          T2INC                                      (*(volatile unsigned short int *) 0x4000250C)
#define          T2WUFB0                                    (*(volatile unsigned short int *) 0x40002510)
#define          T2WUFB1                                    (*(volatile unsigned short int *) 0x40002514)
#define          T2WUFC0                                    (*(volatile unsigned short int *) 0x40002518)
#define          T2WUFC1                                    (*(volatile unsigned short int *) 0x4000251C)
#define          T2WUFD0                                    (*(volatile unsigned short int *) 0x40002520)
#define          T2WUFD1                                    (*(volatile unsigned short int *) 0x40002524)
#define          T2IEN                                      (*(volatile unsigned short int *) 0x40002528)
#define          T2STA                                      (*(volatile unsigned short int *) 0x4000252C)
#define          T2CLRI                                     (*(volatile unsigned short int *) 0x40002530)
#define          T2WUFA0                                    (*(volatile unsigned short int *) 0x4000253C)
#define          T2WUFA1                                    (*(volatile unsigned short int *) 0x40002540)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for T2VAL0*/
#define T2VAL0_RVAL                    0x0 

/* T2VAL0[VALUE] - Current Value */
#define T2VAL0_VALUE_MSK               (0xFFFF << 0  )

/* Reset Value for T2VAL1*/
#define T2VAL1_RVAL                    0x0 

/* T2VAL1[VALUE] - Current Value */
#define T2VAL1_VALUE_MSK               (0xFFFF << 0  )

/* Reset Value for T2CON*/
#define T2CON_RVAL                     0x40 

/* T2CON[STOPINC] - Stop wake up field A being updated */
#define T2CON_STOPINC_BBA              (*(volatile unsigned long *) 0x4204A12C)
#define T2CON_STOPINC_MSK              (0x1   << 11 )
#define T2CON_STOPINC                  (0x1   << 11 )
#define T2CON_STOPINC_DIS              (0x0   << 11 ) /* DIS                      */
#define T2CON_STOPINC_EN               (0x1   << 11 ) /* EN                       */

/* T2CON[CLK] - Clock */
#define T2CON_CLK_MSK                  (0x3   << 9  )
#define T2CON_CLK_PCLK                 (0x0   << 9  ) /* PCLK                     */
#define T2CON_CLK_LFXTAL               (0x1   << 9  ) /* LFXTAL                   */
#define T2CON_CLK_LFOSC                (0x2   << 9  ) /* LFOSC                    */
#define T2CON_CLK_EXTCLK               (0x3   << 9  ) /* EXTCLK                   */

/* T2CON[WUEN] - WUEN */
#define T2CON_WUEN_BBA                 (*(volatile unsigned long *) 0x4204A120)
#define T2CON_WUEN_MSK                 (0x1   << 8  )
#define T2CON_WUEN                     (0x1   << 8  )
#define T2CON_WUEN_DIS                 (0x0   << 8  ) /* DIS                      */
#define T2CON_WUEN_EN                  (0x1   << 8  ) /* EN                       */

/* T2CON[ENABLE] - Enable */
#define T2CON_ENABLE_BBA               (*(volatile unsigned long *) 0x4204A11C)
#define T2CON_ENABLE_MSK               (0x1   << 7  )
#define T2CON_ENABLE                   (0x1   << 7  )
#define T2CON_ENABLE_DIS               (0x0   << 7  ) /* DIS                      */
#define T2CON_ENABLE_EN                (0x1   << 7  ) /* EN                       */

/* T2CON[MOD] - Mode */
#define T2CON_MOD_BBA                  (*(volatile unsigned long *) 0x4204A118)
#define T2CON_MOD_MSK                  (0x1   << 6  )
#define T2CON_MOD                      (0x1   << 6  )
#define T2CON_MOD_PERIODIC             (0x0   << 6  ) /* PERIODIC                 */
#define T2CON_MOD_FREERUN              (0x1   << 6  ) /* FREERUN                  */

/* T2CON[FREEZE] - Freeze */
#define T2CON_FREEZE_BBA               (*(volatile unsigned long *) 0x4204A10C)
#define T2CON_FREEZE_MSK               (0x1   << 3  )
#define T2CON_FREEZE                   (0x1   << 3  )
#define T2CON_FREEZE_DIS               (0x0   << 3  ) /* DIS                      */
#define T2CON_FREEZE_EN                (0x1   << 3  ) /* EN                       */

/* T2CON[PRE] - Prescaler */
#define T2CON_PRE_MSK                  (0x3   << 0  )
#define T2CON_PRE_DIV1                 (0x0   << 0  ) /* DIV1                     */
#define T2CON_PRE_DIV16                (0x1   << 0  ) /* DIV16                    */
#define T2CON_PRE_DIV256               (0x2   << 0  ) /* DIV256                   */
#define T2CON_PRE_DIV32768             (0x3   << 0  ) /* DIV32768                 */

/* Reset Value for T2INC*/
#define T2INC_RVAL                     0xC8 

/* T2INC[VALUE] - 12 bit value */
#define T2INC_VALUE_MSK                (0xFFF << 0  )

/* Reset Value for T2WUFB0*/
#define T2WUFB0_RVAL                   0x1FFF 

/* T2WUFB0[VALUE] - Current Value */
#define T2WUFB0_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for T2WUFB1*/
#define T2WUFB1_RVAL                   0x0 

/* T2WUFB1[VALUE] - Current Value */
#define T2WUFB1_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for T2WUFC0*/
#define T2WUFC0_RVAL                   0x2FFF 

/* T2WUFC0[VALUE] - Current Value */
#define T2WUFC0_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for T2WUFC1*/
#define T2WUFC1_RVAL                   0x0 

/* T2WUFC1[VALUE] - Current Value */
#define T2WUFC1_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for T2WUFD0*/
#define T2WUFD0_RVAL                   0x3FFF 

/* T2WUFD0[VALUE] - Current Value */
#define T2WUFD0_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for T2WUFD1*/
#define T2WUFD1_RVAL                   0x0 

/* T2WUFD1[VALUE] - Current Value */
#define T2WUFD1_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for T2IEN*/
#define T2IEN_RVAL                     0x0 

/* T2IEN[ROLL] - Enable interrupt on Rollover */
#define T2IEN_ROLL_BBA                 (*(volatile unsigned long *) 0x4204A510)
#define T2IEN_ROLL_MSK                 (0x1   << 4  )
#define T2IEN_ROLL                     (0x1   << 4  )
#define T2IEN_ROLL_DIS                 (0x0   << 4  ) /* DIS                      */
#define T2IEN_ROLL_EN                  (0x1   << 4  ) /* EN                       */

/* T2IEN[WUFD] - Enable interrupt on WUFD */
#define T2IEN_WUFD_BBA                 (*(volatile unsigned long *) 0x4204A50C)
#define T2IEN_WUFD_MSK                 (0x1   << 3  )
#define T2IEN_WUFD                     (0x1   << 3  )
#define T2IEN_WUFD_DIS                 (0x0   << 3  ) /* DIS                      */
#define T2IEN_WUFD_EN                  (0x1   << 3  ) /* EN                       */

/* T2IEN[WUFC] - Enable interrupt on WUFC */
#define T2IEN_WUFC_BBA                 (*(volatile unsigned long *) 0x4204A508)
#define T2IEN_WUFC_MSK                 (0x1   << 2  )
#define T2IEN_WUFC                     (0x1   << 2  )
#define T2IEN_WUFC_DIS                 (0x0   << 2  ) /* DIS                      */
#define T2IEN_WUFC_EN                  (0x1   << 2  ) /* EN                       */

/* T2IEN[WUFB] - Enable interrupt on WUFB */
#define T2IEN_WUFB_BBA                 (*(volatile unsigned long *) 0x4204A504)
#define T2IEN_WUFB_MSK                 (0x1   << 1  )
#define T2IEN_WUFB                     (0x1   << 1  )
#define T2IEN_WUFB_DIS                 (0x0   << 1  ) /* DIS                      */
#define T2IEN_WUFB_EN                  (0x1   << 1  ) /* EN                       */

/* T2IEN[WUFA] - Enable interrupt on WUFA */
#define T2IEN_WUFA_BBA                 (*(volatile unsigned long *) 0x4204A500)
#define T2IEN_WUFA_MSK                 (0x1   << 0  )
#define T2IEN_WUFA                     (0x1   << 0  )
#define T2IEN_WUFA_DIS                 (0x0   << 0  ) /* DIS                      */
#define T2IEN_WUFA_EN                  (0x1   << 0  ) /* EN                       */

/* Reset Value for T2STA*/
#define T2STA_RVAL                     0x0 

/* T2STA[CON] - Sync */
#define T2STA_CON_BBA                  (*(volatile unsigned long *) 0x4204A5A0)
#define T2STA_CON_MSK                  (0x1   << 8  )
#define T2STA_CON                      (0x1   << 8  )
#define T2STA_CON_CLR                  (0x0   << 8  ) /* CLR                      */
#define T2STA_CON_SET                  (0x1   << 8  ) /* SET                      */

/* T2STA[FREEZE] - Timer Value Freeze */
#define T2STA_FREEZE_BBA               (*(volatile unsigned long *) 0x4204A59C)
#define T2STA_FREEZE_MSK               (0x1   << 7  )
#define T2STA_FREEZE                   (0x1   << 7  )
#define T2STA_FREEZE_CLR               (0x0   << 7  ) /* CLR                      */
#define T2STA_FREEZE_SET               (0x1   << 7  ) /* SET                      */

/* T2STA[ROLL] - Rollover Interrupt */
#define T2STA_ROLL_BBA                 (*(volatile unsigned long *) 0x4204A590)
#define T2STA_ROLL_MSK                 (0x1   << 4  )
#define T2STA_ROLL                     (0x1   << 4  )
#define T2STA_ROLL_CLR                 (0x0   << 4  ) /* CLR                      */
#define T2STA_ROLL_SET                 (0x1   << 4  ) /* SET                      */

/* T2STA[WUFD] - WUFD Interrupt */
#define T2STA_WUFD_BBA                 (*(volatile unsigned long *) 0x4204A58C)
#define T2STA_WUFD_MSK                 (0x1   << 3  )
#define T2STA_WUFD                     (0x1   << 3  )
#define T2STA_WUFD_CLR                 (0x0   << 3  ) /* CLR                      */
#define T2STA_WUFD_SET                 (0x1   << 3  ) /* SET                      */

/* T2STA[WUFC] - WUFC Interrupt */
#define T2STA_WUFC_BBA                 (*(volatile unsigned long *) 0x4204A588)
#define T2STA_WUFC_MSK                 (0x1   << 2  )
#define T2STA_WUFC                     (0x1   << 2  )
#define T2STA_WUFC_CLR                 (0x0   << 2  ) /* CLR                      */
#define T2STA_WUFC_SET                 (0x1   << 2  ) /* SET                      */

/* T2STA[WUFB] - WUFB Interrupt */
#define T2STA_WUFB_BBA                 (*(volatile unsigned long *) 0x4204A584)
#define T2STA_WUFB_MSK                 (0x1   << 1  )
#define T2STA_WUFB                     (0x1   << 1  )
#define T2STA_WUFB_CLR                 (0x0   << 1  ) /* CLR                      */
#define T2STA_WUFB_SET                 (0x1   << 1  ) /* SET                      */

/* T2STA[WUFA] - WUFA Interrupt */
#define T2STA_WUFA_BBA                 (*(volatile unsigned long *) 0x4204A580)
#define T2STA_WUFA_MSK                 (0x1   << 0  )
#define T2STA_WUFA                     (0x1   << 0  )
#define T2STA_WUFA_CLR                 (0x0   << 0  ) /* CLR                      */
#define T2STA_WUFA_SET                 (0x1   << 0  ) /* SET                      */

/* Reset Value for T2CLRI*/
#define T2CLRI_RVAL                    0x0 

/* T2CLRI[ROLL] - Clear interrupt on Rollover */
#define T2CLRI_ROLL_BBA                (*(volatile unsigned long *) 0x4204A610)
#define T2CLRI_ROLL_MSK                (0x1   << 4  )
#define T2CLRI_ROLL                    (0x1   << 4  )
#define T2CLRI_ROLL_CLR                (0x1   << 4  ) /* CLR                      */

/* T2CLRI[WUFD] - Clear interrupt on WUFD */
#define T2CLRI_WUFD_BBA                (*(volatile unsigned long *) 0x4204A60C)
#define T2CLRI_WUFD_MSK                (0x1   << 3  )
#define T2CLRI_WUFD                    (0x1   << 3  )
#define T2CLRI_WUFD_CLR                (0x1   << 3  ) /* CLR                      */

/* T2CLRI[WUFC] - Clear interrupt on WUFC */
#define T2CLRI_WUFC_BBA                (*(volatile unsigned long *) 0x4204A608)
#define T2CLRI_WUFC_MSK                (0x1   << 2  )
#define T2CLRI_WUFC                    (0x1   << 2  )
#define T2CLRI_WUFC_CLR                (0x1   << 2  ) /* CLR                      */

/* T2CLRI[WUFB] - Clear interrupt on WUFB */
#define T2CLRI_WUFB_BBA                (*(volatile unsigned long *) 0x4204A604)
#define T2CLRI_WUFB_MSK                (0x1   << 1  )
#define T2CLRI_WUFB                    (0x1   << 1  )
#define T2CLRI_WUFB_CLR                (0x1   << 1  ) /* CLR                      */

/* T2CLRI[WUFA] - Clear interrupt on WUFA */
#define T2CLRI_WUFA_BBA                (*(volatile unsigned long *) 0x4204A600)
#define T2CLRI_WUFA_MSK                (0x1   << 0  )
#define T2CLRI_WUFA                    (0x1   << 0  )
#define T2CLRI_WUFA_CLR                (0x1   << 0  ) /* CLR                      */

/* Reset Value for T2WUFA0*/
#define T2WUFA0_RVAL                   0x1900 

/* T2WUFA0[VALUE] - Current Value */
#define T2WUFA0_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for T2WUFA1*/
#define T2WUFA1_RVAL                   0x0 

/* T2WUFA1[VALUE] - Current Value */
#define T2WUFA1_VALUE_MSK              (0xFFFF << 0  )
// ------------------------------------------------------------------------------------------------
// -----                                        CLKCTL                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Clock Control (pADI_CLKCTL)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_CLKCTL Structure                  */
  __IO uint16_t  CLKCON0;                   /*!< System clocking architecture control register */
  __I  uint16_t  RESERVED0;
  __IO uint16_t  CLKCON1;                   /*!< System Clocks Control Register 1      */
  __I  uint16_t  RESERVED1[19];
  __IO uint16_t  CLKDIS;                    /*!< System Clocks Control Register 1      */
  __I  uint16_t  RESERVED2[7];
  __IO uint16_t  CLKCON2;                   /*!< System Clocks Control Register 2      */
  __I  uint16_t  RESERVED3[489];
  __IO uint8_t   XOSCCON;                   /*!< Crystal Oscillator control            */
  __I  uint8_t   RESERVED4[51];
  __IO uint16_t  CLKSYSDIV;                 /*!< Sys Clock div2 Register               */
} ADI_CLKCTL_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          CLKCON0                                    (*(volatile unsigned short int *) 0x40002000)
#define          CLKCON1                                    (*(volatile unsigned short int *) 0x40002004)
#define          CLKDIS                                     (*(volatile unsigned short int *) 0x4000202C)
#define          CLKCON2                                    (*(volatile unsigned short int *) 0x4000203C)
#define          XOSCCON                                    (*(volatile unsigned char      *) 0x40002410)
#define          CLKSYSDIV                                  (*(volatile unsigned short int *) 0x40002444)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for CLKCON0*/
#define CLKCON0_RVAL                   0x0 

/* CLKCON0[CLKOUT] - GPIO output clock multiplexer select bits */
#define CLKCON0_CLKOUT_MSK             (0x7   << 5  )
#define CLKCON0_CLKOUT_UCLKCG          (0x0   << 5  ) /* UCLKCG                   */
#define CLKCON0_CLKOUT_UCLK            (0x1   << 5  ) /* UCLK                     */
#define CLKCON0_CLKOUT_PCLK            (0x2   << 5  ) /* PCLK                     */
#define CLKCON0_CLKOUT_HFOSC           (0x5   << 5  ) /* HFOSC                    */
#define CLKCON0_CLKOUT_LFOSC           (0x6   << 5  ) /* LFOSC                    */
#define CLKCON0_CLKOUT_LFXTAL          (0x7   << 5  ) /* LFXTAL                   */

/* CLKCON0[CLKMUX] - Digital subsystem clock source select bits. */
#define CLKCON0_CLKMUX_MSK             (0x3   << 3  )
#define CLKCON0_CLKMUX_HFOSC           (0x0   << 3  ) /* HFOSC                    */
#define CLKCON0_CLKMUX_LFXTAL          (0x1   << 3  ) /* LFXTAL                   */
#define CLKCON0_CLKMUX_LFOSC           (0x2   << 3  ) /* LFOSC                    */
#define CLKCON0_CLKMUX_EXTCLK          (0x3   << 3  ) /* EXTCLK                   */

/* CLKCON0[CD] - Clock divide bits */
#define CLKCON0_CD_MSK                 (0x7   << 0  )
#define CLKCON0_CD_DIV1                (0x0   << 0  ) /* DIV1                     */
#define CLKCON0_CD_DIV2                (0x1   << 0  ) /* DIV2                     */
#define CLKCON0_CD_DIV4                (0x2   << 0  ) /* DIV4                     */
#define CLKCON0_CD_DIV8                (0x3   << 0  ) /* DIV8                     */
#define CLKCON0_CD_DIV16               (0x4   << 0  ) /* DIV16                    */
#define CLKCON0_CD_DIV32               (0x5   << 0  ) /* DIV32                    */
#define CLKCON0_CD_DIV64               (0x6   << 0  ) /* DIV64                    */
#define CLKCON0_CD_DIV128              (0x7   << 0  ) /* DIV128                   */

/* Reset Value for CLKCON1*/
#define CLKCON1_RVAL                   0x0 

/* CLKCON1[PWMCD] - Clock divide bits for PWM system clock */
#define CLKCON1_PWMCD_MSK              (0x7   << 12 )
#define CLKCON1_PWMCD_DIV1             (0x0   << 12 ) /* DIV1                     */
#define CLKCON1_PWMCD_DIV2             (0x1   << 12 ) /* DIV2                     */
#define CLKCON1_PWMCD_DIV4             (0x2   << 12 ) /* DIV4                     */
#define CLKCON1_PWMCD_DIV8             (0x3   << 12 ) /* DIV8                     */
#define CLKCON1_PWMCD_DIV16            (0x4   << 12 ) /* DIV16                    */
#define CLKCON1_PWMCD_DIV32            (0x5   << 12 ) /* DIV32                    */
#define CLKCON1_PWMCD_DIV64            (0x6   << 12 ) /* DIV64                    */
#define CLKCON1_PWMCD_DIV128           (0x7   << 12 ) /* DIV128                   */

/* CLKCON1[UARTCD] - Clock divide bits for UART system clock */
#define CLKCON1_UARTCD_MSK             (0x7   << 9  )
#define CLKCON1_UARTCD_DIV1            (0x0   << 9  ) /* DIV1                     */
#define CLKCON1_UARTCD_DIV2            (0x1   << 9  ) /* DIV2                     */
#define CLKCON1_UARTCD_DIV4            (0x2   << 9  ) /* DIV4                     */
#define CLKCON1_UARTCD_DIV8            (0x3   << 9  ) /* DIV8                     */
#define CLKCON1_UARTCD_DIV16           (0x4   << 9  ) /* DIV16                    */
#define CLKCON1_UARTCD_DIV32           (0x5   << 9  ) /* DIV32                    */
#define CLKCON1_UARTCD_DIV64           (0x6   << 9  ) /* DIV64                    */
#define CLKCON1_UARTCD_DIV128          (0x7   << 9  ) /* DIV128                   */

/* CLKCON1[I2CCD] - Clock divide bits for I2C system clock */
#define CLKCON1_I2CCD_MSK              (0x7   << 6  )
#define CLKCON1_I2CCD_DIV1             (0x0   << 6  ) /* DIV1                     */
#define CLKCON1_I2CCD_DIV2             (0x1   << 6  ) /* DIV2                     */
#define CLKCON1_I2CCD_DIV4             (0x2   << 6  ) /* DIV4                     */
#define CLKCON1_I2CCD_DIV8             (0x3   << 6  ) /* DIV8                     */
#define CLKCON1_I2CCD_DIV16            (0x4   << 6  ) /* DIV16                    */
#define CLKCON1_I2CCD_DIV32            (0x5   << 6  ) /* DIV32                    */
#define CLKCON1_I2CCD_DIV64            (0x6   << 6  ) /* DIV64                    */
#define CLKCON1_I2CCD_DIV128           (0x7   << 6  ) /* DIV128                   */

/* CLKCON1[SPI1CD] - Clock divide bits for SPI1 system clock */
#define CLKCON1_SPI1CD_MSK             (0x7   << 3  )
#define CLKCON1_SPI1CD_DIV1            (0x0   << 3  ) /* DIV1                     */
#define CLKCON1_SPI1CD_DIV2            (0x1   << 3  ) /* DIV2                     */
#define CLKCON1_SPI1CD_DIV4            (0x2   << 3  ) /* DIV4                     */
#define CLKCON1_SPI1CD_DIV8            (0x3   << 3  ) /* DIV8                     */
#define CLKCON1_SPI1CD_DIV16           (0x4   << 3  ) /* DIV16                    */
#define CLKCON1_SPI1CD_DIV32           (0x5   << 3  ) /* DIV32                    */
#define CLKCON1_SPI1CD_DIV64           (0x6   << 3  ) /* DIV64                    */
#define CLKCON1_SPI1CD_DIV128          (0x7   << 3  ) /* DIV128                   */

/* CLKCON1[SPI0CD] - Clock divide bits for SPI0 system clock */
#define CLKCON1_SPI0CD_MSK             (0x7   << 0  )
#define CLKCON1_SPI0CD_DIV1            (0x0   << 0  ) /* DIV1                     */
#define CLKCON1_SPI0CD_DIV2            (0x1   << 0  ) /* DIV2                     */
#define CLKCON1_SPI0CD_DIV4            (0x2   << 0  ) /* DIV4                     */
#define CLKCON1_SPI0CD_DIV8            (0x3   << 0  ) /* DIV8                     */
#define CLKCON1_SPI0CD_DIV16           (0x4   << 0  ) /* DIV16                    */
#define CLKCON1_SPI0CD_DIV32           (0x5   << 0  ) /* DIV32                    */
#define CLKCON1_SPI0CD_DIV64           (0x6   << 0  ) /* DIV64                    */
#define CLKCON1_SPI0CD_DIV128          (0x7   << 0  ) /* DIV128                   */

/* Reset Value for CLKDIS*/
#define CLKDIS_RVAL                    0xFFFF 

/* CLKDIS[DISADCCLK] - Disable ADC system clock */
#define CLKDIS_DISADCCLK_BBA           (*(volatile unsigned long *) 0x420405A4)
#define CLKDIS_DISADCCLK_MSK           (0x1   << 9  )
#define CLKDIS_DISADCCLK               (0x1   << 9  )
#define CLKDIS_DISADCCLK_DIS           (0x0   << 9  ) /* DIS                      */
#define CLKDIS_DISADCCLK_EN            (0x1   << 9  ) /* EN                       */

/* CLKDIS[DISDMACLK] - Disable DMA system clock */
#define CLKDIS_DISDMACLK_BBA           (*(volatile unsigned long *) 0x420405A0)
#define CLKDIS_DISDMACLK_MSK           (0x1   << 8  )
#define CLKDIS_DISDMACLK               (0x1   << 8  )
#define CLKDIS_DISDMACLK_DIS           (0x0   << 8  ) /* DIS                      */
#define CLKDIS_DISDMACLK_EN            (0x1   << 8  ) /* EN                       */

/* CLKDIS[DISDACCLK] - Disable DAC system clock */
#define CLKDIS_DISDACCLK_BBA           (*(volatile unsigned long *) 0x4204059C)
#define CLKDIS_DISDACCLK_MSK           (0x1   << 7  )
#define CLKDIS_DISDACCLK               (0x1   << 7  )
#define CLKDIS_DISDACCLK_DIS           (0x0   << 7  ) /* DIS                      */
#define CLKDIS_DISDACCLK_EN            (0x1   << 7  ) /* EN                       */

/* CLKDIS[DIST1CLK] - Disable Timer 1 system clock */
#define CLKDIS_DIST1CLK_BBA            (*(volatile unsigned long *) 0x42040598)
#define CLKDIS_DIST1CLK_MSK            (0x1   << 6  )
#define CLKDIS_DIST1CLK                (0x1   << 6  )
#define CLKDIS_DIST1CLK_DIS            (0x0   << 6  ) /* DIS                      */
#define CLKDIS_DIST1CLK_EN             (0x1   << 6  ) /* EN                       */

/* CLKDIS[DIST0CLK] - Disable Timer 0 system clock */
#define CLKDIS_DIST0CLK_BBA            (*(volatile unsigned long *) 0x42040594)
#define CLKDIS_DIST0CLK_MSK            (0x1   << 5  )
#define CLKDIS_DIST0CLK                (0x1   << 5  )
#define CLKDIS_DIST0CLK_DIS            (0x0   << 5  ) /* DIS                      */
#define CLKDIS_DIST0CLK_EN             (0x1   << 5  ) /* EN                       */

/* CLKDIS[DISPWMCLK] - Disable PWM system clock */
#define CLKDIS_DISPWMCLK_BBA           (*(volatile unsigned long *) 0x42040590)
#define CLKDIS_DISPWMCLK_MSK           (0x1   << 4  )
#define CLKDIS_DISPWMCLK               (0x1   << 4  )
#define CLKDIS_DISPWMCLK_DIS           (0x0   << 4  ) /* DIS                      */
#define CLKDIS_DISPWMCLK_EN            (0x1   << 4  ) /* EN                       */

/* CLKDIS[DISUARTCLK] - Disable UART system clock */
#define CLKDIS_DISUARTCLK_BBA          (*(volatile unsigned long *) 0x4204058C)
#define CLKDIS_DISUARTCLK_MSK          (0x1   << 3  )
#define CLKDIS_DISUARTCLK              (0x1   << 3  )
#define CLKDIS_DISUARTCLK_DIS          (0x0   << 3  ) /* DIS                      */
#define CLKDIS_DISUARTCLK_EN           (0x1   << 3  ) /* EN                       */

/* CLKDIS[DISI2CCLK] - Disable I2C system clock */
#define CLKDIS_DISI2CCLK_BBA           (*(volatile unsigned long *) 0x42040588)
#define CLKDIS_DISI2CCLK_MSK           (0x1   << 2  )
#define CLKDIS_DISI2CCLK               (0x1   << 2  )
#define CLKDIS_DISI2CCLK_DIS           (0x0   << 2  ) /* DIS                      */
#define CLKDIS_DISI2CCLK_EN            (0x1   << 2  ) /* EN                       */

/* CLKDIS[DISSPI1CLK] - Disable SPI1 system clock */
#define CLKDIS_DISSPI1CLK_BBA          (*(volatile unsigned long *) 0x42040584)
#define CLKDIS_DISSPI1CLK_MSK          (0x1   << 1  )
#define CLKDIS_DISSPI1CLK              (0x1   << 1  )
#define CLKDIS_DISSPI1CLK_DIS          (0x0   << 1  ) /* DIS                      */
#define CLKDIS_DISSPI1CLK_EN           (0x1   << 1  ) /* EN                       */

/* CLKDIS[DISSPI0CLK] - Disable SPI0 system clock bits */
#define CLKDIS_DISSPI0CLK_BBA          (*(volatile unsigned long *) 0x42040580)
#define CLKDIS_DISSPI0CLK_MSK          (0x1   << 0  )
#define CLKDIS_DISSPI0CLK              (0x1   << 0  )
#define CLKDIS_DISSPI0CLK_DIS          (0x0   << 0  ) /* DIS                      */
#define CLKDIS_DISSPI0CLK_EN           (0x1   << 0  ) /* EN                       */

/* CLKCON2[DISUART1CLK] - Disable UART1 system clock */
#define CLKCON2_DISUART1CLK_BBA        (*(volatile unsigned long *) 0x42040780)
#define CLKCON2_DISUART1CLK_MSK        (0x1   << 0  )
#define CLKCON2_DISUART1CLK            (0x1   << 0  )
#define CLKCON2_DISUART1CLK_DIS        (0x0   << 0  ) /* DIS                      */
#define CLKCON2_DISUART1CLK_EN         (0x1   << 0  ) /* EN                       */

/* CLKCON2[DISUART2CLK] - Disable UART2 system clock */
#define CLKCON2_DISUART2CLK_BBA        (*(volatile unsigned long *) 0x42040784)
#define CLKCON2_DISUART2CLK_MSK        (0x1   << 1  )
#define CLKCON2_DISUART2CLK            (0x1   << 1  )
#define CLKCON2_DISUART2CLK_DIS        (0x0   << 1  ) /* DIS                      */
#define CLKCON2_DISUART2CLK_EN         (0x1   << 1  ) /* EN                       */

/* CLKCON2[UART1CD] - Clock divide bits for UART1 system clock */
#define CLKCON2_UART1CD_MSK            (0x7   << 8  )
#define CLKCON2_UART1CD_DIV1           (0x0   << 8  ) /* DIV1                     */
#define CLKCON2_UART1CD_DIV2           (0x1   << 8  ) /* DIV2                     */
#define CLKCON2_UART1CD_DIV4           (0x2   << 8  ) /* DIV4                     */
#define CLKCON2_UART1CD_DIV8           (0x3   << 8  ) /* DIV8                     */
#define CLKCON2_UART1CD_DIV16          (0x4   << 8  ) /* DIV16                    */
#define CLKCON2_UART1CD_DIV32          (0x5   << 8  ) /* DIV32                    */
#define CLKCON2_UART1CD_DIV64          (0x6   << 8  ) /* DIV64                    */
#define CLKCON2_UART1CD_DIV128         (0x7   << 8  ) /* DIV128                   */

/* CLKCON2[UART2CD] - Clock divide bits for UART2 system clock */
#define CLKCON2_UART2CD_MSK            (0x7   << 11 )
#define CLKCON2_UART2CD_DIV1           (0x0   << 11 ) /* DIV1                     */
#define CLKCON2_UART2CD_DIV2           (0x1   << 11 ) /* DIV2                     */
#define CLKCON2_UART2CD_DIV4           (0x2   << 11 ) /* DIV4                     */
#define CLKCON2_UART2CD_DIV8           (0x3   << 11 ) /* DIV8                     */
#define CLKCON2_UART2CD_DIV16          (0x4   << 11 ) /* DIV16                    */
#define CLKCON2_UART2CD_DIV32          (0x5   << 11 ) /* DIV32                    */
#define CLKCON2_UART2CD_DIV64          (0x6   << 11 ) /* DIV64                    */
#define CLKCON2_UART2CD_DIV128         (0x7   << 11 ) /* DIV128                   */

/* CLKCON2[DACCD] - Clock divide bit for DAC system clock */
#define CLKCON2_DACCD_BBA              (*(volatile unsigned long *) 0x420407B8)
#define CLKCON2_DACCD_MSK              (0x1   << 14 )
#define CLKCON2_DACCD                  (0x1   << 14 )
#define CLKCON2_DACCD_DIV8             (0x0   << 14 ) /* DIV8                     */
#define CLKCON2_DACCD_DIV16            (0x1   << 14 ) /* DIV16                    */

/* Reset Value for XOSCCON*/
#define XOSCCON_RVAL                   0x0 

/* XOSCCON[DIV2] - Divide by two enable */
#define XOSCCON_DIV2_BBA               (*(volatile unsigned long *) 0x42048208)
#define XOSCCON_DIV2_MSK               (0x1   << 2  )
#define XOSCCON_DIV2                   (0x1   << 2  )
#define XOSCCON_DIV2_DIS               (0x0   << 2  ) /* DIS                      */
#define XOSCCON_DIV2_EN                (0x1   << 2  ) /* EN                       */

/* XOSCCON[ENABLE] - Crystal oscillator circuit enable (Enable the oscillator circuitry.) */
#define XOSCCON_ENABLE_BBA             (*(volatile unsigned long *) 0x42048200)
#define XOSCCON_ENABLE_MSK             (0x1   << 0  )
#define XOSCCON_ENABLE                 (0x1   << 0  )
#define XOSCCON_ENABLE_DIS             (0x0   << 0  ) /* DIS                      */
#define XOSCCON_ENABLE_EN              (0x1   << 0  ) /* EN                       */

/* Reset Value for CLKSYSDIV*/
//#define CLKSYSDIV_RVAL                 0x0 

/* CLKSYSDIV[UCLKCD] - bits */
#define CLKSYSDIV_UCLKCD_MSK           (0x3   << 0  )
#define CLKSYSDIV_UCLKCD_DIV1          (0x0   << 0  )
#define CLKSYSDIV_UCLKCD_DIV2          (0x1   << 0  )
#define CLKSYSDIV_UCLKCD_DIV4          (0x2   << 0  )
#define CLKSYSDIV_UCLKCD_DIV8          (0x3   << 0  )
// ------------------------------------------------------------------------------------------------
// -----                                        FEE                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Flash Controller (pADI_FEE)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_FEE Structure                     */
  __IO uint16_t  FEESTA;                    /*!< Status Register                       */
  __I  uint16_t  RESERVED0;
  __IO uint16_t  FEECON0;                   /*!< Command Control Register              */
  __I  uint16_t  RESERVED1;
  __IO uint16_t  FEECMD;                    /*!< Command register                      */
  __I  uint16_t  RESERVED2[3];
  __IO uint16_t  FEEADR0L;                  /*!< Low Page  (Lower 16 bits)             */
  __I  uint16_t  RESERVED3;
  __IO uint16_t  FEEADR0H;                  /*!< Low Page  (Upper 16 bits)             */
  __I  uint16_t  RESERVED4;
  __IO uint16_t  FEEADR1L;                  /*!< Hi Page  (Lower 16 bits)              */
  __I  uint16_t  RESERVED5;
  __IO uint16_t  FEEADR1H;                  /*!< Hi Page  (Upper 16 bits)              */
  __I  uint16_t  RESERVED6;
  __IO uint16_t  FEEKEY;                    /*!< Key                                   */
  __I  uint16_t  RESERVED7[3];
  __IO uint16_t  FEEPROL;                   /*!< Write Protection (Lower 16 bits)      */
  __I  uint16_t  RESERVED8;
  __IO uint16_t  FEEPROH;                   /*!< Write Protection (Upper 16 bits)      */
  __I  uint16_t  RESERVED9;
  __IO uint16_t  FEESIGL;                   /*!< Signature (Lower 16 bits)             */
  __I  uint16_t  RESERVED10;
  __IO uint16_t  FEESIGH;                   /*!< Signature (Upper 16 bits)             */
  __I  uint16_t  RESERVED11;
  __IO uint16_t  FEECON1;                   /*!< User Setup register                   */
  __I  uint16_t  RESERVED12[7];
  __IO uint16_t  FEEADRAL;                  /*!< Abort address (Lower 16 bits)         */
  __I  uint16_t  RESERVED13;
  __IO uint16_t  FEEADRAH;                  /*!< Abort address (Upper 16 bits)         */
  __I  uint16_t  RESERVED14[21];
  __IO uint16_t  FEEAEN0;                   /*!< Lower 16 bits of the sys irq abort enable register. */
  __I  uint16_t  RESERVED15;
  __IO uint16_t  FEEAEN1;                   /*!< Upper 16 bits of the sys irq abort enable register. */
  __I  uint16_t  RESERVED16;
  __IO uint16_t  FEEAEN2;                   /*!< Upper 32..47 bits of the sys irq abort enable register. */
} ADI_FEE_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          FEESTA                                     (*(volatile unsigned short int *) 0x40002800)
#define          FEECON0                                    (*(volatile unsigned short int *) 0x40002804)
#define          FEECMD                                     (*(volatile unsigned short int *) 0x40002808)
#define          FEEADR0L                                   (*(volatile unsigned short int *) 0x40002810)
#define          FEEADR0H                                   (*(volatile unsigned short int *) 0x40002814)
#define          FEEADR1L                                   (*(volatile unsigned short int *) 0x40002818)
#define          FEEADR1H                                   (*(volatile unsigned short int *) 0x4000281C)
#define          FEEKEY                                     (*(volatile unsigned short int *) 0x40002820)
#define          FEEPROL                                    (*(volatile unsigned short int *) 0x40002828)
#define          FEEPROH                                    (*(volatile unsigned short int *) 0x4000282C)
#define          FEESIGL                                    (*(volatile unsigned short int *) 0x40002830)
#define          FEESIGH                                    (*(volatile unsigned short int *) 0x40002834)
#define          FEECON1                                    (*(volatile unsigned short int *) 0x40002838)
#define          FEEADRAL                                   (*(volatile unsigned short int *) 0x40002848)
#define          FEEADRAH                                   (*(volatile unsigned short int *) 0x4000284C)
#define          FEEAEN0                                    (*(volatile unsigned short int *) 0x40002878)
#define          FEEAEN1                                    (*(volatile unsigned short int *) 0x4000287C)
#define          FEEAEN2                                    (*(volatile unsigned short int *) 0x40002880)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for FEESTA*/
#define FEESTA_RVAL                    0x0 

/* FEESTA[SIGNERR] - Info space signature check on reset error */
#define FEESTA_SIGNERR_BBA             (*(volatile unsigned long *) 0x42050018)
#define FEESTA_SIGNERR_MSK             (0x1   << 6  )
#define FEESTA_SIGNERR                 (0x1   << 6  )
#define FEESTA_SIGNERR_CLR             (0x0   << 6  ) /* CLR                      */
#define FEESTA_SIGNERR_SET             (0x1   << 6  ) /* SET                      */

/* FEESTA[CMDRES] - Command result */
#define FEESTA_CMDRES_MSK              (0x3   << 4  )
#define FEESTA_CMDRES_SUCCESS          (0x0   << 4  ) /* SUCCESS                  */
#define FEESTA_CMDRES_PROTECTED        (0x1   << 4  ) /* PROTECTED                */
#define FEESTA_CMDRES_VERIFYERR        (0x2   << 4  ) /* VERIFYERR                */
#define FEESTA_CMDRES_ABORT            (0x3   << 4  ) /* ABORT                    */

/* FEESTA[WRDONE] - Write Complete */
#define FEESTA_WRDONE_BBA              (*(volatile unsigned long *) 0x4205000C)
#define FEESTA_WRDONE_MSK              (0x1   << 3  )
#define FEESTA_WRDONE                  (0x1   << 3  )
#define FEESTA_WRDONE_CLR              (0x0   << 3  ) /* CLR                      */
#define FEESTA_WRDONE_SET              (0x1   << 3  ) /* SET                      */

/* FEESTA[CMDDONE] - Command complete */
#define FEESTA_CMDDONE_BBA             (*(volatile unsigned long *) 0x42050008)
#define FEESTA_CMDDONE_MSK             (0x1   << 2  )
#define FEESTA_CMDDONE                 (0x1   << 2  )
#define FEESTA_CMDDONE_CLR             (0x0   << 2  ) /* CLR                      */
#define FEESTA_CMDDONE_SET             (0x1   << 2  ) /* SET                      */

/* FEESTA[WRBUSY] - Write busy */
#define FEESTA_WRBUSY_BBA              (*(volatile unsigned long *) 0x42050004)
#define FEESTA_WRBUSY_MSK              (0x1   << 1  )
#define FEESTA_WRBUSY                  (0x1   << 1  )
#define FEESTA_WRBUSY_CLR              (0x0   << 1  ) /* CLR                      */
#define FEESTA_WRBUSY_SET              (0x1   << 1  ) /* SET                      */

/* FEESTA[CMDBUSY] - Command busy */
#define FEESTA_CMDBUSY_BBA             (*(volatile unsigned long *) 0x42050000)
#define FEESTA_CMDBUSY_MSK             (0x1   << 0  )
#define FEESTA_CMDBUSY                 (0x1   << 0  )
#define FEESTA_CMDBUSY_CLR             (0x0   << 0  ) /* CLR                      */
#define FEESTA_CMDBUSY_SET             (0x1   << 0  ) /* SET                      */

/* Reset Value for FEECON0*/
#define FEECON0_RVAL                   0x0 

/* FEECON0[WREN] - Write enable. */
#define FEECON0_WREN_BBA               (*(volatile unsigned long *) 0x42050088)
#define FEECON0_WREN_MSK               (0x1   << 2  )
#define FEECON0_WREN                   (0x1   << 2  )
#define FEECON0_WREN_DIS               (0x0   << 2  ) /* DIS                      */
#define FEECON0_WREN_EN                (0x1   << 2  ) /* EN                       */

/* FEECON0[IENERR] - Error interrupt enable */
#define FEECON0_IENERR_BBA             (*(volatile unsigned long *) 0x42050084)
#define FEECON0_IENERR_MSK             (0x1   << 1  )
#define FEECON0_IENERR                 (0x1   << 1  )
#define FEECON0_IENERR_DIS             (0x0   << 1  ) /* DIS                      */
#define FEECON0_IENERR_EN              (0x1   << 1  ) /* EN                       */

/* FEECON0[IENCMD] - Command complete interrupt enable */
#define FEECON0_IENCMD_BBA             (*(volatile unsigned long *) 0x42050080)
#define FEECON0_IENCMD_MSK             (0x1   << 0  )
#define FEECON0_IENCMD                 (0x1   << 0  )
#define FEECON0_IENCMD_DIS             (0x0   << 0  ) /* DIS                      */
#define FEECON0_IENCMD_EN              (0x1   << 0  ) /* EN                       */

/* Reset Value for FEECMD*/
#define FEECMD_RVAL                    0x0 

/* FEECMD[CMD] - Command */
#define FEECMD_CMD_MSK                 (0xF   << 0  )
#define FEECMD_CMD_IDLE                (0x0   << 0  ) /* IDLE - No command executed */
#define FEECMD_CMD_ERASEPAGE           (0x1   << 0  ) /* ERASEPAGE - Erase Page   */
#define FEECMD_CMD_SIGN                (0x2   << 0  ) /* SIGN - Sign Range        */
#define FEECMD_CMD_MASSERASE           (0x3   << 0  ) /* MASSERASE - Mass Erase User Space */
#define FEECMD_CMD_ABORT               (0x4   << 0  ) /* ABORT - Abort a running command */

/* Reset Value for FEEADR0L*/
#define FEEADR0L_RVAL                  0x0 

/* FEEADR0L[VALUE] - Value */
#define FEEADR0L_VALUE_MSK             (0xFFFF << 0  )

/* Reset Value for FEEADR0H*/
#define FEEADR0H_RVAL                  0x0 

/* FEEADR0H[VALUE] - Value */
#define FEEADR0H_VALUE_MSK             (0x3   << 0  )

/* Reset Value for FEEADR1L*/
#define FEEADR1L_RVAL                  0x0 

/* FEEADR1L[VALUE] - Value */
#define FEEADR1L_VALUE_MSK             (0xFFFF << 0  )

/* Reset Value for FEEADR1H*/
#define FEEADR1H_RVAL                  0x0 

/* FEEADR1H[VALUE] - Value */
#define FEEADR1H_VALUE_MSK             (0x3   << 0  )

/* Reset Value for FEEKEY*/
#define FEEKEY_RVAL                    0x0 

/* FEEKEY[VALUE] - Value */
#define FEEKEY_VALUE_MSK               (0xFFFF << 0  )
#define FEEKEY_VALUE_USERKEY1          (0xF456 << 0  ) /* USERKEY1                 */
#define FEEKEY_VALUE_USERKEY2          (0xF123 << 0  ) /* USERKEY2                 */

/* Reset Value for FEEPROL*/
#define FEEPROL_RVAL                   0xFFFF 

/* FEEPROL[VALUE] - Value */
#define FEEPROL_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for FEEPROH*/
#define FEEPROH_RVAL                   0xFFFF 

/* FEEPROH[VALUE] - Value */
#define FEEPROH_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for FEESIGL*/
#define FEESIGL_RVAL                   0xFFFF 

/* FEESIGL[VALUE] - Value */
#define FEESIGL_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for FEESIGH*/
#define FEESIGH_RVAL                   0xFFFF 

/* FEESIGH[VALUE] - Value */
#define FEESIGH_VALUE_MSK              (0xFF  << 0  )

/* Reset Value for FEECON1*/
#define FEECON1_RVAL                   0x1 

/* FEECON1[DBG] - Serial Wire debug enable , */
#define FEECON1_DBG_BBA                (*(volatile unsigned long *) 0x42050700)
#define FEECON1_DBG_MSK                (0x1   << 0  )
#define FEECON1_DBG                    (0x1   << 0  )
#define FEECON1_DBG_DIS                (0x0   << 0  ) /* DIS                      */
#define FEECON1_DBG_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for FEEADRAL*/
#define FEEADRAL_RVAL                  0x800 

/* FEEADRAL[VALUE] - Value */
#define FEEADRAL_VALUE_MSK             (0xFFFF << 0  )

/* Reset Value for FEEADRAH*/
#define FEEADRAH_RVAL                  0x2 

/* FEEADRAH[VALUE] - Value */
#define FEEADRAH_VALUE_MSK             (0xFFFF << 0  )

/* Reset Value for FEEAEN0*/
#define FEEAEN0_RVAL                   0x0 

/* FEEAEN0[SINC2] -  */
#define FEEAEN0_SINC2_BBA              (*(volatile unsigned long *) 0x42050F3C)
#define FEEAEN0_SINC2_MSK              (0x1   << 15 )
#define FEEAEN0_SINC2                  (0x1   << 15 )
#define FEEAEN0_SINC2_DIS              (0x0   << 15 ) /* DIS                      */
#define FEEAEN0_SINC2_EN               (0x1   << 15 ) /* EN                       */

/* FEEAEN0[ADC1] -  */
#define FEEAEN0_ADC1_BBA               (*(volatile unsigned long *) 0x42050F38)
#define FEEAEN0_ADC1_MSK               (0x1   << 14 )
#define FEEAEN0_ADC1                   (0x1   << 14 )
#define FEEAEN0_ADC1_DIS               (0x0   << 14 ) /* DIS                      */
#define FEEAEN0_ADC1_EN                (0x1   << 14 ) /* EN                       */

/* FEEAEN0[T1] -  */
#define FEEAEN0_T1_BBA                 (*(volatile unsigned long *) 0x42050F30)
#define FEEAEN0_T1_MSK                 (0x1   << 12 )
#define FEEAEN0_T1                     (0x1   << 12 )
#define FEEAEN0_T1_DIS                 (0x0   << 12 ) /* DIS                      */
#define FEEAEN0_T1_EN                  (0x1   << 12 ) /* EN                       */

/* FEEAEN0[T0] -  */
#define FEEAEN0_T0_BBA                 (*(volatile unsigned long *) 0x42050F2C)
#define FEEAEN0_T0_MSK                 (0x1   << 11 )
#define FEEAEN0_T0                     (0x1   << 11 )
#define FEEAEN0_T0_DIS                 (0x0   << 11 ) /* DIS                      */
#define FEEAEN0_T0_EN                  (0x1   << 11 ) /* EN                       */

/* FEEAEN0[T3] -  */
#define FEEAEN0_T3_BBA                 (*(volatile unsigned long *) 0x42050F24)
#define FEEAEN0_T3_MSK                 (0x1   << 9  )
#define FEEAEN0_T3                     (0x1   << 9  )
#define FEEAEN0_T3_DIS                 (0x0   << 9  ) /* DIS                      */
#define FEEAEN0_T3_EN                  (0x1   << 9  ) /* EN                       */

/* FEEAEN0[EXTINT7] -  */
#define FEEAEN0_EXTINT7_BBA            (*(volatile unsigned long *) 0x42050F20)
#define FEEAEN0_EXTINT7_MSK            (0x1   << 8  )
#define FEEAEN0_EXTINT7                (0x1   << 8  )
#define FEEAEN0_EXTINT7_DIS            (0x0   << 8  ) /* DIS                      */
#define FEEAEN0_EXTINT7_EN             (0x1   << 8  ) /* EN                       */

/* FEEAEN0[EXTINT6] -  */
#define FEEAEN0_EXTINT6_BBA            (*(volatile unsigned long *) 0x42050F1C)
#define FEEAEN0_EXTINT6_MSK            (0x1   << 7  )
#define FEEAEN0_EXTINT6                (0x1   << 7  )
#define FEEAEN0_EXTINT6_DIS            (0x0   << 7  ) /* DIS                      */
#define FEEAEN0_EXTINT6_EN             (0x1   << 7  ) /* EN                       */

/* FEEAEN0[EXTINT5] -  */
#define FEEAEN0_EXTINT5_BBA            (*(volatile unsigned long *) 0x42050F18)
#define FEEAEN0_EXTINT5_MSK            (0x1   << 6  )
#define FEEAEN0_EXTINT5                (0x1   << 6  )
#define FEEAEN0_EXTINT5_DIS            (0x0   << 6  ) /* DIS                      */
#define FEEAEN0_EXTINT5_EN             (0x1   << 6  ) /* EN                       */

/* FEEAEN0[EXTINT4] -  */
#define FEEAEN0_EXTINT4_BBA            (*(volatile unsigned long *) 0x42050F14)
#define FEEAEN0_EXTINT4_MSK            (0x1   << 5  )
#define FEEAEN0_EXTINT4                (0x1   << 5  )
#define FEEAEN0_EXTINT4_DIS            (0x0   << 5  ) /* DIS                      */
#define FEEAEN0_EXTINT4_EN             (0x1   << 5  ) /* EN                       */

/* FEEAEN0[EXTINT3] -  */
#define FEEAEN0_EXTINT3_BBA            (*(volatile unsigned long *) 0x42050F10)
#define FEEAEN0_EXTINT3_MSK            (0x1   << 4  )
#define FEEAEN0_EXTINT3                (0x1   << 4  )
#define FEEAEN0_EXTINT3_DIS            (0x0   << 4  ) /* DIS                      */
#define FEEAEN0_EXTINT3_EN             (0x1   << 4  ) /* EN                       */

/* FEEAEN0[EXTINT2] -  */
#define FEEAEN0_EXTINT2_BBA            (*(volatile unsigned long *) 0x42050F0C)
#define FEEAEN0_EXTINT2_MSK            (0x1   << 3  )
#define FEEAEN0_EXTINT2                (0x1   << 3  )
#define FEEAEN0_EXTINT2_DIS            (0x0   << 3  ) /* DIS                      */
#define FEEAEN0_EXTINT2_EN             (0x1   << 3  ) /* EN                       */

/* FEEAEN0[EXTINT1] -  */
#define FEEAEN0_EXTINT1_BBA            (*(volatile unsigned long *) 0x42050F08)
#define FEEAEN0_EXTINT1_MSK            (0x1   << 2  )
#define FEEAEN0_EXTINT1                (0x1   << 2  )
#define FEEAEN0_EXTINT1_DIS            (0x0   << 2  ) /* DIS                      */
#define FEEAEN0_EXTINT1_EN             (0x1   << 2  ) /* EN                       */

/* FEEAEN0[EXTINT0] -  */
#define FEEAEN0_EXTINT0_BBA            (*(volatile unsigned long *) 0x42050F04)
#define FEEAEN0_EXTINT0_MSK            (0x1   << 1  )
#define FEEAEN0_EXTINT0                (0x1   << 1  )
#define FEEAEN0_EXTINT0_DIS            (0x0   << 1  ) /* DIS                      */
#define FEEAEN0_EXTINT0_EN             (0x1   << 1  ) /* EN                       */

/* FEEAEN0[T2] -  */
#define FEEAEN0_T2_BBA                 (*(volatile unsigned long *) 0x42050F00)
#define FEEAEN0_T2_MSK                 (0x1   << 0  )
#define FEEAEN0_T2                     (0x1   << 0  )
#define FEEAEN0_T2_DIS                 (0x0   << 0  ) /* DIS                      */
#define FEEAEN0_T2_EN                  (0x1   << 0  ) /* EN                       */

/* Reset Value for FEEAEN1*/
#define FEEAEN1_RVAL                   0x0 

/* FEEAEN1[DMADAC] -  */
#define FEEAEN1_DMADAC_BBA             (*(volatile unsigned long *) 0x42050FBC)
#define FEEAEN1_DMADAC_MSK             (0x1   << 15 )
#define FEEAEN1_DMADAC                 (0x1   << 15 )
#define FEEAEN1_DMADAC_DIS             (0x0   << 15 ) /* DIS                      */
#define FEEAEN1_DMADAC_EN              (0x1   << 15 ) /* EN                       */

/* FEEAEN1[DMAI2CMRX] -  */
#define FEEAEN1_DMAI2CMRX_BBA          (*(volatile unsigned long *) 0x42050FB8)
#define FEEAEN1_DMAI2CMRX_MSK          (0x1   << 14 )
#define FEEAEN1_DMAI2CMRX              (0x1   << 14 )
#define FEEAEN1_DMAI2CMRX_DIS          (0x0   << 14 ) /* DIS                      */
#define FEEAEN1_DMAI2CMRX_EN           (0x1   << 14 ) /* EN                       */

/* FEEAEN1[DMAI2CMTX] -  */
#define FEEAEN1_DMAI2CMTX_BBA          (*(volatile unsigned long *) 0x42050FB4)
#define FEEAEN1_DMAI2CMTX_MSK          (0x1   << 13 )
#define FEEAEN1_DMAI2CMTX              (0x1   << 13 )
#define FEEAEN1_DMAI2CMTX_DIS          (0x0   << 13 ) /* DIS                      */
#define FEEAEN1_DMAI2CMTX_EN           (0x1   << 13 ) /* EN                       */

/* FEEAEN1[DMAI2CSRX] -  */
#define FEEAEN1_DMAI2CSRX_BBA          (*(volatile unsigned long *) 0x42050FB0)
#define FEEAEN1_DMAI2CSRX_MSK          (0x1   << 12 )
#define FEEAEN1_DMAI2CSRX              (0x1   << 12 )
#define FEEAEN1_DMAI2CSRX_DIS          (0x0   << 12 ) /* DIS                      */
#define FEEAEN1_DMAI2CSRX_EN           (0x1   << 12 ) /* EN                       */

/* FEEAEN1[DMAI2CSTX] -  */
#define FEEAEN1_DMAI2CSTX_BBA          (*(volatile unsigned long *) 0x42050FAC)
#define FEEAEN1_DMAI2CSTX_MSK          (0x1   << 11 )
#define FEEAEN1_DMAI2CSTX              (0x1   << 11 )
#define FEEAEN1_DMAI2CSTX_DIS          (0x0   << 11 ) /* DIS                      */
#define FEEAEN1_DMAI2CSTX_EN           (0x1   << 11 ) /* EN                       */

/* FEEAEN1[DMAUARTRX] -  */
#define FEEAEN1_DMAUARTRX_BBA          (*(volatile unsigned long *) 0x42050FA8)
#define FEEAEN1_DMAUARTRX_MSK          (0x1   << 10 )
#define FEEAEN1_DMAUARTRX              (0x1   << 10 )
#define FEEAEN1_DMAUARTRX_DIS          (0x0   << 10 ) /* DIS                      */
#define FEEAEN1_DMAUARTRX_EN           (0x1   << 10 ) /* EN                       */

/* FEEAEN1[DMAUARTTX] -  */
#define FEEAEN1_DMAUARTTX_BBA          (*(volatile unsigned long *) 0x42050FA4)
#define FEEAEN1_DMAUARTTX_MSK          (0x1   << 9  )
#define FEEAEN1_DMAUARTTX              (0x1   << 9  )
#define FEEAEN1_DMAUARTTX_DIS          (0x0   << 9  ) /* DIS                      */
#define FEEAEN1_DMAUARTTX_EN           (0x1   << 9  ) /* EN                       */

/* FEEAEN1[DMASPI1RX] -  */
#define FEEAEN1_DMASPI1RX_BBA          (*(volatile unsigned long *) 0x42050FA0)
#define FEEAEN1_DMASPI1RX_MSK          (0x1   << 8  )
#define FEEAEN1_DMASPI1RX              (0x1   << 8  )
#define FEEAEN1_DMASPI1RX_DIS          (0x0   << 8  ) /* DIS                      */
#define FEEAEN1_DMASPI1RX_EN           (0x1   << 8  ) /* EN                       */

/* FEEAEN1[DMASPI1TX] -  */
#define FEEAEN1_DMASPI1TX_BBA          (*(volatile unsigned long *) 0x42050F9C)
#define FEEAEN1_DMASPI1TX_MSK          (0x1   << 7  )
#define FEEAEN1_DMASPI1TX              (0x1   << 7  )
#define FEEAEN1_DMASPI1TX_DIS          (0x0   << 7  ) /* DIS                      */
#define FEEAEN1_DMASPI1TX_EN           (0x1   << 7  ) /* EN                       */

/* FEEAEN1[DMAERROR] -  */
#define FEEAEN1_DMAERROR_BBA           (*(volatile unsigned long *) 0x42050F98)
#define FEEAEN1_DMAERROR_MSK           (0x1   << 6  )
#define FEEAEN1_DMAERROR               (0x1   << 6  )
#define FEEAEN1_DMAERROR_DIS           (0x0   << 6  ) /* DIS                      */
#define FEEAEN1_DMAERROR_EN            (0x1   << 6  ) /* EN                       */

/* FEEAEN1[I2CM] -  */
#define FEEAEN1_I2CM_BBA               (*(volatile unsigned long *) 0x42050F94)
#define FEEAEN1_I2CM_MSK               (0x1   << 5  )
#define FEEAEN1_I2CM                   (0x1   << 5  )
#define FEEAEN1_I2CM_DIS               (0x0   << 5  ) /* DIS                      */
#define FEEAEN1_I2CM_EN                (0x1   << 5  ) /* EN                       */

/* FEEAEN1[I2CS] -  */
#define FEEAEN1_I2CS_BBA               (*(volatile unsigned long *) 0x42050F90)
#define FEEAEN1_I2CS_MSK               (0x1   << 4  )
#define FEEAEN1_I2CS                   (0x1   << 4  )
#define FEEAEN1_I2CS_DIS               (0x0   << 4  ) /* DIS                      */
#define FEEAEN1_I2CS_EN                (0x1   << 4  ) /* EN                       */

/* FEEAEN1[SPI1] -  */
#define FEEAEN1_SPI1_BBA               (*(volatile unsigned long *) 0x42050F8C)
#define FEEAEN1_SPI1_MSK               (0x1   << 3  )
#define FEEAEN1_SPI1                   (0x1   << 3  )
#define FEEAEN1_SPI1_DIS               (0x0   << 3  ) /* DIS                      */
#define FEEAEN1_SPI1_EN                (0x1   << 3  ) /* EN                       */

/* FEEAEN1[SPI0] -  */
#define FEEAEN1_SPI0_BBA               (*(volatile unsigned long *) 0x42050F88)
#define FEEAEN1_SPI0_MSK               (0x1   << 2  )
#define FEEAEN1_SPI0                   (0x1   << 2  )
#define FEEAEN1_SPI0_DIS               (0x0   << 2  ) /* DIS                      */
#define FEEAEN1_SPI0_EN                (0x1   << 2  ) /* EN                       */

/* FEEAEN1[UART] -  */
#define FEEAEN1_UART_BBA               (*(volatile unsigned long *) 0x42050F84)
#define FEEAEN1_UART_MSK               (0x1   << 1  )
#define FEEAEN1_UART                   (0x1   << 1  )
#define FEEAEN1_UART_DIS               (0x0   << 1  ) /* DIS                      */
#define FEEAEN1_UART_EN                (0x1   << 1  ) /* EN                       */

/* FEEAEN1[FEE] -  */
#define FEEAEN1_FEE_BBA                (*(volatile unsigned long *) 0x42050F80)
#define FEEAEN1_FEE_MSK                (0x1   << 0  )
#define FEEAEN1_FEE                    (0x1   << 0  )
#define FEEAEN1_FEE_DIS                (0x0   << 0  ) /* DIS                      */
#define FEEAEN1_FEE_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for FEEAEN2*/
#define FEEAEN2_RVAL                   0x0 

/* FEEAEN2[PWM2] -  */
#define FEEAEN2_PWM2_BBA               (*(volatile unsigned long *) 0x42051018)
#define FEEAEN2_PWM2_MSK               (0x1   << 6  )
#define FEEAEN2_PWM2                   (0x1   << 6  )
#define FEEAEN2_PWM2_DIS               (0x0   << 6  ) /* DIS                      */
#define FEEAEN2_PWM2_EN                (0x1   << 6  ) /* EN                       */

/* FEEAEN2[PWM1] -  */
#define FEEAEN2_PWM1_BBA               (*(volatile unsigned long *) 0x42051014)
#define FEEAEN2_PWM1_MSK               (0x1   << 5  )
#define FEEAEN2_PWM1                   (0x1   << 5  )
#define FEEAEN2_PWM1_DIS               (0x0   << 5  ) /* DIS                      */
#define FEEAEN2_PWM1_EN                (0x1   << 5  ) /* EN                       */

/* FEEAEN2[PWM0] -  */
#define FEEAEN2_PWM0_BBA               (*(volatile unsigned long *) 0x42051010)
#define FEEAEN2_PWM0_MSK               (0x1   << 4  )
#define FEEAEN2_PWM0                   (0x1   << 4  )
#define FEEAEN2_PWM0_DIS               (0x0   << 4  ) /* DIS                      */
#define FEEAEN2_PWM0_EN                (0x1   << 4  ) /* EN                       */

/* FEEAEN2[PWMTRIP] -  */
#define FEEAEN2_PWMTRIP_BBA            (*(volatile unsigned long *) 0x4205100C)
#define FEEAEN2_PWMTRIP_MSK            (0x1   << 3  )
#define FEEAEN2_PWMTRIP                (0x1   << 3  )
#define FEEAEN2_PWMTRIP_DIS            (0x0   << 3  ) /* DIS                      */
#define FEEAEN2_PWMTRIP_EN             (0x1   << 3  ) /* EN                       */

/* FEEAEN2[DMASINC2] -  */
#define FEEAEN2_DMASINC2_BBA           (*(volatile unsigned long *) 0x42051008)
#define FEEAEN2_DMASINC2_MSK           (0x1   << 2  )
#define FEEAEN2_DMASINC2               (0x1   << 2  )
#define FEEAEN2_DMASINC2_DIS           (0x0   << 2  ) /* DIS                      */
#define FEEAEN2_DMASINC2_EN            (0x1   << 2  ) /* EN                       */

/* FEEAEN2[DMAADC1] -  */
#define FEEAEN2_DMAADC1_BBA            (*(volatile unsigned long *) 0x42051004)
#define FEEAEN2_DMAADC1_MSK            (0x1   << 1  )
#define FEEAEN2_DMAADC1                (0x1   << 1  )
#define FEEAEN2_DMAADC1_DIS            (0x0   << 1  ) /* DIS                      */
#define FEEAEN2_DMAADC1_EN             (0x1   << 1  ) /* EN                       */

// ------------------------------------------------------------------------------------------------
// -----                                        I2C                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief I2C (pADI_I2C)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_I2C Structure                     */
  __IO uint16_t  I2CMCON;                   /*!< Master Control Register               */
  __I  uint16_t  RESERVED0;
  __IO uint16_t  I2CMSTA;                   /*!< Master Status Register                */
  __I  uint16_t  RESERVED1;
  __IO uint8_t   I2CMRX;                    /*!< Master Receive Data                   */
  __I  uint8_t   RESERVED2[3];
  __IO uint8_t   I2CMTX;                    /*!< Master Transmit Data                  */
  __I  uint8_t   RESERVED3[3];
  __IO uint16_t  I2CMRXCNT;                 /*!< Master Receive Data Count             */
  __I  uint16_t  RESERVED4;
  __IO uint16_t  I2CMCRXCNT;                /*!< Master Current Receive Data Count     */
  __I  uint16_t  RESERVED5;
  __IO uint8_t   I2CADR0;                   /*!< 1st Master Address Byte               */
  __I  uint8_t   RESERVED6[3];
  __IO uint8_t   I2CADR1;                   /*!< 2nd Master Address Byte               */
  __I  uint8_t   RESERVED7[7];
  __IO uint16_t  I2CDIV;                    /*!< Serial clock period divisor register  */
  __I  uint16_t  RESERVED8;
  __IO uint16_t  I2CSCON;                   /*!< Slave Control Register                */
  __I  uint16_t  RESERVED9;
  __IO uint16_t  I2CSSTA;                   /*!< "Slave I2C Status, Error and IRQ Register" */
  __I  uint16_t  RESERVED10;
  __IO uint16_t  I2CSRX;                    /*!< Slave Receive Data Register           */
  __I  uint16_t  RESERVED11;
  __IO uint16_t  I2CSTX;                    /*!< Slave Transmit Data Register          */
  __I  uint16_t  RESERVED12;
  __IO uint16_t  I2CALT;                    /*!< Hardware General Call ID              */
  __I  uint16_t  RESERVED13;
  __IO uint16_t  I2CID0;                    /*!< 1st Slave Address Device ID           */
  __I  uint16_t  RESERVED14;
  __IO uint16_t  I2CID1;                    /*!< 2nd Slave Address Device ID           */
  __I  uint16_t  RESERVED15;
  __IO uint16_t  I2CID2;                    /*!< 3rd Slave Address Device ID           */
  __I  uint16_t  RESERVED16;
  __IO uint16_t  I2CID3;                    /*!< 4th Slave Address Device ID           */
  __I  uint16_t  RESERVED17;
  __IO uint16_t  I2CFSTA;                   /*!< Master and Slave Rx/Tx FIFO Status Register */
  __I  uint16_t  RESERVED18;
  __IO uint16_t  I2CSHCON;                  /*!< Shared control register               */
  __I  uint16_t  RESERVED19[3];
  __IO uint16_t  I2CASSCL;                  /*!< Automatic Stretch control register    */
} ADI_I2C_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          I2CMCON                                    (*(volatile unsigned short int *) 0x40003000)
#define          I2CMSTA                                    (*(volatile unsigned short int *) 0x40003004)
#define          I2CMRX                                     (*(volatile unsigned char      *) 0x40003008)
#define          I2CMTX                                     (*(volatile unsigned char      *) 0x4000300C)
#define          I2CMRXCNT                                  (*(volatile unsigned short int *) 0x40003010)
#define          I2CMCRXCNT                                 (*(volatile unsigned short int *) 0x40003014)
#define          I2CADR0                                    (*(volatile unsigned char      *) 0x40003018)
#define          I2CADR1                                    (*(volatile unsigned char      *) 0x4000301C)
#define          I2CDIV                                     (*(volatile unsigned short int *) 0x40003024)
#define          I2CSCON                                    (*(volatile unsigned short int *) 0x40003028)
#define          I2CSSTA                                    (*(volatile unsigned short int *) 0x4000302C)
#define          I2CSRX                                     (*(volatile unsigned short int *) 0x40003030)
#define          I2CSTX                                     (*(volatile unsigned short int *) 0x40003034)
#define          I2CALT                                     (*(volatile unsigned short int *) 0x40003038)
#define          I2CID0                                     (*(volatile unsigned short int *) 0x4000303C)
#define          I2CID1                                     (*(volatile unsigned short int *) 0x40003040)
#define          I2CID2                                     (*(volatile unsigned short int *) 0x40003044)
#define          I2CID3                                     (*(volatile unsigned short int *) 0x40003048)
#define          I2CFSTA                                    (*(volatile unsigned short int *) 0x4000304C)
#define          I2CSHCON                                   (*(volatile unsigned short int *) 0x40003050)
#define          I2CASSCL                                   (*(volatile unsigned short int *) 0x40003058)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for I2CMCON*/
#define I2CMCON_RVAL                   0x0 

/* I2CMCON[TXDMA] - Enable master Tx DMA request */
#define I2CMCON_TXDMA_BBA              (*(volatile unsigned long *) 0x4206002C)
#define I2CMCON_TXDMA_MSK              (0x1   << 11 )
#define I2CMCON_TXDMA                  (0x1   << 11 )
#define I2CMCON_TXDMA_DIS              (0x0   << 11 ) /* DIS                      */
#define I2CMCON_TXDMA_EN               (0x1   << 11 ) /* EN                       */

/* I2CMCON[RXDMA] - Enable master Rx DMA request */
#define I2CMCON_RXDMA_BBA              (*(volatile unsigned long *) 0x42060028)
#define I2CMCON_RXDMA_MSK              (0x1   << 10 )
#define I2CMCON_RXDMA                  (0x1   << 10 )
#define I2CMCON_RXDMA_DIS              (0x0   << 10 ) /* DIS                      */
#define I2CMCON_RXDMA_EN               (0x1   << 10 ) /* EN                       */

/* I2CMCON[IENCMP] - Transaction completed interrupt enable */
#define I2CMCON_IENCMP_BBA             (*(volatile unsigned long *) 0x42060020)
#define I2CMCON_IENCMP_MSK             (0x1   << 8  )
#define I2CMCON_IENCMP                 (0x1   << 8  )
#define I2CMCON_IENCMP_DIS             (0x0   << 8  ) /* DIS                      */
#define I2CMCON_IENCMP_EN              (0x1   << 8  ) /* EN                       */

/* I2CMCON[IENNACK] - ACK not received interrupt enable */
#define I2CMCON_IENNACK_BBA            (*(volatile unsigned long *) 0x4206001C)
#define I2CMCON_IENNACK_MSK            (0x1   << 7  )
#define I2CMCON_IENNACK                (0x1   << 7  )
#define I2CMCON_IENNACK_DIS            (0x0   << 7  ) /* DIS                      */
#define I2CMCON_IENNACK_EN             (0x1   << 7  ) /* EN                       */

/* I2CMCON[IENALOST] - Arbitration lost interrupt enable */
#define I2CMCON_IENALOST_BBA           (*(volatile unsigned long *) 0x42060018)
#define I2CMCON_IENALOST_MSK           (0x1   << 6  )
#define I2CMCON_IENALOST               (0x1   << 6  )
#define I2CMCON_IENALOST_DIS           (0x0   << 6  ) /* DIS                      */
#define I2CMCON_IENALOST_EN            (0x1   << 6  ) /* EN                       */

/* I2CMCON[IENTX] - Transmit request interrupt enable */
#define I2CMCON_IENTX_BBA              (*(volatile unsigned long *) 0x42060014)
#define I2CMCON_IENTX_MSK              (0x1   << 5  )
#define I2CMCON_IENTX                  (0x1   << 5  )
#define I2CMCON_IENTX_DIS              (0x0   << 5  ) /* DIS                      */
#define I2CMCON_IENTX_EN               (0x1   << 5  ) /* EN                       */

/* I2CMCON[IENRX] - Receive request interrupt enable */
#define I2CMCON_IENRX_BBA              (*(volatile unsigned long *) 0x42060010)
#define I2CMCON_IENRX_MSK              (0x1   << 4  )
#define I2CMCON_IENRX                  (0x1   << 4  )
#define I2CMCON_IENRX_DIS              (0x0   << 4  ) /* DIS                      */
#define I2CMCON_IENRX_EN               (0x1   << 4  ) /* EN                       */

/* I2CMCON[STRETCH] - Stretch SCL */
#define I2CMCON_STRETCH_BBA            (*(volatile unsigned long *) 0x4206000C)
#define I2CMCON_STRETCH_MSK            (0x1   << 3  )
#define I2CMCON_STRETCH                (0x1   << 3  )
#define I2CMCON_STRETCH_DIS            (0x0   << 3  ) /* DIS                      */
#define I2CMCON_STRETCH_EN             (0x1   << 3  ) /* EN                       */

/* I2CMCON[LOOPBACK] - Internal loop back */
#define I2CMCON_LOOPBACK_BBA           (*(volatile unsigned long *) 0x42060008)
#define I2CMCON_LOOPBACK_MSK           (0x1   << 2  )
#define I2CMCON_LOOPBACK               (0x1   << 2  )
#define I2CMCON_LOOPBACK_DIS           (0x0   << 2  ) /* DIS                      */
#define I2CMCON_LOOPBACK_EN            (0x1   << 2  ) /* EN                       */

/* I2CMCON[COMPETE] - Compete for ownership */
#define I2CMCON_COMPETE_BBA            (*(volatile unsigned long *) 0x42060004)
#define I2CMCON_COMPETE_MSK            (0x1   << 1  )
#define I2CMCON_COMPETE                (0x1   << 1  )
#define I2CMCON_COMPETE_DIS            (0x0   << 1  ) /* DIS                      */
#define I2CMCON_COMPETE_EN             (0x1   << 1  ) /* EN                       */

/* I2CMCON[MAS] - Master Enable */
#define I2CMCON_MAS_BBA                (*(volatile unsigned long *) 0x42060000)
#define I2CMCON_MAS_MSK                (0x1   << 0  )
#define I2CMCON_MAS                    (0x1   << 0  )
#define I2CMCON_MAS_DIS                (0x0   << 0  ) /* DIS                      */
#define I2CMCON_MAS_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for I2CMSTA*/
#define I2CMSTA_RVAL                   0x0 

/* I2CMSTA[TXUR] - Master Transmit FIFO underflow */
#define I2CMSTA_TXUR_BBA               (*(volatile unsigned long *) 0x420600B0)
#define I2CMSTA_TXUR_MSK               (0x1   << 12 )
#define I2CMSTA_TXUR                   (0x1   << 12 )
#define I2CMSTA_TXUR_CLR               (0x0   << 12 ) /* CLR                      */
#define I2CMSTA_TXUR_SET               (0x1   << 12 ) /* SET                      */

/* I2CMSTA[MSTOP] - STOP driven by th eI2C master */
#define I2CMSTA_MSTOP_BBA              (*(volatile unsigned long *) 0x420600AC)
#define I2CMSTA_MSTOP_MSK              (0x1   << 11 )
#define I2CMSTA_MSTOP                  (0x1   << 11 )
#define I2CMSTA_MSTOP_CLR              (0x0   << 11 ) /* CLR                      */
#define I2CMSTA_MSTOP_SET              (0x1   << 11 ) /* SET                      */

/* I2CMSTA[LINEBUSY] - Line is busy */
#define I2CMSTA_LINEBUSY_BBA           (*(volatile unsigned long *) 0x420600A8)
#define I2CMSTA_LINEBUSY_MSK           (0x1   << 10 )
#define I2CMSTA_LINEBUSY               (0x1   << 10 )
#define I2CMSTA_LINEBUSY_CLR           (0x0   << 10 ) /* CLR                      */
#define I2CMSTA_LINEBUSY_SET           (0x1   << 10 ) /* SET                      */

/* I2CMSTA[RXOF] - Receive FIFO overflow */
#define I2CMSTA_RXOF_BBA               (*(volatile unsigned long *) 0x420600A4)
#define I2CMSTA_RXOF_MSK               (0x1   << 9  )
#define I2CMSTA_RXOF                   (0x1   << 9  )
#define I2CMSTA_RXOF_CLR               (0x0   << 9  ) /* CLR                      */
#define I2CMSTA_RXOF_SET               (0x1   << 9  ) /* SET                      */

/* I2CMSTA[TCOMP] - Transaction completed */
#define I2CMSTA_TCOMP_BBA              (*(volatile unsigned long *) 0x420600A0)
#define I2CMSTA_TCOMP_MSK              (0x1   << 8  )
#define I2CMSTA_TCOMP                  (0x1   << 8  )
#define I2CMSTA_TCOMP_CLR              (0x0   << 8  ) /* CLR                      */
#define I2CMSTA_TCOMP_SET              (0x1   << 8  ) /* SET                      */

/* I2CMSTA[NACKDATA] - Ack not received in response to data write */
#define I2CMSTA_NACKDATA_BBA           (*(volatile unsigned long *) 0x4206009C)
#define I2CMSTA_NACKDATA_MSK           (0x1   << 7  )
#define I2CMSTA_NACKDATA               (0x1   << 7  )
#define I2CMSTA_NACKDATA_CLR           (0x0   << 7  ) /* CLR                      */
#define I2CMSTA_NACKDATA_SET           (0x1   << 7  ) /* SET                      */

/* I2CMSTA[BUSY] - Master Busy */
#define I2CMSTA_BUSY_BBA               (*(volatile unsigned long *) 0x42060098)
#define I2CMSTA_BUSY_MSK               (0x1   << 6  )
#define I2CMSTA_BUSY                   (0x1   << 6  )
#define I2CMSTA_BUSY_CLR               (0x0   << 6  ) /* CLR                      */
#define I2CMSTA_BUSY_SET               (0x1   << 6  ) /* SET                      */

/* I2CMSTA[ALOST] - Arbitration lost */
#define I2CMSTA_ALOST_BBA              (*(volatile unsigned long *) 0x42060094)
#define I2CMSTA_ALOST_MSK              (0x1   << 5  )
#define I2CMSTA_ALOST                  (0x1   << 5  )
#define I2CMSTA_ALOST_CLR              (0x0   << 5  ) /* CLR                      */
#define I2CMSTA_ALOST_SET              (0x1   << 5  ) /* SET                      */

/* I2CMSTA[NACKADDR] - Ack not received in response to an address */
#define I2CMSTA_NACKADDR_BBA           (*(volatile unsigned long *) 0x42060090)
#define I2CMSTA_NACKADDR_MSK           (0x1   << 4  )
#define I2CMSTA_NACKADDR               (0x1   << 4  )
#define I2CMSTA_NACKADDR_CLR           (0x0   << 4  ) /* CLR                      */
#define I2CMSTA_NACKADDR_SET           (0x1   << 4  ) /* SET                      */

/* I2CMSTA[RXREQ] - Receive request */
#define I2CMSTA_RXREQ_BBA              (*(volatile unsigned long *) 0x4206008C)
#define I2CMSTA_RXREQ_MSK              (0x1   << 3  )
#define I2CMSTA_RXREQ                  (0x1   << 3  )
#define I2CMSTA_RXREQ_CLR              (0x0   << 3  ) /* CLR                      */
#define I2CMSTA_RXREQ_SET              (0x1   << 3  ) /* SET                      */

/* I2CMSTA[TXREQ] - Transmit request */
#define I2CMSTA_TXREQ_BBA              (*(volatile unsigned long *) 0x42060088)
#define I2CMSTA_TXREQ_MSK              (0x1   << 2  )
#define I2CMSTA_TXREQ                  (0x1   << 2  )
#define I2CMSTA_TXREQ_CLR              (0x0   << 2  ) /* CLR                      */
#define I2CMSTA_TXREQ_SET              (0x1   << 2  ) /* SET                      */

/* I2CMSTA[TXFSTA] - Transmit FIFO Status */
#define I2CMSTA_TXFSTA_MSK             (0x3   << 0  )
#define I2CMSTA_TXFSTA_EMPTY           (0x0   << 0  ) /* EMPTY                    */
#define I2CMSTA_TXFSTA_ONEBYTE         (0x1   << 0  ) /* ONEBYTE                  */
#define I2CMSTA_TXFSTA_FULL            (0x3   << 0  ) /* FULL                     */

/* Reset Value for I2CMRX*/
#define I2CMRX_RVAL                    0x0 

/* I2CMRX[VALUE] - Current Receive Value */
#define I2CMRX_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for I2CMTX*/
#define I2CMTX_RVAL                    0x0 

/* I2CMTX[VALUE] - Current Transmit Value */
#define I2CMTX_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for I2CMRXCNT*/
#define I2CMRXCNT_RVAL                 0x0 

/* I2CMRXCNT[EXTEND] - Extended Read */
#define I2CMRXCNT_EXTEND_BBA           (*(volatile unsigned long *) 0x42060220)
#define I2CMRXCNT_EXTEND_MSK           (0x1   << 8  )
#define I2CMRXCNT_EXTEND               (0x1   << 8  )
#define I2CMRXCNT_EXTEND_DIS           (0x0   << 8  ) /* DIS                      */
#define I2CMRXCNT_EXTEND_EN            (0x1   << 8  ) /* EN                       */

/* I2CMRXCNT[COUNT] - Receive count */
#define I2CMRXCNT_COUNT_MSK            (0xFF  << 0  )

/* Reset Value for I2CMCRXCNT*/
#define I2CMCRXCNT_RVAL                0x0 

/* I2CMCRXCNT[VALUE] - Current Receive count */
#define I2CMCRXCNT_VALUE_MSK           (0xFF  << 0  )

/* Reset Value for I2CADR0*/
#define I2CADR0_RVAL                   0x0 

/* I2CADR0[VALUE] - Address byte */
#define I2CADR0_VALUE_MSK              (0xFF  << 0  )

/* Reset Value for I2CADR1*/
#define I2CADR1_RVAL                   0x0 

/* I2CADR1[VALUE] - Address byte */
#define I2CADR1_VALUE_MSK              (0xFF  << 0  )

/* Reset Value for I2CDIV*/
#define I2CDIV_RVAL                    0x1F1F 

/* I2CDIV[HIGH] - High Time */
#define I2CDIV_HIGH_MSK                (0xFF  << 8  )

/* I2CDIV[LOW] - Low Time */
#define I2CDIV_LOW_MSK                 (0xFF  << 0  )

/* Reset Value for I2CSCON*/
#define I2CSCON_RVAL                   0x0 

/* I2CSCON[TXDMA] - Enable slave Tx DMA request */
#define I2CSCON_TXDMA_BBA              (*(volatile unsigned long *) 0x42060538)
#define I2CSCON_TXDMA_MSK              (0x1   << 14 )
#define I2CSCON_TXDMA                  (0x1   << 14 )
#define I2CSCON_TXDMA_DIS              (0x0   << 14 ) /* DIS                      */
#define I2CSCON_TXDMA_EN               (0x1   << 14 ) /* EN                       */

/* I2CSCON[RXDMA] - Enable slave Rx DMA request */
#define I2CSCON_RXDMA_BBA              (*(volatile unsigned long *) 0x42060534)
#define I2CSCON_RXDMA_MSK              (0x1   << 13 )
#define I2CSCON_RXDMA                  (0x1   << 13 )
#define I2CSCON_RXDMA_DIS              (0x0   << 13 ) /* DIS                      */
#define I2CSCON_RXDMA_EN               (0x1   << 13 ) /* EN                       */

/* I2CSCON[IENREPST] - Repeated start interrupt enable */
#define I2CSCON_IENREPST_BBA           (*(volatile unsigned long *) 0x42060530)
#define I2CSCON_IENREPST_MSK           (0x1   << 12 )
#define I2CSCON_IENREPST               (0x1   << 12 )
#define I2CSCON_IENREPST_DIS           (0x0   << 12 ) /* DIS                      */
#define I2CSCON_IENREPST_EN            (0x1   << 12 ) /* EN                       */

/* I2CSCON[IENTX] - Transmit request interrupt enable */
#define I2CSCON_IENTX_BBA              (*(volatile unsigned long *) 0x42060528)
#define I2CSCON_IENTX_MSK              (0x1   << 10 )
#define I2CSCON_IENTX                  (0x1   << 10 )
#define I2CSCON_IENTX_DIS              (0x0   << 10 ) /* DIS                      */
#define I2CSCON_IENTX_EN               (0x1   << 10 ) /* EN                       */

/* I2CSCON[IENRX] - Receive request interrupt enable */
#define I2CSCON_IENRX_BBA              (*(volatile unsigned long *) 0x42060524)
#define I2CSCON_IENRX_MSK              (0x1   << 9  )
#define I2CSCON_IENRX                  (0x1   << 9  )
#define I2CSCON_IENRX_DIS              (0x0   << 9  ) /* DIS                      */
#define I2CSCON_IENRX_EN               (0x1   << 9  ) /* EN                       */

/* I2CSCON[IENSTOP] - Stop condition detected interrupt enable */
#define I2CSCON_IENSTOP_BBA            (*(volatile unsigned long *) 0x42060520)
#define I2CSCON_IENSTOP_MSK            (0x1   << 8  )
#define I2CSCON_IENSTOP                (0x1   << 8  )
#define I2CSCON_IENSTOP_DIS            (0x0   << 8  ) /* DIS                      */
#define I2CSCON_IENSTOP_EN             (0x1   << 8  ) /* EN                       */

/* I2CSCON[NACK] - NACK next communication */
#define I2CSCON_NACK_BBA               (*(volatile unsigned long *) 0x4206051C)
#define I2CSCON_NACK_MSK               (0x1   << 7  )
#define I2CSCON_NACK                   (0x1   << 7  )
#define I2CSCON_NACK_DIS               (0x0   << 7  ) /* DIS                      */
#define I2CSCON_NACK_EN                (0x1   << 7  ) /* EN                       */

/* I2CSCON[STRETCH] - Stretch SCL */
#define I2CSCON_STRETCH_BBA            (*(volatile unsigned long *) 0x42060518)
#define I2CSCON_STRETCH_MSK            (0x1   << 6  )
#define I2CSCON_STRETCH                (0x1   << 6  )
#define I2CSCON_STRETCH_DIS            (0x0   << 6  ) /* DIS                      */
#define I2CSCON_STRETCH_EN             (0x1   << 6  ) /* EN                       */

/* I2CSCON[EARLYTXR] - Early transmit request mode */
#define I2CSCON_EARLYTXR_BBA           (*(volatile unsigned long *) 0x42060514)
#define I2CSCON_EARLYTXR_MSK           (0x1   << 5  )
#define I2CSCON_EARLYTXR               (0x1   << 5  )
#define I2CSCON_EARLYTXR_DIS           (0x0   << 5  ) /* DIS                      */
#define I2CSCON_EARLYTXR_EN            (0x1   << 5  ) /* EN                       */

/* I2CSCON[GCSB] - General call status bit clear */
#define I2CSCON_GCSB_BBA               (*(volatile unsigned long *) 0x42060510)
#define I2CSCON_GCSB_MSK               (0x1   << 4  )
#define I2CSCON_GCSB                   (0x1   << 4  )
#define I2CSCON_GCSB_CLR               (0x1   << 4  ) /* CLR                      */

/* I2CSCON[HGC] - Hardware general Call enable */
#define I2CSCON_HGC_BBA                (*(volatile unsigned long *) 0x4206050C)
#define I2CSCON_HGC_MSK                (0x1   << 3  )
#define I2CSCON_HGC                    (0x1   << 3  )
#define I2CSCON_HGC_DIS                (0x0   << 3  ) /* DIS                      */
#define I2CSCON_HGC_EN                 (0x1   << 3  ) /* EN                       */

/* I2CSCON[GC] - General Call enable */
#define I2CSCON_GC_BBA                 (*(volatile unsigned long *) 0x42060508)
#define I2CSCON_GC_MSK                 (0x1   << 2  )
#define I2CSCON_GC                     (0x1   << 2  )
#define I2CSCON_GC_DIS                 (0x0   << 2  ) /* DIS                      */
#define I2CSCON_GC_EN                  (0x1   << 2  ) /* EN                       */

/* I2CSCON[ADR10] - Enable 10 bit addressing */
#define I2CSCON_ADR10_BBA              (*(volatile unsigned long *) 0x42060504)
#define I2CSCON_ADR10_MSK              (0x1   << 1  )
#define I2CSCON_ADR10                  (0x1   << 1  )
#define I2CSCON_ADR10_DIS              (0x0   << 1  ) /* DIS                      */
#define I2CSCON_ADR10_EN               (0x1   << 1  ) /* EN                       */

/* I2CSCON[SLV] - Slave Enable */
#define I2CSCON_SLV_BBA                (*(volatile unsigned long *) 0x42060500)
#define I2CSCON_SLV_MSK                (0x1   << 0  )
#define I2CSCON_SLV                    (0x1   << 0  )
#define I2CSCON_SLV_DIS                (0x0   << 0  ) /* DIS                      */
#define I2CSCON_SLV_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for I2CSSTA*/
#define I2CSSTA_RVAL                   0x1 

/* I2CSSTA[START] - Start and matching address */
#define I2CSSTA_START_BBA              (*(volatile unsigned long *) 0x420605B8)
#define I2CSSTA_START_MSK              (0x1   << 14 )
#define I2CSSTA_START                  (0x1   << 14 )
#define I2CSSTA_START_CLR              (0x0   << 14 ) /* CLR                      */
#define I2CSSTA_START_SET              (0x1   << 14 ) /* SET                      */

/* I2CSSTA[REPSTART] - Repeated start and matching address */
#define I2CSSTA_REPSTART_BBA           (*(volatile unsigned long *) 0x420605B4)
#define I2CSSTA_REPSTART_MSK           (0x1   << 13 )
#define I2CSSTA_REPSTART               (0x1   << 13 )
#define I2CSSTA_REPSTART_CLR           (0x0   << 13 ) /* CLR                      */
#define I2CSSTA_REPSTART_SET           (0x1   << 13 ) /* SET                      */

/* I2CSSTA[IDMAT] - Device ID matched */
#define I2CSSTA_IDMAT_MSK              (0x3   << 11 )
#define I2CSSTA_IDMAT_CLR              (0x0   << 11 ) /* CLR                      */
#define I2CSSTA_IDMAT_SET              (0x1   << 11 ) /* SET                      */

/* I2CSSTA[STOP] - Stop after start and matching address */
#define I2CSSTA_STOP_BBA               (*(volatile unsigned long *) 0x420605A8)
#define I2CSSTA_STOP_MSK               (0x1   << 10 )
#define I2CSSTA_STOP                   (0x1   << 10 )
#define I2CSSTA_STOP_CLR               (0x0   << 10 ) /* CLR                      */
#define I2CSSTA_STOP_SET               (0x1   << 10 ) /* SET                      */

/* I2CSSTA[GCID] - General ID */
#define I2CSSTA_GCID_MSK               (0x3   << 8  )
#define I2CSSTA_GCID_CLR               (0x0   << 8  ) /* CLR                      */
#define I2CSSTA_GCID_SET               (0x1   << 8  ) /* SET                      */

/* I2CSSTA[GCINT] - General call */
#define I2CSSTA_GCINT_BBA              (*(volatile unsigned long *) 0x4206059C)
#define I2CSSTA_GCINT_MSK              (0x1   << 7  )
#define I2CSSTA_GCINT                  (0x1   << 7  )
#define I2CSSTA_GCINT_CLR              (0x0   << 7  ) /* CLR                      */
#define I2CSSTA_GCINT_SET              (0x1   << 7  ) /* SET                      */

/* I2CSSTA[BUSY] - Slave busy */
#define I2CSSTA_BUSY_BBA               (*(volatile unsigned long *) 0x42060598)
#define I2CSSTA_BUSY_MSK               (0x1   << 6  )
#define I2CSSTA_BUSY                   (0x1   << 6  )
#define I2CSSTA_BUSY_CLR               (0x0   << 6  ) /* CLR                      */
#define I2CSSTA_BUSY_SET               (0x1   << 6  ) /* SET                      */

/* I2CSSTA[NOACK] - Ack not generated by the slave */
#define I2CSSTA_NOACK_BBA              (*(volatile unsigned long *) 0x42060594)
#define I2CSSTA_NOACK_MSK              (0x1   << 5  )
#define I2CSSTA_NOACK                  (0x1   << 5  )
#define I2CSSTA_NOACK_CLR              (0x0   << 5  ) /* CLR                      */
#define I2CSSTA_NOACK_SET              (0x1   << 5  ) /* SET                      */

/* I2CSSTA[RXOF] - Receive FIFO */
#define I2CSSTA_RXOF_BBA               (*(volatile unsigned long *) 0x42060590)
#define I2CSSTA_RXOF_MSK               (0x1   << 4  )
#define I2CSSTA_RXOF                   (0x1   << 4  )
#define I2CSSTA_RXOF_CLR               (0x0   << 4  ) /* CLR                      */
#define I2CSSTA_RXOF_SET               (0x1   << 4  ) /* SET                      */

/* I2CSSTA[RXREQ] - Receive */
#define I2CSSTA_RXREQ_BBA              (*(volatile unsigned long *) 0x4206058C)
#define I2CSSTA_RXREQ_MSK              (0x1   << 3  )
#define I2CSSTA_RXREQ                  (0x1   << 3  )
#define I2CSSTA_RXREQ_CLR              (0x0   << 3  ) /* CLR                      */
#define I2CSSTA_RXREQ_SET              (0x1   << 3  ) /* SET                      */

/* I2CSSTA[TXREQ] - Transmit */
#define I2CSSTA_TXREQ_BBA              (*(volatile unsigned long *) 0x42060588)
#define I2CSSTA_TXREQ_MSK              (0x1   << 2  )
#define I2CSSTA_TXREQ                  (0x1   << 2  )
#define I2CSSTA_TXREQ_CLR              (0x0   << 2  ) /* CLR                      */
#define I2CSSTA_TXREQ_SET              (0x1   << 2  ) /* SET                      */

/* I2CSSTA[TXUR] - Transmit FIFO underflow */
#define I2CSSTA_TXUR_BBA               (*(volatile unsigned long *) 0x42060584)
#define I2CSSTA_TXUR_MSK               (0x1   << 1  )
#define I2CSSTA_TXUR                   (0x1   << 1  )
#define I2CSSTA_TXUR_CLR               (0x0   << 1  ) /* CLR                      */
#define I2CSSTA_TXUR_SET               (0x1   << 1  ) /* SET                      */

/* I2CSSTA[TXFSEREQ] - Tx FIFO status or early request */
#define I2CSSTA_TXFSEREQ_BBA           (*(volatile unsigned long *) 0x42060580)
#define I2CSSTA_TXFSEREQ_MSK           (0x1   << 0  )
#define I2CSSTA_TXFSEREQ               (0x1   << 0  )
#define I2CSSTA_TXFSEREQ_CLR           (0x0   << 0  ) /* CLR                      */
#define I2CSSTA_TXFSEREQ_SET           (0x1   << 0  ) /* SET                      */

/* Reset Value for I2CSRX*/
#define I2CSRX_RVAL                    0x0 

/* I2CSRX[VALUE] - Receive register */
#define I2CSRX_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for I2CSTX*/
#define I2CSTX_RVAL                    0x0 

/* I2CSTX[VALUE] - Transmit register */
#define I2CSTX_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for I2CALT*/
#define I2CALT_RVAL                    0x0 

/* I2CALT[VALUE] - Alt register */
#define I2CALT_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for I2CID0*/
#define I2CID0_RVAL                    0x0 

/* I2CID0[VALUE] - Slave ID */
#define I2CID0_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for I2CID1*/
#define I2CID1_RVAL                    0x0 

/* I2CID1[VALUE] - Slave ID */
#define I2CID1_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for I2CID2*/
#define I2CID2_RVAL                    0x0 

/* I2CID2[VALUE] - Slave ID */
#define I2CID2_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for I2CID3*/
#define I2CID3_RVAL                    0x0 

/* I2CID3[VALUE] - Slave ID */
#define I2CID3_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for I2CFSTA*/
#define I2CFSTA_RVAL                   0x0 

/* I2CFSTA[MFLUSH] - Flush the master transmit FIFO */
#define I2CFSTA_MFLUSH_BBA             (*(volatile unsigned long *) 0x420609A4)
#define I2CFSTA_MFLUSH_MSK             (0x1   << 9  )
#define I2CFSTA_MFLUSH                 (0x1   << 9  )
#define I2CFSTA_MFLUSH_DIS             (0x0   << 9  ) /* DIS                      */
#define I2CFSTA_MFLUSH_EN              (0x1   << 9  ) /* EN                       */

/* I2CFSTA[SFLUSH] - Flush the slave transmit FIFO */
#define I2CFSTA_SFLUSH_BBA             (*(volatile unsigned long *) 0x420609A0)
#define I2CFSTA_SFLUSH_MSK             (0x1   << 8  )
#define I2CFSTA_SFLUSH                 (0x1   << 8  )
#define I2CFSTA_SFLUSH_DIS             (0x0   << 8  ) /* DIS                      */
#define I2CFSTA_SFLUSH_EN              (0x1   << 8  ) /* EN                       */

/* I2CFSTA[MRXFSTA] - Master receive FIFO Status */
#define I2CFSTA_MRXFSTA_MSK            (0x3   << 6  )
#define I2CFSTA_MRXFSTA_EMPTY          (0x0   << 6  ) /* EMPTY                    */
#define I2CFSTA_MRXFSTA_ONEBYTE        (0x1   << 6  ) /* ONEBYTE                  */
#define I2CFSTA_MRXFSTA_TWOBYTES       (0x2   << 6  ) /* TWOBYTES                 */

/* I2CFSTA[MTXFSTA] - Master Transmit FIFO Status */
#define I2CFSTA_MTXFSTA_MSK            (0x3   << 4  )
#define I2CFSTA_MTXFSTA_EMPTY          (0x0   << 4  ) /* EMPTY                    */
#define I2CFSTA_MTXFSTA_ONEBYTE        (0x1   << 4  ) /* ONEBYTE                  */
#define I2CFSTA_MTXFSTA_TWOBYTES       (0x2   << 4  ) /* TWOBYTES                 */

/* I2CFSTA[SRXFSTA] - Slave receive FIFO Status */
#define I2CFSTA_SRXFSTA_MSK            (0x3   << 2  )
#define I2CFSTA_SRXFSTA_EMPTY          (0x0   << 2  ) /* EMPTY                    */
#define I2CFSTA_SRXFSTA_ONEBYTE        (0x1   << 2  ) /* ONEBYTE                  */
#define I2CFSTA_SRXFSTA_TWOBYTES       (0x2   << 2  ) /* TWOBYTES                 */

/* I2CFSTA[STXFSTA] - Slave Transmit FIFO Status */
#define I2CFSTA_STXFSTA_MSK            (0x3   << 0  )
#define I2CFSTA_STXFSTA_EMPTY          (0x0   << 0  ) /* EMPTY                    */
#define I2CFSTA_STXFSTA_ONEBYTE        (0x1   << 0  ) /* ONEBYTE                  */
#define I2CFSTA_STXFSTA_TWOBYTES       (0x2   << 0  ) /* TWOBYTES                 */

/* I2CSHCON[RESET] - Reset START STOP detect circuit */
#define I2CSHCON_RESET_MSK             (0x1   << 0  )
#define I2CSHCON_RESET                 (0x1   << 0  )
#define I2CSHCON_RESET_DIS             (0x0   << 0  )
#define I2CSHCON_RESET_EN              (0x1   << 0  )

/* I2CASSCL[SSRTSTA] - stretch timeout for slave */
#define I2CASSCL_SSRTSTA_MSK           (0x1   << 9  )
#define I2CASSCL_SSRTSTA               (0x1   << 9  )
#define I2CASSCL_SSRTSTA_DIS           (0x0   << 9  )
#define I2CASSCL_SSRTSTA_EN            (0x1   << 9  )

/* I2CASSCL[MSRTSTA] - stretch timeout for master */
#define I2CASSCL_MSRTSTA_MSK           (0x1   << 8  )
#define I2CASSCL_MSRTSTA               (0x1   << 8  )
#define I2CASSCL_MSRTSTA_DIS           (0x0   << 8  )
#define I2CASSCL_MSRTSTA_EN            (0x1   << 8  )

/* I2CASSCL[SSTRCON] - automatic stretch mode for slave */
#define I2CASSCL_SSTRCON_MSK           (0xF   << 4  )

/* I2CASSCL[MSTRCON] - automatic stretch mode for master */
#define I2CASSCL_MSTRCON_MSK           (0xF   << 0  )

// ------------------------------------------------------------------------------------------------
// -----                                        SPI0                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Serial Peripheral Interface (pADI_SPI0)
  */

typedef struct {                            /*!< pADI_SPI0 Structure                    */
  __IO uint16_t  SPISTA;                    /*!< Status Register                       */
  __I  uint16_t  RESERVED0;
  __IO uint8_t   SPIRX;                     /*!< 8-bit Receive register.               */
  __I  uint8_t   RESERVED1[3];
  __IO uint8_t   SPITX;                     /*!< 8-bit Transmit register               */
  __I  uint8_t   RESERVED2[3];
  __IO uint16_t  SPIDIV;                    /*!< SPI Clock Divider Registers           */
  __I  uint16_t  RESERVED3;
  __IO uint16_t  SPICON;                    /*!< 16-bit configuration register         */
  __I  uint16_t  RESERVED4;
  __IO uint16_t  SPIDMA;                    /*!< DMA enable register                   */
  __I  uint16_t  RESERVED5;
  __IO uint16_t  SPICNT;                    /*!< 8-bit received byte count register    */
} ADI_SPI_TypeDef;

#define          SPI0STA                                    (*(volatile unsigned short int *) 0x40004000)
#define          SPI0RX                                     (*(volatile unsigned char      *) 0x40004004)
#define          SPI0TX                                     (*(volatile unsigned char      *) 0x40004008)
#define          SPI0DIV                                    (*(volatile unsigned short int *) 0x4000400C)
#define          SPI0CON                                    (*(volatile unsigned short int *) 0x40004010)
#define          SPI0DMA                                    (*(volatile unsigned short int *) 0x40004014)
#define          SPI0CNT                                    (*(volatile unsigned short int *) 0x40004018)

/* Reset Value for SPI0STA*/
#define SPI0STA_RVAL                   0x0 

/* SPI0STA[CSRSG] - Detected a rising edge on CS, in CONT mode */
#define SPI0STA_CSRSG_BBA              (*(volatile unsigned long *) 0x42080038)
#define SPI0STA_CSRSG_MSK              (0x1   << 14 )
#define SPI0STA_CSRSG                  (0x1   << 14 )
#define SPI0STA_CSRSG_CLR              (0x0   << 14 ) /* Cleared to 0 when the Status register is read. */
#define SPI0STA_CSRSG_SET              (0x1   << 14 ) /* Set to 1 when there was a rising edge in CS line, when the device was in master mode, continuous transfer, High Frequency mode and CSIRQ_EN was asserted. */

/* SPI0STA[CSFLG] - Detected a falling edge on CS, in CONT mode */
#define SPI0STA_CSFLG_BBA              (*(volatile unsigned long *) 0x42080034)
#define SPI0STA_CSFLG_MSK              (0x1   << 13 )
#define SPI0STA_CSFLG                  (0x1   << 13 )
#define SPI0STA_CSFLG_CLR              (0x0   << 13 ) /* Cleared to 0 when the Status register is read. */
#define SPI0STA_CSFLG_SET              (0x1   << 13 ) /* Set to 1 when there was a falling edge in CS line, when the device was in master mode, continuous transfer, High Frequency mode and CSIRQ_EN was asserted */

/* SPI0STA[CSERR] - Detected an abrupt CS deassertion */
#define SPI0STA_CSERR_BBA              (*(volatile unsigned long *) 0x42080030)
#define SPI0STA_CSERR_MSK              (0x1   << 12 )
#define SPI0STA_CSERR                  (0x1   << 12 )
#define SPI0STA_CSERR_CLR              (0x0   << 12 ) /* CLR                      */
#define SPI0STA_CSERR_SET              (0x1   << 12 ) /* SET                      */

/* SPI0STA[RXS] - Set when there are more bytes in the RX FIFO than the TIM bit says */
#define SPI0STA_RXS_BBA                (*(volatile unsigned long *) 0x4208002C)
#define SPI0STA_RXS_MSK                (0x1   << 11 )
#define SPI0STA_RXS                    (0x1   << 11 )
#define SPI0STA_RXS_CLR                (0x0   << 11 ) /* CLR                      */
#define SPI0STA_RXS_SET                (0x1   << 11 ) /* SET                      */

/* SPI0STA[RXFSTA] - Receive FIFO Status */
#define SPI0STA_RXFSTA_MSK             (0x7   << 8  )
#define SPI0STA_RXFSTA_EMPTY           (0x0   << 8  ) /* EMPTY                    */
#define SPI0STA_RXFSTA_ONEBYTE         (0x1   << 8  ) /* ONEBYTE                  */
#define SPI0STA_RXFSTA_TWOBYTES        (0x2   << 8  ) /* TWOBYTES                 */
#define SPI0STA_RXFSTA_THREEBYTES      (0x3   << 8  ) /* THREEBYTES               */
#define SPI0STA_RXFSTA_FOURBYTES       (0x4   << 8  ) /* FOURBYTES                */

/* SPI0STA[RXOF] - Receive FIFO overflow */
#define SPI0STA_RXOF_BBA               (*(volatile unsigned long *) 0x4208001C)
#define SPI0STA_RXOF_MSK               (0x1   << 7  )
#define SPI0STA_RXOF                   (0x1   << 7  )
#define SPI0STA_RXOF_CLR               (0x0   << 7  ) /* CLR                      */
#define SPI0STA_RXOF_SET               (0x1   << 7  ) /* SET                      */

/* SPI0STA[RX] - Set when a receive interrupt occurs */
#define SPI0STA_RX_BBA                 (*(volatile unsigned long *) 0x42080018)
#define SPI0STA_RX_MSK                 (0x1   << 6  )
#define SPI0STA_RX                     (0x1   << 6  )
#define SPI0STA_RX_CLR                 (0x0   << 6  ) /* CLR                      */
#define SPI0STA_RX_SET                 (0x1   << 6  ) /* SET                      */

/* SPI0STA[TX] - Set when a transmit interrupt occurs */
#define SPI0STA_TX_BBA                 (*(volatile unsigned long *) 0x42080014)
#define SPI0STA_TX_MSK                 (0x1   << 5  )
#define SPI0STA_TX                     (0x1   << 5  )
#define SPI0STA_TX_CLR                 (0x0   << 5  ) /* CLR                      */
#define SPI0STA_TX_SET                 (0x1   << 5  ) /* SET                      */

/* SPI0STA[TXUR] - Transmit FIFO underflow */
#define SPI0STA_TXUR_BBA               (*(volatile unsigned long *) 0x42080010)
#define SPI0STA_TXUR_MSK               (0x1   << 4  )
#define SPI0STA_TXUR                   (0x1   << 4  )
#define SPI0STA_TXUR_CLR               (0x0   << 4  ) /* CLR                      */
#define SPI0STA_TXUR_SET               (0x1   << 4  ) /* SET                      */

/* SPI0STA[TXFSTA] - transmit FIFO Status */
#define SPI0STA_TXFSTA_MSK             (0x7   << 1  )
#define SPI0STA_TXFSTA_EMPTY           (0x0   << 1  ) /* EMPTY                    */
#define SPI0STA_TXFSTA_ONEBYTE         (0x1   << 1  ) /* ONEBYTE                  */
#define SPI0STA_TXFSTA_TWOBYTES        (0x2   << 1  ) /* TWOBYTES                 */
#define SPI0STA_TXFSTA_THREEBYTES      (0x3   << 1  ) /* THREEBYTES               */
#define SPI0STA_TXFSTA_FOURBYTES       (0x4   << 1  ) /* FOURBYTES                */

/* SPI0STA[IRQ] - Interrupt status bit */
#define SPI0STA_IRQ_BBA                (*(volatile unsigned long *) 0x42080000)
#define SPI0STA_IRQ_MSK                (0x1   << 0  )
#define SPI0STA_IRQ                    (0x1   << 0  )
#define SPI0STA_IRQ_CLR                (0x0   << 0  ) /* CLR                      */
#define SPI0STA_IRQ_SET                (0x1   << 0  ) /* SET                      */

/* Reset Value for SPI0RX*/
#define SPI0RX_RVAL                    0x0 

/* SPI0RX[VALUE] - Received data */
#define SPI0RX_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for SPI0TX*/
#define SPI0TX_RVAL                    0x0 

/* SPI0TX[VALUE] - Data to transmit */
#define SPI0TX_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for SPI0DIV*/
#define SPI0DIV_RVAL                   0x0 

/* SPI0DIV[CSIRQ_EN] - Enable interrupt on every CS edge in CONT mode */
#define SPI0DIV_CSIRQ_EN_BBA           (*(volatile unsigned long *) 0x420801A0)
#define SPI0DIV_CSIRQ_EN_MSK           (0x1   << 8  )
#define SPI0DIV_CSIRQ_EN               (0x1   << 8  )
#define SPI0DIV_CSIRQ_EN_DIS           (0x0   << 8  )
#define SPI0DIV_CSIRQ_EN_EN            (0x1   << 8  )

/* SPI0DIV[BCRST] - Bit counter reset */
#define SPI0DIV_BCRST_BBA              (*(volatile unsigned long *) 0x4208019C)
#define SPI0DIV_BCRST_MSK              (0x1   << 7  )
#define SPI0DIV_BCRST                  (0x1   << 7  )
#define SPI0DIV_BCRST_DIS              (0x0   << 7  ) /* DIS                      */
#define SPI0DIV_BCRST_EN               (0x1   << 7  ) /* EN                       */

/* SPI0DIV[HFM] - High Frequency Mode */
#define SPI0DIV_HFM_BBA                (*(volatile unsigned long *) 0x42080198)
#define SPI0DIV_HFM_MSK                (0x1   << 6  )
#define SPI0DIV_HFM                    (0x1   << 6  )
#define SPI0DIV_HFM_DIS                (0x0   << 6  )
#define SPI0DIV_HFM_EN                 (0x1   << 6  )

/* SPI0DIV[DIV] - Factor used to divide UCLK to generate the serial clock */
#define SPI0DIV_DIV_MSK                (0x3F  << 0  )

/* Reset Value for SPI0CON*/
#define SPI0CON_RVAL                   0x0 

/* SPI0CON[MOD] - SPI IRQ Mode bits */
#define SPI0CON_MOD_MSK                (0x3   << 14 )
#define SPI0CON_MOD_TX1RX1             (0x0   << 14 ) /* TX1RX1                   */
#define SPI0CON_MOD_TX2RX2             (0x1   << 14 ) /* TX2RX2                   */
#define SPI0CON_MOD_TX3RX3             (0x2   << 14 ) /* TX3RX3                   */
#define SPI0CON_MOD_TX4RX4             (0x3   << 14 ) /* TX4RX4                   */

/* SPI0CON[TFLUSH] - TX FIFO Flush Enable bit */
#define SPI0CON_TFLUSH_BBA             (*(volatile unsigned long *) 0x42080234)
#define SPI0CON_TFLUSH_MSK             (0x1   << 13 )
#define SPI0CON_TFLUSH                 (0x1   << 13 )
#define SPI0CON_TFLUSH_DIS             (0x0   << 13 ) /* DIS                      */
#define SPI0CON_TFLUSH_EN              (0x1   << 13 ) /* EN                       */

/* SPI0CON[RFLUSH] - RX FIFO Flush Enable bit */
#define SPI0CON_RFLUSH_BBA             (*(volatile unsigned long *) 0x42080230)
#define SPI0CON_RFLUSH_MSK             (0x1   << 12 )
#define SPI0CON_RFLUSH                 (0x1   << 12 )
#define SPI0CON_RFLUSH_DIS             (0x0   << 12 ) /* DIS                      */
#define SPI0CON_RFLUSH_EN              (0x1   << 12 ) /* EN                       */

/* SPI0CON[CON] - Continuous transfer enable */
#define SPI0CON_CON_BBA                (*(volatile unsigned long *) 0x4208022C)
#define SPI0CON_CON_MSK                (0x1   << 11 )
#define SPI0CON_CON                    (0x1   << 11 )
#define SPI0CON_CON_DIS                (0x0   << 11 ) /* DIS                      */
#define SPI0CON_CON_EN                 (0x1   << 11 ) /* EN                       */

/* SPI0CON[LOOPBACK] - Loopback enable bit */
#define SPI0CON_LOOPBACK_BBA           (*(volatile unsigned long *) 0x42080228)
#define SPI0CON_LOOPBACK_MSK           (0x1   << 10 )
#define SPI0CON_LOOPBACK               (0x1   << 10 )
#define SPI0CON_LOOPBACK_DIS           (0x0   << 10 ) /* DIS                      */
#define SPI0CON_LOOPBACK_EN            (0x1   << 10 ) /* EN                       */

/* SPI0CON[SOEN] - Slave MISO output enable bit */
#define SPI0CON_SOEN_BBA               (*(volatile unsigned long *) 0x42080224)
#define SPI0CON_SOEN_MSK               (0x1   << 9  )
#define SPI0CON_SOEN                   (0x1   << 9  )
#define SPI0CON_SOEN_DIS               (0x0   << 9  ) /* DIS                      */
#define SPI0CON_SOEN_EN                (0x1   << 9  ) /* EN                       */

/* SPI0CON[RXOF] - RX Oveflow Overwrite enable */
#define SPI0CON_RXOF_BBA               (*(volatile unsigned long *) 0x42080220)
#define SPI0CON_RXOF_MSK               (0x1   << 8  )
#define SPI0CON_RXOF                   (0x1   << 8  )
#define SPI0CON_RXOF_DIS               (0x0   << 8  ) /* DIS                      */
#define SPI0CON_RXOF_EN                (0x1   << 8  ) /* EN                       */

/* SPI0CON[ZEN] - Transmit zeros when empty */
#define SPI0CON_ZEN_BBA                (*(volatile unsigned long *) 0x4208021C)
#define SPI0CON_ZEN_MSK                (0x1   << 7  )
#define SPI0CON_ZEN                    (0x1   << 7  )
#define SPI0CON_ZEN_DIS                (0x0   << 7  ) /* DIS                      */
#define SPI0CON_ZEN_EN                 (0x1   << 7  ) /* EN                       */

/* SPI0CON[TIM] - Transfer and interrupt mode */
#define SPI0CON_TIM_BBA                (*(volatile unsigned long *) 0x42080218)
#define SPI0CON_TIM_MSK                (0x1   << 6  )
#define SPI0CON_TIM                    (0x1   << 6  )
#define SPI0CON_TIM_RXRD               (0x0   << 6  ) /* RXRD - Cleared by user to initiate transfer with a read of the SPIRX register */
#define SPI0CON_TIM_TXWR               (0x1   << 6  ) /* TXWR - Set by user to initiate transfer with a write to the SPITX register. */

/* SPI0CON[LSB] - LSB First Transfer enable */
#define SPI0CON_LSB_BBA                (*(volatile unsigned long *) 0x42080214)
#define SPI0CON_LSB_MSK                (0x1   << 5  )
#define SPI0CON_LSB                    (0x1   << 5  )
#define SPI0CON_LSB_DIS                (0x0   << 5  ) /* DIS                      */
#define SPI0CON_LSB_EN                 (0x1   << 5  ) /* EN                       */

/* SPI0CON[WOM] - Wired OR enable */
#define SPI0CON_WOM_BBA                (*(volatile unsigned long *) 0x42080210)
#define SPI0CON_WOM_MSK                (0x1   << 4  )
#define SPI0CON_WOM                    (0x1   << 4  )
#define SPI0CON_WOM_DIS                (0x0   << 4  ) /* DIS                      */
#define SPI0CON_WOM_EN                 (0x1   << 4  ) /* EN                       */

/* SPI0CON[CPOL] - Clock polarity mode */
#define SPI0CON_CPOL_BBA               (*(volatile unsigned long *) 0x4208020C)
#define SPI0CON_CPOL_MSK               (0x1   << 3  )
#define SPI0CON_CPOL                   (0x1   << 3  )
#define SPI0CON_CPOL_LOW               (0x0   << 3  ) /* LOW                      */
#define SPI0CON_CPOL_HIGH              (0x1   << 3  ) /* HIGH                     */

/* SPI0CON[CPHA] - Clock phase mode */
#define SPI0CON_CPHA_BBA               (*(volatile unsigned long *) 0x42080208)
#define SPI0CON_CPHA_MSK               (0x1   << 2  )
#define SPI0CON_CPHA                   (0x1   << 2  )
#define SPI0CON_CPHA_SAMPLELEADING     (0x0   << 2  ) /* SAMPLELEADING            */
#define SPI0CON_CPHA_SAMPLETRAILING    (0x1   << 2  ) /* SAMPLETRAILING           */

/* SPI0CON[MASEN] - Master enable */
#define SPI0CON_MASEN_BBA              (*(volatile unsigned long *) 0x42080204)
#define SPI0CON_MASEN_MSK              (0x1   << 1  )
#define SPI0CON_MASEN                  (0x1   << 1  )
#define SPI0CON_MASEN_DIS              (0x0   << 1  ) /* DIS                      */
#define SPI0CON_MASEN_EN               (0x1   << 1  ) /* EN                       */

/* SPI0CON[ENABLE] - SPI Enable bit */
#define SPI0CON_ENABLE_BBA             (*(volatile unsigned long *) 0x42080200)
#define SPI0CON_ENABLE_MSK             (0x1   << 0  )
#define SPI0CON_ENABLE                 (0x1   << 0  )
#define SPI0CON_ENABLE_DIS             (0x0   << 0  ) /* DIS                      */
#define SPI0CON_ENABLE_EN              (0x1   << 0  ) /* EN                       */

/* Reset Value for SPI0DMA*/
#define SPI0DMA_RVAL                   0x0 

/* SPI0DMA[IENRXDMA] - Enable receive DMA request */
#define SPI0DMA_IENRXDMA_BBA           (*(volatile unsigned long *) 0x42080288)
#define SPI0DMA_IENRXDMA_MSK           (0x1   << 2  )
#define SPI0DMA_IENRXDMA               (0x1   << 2  )
#define SPI0DMA_IENRXDMA_DIS           (0x0   << 2  ) /* DIS                      */
#define SPI0DMA_IENRXDMA_EN            (0x1   << 2  ) /* EN                       */

/* SPI0DMA[IENTXDMA] - Enable transmit DMA request */
#define SPI0DMA_IENTXDMA_BBA           (*(volatile unsigned long *) 0x42080284)
#define SPI0DMA_IENTXDMA_MSK           (0x1   << 1  )
#define SPI0DMA_IENTXDMA               (0x1   << 1  )
#define SPI0DMA_IENTXDMA_DIS           (0x0   << 1  ) /* DIS                      */
#define SPI0DMA_IENTXDMA_EN            (0x1   << 1  ) /* EN                       */

/* SPI0DMA[ENABLE] - Enable DMA for data transfer */
#define SPI0DMA_ENABLE_BBA             (*(volatile unsigned long *) 0x42080280)
#define SPI0DMA_ENABLE_MSK             (0x1   << 0  )
#define SPI0DMA_ENABLE                 (0x1   << 0  )
#define SPI0DMA_ENABLE_DIS             (0x0   << 0  ) /* DIS                      */
#define SPI0DMA_ENABLE_EN              (0x1   << 0  ) /* EN                       */

/* Reset Value for SPI0CNT*/
#define SPI0CNT_RVAL                   0x0 

/* SPI0CNT[VALUE] - Count */
#define SPI0CNT_VALUE_MSK              (0xFF  << 0  )

#define          SPI1STA                                    (*(volatile unsigned short int *) 0x40004400)
#define          SPI1RX                                     (*(volatile unsigned char      *) 0x40004404)
#define          SPI1TX                                     (*(volatile unsigned char      *) 0x40004408)
#define          SPI1DIV                                    (*(volatile unsigned short int *) 0x4000440C)
#define          SPI1CON                                    (*(volatile unsigned short int *) 0x40004410)
#define          SPI1DMA                                    (*(volatile unsigned short int *) 0x40004414)
#define          SPI1CNT                                    (*(volatile unsigned short int *) 0x40004418)

/* Reset Value for SPI1STA*/
#define SPI1STA_RVAL                   0x0 

/* SPI1STA[CSRSG] - Detected a rising edge on CS, in CONT mode */
#define SPI1STA_CSRSG_BBA              (*(volatile unsigned long *) 0x42000038)
#define SPI1STA_CSRSG_MSK              (0x1   << 14 )
#define SPI1STA_CSRSG                  (0x1   << 14 )
#define SPI1STA_CSRSG_CLR              (0x0   << 14 ) /* Cleared to 0 when the Status register is read */
#define SPI1STA_CSRSG_SET              (0x1   << 14 ) /* Set to 1 when there was a rising edge in CS line, when the device was in master mode, continuous transfer, High Frequency mode and CSIRQ_EN was asserted. */

/* SPI1STA[CSFLG] - Detected a falling edge on CS, in CONT mode */
#define SPI1STA_CSFLG_BBA              (*(volatile unsigned long *) 0x42000034)
#define SPI1STA_CSFLG_MSK              (0x1   << 13 )
#define SPI1STA_CSFLG                  (0x1   << 13 )
#define SPI1STA_CSFLG_CLR              (0x0   << 13 ) /* Cleared to 0 when the Status register is read. */
#define SPI1STA_CSFLG_SET              (0x1   << 13 ) /* Set to 1 when there was a falling edge in CS line, when the device was in master mode, continuous transfer, High Frequency mode and CSIRQ_EN was asserted. */

/* SPI1STA[CSERR] - Detected an abrupt CS deassertion */
#define SPI1STA_CSERR_BBA              (*(volatile unsigned long *) 0x42088030)
#define SPI1STA_CSERR_MSK              (0x1   << 12 )
#define SPI1STA_CSERR                  (0x1   << 12 )
#define SPI1STA_CSERR_CLR              (0x0   << 12 ) /* CLR                      */
#define SPI1STA_CSERR_SET              (0x1   << 12 ) /* SET                      */

/* SPI1STA[RXS] - Set when there are more bytes in the RX FIFO than the TIM bit says */
#define SPI1STA_RXS_BBA                (*(volatile unsigned long *) 0x4208802C)
#define SPI1STA_RXS_MSK                (0x1   << 11 )
#define SPI1STA_RXS                    (0x1   << 11 )
#define SPI1STA_RXS_CLR                (0x0   << 11 ) /* CLR                      */
#define SPI1STA_RXS_SET                (0x1   << 11 ) /* SET                      */

/* SPI1STA[RXFSTA] - Receive FIFO Status */
#define SPI1STA_RXFSTA_MSK             (0x7   << 8  )
#define SPI1STA_RXFSTA_EMPTY           (0x0   << 8  ) /* EMPTY                    */
#define SPI1STA_RXFSTA_ONEBYTE         (0x1   << 8  ) /* ONEBYTE                  */
#define SPI1STA_RXFSTA_TWOBYTES        (0x2   << 8  ) /* TWOBYTES                 */
#define SPI1STA_RXFSTA_THREEBYTES      (0x3   << 8  ) /* THREEBYTES               */
#define SPI1STA_RXFSTA_FOURBYTES       (0x4   << 8  ) /* FOURBYTES                */

/* SPI1STA[RXOF] - Receive FIFO overflow */
#define SPI1STA_RXOF_BBA               (*(volatile unsigned long *) 0x4208801C)
#define SPI1STA_RXOF_MSK               (0x1   << 7  )
#define SPI1STA_RXOF                   (0x1   << 7  )
#define SPI1STA_RXOF_CLR               (0x0   << 7  ) /* CLR                      */
#define SPI1STA_RXOF_SET               (0x1   << 7  ) /* SET                      */

/* SPI1STA[RX] - Set when a receive interrupt occurs */
#define SPI1STA_RX_BBA                 (*(volatile unsigned long *) 0x42088018)
#define SPI1STA_RX_MSK                 (0x1   << 6  )
#define SPI1STA_RX                     (0x1   << 6  )
#define SPI1STA_RX_CLR                 (0x0   << 6  ) /* CLR                      */
#define SPI1STA_RX_SET                 (0x1   << 6  ) /* SET                      */

/* SPI1STA[TX] - Set when a transmit interrupt occurs */
#define SPI1STA_TX_BBA                 (*(volatile unsigned long *) 0x42088014)
#define SPI1STA_TX_MSK                 (0x1   << 5  )
#define SPI1STA_TX                     (0x1   << 5  )
#define SPI1STA_TX_CLR                 (0x0   << 5  ) /* CLR                      */
#define SPI1STA_TX_SET                 (0x1   << 5  ) /* SET                      */

/* SPI1STA[TXUR] - Transmit FIFO underflow */
#define SPI1STA_TXUR_BBA               (*(volatile unsigned long *) 0x42088010)
#define SPI1STA_TXUR_MSK               (0x1   << 4  )
#define SPI1STA_TXUR                   (0x1   << 4  )
#define SPI1STA_TXUR_CLR               (0x0   << 4  ) /* CLR                      */
#define SPI1STA_TXUR_SET               (0x1   << 4  ) /* SET                      */

/* SPI1STA[TXFSTA] - transmit FIFO Status */
#define SPI1STA_TXFSTA_MSK             (0x7   << 1  )
#define SPI1STA_TXFSTA_EMPTY           (0x0   << 1  ) /* EMPTY                    */
#define SPI1STA_TXFSTA_ONEBYTE         (0x1   << 1  ) /* ONEBYTE                  */
#define SPI1STA_TXFSTA_TWOBYTES        (0x2   << 1  ) /* TWOBYTES                 */
#define SPI1STA_TXFSTA_THREEBYTES      (0x3   << 1  ) /* THREEBYTES               */
#define SPI1STA_TXFSTA_FOURBYTES       (0x4   << 1  ) /* FOURBYTES                */

/* SPI1STA[IRQ] - Interrupt status bit */
#define SPI1STA_IRQ_BBA                (*(volatile unsigned long *) 0x42088000)
#define SPI1STA_IRQ_MSK                (0x1   << 0  )
#define SPI1STA_IRQ                    (0x1   << 0  )
#define SPI1STA_IRQ_CLR                (0x0   << 0  ) /* CLR                      */
#define SPI1STA_IRQ_SET                (0x1   << 0  ) /* SET                      */

/* Reset Value for SPI1RX*/
#define SPI1RX_RVAL                    0x0 

/* SPI1RX[VALUE] - Received data */
#define SPI1RX_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for SPI1TX*/
#define SPI1TX_RVAL                    0x0 

/* SPI1TX[VALUE] - Data to transmit */
#define SPI1TX_VALUE_MSK               (0xFF  << 0  )

/* Reset Value for SPI1DIV*/
#define SPI1DIV_RVAL                   0x0 

/* SPI1DIV[CSIRQ_EN] - Enable interrupt on every CS edge in CONT mode */
#define SPI1DIV_CSIRQ_EN_BBA           (*(volatile unsigned long *) 0x420001A0)
#define SPI1DIV_CSIRQ_EN_MSK           (0x1   << 8  )
#define SPI1DIV_CSIRQ_EN               (0x1   << 8  )
#define SPI1DIV_CSIRQ_EN_DIS           (0x0   << 8  )
#define SPI1DIV_CSIRQ_EN_EN            (0x1   << 8  )

/* SPI1DIV[BCRST] - Bit counter reset */
#define SPI1DIV_BCRST_BBA              (*(volatile unsigned long *) 0x4208819C)
#define SPI1DIV_BCRST_MSK              (0x1   << 7  )
#define SPI1DIV_BCRST                  (0x1   << 7  )
#define SPI1DIV_BCRST_DIS              (0x0   << 7  ) /* DIS                      */
#define SPI1DIV_BCRST_EN               (0x1   << 7  ) /* EN                       */

/* SPI1DIV[HFM] - High Frequency Mode */
#define SPI1DIV_HFM_BBA                (*(volatile unsigned long *) 0x42000198)
#define SPI1DIV_HFM_MSK                (0x1   << 6  )
#define SPI1DIV_HFM                    (0x1   << 6  )
#define SPI1DIV_HFM_DIS                (0x0   << 6  )
#define SPI1DIV_HFM_EN                 (0x1   << 6  )

/* SPI1DIV[DIV] - Factor used to divide UCLK to generate the serial clock */
#define SPI1DIV_DIV_MSK                (0x3F  << 0  )

/* Reset Value for SPI1CON*/
#define SPI1CON_RVAL                   0x0 

/* SPI1CON[MOD] - SPI IRQ Mode bits */
#define SPI1CON_MOD_MSK                (0x3   << 14 )
#define SPI1CON_MOD_TX1RX1             (0x0   << 14 ) /* TX1RX1                   */
#define SPI1CON_MOD_TX2RX2             (0x1   << 14 ) /* TX2RX2                   */
#define SPI1CON_MOD_TX3RX3             (0x2   << 14 ) /* TX3RX3                   */
#define SPI1CON_MOD_TX4RX4             (0x3   << 14 ) /* TX4RX4                   */

/* SPI1CON[TFLUSH] - TX FIFO Flush Enable bit */
#define SPI1CON_TFLUSH_BBA             (*(volatile unsigned long *) 0x42088234)
#define SPI1CON_TFLUSH_MSK             (0x1   << 13 )
#define SPI1CON_TFLUSH                 (0x1   << 13 )
#define SPI1CON_TFLUSH_DIS             (0x0   << 13 ) /* DIS                      */
#define SPI1CON_TFLUSH_EN              (0x1   << 13 ) /* EN                       */

/* SPI1CON[RFLUSH] - RX FIFO Flush Enable bit */
#define SPI1CON_RFLUSH_BBA             (*(volatile unsigned long *) 0x42088230)
#define SPI1CON_RFLUSH_MSK             (0x1   << 12 )
#define SPI1CON_RFLUSH                 (0x1   << 12 )
#define SPI1CON_RFLUSH_DIS             (0x0   << 12 ) /* DIS                      */
#define SPI1CON_RFLUSH_EN              (0x1   << 12 ) /* EN                       */

/* SPI1CON[CON] - Continuous transfer enable */
#define SPI1CON_CON_BBA                (*(volatile unsigned long *) 0x4208822C)
#define SPI1CON_CON_MSK                (0x1   << 11 )
#define SPI1CON_CON                    (0x1   << 11 )
#define SPI1CON_CON_DIS                (0x0   << 11 ) /* DIS                      */
#define SPI1CON_CON_EN                 (0x1   << 11 ) /* EN                       */

/* SPI1CON[LOOPBACK] - Loopback enable bit */
#define SPI1CON_LOOPBACK_BBA           (*(volatile unsigned long *) 0x42088228)
#define SPI1CON_LOOPBACK_MSK           (0x1   << 10 )
#define SPI1CON_LOOPBACK               (0x1   << 10 )
#define SPI1CON_LOOPBACK_DIS           (0x0   << 10 ) /* DIS                      */
#define SPI1CON_LOOPBACK_EN            (0x1   << 10 ) /* EN                       */

/* SPI1CON[SOEN] - Slave MISO output enable bit */
#define SPI1CON_SOEN_BBA               (*(volatile unsigned long *) 0x42088224)
#define SPI1CON_SOEN_MSK               (0x1   << 9  )
#define SPI1CON_SOEN                   (0x1   << 9  )
#define SPI1CON_SOEN_DIS               (0x0   << 9  ) /* DIS                      */
#define SPI1CON_SOEN_EN                (0x1   << 9  ) /* EN                       */

/* SPI1CON[RXOF] - RX Oveflow Overwrite enable */
#define SPI1CON_RXOF_BBA               (*(volatile unsigned long *) 0x42088220)
#define SPI1CON_RXOF_MSK               (0x1   << 8  )
#define SPI1CON_RXOF                   (0x1   << 8  )
#define SPI1CON_RXOF_DIS               (0x0   << 8  ) /* DIS                      */
#define SPI1CON_RXOF_EN                (0x1   << 8  ) /* EN                       */

/* SPI1CON[ZEN] - Transmit zeros when empty */
#define SPI1CON_ZEN_BBA                (*(volatile unsigned long *) 0x4208821C)
#define SPI1CON_ZEN_MSK                (0x1   << 7  )
#define SPI1CON_ZEN                    (0x1   << 7  )
#define SPI1CON_ZEN_DIS                (0x0   << 7  ) /* DIS                      */
#define SPI1CON_ZEN_EN                 (0x1   << 7  ) /* EN                       */

/* SPI1CON[TIM] - Transfer and interrupt mode */
#define SPI1CON_TIM_BBA                (*(volatile unsigned long *) 0x42088218)
#define SPI1CON_TIM_MSK                (0x1   << 6  )
#define SPI1CON_TIM                    (0x1   << 6  )
#define SPI1CON_TIM_RXRD               (0x0   << 6  ) /* RXRD - Cleared by user to initiate transfer with a read of the SPIRX register */
#define SPI1CON_TIM_TXWR               (0x1   << 6  ) /* TXWR - Set by user to initiate transfer with a write to the SPITX register. */

/* SPI1CON[LSB] - LSB First Transfer enable */
#define SPI1CON_LSB_BBA                (*(volatile unsigned long *) 0x42088214)
#define SPI1CON_LSB_MSK                (0x1   << 5  )
#define SPI1CON_LSB                    (0x1   << 5  )
#define SPI1CON_LSB_DIS                (0x0   << 5  ) /* DIS                      */
#define SPI1CON_LSB_EN                 (0x1   << 5  ) /* EN                       */

/* SPI1CON[WOM] - Wired OR enable */
#define SPI1CON_WOM_BBA                (*(volatile unsigned long *) 0x42088210)
#define SPI1CON_WOM_MSK                (0x1   << 4  )
#define SPI1CON_WOM                    (0x1   << 4  )
#define SPI1CON_WOM_DIS                (0x0   << 4  ) /* DIS                      */
#define SPI1CON_WOM_EN                 (0x1   << 4  ) /* EN                       */

/* SPI1CON[CPOL] - Clock polarity mode */
#define SPI1CON_CPOL_BBA               (*(volatile unsigned long *) 0x4208820C)
#define SPI1CON_CPOL_MSK               (0x1   << 3  )
#define SPI1CON_CPOL                   (0x1   << 3  )
#define SPI1CON_CPOL_LOW               (0x0   << 3  ) /* LOW                      */
#define SPI1CON_CPOL_HIGH              (0x1   << 3  ) /* HIGH                     */

/* SPI1CON[CPHA] - Clock phase mode */
#define SPI1CON_CPHA_BBA               (*(volatile unsigned long *) 0x42088208)
#define SPI1CON_CPHA_MSK               (0x1   << 2  )
#define SPI1CON_CPHA                   (0x1   << 2  )
#define SPI1CON_CPHA_SAMPLELEADING     (0x0   << 2  ) /* SAMPLELEADING            */
#define SPI1CON_CPHA_SAMPLETRAILING    (0x1   << 2  ) /* SAMPLETRAILING           */

/* SPI1CON[MASEN] - Master enable */
#define SPI1CON_MASEN_BBA              (*(volatile unsigned long *) 0x42088204)
#define SPI1CON_MASEN_MSK              (0x1   << 1  )
#define SPI1CON_MASEN                  (0x1   << 1  )
#define SPI1CON_MASEN_DIS              (0x0   << 1  ) /* DIS                      */
#define SPI1CON_MASEN_EN               (0x1   << 1  ) /* EN                       */

/* SPI1CON[ENABLE] - SPI Enable bit */
#define SPI1CON_ENABLE_BBA             (*(volatile unsigned long *) 0x42088200)
#define SPI1CON_ENABLE_MSK             (0x1   << 0  )
#define SPI1CON_ENABLE                 (0x1   << 0  )
#define SPI1CON_ENABLE_DIS             (0x0   << 0  ) /* DIS                      */
#define SPI1CON_ENABLE_EN              (0x1   << 0  ) /* EN                       */

/* Reset Value for SPI1DMA*/
#define SPI1DMA_RVAL                   0x0 

/* SPI1DMA[IENRXDMA] - Enable receive DMA request */
#define SPI1DMA_IENRXDMA_BBA           (*(volatile unsigned long *) 0x42088288)
#define SPI1DMA_IENRXDMA_MSK           (0x1   << 2  )
#define SPI1DMA_IENRXDMA               (0x1   << 2  )
#define SPI1DMA_IENRXDMA_DIS           (0x0   << 2  ) /* DIS                      */
#define SPI1DMA_IENRXDMA_EN            (0x1   << 2  ) /* EN                       */

/* SPI1DMA[IENTXDMA] - Enable transmit DMA request */
#define SPI1DMA_IENTXDMA_BBA           (*(volatile unsigned long *) 0x42088284)
#define SPI1DMA_IENTXDMA_MSK           (0x1   << 1  )
#define SPI1DMA_IENTXDMA               (0x1   << 1  )
#define SPI1DMA_IENTXDMA_DIS           (0x0   << 1  ) /* DIS                      */
#define SPI1DMA_IENTXDMA_EN            (0x1   << 1  ) /* EN                       */

/* SPI1DMA[ENABLE] - Enable DMA for data transfer */
#define SPI1DMA_ENABLE_BBA             (*(volatile unsigned long *) 0x42088280)
#define SPI1DMA_ENABLE_MSK             (0x1   << 0  )
#define SPI1DMA_ENABLE                 (0x1   << 0  )
#define SPI1DMA_ENABLE_DIS             (0x0   << 0  ) /* DIS                      */
#define SPI1DMA_ENABLE_EN              (0x1   << 0  ) /* EN                       */

/* Reset Value for SPI1CNT*/
#define SPI1CNT_RVAL                   0x0 

/* SPI1CNT[VALUE] - Count */
#define SPI1CNT_VALUE_MSK              (0xFF  << 0  )
// ------------------------------------------------------------------------------------------------
// -----                                        UART                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief UART (pADI_UART)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_UART Structure                    */
  
  union {
    __IO uint8_t   COMTX;                   /*!< Transmit Holding register             */
    __IO uint8_t   COMRX;                   /*!< Receive Buffer register               */
  } ;
  __I  uint8_t   RESERVED0[3];
  __IO uint8_t   COMIEN;                    /*!< Interrupt Enable register             */
  __I  uint8_t   RESERVED1[3];
  __IO uint8_t   COMIIR;                    /*!< Interrupt Identification register     */
  __I  uint8_t   RESERVED2[3];
  __IO uint8_t   COMLCR;                    /*!< Line Control register                 */
  __I  uint8_t   RESERVED3[3];
  __IO uint8_t   COMMCR;                    /*!< Module Control register               */
  __I  uint8_t   RESERVED4[3];
  __IO uint8_t   COMLSR;                    /*!< Line Status register                  */
  __I  uint8_t   RESERVED5[3];
  __IO uint8_t   COMMSR;                    /*!< Modem Status register                 */
  __I  uint8_t   RESERVED6[11];
  __IO uint16_t  COMFBR;                    /*!< Fractional baud rate divider register. */
  __I  uint16_t  RESERVED7;
  __IO uint16_t  COMDIV;                    /*!< Baud rate Divisor register            */
  __I  uint16_t  RESERVED8[3];
  __IO uint8_t   COMCON;                    /*!< UART control register                 */
} ADI_UART_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          COM0TX                                      (*(volatile unsigned char      *) 0x40005000)
#define          COM0RX                                      (*(volatile unsigned char      *) 0x40005000)
#define          COM0IEN                                     (*(volatile unsigned char      *) 0x40005004)
#define          COM0IIR                                     (*(volatile unsigned char      *) 0x40005008)
#define          COM0LCR                                     (*(volatile unsigned char      *) 0x4000500C)
#define          COM0MCR                                     (*(volatile unsigned char      *) 0x40005010)
#define          COM0LSR                                     (*(volatile unsigned char      *) 0x40005014)
#define          COM0MSR                                     (*(volatile unsigned char      *) 0x40005018)
#define          COM0FBR                                     (*(volatile unsigned short int *) 0x40005024)
#define          COM0DIV                                     (*(volatile unsigned short int *) 0x40005028)
#define          COM0CON                                     (*(volatile unsigned char      *) 0x40005030)
#define          COM1TX                                      (*(volatile unsigned char      *) 0x40005400)
#define          COM1RX                                      (*(volatile unsigned char      *) 0x40005400)
#define          COM1IEN                                     (*(volatile unsigned char      *) 0x40005404)
#define          COM1IIR                                     (*(volatile unsigned char      *) 0x40005408)
#define          COM1LCR                                     (*(volatile unsigned char      *) 0x4000540C)
#define          COM1MCR                                     (*(volatile unsigned char      *) 0x40005410)
#define          COM1LSR                                     (*(volatile unsigned char      *) 0x40005414)
#define          COM1MSR                                     (*(volatile unsigned char      *) 0x40005418)
#define          COM1FBR                                     (*(volatile unsigned short int *) 0x40005424)
#define          COM1DIV                                     (*(volatile unsigned short int *) 0x40005428)
#define          COM1CON                                     (*(volatile unsigned char      *) 0x40005430)
#define          COM2TX                                      (*(volatile unsigned char      *) 0x40005800)
#define          COM2RX                                      (*(volatile unsigned char      *) 0x40005800)
#define          COM2IEN                                     (*(volatile unsigned char      *) 0x40005804)
#define          COM2IIR                                     (*(volatile unsigned char      *) 0x40005808)
#define          COM2LCR                                     (*(volatile unsigned char      *) 0x4000580C)
#define          COM2MCR                                     (*(volatile unsigned char      *) 0x40005810)
#define          COM2LSR                                     (*(volatile unsigned char      *) 0x40005814)
#define          COM2MSR                                     (*(volatile unsigned char      *) 0x40005818)
#define          COM2FBR                                     (*(volatile unsigned short int *) 0x40005824)
#define          COM2DIV                                     (*(volatile unsigned short int *) 0x40005828)
#define          COM2CON                                     (*(volatile unsigned char      *) 0x40005830)

#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for COMTX*/
#define COMTX_RVAL                     0x0 

/* COMTX[VALUE] - Value */
#define COMTX_VALUE_MSK                (0xFF  << 0  )

/* Reset Value for COMRX*/
#define COMRX_RVAL                     0x0 

/* COMRX[VALUE] - Value */
#define COMRX_VALUE_MSK                (0xFF  << 0  )

/* Reset Value for COMIEN*/
#define COMIEN_RVAL                    0x0 

/* COMIEN[EDMAR] - Enable DMA requests in transmit mode */
#define COMIEN_EDMAR_BBA               (*(volatile unsigned long *) 0x420A0094)
#define COMIEN_EDMAR_MSK               (0x1   << 5  )
#define COMIEN_EDMAR                   (0x1   << 5  )
#define COMIEN_EDMAR_DIS               (0x0   << 5  ) /* DIS                      */
#define COMIEN_EDMAR_EN                (0x1   << 5  ) /* EN                       */

/* COMIEN[EDMAT] - Enable DMA requests in receive mode */
#define COMIEN_EDMAT_BBA               (*(volatile unsigned long *) 0x420A0090)
#define COMIEN_EDMAT_MSK               (0x1   << 4  )
#define COMIEN_EDMAT                   (0x1   << 4  )
#define COMIEN_EDMAT_DIS               (0x0   << 4  ) /* DIS                      */
#define COMIEN_EDMAT_EN                (0x1   << 4  ) /* EN                       */

/* COMIEN[EDSSI] - Enable Modem Status interrupt */
#define COMIEN_EDSSI_BBA               (*(volatile unsigned long *) 0x420A008C)
#define COMIEN_EDSSI_MSK               (0x1   << 3  )
#define COMIEN_EDSSI                   (0x1   << 3  )
#define COMIEN_EDSSI_DIS               (0x0   << 3  ) /* DIS                      */
#define COMIEN_EDSSI_EN                (0x1   << 3  ) /* EN                       */

/* COMIEN[ELSI] - Enable Rx status interrupt */
#define COMIEN_ELSI_BBA                (*(volatile unsigned long *) 0x420A0088)
#define COMIEN_ELSI_MSK                (0x1   << 2  )
#define COMIEN_ELSI                    (0x1   << 2  )
#define COMIEN_ELSI_DIS                (0x0   << 2  ) /* DIS                      */
#define COMIEN_ELSI_EN                 (0x1   << 2  ) /* EN                       */

/* COMIEN[ETBEI] - Enable transmit buffer empty interrupt */
#define COMIEN_ETBEI_BBA               (*(volatile unsigned long *) 0x420A0084)
#define COMIEN_ETBEI_MSK               (0x1   << 1  )
#define COMIEN_ETBEI                   (0x1   << 1  )
#define COMIEN_ETBEI_DIS               (0x0   << 1  ) /* DIS                      */
#define COMIEN_ETBEI_EN                (0x1   << 1  ) /* EN                       */

/* COMIEN[ERBFI] - Enable receive buffer full interrupt */
#define COMIEN_ERBFI_BBA               (*(volatile unsigned long *) 0x420A0080)
#define COMIEN_ERBFI_MSK               (0x1   << 0  )
#define COMIEN_ERBFI                   (0x1   << 0  )
#define COMIEN_ERBFI_DIS               (0x0   << 0  ) /* DIS                      */
#define COMIEN_ERBFI_EN                (0x1   << 0  ) /* EN                       */

/* Reset Value for COMIIR*/
#define COMIIR_RVAL                    0x1 

/* COMIIR[STA] - Status bits. */
#define COMIIR_STA_MSK                 (0x3   << 1  )
#define COMIIR_STA_MODEMSTATUS         (0x0   << 1  ) /* MODEMSTATUS - Modem status interrupt. */
#define COMIIR_STA_TXBUFEMPTY          (0x1   << 1  ) /* TXBUFEMPTY - Transmit buffer empty interrupt. */
#define COMIIR_STA_RXBUFFULL           (0x2   << 1  ) /* RXBUFFULL - Receive buffer full interrupt. Read RBR register to clear. */
#define COMIIR_STA_RXLINESTATUS        (0x3   << 1  ) /* RXLINESTATUS - Receive line status interrupt. Read LSR register to clear. */

/* COMIIR[NINT] - Interrupt flag. */
#define COMIIR_NINT_BBA                (*(volatile unsigned long *) 0x420A0100)
#define COMIIR_NINT_MSK                (0x1   << 0  )
#define COMIIR_NINT                    (0x1   << 0  )
#define COMIIR_NINT_CLR                (0x0   << 0  ) /* CLR                      */
#define COMIIR_NINT_SET                (0x1   << 0  ) /* SET                      */

/* Reset Value for COMLCR*/
#define COMLCR_RVAL                    0x0 

/* COMLCR[BRK] - Set Break. */
#define COMLCR_BRK_BBA                 (*(volatile unsigned long *) 0x420A0198)
#define COMLCR_BRK_MSK                 (0x1   << 6  )
#define COMLCR_BRK                     (0x1   << 6  )
#define COMLCR_BRK_DIS                 (0x0   << 6  ) /* DIS                      */
#define COMLCR_BRK_EN                  (0x1   << 6  ) /* EN                       */

/* COMLCR[SP] - Stick Parity. */
#define COMLCR_SP_BBA                  (*(volatile unsigned long *) 0x420A0194)
#define COMLCR_SP_MSK                  (0x1   << 5  )
#define COMLCR_SP                      (0x1   << 5  )
#define COMLCR_SP_DIS                  (0x0   << 5  ) /* DIS                      */
#define COMLCR_SP_EN                   (0x1   << 5  ) /* EN                       */

/* COMLCR[EPS] - Even Parity Select Bit. */
#define COMLCR_EPS_BBA                 (*(volatile unsigned long *) 0x420A0190)
#define COMLCR_EPS_MSK                 (0x1   << 4  )
#define COMLCR_EPS                     (0x1   << 4  )
#define COMLCR_EPS_DIS                 (0x0   << 4  ) /* DIS                      */
#define COMLCR_EPS_EN                  (0x1   << 4  ) /* EN                       */

/* COMLCR[PEN] - Parity Enable Bit. */
#define COMLCR_PEN_BBA                 (*(volatile unsigned long *) 0x420A018C)
#define COMLCR_PEN_MSK                 (0x1   << 3  )
#define COMLCR_PEN                     (0x1   << 3  )
#define COMLCR_PEN_DIS                 (0x0   << 3  ) /* DIS                      */
#define COMLCR_PEN_EN                  (0x1   << 3  ) /* EN                       */

/* COMLCR[STOP] - Stop Bit. */
#define COMLCR_STOP_BBA                (*(volatile unsigned long *) 0x420A0188)
#define COMLCR_STOP_MSK                (0x1   << 2  )
#define COMLCR_STOP                    (0x1   << 2  )
#define COMLCR_STOP_DIS                (0x0   << 2  ) /* DIS                      */
#define COMLCR_STOP_EN                 (0x1   << 2  ) /* EN                       */

/* COMLCR[WLS] - Word Length Select bits */
#define COMLCR_WLS_MSK                 (0x3   << 0  )
#define COMLCR_WLS_5BITS               (0x0   << 0  ) /* 5BITS                    */
#define COMLCR_WLS_6BITS               (0x1   << 0  ) /* 6BITS                    */
#define COMLCR_WLS_7BITS               (0x2   << 0  ) /* 7BITS                    */
#define COMLCR_WLS_8BITS               (0x3   << 0  ) /* 8BITS                    */

/* Reset Value for COMMCR*/
#define COMMCR_RVAL                    0x0 

/* COMMCR[LOOPBACK] - Loop Back. */
#define COMMCR_LOOPBACK_BBA            (*(volatile unsigned long *) 0x420A0210)
#define COMMCR_LOOPBACK_MSK            (0x1   << 4  )
#define COMMCR_LOOPBACK                (0x1   << 4  )
#define COMMCR_LOOPBACK_DIS            (0x0   << 4  ) /* DIS                      */
#define COMMCR_LOOPBACK_EN             (0x1   << 4  ) /* EN                       */

/* COMMCR[OUT1] - Parity Enable Bit. */
#define COMMCR_OUT1_BBA                (*(volatile unsigned long *) 0x420A020C)
#define COMMCR_OUT1_MSK                (0x1   << 3  )
#define COMMCR_OUT1                    (0x1   << 3  )
#define COMMCR_OUT1_DIS                (0x0   << 3  ) /* DIS                      */
#define COMMCR_OUT1_EN                 (0x1   << 3  ) /* EN                       */

/* COMMCR[OUT2] - Stop Bit. */
#define COMMCR_OUT2_BBA                (*(volatile unsigned long *) 0x420A0208)
#define COMMCR_OUT2_MSK                (0x1   << 2  )
#define COMMCR_OUT2                    (0x1   << 2  )
#define COMMCR_OUT2_DIS                (0x0   << 2  ) /* DIS                      */
#define COMMCR_OUT2_EN                 (0x1   << 2  ) /* EN                       */

/* COMMCR[RTS] - Request To Send. */
#define COMMCR_RTS_BBA                 (*(volatile unsigned long *) 0x420A0204)
#define COMMCR_RTS_MSK                 (0x1   << 1  )
#define COMMCR_RTS                     (0x1   << 1  )
#define COMMCR_RTS_DIS                 (0x0   << 1  ) /* DIS                      */
#define COMMCR_RTS_EN                  (0x1   << 1  ) /* EN                       */

/* COMMCR[DTR] - Data Terminal Ready. */
#define COMMCR_DTR_BBA                 (*(volatile unsigned long *) 0x420A0200)
#define COMMCR_DTR_MSK                 (0x1   << 0  )
#define COMMCR_DTR                     (0x1   << 0  )
#define COMMCR_DTR_DIS                 (0x0   << 0  ) /* DIS                      */
#define COMMCR_DTR_EN                  (0x1   << 0  ) /* EN                       */

/* Reset Value for COMLSR*/
#define COMLSR_RVAL                    0x60 

/* COMLSR[TEMT] - COMTX and Shift Register Empty Status Bit. */
#define COMLSR_TEMT_BBA                (*(volatile unsigned long *) 0x420A0298)
#define COMLSR_TEMT_MSK                (0x1   << 6  )
#define COMLSR_TEMT                    (0x1   << 6  )
#define COMLSR_TEMT_CLR                (0x0   << 6  ) /* CLR                      */
#define COMLSR_TEMT_SET                (0x1   << 6  ) /* SET                      */

/* COMLSR[THRE] - COMTX Empty Status Bit. */
#define COMLSR_THRE_BBA                (*(volatile unsigned long *) 0x420A0294)
#define COMLSR_THRE_MSK                (0x1   << 5  )
#define COMLSR_THRE                    (0x1   << 5  )
#define COMLSR_THRE_CLR                (0x0   << 5  ) /* CLR                      */
#define COMLSR_THRE_SET                (0x1   << 5  ) /* SET                      */

/* COMLSR[BI] - Break Indicator. */
#define COMLSR_BI_BBA                  (*(volatile unsigned long *) 0x420A0290)
#define COMLSR_BI_MSK                  (0x1   << 4  )
#define COMLSR_BI                      (0x1   << 4  )
#define COMLSR_BI_CLR                  (0x0   << 4  ) /* CLR                      */
#define COMLSR_BI_SET                  (0x1   << 4  ) /* SET                      */

/* COMLSR[FE] - Framing Error. */
#define COMLSR_FE_BBA                  (*(volatile unsigned long *) 0x420A028C)
#define COMLSR_FE_MSK                  (0x1   << 3  )
#define COMLSR_FE                      (0x1   << 3  )
#define COMLSR_FE_CLR                  (0x0   << 3  ) /* CLR                      */
#define COMLSR_FE_SET                  (0x1   << 3  ) /* SET                      */

/* COMLSR[PE] - Parity Error. */
#define COMLSR_PE_BBA                  (*(volatile unsigned long *) 0x420A0288)
#define COMLSR_PE_MSK                  (0x1   << 2  )
#define COMLSR_PE                      (0x1   << 2  )
#define COMLSR_PE_CLR                  (0x0   << 2  ) /* CLR                      */
#define COMLSR_PE_SET                  (0x1   << 2  ) /* SET                      */

/* COMLSR[OE] - Overrun Error. */
#define COMLSR_OE_BBA                  (*(volatile unsigned long *) 0x420A0284)
#define COMLSR_OE_MSK                  (0x1   << 1  )
#define COMLSR_OE                      (0x1   << 1  )
#define COMLSR_OE_CLR                  (0x0   << 1  ) /* CLR                      */
#define COMLSR_OE_SET                  (0x1   << 1  ) /* SET                      */

/* COMLSR[DR] - Data Ready. */
#define COMLSR_DR_BBA                  (*(volatile unsigned long *) 0x420A0280)
#define COMLSR_DR_MSK                  (0x1   << 0  )
#define COMLSR_DR                      (0x1   << 0  )
#define COMLSR_DR_CLR                  (0x0   << 0  ) /* CLR                      */
#define COMLSR_DR_SET                  (0x1   << 0  ) /* SET                      */

/* Reset Value for COMMSR*/
#define COMMSR_RVAL                    0x0 

/* COMMSR[DCD] - Data Carrier Detect. */
#define COMMSR_DCD_BBA                 (*(volatile unsigned long *) 0x420A031C)
#define COMMSR_DCD_MSK                 (0x1   << 7  )
#define COMMSR_DCD                     (0x1   << 7  )
#define COMMSR_DCD_DIS                 (0x0   << 7  ) /* DIS                      */
#define COMMSR_DCD_EN                  (0x1   << 7  ) /* EN                       */

/* COMMSR[RI] - Ring Indicator. */
#define COMMSR_RI_BBA                  (*(volatile unsigned long *) 0x420A0318)
#define COMMSR_RI_MSK                  (0x1   << 6  )
#define COMMSR_RI                      (0x1   << 6  )
#define COMMSR_RI_DIS                  (0x0   << 6  ) /* DIS                      */
#define COMMSR_RI_EN                   (0x1   << 6  ) /* EN                       */

/* COMMSR[DSR] - Data Set Ready. */
#define COMMSR_DSR_BBA                 (*(volatile unsigned long *) 0x420A0314)
#define COMMSR_DSR_MSK                 (0x1   << 5  )
#define COMMSR_DSR                     (0x1   << 5  )
#define COMMSR_DSR_DIS                 (0x0   << 5  ) /* DIS                      */
#define COMMSR_DSR_EN                  (0x1   << 5  ) /* EN                       */

/* COMMSR[CTS] - Clear To Send. */
#define COMMSR_CTS_BBA                 (*(volatile unsigned long *) 0x420A0310)
#define COMMSR_CTS_MSK                 (0x1   << 4  )
#define COMMSR_CTS                     (0x1   << 4  )
#define COMMSR_CTS_DIS                 (0x0   << 4  ) /* DIS                      */
#define COMMSR_CTS_EN                  (0x1   << 4  ) /* EN                       */

/* COMMSR[DDCD] - Delta DCD. */
#define COMMSR_DDCD_BBA                (*(volatile unsigned long *) 0x420A030C)
#define COMMSR_DDCD_MSK                (0x1   << 3  )
#define COMMSR_DDCD                    (0x1   << 3  )
#define COMMSR_DDCD_DIS                (0x0   << 3  ) /* DIS                      */
#define COMMSR_DDCD_EN                 (0x1   << 3  ) /* EN                       */

/* COMMSR[TERI] - Trailing Edge RI. */
#define COMMSR_TERI_BBA                (*(volatile unsigned long *) 0x420A0308)
#define COMMSR_TERI_MSK                (0x1   << 2  )
#define COMMSR_TERI                    (0x1   << 2  )
#define COMMSR_TERI_DIS                (0x0   << 2  ) /* DIS                      */
#define COMMSR_TERI_EN                 (0x1   << 2  ) /* EN                       */

/* COMMSR[DDSR] - Delta DSR. */
#define COMMSR_DDSR_BBA                (*(volatile unsigned long *) 0x420A0304)
#define COMMSR_DDSR_MSK                (0x1   << 1  )
#define COMMSR_DDSR                    (0x1   << 1  )
#define COMMSR_DDSR_DIS                (0x0   << 1  ) /* DIS                      */
#define COMMSR_DDSR_EN                 (0x1   << 1  ) /* EN                       */

/* COMMSR[DCTS] - Delta CTS. */
#define COMMSR_DCTS_BBA                (*(volatile unsigned long *) 0x420A0300)
#define COMMSR_DCTS_MSK                (0x1   << 0  )
#define COMMSR_DCTS                    (0x1   << 0  )
#define COMMSR_DCTS_DIS                (0x0   << 0  ) /* DIS                      */
#define COMMSR_DCTS_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for COMFBR*/
#define COMFBR_RVAL                    0x0 

/* COMFBR[ENABLE] - Enable */
#define COMFBR_ENABLE_BBA              (*(volatile unsigned long *) 0x420A04BC)
#define COMFBR_ENABLE_MSK              (0x1   << 15 )
#define COMFBR_ENABLE                  (0x1   << 15 )
#define COMFBR_ENABLE_DIS              (0x0   << 15 ) /* DIS                      */
#define COMFBR_ENABLE_EN               (0x1   << 15 ) /* EN                       */

/* COMFBR[DIVM] - Fractional M Divide bits */
#define COMFBR_DIVM_MSK                (0x3   << 11 )

/* COMFBR[DIVN] - Fractional N Divide bits */
#define COMFBR_DIVN_MSK                (0x7FF << 0  )

/* Reset Value for COMDIV*/
#define COMDIV_RVAL                    0x1 

/* COMDIV[VALUE] - Sets the baudrate */
#define COMDIV_VALUE_MSK               (0xFFFF << 0  )

/* Reset Value for COMCON*/
#define COMCON_RVAL                    0x0 

/* COMCON[DISABLE] - Uart Disable */
#define COMCON_DISABLE_BBA             (*(volatile unsigned long *) 0x420A0600)
#define COMCON_DISABLE_MSK             (0x1   << 0  )
#define COMCON_DISABLE                 (0x1   << 0  )
#define COMCON_DISABLE_DIS             (0x0   << 0  ) /* DIS                      */
#define COMCON_DISABLE_EN              (0x1   << 0  ) /* EN                       */
// ------------------------------------------------------------------------------------------------
// -----                                        GPIO0                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief General Purpose Input Output (pADI_GP0)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_GP0 Structure                     */
  __IO uint16_t  GPCON;                     /*!< GPIO Port 0 configuration             */
  __I  uint16_t  RESERVED0;
  __IO uint8_t   GPOEN;                     /*!< GPIO Port 0 output enable             */
  __I  uint8_t   RESERVED1[3];
  __IO uint8_t   GPPUL;                     /*!< GPIO Port 0 output pull up enable.    */
  __I  uint8_t   RESERVED2[3];
  __IO uint8_t   GPOCE;                     /*!< GPIO Port 0 tri state                 */
  __I  uint8_t   RESERVED3[7];
  __IO uint8_t   GPIN;                      /*!< GPIO Port 0 data input.               */
  __I  uint8_t   RESERVED4[3];
  __IO uint8_t   GPOUT;                     /*!< GPIO Port 0 data out.                 */
  __I  uint8_t   RESERVED5[3];
  __IO uint8_t   GPSET;                     /*!< GPIO Port 0 data out set              */
  __I  uint8_t   RESERVED6[3];
  __IO uint8_t   GPCLR;                     /*!< GPIO Port 0 data out clear.           */
  __I  uint8_t   RESERVED7[3];
  __IO uint8_t   GPTGL;                     /*!< GPIO Port 0 pin toggle.               */
} ADI_GPIO_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          GP0CON                                     (*(volatile unsigned short int *) 0x40006000)
#define          GP0OEN                                     (*(volatile unsigned char      *) 0x40006004)
#define          GP0PUL                                     (*(volatile unsigned char      *) 0x40006008)
#define          GP0OCE                                     (*(volatile unsigned char      *) 0x4000600C)
#define          GP0IN                                      (*(volatile unsigned char      *) 0x40006014)
#define          GP0OUT                                     (*(volatile unsigned char      *) 0x40006018)
#define          GP0SET                                     (*(volatile unsigned char      *) 0x4000601C)
#define          GP0CLR                                     (*(volatile unsigned char      *) 0x40006020)
#define          GP0TGL                                     (*(volatile unsigned char      *) 0x40006024)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for GP0CON*/
#define GP0CON_RVAL                    0x0 

/* GP0CON[CON7] - Configuration bits for P0.7 */
#define GP0CON_CON7_MSK                (0x3   << 14 )
#define GP0CON_CON7_PORB               (0x0   << 14 ) /* PORB                     */
#define GP0CON_CON7_GPIO               (0x1   << 14 ) /* GPIO                     */
#define GP0CON_CON7_UARTTXD            (0x2   << 14 ) /* UARTTXD                  */

/* GP0CON[CON6] - Configuration bits for P0.6 */
#define GP0CON_CON6_MSK                (0x3   << 12 )
#define GP0CON_CON6_GPIOIRQ2           (0x0   << 12 ) /* GPIOIRQ2                 */
#define GP0CON_CON6_UARTRXD            (0x1   << 12 ) /* UARTRXD                  */

/* GP0CON[CON5] - Configuration bits for P0.5 */
#define GP0CON_CON5_MSK                (0x3   << 10 )
#define GP0CON_CON5_GPIOIRQ1           (0x0   << 10 ) /* GPIOIRQ1                 */
#define GP0CON_CON5_UARTCTS            (0x1   << 10 ) /* UARTCTS                  */

/* GP0CON[CON4] - Configuration bits for P0.4 */
#define GP0CON_CON4_MSK                (0x3   << 8  )
#define GP0CON_CON4_GPIO               (0x0   << 8  ) /* GPIO                     */
#define GP0CON_CON4_UARTRTS            (0x1   << 8  ) /* UARTRTS                  */
#define GP0CON_CON4_ECLKOUT            (0x2   << 8  ) /* ECLKOUT                  */

/* GP0CON[CON3] - Configuration bits for P0.3 */
#define GP0CON_CON3_MSK                (0x3   << 6  )
#define GP0CON_CON3_GPIOIRQ0           (0x0   << 6  ) /* GPIOIRQ0                 */
#define GP0CON_CON3_SPI1CS0            (0x1   << 6  ) /* SPI1CS0                  */

/* GP0CON[CON2] - Configuration bits for P0.2 */
#define GP0CON_CON2_MSK                (0x3   << 4  )
#define GP0CON_CON2_GPIO               (0x0   << 4  ) /* GPIO                     */
#define GP0CON_CON2_SPI1MOSI           (0x1   << 4  ) /* SPI1MOSI                 */
#define GP0CON_CON2_I2CSDA             (0x2   << 4  ) /* I2CSDA                   */
#define GP0CON_CON2_UARTTXD            (0x3   << 4  ) /* UARTTXD                  */

/* GP0CON[CON1] - Configuration bits for P0.1 */
#define GP0CON_CON1_MSK                (0x3   << 2  )
#define GP0CON_CON1_GPIO               (0x0   << 2  ) /* GPIO                     */
#define GP0CON_CON1_SPI1SCLK           (0x1   << 2  ) /* SPI1SCLK                 */
#define GP0CON_CON1_I2CSCL             (0x2   << 2  ) /* I2CSCL                   */
#define GP0CON_CON1_UARTRXD            (0x3   << 2  ) /* UARTRXD                  */

/* GP0CON[CON0] - Configuration bits for P0.0 */
#define GP0CON_CON0_MSK                (0x3   << 0  )
#define GP0CON_CON0_GPIO               (0x0   << 0  ) /* GPIO                     */
#define GP0CON_CON0_SPI1MISO           (0x1   << 0  ) /* SPI1MISO                 */

/* Reset Value for GP0OEN*/
#define GP0OEN_RVAL                    0x0 

/* GP0OEN[OEN7] - Direction for port pin */
#define GP0OEN_OEN7_BBA                (*(volatile unsigned long *) 0x420C009C)
#define GP0OEN_OEN7_MSK                (0x1   << 7  )
#define GP0OEN_OEN7                    (0x1   << 7  )
#define GP0OEN_OEN7_IN                 (0x0   << 7  ) /* IN                       */
#define GP0OEN_OEN7_OUT                (0x1   << 7  ) /* OUT                      */

/* GP0OEN[OEN6] - Direction for port pin */
#define GP0OEN_OEN6_BBA                (*(volatile unsigned long *) 0x420C0098)
#define GP0OEN_OEN6_MSK                (0x1   << 6  )
#define GP0OEN_OEN6                    (0x1   << 6  )
#define GP0OEN_OEN6_IN                 (0x0   << 6  ) /* IN                       */
#define GP0OEN_OEN6_OUT                (0x1   << 6  ) /* OUT                      */

/* GP0OEN[OEN5] - Direction for port pin */
#define GP0OEN_OEN5_BBA                (*(volatile unsigned long *) 0x420C0094)
#define GP0OEN_OEN5_MSK                (0x1   << 5  )
#define GP0OEN_OEN5                    (0x1   << 5  )
#define GP0OEN_OEN5_IN                 (0x0   << 5  ) /* IN                       */
#define GP0OEN_OEN5_OUT                (0x1   << 5  ) /* OUT                      */

/* GP0OEN[OEN4] - Direction for port pin */
#define GP0OEN_OEN4_BBA                (*(volatile unsigned long *) 0x420C0090)
#define GP0OEN_OEN4_MSK                (0x1   << 4  )
#define GP0OEN_OEN4                    (0x1   << 4  )
#define GP0OEN_OEN4_IN                 (0x0   << 4  ) /* IN                       */
#define GP0OEN_OEN4_OUT                (0x1   << 4  ) /* OUT                      */

/* GP0OEN[OEN3] - Direction for port pin */
#define GP0OEN_OEN3_BBA                (*(volatile unsigned long *) 0x420C008C)
#define GP0OEN_OEN3_MSK                (0x1   << 3  )
#define GP0OEN_OEN3                    (0x1   << 3  )
#define GP0OEN_OEN3_IN                 (0x0   << 3  ) /* IN                       */
#define GP0OEN_OEN3_OUT                (0x1   << 3  ) /* OUT                      */

/* GP0OEN[OEN2] - Direction for port pin */
#define GP0OEN_OEN2_BBA                (*(volatile unsigned long *) 0x420C0088)
#define GP0OEN_OEN2_MSK                (0x1   << 2  )
#define GP0OEN_OEN2                    (0x1   << 2  )
#define GP0OEN_OEN2_IN                 (0x0   << 2  ) /* IN                       */
#define GP0OEN_OEN2_OUT                (0x1   << 2  ) /* OUT                      */

/* GP0OEN[OEN1] - Direction for port pin */
#define GP0OEN_OEN1_BBA                (*(volatile unsigned long *) 0x420C0084)
#define GP0OEN_OEN1_MSK                (0x1   << 1  )
#define GP0OEN_OEN1                    (0x1   << 1  )
#define GP0OEN_OEN1_IN                 (0x0   << 1  ) /* IN                       */
#define GP0OEN_OEN1_OUT                (0x1   << 1  ) /* OUT                      */

/* GP0OEN[OEN0] - Direction for port pin */
#define GP0OEN_OEN0_BBA                (*(volatile unsigned long *) 0x420C0080)
#define GP0OEN_OEN0_MSK                (0x1   << 0  )
#define GP0OEN_OEN0                    (0x1   << 0  )
#define GP0OEN_OEN0_IN                 (0x0   << 0  ) /* IN                       */
#define GP0OEN_OEN0_OUT                (0x1   << 0  ) /* OUT                      */

/* Reset Value for GP0PUL*/
#define GP0PUL_RVAL                    0xFF 

/* GP0PUL[PUL7] - Pull Up Enable for port pin */
#define GP0PUL_PUL7_BBA                (*(volatile unsigned long *) 0x420C011C)
#define GP0PUL_PUL7_MSK                (0x1   << 7  )
#define GP0PUL_PUL7                    (0x1   << 7  )
#define GP0PUL_PUL7_DIS                (0x0   << 7  ) /* DIS                      */
#define GP0PUL_PUL7_EN                 (0x1   << 7  ) /* EN                       */

/* GP0PUL[PUL6] - Pull Up Enable for port pin */
#define GP0PUL_PUL6_BBA                (*(volatile unsigned long *) 0x420C0118)
#define GP0PUL_PUL6_MSK                (0x1   << 6  )
#define GP0PUL_PUL6                    (0x1   << 6  )
#define GP0PUL_PUL6_DIS                (0x0   << 6  ) /* DIS                      */
#define GP0PUL_PUL6_EN                 (0x1   << 6  ) /* EN                       */

/* GP0PUL[PUL5] - Pull Up Enable for port pin */
#define GP0PUL_PUL5_BBA                (*(volatile unsigned long *) 0x420C0114)
#define GP0PUL_PUL5_MSK                (0x1   << 5  )
#define GP0PUL_PUL5                    (0x1   << 5  )
#define GP0PUL_PUL5_DIS                (0x0   << 5  ) /* DIS                      */
#define GP0PUL_PUL5_EN                 (0x1   << 5  ) /* EN                       */

/* GP0PUL[PUL4] - Pull Up Enable for port pin */
#define GP0PUL_PUL4_BBA                (*(volatile unsigned long *) 0x420C0110)
#define GP0PUL_PUL4_MSK                (0x1   << 4  )
#define GP0PUL_PUL4                    (0x1   << 4  )
#define GP0PUL_PUL4_DIS                (0x0   << 4  ) /* DIS                      */
#define GP0PUL_PUL4_EN                 (0x1   << 4  ) /* EN                       */

/* GP0PUL[PUL3] - Pull Up Enable for port pin */
#define GP0PUL_PUL3_BBA                (*(volatile unsigned long *) 0x420C010C)
#define GP0PUL_PUL3_MSK                (0x1   << 3  )
#define GP0PUL_PUL3                    (0x1   << 3  )
#define GP0PUL_PUL3_DIS                (0x0   << 3  ) /* DIS                      */
#define GP0PUL_PUL3_EN                 (0x1   << 3  ) /* EN                       */

/* GP0PUL[PUL2] - Pull Up Enable for port pin */
#define GP0PUL_PUL2_BBA                (*(volatile unsigned long *) 0x420C0108)
#define GP0PUL_PUL2_MSK                (0x1   << 2  )
#define GP0PUL_PUL2                    (0x1   << 2  )
#define GP0PUL_PUL2_DIS                (0x0   << 2  ) /* DIS                      */
#define GP0PUL_PUL2_EN                 (0x1   << 2  ) /* EN                       */

/* GP0PUL[PUL1] - Pull Up Enable for port pin */
#define GP0PUL_PUL1_BBA                (*(volatile unsigned long *) 0x420C0104)
#define GP0PUL_PUL1_MSK                (0x1   << 1  )
#define GP0PUL_PUL1                    (0x1   << 1  )
#define GP0PUL_PUL1_DIS                (0x0   << 1  ) /* DIS                      */
#define GP0PUL_PUL1_EN                 (0x1   << 1  ) /* EN                       */

/* GP0PUL[PUL0] - Pull Up Enable for port pin */
#define GP0PUL_PUL0_BBA                (*(volatile unsigned long *) 0x420C0100)
#define GP0PUL_PUL0_MSK                (0x1   << 0  )
#define GP0PUL_PUL0                    (0x1   << 0  )
#define GP0PUL_PUL0_DIS                (0x0   << 0  ) /* DIS                      */
#define GP0PUL_PUL0_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for GP0OCE*/
#define GP0OCE_RVAL                    0x0 

/* GP0OCE[OCE7] - open circuit Enable for port pin */
#define GP0OCE_OCE7_BBA                (*(volatile unsigned long *) 0x420C019C)
#define GP0OCE_OCE7_MSK                (0x1   << 7  )
#define GP0OCE_OCE7                    (0x1   << 7  )
#define GP0OCE_OCE7_DIS                (0x0   << 7  ) /* DIS                      */
#define GP0OCE_OCE7_EN                 (0x1   << 7  ) /* EN                       */

/* GP0OCE[OCE6] - open circuit Enable for port pin */
#define GP0OCE_OCE6_BBA                (*(volatile unsigned long *) 0x420C0198)
#define GP0OCE_OCE6_MSK                (0x1   << 6  )
#define GP0OCE_OCE6                    (0x1   << 6  )
#define GP0OCE_OCE6_DIS                (0x0   << 6  ) /* DIS                      */
#define GP0OCE_OCE6_EN                 (0x1   << 6  ) /* EN                       */

/* GP0OCE[OCE5] - open circuit Enable for port pin */
#define GP0OCE_OCE5_BBA                (*(volatile unsigned long *) 0x420C0194)
#define GP0OCE_OCE5_MSK                (0x1   << 5  )
#define GP0OCE_OCE5                    (0x1   << 5  )
#define GP0OCE_OCE5_DIS                (0x0   << 5  ) /* DIS                      */
#define GP0OCE_OCE5_EN                 (0x1   << 5  ) /* EN                       */

/* GP0OCE[OCE4] - open circuit Enable for port pin */
#define GP0OCE_OCE4_BBA                (*(volatile unsigned long *) 0x420C0190)
#define GP0OCE_OCE4_MSK                (0x1   << 4  )
#define GP0OCE_OCE4                    (0x1   << 4  )
#define GP0OCE_OCE4_DIS                (0x0   << 4  ) /* DIS                      */
#define GP0OCE_OCE4_EN                 (0x1   << 4  ) /* EN                       */

/* GP0OCE[OCE3] - open circuit Enable for port pin */
#define GP0OCE_OCE3_BBA                (*(volatile unsigned long *) 0x420C018C)
#define GP0OCE_OCE3_MSK                (0x1   << 3  )
#define GP0OCE_OCE3                    (0x1   << 3  )
#define GP0OCE_OCE3_DIS                (0x0   << 3  ) /* DIS                      */
#define GP0OCE_OCE3_EN                 (0x1   << 3  ) /* EN                       */

/* GP0OCE[OCE2] - open circuit Enable for port pin */
#define GP0OCE_OCE2_BBA                (*(volatile unsigned long *) 0x420C0188)
#define GP0OCE_OCE2_MSK                (0x1   << 2  )
#define GP0OCE_OCE2                    (0x1   << 2  )
#define GP0OCE_OCE2_DIS                (0x0   << 2  ) /* DIS                      */
#define GP0OCE_OCE2_EN                 (0x1   << 2  ) /* EN                       */

/* GP0OCE[OCE1] - open circuit Enable for port pin */
#define GP0OCE_OCE1_BBA                (*(volatile unsigned long *) 0x420C0184)
#define GP0OCE_OCE1_MSK                (0x1   << 1  )
#define GP0OCE_OCE1                    (0x1   << 1  )
#define GP0OCE_OCE1_DIS                (0x0   << 1  ) /* DIS                      */
#define GP0OCE_OCE1_EN                 (0x1   << 1  ) /* EN                       */

/* GP0OCE[OCE0] - open circuit Enable for port pin */
#define GP0OCE_OCE0_BBA                (*(volatile unsigned long *) 0x420C0180)
#define GP0OCE_OCE0_MSK                (0x1   << 0  )
#define GP0OCE_OCE0                    (0x1   << 0  )
#define GP0OCE_OCE0_DIS                (0x0   << 0  ) /* DIS                      */
#define GP0OCE_OCE0_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for GP0IN*/
#define GP0IN_RVAL                     0xFF 

/* GP0IN[IN7] - Input for port pin */
#define GP0IN_IN7_BBA                  (*(volatile unsigned long *) 0x420C029C)
#define GP0IN_IN7_MSK                  (0x1   << 7  )
#define GP0IN_IN7                      (0x1   << 7  )
#define GP0IN_IN7_LOW                  (0x0   << 7  ) /* LOW                      */
#define GP0IN_IN7_HIGH                 (0x1   << 7  ) /* HIGH                     */

/* GP0IN[IN6] - Input for port pin */
#define GP0IN_IN6_BBA                  (*(volatile unsigned long *) 0x420C0298)
#define GP0IN_IN6_MSK                  (0x1   << 6  )
#define GP0IN_IN6                      (0x1   << 6  )
#define GP0IN_IN6_LOW                  (0x0   << 6  ) /* LOW                      */
#define GP0IN_IN6_HIGH                 (0x1   << 6  ) /* HIGH                     */

/* GP0IN[IN5] - Input for port pin */
#define GP0IN_IN5_BBA                  (*(volatile unsigned long *) 0x420C0294)
#define GP0IN_IN5_MSK                  (0x1   << 5  )
#define GP0IN_IN5                      (0x1   << 5  )
#define GP0IN_IN5_LOW                  (0x0   << 5  ) /* LOW                      */
#define GP0IN_IN5_HIGH                 (0x1   << 5  ) /* HIGH                     */

/* GP0IN[IN4] - Input for port pin */
#define GP0IN_IN4_BBA                  (*(volatile unsigned long *) 0x420C0290)
#define GP0IN_IN4_MSK                  (0x1   << 4  )
#define GP0IN_IN4                      (0x1   << 4  )
#define GP0IN_IN4_LOW                  (0x0   << 4  ) /* LOW                      */
#define GP0IN_IN4_HIGH                 (0x1   << 4  ) /* HIGH                     */

/* GP0IN[IN3] - Input for port pin */
#define GP0IN_IN3_BBA                  (*(volatile unsigned long *) 0x420C028C)
#define GP0IN_IN3_MSK                  (0x1   << 3  )
#define GP0IN_IN3                      (0x1   << 3  )
#define GP0IN_IN3_LOW                  (0x0   << 3  ) /* LOW                      */
#define GP0IN_IN3_HIGH                 (0x1   << 3  ) /* HIGH                     */

/* GP0IN[IN2] - Input for port pin */
#define GP0IN_IN2_BBA                  (*(volatile unsigned long *) 0x420C0288)
#define GP0IN_IN2_MSK                  (0x1   << 2  )
#define GP0IN_IN2                      (0x1   << 2  )
#define GP0IN_IN2_LOW                  (0x0   << 2  ) /* LOW                      */
#define GP0IN_IN2_HIGH                 (0x1   << 2  ) /* HIGH                     */

/* GP0IN[IN1] - Input for port pin */
#define GP0IN_IN1_BBA                  (*(volatile unsigned long *) 0x420C0284)
#define GP0IN_IN1_MSK                  (0x1   << 1  )
#define GP0IN_IN1                      (0x1   << 1  )
#define GP0IN_IN1_LOW                  (0x0   << 1  ) /* LOW                      */
#define GP0IN_IN1_HIGH                 (0x1   << 1  ) /* HIGH                     */

/* GP0IN[IN0] - Input for port pin */
#define GP0IN_IN0_BBA                  (*(volatile unsigned long *) 0x420C0280)
#define GP0IN_IN0_MSK                  (0x1   << 0  )
#define GP0IN_IN0                      (0x1   << 0  )
#define GP0IN_IN0_LOW                  (0x0   << 0  ) /* LOW                      */
#define GP0IN_IN0_HIGH                 (0x1   << 0  ) /* HIGH                     */

/* Reset Value for GP0OUT*/
#define GP0OUT_RVAL                    0x0 

/* GP0OUT[OUT7] - Output for port pin */
#define GP0OUT_OUT7_BBA                (*(volatile unsigned long *) 0x420C031C)
#define GP0OUT_OUT7_MSK                (0x1   << 7  )
#define GP0OUT_OUT7                    (0x1   << 7  )
#define GP0OUT_OUT7_LOW                (0x0   << 7  ) /* LOW                      */
#define GP0OUT_OUT7_HIGH               (0x1   << 7  ) /* HIGH                     */

/* GP0OUT[OUT6] - Output for port pin */
#define GP0OUT_OUT6_BBA                (*(volatile unsigned long *) 0x420C0318)
#define GP0OUT_OUT6_MSK                (0x1   << 6  )
#define GP0OUT_OUT6                    (0x1   << 6  )
#define GP0OUT_OUT6_LOW                (0x0   << 6  ) /* LOW                      */
#define GP0OUT_OUT6_HIGH               (0x1   << 6  ) /* HIGH                     */

/* GP0OUT[OUT5] - Output for port pin */
#define GP0OUT_OUT5_BBA                (*(volatile unsigned long *) 0x420C0314)
#define GP0OUT_OUT5_MSK                (0x1   << 5  )
#define GP0OUT_OUT5                    (0x1   << 5  )
#define GP0OUT_OUT5_LOW                (0x0   << 5  ) /* LOW                      */
#define GP0OUT_OUT5_HIGH               (0x1   << 5  ) /* HIGH                     */

/* GP0OUT[OUT4] - Output for port pin */
#define GP0OUT_OUT4_BBA                (*(volatile unsigned long *) 0x420C0310)
#define GP0OUT_OUT4_MSK                (0x1   << 4  )
#define GP0OUT_OUT4                    (0x1   << 4  )
#define GP0OUT_OUT4_LOW                (0x0   << 4  ) /* LOW                      */
#define GP0OUT_OUT4_HIGH               (0x1   << 4  ) /* HIGH                     */

/* GP0OUT[OUT3] - Output for port pin */
#define GP0OUT_OUT3_BBA                (*(volatile unsigned long *) 0x420C030C)
#define GP0OUT_OUT3_MSK                (0x1   << 3  )
#define GP0OUT_OUT3                    (0x1   << 3  )
#define GP0OUT_OUT3_LOW                (0x0   << 3  ) /* LOW                      */
#define GP0OUT_OUT3_HIGH               (0x1   << 3  ) /* HIGH                     */

/* GP0OUT[OUT2] - Output for port pin */
#define GP0OUT_OUT2_BBA                (*(volatile unsigned long *) 0x420C0308)
#define GP0OUT_OUT2_MSK                (0x1   << 2  )
#define GP0OUT_OUT2                    (0x1   << 2  )
#define GP0OUT_OUT2_LOW                (0x0   << 2  ) /* LOW                      */
#define GP0OUT_OUT2_HIGH               (0x1   << 2  ) /* HIGH                     */

/* GP0OUT[OUT1] - Output for port pin */
#define GP0OUT_OUT1_BBA                (*(volatile unsigned long *) 0x420C0304)
#define GP0OUT_OUT1_MSK                (0x1   << 1  )
#define GP0OUT_OUT1                    (0x1   << 1  )
#define GP0OUT_OUT1_LOW                (0x0   << 1  ) /* LOW                      */
#define GP0OUT_OUT1_HIGH               (0x1   << 1  ) /* HIGH                     */

/* GP0OUT[OUT0] - Output for port pin */
#define GP0OUT_OUT0_BBA                (*(volatile unsigned long *) 0x420C0300)
#define GP0OUT_OUT0_MSK                (0x1   << 0  )
#define GP0OUT_OUT0                    (0x1   << 0  )
#define GP0OUT_OUT0_LOW                (0x0   << 0  ) /* LOW                      */
#define GP0OUT_OUT0_HIGH               (0x1   << 0  ) /* HIGH                     */

/* Reset Value for GP0SET*/
#define GP0SET_RVAL                    0x0 

/* GP0SET[SET7] - Set Output High for port pin */
#define GP0SET_SET7_BBA                (*(volatile unsigned long *) 0x420C039C)
#define GP0SET_SET7_MSK                (0x1   << 7  )
#define GP0SET_SET7                    (0x1   << 7  )
#define GP0SET_SET7_SET                (0x1   << 7  ) /* SET                      */

/* GP0SET[SET6] - Set Output High for port pin */
#define GP0SET_SET6_BBA                (*(volatile unsigned long *) 0x420C0398)
#define GP0SET_SET6_MSK                (0x1   << 6  )
#define GP0SET_SET6                    (0x1   << 6  )
#define GP0SET_SET6_SET                (0x1   << 6  ) /* SET                      */

/* GP0SET[SET5] - Set Output High for port pin */
#define GP0SET_SET5_BBA                (*(volatile unsigned long *) 0x420C0394)
#define GP0SET_SET5_MSK                (0x1   << 5  )
#define GP0SET_SET5                    (0x1   << 5  )
#define GP0SET_SET5_SET                (0x1   << 5  ) /* SET                      */

/* GP0SET[SET4] - Set Output High for port pin */
#define GP0SET_SET4_BBA                (*(volatile unsigned long *) 0x420C0390)
#define GP0SET_SET4_MSK                (0x1   << 4  )
#define GP0SET_SET4                    (0x1   << 4  )
#define GP0SET_SET4_SET                (0x1   << 4  ) /* SET                      */

/* GP0SET[SET3] - Set Output High for port pin */
#define GP0SET_SET3_BBA                (*(volatile unsigned long *) 0x420C038C)
#define GP0SET_SET3_MSK                (0x1   << 3  )
#define GP0SET_SET3                    (0x1   << 3  )
#define GP0SET_SET3_SET                (0x1   << 3  ) /* SET                      */

/* GP0SET[SET2] - Set Output High for port pin */
#define GP0SET_SET2_BBA                (*(volatile unsigned long *) 0x420C0388)
#define GP0SET_SET2_MSK                (0x1   << 2  )
#define GP0SET_SET2                    (0x1   << 2  )
#define GP0SET_SET2_SET                (0x1   << 2  ) /* SET                      */

/* GP0SET[SET1] - Set Output High for port pin */
#define GP0SET_SET1_BBA                (*(volatile unsigned long *) 0x420C0384)
#define GP0SET_SET1_MSK                (0x1   << 1  )
#define GP0SET_SET1                    (0x1   << 1  )
#define GP0SET_SET1_SET                (0x1   << 1  ) /* SET                      */

/* GP0SET[SET0] - Set Output High for port pin */
#define GP0SET_SET0_BBA                (*(volatile unsigned long *) 0x420C0380)
#define GP0SET_SET0_MSK                (0x1   << 0  )
#define GP0SET_SET0                    (0x1   << 0  )
#define GP0SET_SET0_SET                (0x1   << 0  ) /* SET                      */

/* Reset Value for GP0CLR*/
#define GP0CLR_RVAL                    0x0 

/* GP0CLR[CLR7] - Set Output Low for port pin */
#define GP0CLR_CLR7_BBA                (*(volatile unsigned long *) 0x420C041C)
#define GP0CLR_CLR7_MSK                (0x1   << 7  )
#define GP0CLR_CLR7                    (0x1   << 7  )
#define GP0CLR_CLR7_CLR                (0x1   << 7  ) /* CLR                      */

/* GP0CLR[CLR6] - Set Output Low for port pin */
#define GP0CLR_CLR6_BBA                (*(volatile unsigned long *) 0x420C0418)
#define GP0CLR_CLR6_MSK                (0x1   << 6  )
#define GP0CLR_CLR6                    (0x1   << 6  )
#define GP0CLR_CLR6_CLR                (0x1   << 6  ) /* CLR                      */

/* GP0CLR[CLR5] - Set Output Low for port pin */
#define GP0CLR_CLR5_BBA                (*(volatile unsigned long *) 0x420C0414)
#define GP0CLR_CLR5_MSK                (0x1   << 5  )
#define GP0CLR_CLR5                    (0x1   << 5  )
#define GP0CLR_CLR5_CLR                (0x1   << 5  ) /* CLR                      */

/* GP0CLR[CLR4] - Set Output Low for port pin */
#define GP0CLR_CLR4_BBA                (*(volatile unsigned long *) 0x420C0410)
#define GP0CLR_CLR4_MSK                (0x1   << 4  )
#define GP0CLR_CLR4                    (0x1   << 4  )
#define GP0CLR_CLR4_CLR                (0x1   << 4  ) /* CLR                      */

/* GP0CLR[CLR3] - Set Output Low for port pin */
#define GP0CLR_CLR3_BBA                (*(volatile unsigned long *) 0x420C040C)
#define GP0CLR_CLR3_MSK                (0x1   << 3  )
#define GP0CLR_CLR3                    (0x1   << 3  )
#define GP0CLR_CLR3_CLR                (0x1   << 3  ) /* CLR                      */

/* GP0CLR[CLR2] - Set Output Low for port pin */
#define GP0CLR_CLR2_BBA                (*(volatile unsigned long *) 0x420C0408)
#define GP0CLR_CLR2_MSK                (0x1   << 2  )
#define GP0CLR_CLR2                    (0x1   << 2  )
#define GP0CLR_CLR2_CLR                (0x1   << 2  ) /* CLR                      */

/* GP0CLR[CLR1] - Set Output Low for port pin */
#define GP0CLR_CLR1_BBA                (*(volatile unsigned long *) 0x420C0404)
#define GP0CLR_CLR1_MSK                (0x1   << 1  )
#define GP0CLR_CLR1                    (0x1   << 1  )
#define GP0CLR_CLR1_CLR                (0x1   << 1  ) /* CLR                      */

/* GP0CLR[CLR0] - Set Output Low for port pin */
#define GP0CLR_CLR0_BBA                (*(volatile unsigned long *) 0x420C0400)
#define GP0CLR_CLR0_MSK                (0x1   << 0  )
#define GP0CLR_CLR0                    (0x1   << 0  )
#define GP0CLR_CLR0_CLR                (0x1   << 0  ) /* CLR                      */

/* Reset Value for GP0TGL*/
#define GP0TGL_RVAL                    0x0 

/* GP0TGL[TGL7] - Toggle Output for port pin */
#define GP0TGL_TGL7_BBA                (*(volatile unsigned long *) 0x420C049C)
#define GP0TGL_TGL7_MSK                (0x1   << 7  )
#define GP0TGL_TGL7                    (0x1   << 7  )
#define GP0TGL_TGL7_TGL                (0x1   << 7  ) /* TGL                      */

/* GP0TGL[TGL6] - Toggle Output for port pin */
#define GP0TGL_TGL6_BBA                (*(volatile unsigned long *) 0x420C0498)
#define GP0TGL_TGL6_MSK                (0x1   << 6  )
#define GP0TGL_TGL6                    (0x1   << 6  )
#define GP0TGL_TGL6_TGL                (0x1   << 6  ) /* TGL                      */

/* GP0TGL[TGL5] - Toggle Output for port pin */
#define GP0TGL_TGL5_BBA                (*(volatile unsigned long *) 0x420C0494)
#define GP0TGL_TGL5_MSK                (0x1   << 5  )
#define GP0TGL_TGL5                    (0x1   << 5  )
#define GP0TGL_TGL5_TGL                (0x1   << 5  ) /* TGL                      */

/* GP0TGL[TGL4] - Toggle Output for port pin */
#define GP0TGL_TGL4_BBA                (*(volatile unsigned long *) 0x420C0490)
#define GP0TGL_TGL4_MSK                (0x1   << 4  )
#define GP0TGL_TGL4                    (0x1   << 4  )
#define GP0TGL_TGL4_TGL                (0x1   << 4  ) /* TGL                      */

/* GP0TGL[TGL3] - Toggle Output for port pin */
#define GP0TGL_TGL3_BBA                (*(volatile unsigned long *) 0x420C048C)
#define GP0TGL_TGL3_MSK                (0x1   << 3  )
#define GP0TGL_TGL3                    (0x1   << 3  )
#define GP0TGL_TGL3_TGL                (0x1   << 3  ) /* TGL                      */

/* GP0TGL[TGL2] - Toggle Output for port pin */
#define GP0TGL_TGL2_BBA                (*(volatile unsigned long *) 0x420C0488)
#define GP0TGL_TGL2_MSK                (0x1   << 2  )
#define GP0TGL_TGL2                    (0x1   << 2  )
#define GP0TGL_TGL2_TGL                (0x1   << 2  ) /* TGL                      */

/* GP0TGL[TGL1] - Toggle Output for port pin */
#define GP0TGL_TGL1_BBA                (*(volatile unsigned long *) 0x420C0484)
#define GP0TGL_TGL1_MSK                (0x1   << 1  )
#define GP0TGL_TGL1                    (0x1   << 1  )
#define GP0TGL_TGL1_TGL                (0x1   << 1  ) /* TGL                      */

/* GP0TGL[TGL0] - Toggle Output for port pin */
#define GP0TGL_TGL0_BBA                (*(volatile unsigned long *) 0x420C0480)
#define GP0TGL_TGL0_MSK                (0x1   << 0  )
#define GP0TGL_TGL0                    (0x1   << 0  )
#define GP0TGL_TGL0_TGL                (0x1   << 0  ) /* TGL                      */
#if (__NO_MMR_STRUCTS__==1)

#define          GP1CON                                     (*(volatile unsigned short int *) 0x40006030)
#define          GP1OEN                                     (*(volatile unsigned char      *) 0x40006034)
#define          GP1PUL                                     (*(volatile unsigned char      *) 0x40006038)
#define          GP1OCE                                     (*(volatile unsigned char      *) 0x4000603C)
#define          GP1IN                                      (*(volatile unsigned char      *) 0x40006044)
#define          GP1OUT                                     (*(volatile unsigned char      *) 0x40006048)
#define          GP1SET                                     (*(volatile unsigned char      *) 0x4000604C)
#define          GP1CLR                                     (*(volatile unsigned char      *) 0x40006050)
#define          GP1TGL                                     (*(volatile unsigned char      *) 0x40006054)
#endif // (__NO_MMR_STRUCTS__==1)

/* Reset Value for GP1CON*/
#define GP1CON_RVAL                    0x0 

/* GP1CON[CON7] - Configuration bits for P1.7 */
#define GP1CON_CON7_MSK                (0x3   << 14 )
#define GP1CON_CON7_GPIOIRQ7           (0x0   << 14 ) /* GPIOIRQ7                 */
#define GP1CON_CON7_PWM5               (0x1   << 14 ) /* PWM5                     */
#define GP1CON_CON7_SPI0CS             (0x2   << 14 ) /* SPI0CS                   */

/* GP1CON[CON6] - Configuration bits for P1.6 */
#define GP1CON_CON6_MSK                (0x3   << 12 )
#define GP1CON_CON6_GPIOIRQ6           (0x0   << 12 ) /* GPIOIRQ6                 */
#define GP1CON_CON6_PWM4               (0x1   << 12 ) /* PWM4                     */
#define GP1CON_CON6_SPI0MOSI           (0x2   << 12 ) /* SPI0MOSI                 */

/* GP1CON[CON5] - Configuration bits for P1.5 */
#define GP1CON_CON5_MSK                (0x3   << 10 )
#define GP1CON_CON5_GPIOIRQ5           (0x0   << 10 ) /* GPIOIRQ5                 */
#define GP1CON_CON5_PWM3               (0x1   << 10 ) /* PWM3                     */
#define GP1CON_CON5_SPI0SCLK           (0x2   << 10 ) /* SPI0SCLK                 */

/* GP1CON[CON4] - Configuration bits for P1.4 */
#define GP1CON_CON4_MSK                (0x3   << 8  )
#define GP1CON_CON4_GPIO               (0x0   << 8  ) /* GPIO                     */
#define GP1CON_CON4_PWM2               (0x1   << 8  ) /* PWM2                     */
#define GP1CON_CON4_SPI0MISO           (0x2   << 8  ) /* SPI0MISO                 */

/* GP1CON[CON3] - Configuration bits for P1.3 */
#define GP1CON_CON3_MSK                (0x3   << 6  )
#define GP1CON_CON3_GPIO               (0x0   << 6  ) /* GPIO                     */
#define GP1CON_CON3_PWM1               (0x1   << 6  ) /* PWM1                     */

/* GP1CON[CON2] - Configuration bits for P1.2 */
#define GP1CON_CON2_MSK                (0x3   << 4  )
#define GP1CON_CON2_GPIO               (0x0   << 4  ) /* GPIO                     */
#define GP1CON_CON2_PWM0               (0x1   << 4  ) /* PWM0                     */

/* GP1CON[CON1] - Configuration bits for P1.1 */
#define GP1CON_CON1_MSK                (0x3   << 2  )
#define GP1CON_CON1_GPIOIRQ4           (0x0   << 2  ) /* GPIOIRQ4                 */
#define GP1CON_CON1_PWMTRIP            (0x1   << 2  ) /* PWMTRIP                  */

/* GP1CON[CON0] - Configuration bits for P1.0 */
#define GP1CON_CON0_MSK                (0x3   << 0  )
#define GP1CON_CON0_GPIOIRQ3           (0x0   << 0  ) /* GPIOIRQ3                 */
#define GP1CON_CON0_PWMSYNC            (0x1   << 0  ) /* PWMSYNC                  */
#define GP1CON_CON0_EXTCLKIN           (0x2   << 0  ) /* EXTCLKIN                 */

/* Reset Value for GP1OEN*/
#define GP1OEN_RVAL                    0x0 

/* GP1OEN[OEN7] - Direction for port pin */
#define GP1OEN_OEN7_BBA                (*(volatile unsigned long *) 0x420C069C)
#define GP1OEN_OEN7_MSK                (0x1   << 7  )
#define GP1OEN_OEN7                    (0x1   << 7  )
#define GP1OEN_OEN7_IN                 (0x0   << 7  ) /* IN                       */
#define GP1OEN_OEN7_OUT                (0x1   << 7  ) /* OUT                      */

/* GP1OEN[OEN6] - Direction for port pin */
#define GP1OEN_OEN6_BBA                (*(volatile unsigned long *) 0x420C0698)
#define GP1OEN_OEN6_MSK                (0x1   << 6  )
#define GP1OEN_OEN6                    (0x1   << 6  )
#define GP1OEN_OEN6_IN                 (0x0   << 6  ) /* IN                       */
#define GP1OEN_OEN6_OUT                (0x1   << 6  ) /* OUT                      */

/* GP1OEN[OEN5] - Direction for port pin */
#define GP1OEN_OEN5_BBA                (*(volatile unsigned long *) 0x420C0694)
#define GP1OEN_OEN5_MSK                (0x1   << 5  )
#define GP1OEN_OEN5                    (0x1   << 5  )
#define GP1OEN_OEN5_IN                 (0x0   << 5  ) /* IN                       */
#define GP1OEN_OEN5_OUT                (0x1   << 5  ) /* OUT                      */

/* GP1OEN[OEN4] - Direction for port pin */
#define GP1OEN_OEN4_BBA                (*(volatile unsigned long *) 0x420C0690)
#define GP1OEN_OEN4_MSK                (0x1   << 4  )
#define GP1OEN_OEN4                    (0x1   << 4  )
#define GP1OEN_OEN4_IN                 (0x0   << 4  ) /* IN                       */
#define GP1OEN_OEN4_OUT                (0x1   << 4  ) /* OUT                      */

/* GP1OEN[OEN3] - Direction for port pin */
#define GP1OEN_OEN3_BBA                (*(volatile unsigned long *) 0x420C068C)
#define GP1OEN_OEN3_MSK                (0x1   << 3  )
#define GP1OEN_OEN3                    (0x1   << 3  )
#define GP1OEN_OEN3_IN                 (0x0   << 3  ) /* IN                       */
#define GP1OEN_OEN3_OUT                (0x1   << 3  ) /* OUT                      */

/* GP1OEN[OEN2] - Direction for port pin */
#define GP1OEN_OEN2_BBA                (*(volatile unsigned long *) 0x420C0688)
#define GP1OEN_OEN2_MSK                (0x1   << 2  )
#define GP1OEN_OEN2                    (0x1   << 2  )
#define GP1OEN_OEN2_IN                 (0x0   << 2  ) /* IN                       */
#define GP1OEN_OEN2_OUT                (0x1   << 2  ) /* OUT                      */

/* GP1OEN[OEN1] - Direction for port pin */
#define GP1OEN_OEN1_BBA                (*(volatile unsigned long *) 0x420C0684)
#define GP1OEN_OEN1_MSK                (0x1   << 1  )
#define GP1OEN_OEN1                    (0x1   << 1  )
#define GP1OEN_OEN1_IN                 (0x0   << 1  ) /* IN                       */
#define GP1OEN_OEN1_OUT                (0x1   << 1  ) /* OUT                      */

/* GP1OEN[OEN0] - Direction for port pin */
#define GP1OEN_OEN0_BBA                (*(volatile unsigned long *) 0x420C0680)
#define GP1OEN_OEN0_MSK                (0x1   << 0  )
#define GP1OEN_OEN0                    (0x1   << 0  )
#define GP1OEN_OEN0_IN                 (0x0   << 0  ) /* IN                       */
#define GP1OEN_OEN0_OUT                (0x1   << 0  ) /* OUT                      */

/* Reset Value for GP1PUL*/
#define GP1PUL_RVAL                    0xFF 

/* GP1PUL[PUL7] - Pull Up Enable for port pin */
#define GP1PUL_PUL7_BBA                (*(volatile unsigned long *) 0x420C071C)
#define GP1PUL_PUL7_MSK                (0x1   << 7  )
#define GP1PUL_PUL7                    (0x1   << 7  )
#define GP1PUL_PUL7_DIS                (0x0   << 7  ) /* DIS                      */
#define GP1PUL_PUL7_EN                 (0x1   << 7  ) /* EN                       */

/* GP1PUL[PUL6] - Pull Up Enable for port pin */
#define GP1PUL_PUL6_BBA                (*(volatile unsigned long *) 0x420C0718)
#define GP1PUL_PUL6_MSK                (0x1   << 6  )
#define GP1PUL_PUL6                    (0x1   << 6  )
#define GP1PUL_PUL6_DIS                (0x0   << 6  ) /* DIS                      */
#define GP1PUL_PUL6_EN                 (0x1   << 6  ) /* EN                       */

/* GP1PUL[PUL5] - Pull Up Enable for port pin */
#define GP1PUL_PUL5_BBA                (*(volatile unsigned long *) 0x420C0714)
#define GP1PUL_PUL5_MSK                (0x1   << 5  )
#define GP1PUL_PUL5                    (0x1   << 5  )
#define GP1PUL_PUL5_DIS                (0x0   << 5  ) /* DIS                      */
#define GP1PUL_PUL5_EN                 (0x1   << 5  ) /* EN                       */

/* GP1PUL[PUL4] - Pull Up Enable for port pin */
#define GP1PUL_PUL4_BBA                (*(volatile unsigned long *) 0x420C0710)
#define GP1PUL_PUL4_MSK                (0x1   << 4  )
#define GP1PUL_PUL4                    (0x1   << 4  )
#define GP1PUL_PUL4_DIS                (0x0   << 4  ) /* DIS                      */
#define GP1PUL_PUL4_EN                 (0x1   << 4  ) /* EN                       */

/* GP1PUL[PUL3] - Pull Up Enable for port pin */
#define GP1PUL_PUL3_BBA                (*(volatile unsigned long *) 0x420C070C)
#define GP1PUL_PUL3_MSK                (0x1   << 3  )
#define GP1PUL_PUL3                    (0x1   << 3  )
#define GP1PUL_PUL3_DIS                (0x0   << 3  ) /* DIS                      */
#define GP1PUL_PUL3_EN                 (0x1   << 3  ) /* EN                       */

/* GP1PUL[PUL2] - Pull Up Enable for port pin */
#define GP1PUL_PUL2_BBA                (*(volatile unsigned long *) 0x420C0708)
#define GP1PUL_PUL2_MSK                (0x1   << 2  )
#define GP1PUL_PUL2                    (0x1   << 2  )
#define GP1PUL_PUL2_DIS                (0x0   << 2  ) /* DIS                      */
#define GP1PUL_PUL2_EN                 (0x1   << 2  ) /* EN                       */

/* GP1PUL[PUL1] - Pull Up Enable for port pin */
#define GP1PUL_PUL1_BBA                (*(volatile unsigned long *) 0x420C0704)
#define GP1PUL_PUL1_MSK                (0x1   << 1  )
#define GP1PUL_PUL1                    (0x1   << 1  )
#define GP1PUL_PUL1_DIS                (0x0   << 1  ) /* DIS                      */
#define GP1PUL_PUL1_EN                 (0x1   << 1  ) /* EN                       */

/* GP1PUL[PUL0] - Pull Up Enable for port pin */
#define GP1PUL_PUL0_BBA                (*(volatile unsigned long *) 0x420C0700)
#define GP1PUL_PUL0_MSK                (0x1   << 0  )
#define GP1PUL_PUL0                    (0x1   << 0  )
#define GP1PUL_PUL0_DIS                (0x0   << 0  ) /* DIS                      */
#define GP1PUL_PUL0_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for GP1OCE*/
#define GP1OCE_RVAL                    0x0 

/* GP1OCE[OCE7] - open circuit Enable for port pin */
#define GP1OCE_OCE7_BBA                (*(volatile unsigned long *) 0x420C079C)
#define GP1OCE_OCE7_MSK                (0x1   << 7  )
#define GP1OCE_OCE7                    (0x1   << 7  )
#define GP1OCE_OCE7_DIS                (0x0   << 7  ) /* DIS                      */
#define GP1OCE_OCE7_EN                 (0x1   << 7  ) /* EN                       */

/* GP1OCE[OCE6] - open circuit Enable for port pin */
#define GP1OCE_OCE6_BBA                (*(volatile unsigned long *) 0x420C0798)
#define GP1OCE_OCE6_MSK                (0x1   << 6  )
#define GP1OCE_OCE6                    (0x1   << 6  )
#define GP1OCE_OCE6_DIS                (0x0   << 6  ) /* DIS                      */
#define GP1OCE_OCE6_EN                 (0x1   << 6  ) /* EN                       */

/* GP1OCE[OCE5] - open circuit Enable for port pin */
#define GP1OCE_OCE5_BBA                (*(volatile unsigned long *) 0x420C0794)
#define GP1OCE_OCE5_MSK                (0x1   << 5  )
#define GP1OCE_OCE5                    (0x1   << 5  )
#define GP1OCE_OCE5_DIS                (0x0   << 5  ) /* DIS                      */
#define GP1OCE_OCE5_EN                 (0x1   << 5  ) /* EN                       */

/* GP1OCE[OCE4] - open circuit Enable for port pin */
#define GP1OCE_OCE4_BBA                (*(volatile unsigned long *) 0x420C0790)
#define GP1OCE_OCE4_MSK                (0x1   << 4  )
#define GP1OCE_OCE4                    (0x1   << 4  )
#define GP1OCE_OCE4_DIS                (0x0   << 4  ) /* DIS                      */
#define GP1OCE_OCE4_EN                 (0x1   << 4  ) /* EN                       */

/* GP1OCE[OCE3] - open circuit Enable for port pin */
#define GP1OCE_OCE3_BBA                (*(volatile unsigned long *) 0x420C078C)
#define GP1OCE_OCE3_MSK                (0x1   << 3  )
#define GP1OCE_OCE3                    (0x1   << 3  )
#define GP1OCE_OCE3_DIS                (0x0   << 3  ) /* DIS                      */
#define GP1OCE_OCE3_EN                 (0x1   << 3  ) /* EN                       */

/* GP1OCE[OCE2] - open circuit Enable for port pin */
#define GP1OCE_OCE2_BBA                (*(volatile unsigned long *) 0x420C0788)
#define GP1OCE_OCE2_MSK                (0x1   << 2  )
#define GP1OCE_OCE2                    (0x1   << 2  )
#define GP1OCE_OCE2_DIS                (0x0   << 2  ) /* DIS                      */
#define GP1OCE_OCE2_EN                 (0x1   << 2  ) /* EN                       */

/* GP1OCE[OCE1] - open circuit Enable for port pin */
#define GP1OCE_OCE1_BBA                (*(volatile unsigned long *) 0x420C0784)
#define GP1OCE_OCE1_MSK                (0x1   << 1  )
#define GP1OCE_OCE1                    (0x1   << 1  )
#define GP1OCE_OCE1_DIS                (0x0   << 1  ) /* DIS                      */
#define GP1OCE_OCE1_EN                 (0x1   << 1  ) /* EN                       */

/* GP1OCE[OCE0] - open circuit Enable for port pin */
#define GP1OCE_OCE0_BBA                (*(volatile unsigned long *) 0x420C0780)
#define GP1OCE_OCE0_MSK                (0x1   << 0  )
#define GP1OCE_OCE0                    (0x1   << 0  )
#define GP1OCE_OCE0_DIS                (0x0   << 0  ) /* DIS                      */
#define GP1OCE_OCE0_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for GP1IN*/
#define GP1IN_RVAL                     0xFF 

/* GP1IN[IN7] - Input for port pin */
#define GP1IN_IN7_BBA                  (*(volatile unsigned long *) 0x420C089C)
#define GP1IN_IN7_MSK                  (0x1   << 7  )
#define GP1IN_IN7                      (0x1   << 7  )
#define GP1IN_IN7_LOW                  (0x0   << 7  ) /* LOW                      */
#define GP1IN_IN7_HIGH                 (0x1   << 7  ) /* HIGH                     */

/* GP1IN[IN6] - Input for port pin */
#define GP1IN_IN6_BBA                  (*(volatile unsigned long *) 0x420C0898)
#define GP1IN_IN6_MSK                  (0x1   << 6  )
#define GP1IN_IN6                      (0x1   << 6  )
#define GP1IN_IN6_LOW                  (0x0   << 6  ) /* LOW                      */
#define GP1IN_IN6_HIGH                 (0x1   << 6  ) /* HIGH                     */

/* GP1IN[IN5] - Input for port pin */
#define GP1IN_IN5_BBA                  (*(volatile unsigned long *) 0x420C0894)
#define GP1IN_IN5_MSK                  (0x1   << 5  )
#define GP1IN_IN5                      (0x1   << 5  )
#define GP1IN_IN5_LOW                  (0x0   << 5  ) /* LOW                      */
#define GP1IN_IN5_HIGH                 (0x1   << 5  ) /* HIGH                     */

/* GP1IN[IN4] - Input for port pin */
#define GP1IN_IN4_BBA                  (*(volatile unsigned long *) 0x420C0890)
#define GP1IN_IN4_MSK                  (0x1   << 4  )
#define GP1IN_IN4                      (0x1   << 4  )
#define GP1IN_IN4_LOW                  (0x0   << 4  ) /* LOW                      */
#define GP1IN_IN4_HIGH                 (0x1   << 4  ) /* HIGH                     */

/* GP1IN[IN3] - Input for port pin */
#define GP1IN_IN3_BBA                  (*(volatile unsigned long *) 0x420C088C)
#define GP1IN_IN3_MSK                  (0x1   << 3  )
#define GP1IN_IN3                      (0x1   << 3  )
#define GP1IN_IN3_LOW                  (0x0   << 3  ) /* LOW                      */
#define GP1IN_IN3_HIGH                 (0x1   << 3  ) /* HIGH                     */

/* GP1IN[IN2] - Input for port pin */
#define GP1IN_IN2_BBA                  (*(volatile unsigned long *) 0x420C0888)
#define GP1IN_IN2_MSK                  (0x1   << 2  )
#define GP1IN_IN2                      (0x1   << 2  )
#define GP1IN_IN2_LOW                  (0x0   << 2  ) /* LOW                      */
#define GP1IN_IN2_HIGH                 (0x1   << 2  ) /* HIGH                     */

/* GP1IN[IN1] - Input for port pin */
#define GP1IN_IN1_BBA                  (*(volatile unsigned long *) 0x420C0884)
#define GP1IN_IN1_MSK                  (0x1   << 1  )
#define GP1IN_IN1                      (0x1   << 1  )
#define GP1IN_IN1_LOW                  (0x0   << 1  ) /* LOW                      */
#define GP1IN_IN1_HIGH                 (0x1   << 1  ) /* HIGH                     */

/* GP1IN[IN0] - Input for port pin */
#define GP1IN_IN0_BBA                  (*(volatile unsigned long *) 0x420C0880)
#define GP1IN_IN0_MSK                  (0x1   << 0  )
#define GP1IN_IN0                      (0x1   << 0  )
#define GP1IN_IN0_LOW                  (0x0   << 0  ) /* LOW                      */
#define GP1IN_IN0_HIGH                 (0x1   << 0  ) /* HIGH                     */

/* Reset Value for GP1OUT*/
#define GP1OUT_RVAL                    0x0 

/* GP1OUT[OUT7] - Output for port pin */
#define GP1OUT_OUT7_BBA                (*(volatile unsigned long *) 0x420C091C)
#define GP1OUT_OUT7_MSK                (0x1   << 7  )
#define GP1OUT_OUT7                    (0x1   << 7  )
#define GP1OUT_OUT7_LOW                (0x0   << 7  ) /* LOW                      */
#define GP1OUT_OUT7_HIGH               (0x1   << 7  ) /* HIGH                     */

/* GP1OUT[OUT6] - Output for port pin */
#define GP1OUT_OUT6_BBA                (*(volatile unsigned long *) 0x420C0918)
#define GP1OUT_OUT6_MSK                (0x1   << 6  )
#define GP1OUT_OUT6                    (0x1   << 6  )
#define GP1OUT_OUT6_LOW                (0x0   << 6  ) /* LOW                      */
#define GP1OUT_OUT6_HIGH               (0x1   << 6  ) /* HIGH                     */

/* GP1OUT[OUT5] - Output for port pin */
#define GP1OUT_OUT5_BBA                (*(volatile unsigned long *) 0x420C0914)
#define GP1OUT_OUT5_MSK                (0x1   << 5  )
#define GP1OUT_OUT5                    (0x1   << 5  )
#define GP1OUT_OUT5_LOW                (0x0   << 5  ) /* LOW                      */
#define GP1OUT_OUT5_HIGH               (0x1   << 5  ) /* HIGH                     */

/* GP1OUT[OUT4] - Output for port pin */
#define GP1OUT_OUT4_BBA                (*(volatile unsigned long *) 0x420C0910)
#define GP1OUT_OUT4_MSK                (0x1   << 4  )
#define GP1OUT_OUT4                    (0x1   << 4  )
#define GP1OUT_OUT4_LOW                (0x0   << 4  ) /* LOW                      */
#define GP1OUT_OUT4_HIGH               (0x1   << 4  ) /* HIGH                     */

/* GP1OUT[OUT3] - Output for port pin */
#define GP1OUT_OUT3_BBA                (*(volatile unsigned long *) 0x420C090C)
#define GP1OUT_OUT3_MSK                (0x1   << 3  )
#define GP1OUT_OUT3                    (0x1   << 3  )
#define GP1OUT_OUT3_LOW                (0x0   << 3  ) /* LOW                      */
#define GP1OUT_OUT3_HIGH               (0x1   << 3  ) /* HIGH                     */

/* GP1OUT[OUT2] - Output for port pin */
#define GP1OUT_OUT2_BBA                (*(volatile unsigned long *) 0x420C0908)
#define GP1OUT_OUT2_MSK                (0x1   << 2  )
#define GP1OUT_OUT2                    (0x1   << 2  )
#define GP1OUT_OUT2_LOW                (0x0   << 2  ) /* LOW                      */
#define GP1OUT_OUT2_HIGH               (0x1   << 2  ) /* HIGH                     */

/* GP1OUT[OUT1] - Output for port pin */
#define GP1OUT_OUT1_BBA                (*(volatile unsigned long *) 0x420C0904)
#define GP1OUT_OUT1_MSK                (0x1   << 1  )
#define GP1OUT_OUT1                    (0x1   << 1  )
#define GP1OUT_OUT1_LOW                (0x0   << 1  ) /* LOW                      */
#define GP1OUT_OUT1_HIGH               (0x1   << 1  ) /* HIGH                     */

/* GP1OUT[OUT0] - Output for port pin */
#define GP1OUT_OUT0_BBA                (*(volatile unsigned long *) 0x420C0900)
#define GP1OUT_OUT0_MSK                (0x1   << 0  )
#define GP1OUT_OUT0                    (0x1   << 0  )
#define GP1OUT_OUT0_LOW                (0x0   << 0  ) /* LOW                      */
#define GP1OUT_OUT0_HIGH               (0x1   << 0  ) /* HIGH                     */

/* Reset Value for GP1SET*/
#define GP1SET_RVAL                    0x0 

/* GP1SET[SET7] - Set Output High for port pin */
#define GP1SET_SET7_BBA                (*(volatile unsigned long *) 0x420C099C)
#define GP1SET_SET7_MSK                (0x1   << 7  )
#define GP1SET_SET7                    (0x1   << 7  )
#define GP1SET_SET7_SET                (0x1   << 7  ) /* SET                      */

/* GP1SET[SET6] - Set Output High for port pin */
#define GP1SET_SET6_BBA                (*(volatile unsigned long *) 0x420C0998)
#define GP1SET_SET6_MSK                (0x1   << 6  )
#define GP1SET_SET6                    (0x1   << 6  )
#define GP1SET_SET6_SET                (0x1   << 6  ) /* SET                      */

/* GP1SET[SET5] - Set Output High for port pin */
#define GP1SET_SET5_BBA                (*(volatile unsigned long *) 0x420C0994)
#define GP1SET_SET5_MSK                (0x1   << 5  )
#define GP1SET_SET5                    (0x1   << 5  )
#define GP1SET_SET5_SET                (0x1   << 5  ) /* SET                      */

/* GP1SET[SET4] - Set Output High for port pin */
#define GP1SET_SET4_BBA                (*(volatile unsigned long *) 0x420C0990)
#define GP1SET_SET4_MSK                (0x1   << 4  )
#define GP1SET_SET4                    (0x1   << 4  )
#define GP1SET_SET4_SET                (0x1   << 4  ) /* SET                      */

/* GP1SET[SET3] - Set Output High for port pin */
#define GP1SET_SET3_BBA                (*(volatile unsigned long *) 0x420C098C)
#define GP1SET_SET3_MSK                (0x1   << 3  )
#define GP1SET_SET3                    (0x1   << 3  )
#define GP1SET_SET3_SET                (0x1   << 3  ) /* SET                      */

/* GP1SET[SET2] - Set Output High for port pin */
#define GP1SET_SET2_BBA                (*(volatile unsigned long *) 0x420C0988)
#define GP1SET_SET2_MSK                (0x1   << 2  )
#define GP1SET_SET2                    (0x1   << 2  )
#define GP1SET_SET2_SET                (0x1   << 2  ) /* SET                      */

/* GP1SET[SET1] - Set Output High for port pin */
#define GP1SET_SET1_BBA                (*(volatile unsigned long *) 0x420C0984)
#define GP1SET_SET1_MSK                (0x1   << 1  )
#define GP1SET_SET1                    (0x1   << 1  )
#define GP1SET_SET1_SET                (0x1   << 1  ) /* SET                      */

/* GP1SET[SET0] - Set Output High for port pin */
#define GP1SET_SET0_BBA                (*(volatile unsigned long *) 0x420C0980)
#define GP1SET_SET0_MSK                (0x1   << 0  )
#define GP1SET_SET0                    (0x1   << 0  )
#define GP1SET_SET0_SET                (0x1   << 0  ) /* SET                      */

/* Reset Value for GP1CLR*/
#define GP1CLR_RVAL                    0x0 

/* GP1CLR[CLR7] - Set Output Low for port pin */
#define GP1CLR_CLR7_BBA                (*(volatile unsigned long *) 0x420C0A1C)
#define GP1CLR_CLR7_MSK                (0x1   << 7  )
#define GP1CLR_CLR7                    (0x1   << 7  )
#define GP1CLR_CLR7_CLR                (0x1   << 7  ) /* CLR                      */

/* GP1CLR[CLR6] - Set Output Low for port pin */
#define GP1CLR_CLR6_BBA                (*(volatile unsigned long *) 0x420C0A18)
#define GP1CLR_CLR6_MSK                (0x1   << 6  )
#define GP1CLR_CLR6                    (0x1   << 6  )
#define GP1CLR_CLR6_CLR                (0x1   << 6  ) /* CLR                      */

/* GP1CLR[CLR5] - Set Output Low for port pin */
#define GP1CLR_CLR5_BBA                (*(volatile unsigned long *) 0x420C0A14)
#define GP1CLR_CLR5_MSK                (0x1   << 5  )
#define GP1CLR_CLR5                    (0x1   << 5  )
#define GP1CLR_CLR5_CLR                (0x1   << 5  ) /* CLR                      */

/* GP1CLR[CLR4] - Set Output Low for port pin */
#define GP1CLR_CLR4_BBA                (*(volatile unsigned long *) 0x420C0A10)
#define GP1CLR_CLR4_MSK                (0x1   << 4  )
#define GP1CLR_CLR4                    (0x1   << 4  )
#define GP1CLR_CLR4_CLR                (0x1   << 4  ) /* CLR                      */

/* GP1CLR[CLR3] - Set Output Low for port pin */
#define GP1CLR_CLR3_BBA                (*(volatile unsigned long *) 0x420C0A0C)
#define GP1CLR_CLR3_MSK                (0x1   << 3  )
#define GP1CLR_CLR3                    (0x1   << 3  )
#define GP1CLR_CLR3_CLR                (0x1   << 3  ) /* CLR                      */

/* GP1CLR[CLR2] - Set Output Low for port pin */
#define GP1CLR_CLR2_BBA                (*(volatile unsigned long *) 0x420C0A08)
#define GP1CLR_CLR2_MSK                (0x1   << 2  )
#define GP1CLR_CLR2                    (0x1   << 2  )
#define GP1CLR_CLR2_CLR                (0x1   << 2  ) /* CLR                      */

/* GP1CLR[CLR1] - Set Output Low for port pin */
#define GP1CLR_CLR1_BBA                (*(volatile unsigned long *) 0x420C0A04)
#define GP1CLR_CLR1_MSK                (0x1   << 1  )
#define GP1CLR_CLR1                    (0x1   << 1  )
#define GP1CLR_CLR1_CLR                (0x1   << 1  ) /* CLR                      */

/* GP1CLR[CLR0] - Set Output Low for port pin */
#define GP1CLR_CLR0_BBA                (*(volatile unsigned long *) 0x420C0A00)
#define GP1CLR_CLR0_MSK                (0x1   << 0  )
#define GP1CLR_CLR0                    (0x1   << 0  )
#define GP1CLR_CLR0_CLR                (0x1   << 0  ) /* CLR                      */

/* Reset Value for GP1TGL*/
#define GP1TGL_RVAL                    0x0 

/* GP1TGL[TGL7] - Toggle Output for port pin */
#define GP1TGL_TGL7_BBA                (*(volatile unsigned long *) 0x420C0A9C)
#define GP1TGL_TGL7_MSK                (0x1   << 7  )
#define GP1TGL_TGL7                    (0x1   << 7  )
#define GP1TGL_TGL7_TGL                (0x1   << 7  ) /* TGL                      */

/* GP1TGL[TGL6] - Toggle Output for port pin */
#define GP1TGL_TGL6_BBA                (*(volatile unsigned long *) 0x420C0A98)
#define GP1TGL_TGL6_MSK                (0x1   << 6  )
#define GP1TGL_TGL6                    (0x1   << 6  )
#define GP1TGL_TGL6_TGL                (0x1   << 6  ) /* TGL                      */

/* GP1TGL[TGL5] - Toggle Output for port pin */
#define GP1TGL_TGL5_BBA                (*(volatile unsigned long *) 0x420C0A94)
#define GP1TGL_TGL5_MSK                (0x1   << 5  )
#define GP1TGL_TGL5                    (0x1   << 5  )
#define GP1TGL_TGL5_TGL                (0x1   << 5  ) /* TGL                      */

/* GP1TGL[TGL4] - Toggle Output for port pin */
#define GP1TGL_TGL4_BBA                (*(volatile unsigned long *) 0x420C0A90)
#define GP1TGL_TGL4_MSK                (0x1   << 4  )
#define GP1TGL_TGL4                    (0x1   << 4  )
#define GP1TGL_TGL4_TGL                (0x1   << 4  ) /* TGL                      */

/* GP1TGL[TGL3] - Toggle Output for port pin */
#define GP1TGL_TGL3_BBA                (*(volatile unsigned long *) 0x420C0A8C)
#define GP1TGL_TGL3_MSK                (0x1   << 3  )
#define GP1TGL_TGL3                    (0x1   << 3  )
#define GP1TGL_TGL3_TGL                (0x1   << 3  ) /* TGL                      */

/* GP1TGL[TGL2] - Toggle Output for port pin */
#define GP1TGL_TGL2_BBA                (*(volatile unsigned long *) 0x420C0A88)
#define GP1TGL_TGL2_MSK                (0x1   << 2  )
#define GP1TGL_TGL2                    (0x1   << 2  )
#define GP1TGL_TGL2_TGL                (0x1   << 2  ) /* TGL                      */

/* GP1TGL[TGL1] - Toggle Output for port pin */
#define GP1TGL_TGL1_BBA                (*(volatile unsigned long *) 0x420C0A84)
#define GP1TGL_TGL1_MSK                (0x1   << 1  )
#define GP1TGL_TGL1                    (0x1   << 1  )
#define GP1TGL_TGL1_TGL                (0x1   << 1  ) /* TGL                      */

/* GP1TGL[TGL0] - Toggle Output for port pin */
#define GP1TGL_TGL0_BBA                (*(volatile unsigned long *) 0x420C0A80)
#define GP1TGL_TGL0_MSK                (0x1   << 0  )
#define GP1TGL_TGL0                    (0x1   << 0  )
#define GP1TGL_TGL0_TGL                (0x1   << 0  ) /* TGL                      */
#if (__NO_MMR_STRUCTS__==1)

#define          GP2CON                                     (*(volatile unsigned short int *) 0x40006060)
#define          GP2OEN                                     (*(volatile unsigned char      *) 0x40006064)
#define          GP2PUL                                     (*(volatile unsigned char      *) 0x40006068)
#define          GP2OCE                                     (*(volatile unsigned char      *) 0x4000606C)
#define          GP2IN                                      (*(volatile unsigned char      *) 0x40006074)
#define          GP2OUT                                     (*(volatile unsigned char      *) 0x40006078)
#define          GP2SET                                     (*(volatile unsigned char      *) 0x4000607C)
#define          GP2CLR                                     (*(volatile unsigned char      *) 0x40006080)
#define          GP2TGL                                     (*(volatile unsigned char      *) 0x40006084)
#endif // (__NO_MMR_STRUCTS__==1)

/* Reset Value for GP2CON*/
#define GP2CON_RVAL                    0x0 

/* GP2CON[CON4] - Configuration bits for P2.4 */
#define GP2CON_CON4_MSK                (0x3   << 8  )
#define GP2CON_CON4_SWDATA             (0x1   << 8  ) /* SWDATA                   */

/* GP2CON[CON3] - Configuration bits for P2.3 */
#define GP2CON_CON3_MSK                (0x3   << 6  )
#define GP2CON_CON3_SWCLK              (0x1   << 6  ) /* SWCLK                    */

/* GP2CON[CON2] - Configuration bits for P2.2 */
#define GP2CON_CON2_MSK                (0x3   << 4  )
#define GP2CON_CON2_GPIO               (0x0   << 4  ) /* GPIO                     */

/* GP2CON[CON1] - Configuration bits for P2.1 */
#define GP2CON_CON1_MSK                (0x3   << 2  )
#define GP2CON_CON1_GPIO               (0x0   << 2  ) /* GPIO                     */
#define GP2CON_CON1_I2CSDA             (0x1   << 2  ) /* I2CSDA                   */

/* GP2CON[CON0] - Configuration bits for P2.0 */
#define GP2CON_CON0_MSK                (0x3   << 0  )
#define GP2CON_CON0_GPIO               (0x0   << 0  ) /* GPIO                     */
#define GP2CON_CON0_I2CSCL             (0x1   << 0  ) /* I2CSCL                   */

/* Reset Value for GP2OEN*/
#define GP2OEN_RVAL                    0x0 

/* GP2OEN[OEN7] - Direction for port pin */
#define GP2OEN_OEN7_BBA                (*(volatile unsigned long *) 0x420C0C9C)
#define GP2OEN_OEN7_MSK                (0x1   << 7  )
#define GP2OEN_OEN7                    (0x1   << 7  )
#define GP2OEN_OEN7_IN                 (0x0   << 7  ) /* IN                       */
#define GP2OEN_OEN7_OUT                (0x1   << 7  ) /* OUT                      */

/* GP2OEN[OEN6] - Direction for port pin */
#define GP2OEN_OEN6_BBA                (*(volatile unsigned long *) 0x420C0C98)
#define GP2OEN_OEN6_MSK                (0x1   << 6  )
#define GP2OEN_OEN6                    (0x1   << 6  )
#define GP2OEN_OEN6_IN                 (0x0   << 6  ) /* IN                       */
#define GP2OEN_OEN6_OUT                (0x1   << 6  ) /* OUT                      */

/* GP2OEN[OEN5] - Direction for port pin */
#define GP2OEN_OEN5_BBA                (*(volatile unsigned long *) 0x420C0C94)
#define GP2OEN_OEN5_MSK                (0x1   << 5  )
#define GP2OEN_OEN5                    (0x1   << 5  )
#define GP2OEN_OEN5_IN                 (0x0   << 5  ) /* IN                       */
#define GP2OEN_OEN5_OUT                (0x1   << 5  ) /* OUT                      */

/* GP2OEN[OEN4] - Direction for port pin */
#define GP2OEN_OEN4_BBA                (*(volatile unsigned long *) 0x420C0C90)
#define GP2OEN_OEN4_MSK                (0x1   << 4  )
#define GP2OEN_OEN4                    (0x1   << 4  )
#define GP2OEN_OEN4_IN                 (0x0   << 4  ) /* IN                       */
#define GP2OEN_OEN4_OUT                (0x1   << 4  ) /* OUT                      */

/* GP2OEN[OEN3] - Direction for port pin */
#define GP2OEN_OEN3_BBA                (*(volatile unsigned long *) 0x420C0C8C)
#define GP2OEN_OEN3_MSK                (0x1   << 3  )
#define GP2OEN_OEN3                    (0x1   << 3  )
#define GP2OEN_OEN3_IN                 (0x0   << 3  ) /* IN                       */
#define GP2OEN_OEN3_OUT                (0x1   << 3  ) /* OUT                      */

/* GP2OEN[OEN2] - Direction for port pin */
#define GP2OEN_OEN2_BBA                (*(volatile unsigned long *) 0x420C0C88)
#define GP2OEN_OEN2_MSK                (0x1   << 2  )
#define GP2OEN_OEN2                    (0x1   << 2  )
#define GP2OEN_OEN2_IN                 (0x0   << 2  ) /* IN                       */
#define GP2OEN_OEN2_OUT                (0x1   << 2  ) /* OUT                      */

/* GP2OEN[OEN1] - Direction for port pin */
#define GP2OEN_OEN1_BBA                (*(volatile unsigned long *) 0x420C0C84)
#define GP2OEN_OEN1_MSK                (0x1   << 1  )
#define GP2OEN_OEN1                    (0x1   << 1  )
#define GP2OEN_OEN1_IN                 (0x0   << 1  ) /* IN                       */
#define GP2OEN_OEN1_OUT                (0x1   << 1  ) /* OUT                      */

/* GP2OEN[OEN0] - Direction for port pin */
#define GP2OEN_OEN0_BBA                (*(volatile unsigned long *) 0x420C0C80)
#define GP2OEN_OEN0_MSK                (0x1   << 0  )
#define GP2OEN_OEN0                    (0x1   << 0  )
#define GP2OEN_OEN0_IN                 (0x0   << 0  ) /* IN                       */
#define GP2OEN_OEN0_OUT                (0x1   << 0  ) /* OUT                      */

/* Reset Value for GP2PUL*/
#define GP2PUL_RVAL                    0xFF 

/* GP2PUL[PUL7] - Pull Up Enable for port pin */
#define GP2PUL_PUL7_BBA                (*(volatile unsigned long *) 0x420C0D1C)
#define GP2PUL_PUL7_MSK                (0x1   << 7  )
#define GP2PUL_PUL7                    (0x1   << 7  )
#define GP2PUL_PUL7_DIS                (0x0   << 7  ) /* DIS                      */
#define GP2PUL_PUL7_EN                 (0x1   << 7  ) /* EN                       */

/* GP2PUL[PUL6] - Pull Up Enable for port pin */
#define GP2PUL_PUL6_BBA                (*(volatile unsigned long *) 0x420C0D18)
#define GP2PUL_PUL6_MSK                (0x1   << 6  )
#define GP2PUL_PUL6                    (0x1   << 6  )
#define GP2PUL_PUL6_DIS                (0x0   << 6  ) /* DIS                      */
#define GP2PUL_PUL6_EN                 (0x1   << 6  ) /* EN                       */

/* GP2PUL[PUL5] - Pull Up Enable for port pin */
#define GP2PUL_PUL5_BBA                (*(volatile unsigned long *) 0x420C0D14)
#define GP2PUL_PUL5_MSK                (0x1   << 5  )
#define GP2PUL_PUL5                    (0x1   << 5  )
#define GP2PUL_PUL5_DIS                (0x0   << 5  ) /* DIS                      */
#define GP2PUL_PUL5_EN                 (0x1   << 5  ) /* EN                       */

/* GP2PUL[PUL4] - Pull Up Enable for port pin */
#define GP2PUL_PUL4_BBA                (*(volatile unsigned long *) 0x420C0D10)
#define GP2PUL_PUL4_MSK                (0x1   << 4  )
#define GP2PUL_PUL4                    (0x1   << 4  )
#define GP2PUL_PUL4_DIS                (0x0   << 4  ) /* DIS                      */
#define GP2PUL_PUL4_EN                 (0x1   << 4  ) /* EN                       */

/* GP2PUL[PUL3] - Pull Up Enable for port pin */
#define GP2PUL_PUL3_BBA                (*(volatile unsigned long *) 0x420C0D0C)
#define GP2PUL_PUL3_MSK                (0x1   << 3  )
#define GP2PUL_PUL3                    (0x1   << 3  )
#define GP2PUL_PUL3_DIS                (0x0   << 3  ) /* DIS                      */
#define GP2PUL_PUL3_EN                 (0x1   << 3  ) /* EN                       */

/* GP2PUL[PUL2] - Pull Up Enable for port pin */
#define GP2PUL_PUL2_BBA                (*(volatile unsigned long *) 0x420C0D08)
#define GP2PUL_PUL2_MSK                (0x1   << 2  )
#define GP2PUL_PUL2                    (0x1   << 2  )
#define GP2PUL_PUL2_DIS                (0x0   << 2  ) /* DIS                      */
#define GP2PUL_PUL2_EN                 (0x1   << 2  ) /* EN                       */

/* GP2PUL[PUL1] - Pull Up Enable for port pin */
#define GP2PUL_PUL1_BBA                (*(volatile unsigned long *) 0x420C0D04)
#define GP2PUL_PUL1_MSK                (0x1   << 1  )
#define GP2PUL_PUL1                    (0x1   << 1  )
#define GP2PUL_PUL1_DIS                (0x0   << 1  ) /* DIS                      */
#define GP2PUL_PUL1_EN                 (0x1   << 1  ) /* EN                       */

/* GP2PUL[PUL0] - Pull Up Enable for port pin */
#define GP2PUL_PUL0_BBA                (*(volatile unsigned long *) 0x420C0D00)
#define GP2PUL_PUL0_MSK                (0x1   << 0  )
#define GP2PUL_PUL0                    (0x1   << 0  )
#define GP2PUL_PUL0_DIS                (0x0   << 0  ) /* DIS                      */
#define GP2PUL_PUL0_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for GP2OCE*/
#define GP2OCE_RVAL                    0x0 

/* GP2OCE[OCE7] - open circuit Enable for port pin */
#define GP2OCE_OCE7_BBA                (*(volatile unsigned long *) 0x420C0D9C)
#define GP2OCE_OCE7_MSK                (0x1   << 7  )
#define GP2OCE_OCE7                    (0x1   << 7  )
#define GP2OCE_OCE7_DIS                (0x0   << 7  ) /* DIS                      */
#define GP2OCE_OCE7_EN                 (0x1   << 7  ) /* EN                       */

/* GP2OCE[OCE6] - open circuit Enable for port pin */
#define GP2OCE_OCE6_BBA                (*(volatile unsigned long *) 0x420C0D98)
#define GP2OCE_OCE6_MSK                (0x1   << 6  )
#define GP2OCE_OCE6                    (0x1   << 6  )
#define GP2OCE_OCE6_DIS                (0x0   << 6  ) /* DIS                      */
#define GP2OCE_OCE6_EN                 (0x1   << 6  ) /* EN                       */

/* GP2OCE[OCE5] - open circuit Enable for port pin */
#define GP2OCE_OCE5_BBA                (*(volatile unsigned long *) 0x420C0D94)
#define GP2OCE_OCE5_MSK                (0x1   << 5  )
#define GP2OCE_OCE5                    (0x1   << 5  )
#define GP2OCE_OCE5_DIS                (0x0   << 5  ) /* DIS                      */
#define GP2OCE_OCE5_EN                 (0x1   << 5  ) /* EN                       */

/* GP2OCE[OCE4] - open circuit Enable for port pin */
#define GP2OCE_OCE4_BBA                (*(volatile unsigned long *) 0x420C0D90)
#define GP2OCE_OCE4_MSK                (0x1   << 4  )
#define GP2OCE_OCE4                    (0x1   << 4  )
#define GP2OCE_OCE4_DIS                (0x0   << 4  ) /* DIS                      */
#define GP2OCE_OCE4_EN                 (0x1   << 4  ) /* EN                       */

/* GP2OCE[OCE3] - open circuit Enable for port pin */
#define GP2OCE_OCE3_BBA                (*(volatile unsigned long *) 0x420C0D8C)
#define GP2OCE_OCE3_MSK                (0x1   << 3  )
#define GP2OCE_OCE3                    (0x1   << 3  )
#define GP2OCE_OCE3_DIS                (0x0   << 3  ) /* DIS                      */
#define GP2OCE_OCE3_EN                 (0x1   << 3  ) /* EN                       */

/* GP2OCE[OCE2] - open circuit Enable for port pin */
#define GP2OCE_OCE2_BBA                (*(volatile unsigned long *) 0x420C0D88)
#define GP2OCE_OCE2_MSK                (0x1   << 2  )
#define GP2OCE_OCE2                    (0x1   << 2  )
#define GP2OCE_OCE2_DIS                (0x0   << 2  ) /* DIS                      */
#define GP2OCE_OCE2_EN                 (0x1   << 2  ) /* EN                       */

/* GP2OCE[OCE1] - open circuit Enable for port pin */
#define GP2OCE_OCE1_BBA                (*(volatile unsigned long *) 0x420C0D84)
#define GP2OCE_OCE1_MSK                (0x1   << 1  )
#define GP2OCE_OCE1                    (0x1   << 1  )
#define GP2OCE_OCE1_DIS                (0x0   << 1  ) /* DIS                      */
#define GP2OCE_OCE1_EN                 (0x1   << 1  ) /* EN                       */

/* GP2OCE[OCE0] - open circuit Enable for port pin */
#define GP2OCE_OCE0_BBA                (*(volatile unsigned long *) 0x420C0D80)
#define GP2OCE_OCE0_MSK                (0x1   << 0  )
#define GP2OCE_OCE0                    (0x1   << 0  )
#define GP2OCE_OCE0_DIS                (0x0   << 0  ) /* DIS                      */
#define GP2OCE_OCE0_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for GP2IN*/
#define GP2IN_RVAL                     0x0 

/* GP2IN[IN7] - Input for port pin */
#define GP2IN_IN7_BBA                  (*(volatile unsigned long *) 0x420C0E9C)
#define GP2IN_IN7_MSK                  (0x1   << 7  )
#define GP2IN_IN7                      (0x1   << 7  )
#define GP2IN_IN7_LOW                  (0x0   << 7  ) /* LOW                      */
#define GP2IN_IN7_HIGH                 (0x1   << 7  ) /* HIGH                     */

/* GP2IN[IN6] - Input for port pin */
#define GP2IN_IN6_BBA                  (*(volatile unsigned long *) 0x420C0E98)
#define GP2IN_IN6_MSK                  (0x1   << 6  )
#define GP2IN_IN6                      (0x1   << 6  )
#define GP2IN_IN6_LOW                  (0x0   << 6  ) /* LOW                      */
#define GP2IN_IN6_HIGH                 (0x1   << 6  ) /* HIGH                     */

/* GP2IN[IN5] - Input for port pin */
#define GP2IN_IN5_BBA                  (*(volatile unsigned long *) 0x420C0E94)
#define GP2IN_IN5_MSK                  (0x1   << 5  )
#define GP2IN_IN5                      (0x1   << 5  )
#define GP2IN_IN5_LOW                  (0x0   << 5  ) /* LOW                      */
#define GP2IN_IN5_HIGH                 (0x1   << 5  ) /* HIGH                     */

/* GP2IN[IN4] - Input for port pin */
#define GP2IN_IN4_BBA                  (*(volatile unsigned long *) 0x420C0E90)
#define GP2IN_IN4_MSK                  (0x1   << 4  )
#define GP2IN_IN4                      (0x1   << 4  )
#define GP2IN_IN4_LOW                  (0x0   << 4  ) /* LOW                      */
#define GP2IN_IN4_HIGH                 (0x1   << 4  ) /* HIGH                     */

/* GP2IN[IN3] - Input for port pin */
#define GP2IN_IN3_BBA                  (*(volatile unsigned long *) 0x420C0E8C)
#define GP2IN_IN3_MSK                  (0x1   << 3  )
#define GP2IN_IN3                      (0x1   << 3  )
#define GP2IN_IN3_LOW                  (0x0   << 3  ) /* LOW                      */
#define GP2IN_IN3_HIGH                 (0x1   << 3  ) /* HIGH                     */

/* GP2IN[IN2] - Input for port pin */
#define GP2IN_IN2_BBA                  (*(volatile unsigned long *) 0x420C0E88)
#define GP2IN_IN2_MSK                  (0x1   << 2  )
#define GP2IN_IN2                      (0x1   << 2  )
#define GP2IN_IN2_LOW                  (0x0   << 2  ) /* LOW                      */
#define GP2IN_IN2_HIGH                 (0x1   << 2  ) /* HIGH                     */

/* GP2IN[IN1] - Input for port pin */
#define GP2IN_IN1_BBA                  (*(volatile unsigned long *) 0x420C0E84)
#define GP2IN_IN1_MSK                  (0x1   << 1  )
#define GP2IN_IN1                      (0x1   << 1  )
#define GP2IN_IN1_LOW                  (0x0   << 1  ) /* LOW                      */
#define GP2IN_IN1_HIGH                 (0x1   << 1  ) /* HIGH                     */

/* GP2IN[IN0] - Input for port pin */
#define GP2IN_IN0_BBA                  (*(volatile unsigned long *) 0x420C0E80)
#define GP2IN_IN0_MSK                  (0x1   << 0  )
#define GP2IN_IN0                      (0x1   << 0  )
#define GP2IN_IN0_LOW                  (0x0   << 0  ) /* LOW                      */
#define GP2IN_IN0_HIGH                 (0x1   << 0  ) /* HIGH                     */

/* Reset Value for GP2OUT*/
#define GP2OUT_RVAL                    0x0 

/* GP2OUT[OUT7] - Output for port pin */
#define GP2OUT_OUT7_BBA                (*(volatile unsigned long *) 0x420C0F1C)
#define GP2OUT_OUT7_MSK                (0x1   << 7  )
#define GP2OUT_OUT7                    (0x1   << 7  )
#define GP2OUT_OUT7_LOW                (0x0   << 7  ) /* LOW                      */
#define GP2OUT_OUT7_HIGH               (0x1   << 7  ) /* HIGH                     */

/* GP2OUT[OUT6] - Output for port pin */
#define GP2OUT_OUT6_BBA                (*(volatile unsigned long *) 0x420C0F18)
#define GP2OUT_OUT6_MSK                (0x1   << 6  )
#define GP2OUT_OUT6                    (0x1   << 6  )
#define GP2OUT_OUT6_LOW                (0x0   << 6  ) /* LOW                      */
#define GP2OUT_OUT6_HIGH               (0x1   << 6  ) /* HIGH                     */

/* GP2OUT[OUT5] - Output for port pin */
#define GP2OUT_OUT5_BBA                (*(volatile unsigned long *) 0x420C0F14)
#define GP2OUT_OUT5_MSK                (0x1   << 5  )
#define GP2OUT_OUT5                    (0x1   << 5  )
#define GP2OUT_OUT5_LOW                (0x0   << 5  ) /* LOW                      */
#define GP2OUT_OUT5_HIGH               (0x1   << 5  ) /* HIGH                     */

/* GP2OUT[OUT4] - Output for port pin */
#define GP2OUT_OUT4_BBA                (*(volatile unsigned long *) 0x420C0F10)
#define GP2OUT_OUT4_MSK                (0x1   << 4  )
#define GP2OUT_OUT4                    (0x1   << 4  )
#define GP2OUT_OUT4_LOW                (0x0   << 4  ) /* LOW                      */
#define GP2OUT_OUT4_HIGH               (0x1   << 4  ) /* HIGH                     */

/* GP2OUT[OUT3] - Output for port pin */
#define GP2OUT_OUT3_BBA                (*(volatile unsigned long *) 0x420C0F0C)
#define GP2OUT_OUT3_MSK                (0x1   << 3  )
#define GP2OUT_OUT3                    (0x1   << 3  )
#define GP2OUT_OUT3_LOW                (0x0   << 3  ) /* LOW                      */
#define GP2OUT_OUT3_HIGH               (0x1   << 3  ) /* HIGH                     */

/* GP2OUT[OUT2] - Output for port pin */
#define GP2OUT_OUT2_BBA                (*(volatile unsigned long *) 0x420C0F08)
#define GP2OUT_OUT2_MSK                (0x1   << 2  )
#define GP2OUT_OUT2                    (0x1   << 2  )
#define GP2OUT_OUT2_LOW                (0x0   << 2  ) /* LOW                      */
#define GP2OUT_OUT2_HIGH               (0x1   << 2  ) /* HIGH                     */

/* GP2OUT[OUT1] - Output for port pin */
#define GP2OUT_OUT1_BBA                (*(volatile unsigned long *) 0x420C0F04)
#define GP2OUT_OUT1_MSK                (0x1   << 1  )
#define GP2OUT_OUT1                    (0x1   << 1  )
#define GP2OUT_OUT1_LOW                (0x0   << 1  ) /* LOW                      */
#define GP2OUT_OUT1_HIGH               (0x1   << 1  ) /* HIGH                     */

/* GP2OUT[OUT0] - Output for port pin */
#define GP2OUT_OUT0_BBA                (*(volatile unsigned long *) 0x420C0F00)
#define GP2OUT_OUT0_MSK                (0x1   << 0  )
#define GP2OUT_OUT0                    (0x1   << 0  )
#define GP2OUT_OUT0_LOW                (0x0   << 0  ) /* LOW                      */
#define GP2OUT_OUT0_HIGH               (0x1   << 0  ) /* HIGH                     */

/* Reset Value for GP2SET*/
#define GP2SET_RVAL                    0x0 

/* GP2SET[SET7] - Set Output High for port pin */
#define GP2SET_SET7_BBA                (*(volatile unsigned long *) 0x420C0F9C)
#define GP2SET_SET7_MSK                (0x1   << 7  )
#define GP2SET_SET7                    (0x1   << 7  )
#define GP2SET_SET7_SET                (0x1   << 7  ) /* SET                      */

/* GP2SET[SET6] - Set Output High for port pin */
#define GP2SET_SET6_BBA                (*(volatile unsigned long *) 0x420C0F98)
#define GP2SET_SET6_MSK                (0x1   << 6  )
#define GP2SET_SET6                    (0x1   << 6  )
#define GP2SET_SET6_SET                (0x1   << 6  ) /* SET                      */

/* GP2SET[SET5] - Set Output High for port pin */
#define GP2SET_SET5_BBA                (*(volatile unsigned long *) 0x420C0F94)
#define GP2SET_SET5_MSK                (0x1   << 5  )
#define GP2SET_SET5                    (0x1   << 5  )
#define GP2SET_SET5_SET                (0x1   << 5  ) /* SET                      */

/* GP2SET[SET4] - Set Output High for port pin */
#define GP2SET_SET4_BBA                (*(volatile unsigned long *) 0x420C0F90)
#define GP2SET_SET4_MSK                (0x1   << 4  )
#define GP2SET_SET4                    (0x1   << 4  )
#define GP2SET_SET4_SET                (0x1   << 4  ) /* SET                      */

/* GP2SET[SET3] - Set Output High for port pin */
#define GP2SET_SET3_BBA                (*(volatile unsigned long *) 0x420C0F8C)
#define GP2SET_SET3_MSK                (0x1   << 3  )
#define GP2SET_SET3                    (0x1   << 3  )
#define GP2SET_SET3_SET                (0x1   << 3  ) /* SET                      */

/* GP2SET[SET2] - Set Output High for port pin */
#define GP2SET_SET2_BBA                (*(volatile unsigned long *) 0x420C0F88)
#define GP2SET_SET2_MSK                (0x1   << 2  )
#define GP2SET_SET2                    (0x1   << 2  )
#define GP2SET_SET2_SET                (0x1   << 2  ) /* SET                      */

/* GP2SET[SET1] - Set Output High for port pin */
#define GP2SET_SET1_BBA                (*(volatile unsigned long *) 0x420C0F84)
#define GP2SET_SET1_MSK                (0x1   << 1  )
#define GP2SET_SET1                    (0x1   << 1  )
#define GP2SET_SET1_SET                (0x1   << 1  ) /* SET                      */

/* GP2SET[SET0] - Set Output High for port pin */
#define GP2SET_SET0_BBA                (*(volatile unsigned long *) 0x420C0F80)
#define GP2SET_SET0_MSK                (0x1   << 0  )
#define GP2SET_SET0                    (0x1   << 0  )
#define GP2SET_SET0_SET                (0x1   << 0  ) /* SET                      */

/* Reset Value for GP2CLR*/
#define GP2CLR_RVAL                    0x0 

/* GP2CLR[CLR7] - Set Output Low for port pin */
#define GP2CLR_CLR7_BBA                (*(volatile unsigned long *) 0x420C101C)
#define GP2CLR_CLR7_MSK                (0x1   << 7  )
#define GP2CLR_CLR7                    (0x1   << 7  )
#define GP2CLR_CLR7_CLR                (0x1   << 7  ) /* CLR                      */

/* GP2CLR[CLR6] - Set Output Low for port pin */
#define GP2CLR_CLR6_BBA                (*(volatile unsigned long *) 0x420C1018)
#define GP2CLR_CLR6_MSK                (0x1   << 6  )
#define GP2CLR_CLR6                    (0x1   << 6  )
#define GP2CLR_CLR6_CLR                (0x1   << 6  ) /* CLR                      */

/* GP2CLR[CLR5] - Set Output Low for port pin */
#define GP2CLR_CLR5_BBA                (*(volatile unsigned long *) 0x420C1014)
#define GP2CLR_CLR5_MSK                (0x1   << 5  )
#define GP2CLR_CLR5                    (0x1   << 5  )
#define GP2CLR_CLR5_CLR                (0x1   << 5  ) /* CLR                      */

/* GP2CLR[CLR4] - Set Output Low for port pin */
#define GP2CLR_CLR4_BBA                (*(volatile unsigned long *) 0x420C1010)
#define GP2CLR_CLR4_MSK                (0x1   << 4  )
#define GP2CLR_CLR4                    (0x1   << 4  )
#define GP2CLR_CLR4_CLR                (0x1   << 4  ) /* CLR                      */

/* GP2CLR[CLR3] - Set Output Low for port pin */
#define GP2CLR_CLR3_BBA                (*(volatile unsigned long *) 0x420C100C)
#define GP2CLR_CLR3_MSK                (0x1   << 3  )
#define GP2CLR_CLR3                    (0x1   << 3  )
#define GP2CLR_CLR3_CLR                (0x1   << 3  ) /* CLR                      */

/* GP2CLR[CLR2] - Set Output Low for port pin */
#define GP2CLR_CLR2_BBA                (*(volatile unsigned long *) 0x420C1008)
#define GP2CLR_CLR2_MSK                (0x1   << 2  )
#define GP2CLR_CLR2                    (0x1   << 2  )
#define GP2CLR_CLR2_CLR                (0x1   << 2  ) /* CLR                      */

/* GP2CLR[CLR1] - Set Output Low for port pin */
#define GP2CLR_CLR1_BBA                (*(volatile unsigned long *) 0x420C1004)
#define GP2CLR_CLR1_MSK                (0x1   << 1  )
#define GP2CLR_CLR1                    (0x1   << 1  )
#define GP2CLR_CLR1_CLR                (0x1   << 1  ) /* CLR                      */

/* GP2CLR[CLR0] - Set Output Low for port pin */
#define GP2CLR_CLR0_BBA                (*(volatile unsigned long *) 0x420C1000)
#define GP2CLR_CLR0_MSK                (0x1   << 0  )
#define GP2CLR_CLR0                    (0x1   << 0  )
#define GP2CLR_CLR0_CLR                (0x1   << 0  ) /* CLR                      */

/* Reset Value for GP2TGL*/
#define GP2TGL_RVAL                    0x0 

/* GP2TGL[TGL7] - Toggle Output for port pin */
#define GP2TGL_TGL7_BBA                (*(volatile unsigned long *) 0x420C109C)
#define GP2TGL_TGL7_MSK                (0x1   << 7  )
#define GP2TGL_TGL7                    (0x1   << 7  )
#define GP2TGL_TGL7_TGL                (0x1   << 7  ) /* TGL                      */

/* GP2TGL[TGL6] - Toggle Output for port pin */
#define GP2TGL_TGL6_BBA                (*(volatile unsigned long *) 0x420C1098)
#define GP2TGL_TGL6_MSK                (0x1   << 6  )
#define GP2TGL_TGL6                    (0x1   << 6  )
#define GP2TGL_TGL6_TGL                (0x1   << 6  ) /* TGL                      */

/* GP2TGL[TGL5] - Toggle Output for port pin */
#define GP2TGL_TGL5_BBA                (*(volatile unsigned long *) 0x420C1094)
#define GP2TGL_TGL5_MSK                (0x1   << 5  )
#define GP2TGL_TGL5                    (0x1   << 5  )
#define GP2TGL_TGL5_TGL                (0x1   << 5  ) /* TGL                      */

/* GP2TGL[TGL4] - Toggle Output for port pin */
#define GP2TGL_TGL4_BBA                (*(volatile unsigned long *) 0x420C1090)
#define GP2TGL_TGL4_MSK                (0x1   << 4  )
#define GP2TGL_TGL4                    (0x1   << 4  )
#define GP2TGL_TGL4_TGL                (0x1   << 4  ) /* TGL                      */

/* GP2TGL[TGL3] - Toggle Output for port pin */
#define GP2TGL_TGL3_BBA                (*(volatile unsigned long *) 0x420C108C)
#define GP2TGL_TGL3_MSK                (0x1   << 3  )
#define GP2TGL_TGL3                    (0x1   << 3  )
#define GP2TGL_TGL3_TGL                (0x1   << 3  ) /* TGL                      */

/* GP2TGL[TGL2] - Toggle Output for port pin */
#define GP2TGL_TGL2_BBA                (*(volatile unsigned long *) 0x420C1088)
#define GP2TGL_TGL2_MSK                (0x1   << 2  )
#define GP2TGL_TGL2                    (0x1   << 2  )
#define GP2TGL_TGL2_TGL                (0x1   << 2  ) /* TGL                      */

/* GP2TGL[TGL1] - Toggle Output for port pin */
#define GP2TGL_TGL1_BBA                (*(volatile unsigned long *) 0x420C1084)
#define GP2TGL_TGL1_MSK                (0x1   << 1  )
#define GP2TGL_TGL1                    (0x1   << 1  )
#define GP2TGL_TGL1_TGL                (0x1   << 1  ) /* TGL                      */

/* GP2TGL[TGL0] - Toggle Output for port pin */
#define GP2TGL_TGL0_BBA                (*(volatile unsigned long *) 0x420C1080)
#define GP2TGL_TGL0_MSK                (0x1   << 0  )
#define GP2TGL_TGL0                    (0x1   << 0  )
#define GP2TGL_TGL0_TGL                (0x1   << 0  ) /* TGL                      */
// ------------------------------------------------------------------------------------------------
// -----                                        ANA                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Analog Control (pADI_ANA)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_ANA Structure                     */
  __I  uint32_t  RESERVED0[12];
  __IO uint16_t  REFCTRL;                   /*!< Internal Reference Control register   */
  __I  uint16_t  RESERVED1[7];
  __IO uint8_t   IEXCCON;                   /*!< Controls the on-chip Excitation Current Sources */
  __I  uint8_t   RESERVED2[3];
  __IO uint8_t   IEXCDAT;                   /*!< Sets the output current setting for both Excitation Current sources */
} ADI_ANA_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          REFCTRL                                    (*(volatile unsigned short int *) 0x40008840)
#define          IEXCCON                                    (*(volatile unsigned char      *) 0x40008850)
#define          IEXCDAT                                    (*(volatile unsigned char      *) 0x40008854)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for REFCTRL*/
#define REFCTRL_RVAL                   0x0 

/* REFCTRL[REFPD] - Power down reference */
#define REFCTRL_REFPD_BBA              (*(volatile unsigned long *) 0x42110800)
#define REFCTRL_REFPD_MSK              (0x1   << 0  )
#define REFCTRL_REFPD                  (0x1   << 0  )
#define REFCTRL_REFPD_DIS              (0x0   << 0  ) /* DIS                      */
#define REFCTRL_REFPD_EN               (0x1   << 0  ) /* EN                       */

/* Reset Value for IEXCCON*/
#define IEXCCON_RVAL                   0xC0 

/* IEXCCON[PD] - IEXC Power down- bits */
#define IEXCCON_PD_BBA                 (*(volatile unsigned long *) 0x42110A1C)
#define IEXCCON_PD_MSK                 (0x1   << 7  )
#define IEXCCON_PD                     (0x1   << 7  )
#define IEXCCON_PD_En                  (0x1   << 7  ) /* En                       */
#define IEXCCON_PD_off                 (0x0   << 7  ) /* off                      */

/* IEXCCON[REFSEL] - IREF Source- bits */
#define IEXCCON_REFSEL_BBA             (*(volatile unsigned long *) 0x42110A18)
#define IEXCCON_REFSEL_MSK             (0x1   << 6  )
#define IEXCCON_REFSEL                 (0x1   << 6  )
#define IEXCCON_REFSEL_Ext             (0x0   << 6  ) /* Ext                      */
#define IEXCCON_REFSEL_Int             (0x1   << 6  ) /* Int                      */

/* IEXCCON[IPSEL1] - Select IEXC1 pin AIN- bits */
#define IEXCCON_IPSEL1_MSK             (0x7   << 3  )
#define IEXCCON_IPSEL1_Off             (0x0   << 3  ) /* Off                      */
#define IEXCCON_IPSEL1_AIN4            (0x4   << 3  ) /* AIN4                     */
#define IEXCCON_IPSEL1_AIN5            (0x5   << 3  ) /* AIN5                     */
#define IEXCCON_IPSEL1_AIN6            (0x6   << 3  ) /* AIN6                     */
#define IEXCCON_IPSEL1_AIN7            (0x7   << 3  ) /* AIN7                     */

/* IEXCCON[IPSEL0] - Select IEXC0 pin AIN- bits */
#define IEXCCON_IPSEL0_MSK             (0x7   << 0  )
#define IEXCCON_IPSEL0_Off             (0x0   << 0  ) /* Off                      */
#define IEXCCON_IPSEL0_AIN4            (0x4   << 0  ) /* AIN4                     */
#define IEXCCON_IPSEL0_AIN5            (0x5   << 0  ) /* AIN5                     */
#define IEXCCON_IPSEL0_AIN6            (0x6   << 0  ) /* AIN6                     */
#define IEXCCON_IPSEL0_AIN7            (0x7   << 0  ) /* AIN7                     */

/* Reset Value for IEXCDAT*/
#define IEXCDAT_RVAL                   0x6 

/* IEXCDAT[IDAT] - Output Current- bits */
#define IEXCDAT_IDAT_MSK               (0x1F  << 1  )
#define IEXCDAT_IDAT_0uA               (0x0   << 1  ) /* 0uA                      */
#define IEXCDAT_IDAT_50uA              (0x4   << 1  ) /* 50uA                     */
#define IEXCDAT_IDAT_100uA             (0x5   << 1  ) /* 100uA                    */
#define IEXCDAT_IDAT_150uA             (0x6   << 1  ) /* 150uA                    */
#define IEXCDAT_IDAT_200uA             (0x7   << 1  ) /* 200uA                    */
#define IEXCDAT_IDAT_250uA             (0x14  << 1  ) /* 250uA                    */
#define IEXCDAT_IDAT_300uA             (0xA   << 1  ) /* 300uA                    */
#define IEXCDAT_IDAT_400uA             (0xB   << 1  ) /* 400uA                    */
#define IEXCDAT_IDAT_450uA             (0xE   << 1  ) /* 450uA                    */
#define IEXCDAT_IDAT_500uA             (0x15  << 1  ) /* 500uA                    */
#define IEXCDAT_IDAT_600uA             (0xF   << 1  ) /* 600uA                    */
#define IEXCDAT_IDAT_750uA             (0x16  << 1  ) /* 750uA                    */
#define IEXCDAT_IDAT_800uA             (0x13  << 1  ) /* 800uA                    */
#define IEXCDAT_IDAT_1mA               (0x1F  << 1  ) /* 1mA                      */

/* IEXCDAT[IDAT0] - 10uA Enable */
#define IEXCDAT_IDAT0_BBA              (*(volatile unsigned long *) 0x42110A80)
#define IEXCDAT_IDAT0_MSK              (0x1   << 0  )
#define IEXCDAT_IDAT0                  (0x1   << 0  )
#define IEXCDAT_IDAT0_DIS              (0x0   << 0  ) /* DIS                      */
#define IEXCDAT_IDAT0_EN               (0x1   << 0  ) /* EN                       */
// ------------------------------------------------------------------------------------------------
// -----                                        DMA                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Direct Memory Access (pADI_DMA)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_DMA Structure                     */
  __IO uint32_t  DMASTA;                    /*!< Returns the status of the controller when not in the reset state. */
  __IO uint32_t  DMACFG;                    /*!< Configuraton                          */
  __IO uint32_t  DMAPDBPTR;                 /*!< Channel primary control database pointer */
  __IO uint32_t  DMAADBPTR;                 /*!< Channel alt control database pointer  */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  DMASWREQ;                  /*!< Channel Software Request              */
  __I  uint32_t  RESERVED1[2];
  __IO uint32_t  DMARMSKSET;                /*!< Channel Request Mask Set              */
  __IO uint32_t  DMARMSKCLR;                /*!< Channel Request Mask Clear            */
  __IO uint32_t  DMAENSET;                  /*!< Channel Enable Set                    */
  __IO uint32_t  DMAENCLR;                  /*!< Channel Enable Clear                  */
  __IO uint32_t  DMAALTSET;                 /*!< Channel Primary-Alternate Set         */
  __IO uint32_t  DMAALTCLR;                 /*!< Channel Primary-Alternate Clear       */
  __IO uint32_t  DMAPRISET;                 /*!< Channel Priority Set                  */
  __IO uint32_t  DMAPRICLR;                 /*!< Channel Priority Clear                */
  __I  uint32_t  RESERVED2[3];
  __IO uint32_t  DMAERRCLR;                 /*!< Bus Error Clear                       */
  __I  uint32_t  RESERVED3[492];
  __IO uint32_t  DMABSSET;                  /*!< DMA channel bytes swap enable set     */
  __IO uint32_t  DMABSCLR;                  /*!< DMA channel bytes swap enable clear   */
  __I  uint32_t  RESERVED4[66];
  __IO uint32_t  DMAGETNMINUS1;             /*!< Request n_minus_1 register update   */
  __IO uint32_t  DMANMINUS1;                /*!< Current n_minus_1 value   */
} ADI_DMA_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          DMASTA                                     (*(volatile unsigned long      *) 0x40010000)
#define          DMACFG                                     (*(volatile unsigned long      *) 0x40010004)
#define          DMAPDBPTR                                  (*(volatile unsigned long      *) 0x40010008)
#define          DMAADBPTR                                  (*(volatile unsigned long      *) 0x4001000C)
#define          DMASWREQ                                   (*(volatile unsigned long      *) 0x40010014)
#define          DMARMSKSET                                 (*(volatile unsigned long      *) 0x40010020)
#define          DMARMSKCLR                                 (*(volatile unsigned long      *) 0x40010024)
#define          DMAENSET                                   (*(volatile unsigned long      *) 0x40010028)
#define          DMAENCLR                                   (*(volatile unsigned long      *) 0x4001002C)
#define          DMAALTSET                                  (*(volatile unsigned long      *) 0x40010030)
#define          DMAALTCLR                                  (*(volatile unsigned long      *) 0x40010034)
#define          DMAPRISET                                  (*(volatile unsigned long      *) 0x40010038)
#define          DMAPRICLR                                  (*(volatile unsigned long      *) 0x4001003C)
#define          DMAERRCLR                                  (*(volatile unsigned long      *) 0x4001004C)
#define          DMABSSET                                   (*(volatile unsigned long      *) 0x40010800)
#define          DMABSCLR                                   (*(volatile unsigned long      *) 0x40010804)
#define          DMAGETNMINUS1                              (*(volatile unsigned long      *) 0x40010910)
#define          DMANMINUS1                                 (*(volatile unsigned long      *) 0x40010914)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for DMASTA*/
#define DMASTA_RVAL                    0xB0000 

/* DMASTA[CHNLSMINUS1] - Number of available DMA channels minus one. */
#define DMASTA_CHNLSMINUS1_MSK         (0x1F  << 16 )
#define DMASTA_CHNLSMINUS1_FOURTEENCHNLS (0xD   << 16 ) /* FOURTEENCHNLS - Controller configured to use 14 DMA channels */
#define DMASTA_CHNLSMINUS1_TWELVECHNLS (0xB   << 16 ) /* TWELVECHNLS - Controller configured to use 12 DMA channels */

/* DMASTA[STATE] - Current state of the control state machine. */
#define DMASTA_STATE_MSK               (0xF   << 4  )
#define DMASTA_STATE_IDLE              (0x0   << 4  ) /* IDLE - Idle              */
#define DMASTA_STATE_RDCHNLDATA        (0x1   << 4  ) /* RDCHNLDATA - Reading channel controller data */
#define DMASTA_STATE_RDSRCENDPTR       (0x2   << 4  ) /* RDSRCENDPTR - Reading source data end pointer */
#define DMASTA_STATE_RDDSTENDPTR       (0x3   << 4  ) /* RDDSTENDPTR - Reading destination data end pointer */
#define DMASTA_STATE_RDSRCDATA         (0x4   << 4  ) /* RDSRCDATA - Reading source data */
#define DMASTA_STATE_WRDSTDATA         (0x5   << 4  ) /* WRDSTDATA - Writing destination data */
#define DMASTA_STATE_WAITDMAREQCLR     (0x6   << 4  ) /* WAITDMAREQCLR - Waiting for DMA request to clear */
#define DMASTA_STATE_WRCHNLDATA        (0x7   << 4  ) /* WRCHNLDATA - Writing channel controller data */
#define DMASTA_STATE_STALLED           (0x8   << 4  ) /* STALLED - Stalled        */
#define DMASTA_STATE_DONE              (0x9   << 4  ) /* DONE - Done              */
#define DMASTA_STATE_SCATRGATHR        (0xA   << 4  ) /* SCATRGATHR - Peripheral scatter-gather transition */

/* DMASTA[ENABLE] - Master DMA controller enable status. */
#define DMASTA_ENABLE_BBA              (*(volatile unsigned long *) 0x42200000)
#define DMASTA_ENABLE_MSK              (0x1   << 0  )
#define DMASTA_ENABLE                  (0x1   << 0  )
#define DMASTA_ENABLE_CLR              (0x0   << 0  ) /* CLR                      */
#define DMASTA_ENABLE_SET              (0x1   << 0  ) /* SET                      */

/* Reset Value for DMACFG*/
#define DMACFG_RVAL                    0x0 

/* DMACFG[ENABLE] - Master DMA controller enable */
#define DMACFG_ENABLE_BBA              (*(volatile unsigned long *) 0x42200080)
#define DMACFG_ENABLE_MSK              (0x1   << 0  )
#define DMACFG_ENABLE                  (0x1   << 0  )
#define DMACFG_ENABLE_DIS              (0x0   << 0  ) /* DIS                      */
#define DMACFG_ENABLE_EN               (0x1   << 0  ) /* EN                       */

/* Reset Value for DMAPDBPTR*/
#define DMAPDBPTR_RVAL                 0x0 

/* DMAPDBPTR[CTRLBASEPTR] - Pointer to the base address of the primary data structure */
#define DMAPDBPTR_CTRLBASEPTR_MSK      (0xFFFFFFFF << 0  )

/* Reset Value for DMAADBPTR*/
#define DMAADBPTR_RVAL                 0x100 

/* DMAADBPTR[ALTCBPTR] - Pointer to the base address of the alternate data structure */
#define DMAADBPTR_ALTCBPTR_MSK         (0xFFFFFFFF << 0  )

/* Reset Value for DMASWREQ*/
#define DMASWREQ_RVAL                  0x0 
/* DMASWREQ[UART1RX] - DMA UART1 RX */
#define DMASWREQ_UART1RX_BBA             (*(volatile unsigned long *) 0x422002BC)
#define DMASWREQ_UART1RX_MSK             (0x1   << 15 )
#define DMASWREQ_UART1RX                 (0x1   << 15 )
#define DMASWREQ_UART1RX_DIS             (0x0   << 15 ) /* DIS                      */
#define DMASWREQ_UART1RX_EN              (0x1   << 15 ) /* EN                       */
/* DMASWREQ[UART1TX] - DMA UART1 TX */
#define DMASWREQ_UART1TX_BBA             (*(volatile unsigned long *) 0x422002B8)
#define DMASWREQ_UART1TX_MSK             (0x1   << 14 )
#define DMASWREQ_UART1TX                 (0x1   << 14 )
#define DMASWREQ_UART1TX_DIS             (0x0   << 14 ) /* DIS                      */
#define DMASWREQ_UART1TX_EN              (0x1   << 14 ) /* EN                       */
/* DMASWREQ[SPI0RX] - DMA SPI 0 RX */
#define DMASWREQ_SPI0RX_BBA             (*(volatile unsigned long *) 0x422002B4)
#define DMASWREQ_SPI0RX_MSK             (0x1   << 13 )
#define DMASWREQ_SPI0RX                 (0x1   << 13 )
#define DMASWREQ_SPI0RX_DIS             (0x0   << 13 ) /* DIS                      */
#define DMASWREQ_SPI0RX_EN              (0x1   << 13 ) /* EN                       */
/* DMASWREQ[SPI0TX] - DMA SPI 0 TX */
#define DMASWREQ_SPI0TX_BBA             (*(volatile unsigned long *) 0x422002B0)
#define DMASWREQ_SPI0TX_MSK             (0x1   << 12 )
#define DMASWREQ_SPI0TX                 (0x1   << 12 )
#define DMASWREQ_SPI0TX_DIS             (0x0   << 12 ) /* DIS                      */
#define DMASWREQ_SPI0TX_EN              (0x1   << 12 ) /* EN                       */
/* DMASWREQ[SINC2] - SINC2 Output Step detection */
#define DMASWREQ_SINC2_BBA             (*(volatile unsigned long *) 0x422002AC)
#define DMASWREQ_SINC2_MSK             (0x1   << 11 )
#define DMASWREQ_SINC2                 (0x1   << 11 )
#define DMASWREQ_SINC2_DIS             (0x0   << 11 ) /* DIS                      */
#define DMASWREQ_SINC2_EN              (0x1   << 11 ) /* EN                       */

/* DMASWREQ[ADC1] - ADC1 */
#define DMASWREQ_ADC1_BBA              (*(volatile unsigned long *) 0x422002A8)
#define DMASWREQ_ADC1_MSK              (0x1   << 10 )
#define DMASWREQ_ADC1                  (0x1   << 10 )
#define DMASWREQ_ADC1_DIS              (0x0   << 10 ) /* DIS                      */
#define DMASWREQ_ADC1_EN               (0x1   << 10 ) /* EN                       */

/* DMASWREQ[DAC] - DAC DMA Output */
#define DMASWREQ_DAC_BBA               (*(volatile unsigned long *) 0x422002A0)
#define DMASWREQ_DAC_MSK               (0x1   << 8  )
#define DMASWREQ_DAC                   (0x1   << 8  )
#define DMASWREQ_DAC_DIS               (0x0   << 8  ) /* DIS                      */
#define DMASWREQ_DAC_EN                (0x1   << 8  ) /* EN                       */

/* DMASWREQ[I2CMRX] - DMA I2C Master RX */
#define DMASWREQ_I2CMRX_BBA            (*(volatile unsigned long *) 0x4220029C)
#define DMASWREQ_I2CMRX_MSK            (0x1   << 7  )
#define DMASWREQ_I2CMRX                (0x1   << 7  )
#define DMASWREQ_I2CMRX_DIS            (0x0   << 7  ) /* DIS                      */
#define DMASWREQ_I2CMRX_EN             (0x1   << 7  ) /* EN                       */

/* DMASWREQ[I2CMTX] - DMA I2C Master TX */
#define DMASWREQ_I2CMTX_BBA            (*(volatile unsigned long *) 0x42200298)
#define DMASWREQ_I2CMTX_MSK            (0x1   << 6  )
#define DMASWREQ_I2CMTX                (0x1   << 6  )
#define DMASWREQ_I2CMTX_DIS            (0x0   << 6  ) /* DIS                      */
#define DMASWREQ_I2CMTX_EN             (0x1   << 6  ) /* EN                       */

/* DMASWREQ[I2CSRX] - DMA I2C Slave RX */
#define DMASWREQ_I2CSRX_BBA            (*(volatile unsigned long *) 0x42200294)
#define DMASWREQ_I2CSRX_MSK            (0x1   << 5  )
#define DMASWREQ_I2CSRX                (0x1   << 5  )
#define DMASWREQ_I2CSRX_DIS            (0x0   << 5  ) /* DIS                      */
#define DMASWREQ_I2CSRX_EN             (0x1   << 5  ) /* EN                       */

/* DMASWREQ[I2CSTX] - DMA I2C Slave TX */
#define DMASWREQ_I2CSTX_BBA            (*(volatile unsigned long *) 0x42200290)
#define DMASWREQ_I2CSTX_MSK            (0x1   << 4  )
#define DMASWREQ_I2CSTX                (0x1   << 4  )
#define DMASWREQ_I2CSTX_DIS            (0x0   << 4  ) /* DIS                      */
#define DMASWREQ_I2CSTX_EN             (0x1   << 4  ) /* EN                       */

/* DMASWREQ[UARTRX] - DMA UART RX */
#define DMASWREQ_UARTRX_BBA            (*(volatile unsigned long *) 0x4220028C)
#define DMASWREQ_UARTRX_MSK            (0x1   << 3  )
#define DMASWREQ_UARTRX                (0x1   << 3  )
#define DMASWREQ_UARTRX_DIS            (0x0   << 3  ) /* DIS                      */
#define DMASWREQ_UARTRX_EN             (0x1   << 3  ) /* EN                       */

/* DMASWREQ[UARTTX] - DMA UART TX */
#define DMASWREQ_UARTTX_BBA            (*(volatile unsigned long *) 0x42200288)
#define DMASWREQ_UARTTX_MSK            (0x1   << 2  )
#define DMASWREQ_UARTTX                (0x1   << 2  )
#define DMASWREQ_UARTTX_DIS            (0x0   << 2  ) /* DIS                      */
#define DMASWREQ_UARTTX_EN             (0x1   << 2  ) /* EN                       */

/* DMASWREQ[SPI1RX] - DMA SPI 1 RX */
#define DMASWREQ_SPI1RX_BBA            (*(volatile unsigned long *) 0x42200284)
#define DMASWREQ_SPI1RX_MSK            (0x1   << 1  )
#define DMASWREQ_SPI1RX                (0x1   << 1  )
#define DMASWREQ_SPI1RX_DIS            (0x0   << 1  ) /* DIS                      */
#define DMASWREQ_SPI1RX_EN             (0x1   << 1  ) /* EN                       */

/* DMASWREQ[SPI1TX] - DMA SPI 1 TX */
#define DMASWREQ_SPI1TX_BBA            (*(volatile unsigned long *) 0x42200280)
#define DMASWREQ_SPI1TX_MSK            (0x1   << 0  )
#define DMASWREQ_SPI1TX                (0x1   << 0  )
#define DMASWREQ_SPI1TX_DIS            (0x0   << 0  ) /* DIS                      */
#define DMASWREQ_SPI1TX_EN             (0x1   << 0  ) /* EN                       */

/* Reset Value for DMARMSKSET*/
#define DMARMSKSET_RVAL                0x0 
/* DMARMSKSET[UART1RX] - DMA UART1 RX */
#define DMARMSKSET_UART1RX_BBA             (*(volatile unsigned long *) 0x4220043C)
#define DMARMSKSET_UART1RX_MSK             (0x1   << 15 )
#define DMARMSKSET_UART1RX                 (0x1   << 15 )
#define DMARMSKSET_UART1RX_DIS             (0x0   << 15 ) /* DIS                      */
#define DMARMSKSET_UART1RX_EN              (0x1   << 15 ) /* EN                       */
/* DMARMSKSET[UART1TX] - DMA UART1 TX */
#define DMARMSKSET_UART1TX_BBA             (*(volatile unsigned long *) 0x42200438)
#define DMARMSKSET_UART1TX_MSK             (0x1   << 14 )
#define DMARMSKSET_UART1TX                 (0x1   << 14 )
#define DMARMSKSET_UART1TX_DIS             (0x0   << 14 ) /* DIS                      */
#define DMARMSKSET_UART1TX_EN              (0x1   << 14 ) /* EN                       */
/* DMARMSKSET[SPI0RX] - DMA SPI 0 RX */
#define DMARMSKSET_SPI0RX_BBA             (*(volatile unsigned long *) 0x42200434)
#define DMARMSKSET_SPI0RX_MSK             (0x1   << 13 )
#define DMARMSKSET_SPI0RX                 (0x1   << 13 )
#define DMARMSKSET_SPI0RX_DIS             (0x0   << 13 ) /* DIS                      */
#define DMARMSKSET_SPI0RX_EN              (0x1   << 13 ) /* EN                       */
/* DMARMSKSET[SPI0TX] - DMA SPI 0 TX */
#define DMARMSKSET_SPI0TX_BBA             (*(volatile unsigned long *) 0x42200430)
#define DMARMSKSET_SPI0TX_MSK             (0x1   << 12 )
#define DMARMSKSET_SPI0TX                 (0x1   << 12 )
#define DMARMSKSET_SPI0TX_DIS             (0x0   << 12 ) /* DIS                      */
#define DMARMSKSET_SPI0TX_EN              (0x1   << 12 ) /* EN                       */

/* DMARMSKSET[SINC2] - SINC2 Output Step detection */
#define DMARMSKSET_SINC2_BBA           (*(volatile unsigned long *) 0x4220042C)
#define DMARMSKSET_SINC2_MSK           (0x1   << 11 )
#define DMARMSKSET_SINC2               (0x1   << 11 )
#define DMARMSKSET_SINC2_DIS           (0x0   << 11 ) /* DIS                      */
#define DMARMSKSET_SINC2_EN            (0x1   << 11 ) /* EN                       */

/* DMARMSKSET[ADC1] - ADC1 */
#define DMARMSKSET_ADC1_BBA            (*(volatile unsigned long *) 0x42200428)
#define DMARMSKSET_ADC1_MSK            (0x1   << 10 )
#define DMARMSKSET_ADC1                (0x1   << 10 )
#define DMARMSKSET_ADC1_DIS            (0x0   << 10 ) /* DIS                      */
#define DMARMSKSET_ADC1_EN             (0x1   << 10 ) /* EN                       */

/* DMARMSKSET[DAC] - DAC DMA Output */
#define DMARMSKSET_DAC_BBA             (*(volatile unsigned long *) 0x42200420)
#define DMARMSKSET_DAC_MSK             (0x1   << 8  )
#define DMARMSKSET_DAC                 (0x1   << 8  )
#define DMARMSKSET_DAC_DIS             (0x0   << 8  ) /* DIS                      */
#define DMARMSKSET_DAC_EN              (0x1   << 8  ) /* EN                       */

/* DMARMSKSET[I2CMRX] - DMA I2C Master RX */
#define DMARMSKSET_I2CMRX_BBA          (*(volatile unsigned long *) 0x4220041C)
#define DMARMSKSET_I2CMRX_MSK          (0x1   << 7  )
#define DMARMSKSET_I2CMRX              (0x1   << 7  )
#define DMARMSKSET_I2CMRX_DIS          (0x0   << 7  ) /* DIS                      */
#define DMARMSKSET_I2CMRX_EN           (0x1   << 7  ) /* EN                       */

/* DMARMSKSET[I2CMTX] - DMA I2C Master TX */
#define DMARMSKSET_I2CMTX_BBA          (*(volatile unsigned long *) 0x42200418)
#define DMARMSKSET_I2CMTX_MSK          (0x1   << 6  )
#define DMARMSKSET_I2CMTX              (0x1   << 6  )
#define DMARMSKSET_I2CMTX_DIS          (0x0   << 6  ) /* DIS                      */
#define DMARMSKSET_I2CMTX_EN           (0x1   << 6  ) /* EN                       */

/* DMARMSKSET[I2CSRX] - DMA I2C Slave RX */
#define DMARMSKSET_I2CSRX_BBA          (*(volatile unsigned long *) 0x42200414)
#define DMARMSKSET_I2CSRX_MSK          (0x1   << 5  )
#define DMARMSKSET_I2CSRX              (0x1   << 5  )
#define DMARMSKSET_I2CSRX_DIS          (0x0   << 5  ) /* DIS                      */
#define DMARMSKSET_I2CSRX_EN           (0x1   << 5  ) /* EN                       */

/* DMARMSKSET[I2CSTX] - DMA I2C Slave TX */
#define DMARMSKSET_I2CSTX_BBA          (*(volatile unsigned long *) 0x42200410)
#define DMARMSKSET_I2CSTX_MSK          (0x1   << 4  )
#define DMARMSKSET_I2CSTX              (0x1   << 4  )
#define DMARMSKSET_I2CSTX_DIS          (0x0   << 4  ) /* DIS                      */
#define DMARMSKSET_I2CSTX_EN           (0x1   << 4  ) /* EN                       */

/* DMARMSKSET[UARTRX] - DMA UART RX */
#define DMARMSKSET_UARTRX_BBA          (*(volatile unsigned long *) 0x4220040C)
#define DMARMSKSET_UARTRX_MSK          (0x1   << 3  )
#define DMARMSKSET_UARTRX              (0x1   << 3  )
#define DMARMSKSET_UARTRX_DIS          (0x0   << 3  ) /* DIS                      */
#define DMARMSKSET_UARTRX_EN           (0x1   << 3  ) /* EN                       */

/* DMARMSKSET[UARTTX] - DMA UART TX */
#define DMARMSKSET_UARTTX_BBA          (*(volatile unsigned long *) 0x42200408)
#define DMARMSKSET_UARTTX_MSK          (0x1   << 2  )
#define DMARMSKSET_UARTTX              (0x1   << 2  )
#define DMARMSKSET_UARTTX_DIS          (0x0   << 2  ) /* DIS                      */
#define DMARMSKSET_UARTTX_EN           (0x1   << 2  ) /* EN                       */

/* DMARMSKSET[SPI1RX] - DMA SPI 1 RX */
#define DMARMSKSET_SPI1RX_BBA          (*(volatile unsigned long *) 0x42200404)
#define DMARMSKSET_SPI1RX_MSK          (0x1   << 1  )
#define DMARMSKSET_SPI1RX              (0x1   << 1  )
#define DMARMSKSET_SPI1RX_DIS          (0x0   << 1  ) /* DIS                      */
#define DMARMSKSET_SPI1RX_EN           (0x1   << 1  ) /* EN                       */

/* DMARMSKSET[SPI1TX] - DMA SPI 1 TX */
#define DMARMSKSET_SPI1TX_BBA          (*(volatile unsigned long *) 0x42200400)
#define DMARMSKSET_SPI1TX_MSK          (0x1   << 0  )
#define DMARMSKSET_SPI1TX              (0x1   << 0  )
#define DMARMSKSET_SPI1TX_DIS          (0x0   << 0  ) /* DIS                      */
#define DMARMSKSET_SPI1TX_EN           (0x1   << 0  ) /* EN                       */

/* Reset Value for DMARMSKCLR*/
#define DMARMSKCLR_RVAL                0x0 

/* DMARMSKCLR[UART1RX] - DMA UART1 RX */
#define DMARMSKCLR_UART1RX_BBA             (*(volatile unsigned long *) 0x422004BC)
#define DMARMSKCLR_UART1RX_MSK             (0x1   << 15 )
#define DMARMSKCLR_UART1RX                 (0x1   << 15 )
#define DMARMSKCLR_UART1RX_DIS             (0x0   << 15 ) /* DIS                      */
#define DMARMSKCLR_UART1RX_EN              (0x1   << 15 ) /* EN                       */
/* DMARMSKCLR[UART1TX] - DMA UART1 TX */
#define DMARMSKCLR_UART1TX_BBA             (*(volatile unsigned long *) 0x422004B8)
#define DMARMSKCLR_UART1TX_MSK             (0x1   << 14 )
#define DMARMSKCLR_UART1TX                 (0x1   << 14 )
#define DMARMSKCLR_UART1TX_DIS             (0x0   << 14 ) /* DIS                      */
#define DMARMSKCLR_UART1TX_EN              (0x1   << 14 ) /* EN                       */
/* DMARMSKCLR[SPI0RX] - DMA SPI 0 RX */
#define DMARMSKCLR_SPI0RX_BBA             (*(volatile unsigned long *) 0x422004B4)
#define DMARMSKCLR_SPI0RX_MSK             (0x1   << 13 )
#define DMARMSKCLR_SPI0RX                 (0x1   << 13 )
#define DMARMSKCLR_SPI0RX_DIS             (0x0   << 13 ) /* DIS                      */
#define DMARMSKCLR_SPI0RX_EN              (0x1   << 13 ) /* EN                       */
/* DMARMSKCLR[SPI0TX] - DMA SPI 0 TX */
#define DMARMSKCLR_SPI0TX_BBA             (*(volatile unsigned long *) 0x422004B0)
#define DMARMSKCLR_SPI0TX_MSK             (0x1   << 12 )
#define DMARMSKCLR_SPI0TX                 (0x1   << 12 )
#define DMARMSKCLR_SPI0TX_DIS             (0x0   << 12 ) /* DIS                      */
#define DMARMSKCLR_SPI0TX_EN              (0x1   << 12 ) /* EN                       */
/* DMARMSKCLR[SINC2] - SINC2 Output Step detection */
#define DMARMSKCLR_SINC2_BBA           (*(volatile unsigned long *) 0x422004AC)
#define DMARMSKCLR_SINC2_MSK           (0x1   << 11 )
#define DMARMSKCLR_SINC2               (0x1   << 11 )
#define DMARMSKCLR_SINC2_DIS           (0x0   << 11 ) /* DIS                      */
#define DMARMSKCLR_SINC2_EN            (0x1   << 11 ) /* EN                       */

/* DMARMSKCLR[ADC1] - ADC1 */
#define DMARMSKCLR_ADC1_BBA            (*(volatile unsigned long *) 0x422004A8)
#define DMARMSKCLR_ADC1_MSK            (0x1   << 10 )
#define DMARMSKCLR_ADC1                (0x1   << 10 )
#define DMARMSKCLR_ADC1_DIS            (0x0   << 10 ) /* DIS                      */
#define DMARMSKCLR_ADC1_EN             (0x1   << 10 ) /* EN                       */

/* DMARMSKCLR[DAC] - DAC DMA Output */
#define DMARMSKCLR_DAC_BBA             (*(volatile unsigned long *) 0x422004A0)
#define DMARMSKCLR_DAC_MSK             (0x1   << 8  )
#define DMARMSKCLR_DAC                 (0x1   << 8  )
#define DMARMSKCLR_DAC_DIS             (0x0   << 8  ) /* DIS                      */
#define DMARMSKCLR_DAC_EN              (0x1   << 8  ) /* EN                       */

/* DMARMSKCLR[I2CMRX] - DMA I2C Master RX */
#define DMARMSKCLR_I2CMRX_BBA          (*(volatile unsigned long *) 0x4220049C)
#define DMARMSKCLR_I2CMRX_MSK          (0x1   << 7  )
#define DMARMSKCLR_I2CMRX              (0x1   << 7  )
#define DMARMSKCLR_I2CMRX_DIS          (0x0   << 7  ) /* DIS                      */
#define DMARMSKCLR_I2CMRX_EN           (0x1   << 7  ) /* EN                       */

/* DMARMSKCLR[I2CMTX] - DMA I2C Master TX */
#define DMARMSKCLR_I2CMTX_BBA          (*(volatile unsigned long *) 0x42200498)
#define DMARMSKCLR_I2CMTX_MSK          (0x1   << 6  )
#define DMARMSKCLR_I2CMTX              (0x1   << 6  )
#define DMARMSKCLR_I2CMTX_DIS          (0x0   << 6  ) /* DIS                      */
#define DMARMSKCLR_I2CMTX_EN           (0x1   << 6  ) /* EN                       */

/* DMARMSKCLR[I2CSRX] - DMA I2C Slave RX */
#define DMARMSKCLR_I2CSRX_BBA          (*(volatile unsigned long *) 0x42200494)
#define DMARMSKCLR_I2CSRX_MSK          (0x1   << 5  )
#define DMARMSKCLR_I2CSRX              (0x1   << 5  )
#define DMARMSKCLR_I2CSRX_DIS          (0x0   << 5  ) /* DIS                      */
#define DMARMSKCLR_I2CSRX_EN           (0x1   << 5  ) /* EN                       */

/* DMARMSKCLR[I2CSTX] - DMA I2C Slave TX */
#define DMARMSKCLR_I2CSTX_BBA          (*(volatile unsigned long *) 0x42200490)
#define DMARMSKCLR_I2CSTX_MSK          (0x1   << 4  )
#define DMARMSKCLR_I2CSTX              (0x1   << 4  )
#define DMARMSKCLR_I2CSTX_DIS          (0x0   << 4  ) /* DIS                      */
#define DMARMSKCLR_I2CSTX_EN           (0x1   << 4  ) /* EN                       */

/* DMARMSKCLR[UARTRX] - DMA UART RX */
#define DMARMSKCLR_UARTRX_BBA          (*(volatile unsigned long *) 0x4220048C)
#define DMARMSKCLR_UARTRX_MSK          (0x1   << 3  )
#define DMARMSKCLR_UARTRX              (0x1   << 3  )
#define DMARMSKCLR_UARTRX_DIS          (0x0   << 3  ) /* DIS                      */
#define DMARMSKCLR_UARTRX_EN           (0x1   << 3  ) /* EN                       */

/* DMARMSKCLR[UARTTX] - DMA UART TX */
#define DMARMSKCLR_UARTTX_BBA          (*(volatile unsigned long *) 0x42200488)
#define DMARMSKCLR_UARTTX_MSK          (0x1   << 2  )
#define DMARMSKCLR_UARTTX              (0x1   << 2  )
#define DMARMSKCLR_UARTTX_DIS          (0x0   << 2  ) /* DIS                      */
#define DMARMSKCLR_UARTTX_EN           (0x1   << 2  ) /* EN                       */

/* DMARMSKCLR[SPI1RX] - DMA SPI 1 RX */
#define DMARMSKCLR_SPI1RX_BBA          (*(volatile unsigned long *) 0x42200484)
#define DMARMSKCLR_SPI1RX_MSK          (0x1   << 1  )
#define DMARMSKCLR_SPI1RX              (0x1   << 1  )
#define DMARMSKCLR_SPI1RX_DIS          (0x0   << 1  ) /* DIS                      */
#define DMARMSKCLR_SPI1RX_EN           (0x1   << 1  ) /* EN                       */

/* DMARMSKCLR[SPI1TX] - DMA SPI 1 TX */
#define DMARMSKCLR_SPI1TX_BBA          (*(volatile unsigned long *) 0x42200480)
#define DMARMSKCLR_SPI1TX_MSK          (0x1   << 0  )
#define DMARMSKCLR_SPI1TX              (0x1   << 0  )
#define DMARMSKCLR_SPI1TX_DIS          (0x0   << 0  ) /* DIS                      */
#define DMARMSKCLR_SPI1TX_EN           (0x1   << 0  ) /* EN                       */

/* Reset Value for DMAENSET*/
#define DMAENSET_RVAL                  0x0 


/* DMAENSET[UART1RX] - DMA UART1 RX */
#define DMAENSET_UART1RX_BBA             (*(volatile unsigned long *) 0x4220053C)
#define DMAENSET_UART1RX_MSK             (0x1   << 15 )
#define DMAENSET_UART1RX                 (0x1   << 15 )
#define DMAENSET_UART1RX_DIS             (0x0   << 15 ) /* DIS                      */
#define DMAENSET_UART1RX_EN              (0x1   << 15 ) /* EN                       */
/* DMAENSET[UART1TX] - DMA UART1 TX */
#define DMAENSET_UART1TX_BBA             (*(volatile unsigned long *) 0x42200538)
#define DMAENSET_UART1TX_MSK             (0x1   << 14 )
#define DMAENSET_UART1TX                 (0x1   << 14 )
#define DMAENSET_UART1TX_DIS             (0x0   << 14 ) /* DIS                      */
#define DMAENSET_UART1TX_EN              (0x1   << 14 ) /* EN                       */
/* DMAENSET[SPI0RX] - DMA SPI 0 RX */
#define DMAENSET_SPI0RX_BBA             (*(volatile unsigned long *) 0x42200534)
#define DMAENSET_SPI0RX_MSK             (0x1   << 13 )
#define DMAENSET_SPI0RX                 (0x1   << 13 )
#define DMAENSET_SPI0RX_DIS             (0x0   << 13 ) /* DIS                      */
#define DMAENSET_SPI0RX_EN              (0x1   << 13 ) /* EN                       */
/* DMAENSET[SPI0TX] - DMA SPI 0 TX */
#define DMAENSET_SPI0TX_BBA             (*(volatile unsigned long *) 0x42200530)
#define DMAENSET_SPI0TX_MSK             (0x1   << 12 )
#define DMAENSET_SPI0TX                 (0x1   << 12 )
#define DMAENSET_SPI0TX_DIS             (0x0   << 12 ) /* DIS                      */
#define DMAENSET_SPI0TX_EN              (0x1   << 12 ) /* EN                       */

/* DMAENSET[SINC2] - SINC2 Output Step detection */
#define DMAENSET_SINC2_BBA             (*(volatile unsigned long *) 0x4220052C)
#define DMAENSET_SINC2_MSK             (0x1   << 11 )
#define DMAENSET_SINC2                 (0x1   << 11 )
#define DMAENSET_SINC2_DIS             (0x0   << 11 ) /* DIS                      */
#define DMAENSET_SINC2_EN              (0x1   << 11 ) /* EN                       */

/* DMAENSET[ADC1] - ADC1 */
#define DMAENSET_ADC1_BBA              (*(volatile unsigned long *) 0x42200528)
#define DMAENSET_ADC1_MSK              (0x1   << 10 )
#define DMAENSET_ADC1                  (0x1   << 10 )
#define DMAENSET_ADC1_DIS              (0x0   << 10 ) /* DIS                      */
#define DMAENSET_ADC1_EN               (0x1   << 10 ) /* EN                       */

/* DMAENSET[DAC] - DAC DMA Output */
#define DMAENSET_DAC_BBA               (*(volatile unsigned long *) 0x42200520)
#define DMAENSET_DAC_MSK               (0x1   << 8  )
#define DMAENSET_DAC                   (0x1   << 8  )
#define DMAENSET_DAC_DIS               (0x0   << 8  ) /* DIS                      */
#define DMAENSET_DAC_EN                (0x1   << 8  ) /* EN                       */

/* DMAENSET[I2CMRX] - DMA I2C Master RX */
#define DMAENSET_I2CMRX_BBA            (*(volatile unsigned long *) 0x4220051C)
#define DMAENSET_I2CMRX_MSK            (0x1   << 7  )
#define DMAENSET_I2CMRX                (0x1   << 7  )
#define DMAENSET_I2CMRX_DIS            (0x0   << 7  ) /* DIS                      */
#define DMAENSET_I2CMRX_EN             (0x1   << 7  ) /* EN                       */

/* DMAENSET[I2CMTX] - DMA I2C Master TX */
#define DMAENSET_I2CMTX_BBA            (*(volatile unsigned long *) 0x42200518)
#define DMAENSET_I2CMTX_MSK            (0x1   << 6  )
#define DMAENSET_I2CMTX                (0x1   << 6  )
#define DMAENSET_I2CMTX_DIS            (0x0   << 6  ) /* DIS                      */
#define DMAENSET_I2CMTX_EN             (0x1   << 6  ) /* EN                       */

/* DMAENSET[I2CSRX] - DMA I2C Slave RX */
#define DMAENSET_I2CSRX_BBA            (*(volatile unsigned long *) 0x42200514)
#define DMAENSET_I2CSRX_MSK            (0x1   << 5  )
#define DMAENSET_I2CSRX                (0x1   << 5  )
#define DMAENSET_I2CSRX_DIS            (0x0   << 5  ) /* DIS                      */
#define DMAENSET_I2CSRX_EN             (0x1   << 5  ) /* EN                       */

/* DMAENSET[I2CSTX] - DMA I2C Slave TX */
#define DMAENSET_I2CSTX_BBA            (*(volatile unsigned long *) 0x42200510)
#define DMAENSET_I2CSTX_MSK            (0x1   << 4  )
#define DMAENSET_I2CSTX                (0x1   << 4  )
#define DMAENSET_I2CSTX_DIS            (0x0   << 4  ) /* DIS                      */
#define DMAENSET_I2CSTX_EN             (0x1   << 4  ) /* EN                       */

/* DMAENSET[UARTRX] - DMA UART RX */
#define DMAENSET_UARTRX_BBA            (*(volatile unsigned long *) 0x4220050C)
#define DMAENSET_UARTRX_MSK            (0x1   << 3  )
#define DMAENSET_UARTRX                (0x1   << 3  )
#define DMAENSET_UARTRX_DIS            (0x0   << 3  ) /* DIS                      */
#define DMAENSET_UARTRX_EN             (0x1   << 3  ) /* EN                       */

/* DMAENSET[UARTTX] - DMA UART TX */
#define DMAENSET_UARTTX_BBA            (*(volatile unsigned long *) 0x42200508)
#define DMAENSET_UARTTX_MSK            (0x1   << 2  )
#define DMAENSET_UARTTX                (0x1   << 2  )
#define DMAENSET_UARTTX_DIS            (0x0   << 2  ) /* DIS                      */
#define DMAENSET_UARTTX_EN             (0x1   << 2  ) /* EN                       */

/* DMAENSET[SPI1RX] - DMA SPI 1 RX */
#define DMAENSET_SPI1RX_BBA            (*(volatile unsigned long *) 0x42200504)
#define DMAENSET_SPI1RX_MSK            (0x1   << 1  )
#define DMAENSET_SPI1RX                (0x1   << 1  )
#define DMAENSET_SPI1RX_DIS            (0x0   << 1  ) /* DIS                      */
#define DMAENSET_SPI1RX_EN             (0x1   << 1  ) /* EN                       */

/* DMAENSET[SPI1TX] - DMA SPI 1 TX */
#define DMAENSET_SPI1TX_BBA            (*(volatile unsigned long *) 0x42200500)
#define DMAENSET_SPI1TX_MSK            (0x1   << 0  )
#define DMAENSET_SPI1TX                (0x1   << 0  )
#define DMAENSET_SPI1TX_DIS            (0x0   << 0  ) /* DIS                      */
#define DMAENSET_SPI1TX_EN             (0x1   << 0  ) /* EN                       */

/* Reset Value for DMAENCLR*/
#define DMAENCLR_RVAL                  0x0 

/* DMAENCLR[UART1RX] - DMA UART1 RX */
#define DMAENCLR_UART1RX_BBA             (*(volatile unsigned long *) 0x422005BC)
#define DMAENCLR_UART1RX_MSK             (0x1   << 15 )
#define DMAENCLR_UART1RX                 (0x1   << 15 )
#define DMAENCLR_UART1RX_DIS             (0x0   << 15 ) /* DIS                      */
#define DMAENCLR_UART1RX_EN              (0x1   << 15 ) /* EN                       */
/* DMAENCLR[UART1TX] - DMA UART1 TX */
#define DMAENCLR_UART1TX_BBA             (*(volatile unsigned long *) 0x422005B8)
#define DMAENCLR_UART1TX_MSK             (0x1   << 14 )
#define DMAENCLR_UART1TX                 (0x1   << 14 )
#define DMAENCLR_UART1TX_DIS             (0x0   << 14 ) /* DIS                      */
#define DMAENCLR_UART1TX_EN              (0x1   << 14 ) /* EN                       */
/* DMAENCLR[SPI0RX] - DMA SPI 0 RX */
#define DMAENCLR_SPI0RX_BBA             (*(volatile unsigned long *) 0x422005B4)
#define DMAENCLR_SPI0RX_MSK             (0x1   << 13 )
#define DMAENCLR_SPI0RX                 (0x1   << 13 )
#define DMAENCLR_SPI0RX_DIS             (0x0   << 13 ) /* DIS                      */
#define DMAENCLR_SPI0RX_EN              (0x1   << 13 ) /* EN                       */
/* DMAENCLR[SPI0TX] - DMA SPI 0 TX */
#define DMAENCLR_SPI0TX_BBA             (*(volatile unsigned long *) 0x422005B0)
#define DMAENCLR_SPI0TX_MSK             (0x1   << 12 )
#define DMAENCLR_SPI0TX                 (0x1   << 12 )
#define DMAENCLR_SPI0TX_DIS             (0x0   << 12 ) /* DIS                      */
#define DMAENCLR_SPI0TX_EN              (0x1   << 12 ) /* EN                       */


/* DMAENCLR[SINC2] - SINC2 Output Step detection */
#define DMAENCLR_SINC2_BBA             (*(volatile unsigned long *) 0x422005AC)
#define DMAENCLR_SINC2_MSK             (0x1   << 11 )
#define DMAENCLR_SINC2                 (0x1   << 11 )
#define DMAENCLR_SINC2_DIS             (0x0   << 11 ) /* DIS                      */
#define DMAENCLR_SINC2_EN              (0x1   << 11 ) /* EN                       */

/* DMAENCLR[ADC1] - ADC1 */
#define DMAENCLR_ADC1_BBA              (*(volatile unsigned long *) 0x422005A8)
#define DMAENCLR_ADC1_MSK              (0x1   << 10 )
#define DMAENCLR_ADC1                  (0x1   << 10 )
#define DMAENCLR_ADC1_DIS              (0x0   << 10 ) /* DIS                      */
#define DMAENCLR_ADC1_EN               (0x1   << 10 ) /* EN                       */

/* DMAENCLR[DAC] - DAC DMA Output */
#define DMAENCLR_DAC_BBA               (*(volatile unsigned long *) 0x422005A0)
#define DMAENCLR_DAC_MSK               (0x1   << 8  )
#define DMAENCLR_DAC                   (0x1   << 8  )
#define DMAENCLR_DAC_DIS               (0x0   << 8  ) /* DIS                      */
#define DMAENCLR_DAC_EN                (0x1   << 8  ) /* EN                       */

/* DMAENCLR[I2CMRX] - DMA I2C Master RX */
#define DMAENCLR_I2CMRX_BBA            (*(volatile unsigned long *) 0x4220059C)
#define DMAENCLR_I2CMRX_MSK            (0x1   << 7  )
#define DMAENCLR_I2CMRX                (0x1   << 7  )
#define DMAENCLR_I2CMRX_DIS            (0x0   << 7  ) /* DIS                      */
#define DMAENCLR_I2CMRX_EN             (0x1   << 7  ) /* EN                       */

/* DMAENCLR[I2CMTX] - DMA I2C Master TX */
#define DMAENCLR_I2CMTX_BBA            (*(volatile unsigned long *) 0x42200598)
#define DMAENCLR_I2CMTX_MSK            (0x1   << 6  )
#define DMAENCLR_I2CMTX                (0x1   << 6  )
#define DMAENCLR_I2CMTX_DIS            (0x0   << 6  ) /* DIS                      */
#define DMAENCLR_I2CMTX_EN             (0x1   << 6  ) /* EN                       */

/* DMAENCLR[I2CSRX] - DMA I2C Slave RX */
#define DMAENCLR_I2CSRX_BBA            (*(volatile unsigned long *) 0x42200594)
#define DMAENCLR_I2CSRX_MSK            (0x1   << 5  )
#define DMAENCLR_I2CSRX                (0x1   << 5  )
#define DMAENCLR_I2CSRX_DIS            (0x0   << 5  ) /* DIS                      */
#define DMAENCLR_I2CSRX_EN             (0x1   << 5  ) /* EN                       */

/* DMAENCLR[I2CSTX] - DMA I2C Slave TX */
#define DMAENCLR_I2CSTX_BBA            (*(volatile unsigned long *) 0x42200590)
#define DMAENCLR_I2CSTX_MSK            (0x1   << 4  )
#define DMAENCLR_I2CSTX                (0x1   << 4  )
#define DMAENCLR_I2CSTX_DIS            (0x0   << 4  ) /* DIS                      */
#define DMAENCLR_I2CSTX_EN             (0x1   << 4  ) /* EN                       */

/* DMAENCLR[UARTRX] - DMA UART RX */
#define DMAENCLR_UARTRX_BBA            (*(volatile unsigned long *) 0x4220058C)
#define DMAENCLR_UARTRX_MSK            (0x1   << 3  )
#define DMAENCLR_UARTRX                (0x1   << 3  )
#define DMAENCLR_UARTRX_DIS            (0x0   << 3  ) /* DIS                      */
#define DMAENCLR_UARTRX_EN             (0x1   << 3  ) /* EN                       */

/* DMAENCLR[UARTTX] - DMA UART TX */
#define DMAENCLR_UARTTX_BBA            (*(volatile unsigned long *) 0x42200588)
#define DMAENCLR_UARTTX_MSK            (0x1   << 2  )
#define DMAENCLR_UARTTX                (0x1   << 2  )
#define DMAENCLR_UARTTX_DIS            (0x0   << 2  ) /* DIS                      */
#define DMAENCLR_UARTTX_EN             (0x1   << 2  ) /* EN                       */

/* DMAENCLR[SPI1RX] - DMA SPI 1 RX */
#define DMAENCLR_SPI1RX_BBA            (*(volatile unsigned long *) 0x42200584)
#define DMAENCLR_SPI1RX_MSK            (0x1   << 1  )
#define DMAENCLR_SPI1RX                (0x1   << 1  )
#define DMAENCLR_SPI1RX_DIS            (0x0   << 1  ) /* DIS                      */
#define DMAENCLR_SPI1RX_EN             (0x1   << 1  ) /* EN                       */

/* DMAENCLR[SPI1TX] - DMA SPI 1 TX */
#define DMAENCLR_SPI1TX_BBA            (*(volatile unsigned long *) 0x42200580)
#define DMAENCLR_SPI1TX_MSK            (0x1   << 0  )
#define DMAENCLR_SPI1TX                (0x1   << 0  )
#define DMAENCLR_SPI1TX_DIS            (0x0   << 0  ) /* DIS                      */
#define DMAENCLR_SPI1TX_EN             (0x1   << 0  ) /* EN                       */

/* Reset Value for DMAALTSET*/
#define DMAALTSET_RVAL                 0x0 

/* DMAALTSET[UART1RX] - DMA UART1 RX */
#define DMAALTSET_UART1RX_BBA             (*(volatile unsigned long *) 0x4220063C)
#define DMAALTSET_UART1RX_MSK             (0x1   << 15 )
#define DMAALTSET_UART1RX                 (0x1   << 15 )
#define DMAALTSET_UART1RX_DIS             (0x0   << 15 ) /* DIS                      */
#define DMAALTSET_UART1RX_EN              (0x1   << 15 ) /* EN                       */
/* DMAALTSET[UART1TX] - DMA UART1 TX */
#define DMAALTSET_UART1TX_BBA             (*(volatile unsigned long *) 0x42200638)
#define DMAALTSET_UART1TX_MSK             (0x1   << 14 )
#define DMAALTSET_UART1TX                 (0x1   << 14 )
#define DMAALTSET_UART1TX_DIS             (0x0   << 14 ) /* DIS                      */
#define DMAALTSET_UART1TX_EN              (0x1   << 14 ) /* EN                       */
/* DMAALTSET[SPI0RX] - DMA SPI 0 RX */
#define DMAALTSET_SPI0RX_BBA             (*(volatile unsigned long *) 0x42200634)
#define DMAALTSET_SPI0RX_MSK             (0x1   << 13 )
#define DMAALTSET_SPI0RX                 (0x1   << 13 )
#define DMAALTSET_SPI0RX_DIS             (0x0   << 13 ) /* DIS                      */
#define DMAALTSET_SPI0RX_EN              (0x1   << 13 ) /* EN                       */
/* DMAALTSET[SPI0TX] - DMA SPI 0 TX */
#define DMAALTSET_SPI0TX_BBA             (*(volatile unsigned long *) 0x42200630)
#define DMAALTSET_SPI0TX_MSK             (0x1   << 12 )
#define DMAALTSET_SPI0TX                 (0x1   << 12 )
#define DMAALTSET_SPI0TX_DIS             (0x0   << 12 ) /* DIS                      */
#define DMAALTSET_SPI0TX_EN              (0x1   << 12 ) /* EN                       */

/* DMAALTSET[SINC2] - SINC2 Output Step detection */
#define DMAALTSET_SINC2_BBA            (*(volatile unsigned long *) 0x4220062C)
#define DMAALTSET_SINC2_MSK            (0x1   << 11 )
#define DMAALTSET_SINC2                (0x1   << 11 )
#define DMAALTSET_SINC2_DIS            (0x0   << 11 ) /* DIS                      */
#define DMAALTSET_SINC2_EN             (0x1   << 11 ) /* EN                       */

/* DMAALTSET[ADC1] - ADC1 */
#define DMAALTSET_ADC1_BBA             (*(volatile unsigned long *) 0x42200628)
#define DMAALTSET_ADC1_MSK             (0x1   << 10 )
#define DMAALTSET_ADC1                 (0x1   << 10 )
#define DMAALTSET_ADC1_DIS             (0x0   << 10 ) /* DIS                      */
#define DMAALTSET_ADC1_EN              (0x1   << 10 ) /* EN                       */

/* DMAALTSET[DAC] - DAC DMA Output */
#define DMAALTSET_DAC_BBA              (*(volatile unsigned long *) 0x42200620)
#define DMAALTSET_DAC_MSK              (0x1   << 8  )
#define DMAALTSET_DAC                  (0x1   << 8  )
#define DMAALTSET_DAC_DIS              (0x0   << 8  ) /* DIS                      */
#define DMAALTSET_DAC_EN               (0x1   << 8  ) /* EN                       */

/* DMAALTSET[I2CMRX] - DMA I2C Master RX */
#define DMAALTSET_I2CMRX_BBA           (*(volatile unsigned long *) 0x4220061C)
#define DMAALTSET_I2CMRX_MSK           (0x1   << 7  )
#define DMAALTSET_I2CMRX               (0x1   << 7  )
#define DMAALTSET_I2CMRX_DIS           (0x0   << 7  ) /* DIS                      */
#define DMAALTSET_I2CMRX_EN            (0x1   << 7  ) /* EN                       */

/* DMAALTSET[I2CMTX] - DMA I2C Master TX */
#define DMAALTSET_I2CMTX_BBA           (*(volatile unsigned long *) 0x42200618)
#define DMAALTSET_I2CMTX_MSK           (0x1   << 6  )
#define DMAALTSET_I2CMTX               (0x1   << 6  )
#define DMAALTSET_I2CMTX_DIS           (0x0   << 6  ) /* DIS                      */
#define DMAALTSET_I2CMTX_EN            (0x1   << 6  ) /* EN                       */

/* DMAALTSET[I2CSRX] - DMA I2C Slave RX */
#define DMAALTSET_I2CSRX_BBA           (*(volatile unsigned long *) 0x42200614)
#define DMAALTSET_I2CSRX_MSK           (0x1   << 5  )
#define DMAALTSET_I2CSRX               (0x1   << 5  )
#define DMAALTSET_I2CSRX_DIS           (0x0   << 5  ) /* DIS                      */
#define DMAALTSET_I2CSRX_EN            (0x1   << 5  ) /* EN                       */

/* DMAALTSET[I2CSTX] - DMA I2C Slave TX */
#define DMAALTSET_I2CSTX_BBA           (*(volatile unsigned long *) 0x42200610)
#define DMAALTSET_I2CSTX_MSK           (0x1   << 4  )
#define DMAALTSET_I2CSTX               (0x1   << 4  )
#define DMAALTSET_I2CSTX_DIS           (0x0   << 4  ) /* DIS                      */
#define DMAALTSET_I2CSTX_EN            (0x1   << 4  ) /* EN                       */

/* DMAALTSET[UARTRX] - DMA UART RX */
#define DMAALTSET_UARTRX_BBA           (*(volatile unsigned long *) 0x4220060C)
#define DMAALTSET_UARTRX_MSK           (0x1   << 3  )
#define DMAALTSET_UARTRX               (0x1   << 3  )
#define DMAALTSET_UARTRX_DIS           (0x0   << 3  ) /* DIS                      */
#define DMAALTSET_UARTRX_EN            (0x1   << 3  ) /* EN                       */

/* DMAALTSET[UARTTX] - DMA UART TX */
#define DMAALTSET_UARTTX_BBA           (*(volatile unsigned long *) 0x42200608)
#define DMAALTSET_UARTTX_MSK           (0x1   << 2  )
#define DMAALTSET_UARTTX               (0x1   << 2  )
#define DMAALTSET_UARTTX_DIS           (0x0   << 2  ) /* DIS                      */
#define DMAALTSET_UARTTX_EN            (0x1   << 2  ) /* EN                       */

/* DMAALTSET[SPI1RX] - DMA SPI 1 RX */
#define DMAALTSET_SPI1RX_BBA           (*(volatile unsigned long *) 0x42200604)
#define DMAALTSET_SPI1RX_MSK           (0x1   << 1  )
#define DMAALTSET_SPI1RX               (0x1   << 1  )
#define DMAALTSET_SPI1RX_DIS           (0x0   << 1  ) /* DIS                      */
#define DMAALTSET_SPI1RX_EN            (0x1   << 1  ) /* EN                       */

/* DMAALTSET[SPI1TX] - DMA SPI 1 TX */
#define DMAALTSET_SPI1TX_BBA           (*(volatile unsigned long *) 0x42200600)
#define DMAALTSET_SPI1TX_MSK           (0x1   << 0  )
#define DMAALTSET_SPI1TX               (0x1   << 0  )
#define DMAALTSET_SPI1TX_DIS           (0x0   << 0  ) /* DIS                      */
#define DMAALTSET_SPI1TX_EN            (0x1   << 0  ) /* EN                       */

/* Reset Value for DMAALTCLR*/
#define DMAALTCLR_RVAL                 0x0 

/* DMAALTCLR[UART1RX] - DMA UART1 RX */
#define DMAALTCLR_UART1RX_BBA             (*(volatile unsigned long *) 0x422006BC)
#define DMAALTCLR_UART1RX_MSK             (0x1   << 15 )
#define DMAALTCLR_UART1RX                 (0x1   << 15 )
#define DMAALTCLR_UART1RX_DIS             (0x0   << 15 ) /* DIS                      */
#define DMAALTCLR_UART1RX_EN              (0x1   << 15 ) /* EN                       */
/* DMAALTCLR[UART1TX] - DMA UART1 TX */
#define DMAALTCLR_UART1TX_BBA             (*(volatile unsigned long *) 0x422006B8)
#define DMAALTCLR_UART1TX_MSK             (0x1   << 14 )
#define DMAALTCLR_UART1TX                 (0x1   << 14 )
#define DMAALTCLR_UART1TX_DIS             (0x0   << 14 ) /* DIS                      */
#define DMAALTCLR_UART1TX_EN              (0x1   << 14 ) /* EN                       */
/* DMAALTCLR[SPI0RX] - DMA SPI 0 RX */
#define DMAALTCLR_SPI0RX_BBA             (*(volatile unsigned long *) 0x422006B4)
#define DMAALTCLR_SPI0RX_MSK             (0x1   << 13 )
#define DMAALTCLR_SPI0RX                 (0x1   << 13 )
#define DMAALTCLR_SPI0RX_DIS             (0x0   << 13 ) /* DIS                      */
#define DMAALTCLR_SPI0RX_EN              (0x1   << 13 ) /* EN                       */
/* DMAALTCLR[SPI0TX] - DMA SPI 0 TX */
#define DMAALTCLR_SPI0TX_BBA             (*(volatile unsigned long *) 0x422006B0)
#define DMAALTCLR_SPI0TX_MSK             (0x1   << 12 )
#define DMAALTCLR_SPI0TX                 (0x1   << 12 )
#define DMAALTCLR_SPI0TX_DIS             (0x0   << 12 ) /* DIS                      */
#define DMAALTCLR_SPI0TX_EN              (0x1   << 12 ) /* EN                       */


/* DMAALTCLR[SINC2] - SINC2 Output Step detection */
#define DMAALTCLR_SINC2_BBA            (*(volatile unsigned long *) 0x422006AC)
#define DMAALTCLR_SINC2_MSK            (0x1   << 11 )
#define DMAALTCLR_SINC2                (0x1   << 11 )
#define DMAALTCLR_SINC2_DIS            (0x0   << 11 ) /* DIS                      */
#define DMAALTCLR_SINC2_EN             (0x1   << 11 ) /* EN                       */

/* DMAALTCLR[ADC1] - ADC1 */
#define DMAALTCLR_ADC1_BBA             (*(volatile unsigned long *) 0x422006A8)
#define DMAALTCLR_ADC1_MSK             (0x1   << 10 )
#define DMAALTCLR_ADC1                 (0x1   << 10 )
#define DMAALTCLR_ADC1_DIS             (0x0   << 10 ) /* DIS                      */
#define DMAALTCLR_ADC1_EN              (0x1   << 10 ) /* EN                       */

/* DMAALTCLR[DAC] - DAC DMA Output */
#define DMAALTCLR_DAC_BBA              (*(volatile unsigned long *) 0x422006A0)
#define DMAALTCLR_DAC_MSK              (0x1   << 8  )
#define DMAALTCLR_DAC                  (0x1   << 8  )
#define DMAALTCLR_DAC_DIS              (0x0   << 8  ) /* DIS                      */
#define DMAALTCLR_DAC_EN               (0x1   << 8  ) /* EN                       */

/* DMAALTCLR[I2CMRX] - DMA I2C Master RX */
#define DMAALTCLR_I2CMRX_BBA           (*(volatile unsigned long *) 0x4220069C)
#define DMAALTCLR_I2CMRX_MSK           (0x1   << 7  )
#define DMAALTCLR_I2CMRX               (0x1   << 7  )
#define DMAALTCLR_I2CMRX_DIS           (0x0   << 7  ) /* DIS                      */
#define DMAALTCLR_I2CMRX_EN            (0x1   << 7  ) /* EN                       */

/* DMAALTCLR[I2CMTX] - DMA I2C Master TX */
#define DMAALTCLR_I2CMTX_BBA           (*(volatile unsigned long *) 0x42200698)
#define DMAALTCLR_I2CMTX_MSK           (0x1   << 6  )
#define DMAALTCLR_I2CMTX               (0x1   << 6  )
#define DMAALTCLR_I2CMTX_DIS           (0x0   << 6  ) /* DIS                      */
#define DMAALTCLR_I2CMTX_EN            (0x1   << 6  ) /* EN                       */

/* DMAALTCLR[I2CSRX] - DMA I2C Slave RX */
#define DMAALTCLR_I2CSRX_BBA           (*(volatile unsigned long *) 0x42200694)
#define DMAALTCLR_I2CSRX_MSK           (0x1   << 5  )
#define DMAALTCLR_I2CSRX               (0x1   << 5  )
#define DMAALTCLR_I2CSRX_DIS           (0x0   << 5  ) /* DIS                      */
#define DMAALTCLR_I2CSRX_EN            (0x1   << 5  ) /* EN                       */

/* DMAALTCLR[I2CSTX] - DMA I2C Slave TX */
#define DMAALTCLR_I2CSTX_BBA           (*(volatile unsigned long *) 0x42200690)
#define DMAALTCLR_I2CSTX_MSK           (0x1   << 4  )
#define DMAALTCLR_I2CSTX               (0x1   << 4  )
#define DMAALTCLR_I2CSTX_DIS           (0x0   << 4  ) /* DIS                      */
#define DMAALTCLR_I2CSTX_EN            (0x1   << 4  ) /* EN                       */

/* DMAALTCLR[UARTRX] - DMA UART RX */
#define DMAALTCLR_UARTRX_BBA           (*(volatile unsigned long *) 0x4220068C)
#define DMAALTCLR_UARTRX_MSK           (0x1   << 3  )
#define DMAALTCLR_UARTRX               (0x1   << 3  )
#define DMAALTCLR_UARTRX_DIS           (0x0   << 3  ) /* DIS                      */
#define DMAALTCLR_UARTRX_EN            (0x1   << 3  ) /* EN                       */

/* DMAALTCLR[UARTTX] - DMA UART TX */
#define DMAALTCLR_UARTTX_BBA           (*(volatile unsigned long *) 0x42200688)
#define DMAALTCLR_UARTTX_MSK           (0x1   << 2  )
#define DMAALTCLR_UARTTX               (0x1   << 2  )
#define DMAALTCLR_UARTTX_DIS           (0x0   << 2  ) /* DIS                      */
#define DMAALTCLR_UARTTX_EN            (0x1   << 2  ) /* EN                       */

/* DMAALTCLR[SPI1RX] - DMA SPI 1 RX */
#define DMAALTCLR_SPI1RX_BBA           (*(volatile unsigned long *) 0x42200684)
#define DMAALTCLR_SPI1RX_MSK           (0x1   << 1  )
#define DMAALTCLR_SPI1RX               (0x1   << 1  )
#define DMAALTCLR_SPI1RX_DIS           (0x0   << 1  ) /* DIS                      */
#define DMAALTCLR_SPI1RX_EN            (0x1   << 1  ) /* EN                       */

/* DMAALTCLR[SPI1TX] - DMA SPI 1 TX */
#define DMAALTCLR_SPI1TX_BBA           (*(volatile unsigned long *) 0x42200680)
#define DMAALTCLR_SPI1TX_MSK           (0x1   << 0  )
#define DMAALTCLR_SPI1TX               (0x1   << 0  )
#define DMAALTCLR_SPI1TX_DIS           (0x0   << 0  ) /* DIS                      */
#define DMAALTCLR_SPI1TX_EN            (0x1   << 0  ) /* EN                       */

/* Reset Value for DMAPRISET*/
#define DMAPRISET_RVAL                 0x0 
/*DMAPRISET[UART1RX] - DMA UART1 RX */
#define DMAPRISET_UART1RX_BBA             (*(volatile unsigned long *) 0x4220073C)
#define DMAPRISET_UART1RX_MSK             (0x1   << 15 )
#define DMAPRISET_UART1RX                 (0x1   << 15 )
#define DMAPRISET_UART1RX_DIS             (0x0   << 15 ) /* DIS                      */
#define DMAPRISET_UART1RX_EN              (0x1   << 15 ) /* EN                       */
/* DMAPRISET[UART1TX] - DMA UART1 TX */
#define DMAPRISET_UART1TX_BBA             (*(volatile unsigned long *) 0x42200738)
#define DMAPRISET_UART1TX_MSK             (0x1   << 14 )
#define DMAPRISET_UART1TX                 (0x1   << 14 )
#define DMAPRISET_UART1TX_DIS             (0x0   << 14 ) /* DIS                      */
#define DMAPRISET_UART1TX_EN              (0x1   << 14 ) /* EN                       */
/* DMAPRISET[SPI0RX] - DMA SPI 0 RX */
#define DMAPRISET_SPI0RX_BBA             (*(volatile unsigned long *) 0x42200734)
#define DMAPRISET_SPI0RX_MSK             (0x1   << 13 )
#define DMAPRISET_SPI0RX                 (0x1   << 13 )
#define DMAPRISET_SPI0RX_DIS             (0x0   << 13 ) /* DIS                      */
#define DMAPRISET_SPI0RX_EN              (0x1   << 13 ) /* EN                       */
/* DMAPRISET[SPI0TX] - DMA SPI 0 TX */
#define DMAPRISET_SPI0TX_BBA             (*(volatile unsigned long *) 0x42200730)
#define DMAPRISET_SPI0TX_MSK             (0x1   << 12 )
#define DMAPRISET_SPI0TX                 (0x1   << 12 )
#define DMAPRISET_SPI0TX_DIS             (0x0   << 12 ) /* DIS                      */
#define DMAPRISET_SPI0TX_EN              (0x1   << 12 ) /* EN                       */


/* DMAPRISET[SINC2] - SINC2 Output Step detection */
#define DMAPRISET_SINC2_BBA            (*(volatile unsigned long *) 0x4220072C)
#define DMAPRISET_SINC2_MSK            (0x1   << 11 )
#define DMAPRISET_SINC2                (0x1   << 11 )
#define DMAPRISET_SINC2_DIS            (0x0   << 11 ) /* DIS                      */
#define DMAPRISET_SINC2_EN             (0x1   << 11 ) /* EN                       */

/* DMAPRISET[ADC1] - ADC1 */
#define DMAPRISET_ADC1_BBA             (*(volatile unsigned long *) 0x42200728)
#define DMAPRISET_ADC1_MSK             (0x1   << 10 )
#define DMAPRISET_ADC1                 (0x1   << 10 )
#define DMAPRISET_ADC1_DIS             (0x0   << 10 ) /* DIS                      */
#define DMAPRISET_ADC1_EN              (0x1   << 10 ) /* EN                       */

/* DMAPRISET[DAC] - DAC DMA Output */
#define DMAPRISET_DAC_BBA              (*(volatile unsigned long *) 0x42200720)
#define DMAPRISET_DAC_MSK              (0x1   << 8  )
#define DMAPRISET_DAC                  (0x1   << 8  )
#define DMAPRISET_DAC_DIS              (0x0   << 8  ) /* DIS                      */
#define DMAPRISET_DAC_EN               (0x1   << 8  ) /* EN                       */

/* DMAPRISET[I2CMRX] - DMA I2C Master RX */
#define DMAPRISET_I2CMRX_BBA           (*(volatile unsigned long *) 0x4220071C)
#define DMAPRISET_I2CMRX_MSK           (0x1   << 7  )
#define DMAPRISET_I2CMRX               (0x1   << 7  )
#define DMAPRISET_I2CMRX_DIS           (0x0   << 7  ) /* DIS                      */
#define DMAPRISET_I2CMRX_EN            (0x1   << 7  ) /* EN                       */

/* DMAPRISET[I2CMTX] - DMA I2C Master TX */
#define DMAPRISET_I2CMTX_BBA           (*(volatile unsigned long *) 0x42200718)
#define DMAPRISET_I2CMTX_MSK           (0x1   << 6  )
#define DMAPRISET_I2CMTX               (0x1   << 6  )
#define DMAPRISET_I2CMTX_DIS           (0x0   << 6  ) /* DIS                      */
#define DMAPRISET_I2CMTX_EN            (0x1   << 6  ) /* EN                       */

/* DMAPRISET[I2CSRX] - DMA I2C Slave RX */
#define DMAPRISET_I2CSRX_BBA           (*(volatile unsigned long *) 0x42200714)
#define DMAPRISET_I2CSRX_MSK           (0x1   << 5  )
#define DMAPRISET_I2CSRX               (0x1   << 5  )
#define DMAPRISET_I2CSRX_DIS           (0x0   << 5  ) /* DIS                      */
#define DMAPRISET_I2CSRX_EN            (0x1   << 5  ) /* EN                       */

/* DMAPRISET[I2CSTX] - DMA I2C Slave TX */
#define DMAPRISET_I2CSTX_BBA           (*(volatile unsigned long *) 0x42200710)
#define DMAPRISET_I2CSTX_MSK           (0x1   << 4  )
#define DMAPRISET_I2CSTX               (0x1   << 4  )
#define DMAPRISET_I2CSTX_DIS           (0x0   << 4  ) /* DIS                      */
#define DMAPRISET_I2CSTX_EN            (0x1   << 4  ) /* EN                       */

/* DMAPRISET[UARTRX] - DMA UART RX */
#define DMAPRISET_UARTRX_BBA           (*(volatile unsigned long *) 0x4220070C)
#define DMAPRISET_UARTRX_MSK           (0x1   << 3  )
#define DMAPRISET_UARTRX               (0x1   << 3  )
#define DMAPRISET_UARTRX_DIS           (0x0   << 3  ) /* DIS                      */
#define DMAPRISET_UARTRX_EN            (0x1   << 3  ) /* EN                       */

/* DMAPRISET[UARTTX] - DMA UART TX */
#define DMAPRISET_UARTTX_BBA           (*(volatile unsigned long *) 0x42200708)
#define DMAPRISET_UARTTX_MSK           (0x1   << 2  )
#define DMAPRISET_UARTTX               (0x1   << 2  )
#define DMAPRISET_UARTTX_DIS           (0x0   << 2  ) /* DIS                      */
#define DMAPRISET_UARTTX_EN            (0x1   << 2  ) /* EN                       */

/* DMAPRISET[SPI1RX] - DMA SPI 1 RX */
#define DMAPRISET_SPI1RX_BBA           (*(volatile unsigned long *) 0x42200704)
#define DMAPRISET_SPI1RX_MSK           (0x1   << 1  )
#define DMAPRISET_SPI1RX               (0x1   << 1  )
#define DMAPRISET_SPI1RX_DIS           (0x0   << 1  ) /* DIS                      */
#define DMAPRISET_SPI1RX_EN            (0x1   << 1  ) /* EN                       */

/* DMAPRISET[SPI1TX] - DMA SPI 1 TX */
#define DMAPRISET_SPI1TX_BBA           (*(volatile unsigned long *) 0x42200700)
#define DMAPRISET_SPI1TX_MSK           (0x1   << 0  )
#define DMAPRISET_SPI1TX               (0x1   << 0  )
#define DMAPRISET_SPI1TX_DIS           (0x0   << 0  ) /* DIS                      */
#define DMAPRISET_SPI1TX_EN            (0x1   << 0  ) /* EN                       */

/* Reset Value for DMAPRICLR*/
#define DMAPRICLR_RVAL                 0x0 

/*DMAPRICLR[UART1RX] - DMA UART1 RX */
#define DMAPRICLR_UART1RX_BBA             (*(volatile unsigned long *) 0x422007BC)
#define DMAPRICLR_UART1RX_MSK             (0x1   << 15 )
#define DMAPRICLR_UART1RX                 (0x1   << 15 )
#define DMAPRICLR_UART1RX_DIS             (0x0   << 15 ) /* DIS                      */
#define DMAPRICLR_UART1RX_EN              (0x1   << 15 ) /* EN                       */
/* DMAPRICLR[UART1TX] - DMA UART1 TX */
#define DMAPRICLR_UART1TX_BBA             (*(volatile unsigned long *) 0x422007B8)
#define DMAPRICLR_UART1TX_MSK             (0x1   << 14 )
#define DMAPRICLR_UART1TX                 (0x1   << 14 )
#define DMAPRICLR_UART1TX_DIS             (0x0   << 14 ) /* DIS                      */
#define DMAPRICLR_UART1TX_EN              (0x1   << 14 ) /* EN                       */
/* DMAPRICLR[SPI0RX] - DMA SPI 0 RX */
#define DMAPRICLR_SPI0RX_BBA             (*(volatile unsigned long *) 0x422007B4)
#define DMAPRICLR_SPI0RX_MSK             (0x1   << 13 )
#define DMAPRICLR_SPI0RX                 (0x1   << 13 )
#define DMAPRICLR_SPI0RX_DIS             (0x0   << 13 ) /* DIS                      */
#define DMAPRICLR_SPI0RX_EN              (0x1   << 13 ) /* EN                       */
/* DMAPRICLR[SPI0TX] - DMA SPI 0 TX */
#define DMAPRICLR_SPI0TX_BBA             (*(volatile unsigned long *) 0x422007B0)
#define DMAPRICLR_SPI0TX_MSK             (0x1   << 12 )
#define DMAPRICLR_SPI0TX                 (0x1   << 12 )
#define DMAPRICLR_SPI0TX_DIS             (0x0   << 12 ) /* DIS                      */
#define DMAPRICLR_SPI0TX_EN              (0x1   << 12 ) /* EN                       */

/* DMAPRICLR[SINC2] - SINC2 Output Step detection */
#define DMAPRICLR_SINC2_BBA            (*(volatile unsigned long *) 0x422007AC)
#define DMAPRICLR_SINC2_MSK            (0x1   << 11 )
#define DMAPRICLR_SINC2                (0x1   << 11 )
#define DMAPRICLR_SINC2_DIS            (0x0   << 11 ) /* DIS                      */
#define DMAPRICLR_SINC2_EN             (0x1   << 11 ) /* EN                       */

/* DMAPRICLR[ADC1] - ADC1 */
#define DMAPRICLR_ADC1_BBA             (*(volatile unsigned long *) 0x422007A8)
#define DMAPRICLR_ADC1_MSK             (0x1   << 10 )
#define DMAPRICLR_ADC1                 (0x1   << 10 )
#define DMAPRICLR_ADC1_DIS             (0x0   << 10 ) /* DIS                      */
#define DMAPRICLR_ADC1_EN              (0x1   << 10 ) /* EN                       */

/* DMAPRICLR[DAC] - DAC DMA Output */
#define DMAPRICLR_DAC_BBA              (*(volatile unsigned long *) 0x422007A0)
#define DMAPRICLR_DAC_MSK              (0x1   << 8  )
#define DMAPRICLR_DAC                  (0x1   << 8  )
#define DMAPRICLR_DAC_DIS              (0x0   << 8  ) /* DIS                      */
#define DMAPRICLR_DAC_EN               (0x1   << 8  ) /* EN                       */

/* DMAPRICLR[I2CMRX] - DMA I2C Master RX */
#define DMAPRICLR_I2CMRX_BBA           (*(volatile unsigned long *) 0x4220079C)
#define DMAPRICLR_I2CMRX_MSK           (0x1   << 7  )
#define DMAPRICLR_I2CMRX               (0x1   << 7  )
#define DMAPRICLR_I2CMRX_DIS           (0x0   << 7  ) /* DIS                      */
#define DMAPRICLR_I2CMRX_EN            (0x1   << 7  ) /* EN                       */

/* DMAPRICLR[I2CMTX] - DMA I2C Master TX */
#define DMAPRICLR_I2CMTX_BBA           (*(volatile unsigned long *) 0x42200798)
#define DMAPRICLR_I2CMTX_MSK           (0x1   << 6  )
#define DMAPRICLR_I2CMTX               (0x1   << 6  )
#define DMAPRICLR_I2CMTX_DIS           (0x0   << 6  ) /* DIS                      */
#define DMAPRICLR_I2CMTX_EN            (0x1   << 6  ) /* EN                       */

/* DMAPRICLR[I2CSRX] - DMA I2C Slave RX */
#define DMAPRICLR_I2CSRX_BBA           (*(volatile unsigned long *) 0x42200794)
#define DMAPRICLR_I2CSRX_MSK           (0x1   << 5  )
#define DMAPRICLR_I2CSRX               (0x1   << 5  )
#define DMAPRICLR_I2CSRX_DIS           (0x0   << 5  ) /* DIS                      */
#define DMAPRICLR_I2CSRX_EN            (0x1   << 5  ) /* EN                       */

/* DMAPRICLR[I2CSTX] - DMA I2C Slave TX */
#define DMAPRICLR_I2CSTX_BBA           (*(volatile unsigned long *) 0x42200790)
#define DMAPRICLR_I2CSTX_MSK           (0x1   << 4  )
#define DMAPRICLR_I2CSTX               (0x1   << 4  )
#define DMAPRICLR_I2CSTX_DIS           (0x0   << 4  ) /* DIS                      */
#define DMAPRICLR_I2CSTX_EN            (0x1   << 4  ) /* EN                       */

/* DMAPRICLR[UARTRX] - DMA UART RX */
#define DMAPRICLR_UARTRX_BBA           (*(volatile unsigned long *) 0x4220078C)
#define DMAPRICLR_UARTRX_MSK           (0x1   << 3  )
#define DMAPRICLR_UARTRX               (0x1   << 3  )
#define DMAPRICLR_UARTRX_DIS           (0x0   << 3  ) /* DIS                      */
#define DMAPRICLR_UARTRX_EN            (0x1   << 3  ) /* EN                       */

/* DMAPRICLR[UARTTX] - DMA UART TX */
#define DMAPRICLR_UARTTX_BBA           (*(volatile unsigned long *) 0x42200788)
#define DMAPRICLR_UARTTX_MSK           (0x1   << 2  )
#define DMAPRICLR_UARTTX               (0x1   << 2  )
#define DMAPRICLR_UARTTX_DIS           (0x0   << 2  ) /* DIS                      */
#define DMAPRICLR_UARTTX_EN            (0x1   << 2  ) /* EN                       */

/* DMAPRICLR[SPI1RX] - DMA SPI 1 RX */
#define DMAPRICLR_SPI1RX_BBA           (*(volatile unsigned long *) 0x42200784)
#define DMAPRICLR_SPI1RX_MSK           (0x1   << 1  )
#define DMAPRICLR_SPI1RX               (0x1   << 1  )
#define DMAPRICLR_SPI1RX_DIS           (0x0   << 1  ) /* DIS                      */
#define DMAPRICLR_SPI1RX_EN            (0x1   << 1  ) /* EN                       */

/* DMAPRICLR[SPI1TX] - DMA SPI 1 TX */
#define DMAPRICLR_SPI1TX_BBA           (*(volatile unsigned long *) 0x42200780)
#define DMAPRICLR_SPI1TX_MSK           (0x1   << 0  )
#define DMAPRICLR_SPI1TX               (0x1   << 0  )
#define DMAPRICLR_SPI1TX_DIS           (0x0   << 0  ) /* DIS                      */
#define DMAPRICLR_SPI1TX_EN            (0x1   << 0  ) /* EN                       */

/* Reset Value for DMAERRCLR*/
#define DMAERRCLR_RVAL                 0x0 

/* DMAERRCLR[ERROR] - DMA Error status */
#define DMAERRCLR_ERROR_BBA            (*(volatile unsigned long *) 0x42200980)
#define DMAERRCLR_ERROR_MSK            (0x1   << 0  )
#define DMAERRCLR_ERROR                (0x1   << 0  )
#define DMAERRCLR_ERROR_DIS            (0x0   << 0  ) /* DIS                      */
#define DMAERRCLR_ERROR_EN             (0x1   << 0  ) /* EN                       */

/* Reset Value for DMABSSET*/
#define DMABSSET_RVAL                  0x0 

/* DMABSSET[CHBSWAPSET] - Byte swap status */
#define DMABSSET_CHBSWAPSET_MSK        (0x3FFF << 0  )

/* Reset Value for DMABSCLR*/
#define DMABSCLR_RVAL                  0x0 

/* DMABSCLR[CHBSWAPCLR] - Disable byte swap */
#define DMABSCLR_CHBSWAPCLR_MSK        (0x3FFF << 0  )
// ------------------------------------------------------------------------------------------------
// -----                                        NVIC                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Nested Vectored Interrupt Controller (pADI_NVIC)
  */

#if (__NO_MMR_STRUCTS__==0)
#else // (__NO_MMR_STRUCTS__==0)
#define          ICTR                                       (*(volatile unsigned long      *) 0xE000E004)
#define          STCSR                                      (*(volatile unsigned long      *) 0xE000E010)
#define          STRVR                                      (*(volatile unsigned long      *) 0xE000E014)
#define          STCVR                                      (*(volatile unsigned long      *) 0xE000E018)
#define          STCR                                       (*(volatile unsigned long      *) 0xE000E01C)
#define          ISER0                                      (*(volatile unsigned long      *) 0xE000E100)
#define          ISER1                                      (*(volatile unsigned long      *) 0xE000E104)
#define          ICER0                                      (*(volatile unsigned long      *) 0xE000E180)
#define          ICER1                                      (*(volatile unsigned long      *) 0xE000E184)
#define          ISPR0                                      (*(volatile unsigned long      *) 0xE000E200)
#define          ISPR1                                      (*(volatile unsigned long      *) 0xE000E204)
#define          ICPR0                                      (*(volatile unsigned long      *) 0xE000E280)
#define          ICPR1                                      (*(volatile unsigned long      *) 0xE000E284)
#define          IABR0                                      (*(volatile unsigned long      *) 0xE000E300)
#define          IABR1                                      (*(volatile unsigned long      *) 0xE000E304)
#define          IPR0                                       (*(volatile unsigned long      *) 0xE000E400)
#define          IPR1                                       (*(volatile unsigned long      *) 0xE000E404)
#define          IPR2                                       (*(volatile unsigned long      *) 0xE000E408)
#define          IPR3                                       (*(volatile unsigned long      *) 0xE000E40C)
#define          IPR4                                       (*(volatile unsigned long      *) 0xE000E410)
#define          IPR5                                       (*(volatile unsigned long      *) 0xE000E414)
#define          IPR6                                       (*(volatile unsigned long      *) 0xE000E418)
#define          IPR7                                       (*(volatile unsigned long      *) 0xE000E41C)
#define          IPR8                                       (*(volatile unsigned long      *) 0xE000E420)
#define          IPR9                                       (*(volatile unsigned long      *) 0xE000E424)
#define          CPUID                                      (*(volatile unsigned long      *) 0xE000ED00)
#define          ICSR                                       (*(volatile unsigned long      *) 0xE000ED04)
#define          VTOR                                       (*(volatile unsigned long      *) 0xE000ED08)
#define          AIRCR                                      (*(volatile unsigned long      *) 0xE000ED0C)
#define          SCR                                        (*(volatile unsigned long      *) 0xE000ED10)
#define          CCR                                        (*(volatile unsigned long      *) 0xE000ED14)
#define          SHPR1                                      (*(volatile unsigned long      *) 0xE000ED18)
#define          SHPR2                                      (*(volatile unsigned long      *) 0xE000ED1C)
#define          SHPR3                                      (*(volatile unsigned long      *) 0xE000ED20)
#define          SHCSR                                      (*(volatile unsigned long      *) 0xE000ED24)
#define          CFSR                                       (*(volatile unsigned long      *) 0xE000ED28)
#define          HFSR                                       (*(volatile unsigned long      *) 0xE000ED2C)
#define          MMFAR                                      (*(volatile unsigned long      *) 0xE000ED34)
#define          BFAR                                       (*(volatile unsigned long      *) 0xE000ED38)
#define          STIR                                       (*(volatile unsigned long      *) 0xE000EF00)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for ICTR*/
#define ICTR_RVAL                      0x1 

/* ICTR[INTLINESNUM] - Total number of interrupt lines in groups of 32 */
#define ICTR_INTLINESNUM_MSK           (0xF   << 0  )

/* Reset Value for STCSR*/
#define STCSR_RVAL                     0x0 

/* STCSR[COUNTFLAG] - Returns 1 if timer counted to 0 since last time this register was read */
#define STCSR_COUNTFLAG_MSK            (0x1   << 16 )
#define STCSR_COUNTFLAG                (0x1   << 16 )
#define STCSR_COUNTFLAG_DIS            (0x0   << 16 ) /* DIS                      */
#define STCSR_COUNTFLAG_EN             (0x1   << 16 ) /* EN                       */

/* STCSR[CLKSOURCE] - clock source used for SysTick */
#define STCSR_CLKSOURCE_MSK            (0x1   << 2  )
#define STCSR_CLKSOURCE                (0x1   << 2  )
#define STCSR_CLKSOURCE_DIS            (0x0   << 2  ) /* DIS                      */
#define STCSR_CLKSOURCE_EN             (0x1   << 2  ) /* EN                       */

/* STCSR[TICKINT] - If 1, counting down to 0 will cause the SysTick exception to pended. */
#define STCSR_TICKINT_MSK              (0x1   << 1  )
#define STCSR_TICKINT                  (0x1   << 1  )
#define STCSR_TICKINT_DIS              (0x0   << 1  ) /* DIS                      */
#define STCSR_TICKINT_EN               (0x1   << 1  ) /* EN                       */

/* STCSR[ENABLE] - Enable bit */
#define STCSR_ENABLE_MSK               (0x1   << 0  )
#define STCSR_ENABLE                   (0x1   << 0  )
#define STCSR_ENABLE_DIS               (0x0   << 0  ) /* DIS                      */
#define STCSR_ENABLE_EN                (0x1   << 0  ) /* EN                       */

/* Reset Value for STRVR*/
#define STRVR_RVAL                     0x0 

/* STRVR[RELOAD] - Value to load into the Current Value register when the counter reaches 0 */
#define STRVR_RELOAD_MSK               (0xFFFFFF << 0  )

/* Reset Value for STCVR*/
#define STCVR_RVAL                     0x0 

/* STCVR[CURRENT] - Current counter value */
#define STCVR_CURRENT_MSK              (0xFFFFFFFF << 0  )

/* Reset Value for STCR*/
#define STCR_RVAL                      0x0 

/* STCR[NOREF] - If reads as 1, the Reference clock is not provided */
#define STCR_NOREF_MSK                 (0x1   << 31 )
#define STCR_NOREF                     (0x1   << 31 )
#define STCR_NOREF_DIS                 (0x0   << 31 ) /* DIS                      */
#define STCR_NOREF_EN                  (0x1   << 31 ) /* EN                       */

/* STCR[SKEW] - If reads as 1, the calibration value for 10ms is inexact */
#define STCR_SKEW_MSK                  (0x1   << 30 )
#define STCR_SKEW                      (0x1   << 30 )
#define STCR_SKEW_DIS                  (0x0   << 30 ) /* DIS                      */
#define STCR_SKEW_EN                   (0x1   << 30 ) /* EN                       */

/* STCR[TENMS] - An optional Reload value to be used for 10ms (100Hz) timing */
#define STCR_TENMS_MSK                 (0xFFFFFF << 0  )

/* Reset Value for ISER0*/
#define ISER0_RVAL                     0x0 

/* ISER0[DMADAC] -  */
#define ISER0_DMADAC_MSK               (0x1   << 31 )
#define ISER0_DMADAC                   (0x1   << 31 )
#define ISER0_DMADAC_DIS               (0x0   << 31 ) /* DIS                      */
#define ISER0_DMADAC_EN                (0x1   << 31 ) /* EN                       */

/* ISER0[DMAI2CMRX] -  */
#define ISER0_DMAI2CMRX_MSK            (0x1   << 30 )
#define ISER0_DMAI2CMRX                (0x1   << 30 )
#define ISER0_DMAI2CMRX_DIS            (0x0   << 30 ) /* DIS                      */
#define ISER0_DMAI2CMRX_EN             (0x1   << 30 ) /* EN                       */

/* ISER0[DMAI2CMTX] -  */
#define ISER0_DMAI2CMTX_MSK            (0x1   << 29 )
#define ISER0_DMAI2CMTX                (0x1   << 29 )
#define ISER0_DMAI2CMTX_DIS            (0x0   << 29 ) /* DIS                      */
#define ISER0_DMAI2CMTX_EN             (0x1   << 29 ) /* EN                       */

/* ISER0[DMAI2CSRX] -  */
#define ISER0_DMAI2CSRX_MSK            (0x1   << 28 )
#define ISER0_DMAI2CSRX                (0x1   << 28 )
#define ISER0_DMAI2CSRX_DIS            (0x0   << 28 ) /* DIS                      */
#define ISER0_DMAI2CSRX_EN             (0x1   << 28 ) /* EN                       */

/* ISER0[DMAI2CSTX] -  */
#define ISER0_DMAI2CSTX_MSK            (0x1   << 27 )
#define ISER0_DMAI2CSTX                (0x1   << 27 )
#define ISER0_DMAI2CSTX_DIS            (0x0   << 27 ) /* DIS                      */
#define ISER0_DMAI2CSTX_EN             (0x1   << 27 ) /* EN                       */

/* ISER0[DMAUARTRX] -  */
#define ISER0_DMAUARTRX_MSK            (0x1   << 26 )
#define ISER0_DMAUARTRX                (0x1   << 26 )
#define ISER0_DMAUARTRX_DIS            (0x0   << 26 ) /* DIS                      */
#define ISER0_DMAUARTRX_EN             (0x1   << 26 ) /* EN                       */

/* ISER0[DMAUARTTX] -  */
#define ISER0_DMAUARTTX_MSK            (0x1   << 25 )
#define ISER0_DMAUARTTX                (0x1   << 25 )
#define ISER0_DMAUARTTX_DIS            (0x0   << 25 ) /* DIS                      */
#define ISER0_DMAUARTTX_EN             (0x1   << 25 ) /* EN                       */

/* ISER0[DMASPI1RX] -  */
#define ISER0_DMASPI1RX_MSK            (0x1   << 24 )
#define ISER0_DMASPI1RX                (0x1   << 24 )
#define ISER0_DMASPI1RX_DIS            (0x0   << 24 ) /* DIS                      */
#define ISER0_DMASPI1RX_EN             (0x1   << 24 ) /* EN                       */

/* ISER0[DMASPI1TX] -  */
#define ISER0_DMASPI1TX_MSK            (0x1   << 23 )
#define ISER0_DMASPI1TX                (0x1   << 23 )
#define ISER0_DMASPI1TX_DIS            (0x0   << 23 ) /* DIS                      */
#define ISER0_DMASPI1TX_EN             (0x1   << 23 ) /* EN                       */

/* ISER0[DMAERROR] -  */
#define ISER0_DMAERROR_MSK             (0x1   << 22 )
#define ISER0_DMAERROR                 (0x1   << 22 )
#define ISER0_DMAERROR_DIS             (0x0   << 22 ) /* DIS                      */
#define ISER0_DMAERROR_EN              (0x1   << 22 ) /* EN                       */

/* ISER0[I2CM] -  */
#define ISER0_I2CM_MSK                 (0x1   << 21 )
#define ISER0_I2CM                     (0x1   << 21 )
#define ISER0_I2CM_DIS                 (0x0   << 21 ) /* DIS                      */
#define ISER0_I2CM_EN                  (0x1   << 21 ) /* EN                       */

/* ISER0[I2CS] -  */
#define ISER0_I2CS_MSK                 (0x1   << 20 )
#define ISER0_I2CS                     (0x1   << 20 )
#define ISER0_I2CS_DIS                 (0x0   << 20 ) /* DIS                      */
#define ISER0_I2CS_EN                  (0x1   << 20 ) /* EN                       */

/* ISER0[SPI1] -  */
#define ISER0_SPI1_MSK                 (0x1   << 19 )
#define ISER0_SPI1                     (0x1   << 19 )
#define ISER0_SPI1_DIS                 (0x0   << 19 ) /* DIS                      */
#define ISER0_SPI1_EN                  (0x1   << 19 ) /* EN                       */

/* ISER0[SPI0] -  */
#define ISER0_SPI0_MSK                 (0x1   << 18 )
#define ISER0_SPI0                     (0x1   << 18 )
#define ISER0_SPI0_DIS                 (0x0   << 18 ) /* DIS                      */
#define ISER0_SPI0_EN                  (0x1   << 18 ) /* EN                       */

/* ISER0[UART] -  */
#define ISER0_UART_MSK                 (0x1   << 17 )
#define ISER0_UART                     (0x1   << 17 )
#define ISER0_UART_DIS                 (0x0   << 17 ) /* DIS                      */
#define ISER0_UART_EN                  (0x1   << 17 ) /* EN                       */

/* ISER0[FEE] -  */
#define ISER0_FEE_MSK                  (0x1   << 16 )
#define ISER0_FEE                      (0x1   << 16 )
#define ISER0_FEE_DIS                  (0x0   << 16 ) /* DIS                      */
#define ISER0_FEE_EN                   (0x1   << 16 ) /* EN                       */

/* ISER0[SINC2] -  */
#define ISER0_SINC2_MSK                (0x1   << 15 )
#define ISER0_SINC2                    (0x1   << 15 )
#define ISER0_SINC2_DIS                (0x0   << 15 ) /* DIS                      */
#define ISER0_SINC2_EN                 (0x1   << 15 ) /* EN                       */

/* ISER0[ADC1] -  */
#define ISER0_ADC1_MSK                 (0x1   << 14 )
#define ISER0_ADC1                     (0x1   << 14 )
#define ISER0_ADC1_DIS                 (0x0   << 14 ) /* DIS                      */
#define ISER0_ADC1_EN                  (0x1   << 14 ) /* EN                       */

/* ISER0[T1] -  */
#define ISER0_T1_MSK                   (0x1   << 12 )
#define ISER0_T1                       (0x1   << 12 )
#define ISER0_T1_DIS                   (0x0   << 12 ) /* DIS                      */
#define ISER0_T1_EN                    (0x1   << 12 ) /* EN                       */

/* ISER0[T0] -  */
#define ISER0_T0_MSK                   (0x1   << 11 )
#define ISER0_T0                       (0x1   << 11 )
#define ISER0_T0_DIS                   (0x0   << 11 ) /* DIS                      */
#define ISER0_T0_EN                    (0x1   << 11 ) /* EN                       */

/* ISER0[T3] -  */
#define ISER0_T3_MSK                   (0x1   << 9  )
#define ISER0_T3                       (0x1   << 9  )
#define ISER0_T3_DIS                   (0x0   << 9  ) /* DIS                      */
#define ISER0_T3_EN                    (0x1   << 9  ) /* EN                       */

/* ISER0[EXTINT7] -  */
#define ISER0_EXTINT7_MSK              (0x1   << 8  )
#define ISER0_EXTINT7                  (0x1   << 8  )
#define ISER0_EXTINT7_DIS              (0x0   << 8  ) /* DIS                      */
#define ISER0_EXTINT7_EN               (0x1   << 8  ) /* EN                       */

/* ISER0[EXTINT6] -  */
#define ISER0_EXTINT6_MSK              (0x1   << 7  )
#define ISER0_EXTINT6                  (0x1   << 7  )
#define ISER0_EXTINT6_DIS              (0x0   << 7  ) /* DIS                      */
#define ISER0_EXTINT6_EN               (0x1   << 7  ) /* EN                       */

/* ISER0[EXTINT5] -  */
#define ISER0_EXTINT5_MSK              (0x1   << 6  )
#define ISER0_EXTINT5                  (0x1   << 6  )
#define ISER0_EXTINT5_DIS              (0x0   << 6  ) /* DIS                      */
#define ISER0_EXTINT5_EN               (0x1   << 6  ) /* EN                       */

/* ISER0[EXTINT4] -  */
#define ISER0_EXTINT4_MSK              (0x1   << 5  )
#define ISER0_EXTINT4                  (0x1   << 5  )
#define ISER0_EXTINT4_DIS              (0x0   << 5  ) /* DIS                      */
#define ISER0_EXTINT4_EN               (0x1   << 5  ) /* EN                       */

/* ISER0[EXTINT3] -  */
#define ISER0_EXTINT3_MSK              (0x1   << 4  )
#define ISER0_EXTINT3                  (0x1   << 4  )
#define ISER0_EXTINT3_DIS              (0x0   << 4  ) /* DIS                      */
#define ISER0_EXTINT3_EN               (0x1   << 4  ) /* EN                       */

/* ISER0[EXTINT2] -  */
#define ISER0_EXTINT2_MSK              (0x1   << 3  )
#define ISER0_EXTINT2                  (0x1   << 3  )
#define ISER0_EXTINT2_DIS              (0x0   << 3  ) /* DIS                      */
#define ISER0_EXTINT2_EN               (0x1   << 3  ) /* EN                       */

/* ISER0[EXTINT1] -  */
#define ISER0_EXTINT1_MSK              (0x1   << 2  )
#define ISER0_EXTINT1                  (0x1   << 2  )
#define ISER0_EXTINT1_DIS              (0x0   << 2  ) /* DIS                      */
#define ISER0_EXTINT1_EN               (0x1   << 2  ) /* EN                       */

/* ISER0[EXTINT0] -  */
#define ISER0_EXTINT0_MSK              (0x1   << 1  )
#define ISER0_EXTINT0                  (0x1   << 1  )
#define ISER0_EXTINT0_DIS              (0x0   << 1  ) /* DIS                      */
#define ISER0_EXTINT0_EN               (0x1   << 1  ) /* EN                       */

/* ISER0[T2] -  */
#define ISER0_T2_MSK                   (0x1   << 0  )
#define ISER0_T2                       (0x1   << 0  )
#define ISER0_T2_DIS                   (0x0   << 0  ) /* DIS                      */
#define ISER0_T2_EN                    (0x1   << 0  ) /* EN                       */

/* Reset Value for ISER1*/
#define ISER1_RVAL                     0x0 

/* ISER1[PWM2] -  */
#define ISER1_PWM2_MSK                 (0x1   << 6  )
#define ISER1_PWM2                     (0x1   << 6  )
#define ISER1_PWM2_DIS                 (0x0   << 6  ) /* DIS                      */
#define ISER1_PWM2_EN                  (0x1   << 6  ) /* EN                       */

/* ISER1[PWM1] -  */
#define ISER1_PWM1_MSK                 (0x1   << 5  )
#define ISER1_PWM1                     (0x1   << 5  )
#define ISER1_PWM1_DIS                 (0x0   << 5  ) /* DIS                      */
#define ISER1_PWM1_EN                  (0x1   << 5  ) /* EN                       */

/* ISER1[PWM0] -  */
#define ISER1_PWM0_MSK                 (0x1   << 4  )
#define ISER1_PWM0                     (0x1   << 4  )
#define ISER1_PWM0_DIS                 (0x0   << 4  ) /* DIS                      */
#define ISER1_PWM0_EN                  (0x1   << 4  ) /* EN                       */

/* ISER1[PWMTRIP] -  */
#define ISER1_PWMTRIP_MSK              (0x1   << 3  )
#define ISER1_PWMTRIP                  (0x1   << 3  )
#define ISER1_PWMTRIP_DIS              (0x0   << 3  ) /* DIS                      */
#define ISER1_PWMTRIP_EN               (0x1   << 3  ) /* EN                       */

/* ISER1[DMASINC2] -  */
#define ISER1_DMASINC2_MSK             (0x1   << 2  )
#define ISER1_DMASINC2                 (0x1   << 2  )
#define ISER1_DMASINC2_DIS             (0x0   << 2  ) /* DIS                      */
#define ISER1_DMASINC2_EN              (0x1   << 2  ) /* EN                       */

/* ISER1[DMAADC1] -  */
#define ISER1_DMAADC1_MSK              (0x1   << 1  )
#define ISER1_DMAADC1                  (0x1   << 1  )
#define ISER1_DMAADC1_DIS              (0x0   << 1  ) /* DIS                      */
#define ISER1_DMAADC1_EN               (0x1   << 1  ) /* EN                       */

/* Reset Value for ICER0*/
#define ICER0_RVAL                     0x0 

/* ICER0[DMADAC] -  */
#define ICER0_DMADAC_MSK               (0x1   << 31 )
#define ICER0_DMADAC                   (0x1   << 31 )
#define ICER0_DMADAC_DIS               (0x0   << 31 ) /* DIS                      */
#define ICER0_DMADAC_EN                (0x1   << 31 ) /* EN                       */

/* ICER0[DMAI2CMRX] -  */
#define ICER0_DMAI2CMRX_MSK            (0x1   << 30 )
#define ICER0_DMAI2CMRX                (0x1   << 30 )
#define ICER0_DMAI2CMRX_DIS            (0x0   << 30 ) /* DIS                      */
#define ICER0_DMAI2CMRX_EN             (0x1   << 30 ) /* EN                       */

/* ICER0[DMAI2CMTX] -  */
#define ICER0_DMAI2CMTX_MSK            (0x1   << 29 )
#define ICER0_DMAI2CMTX                (0x1   << 29 )
#define ICER0_DMAI2CMTX_DIS            (0x0   << 29 ) /* DIS                      */
#define ICER0_DMAI2CMTX_EN             (0x1   << 29 ) /* EN                       */

/* ICER0[DMAI2CSRX] -  */
#define ICER0_DMAI2CSRX_MSK            (0x1   << 28 )
#define ICER0_DMAI2CSRX                (0x1   << 28 )
#define ICER0_DMAI2CSRX_DIS            (0x0   << 28 ) /* DIS                      */
#define ICER0_DMAI2CSRX_EN             (0x1   << 28 ) /* EN                       */

/* ICER0[DMAI2CSTX] -  */
#define ICER0_DMAI2CSTX_MSK            (0x1   << 27 )
#define ICER0_DMAI2CSTX                (0x1   << 27 )
#define ICER0_DMAI2CSTX_DIS            (0x0   << 27 ) /* DIS                      */
#define ICER0_DMAI2CSTX_EN             (0x1   << 27 ) /* EN                       */

/* ICER0[DMAUARTRX] -  */
#define ICER0_DMAUARTRX_MSK            (0x1   << 26 )
#define ICER0_DMAUARTRX                (0x1   << 26 )
#define ICER0_DMAUARTRX_DIS            (0x0   << 26 ) /* DIS                      */
#define ICER0_DMAUARTRX_EN             (0x1   << 26 ) /* EN                       */

/* ICER0[DMAUARTTX] -  */
#define ICER0_DMAUARTTX_MSK            (0x1   << 25 )
#define ICER0_DMAUARTTX                (0x1   << 25 )
#define ICER0_DMAUARTTX_DIS            (0x0   << 25 ) /* DIS                      */
#define ICER0_DMAUARTTX_EN             (0x1   << 25 ) /* EN                       */

/* ICER0[DMASPI1RX] -  */
#define ICER0_DMASPI1RX_MSK            (0x1   << 24 )
#define ICER0_DMASPI1RX                (0x1   << 24 )
#define ICER0_DMASPI1RX_DIS            (0x0   << 24 ) /* DIS                      */
#define ICER0_DMASPI1RX_EN             (0x1   << 24 ) /* EN                       */

/* ICER0[DMASPI1TX] -  */
#define ICER0_DMASPI1TX_MSK            (0x1   << 23 )
#define ICER0_DMASPI1TX                (0x1   << 23 )
#define ICER0_DMASPI1TX_DIS            (0x0   << 23 ) /* DIS                      */
#define ICER0_DMASPI1TX_EN             (0x1   << 23 ) /* EN                       */

/* ICER0[DMAERROR] -  */
#define ICER0_DMAERROR_MSK             (0x1   << 22 )
#define ICER0_DMAERROR                 (0x1   << 22 )
#define ICER0_DMAERROR_DIS             (0x0   << 22 ) /* DIS                      */
#define ICER0_DMAERROR_EN              (0x1   << 22 ) /* EN                       */

/* ICER0[I2CM] -  */
#define ICER0_I2CM_MSK                 (0x1   << 21 )
#define ICER0_I2CM                     (0x1   << 21 )
#define ICER0_I2CM_DIS                 (0x0   << 21 ) /* DIS                      */
#define ICER0_I2CM_EN                  (0x1   << 21 ) /* EN                       */

/* ICER0[I2CS] -  */
#define ICER0_I2CS_MSK                 (0x1   << 20 )
#define ICER0_I2CS                     (0x1   << 20 )
#define ICER0_I2CS_DIS                 (0x0   << 20 ) /* DIS                      */
#define ICER0_I2CS_EN                  (0x1   << 20 ) /* EN                       */

/* ICER0[SPI1] -  */
#define ICER0_SPI1_MSK                 (0x1   << 19 )
#define ICER0_SPI1                     (0x1   << 19 )
#define ICER0_SPI1_DIS                 (0x0   << 19 ) /* DIS                      */
#define ICER0_SPI1_EN                  (0x1   << 19 ) /* EN                       */

/* ICER0[SPI0] -  */
#define ICER0_SPI0_MSK                 (0x1   << 18 )
#define ICER0_SPI0                     (0x1   << 18 )
#define ICER0_SPI0_DIS                 (0x0   << 18 ) /* DIS                      */
#define ICER0_SPI0_EN                  (0x1   << 18 ) /* EN                       */

/* ICER0[UART] -  */
#define ICER0_UART_MSK                 (0x1   << 17 )
#define ICER0_UART                     (0x1   << 17 )
#define ICER0_UART_DIS                 (0x0   << 17 ) /* DIS                      */
#define ICER0_UART_EN                  (0x1   << 17 ) /* EN                       */

/* ICER0[FEE] -  */
#define ICER0_FEE_MSK                  (0x1   << 16 )
#define ICER0_FEE                      (0x1   << 16 )
#define ICER0_FEE_DIS                  (0x0   << 16 ) /* DIS                      */
#define ICER0_FEE_EN                   (0x1   << 16 ) /* EN                       */

/* ICER0[SINC2] -  */
#define ICER0_SINC2_MSK                (0x1   << 15 )
#define ICER0_SINC2                    (0x1   << 15 )
#define ICER0_SINC2_DIS                (0x0   << 15 ) /* DIS                      */
#define ICER0_SINC2_EN                 (0x1   << 15 ) /* EN                       */

/* ICER0[ADC1] -  */
#define ICER0_ADC1_MSK                 (0x1   << 14 )
#define ICER0_ADC1                     (0x1   << 14 )
#define ICER0_ADC1_DIS                 (0x0   << 14 ) /* DIS                      */
#define ICER0_ADC1_EN                  (0x1   << 14 ) /* EN                       */

/* ICER0[T1] -  */
#define ICER0_T1_MSK                   (0x1   << 12 )
#define ICER0_T1                       (0x1   << 12 )
#define ICER0_T1_DIS                   (0x0   << 12 ) /* DIS                      */
#define ICER0_T1_EN                    (0x1   << 12 ) /* EN                       */

/* ICER0[T0] -  */
#define ICER0_T0_MSK                   (0x1   << 11 )
#define ICER0_T0                       (0x1   << 11 )
#define ICER0_T0_DIS                   (0x0   << 11 ) /* DIS                      */
#define ICER0_T0_EN                    (0x1   << 11 ) /* EN                       */

/* ICER0[T3] -  */
#define ICER0_T3_MSK                   (0x1   << 9  )
#define ICER0_T3                       (0x1   << 9  )
#define ICER0_T3_DIS                   (0x0   << 9  ) /* DIS                      */
#define ICER0_T3_EN                    (0x1   << 9  ) /* EN                       */

/* ICER0[EXTINT7] -  */
#define ICER0_EXTINT7_MSK              (0x1   << 8  )
#define ICER0_EXTINT7                  (0x1   << 8  )
#define ICER0_EXTINT7_DIS              (0x0   << 8  ) /* DIS                      */
#define ICER0_EXTINT7_EN               (0x1   << 8  ) /* EN                       */

/* ICER0[EXTINT6] -  */
#define ICER0_EXTINT6_MSK              (0x1   << 7  )
#define ICER0_EXTINT6                  (0x1   << 7  )
#define ICER0_EXTINT6_DIS              (0x0   << 7  ) /* DIS                      */
#define ICER0_EXTINT6_EN               (0x1   << 7  ) /* EN                       */

/* ICER0[EXTINT5] -  */
#define ICER0_EXTINT5_MSK              (0x1   << 6  )
#define ICER0_EXTINT5                  (0x1   << 6  )
#define ICER0_EXTINT5_DIS              (0x0   << 6  ) /* DIS                      */
#define ICER0_EXTINT5_EN               (0x1   << 6  ) /* EN                       */

/* ICER0[EXTINT4] -  */
#define ICER0_EXTINT4_MSK              (0x1   << 5  )
#define ICER0_EXTINT4                  (0x1   << 5  )
#define ICER0_EXTINT4_DIS              (0x0   << 5  ) /* DIS                      */
#define ICER0_EXTINT4_EN               (0x1   << 5  ) /* EN                       */

/* ICER0[EXTINT3] -  */
#define ICER0_EXTINT3_MSK              (0x1   << 4  )
#define ICER0_EXTINT3                  (0x1   << 4  )
#define ICER0_EXTINT3_DIS              (0x0   << 4  ) /* DIS                      */
#define ICER0_EXTINT3_EN               (0x1   << 4  ) /* EN                       */

/* ICER0[EXTINT2] -  */
#define ICER0_EXTINT2_MSK              (0x1   << 3  )
#define ICER0_EXTINT2                  (0x1   << 3  )
#define ICER0_EXTINT2_DIS              (0x0   << 3  ) /* DIS                      */
#define ICER0_EXTINT2_EN               (0x1   << 3  ) /* EN                       */

/* ICER0[EXTINT1] -  */
#define ICER0_EXTINT1_MSK              (0x1   << 2  )
#define ICER0_EXTINT1                  (0x1   << 2  )
#define ICER0_EXTINT1_DIS              (0x0   << 2  ) /* DIS                      */
#define ICER0_EXTINT1_EN               (0x1   << 2  ) /* EN                       */

/* ICER0[EXTINT0] -  */
#define ICER0_EXTINT0_MSK              (0x1   << 1  )
#define ICER0_EXTINT0                  (0x1   << 1  )
#define ICER0_EXTINT0_DIS              (0x0   << 1  ) /* DIS                      */
#define ICER0_EXTINT0_EN               (0x1   << 1  ) /* EN                       */

/* ICER0[T2] -  */
#define ICER0_T2_MSK                   (0x1   << 0  )
#define ICER0_T2                       (0x1   << 0  )
#define ICER0_T2_DIS                   (0x0   << 0  ) /* DIS                      */
#define ICER0_T2_EN                    (0x1   << 0  ) /* EN                       */

/* Reset Value for ICER1*/
#define ICER1_RVAL                     0x0 

/* ICER1[PWM2] -  */
#define ICER1_PWM2_MSK                 (0x1   << 6  )
#define ICER1_PWM2                     (0x1   << 6  )
#define ICER1_PWM2_DIS                 (0x0   << 6  ) /* DIS                      */
#define ICER1_PWM2_EN                  (0x1   << 6  ) /* EN                       */

/* ICER1[PWM1] -  */
#define ICER1_PWM1_MSK                 (0x1   << 5  )
#define ICER1_PWM1                     (0x1   << 5  )
#define ICER1_PWM1_DIS                 (0x0   << 5  ) /* DIS                      */
#define ICER1_PWM1_EN                  (0x1   << 5  ) /* EN                       */

/* ICER1[PWM0] -  */
#define ICER1_PWM0_MSK                 (0x1   << 4  )
#define ICER1_PWM0                     (0x1   << 4  )
#define ICER1_PWM0_DIS                 (0x0   << 4  ) /* DIS                      */
#define ICER1_PWM0_EN                  (0x1   << 4  ) /* EN                       */

/* ICER1[PWMTRIP] -  */
#define ICER1_PWMTRIP_MSK              (0x1   << 3  )
#define ICER1_PWMTRIP                  (0x1   << 3  )
#define ICER1_PWMTRIP_DIS              (0x0   << 3  ) /* DIS                      */
#define ICER1_PWMTRIP_EN               (0x1   << 3  ) /* EN                       */

/* ICER1[DMASINC2] -  */
#define ICER1_DMASINC2_MSK             (0x1   << 2  )
#define ICER1_DMASINC2                 (0x1   << 2  )
#define ICER1_DMASINC2_DIS             (0x0   << 2  ) /* DIS                      */
#define ICER1_DMASINC2_EN              (0x1   << 2  ) /* EN                       */

/* ICER1[DMAADC1] -  */
#define ICER1_DMAADC1_MSK              (0x1   << 1  )
#define ICER1_DMAADC1                  (0x1   << 1  )
#define ICER1_DMAADC1_DIS              (0x0   << 1  ) /* DIS                      */
#define ICER1_DMAADC1_EN               (0x1   << 1  ) /* EN                       */

/* Reset Value for ISPR0*/
#define ISPR0_RVAL                     0x0 

/* ISPR0[DMADAC] -  */
#define ISPR0_DMADAC_MSK               (0x1   << 31 )
#define ISPR0_DMADAC                   (0x1   << 31 )
#define ISPR0_DMADAC_DIS               (0x0   << 31 ) /* DIS                      */
#define ISPR0_DMADAC_EN                (0x1   << 31 ) /* EN                       */

/* ISPR0[DMAI2CMRX] -  */
#define ISPR0_DMAI2CMRX_MSK            (0x1   << 30 )
#define ISPR0_DMAI2CMRX                (0x1   << 30 )
#define ISPR0_DMAI2CMRX_DIS            (0x0   << 30 ) /* DIS                      */
#define ISPR0_DMAI2CMRX_EN             (0x1   << 30 ) /* EN                       */

/* ISPR0[DMAI2CMTX] -  */
#define ISPR0_DMAI2CMTX_MSK            (0x1   << 29 )
#define ISPR0_DMAI2CMTX                (0x1   << 29 )
#define ISPR0_DMAI2CMTX_DIS            (0x0   << 29 ) /* DIS                      */
#define ISPR0_DMAI2CMTX_EN             (0x1   << 29 ) /* EN                       */

/* ISPR0[DMAI2CSRX] -  */
#define ISPR0_DMAI2CSRX_MSK            (0x1   << 28 )
#define ISPR0_DMAI2CSRX                (0x1   << 28 )
#define ISPR0_DMAI2CSRX_DIS            (0x0   << 28 ) /* DIS                      */
#define ISPR0_DMAI2CSRX_EN             (0x1   << 28 ) /* EN                       */

/* ISPR0[DMAI2CSTX] -  */
#define ISPR0_DMAI2CSTX_MSK            (0x1   << 27 )
#define ISPR0_DMAI2CSTX                (0x1   << 27 )
#define ISPR0_DMAI2CSTX_DIS            (0x0   << 27 ) /* DIS                      */
#define ISPR0_DMAI2CSTX_EN             (0x1   << 27 ) /* EN                       */

/* ISPR0[DMAUARTRX] -  */
#define ISPR0_DMAUARTRX_MSK            (0x1   << 26 )
#define ISPR0_DMAUARTRX                (0x1   << 26 )
#define ISPR0_DMAUARTRX_DIS            (0x0   << 26 ) /* DIS                      */
#define ISPR0_DMAUARTRX_EN             (0x1   << 26 ) /* EN                       */

/* ISPR0[DMAUARTTX] -  */
#define ISPR0_DMAUARTTX_MSK            (0x1   << 25 )
#define ISPR0_DMAUARTTX                (0x1   << 25 )
#define ISPR0_DMAUARTTX_DIS            (0x0   << 25 ) /* DIS                      */
#define ISPR0_DMAUARTTX_EN             (0x1   << 25 ) /* EN                       */

/* ISPR0[DMASPI1RX] -  */
#define ISPR0_DMASPI1RX_MSK            (0x1   << 24 )
#define ISPR0_DMASPI1RX                (0x1   << 24 )
#define ISPR0_DMASPI1RX_DIS            (0x0   << 24 ) /* DIS                      */
#define ISPR0_DMASPI1RX_EN             (0x1   << 24 ) /* EN                       */

/* ISPR0[DMASPI1TX] -  */
#define ISPR0_DMASPI1TX_MSK            (0x1   << 23 )
#define ISPR0_DMASPI1TX                (0x1   << 23 )
#define ISPR0_DMASPI1TX_DIS            (0x0   << 23 ) /* DIS                      */
#define ISPR0_DMASPI1TX_EN             (0x1   << 23 ) /* EN                       */

/* ISPR0[DMAERROR] -  */
#define ISPR0_DMAERROR_MSK             (0x1   << 22 )
#define ISPR0_DMAERROR                 (0x1   << 22 )
#define ISPR0_DMAERROR_DIS             (0x0   << 22 ) /* DIS                      */
#define ISPR0_DMAERROR_EN              (0x1   << 22 ) /* EN                       */

/* ISPR0[I2CM] -  */
#define ISPR0_I2CM_MSK                 (0x1   << 21 )
#define ISPR0_I2CM                     (0x1   << 21 )
#define ISPR0_I2CM_DIS                 (0x0   << 21 ) /* DIS                      */
#define ISPR0_I2CM_EN                  (0x1   << 21 ) /* EN                       */

/* ISPR0[I2CS] -  */
#define ISPR0_I2CS_MSK                 (0x1   << 20 )
#define ISPR0_I2CS                     (0x1   << 20 )
#define ISPR0_I2CS_DIS                 (0x0   << 20 ) /* DIS                      */
#define ISPR0_I2CS_EN                  (0x1   << 20 ) /* EN                       */

/* ISPR0[SPI1] -  */
#define ISPR0_SPI1_MSK                 (0x1   << 19 )
#define ISPR0_SPI1                     (0x1   << 19 )
#define ISPR0_SPI1_DIS                 (0x0   << 19 ) /* DIS                      */
#define ISPR0_SPI1_EN                  (0x1   << 19 ) /* EN                       */

/* ISPR0[SPI0] -  */
#define ISPR0_SPI0_MSK                 (0x1   << 18 )
#define ISPR0_SPI0                     (0x1   << 18 )
#define ISPR0_SPI0_DIS                 (0x0   << 18 ) /* DIS                      */
#define ISPR0_SPI0_EN                  (0x1   << 18 ) /* EN                       */

/* ISPR0[UART] -  */
#define ISPR0_UART_MSK                 (0x1   << 17 )
#define ISPR0_UART                     (0x1   << 17 )
#define ISPR0_UART_DIS                 (0x0   << 17 ) /* DIS                      */
#define ISPR0_UART_EN                  (0x1   << 17 ) /* EN                       */

/* ISPR0[FEE] -  */
#define ISPR0_FEE_MSK                  (0x1   << 16 )
#define ISPR0_FEE                      (0x1   << 16 )
#define ISPR0_FEE_DIS                  (0x0   << 16 ) /* DIS                      */
#define ISPR0_FEE_EN                   (0x1   << 16 ) /* EN                       */

/* ISPR0[SINC2] -  */
#define ISPR0_SINC2_MSK                (0x1   << 15 )
#define ISPR0_SINC2                    (0x1   << 15 )
#define ISPR0_SINC2_DIS                (0x0   << 15 ) /* DIS                      */
#define ISPR0_SINC2_EN                 (0x1   << 15 ) /* EN                       */

/* ISPR0[ADC1] -  */
#define ISPR0_ADC1_MSK                 (0x1   << 14 )
#define ISPR0_ADC1                     (0x1   << 14 )
#define ISPR0_ADC1_DIS                 (0x0   << 14 ) /* DIS                      */
#define ISPR0_ADC1_EN                  (0x1   << 14 ) /* EN                       */

/* ISPR0[T1] -  */
#define ISPR0_T1_MSK                   (0x1   << 12 )
#define ISPR0_T1                       (0x1   << 12 )
#define ISPR0_T1_DIS                   (0x0   << 12 ) /* DIS                      */
#define ISPR0_T1_EN                    (0x1   << 12 ) /* EN                       */

/* ISPR0[T0] -  */
#define ISPR0_T0_MSK                   (0x1   << 11 )
#define ISPR0_T0                       (0x1   << 11 )
#define ISPR0_T0_DIS                   (0x0   << 11 ) /* DIS                      */
#define ISPR0_T0_EN                    (0x1   << 11 ) /* EN                       */

/* ISPR0[T3] -  */
#define ISPR0_T3_MSK                   (0x1   << 9  )
#define ISPR0_T3                       (0x1   << 9  )
#define ISPR0_T3_DIS                   (0x0   << 9  ) /* DIS                      */
#define ISPR0_T3_EN                    (0x1   << 9  ) /* EN                       */

/* ISPR0[EXTINT7] -  */
#define ISPR0_EXTINT7_MSK              (0x1   << 8  )
#define ISPR0_EXTINT7                  (0x1   << 8  )
#define ISPR0_EXTINT7_DIS              (0x0   << 8  ) /* DIS                      */
#define ISPR0_EXTINT7_EN               (0x1   << 8  ) /* EN                       */

/* ISPR0[EXTINT6] -  */
#define ISPR0_EXTINT6_MSK              (0x1   << 7  )
#define ISPR0_EXTINT6                  (0x1   << 7  )
#define ISPR0_EXTINT6_DIS              (0x0   << 7  ) /* DIS                      */
#define ISPR0_EXTINT6_EN               (0x1   << 7  ) /* EN                       */

/* ISPR0[EXTINT5] -  */
#define ISPR0_EXTINT5_MSK              (0x1   << 6  )
#define ISPR0_EXTINT5                  (0x1   << 6  )
#define ISPR0_EXTINT5_DIS              (0x0   << 6  ) /* DIS                      */
#define ISPR0_EXTINT5_EN               (0x1   << 6  ) /* EN                       */

/* ISPR0[EXTINT4] -  */
#define ISPR0_EXTINT4_MSK              (0x1   << 5  )
#define ISPR0_EXTINT4                  (0x1   << 5  )
#define ISPR0_EXTINT4_DIS              (0x0   << 5  ) /* DIS                      */
#define ISPR0_EXTINT4_EN               (0x1   << 5  ) /* EN                       */

/* ISPR0[EXTINT3] -  */
#define ISPR0_EXTINT3_MSK              (0x1   << 4  )
#define ISPR0_EXTINT3                  (0x1   << 4  )
#define ISPR0_EXTINT3_DIS              (0x0   << 4  ) /* DIS                      */
#define ISPR0_EXTINT3_EN               (0x1   << 4  ) /* EN                       */

/* ISPR0[EXTINT2] -  */
#define ISPR0_EXTINT2_MSK              (0x1   << 3  )
#define ISPR0_EXTINT2                  (0x1   << 3  )
#define ISPR0_EXTINT2_DIS              (0x0   << 3  ) /* DIS                      */
#define ISPR0_EXTINT2_EN               (0x1   << 3  ) /* EN                       */

/* ISPR0[EXTINT1] -  */
#define ISPR0_EXTINT1_MSK              (0x1   << 2  )
#define ISPR0_EXTINT1                  (0x1   << 2  )
#define ISPR0_EXTINT1_DIS              (0x0   << 2  ) /* DIS                      */
#define ISPR0_EXTINT1_EN               (0x1   << 2  ) /* EN                       */

/* ISPR0[EXTINT0] -  */
#define ISPR0_EXTINT0_MSK              (0x1   << 1  )
#define ISPR0_EXTINT0                  (0x1   << 1  )
#define ISPR0_EXTINT0_DIS              (0x0   << 1  ) /* DIS                      */
#define ISPR0_EXTINT0_EN               (0x1   << 1  ) /* EN                       */

/* ISPR0[T2] -  */
#define ISPR0_T2_MSK                   (0x1   << 0  )
#define ISPR0_T2                       (0x1   << 0  )
#define ISPR0_T2_DIS                   (0x0   << 0  ) /* DIS                      */
#define ISPR0_T2_EN                    (0x1   << 0  ) /* EN                       */

/* Reset Value for ISPR1*/
#define ISPR1_RVAL                     0x0 

/* ISPR1[PWM2] -  */
#define ISPR1_PWM2_MSK                 (0x1   << 6  )
#define ISPR1_PWM2                     (0x1   << 6  )
#define ISPR1_PWM2_DIS                 (0x0   << 6  ) /* DIS                      */
#define ISPR1_PWM2_EN                  (0x1   << 6  ) /* EN                       */

/* ISPR1[PWM1] -  */
#define ISPR1_PWM1_MSK                 (0x1   << 5  )
#define ISPR1_PWM1                     (0x1   << 5  )
#define ISPR1_PWM1_DIS                 (0x0   << 5  ) /* DIS                      */
#define ISPR1_PWM1_EN                  (0x1   << 5  ) /* EN                       */

/* ISPR1[PWM0] -  */
#define ISPR1_PWM0_MSK                 (0x1   << 4  )
#define ISPR1_PWM0                     (0x1   << 4  )
#define ISPR1_PWM0_DIS                 (0x0   << 4  ) /* DIS                      */
#define ISPR1_PWM0_EN                  (0x1   << 4  ) /* EN                       */

/* ISPR1[PWMTRIP] -  */
#define ISPR1_PWMTRIP_MSK              (0x1   << 3  )
#define ISPR1_PWMTRIP                  (0x1   << 3  )
#define ISPR1_PWMTRIP_DIS              (0x0   << 3  ) /* DIS                      */
#define ISPR1_PWMTRIP_EN               (0x1   << 3  ) /* EN                       */

/* ISPR1[DMASINC2] -  */
#define ISPR1_DMASINC2_MSK             (0x1   << 2  )
#define ISPR1_DMASINC2                 (0x1   << 2  )
#define ISPR1_DMASINC2_DIS             (0x0   << 2  ) /* DIS                      */
#define ISPR1_DMASINC2_EN              (0x1   << 2  ) /* EN                       */

/* ISPR1[DMAADC1] -  */
#define ISPR1_DMAADC1_MSK              (0x1   << 1  )
#define ISPR1_DMAADC1                  (0x1   << 1  )
#define ISPR1_DMAADC1_DIS              (0x0   << 1  ) /* DIS                      */
#define ISPR1_DMAADC1_EN               (0x1   << 1  ) /* EN                       */

/* Reset Value for ICPR0*/
#define ICPR0_RVAL                     0x0 

/* ICPR0[DMADAC] -  */
#define ICPR0_DMADAC_MSK               (0x1   << 31 )
#define ICPR0_DMADAC                   (0x1   << 31 )
#define ICPR0_DMADAC_DIS               (0x0   << 31 ) /* DIS                      */
#define ICPR0_DMADAC_EN                (0x1   << 31 ) /* EN                       */

/* ICPR0[DMAI2CMRX] -  */
#define ICPR0_DMAI2CMRX_MSK            (0x1   << 30 )
#define ICPR0_DMAI2CMRX                (0x1   << 30 )
#define ICPR0_DMAI2CMRX_DIS            (0x0   << 30 ) /* DIS                      */
#define ICPR0_DMAI2CMRX_EN             (0x1   << 30 ) /* EN                       */

/* ICPR0[DMAI2CMTX] -  */
#define ICPR0_DMAI2CMTX_MSK            (0x1   << 29 )
#define ICPR0_DMAI2CMTX                (0x1   << 29 )
#define ICPR0_DMAI2CMTX_DIS            (0x0   << 29 ) /* DIS                      */
#define ICPR0_DMAI2CMTX_EN             (0x1   << 29 ) /* EN                       */

/* ICPR0[DMAI2CSRX] -  */
#define ICPR0_DMAI2CSRX_MSK            (0x1   << 28 )
#define ICPR0_DMAI2CSRX                (0x1   << 28 )
#define ICPR0_DMAI2CSRX_DIS            (0x0   << 28 ) /* DIS                      */
#define ICPR0_DMAI2CSRX_EN             (0x1   << 28 ) /* EN                       */

/* ICPR0[DMAI2CSTX] -  */
#define ICPR0_DMAI2CSTX_MSK            (0x1   << 27 )
#define ICPR0_DMAI2CSTX                (0x1   << 27 )
#define ICPR0_DMAI2CSTX_DIS            (0x0   << 27 ) /* DIS                      */
#define ICPR0_DMAI2CSTX_EN             (0x1   << 27 ) /* EN                       */

/* ICPR0[DMAUARTRX] -  */
#define ICPR0_DMAUARTRX_MSK            (0x1   << 26 )
#define ICPR0_DMAUARTRX                (0x1   << 26 )
#define ICPR0_DMAUARTRX_DIS            (0x0   << 26 ) /* DIS                      */
#define ICPR0_DMAUARTRX_EN             (0x1   << 26 ) /* EN                       */

/* ICPR0[DMAUARTTX] -  */
#define ICPR0_DMAUARTTX_MSK            (0x1   << 25 )
#define ICPR0_DMAUARTTX                (0x1   << 25 )
#define ICPR0_DMAUARTTX_DIS            (0x0   << 25 ) /* DIS                      */
#define ICPR0_DMAUARTTX_EN             (0x1   << 25 ) /* EN                       */

/* ICPR0[DMASPI1RX] -  */
#define ICPR0_DMASPI1RX_MSK            (0x1   << 24 )
#define ICPR0_DMASPI1RX                (0x1   << 24 )
#define ICPR0_DMASPI1RX_DIS            (0x0   << 24 ) /* DIS                      */
#define ICPR0_DMASPI1RX_EN             (0x1   << 24 ) /* EN                       */

/* ICPR0[DMASPI1TX] -  */
#define ICPR0_DMASPI1TX_MSK            (0x1   << 23 )
#define ICPR0_DMASPI1TX                (0x1   << 23 )
#define ICPR0_DMASPI1TX_DIS            (0x0   << 23 ) /* DIS                      */
#define ICPR0_DMASPI1TX_EN             (0x1   << 23 ) /* EN                       */

/* ICPR0[DMAERROR] -  */
#define ICPR0_DMAERROR_MSK             (0x1   << 22 )
#define ICPR0_DMAERROR                 (0x1   << 22 )
#define ICPR0_DMAERROR_DIS             (0x0   << 22 ) /* DIS                      */
#define ICPR0_DMAERROR_EN              (0x1   << 22 ) /* EN                       */

/* ICPR0[I2CM] -  */
#define ICPR0_I2CM_MSK                 (0x1   << 21 )
#define ICPR0_I2CM                     (0x1   << 21 )
#define ICPR0_I2CM_DIS                 (0x0   << 21 ) /* DIS                      */
#define ICPR0_I2CM_EN                  (0x1   << 21 ) /* EN                       */

/* ICPR0[I2CS] -  */
#define ICPR0_I2CS_MSK                 (0x1   << 20 )
#define ICPR0_I2CS                     (0x1   << 20 )
#define ICPR0_I2CS_DIS                 (0x0   << 20 ) /* DIS                      */
#define ICPR0_I2CS_EN                  (0x1   << 20 ) /* EN                       */

/* ICPR0[SPI1] -  */
#define ICPR0_SPI1_MSK                 (0x1   << 19 )
#define ICPR0_SPI1                     (0x1   << 19 )
#define ICPR0_SPI1_DIS                 (0x0   << 19 ) /* DIS                      */
#define ICPR0_SPI1_EN                  (0x1   << 19 ) /* EN                       */

/* ICPR0[SPI0] -  */
#define ICPR0_SPI0_MSK                 (0x1   << 18 )
#define ICPR0_SPI0                     (0x1   << 18 )
#define ICPR0_SPI0_DIS                 (0x0   << 18 ) /* DIS                      */
#define ICPR0_SPI0_EN                  (0x1   << 18 ) /* EN                       */

/* ICPR0[UART] -  */
#define ICPR0_UART_MSK                 (0x1   << 17 )
#define ICPR0_UART                     (0x1   << 17 )
#define ICPR0_UART_DIS                 (0x0   << 17 ) /* DIS                      */
#define ICPR0_UART_EN                  (0x1   << 17 ) /* EN                       */

/* ICPR0[FEE] -  */
#define ICPR0_FEE_MSK                  (0x1   << 16 )
#define ICPR0_FEE                      (0x1   << 16 )
#define ICPR0_FEE_DIS                  (0x0   << 16 ) /* DIS                      */
#define ICPR0_FEE_EN                   (0x1   << 16 ) /* EN                       */

/* ICPR0[SINC2] -  */
#define ICPR0_SINC2_MSK                (0x1   << 15 )
#define ICPR0_SINC2                    (0x1   << 15 )
#define ICPR0_SINC2_DIS                (0x0   << 15 ) /* DIS                      */
#define ICPR0_SINC2_EN                 (0x1   << 15 ) /* EN                       */

/* ICPR0[ADC1] -  */
#define ICPR0_ADC1_MSK                 (0x1   << 14 )
#define ICPR0_ADC1                     (0x1   << 14 )
#define ICPR0_ADC1_DIS                 (0x0   << 14 ) /* DIS                      */
#define ICPR0_ADC1_EN                  (0x1   << 14 ) /* EN                       */

/* ICPR0[T1] -  */
#define ICPR0_T1_MSK                   (0x1   << 12 )
#define ICPR0_T1                       (0x1   << 12 )
#define ICPR0_T1_DIS                   (0x0   << 12 ) /* DIS                      */
#define ICPR0_T1_EN                    (0x1   << 12 ) /* EN                       */

/* ICPR0[T0] -  */
#define ICPR0_T0_MSK                   (0x1   << 11 )
#define ICPR0_T0                       (0x1   << 11 )
#define ICPR0_T0_DIS                   (0x0   << 11 ) /* DIS                      */
#define ICPR0_T0_EN                    (0x1   << 11 ) /* EN                       */

/* ICPR0[T3] -  */
#define ICPR0_T3_MSK                   (0x1   << 9  )
#define ICPR0_T3                       (0x1   << 9  )
#define ICPR0_T3_DIS                   (0x0   << 9  ) /* DIS                      */
#define ICPR0_T3_EN                    (0x1   << 9  ) /* EN                       */

/* ICPR0[EXTINT7] -  */
#define ICPR0_EXTINT7_MSK              (0x1   << 8  )
#define ICPR0_EXTINT7                  (0x1   << 8  )
#define ICPR0_EXTINT7_DIS              (0x0   << 8  ) /* DIS                      */
#define ICPR0_EXTINT7_EN               (0x1   << 8  ) /* EN                       */

/* ICPR0[EXTINT6] -  */
#define ICPR0_EXTINT6_MSK              (0x1   << 7  )
#define ICPR0_EXTINT6                  (0x1   << 7  )
#define ICPR0_EXTINT6_DIS              (0x0   << 7  ) /* DIS                      */
#define ICPR0_EXTINT6_EN               (0x1   << 7  ) /* EN                       */

/* ICPR0[EXTINT5] -  */
#define ICPR0_EXTINT5_MSK              (0x1   << 6  )
#define ICPR0_EXTINT5                  (0x1   << 6  )
#define ICPR0_EXTINT5_DIS              (0x0   << 6  ) /* DIS                      */
#define ICPR0_EXTINT5_EN               (0x1   << 6  ) /* EN                       */

/* ICPR0[EXTINT4] -  */
#define ICPR0_EXTINT4_MSK              (0x1   << 5  )
#define ICPR0_EXTINT4                  (0x1   << 5  )
#define ICPR0_EXTINT4_DIS              (0x0   << 5  ) /* DIS                      */
#define ICPR0_EXTINT4_EN               (0x1   << 5  ) /* EN                       */

/* ICPR0[EXTINT3] -  */
#define ICPR0_EXTINT3_MSK              (0x1   << 4  )
#define ICPR0_EXTINT3                  (0x1   << 4  )
#define ICPR0_EXTINT3_DIS              (0x0   << 4  ) /* DIS                      */
#define ICPR0_EXTINT3_EN               (0x1   << 4  ) /* EN                       */

/* ICPR0[EXTINT2] -  */
#define ICPR0_EXTINT2_MSK              (0x1   << 3  )
#define ICPR0_EXTINT2                  (0x1   << 3  )
#define ICPR0_EXTINT2_DIS              (0x0   << 3  ) /* DIS                      */
#define ICPR0_EXTINT2_EN               (0x1   << 3  ) /* EN                       */

/* ICPR0[EXTINT1] -  */
#define ICPR0_EXTINT1_MSK              (0x1   << 2  )
#define ICPR0_EXTINT1                  (0x1   << 2  )
#define ICPR0_EXTINT1_DIS              (0x0   << 2  ) /* DIS                      */
#define ICPR0_EXTINT1_EN               (0x1   << 2  ) /* EN                       */

/* ICPR0[EXTINT0] -  */
#define ICPR0_EXTINT0_MSK              (0x1   << 1  )
#define ICPR0_EXTINT0                  (0x1   << 1  )
#define ICPR0_EXTINT0_DIS              (0x0   << 1  ) /* DIS                      */
#define ICPR0_EXTINT0_EN               (0x1   << 1  ) /* EN                       */

/* ICPR0[T2] -  */
#define ICPR0_T2_MSK                   (0x1   << 0  )
#define ICPR0_T2                       (0x1   << 0  )
#define ICPR0_T2_DIS                   (0x0   << 0  ) /* DIS                      */
#define ICPR0_T2_EN                    (0x1   << 0  ) /* EN                       */

/* Reset Value for ICPR1*/
#define ICPR1_RVAL                     0x0 

/* ICPR1[PWM2] -  */
#define ICPR1_PWM2_MSK                 (0x1   << 6  )
#define ICPR1_PWM2                     (0x1   << 6  )
#define ICPR1_PWM2_DIS                 (0x0   << 6  ) /* DIS                      */
#define ICPR1_PWM2_EN                  (0x1   << 6  ) /* EN                       */

/* ICPR1[PWM1] -  */
#define ICPR1_PWM1_MSK                 (0x1   << 5  )
#define ICPR1_PWM1                     (0x1   << 5  )
#define ICPR1_PWM1_DIS                 (0x0   << 5  ) /* DIS                      */
#define ICPR1_PWM1_EN                  (0x1   << 5  ) /* EN                       */

/* ICPR1[PWM0] -  */
#define ICPR1_PWM0_MSK                 (0x1   << 4  )
#define ICPR1_PWM0                     (0x1   << 4  )
#define ICPR1_PWM0_DIS                 (0x0   << 4  ) /* DIS                      */
#define ICPR1_PWM0_EN                  (0x1   << 4  ) /* EN                       */

/* ICPR1[PWMTRIP] -  */
#define ICPR1_PWMTRIP_MSK              (0x1   << 3  )
#define ICPR1_PWMTRIP                  (0x1   << 3  )
#define ICPR1_PWMTRIP_DIS              (0x0   << 3  ) /* DIS                      */
#define ICPR1_PWMTRIP_EN               (0x1   << 3  ) /* EN                       */

/* ICPR1[DMASINC2] -  */
#define ICPR1_DMASINC2_MSK             (0x1   << 2  )
#define ICPR1_DMASINC2                 (0x1   << 2  )
#define ICPR1_DMASINC2_DIS             (0x0   << 2  ) /* DIS                      */
#define ICPR1_DMASINC2_EN              (0x1   << 2  ) /* EN                       */

/* ICPR1[DMAADC1] -  */
#define ICPR1_DMAADC1_MSK              (0x1   << 1  )
#define ICPR1_DMAADC1                  (0x1   << 1  )
#define ICPR1_DMAADC1_DIS              (0x0   << 1  ) /* DIS                      */
#define ICPR1_DMAADC1_EN               (0x1   << 1  ) /* EN                       */

/* Reset Value for IABR0*/
#define IABR0_RVAL                     0x0 

/* IABR0[DMADAC] -  */
#define IABR0_DMADAC_MSK               (0x1   << 31 )
#define IABR0_DMADAC                   (0x1   << 31 )
#define IABR0_DMADAC_DIS               (0x0   << 31 ) /* DIS                      */
#define IABR0_DMADAC_EN                (0x1   << 31 ) /* EN                       */

/* IABR0[DMAI2CMRX] -  */
#define IABR0_DMAI2CMRX_MSK            (0x1   << 30 )
#define IABR0_DMAI2CMRX                (0x1   << 30 )
#define IABR0_DMAI2CMRX_DIS            (0x0   << 30 ) /* DIS                      */
#define IABR0_DMAI2CMRX_EN             (0x1   << 30 ) /* EN                       */

/* IABR0[DMAI2CMTX] -  */
#define IABR0_DMAI2CMTX_MSK            (0x1   << 29 )
#define IABR0_DMAI2CMTX                (0x1   << 29 )
#define IABR0_DMAI2CMTX_DIS            (0x0   << 29 ) /* DIS                      */
#define IABR0_DMAI2CMTX_EN             (0x1   << 29 ) /* EN                       */

/* IABR0[DMAI2CSRX] -  */
#define IABR0_DMAI2CSRX_MSK            (0x1   << 28 )
#define IABR0_DMAI2CSRX                (0x1   << 28 )
#define IABR0_DMAI2CSRX_DIS            (0x0   << 28 ) /* DIS                      */
#define IABR0_DMAI2CSRX_EN             (0x1   << 28 ) /* EN                       */

/* IABR0[DMAI2CSTX] -  */
#define IABR0_DMAI2CSTX_MSK            (0x1   << 27 )
#define IABR0_DMAI2CSTX                (0x1   << 27 )
#define IABR0_DMAI2CSTX_DIS            (0x0   << 27 ) /* DIS                      */
#define IABR0_DMAI2CSTX_EN             (0x1   << 27 ) /* EN                       */

/* IABR0[DMAUARTRX] -  */
#define IABR0_DMAUARTRX_MSK            (0x1   << 26 )
#define IABR0_DMAUARTRX                (0x1   << 26 )
#define IABR0_DMAUARTRX_DIS            (0x0   << 26 ) /* DIS                      */
#define IABR0_DMAUARTRX_EN             (0x1   << 26 ) /* EN                       */

/* IABR0[DMAUARTTX] -  */
#define IABR0_DMAUARTTX_MSK            (0x1   << 25 )
#define IABR0_DMAUARTTX                (0x1   << 25 )
#define IABR0_DMAUARTTX_DIS            (0x0   << 25 ) /* DIS                      */
#define IABR0_DMAUARTTX_EN             (0x1   << 25 ) /* EN                       */

/* IABR0[DMASPI1RX] -  */
#define IABR0_DMASPI1RX_MSK            (0x1   << 24 )
#define IABR0_DMASPI1RX                (0x1   << 24 )
#define IABR0_DMASPI1RX_DIS            (0x0   << 24 ) /* DIS                      */
#define IABR0_DMASPI1RX_EN             (0x1   << 24 ) /* EN                       */

/* IABR0[DMASPI1TX] -  */
#define IABR0_DMASPI1TX_MSK            (0x1   << 23 )
#define IABR0_DMASPI1TX                (0x1   << 23 )
#define IABR0_DMASPI1TX_DIS            (0x0   << 23 ) /* DIS                      */
#define IABR0_DMASPI1TX_EN             (0x1   << 23 ) /* EN                       */

/* IABR0[DMAERROR] -  */
#define IABR0_DMAERROR_MSK             (0x1   << 22 )
#define IABR0_DMAERROR                 (0x1   << 22 )
#define IABR0_DMAERROR_DIS             (0x0   << 22 ) /* DIS                      */
#define IABR0_DMAERROR_EN              (0x1   << 22 ) /* EN                       */

/* IABR0[I2CM] -  */
#define IABR0_I2CM_MSK                 (0x1   << 21 )
#define IABR0_I2CM                     (0x1   << 21 )
#define IABR0_I2CM_DIS                 (0x0   << 21 ) /* DIS                      */
#define IABR0_I2CM_EN                  (0x1   << 21 ) /* EN                       */

/* IABR0[I2CS] -  */
#define IABR0_I2CS_MSK                 (0x1   << 20 )
#define IABR0_I2CS                     (0x1   << 20 )
#define IABR0_I2CS_DIS                 (0x0   << 20 ) /* DIS                      */
#define IABR0_I2CS_EN                  (0x1   << 20 ) /* EN                       */

/* IABR0[SPI1] -  */
#define IABR0_SPI1_MSK                 (0x1   << 19 )
#define IABR0_SPI1                     (0x1   << 19 )
#define IABR0_SPI1_DIS                 (0x0   << 19 ) /* DIS                      */
#define IABR0_SPI1_EN                  (0x1   << 19 ) /* EN                       */

/* IABR0[SPI0] -  */
#define IABR0_SPI0_MSK                 (0x1   << 18 )
#define IABR0_SPI0                     (0x1   << 18 )
#define IABR0_SPI0_DIS                 (0x0   << 18 ) /* DIS                      */
#define IABR0_SPI0_EN                  (0x1   << 18 ) /* EN                       */

/* IABR0[UART] -  */
#define IABR0_UART_MSK                 (0x1   << 17 )
#define IABR0_UART                     (0x1   << 17 )
#define IABR0_UART_DIS                 (0x0   << 17 ) /* DIS                      */
#define IABR0_UART_EN                  (0x1   << 17 ) /* EN                       */

/* IABR0[FEE] -  */
#define IABR0_FEE_MSK                  (0x1   << 16 )
#define IABR0_FEE                      (0x1   << 16 )
#define IABR0_FEE_DIS                  (0x0   << 16 ) /* DIS                      */
#define IABR0_FEE_EN                   (0x1   << 16 ) /* EN                       */

/* IABR0[SINC2] -  */
#define IABR0_SINC2_MSK                (0x1   << 15 )
#define IABR0_SINC2                    (0x1   << 15 )
#define IABR0_SINC2_DIS                (0x0   << 15 ) /* DIS                      */
#define IABR0_SINC2_EN                 (0x1   << 15 ) /* EN                       */

/* IABR0[ADC1] -  */
#define IABR0_ADC1_MSK                 (0x1   << 14 )
#define IABR0_ADC1                     (0x1   << 14 )
#define IABR0_ADC1_DIS                 (0x0   << 14 ) /* DIS                      */
#define IABR0_ADC1_EN                  (0x1   << 14 ) /* EN                       */

/* IABR0[T1] -  */
#define IABR0_T1_MSK                   (0x1   << 12 )
#define IABR0_T1                       (0x1   << 12 )
#define IABR0_T1_DIS                   (0x0   << 12 ) /* DIS                      */
#define IABR0_T1_EN                    (0x1   << 12 ) /* EN                       */

/* IABR0[T0] -  */
#define IABR0_T0_MSK                   (0x1   << 11 )
#define IABR0_T0                       (0x1   << 11 )
#define IABR0_T0_DIS                   (0x0   << 11 ) /* DIS                      */
#define IABR0_T0_EN                    (0x1   << 11 ) /* EN                       */

/* IABR0[T3] -  */
#define IABR0_T3_MSK                   (0x1   << 9  )
#define IABR0_T3                       (0x1   << 9  )
#define IABR0_T3_DIS                   (0x0   << 9  ) /* DIS                      */
#define IABR0_T3_EN                    (0x1   << 9  ) /* EN                       */

/* IABR0[EXTINT7] -  */
#define IABR0_EXTINT7_MSK              (0x1   << 8  )
#define IABR0_EXTINT7                  (0x1   << 8  )
#define IABR0_EXTINT7_DIS              (0x0   << 8  ) /* DIS                      */
#define IABR0_EXTINT7_EN               (0x1   << 8  ) /* EN                       */

/* IABR0[EXTINT6] -  */
#define IABR0_EXTINT6_MSK              (0x1   << 7  )
#define IABR0_EXTINT6                  (0x1   << 7  )
#define IABR0_EXTINT6_DIS              (0x0   << 7  ) /* DIS                      */
#define IABR0_EXTINT6_EN               (0x1   << 7  ) /* EN                       */

/* IABR0[EXTINT5] -  */
#define IABR0_EXTINT5_MSK              (0x1   << 6  )
#define IABR0_EXTINT5                  (0x1   << 6  )
#define IABR0_EXTINT5_DIS              (0x0   << 6  ) /* DIS                      */
#define IABR0_EXTINT5_EN               (0x1   << 6  ) /* EN                       */

/* IABR0[EXTINT4] -  */
#define IABR0_EXTINT4_MSK              (0x1   << 5  )
#define IABR0_EXTINT4                  (0x1   << 5  )
#define IABR0_EXTINT4_DIS              (0x0   << 5  ) /* DIS                      */
#define IABR0_EXTINT4_EN               (0x1   << 5  ) /* EN                       */

/* IABR0[EXTINT3] -  */
#define IABR0_EXTINT3_MSK              (0x1   << 4  )
#define IABR0_EXTINT3                  (0x1   << 4  )
#define IABR0_EXTINT3_DIS              (0x0   << 4  ) /* DIS                      */
#define IABR0_EXTINT3_EN               (0x1   << 4  ) /* EN                       */

/* IABR0[EXTINT2] -  */
#define IABR0_EXTINT2_MSK              (0x1   << 3  )
#define IABR0_EXTINT2                  (0x1   << 3  )
#define IABR0_EXTINT2_DIS              (0x0   << 3  ) /* DIS                      */
#define IABR0_EXTINT2_EN               (0x1   << 3  ) /* EN                       */

/* IABR0[EXTINT1] -  */
#define IABR0_EXTINT1_MSK              (0x1   << 2  )
#define IABR0_EXTINT1                  (0x1   << 2  )
#define IABR0_EXTINT1_DIS              (0x0   << 2  ) /* DIS                      */
#define IABR0_EXTINT1_EN               (0x1   << 2  ) /* EN                       */

/* IABR0[EXTINT0] -  */
#define IABR0_EXTINT0_MSK              (0x1   << 1  )
#define IABR0_EXTINT0                  (0x1   << 1  )
#define IABR0_EXTINT0_DIS              (0x0   << 1  ) /* DIS                      */
#define IABR0_EXTINT0_EN               (0x1   << 1  ) /* EN                       */

/* IABR0[T2] -  */
#define IABR0_T2_MSK                   (0x1   << 0  )
#define IABR0_T2                       (0x1   << 0  )
#define IABR0_T2_DIS                   (0x0   << 0  ) /* DIS                      */
#define IABR0_T2_EN                    (0x1   << 0  ) /* EN                       */

/* Reset Value for IABR1*/
#define IABR1_RVAL                     0x0 

/* IABR1[PWM2] -  */
#define IABR1_PWM2_MSK                 (0x1   << 6  )
#define IABR1_PWM2                     (0x1   << 6  )
#define IABR1_PWM2_DIS                 (0x0   << 6  ) /* DIS                      */
#define IABR1_PWM2_EN                  (0x1   << 6  ) /* EN                       */

/* IABR1[PWM1] -  */
#define IABR1_PWM1_MSK                 (0x1   << 5  )
#define IABR1_PWM1                     (0x1   << 5  )
#define IABR1_PWM1_DIS                 (0x0   << 5  ) /* DIS                      */
#define IABR1_PWM1_EN                  (0x1   << 5  ) /* EN                       */

/* IABR1[PWM0] -  */
#define IABR1_PWM0_MSK                 (0x1   << 4  )
#define IABR1_PWM0                     (0x1   << 4  )
#define IABR1_PWM0_DIS                 (0x0   << 4  ) /* DIS                      */
#define IABR1_PWM0_EN                  (0x1   << 4  ) /* EN                       */

/* IABR1[PWMTRIP] -  */
#define IABR1_PWMTRIP_MSK              (0x1   << 3  )
#define IABR1_PWMTRIP                  (0x1   << 3  )
#define IABR1_PWMTRIP_DIS              (0x0   << 3  ) /* DIS                      */
#define IABR1_PWMTRIP_EN               (0x1   << 3  ) /* EN                       */

/* IABR1[DMASINC2] -  */
#define IABR1_DMASINC2_MSK             (0x1   << 2  )
#define IABR1_DMASINC2                 (0x1   << 2  )
#define IABR1_DMASINC2_DIS             (0x0   << 2  ) /* DIS                      */
#define IABR1_DMASINC2_EN              (0x1   << 2  ) /* EN                       */

/* IABR1[DMAADC1] -  */
#define IABR1_DMAADC1_MSK              (0x1   << 1  )
#define IABR1_DMAADC1                  (0x1   << 1  )
#define IABR1_DMAADC1_DIS              (0x0   << 1  ) /* DIS                      */
#define IABR1_DMAADC1_EN               (0x1   << 1  ) /* EN                       */

/* Reset Value for IPR0*/
#define IPR0_RVAL                      0x0 

/* IPR0[EXTINT2] -  */
#define IPR0_EXTINT2_MSK               (0xFF  << 24 )

/* IPR0[EXTINT1] -  */
#define IPR0_EXTINT1_MSK               (0xFF  << 16 )

/* IPR0[EXTINT0] - Priority of interrupt number 1 */
#define IPR0_EXTINT0_MSK               (0xFF  << 8  )

/* IPR0[T2] - Priority of interrupt number 0 */
#define IPR0_T2_MSK                    (0xFF  << 0  )

/* Reset Value for IPR1*/
#define IPR1_RVAL                      0x0 

/* IPR1[EXTINT6] -  */
#define IPR1_EXTINT6_MSK               (0xFF  << 24 )

/* IPR1[EXTINT5] -  */
#define IPR1_EXTINT5_MSK               (0xFF  << 16 )

/* IPR1[EXTINT4] -  */
#define IPR1_EXTINT4_MSK               (0xFF  << 8  )

/* IPR1[EXTINT3] -  */
#define IPR1_EXTINT3_MSK               (0xFF  << 0  )

/* Reset Value for IPR2*/
#define IPR2_RVAL                      0x0 

/* IPR2[T0] -  */
#define IPR2_T0_MSK                    (0xFF  << 24 )

/* IPR2[T3] -  */
#define IPR2_T3_MSK                    (0xFF  << 8  )

/* IPR2[EXTINT7] -  */
#define IPR2_EXTINT7_MSK               (0xFF  << 0  )

/* Reset Value for IPR3*/
#define IPR3_RVAL                      0x0 

/* IPR3[SINC2] -  */
#define IPR3_SINC2_MSK                 (0xFF  << 24 )

/* IPR3[ADC1] -  */
#define IPR3_ADC1_MSK                  (0xFF  << 16 )

/* IPR3[T1] -  */
#define IPR3_T1_MSK                    (0xFF  << 0  )

/* Reset Value for IPR4*/
#define IPR4_RVAL                      0x0 

/* IPR4[SPI1] -  */
#define IPR4_SPI1_MSK                  (0xFF  << 24 )

/* IPR4[SPI0] -  */
#define IPR4_SPI0_MSK                  (0xFF  << 16 )

/* IPR4[UART] -  */
#define IPR4_UART_MSK                  (0xFF  << 8  )

/* IPR4[FEE] -  */
#define IPR4_FEE_MSK                   (0xFF  << 0  )

/* Reset Value for IPR5*/
#define IPR5_RVAL                      0x0 

/* IPR5[DMASPI1TX] -  */
#define IPR5_DMASPI1TX_MSK             (0xFF  << 24 )

/* IPR5[DMAERROR] -  */
#define IPR5_DMAERROR_MSK              (0xFF  << 16 )

/* IPR5[I2CM] -  */
#define IPR5_I2CM_MSK                  (0xFF  << 8  )

/* IPR5[I2CS] -  */
#define IPR5_I2CS_MSK                  (0xFF  << 0  )

/* Reset Value for IPR6*/
#define IPR6_RVAL                      0x0 

/* IPR6[DMAI2CSTX] -  */
#define IPR6_DMAI2CSTX_MSK             (0xFF  << 24 )

/* IPR6[DMAUARTRX] -  */
#define IPR6_DMAUARTRX_MSK             (0xFF  << 16 )

/* IPR6[DMAUARTTX] -  */
#define IPR6_DMAUARTTX_MSK             (0xFF  << 8  )

/* IPR6[DMASPI1RX] -  */
#define IPR6_DMASPI1RX_MSK             (0xFF  << 0  )

/* Reset Value for IPR7*/
#define IPR7_RVAL                      0x0 

/* IPR7[DMADAC] -  */
#define IPR7_DMADAC_MSK                (0xFF  << 24 )

/* IPR7[DMAI2CMRX] -  */
#define IPR7_DMAI2CMRX_MSK             (0xFF  << 16 )

/* IPR7[DMAI2CMTX] -  */
#define IPR7_DMAI2CMTX_MSK             (0xFF  << 8  )

/* IPR7[DMAI2CSRX] -  */
#define IPR7_DMAI2CSRX_MSK             (0xFF  << 0  )

/* Reset Value for IPR8*/
#define IPR8_RVAL                      0x0 

/* IPR8[PWMTRIP] -  */
#define IPR8_PWMTRIP_MSK               (0xFF  << 24 )

/* IPR8[DMASINC2] -  */
#define IPR8_DMASINC2_MSK              (0xFF  << 16 )

/* IPR8[DMAADC1] -  */
#define IPR8_DMAADC1_MSK               (0xFF  << 8  )

/* Reset Value for IPR9*/
#define IPR9_RVAL                      0x0 

/* IPR9[PWM2] -  */
#define IPR9_PWM2_MSK                  (0xFF  << 16 )

/* IPR9[PWM1] -  */
#define IPR9_PWM1_MSK                  (0xFF  << 8  )

/* IPR9[PWM0] -  */
#define IPR9_PWM0_MSK                  (0xFF  << 0  )

/* Reset Value for CPUID*/
#define CPUID_RVAL                     0x412FC230 

/* CPUID[IMPLEMENTER] - Indicates implementor */
#define CPUID_IMPLEMENTER_MSK          (0xFF  << 24 )

/* CPUID[VARIANT] - Indicates processor revision */
#define CPUID_VARIANT_MSK              (0xF   << 20 )

/* CPUID[PARTNO] - Indicates part number */
#define CPUID_PARTNO_MSK               (0xFFF << 4  )

/* CPUID[REVISION] - Indicates patch release */
#define CPUID_REVISION_MSK             (0xF   << 0  )

/* Reset Value for ICSR*/
#define ICSR_RVAL                      0x0 

/* ICSR[NMIPENDSET] - Setting this bit will activate an NMI */
#define ICSR_NMIPENDSET_MSK            (0x1   << 31 )
#define ICSR_NMIPENDSET                (0x1   << 31 )
#define ICSR_NMIPENDSET_DIS            (0x0   << 31 ) /* DIS                      */
#define ICSR_NMIPENDSET_EN             (0x1   << 31 ) /* EN                       */

/* ICSR[PENDSVSET] - Set a pending PendSV interrupt */
#define ICSR_PENDSVSET_MSK             (0x1   << 28 )
#define ICSR_PENDSVSET                 (0x1   << 28 )
#define ICSR_PENDSVSET_DIS             (0x0   << 28 ) /* DIS                      */
#define ICSR_PENDSVSET_EN              (0x1   << 28 ) /* EN                       */

/* ICSR[PENDSVCLR] - Clear a pending PendSV interrupt */
#define ICSR_PENDSVCLR_MSK             (0x1   << 27 )
#define ICSR_PENDSVCLR                 (0x1   << 27 )
#define ICSR_PENDSVCLR_DIS             (0x0   << 27 ) /* DIS                      */
#define ICSR_PENDSVCLR_EN              (0x1   << 27 ) /* EN                       */

/* ICSR[PENDSTSET] - Set a pending SysTick. Reads back with current state */
#define ICSR_PENDSTSET_MSK             (0x1   << 26 )
#define ICSR_PENDSTSET                 (0x1   << 26 )
#define ICSR_PENDSTSET_DIS             (0x0   << 26 ) /* DIS                      */
#define ICSR_PENDSTSET_EN              (0x1   << 26 ) /* EN                       */

/* ICSR[PENDSTCLR] - Clear a pending SysTick */
#define ICSR_PENDSTCLR_MSK             (0x1   << 25 )
#define ICSR_PENDSTCLR                 (0x1   << 25 )
#define ICSR_PENDSTCLR_DIS             (0x0   << 25 ) /* DIS                      */
#define ICSR_PENDSTCLR_EN              (0x1   << 25 ) /* EN                       */

/* ICSR[ISRPREEMPT] - If set, a pending exception will be serviced on exit from the debug halt state */
#define ICSR_ISRPREEMPT_MSK            (0x1   << 23 )
#define ICSR_ISRPREEMPT                (0x1   << 23 )
#define ICSR_ISRPREEMPT_DIS            (0x0   << 23 ) /* DIS                      */
#define ICSR_ISRPREEMPT_EN             (0x1   << 23 ) /* EN                       */

/* ICSR[ISRPENDING] - Indicates if an external configurable is pending */
#define ICSR_ISRPENDING_MSK            (0x1   << 22 )
#define ICSR_ISRPENDING                (0x1   << 22 )
#define ICSR_ISRPENDING_DIS            (0x0   << 22 ) /* DIS                      */
#define ICSR_ISRPENDING_EN             (0x1   << 22 ) /* EN                       */

/* ICSR[VECTPENDING] - Indicates the exception number for the highest priority pending exception */
#define ICSR_VECTPENDING_MSK           (0x1FF << 12 )

/* ICSR[RETTOBASE] -  */
#define ICSR_RETTOBASE_MSK             (0x1   << 11 )
#define ICSR_RETTOBASE                 (0x1   << 11 )
#define ICSR_RETTOBASE_DIS             (0x0   << 11 ) /* DIS                      */
#define ICSR_RETTOBASE_EN              (0x1   << 11 ) /* EN                       */

/* ICSR[VECTACTIVE] - Thread mode, or exception number */
#define ICSR_VECTACTIVE_MSK            (0x1FF << 0  )

/* Reset Value for VTOR*/
#define VTOR_RVAL                      0x0 

/* VTOR[TBLBASE] -  */
#define VTOR_TBLBASE_MSK               (0x1   << 29 )
#define VTOR_TBLBASE                   (0x1   << 29 )
#define VTOR_TBLBASE_DIS               (0x0   << 29 ) /* DIS                      */
#define VTOR_TBLBASE_EN                (0x1   << 29 ) /* EN                       */

/* VTOR[TBLOFF] -  */
#define VTOR_TBLOFF_MSK                (0x3FFFFF << 7  )

/* Reset Value for AIRCR*/
#define AIRCR_RVAL                     0xFA050000 

/* AIRCR[VECTKEYSTAT] - Reads as 0xFA05 */
#define AIRCR_VECTKEYSTAT_MSK          (0xFFFF << 16 )

/* AIRCR[ENDIANESS] - This bit is static or configured by a hardware input on reset */
#define AIRCR_ENDIANESS_MSK            (0x1   << 15 )
#define AIRCR_ENDIANESS                (0x1   << 15 )
#define AIRCR_ENDIANESS_DIS            (0x0   << 15 ) /* DIS                      */
#define AIRCR_ENDIANESS_EN             (0x1   << 15 ) /* EN                       */

/* AIRCR[PRIGROUP] - Priority grouping position */
#define AIRCR_PRIGROUP_MSK             (0x7   << 8  )

/* AIRCR[SYSRESETREQ] - System Reset Request */
#define AIRCR_SYSRESETREQ_MSK          (0x1   << 2  )
#define AIRCR_SYSRESETREQ              (0x1   << 2  )
#define AIRCR_SYSRESETREQ_DIS          (0x0   << 2  ) /* DIS                      */
#define AIRCR_SYSRESETREQ_EN           (0x1   << 2  ) /* EN                       */

/* AIRCR[VECTCLRACTIVE] - Clears all active state information for fixed and configurable exceptions */
#define AIRCR_VECTCLRACTIVE_MSK        (0x1   << 1  )
#define AIRCR_VECTCLRACTIVE            (0x1   << 1  )
#define AIRCR_VECTCLRACTIVE_DIS        (0x0   << 1  ) /* DIS                      */
#define AIRCR_VECTCLRACTIVE_EN         (0x1   << 1  ) /* EN                       */

/* AIRCR[VECTRESET] - Local system reset */
#define AIRCR_VECTRESET_MSK            (0x1   << 0  )
#define AIRCR_VECTRESET                (0x1   << 0  )
#define AIRCR_VECTRESET_DIS            (0x0   << 0  ) /* DIS                      */
#define AIRCR_VECTRESET_EN             (0x1   << 0  ) /* EN                       */

/* Reset Value for SCR*/
#define SCR_RVAL                       0x0 

/* SCR[SEVONPEND] -  */
#define SCR_SEVONPEND_MSK              (0x1   << 4  )
#define SCR_SEVONPEND                  (0x1   << 4  )
#define SCR_SEVONPEND_DIS              (0x0   << 4  ) /* DIS                      */
#define SCR_SEVONPEND_EN               (0x1   << 4  ) /* EN                       */

/* SCR[SLEEPDEEP] - Sleep deep bit */
#define SCR_SLEEPDEEP_MSK              (0x1   << 2  )
#define SCR_SLEEPDEEP                  (0x1   << 2  )
#define SCR_SLEEPDEEP_DIS              (0x0   << 2  ) /* DIS                      */
#define SCR_SLEEPDEEP_EN               (0x1   << 2  ) /* EN                       */

/* SCR[SLEEPONEXIT] - Sleep on exit when returning from handler mode to thread mode */
#define SCR_SLEEPONEXIT_MSK            (0x1   << 1  )
#define SCR_SLEEPONEXIT                (0x1   << 1  )
#define SCR_SLEEPONEXIT_DIS            (0x0   << 1  ) /* DIS                      */
#define SCR_SLEEPONEXIT_EN             (0x1   << 1  ) /* EN                       */

/* Reset Value for CCR*/
#define CCR_RVAL                       0x200 

/* CCR[STKALIGN] -  */
#define CCR_STKALIGN_MSK               (0x1   << 9  )
#define CCR_STKALIGN                   (0x1   << 9  )
#define CCR_STKALIGN_DIS               (0x0   << 9  ) /* DIS                      */
#define CCR_STKALIGN_EN                (0x1   << 9  ) /* EN                       */

/* CCR[BFHFNMIGN] -  */
#define CCR_BFHFNMIGN_MSK              (0x1   << 8  )
#define CCR_BFHFNMIGN                  (0x1   << 8  )
#define CCR_BFHFNMIGN_DIS              (0x0   << 8  ) /* DIS                      */
#define CCR_BFHFNMIGN_EN               (0x1   << 8  ) /* EN                       */

/* CCR[DIV0TRP] -  */
#define CCR_DIV0TRP_MSK                (0x1   << 4  )
#define CCR_DIV0TRP                    (0x1   << 4  )
#define CCR_DIV0TRP_DIS                (0x0   << 4  ) /* DIS                      */
#define CCR_DIV0TRP_EN                 (0x1   << 4  ) /* EN                       */

/* CCR[UNALIGNTRP] -  */
#define CCR_UNALIGNTRP_MSK             (0x1   << 3  )
#define CCR_UNALIGNTRP                 (0x1   << 3  )
#define CCR_UNALIGNTRP_DIS             (0x0   << 3  ) /* DIS                      */
#define CCR_UNALIGNTRP_EN              (0x1   << 3  ) /* EN                       */

/* CCR[USERSETMPEND] -  */
#define CCR_USERSETMPEND_MSK           (0x1   << 1  )
#define CCR_USERSETMPEND               (0x1   << 1  )
#define CCR_USERSETMPEND_DIS           (0x0   << 1  ) /* DIS                      */
#define CCR_USERSETMPEND_EN            (0x1   << 1  ) /* EN                       */

/* CCR[NONBASETHRDENA] -  */
#define CCR_NONBASETHRDENA_MSK         (0x1   << 0  )
#define CCR_NONBASETHRDENA             (0x1   << 0  )
#define CCR_NONBASETHRDENA_DIS         (0x0   << 0  ) /* DIS                      */
#define CCR_NONBASETHRDENA_EN          (0x1   << 0  ) /* EN                       */

/* Reset Value for SHPR1*/
#define SHPR1_RVAL                     0x0 

/* SHPR1[PRI7] - Priority of system handler 7 - reserved */
#define SHPR1_PRI7_MSK                 (0xFF  << 24 )

/* SHPR1[PRI6] - Priority of system handler 6 - UsageFault */
#define SHPR1_PRI6_MSK                 (0xFF  << 16 )

/* SHPR1[PRI5] - Priority of system handler 5 - BusFault */
#define SHPR1_PRI5_MSK                 (0xFF  << 8  )

/* SHPR1[PRI4] - Priority of system handler 4 - MemManage */
#define SHPR1_PRI4_MSK                 (0xFF  << 0  )

/* Reset Value for SHPR2*/
#define SHPR2_RVAL                     0x0 

/* SHPR2[PRI11] - Priority of system handler 11 - SVCall */
#define SHPR2_PRI11_MSK                (0xFF  << 24 )

/* SHPR2[PRI10] - Priority of system handler 10 - reserved */
#define SHPR2_PRI10_MSK                (0xFF  << 16 )

/* SHPR2[PRI9] - Priority of system handler 9 - reserved */
#define SHPR2_PRI9_MSK                 (0xFF  << 8  )

/* SHPR2[PRI8] - Priority of system handler 8 - reserved */
#define SHPR2_PRI8_MSK                 (0xFF  << 0  )

/* Reset Value for SHPR3*/
#define SHPR3_RVAL                     0x0 

/* SHPR3[PRI15] - Priority of system handler 15 - SysTick */
#define SHPR3_PRI15_MSK                (0xFF  << 24 )

/* SHPR3[PRI14] - Priority of system handler 14 - PendSV */
#define SHPR3_PRI14_MSK                (0xFF  << 16 )

/* SHPR3[PRI13] - Priority of system handler 13 - reserved */
#define SHPR3_PRI13_MSK                (0xFF  << 8  )

/* SHPR3[PRI12] - Priority of system handler 12 - DebugMonitor */
#define SHPR3_PRI12_MSK                (0xFF  << 0  )

/* Reset Value for SHCSR*/
#define SHCSR_RVAL                     0x0 

/* SHCSR[USGFAULTENA] - Enable for UsageFault */
#define SHCSR_USGFAULTENA_MSK          (0x1   << 18 )
#define SHCSR_USGFAULTENA              (0x1   << 18 )
#define SHCSR_USGFAULTENA_DIS          (0x0   << 18 ) /* DIS                      */
#define SHCSR_USGFAULTENA_EN           (0x1   << 18 ) /* EN                       */

/* SHCSR[BUSFAULTENA] - Enable for BusFault. */
#define SHCSR_BUSFAULTENA_MSK          (0x1   << 17 )
#define SHCSR_BUSFAULTENA              (0x1   << 17 )
#define SHCSR_BUSFAULTENA_DIS          (0x0   << 17 ) /* DIS                      */
#define SHCSR_BUSFAULTENA_EN           (0x1   << 17 ) /* EN                       */

/* SHCSR[MEMFAULTENA] - Enable for MemManage fault. */
#define SHCSR_MEMFAULTENA_MSK          (0x1   << 16 )
#define SHCSR_MEMFAULTENA              (0x1   << 16 )
#define SHCSR_MEMFAULTENA_DIS          (0x0   << 16 ) /* DIS                      */
#define SHCSR_MEMFAULTENA_EN           (0x1   << 16 ) /* EN                       */

/* SHCSR[SVCALLPENDED] - Reads as 1 if SVCall is Pending */
#define SHCSR_SVCALLPENDED_MSK         (0x1   << 15 )
#define SHCSR_SVCALLPENDED             (0x1   << 15 )
#define SHCSR_SVCALLPENDED_DIS         (0x0   << 15 ) /* DIS                      */
#define SHCSR_SVCALLPENDED_EN          (0x1   << 15 ) /* EN                       */

/* SHCSR[BUSFAULTPENDED] - Reads as 1 if BusFault is Pending */
#define SHCSR_BUSFAULTPENDED_MSK       (0x1   << 14 )
#define SHCSR_BUSFAULTPENDED           (0x1   << 14 )
#define SHCSR_BUSFAULTPENDED_DIS       (0x0   << 14 ) /* DIS                      */
#define SHCSR_BUSFAULTPENDED_EN        (0x1   << 14 ) /* EN                       */

/* SHCSR[MEMFAULTPENDED] - Reads as 1 if MemManage is Pending */
#define SHCSR_MEMFAULTPENDED_MSK       (0x1   << 13 )
#define SHCSR_MEMFAULTPENDED           (0x1   << 13 )
#define SHCSR_MEMFAULTPENDED_DIS       (0x0   << 13 ) /* DIS                      */
#define SHCSR_MEMFAULTPENDED_EN        (0x1   << 13 ) /* EN                       */

/* SHCSR[USGFAULTPENDED] - Reads as 1 if UsageFault is Pending */
#define SHCSR_USGFAULTPENDED_MSK       (0x1   << 12 )
#define SHCSR_USGFAULTPENDED           (0x1   << 12 )
#define SHCSR_USGFAULTPENDED_DIS       (0x0   << 12 ) /* DIS                      */
#define SHCSR_USGFAULTPENDED_EN        (0x1   << 12 ) /* EN                       */

/* SHCSR[SYSTICKACT] - Reads as 1 if SysTick is Active */
#define SHCSR_SYSTICKACT_MSK           (0x1   << 11 )
#define SHCSR_SYSTICKACT               (0x1   << 11 )
#define SHCSR_SYSTICKACT_DIS           (0x0   << 11 ) /* DIS                      */
#define SHCSR_SYSTICKACT_EN            (0x1   << 11 ) /* EN                       */

/* SHCSR[PENDSVACT] - Reads as 1 if PendSV is Active */
#define SHCSR_PENDSVACT_MSK            (0x1   << 10 )
#define SHCSR_PENDSVACT                (0x1   << 10 )
#define SHCSR_PENDSVACT_DIS            (0x0   << 10 ) /* DIS                      */
#define SHCSR_PENDSVACT_EN             (0x1   << 10 ) /* EN                       */

/* SHCSR[MONITORACT] - Reads as 1 if the Monitor is Active */
#define SHCSR_MONITORACT_MSK           (0x1   << 8  )
#define SHCSR_MONITORACT               (0x1   << 8  )
#define SHCSR_MONITORACT_DIS           (0x0   << 8  ) /* DIS                      */
#define SHCSR_MONITORACT_EN            (0x1   << 8  ) /* EN                       */

/* SHCSR[SVCALLACT] - Reads as 1 if SVCall is Active */
#define SHCSR_SVCALLACT_MSK            (0x1   << 7  )
#define SHCSR_SVCALLACT                (0x1   << 7  )
#define SHCSR_SVCALLACT_DIS            (0x0   << 7  ) /* DIS                      */
#define SHCSR_SVCALLACT_EN             (0x1   << 7  ) /* EN                       */

/* SHCSR[USGFAULTACT] - Reads as 1 if UsageFault is Active. */
#define SHCSR_USGFAULTACT_MSK          (0x1   << 3  )
#define SHCSR_USGFAULTACT              (0x1   << 3  )
#define SHCSR_USGFAULTACT_DIS          (0x0   << 3  ) /* DIS                      */
#define SHCSR_USGFAULTACT_EN           (0x1   << 3  ) /* EN                       */

/* SHCSR[BUSFAULTACT] - Reads as 1 if BusFault is Active. */
#define SHCSR_BUSFAULTACT_MSK          (0x1   << 1  )
#define SHCSR_BUSFAULTACT              (0x1   << 1  )
#define SHCSR_BUSFAULTACT_DIS          (0x0   << 1  ) /* DIS                      */
#define SHCSR_BUSFAULTACT_EN           (0x1   << 1  ) /* EN                       */

/* SHCSR[MEMFAULTACT] - Reads as 1 if MemManage is Active */
#define SHCSR_MEMFAULTACT_MSK          (0x1   << 0  )
#define SHCSR_MEMFAULTACT              (0x1   << 0  )
#define SHCSR_MEMFAULTACT_DIS          (0x0   << 0  ) /* DIS                      */
#define SHCSR_MEMFAULTACT_EN           (0x1   << 0  ) /* EN                       */

/* Reset Value for CFSR*/
#define CFSR_RVAL                      0x0 

/* CFSR[DIVBYZERO] - Divide by zero error */
#define CFSR_DIVBYZERO_MSK             (0x1   << 25 )
#define CFSR_DIVBYZERO                 (0x1   << 25 )
#define CFSR_DIVBYZERO_DIS             (0x0   << 25 ) /* DIS                      */
#define CFSR_DIVBYZERO_EN              (0x1   << 25 ) /* EN                       */

/* CFSR[UNALIGNED] - Unaligned access error */
#define CFSR_UNALIGNED_MSK             (0x1   << 24 )
#define CFSR_UNALIGNED                 (0x1   << 24 )
#define CFSR_UNALIGNED_DIS             (0x0   << 24 ) /* DIS                      */
#define CFSR_UNALIGNED_EN              (0x1   << 24 ) /* EN                       */

/* CFSR[NOCP] - Coprocessor access error */
#define CFSR_NOCP_MSK                  (0x1   << 19 )
#define CFSR_NOCP                      (0x1   << 19 )
#define CFSR_NOCP_DIS                  (0x0   << 19 ) /* DIS                      */
#define CFSR_NOCP_EN                   (0x1   << 19 ) /* EN                       */

/* CFSR[INVPC] - Integrity check error on EXC_RETURN */
#define CFSR_INVPC_MSK                 (0x1   << 18 )
#define CFSR_INVPC                     (0x1   << 18 )
#define CFSR_INVPC_DIS                 (0x0   << 18 ) /* DIS                      */
#define CFSR_INVPC_EN                  (0x1   << 18 ) /* EN                       */

/* CFSR[INVSTATE] - Invalid EPSR.T bit or illegal EPSR.IT bits for executing */
#define CFSR_INVSTATE_MSK              (0x1   << 17 )
#define CFSR_INVSTATE                  (0x1   << 17 )
#define CFSR_INVSTATE_DIS              (0x0   << 17 ) /* DIS                      */
#define CFSR_INVSTATE_EN               (0x1   << 17 ) /* EN                       */

/* CFSR[UNDEFINSTR] - Undefined instruction executed */
#define CFSR_UNDEFINSTR_MSK            (0x1   << 16 )
#define CFSR_UNDEFINSTR                (0x1   << 16 )
#define CFSR_UNDEFINSTR_DIS            (0x0   << 16 ) /* DIS                      */
#define CFSR_UNDEFINSTR_EN             (0x1   << 16 ) /* EN                       */

/* CFSR[BFARVALID] - This bit is set if the BFAR register has valid contents */
#define CFSR_BFARVALID_MSK             (0x1   << 15 )
#define CFSR_BFARVALID                 (0x1   << 15 )
#define CFSR_BFARVALID_DIS             (0x0   << 15 ) /* DIS                      */
#define CFSR_BFARVALID_EN              (0x1   << 15 ) /* EN                       */

/* CFSR[STKERR] - This bit indicates a derived bus fault has occurred on exception entry */
#define CFSR_STKERR_MSK                (0x1   << 12 )
#define CFSR_STKERR                    (0x1   << 12 )
#define CFSR_STKERR_DIS                (0x0   << 12 ) /* DIS                      */
#define CFSR_STKERR_EN                 (0x1   << 12 ) /* EN                       */

/* CFSR[UNSTKERR] - This bit indicates a derived bus fault has occurred on exception return */
#define CFSR_UNSTKERR_MSK              (0x1   << 11 )
#define CFSR_UNSTKERR                  (0x1   << 11 )
#define CFSR_UNSTKERR_DIS              (0x0   << 11 ) /* DIS                      */
#define CFSR_UNSTKERR_EN               (0x1   << 11 ) /* EN                       */

/* CFSR[IMPRECISERR] - Imprecise data access error */
#define CFSR_IMPRECISERR_MSK           (0x1   << 10 )
#define CFSR_IMPRECISERR               (0x1   << 10 )
#define CFSR_IMPRECISERR_DIS           (0x0   << 10 ) /* DIS                      */
#define CFSR_IMPRECISERR_EN            (0x1   << 10 ) /* EN                       */

/* CFSR[PRECISERR] - Precise data access error. The BFAR is written with the faulting address */
#define CFSR_PRECISERR_MSK             (0x1   << 9  )
#define CFSR_PRECISERR                 (0x1   << 9  )
#define CFSR_PRECISERR_DIS             (0x0   << 9  ) /* DIS                      */
#define CFSR_PRECISERR_EN              (0x1   << 9  ) /* EN                       */

/* CFSR[IBUSERR] - This bit indicates a bus fault on an instruction prefetch */
#define CFSR_IBUSERR_MSK               (0x1   << 8  )
#define CFSR_IBUSERR                   (0x1   << 8  )
#define CFSR_IBUSERR_DIS               (0x0   << 8  ) /* DIS                      */
#define CFSR_IBUSERR_EN                (0x1   << 8  ) /* EN                       */

/* CFSR[MMARVALID] - This bit is set if the MMAR register has valid contents. */
#define CFSR_MMARVALID_MSK             (0x1   << 7  )
#define CFSR_MMARVALID                 (0x1   << 7  )
#define CFSR_MMARVALID_DIS             (0x0   << 7  ) /* DIS                      */
#define CFSR_MMARVALID_EN              (0x1   << 7  ) /* EN                       */

/* CFSR[MSTKERR] - A derived MemManage fault has occurred on exception entry */
#define CFSR_MSTKERR_MSK               (0x1   << 4  )
#define CFSR_MSTKERR                   (0x1   << 4  )
#define CFSR_MSTKERR_DIS               (0x0   << 4  ) /* DIS                      */
#define CFSR_MSTKERR_EN                (0x1   << 4  ) /* EN                       */

/* CFSR[MUNSTKERR] - A derived MemManage fault has occurred on exception return */
#define CFSR_MUNSTKERR_MSK             (0x1   << 3  )
#define CFSR_MUNSTKERR                 (0x1   << 3  )
#define CFSR_MUNSTKERR_DIS             (0x0   << 3  ) /* DIS                      */
#define CFSR_MUNSTKERR_EN              (0x1   << 3  ) /* EN                       */

/* CFSR[DACCVIOL] - Data access violation. The MMAR is set to the data address which the load store tried to access. */
#define CFSR_DACCVIOL_MSK              (0x1   << 1  )
#define CFSR_DACCVIOL                  (0x1   << 1  )
#define CFSR_DACCVIOL_DIS              (0x0   << 1  ) /* DIS                      */
#define CFSR_DACCVIOL_EN               (0x1   << 1  ) /* EN                       */

/* CFSR[IACCVIOL] - violation on an instruction fetch. */
#define CFSR_IACCVIOL_MSK              (0x1   << 0  )
#define CFSR_IACCVIOL                  (0x1   << 0  )
#define CFSR_IACCVIOL_DIS              (0x0   << 0  ) /* DIS                      */
#define CFSR_IACCVIOL_EN               (0x1   << 0  ) /* EN                       */

/* Reset Value for HFSR*/
#define HFSR_RVAL                      0x0 

/* HFSR[DEBUGEVT] - Debug event, and the Debug Fault Status Register has been updated. */
#define HFSR_DEBUGEVT_MSK              (0x1   << 31 )
#define HFSR_DEBUGEVT                  (0x1   << 31 )
#define HFSR_DEBUGEVT_DIS              (0x0   << 31 ) /* DIS                      */
#define HFSR_DEBUGEVT_EN               (0x1   << 31 ) /* EN                       */

/* HFSR[FORCED] - Configurable fault cannot be activated due to priority or it was disabled. Priority escalated to a HardFault. */
#define HFSR_FORCED_MSK                (0x1   << 30 )
#define HFSR_FORCED                    (0x1   << 30 )
#define HFSR_FORCED_DIS                (0x0   << 30 ) /* DIS                      */
#define HFSR_FORCED_EN                 (0x1   << 30 ) /* EN                       */

/* HFSR[VECTTBL] - Fault was due to vector table read on exception processing */
#define HFSR_VECTTBL_MSK               (0x1   << 1  )
#define HFSR_VECTTBL                   (0x1   << 1  )
#define HFSR_VECTTBL_DIS               (0x0   << 1  ) /* DIS                      */
#define HFSR_VECTTBL_EN                (0x1   << 1  ) /* EN                       */

/* Reset Value for MMFAR*/
#define MMFAR_RVAL                     0x0 

/* MMFAR[ADDRESS] - Data address MPU faulted. */
#define MMFAR_ADDRESS_MSK              (0xFFFFFFFF << 0  )

/* Reset Value for BFAR*/
#define BFAR_RVAL                      0x0 

/* BFAR[ADDRESS] - Updated on precise data access faults */
#define BFAR_ADDRESS_MSK               (0xFFFFFFFF << 0  )

/* Reset Value for STIR*/
#define STIR_RVAL                      0x0 

/* STIR[INTID] - The value written in this field is the interrupt to be triggered. */
#define STIR_INTID_MSK                 (0x3FF << 0  )
// ------------------------------------------------------------------------------------------------
// -----                                        ADC1                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Analog to Digital Converter (pADI_ADC1)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_ADC1 Structure                    */
  __IO uint8_t   STA;                       /*!< ADC Status register                   */
  __I  uint8_t   RESERVED0[3];
  __IO uint8_t   MSKI;                      /*!< Interrupt control register            */
  __I  uint8_t   RESERVED1[3];
  __IO uint32_t  CON;                       /*!< Main control register                 */
  __IO uint16_t  OF;                        /*!< Offset calibration register           */
  __I  uint16_t  RESERVED2;
  __IO uint16_t  INTGN;                     /*!< Gain calibration register when using internal reference */
  __I  uint16_t  RESERVED3;
  __IO uint16_t  EXTGN;                     /*!< Gain calibration register when using external reference */
  __I  uint16_t  RESERVED4;
  __IO uint16_t  VDDGN;                     /*!< Gain calibration register when using AVDD as the ADC reference */
  __I  uint16_t  RESERVED5;
  __IO uint16_t  ADCCFG;                    /*!< "Control register for the VBIAS voltage generator, ground switch and external reference buffer" */
  __I  uint16_t  RESERVED6;
  __IO uint16_t  FLT;                       /*!< Filter configuration register         */
  __I  uint16_t  RESERVED7;
  __IO uint16_t  MDE;                       /*!< mode control register                 */
  __I  uint16_t  RESERVED8;
  __IO uint16_t  RCR;                       /*!< Number of ADC conversions before an ADC interrupt is generated. */
  __I  uint16_t  RESERVED9;
  __IO uint16_t  RCV;                       /*!< "This 16-bit, read-only MMR holds the current number of ADC conversion results" */
  __I  uint16_t  RESERVED10;
  __IO uint16_t  TH;                        /*!< Sets the threshold                    */
  __I  uint16_t  RESERVED11;
  __IO uint8_t   THC;                       /*!< Determines how many cumulative ADC conversion result readings above ADCxTH must occur */
  __I  uint8_t   RESERVED12[3];
  __IO uint8_t   THV;                       /*!< 8-bit threshold exceeded counter register */
  __I  uint8_t   RESERVED13[3];
  __IO uint32_t  ACC;                       /*!< 32-bit accumulator register           */
  __IO uint32_t  ATH;                       /*!< Holds the threshold value for the accumulator comparator */
  __IO uint8_t   PRO;                       /*!< Configuration register for Post processing of ADC results */
  __I  uint8_t   RESERVED14[3];
  __IO uint32_t  DAT;                       /*!< conversion result register            */
} ADI_ADC_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          ADCCFG                                     (*(volatile unsigned short int *) 0x4003001C)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for ADCCFG*/
#define ADCCFG_RVAL                    0xF 

/* ADCCFG[SIMU] - Enable both ADCs */
#define ADCCFG_SIMU_BBA                (*(volatile unsigned long *) 0x426003BC)
#define ADCCFG_SIMU_MSK                (0x1   << 15 )
#define ADCCFG_SIMU                    (0x1   << 15 )
#define ADCCFG_SIMU_DIS                (0x0   << 15 ) /* DIS                      */
#define ADCCFG_SIMU_EN                 (0x1   << 15 ) /* EN                       */

/* ADCCFG[BOOST30] - Boost the Vbias current source ability by 30 times */
#define ADCCFG_BOOST30_BBA             (*(volatile unsigned long *) 0x426003B4)
#define ADCCFG_BOOST30_MSK             (0x1   << 13 )
#define ADCCFG_BOOST30                 (0x1   << 13 )
#define ADCCFG_BOOST30_DIS             (0x0   << 13 ) /* DIS                      */
#define ADCCFG_BOOST30_EN              (0x1   << 13 ) /* EN                       */

/* ADCCFG[PINSEL] - Enable vbias generator, send vbias to selected ain pin bits */
#define ADCCFG_PINSEL_MSK              (0x7   << 8  )
#define ADCCFG_PINSEL_DIS              (0x0   << 8  ) /* Disable VBIAS generator  */
#define ADCCFG_PINSEL_AIN7             (0x4   << 8  ) /* AIN7                     */
#define ADCCFG_PINSEL_AIN11            (0x6   << 8  ) /* AIN11                    */

/* ADCCFG[GNDSWON] - GND_SW */
#define ADCCFG_GNDSWON_BBA             (*(volatile unsigned long *) 0x4260039C)
#define ADCCFG_GNDSWON_MSK             (0x1   << 7  )
#define ADCCFG_GNDSWON                 (0x1   << 7  )
#define ADCCFG_GNDSWON_DIS             (0x0   << 7  ) /* DIS                      */
#define ADCCFG_GNDSWON_EN              (0x1   << 7  ) /* EN                       */

/* ADCCFG[GNDSWRESEN] - 20k resistor in series with GND_SW */
#define ADCCFG_GNDSWRESEN_BBA          (*(volatile unsigned long *) 0x42600398)
#define ADCCFG_GNDSWRESEN_MSK          (0x1   << 6  )
#define ADCCFG_GNDSWRESEN              (0x1   << 6  )
#define ADCCFG_GNDSWRESEN_DIS          (0x0   << 6  ) /* DIS                      */
#define ADCCFG_GNDSWRESEN_EN           (0x1   << 6  ) /* EN                       */

/* ADCCFG[EXTBUF] - Control signals for ext_ref buffers bits */
#define ADCCFG_EXTBUF_MSK              (0x3   << 0  )
#define ADCCFG_EXTBUF_OFF              (0x0   << 0  ) /* OFF                      */
#define ADCCFG_EXTBUF_VREFPN           (0x1   << 0  ) /* VREFPN                   */
#define ADCCFG_EXTBUF_VREFP_VREF2P     (0x2   << 0  ) /* VREFP_VREF2P             */
#define ADCCFG_EXTBUF_VREFP_ONLY       (0x3   << 0  ) /* VREFP_Only             */

#if (__NO_MMR_STRUCTS__==1)

#define          ADC1STA                                    (*(volatile unsigned char      *) 0x40030080)
#define          ADC1MSKI                                   (*(volatile unsigned char      *) 0x40030084)
#define          ADC1CON                                    (*(volatile unsigned long      *) 0x40030088)
#define          ADC1OF                                     (*(volatile unsigned short int *) 0x4003008C)
#define          ADC1INTGN                                  (*(volatile unsigned short int *) 0x40030090)
#define          ADC1EXTGN                                  (*(volatile unsigned short int *) 0x40030094)
#define          ADC1VDDGN                                  (*(volatile unsigned short int *) 0x40030098)
#define          ADCSCFG1                                   (*(volatile unsigned short int *) 0x4003009C)
#define          ADC1FLT                                    (*(volatile unsigned short int *) 0x400300A0)
#define          ADC1MDE                                    (*(volatile unsigned short int *) 0x400300A4)
#define          ADC1RCR                                    (*(volatile unsigned short int *) 0x400300A8)
#define          ADC1RCV                                    (*(volatile unsigned short int *) 0x400300AC)
#define          ADC1TH                                     (*(volatile unsigned short int *) 0x400300B0)
#define          ADC1THC                                    (*(volatile unsigned char      *) 0x400300B4)
#define          ADC1THV                                    (*(volatile unsigned char      *) 0x400300B8)
#define          ADC1ACC                                    (*(volatile unsigned long      *) 0x400300BC)
#define          ADC1ATH                                    (*(volatile unsigned long      *) 0x400300C0)
#define          ADC1PRO                                    (*(volatile unsigned char      *) 0x400300C4)
#define          ADC1DAT                                    (*(volatile unsigned long      *) 0x400300C8)
#endif // (__NO_MMR_STRUCTS__==1)

/* Reset Value for ADC1STA*/
#define ADC1STA_RVAL                   0x0 

/* ADC1STA[CAL] - ADC Calibration status register */
#define ADC1STA_CAL_BBA                (*(volatile unsigned long *) 0x42601014)
#define ADC1STA_CAL_MSK                (0x1   << 5  )
#define ADC1STA_CAL                    (0x1   << 5  )
#define ADC1STA_CAL_DIS                (0x0   << 5  ) /* DIS                      */
#define ADC1STA_CAL_EN                 (0x1   << 5  ) /* EN                       */

/* ADC1STA[ERR] - ADC conversion error status bit. */
#define ADC1STA_ERR_BBA                (*(volatile unsigned long *) 0x42601010)
#define ADC1STA_ERR_MSK                (0x1   << 4  )
#define ADC1STA_ERR                    (0x1   << 4  )
#define ADC1STA_ERR_DIS                (0x0   << 4  ) /* DIS                      */
#define ADC1STA_ERR_EN                 (0x1   << 4  ) /* EN                       */

/* ADC1STA[ATHEX] - ADC Accumulator Comparator Threshold status bit. */
#define ADC1STA_ATHEX_BBA              (*(volatile unsigned long *) 0x4260100C)
#define ADC1STA_ATHEX_MSK              (0x1   << 3  )
#define ADC1STA_ATHEX                  (0x1   << 3  )
#define ADC1STA_ATHEX_DIS              (0x0   << 3  ) /* DIS                      */
#define ADC1STA_ATHEX_EN               (0x1   << 3  ) /* EN                       */

/* ADC1STA[THEX] - ADC comparator threshold. */
#define ADC1STA_THEX_BBA               (*(volatile unsigned long *) 0x42601008)
#define ADC1STA_THEX_MSK               (0x1   << 2  )
#define ADC1STA_THEX                   (0x1   << 2  )
#define ADC1STA_THEX_DIS               (0x0   << 2  ) /* DIS                      */
#define ADC1STA_THEX_EN                (0x1   << 2  ) /* EN                       */

/* ADC1STA[OVR] - ADC overrange bit. */
#define ADC1STA_OVR_BBA                (*(volatile unsigned long *) 0x42601004)
#define ADC1STA_OVR_MSK                (0x1   << 1  )
#define ADC1STA_OVR                    (0x1   << 1  )
#define ADC1STA_OVR_DIS                (0x0   << 1  ) /* DIS                      */
#define ADC1STA_OVR_EN                 (0x1   << 1  ) /* EN                       */

/* ADC1STA[RDY] - valid conversion result */
#define ADC1STA_RDY_BBA                (*(volatile unsigned long *) 0x42601000)
#define ADC1STA_RDY_MSK                (0x1   << 0  )
#define ADC1STA_RDY                    (0x1   << 0  )
#define ADC1STA_RDY_DIS                (0x0   << 0  ) /* DIS                      */
#define ADC1STA_RDY_EN                 (0x1   << 0  ) /* EN                       */

/* Reset Value for ADC1MSKI*/
#define ADC1MSKI_RVAL                  0x0 

/* ADC1MSKI[ATHEX] - ADC Accumulator Comparator Threshold status bit mask */
#define ADC1MSKI_ATHEX_BBA             (*(volatile unsigned long *) 0x4260108C)
#define ADC1MSKI_ATHEX_MSK             (0x1   << 3  )
#define ADC1MSKI_ATHEX                 (0x1   << 3  )
#define ADC1MSKI_ATHEX_DIS             (0x0   << 3  ) /* DIS                      */
#define ADC1MSKI_ATHEX_EN              (0x1   << 3  ) /* EN                       */

/* ADC1MSKI[THEX] - ADC comparator threshold mask */
#define ADC1MSKI_THEX_BBA              (*(volatile unsigned long *) 0x42601088)
#define ADC1MSKI_THEX_MSK              (0x1   << 2  )
#define ADC1MSKI_THEX                  (0x1   << 2  )
#define ADC1MSKI_THEX_DIS              (0x0   << 2  ) /* DIS                      */
#define ADC1MSKI_THEX_EN               (0x1   << 2  ) /* EN                       */

/* ADC1MSKI[OVR] - ADC overrange bit mask. */
#define ADC1MSKI_OVR_BBA               (*(volatile unsigned long *) 0x42601084)
#define ADC1MSKI_OVR_MSK               (0x1   << 1  )
#define ADC1MSKI_OVR                   (0x1   << 1  )
#define ADC1MSKI_OVR_DIS               (0x0   << 1  ) /* DIS                      */
#define ADC1MSKI_OVR_EN                (0x1   << 1  ) /* EN                       */

/* ADC1MSKI[RDY] - valid conversion result mask */
#define ADC1MSKI_RDY_BBA               (*(volatile unsigned long *) 0x42601080)
#define ADC1MSKI_RDY_MSK               (0x1   << 0  )
#define ADC1MSKI_RDY                   (0x1   << 0  )
#define ADC1MSKI_RDY_DIS               (0x0   << 0  ) /* DIS                      */
#define ADC1MSKI_RDY_EN                (0x1   << 0  ) /* EN                       */

/* Reset Value for ADC1CON*/
#define ADC1CON_RVAL                   0x303FF 

/* ADC1CON[ADCEN] - Enable Bit */
#define ADC1CON_ADCEN_BBA              (*(volatile unsigned long *) 0x4260114C)
#define ADC1CON_ADCEN_MSK              (0x1   << 19 )
#define ADC1CON_ADCEN                  (0x1   << 19 )
#define ADC1CON_ADCEN_DIS              (0x0   << 19 ) /* DIS                      */
#define ADC1CON_ADCEN_EN               (0x1   << 19 ) /* EN                       */

/* ADC1CON[ADCCODE] - ADC Output Coding bits */
#define ADC1CON_ADCCODE_BBA            (*(volatile unsigned long *) 0x42601148)
#define ADC1CON_ADCCODE_MSK            (0x1   << 18 )
#define ADC1CON_ADCCODE                (0x1   << 18 )
#define ADC1CON_ADCCODE_INT            (0x0   << 18 ) /* INT                      */
#define ADC1CON_ADCCODE_UINT           (0x1   << 18 ) /* UINT                     */

/* ADC1CON[BUFPOWN] - Negative buffer power down */
#define ADC1CON_BUFPOWN_BBA            (*(volatile unsigned long *) 0x42601144)
#define ADC1CON_BUFPOWN_MSK            (0x1   << 17 )
#define ADC1CON_BUFPOWN                (0x1   << 17 )
#define ADC1CON_BUFPOWN_DIS            (0x0   << 17 ) /* Disable powerdown - Negative buffer is enabled */
#define ADC1CON_BUFPOWN_EN             (0x1   << 17 ) /* Enable powerdown - Negative buffer is disabled */

/* ADC1CON[BUFPOWP] - Positive buffer power down */
#define ADC1CON_BUFPOWP_BBA            (*(volatile unsigned long *) 0x42601140)
#define ADC1CON_BUFPOWP_MSK            (0x1   << 16 )
#define ADC1CON_BUFPOWP                (0x1   << 16 )
#define ADC1CON_BUFPOWP_DIS            (0x0   << 16 ) /* Disable powerdown - Positive buffer is enabled */
#define ADC1CON_BUFPOWP_EN             (0x1   << 16 ) /* Enable powerdown - Positive buffer is disabled */

/* ADC1CON[BUFBYPP] - Positive buffer bypass */
#define ADC1CON_BUFBYPP_BBA            (*(volatile unsigned long *) 0x4260113C)
#define ADC1CON_BUFBYPP_MSK            (0x1   << 15 )
#define ADC1CON_BUFBYPP                (0x1   << 15 )
#define ADC1CON_BUFBYPP_DIS            (0x0   << 15 ) /* DIS                      */
#define ADC1CON_BUFBYPP_EN             (0x1   << 15 ) /* EN                       */

/* ADC1CON[BUFBYPN] - Negative buffer bypass */
#define ADC1CON_BUFBYPN_BBA            (*(volatile unsigned long *) 0x42601138)
#define ADC1CON_BUFBYPN_MSK            (0x1   << 14 )
#define ADC1CON_BUFBYPN                (0x1   << 14 )
#define ADC1CON_BUFBYPN_DIS            (0x0   << 14 ) /* DIS                      */
#define ADC1CON_BUFBYPN_EN             (0x1   << 14 ) /* EN                       */

/* ADC1CON[ADCREF] - Reference selection */
#define ADC1CON_ADCREF_MSK             (0x3   << 12 )
#define ADC1CON_ADCREF_INTREF          (0x0   << 12 ) /* INTREF                   */
#define ADC1CON_ADCREF_EXTREF          (0x1   << 12 ) /* EXTREF                   */
#define ADC1CON_ADCREF_EXTREF2         (0x2   << 12 ) /* EXTREF2                  */
#define ADC1CON_ADCREF_AVDDREF         (0x3   << 12 ) /* AVDDREF                  */

/* ADC1CON[ADCDIAG] - Diagnostic Current bits bits */
#define ADC1CON_ADCDIAG_MSK            (0x3   << 10 )
#define ADC1CON_ADCDIAG_DIAG_OFF       (0x0   << 10 ) /* DIAG_OFF                 */
#define ADC1CON_ADCDIAG_DIAG_POS       (0x1   << 10 ) /* DIAG_POS                 */
#define ADC1CON_ADCDIAG_DIAG_NEG       (0x2   << 10 ) /* DIAG_NEG                 */
#define ADC1CON_ADCDIAG_DIAG_ALL       (0x3   << 10 ) /* DIAG_ALL                 */

/* ADC1CON[ADCCP] - AIN+ bits */
#define ADC1CON_ADCCP_MSK              (0x1F  << 5  )
#define ADC1CON_ADCCP_AIN0             (0x0   << 5  ) /* AIN0                     */
#define ADC1CON_ADCCP_AIN1             (0x1   << 5  ) /* AIN1                     */
#define ADC1CON_ADCCP_AIN2             (0x2   << 5  ) /* AIN2                     */
#define ADC1CON_ADCCP_AIN3             (0x3   << 5  ) /* AIN3                     */
#define ADC1CON_ADCCP_AIN4             (0x4   << 5  ) /* AIN4                     */
#define ADC1CON_ADCCP_AIN5             (0x5   << 5  ) /* AIN5                     */
#define ADC1CON_ADCCP_AIN6             (0x6   << 5  ) /* AIN6                     */
#define ADC1CON_ADCCP_AIN7             (0x7   << 5  ) /* AIN7                     */
#define ADC1CON_ADCCP_AIN8             (0x8   << 5  ) /* AIN8                     */
#define ADC1CON_ADCCP_AIN9             (0x9   << 5  ) /* AIN9                     */
#define ADC1CON_ADCCP_AIN10            (0xA   << 5  ) /* AIN10                    */
#define ADC1CON_ADCCP_AIN11            (0xB   << 5  ) /* AIN11                    */
#define ADC1CON_ADCCP_DAC              (0xC   << 5  ) /* DAC                      */
#define ADC1CON_ADCCP_AVDD4            (0xD   << 5  ) /* AVDD4                    */
#define ADC1CON_ADCCP_IOVDD4           (0xE   << 5  ) /* IOVDD4                   */
#define ADC1CON_ADCCP_AGND             (0xF   << 5  ) /* AGND                     */
#define ADC1CON_ADCCP_TEMP             (0x10  << 5  ) /* TEMP                     */

/* ADC1CON[ADCCN] - AIN- bits */
#define ADC1CON_ADCCN_MSK              (0x1F  << 0  )
#define ADC1CON_ADCCN_AIN0             (0x0   << 0  ) /* AIN0                     */
#define ADC1CON_ADCCN_AIN1             (0x1   << 0  ) /* AIN1                     */
#define ADC1CON_ADCCN_AIN2             (0x2   << 0  ) /* AIN2                     */
#define ADC1CON_ADCCN_AIN3             (0x3   << 0  ) /* AIN3                     */
#define ADC1CON_ADCCN_AIN4             (0x4   << 0  ) /* AIN4                     */
#define ADC1CON_ADCCN_AIN5             (0x5   << 0  ) /* AIN5                     */
#define ADC1CON_ADCCN_AIN6             (0x6   << 0  ) /* AIN6                     */
#define ADC1CON_ADCCN_AIN7             (0x7   << 0  ) /* AIN7                     */
#define ADC1CON_ADCCN_AIN8             (0x8   << 0  ) /* AIN8                     */
#define ADC1CON_ADCCN_AIN9             (0x9   << 0  ) /* AIN9                     */
#define ADC1CON_ADCCN_AIN10            (0xA   << 0  ) /* AIN10                    */
#define ADC1CON_ADCCN_AIN11            (0xB   << 0  ) /* AIN11                    */
#define ADC1CON_ADCCN_DAC              (0xC   << 0  ) /* DAC                      */
#define ADC1CON_ADCCN_AGND             (0xF   << 0  ) /* AGND                     */
#define ADC1CON_ADCCN_TEMP             (0x11  << 0  ) /* TEMP                     */

/* Reset Value for ADC1OF*/
#define ADC1OF_RVAL                    0x0 

/* ADC1OF[VALUE] - Offset */
#define ADC1OF_VALUE_MSK               (0xFFFF << 0  )

/* Reset Value for ADC1INTGN*/
#define ADC1INTGN_RVAL                 0x5555 

/* ADC1INTGN[VALUE] - Gain with Int Ref */
#define ADC1INTGN_VALUE_MSK            (0xFFFF << 0  )

/* Reset Value for ADC1EXTGN*/
#define ADC1EXTGN_RVAL                 0x5555 

/* ADC1EXTGN[VALUE] - Gain with Ext Ref */
#define ADC1EXTGN_VALUE_MSK            (0xFFFF << 0  )

/* Reset Value for ADC1VDDGN*/
#define ADC1VDDGN_RVAL                 0x5555 

/* ADC1VDDGN[VALUE] - Gain with Avdd Ref */
#define ADC1VDDGN_VALUE_MSK            (0xFFFF << 0  )

/* Reset Value for ADCSCFG1*/
#define ADCSCFG1_RVAL                  0xF 

/* ADCSCFG1[SIMU] - Enable both ADCs */
#define ADCSCFG1_SIMU_BBA              (*(volatile unsigned long *) 0x426013BC)
#define ADCSCFG1_SIMU_MSK              (0x1   << 15 )
#define ADCSCFG1_SIMU                  (0x1   << 15 )
#define ADCSCFG1_SIMU_DIS              (0x0   << 15 ) /* DIS                      */
#define ADCSCFG1_SIMU_EN               (0x1   << 15 ) /* EN                       */

/* ADCSCFG1[BOOST30] - Boost the Vbias current source ability by 30 times */
#define ADCSCFG1_BOOST30_BBA           (*(volatile unsigned long *) 0x426013B4)
#define ADCSCFG1_BOOST30_MSK           (0x1   << 13 )
#define ADCSCFG1_BOOST30               (0x1   << 13 )
#define ADCSCFG1_BOOST30_DIS           (0x0   << 13 ) /* DIS                      */
#define ADCSCFG1_BOOST30_EN            (0x1   << 13 ) /* EN                       */

/* ADCSCFG1[PINSEL] - Enable vbias generator, send vbias to selected ain pin bits */
#define ADCSCFG1_PINSEL_MSK            (0x7   << 8  )
#define ADCSCFG1_PINSEL_DIS            (0x0   << 8  ) /* Disable VBIAS generator  */
#define ADCSCFG1_PINSEL_AIN7           (0x4   << 8  ) /* AIN7                     */
#define ADCSCFG1_PINSEL_AIN11          (0x6   << 8  ) /* AIN11                    */

/* ADCSCFG1[GNDSWON] - GND_SW */
#define ADCSCFG1_GNDSWON_BBA           (*(volatile unsigned long *) 0x4260139C)
#define ADCSCFG1_GNDSWON_MSK           (0x1   << 7  )
#define ADCSCFG1_GNDSWON               (0x1   << 7  )
#define ADCSCFG1_GNDSWON_DIS           (0x0   << 7  ) /* DIS                      */
#define ADCSCFG1_GNDSWON_EN            (0x1   << 7  ) /* EN                       */

/* ADCSCFG1[GNDSWRESEN] - 20k resistor in series with GND_SW */
#define ADCSCFG1_GNDSWRESEN_BBA        (*(volatile unsigned long *) 0x42601398)
#define ADCSCFG1_GNDSWRESEN_MSK        (0x1   << 6  )
#define ADCSCFG1_GNDSWRESEN            (0x1   << 6  )
#define ADCSCFG1_GNDSWRESEN_DIS        (0x0   << 6  ) /* DIS                      */
#define ADCSCFG1_GNDSWRESEN_EN         (0x1   << 6  ) /* EN                       */

/* ADCSCFG1[EXTBUF] - Control signals for ext_ref buffers bits */
#define ADCSCFG1_EXTBUF_MSK            (0x3   << 0  )
#define ADCSCFG1_EXTBUF_OFF            (0x0   << 0  ) /* OFF                      */
#define ADCSCFG1_EXTBUF_VREFPN         (0x1   << 0  ) /* VREFPN                   */
#define ADCSCFG1_EXTBUF_VREFP_VREF2P   (0x2   << 0  ) /* VREFP_VREF2P             */

/* Reset Value for ADC1FLT*/
#define ADC1FLT_RVAL                   0x7D 

/* ADC1FLT[CHOP] - Enables System-Chopping bits */
#define ADC1FLT_CHOP_BBA               (*(volatile unsigned long *) 0x4260143C)
#define ADC1FLT_CHOP_MSK               (0x1   << 15 )
#define ADC1FLT_CHOP                   (0x1   << 15 )
#define ADC1FLT_CHOP_OFF               (0x0   << 15 ) /* OFF                      */
#define ADC1FLT_CHOP_ON                (0x1   << 15 ) /* ON                       */

/* ADC1FLT[RAVG2] - Enables a running Average-By-2 bits */
#define ADC1FLT_RAVG2_BBA              (*(volatile unsigned long *) 0x42601438)
#define ADC1FLT_RAVG2_MSK              (0x1   << 14 )
#define ADC1FLT_RAVG2                  (0x1   << 14 )
#define ADC1FLT_RAVG2_OFF              (0x0   << 14 ) /* OFF                      */
#define ADC1FLT_RAVG2_ON               (0x1   << 14 ) /* ON                       */

/* ADC1FLT[SINC4EN] - Enable the Sinc4 filter instead of Sinc3 filter. */
#define ADC1FLT_SINC4EN_BBA            (*(volatile unsigned long *) 0x42601430)
#define ADC1FLT_SINC4EN_MSK            (0x1   << 12 )
#define ADC1FLT_SINC4EN                (0x1   << 12 )
#define ADC1FLT_SINC4EN_DIS            (0x0   << 12 ) /* DIS                      */
#define ADC1FLT_SINC4EN_EN             (0x1   << 12 ) /* EN                       */

/* ADC1FLT[AF] - Averaging filter */
#define ADC1FLT_AF_MSK                 (0xF   << 8  )

/* ADC1FLT[NOTCH2] - Inserts a notch at FNOTCH2 */
#define ADC1FLT_NOTCH2_BBA             (*(volatile unsigned long *) 0x4260141C)
#define ADC1FLT_NOTCH2_MSK             (0x1   << 7  )
#define ADC1FLT_NOTCH2                 (0x1   << 7  )
#define ADC1FLT_NOTCH2_DIS             (0x0   << 7  ) /* DIS                      */
#define ADC1FLT_NOTCH2_EN              (0x1   << 7  ) /* EN                       */

/* ADC1FLT[SF] - SINC Filter value */
#define ADC1FLT_SF_MSK                 (0x7F  << 0  )

/* Reset Value for ADC1MDE*/
#define ADC1MDE_RVAL                   0x0 

/* ADC1MDE[PGA] - PGA Gain Select bit */
#define ADC1MDE_PGA_MSK                (0xF   << 4  )
#define ADC1MDE_PGA_G1                 (0x0   << 4  ) /* G1                       */
#define ADC1MDE_PGA_G2                 (0x1   << 4  ) /* G2                       */
#define ADC1MDE_PGA_G4                 (0x2   << 4  ) /* G4                       */
#define ADC1MDE_PGA_G8                 (0x3   << 4  ) /* G8                       */
#define ADC1MDE_PGA_G16                (0x4   << 4  ) /* G16                      */
#define ADC1MDE_PGA_G32                (0x5   << 4  ) /* G32                      */
#define ADC1MDE_PGA_G64                (0x6   << 4  ) /* G64                      */
#define ADC1MDE_PGA_G128               (0x7   << 4  ) /* G128                     */

/* ADC1MDE[ADCMOD2] - ADC modulator gain of 2 control bits */
#define ADC1MDE_ADCMOD2_BBA            (*(volatile unsigned long *) 0x4260148C)
#define ADC1MDE_ADCMOD2_MSK            (0x1   << 3  )
#define ADC1MDE_ADCMOD2                (0x1   << 3  )
#define ADC1MDE_ADCMOD2_MOD2OFF        (0x0   << 3  ) /* MOD2OFF                  */
#define ADC1MDE_ADCMOD2_MOD2ON         (0x1   << 3  ) /* MOD2ON                   */

/* ADC1MDE[ADCMD] - ADC Mode bits */
#define ADC1MDE_ADCMD_MSK              (0x7   << 0  )
#define ADC1MDE_ADCMD_OFF              (0x0   << 0  ) /* OFF                      */
#define ADC1MDE_ADCMD_CONT             (0x1   << 0  ) /* CONT                     */
#define ADC1MDE_ADCMD_SINGLE           (0x2   << 0  ) /* SINGLE                   */
#define ADC1MDE_ADCMD_IDLE             (0x3   << 0  ) /* IDLE                     */
#define ADC1MDE_ADCMD_INTOCAL          (0x4   << 0  ) /* INTOCAL                  */
#define ADC1MDE_ADCMD_INTGCAL          (0x5   << 0  ) /* INTGCAL                  */
#define ADC1MDE_ADCMD_SYSOCAL          (0x6   << 0  ) /* SYSOCAL                  */
#define ADC1MDE_ADCMD_SYSGCAL          (0x7   << 0  ) /* SYSGCAL                  */

/* Reset Value for ADC1RCR*/
#define ADC1RCR_RVAL                   0x1 

/* ADC1RCR[VALUE] -  */
#define ADC1RCR_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for ADC1RCV*/
#define ADC1RCV_RVAL                   0x0 

/* ADC1RCV[VALUE] -  */
#define ADC1RCV_VALUE_MSK              (0xFFFF << 0  )

/* Reset Value for ADC1TH*/
#define ADC1TH_RVAL                    0x0 

/* ADC1TH[VALUE] -  */
#define ADC1TH_VALUE_MSK               (0xFFFF << 0  )

/* Reset Value for ADC1THC*/
#define ADC1THC_RVAL                   0x1 

/* ADC1THC[VALUE] -  */
#define ADC1THC_VALUE_MSK              (0xFF  << 0  )

/* Reset Value for ADC1THV*/
#define ADC1THV_RVAL                   0x0 

/* ADC1THV[VALUE] -  */
#define ADC1THV_VALUE_MSK              (0xFF  << 0  )

/* Reset Value for ADC1ACC*/
#define ADC1ACC_RVAL                   0x0 

/* ADC1ACC[VALUE] -  */
#define ADC1ACC_VALUE_MSK              (0xFFFFFFFF << 0  )

/* Reset Value for ADC1ATH*/
#define ADC1ATH_RVAL                   0x0 

/* ADC1ATH[VALUE] -  */
#define ADC1ATH_VALUE_MSK              (0xFFFFFFFF << 0  )

/* Reset Value for ADC1PRO*/
#define ADC1PRO_RVAL                   0x0 

/* ADC1PRO[ACCEN] - ADC Accumulator Enable bits */
#define ADC1PRO_ACCEN_MSK              (0x3   << 4  )
#define ADC1PRO_ACCEN_Off              (0x0   << 4  ) /* Off                      */
#define ADC1PRO_ACCEN_En               (0x1   << 4  ) /* En                       */
#define ADC1PRO_ACCEN_EnNDec           (0x2   << 4  ) /* EnNDec                   */
#define ADC1PRO_ACCEN_EnAccCnt         (0x3   << 4  ) /* EnAccCnt                 */

/* ADC1PRO[CMPEN] - ADC Comparator Enable bits */
#define ADC1PRO_CMPEN_MSK              (0x3   << 2  )
#define ADC1PRO_CMPEN_Off              (0x0   << 2  ) /* Off                      */
#define ADC1PRO_CMPEN_En               (0x1   << 2  ) /* En                       */
#define ADC1PRO_CMPEN_EnCnt            (0x2   << 2  ) /* EnCnt                    */
#define ADC1PRO_CMPEN_EnCntDec         (0x3   << 2  ) /* EnCntDec                 */

/* ADC1PRO[OREN] - ADC OverRange Enable */
#define ADC1PRO_OREN_BBA               (*(volatile unsigned long *) 0x42601884)
#define ADC1PRO_OREN_MSK               (0x1   << 1  )
#define ADC1PRO_OREN                   (0x1   << 1  )
#define ADC1PRO_OREN_DIS               (0x0   << 1  ) /* DIS                      */
#define ADC1PRO_OREN_EN                (0x1   << 1  ) /* EN                       */

/* ADC1PRO[RCEN] - ADC Result Counter Enable */
#define ADC1PRO_RCEN_BBA               (*(volatile unsigned long *) 0x42601880)
#define ADC1PRO_RCEN_MSK               (0x1   << 0  )
#define ADC1PRO_RCEN                   (0x1   << 0  )
#define ADC1PRO_RCEN_DIS               (0x0   << 0  ) /* DIS                      */
#define ADC1PRO_RCEN_EN                (0x1   << 0  ) /* EN                       */

/* Reset Value for ADC1DAT*/
#define ADC1DAT_RVAL                   0x0 

/* ADC1DAT[VALUE] -  */
#define ADC1DAT_VALUE_MSK              (0xFFFFFFFF << 0  )
// ------------------------------------------------------------------------------------------------
// -----                                        ADCSTEP                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Analog to Digital Converter (pADI_ADCSTEP)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_ADCSTEP Structure                 */
  __IO uint16_t  DETCON;                    /*!< Control register for reference detection and the step detection filter */
  __I  uint16_t  RESERVED0;
  __IO uint8_t   DETSTA;                    /*!< Status register for detection         */
  __I  uint8_t   RESERVED1[3];
  __IO uint16_t  STEPTH;                    /*!< Threshold for step detection filter   */
  __I  uint16_t  RESERVED2;
  __IO uint32_t  STEPDAT;                   /*!< Offers coarse data from the output of the step detection filter */
} ADI_ADCSTEP_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          DETCON                                     (*(volatile unsigned short int *) 0x400300E0)
#define          DETSTA                                     (*(volatile unsigned char      *) 0x400300E4)
#define          STEPTH                                     (*(volatile unsigned short int *) 0x400300E8)
#define          STEPDAT                                    (*(volatile unsigned long      *) 0x400300EC)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for DETCON*/
#define DETCON_RVAL                    0x0 

/* DETCON[REFDET] - Enable external reference detection circuit */
#define DETCON_REFDET_BBA              (*(volatile unsigned long *) 0x42601C20)
#define DETCON_REFDET_MSK              (0x1   << 8  )
#define DETCON_REFDET                  (0x1   << 8  )
#define DETCON_REFDET_DIS              (0x0   << 8  ) /* DIS                      */
#define DETCON_REFDET_EN               (0x1   << 8  ) /* EN                       */

/* DETCON[SINC2] - Enable Sinc2 filter */
#define DETCON_SINC2_BBA               (*(volatile unsigned long *) 0x42601C1C)
#define DETCON_SINC2_MSK               (0x1   << 7  )
#define DETCON_SINC2                   (0x1   << 7  )
#define DETCON_SINC2_DIS               (0x0   << 7  ) /* DIS                      */
#define DETCON_SINC2_EN                (0x1   << 7  ) /* EN                       */

/* DETCON[STEPCTRL] - Control the method to generate the step flag */
#define DETCON_STEPCTRL_BBA            (*(volatile unsigned long *) 0x42601C0C)
#define DETCON_STEPCTRL_MSK            (0x1   << 3  )
#define DETCON_STEPCTRL                (0x1   << 3  )
#define DETCON_STEPCTRL_DIS            (0x0   << 3  ) /* DIS                      */
#define DETCON_STEPCTRL_EN             (0x1   << 3  ) /* EN                       */

/* DETCON[ADCSEL] - Select ADC */
#define DETCON_ADCSEL_BBA              (*(volatile unsigned long *) 0x42601C08)
#define DETCON_ADCSEL_MSK              (0x1   << 2  )
#define DETCON_ADCSEL                  (0x1   << 2  )
#define DETCON_ADCSEL_DIS              (0x0   << 2  ) /* DIS                      */
#define DETCON_ADCSEL_EN               (0x1   << 2  ) /* EN                       */

/* DETCON[RATE] - Control the sinc2 filter's time interval */
#define DETCON_RATE_MSK                (0x3   << 0  )

/* Reset Value for DETSTA*/
#define DETSTA_RVAL                    0x0 

/* DETSTA[REFSTA] -  */
#define DETSTA_REFSTA_BBA              (*(volatile unsigned long *) 0x42601C90)
#define DETSTA_REFSTA_MSK              (0x1   << 4  )
#define DETSTA_REFSTA                  (0x1   << 4  )
#define DETSTA_REFSTA_DIS              (0x0   << 4  ) /* DIS                      */
#define DETSTA_REFSTA_EN               (0x1   << 4  ) /* EN                       */

/* DETSTA[DATOF] - STEPDAT Overflow */
#define DETSTA_DATOF_BBA               (*(volatile unsigned long *) 0x42601C8C)
#define DETSTA_DATOF_MSK               (0x1   << 3  )
#define DETSTA_DATOF                   (0x1   << 3  )
#define DETSTA_DATOF_DIS               (0x0   << 3  ) /* DIS                      */
#define DETSTA_DATOF_EN                (0x1   << 3  ) /* EN                       */

/* DETSTA[STEPERR] -  */
#define DETSTA_STEPERR_BBA             (*(volatile unsigned long *) 0x42601C88)
#define DETSTA_STEPERR_MSK             (0x1   << 2  )
#define DETSTA_STEPERR                 (0x1   << 2  )
#define DETSTA_STEPERR_DIS             (0x0   << 2  ) /* DIS                      */
#define DETSTA_STEPERR_EN              (0x1   << 2  ) /* EN                       */

/* DETSTA[STEPFLAG] -  */
#define DETSTA_STEPFLAG_BBA            (*(volatile unsigned long *) 0x42601C84)
#define DETSTA_STEPFLAG_MSK            (0x1   << 1  )
#define DETSTA_STEPFLAG                (0x1   << 1  )
#define DETSTA_STEPFLAG_DIS            (0x0   << 1  ) /* DIS                      */
#define DETSTA_STEPFLAG_EN             (0x1   << 1  ) /* EN                       */

/* DETSTA[STEPDATRDY] -  */
#define DETSTA_STEPDATRDY_BBA          (*(volatile unsigned long *) 0x42601C80)
#define DETSTA_STEPDATRDY_MSK          (0x1   << 0  )
#define DETSTA_STEPDATRDY              (0x1   << 0  )
#define DETSTA_STEPDATRDY_DIS          (0x0   << 0  ) /* DIS                      */
#define DETSTA_STEPDATRDY_EN           (0x1   << 0  ) /* EN                       */

/* Reset Value for STEPTH*/
#define STEPTH_RVAL                    0x0 

/* STEPTH[VALUE] -  */
#define STEPTH_VALUE_MSK               (0x1FF << 0  )

/* Reset Value for STEPDAT*/
#define STEPDAT_RVAL                   0x0 

/* STEPDAT[VALUE] -  */
#define STEPDAT_VALUE_MSK              (0xFFFFFFFF << 0  )
// ------------------------------------------------------------------------------------------------
// -----                                        ADCDMA                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Analog to Digital Converter (pADI_ADCDMA)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_ADCDMA Structure                  */
  __I  uint32_t  RESERVED0[2];
  __IO uint16_t  ADCDMACON;                 /*!< ADC DMA mode Configuration register   */
} ADI_ADCDMA_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          ADCDMACON                                  (*(volatile unsigned short int *) 0x400300F8)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for ADCDMACON*/
#define ADCDMACON_RVAL                 0x0 

/* ADCDMACON[SINC2DMAEN] -  */
#define ADCDMACON_SINC2DMAEN_BBA       (*(volatile unsigned long *) 0x42601F10)
#define ADCDMACON_SINC2DMAEN_MSK       (0x1   << 4  )
#define ADCDMACON_SINC2DMAEN           (0x1   << 4  )
#define ADCDMACON_SINC2DMAEN_DIS       (0x0   << 4  ) /* DIS                      */
#define ADCDMACON_SINC2DMAEN_EN        (0x1   << 4  ) /* EN                       */

/* ADCDMACON[ADC1DMAEN] -  */
#define ADCDMACON_ADC1DMAEN_BBA        (*(volatile unsigned long *) 0x42601F0C)
#define ADCDMACON_ADC1DMAEN_MSK        (0x1   << 3  )
#define ADCDMACON_ADC1DMAEN            (0x1   << 3  )
#define ADCDMACON_ADC1DMAEN_DIS        (0x0   << 3  ) /* DIS                      */
#define ADCDMACON_ADC1DMAEN_EN         (0x1   << 3  ) /* EN                       */

/* ADCDMACON[ADC1CTRL] -  */
#define ADCDMACON_ADC1CTRL_BBA         (*(volatile unsigned long *) 0x42601F08)
#define ADCDMACON_ADC1CTRL_MSK         (0x1   << 2  )
#define ADCDMACON_ADC1CTRL             (0x1   << 2  )
#define ADCDMACON_ADC1CTRL_DIS         (0x0   << 2  ) /* DIS                      */
#define ADCDMACON_ADC1CTRL_EN          (0x1   << 2  ) /* EN                       */

// ------------------------------------------------------------------------------------------------
// -----                                        DAC                                        -----
// ------------------------------------------------------------------------------------------------


/**
  * @brief Digital To Analog Converter (pADI_DAC)
  */

#if (__NO_MMR_STRUCTS__==0)
typedef struct {                            /*!< pADI_DAC Structure                     */
  __IO uint16_t  DACCON;                    /*!< Control Register                      */
  __I  uint16_t  RESERVED0;
  __IO uint32_t  DACDAT;                    /*!< Data Register                         */
} ADI_DAC_TypeDef;
#else // (__NO_MMR_STRUCTS__==0)
#define          DACCON                                     (*(volatile unsigned short int *) 0x40020000)
#define          DACDAT                                     (*(volatile unsigned long      *) 0x40020004)
#endif // (__NO_MMR_STRUCTS__==0)

/* Reset Value for DACCON*/
#define DACCON_RVAL                    0x200 

/* DACCON[DMAEN] - bits */
#define DACCON_DMAEN_BBA               (*(volatile unsigned long *) 0x42400028)
#define DACCON_DMAEN_MSK               (0x1   << 10 )
#define DACCON_DMAEN                   (0x1   << 10 )
#define DACCON_DMAEN_Off               (0x0   << 10 ) /* Off                      */
#define DACCON_DMAEN_On                (0x1   << 10 ) /* On                       */

/* DACCON[PD] -  */
#define DACCON_PD_BBA                  (*(volatile unsigned long *) 0x42400024)
#define DACCON_PD_MSK                  (0x1   << 9  )
#define DACCON_PD                      (0x1   << 9  )
#define DACCON_PD_DIS                  (0x0   << 9  ) /* DIS                      */
#define DACCON_PD_EN                   (0x1   << 9  ) /* EN                       */

/* DACCON[NPN] -  */
#define DACCON_NPN_BBA                 (*(volatile unsigned long *) 0x42400020)
#define DACCON_NPN_MSK                 (0x1   << 8  )
#define DACCON_NPN                     (0x1   << 8  )
#define DACCON_NPN_DIS                 (0x0   << 8  ) /* DIS                      */
#define DACCON_NPN_EN                  (0x1   << 8  ) /* EN                       */

/* DACCON[BUFBYP] -  */
#define DACCON_BUFBYP_BBA              (*(volatile unsigned long *) 0x42400018)
#define DACCON_BUFBYP_MSK              (0x1   << 6  )
#define DACCON_BUFBYP                  (0x1   << 6  )
#define DACCON_BUFBYP_DIS              (0x0   << 6  ) /* DIS                      */
#define DACCON_BUFBYP_EN               (0x1   << 6  ) /* EN                       */

/* DACCON[CLK] - bits */
#define DACCON_CLK_BBA                 (*(volatile unsigned long *) 0x42400014)
#define DACCON_CLK_MSK                 (0x1   << 5  )
#define DACCON_CLK                     (0x1   << 5  )
#define DACCON_CLK_HCLK                (0x0   << 5  ) /* HCLK                     */
#define DACCON_CLK_Timer1              (0x1   << 5  ) /* Timer1                   */

/* DACCON[CLR] - bits */
#define DACCON_CLR_BBA                 (*(volatile unsigned long *) 0x42400010)
#define DACCON_CLR_MSK                 (0x1   << 4  )
#define DACCON_CLR                     (0x1   << 4  )
#define DACCON_CLR_Off                 (0x1   << 4  ) /* Off                      */
#define DACCON_CLR_On                  (0x0   << 4  ) /* On                       */

/* DACCON[MDE] - Mode bits */
#define DACCON_MDE_MSK                 (0x3   << 2  )
#define DACCON_MDE_12bit               (0x0   << 2  ) /* 12bit                    */
#define DACCON_MDE_16BitSlow           (0x3   << 2  ) /* 16BitSlow                */
#define DACCON_MDE_16BitFast           (0x2   << 2  ) /* 16BitFast                */

/* DACCON[RNG] - DAC Range bits */
#define DACCON_RNG_MSK                 (0x3   << 0  )
#define DACCON_RNG_IntVref             (0x0   << 0  ) /* IntVref                  */
#define DACCON_RNG_AVdd                (0x3   << 0  ) /* AVdd                     */

/* Reset Value for DACDAT*/
#define DACDAT_RVAL                    0x0 

/* DACDAT[VALUE] - Data */
#define DACDAT_VALUE_MSK               (0xFFFFF << 12 )
/********************************************
** End of section using anonymous unions   **
*********************************************/

#if defined(__ARMCC_VERSION)
  #pragma pop
#elif defined(__CWCC__)
  #pragma pop
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

/********************************************
** Miscellaneous Definitions               **
*********************************************/

//iEiNr in EiCfg()
#define EXTINT0 0x0 
#define EXTINT1 0x1 
#define EXTINT2 0x2 
#define EXTINT3 0x3 
#define EXTINT4 0x4 
#define EXTINT5 0x5 
#define EXTINT6 0x6 
#define EXTINT7 0x7 
#define EXTINT8 0x8 

//iEnable in EiCfg()	
#define INT_DIS	0x0	
#define INT_EN 0x1

//iMode in EiCfg()	
#define INT_RISE 0x0	
#define INT_FALL 0x1
#define INT_EDGES 0x2
#define INT_HIGH 0x3
#define INT_LOW	0x4	

//Bit values.
#define	BIT0		1
#define	BIT1		2
#define	BIT2		4
#define	BIT3		8
#define	BIT4		0x10
#define	BIT5		0x20
#define	BIT6		0x40
#define	BIT7		0x80


// ------------------------------------------------------------------------------------------------
// -----                                 Peripheral memory map                                -----
// ------------------------------------------------------------------------------------------------
#define ADI_TM0_ADDR                             0x40000000
#define ADI_TM1_ADDR                             0x40000400
#define ADI_PWM_ADDR                             0x40001000
#define ADI_PWRCTL_ADDR                          0x40002400
#define ADI_RESET_ADDR                           0x40002440
#define ADI_INTERRUPT_ADDR                       0x40002420
#define ADI_WDT_ADDR                             0x40002580
#define ADI_WUT_ADDR                             0x40002500
#define ADI_CLKCTL_ADDR                          0x40002000
#define ADI_FEE_ADDR                             0x40002800
#define ADI_I2C_ADDR                             0x40003000
#define ADI_SPI0_ADDR                            0x40004000
#define ADI_SPI1_ADDR                            0x40004400
#define ADI_UART_ADDR                            0x40005000
#define ADI_UART1_ADDR                           0x40005400
#define ADI_UART2_ADDR                           0x40005800
#define ADI_GP0_ADDR                             0x40006000
#define ADI_GP1_ADDR                             0x40006030
#define ADI_GP2_ADDR                             0x40006060
#define ADI_GP3_ADDR                             0x40006090
#define ADI_ANA_ADDR                             0x40008810
#define ADI_DMA_ADDR                             0x40010000
#define ADI_NVIC_ADDR                            0xE000E000
#define ADI_ADC1_ADDR                            0x40030080
#define ADI_ADCSTEP_ADDR                         0x400300E0
#define ADI_ADCTEST_ADDR                         0x40030050
#define ADI_ADCDMA_ADDR                          0x400300F0
#define ADI_EREFBUF_ADDR                         0x400300D0
#define ADI_DAC_ADDR                             0x40020000

// ------------------------------------------------------------------------------------------------
// -----                                Peripheral declaration                                -----
// ------------------------------------------------------------------------------------------------
#define pADI_TM0                      ((ADI_TIMER_TypeDef              *)ADI_TM0_ADDR)
#define pADI_TM1                      ((ADI_TIMER_TypeDef              *)ADI_TM1_ADDR)
#define pADI_PWM                      ((ADI_PWM_TypeDef                *)ADI_PWM_ADDR)
#define pADI_PWRCTL                   ((ADI_PWRCTL_TypeDef             *)ADI_PWRCTL_ADDR)
#define pADI_RESET                    ((ADI_RESET_TypeDef              *)ADI_RESET_ADDR)
#define pADI_INTERRUPT                ((ADI_INTERRUPT_TypeDef          *)ADI_INTERRUPT_ADDR)
#define pADI_WDT                      ((ADI_WDT_TypeDef                *)ADI_WDT_ADDR)
#define pADI_WUT                      ((ADI_WUT_TypeDef                *)ADI_WUT_ADDR)
#define pADI_CLKCTL                   ((ADI_CLKCTL_TypeDef             *)ADI_CLKCTL_ADDR)
#define pADI_FEE                      ((ADI_FEE_TypeDef                *)ADI_FEE_ADDR)
#define pADI_I2C                      ((ADI_I2C_TypeDef                *)ADI_I2C_ADDR)
#define pADI_SPI0                     ((ADI_SPI_TypeDef                *)ADI_SPI0_ADDR)
#define pADI_SPI1                     ((ADI_SPI_TypeDef                *)ADI_SPI1_ADDR)
#define pADI_UART                     ((ADI_UART_TypeDef               *)ADI_UART_ADDR)
#define pADI_UART1                    ((ADI_UART_TypeDef               *)ADI_UART1_ADDR)
#define pADI_UART2                    ((ADI_UART_TypeDef               *)ADI_UART2_ADDR)
#define pADI_GP0                      ((ADI_GPIO_TypeDef               *)ADI_GP0_ADDR)
#define pADI_GP1                      ((ADI_GPIO_TypeDef               *)ADI_GP1_ADDR)
#define pADI_GP2                      ((ADI_GPIO_TypeDef               *)ADI_GP2_ADDR)
#define pADI_GP3                      ((ADI_GPIO_TypeDef               *)ADI_GP3_ADDR)
#define pADI_ANA                      ((ADI_ANA_TypeDef                *)ADI_ANA_ADDR)
#define pADI_DMA                      ((ADI_DMA_TypeDef                *)ADI_DMA_ADDR)
#define pADI_ADC1                     ((ADI_ADC_TypeDef                *)ADI_ADC1_ADDR)
#define pADI_ADCSTEP                  ((ADI_ADCSTEP_TypeDef            *)ADI_ADCSTEP_ADDR)
#define pADI_ADCDMA                   ((ADI_ADCDMA_TypeDef             *)ADI_ADCDMA_ADDR)
#define pADI_DAC                      ((ADI_DAC_TypeDef                *)ADI_DAC_ADDR)

/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group ADUCM363 */
/** @} */ /* End of group CMSIS */

#ifdef __cplusplus
}
#endif 


#endif  // __ADUCM363_H__
