/**
  ******************************************************************************
  * @file    stm32u073xx.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32U073xx Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral's registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup stm32u073xx
  * @{
  */

#ifndef STM32U073xx_H
#define STM32U073xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M0+ Processor and Core Peripherals
   */
#define __CM0PLUS_REV             0 /*!< Core Revision r0p0                            */
#define __MPU_PRESENT             1 /*!< STM32U0xx  provides an MPU                    */
#define __VTOR_PRESENT            1 /*!< Vector  Table  Register supported             */
#define __NVIC_PRIO_BITS          2 /*!< STM32U0xx uses 2 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0 /*!< Set to 1 if different SysTick Config is used  */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32U0XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M Processor Exceptions Numbers *****************************************************************/
  Reset_IRQn                  = -15,    /*!< -15 Reset Vector, invoked on Power up and warm reset              */
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M Hard Fault Interrupt                                   */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M SV Call Interrupt                                     */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M Pend SV Interrupt                                     */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M System Tick Interrupt                                 */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IWDG_IRQn              = 0,      /*!< Window watchdog interrupt + Independent watchdog interrupt        */
  PVD_PVM_IRQn                = 1,      /*!< PVD through EXTI Line detection Interrupt(EXTI lines 16/19/20/21) */
  RTC_TAMP_IRQn               = 2,      /*!< RTC and TAMP interrupts (combined EXTI lines 20 & 21)             */
  FLASH_ECC_IRQn              = 3,      /*!< FLASH global Interrupt + FLASH ECC interrupt                      */
  RCC_CRS_IRQn                = 4,      /*!< RCC global Interrupt + CRS global interrupt                       */
  EXTI0_1_IRQn                = 5,      /*!< EXTI Line0 & Line1 Interrupt                                      */
  EXTI2_3_IRQn                = 6,      /*!< EXTI Line2 & Line3 Interrupt                                      */
  EXTI4_15_IRQn               = 7,      /*!< EXTI Line4 to Line15 Interrupt                                    */
  USB_DRD_FS_IRQn             = 8,      /*!< USB global interrupt (combined with EXTI 36)                      */
  DMA1_Channel1_IRQn          = 9,      /*!< DMA1 Channel 1 Interrupt                                          */
  DMA1_Channel2_3_IRQn        = 10,     /*!< DMA1 Channel 2 and Channel 3 Interrupts                           */
  DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX_OVR_IRQn  = 11,     /*!< DMAMUX_OVR_IT + DMA1 channel 4 to 7 +  DMA2 channel 1 to 5        */
  ADC_COMP1_2_IRQn            = 12,     /*!< ADC and COMP1/COMP2 interrupts (ADC combined with EXTI 17 & 18)   */
  TIM1_BRK_UP_TRG_COM_IRQn    = 13,     /*!< TIM1 break, update, trigger, commutation, error, direction change and index interrupts        */
  TIM1_CC_IRQn                = 14,     /*!< TIM1 Capture Compare interrupt                                    */
  TIM2_IRQn                   = 15,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 16,     /*!< TIM3 global Interrupt                                             */
  TIM6_DAC_LPTIM1_IRQn        = 17,     /*!< TIM6 + LPTIM1 + DAC global interrupt (combined with EXTI 31)      */
  TIM7_LPTIM2_IRQn            = 18,     /*!< TIM7 + LPTIM2 global interrupt (combined with EXTI 32)            */
  TIM15_LPTIM3_IRQn           = 19,     /*!< TIM15 + LPTIM3 global interrupt (combined with EXTI 33)           */
  TIM16_IRQn                  = 20,     /*!< TIM16 global interrupt                                            */
  TSC_IRQn                    = 21,     /*!< TSC global interrupt                                              */
  LCD_IRQn                    = 22,     /*!< LCD global interrupt                                              */
  I2C1_IRQn                   = 23,     /*!< I2C1 global interrupt (combined with EXTI 23)                     */
  I2C2_3_4_IRQn               = 24,     /*!< I2C2 + I2C3 global interrupt (combined with EXTI 22) + I2C4 global interrupt        */
  SPI1_IRQn                   = 25,     /*!< SPI1/I2S1 global interrupt                                        */
  SPI2_3_IRQn                 = 26,     /*!< SPI2 and SPI3 global interrupt                                    */
  USART1_IRQn                 = 27,     /*!< USART1 global interrupt (combined with EXTI 25)                   */
  USART2_LPUART2_IRQn         = 28,     /*!< USART2 global interrupt (combined with EXTI 26) + LPUART2 global interrupt (combined with EXTI lines 35)      */
  USART3_LPUART1_IRQn         = 29,     /*!< USART3  (combined with EXTI 24) + LPUART1 global interrupt (combined with EXTI lines 28)                      */
  USART4_LPUART3_IRQn         = 30,     /*!< USART4  global interrupt (combined with EXTI 20) + LPUART3 (combined with EXTI lines 34)                      */
  RNG_CRYP_IRQn               = 31,     /*!< RNG + CRYPTO global interrupt                                     */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm0plus.h"               /* Cortex-M0+ processor and core peripherals */
#include "system_stm32u0xx.h"
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */

typedef struct
{
  __IO uint32_t ISR;          /*!< ADC interrupt and status register,             Address offset: 0x00 */
  __IO uint32_t IER;          /*!< ADC interrupt enable register,                 Address offset: 0x04 */
  __IO uint32_t CR;           /*!< ADC control register,                          Address offset: 0x08 */
  __IO uint32_t CFGR1;        /*!< ADC configuration register 1,                  Address offset: 0x0C */
  __IO uint32_t CFGR2;        /*!< ADC configuration register 2,                  Address offset: 0x10 */
  __IO uint32_t SMPR;         /*!< ADC sampling time register,                    Address offset: 0x14 */
       uint32_t RESERVED1;    /*!< Reserved,                                                      0x18 */
       uint32_t RESERVED2;    /*!< Reserved,                                                      0x1C */
  __IO uint32_t AWD1TR;       /*!< ADC analog watchdog 1 threshold register,      Address offset: 0x20 */
  __IO uint32_t AWD2TR;       /*!< ADC analog watchdog 2 threshold register,      Address offset: 0x24 */
  __IO uint32_t CHSELR;       /*!< ADC group regular sequencer register,          Address offset: 0x28 */
  __IO uint32_t AWD3TR;       /*!< ADC analog watchdog 3 threshold register,      Address offset: 0x2C */
       uint32_t RESERVED3[4]; /*!< Reserved,                                               0x30 - 0x3C */
  __IO uint32_t DR;           /*!< ADC group regular data register,               Address offset: 0x40 */
       uint32_t RESERVED4[23];/*!< Reserved,                                               0x44 - 0x9C */
  __IO uint32_t AWD2CR;       /*!< ADC analog watchdog 2 configuration register,  Address offset: 0xA0 */
  __IO uint32_t AWD3CR;       /*!< ADC analog watchdog 3 configuration register,  Address offset: 0xA4 */
       uint32_t RESERVED5[3]; /*!< Reserved,                                               0xA8 - 0xB0 */
  __IO uint32_t CALFACT;      /*!< ADC Calibration factor register,               Address offset: 0xB4 */
} ADC_TypeDef;

typedef struct
{
  __IO uint32_t CCR;          /*!< ADC common configuration register,             Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;

/**
  * @brief Comparator
  */
typedef struct
{
  __IO uint32_t CSR;         /*!< COMP control and status register,                                                 Address offset: 0x00 */
} COMP_TypeDef;

typedef struct
{
  __IO uint32_t CSR_ODD;        /*!< COMP control and status register located in register of comparator instance odd, used for bits common to several COMP instances, Address offset: 0x00 */
  __IO uint32_t CSR_EVEN;       /*!< COMP control and status register located in register of comparator instance even, used for bits common to several COMP instances, Address offset: 0x04 */
} COMP_Common_TypeDef;

/**
  * @brief CRC calculation unit
  */
typedef struct
{
  __IO uint32_t DR;             /*!< CRC Data register,                           Address offset: 0x00 */
  __IO uint32_t IDR;            /*!< CRC Independent data register,               Address offset: 0x04 */
  __IO uint32_t CR;             /*!< CRC Control register,                        Address offset: 0x08 */
       uint32_t RESERVED2;      /*!< Reserved,                                                    0x0C */
  __IO uint32_t INIT;           /*!< Initial CRC value register,                  Address offset: 0x10 */
  __IO uint32_t POL;            /*!< CRC polynomial register,                     Address offset: 0x14 */
       uint32_t RESERVED3[246]; /*!< Reserved,                                                         */
  __IO uint32_t HWCFGR;         /*!< CRC IP HWCFGR register,                     Address offset: 0x3F0 */
  __IO uint32_t VERR;           /*!< CRC IP version register,                    Address offset: 0x3F4 */
  __IO uint32_t PIDR;           /*!< CRC IP type identification register,        Address offset: 0x3F8 */
  __IO uint32_t SIDR;           /*!< CRC IP map Size ID register,                Address offset: 0x3FC */
} CRC_TypeDef;

/**
  * @brief Clock Recovery System
  */
typedef struct
{
  __IO uint32_t CR;            /*!< CRS control register,               Address offset: 0x00 */
  __IO uint32_t CFGR;          /*!< CRS configuration register,         Address offset: 0x04 */
  __IO uint32_t ISR;           /*!< CRS interrupt and status register,  Address offset: 0x08 */
  __IO uint32_t ICR;           /*!< CRS interrupt flag clear register,  Address offset: 0x0C */
} CRS_TypeDef;

/**
  * @brief Digital to Analog Converter
  */
typedef struct
{
  __IO uint32_t CR;          /*!< DAC control register,                                    Address offset: 0x00 */
  __IO uint32_t SWTRIGR;     /*!< DAC software trigger register,                           Address offset: 0x04 */
  __IO uint32_t DHR12R1;     /*!< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
  __IO uint32_t DHR12L1;     /*!< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
  __IO uint32_t DHR8R1;      /*!< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
  __IO uint32_t DHR12R2;     /*!< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
  __IO uint32_t DHR12L2;     /*!< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
  __IO uint32_t DHR8R2;      /*!< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
  __IO uint32_t DHR12RD;     /*!< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
  __IO uint32_t DHR12LD;     /*!< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
  __IO uint32_t DHR8RD;      /*!< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
  __IO uint32_t DOR1;        /*!< DAC channel1 data output register,                       Address offset: 0x2C */
  __IO uint32_t DOR2;        /*!< DAC channel2 data output register,                       Address offset: 0x30 */
  __IO uint32_t SR;          /*!< DAC status register,                                     Address offset: 0x34 */
  __IO uint32_t CCR;         /*!< DAC calibration control register,                        Address offset: 0x38 */
  __IO uint32_t MCR;         /*!< DAC mode control register,                               Address offset: 0x3C */
  __IO uint32_t SHSR1;       /*!< DAC Sample and Hold sample time register 1,              Address offset: 0x40 */
  __IO uint32_t SHSR2;       /*!< DAC Sample and Hold sample time register 2,              Address offset: 0x44 */
  __IO uint32_t SHHR;        /*!< DAC Sample and Hold hold time register,                  Address offset: 0x48 */
  __IO uint32_t SHRR;        /*!< DAC Sample and Hold refresh time register,               Address offset: 0x4C */
} DAC_TypeDef;

/**
  * @brief Debug MCU
  */

typedef struct
{
  __IO uint32_t IDCODE;           /*!< MCU device ID code,                        Address offset: 0x00      */
  __IO uint32_t CR;               /*!< Debug configuration register,              Address offset: 0x04      */
  __IO uint32_t APBFZ1;           /*!< Debug APB freeze register 1,               Address offset: 0x08      */
  __IO uint32_t APBFZ2;           /*!< Debug APB freeze register 2,               Address offset: 0x0C      */
} DBGMCU_TypeDef;

/**
  * @brief DMA Controller
  */
typedef struct
{
  __IO uint32_t CCR;         /*!< DMA channel x configuration register        */
  __IO uint32_t CNDTR;       /*!< DMA channel x number of data register       */
  __IO uint32_t CPAR;        /*!< DMA channel x peripheral address register   */
  __IO uint32_t CMAR;        /*!< DMA channel x memory address register       */
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;         /*!< DMA interrupt status register,                 Address offset: 0x00 */
  __IO uint32_t IFCR;        /*!< DMA interrupt flag clear register,             Address offset: 0x04 */
} DMA_TypeDef;

/**
  * @brief DMA Multiplexer
  */
typedef struct
{
  __IO uint32_t   CCR;       /*!< DMA Multiplexer Channel x Control Register    Address offset: 0x0004 * (channel x) */
}DMAMUX_Channel_TypeDef;

typedef struct
{
  __IO uint32_t   CSR;       /*!< DMA Channel Status Register                    Address offset: 0x0080   */
  __IO uint32_t   CFR;       /*!< DMA Channel Clear Flag Register                Address offset: 0x0084   */
}DMAMUX_ChannelStatus_TypeDef;

typedef struct
{
  __IO uint32_t   RGCR;        /*!< DMA Request Generator x Control Register     Address offset: 0x0100 + 0x0004 * (Req Gen x) */
}DMAMUX_RequestGen_TypeDef;

typedef struct
{
  __IO uint32_t   RGSR;        /*!< DMA Request Generator Status Register        Address offset: 0x0140   */
  __IO uint32_t   RGCFR;       /*!< DMA Request Generator Clear Flag Register    Address offset: 0x0144   */
}DMAMUX_RequestGenStatus_TypeDef;

/**
  * @brief Asynch Interrupt/Event Controller (EXTI)
  */
typedef struct
{
  __IO uint32_t RTSR1;          /*!< EXTI Rising Trigger Selection Register 1,        Address offset:   0x00 */
  __IO uint32_t FTSR1;          /*!< EXTI Falling Trigger Selection Register 1,       Address offset:   0x04 */
  __IO uint32_t SWIER1;         /*!< EXTI Software Interrupt event Register 1,        Address offset:   0x08 */
  __IO uint32_t RPR1;           /*!< EXTI Rising Pending Register 1,                  Address offset:   0x0C */
  __IO uint32_t FPR1;           /*!< EXTI Falling Pending Register 1,                 Address offset:   0x10 */
       uint32_t RESERVED1[19];  /*!< Reserved 1,                                                0x14 -- 0x5C */
  __IO uint32_t EXTICR[4];      /*!< EXIT External Interrupt Configuration Register,            0x60 -- 0x6C */
       uint32_t RESERVED3[4];   /*!< Reserved 3,                                                0x70 -- 0x7C */
  __IO uint32_t IMR1;           /*!< EXTI Interrupt Mask Register 1,                  Address offset:   0x80 */
  __IO uint32_t EMR1;           /*!< EXTI Event Mask Register 1,                      Address offset:   0x84 */
       uint32_t RESERVED4[2];   /*!< Reserved 4,                                                0x88 -- 0x8C */
  __IO uint32_t IMR2;           /*!< EXTI Interrupt Mask Register 2,                  Address offset:   0x90 */
  __IO uint32_t EMR2;           /*!< EXTI Event Mask Register 2,                      Address offset:   0x94 */
} EXTI_TypeDef;

/**
  * @brief FLASH Registers
  */
typedef struct
{
  __IO uint32_t ACR;           /*!< FLASH Access Control register,                     Address offset: 0x00 */
       uint32_t RESERVED1;     /*!< Reserved1,                                         Address offset: 0x04 */
  __IO uint32_t KEYR;          /*!< FLASH Key register,                                Address offset: 0x08 */
  __IO uint32_t OPTKEYR;       /*!< FLASH Option Key register,                         Address offset: 0x0C */
  __IO uint32_t SR;            /*!< FLASH Status register,                             Address offset: 0x10 */
  __IO uint32_t CR;            /*!< FLASH Control register,                            Address offset: 0x14 */
  __IO uint32_t ECCR;          /*!< FLASH ECC register,                                Address offset: 0x18 */
       uint32_t RESERVED2;     /*!< Reserved2,                                         Address offset: 0x1C */
  __IO uint32_t OPTR;          /*!< FLASH Option register,                             Address offset: 0x20 */
       uint32_t RESERVED3[2];  /*!< Reserved3,                                                 0x24 -- 0x28 */
  __IO uint32_t WRP1AR;        /*!< FLASH Bank WRP area A address register,            Address offset: 0x2C */
  __IO uint32_t WRP1BR;        /*!< FLASH Bank WRP area B address register,            Address offset: 0x30 */
       uint32_t RESERVED4[19]; /*!< Reserved4,                                                0x34 -- 0x7C */
  __IO uint32_t SECR;          /*!< FLASH Security option register,                    Address offset: 0x80 */
       uint32_t RESERVED5;     /*!< Reserved5,                                         Address offset: 0x84 */
  __IO uint32_t OEM1KEYW0R;    /*!< FLASH OEM1 key register 1,                         Address offset: 0x88 */
  __IO uint32_t OEM1KEYW1R;    /*!< FLASH OEM1 key register 2,                         Address offset: 0x8C */
  __IO uint32_t OEM1KEYW2R;    /*!< FLASH OEM1 key register 3,                         Address offset: 0x90 */
  __IO uint32_t OEM1KEYW3R;    /*!< FLASH OEM1 key register 4,                         Address offset: 0x94 */
  __IO uint32_t OEM2KEYW0R;    /*!< FLASH OEM2 key register 5,                         Address offset: 0x98 */
  __IO uint32_t OEM2KEYW1R;    /*!< FLASH OEM2 key register 6,                         Address offset: 0x9C */
  __IO uint32_t OEM2KEYW2R;    /*!< FLASH OEM2 key register 7,                         Address offset: 0xA0 */
  __IO uint32_t OEM2KEYW3R;    /*!< FLASH OEM2 key register 8,                         Address offset: 0xA4 */
  __IO uint32_t OEMKEYSR;      /*!< FLASH OEM key status register,                     Address offset: 0xA8 */
  __IO uint32_t HDPCR;         /*!< FLASH HDP control register,                        Address offset: 0xAC */
  __IO uint32_t HDPEXTR;       /*!< FLASH HDP extension register,                      Address offset: 0xB0 */
} FLASH_TypeDef;

/**
  * @brief General Purpose I/O
  */
typedef struct
{
  __IO uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
  __IO uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  __IO uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */
} GPIO_TypeDef;

/**
  * @brief Instruction Cache
  */

typedef struct
{
  __IO uint32_t CR;             /*!< ICACHE control register,               Address offset: 0x00 */
  __IO uint32_t SR;             /*!< ICACHE status register,                Address offset: 0x04 */
  __IO uint32_t IER;            /*!< ICACHE interrupt enable register,      Address offset: 0x08 */
  __IO uint32_t FCR;            /*!< ICACHE Flag clear register,            Address offset: 0x0C */
  __IO uint32_t HMONR;          /*!< ICACHE hit monitor register,           Address offset: 0x10 */
  __IO uint32_t MMONR;          /*!< ICACHE miss monitor register,          Address offset: 0x14 */
       uint32_t RESERVED1[2];   /*!< Reserved,                              Address offset: 0x018-0x01C */
  __IO uint32_t CCR0;           /*!< ICACHE region 0 control register,      Address offset: 0x20 */
  __IO uint32_t CCR1;           /*!< ICACHE region 1 control register,      Address offset: 0x24 */
  __IO uint32_t CCR2;           /*!< ICACHE region 2 control register,      Address offset: 0x28 */
  __IO uint32_t CCR3;           /*!< ICACHE region 3 control register,      Address offset: 0x2C */
       uint32_t RESERVED2[240]; /*!< Reserved,                              Address offset: 0x30-0x3EC */
  __IO uint32_t HWCFGR;         /*!< ICACHE HW configuration register,      Address offset: 0x3F0 */
  __IO uint32_t VERR;           /*!< ICACHE version register,               Address offset: 0x3F4 */
  __IO uint32_t IPIDR;          /*!< ICACHE IP identification register,     Address offset: 0x3F8 */
  __IO uint32_t SIDR;           /*!< ICACHE size identification register,   Address offset: 0x3FC */
} ICACHE_TypeDef;

/**
  * @brief Inter-integrated Circuit Interface
  */
typedef struct
{
  __IO uint32_t CR1;         /*!< I2C Control register 1,            Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< I2C Control register 2,            Address offset: 0x04 */
  __IO uint32_t OAR1;        /*!< I2C Own address 1 register,        Address offset: 0x08 */
  __IO uint32_t OAR2;        /*!< I2C Own address 2 register,        Address offset: 0x0C */
  __IO uint32_t TIMINGR;     /*!< I2C Timing register,               Address offset: 0x10 */
  __IO uint32_t RESERVED;    /*!< Reserved,                          Address offset: 0x14 */
  __IO uint32_t ISR;         /*!< I2C Interrupt and status register, Address offset: 0x18 */
  __IO uint32_t ICR;         /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  __IO uint32_t RESERVED0;   /*!< Reserved,                          Address offset: 0x20 */
  __IO uint32_t RXDR;        /*!< I2C Receive data register,         Address offset: 0x24 */
  __IO uint32_t TXDR;        /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;

/**
  * @brief Independent WATCHDOG
  */
typedef struct
{
  __IO uint32_t KR;          /*!< IWDG Key register,                Address offset: 0x00 */
  __IO uint32_t PR;          /*!< IWDG Prescaler register,          Address offset: 0x04 */
  __IO uint32_t RLR;         /*!< IWDG Reload register,             Address offset: 0x08 */
  __IO uint32_t SR;          /*!< IWDG Status register,             Address offset: 0x0C */
  __IO uint32_t WINR;        /*!< IWDG Window register,             Address offset: 0x10 */
  __IO uint32_t EWCR;        /*!< IWDG wake-up interrupt register,  Address offset: 0x14 */
} IWDG_TypeDef;

/**
  * @brief LCD
  */

typedef struct
{
  __IO uint32_t CR;          /*!< LCD control register,         Address offset: 0x00 */
  __IO uint32_t FCR;         /*!< LCD frame control register,   Address offset: 0x04 */
  __IO uint32_t SR;          /*!< LCD status register,          Address offset: 0x08 */
  __IO uint32_t CLR;         /*!< LCD clear register,           Address offset: 0x0C */
  uint32_t RESERVED;         /*!< Reserved,                     Address offset: 0x10 */
  __IO uint32_t RAM[16];     /*!< LCD display memory,           Address offset: 0x14-0x50 */
} LCD_TypeDef;

/**
  * @brief LPTIMER
  */
typedef struct
{
  __IO uint32_t ISR;          /*!< LPTIM Interrupt and Status register,              Address offset: 0x00 */
  __IO uint32_t ICR;          /*!< LPTIM Interrupt Clear register,                   Address offset: 0x04 */
  __IO uint32_t DIER;         /*!< LPTIM Interrupt Enable register,                  Address offset: 0x08 */
  __IO uint32_t CFGR;         /*!< LPTIM Configuration register,                     Address offset: 0x0C */
  __IO uint32_t CR;           /*!< LPTIM Control register,                           Address offset: 0x10 */
  __IO uint32_t CCR1;         /*!< LPTIM Capture/Compare register 1,                 Address offset: 0x14 */
  __IO uint32_t ARR;          /*!< LPTIM Autoreload register,                        Address offset: 0x18 */
  __IO uint32_t CNT;          /*!< LPTIM Counter register,                           Address offset: 0x1C */
  __IO uint32_t RESERVED0;    /*!< Reserved,                                         Address offset: 0x20 */
  __IO uint32_t CFGR2;        /*!< LPTIM Configuration register 2,                   Address offset: 0x24 */
  __IO uint32_t RCR;          /*!< LPTIM Repetition register,                        Address offset: 0x28 */
  __IO uint32_t CCMR1;        /*!< LPTIM Capture/Compare mode register 1,            Address offset: 0x2C */
  __IO uint32_t CCMR2;        /*!< LPTIM Capture/Compare mode register 2,            Address offset: 0x30 */
  __IO uint32_t CCR2;         /*!< LPTIM Capture/Compare register 2,                 Address offset: 0x34 */
  __IO uint32_t CCR3;         /*!< LPTIM Capture/Compare register 3,                 Address offset: 0x38 */
  __IO uint32_t CCR4;         /*!< LPTIM Capture/Compare register 4,                 Address offset: 0x3C */
} LPTIM_TypeDef;

/**
  * @brief Operational Amplifier (OPAMP)
  */
typedef struct
{
  __IO uint32_t CSR;         /*!< OPAMP control/status register,                     Address offset: 0x00 */
  __IO uint32_t OTR;         /*!< OPAMP offset trimming register for normal mode,    Address offset: 0x04 */
  __IO uint32_t LPOTR;       /*!< OPAMP offset trimming register for low power mode, Address offset: 0x08 */
} OPAMP_TypeDef;

typedef struct
{
  __IO uint32_t CSR;         /*!< OPAMP control/status register, used for bits common to
                                  several OPAMP instances, Address offset: 0x00         */
} OPAMP_Common_TypeDef;


/**
  * @brief Power Control
  */
typedef struct
{
  __IO uint32_t CR1;          /*!< PWR Power Control Register 1,                     Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< PWR Power Control Register 2,                     Address offset: 0x04 */
  __IO uint32_t CR3;          /*!< PWR Power Control Register 3,                     Address offset: 0x08 */
  __IO uint32_t CR4;          /*!< PWR Power Control Register 4,                     Address offset: 0x0C */
  __IO uint32_t SR1;          /*!< PWR Power Status Register 1,                      Address offset: 0x10 */
  __IO uint32_t SR2;          /*!< PWR Power Status Register 2,                      Address offset: 0x14 */
  __IO uint32_t SCR;          /*!< PWR Power Status Reset Register,                  Address offset: 0x18 */
       uint32_t RESERVED0;    /*!< Reserved,                                         Address offset: 0x1C */
  __IO uint32_t PUCRA;        /*!< PWR Pull-Up Control Register of port A,           Address offset: 0x20 */
  __IO uint32_t PDCRA;        /*!< PWR Pull-Down Control Register of port A,         Address offset: 0x24 */
  __IO uint32_t PUCRB;        /*!< PWR Pull-Up Control Register of port B,           Address offset: 0x28 */
  __IO uint32_t PDCRB;        /*!< PWR Pull-Down Control Register of port B,         Address offset: 0x2C */
  __IO uint32_t PUCRC;        /*!< PWR Pull-Up Control Register of port C,           Address offset: 0x30 */
  __IO uint32_t PDCRC;        /*!< PWR Pull-Down Control Register of port C,         Address offset: 0x34 */
  __IO uint32_t PUCRD;        /*!< PWR Pull-Up Control Register of port D,           Address offset: 0x38 */
  __IO uint32_t PDCRD;        /*!< PWR Pull-Down Control Register of port D,         Address offset: 0x3C */
  __IO uint32_t PUCRE;        /*!< PWR Pull-Up Control Register of port E,           Address offset: 0x40 */
  __IO uint32_t PDCRE;        /*!< PWR Pull-Down Control Register of port E,         Address offset: 0x44 */
  __IO uint32_t PUCRF;        /*!< PWR Pull-Up Control Register of port F,           Address offset: 0x48 */
  __IO uint32_t PDCRF;        /*!< PWR Pull-Down Control Register of port F,         Address offset: 0x4C */
} PWR_TypeDef;

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;             /*!< RCC Clock Sources Control Register,                                     Address offset: 0x00 */
  __IO uint32_t ICSCR;          /*!< RCC Internal Clock Sources Calibration Register,                        Address offset: 0x04 */
  __IO uint32_t CFGR;           /*!< RCC Regulated Domain Clocks Configuration Register,                     Address offset: 0x08 */
  __IO uint32_t PLLCFGR;        /*!< RCC system PLL configuration register,                                  Address offset: 0x0C */
       uint32_t RESERVED0[2];   /*!< Reserved,                                                               Address offset: 0x10 -- 0x14 */
  __IO uint32_t CIER;           /*!< RCC Clock Interrupt Enable Register,                                    Address offset: 0x18 */
  __IO uint32_t CIFR;           /*!< RCC Clock Interrupt Flag Register,                                      Address offset: 0x1C */
  __IO uint32_t CICR;           /*!< RCC Clock Interrupt Clear Register,                                     Address offset: 0x20 */
       uint32_t RESERVED1;      /*!< Reserved,                                                               Address offset: 0x24 */
  __IO uint32_t AHBRSTR;        /*!< RCC AHB peripheral reset register,                                      Address offset: 0x28 */
  __IO uint32_t IOPRSTR;        /*!< RCC AHB2 peripheral reset register,                                     Address offset: 0x2C */
       uint32_t RESERVED2[2];   /*!< Reserved,                                                               Address offset: 0x30 -- 0x34 */
  __IO uint32_t APBRSTR1;       /*!< RCC APB1 peripheral reset register,                                     Address offset: 0x38 */
       uint32_t RESERVED3;      /*!< Reserved,                                                               Address offset: 0x3C */
  __IO uint32_t APBRSTR2;       /*!< RCC APB2 peripheral reset register,                                     Address offset: 0x40 */
       uint32_t RESERVED4;      /*!< Reserved,                                                               Address offset: 0x44 */
  __IO uint32_t AHBENR;         /*!< RCC AHB peripheral clocks enable register,                              Address offset: 0x48 */
  __IO uint32_t IOPENR;         /*!< RCC IO port enable register,                                            Address offset: 0x4C */
  __IO uint32_t DBGCFGR;        /*!< RCC DBGCFGR control register,                                           Address offset: 0x50 */
       uint32_t RESERVED5;      /*!< Reserved,                                                               Address offset: 0x54 */
  __IO uint32_t APBENR1;        /*!< RCC APB1 peripherals clock enable register,                             Address offset: 0x58 */
       uint32_t RESERVED6;      /*!< Reserved,                                                               Address offset: 0x5C */
  __IO uint32_t APBENR2;        /*!< RCC APB2 peripherals clock enable register,                             Address offset: 0x60 */
       uint32_t RESERVED7;      /*!< Reserved,                                                               Address offset: 0x64 */
  __IO uint32_t AHBSMENR;       /*!< RCC AHB peripheral clocks enable in sleep mode register,                Address offset: 0x68 */
  __IO uint32_t IOPSMENR;       /*!< RCC IO port peripheral clocks enable in sleep mode register,            Address offset: 0x6C */
       uint32_t RESERVED8[2];   /*!< Reserved,                                                               Address offset: 0x70 -- 0x74 */
  __IO uint32_t APBSMENR1;      /*!< RCC APB1 peripheral clocks enable in sleep mode register,               Address offset: 0x78 */
       uint32_t RESERVED9;      /*!< Reserved,                                                               Address offset: 0x7C */
  __IO uint32_t APBSMENR2;      /*!< RCC APB2 peripheral clocks enable in sleep mode register,               Address offset: 0x80 */
       uint32_t RESERVED10;     /*!< Reserved,                                                               Address offset: 0x84 */
  __IO uint32_t CCIPR;          /*!< RCC Peripherals Independent Clocks Configuration Register,              Address offset: 0x88 */
       uint32_t RESERVED11;     /*!< Reserved,                                                               Address offset: 0x8C */
  __IO uint32_t BDCR;           /*!< RCC backup domain control register,                                     Address offset: 0x90 */
  __IO uint32_t CSR;            /*!< RCC clock control & status register,                                    Address offset: 0x94 */
  __IO uint32_t CRRCR;          /*!< RCC clock recovery RC register,                                         Address offset: 0x98 */
} RCC_TypeDef;

/**
  * @brief RNG
  */

typedef struct
{
  __IO uint32_t CR;  /*!< RNG control register, Address offset: 0x00 */
  __IO uint32_t SR;  /*!< RNG status register,  Address offset: 0x04 */
  __IO uint32_t DR;  /*!< RNG data register,    Address offset: 0x08 */
  uint32_t RESERVED;
  __IO uint32_t HTCR;  /*!< RNG health test configuration register, Address offset: 0x10 */
} RNG_TypeDef;

/**
  * @brief Real-Time Clock
  */
typedef struct
{
 __IO uint32_t TR;          /*!< RTC time register,                              Address offset: 0x00 */
 __IO uint32_t DR;          /*!< RTC date register,                              Address offset: 0x04 */
 __IO uint32_t SSR;         /*!< RTC sub second register,                        Address offset: 0x08 */
 __IO uint32_t ICSR;        /*!< RTC initialization control and status register, Address offset: 0x0C */
 __IO uint32_t PRER;        /*!< RTC prescaler register,                         Address offset: 0x10 */
 __IO uint32_t WUTR;        /*!< RTC wakeup timer register,                      Address offset: 0x14 */
 __IO uint32_t CR;          /*!< RTC control register,                           Address offset: 0x18 */
      uint32_t RESERVED0[2];/*!< Reserved,                                       Address offset: 0x20 */
 __IO uint32_t WPR;         /*!< RTC write protection register,                  Address offset: 0x24 */
 __IO uint32_t CALR;        /*!< RTC calibration register,                       Address offset: 0x28 */
 __IO uint32_t SHIFTR;      /*!< RTC shift control register,                     Address offset: 0x2C */
 __IO uint32_t TSTR;        /*!< RTC time stamp time register,                   Address offset: 0x30 */
 __IO uint32_t TSDR;        /*!< RTC time stamp date register,                   Address offset: 0x34 */
 __IO uint32_t TSSSR;       /*!< RTC time-stamp sub second register,             Address offset: 0x38 */
      uint32_t RESERVED1;   /*!< Reserved,                                       Address offset: 0x3C */
 __IO uint32_t ALRMAR;      /*!< RTC alarm A register,                           Address offset: 0x40 */
 __IO uint32_t ALRMASSR;    /*!< RTC alarm A sub second register,                Address offset: 0x44 */
 __IO uint32_t ALRMBR;      /*!< RTC alarm B register,                           Address offset: 0x48 */
 __IO uint32_t ALRMBSSR;    /*!< RTC alarm B sub second register,                Address offset: 0x4C */
 __IO uint32_t SR;          /*!< RTC Status register,                            Address offset: 0x50 */
 __IO uint32_t MISR;        /*!< RTC masked interrupt status register,           Address offset: 0x54 */
      uint32_t RESERVED2;   /*!< Reserved,                                       Address offset: 0x3C */
 __IO uint32_t SCR;         /*!< RTC status Clear register,                      Address offset: 0x5C */
      uint32_t RESERVED3[4];/*!< Reserved,                                       Address offset: 0x58 */
 __IO uint32_t ALRABINR;    /*!< RTC alarm A binary mode register,               Address offset: 0x70 */
 __IO uint32_t ALRBBINR;    /*!< RTC alarm B binary mode register,               Address offset: 0x74 */
} RTC_TypeDef;

/**
  * @brief Tamper and backup registers
  */
typedef struct
{
  __IO uint32_t CR1;            /*!< TAMP configuration register 1,            Address offset: 0x00 */
  __IO uint32_t CR2;            /*!< TAMP configuration register 2,            Address offset: 0x04 */
  __IO uint32_t CR3;            /*!< TAMP configuration register 3,            Address offset: 0x08 */
  __IO uint32_t FLTCR;          /*!< TAMP filter control register,             Address offset: 0x0C */
       uint32_t RESERVED1[7];   /*!< Reserved,                                 Address offset: 0x28 */
  __IO uint32_t IER;            /*!< TAMP interrupt enable register,           Address offset: 0x2C */
  __IO uint32_t SR;             /*!< TAMP status register,                     Address offset: 0x30 */
  __IO uint32_t MISR;           /*!< TAMP masked interrupt status register,    Address offset: 0x34 */
       uint32_t RESERVED2;      /*!< Reserved,                                 Address offset: 0x38 */
  __IO uint32_t SCR;            /*!< TAMP status clear register,               Address offset: 0x3C */
       uint32_t RESERVED4[48];  /*!< Reserved,                                 Address offset: 0x40 -- 0xFC */
  __IO uint32_t BKP0R;          /*!< TAMP backup register 0,                   Address offset: 0x100 */
  __IO uint32_t BKP1R;          /*!< TAMP backup register 1,                   Address offset: 0x104 */
  __IO uint32_t BKP2R;          /*!< TAMP backup register 2,                   Address offset: 0x108 */
  __IO uint32_t BKP3R;          /*!< TAMP backup register 3,                   Address offset: 0x10C */
  __IO uint32_t BKP4R;          /*!< TAMP backup register 4,                   Address offset: 0x110 */
  __IO uint32_t BKP5R;          /*!< TAMP backup register 5,                   Address offset: 0x114 */
  __IO uint32_t BKP6R;          /*!< TAMP backup register 6,                   Address offset: 0x118 */
  __IO uint32_t BKP7R;          /*!< TAMP backup register 7,                   Address offset: 0x11C */
  __IO uint32_t BKP8R;          /*!< TAMP backup register 8,                   Address offset: 0x120 */
} TAMP_TypeDef;

/**
  * @brief System configuration controller
  */

typedef struct
{
  __IO uint32_t CFGR1;          /*!< SYSCFG Control register,                      Address offset: 0x00 */
       uint32_t RESERVED0[5];   /*!< Reserved                                               0x04 --0x14 */
  __IO uint32_t CFGR2;          /*!< SYSCFG Class B register,                      Address offset: 0x18 */
  __IO uint32_t SCSR;           /*!< SYSCFG Backup Sram Erase Register,            Address offset: 0x1C */
  __IO uint32_t SKR;            /*!< SYSCFG Backup Sram Key Register,              Address offset: 0x20 */
  __IO uint32_t TSCCR;          /*!< SYSCFG TSC Comp Register,                     Address offset: 0x24 */
       uint32_t RESERVED1[22];  /*!< Reserved                                               0x28 --0x2C */
  __IO uint32_t IT_LINE_SR[32]; /*!< SYSCFG configuration IT_LINE register,        Address offset: 0x80 */
} SYSCFG_TypeDef;

/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR1;      /*!< SPI Control register 1 (not used in I2S mode),       Address offset: 0x00 */
  __IO uint32_t CR2;      /*!< SPI Control register 2,                              Address offset: 0x04 */
  __IO uint32_t SR;       /*!< SPI Status register,                                 Address offset: 0x08 */
  __IO uint32_t DR;       /*!< SPI data register,                                   Address offset: 0x0C */
  __IO uint32_t CRCPR;    /*!< SPI CRC polynomial register (not used in I2S mode),  Address offset: 0x10 */
  __IO uint32_t RXCRCR;   /*!< SPI Rx CRC register (not used in I2S mode),          Address offset: 0x14 */
  __IO uint32_t TXCRCR;   /*!< SPI Tx CRC register (not used in I2S mode),          Address offset: 0x18 */
  __IO uint32_t I2SCFGR;  /*!< SPI_I2S configuration register,                      Address offset: 0x1C */
  __IO uint32_t I2SPR;    /*!< SPI_I2S prescaler register,                          Address offset: 0x20 */
} SPI_TypeDef;

/**
  * @brief TIM
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,                   Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,                   Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,          Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                      Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,            Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register,      Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                     Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler register,                   Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,                 Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,          Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,           Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,           Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,           Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,           Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,         Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,                 Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
  __IO uint32_t OR1;         /*!< TIM option register,                      Address offset: 0x50 */
  __IO uint32_t CCMR3;       /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
  __IO uint32_t CCR5;        /*!< TIM capture/compare register5,            Address offset: 0x58 */
  __IO uint32_t CCR6;        /*!< TIM capture/compare register6,            Address offset: 0x5C */
  __IO uint32_t AF1;         /*!< TIM alternate function register 1,        Address offset: 0x60 */
  __IO uint32_t AF2;         /*!< TIM alternate function register 2,        Address offset: 0x64 */
  __IO uint32_t TISEL;       /*!< TIM Input Selection register,             Address offset: 0x68 */
} TIM_TypeDef;

/**
  * @brief Touch Sensing Controller (TSC)
  */

typedef struct
{
  __IO uint32_t CR;          /*!< TSC control register,                                     Address offset: 0x00 */
  __IO uint32_t IER;         /*!< TSC interrupt enable register,                            Address offset: 0x04 */
  __IO uint32_t ICR;         /*!< TSC interrupt clear register,                             Address offset: 0x08 */
  __IO uint32_t ISR;         /*!< TSC interrupt status register,                            Address offset: 0x0C */
  __IO uint32_t IOHCR;       /*!< TSC I/O hysteresis control register,                      Address offset: 0x10 */
  uint32_t      RESERVED1;   /*!< Reserved,                                                 Address offset: 0x14 */
  __IO uint32_t IOASCR;      /*!< TSC I/O analog switch control register,                   Address offset: 0x18 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                 Address offset: 0x1C */
  __IO uint32_t IOSCR;       /*!< TSC I/O sampling control register,                        Address offset: 0x20 */
  uint32_t      RESERVED3;   /*!< Reserved,                                                 Address offset: 0x24 */
  __IO uint32_t IOCCR;       /*!< TSC I/O channel control register,                         Address offset: 0x28 */
  uint32_t      RESERVED4;   /*!< Reserved,                                                 Address offset: 0x2C */
  __IO uint32_t IOGCSR;      /*!< TSC I/O group control status register,                    Address offset: 0x30 */
  __IO uint32_t IOGXCR[7];   /*!< TSC I/O group x counter register,                         Address offset: 0x34-50 */
} TSC_TypeDef;

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
  __IO uint32_t CR1;    /*!< USART Control register 1,                 Address offset: 0x00 */
  __IO uint32_t CR2;    /*!< USART Control register 2,                 Address offset: 0x04 */
  __IO uint32_t CR3;    /*!< USART Control register 3,                 Address offset: 0x08 */
  __IO uint32_t BRR;    /*!< USART Baud rate register,                 Address offset: 0x0C */
  __IO uint32_t GTPR;   /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  __IO uint32_t RTOR;   /*!< USART Receiver Time Out register,         Address offset: 0x14 */
  __IO uint32_t RQR;    /*!< USART Request register,                   Address offset: 0x18 */
  __IO uint32_t ISR;    /*!< USART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;    /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  __IO uint32_t RDR;    /*!< USART Receive Data register,              Address offset: 0x24 */
  __IO uint32_t TDR;    /*!< USART Transmit Data register,             Address offset: 0x28 */
  __IO uint32_t PRESC;  /*!< USART clock Prescaler register,           Address offset: 0x2C */
} USART_TypeDef;

/**
  * @brief VREFBUF
  */
typedef struct
{
  __IO uint32_t CSR;         /*!< VREFBUF control and status register,         Address offset: 0x00 */
  __IO uint32_t CCR;         /*!< VREFBUF calibration and control register,    Address offset: 0x04 */
} VREFBUF_TypeDef;

/**
  * @brief Window WATCHDOG
  */

typedef struct
{
  __IO uint32_t CR;   /*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

/**
  * @brief Universal Serial Bus Full Speed Dual Role Device
  */
typedef struct
{
  __IO uint32_t CHEP0R;          /*!< USB Channel/Endpoint 0 register,      Address offset: 0x00 */
  __IO uint32_t CHEP1R;          /*!< USB Channel/Endpoint 1 register,      Address offset: 0x04 */
  __IO uint32_t CHEP2R;          /*!< USB Channel/Endpoint 2 register,      Address offset: 0x08 */
  __IO uint32_t CHEP3R;          /*!< USB Channel/Endpoint 3 register,      Address offset: 0x0C */
  __IO uint32_t CHEP4R;          /*!< USB Channel/Endpoint 4 register,      Address offset: 0x10 */
  __IO uint32_t CHEP5R;          /*!< USB Channel/Endpoint 5 register,      Address offset: 0x14 */
  __IO uint32_t CHEP6R;          /*!< USB Channel/Endpoint 6 register,      Address offset: 0x18 */
  __IO uint32_t CHEP7R;          /*!< USB Channel/Endpoint 7 register,      Address offset: 0x1C */
  __IO uint32_t RESERVED0[8];    /*!< Reserved,                                                  */
  __IO uint32_t CNTR;            /*!< Control register,                     Address offset: 0x40 */
  __IO uint32_t ISTR;            /*!< Interrupt status register,            Address offset: 0x44 */
  __IO uint32_t FNR;             /*!< Frame number register,                Address offset: 0x48 */
  __IO uint32_t DADDR;           /*!< Device address register,              Address offset: 0x4C */
  __IO uint32_t RESERVED1;       /*!< Reserved */
  __IO uint32_t LPMCSR;          /*!< LPM Control and Status register,      Address offset: 0x54 */
  __IO uint32_t BCDR;            /*!< Battery Charging detector register,   Address offset: 0x58 */
} USB_DRD_TypeDef;

/**
  * @brief Universal Serial Bus PacketMemoryArea Buffer Descriptor Table
  */
typedef struct
{
  __IO uint32_t TXBD;             /*!<Transmission buffer address*/
  __IO uint32_t RXBD;             /*!<Reception buffer address */
} USB_DRD_PMABuffDescTypeDef;

/*!< USB PMA SIZE */
#define USB_DRD_PMA_SIZE        (1024U)         /*!< USB PMA Size 1Kbyte */

/**
  * @}
  */

/* =========================================================================================================================== */
/* ================                          Device Specific Peripheral Address Map                           ================ */
/* =========================================================================================================================== */

/*!< Device Electronic Signature */
#define PACKAGE_BASE          (0x1FFF6D00UL)        /*!< Package data register base address     */
#define UID_BASE              (0x1FFF6E50UL)        /*!< Unique device ID register base address */
#define FLASHSIZE_BASE        (0x1FFF6EA0UL)        /*!< Flash size data register base address  */

/*!< Bootloader Firmware */
/************ Bootloader Exit Secure Memory Firmware *************/
#define BL_EXIT_SEC_MEM_BASE    (0x1FFF6000UL)

/**
  * @}
  */

/** @addtogroup STM32U0xx_Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            (0x08000000UL)              /*!< FLASH base address */
#define SRAM1_BASE            (0x20000000UL)              /*!< SRAM1 base address */
#define SRAM1_SIZE_MAX        (0x00008000UL)              /*!< maximum SRAM1 size (up to 32 KBytes) */
#define BKPSRAM2_BASE         (SRAM1_BASE + 0x00008000UL) /*!< SRAM2 BKP(up to 8 KB) base address */
#define PERIPH_BASE           (0x40000000UL)              /*!< Peripheral base address */
#define IOPORT_BASE           (0x50000000UL)              /*!< IOPORT base address */

#define FLASH_SIZE_DEFAULT    0x40000U                    /*!< Flash memory default size */

#define FLASH_SIZE            ((((*((uint16_t *)FLASHSIZE_BASE)) == 0xFFFFU)) ? FLASH_SIZE_DEFAULT : \
                               ((((*((uint16_t *)FLASHSIZE_BASE)) == 0x0000U)) ? FLASH_SIZE_DEFAULT : \
                                (((uint32_t)(*((uint16_t *)FLASHSIZE_BASE)) & (0xFFFFU)) << 10U)))

/* Internal Flash OTP Area */
#define FLASH_OTP_BASE          (0x1FFF6800UL)            /*!< FLASH OTP (one-time programmable) base address */
#define FLASH_OTP_SIZE          (0x400U)                  /*!< 1024 bytes OTP (one-time programmable)         */

/* Flash system Area */
#define FLASH_SYSTEM_BASE_NS    (0x1FFF0000UL)            /*!< FLASH System non-secure base address  */
#define FLASH_SYSTEM_SIZE       (0x6800U)                 /*!< 26 Kbytes system Flash */

/* Peripheral memory map */
#define APBPERIPH_BASE        PERIPH_BASE
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)

/*!< APB peripherals */
#define TIM2_BASE             (APBPERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APBPERIPH_BASE + 0x0400UL)
#define TIM6_BASE             (APBPERIPH_BASE + 0x1000UL)
#define TIM7_BASE             (APBPERIPH_BASE + 0x1400UL)
#define LCD_BASE              (APBPERIPH_BASE + 0x2400UL)
#define RTC_BASE              (APBPERIPH_BASE + 0x2800UL)
#define WWDG_BASE             (APBPERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             (APBPERIPH_BASE + 0x3000UL)
#define SPI2_BASE             (APBPERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APBPERIPH_BASE + 0x3C00UL)
#define USART2_BASE           (APBPERIPH_BASE + 0x4400UL)
#define USART3_BASE           (APBPERIPH_BASE + 0x4800UL)
#define USART4_BASE           (APBPERIPH_BASE + 0x4C00UL)
#define I2C1_BASE             (APBPERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APBPERIPH_BASE + 0x5800UL)
#define USB_DRD_BASE          (APBPERIPH_BASE + 0x5C00UL)   /*USB FS*/
#define CRS_BASE              (APBPERIPH_BASE + 0x6C00UL)
#define PWR_BASE              (APBPERIPH_BASE + 0x7000UL)
#define DAC1_BASE             (APBPERIPH_BASE + 0x7400UL)
#define OPAMP1_BASE           (APBPERIPH_BASE + 0x7800UL)
#define LPTIM1_BASE           (APBPERIPH_BASE + 0x7C00UL)
#define LPUART1_BASE          (APBPERIPH_BASE + 0x8000UL)
#define LPUART2_BASE          (APBPERIPH_BASE + 0x8400UL)
#define I2C3_BASE             (APBPERIPH_BASE + 0x8800UL)
#define LPUART3_BASE          (APBPERIPH_BASE + 0x8C00UL)
#define LPTIM3_BASE           (APBPERIPH_BASE + 0x9000UL)
#define LPTIM2_BASE           (APBPERIPH_BASE + 0x9400UL)
#define USB_DRD_PMAADDR       (APBPERIPH_BASE + 0x9800UL)   /*USB RAM1*/
#define I2C4_BASE             (APBPERIPH_BASE + 0xA000UL)
#define TAMP_BASE             (APBPERIPH_BASE + 0xB000UL)   /*TAMPER (+ BKP Regs)*/
#define SYSCFG_BASE           (APBPERIPH_BASE + 0x10000UL)  /*SYSCFG IF + COMP*/
#define VREFBUF_BASE          (SYSCFG_BASE + 0x0030UL)
#define COMP1_BASE            (SYSCFG_BASE + 0x0200UL)
#define COMP2_BASE            (SYSCFG_BASE + 0x0204UL)
#define ADC1_BASE             (APBPERIPH_BASE + 0x12400UL)
#define ADC1_COMMON_BASE      (APBPERIPH_BASE + 0x12708UL)
#define TIM1_BASE             (APBPERIPH_BASE + 0x12C00UL)
#define SPI1_BASE             (APBPERIPH_BASE + 0x13000UL)
#define USART1_BASE           (APBPERIPH_BASE + 0x13800UL)
#define TIM15_BASE            (APBPERIPH_BASE + 0x14000UL)
#define TIM16_BASE            (APBPERIPH_BASE + 0x14400UL)
#define DBGMCU_BASE           (APBPERIPH_BASE + 0x15800UL)

/*!< AHB peripherals */
#define DMA1_BASE             (AHBPERIPH_BASE + 0x0000UL)

#define DMA1_Channel1_BASE    (DMA1_BASE + 0x0008UL)
#define DMA1_Channel2_BASE    (DMA1_BASE + 0x001CUL)
#define DMA1_Channel3_BASE    (DMA1_BASE + 0x0030UL)
#define DMA1_Channel4_BASE    (DMA1_BASE + 0x0044UL)
#define DMA1_Channel5_BASE    (DMA1_BASE + 0x0058UL)
#define DMA1_Channel6_BASE    (DMA1_BASE + 0x006CUL)
#define DMA1_Channel7_BASE    (DMA1_BASE + 0x0080UL)

#define DMA2_BASE             (AHBPERIPH_BASE + 0x0400UL)

#define DMA2_Channel1_BASE    (DMA2_BASE + 0x0008UL)
#define DMA2_Channel2_BASE    (DMA2_BASE + 0x001CUL)
#define DMA2_Channel3_BASE    (DMA2_BASE + 0x0030UL)
#define DMA2_Channel4_BASE    (DMA2_BASE + 0x0044UL)
#define DMA2_Channel5_BASE    (DMA2_BASE + 0x0058UL)

#define DMAMUX1_BASE           (AHBPERIPH_BASE + 0x0800UL)

#define DMAMUX1_Channel0_BASE    (DMAMUX1_BASE)
#define DMAMUX1_Channel1_BASE    (DMAMUX1_BASE + 0x0004UL)
#define DMAMUX1_Channel2_BASE    (DMAMUX1_BASE + 0x0008UL)
#define DMAMUX1_Channel3_BASE    (DMAMUX1_BASE + 0x000CUL)
#define DMAMUX1_Channel4_BASE    (DMAMUX1_BASE + 0x0010UL)
#define DMAMUX1_Channel5_BASE    (DMAMUX1_BASE + 0x0014UL)
#define DMAMUX1_Channel6_BASE    (DMAMUX1_BASE + 0x0018UL)
#define DMAMUX1_Channel7_BASE    (DMAMUX1_BASE + 0x001CUL)
#define DMAMUX1_Channel8_BASE    (DMAMUX1_BASE + 0x0020UL)
#define DMAMUX1_Channel9_BASE    (DMAMUX1_BASE + 0x0024UL)
#define DMAMUX1_Channel10_BASE   (DMAMUX1_BASE + 0x0028UL)
#define DMAMUX1_Channel11_BASE   (DMAMUX1_BASE + 0x002CUL)

#define DMAMUX1_RequestGenerator0_BASE  (DMAMUX1_BASE + 0x0100UL)
#define DMAMUX1_RequestGenerator1_BASE  (DMAMUX1_BASE + 0x0104UL)
#define DMAMUX1_RequestGenerator2_BASE  (DMAMUX1_BASE + 0x0108UL)
#define DMAMUX1_RequestGenerator3_BASE  (DMAMUX1_BASE + 0x010CUL)

#define DMAMUX1_ChannelStatus_BASE      (DMAMUX1_BASE + 0x0080UL)
#define DMAMUX1_RequestGenStatus_BASE   (DMAMUX1_BASE + 0x0140UL)
#define DMAMUX1_IdRegisters_BASE        (DMAMUX1_BASE + 0x000003EC)

#define RCC_BASE              (AHBPERIPH_BASE + 0x1000UL)
#define EXTI_BASE             (AHBPERIPH_BASE + 0x1800UL) /* AIEC */
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x2000UL)
#define CRC_BASE              (AHBPERIPH_BASE + 0x3000UL)
#define TSC_BASE              (AHBPERIPH_BASE + 0x4000UL)
#define RNG_BASE              (AHBPERIPH_BASE + 0x5000UL)

/*!< IOPORT (GPIO) memory map */
#define GPIOA_BASE            (IOPORT_BASE + 0x0000UL)
#define GPIOB_BASE            (IOPORT_BASE + 0x0400UL)
#define GPIOC_BASE            (IOPORT_BASE + 0x0800UL)
#define GPIOD_BASE            (IOPORT_BASE + 0x0C00UL)
#define GPIOE_BASE            (IOPORT_BASE + 0x1000UL)
#define GPIOF_BASE            (IOPORT_BASE + 0x1400UL)

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC1_COMMON         ((ADC_Common_TypeDef *) ADC1_COMMON_BASE)
#define COMP1               ((COMP_TypeDef *) COMP1_BASE)
#define COMP2               ((COMP_TypeDef *) COMP2_BASE)
#define COMP12_COMMON       ((COMP_Common_TypeDef *) COMP1_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define CRS                 ((CRS_TypeDef *) CRS_BASE)
#define DAC1                ((DAC_TypeDef *) DAC1_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)

#define DMA1_Channel1       ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)

#define DMA2                ((DMA_TypeDef *) DMA2_BASE)

#define DMA2_Channel1       ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE)
#define DMA2_Channel2       ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#define DMA2_Channel3       ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE)
#define DMA2_Channel4       ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE)
#define DMA2_Channel5       ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE)

#define DMAMUX1             ((DMAMUX_Channel_TypeDef *) DMAMUX1_BASE)

#define DMAMUX1_Channel0    ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel0_BASE)
#define DMAMUX1_Channel1    ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel1_BASE)
#define DMAMUX1_Channel2    ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel2_BASE)
#define DMAMUX1_Channel3    ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel3_BASE)
#define DMAMUX1_Channel4    ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel4_BASE)
#define DMAMUX1_Channel5    ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel5_BASE)
#define DMAMUX1_Channel6    ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel6_BASE)

#define DMAMUX1_Channel7    ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel7_BASE)
#define DMAMUX1_Channel8    ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel8_BASE)
#define DMAMUX1_Channel9    ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel9_BASE)
#define DMAMUX1_Channel10   ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel10_BASE)
#define DMAMUX1_Channel11   ((DMAMUX_Channel_TypeDef *) DMAMUX1_Channel11_BASE)

#define DMAMUX1_RequestGenerator0  ((DMAMUX_RequestGen_TypeDef *) DMAMUX1_RequestGenerator0_BASE)
#define DMAMUX1_RequestGenerator1  ((DMAMUX_RequestGen_TypeDef *) DMAMUX1_RequestGenerator1_BASE)
#define DMAMUX1_RequestGenerator2  ((DMAMUX_RequestGen_TypeDef *) DMAMUX1_RequestGenerator2_BASE)
#define DMAMUX1_RequestGenerator3  ((DMAMUX_RequestGen_TypeDef *) DMAMUX1_RequestGenerator3_BASE)

#define DMAMUX1_ChannelStatus      ((DMAMUX_ChannelStatus_TypeDef *) DMAMUX1_ChannelStatus_BASE)
#define DMAMUX1_RequestGenStatus   ((DMAMUX_RequestGenStatus_TypeDef *) DMAMUX1_RequestGenStatus_BASE)

#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define I2C4                ((I2C_TypeDef *) I2C4_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define LCD                 ((LCD_TypeDef *) LCD_BASE)
#define LPTIM1              ((LPTIM_TypeDef *) LPTIM1_BASE)
#define LPTIM2              ((LPTIM_TypeDef *) LPTIM2_BASE)
#define LPTIM3              ((LPTIM_TypeDef *) LPTIM3_BASE)
#define LPUART1             ((USART_TypeDef *) LPUART1_BASE)
#define LPUART2             ((USART_TypeDef *) LPUART2_BASE)
#define LPUART3             ((USART_TypeDef *) LPUART3_BASE)
#define OPAMP1              ((OPAMP_TypeDef *) OPAMP1_BASE)
#define OPAMP1_COMMON       ((OPAMP_Common_TypeDef *) OPAMP1_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define RNG                 ((RNG_TypeDef *) RNG_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define TAMP                ((TAMP_TypeDef *) TAMP_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define TIM15               ((TIM_TypeDef *) TIM15_BASE)
#define TIM16               ((TIM_TypeDef *) TIM16_BASE)
#define TSC                 ((TSC_TypeDef *) TSC_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define USART4              ((USART_TypeDef *) USART4_BASE)
#define USB_DRD_FS          ((USB_DRD_TypeDef *) USB_DRD_BASE)
#define USB_DRD_PMA_BUFF    ((USB_DRD_PMABuffDescTypeDef *) USB_DRD_PMAADDR)
#define VREFBUF             ((VREFBUF_TypeDef *) VREFBUF_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

  /** @addtogroup Hardware_Constant_Definition
    * @{
    */
#define LSI_STARTUP_TIME 130U /*!< LSI Maximum startup time in us */

  /**
    * @}
    */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                      Analog to Digital Converter (ADC)                     */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for ADC_ISR register  *******************/
#define ADC_ISR_ADRDY_Pos              (0UL)
#define ADC_ISR_ADRDY_Msk              (0x1UL << ADC_ISR_ADRDY_Pos)            /*!< 0x00000001 */
#define ADC_ISR_ADRDY                  ADC_ISR_ADRDY_Msk                       /*!< ADC ready flag */
#define ADC_ISR_EOSMP_Pos              (1UL)
#define ADC_ISR_EOSMP_Msk              (0x1UL << ADC_ISR_EOSMP_Pos)            /*!< 0x00000002 */
#define ADC_ISR_EOSMP                  ADC_ISR_EOSMP_Msk                       /*!< ADC group regular end of sampling flag */
#define ADC_ISR_EOC_Pos                (2UL)
#define ADC_ISR_EOC_Msk                (0x1UL << ADC_ISR_EOC_Pos)              /*!< 0x00000004 */
#define ADC_ISR_EOC                    ADC_ISR_EOC_Msk                         /*!< ADC group regular end of unitary conversion flag */
#define ADC_ISR_EOS_Pos                (3UL)
#define ADC_ISR_EOS_Msk                (0x1UL << ADC_ISR_EOS_Pos)              /*!< 0x00000008 */
#define ADC_ISR_EOS                    ADC_ISR_EOS_Msk                         /*!< ADC group regular end of sequence conversions flag */
#define ADC_ISR_OVR_Pos                (4UL)
#define ADC_ISR_OVR_Msk                (0x1UL << ADC_ISR_OVR_Pos)              /*!< 0x00000010 */
#define ADC_ISR_OVR                    ADC_ISR_OVR_Msk                         /*!< ADC group regular overrun flag */
#define ADC_ISR_AWD1_Pos               (7UL)
#define ADC_ISR_AWD1_Msk               (0x1UL << ADC_ISR_AWD1_Pos)             /*!< 0x00000080 */
#define ADC_ISR_AWD1                   ADC_ISR_AWD1_Msk                        /*!< ADC analog watchdog 1 flag */
#define ADC_ISR_AWD2_Pos               (8UL)
#define ADC_ISR_AWD2_Msk               (0x1UL << ADC_ISR_AWD2_Pos)             /*!< 0x00000100 */
#define ADC_ISR_AWD2                   ADC_ISR_AWD2_Msk                        /*!< ADC analog watchdog 2 flag */
#define ADC_ISR_AWD3_Pos               (9UL)
#define ADC_ISR_AWD3_Msk               (0x1UL << ADC_ISR_AWD3_Pos)             /*!< 0x00000200 */
#define ADC_ISR_AWD3                   ADC_ISR_AWD3_Msk                        /*!< ADC analog watchdog 3 flag */
#define ADC_ISR_EOCAL_Pos              (11UL)
#define ADC_ISR_EOCAL_Msk              (0x1UL << ADC_ISR_EOCAL_Pos)            /*!< 0x00000800 */
#define ADC_ISR_EOCAL                  ADC_ISR_EOCAL_Msk                       /*!< ADC end of calibration flag */
#define ADC_ISR_CCRDY_Pos              (13UL)
#define ADC_ISR_CCRDY_Msk              (0x1UL << ADC_ISR_CCRDY_Pos)            /*!< 0x00002000 */
#define ADC_ISR_CCRDY                  ADC_ISR_CCRDY_Msk                       /*!< ADC channel configuration ready flag */

/* Legacy defines */
#define ADC_ISR_EOSEQ           (ADC_ISR_EOS)

/********************  Bit definition for ADC_IER register  *******************/
#define ADC_IER_ADRDYIE_Pos            (0UL)
#define ADC_IER_ADRDYIE_Msk            (0x1UL << ADC_IER_ADRDYIE_Pos)          /*!< 0x00000001 */
#define ADC_IER_ADRDYIE                ADC_IER_ADRDYIE_Msk                     /*!< ADC ready interrupt */
#define ADC_IER_EOSMPIE_Pos            (1UL)
#define ADC_IER_EOSMPIE_Msk            (0x1UL << ADC_IER_EOSMPIE_Pos)          /*!< 0x00000002 */
#define ADC_IER_EOSMPIE                ADC_IER_EOSMPIE_Msk                     /*!< ADC group regular end of sampling interrupt */
#define ADC_IER_EOCIE_Pos              (2UL)
#define ADC_IER_EOCIE_Msk              (0x1UL << ADC_IER_EOCIE_Pos)            /*!< 0x00000004 */
#define ADC_IER_EOCIE                  ADC_IER_EOCIE_Msk                       /*!< ADC group regular end of unitary conversion interrupt */
#define ADC_IER_EOSIE_Pos              (3UL)
#define ADC_IER_EOSIE_Msk              (0x1UL << ADC_IER_EOSIE_Pos)            /*!< 0x00000008 */
#define ADC_IER_EOSIE                  ADC_IER_EOSIE_Msk                       /*!< ADC group regular end of sequence conversions interrupt */
#define ADC_IER_OVRIE_Pos              (4UL)
#define ADC_IER_OVRIE_Msk              (0x1UL << ADC_IER_OVRIE_Pos)            /*!< 0x00000010 */
#define ADC_IER_OVRIE                  ADC_IER_OVRIE_Msk                       /*!< ADC group regular overrun interrupt */
#define ADC_IER_AWD1IE_Pos             (7UL)
#define ADC_IER_AWD1IE_Msk             (0x1UL << ADC_IER_AWD1IE_Pos)           /*!< 0x00000080 */
#define ADC_IER_AWD1IE                 ADC_IER_AWD1IE_Msk                      /*!< ADC analog watchdog 1 interrupt */
#define ADC_IER_AWD2IE_Pos             (8UL)
#define ADC_IER_AWD2IE_Msk             (0x1UL << ADC_IER_AWD2IE_Pos)           /*!< 0x00000100 */
#define ADC_IER_AWD2IE                 ADC_IER_AWD2IE_Msk                      /*!< ADC analog watchdog 2 interrupt */
#define ADC_IER_AWD3IE_Pos             (9UL)
#define ADC_IER_AWD3IE_Msk             (0x1UL << ADC_IER_AWD3IE_Pos)           /*!< 0x00000200 */
#define ADC_IER_AWD3IE                 ADC_IER_AWD3IE_Msk                      /*!< ADC analog watchdog 3 interrupt */
#define ADC_IER_EOCALIE_Pos            (11UL)
#define ADC_IER_EOCALIE_Msk            (0x1UL << ADC_IER_EOCALIE_Pos)          /*!< 0x00000800 */
#define ADC_IER_EOCALIE                ADC_IER_EOCALIE_Msk                     /*!< ADC end of calibration interrupt */
#define ADC_IER_CCRDYIE_Pos            (13UL)
#define ADC_IER_CCRDYIE_Msk            (0x1UL << ADC_IER_CCRDYIE_Pos)          /*!< 0x00002000 */
#define ADC_IER_CCRDYIE                ADC_IER_CCRDYIE_Msk                     /*!< ADC channel configuration ready interrupt */

/* Legacy defines */
#define ADC_IER_EOSEQIE           (ADC_IER_EOSIE)

/********************  Bit definition for ADC_CR register  ********************/
#define ADC_CR_ADEN_Pos                (0UL)
#define ADC_CR_ADEN_Msk                (0x1UL << ADC_CR_ADEN_Pos)              /*!< 0x00000001 */
#define ADC_CR_ADEN                    ADC_CR_ADEN_Msk                         /*!< ADC enable */
#define ADC_CR_ADDIS_Pos               (1UL)
#define ADC_CR_ADDIS_Msk               (0x1UL << ADC_CR_ADDIS_Pos)             /*!< 0x00000002 */
#define ADC_CR_ADDIS                   ADC_CR_ADDIS_Msk                        /*!< ADC disable */
#define ADC_CR_ADSTART_Pos             (2UL)
#define ADC_CR_ADSTART_Msk             (0x1UL << ADC_CR_ADSTART_Pos)           /*!< 0x00000004 */
#define ADC_CR_ADSTART                 ADC_CR_ADSTART_Msk                      /*!< ADC group regular conversion start */
#define ADC_CR_ADSTP_Pos               (4UL)
#define ADC_CR_ADSTP_Msk               (0x1UL << ADC_CR_ADSTP_Pos)             /*!< 0x00000010 */
#define ADC_CR_ADSTP                   ADC_CR_ADSTP_Msk                        /*!< ADC group regular conversion stop */
#define ADC_CR_ADVREGEN_Pos            (28UL)
#define ADC_CR_ADVREGEN_Msk            (0x1UL << ADC_CR_ADVREGEN_Pos)          /*!< 0x10000000 */
#define ADC_CR_ADVREGEN                ADC_CR_ADVREGEN_Msk                     /*!< ADC voltage regulator enable */
#define ADC_CR_ADCAL_Pos               (31UL)
#define ADC_CR_ADCAL_Msk               (0x1UL << ADC_CR_ADCAL_Pos)             /*!< 0x80000000 */
#define ADC_CR_ADCAL                   ADC_CR_ADCAL_Msk                        /*!< ADC calibration */

/********************  Bit definition for ADC_CFGR1 register  *****************/
#define ADC_CFGR1_DMAEN_Pos            (0UL)
#define ADC_CFGR1_DMAEN_Msk            (0x1UL << ADC_CFGR1_DMAEN_Pos)          /*!< 0x00000001 */
#define ADC_CFGR1_DMAEN                ADC_CFGR1_DMAEN_Msk                     /*!< ADC DMA transfer enable */
#define ADC_CFGR1_DMACFG_Pos           (1UL)
#define ADC_CFGR1_DMACFG_Msk           (0x1UL << ADC_CFGR1_DMACFG_Pos)         /*!< 0x00000002 */
#define ADC_CFGR1_DMACFG               ADC_CFGR1_DMACFG_Msk                    /*!< ADC DMA transfer configuration */

#define ADC_CFGR1_SCANDIR_Pos          (2UL)
#define ADC_CFGR1_SCANDIR_Msk          (0x1UL << ADC_CFGR1_SCANDIR_Pos)        /*!< 0x00000004 */
#define ADC_CFGR1_SCANDIR              ADC_CFGR1_SCANDIR_Msk                   /*!< ADC group regular sequencer scan direction */

#define ADC_CFGR1_RES_Pos              (3UL)
#define ADC_CFGR1_RES_Msk              (0x3UL << ADC_CFGR1_RES_Pos)            /*!< 0x00000018 */
#define ADC_CFGR1_RES                  ADC_CFGR1_RES_Msk                       /*!< ADC data resolution */
#define ADC_CFGR1_RES_0                (0x1U << ADC_CFGR1_RES_Pos)             /*!< 0x00000008 */
#define ADC_CFGR1_RES_1                (0x2U << ADC_CFGR1_RES_Pos)             /*!< 0x00000010 */

#define ADC_CFGR1_ALIGN_Pos            (5UL)
#define ADC_CFGR1_ALIGN_Msk            (0x1UL << ADC_CFGR1_ALIGN_Pos)          /*!< 0x00000020 */
#define ADC_CFGR1_ALIGN                ADC_CFGR1_ALIGN_Msk                     /*!< ADC data alignment */

#define ADC_CFGR1_EXTSEL_Pos           (6UL)
#define ADC_CFGR1_EXTSEL_Msk           (0x7UL << ADC_CFGR1_EXTSEL_Pos)         /*!< 0x000001C0 */
#define ADC_CFGR1_EXTSEL               ADC_CFGR1_EXTSEL_Msk                    /*!< ADC group regular external trigger source */
#define ADC_CFGR1_EXTSEL_0             (0x1UL << ADC_CFGR1_EXTSEL_Pos)         /*!< 0x00000040 */
#define ADC_CFGR1_EXTSEL_1             (0x2UL << ADC_CFGR1_EXTSEL_Pos)         /*!< 0x00000080 */
#define ADC_CFGR1_EXTSEL_2             (0x4UL << ADC_CFGR1_EXTSEL_Pos)         /*!< 0x00000100 */

#define ADC_CFGR1_EXTEN_Pos            (10UL)
#define ADC_CFGR1_EXTEN_Msk            (0x3UL << ADC_CFGR1_EXTEN_Pos)          /*!< 0x00000C00 */
#define ADC_CFGR1_EXTEN                ADC_CFGR1_EXTEN_Msk                     /*!< ADC group regular external trigger polarity */
#define ADC_CFGR1_EXTEN_0              (0x1UL << ADC_CFGR1_EXTEN_Pos)          /*!< 0x00000400 */
#define ADC_CFGR1_EXTEN_1              (0x2UL << ADC_CFGR1_EXTEN_Pos)          /*!< 0x00000800 */

#define ADC_CFGR1_OVRMOD_Pos           (12UL)
#define ADC_CFGR1_OVRMOD_Msk           (0x1UL << ADC_CFGR1_OVRMOD_Pos)         /*!< 0x00001000 */
#define ADC_CFGR1_OVRMOD               ADC_CFGR1_OVRMOD_Msk                    /*!< ADC group regular overrun configuration */
#define ADC_CFGR1_CONT_Pos             (13UL)
#define ADC_CFGR1_CONT_Msk             (0x1UL << ADC_CFGR1_CONT_Pos)           /*!< 0x00002000 */
#define ADC_CFGR1_CONT                 ADC_CFGR1_CONT_Msk                      /*!< ADC group regular continuous conversion mode */
#define ADC_CFGR1_WAIT_Pos             (14UL)
#define ADC_CFGR1_WAIT_Msk             (0x1UL << ADC_CFGR1_WAIT_Pos)           /*!< 0x00004000 */
#define ADC_CFGR1_WAIT                 ADC_CFGR1_WAIT_Msk                      /*!< ADC low power auto wait */
#define ADC_CFGR1_AUTOFF_Pos           (15UL)
#define ADC_CFGR1_AUTOFF_Msk           (0x1UL << ADC_CFGR1_AUTOFF_Pos)         /*!< 0x00008000 */
#define ADC_CFGR1_AUTOFF               ADC_CFGR1_AUTOFF_Msk                    /*!< ADC low power auto power off */
#define ADC_CFGR1_DISCEN_Pos           (16UL)
#define ADC_CFGR1_DISCEN_Msk           (0x1UL << ADC_CFGR1_DISCEN_Pos)         /*!< 0x00010000 */
#define ADC_CFGR1_DISCEN               ADC_CFGR1_DISCEN_Msk                    /*!< ADC group regular sequencer discontinuous mode */
#define ADC_CFGR1_CHSELRMOD_Pos        (21UL)
#define ADC_CFGR1_CHSELRMOD_Msk        (0x1UL << ADC_CFGR1_CHSELRMOD_Pos)      /*!< 0x00200000 */
#define ADC_CFGR1_CHSELRMOD            ADC_CFGR1_CHSELRMOD_Msk                 /*!< ADC group regular sequencer mode */

#define ADC_CFGR1_AWD1SGL_Pos          (22UL)
#define ADC_CFGR1_AWD1SGL_Msk          (0x1UL << ADC_CFGR1_AWD1SGL_Pos)        /*!< 0x00400000 */
#define ADC_CFGR1_AWD1SGL              ADC_CFGR1_AWD1SGL_Msk                   /*!< ADC analog watchdog 1 monitoring a single channel or all channels */
#define ADC_CFGR1_AWD1EN_Pos           (23UL)
#define ADC_CFGR1_AWD1EN_Msk           (0x1UL << ADC_CFGR1_AWD1EN_Pos)         /*!< 0x00800000 */
#define ADC_CFGR1_AWD1EN               ADC_CFGR1_AWD1EN_Msk                    /*!< ADC analog watchdog 1 enable on scope ADC group regular */

#define ADC_CFGR1_AWD1CH_Pos           (26UL)
#define ADC_CFGR1_AWD1CH_Msk           (0x1FUL << ADC_CFGR1_AWD1CH_Pos)        /*!< 0x7C000000 */
#define ADC_CFGR1_AWD1CH               ADC_CFGR1_AWD1CH_Msk                    /*!< ADC analog watchdog 1 monitored channel selection */
#define ADC_CFGR1_AWD1CH_0             (0x01UL << ADC_CFGR1_AWD1CH_Pos)        /*!< 0x04000000 */
#define ADC_CFGR1_AWD1CH_1             (0x02UL << ADC_CFGR1_AWD1CH_Pos)        /*!< 0x08000000 */
#define ADC_CFGR1_AWD1CH_2             (0x04UL << ADC_CFGR1_AWD1CH_Pos)        /*!< 0x10000000 */
#define ADC_CFGR1_AWD1CH_3             (0x08UL << ADC_CFGR1_AWD1CH_Pos)        /*!< 0x20000000 */
#define ADC_CFGR1_AWD1CH_4             (0x10UL << ADC_CFGR1_AWD1CH_Pos)        /*!< 0x40000000 */

/* Legacy defines */
#define ADC_CFGR1_AUTDLY          (ADC_CFGR1_WAIT)

/********************  Bit definition for ADC_CFGR2 register  *****************/
#define ADC_CFGR2_OVSE_Pos             (0UL)
#define ADC_CFGR2_OVSE_Msk             (0x1UL << ADC_CFGR2_OVSE_Pos)           /*!< 0x00000001 */
#define ADC_CFGR2_OVSE                 ADC_CFGR2_OVSE_Msk                      /*!< ADC oversampler enable on scope ADC group regular */

#define ADC_CFGR2_OVSR_Pos             (2UL)
#define ADC_CFGR2_OVSR_Msk             (0x7UL << ADC_CFGR2_OVSR_Pos)           /*!< 0x0000001C */
#define ADC_CFGR2_OVSR                 ADC_CFGR2_OVSR_Msk                      /*!< ADC oversampling ratio */
#define ADC_CFGR2_OVSR_0               (0x1UL << ADC_CFGR2_OVSR_Pos)           /*!< 0x00000004 */
#define ADC_CFGR2_OVSR_1               (0x2UL << ADC_CFGR2_OVSR_Pos)           /*!< 0x00000008 */
#define ADC_CFGR2_OVSR_2               (0x4UL << ADC_CFGR2_OVSR_Pos)           /*!< 0x00000010 */

#define ADC_CFGR2_OVSS_Pos             (5UL)
#define ADC_CFGR2_OVSS_Msk             (0xFUL << ADC_CFGR2_OVSS_Pos)           /*!< 0x000001E0 */
#define ADC_CFGR2_OVSS                 ADC_CFGR2_OVSS_Msk                      /*!< ADC oversampling shift */
#define ADC_CFGR2_OVSS_0               (0x1UL << ADC_CFGR2_OVSS_Pos)           /*!< 0x00000020 */
#define ADC_CFGR2_OVSS_1               (0x2UL << ADC_CFGR2_OVSS_Pos)           /*!< 0x00000040 */
#define ADC_CFGR2_OVSS_2               (0x4UL << ADC_CFGR2_OVSS_Pos)           /*!< 0x00000080 */
#define ADC_CFGR2_OVSS_3               (0x8UL << ADC_CFGR2_OVSS_Pos)           /*!< 0x00000100 */

#define ADC_CFGR2_TOVS_Pos             (9UL)
#define ADC_CFGR2_TOVS_Msk             (0x1UL << ADC_CFGR2_TOVS_Pos)           /*!< 0x00000200 */
#define ADC_CFGR2_TOVS                 ADC_CFGR2_TOVS_Msk                      /*!< ADC oversampling discontinuous mode (triggered mode) for ADC group regular */

#define ADC_CFGR2_LFTRIG_Pos           (29UL)
#define ADC_CFGR2_LFTRIG_Msk           (0x1UL << ADC_CFGR2_LFTRIG_Pos)         /*!< 0x20000000 */
#define ADC_CFGR2_LFTRIG               ADC_CFGR2_LFTRIG_Msk                    /*!< ADC low frequency trigger mode */

#define ADC_CFGR2_CKMODE_Pos           (30UL)
#define ADC_CFGR2_CKMODE_Msk           (0x3UL << ADC_CFGR2_CKMODE_Pos)         /*!< 0xC0000000 */
#define ADC_CFGR2_CKMODE               ADC_CFGR2_CKMODE_Msk                    /*!< ADC clock source and prescaler (prescaler only for clock source synchronous) */
#define ADC_CFGR2_CKMODE_1             (0x2UL << ADC_CFGR2_CKMODE_Pos)         /*!< 0x80000000 */
#define ADC_CFGR2_CKMODE_0             (0x1UL << ADC_CFGR2_CKMODE_Pos)         /*!< 0x40000000 */

/********************  Bit definition for ADC_SMPR register  ******************/
#define ADC_SMPR_SMP1_Pos              (0UL)
#define ADC_SMPR_SMP1_Msk              (0x7UL << ADC_SMPR_SMP1_Pos)            /*!< 0x00000007 */
#define ADC_SMPR_SMP1                  ADC_SMPR_SMP1_Msk                       /*!< ADC group of channels sampling time 1 */
#define ADC_SMPR_SMP1_0                (0x1UL << ADC_SMPR_SMP1_Pos)            /*!< 0x00000001 */
#define ADC_SMPR_SMP1_1                (0x2UL << ADC_SMPR_SMP1_Pos)            /*!< 0x00000002 */
#define ADC_SMPR_SMP1_2                (0x4UL << ADC_SMPR_SMP1_Pos)            /*!< 0x00000004 */

#define ADC_SMPR_SMP2_Pos              (4UL)
#define ADC_SMPR_SMP2_Msk              (0x7UL << ADC_SMPR_SMP2_Pos)            /*!< 0x00000070 */
#define ADC_SMPR_SMP2                  ADC_SMPR_SMP2_Msk                       /*!< ADC group of channels sampling time 2 */
#define ADC_SMPR_SMP2_0                (0x1UL << ADC_SMPR_SMP2_Pos)            /*!< 0x00000010 */
#define ADC_SMPR_SMP2_1                (0x2UL << ADC_SMPR_SMP2_Pos)            /*!< 0x00000020 */
#define ADC_SMPR_SMP2_2                (0x4UL << ADC_SMPR_SMP2_Pos)            /*!< 0x00000040 */

#define ADC_SMPR_SMPSEL_Pos            (8UL)
#define ADC_SMPR_SMPSEL_Msk            (0x7FFFFUL << ADC_SMPR_SMPSEL_Pos)      /*!< 0x07FFFF00 */
#define ADC_SMPR_SMPSEL                ADC_SMPR_SMPSEL_Msk                     /*!< ADC all channels sampling time selection */
#define ADC_SMPR_SMPSEL0_Pos           (8UL)
#define ADC_SMPR_SMPSEL0_Msk           (0x1UL << ADC_SMPR_SMPSEL0_Pos)         /*!< 0x00000100 */
#define ADC_SMPR_SMPSEL0               ADC_SMPR_SMPSEL0_Msk                    /*!< ADC channel 0 sampling time selection */
#define ADC_SMPR_SMPSEL1_Pos           (9UL)
#define ADC_SMPR_SMPSEL1_Msk           (0x1UL << ADC_SMPR_SMPSEL1_Pos)         /*!< 0x00000200 */
#define ADC_SMPR_SMPSEL1               ADC_SMPR_SMPSEL1_Msk                    /*!< ADC channel 1 sampling time selection */
#define ADC_SMPR_SMPSEL2_Pos           (10UL)
#define ADC_SMPR_SMPSEL2_Msk           (0x1UL << ADC_SMPR_SMPSEL2_Pos)         /*!< 0x00000400 */
#define ADC_SMPR_SMPSEL2               ADC_SMPR_SMPSEL2_Msk                    /*!< ADC channel 2 sampling time selection */
#define ADC_SMPR_SMPSEL3_Pos           (11UL)
#define ADC_SMPR_SMPSEL3_Msk           (0x1UL << ADC_SMPR_SMPSEL3_Pos)         /*!< 0x00000800 */
#define ADC_SMPR_SMPSEL3               ADC_SMPR_SMPSEL3_Msk                    /*!< ADC channel 3 sampling time selection */
#define ADC_SMPR_SMPSEL4_Pos           (12UL)
#define ADC_SMPR_SMPSEL4_Msk           (0x1UL << ADC_SMPR_SMPSEL4_Pos)         /*!< 0x00001000 */
#define ADC_SMPR_SMPSEL4               ADC_SMPR_SMPSEL4_Msk                    /*!< ADC channel 4 sampling time selection */
#define ADC_SMPR_SMPSEL5_Pos           (13UL)
#define ADC_SMPR_SMPSEL5_Msk           (0x1UL << ADC_SMPR_SMPSEL5_Pos)         /*!< 0x00002000 */
#define ADC_SMPR_SMPSEL5               ADC_SMPR_SMPSEL5_Msk                    /*!< ADC channel 5 sampling time selection */
#define ADC_SMPR_SMPSEL6_Pos           (14UL)
#define ADC_SMPR_SMPSEL6_Msk           (0x1UL << ADC_SMPR_SMPSEL6_Pos)         /*!< 0x00004000 */
#define ADC_SMPR_SMPSEL6               ADC_SMPR_SMPSEL6_Msk                    /*!< ADC channel 6 sampling time selection */
#define ADC_SMPR_SMPSEL7_Pos           (15UL)
#define ADC_SMPR_SMPSEL7_Msk           (0x1UL << ADC_SMPR_SMPSEL7_Pos)         /*!< 0x00008000 */
#define ADC_SMPR_SMPSEL7               ADC_SMPR_SMPSEL7_Msk                    /*!< ADC channel 7 sampling time selection */
#define ADC_SMPR_SMPSEL8_Pos           (16UL)
#define ADC_SMPR_SMPSEL8_Msk           (0x1UL << ADC_SMPR_SMPSEL8_Pos)         /*!< 0x00010000 */
#define ADC_SMPR_SMPSEL8               ADC_SMPR_SMPSEL8_Msk                    /*!< ADC channel 8 sampling time selection */
#define ADC_SMPR_SMPSEL9_Pos           (17UL)
#define ADC_SMPR_SMPSEL9_Msk           (0x1UL << ADC_SMPR_SMPSEL9_Pos)         /*!< 0x00020000 */
#define ADC_SMPR_SMPSEL9               ADC_SMPR_SMPSEL9_Msk                    /*!< ADC channel 9 sampling time selection */
#define ADC_SMPR_SMPSEL10_Pos          (18UL)
#define ADC_SMPR_SMPSEL10_Msk          (0x1UL << ADC_SMPR_SMPSEL10_Pos)        /*!< 0x00040000 */
#define ADC_SMPR_SMPSEL10              ADC_SMPR_SMPSEL10_Msk                   /*!< ADC channel 10 sampling time selection */
#define ADC_SMPR_SMPSEL11_Pos          (19UL)
#define ADC_SMPR_SMPSEL11_Msk          (0x1UL << ADC_SMPR_SMPSEL11_Pos)        /*!< 0x00080000 */
#define ADC_SMPR_SMPSEL11              ADC_SMPR_SMPSEL11_Msk                   /*!< ADC channel 11 sampling time selection */
#define ADC_SMPR_SMPSEL12_Pos          (20UL)
#define ADC_SMPR_SMPSEL12_Msk          (0x1UL << ADC_SMPR_SMPSEL12_Pos)        /*!< 0x00100000 */
#define ADC_SMPR_SMPSEL12              ADC_SMPR_SMPSEL12_Msk                   /*!< ADC channel 12 sampling time selection */
#define ADC_SMPR_SMPSEL13_Pos          (21UL)
#define ADC_SMPR_SMPSEL13_Msk          (0x1UL << ADC_SMPR_SMPSEL13_Pos)        /*!< 0x00200000 */
#define ADC_SMPR_SMPSEL13              ADC_SMPR_SMPSEL13_Msk                   /*!< ADC channel 13 sampling time selection */
#define ADC_SMPR_SMPSEL14_Pos          (22UL)
#define ADC_SMPR_SMPSEL14_Msk          (0x1UL << ADC_SMPR_SMPSEL14_Pos)        /*!< 0x00400000 */
#define ADC_SMPR_SMPSEL14              ADC_SMPR_SMPSEL14_Msk                   /*!< ADC channel 14 sampling time selection */
#define ADC_SMPR_SMPSEL15_Pos          (23UL)
#define ADC_SMPR_SMPSEL15_Msk          (0x1UL << ADC_SMPR_SMPSEL15_Pos)        /*!< 0x00800000 */
#define ADC_SMPR_SMPSEL15              ADC_SMPR_SMPSEL15_Msk                   /*!< ADC channel 15 sampling time selection */
#define ADC_SMPR_SMPSEL16_Pos          (24UL)
#define ADC_SMPR_SMPSEL16_Msk          (0x1UL << ADC_SMPR_SMPSEL16_Pos)        /*!< 0x01000000 */
#define ADC_SMPR_SMPSEL16              ADC_SMPR_SMPSEL16_Msk                   /*!< ADC channel 16 sampling time selection */
#define ADC_SMPR_SMPSEL17_Pos          (25UL)
#define ADC_SMPR_SMPSEL17_Msk          (0x1UL << ADC_SMPR_SMPSEL17_Pos)        /*!< 0x02000000 */
#define ADC_SMPR_SMPSEL17              ADC_SMPR_SMPSEL17_Msk                   /*!< ADC channel 17 sampling time selection */
#define ADC_SMPR_SMPSEL18_Pos          (26UL)
#define ADC_SMPR_SMPSEL18_Msk          (0x1UL << ADC_SMPR_SMPSEL18_Pos)        /*!< 0x04000000 */
#define ADC_SMPR_SMPSEL18              ADC_SMPR_SMPSEL18_Msk                   /*!< ADC channel 18 sampling time selection */
#define ADC_SMPR_SMPSEL19_Pos          (27UL)
#define ADC_SMPR_SMPSEL19_Msk          (0x1UL << ADC_SMPR_SMPSEL19_Pos)        /*!< 0x08000000 */
#define ADC_SMPR_SMPSEL19              ADC_SMPR_SMPSEL19_Msk                   /*!< ADC channel 19 sampling time selection */

/********************  Bit definition for ADC_AWD1TR register  *******************/
#define ADC_AWD1TR_LT1_Pos             (0UL)
#define ADC_AWD1TR_LT1_Msk             (0xFFFUL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000FFF */
#define ADC_AWD1TR_LT1                 ADC_AWD1TR_LT1_Msk                      /*!< ADC analog watchdog 1 threshold low */
#define ADC_AWD1TR_LT1_0               (0x001UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000001 */
#define ADC_AWD1TR_LT1_1               (0x002UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000002 */
#define ADC_AWD1TR_LT1_2               (0x004UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000004 */
#define ADC_AWD1TR_LT1_3               (0x008UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000008 */
#define ADC_AWD1TR_LT1_4               (0x010UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000010 */
#define ADC_AWD1TR_LT1_5               (0x020UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000020 */
#define ADC_AWD1TR_LT1_6               (0x040UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000040 */
#define ADC_AWD1TR_LT1_7               (0x080UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000080 */
#define ADC_AWD1TR_LT1_8               (0x100UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000100 */
#define ADC_AWD1TR_LT1_9               (0x200UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000200 */
#define ADC_AWD1TR_LT1_10              (0x400UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000400 */
#define ADC_AWD1TR_LT1_11              (0x800UL << ADC_AWD1TR_LT1_Pos)         /*!< 0x00000800 */

#define ADC_AWD1TR_HT1_Pos             (16UL)
#define ADC_AWD1TR_HT1_Msk             (0xFFFUL << ADC_AWD1TR_HT1_Pos)         /*!< 0x0FFF0000 */
#define ADC_AWD1TR_HT1                 ADC_AWD1TR_HT1_Msk                      /*!< ADC Analog watchdog 1 threshold high */
#define ADC_AWD1TR_HT1_0               (0x001UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x00010000 */
#define ADC_AWD1TR_HT1_1               (0x002UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x00020000 */
#define ADC_AWD1TR_HT1_2               (0x004UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x00040000 */
#define ADC_AWD1TR_HT1_3               (0x008UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x00080000 */
#define ADC_AWD1TR_HT1_4               (0x010UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x00100000 */
#define ADC_AWD1TR_HT1_5               (0x020UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x00200000 */
#define ADC_AWD1TR_HT1_6               (0x040UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x00400000 */
#define ADC_AWD1TR_HT1_7               (0x080UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x00800000 */
#define ADC_AWD1TR_HT1_8               (0x100UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x01000000 */
#define ADC_AWD1TR_HT1_9               (0x200UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x02000000 */
#define ADC_AWD1TR_HT1_10              (0x400UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x04000000 */
#define ADC_AWD1TR_HT1_11              (0x800UL << ADC_AWD1TR_HT1_Pos)         /*!< 0x08000000 */

/********************  Bit definition for ADC_AWD2TR register  *******************/
#define ADC_AWD2TR_LT2_Pos             (0UL)
#define ADC_AWD2TR_LT2_Msk             (0xFFFUL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000FFF */
#define ADC_AWD2TR_LT2                 ADC_AWD2TR_LT2_Msk                      /*!< ADC analog watchdog 2 threshold low */
#define ADC_AWD2TR_LT2_0               (0x001UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000001 */
#define ADC_AWD2TR_LT2_1               (0x002UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000002 */
#define ADC_AWD2TR_LT2_2               (0x004UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000004 */
#define ADC_AWD2TR_LT2_3               (0x008UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000008 */
#define ADC_AWD2TR_LT2_4               (0x010UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000010 */
#define ADC_AWD2TR_LT2_5               (0x020UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000020 */
#define ADC_AWD2TR_LT2_6               (0x040UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000040 */
#define ADC_AWD2TR_LT2_7               (0x080UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000080 */
#define ADC_AWD2TR_LT2_8               (0x100UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000100 */
#define ADC_AWD2TR_LT2_9               (0x200UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000200 */
#define ADC_AWD2TR_LT2_10              (0x400UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000400 */
#define ADC_AWD2TR_LT2_11              (0x800UL << ADC_AWD2TR_LT2_Pos)         /*!< 0x00000800 */

#define ADC_AWD2TR_HT2_Pos             (16UL)
#define ADC_AWD2TR_HT2_Msk             (0xFFFUL << ADC_AWD2TR_HT2_Pos)         /*!< 0x0FFF0000 */
#define ADC_AWD2TR_HT2                 ADC_AWD2TR_HT2_Msk                      /*!< ADC analog watchdog 2 threshold high */
#define ADC_AWD2TR_HT2_0               (0x001UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x00010000 */
#define ADC_AWD2TR_HT2_1               (0x002UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x00020000 */
#define ADC_AWD2TR_HT2_2               (0x004UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x00040000 */
#define ADC_AWD2TR_HT2_3               (0x008UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x00080000 */
#define ADC_AWD2TR_HT2_4               (0x010UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x00100000 */
#define ADC_AWD2TR_HT2_5               (0x020UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x00200000 */
#define ADC_AWD2TR_HT2_6               (0x040UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x00400000 */
#define ADC_AWD2TR_HT2_7               (0x080UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x00800000 */
#define ADC_AWD2TR_HT2_8               (0x100UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x01000000 */
#define ADC_AWD2TR_HT2_9               (0x200UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x02000000 */
#define ADC_AWD2TR_HT2_10              (0x400UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x04000000 */
#define ADC_AWD2TR_HT2_11              (0x800UL << ADC_AWD2TR_HT2_Pos)         /*!< 0x08000000 */

/********************  Bit definition for ADC_CHSELR register  ****************/
#define ADC_CHSELR_CHSEL_Pos           (0UL)
#define ADC_CHSELR_CHSEL_Msk           (0x7FFFFFUL << ADC_CHSELR_CHSEL_Pos)    /*!< 0x0007FFFFF */
#define ADC_CHSELR_CHSEL               ADC_CHSELR_CHSEL_Msk                    /*!< ADC group regular sequencer channels, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL22_Pos         (22UL)
#define ADC_CHSELR_CHSEL22_Msk         (0x1UL << ADC_CHSELR_CHSEL22_Pos)       /*!< 0x00400000 */
#define ADC_CHSELR_CHSEL22             ADC_CHSELR_CHSEL22_Msk                  /*!< ADC group regular sequencer channel 22, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL21_Pos         (21UL)
#define ADC_CHSELR_CHSEL21_Msk         (0x1UL << ADC_CHSELR_CHSEL21_Pos)       /*!< 0x00200000 */
#define ADC_CHSELR_CHSEL21             ADC_CHSELR_CHSEL21_Msk                  /*!< ADC group regular sequencer channel 21, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL20_Pos         (20UL)
#define ADC_CHSELR_CHSEL20_Msk         (0x1UL << ADC_CHSELR_CHSEL20_Pos)       /*!< 0x00100000 */
#define ADC_CHSELR_CHSEL20             ADC_CHSELR_CHSEL20_Msk                  /*!< ADC group regular sequencer channel 20, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL19_Pos         (19UL)
#define ADC_CHSELR_CHSEL19_Msk         (0x1UL << ADC_CHSELR_CHSEL19_Pos)       /*!< 0x00080000 */
#define ADC_CHSELR_CHSEL19             ADC_CHSELR_CHSEL19_Msk                  /*!< ADC group regular sequencer channel 19, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL18_Pos         (18UL)
#define ADC_CHSELR_CHSEL18_Msk         (0x1UL << ADC_CHSELR_CHSEL18_Pos)       /*!< 0x00040000 */
#define ADC_CHSELR_CHSEL18             ADC_CHSELR_CHSEL18_Msk                  /*!< ADC group regular sequencer channel 18, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL17_Pos         (17UL)
#define ADC_CHSELR_CHSEL17_Msk         (0x1UL << ADC_CHSELR_CHSEL17_Pos)       /*!< 0x00020000 */
#define ADC_CHSELR_CHSEL17             ADC_CHSELR_CHSEL17_Msk                  /*!< ADC group regular sequencer channel 17, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL16_Pos         (16UL)
#define ADC_CHSELR_CHSEL16_Msk         (0x1UL << ADC_CHSELR_CHSEL16_Pos)       /*!< 0x00010000 */
#define ADC_CHSELR_CHSEL16             ADC_CHSELR_CHSEL16_Msk                  /*!< ADC group regular sequencer channel 16, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL15_Pos         (15UL)
#define ADC_CHSELR_CHSEL15_Msk         (0x1UL << ADC_CHSELR_CHSEL15_Pos)       /*!< 0x00008000 */
#define ADC_CHSELR_CHSEL15             ADC_CHSELR_CHSEL15_Msk                  /*!< ADC group regular sequencer channel 15, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL14_Pos         (14UL)
#define ADC_CHSELR_CHSEL14_Msk         (0x1UL << ADC_CHSELR_CHSEL14_Pos)       /*!< 0x00004000 */
#define ADC_CHSELR_CHSEL14             ADC_CHSELR_CHSEL14_Msk                  /*!< ADC group regular sequencer channel 14, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL13_Pos         (13UL)
#define ADC_CHSELR_CHSEL13_Msk         (0x1UL << ADC_CHSELR_CHSEL13_Pos)       /*!< 0x00002000 */
#define ADC_CHSELR_CHSEL13             ADC_CHSELR_CHSEL13_Msk                  /*!< ADC group regular sequencer channel 13, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL12_Pos         (12UL)
#define ADC_CHSELR_CHSEL12_Msk         (0x1UL << ADC_CHSELR_CHSEL12_Pos)       /*!< 0x00001000 */
#define ADC_CHSELR_CHSEL12             ADC_CHSELR_CHSEL12_Msk                  /*!< ADC group regular sequencer channel 12, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL11_Pos         (11UL)
#define ADC_CHSELR_CHSEL11_Msk         (0x1UL << ADC_CHSELR_CHSEL11_Pos)       /*!< 0x00000800 */
#define ADC_CHSELR_CHSEL11             ADC_CHSELR_CHSEL11_Msk                  /*!< ADC group regular sequencer channel 11, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL10_Pos         (10UL)
#define ADC_CHSELR_CHSEL10_Msk         (0x1UL << ADC_CHSELR_CHSEL10_Pos)       /*!< 0x00000400 */
#define ADC_CHSELR_CHSEL10             ADC_CHSELR_CHSEL10_Msk                  /*!< ADC group regular sequencer channel 10, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL9_Pos          (9UL)
#define ADC_CHSELR_CHSEL9_Msk          (0x1UL << ADC_CHSELR_CHSEL9_Pos)        /*!< 0x00000200 */
#define ADC_CHSELR_CHSEL9              ADC_CHSELR_CHSEL9_Msk                   /*!< ADC group regular sequencer channel 9, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL8_Pos          (8UL)
#define ADC_CHSELR_CHSEL8_Msk          (0x1UL << ADC_CHSELR_CHSEL8_Pos)        /*!< 0x00000100 */
#define ADC_CHSELR_CHSEL8              ADC_CHSELR_CHSEL8_Msk                   /*!< ADC group regular sequencer channel 8, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL7_Pos          (7UL)
#define ADC_CHSELR_CHSEL7_Msk          (0x1UL << ADC_CHSELR_CHSEL7_Pos)        /*!< 0x00000080 */
#define ADC_CHSELR_CHSEL7              ADC_CHSELR_CHSEL7_Msk                   /*!< ADC group regular sequencer channel 7, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL6_Pos          (6UL)
#define ADC_CHSELR_CHSEL6_Msk          (0x1UL << ADC_CHSELR_CHSEL6_Pos)        /*!< 0x00000040 */
#define ADC_CHSELR_CHSEL6              ADC_CHSELR_CHSEL6_Msk                   /*!< ADC group regular sequencer channel 6, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL5_Pos          (5UL)
#define ADC_CHSELR_CHSEL5_Msk          (0x1UL << ADC_CHSELR_CHSEL5_Pos)        /*!< 0x00000020 */
#define ADC_CHSELR_CHSEL5              ADC_CHSELR_CHSEL5_Msk                   /*!< ADC group regular sequencer channel 5, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL4_Pos          (4UL)
#define ADC_CHSELR_CHSEL4_Msk          (0x1UL << ADC_CHSELR_CHSEL4_Pos)        /*!< 0x00000010 */
#define ADC_CHSELR_CHSEL4              ADC_CHSELR_CHSEL4_Msk                   /*!< ADC group regular sequencer channel 4, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL3_Pos          (3UL)
#define ADC_CHSELR_CHSEL3_Msk          (0x1UL << ADC_CHSELR_CHSEL3_Pos)        /*!< 0x00000008 */
#define ADC_CHSELR_CHSEL3              ADC_CHSELR_CHSEL3_Msk                   /*!< ADC group regular sequencer channel 3, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL2_Pos          (2UL)
#define ADC_CHSELR_CHSEL2_Msk          (0x1UL << ADC_CHSELR_CHSEL2_Pos)        /*!< 0x00000004 */
#define ADC_CHSELR_CHSEL2              ADC_CHSELR_CHSEL2_Msk                   /*!< ADC group regular sequencer channel 2, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL1_Pos          (1UL)
#define ADC_CHSELR_CHSEL1_Msk          (0x1UL << ADC_CHSELR_CHSEL1_Pos)        /*!< 0x00000002 */
#define ADC_CHSELR_CHSEL1              ADC_CHSELR_CHSEL1_Msk                   /*!< ADC group regular sequencer channel 1, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL0_Pos          (0UL)
#define ADC_CHSELR_CHSEL0_Msk          (0x1UL << ADC_CHSELR_CHSEL0_Pos)        /*!< 0x00000001 */
#define ADC_CHSELR_CHSEL0              ADC_CHSELR_CHSEL0_Msk                   /*!< ADC group regular sequencer channel 0, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_SQ_ALL_Pos          (0UL)
#define ADC_CHSELR_SQ_ALL_Msk          (0xFFFFFFFFUL << ADC_CHSELR_SQ_ALL_Pos) /*!< 0xFFFFFFFF */
#define ADC_CHSELR_SQ_ALL              ADC_CHSELR_SQ_ALL_Msk                   /*!< ADC group regular sequencer all ranks, available when ADC_CFGR1_CHSELRMOD is set */

#define ADC_CHSELR_SQ8_Pos             (28UL)
#define ADC_CHSELR_SQ8_Msk             (0xFUL << ADC_CHSELR_SQ8_Pos)           /*!< 0xF0000000 */
#define ADC_CHSELR_SQ8                 ADC_CHSELR_SQ8_Msk                      /*!< ADC group regular sequencer rank 8, available when ADC_CFGR1_CHSELRMOD is set */
#define ADC_CHSELR_SQ8_0               (0x1UL << ADC_CHSELR_SQ8_Pos)           /*!< 0x10000000 */
#define ADC_CHSELR_SQ8_1               (0x2UL << ADC_CHSELR_SQ8_Pos)           /*!< 0x20000000 */
#define ADC_CHSELR_SQ8_2               (0x4UL << ADC_CHSELR_SQ8_Pos)           /*!< 0x40000000 */
#define ADC_CHSELR_SQ8_3               (0x8UL << ADC_CHSELR_SQ8_Pos)           /*!< 0x80000000 */

#define ADC_CHSELR_SQ7_Pos             (24UL)
#define ADC_CHSELR_SQ7_Msk             (0xFUL << ADC_CHSELR_SQ7_Pos)           /*!< 0x0F000000 */
#define ADC_CHSELR_SQ7                 ADC_CHSELR_SQ7_Msk                      /*!< ADC group regular sequencer rank 7, available when ADC_CFGR1_CHSELRMOD is set */
#define ADC_CHSELR_SQ7_0               (0x1UL << ADC_CHSELR_SQ7_Pos)           /*!< 0x01000000 */
#define ADC_CHSELR_SQ7_1               (0x2UL << ADC_CHSELR_SQ7_Pos)           /*!< 0x02000000 */
#define ADC_CHSELR_SQ7_2               (0x4UL << ADC_CHSELR_SQ7_Pos)           /*!< 0x04000000 */
#define ADC_CHSELR_SQ7_3               (0x8UL << ADC_CHSELR_SQ7_Pos)           /*!< 0x08000000 */

#define ADC_CHSELR_SQ6_Pos             (20UL)
#define ADC_CHSELR_SQ6_Msk             (0xFUL << ADC_CHSELR_SQ6_Pos)           /*!< 0x00F00000 */
#define ADC_CHSELR_SQ6                 ADC_CHSELR_SQ6_Msk                      /*!< ADC group regular sequencer rank 6, available when ADC_CFGR1_CHSELRMOD is set */
#define ADC_CHSELR_SQ6_0               (0x1UL << ADC_CHSELR_SQ6_Pos)           /*!< 0x00100000 */
#define ADC_CHSELR_SQ6_1               (0x2UL << ADC_CHSELR_SQ6_Pos)           /*!< 0x00200000 */
#define ADC_CHSELR_SQ6_2               (0x4UL << ADC_CHSELR_SQ6_Pos)           /*!< 0x00400000 */
#define ADC_CHSELR_SQ6_3               (0x8UL << ADC_CHSELR_SQ6_Pos)           /*!< 0x00800000 */

#define ADC_CHSELR_SQ5_Pos             (16UL)
#define ADC_CHSELR_SQ5_Msk             (0xFUL << ADC_CHSELR_SQ5_Pos)           /*!< 0x000F0000 */
#define ADC_CHSELR_SQ5                 ADC_CHSELR_SQ5_Msk                      /*!< ADC group regular sequencer rank 5, available when ADC_CFGR1_CHSELRMOD is set */
#define ADC_CHSELR_SQ5_0               (0x1UL << ADC_CHSELR_SQ5_Pos)           /*!< 0x00010000 */
#define ADC_CHSELR_SQ5_1               (0x2UL << ADC_CHSELR_SQ5_Pos)           /*!< 0x00020000 */
#define ADC_CHSELR_SQ5_2               (0x4UL << ADC_CHSELR_SQ5_Pos)           /*!< 0x00040000 */
#define ADC_CHSELR_SQ5_3               (0x8UL << ADC_CHSELR_SQ5_Pos)           /*!< 0x00080000 */

#define ADC_CHSELR_SQ4_Pos             (12UL)
#define ADC_CHSELR_SQ4_Msk             (0xFUL << ADC_CHSELR_SQ4_Pos)           /*!< 0x0000F000 */
#define ADC_CHSELR_SQ4                 ADC_CHSELR_SQ4_Msk                      /*!< ADC group regular sequencer rank 4, available when ADC_CFGR1_CHSELRMOD is set */
#define ADC_CHSELR_SQ4_0               (0x1UL << ADC_CHSELR_SQ4_Pos)           /*!< 0x00001000 */
#define ADC_CHSELR_SQ4_1               (0x2UL << ADC_CHSELR_SQ4_Pos)           /*!< 0x00002000 */
#define ADC_CHSELR_SQ4_2               (0x4UL << ADC_CHSELR_SQ4_Pos)           /*!< 0x00004000 */
#define ADC_CHSELR_SQ4_3               (0x8UL << ADC_CHSELR_SQ4_Pos)           /*!< 0x00008000 */

#define ADC_CHSELR_SQ3_Pos             (8UL)
#define ADC_CHSELR_SQ3_Msk             (0xFUL << ADC_CHSELR_SQ3_Pos)           /*!< 0x00000F00 */
#define ADC_CHSELR_SQ3                 ADC_CHSELR_SQ3_Msk                      /*!< ADC group regular sequencer rank 3, available when ADC_CFGR1_CHSELRMOD is set */
#define ADC_CHSELR_SQ3_0               (0x1UL << ADC_CHSELR_SQ3_Pos)           /*!< 0x00000100 */
#define ADC_CHSELR_SQ3_1               (0x2UL << ADC_CHSELR_SQ3_Pos)           /*!< 0x00000200 */
#define ADC_CHSELR_SQ3_2               (0x4UL << ADC_CHSELR_SQ3_Pos)           /*!< 0x00000400 */
#define ADC_CHSELR_SQ3_3               (0x8UL << ADC_CHSELR_SQ3_Pos)           /*!< 0x00000800 */

#define ADC_CHSELR_SQ2_Pos             (4UL)
#define ADC_CHSELR_SQ2_Msk             (0xFUL << ADC_CHSELR_SQ2_Pos)           /*!< 0x000000F0 */
#define ADC_CHSELR_SQ2                 ADC_CHSELR_SQ2_Msk                      /*!< ADC group regular sequencer rank 2, available when ADC_CFGR1_CHSELRMOD is set */
#define ADC_CHSELR_SQ2_0               (0x1UL << ADC_CHSELR_SQ2_Pos)           /*!< 0x00000010 */
#define ADC_CHSELR_SQ2_1               (0x2UL << ADC_CHSELR_SQ2_Pos)           /*!< 0x00000020 */
#define ADC_CHSELR_SQ2_2               (0x4UL << ADC_CHSELR_SQ2_Pos)           /*!< 0x00000040 */
#define ADC_CHSELR_SQ2_3               (0x8UL << ADC_CHSELR_SQ2_Pos)           /*!< 0x00000080 */

#define ADC_CHSELR_SQ1_Pos             (0UL)
#define ADC_CHSELR_SQ1_Msk             (0xFUL << ADC_CHSELR_SQ1_Pos)           /*!< 0x0000000F */
#define ADC_CHSELR_SQ1                 ADC_CHSELR_SQ1_Msk                      /*!< ADC group regular sequencer rank 1, available when ADC_CFGR1_CHSELRMOD is set */
#define ADC_CHSELR_SQ1_0               (0x1UL << ADC_CHSELR_SQ1_Pos)           /*!< 0x00000001 */
#define ADC_CHSELR_SQ1_1               (0x2UL << ADC_CHSELR_SQ1_Pos)           /*!< 0x00000002 */
#define ADC_CHSELR_SQ1_2               (0x4UL << ADC_CHSELR_SQ1_Pos)           /*!< 0x00000004 */
#define ADC_CHSELR_SQ1_3               (0x8UL << ADC_CHSELR_SQ1_Pos)           /*!< 0x00000008 */

/********************  Bit definition for ADC_AWD3TR register  *******************/
#define ADC_AWD3TR_LT3_Pos             (0UL)
#define ADC_AWD3TR_LT3_Msk             (0xFFFUL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000FFF */
#define ADC_AWD3TR_LT3                 ADC_AWD3TR_LT3_Msk                      /*!< ADC analog watchdog 3 threshold low */
#define ADC_AWD3TR_LT3_0               (0x001UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000001 */
#define ADC_AWD3TR_LT3_1               (0x002UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000002 */
#define ADC_AWD3TR_LT3_2               (0x004UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000004 */
#define ADC_AWD3TR_LT3_3               (0x008UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000008 */
#define ADC_AWD3TR_LT3_4               (0x010UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000010 */
#define ADC_AWD3TR_LT3_5               (0x020UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000020 */
#define ADC_AWD3TR_LT3_6               (0x040UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000040 */
#define ADC_AWD3TR_LT3_7               (0x080UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000080 */
#define ADC_AWD3TR_LT3_8               (0x100UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000100 */
#define ADC_AWD3TR_LT3_9               (0x200UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000200 */
#define ADC_AWD3TR_LT3_10              (0x400UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000400 */
#define ADC_AWD3TR_LT3_11              (0x800UL << ADC_AWD3TR_LT3_Pos)         /*!< 0x00000800 */

#define ADC_AWD3TR_HT3_Pos             (16UL)
#define ADC_AWD3TR_HT3_Msk             (0xFFFUL << ADC_AWD3TR_HT3_Pos)         /*!< 0x0FFF0000 */
#define ADC_AWD3TR_HT3                 ADC_AWD3TR_HT3_Msk                      /*!< ADC analog watchdog 3 threshold high */
#define ADC_AWD3TR_HT3_0               (0x001UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x00010000 */
#define ADC_AWD3TR_HT3_1               (0x002UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x00020000 */
#define ADC_AWD3TR_HT3_2               (0x004UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x00040000 */
#define ADC_AWD3TR_HT3_3               (0x008UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x00080000 */
#define ADC_AWD3TR_HT3_4               (0x010UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x00100000 */
#define ADC_AWD3TR_HT3_5               (0x020UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x00200000 */
#define ADC_AWD3TR_HT3_6               (0x040UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x00400000 */
#define ADC_AWD3TR_HT3_7               (0x080UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x00800000 */
#define ADC_AWD3TR_HT3_8               (0x100UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x01000000 */
#define ADC_AWD3TR_HT3_9               (0x200UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x02000000 */
#define ADC_AWD3TR_HT3_10              (0x400UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x04000000 */
#define ADC_AWD3TR_HT3_11              (0x800UL << ADC_AWD3TR_HT3_Pos)         /*!< 0x08000000 */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_DATA_Pos                (0UL)
#define ADC_DR_DATA_Msk                (0xFFFFUL << ADC_DR_DATA_Pos)           /*!< 0x0000FFFF */
#define ADC_DR_DATA                    ADC_DR_DATA_Msk                         /*!< ADC group regular conversion data */
#define ADC_DR_DATA_0                  (0x0001UL << ADC_DR_DATA_Pos)           /*!< 0x00000001 */
#define ADC_DR_DATA_1                  (0x0002UL << ADC_DR_DATA_Pos)           /*!< 0x00000002 */
#define ADC_DR_DATA_2                  (0x0004UL << ADC_DR_DATA_Pos)           /*!< 0x00000004 */
#define ADC_DR_DATA_3                  (0x0008UL << ADC_DR_DATA_Pos)           /*!< 0x00000008 */
#define ADC_DR_DATA_4                  (0x0010UL << ADC_DR_DATA_Pos)           /*!< 0x00000010 */
#define ADC_DR_DATA_5                  (0x0020UL << ADC_DR_DATA_Pos)           /*!< 0x00000020 */
#define ADC_DR_DATA_6                  (0x0040UL << ADC_DR_DATA_Pos)           /*!< 0x00000040 */
#define ADC_DR_DATA_7                  (0x0080UL << ADC_DR_DATA_Pos)           /*!< 0x00000080 */
#define ADC_DR_DATA_8                  (0x0100UL << ADC_DR_DATA_Pos)           /*!< 0x00000100 */
#define ADC_DR_DATA_9                  (0x0200UL << ADC_DR_DATA_Pos)           /*!< 0x00000200 */
#define ADC_DR_DATA_10                 (0x0400UL << ADC_DR_DATA_Pos)           /*!< 0x00000400 */
#define ADC_DR_DATA_11                 (0x0800UL << ADC_DR_DATA_Pos)           /*!< 0x00000800 */
#define ADC_DR_DATA_12                 (0x1000UL << ADC_DR_DATA_Pos)           /*!< 0x00001000 */
#define ADC_DR_DATA_13                 (0x2000UL << ADC_DR_DATA_Pos)           /*!< 0x00002000 */
#define ADC_DR_DATA_14                 (0x4000UL << ADC_DR_DATA_Pos)           /*!< 0x00004000 */
#define ADC_DR_DATA_15                 (0x8000UL << ADC_DR_DATA_Pos)           /*!< 0x00008000 */

/********************  Bit definition for ADC_AWD2CR register  ****************/
#define ADC_AWD2CR_AWD2CH_Pos          (0UL)
#define ADC_AWD2CR_AWD2CH_Msk          (0x7FFFFUL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x0007FFFF */
#define ADC_AWD2CR_AWD2CH              ADC_AWD2CR_AWD2CH_Msk                   /*!< ADC analog watchdog 2 monitored channel selection */
#define ADC_AWD2CR_AWD2CH_0            (0x00001UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000001 */
#define ADC_AWD2CR_AWD2CH_1            (0x00002UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000002 */
#define ADC_AWD2CR_AWD2CH_2            (0x00004UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000004 */
#define ADC_AWD2CR_AWD2CH_3            (0x00008UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000008 */
#define ADC_AWD2CR_AWD2CH_4            (0x00010UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000010 */
#define ADC_AWD2CR_AWD2CH_5            (0x00020UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000020 */
#define ADC_AWD2CR_AWD2CH_6            (0x00040UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000040 */
#define ADC_AWD2CR_AWD2CH_7            (0x00080UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000080 */
#define ADC_AWD2CR_AWD2CH_8            (0x00100UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000100 */
#define ADC_AWD2CR_AWD2CH_9            (0x00200UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000200 */
#define ADC_AWD2CR_AWD2CH_10           (0x00400UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000400 */
#define ADC_AWD2CR_AWD2CH_11           (0x00800UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00000800 */
#define ADC_AWD2CR_AWD2CH_12           (0x01000UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00001000 */
#define ADC_AWD2CR_AWD2CH_13           (0x02000UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00002000 */
#define ADC_AWD2CR_AWD2CH_14           (0x04000UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00004000 */
#define ADC_AWD2CR_AWD2CH_15           (0x08000UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00008000 */
#define ADC_AWD2CR_AWD2CH_16           (0x10000UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00010000 */
#define ADC_AWD2CR_AWD2CH_17           (0x20000UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00020000 */
#define ADC_AWD2CR_AWD2CH_18           (0x40000UL << ADC_AWD2CR_AWD2CH_Pos)    /*!< 0x00040000 */

/********************  Bit definition for ADC_AWD3CR register  ****************/
#define ADC_AWD3CR_AWD3CH_Pos          (0UL)
#define ADC_AWD3CR_AWD3CH_Msk          (0x7FFFFUL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x0007FFFF */
#define ADC_AWD3CR_AWD3CH              ADC_AWD3CR_AWD3CH_Msk                   /*!< ADC analog watchdog 3 monitored channel selection */
#define ADC_AWD3CR_AWD3CH_0            (0x00001UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000001 */
#define ADC_AWD3CR_AWD3CH_1            (0x00002UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000002 */
#define ADC_AWD3CR_AWD3CH_2            (0x00004UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000004 */
#define ADC_AWD3CR_AWD3CH_3            (0x00008UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000008 */
#define ADC_AWD3CR_AWD3CH_4            (0x00010UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000010 */
#define ADC_AWD3CR_AWD3CH_5            (0x00020UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000020 */
#define ADC_AWD3CR_AWD3CH_6            (0x00040UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000040 */
#define ADC_AWD3CR_AWD3CH_7            (0x00080UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000080 */
#define ADC_AWD3CR_AWD3CH_8            (0x00100UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000100 */
#define ADC_AWD3CR_AWD3CH_9            (0x00200UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000200 */
#define ADC_AWD3CR_AWD3CH_10           (0x00400UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000400 */
#define ADC_AWD3CR_AWD3CH_11           (0x00800UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00000800 */
#define ADC_AWD3CR_AWD3CH_12           (0x01000UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00001000 */
#define ADC_AWD3CR_AWD3CH_13           (0x02000UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00002000 */
#define ADC_AWD3CR_AWD3CH_14           (0x04000UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00004000 */
#define ADC_AWD3CR_AWD3CH_15           (0x08000UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00008000 */
#define ADC_AWD3CR_AWD3CH_16           (0x10000UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00010000 */
#define ADC_AWD3CR_AWD3CH_17           (0x20000UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00020000 */
#define ADC_AWD3CR_AWD3CH_18           (0x40000UL << ADC_AWD3CR_AWD3CH_Pos)    /*!< 0x00040000 */

/********************  Bit definition for ADC_CALFACT register  ***************/
#define ADC_CALFACT_CALFACT_Pos        (0UL)
#define ADC_CALFACT_CALFACT_Msk        (0x7FUL << ADC_CALFACT_CALFACT_Pos)     /*!< 0x0000007F */
#define ADC_CALFACT_CALFACT            ADC_CALFACT_CALFACT_Msk                 /*!< ADC calibration factor in single-ended mode */
#define ADC_CALFACT_CALFACT_0          (0x01UL << ADC_CALFACT_CALFACT_Pos)     /*!< 0x00000001 */
#define ADC_CALFACT_CALFACT_1          (0x02UL << ADC_CALFACT_CALFACT_Pos)     /*!< 0x00000002 */
#define ADC_CALFACT_CALFACT_2          (0x04UL << ADC_CALFACT_CALFACT_Pos)     /*!< 0x00000004 */
#define ADC_CALFACT_CALFACT_3          (0x08UL << ADC_CALFACT_CALFACT_Pos)     /*!< 0x00000008 */
#define ADC_CALFACT_CALFACT_4          (0x10UL << ADC_CALFACT_CALFACT_Pos)     /*!< 0x00000010 */
#define ADC_CALFACT_CALFACT_5          (0x20UL << ADC_CALFACT_CALFACT_Pos)     /*!< 0x00000020 */
#define ADC_CALFACT_CALFACT_6          (0x40UL << ADC_CALFACT_CALFACT_Pos)     /*!< 0x00000040 */

/*************************  ADC Common registers  *****************************/
/********************  Bit definition for ADC_CCR register  *******************/
#define ADC_CCR_PRESC_Pos              (18UL)
#define ADC_CCR_PRESC_Msk              (0xFUL << ADC_CCR_PRESC_Pos)            /*!< 0x003C0000 */
#define ADC_CCR_PRESC                  ADC_CCR_PRESC_Msk                       /*!< ADC common clock prescaler, only for clock source asynchronous */
#define ADC_CCR_PRESC_0                (0x1UL << ADC_CCR_PRESC_Pos)            /*!< 0x00040000 */
#define ADC_CCR_PRESC_1                (0x2UL << ADC_CCR_PRESC_Pos)            /*!< 0x00080000 */
#define ADC_CCR_PRESC_2                (0x4UL << ADC_CCR_PRESC_Pos)            /*!< 0x00100000 */
#define ADC_CCR_PRESC_3                (0x8UL << ADC_CCR_PRESC_Pos)            /*!< 0x00200000 */

#define ADC_CCR_VREFEN_Pos             (22UL)
#define ADC_CCR_VREFEN_Msk             (0x1UL << ADC_CCR_VREFEN_Pos)           /*!< 0x00400000 */
#define ADC_CCR_VREFEN                 ADC_CCR_VREFEN_Msk                      /*!< ADC internal path to VrefInt enable */
#define ADC_CCR_TSEN_Pos               (23UL)
#define ADC_CCR_TSEN_Msk               (0x1UL << ADC_CCR_TSEN_Pos)             /*!< 0x00800000 */
#define ADC_CCR_TSEN                   ADC_CCR_TSEN_Msk                        /*!< ADC internal path to temperature sensor enable */
#define ADC_CCR_VBATEN_Pos             (24UL)
#define ADC_CCR_VBATEN_Msk             (0x1UL << ADC_CCR_VBATEN_Pos)           /*!< 0x01000000 */
#define ADC_CCR_VBATEN                 ADC_CCR_VBATEN_Msk                      /*!< ADC internal path to VBATEN sensor enable */


/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define CRC_DR_DR_Pos            (0UL)
#define CRC_DR_DR_Msk            (0xFFFFFFFFUL << CRC_DR_DR_Pos)               /*!< 0xFFFFFFFF */
#define CRC_DR_DR                CRC_DR_DR_Msk                                 /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
#define CRC_IDR_IDR_Pos          (0UL)
#define CRC_IDR_IDR_Msk          (0xFFFFFFFFUL << CRC_IDR_IDR_Pos)             /*!< 0xFFFFFFFF */
#define CRC_IDR_IDR              CRC_IDR_IDR_Msk                               /*!< General-purpose 32-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
#define CRC_CR_RESET_Pos         (0UL)
#define CRC_CR_RESET_Msk         (0x1UL << CRC_CR_RESET_Pos)                 /*!< 0x00000001 */
#define CRC_CR_RESET             CRC_CR_RESET_Msk                            /*!< RESET the CRC computation unit bit */
#define CRC_CR_POLYSIZE_Pos      (3UL)
#define CRC_CR_POLYSIZE_Msk      (0x3UL << CRC_CR_POLYSIZE_Pos)              /*!< 0x00000018 */
#define CRC_CR_POLYSIZE          CRC_CR_POLYSIZE_Msk                         /*!< Polynomial size bits */
#define CRC_CR_POLYSIZE_0        (0x1UL << CRC_CR_POLYSIZE_Pos)              /*!< 0x00000008 */
#define CRC_CR_POLYSIZE_1        (0x2UL << CRC_CR_POLYSIZE_Pos)              /*!< 0x00000010 */
#define CRC_CR_REV_IN_Pos        (5UL)
#define CRC_CR_REV_IN_Msk        (0x3UL << CRC_CR_REV_IN_Pos)                /*!< 0x00000060 */
#define CRC_CR_REV_IN            CRC_CR_REV_IN_Msk                           /*!< REV_IN Reverse Input Data bits */
#define CRC_CR_REV_IN_0          (0x1UL << CRC_CR_REV_IN_Pos)                /*!< 0x00000020 */
#define CRC_CR_REV_IN_1          (0x2UL << CRC_CR_REV_IN_Pos)                /*!< 0x00000040 */
#define CRC_CR_REV_OUT_Pos       (7UL)
#define CRC_CR_REV_OUT_Msk       (0x3UL << CRC_CR_REV_OUT_Pos)               /*!< 0x00000180 */
#define CRC_CR_REV_OUT           CRC_CR_REV_OUT_Msk                          /*!< REV_OUT Reverse Output Data bits */
#define CRC_CR_REV_OUT_0         (0x1UL << CRC_CR_REV_OUT_Pos)               /*!< 0x00000080 */
#define CRC_CR_REV_OUT_1         (0x2UL << CRC_CR_REV_OUT_Pos)               /*!< 0x00000100 */
#define CRC_CR_RTYPE_IN_Pos      (9UL)
#define CRC_CR_RTYPE_IN_Msk      (0x1UL << CRC_CR_RTYPE_IN_Pos)              /*!< 0x00000200 */
#define CRC_CR_RTYPE_IN          CRC_CR_RTYPE_IN_Msk                         /*!< Reverse type input */
#define CRC_CR_RTYPE_OUT_Pos     (10UL)
#define CRC_CR_RTYPE_OUT_Msk     (0x1UL << CRC_CR_RTYPE_OUT_Pos)             /*!< 0x00000400 */
#define CRC_CR_RTYPE_OUT         CRC_CR_RTYPE_OUT_Msk                        /*!< Reverse type output*/


/*******************  Bit definition for CRC_INIT register  *******************/
#define CRC_INIT_INIT_Pos        (0UL)
#define CRC_INIT_INIT_Msk        (0xFFFFFFFFUL << CRC_INIT_INIT_Pos)           /*!< 0xFFFFFFFF */
#define CRC_INIT_INIT            CRC_INIT_INIT_Msk                             /*!< Initial CRC value bits */

/*******************  Bit definition for CRC_POL register  ********************/
#define CRC_POL_POL_Pos          (0UL)
#define CRC_POL_POL_Msk          (0xFFFFFFFFUL << CRC_POL_POL_Pos)             /*!< 0xFFFFFFFF */
#define CRC_POL_POL              CRC_POL_POL_Msk                               /*!< Coefficients of the polynomial */
/******************************************************************************/
/*                                                                            */
/*                          CRS Clock Recovery System                         */
/******************************************************************************/

/*******************  Bit definition for CRS_CR register  *********************/
#define CRS_CR_SYNCOKIE_Pos       (0UL)
#define CRS_CR_SYNCOKIE_Msk       (0x1UL << CRS_CR_SYNCOKIE_Pos)               /*!< 0x00000001 */
#define CRS_CR_SYNCOKIE           CRS_CR_SYNCOKIE_Msk                          /*!< SYNC event OK interrupt enable */
#define CRS_CR_SYNCWARNIE_Pos     (1UL)
#define CRS_CR_SYNCWARNIE_Msk     (0x1UL << CRS_CR_SYNCWARNIE_Pos)             /*!< 0x00000002 */
#define CRS_CR_SYNCWARNIE         CRS_CR_SYNCWARNIE_Msk                        /*!< SYNC warning interrupt enable */
#define CRS_CR_ERRIE_Pos          (2UL)
#define CRS_CR_ERRIE_Msk          (0x1UL << CRS_CR_ERRIE_Pos)                  /*!< 0x00000004 */
#define CRS_CR_ERRIE              CRS_CR_ERRIE_Msk                             /*!< SYNC error or trimming error interrupt enable */
#define CRS_CR_ESYNCIE_Pos        (3UL)
#define CRS_CR_ESYNCIE_Msk        (0x1UL << CRS_CR_ESYNCIE_Pos)                /*!< 0x00000008 */
#define CRS_CR_ESYNCIE            CRS_CR_ESYNCIE_Msk                           /*!< Expected SYNC interrupt enable */
#define CRS_CR_CEN_Pos            (5UL)
#define CRS_CR_CEN_Msk            (0x1UL << CRS_CR_CEN_Pos)                    /*!< 0x00000020 */
#define CRS_CR_CEN                CRS_CR_CEN_Msk                               /*!< Frequency error counter enable */
#define CRS_CR_AUTOTRIMEN_Pos     (6UL)
#define CRS_CR_AUTOTRIMEN_Msk     (0x1UL << CRS_CR_AUTOTRIMEN_Pos)             /*!< 0x00000040 */
#define CRS_CR_AUTOTRIMEN         CRS_CR_AUTOTRIMEN_Msk                        /*!< Automatic trimming enable */
#define CRS_CR_SWSYNC_Pos         (7UL)
#define CRS_CR_SWSYNC_Msk         (0x1UL << CRS_CR_SWSYNC_Pos)                 /*!< 0x00000080 */
#define CRS_CR_SWSYNC             CRS_CR_SWSYNC_Msk                            /*!< Generate software SYNC event */
#define CRS_CR_TRIM_Pos           (8UL)
#define CRS_CR_TRIM_Msk           (0x3FUL << CRS_CR_TRIM_Pos)                  /*!< 0x00003F00 */
#define CRS_CR_TRIM               CRS_CR_TRIM_Msk                              /*!< HSI48 oscillator smooth trimming */

/*******************  Bit definition for CRS_CFGR register  *********************/
#define CRS_CFGR_RELOAD_Pos       (0UL)
#define CRS_CFGR_RELOAD_Msk       (0xFFFFUL << CRS_CFGR_RELOAD_Pos)            /*!< 0x0000FFFF */
#define CRS_CFGR_RELOAD           CRS_CFGR_RELOAD_Msk                          /*!< Counter reload value */
#define CRS_CFGR_FELIM_Pos        (16UL)
#define CRS_CFGR_FELIM_Msk        (0xFFUL << CRS_CFGR_FELIM_Pos)               /*!< 0x00FF0000 */
#define CRS_CFGR_FELIM            CRS_CFGR_FELIM_Msk                           /*!< Frequency error limit */

#define CRS_CFGR_SYNCDIV_Pos      (24UL)
#define CRS_CFGR_SYNCDIV_Msk      (0x7UL << CRS_CFGR_SYNCDIV_Pos)              /*!< 0x07000000 */
#define CRS_CFGR_SYNCDIV          CRS_CFGR_SYNCDIV_Msk                         /*!< SYNC divider */
#define CRS_CFGR_SYNCDIV_0        (0x1UL << CRS_CFGR_SYNCDIV_Pos)               /*!< 0x01000000 */
#define CRS_CFGR_SYNCDIV_1        (0x2UL << CRS_CFGR_SYNCDIV_Pos)               /*!< 0x02000000 */
#define CRS_CFGR_SYNCDIV_2        (0x4UL << CRS_CFGR_SYNCDIV_Pos)               /*!< 0x04000000 */

#define CRS_CFGR_SYNCSRC_Pos      (28UL)
#define CRS_CFGR_SYNCSRC_Msk      (0x3UL << CRS_CFGR_SYNCSRC_Pos)              /*!< 0x30000000 */
#define CRS_CFGR_SYNCSRC          CRS_CFGR_SYNCSRC_Msk                         /*!< SYNC signal source selection */
#define CRS_CFGR_SYNCSRC_0        (0x1UL << CRS_CFGR_SYNCSRC_Pos)               /*!< 0x10000000 */
#define CRS_CFGR_SYNCSRC_1        (0x2UL << CRS_CFGR_SYNCSRC_Pos)               /*!< 0x20000000 */

#define CRS_CFGR_SYNCPOL_Pos      (31UL)
#define CRS_CFGR_SYNCPOL_Msk      (0x1UL << CRS_CFGR_SYNCPOL_Pos)              /*!< 0x80000000 */
#define CRS_CFGR_SYNCPOL          CRS_CFGR_SYNCPOL_Msk                         /*!< SYNC polarity selection */

/*******************  Bit definition for CRS_ISR register  *********************/
#define CRS_ISR_SYNCOKF_Pos       (0UL)
#define CRS_ISR_SYNCOKF_Msk       (0x1UL << CRS_ISR_SYNCOKF_Pos)               /*!< 0x00000001 */
#define CRS_ISR_SYNCOKF           CRS_ISR_SYNCOKF_Msk                          /*!< SYNC event OK flag */
#define CRS_ISR_SYNCWARNF_Pos     (1UL)
#define CRS_ISR_SYNCWARNF_Msk     (0x1UL << CRS_ISR_SYNCWARNF_Pos)             /*!< 0x00000002 */
#define CRS_ISR_SYNCWARNF         CRS_ISR_SYNCWARNF_Msk                        /*!< SYNC warning flag */
#define CRS_ISR_ERRF_Pos          (2UL)
#define CRS_ISR_ERRF_Msk          (0x1UL << CRS_ISR_ERRF_Pos)                  /*!< 0x00000004 */
#define CRS_ISR_ERRF              CRS_ISR_ERRF_Msk                             /*!< Error flag */
#define CRS_ISR_ESYNCF_Pos        (3UL)
#define CRS_ISR_ESYNCF_Msk        (0x1UL << CRS_ISR_ESYNCF_Pos)                /*!< 0x00000008 */
#define CRS_ISR_ESYNCF            CRS_ISR_ESYNCF_Msk                           /*!< Expected SYNC flag */
#define CRS_ISR_SYNCERR_Pos       (8UL)
#define CRS_ISR_SYNCERR_Msk       (0x1UL << CRS_ISR_SYNCERR_Pos)               /*!< 0x00000100 */
#define CRS_ISR_SYNCERR           CRS_ISR_SYNCERR_Msk                          /*!< SYNC error */
#define CRS_ISR_SYNCMISS_Pos      (9UL)
#define CRS_ISR_SYNCMISS_Msk      (0x1UL << CRS_ISR_SYNCMISS_Pos)              /*!< 0x00000200 */
#define CRS_ISR_SYNCMISS          CRS_ISR_SYNCMISS_Msk                         /*!< SYNC missed */
#define CRS_ISR_TRIMOVF_Pos       (10UL)
#define CRS_ISR_TRIMOVF_Msk       (0x1UL << CRS_ISR_TRIMOVF_Pos)               /*!< 0x00000400 */
#define CRS_ISR_TRIMOVF           CRS_ISR_TRIMOVF_Msk                          /*!< Trimming overflow or underflow */
#define CRS_ISR_FEDIR_Pos         (15UL)
#define CRS_ISR_FEDIR_Msk         (0x1UL << CRS_ISR_FEDIR_Pos)                 /*!< 0x00008000 */
#define CRS_ISR_FEDIR             CRS_ISR_FEDIR_Msk                            /*!< Frequency error direction */
#define CRS_ISR_FECAP_Pos         (16UL)
#define CRS_ISR_FECAP_Msk         (0xFFFFUL << CRS_ISR_FECAP_Pos)              /*!< 0xFFFF0000 */
#define CRS_ISR_FECAP             CRS_ISR_FECAP_Msk                            /*!< Frequency error capture */

/*******************  Bit definition for CRS_ICR register  *********************/
#define CRS_ICR_SYNCOKC_Pos       (0UL)
#define CRS_ICR_SYNCOKC_Msk       (0x1UL << CRS_ICR_SYNCOKC_Pos)               /*!< 0x00000001 */
#define CRS_ICR_SYNCOKC           CRS_ICR_SYNCOKC_Msk                          /*!< SYNC event OK clear flag */
#define CRS_ICR_SYNCWARNC_Pos     (1UL)
#define CRS_ICR_SYNCWARNC_Msk     (0x1UL << CRS_ICR_SYNCWARNC_Pos)             /*!< 0x00000002 */
#define CRS_ICR_SYNCWARNC         CRS_ICR_SYNCWARNC_Msk                        /*!< SYNC warning clear flag */
#define CRS_ICR_ERRC_Pos          (2UL)
#define CRS_ICR_ERRC_Msk          (0x1UL << CRS_ICR_ERRC_Pos)                  /*!< 0x00000004 */
#define CRS_ICR_ERRC              CRS_ICR_ERRC_Msk                             /*!< Error clear flag */
#define CRS_ICR_ESYNCC_Pos        (3UL)
#define CRS_ICR_ESYNCC_Msk        (0x1UL << CRS_ICR_ESYNCC_Pos)                /*!< 0x00000008 */
#define CRS_ICR_ESYNCC            CRS_ICR_ESYNCC_Msk                           /*!< Expected SYNC clear flag */
/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DAC_CR register  ********************/
#define DAC_CR_EN1_Pos              (0UL)
#define DAC_CR_EN1_Msk              (0x1UL << DAC_CR_EN1_Pos)                  /*!< 0x00000001 */
#define DAC_CR_EN1                  DAC_CR_EN1_Msk                             /*!<DAC channel1 enable */
#define DAC_CR_TEN1_Pos             (1UL)
#define DAC_CR_TEN1_Msk             (0x1UL << DAC_CR_TEN1_Pos)                 /*!< 0x00000002 */
#define DAC_CR_TEN1                 DAC_CR_TEN1_Msk                            /*!<DAC channel1 Trigger enable */

#define DAC_CR_TSEL1_Pos            (2UL)
#define DAC_CR_TSEL1_Msk            (0xFUL << DAC_CR_TSEL1_Pos)                /*!< 0x0000003C */
#define DAC_CR_TSEL1                DAC_CR_TSEL1_Msk                           /*!<TSEL1[2:0] (DAC channel1 Trigger selection) */
#define DAC_CR_TSEL1_0              (0x1UL << DAC_CR_TSEL1_Pos)                 /*!< 0x00000004 */
#define DAC_CR_TSEL1_1              (0x2UL << DAC_CR_TSEL1_Pos)                 /*!< 0x00000008 */
#define DAC_CR_TSEL1_2              (0x4UL << DAC_CR_TSEL1_Pos)                 /*!< 0x00000010 */
#define DAC_CR_TSEL1_3              (0x8UL << DAC_CR_TSEL1_Pos)                 /*!< 0x00000020 */


#define DAC_CR_WAVE1_Pos            (6UL)
#define DAC_CR_WAVE1_Msk            (0x3UL << DAC_CR_WAVE1_Pos)                /*!< 0x000000C0 */
#define DAC_CR_WAVE1                DAC_CR_WAVE1_Msk                           /*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define DAC_CR_WAVE1_0              (0x1UL << DAC_CR_WAVE1_Pos)                 /*!< 0x00000040 */
#define DAC_CR_WAVE1_1              (0x2UL << DAC_CR_WAVE1_Pos)                 /*!< 0x00000080 */

#define DAC_CR_MAMP1_Pos            (8UL)
#define DAC_CR_MAMP1_Msk            (0xFUL << DAC_CR_MAMP1_Pos)                /*!< 0x00000F00 */
#define DAC_CR_MAMP1                DAC_CR_MAMP1_Msk                           /*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define DAC_CR_MAMP1_0              (0x1UL << DAC_CR_MAMP1_Pos)                 /*!< 0x00000100 */
#define DAC_CR_MAMP1_1              (0x2UL << DAC_CR_MAMP1_Pos)                 /*!< 0x00000200 */
#define DAC_CR_MAMP1_2              (0x4UL << DAC_CR_MAMP1_Pos)                 /*!< 0x00000400 */
#define DAC_CR_MAMP1_3              (0x8UL << DAC_CR_MAMP1_Pos)                 /*!< 0x00000800 */

#define DAC_CR_DMAEN1_Pos           (12UL)
#define DAC_CR_DMAEN1_Msk           (0x1UL << DAC_CR_DMAEN1_Pos)               /*!< 0x00001000 */
#define DAC_CR_DMAEN1               DAC_CR_DMAEN1_Msk                          /*!<DAC channel1 DMA enable */
#define DAC_CR_DMAUDRIE1_Pos        (13UL)
#define DAC_CR_DMAUDRIE1_Msk        (0x1UL << DAC_CR_DMAUDRIE1_Pos)            /*!< 0x00002000 */
#define DAC_CR_DMAUDRIE1            DAC_CR_DMAUDRIE1_Msk                       /*!<DAC channel 1 DMA underrun interrupt enable  >*/
#define DAC_CR_CEN1_Pos             (14UL)
#define DAC_CR_CEN1_Msk             (0x1UL << DAC_CR_CEN1_Pos)                 /*!< 0x00004000 */
#define DAC_CR_CEN1                 DAC_CR_CEN1_Msk                            /*!<DAC channel 1 calibration enable >*/

#define DAC_CR_EN2_Pos              (16UL)
#define DAC_CR_EN2_Msk              (0x1UL << DAC_CR_EN2_Pos)                  /*!< 0x00010000 */
#define DAC_CR_EN2                  DAC_CR_EN2_Msk                             /*!<DAC channel2 enable */
#define DAC_CR_TEN2_Pos             (17UL)
#define DAC_CR_TEN2_Msk             (0x1UL << DAC_CR_TEN2_Pos)                 /*!< 0x00020000 */
#define DAC_CR_TEN2                 DAC_CR_TEN2_Msk                            /*!<DAC channel2 Trigger enable */

#define DAC_CR_TSEL2_Pos            (18UL)
#define DAC_CR_TSEL2_Msk            (0xFUL << DAC_CR_TSEL2_Pos)                /*!< 0x003C0000 */
#define DAC_CR_TSEL2                DAC_CR_TSEL2_Msk                           /*!<TSEL2[2:0] (DAC channel2 Trigger selection) */
#define DAC_CR_TSEL2_0              (0x1UL << DAC_CR_TSEL2_Pos)                 /*!< 0x00040000 */
#define DAC_CR_TSEL2_1              (0x2UL << DAC_CR_TSEL2_Pos)                 /*!< 0x00080000 */
#define DAC_CR_TSEL2_2              (0x4UL << DAC_CR_TSEL2_Pos)                 /*!< 0x00100000 */
#define DAC_CR_TSEL2_3              (0x8UL << DAC_CR_TSEL2_Pos)                 /*!< 0x00200000 */


#define DAC_CR_WAVE2_Pos            (22UL)
#define DAC_CR_WAVE2_Msk            (0x3UL << DAC_CR_WAVE2_Pos)                /*!< 0x00C00000 */
#define DAC_CR_WAVE2                DAC_CR_WAVE2_Msk                           /*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define DAC_CR_WAVE2_0              (0x1UL << DAC_CR_WAVE2_Pos)                 /*!< 0x00400000 */
#define DAC_CR_WAVE2_1              (0x2UL << DAC_CR_WAVE2_Pos)                 /*!< 0x00800000 */

#define DAC_CR_MAMP2_Pos            (24UL)
#define DAC_CR_MAMP2_Msk            (0xFUL << DAC_CR_MAMP2_Pos)                /*!< 0x0F000000 */
#define DAC_CR_MAMP2                DAC_CR_MAMP2_Msk                           /*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define DAC_CR_MAMP2_0              (0x1UL << DAC_CR_MAMP2_Pos)                 /*!< 0x01000000 */
#define DAC_CR_MAMP2_1              (0x2UL << DAC_CR_MAMP2_Pos)                 /*!< 0x02000000 */
#define DAC_CR_MAMP2_2              (0x4UL << DAC_CR_MAMP2_Pos)                 /*!< 0x04000000 */
#define DAC_CR_MAMP2_3              (0x8UL << DAC_CR_MAMP2_Pos)                 /*!< 0x08000000 */

#define DAC_CR_DMAEN2_Pos           (28UL)
#define DAC_CR_DMAEN2_Msk           (0x1UL << DAC_CR_DMAEN2_Pos)               /*!< 0x10000000 */
#define DAC_CR_DMAEN2               DAC_CR_DMAEN2_Msk                          /*!<DAC channel2 DMA enabled */
#define DAC_CR_DMAUDRIE2_Pos        (29UL)
#define DAC_CR_DMAUDRIE2_Msk        (0x1UL << DAC_CR_DMAUDRIE2_Pos)            /*!< 0x20000000 */
#define DAC_CR_DMAUDRIE2            DAC_CR_DMAUDRIE2_Msk                       /*!<DAC channel2 DMA underrun interrupt enable  >*/
#define DAC_CR_CEN2_Pos             (30UL)
#define DAC_CR_CEN2_Msk             (0x1UL << DAC_CR_CEN2_Pos)                 /*!< 0x40000000 */
#define DAC_CR_CEN2                 DAC_CR_CEN2_Msk                            /*!<DAC channel2 calibration enable >*/

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define DAC_SWTRIGR_SWTRIG1_Pos     (0UL)
#define DAC_SWTRIGR_SWTRIG1_Msk     (0x1UL << DAC_SWTRIGR_SWTRIG1_Pos)         /*!< 0x00000001 */
#define DAC_SWTRIGR_SWTRIG1         DAC_SWTRIGR_SWTRIG1_Msk                    /*!<DAC channel1 software trigger */
#define DAC_SWTRIGR_SWTRIG2_Pos     (1UL)
#define DAC_SWTRIGR_SWTRIG2_Msk     (0x1UL << DAC_SWTRIGR_SWTRIG2_Pos)         /*!< 0x00000002 */
#define DAC_SWTRIGR_SWTRIG2         DAC_SWTRIGR_SWTRIG2_Msk                    /*!<DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define DAC_DHR12R1_DACC1DHR_Pos    (0UL)
#define DAC_DHR12R1_DACC1DHR_Msk    (0xFFFUL << DAC_DHR12R1_DACC1DHR_Pos)      /*!< 0x00000FFF */
#define DAC_DHR12R1_DACC1DHR        DAC_DHR12R1_DACC1DHR_Msk                   /*!<DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define DAC_DHR12L1_DACC1DHR_Pos    (4UL)
#define DAC_DHR12L1_DACC1DHR_Msk    (0xFFFUL << DAC_DHR12L1_DACC1DHR_Pos)      /*!< 0x0000FFF0 */
#define DAC_DHR12L1_DACC1DHR        DAC_DHR12L1_DACC1DHR_Msk                   /*!<DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define DAC_DHR8R1_DACC1DHR_Pos     (0UL)
#define DAC_DHR8R1_DACC1DHR_Msk     (0xFFUL << DAC_DHR8R1_DACC1DHR_Pos)        /*!< 0x000000FF */
#define DAC_DHR8R1_DACC1DHR         DAC_DHR8R1_DACC1DHR_Msk                    /*!<DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define DAC_DHR12R2_DACC2DHR_Pos    (0UL)
#define DAC_DHR12R2_DACC2DHR_Msk    (0xFFFUL << DAC_DHR12R2_DACC2DHR_Pos)      /*!< 0x00000FFF */
#define DAC_DHR12R2_DACC2DHR        DAC_DHR12R2_DACC2DHR_Msk                   /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define DAC_DHR12L2_DACC2DHR_Pos    (4UL)
#define DAC_DHR12L2_DACC2DHR_Msk    (0xFFFUL << DAC_DHR12L2_DACC2DHR_Pos)      /*!< 0x0000FFF0 */
#define DAC_DHR12L2_DACC2DHR        DAC_DHR12L2_DACC2DHR_Msk                   /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define DAC_DHR8R2_DACC2DHR_Pos     (0UL)
#define DAC_DHR8R2_DACC2DHR_Msk     (0xFFUL << DAC_DHR8R2_DACC2DHR_Pos)        /*!< 0x000000FF */
#define DAC_DHR8R2_DACC2DHR         DAC_DHR8R2_DACC2DHR_Msk                    /*!<DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define DAC_DHR12RD_DACC1DHR_Pos    (0UL)
#define DAC_DHR12RD_DACC1DHR_Msk    (0xFFFUL << DAC_DHR12RD_DACC1DHR_Pos)      /*!< 0x00000FFF */
#define DAC_DHR12RD_DACC1DHR        DAC_DHR12RD_DACC1DHR_Msk                   /*!<DAC channel1 12-bit Right aligned data */
#define DAC_DHR12RD_DACC2DHR_Pos    (16UL)
#define DAC_DHR12RD_DACC2DHR_Msk    (0xFFFUL << DAC_DHR12RD_DACC2DHR_Pos)      /*!< 0x0FFF0000 */
#define DAC_DHR12RD_DACC2DHR        DAC_DHR12RD_DACC2DHR_Msk                   /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define DAC_DHR12LD_DACC1DHR_Pos    (4UL)
#define DAC_DHR12LD_DACC1DHR_Msk    (0xFFFUL << DAC_DHR12LD_DACC1DHR_Pos)      /*!< 0x0000FFF0 */
#define DAC_DHR12LD_DACC1DHR        DAC_DHR12LD_DACC1DHR_Msk                   /*!<DAC channel1 12-bit Left aligned data */
#define DAC_DHR12LD_DACC2DHR_Pos    (20UL)
#define DAC_DHR12LD_DACC2DHR_Msk    (0xFFFUL << DAC_DHR12LD_DACC2DHR_Pos)      /*!< 0xFFF00000 */
#define DAC_DHR12LD_DACC2DHR        DAC_DHR12LD_DACC2DHR_Msk                   /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define DAC_DHR8RD_DACC1DHR_Pos     (0UL)
#define DAC_DHR8RD_DACC1DHR_Msk     (0xFFUL << DAC_DHR8RD_DACC1DHR_Pos)        /*!< 0x000000FF */
#define DAC_DHR8RD_DACC1DHR         DAC_DHR8RD_DACC1DHR_Msk                    /*!<DAC channel1 8-bit Right aligned data */
#define DAC_DHR8RD_DACC2DHR_Pos     (8UL)
#define DAC_DHR8RD_DACC2DHR_Msk     (0xFFUL << DAC_DHR8RD_DACC2DHR_Pos)        /*!< 0x0000FF00 */
#define DAC_DHR8RD_DACC2DHR         DAC_DHR8RD_DACC2DHR_Msk                    /*!<DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define DAC_DOR1_DACC1DOR_Pos       (0UL)
#define DAC_DOR1_DACC1DOR_Msk       (0xFFFUL << DAC_DOR1_DACC1DOR_Pos)         /*!< 0x00000FFF */
#define DAC_DOR1_DACC1DOR           DAC_DOR1_DACC1DOR_Msk                      /*!<DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define DAC_DOR2_DACC2DOR_Pos       (0UL)
#define DAC_DOR2_DACC2DOR_Msk       (0xFFFUL << DAC_DOR2_DACC2DOR_Pos)         /*!< 0x00000FFF */
#define DAC_DOR2_DACC2DOR           DAC_DOR2_DACC2DOR_Msk                      /*!<DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define DAC_SR_DMAUDR1_Pos          (13UL)
#define DAC_SR_DMAUDR1_Msk          (0x1UL << DAC_SR_DMAUDR1_Pos)              /*!< 0x00002000 */
#define DAC_SR_DMAUDR1              DAC_SR_DMAUDR1_Msk                         /*!<DAC channel1 DMA underrun flag */
#define DAC_SR_CAL_FLAG1_Pos        (14UL)
#define DAC_SR_CAL_FLAG1_Msk        (0x1UL << DAC_SR_CAL_FLAG1_Pos)            /*!< 0x00004000 */
#define DAC_SR_CAL_FLAG1            DAC_SR_CAL_FLAG1_Msk                       /*!<DAC channel1 calibration offset status */
#define DAC_SR_BWST1_Pos            (15UL)
#define DAC_SR_BWST1_Msk            (0x4001UL << DAC_SR_BWST1_Pos)             /*!< 0x20008000 */
#define DAC_SR_BWST1                DAC_SR_BWST1_Msk                           /*!<DAC channel1 busy writing sample time flag */

#define DAC_SR_DMAUDR2_Pos          (29UL)
#define DAC_SR_DMAUDR2_Msk          (0x1UL << DAC_SR_DMAUDR2_Pos)              /*!< 0x20000000 */
#define DAC_SR_DMAUDR2              DAC_SR_DMAUDR2_Msk                         /*!<DAC channel2 DMA underrun flag */
#define DAC_SR_CAL_FLAG2_Pos        (30UL)
#define DAC_SR_CAL_FLAG2_Msk        (0x1UL << DAC_SR_CAL_FLAG2_Pos)            /*!< 0x40000000 */
#define DAC_SR_CAL_FLAG2            DAC_SR_CAL_FLAG2_Msk                       /*!<DAC channel2 calibration offset status */
#define DAC_SR_BWST2_Pos            (31UL)
#define DAC_SR_BWST2_Msk            (0x1UL << DAC_SR_BWST2_Pos)                /*!< 0x80000000 */
#define DAC_SR_BWST2                DAC_SR_BWST2_Msk                           /*!<DAC channel2 busy writing sample time flag */

/*******************  Bit definition for DAC_CCR register  ********************/
#define DAC_CCR_OTRIM1_Pos          (0UL)
#define DAC_CCR_OTRIM1_Msk          (0x1FUL << DAC_CCR_OTRIM1_Pos)             /*!< 0x0000001F */
#define DAC_CCR_OTRIM1              DAC_CCR_OTRIM1_Msk                         /*!<DAC channel1 offset trimming value */
#define DAC_CCR_OTRIM2_Pos          (16UL)
#define DAC_CCR_OTRIM2_Msk          (0x1FUL << DAC_CCR_OTRIM2_Pos)             /*!< 0x001F0000 */
#define DAC_CCR_OTRIM2              DAC_CCR_OTRIM2_Msk                         /*!<DAC channel2 offset trimming value */

/*******************  Bit definition for DAC_MCR register  *******************/
#define DAC_MCR_MODE1_Pos           (0UL)
#define DAC_MCR_MODE1_Msk           (0x7UL << DAC_MCR_MODE1_Pos)               /*!< 0x00000007 */
#define DAC_MCR_MODE1               DAC_MCR_MODE1_Msk                          /*!<MODE1[2:0] (DAC channel1 mode) */
#define DAC_MCR_MODE1_0             (0x1UL << DAC_MCR_MODE1_Pos)                /*!< 0x00000001 */
#define DAC_MCR_MODE1_1             (0x2UL << DAC_MCR_MODE1_Pos)                /*!< 0x00000002 */
#define DAC_MCR_MODE1_2             (0x4UL << DAC_MCR_MODE1_Pos)                /*!< 0x00000004 */

#define DAC_MCR_MODE2_Pos           (16UL)
#define DAC_MCR_MODE2_Msk           (0x7UL << DAC_MCR_MODE2_Pos)               /*!< 0x00070000 */
#define DAC_MCR_MODE2               DAC_MCR_MODE2_Msk                          /*!<MODE2[2:0] (DAC channel2 mode) */
#define DAC_MCR_MODE2_0             (0x1UL << DAC_MCR_MODE2_Pos)                /*!< 0x00010000 */
#define DAC_MCR_MODE2_1             (0x2UL << DAC_MCR_MODE2_Pos)                /*!< 0x00020000 */
#define DAC_MCR_MODE2_2             (0x4UL << DAC_MCR_MODE2_Pos)                /*!< 0x00040000 */

/******************  Bit definition for DAC_SHSR1 register  ******************/
#define DAC_SHSR1_TSAMPLE1_Pos      (0UL)
#define DAC_SHSR1_TSAMPLE1_Msk      (0x3FFUL << DAC_SHSR1_TSAMPLE1_Pos)        /*!< 0x000003FF */
#define DAC_SHSR1_TSAMPLE1          DAC_SHSR1_TSAMPLE1_Msk                     /*!<DAC channel1 sample time */

/******************  Bit definition for DAC_SHSR2 register  ******************/
#define DAC_SHSR2_TSAMPLE2_Pos      (0UL)
#define DAC_SHSR2_TSAMPLE2_Msk      (0x3FFUL << DAC_SHSR2_TSAMPLE2_Pos)        /*!< 0x000003FF */
#define DAC_SHSR2_TSAMPLE2          DAC_SHSR2_TSAMPLE2_Msk                     /*!<DAC channel2 sample time */

/******************  Bit definition for DAC_SHHR register  ******************/
#define DAC_SHHR_THOLD1_Pos         (0UL)
#define DAC_SHHR_THOLD1_Msk         (0x3FFUL << DAC_SHHR_THOLD1_Pos)           /*!< 0x000003FF */
#define DAC_SHHR_THOLD1             DAC_SHHR_THOLD1_Msk                        /*!<DAC channel1 hold time */
#define DAC_SHHR_THOLD2_Pos         (16UL)
#define DAC_SHHR_THOLD2_Msk         (0x3FFUL << DAC_SHHR_THOLD2_Pos)           /*!< 0x03FF0000 */
#define DAC_SHHR_THOLD2             DAC_SHHR_THOLD2_Msk                        /*!<DAC channel2 hold time */

/******************  Bit definition for DAC_SHRR register  ******************/
#define DAC_SHRR_TREFRESH1_Pos      (0UL)
#define DAC_SHRR_TREFRESH1_Msk      (0xFFUL << DAC_SHRR_TREFRESH1_Pos)         /*!< 0x000000FF */
#define DAC_SHRR_TREFRESH1          DAC_SHRR_TREFRESH1_Msk                     /*!<DAC channel1 refresh time */
#define DAC_SHRR_TREFRESH2_Pos      (16UL)
#define DAC_SHRR_TREFRESH2_Msk      (0xFFUL << DAC_SHRR_TREFRESH2_Pos)         /*!< 0x00FF0000 */
#define DAC_SHRR_TREFRESH2          DAC_SHRR_TREFRESH2_Msk                     /*!<DAC channel2 refresh time */

/******************************************************************************/
/*                                                                            */
/*                           DMA Controller (DMA)                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for DMA_ISR register  ********************/
#define DMA_ISR_GIF1_Pos       (0UL)
#define DMA_ISR_GIF1_Msk       (0x1UL << DMA_ISR_GIF1_Pos)                     /*!< 0x00000001 */
#define DMA_ISR_GIF1           DMA_ISR_GIF1_Msk                                /*!< Channel 1 Global interrupt flag */
#define DMA_ISR_TCIF1_Pos      (1UL)
#define DMA_ISR_TCIF1_Msk      (0x1UL << DMA_ISR_TCIF1_Pos)                    /*!< 0x00000002 */
#define DMA_ISR_TCIF1          DMA_ISR_TCIF1_Msk                               /*!< Channel 1 Transfer Complete flag */
#define DMA_ISR_HTIF1_Pos      (2UL)
#define DMA_ISR_HTIF1_Msk      (0x1UL << DMA_ISR_HTIF1_Pos)                    /*!< 0x00000004 */
#define DMA_ISR_HTIF1          DMA_ISR_HTIF1_Msk                               /*!< Channel 1 Half Transfer flag */
#define DMA_ISR_TEIF1_Pos      (3UL)
#define DMA_ISR_TEIF1_Msk      (0x1UL << DMA_ISR_TEIF1_Pos)                    /*!< 0x00000008 */
#define DMA_ISR_TEIF1          DMA_ISR_TEIF1_Msk                               /*!< Channel 1 Transfer Error flag */
#define DMA_ISR_GIF2_Pos       (4UL)
#define DMA_ISR_GIF2_Msk       (0x1UL << DMA_ISR_GIF2_Pos)                     /*!< 0x00000010 */
#define DMA_ISR_GIF2           DMA_ISR_GIF2_Msk                                /*!< Channel 2 Global interrupt flag */
#define DMA_ISR_TCIF2_Pos      (5UL)
#define DMA_ISR_TCIF2_Msk      (0x1UL << DMA_ISR_TCIF2_Pos)                    /*!< 0x00000020 */
#define DMA_ISR_TCIF2          DMA_ISR_TCIF2_Msk                               /*!< Channel 2 Transfer Complete flag */
#define DMA_ISR_HTIF2_Pos      (6UL)
#define DMA_ISR_HTIF2_Msk      (0x1UL << DMA_ISR_HTIF2_Pos)                    /*!< 0x00000040 */
#define DMA_ISR_HTIF2          DMA_ISR_HTIF2_Msk                               /*!< Channel 2 Half Transfer flag */
#define DMA_ISR_TEIF2_Pos      (7UL)
#define DMA_ISR_TEIF2_Msk      (0x1UL << DMA_ISR_TEIF2_Pos)                    /*!< 0x00000080 */
#define DMA_ISR_TEIF2          DMA_ISR_TEIF2_Msk                               /*!< Channel 2 Transfer Error flag */
#define DMA_ISR_GIF3_Pos       (8UL)
#define DMA_ISR_GIF3_Msk       (0x1UL << DMA_ISR_GIF3_Pos)                     /*!< 0x00000100 */
#define DMA_ISR_GIF3           DMA_ISR_GIF3_Msk                                /*!< Channel 3 Global interrupt flag */
#define DMA_ISR_TCIF3_Pos      (9UL)
#define DMA_ISR_TCIF3_Msk      (0x1UL << DMA_ISR_TCIF3_Pos)                    /*!< 0x00000200 */
#define DMA_ISR_TCIF3          DMA_ISR_TCIF3_Msk                               /*!< Channel 3 Transfer Complete flag */
#define DMA_ISR_HTIF3_Pos      (10UL)
#define DMA_ISR_HTIF3_Msk      (0x1UL << DMA_ISR_HTIF3_Pos)                    /*!< 0x00000400 */
#define DMA_ISR_HTIF3          DMA_ISR_HTIF3_Msk                               /*!< Channel 3 Half Transfer flag */
#define DMA_ISR_TEIF3_Pos      (11UL)
#define DMA_ISR_TEIF3_Msk      (0x1UL << DMA_ISR_TEIF3_Pos)                    /*!< 0x00000800 */
#define DMA_ISR_TEIF3          DMA_ISR_TEIF3_Msk                               /*!< Channel 4 Transfer Error flag */
#define DMA_ISR_GIF4_Pos       (12UL)
#define DMA_ISR_GIF4_Msk       (0x1UL << DMA_ISR_GIF4_Pos)                     /*!< 0x00001000 */
#define DMA_ISR_GIF4           DMA_ISR_GIF4_Msk                                /*!< Channel 4 Global interrupt flag */
#define DMA_ISR_TCIF4_Pos      (13UL)
#define DMA_ISR_TCIF4_Msk      (0x1UL << DMA_ISR_TCIF4_Pos)                    /*!< 0x00002000 */
#define DMA_ISR_TCIF4          DMA_ISR_TCIF4_Msk                               /*!< Channel 4 Transfer Complete flag */
#define DMA_ISR_HTIF4_Pos      (14UL)
#define DMA_ISR_HTIF4_Msk      (0x1UL << DMA_ISR_HTIF4_Pos)                    /*!< 0x00004000 */
#define DMA_ISR_HTIF4          DMA_ISR_HTIF4_Msk                               /*!< Channel 4 Half Transfer flag */
#define DMA_ISR_TEIF4_Pos      (15UL)
#define DMA_ISR_TEIF4_Msk      (0x1UL << DMA_ISR_TEIF4_Pos)                    /*!< 0x00008000 */
#define DMA_ISR_TEIF4          DMA_ISR_TEIF4_Msk                               /*!< Channel 4 Transfer Error flag */
#define DMA_ISR_GIF5_Pos       (16UL)
#define DMA_ISR_GIF5_Msk       (0x1UL << DMA_ISR_GIF5_Pos)                     /*!< 0x00010000 */
#define DMA_ISR_GIF5           DMA_ISR_GIF5_Msk                                /*!< Channel 5 Global interrupt flag */
#define DMA_ISR_TCIF5_Pos      (17UL)
#define DMA_ISR_TCIF5_Msk      (0x1UL << DMA_ISR_TCIF5_Pos)                    /*!< 0x00020000 */
#define DMA_ISR_TCIF5          DMA_ISR_TCIF5_Msk                               /*!< Channel 5 Transfer Complete flag */
#define DMA_ISR_HTIF5_Pos      (18UL)
#define DMA_ISR_HTIF5_Msk      (0x1UL << DMA_ISR_HTIF5_Pos)                    /*!< 0x00040000 */
#define DMA_ISR_HTIF5          DMA_ISR_HTIF5_Msk                               /*!< Channel 5 Half Transfer flag */
#define DMA_ISR_TEIF5_Pos      (19UL)
#define DMA_ISR_TEIF5_Msk      (0x1UL << DMA_ISR_TEIF5_Pos)                    /*!< 0x00080000 */
#define DMA_ISR_TEIF5          DMA_ISR_TEIF5_Msk                               /*!< Channel 5 Transfer Error flag */
#define DMA_ISR_GIF6_Pos       (20UL)
#define DMA_ISR_GIF6_Msk       (0x1UL << DMA_ISR_GIF6_Pos)                     /*!< 0x00100000 */
#define DMA_ISR_GIF6           DMA_ISR_GIF6_Msk                                /*!< Channel 6 Global interrupt flag */
#define DMA_ISR_TCIF6_Pos      (21UL)
#define DMA_ISR_TCIF6_Msk      (0x1UL << DMA_ISR_TCIF6_Pos)                    /*!< 0x00200000 */
#define DMA_ISR_TCIF6          DMA_ISR_TCIF6_Msk                               /*!< Channel 6 Transfer Complete flag */
#define DMA_ISR_HTIF6_Pos      (22UL)
#define DMA_ISR_HTIF6_Msk      (0x1UL << DMA_ISR_HTIF6_Pos)                    /*!< 0x00400000 */
#define DMA_ISR_HTIF6          DMA_ISR_HTIF6_Msk                               /*!< Channel 6 Half Transfer flag */
#define DMA_ISR_TEIF6_Pos      (23UL)
#define DMA_ISR_TEIF6_Msk      (0x1UL << DMA_ISR_TEIF6_Pos)                    /*!< 0x00800000 */
#define DMA_ISR_TEIF6          DMA_ISR_TEIF6_Msk                               /*!< Channel 6 Transfer Error flag */
#define DMA_ISR_GIF7_Pos       (24UL)
#define DMA_ISR_GIF7_Msk       (0x1UL << DMA_ISR_GIF7_Pos)                     /*!< 0x01000000 */
#define DMA_ISR_GIF7           DMA_ISR_GIF7_Msk                                /*!< Channel 7 Global interrupt flag */
#define DMA_ISR_TCIF7_Pos      (25UL)
#define DMA_ISR_TCIF7_Msk      (0x1UL << DMA_ISR_TCIF7_Pos)                    /*!< 0x02000000 */
#define DMA_ISR_TCIF7          DMA_ISR_TCIF7_Msk                               /*!< Channel 7 Transfer Complete flag */
#define DMA_ISR_HTIF7_Pos      (26UL)
#define DMA_ISR_HTIF7_Msk      (0x1UL << DMA_ISR_HTIF7_Pos)                    /*!< 0x04000000 */
#define DMA_ISR_HTIF7          DMA_ISR_HTIF7_Msk                               /*!< Channel 7 Half Transfer flag */
#define DMA_ISR_TEIF7_Pos      (27UL)
#define DMA_ISR_TEIF7_Msk      (0x1UL << DMA_ISR_TEIF7_Pos)                    /*!< 0x08000000 */
#define DMA_ISR_TEIF7          DMA_ISR_TEIF7_Msk                               /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define DMA_IFCR_CGIF1_Pos     (0UL)
#define DMA_IFCR_CGIF1_Msk     (0x1UL << DMA_IFCR_CGIF1_Pos)                   /*!< 0x00000001 */
#define DMA_IFCR_CGIF1         DMA_IFCR_CGIF1_Msk                              /*!< Channel 1 Global interrupt clearr */
#define DMA_IFCR_CTCIF1_Pos    (1UL)
#define DMA_IFCR_CTCIF1_Msk    (0x1UL << DMA_IFCR_CTCIF1_Pos)                  /*!< 0x00000002 */
#define DMA_IFCR_CTCIF1        DMA_IFCR_CTCIF1_Msk                             /*!< Channel 1 Transfer Complete clear */
#define DMA_IFCR_CHTIF1_Pos    (2UL)
#define DMA_IFCR_CHTIF1_Msk    (0x1UL << DMA_IFCR_CHTIF1_Pos)                  /*!< 0x00000004 */
#define DMA_IFCR_CHTIF1        DMA_IFCR_CHTIF1_Msk                             /*!< Channel 1 Half Transfer clear */
#define DMA_IFCR_CTEIF1_Pos    (3UL)
#define DMA_IFCR_CTEIF1_Msk    (0x1UL << DMA_IFCR_CTEIF1_Pos)                  /*!< 0x00000008 */
#define DMA_IFCR_CTEIF1        DMA_IFCR_CTEIF1_Msk                             /*!< Channel 1 Transfer Error clear */
#define DMA_IFCR_CGIF2_Pos     (4UL)
#define DMA_IFCR_CGIF2_Msk     (0x1UL << DMA_IFCR_CGIF2_Pos)                   /*!< 0x00000010 */
#define DMA_IFCR_CGIF2         DMA_IFCR_CGIF2_Msk                              /*!< Channel 2 Global interrupt clear */
#define DMA_IFCR_CTCIF2_Pos    (5UL)
#define DMA_IFCR_CTCIF2_Msk    (0x1UL << DMA_IFCR_CTCIF2_Pos)                  /*!< 0x00000020 */
#define DMA_IFCR_CTCIF2        DMA_IFCR_CTCIF2_Msk                             /*!< Channel 2 Transfer Complete clear */
#define DMA_IFCR_CHTIF2_Pos    (6UL)
#define DMA_IFCR_CHTIF2_Msk    (0x1UL << DMA_IFCR_CHTIF2_Pos)                  /*!< 0x00000040 */
#define DMA_IFCR_CHTIF2        DMA_IFCR_CHTIF2_Msk                             /*!< Channel 2 Half Transfer clear */
#define DMA_IFCR_CTEIF2_Pos    (7UL)
#define DMA_IFCR_CTEIF2_Msk    (0x1UL << DMA_IFCR_CTEIF2_Pos)                  /*!< 0x00000080 */
#define DMA_IFCR_CTEIF2        DMA_IFCR_CTEIF2_Msk                             /*!< Channel 2 Transfer Error clear */
#define DMA_IFCR_CGIF3_Pos     (8UL)
#define DMA_IFCR_CGIF3_Msk     (0x1UL << DMA_IFCR_CGIF3_Pos)                   /*!< 0x00000100 */
#define DMA_IFCR_CGIF3         DMA_IFCR_CGIF3_Msk                              /*!< Channel 3 Global interrupt clear */
#define DMA_IFCR_CTCIF3_Pos    (9UL)
#define DMA_IFCR_CTCIF3_Msk    (0x1UL << DMA_IFCR_CTCIF3_Pos)                  /*!< 0x00000200 */
#define DMA_IFCR_CTCIF3        DMA_IFCR_CTCIF3_Msk                             /*!< Channel 3 Transfer Complete clear */
#define DMA_IFCR_CHTIF3_Pos    (10UL)
#define DMA_IFCR_CHTIF3_Msk    (0x1UL << DMA_IFCR_CHTIF3_Pos)                  /*!< 0x00000400 */
#define DMA_IFCR_CHTIF3        DMA_IFCR_CHTIF3_Msk                             /*!< Channel 3 Half Transfer clear */
#define DMA_IFCR_CTEIF3_Pos    (11UL)
#define DMA_IFCR_CTEIF3_Msk    (0x1UL << DMA_IFCR_CTEIF3_Pos)                  /*!< 0x00000800 */
#define DMA_IFCR_CTEIF3        DMA_IFCR_CTEIF3_Msk                             /*!< Channel 3 Transfer Error clear */
#define DMA_IFCR_CGIF4_Pos     (12UL)
#define DMA_IFCR_CGIF4_Msk     (0x1UL << DMA_IFCR_CGIF4_Pos)                   /*!< 0x00001000 */
#define DMA_IFCR_CGIF4         DMA_IFCR_CGIF4_Msk                              /*!< Channel 4 Global interrupt clear */
#define DMA_IFCR_CTCIF4_Pos    (13UL)
#define DMA_IFCR_CTCIF4_Msk    (0x1UL << DMA_IFCR_CTCIF4_Pos)                  /*!< 0x00002000 */
#define DMA_IFCR_CTCIF4        DMA_IFCR_CTCIF4_Msk                             /*!< Channel 4 Transfer Complete clear */
#define DMA_IFCR_CHTIF4_Pos    (14UL)
#define DMA_IFCR_CHTIF4_Msk    (0x1UL << DMA_IFCR_CHTIF4_Pos)                  /*!< 0x00004000 */
#define DMA_IFCR_CHTIF4        DMA_IFCR_CHTIF4_Msk                             /*!< Channel 4 Half Transfer clear */
#define DMA_IFCR_CTEIF4_Pos    (15UL)
#define DMA_IFCR_CTEIF4_Msk    (0x1UL << DMA_IFCR_CTEIF4_Pos)                  /*!< 0x00008000 */
#define DMA_IFCR_CTEIF4        DMA_IFCR_CTEIF4_Msk                             /*!< Channel 4 Transfer Error clear */
#define DMA_IFCR_CGIF5_Pos     (16UL)
#define DMA_IFCR_CGIF5_Msk     (0x1UL << DMA_IFCR_CGIF5_Pos)                   /*!< 0x00010000 */
#define DMA_IFCR_CGIF5         DMA_IFCR_CGIF5_Msk                              /*!< Channel 5 Global interrupt clear */
#define DMA_IFCR_CTCIF5_Pos    (17UL)
#define DMA_IFCR_CTCIF5_Msk    (0x1UL << DMA_IFCR_CTCIF5_Pos)                  /*!< 0x00020000 */
#define DMA_IFCR_CTCIF5        DMA_IFCR_CTCIF5_Msk                             /*!< Channel 5 Transfer Complete clear */
#define DMA_IFCR_CHTIF5_Pos    (18UL)
#define DMA_IFCR_CHTIF5_Msk    (0x1UL << DMA_IFCR_CHTIF5_Pos)                  /*!< 0x00040000 */
#define DMA_IFCR_CHTIF5        DMA_IFCR_CHTIF5_Msk                             /*!< Channel 5 Half Transfer clear */
#define DMA_IFCR_CTEIF5_Pos    (19UL)
#define DMA_IFCR_CTEIF5_Msk    (0x1UL << DMA_IFCR_CTEIF5_Pos)                  /*!< 0x00080000 */
#define DMA_IFCR_CTEIF5        DMA_IFCR_CTEIF5_Msk                             /*!< Channel 5 Transfer Error clear */
#define DMA_IFCR_CGIF6_Pos     (20UL)
#define DMA_IFCR_CGIF6_Msk     (0x1UL << DMA_IFCR_CGIF6_Pos)                   /*!< 0x00100000 */
#define DMA_IFCR_CGIF6         DMA_IFCR_CGIF6_Msk                              /*!< Channel 6 Global interrupt clear */
#define DMA_IFCR_CTCIF6_Pos    (21UL)
#define DMA_IFCR_CTCIF6_Msk    (0x1UL << DMA_IFCR_CTCIF6_Pos)                  /*!< 0x00200000 */
#define DMA_IFCR_CTCIF6        DMA_IFCR_CTCIF6_Msk                             /*!< Channel 6 Transfer Complete clear */
#define DMA_IFCR_CHTIF6_Pos    (22UL)
#define DMA_IFCR_CHTIF6_Msk    (0x1UL << DMA_IFCR_CHTIF6_Pos)                  /*!< 0x00400000 */
#define DMA_IFCR_CHTIF6        DMA_IFCR_CHTIF6_Msk                             /*!< Channel 6 Half Transfer clear */
#define DMA_IFCR_CTEIF6_Pos    (23UL)
#define DMA_IFCR_CTEIF6_Msk    (0x1UL << DMA_IFCR_CTEIF6_Pos)                  /*!< 0x00800000 */
#define DMA_IFCR_CTEIF6        DMA_IFCR_CTEIF6_Msk                             /*!< Channel 6 Transfer Error clear */
#define DMA_IFCR_CGIF7_Pos     (24UL)
#define DMA_IFCR_CGIF7_Msk     (0x1UL << DMA_IFCR_CGIF7_Pos)                   /*!< 0x01000000 */
#define DMA_IFCR_CGIF7         DMA_IFCR_CGIF7_Msk                              /*!< Channel 7 Global interrupt clear */
#define DMA_IFCR_CTCIF7_Pos    (25UL)
#define DMA_IFCR_CTCIF7_Msk    (0x1UL << DMA_IFCR_CTCIF7_Pos)                  /*!< 0x02000000 */
#define DMA_IFCR_CTCIF7        DMA_IFCR_CTCIF7_Msk                             /*!< Channel 7 Transfer Complete clear */
#define DMA_IFCR_CHTIF7_Pos    (26UL)
#define DMA_IFCR_CHTIF7_Msk    (0x1UL << DMA_IFCR_CHTIF7_Pos)                  /*!< 0x04000000 */
#define DMA_IFCR_CHTIF7        DMA_IFCR_CHTIF7_Msk                             /*!< Channel 7 Half Transfer clear */
#define DMA_IFCR_CTEIF7_Pos    (27UL)
#define DMA_IFCR_CTEIF7_Msk    (0x1UL << DMA_IFCR_CTEIF7_Pos)                  /*!< 0x08000000 */
#define DMA_IFCR_CTEIF7        DMA_IFCR_CTEIF7_Msk                             /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR register  ********************/
#define DMA_CCR_EN_Pos         (0UL)
#define DMA_CCR_EN_Msk         (0x1UL << DMA_CCR_EN_Pos)                       /*!< 0x00000001 */
#define DMA_CCR_EN             DMA_CCR_EN_Msk                                  /*!< Channel enable                      */
#define DMA_CCR_TCIE_Pos       (1UL)
#define DMA_CCR_TCIE_Msk       (0x1UL << DMA_CCR_TCIE_Pos)                     /*!< 0x00000002 */
#define DMA_CCR_TCIE           DMA_CCR_TCIE_Msk                                /*!< Transfer complete interrupt enable  */
#define DMA_CCR_HTIE_Pos       (2UL)
#define DMA_CCR_HTIE_Msk       (0x1UL << DMA_CCR_HTIE_Pos)                     /*!< 0x00000004 */
#define DMA_CCR_HTIE           DMA_CCR_HTIE_Msk                                /*!< Half Transfer interrupt enable      */
#define DMA_CCR_TEIE_Pos       (3UL)
#define DMA_CCR_TEIE_Msk       (0x1UL << DMA_CCR_TEIE_Pos)                     /*!< 0x00000008 */
#define DMA_CCR_TEIE           DMA_CCR_TEIE_Msk                                /*!< Transfer error interrupt enable     */
#define DMA_CCR_DIR_Pos        (4UL)
#define DMA_CCR_DIR_Msk        (0x1UL << DMA_CCR_DIR_Pos)                      /*!< 0x00000010 */
#define DMA_CCR_DIR            DMA_CCR_DIR_Msk                                 /*!< Data transfer direction             */
#define DMA_CCR_CIRC_Pos       (5UL)
#define DMA_CCR_CIRC_Msk       (0x1UL << DMA_CCR_CIRC_Pos)                     /*!< 0x00000020 */
#define DMA_CCR_CIRC           DMA_CCR_CIRC_Msk                                /*!< Circular mode                       */
#define DMA_CCR_PINC_Pos       (6UL)
#define DMA_CCR_PINC_Msk       (0x1UL << DMA_CCR_PINC_Pos)                     /*!< 0x00000040 */
#define DMA_CCR_PINC           DMA_CCR_PINC_Msk                                /*!< Peripheral increment mode           */
#define DMA_CCR_MINC_Pos       (7UL)
#define DMA_CCR_MINC_Msk       (0x1UL << DMA_CCR_MINC_Pos)                     /*!< 0x00000080 */
#define DMA_CCR_MINC           DMA_CCR_MINC_Msk                                /*!< Memory increment mode               */

#define DMA_CCR_PSIZE_Pos      (8UL)
#define DMA_CCR_PSIZE_Msk      (0x3UL << DMA_CCR_PSIZE_Pos)                    /*!< 0x00000300 */
#define DMA_CCR_PSIZE          DMA_CCR_PSIZE_Msk                               /*!< PSIZE[1:0] bits (Peripheral size)   */
#define DMA_CCR_PSIZE_0        (0x1UL << DMA_CCR_PSIZE_Pos)                    /*!< 0x00000100 */
#define DMA_CCR_PSIZE_1        (0x2UL << DMA_CCR_PSIZE_Pos)                    /*!< 0x00000200 */

#define DMA_CCR_MSIZE_Pos      (10UL)
#define DMA_CCR_MSIZE_Msk      (0x3UL << DMA_CCR_MSIZE_Pos)                    /*!< 0x00000C00 */
#define DMA_CCR_MSIZE          DMA_CCR_MSIZE_Msk                               /*!< MSIZE[1:0] bits (Memory size)       */
#define DMA_CCR_MSIZE_0        (0x1UL << DMA_CCR_MSIZE_Pos)                    /*!< 0x00000400 */
#define DMA_CCR_MSIZE_1        (0x2UL << DMA_CCR_MSIZE_Pos)                    /*!< 0x00000800 */

#define DMA_CCR_PL_Pos         (12UL)
#define DMA_CCR_PL_Msk         (0x3UL << DMA_CCR_PL_Pos)                       /*!< 0x00003000 */
#define DMA_CCR_PL             DMA_CCR_PL_Msk                                  /*!< PL[1:0] bits(Channel Priority level)*/
#define DMA_CCR_PL_0           (0x1UL << DMA_CCR_PL_Pos)                       /*!< 0x00001000 */
#define DMA_CCR_PL_1           (0x2UL << DMA_CCR_PL_Pos)                        /*!< 0x00002000 */

#define DMA_CCR_MEM2MEM_Pos    (14UL)
#define DMA_CCR_MEM2MEM_Msk    (0x1UL << DMA_CCR_MEM2MEM_Pos)                  /*!< 0x00004000 */
#define DMA_CCR_MEM2MEM        DMA_CCR_MEM2MEM_Msk                             /*!< Memory to memory mode               */

/******************  Bit definition for DMA_CNDTR register  *******************/
#define DMA_CNDTR_NDT_Pos      (0UL)
#define DMA_CNDTR_NDT_Msk      (0xFFFFUL << DMA_CNDTR_NDT_Pos)                 /*!< 0x0000FFFF */
#define DMA_CNDTR_NDT          DMA_CNDTR_NDT_Msk                               /*!< Number of data to Transfer          */

/******************  Bit definition for DMA_CPAR register  ********************/
#define DMA_CPAR_PA_Pos        (0UL)
#define DMA_CPAR_PA_Msk        (0xFFFFFFFFUL << DMA_CPAR_PA_Pos)               /*!< 0xFFFFFFFF */
#define DMA_CPAR_PA            DMA_CPAR_PA_Msk                                 /*!< Peripheral Address                  */

/******************  Bit definition for DMA_CMAR register  ********************/
#define DMA_CMAR_MA_Pos        (0UL)
#define DMA_CMAR_MA_Msk        (0xFFFFFFFFUL << DMA_CMAR_MA_Pos)               /*!< 0xFFFFFFFF */
#define DMA_CMAR_MA            DMA_CMAR_MA_Msk                                 /*!< Memory Address                      */

/******************************************************************************/
/*                                                                            */
/*                             DMAMUX Controller                              */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DMAMUX_CxCR register  **************/
#define DMAMUX_CxCR_DMAREQ_ID_Pos              (0UL)
#define DMAMUX_CxCR_DMAREQ_ID_Msk              (0xFFUL << DMAMUX_CxCR_DMAREQ_ID_Pos) /*!< 0x000000FF */
#define DMAMUX_CxCR_DMAREQ_ID                  DMAMUX_CxCR_DMAREQ_ID_Msk             /*!< DMA Request ID   */
#define DMAMUX_CxCR_DMAREQ_ID_0                (0x01UL << DMAMUX_CxCR_DMAREQ_ID_Pos) /*!< 0x00000001 */
#define DMAMUX_CxCR_DMAREQ_ID_1                (0x02UL << DMAMUX_CxCR_DMAREQ_ID_Pos) /*!< 0x00000002 */
#define DMAMUX_CxCR_DMAREQ_ID_2                (0x04UL << DMAMUX_CxCR_DMAREQ_ID_Pos) /*!< 0x00000004 */
#define DMAMUX_CxCR_DMAREQ_ID_3                (0x08UL << DMAMUX_CxCR_DMAREQ_ID_Pos) /*!< 0x00000008 */
#define DMAMUX_CxCR_DMAREQ_ID_4                (0x10UL << DMAMUX_CxCR_DMAREQ_ID_Pos) /*!< 0x00000010 */
#define DMAMUX_CxCR_DMAREQ_ID_5                (0x20UL << DMAMUX_CxCR_DMAREQ_ID_Pos) /*!< 0x00000020 */
#define DMAMUX_CxCR_DMAREQ_ID_6                (0x40UL << DMAMUX_CxCR_DMAREQ_ID_Pos) /*!< 0x00000040 */
#define DMAMUX_CxCR_DMAREQ_ID_7                (0x80UL << DMAMUX_CxCR_DMAREQ_ID_Pos) /*!< 0x00000080 */
#define DMAMUX_CxCR_SOIE_Pos                   (8UL)
#define DMAMUX_CxCR_SOIE_Msk                   (0x1UL << DMAMUX_CxCR_SOIE_Pos)  /*!< 0x00000100 */
#define DMAMUX_CxCR_SOIE                       DMAMUX_CxCR_SOIE_Msk             /*!< Synchro overrun interrupt enable     */
#define DMAMUX_CxCR_EGE_Pos                    (9UL)
#define DMAMUX_CxCR_EGE_Msk                    (0x1UL << DMAMUX_CxCR_EGE_Pos)   /*!< 0x00000200 */
#define DMAMUX_CxCR_EGE                        DMAMUX_CxCR_EGE_Msk              /*!< Event generation interrupt enable    */
#define DMAMUX_CxCR_SE_Pos                     (16UL)
#define DMAMUX_CxCR_SE_Msk                     (0x1UL << DMAMUX_CxCR_SE_Pos)    /*!< 0x00010000 */
#define DMAMUX_CxCR_SE                         DMAMUX_CxCR_SE_Msk               /*!< Synchronization enable               */
#define DMAMUX_CxCR_SPOL_Pos                   (17UL)
#define DMAMUX_CxCR_SPOL_Msk                   (0x3UL << DMAMUX_CxCR_SPOL_Pos)  /*!< 0x00060000 */
#define DMAMUX_CxCR_SPOL                       DMAMUX_CxCR_SPOL_Msk             /*!< Synchronization polarity             */
#define DMAMUX_CxCR_SPOL_0                     (0x1UL << DMAMUX_CxCR_SPOL_Pos)  /*!< 0x00020000 */
#define DMAMUX_CxCR_SPOL_1                     (0x2UL << DMAMUX_CxCR_SPOL_Pos)  /*!< 0x00040000 */
#define DMAMUX_CxCR_NBREQ_Pos                  (19UL)
#define DMAMUX_CxCR_NBREQ_Msk                  (0x1FUL << DMAMUX_CxCR_NBREQ_Pos) /*!< 0x00F80000 */
#define DMAMUX_CxCR_NBREQ                      DMAMUX_CxCR_NBREQ_Msk             /*!< Number of request                    */
#define DMAMUX_CxCR_NBREQ_0                    (0x01UL << DMAMUX_CxCR_NBREQ_Pos) /*!< 0x00080000 */
#define DMAMUX_CxCR_NBREQ_1                    (0x02UL << DMAMUX_CxCR_NBREQ_Pos) /*!< 0x00100000 */
#define DMAMUX_CxCR_NBREQ_2                    (0x04UL << DMAMUX_CxCR_NBREQ_Pos) /*!< 0x00200000 */
#define DMAMUX_CxCR_NBREQ_3                    (0x08UL << DMAMUX_CxCR_NBREQ_Pos) /*!< 0x00400000 */
#define DMAMUX_CxCR_NBREQ_4                    (0x10UL << DMAMUX_CxCR_NBREQ_Pos) /*!< 0x00800000 */
#define DMAMUX_CxCR_SYNC_ID_Pos                (24UL)
#define DMAMUX_CxCR_SYNC_ID_Msk                (0x1FUL << DMAMUX_CxCR_SYNC_ID_Pos) /*!< 0x1F000000 */
#define DMAMUX_CxCR_SYNC_ID                    DMAMUX_CxCR_SYNC_ID_Msk             /*!< Synchronization ID                   */
#define DMAMUX_CxCR_SYNC_ID_0                  (0x01UL << DMAMUX_CxCR_SYNC_ID_Pos) /*!< 0x01000000 */
#define DMAMUX_CxCR_SYNC_ID_1                  (0x02UL << DMAMUX_CxCR_SYNC_ID_Pos) /*!< 0x02000000 */
#define DMAMUX_CxCR_SYNC_ID_2                  (0x04UL << DMAMUX_CxCR_SYNC_ID_Pos) /*!< 0x04000000 */
#define DMAMUX_CxCR_SYNC_ID_3                  (0x08UL << DMAMUX_CxCR_SYNC_ID_Pos) /*!< 0x08000000 */
#define DMAMUX_CxCR_SYNC_ID_4                  (0x10UL << DMAMUX_CxCR_SYNC_ID_Pos) /*!< 0x10000000 */

/*******************  Bits definition for DMAMUX_CSR register  **************/
#define DMAMUX_CSR_SOF0_Pos                          (0UL)
#define DMAMUX_CSR_SOF0_Msk                          (0x1UL << DMAMUX_CSR_SOF0_Pos) /*!< 0x00000001 */
#define DMAMUX_CSR_SOF0                              DMAMUX_CSR_SOF0_Msk
#define DMAMUX_CSR_SOF1_Pos                          (1UL)
#define DMAMUX_CSR_SOF1_Msk                          (0x1UL << DMAMUX_CSR_SOF1_Pos) /*!< 0x00000002 */
#define DMAMUX_CSR_SOF1                              DMAMUX_CSR_SOF1_Msk
#define DMAMUX_CSR_SOF2_Pos                          (2UL)
#define DMAMUX_CSR_SOF2_Msk                          (0x1UL << DMAMUX_CSR_SOF2_Pos) /*!< 0x00000004 */
#define DMAMUX_CSR_SOF2                              DMAMUX_CSR_SOF2_Msk
#define DMAMUX_CSR_SOF3_Pos                          (3UL)
#define DMAMUX_CSR_SOF3_Msk                          (0x1UL << DMAMUX_CSR_SOF3_Pos) /*!< 0x00000008 */
#define DMAMUX_CSR_SOF3                              DMAMUX_CSR_SOF3_Msk
#define DMAMUX_CSR_SOF4_Pos                          (4UL)
#define DMAMUX_CSR_SOF4_Msk                          (0x1UL << DMAMUX_CSR_SOF4_Pos) /*!< 0x00000010 */
#define DMAMUX_CSR_SOF4                              DMAMUX_CSR_SOF4_Msk
#define DMAMUX_CSR_SOF5_Pos                          (5UL)
#define DMAMUX_CSR_SOF5_Msk                          (0x1UL << DMAMUX_CSR_SOF5_Pos) /*!< 0x00000020 */
#define DMAMUX_CSR_SOF5                              DMAMUX_CSR_SOF5_Msk
#define DMAMUX_CSR_SOF6_Pos                          (6UL)
#define DMAMUX_CSR_SOF6_Msk                          (0x1UL << DMAMUX_CSR_SOF6_Pos) /*!< 0x00000040 */
#define DMAMUX_CSR_SOF6                              DMAMUX_CSR_SOF6_Msk
#define DMAMUX_CSR_SOF7_Pos                          (7UL)
#define DMAMUX_CSR_SOF7_Msk                          (0x1UL << DMAMUX_CSR_SOF7_Pos) /*!< 0x00000080 */
#define DMAMUX_CSR_SOF7                              DMAMUX_CSR_SOF7_Msk
#define DMAMUX_CSR_SOF8_Pos                          (8UL)
#define DMAMUX_CSR_SOF8_Msk                          (0x1UL << DMAMUX_CSR_SOF8_Pos) /*!< 0x00000100 */
#define DMAMUX_CSR_SOF8                              DMAMUX_CSR_SOF8_Msk
#define DMAMUX_CSR_SOF9_Pos                          (9UL)
#define DMAMUX_CSR_SOF9_Msk                          (0x1UL << DMAMUX_CSR_SOF9_Pos) /*!< 0x00000200 */
#define DMAMUX_CSR_SOF9                              DMAMUX_CSR_SOF9_Msk
#define DMAMUX_CSR_SOF10_Pos                         (10UL)
#define DMAMUX_CSR_SOF10_Msk                         (0x1UL << DMAMUX_CSR_SOF10_Pos) /*!< 0x00000400 */
#define DMAMUX_CSR_SOF10                             DMAMUX_CSR_SOF10_Msk
#define DMAMUX_CSR_SOF11_Pos                         (11UL)
#define DMAMUX_CSR_SOF11_Msk                         (0x1UL << DMAMUX_CSR_SOF11_Pos) /*!< 0x00000800 */
#define DMAMUX_CSR_SOF11                              DMAMUX_CSR_SOF11_Msk

/********************  Bits definition for DMAMUX_CFR register  **************/
#define DMAMUX_CFR_CSOF0_Pos                         (0UL)
#define DMAMUX_CFR_CSOF0_Msk                         (0x1UL << DMAMUX_CFR_CSOF0_Pos) /*!< 0x00000001 */
#define DMAMUX_CFR_CSOF0                             DMAMUX_CFR_CSOF0_Msk
#define DMAMUX_CFR_CSOF1_Pos                         (1UL)
#define DMAMUX_CFR_CSOF1_Msk                         (0x1UL << DMAMUX_CFR_CSOF1_Pos) /*!< 0x00000002 */
#define DMAMUX_CFR_CSOF1                             DMAMUX_CFR_CSOF1_Msk
#define DMAMUX_CFR_CSOF2_Pos                         (2UL)
#define DMAMUX_CFR_CSOF2_Msk                         (0x1UL << DMAMUX_CFR_CSOF2_Pos) /*!< 0x00000004 */
#define DMAMUX_CFR_CSOF2                             DMAMUX_CFR_CSOF2_Msk
#define DMAMUX_CFR_CSOF3_Pos                         (3UL)
#define DMAMUX_CFR_CSOF3_Msk                         (0x1UL << DMAMUX_CFR_CSOF3_Pos) /*!< 0x00000008 */
#define DMAMUX_CFR_CSOF3                             DMAMUX_CFR_CSOF3_Msk
#define DMAMUX_CFR_CSOF4_Pos                         (4UL)
#define DMAMUX_CFR_CSOF4_Msk                         (0x1UL << DMAMUX_CFR_CSOF4_Pos) /*!< 0x00000010 */
#define DMAMUX_CFR_CSOF4                             DMAMUX_CFR_CSOF4_Msk
#define DMAMUX_CFR_CSOF5_Pos                         (5UL)
#define DMAMUX_CFR_CSOF5_Msk                         (0x1UL << DMAMUX_CFR_CSOF5_Pos) /*!< 0x00000020 */
#define DMAMUX_CFR_CSOF5                             DMAMUX_CFR_CSOF5_Msk
#define DMAMUX_CFR_CSOF6_Pos                         (6UL)
#define DMAMUX_CFR_CSOF6_Msk                         (0x1UL << DMAMUX_CFR_CSOF6_Pos) /*!< 0x00000040 */
#define DMAMUX_CFR_CSOF6                             DMAMUX_CFR_CSOF6_Msk
#define DMAMUX_CFR_CSOF7_Pos                         (7UL)
#define DMAMUX_CFR_CSOF7_Msk                         (0x1UL << DMAMUX_CFR_CSOF7_Pos) /*!< 0x00000080 */
#define DMAMUX_CFR_CSOF7                             DMAMUX_CFR_CSOF7_Msk
#define DMAMUX_CFR_CSOF8_Pos                         (8UL)
#define DMAMUX_CFR_CSOF8_Msk                         (0x1UL << DMAMUX_CFR_CSOF8_Pos) /*!< 0x00000100 */
#define DMAMUX_CFR_CSOF8                             DMAMUX_CFR_CSOF8_Msk
#define DMAMUX_CFR_CSOF9_Pos                         (9UL)
#define DMAMUX_CFR_CSOF9_Msk                         (0x1UL << DMAMUX_CFR_CSOF9_Pos) /*!< 0x00000200 */
#define DMAMUX_CFR_CSOF9                             DMAMUX_CFR_CSOF9_Msk
#define DMAMUX_CFR_CSOF10_Pos                        (10UL)
#define DMAMUX_CFR_CSOF10_Msk                        (0x1UL << DMAMUX_CFR_CSOF10_Pos) /*!< 0x00000400 */
#define DMAMUX_CFR_CSOF10                            DMAMUX_CFR_CSOF10_Msk
#define DMAMUX_CFR_CSOF11_Pos                        (11UL)
#define DMAMUX_CFR_CSOF11_Msk                        (0x1UL << DMAMUX_CFR_CSOF11_Pos) /*!< 0x00000800 */
#define DMAMUX_CFR_CSOF11                            DMAMUX_CFR_CSOF11_Msk

/********************  Bits definition for DMAMUX_RGxCR register  ************/
#define DMAMUX_RGxCR_SIG_ID_Pos                (0UL)
#define DMAMUX_RGxCR_SIG_ID_Msk                (0x1FUL << DMAMUX_RGxCR_SIG_ID_Pos) /*!< 0x0000001F */
#define DMAMUX_RGxCR_SIG_ID                    DMAMUX_RGxCR_SIG_ID_Msk             /*!< Signal ID                         */
#define DMAMUX_RGxCR_SIG_ID_0                  (0x01UL << DMAMUX_RGxCR_SIG_ID_Pos) /*!< 0x00000001 */
#define DMAMUX_RGxCR_SIG_ID_1                  (0x02UL << DMAMUX_RGxCR_SIG_ID_Pos) /*!< 0x00000002 */
#define DMAMUX_RGxCR_SIG_ID_2                  (0x04UL << DMAMUX_RGxCR_SIG_ID_Pos) /*!< 0x00000004 */
#define DMAMUX_RGxCR_SIG_ID_3                  (0x08UL << DMAMUX_RGxCR_SIG_ID_Pos) /*!< 0x00000008 */
#define DMAMUX_RGxCR_SIG_ID_4                  (0x10UL << DMAMUX_RGxCR_SIG_ID_Pos) /*!< 0x00000010 */
#define DMAMUX_RGxCR_OIE_Pos                   (8UL)
#define DMAMUX_RGxCR_OIE_Msk                   (0x1UL << DMAMUX_RGxCR_OIE_Pos)  /*!< 0x00000100 */
#define DMAMUX_RGxCR_OIE                       DMAMUX_RGxCR_OIE_Msk             /*!< Overrun interrupt enable             */
#define DMAMUX_RGxCR_GE_Pos                    (16UL)
#define DMAMUX_RGxCR_GE_Msk                    (0x1UL << DMAMUX_RGxCR_GE_Pos)   /*!< 0x00010000 */
#define DMAMUX_RGxCR_GE                        DMAMUX_RGxCR_GE_Msk              /*!< Generation enable                    */
#define DMAMUX_RGxCR_GPOL_Pos                  (17UL)
#define DMAMUX_RGxCR_GPOL_Msk                  (0x3UL << DMAMUX_RGxCR_GPOL_Pos) /*!< 0x00060000 */
#define DMAMUX_RGxCR_GPOL                      DMAMUX_RGxCR_GPOL_Msk            /*!< Generation polarity                  */
#define DMAMUX_RGxCR_GPOL_0                    (0x1UL << DMAMUX_RGxCR_GPOL_Pos) /*!< 0x00020000 */
#define DMAMUX_RGxCR_GPOL_1                    (0x2UL << DMAMUX_RGxCR_GPOL_Pos) /*!< 0x00040000 */
#define DMAMUX_RGxCR_GNBREQ_Pos                (19UL)
#define DMAMUX_RGxCR_GNBREQ_Msk                (0x1FUL << DMAMUX_RGxCR_GNBREQ_Pos) /*!< 0x00F80000 */
#define DMAMUX_RGxCR_GNBREQ                    DMAMUX_RGxCR_GNBREQ_Msk             /*!< Number of request                 */
#define DMAMUX_RGxCR_GNBREQ_0                  (0x01UL << DMAMUX_RGxCR_GNBREQ_Pos) /*!< 0x00080000 */
#define DMAMUX_RGxCR_GNBREQ_1                  (0x02UL << DMAMUX_RGxCR_GNBREQ_Pos) /*!< 0x00100000 */
#define DMAMUX_RGxCR_GNBREQ_2                  (0x04UL << DMAMUX_RGxCR_GNBREQ_Pos) /*!< 0x00200000 */
#define DMAMUX_RGxCR_GNBREQ_3                  (0x08UL << DMAMUX_RGxCR_GNBREQ_Pos) /*!< 0x00400000 */
#define DMAMUX_RGxCR_GNBREQ_4                  (0x10UL << DMAMUX_RGxCR_GNBREQ_Pos) /*!< 0x00800000 */

/********************  Bits definition for DMAMUX_RGSR register  **************/
#define DMAMUX_RGSR_OF0_Pos                    (0UL)
#define DMAMUX_RGSR_OF0_Msk                    (0x1UL << DMAMUX_RGSR_OF0_Pos)   /*!< 0x00000001 */
#define DMAMUX_RGSR_OF0                        DMAMUX_RGSR_OF0_Msk              /*!< Overrun flag 0                       */
#define DMAMUX_RGSR_OF1_Pos                    (1UL)
#define DMAMUX_RGSR_OF1_Msk                    (0x1UL << DMAMUX_RGSR_OF1_Pos)   /*!< 0x00000002 */
#define DMAMUX_RGSR_OF1                        DMAMUX_RGSR_OF1_Msk              /*!< Overrun flag 1                       */
#define DMAMUX_RGSR_OF2_Pos                    (2UL)
#define DMAMUX_RGSR_OF2_Msk                    (0x1UL << DMAMUX_RGSR_OF2_Pos)   /*!< 0x00000004 */
#define DMAMUX_RGSR_OF2                        DMAMUX_RGSR_OF2_Msk              /*!< Overrun flag 2                       */
#define DMAMUX_RGSR_OF3_Pos                    (3UL)
#define DMAMUX_RGSR_OF3_Msk                    (0x1UL << DMAMUX_RGSR_OF3_Pos)   /*!< 0x00000008 */
#define DMAMUX_RGSR_OF3                        DMAMUX_RGSR_OF3_Msk              /*!< Overrun flag 3                       */

/********************  Bits definition for DMAMUX_RGCFR register  **************/
#define DMAMUX_RGCFR_COF0_Pos                  (0UL)
#define DMAMUX_RGCFR_COF0_Msk                  (0x1UL << DMAMUX_RGCFR_COF0_Pos) /*!< 0x00000001 */
#define DMAMUX_RGCFR_COF0                      DMAMUX_RGCFR_COF0_Msk            /*!< Clear Overrun flag 0                 */
#define DMAMUX_RGCFR_COF1_Pos                  (1UL)
#define DMAMUX_RGCFR_COF1_Msk                  (0x1UL << DMAMUX_RGCFR_COF1_Pos) /*!< 0x00000002 */
#define DMAMUX_RGCFR_COF1                      DMAMUX_RGCFR_COF1_Msk            /*!< Clear Overrun flag 1                 */
#define DMAMUX_RGCFR_COF2_Pos                  (2UL)
#define DMAMUX_RGCFR_COF2_Msk                  (0x1UL << DMAMUX_RGCFR_COF2_Pos) /*!< 0x00000004 */
#define DMAMUX_RGCFR_COF2                      DMAMUX_RGCFR_COF2_Msk            /*!< Clear Overrun flag 2                 */
#define DMAMUX_RGCFR_COF3_Pos                  (3UL)
#define DMAMUX_RGCFR_COF3_Msk                  (0x1UL << DMAMUX_RGCFR_COF3_Pos) /*!< 0x00000008 */
#define DMAMUX_RGCFR_COF3                      DMAMUX_RGCFR_COF3_Msk            /*!< Clear Overrun flag 3                 */

/*****************  Bits definition for DMAMUX_IPHW_CFGR2 register  ************/
#define DMAMUX_IPHW_CFGR2_NB_EXT_REQ_Pos       (0UL)
#define DMAMUX_IPHW_CFGR2_NB_EXT_REQ_Msk       (0xFFUL << DMAMUX_IPHW_CFGR2_NB_EXT_REQ_Pos) /*!< 0x000000FF */
#define DMAMUX_IPHW_CFGR2_NB_EXT_REQ           DMAMUX_IPHW_CFGR2_NB_EXT_REQ_Msk /*!< Number of external request sources   */

/*****************  Bits definition for DMAMUX_IPHW_CFGR1 register  ************/
#define DMAMUX_IPHW_CFGR1_NB_STREAMS_Pos       (0UL)
#define DMAMUX_IPHW_CFGR1_NB_STREAMS_Msk       (0xFFUL << DMAMUX_IPHW_CFGR1_NB_STREAMS_Pos) /*!< 0x000000FF */
#define DMAMUX_IPHW_CFGR1_NB_STREAMS           DMAMUX_IPHW_CFGR1_NB_STREAMS_Msk /*!< Number of DMA streams                */

#define DMAMUX_IPHW_CFGR1_NB_PERIPH_REQ_Pos    (8UL)
#define DMAMUX_IPHW_CFGR1_NB_PERIPH_REQ_Msk    (0xFFUL << DMAMUX_IPHW_CFGR1_NB_PERIPH_REQ_Pos) /*!< 0x0000FF00 */
#define DMAMUX_IPHW_CFGR1_NB_PERIPH_REQ        DMAMUX_IPHW_CFGR1_NB_PERIPH_REQ_Msk /*!< Number of peripheral requests     */

#define DMAMUX_IPHW_CFGR1_NB_SYNC_TRIG_Pos     (16UL)
#define DMAMUX_IPHW_CFGR1_NB_SYNC_TRIG_Msk     (0xFFUL << DMAMUX_IPHW_CFGR1_NB_SYNC_TRIG_Pos) /*!< 0x00FF0000 */
#define DMAMUX_IPHW_CFGR1_NB_SYNC_TRIG         DMAMUX_IPHW_CFGR1_NB_SYNC_TRIG_Msk /*!< Number of synchronization triggers */

#define DMAMUX_IPHW_CFGR1_NB_REQ_GEN_Pos       (24UL)
#define DMAMUX_IPHW_CFGR1_NB_REQ_GEN_Msk       (0xFFUL << DMAMUX_IPHW_CFGR1_NB_REQ_GEN_Pos) /*!< 0xFF000000 */
#define DMAMUX_IPHW_CFGR1_NB_REQ_GEN           DMAMUX_IPHW_CFGR1_NB_REQ_GEN_Msk /*!< Number of request generation blocks  */

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for EXTI_RTSR1 register  *******************/
#define EXTI_RTSR1_TR_Pos          (0UL)
#define EXTI_RTSR1_TR_Msk          (0x3FFFFFUL << EXTI_RTSR1_TR_Pos)           /*!< 0x003FFFFF */
#define EXTI_RTSR1_TR              EXTI_RTSR1_TR_Msk                           /*!< Rising trigger event configuration bit */
#define EXTI_RTSR1_TR0_Pos         (0UL)
#define EXTI_RTSR1_TR0_Msk         (0x1UL << EXTI_RTSR1_TR0_Pos)               /*!< 0x00000001 */
#define EXTI_RTSR1_TR0             EXTI_RTSR1_TR0_Msk                          /*!< Rising trigger event configuration bit of line 0 */
#define EXTI_RTSR1_TR1_Pos         (1UL)
#define EXTI_RTSR1_TR1_Msk         (0x1UL << EXTI_RTSR1_TR1_Pos)               /*!< 0x00000002 */
#define EXTI_RTSR1_TR1             EXTI_RTSR1_TR1_Msk                          /*!< Rising trigger event configuration bit of line 1 */
#define EXTI_RTSR1_TR2_Pos         (2UL)
#define EXTI_RTSR1_TR2_Msk         (0x1UL << EXTI_RTSR1_TR2_Pos)               /*!< 0x00000004 */
#define EXTI_RTSR1_TR2             EXTI_RTSR1_TR2_Msk                          /*!< Rising trigger event configuration bit of line 2 */
#define EXTI_RTSR1_TR3_Pos         (3UL)
#define EXTI_RTSR1_TR3_Msk         (0x1UL << EXTI_RTSR1_TR3_Pos)               /*!< 0x00000008 */
#define EXTI_RTSR1_TR3             EXTI_RTSR1_TR3_Msk                          /*!< Rising trigger event configuration bit of line 3 */
#define EXTI_RTSR1_TR4_Pos         (4UL)
#define EXTI_RTSR1_TR4_Msk         (0x1UL << EXTI_RTSR1_TR4_Pos)               /*!< 0x00000010 */
#define EXTI_RTSR1_TR4             EXTI_RTSR1_TR4_Msk                          /*!< Rising trigger event configuration bit of line 4 */
#define EXTI_RTSR1_TR5_Pos         (5UL)
#define EXTI_RTSR1_TR5_Msk         (0x1UL << EXTI_RTSR1_TR5_Pos)               /*!< 0x00000020 */
#define EXTI_RTSR1_TR5             EXTI_RTSR1_TR5_Msk                          /*!< Rising trigger event configuration bit of line 5 */
#define EXTI_RTSR1_TR6_Pos         (6UL)
#define EXTI_RTSR1_TR6_Msk         (0x1UL << EXTI_RTSR1_TR6_Pos)               /*!< 0x00000040 */
#define EXTI_RTSR1_TR6             EXTI_RTSR1_TR6_Msk                          /*!< Rising trigger event configuration bit of line 6 */
#define EXTI_RTSR1_TR7_Pos         (7UL)
#define EXTI_RTSR1_TR7_Msk         (0x1UL << EXTI_RTSR1_TR7_Pos)               /*!< 0x00000080 */
#define EXTI_RTSR1_TR7             EXTI_RTSR1_TR7_Msk                          /*!< Rising trigger event configuration bit of line 7 */
#define EXTI_RTSR1_TR8_Pos         (8UL)
#define EXTI_RTSR1_TR8_Msk         (0x1UL << EXTI_RTSR1_TR8_Pos)               /*!< 0x00000100 */
#define EXTI_RTSR1_TR8             EXTI_RTSR1_TR8_Msk                          /*!< Rising trigger event configuration bit of line 8 */
#define EXTI_RTSR1_TR9_Pos         (9UL)
#define EXTI_RTSR1_TR9_Msk         (0x1UL << EXTI_RTSR1_TR9_Pos)               /*!< 0x00000200 */
#define EXTI_RTSR1_TR9             EXTI_RTSR1_TR9_Msk                          /*!< Rising trigger event configuration bit of line 9 */
#define EXTI_RTSR1_TR10_Pos        (10UL)
#define EXTI_RTSR1_TR10_Msk        (0x1UL << EXTI_RTSR1_TR10_Pos)              /*!< 0x00000400 */
#define EXTI_RTSR1_TR10            EXTI_RTSR1_TR10_Msk                         /*!< Rising trigger event configuration bit of line 10 */
#define EXTI_RTSR1_TR11_Pos        (11UL)
#define EXTI_RTSR1_TR11_Msk        (0x1UL << EXTI_RTSR1_TR11_Pos)              /*!< 0x00000800 */
#define EXTI_RTSR1_TR11            EXTI_RTSR1_TR11_Msk                         /*!< Rising trigger event configuration bit of line 11 */
#define EXTI_RTSR1_TR12_Pos        (12UL)
#define EXTI_RTSR1_TR12_Msk        (0x1UL << EXTI_RTSR1_TR12_Pos)              /*!< 0x00001000 */
#define EXTI_RTSR1_TR12            EXTI_RTSR1_TR12_Msk                         /*!< Rising trigger event configuration bit of line 12 */
#define EXTI_RTSR1_TR13_Pos        (13UL)
#define EXTI_RTSR1_TR13_Msk        (0x1UL << EXTI_RTSR1_TR13_Pos)              /*!< 0x00002000 */
#define EXTI_RTSR1_TR13            EXTI_RTSR1_TR13_Msk                         /*!< Rising trigger event configuration bit of line 13 */
#define EXTI_RTSR1_TR14_Pos        (14UL)
#define EXTI_RTSR1_TR14_Msk        (0x1UL << EXTI_RTSR1_TR14_Pos)              /*!< 0x00004000 */
#define EXTI_RTSR1_TR14            EXTI_RTSR1_TR14_Msk                         /*!< Rising trigger event configuration bit of line 14 */
#define EXTI_RTSR1_TR15_Pos        (15UL)
#define EXTI_RTSR1_TR15_Msk        (0x1UL << EXTI_RTSR1_TR15_Pos)              /*!< 0x00008000 */
#define EXTI_RTSR1_TR15            EXTI_RTSR1_TR15_Msk                         /*!< Rising trigger event configuration bit of line 15 */
#define EXTI_RTSR1_TR16_Pos        (16UL)
#define EXTI_RTSR1_TR16_Msk        (0x1UL << EXTI_RTSR1_TR16_Pos)              /*!< 0x00010000 */
#define EXTI_RTSR1_TR16            EXTI_RTSR1_TR16_Msk                         /*!< Rising trigger event configuration bit of line 16 */
#define EXTI_RTSR1_TR17_Pos        (17UL)
#define EXTI_RTSR1_TR17_Msk        (0x1UL << EXTI_RTSR1_TR17_Pos)              /*!< 0x00020000 */
#define EXTI_RTSR1_TR17            EXTI_RTSR1_TR17_Msk                         /*!< Rising trigger event configuration bit of line 17 */
#define EXTI_RTSR1_TR18_Pos        (18UL)
#define EXTI_RTSR1_TR18_Msk        (0x1UL << EXTI_RTSR1_TR18_Pos)              /*!< 0x00040000 */
#define EXTI_RTSR1_TR18            EXTI_RTSR1_TR18_Msk                         /*!< Rising trigger event configuration bit of line 18 */
#define EXTI_RTSR1_TR19_Pos        (19UL)
#define EXTI_RTSR1_TR19_Msk        (0x1UL << EXTI_RTSR1_TR19_Pos)              /*!< 0x00080000 */
#define EXTI_RTSR1_TR19            EXTI_RTSR1_TR19_Msk                         /*!< Rising trigger event configuration bit of line 19 */
#define EXTI_RTSR1_TR20_Pos        (20UL)
#define EXTI_RTSR1_TR20_Msk        (0x1UL << EXTI_RTSR1_TR20_Pos)              /*!< 0x00100000 */
#define EXTI_RTSR1_TR20            EXTI_RTSR1_TR20_Msk                         /*!< Rising trigger event configuration bit of line 20 */
#define EXTI_RTSR1_TR21_Pos        (21UL)
#define EXTI_RTSR1_TR21_Msk        (0x1UL << EXTI_RTSR1_TR21_Pos)              /*!< 0x00200000 */
#define EXTI_RTSR1_TR21            EXTI_RTSR1_TR21_Msk                         /*!< Rising trigger event configuration bit of line 21 */

/******************  Bit definition for EXTI_FTSR1 register  *******************/
#define EXTI_FTSR1_TR_Pos          (0UL)
#define EXTI_FTSR1_TR_Msk          (0x3FFFFFUL << EXTI_FTSR1_TR_Pos)           /*!< 0x003FFFFF */
#define EXTI_FTSR1_TR              EXTI_FTSR1_TR_Msk                           /*!< Falling trigger event configuration bit */
#define EXTI_FTSR1_TR0_Pos         (0UL)
#define EXTI_FTSR1_TR0_Msk         (0x1UL << EXTI_FTSR1_TR0_Pos)               /*!< 0x00000001 */
#define EXTI_FTSR1_TR0             EXTI_FTSR1_TR0_Msk                          /*!< Falling trigger event configuration bit of line 0 */
#define EXTI_FTSR1_TR1_Pos         (1UL)
#define EXTI_FTSR1_TR1_Msk         (0x1UL << EXTI_FTSR1_TR1_Pos)               /*!< 0x00000002 */
#define EXTI_FTSR1_TR1             EXTI_FTSR1_TR1_Msk                          /*!< Falling trigger event configuration bit of line 1 */
#define EXTI_FTSR1_TR2_Pos         (2UL)
#define EXTI_FTSR1_TR2_Msk         (0x1UL << EXTI_FTSR1_TR2_Pos)               /*!< 0x00000004 */
#define EXTI_FTSR1_TR2             EXTI_FTSR1_TR2_Msk                          /*!< Falling trigger event configuration bit of line 2 */
#define EXTI_FTSR1_TR3_Pos         (3UL)
#define EXTI_FTSR1_TR3_Msk         (0x1UL << EXTI_FTSR1_TR3_Pos)               /*!< 0x00000008 */
#define EXTI_FTSR1_TR3             EXTI_FTSR1_TR3_Msk                          /*!< Falling trigger event configuration bit of line 3 */
#define EXTI_FTSR1_TR4_Pos         (4UL)
#define EXTI_FTSR1_TR4_Msk         (0x1UL << EXTI_FTSR1_TR4_Pos)               /*!< 0x00000010 */
#define EXTI_FTSR1_TR4             EXTI_FTSR1_TR4_Msk                          /*!< Falling trigger event configuration bit of line 4 */
#define EXTI_FTSR1_TR5_Pos         (5UL)
#define EXTI_FTSR1_TR5_Msk         (0x1UL << EXTI_FTSR1_TR5_Pos)               /*!< 0x00000020 */
#define EXTI_FTSR1_TR5             EXTI_FTSR1_TR5_Msk                          /*!< Falling trigger event configuration bit of line 5 */
#define EXTI_FTSR1_TR6_Pos         (6UL)
#define EXTI_FTSR1_TR6_Msk         (0x1UL << EXTI_FTSR1_TR6_Pos)               /*!< 0x00000040 */
#define EXTI_FTSR1_TR6             EXTI_FTSR1_TR6_Msk                          /*!< Falling trigger event configuration bit of line 6 */
#define EXTI_FTSR1_TR7_Pos         (7UL)
#define EXTI_FTSR1_TR7_Msk         (0x1UL << EXTI_FTSR1_TR7_Pos)               /*!< 0x00000080 */
#define EXTI_FTSR1_TR7             EXTI_FTSR1_TR7_Msk                          /*!< Falling trigger event configuration bit of line 7 */
#define EXTI_FTSR1_TR8_Pos         (8UL)
#define EXTI_FTSR1_TR8_Msk         (0x1UL << EXTI_FTSR1_TR8_Pos)               /*!< 0x00000100 */
#define EXTI_FTSR1_TR8             EXTI_FTSR1_TR8_Msk                          /*!< Falling trigger event configuration bit of line 8 */
#define EXTI_FTSR1_TR9_Pos         (9UL)
#define EXTI_FTSR1_TR9_Msk         (0x1UL << EXTI_FTSR1_TR9_Pos)               /*!< 0x00000200 */
#define EXTI_FTSR1_TR9             EXTI_FTSR1_TR9_Msk                          /*!< Falling trigger event configuration bit of line 9 */
#define EXTI_FTSR1_TR10_Pos        (10UL)
#define EXTI_FTSR1_TR10_Msk        (0x1UL << EXTI_FTSR1_TR10_Pos)              /*!< 0x00000400 */
#define EXTI_FTSR1_TR10            EXTI_FTSR1_TR10_Msk                         /*!< Falling trigger event configuration bit of line 10 */
#define EXTI_FTSR1_TR11_Pos        (11UL)
#define EXTI_FTSR1_TR11_Msk        (0x1UL << EXTI_FTSR1_TR11_Pos)              /*!< 0x00000800 */
#define EXTI_FTSR1_TR11            EXTI_FTSR1_TR11_Msk                         /*!< Falling trigger event configuration bit of line 11 */
#define EXTI_FTSR1_TR12_Pos        (12UL)
#define EXTI_FTSR1_TR12_Msk        (0x1UL << EXTI_FTSR1_TR12_Pos)              /*!< 0x00001000 */
#define EXTI_FTSR1_TR12            EXTI_FTSR1_TR12_Msk                         /*!< Falling trigger event configuration bit of line 12 */
#define EXTI_FTSR1_TR13_Pos        (13UL)
#define EXTI_FTSR1_TR13_Msk        (0x1UL << EXTI_FTSR1_TR13_Pos)              /*!< 0x00002000 */
#define EXTI_FTSR1_TR13            EXTI_FTSR1_TR13_Msk                         /*!< Falling trigger event configuration bit of line 13 */
#define EXTI_FTSR1_TR14_Pos        (14UL)
#define EXTI_FTSR1_TR14_Msk        (0x1UL << EXTI_FTSR1_TR14_Pos)              /*!< 0x00004000 */
#define EXTI_FTSR1_TR14            EXTI_FTSR1_TR14_Msk                         /*!< Falling trigger event configuration bit of line 14 */
#define EXTI_FTSR1_TR15_Pos        (15UL)
#define EXTI_FTSR1_TR15_Msk        (0x1UL << EXTI_FTSR1_TR15_Pos)              /*!< 0x00008000 */
#define EXTI_FTSR1_TR15            EXTI_FTSR1_TR15_Msk                         /*!< Falling trigger event configuration bit of line 15 */
#define EXTI_FTSR1_TR16_Pos        (16UL)
#define EXTI_FTSR1_TR16_Msk        (0x1UL << EXTI_FTSR1_TR16_Pos)              /*!< 0x00010000 */
#define EXTI_FTSR1_TR16            EXTI_FTSR1_TR16_Msk                         /*!< Falling trigger event configuration bit of line 16 */
#define EXTI_FTSR1_TR17_Pos        (17UL)
#define EXTI_FTSR1_TR17_Msk        (0x1UL << EXTI_FTSR1_TR17_Pos)              /*!< 0x00020000 */
#define EXTI_FTSR1_TR17            EXTI_FTSR1_TR17_Msk                         /*!< Falling trigger event configuration bit of line 17 */
#define EXTI_FTSR1_TR18_Pos        (18UL)
#define EXTI_FTSR1_TR18_Msk        (0x1UL << EXTI_FTSR1_TR18_Pos)              /*!< 0x00040000 */
#define EXTI_FTSR1_TR18            EXTI_FTSR1_TR18_Msk                         /*!< Falling trigger event configuration bit of line 18 */
#define EXTI_FTSR1_TR19_Pos        (19UL)
#define EXTI_FTSR1_TR19_Msk        (0x1UL << EXTI_FTSR1_TR19_Pos)              /*!< 0x00080000 */
#define EXTI_FTSR1_TR19            EXTI_FTSR1_TR19_Msk                         /*!< Falling trigger event configuration bit of line 19 */
#define EXTI_FTSR1_TR20_Pos        (20UL)
#define EXTI_FTSR1_TR20_Msk        (0x1UL << EXTI_FTSR1_TR20_Pos)              /*!< 0x00100000 */
#define EXTI_FTSR1_TR20            EXTI_FTSR1_TR20_Msk                         /*!< Falling trigger event configuration bit of line 20 */
#define EXTI_FTSR1_TR21_Pos        (21UL)
#define EXTI_FTSR1_TR21_Msk        (0x1UL << EXTI_FTSR1_TR21_Pos)              /*!< 0x00200000 */
#define EXTI_FTSR1_TR21            EXTI_FTSR1_TR21_Msk                         /*!< Falling trigger event configuration bit of line 21 */

/******************  Bit definition for EXTI_SWIER1 register  ******************/
#define EXTI_SWIER1_SWIER0_Pos     (0UL)
#define EXTI_SWIER1_SWIER0_Msk     (0x1UL << EXTI_SWIER1_SWIER0_Pos)           /*!< 0x00000001 */
#define EXTI_SWIER1_SWIER0         EXTI_SWIER1_SWIER0_Msk                      /*!< Software Interrupt on line 0 */
#define EXTI_SWIER1_SWIER1_Pos     (1UL)
#define EXTI_SWIER1_SWIER1_Msk     (0x1UL << EXTI_SWIER1_SWIER1_Pos)           /*!< 0x00000002 */
#define EXTI_SWIER1_SWIER1         EXTI_SWIER1_SWIER1_Msk                      /*!< Software Interrupt on line 1 */
#define EXTI_SWIER1_SWIER2_Pos     (2UL)
#define EXTI_SWIER1_SWIER2_Msk     (0x1UL << EXTI_SWIER1_SWIER2_Pos)           /*!< 0x00000004 */
#define EXTI_SWIER1_SWIER2         EXTI_SWIER1_SWIER2_Msk                      /*!< Software Interrupt on line 2 */
#define EXTI_SWIER1_SWIER3_Pos     (3UL)
#define EXTI_SWIER1_SWIER3_Msk     (0x1UL << EXTI_SWIER1_SWIER3_Pos)           /*!< 0x00000008 */
#define EXTI_SWIER1_SWIER3         EXTI_SWIER1_SWIER3_Msk                      /*!< Software Interrupt on line 3 */
#define EXTI_SWIER1_SWIER4_Pos     (4UL)
#define EXTI_SWIER1_SWIER4_Msk     (0x1UL << EXTI_SWIER1_SWIER4_Pos)           /*!< 0x00000010 */
#define EXTI_SWIER1_SWIER4         EXTI_SWIER1_SWIER4_Msk                      /*!< Software Interrupt on line 4 */
#define EXTI_SWIER1_SWIER5_Pos     (5UL)
#define EXTI_SWIER1_SWIER5_Msk     (0x1UL << EXTI_SWIER1_SWIER5_Pos)           /*!< 0x00000020 */
#define EXTI_SWIER1_SWIER5         EXTI_SWIER1_SWIER5_Msk                      /*!< Software Interrupt on line 5 */
#define EXTI_SWIER1_SWIER6_Pos     (6UL)
#define EXTI_SWIER1_SWIER6_Msk     (0x1UL << EXTI_SWIER1_SWIER6_Pos)           /*!< 0x00000040 */
#define EXTI_SWIER1_SWIER6         EXTI_SWIER1_SWIER6_Msk                      /*!< Software Interrupt on line 6 */
#define EXTI_SWIER1_SWIER7_Pos     (7UL)
#define EXTI_SWIER1_SWIER7_Msk     (0x1UL << EXTI_SWIER1_SWIER7_Pos)           /*!< 0x00000080 */
#define EXTI_SWIER1_SWIER7         EXTI_SWIER1_SWIER7_Msk                      /*!< Software Interrupt on line 7 */
#define EXTI_SWIER1_SWIER8_Pos     (8UL)
#define EXTI_SWIER1_SWIER8_Msk     (0x1UL << EXTI_SWIER1_SWIER8_Pos)           /*!< 0x00000100 */
#define EXTI_SWIER1_SWIER8         EXTI_SWIER1_SWIER8_Msk                      /*!< Software Interrupt on line 8 */
#define EXTI_SWIER1_SWIER9_Pos     (9UL)
#define EXTI_SWIER1_SWIER9_Msk     (0x1UL << EXTI_SWIER1_SWIER9_Pos)           /*!< 0x00000200 */
#define EXTI_SWIER1_SWIER9         EXTI_SWIER1_SWIER9_Msk                      /*!< Software Interrupt on line 9 */
#define EXTI_SWIER1_SWIER10_Pos    (10UL)
#define EXTI_SWIER1_SWIER10_Msk    (0x1UL << EXTI_SWIER1_SWIER10_Pos)          /*!< 0x00000400 */
#define EXTI_SWIER1_SWIER10        EXTI_SWIER1_SWIER10_Msk                     /*!< Software Interrupt on line 10 */
#define EXTI_SWIER1_SWIER11_Pos    (11UL)
#define EXTI_SWIER1_SWIER11_Msk    (0x1UL << EXTI_SWIER1_SWIER11_Pos)          /*!< 0x00000800 */
#define EXTI_SWIER1_SWIER11        EXTI_SWIER1_SWIER11_Msk                     /*!< Software Interrupt on line 11 */
#define EXTI_SWIER1_SWIER12_Pos    (12UL)
#define EXTI_SWIER1_SWIER12_Msk    (0x1UL << EXTI_SWIER1_SWIER12_Pos)          /*!< 0x00001000 */
#define EXTI_SWIER1_SWIER12        EXTI_SWIER1_SWIER12_Msk                     /*!< Software Interrupt on line 12 */
#define EXTI_SWIER1_SWIER13_Pos    (13UL)
#define EXTI_SWIER1_SWIER13_Msk    (0x1UL << EXTI_SWIER1_SWIER13_Pos)          /*!< 0x00002000 */
#define EXTI_SWIER1_SWIER13        EXTI_SWIER1_SWIER13_Msk                     /*!< Software Interrupt on line 13 */
#define EXTI_SWIER1_SWIER14_Pos    (14UL)
#define EXTI_SWIER1_SWIER14_Msk    (0x1UL << EXTI_SWIER1_SWIER14_Pos)          /*!< 0x00004000 */
#define EXTI_SWIER1_SWIER14        EXTI_SWIER1_SWIER14_Msk                     /*!< Software Interrupt on line 14 */
#define EXTI_SWIER1_SWIER15_Pos    (15UL)
#define EXTI_SWIER1_SWIER15_Msk    (0x1UL << EXTI_SWIER1_SWIER15_Pos)          /*!< 0x00008000 */
#define EXTI_SWIER1_SWIER15        EXTI_SWIER1_SWIER15_Msk                     /*!< Software Interrupt on line 15 */
#define EXTI_SWIER1_SWIER16_Pos    (16UL)
#define EXTI_SWIER1_SWIER16_Msk    (0x1UL << EXTI_SWIER1_SWIER16_Pos)          /*!< 0x00010000 */
#define EXTI_SWIER1_SWIER16        EXTI_SWIER1_SWIER16_Msk                     /*!< Software Interrupt on line 16 */
#define EXTI_SWIER1_SWIER17_Pos    (17UL)
#define EXTI_SWIER1_SWIER17_Msk    (0x1UL << EXTI_SWIER1_SWIER17_Pos)          /*!< 0x00020000 */
#define EXTI_SWIER1_SWIER17        EXTI_SWIER1_SWIER17_Msk                     /*!< Software Interrupt on line 17 */
#define EXTI_SWIER1_SWIER18_Pos    (18UL)
#define EXTI_SWIER1_SWIER18_Msk    (0x1UL << EXTI_SWIER1_SWIER18_Pos)          /*!< 0x00040000 */
#define EXTI_SWIER1_SWIER18        EXTI_SWIER1_SWIER18_Msk                     /*!< Software Interrupt on line 18 */
#define EXTI_SWIER1_SWIER19_Pos    (19UL)
#define EXTI_SWIER1_SWIER19_Msk    (0x1UL << EXTI_SWIER1_SWIER19_Pos)          /*!< 0x00080000 */
#define EXTI_SWIER1_SWIER19        EXTI_SWIER1_SWIER19_Msk                     /*!< Software Interrupt on line 19 */
#define EXTI_SWIER1_SWIER20_Pos    (20UL)
#define EXTI_SWIER1_SWIER20_Msk    (0x1UL << EXTI_SWIER1_SWIER20_Pos)          /*!< 0x00100000 */
#define EXTI_SWIER1_SWIER20        EXTI_SWIER1_SWIER20_Msk                     /*!< Software Interrupt on line 20 */
#define EXTI_SWIER1_SWIER21_Pos    (21UL)
#define EXTI_SWIER1_SWIER21_Msk    (0x1UL << EXTI_SWIER1_SWIER21_Pos)          /*!< 0x00200000 */
#define EXTI_SWIER1_SWIER21        EXTI_SWIER1_SWIER21_Msk                     /*!< Software Interrupt on line 21 */

/*******************  Bit definition for EXTI_RPR1 register  ******************/
#define EXTI_RPR1_RPIF0_Pos          (0UL)
#define EXTI_RPR1_RPIF0_Msk          (0x1UL << EXTI_RPR1_RPIF0_Pos)            /*!< 0x00000001 */
#define EXTI_RPR1_RPIF0              EXTI_RPR1_RPIF0_Msk                       /*!< Rising Pending Interrupt Flag on line 0 */
#define EXTI_RPR1_RPIF1_Pos          (1UL)
#define EXTI_RPR1_RPIF1_Msk          (0x1UL << EXTI_RPR1_RPIF1_Pos)            /*!< 0x00000002 */
#define EXTI_RPR1_RPIF1              EXTI_RPR1_RPIF1_Msk                       /*!< Rising Pending Interrupt Flag on line 1 */
#define EXTI_RPR1_RPIF2_Pos          (2UL)
#define EXTI_RPR1_RPIF2_Msk          (0x1UL << EXTI_RPR1_RPIF2_Pos)            /*!< 0x00000004 */
#define EXTI_RPR1_RPIF2              EXTI_RPR1_RPIF2_Msk                       /*!< Rising Pending Interrupt Flag on line 2 */
#define EXTI_RPR1_RPIF3_Pos          (3UL)
#define EXTI_RPR1_RPIF3_Msk          (0x1UL << EXTI_RPR1_RPIF3_Pos)            /*!< 0x00000008 */
#define EXTI_RPR1_RPIF3              EXTI_RPR1_RPIF3_Msk                       /*!< Rising Pending Interrupt Flag on line 3 */
#define EXTI_RPR1_RPIF4_Pos          (4UL)
#define EXTI_RPR1_RPIF4_Msk          (0x1UL << EXTI_RPR1_RPIF4_Pos)            /*!< 0x00000010 */
#define EXTI_RPR1_RPIF4              EXTI_RPR1_RPIF4_Msk                       /*!< Rising Pending Interrupt Flag on line 4 */
#define EXTI_RPR1_RPIF5_Pos          (5UL)
#define EXTI_RPR1_RPIF5_Msk          (0x1UL << EXTI_RPR1_RPIF5_Pos)            /*!< 0x00000020 */
#define EXTI_RPR1_RPIF5              EXTI_RPR1_RPIF5_Msk                       /*!< Rising Pending Interrupt Flag on line 5 */
#define EXTI_RPR1_RPIF6_Pos          (6UL)
#define EXTI_RPR1_RPIF6_Msk          (0x1UL << EXTI_RPR1_RPIF6_Pos)            /*!< 0x00000040 */
#define EXTI_RPR1_RPIF6              EXTI_RPR1_RPIF6_Msk                       /*!< Rising Pending Interrupt Flag on line 6 */
#define EXTI_RPR1_RPIF7_Pos          (7UL)
#define EXTI_RPR1_RPIF7_Msk          (0x1UL << EXTI_RPR1_RPIF7_Pos)            /*!< 0x00000080 */
#define EXTI_RPR1_RPIF7              EXTI_RPR1_RPIF7_Msk                       /*!< Rising Pending Interrupt Flag on line 7 */
#define EXTI_RPR1_RPIF8_Pos          (8UL)
#define EXTI_RPR1_RPIF8_Msk          (0x1UL << EXTI_RPR1_RPIF8_Pos)            /*!< 0x00000100 */
#define EXTI_RPR1_RPIF8              EXTI_RPR1_RPIF8_Msk                       /*!< Rising Pending Interrupt Flag on line 8 */
#define EXTI_RPR1_RPIF9_Pos          (9UL)
#define EXTI_RPR1_RPIF9_Msk          (0x1UL << EXTI_RPR1_RPIF9_Pos)            /*!< 0x00000200 */
#define EXTI_RPR1_RPIF9              EXTI_RPR1_RPIF9_Msk                       /*!< Rising Pending Interrupt Flag on line 9 */
#define EXTI_RPR1_RPIF10_Pos         (10UL)
#define EXTI_RPR1_RPIF10_Msk         (0x1UL << EXTI_RPR1_RPIF10_Pos)           /*!< 0x00000400 */
#define EXTI_RPR1_RPIF10             EXTI_RPR1_RPIF10_Msk                      /*!< Rising Pending Interrupt Flag on line 10 */
#define EXTI_RPR1_RPIF11_Pos         (11UL)
#define EXTI_RPR1_RPIF11_Msk         (0x1UL << EXTI_RPR1_RPIF11_Pos)           /*!< 0x00000800 */
#define EXTI_RPR1_RPIF11             EXTI_RPR1_RPIF11_Msk                      /*!< Rising Pending Interrupt Flag on line 11 */
#define EXTI_RPR1_RPIF12_Pos         (12UL)
#define EXTI_RPR1_RPIF12_Msk         (0x1UL << EXTI_RPR1_RPIF12_Pos)           /*!< 0x00001000 */
#define EXTI_RPR1_RPIF12             EXTI_RPR1_RPIF12_Msk                      /*!< Rising Pending Interrupt Flag on line 12 */
#define EXTI_RPR1_RPIF13_Pos         (13UL)
#define EXTI_RPR1_RPIF13_Msk         (0x1UL << EXTI_RPR1_RPIF13_Pos)           /*!< 0x00002000 */
#define EXTI_RPR1_RPIF13             EXTI_RPR1_RPIF13_Msk                      /*!< Rising Pending Interrupt Flag on line 13 */
#define EXTI_RPR1_RPIF14_Pos         (14UL)
#define EXTI_RPR1_RPIF14_Msk         (0x1UL << EXTI_RPR1_RPIF14_Pos)           /*!< 0x00004000 */
#define EXTI_RPR1_RPIF14             EXTI_RPR1_RPIF14_Msk                      /*!< Rising Pending Interrupt Flag on line 14 */
#define EXTI_RPR1_RPIF15_Pos         (15UL)
#define EXTI_RPR1_RPIF15_Msk         (0x1UL << EXTI_RPR1_RPIF15_Pos)           /*!< 0x00008000 */
#define EXTI_RPR1_RPIF15             EXTI_RPR1_RPIF15_Msk                      /*!< Rising Pending Interrupt Flag on line 15 */
#define EXTI_RPR1_RPIF16_Pos         (16UL)
#define EXTI_RPR1_RPIF16_Msk         (0x1UL << EXTI_RPR1_RPIF16_Pos)           /*!< 0x00010000 */
#define EXTI_RPR1_RPIF16             EXTI_RPR1_RPIF16_Msk                      /*!< Rising Pending Interrupt Flag on line 16 */
#define EXTI_RPR1_RPIF17_Pos         (17UL)
#define EXTI_RPR1_RPIF17_Msk         (0x1UL << EXTI_RPR1_RPIF17_Pos)           /*!< 0x00020000 */
#define EXTI_RPR1_RPIF17             EXTI_RPR1_RPIF17_Msk                      /*!< Rising Pending Interrupt Flag on line 17 */
#define EXTI_RPR1_RPIF18_Pos         (18UL)
#define EXTI_RPR1_RPIF18_Msk         (0x1UL << EXTI_RPR1_RPIF18_Pos)           /*!< 0x00040000 */
#define EXTI_RPR1_RPIF18             EXTI_RPR1_RPIF18_Msk                      /*!< Rising Pending Interrupt Flag on line 18 */
#define EXTI_RPR1_RPIF19_Pos         (19UL)
#define EXTI_RPR1_RPIF19_Msk         (0x1UL << EXTI_RPR1_RPIF19_Pos)           /*!< 0x00080000 */
#define EXTI_RPR1_RPIF19             EXTI_RPR1_RPIF19_Msk                      /*!< Rising Pending Interrupt Flag on line 19 */
#define EXTI_RPR1_RPIF20_Pos         (20UL)
#define EXTI_RPR1_RPIF20_Msk         (0x1UL << EXTI_RPR1_RPIF20_Pos)           /*!< 0x00100000 */
#define EXTI_RPR1_RPIF20             EXTI_RPR1_RPIF20_Msk                      /*!< Rising Pending Interrupt Flag on line 20 */
#define EXTI_RPR1_RPIF21_Pos         (21UL)
#define EXTI_RPR1_RPIF21_Msk         (0x1UL << EXTI_RPR1_RPIF21_Pos)           /*!< 0x00200000 */
#define EXTI_RPR1_RPIF21             EXTI_RPR1_RPIF21_Msk                      /*!< Rising Pending Interrupt Flag on line 21 */

/*******************  Bit definition for EXTI_FPR1 register  ******************/
#define EXTI_FPR1_FPIF0_Pos          (0UL)
#define EXTI_FPR1_FPIF0_Msk          (0x1UL << EXTI_FPR1_FPIF0_Pos)            /*!< 0x00000001 */
#define EXTI_FPR1_FPIF0              EXTI_FPR1_FPIF0_Msk                       /*!< Falling Pending Interrupt Flag on line 0 */
#define EXTI_FPR1_FPIF1_Pos          (1UL)
#define EXTI_FPR1_FPIF1_Msk          (0x1UL << EXTI_FPR1_FPIF1_Pos)            /*!< 0x00000002 */
#define EXTI_FPR1_FPIF1              EXTI_FPR1_FPIF1_Msk                       /*!< Falling Pending Interrupt Flag on line 1 */
#define EXTI_FPR1_FPIF2_Pos          (2UL)
#define EXTI_FPR1_FPIF2_Msk          (0x1UL << EXTI_FPR1_FPIF2_Pos)            /*!< 0x00000004 */
#define EXTI_FPR1_FPIF2              EXTI_FPR1_FPIF2_Msk                       /*!< Falling Pending Interrupt Flag on line 2 */
#define EXTI_FPR1_FPIF3_Pos          (3UL)
#define EXTI_FPR1_FPIF3_Msk          (0x1UL << EXTI_FPR1_FPIF3_Pos)            /*!< 0x00000008 */
#define EXTI_FPR1_FPIF3              EXTI_FPR1_FPIF3_Msk                       /*!< Falling Pending Interrupt Flag on line 3 */
#define EXTI_FPR1_FPIF4_Pos          (4UL)
#define EXTI_FPR1_FPIF4_Msk          (0x1UL << EXTI_FPR1_FPIF4_Pos)            /*!< 0x00000010 */
#define EXTI_FPR1_FPIF4              EXTI_FPR1_FPIF4_Msk                       /*!< Falling Pending Interrupt Flag on line 4 */
#define EXTI_FPR1_FPIF5_Pos          (5UL)
#define EXTI_FPR1_FPIF5_Msk          (0x1UL << EXTI_FPR1_FPIF5_Pos)            /*!< 0x00000020 */
#define EXTI_FPR1_FPIF5              EXTI_FPR1_FPIF5_Msk                       /*!< Falling Pending Interrupt Flag on line 5 */
#define EXTI_FPR1_FPIF6_Pos          (6UL)
#define EXTI_FPR1_FPIF6_Msk          (0x1UL << EXTI_FPR1_FPIF6_Pos)            /*!< 0x00000040 */
#define EXTI_FPR1_FPIF6              EXTI_FPR1_FPIF6_Msk                       /*!< Falling Pending Interrupt Flag on line 6 */
#define EXTI_FPR1_FPIF7_Pos          (7UL)
#define EXTI_FPR1_FPIF7_Msk           (0x1UL << EXTI_FPR1_FPIF7_Pos)            /*!< 0x00000080 */
#define EXTI_FPR1_FPIF7              EXTI_FPR1_FPIF7_Msk                       /*!< Falling Pending Interrupt Flag on line 7 */
#define EXTI_FPR1_FPIF8_Pos          (8UL)
#define EXTI_FPR1_FPIF8_Msk          (0x1UL << EXTI_FPR1_FPIF8_Pos)            /*!< 0x00000100 */
#define EXTI_FPR1_FPIF8              EXTI_FPR1_FPIF8_Msk                       /*!< Falling Pending Interrupt Flag on line 8 */
#define EXTI_FPR1_FPIF9_Pos          (9UL)
#define EXTI_FPR1_FPIF9_Msk          (0x1UL << EXTI_FPR1_FPIF9_Pos)            /*!< 0x00000200 */
#define EXTI_FPR1_FPIF9              EXTI_FPR1_FPIF9_Msk                       /*!< Falling Pending Interrupt Flag on line 9 */
#define EXTI_FPR1_FPIF10_Pos         (10UL)
#define EXTI_FPR1_FPIF10_Msk         (0x1UL << EXTI_FPR1_FPIF10_Pos)           /*!< 0x00000400 */
#define EXTI_FPR1_FPIF10             EXTI_FPR1_FPIF10_Msk                      /*!< Falling Pending Interrupt Flag on line 10 */
#define EXTI_FPR1_FPIF11_Pos         (11UL)
#define EXTI_FPR1_FPIF11_Msk         (0x1UL << EXTI_FPR1_FPIF11_Pos)           /*!< 0x00000800 */
#define EXTI_FPR1_FPIF11             EXTI_FPR1_FPIF11_Msk                      /*!< Falling Pending Interrupt Flag on line 11 */
#define EXTI_FPR1_FPIF12_Pos         (12UL)
#define EXTI_FPR1_FPIF12_Msk         (0x1UL << EXTI_FPR1_FPIF12_Pos)           /*!< 0x00001000 */
#define EXTI_FPR1_FPIF12             EXTI_FPR1_FPIF12_Msk                      /*!< Falling Pending Interrupt Flag on line 12 */
#define EXTI_FPR1_FPIF13_Pos         (13UL)
#define EXTI_FPR1_FPIF13_Msk         (0x1UL << EXTI_FPR1_FPIF13_Pos)           /*!< 0x00020000 */
#define EXTI_FPR1_FPIF13             EXTI_FPR1_FPIF13_Msk                      /*!< Falling Pending Interrupt Flag on line 13 */
#define EXTI_FPR1_FPIF14_Pos         (14UL)
#define EXTI_FPR1_FPIF14_Msk         (0x1UL << EXTI_FPR1_FPIF14_Pos)           /*!< 0x00040000 */
#define EXTI_FPR1_FPIF14             EXTI_FPR1_FPIF14_Msk                      /*!< Falling Pending Interrupt Flag on line 14 */
#define EXTI_FPR1_FPIF15_Pos         (15UL)
#define EXTI_FPR1_FPIF15_Msk         (0x1UL << EXTI_FPR1_FPIF15_Pos)           /*!< 0x00080000 */
#define EXTI_FPR1_FPIF15             EXTI_FPR1_FPIF15_Msk                      /*!< Falling Pending Interrupt Flag on line 15 */
#define EXTI_FPR1_FPIF16_Pos         (16UL)
#define EXTI_FPR1_FPIF16_Msk         (0x1UL << EXTI_FPR1_FPIF16_Pos)           /*!< 0x00010000 */
#define EXTI_FPR1_FPIF16             EXTI_FPR1_FPIF16_Msk                      /*!< Falling Pending Interrupt Flag on line 16 */
#define EXTI_FPR1_FPIF17_Pos         (17UL)
#define EXTI_FPR1_FPIF17_Msk         (0x1UL << EXTI_FPR1_FPIF17_Pos)           /*!< 0x00020000 */
#define EXTI_FPR1_FPIF17             EXTI_FPR1_FPIF17_Msk                      /*!< Falling Pending Interrupt Flag on line 17 */
#define EXTI_FPR1_FPIF18_Pos         (18UL)
#define EXTI_FPR1_FPIF18_Msk         (0x1UL << EXTI_FPR1_FPIF18_Pos)           /*!< 0x00040000 */
#define EXTI_FPR1_FPIF18             EXTI_FPR1_FPIF18_Msk                      /*!< Falling Pending Interrupt Flag on line 18 */
#define EXTI_FPR1_FPIF19_Pos         (19UL)
#define EXTI_FPR1_FPIF19_Msk         (0x1UL << EXTI_FPR1_FPIF19_Pos)           /*!< 0x00080000 */
#define EXTI_FPR1_FPIF19             EXTI_FPR1_FPIF19_Msk                      /*!< Falling Pending Interrupt Flag on line 19 */
#define EXTI_FPR1_FPIF20_Pos         (20UL)
#define EXTI_FPR1_FPIF20_Msk         (0x1UL << EXTI_FPR1_FPIF20_Pos)           /*!< 0x00100000 */
#define EXTI_FPR1_FPIF20             EXTI_FPR1_FPIF20_Msk                      /*!< Falling Pending Interrupt Flag on line 20 */
#define EXTI_FPR1_FPIF21_Pos         (21UL)
#define EXTI_FPR1_FPIF21_Msk         (0x1UL << EXTI_FPR1_FPIF21_Pos)           /*!< 0x00200000 */
#define EXTI_FPR1_FPIF21             EXTI_FPR1_FPIF21_Msk                      /*!< Falling Pending Interrupt Flag on line 21 */

/*****************  Bit definition for EXTI_EXTICR1 register  **************/
#define EXTI_EXTICR1_EXTI0_Pos       (0UL)
#define EXTI_EXTICR1_EXTI0_Msk       (0xFFUL << EXTI_EXTICR1_EXTI0_Pos)        /*!< 0x000000FF */
#define EXTI_EXTICR1_EXTI0           EXTI_EXTICR1_EXTI0_Msk                    /*!< EXTI 0 configuration */
#define EXTI_EXTICR1_EXTI0_0         (0x1UL << EXTI_EXTICR1_EXTI0_Pos)         /*!< 0x00000001 */
#define EXTI_EXTICR1_EXTI0_1         (0x2UL << EXTI_EXTICR1_EXTI0_Pos)         /*!< 0x00000002 */
#define EXTI_EXTICR1_EXTI0_2         (0x4UL << EXTI_EXTICR1_EXTI0_Pos)         /*!< 0x00000004 */
#define EXTI_EXTICR1_EXTI0_3         (0x8UL << EXTI_EXTICR1_EXTI0_Pos)         /*!< 0x00000008 */
#define EXTI_EXTICR1_EXTI0_4         (0x10UL << EXTI_EXTICR1_EXTI0_Pos)        /*!< 0x00000010 */
#define EXTI_EXTICR1_EXTI0_5         (0x20UL << EXTI_EXTICR1_EXTI0_Pos)        /*!< 0x00000020 */
#define EXTI_EXTICR1_EXTI0_6         (0x40UL << EXTI_EXTICR1_EXTI0_Pos)        /*!< 0x00000040 */
#define EXTI_EXTICR1_EXTI0_7         (0x80UL << EXTI_EXTICR1_EXTI0_Pos)        /*!< 0x00000080 */
#define EXTI_EXTICR1_EXTI1_Pos       (8UL)
#define EXTI_EXTICR1_EXTI1_Msk       (0xFFUL << EXTI_EXTICR1_EXTI1_Pos)        /*!< 0x0000FF00 */
#define EXTI_EXTICR1_EXTI1           EXTI_EXTICR1_EXTI1_Msk                    /*!< EXTI 1 configuration */
#define EXTI_EXTICR1_EXTI1_0         (0x1UL << EXTI_EXTICR1_EXTI1_Pos)         /*!< 0x00000100 */
#define EXTI_EXTICR1_EXTI1_1         (0x2UL << EXTI_EXTICR1_EXTI1_Pos)         /*!< 0x00000200 */
#define EXTI_EXTICR1_EXTI1_2         (0x4UL << EXTI_EXTICR1_EXTI1_Pos)         /*!< 0x00000400 */
#define EXTI_EXTICR1_EXTI1_3         (0x8UL << EXTI_EXTICR1_EXTI1_Pos)         /*!< 0x00000800 */
#define EXTI_EXTICR1_EXTI1_4         (0x10UL << EXTI_EXTICR1_EXTI1_Pos)        /*!< 0x00001000 */
#define EXTI_EXTICR1_EXTI1_5         (0x20UL << EXTI_EXTICR1_EXTI1_Pos)        /*!< 0x00002000 */
#define EXTI_EXTICR1_EXTI1_6         (0x40UL << EXTI_EXTICR1_EXTI1_Pos)        /*!< 0x00004000 */
#define EXTI_EXTICR1_EXTI1_7         (0x80UL << EXTI_EXTICR1_EXTI1_Pos)        /*!< 0x00008000 */
#define EXTI_EXTICR1_EXTI2_Pos       (16UL)
#define EXTI_EXTICR1_EXTI2_Msk       (0xFFUL << EXTI_EXTICR1_EXTI2_Pos)        /*!< 0x00FF0000 */
#define EXTI_EXTICR1_EXTI2           EXTI_EXTICR1_EXTI2_Msk                    /*!< EXTI 2 configuration */
#define EXTI_EXTICR1_EXTI2_0         (0x1UL << EXTI_EXTICR1_EXTI2_Pos)         /*!< 0x00010000 */
#define EXTI_EXTICR1_EXTI2_1         (0x2UL << EXTI_EXTICR1_EXTI2_Pos)         /*!< 0x00020000 */
#define EXTI_EXTICR1_EXTI2_2         (0x4UL << EXTI_EXTICR1_EXTI2_Pos)         /*!< 0x00040000 */
#define EXTI_EXTICR1_EXTI2_3         (0x8UL << EXTI_EXTICR1_EXTI2_Pos)         /*!< 0x00080000 */
#define EXTI_EXTICR1_EXTI2_4         (0x10UL << EXTI_EXTICR1_EXTI2_Pos)        /*!< 0x00100000 */
#define EXTI_EXTICR1_EXTI2_5         (0x20UL << EXTI_EXTICR1_EXTI2_Pos)        /*!< 0x00200000 */
#define EXTI_EXTICR1_EXTI2_6         (0x40UL << EXTI_EXTICR1_EXTI2_Pos)        /*!< 0x00400000 */
#define EXTI_EXTICR1_EXTI2_7         (0x80UL << EXTI_EXTICR1_EXTI2_Pos)        /*!< 0x00800000 */
#define EXTI_EXTICR1_EXTI3_Pos       (24UL)
#define EXTI_EXTICR1_EXTI3_Msk       (0xFFUL << EXTI_EXTICR1_EXTI3_Pos)        /*!< 0xFF000000 */
#define EXTI_EXTICR1_EXTI3           EXTI_EXTICR1_EXTI3_Msk                    /*!< EXTI 3 configuration */
#define EXTI_EXTICR1_EXTI3_0         (0x1UL << EXTI_EXTICR1_EXTI3_Pos)         /*!< 0x01000000 */
#define EXTI_EXTICR1_EXTI3_1         (0x2UL << EXTI_EXTICR1_EXTI3_Pos)         /*!< 0x02000000 */
#define EXTI_EXTICR1_EXTI3_2         (0x4UL << EXTI_EXTICR1_EXTI3_Pos)         /*!< 0x04000000 */
#define EXTI_EXTICR1_EXTI3_3         (0x8UL << EXTI_EXTICR1_EXTI3_Pos)         /*!< 0x08000000 */
#define EXTI_EXTICR1_EXTI3_4         (0x10UL << EXTI_EXTICR1_EXTI3_Pos)        /*!< 0x10000000 */
#define EXTI_EXTICR1_EXTI3_5         (0x20UL << EXTI_EXTICR1_EXTI3_Pos)        /*!< 0x20000000 */
#define EXTI_EXTICR1_EXTI3_6         (0x40UL << EXTI_EXTICR1_EXTI3_Pos)        /*!< 0x40000000 */
#define EXTI_EXTICR1_EXTI3_7         (0x80UL << EXTI_EXTICR1_EXTI3_Pos)        /*!< 0x80000000 */

/*****************  Bit definition for EXTI_EXTICR2 register  **************/
#define EXTI_EXTICR2_EXTI4_Pos       (0UL)
#define EXTI_EXTICR2_EXTI4_Msk       (0xFFUL << EXTI_EXTICR2_EXTI4_Pos)        /*!< 0x000000FF */
#define EXTI_EXTICR2_EXTI4           EXTI_EXTICR2_EXTI4_Msk                    /*!< EXTI 4 configuration */
#define EXTI_EXTICR2_EXTI4_0         (0x1UL << EXTI_EXTICR2_EXTI4_Pos)         /*!< 0x00000001 */
#define EXTI_EXTICR2_EXTI4_1         (0x2UL << EXTI_EXTICR2_EXTI4_Pos)         /*!< 0x00000002 */
#define EXTI_EXTICR2_EXTI4_2         (0x4UL << EXTI_EXTICR2_EXTI4_Pos)         /*!< 0x00000004 */
#define EXTI_EXTICR2_EXTI4_3         (0x8UL << EXTI_EXTICR2_EXTI4_Pos)         /*!< 0x00000008 */
#define EXTI_EXTICR2_EXTI4_4         (0x10UL << EXTI_EXTICR2_EXTI4_Pos)        /*!< 0x00000010 */
#define EXTI_EXTICR2_EXTI4_5         (0x20UL << EXTI_EXTICR2_EXTI4_Pos)        /*!< 0x00000020 */
#define EXTI_EXTICR2_EXTI4_6         (0x40UL << EXTI_EXTICR2_EXTI4_Pos)        /*!< 0x00000040 */
#define EXTI_EXTICR2_EXTI4_7         (0x80UL << EXTI_EXTICR2_EXTI4_Pos)        /*!< 0x00000080 */
#define EXTI_EXTICR2_EXTI5_Pos       (8UL)
#define EXTI_EXTICR2_EXTI5_Msk       (0xFFUL << EXTI_EXTICR2_EXTI5_Pos)        /*!< 0x0000FF00 */
#define EXTI_EXTICR2_EXTI5           EXTI_EXTICR2_EXTI5_Msk                    /*!< EXTI 5 configuration */
#define EXTI_EXTICR2_EXTI5_0         (0x1UL << EXTI_EXTICR2_EXTI5_Pos)         /*!< 0x00000100 */
#define EXTI_EXTICR2_EXTI5_1         (0x2UL << EXTI_EXTICR2_EXTI5_Pos)         /*!< 0x00000200 */
#define EXTI_EXTICR2_EXTI5_2         (0x4UL << EXTI_EXTICR2_EXTI5_Pos)         /*!< 0x00000400 */
#define EXTI_EXTICR2_EXTI5_3         (0x8UL << EXTI_EXTICR2_EXTI5_Pos)         /*!< 0x00000800 */
#define EXTI_EXTICR2_EXTI5_4         (0x10UL << EXTI_EXTICR2_EXTI5_Pos)        /*!< 0x00001000 */
#define EXTI_EXTICR2_EXTI5_5         (0x20UL << EXTI_EXTICR2_EXTI5_Pos)        /*!< 0x00002000 */
#define EXTI_EXTICR2_EXTI5_6         (0x40UL << EXTI_EXTICR2_EXTI5_Pos)        /*!< 0x00004000 */
#define EXTI_EXTICR2_EXTI5_7         (0x80UL << EXTI_EXTICR2_EXTI5_Pos)        /*!< 0x00008000 */
#define EXTI_EXTICR2_EXTI6_Pos       (16UL)
#define EXTI_EXTICR2_EXTI6_Msk       (0xFFUL << EXTI_EXTICR2_EXTI6_Pos)        /*!< 0x00FF0000 */
#define EXTI_EXTICR2_EXTI6           EXTI_EXTICR2_EXTI6_Msk                    /*!< EXTI 6 configuration */
#define EXTI_EXTICR2_EXTI6_0         (0x1UL << EXTI_EXTICR2_EXTI6_Pos)         /*!< 0x00010000 */
#define EXTI_EXTICR2_EXTI6_1         (0x2UL << EXTI_EXTICR2_EXTI6_Pos)         /*!< 0x00020000 */
#define EXTI_EXTICR2_EXTI6_2         (0x4UL << EXTI_EXTICR2_EXTI6_Pos)         /*!< 0x00040000 */
#define EXTI_EXTICR2_EXTI6_3         (0x8UL << EXTI_EXTICR2_EXTI6_Pos)         /*!< 0x00080000 */
#define EXTI_EXTICR2_EXTI6_4         (0x10UL << EXTI_EXTICR2_EXTI6_Pos)        /*!< 0x00100000 */
#define EXTI_EXTICR2_EXTI6_5         (0x20UL << EXTI_EXTICR2_EXTI6_Pos)        /*!< 0x00200000 */
#define EXTI_EXTICR2_EXTI6_6         (0x40UL << EXTI_EXTICR2_EXTI6_Pos)        /*!< 0x00400000 */
#define EXTI_EXTICR2_EXTI6_7         (0x80UL << EXTI_EXTICR2_EXTI6_Pos)        /*!< 0x00800000 */
#define EXTI_EXTICR2_EXTI7_Pos       (24UL)
#define EXTI_EXTICR2_EXTI7_Msk       (0xFFUL << EXTI_EXTICR2_EXTI7_Pos)        /*!< 0xFF000000 */
#define EXTI_EXTICR2_EXTI7           EXTI_EXTICR2_EXTI7_Msk                    /*!< EXTI 7 configuration */
#define EXTI_EXTICR2_EXTI7_0         (0x1UL << EXTI_EXTICR2_EXTI7_Pos)         /*!< 0x01000000 */
#define EXTI_EXTICR2_EXTI7_1         (0x2UL << EXTI_EXTICR2_EXTI7_Pos)         /*!< 0x02000000 */
#define EXTI_EXTICR2_EXTI7_2         (0x4UL << EXTI_EXTICR2_EXTI7_Pos)         /*!< 0x04000000 */
#define EXTI_EXTICR2_EXTI7_3         (0x8UL << EXTI_EXTICR2_EXTI7_Pos)         /*!< 0x08000000 */
#define EXTI_EXTICR2_EXTI7_4         (0x10UL << EXTI_EXTICR2_EXTI7_Pos)        /*!< 0x10000000 */
#define EXTI_EXTICR2_EXTI7_5         (0x20UL << EXTI_EXTICR2_EXTI7_Pos)        /*!< 0x20000000 */
#define EXTI_EXTICR2_EXTI7_6         (0x40UL << EXTI_EXTICR2_EXTI7_Pos)        /*!< 0x40000000 */
#define EXTI_EXTICR2_EXTI7_7         (0x80UL << EXTI_EXTICR2_EXTI7_Pos)        /*!< 0x80000000 */

/*****************  Bit definition for EXTI_EXTICR3 register  **************/
#define EXTI_EXTICR3_EXTI8_Pos       (0UL)
#define EXTI_EXTICR3_EXTI8_Msk       (0xFFUL << EXTI_EXTICR3_EXTI8_Pos)        /*!< 0x000000FF */
#define EXTI_EXTICR3_EXTI8           EXTI_EXTICR3_EXTI8_Msk                    /*!< EXTI 8 configuration */
#define EXTI_EXTICR3_EXTI8_0         (0x1UL << EXTI_EXTICR3_EXTI8_Pos)         /*!< 0x00000001 */
#define EXTI_EXTICR3_EXTI8_1         (0x2UL << EXTI_EXTICR3_EXTI8_Pos)         /*!< 0x00000002 */
#define EXTI_EXTICR3_EXTI8_2         (0x4UL << EXTI_EXTICR3_EXTI8_Pos)         /*!< 0x00000004 */
#define EXTI_EXTICR3_EXTI8_3         (0x8UL << EXTI_EXTICR3_EXTI8_Pos)         /*!< 0x00000008 */
#define EXTI_EXTICR3_EXTI8_4         (0x10UL << EXTI_EXTICR3_EXTI8_Pos)        /*!< 0x00000010 */
#define EXTI_EXTICR3_EXTI8_5         (0x20UL << EXTI_EXTICR3_EXTI8_Pos)        /*!< 0x00000020 */
#define EXTI_EXTICR3_EXTI8_6         (0x40UL << EXTI_EXTICR3_EXTI8_Pos)        /*!< 0x00000040 */
#define EXTI_EXTICR3_EXTI8_7         (0x80UL << EXTI_EXTICR3_EXTI8_Pos)        /*!< 0x00000080 */
#define EXTI_EXTICR3_EXTI9_Pos       (8UL)
#define EXTI_EXTICR3_EXTI9_Msk       (0xFFUL << EXTI_EXTICR3_EXTI9_Pos)        /*!< 0x0000FF00 */
#define EXTI_EXTICR3_EXTI9           EXTI_EXTICR3_EXTI9_Msk                    /*!< EXTI 9 configuration */
#define EXTI_EXTICR3_EXTI9_0         (0x1UL << EXTI_EXTICR3_EXTI9_Pos)         /*!< 0x00000100 */
#define EXTI_EXTICR3_EXTI9_1         (0x2UL << EXTI_EXTICR3_EXTI9_Pos)         /*!< 0x00000200 */
#define EXTI_EXTICR3_EXTI9_2         (0x4UL << EXTI_EXTICR3_EXTI9_Pos)         /*!< 0x00000400 */
#define EXTI_EXTICR3_EXTI9_3         (0x8UL << EXTI_EXTICR3_EXTI9_Pos)         /*!< 0x00000800 */
#define EXTI_EXTICR3_EXTI9_4         (0x10UL << EXTI_EXTICR3_EXTI9_Pos)        /*!< 0x00001000 */
#define EXTI_EXTICR3_EXTI9_5         (0x20UL << EXTI_EXTICR3_EXTI9_Pos)        /*!< 0x00002000 */
#define EXTI_EXTICR3_EXTI9_6         (0x40UL << EXTI_EXTICR3_EXTI9_Pos)        /*!< 0x00004000 */
#define EXTI_EXTICR3_EXTI9_7         (0x80UL << EXTI_EXTICR3_EXTI9_Pos)        /*!< 0x00008000 */
#define EXTI_EXTICR3_EXTI10_Pos      (16UL)
#define EXTI_EXTICR3_EXTI10_Msk      (0xFFUL << EXTI_EXTICR3_EXTI10_Pos)       /*!< 0x00FF0000 */
#define EXTI_EXTICR3_EXTI10          EXTI_EXTICR3_EXTI10_Msk                   /*!< EXTI 10 configuration */
#define EXTI_EXTICR3_EXTI10_0        (0x1UL << EXTI_EXTICR3_EXTI10_Pos)        /*!< 0x00010000 */
#define EXTI_EXTICR3_EXTI10_1        (0x2UL << EXTI_EXTICR3_EXTI10_Pos)        /*!< 0x00020000 */
#define EXTI_EXTICR3_EXTI10_2        (0x4UL << EXTI_EXTICR3_EXTI10_Pos)        /*!< 0x00040000 */
#define EXTI_EXTICR3_EXTI10_3        (0x8UL << EXTI_EXTICR3_EXTI10_Pos)        /*!< 0x00080000 */
#define EXTI_EXTICR3_EXTI10_4        (0x10UL << EXTI_EXTICR3_EXTI10_Pos)       /*!< 0x00100000 */
#define EXTI_EXTICR3_EXTI10_5        (0x20UL << EXTI_EXTICR3_EXTI10_Pos)       /*!< 0x00200000 */
#define EXTI_EXTICR3_EXTI10_6        (0x40UL << EXTI_EXTICR3_EXTI10_Pos)       /*!< 0x00400000 */
#define EXTI_EXTICR3_EXTI10_7        (0x80UL << EXTI_EXTICR3_EXTI10_Pos)       /*!< 0x00800000 */
#define EXTI_EXTICR3_EXTI11_Pos      (24UL)
#define EXTI_EXTICR3_EXTI11_Msk      (0xFFUL << EXTI_EXTICR3_EXTI11_Pos)       /*!< 0xFF000000 */
#define EXTI_EXTICR3_EXTI11          EXTI_EXTICR3_EXTI11_Msk                   /*!< EXTI 11 configuration */
#define EXTI_EXTICR3_EXTI11_0        (0x1UL << EXTI_EXTICR3_EXTI11_Pos)        /*!< 0x01000000 */
#define EXTI_EXTICR3_EXTI11_1        (0x2UL << EXTI_EXTICR3_EXTI11_Pos)        /*!< 0x02000000 */
#define EXTI_EXTICR3_EXTI11_2        (0x4UL << EXTI_EXTICR3_EXTI11_Pos)        /*!< 0x04000000 */
#define EXTI_EXTICR3_EXTI11_3        (0x8UL << EXTI_EXTICR3_EXTI11_Pos)        /*!< 0x08000000 */
#define EXTI_EXTICR3_EXTI11_4        (0x10UL << EXTI_EXTICR3_EXTI11_Pos)       /*!< 0x10000000 */
#define EXTI_EXTICR3_EXTI11_5        (0x20UL << EXTI_EXTICR3_EXTI11_Pos)       /*!< 0x20000000 */
#define EXTI_EXTICR3_EXTI11_6        (0x40UL << EXTI_EXTICR3_EXTI11_Pos)       /*!< 0x40000000 */
#define EXTI_EXTICR3_EXTI11_7        (0x80UL << EXTI_EXTICR3_EXTI11_Pos)       /*!< 0x80000000 */

/*****************  Bit definition for EXTI_EXTICR4 register  **************/
#define EXTI_EXTICR4_EXTI12_Pos      (0UL)
#define EXTI_EXTICR4_EXTI12_Msk      (0xFFUL << EXTI_EXTICR4_EXTI12_Pos)       /*!< 0x000000FF */
#define EXTI_EXTICR4_EXTI12          EXTI_EXTICR4_EXTI12_Msk                   /*!< EXTI 12 configuration */
#define EXTI_EXTICR4_EXTI12_0        (0x1UL << EXTI_EXTICR4_EXTI12_Pos)        /*!< 0x00000001 */
#define EXTI_EXTICR4_EXTI12_1        (0x2UL << EXTI_EXTICR4_EXTI12_Pos)        /*!< 0x00000002 */
#define EXTI_EXTICR4_EXTI12_2        (0x4UL << EXTI_EXTICR4_EXTI12_Pos)        /*!< 0x00000004 */
#define EXTI_EXTICR4_EXTI12_3        (0x8UL << EXTI_EXTICR4_EXTI12_Pos)        /*!< 0x00000008 */
#define EXTI_EXTICR4_EXTI12_4        (0x10UL << EXTI_EXTICR4_EXTI12_Pos)       /*!< 0x00000010 */
#define EXTI_EXTICR4_EXTI12_5        (0x20UL << EXTI_EXTICR4_EXTI12_Pos)       /*!< 0x00000020 */
#define EXTI_EXTICR4_EXTI12_6        (0x40UL << EXTI_EXTICR4_EXTI12_Pos)       /*!< 0x00000040 */
#define EXTI_EXTICR4_EXTI12_7        (0x80UL << EXTI_EXTICR4_EXTI12_Pos)       /*!< 0x00000080 */
#define EXTI_EXTICR4_EXTI13_Pos      (8UL)
#define EXTI_EXTICR4_EXTI13_Msk      (0xFFUL << EXTI_EXTICR4_EXTI13_Pos)       /*!< 0x0000FF00 */
#define EXTI_EXTICR4_EXTI13          EXTI_EXTICR4_EXTI13_Msk                   /*!< EXTI 13 configuration */
#define EXTI_EXTICR4_EXTI13_0        (0x1UL << EXTI_EXTICR4_EXTI13_Pos)        /*!< 0x00000100 */
#define EXTI_EXTICR4_EXTI13_1        (0x2UL << EXTI_EXTICR4_EXTI13_Pos)        /*!< 0x00000200 */
#define EXTI_EXTICR4_EXTI13_2        (0x4UL << EXTI_EXTICR4_EXTI13_Pos)        /*!< 0x00000400 */
#define EXTI_EXTICR4_EXTI13_3        (0x8UL << EXTI_EXTICR4_EXTI13_Pos)        /*!< 0x00000800 */
#define EXTI_EXTICR4_EXTI13_4        (0x10UL << EXTI_EXTICR4_EXTI13_Pos)       /*!< 0x00001000 */
#define EXTI_EXTICR4_EXTI13_5        (0x20UL << EXTI_EXTICR4_EXTI13_Pos)       /*!< 0x00002000 */
#define EXTI_EXTICR4_EXTI13_6        (0x40UL << EXTI_EXTICR4_EXTI13_Pos)       /*!< 0x00004000 */
#define EXTI_EXTICR4_EXTI13_7        (0x80UL << EXTI_EXTICR4_EXTI13_Pos)       /*!< 0x00008000 */
#define EXTI_EXTICR4_EXTI14_Pos      (16UL)
#define EXTI_EXTICR4_EXTI14_Msk      (0xFFUL << EXTI_EXTICR4_EXTI14_Pos)       /*!< 0x00FF0000 */
#define EXTI_EXTICR4_EXTI14          EXTI_EXTICR4_EXTI14_Msk                   /*!< EXTI 14 configuration */
#define EXTI_EXTICR4_EXTI14_0        (0x1UL << EXTI_EXTICR4_EXTI14_Pos)        /*!< 0x00010000 */
#define EXTI_EXTICR4_EXTI14_1        (0x2UL << EXTI_EXTICR4_EXTI14_Pos)        /*!< 0x00020000 */
#define EXTI_EXTICR4_EXTI14_2        (0x4UL << EXTI_EXTICR4_EXTI14_Pos)        /*!< 0x00040000 */
#define EXTI_EXTICR4_EXTI14_3        (0x8UL << EXTI_EXTICR4_EXTI14_Pos)        /*!< 0x00080000 */
#define EXTI_EXTICR4_EXTI14_4        (0x10UL << EXTI_EXTICR4_EXTI14_Pos)       /*!< 0x00100000 */
#define EXTI_EXTICR4_EXTI14_5        (0x20UL << EXTI_EXTICR4_EXTI14_Pos)       /*!< 0x00200000 */
#define EXTI_EXTICR4_EXTI14_6        (0x40UL << EXTI_EXTICR4_EXTI14_Pos)       /*!< 0x00400000 */
#define EXTI_EXTICR4_EXTI14_7        (0x80UL << EXTI_EXTICR4_EXTI14_Pos)       /*!< 0x00800000 */
#define EXTI_EXTICR4_EXTI15_Pos      (24UL)
#define EXTI_EXTICR4_EXTI15_Msk      (0xFFUL << EXTI_EXTICR4_EXTI15_Pos)       /*!< 0xFF000000 */
#define EXTI_EXTICR4_EXTI15          EXTI_EXTICR4_EXTI15_Msk                   /*!< EXTI 15 configuration */
#define EXTI_EXTICR4_EXTI15_0        (0x1UL << EXTI_EXTICR4_EXTI15_Pos)        /*!< 0x01000000 */
#define EXTI_EXTICR4_EXTI15_1        (0x2UL << EXTI_EXTICR4_EXTI15_Pos)        /*!< 0x02000000 */
#define EXTI_EXTICR4_EXTI15_2        (0x4UL << EXTI_EXTICR4_EXTI15_Pos)        /*!< 0x04000000 */
#define EXTI_EXTICR4_EXTI15_3        (0x8UL << EXTI_EXTICR4_EXTI15_Pos)        /*!< 0x08000000 */
#define EXTI_EXTICR4_EXTI15_4        (0x10UL << EXTI_EXTICR4_EXTI15_Pos)       /*!< 0x10000000 */
#define EXTI_EXTICR4_EXTI15_5        (0x20UL << EXTI_EXTICR4_EXTI15_Pos)       /*!< 0x20000000 */
#define EXTI_EXTICR4_EXTI15_6        (0x40UL << EXTI_EXTICR4_EXTI15_Pos)       /*!< 0x40000000 */
#define EXTI_EXTICR4_EXTI15_7        (0x80UL << EXTI_EXTICR4_EXTI15_Pos)       /*!< 0x80000000 */

/*******************  Bit definition for EXTI_IMR1 register  *******************/
#define EXTI_IMR1_IM_Pos           (0UL)
#define EXTI_IMR1_IM_Msk           (0xFFFFFFFFUL << EXTI_IMR1_IM_Pos)          /*!< 0xFFFFFFFF */
#define EXTI_IMR1_IM               EXTI_IMR1_IM_Msk                            /*!< Interrupt Mask */
#define EXTI_IMR1_IM0_Pos          (0UL)
#define EXTI_IMR1_IM0_Msk          (0x1UL << EXTI_IMR1_IM0_Pos)                /*!< 0x00000001 */
#define EXTI_IMR1_IM0              EXTI_IMR1_IM0_Msk                           /*!< Interrupt Mask on line 0 */
#define EXTI_IMR1_IM1_Pos          (1UL)
#define EXTI_IMR1_IM1_Msk          (0x1UL << EXTI_IMR1_IM1_Pos)                /*!< 0x00000002 */
#define EXTI_IMR1_IM1              EXTI_IMR1_IM1_Msk                           /*!< Interrupt Mask on line 1 */
#define EXTI_IMR1_IM2_Pos          (2UL)
#define EXTI_IMR1_IM2_Msk          (0x1UL << EXTI_IMR1_IM2_Pos)                /*!< 0x00000004 */
#define EXTI_IMR1_IM2              EXTI_IMR1_IM2_Msk                           /*!< Interrupt Mask on line 2 */
#define EXTI_IMR1_IM3_Pos          (3UL)
#define EXTI_IMR1_IM3_Msk          (0x1UL << EXTI_IMR1_IM3_Pos)                /*!< 0x00000008 */
#define EXTI_IMR1_IM3              EXTI_IMR1_IM3_Msk                           /*!< Interrupt Mask on line 3 */
#define EXTI_IMR1_IM4_Pos          (4UL)
#define EXTI_IMR1_IM4_Msk          (0x1UL << EXTI_IMR1_IM4_Pos)                /*!< 0x00000010 */
#define EXTI_IMR1_IM4              EXTI_IMR1_IM4_Msk                           /*!< Interrupt Mask on line 4 */
#define EXTI_IMR1_IM5_Pos          (5UL)
#define EXTI_IMR1_IM5_Msk          (0x1UL << EXTI_IMR1_IM5_Pos)                /*!< 0x00000020 */
#define EXTI_IMR1_IM5              EXTI_IMR1_IM5_Msk                           /*!< Interrupt Mask on line 5 */
#define EXTI_IMR1_IM6_Pos          (6UL)
#define EXTI_IMR1_IM6_Msk          (0x1UL << EXTI_IMR1_IM6_Pos)                /*!< 0x00000040 */
#define EXTI_IMR1_IM6              EXTI_IMR1_IM6_Msk                           /*!< Interrupt Mask on line 6 */
#define EXTI_IMR1_IM7_Pos          (7UL)
#define EXTI_IMR1_IM7_Msk          (0x1UL << EXTI_IMR1_IM7_Pos)                /*!< 0x00000080 */
#define EXTI_IMR1_IM7              EXTI_IMR1_IM7_Msk                           /*!< Interrupt Mask on line 7 */
#define EXTI_IMR1_IM8_Pos          (8UL)
#define EXTI_IMR1_IM8_Msk          (0x1UL << EXTI_IMR1_IM8_Pos)                /*!< 0x00000100 */
#define EXTI_IMR1_IM8              EXTI_IMR1_IM8_Msk                           /*!< Interrupt Mask on line 8 */
#define EXTI_IMR1_IM9_Pos          (9UL)
#define EXTI_IMR1_IM9_Msk          (0x1UL << EXTI_IMR1_IM9_Pos)                /*!< 0x00000200 */
#define EXTI_IMR1_IM9              EXTI_IMR1_IM9_Msk                           /*!< Interrupt Mask on line 9 */
#define EXTI_IMR1_IM10_Pos         (10UL)
#define EXTI_IMR1_IM10_Msk         (0x1UL << EXTI_IMR1_IM10_Pos)               /*!< 0x00000400 */
#define EXTI_IMR1_IM10             EXTI_IMR1_IM10_Msk                          /*!< Interrupt Mask on line 10 */
#define EXTI_IMR1_IM11_Pos         (11UL)
#define EXTI_IMR1_IM11_Msk         (0x1UL << EXTI_IMR1_IM11_Pos)               /*!< 0x00000800 */
#define EXTI_IMR1_IM11             EXTI_IMR1_IM11_Msk                          /*!< Interrupt Mask on line 11 */
#define EXTI_IMR1_IM12_Pos         (12UL)
#define EXTI_IMR1_IM12_Msk         (0x1UL << EXTI_IMR1_IM12_Pos)               /*!< 0x00001000 */
#define EXTI_IMR1_IM12             EXTI_IMR1_IM12_Msk                          /*!< Interrupt Mask on line 12 */
#define EXTI_IMR1_IM13_Pos         (13UL)
#define EXTI_IMR1_IM13_Msk         (0x1UL << EXTI_IMR1_IM13_Pos)               /*!< 0x00002000 */
#define EXTI_IMR1_IM13             EXTI_IMR1_IM13_Msk                          /*!< Interrupt Mask on line 13 */
#define EXTI_IMR1_IM14_Pos         (14UL)
#define EXTI_IMR1_IM14_Msk         (0x1UL << EXTI_IMR1_IM14_Pos)               /*!< 0x00004000 */
#define EXTI_IMR1_IM14             EXTI_IMR1_IM14_Msk                          /*!< Interrupt Mask on line 14 */
#define EXTI_IMR1_IM15_Pos         (15UL)
#define EXTI_IMR1_IM15_Msk         (0x1UL << EXTI_IMR1_IM15_Pos)               /*!< 0x00008000 */
#define EXTI_IMR1_IM15             EXTI_IMR1_IM15_Msk                          /*!< Interrupt Mask on line 15 */
#define EXTI_IMR1_IM16_Pos         (16UL)
#define EXTI_IMR1_IM16_Msk         (0x1UL << EXTI_IMR1_IM16_Pos)               /*!< 0x00010000 */
#define EXTI_IMR1_IM16             EXTI_IMR1_IM16_Msk                          /*!< Interrupt Mask on line 16 */
#define EXTI_IMR1_IM17_Pos         (17UL)
#define EXTI_IMR1_IM17_Msk         (0x1UL << EXTI_IMR1_IM17_Pos)               /*!< 0x00020000 */
#define EXTI_IMR1_IM17             EXTI_IMR1_IM17_Msk                          /*!< Interrupt Mask on line 17 */
#define EXTI_IMR1_IM18_Pos         (18UL)
#define EXTI_IMR1_IM18_Msk         (0x1UL << EXTI_IMR1_IM18_Pos)               /*!< 0x00040000 */
#define EXTI_IMR1_IM18             EXTI_IMR1_IM18_Msk                          /*!< Interrupt Mask on line 18 */
#define EXTI_IMR1_IM19_Pos         (19UL)
#define EXTI_IMR1_IM19_Msk         (0x1UL << EXTI_IMR1_IM19_Pos)               /*!< 0x00080000 */
#define EXTI_IMR1_IM19             EXTI_IMR1_IM19_Msk                          /*!< Interrupt Mask on line 19 */
#define EXTI_IMR1_IM20_Pos         (20UL)
#define EXTI_IMR1_IM20_Msk         (0x1UL << EXTI_IMR1_IM20_Pos)               /*!< 0x00100000 */
#define EXTI_IMR1_IM20             EXTI_IMR1_IM20_Msk                          /*!< Interrupt Mask on line 20 */
#define EXTI_IMR1_IM21_Pos         (21UL)
#define EXTI_IMR1_IM21_Msk         (0x1UL << EXTI_IMR1_IM21_Pos)               /*!< 0x00200000 */
#define EXTI_IMR1_IM21             EXTI_IMR1_IM21_Msk                          /*!< Interrupt Mask on line 21 */
#define EXTI_IMR1_IM22_Pos         (22UL)
#define EXTI_IMR1_IM22_Msk         (0x1UL << EXTI_IMR1_IM22_Pos)               /*!< 0x00400000 */
#define EXTI_IMR1_IM22             EXTI_IMR1_IM22_Msk                          /*!< Interrupt Mask on line 22 */
#define EXTI_IMR1_IM23_Pos         (23UL)
#define EXTI_IMR1_IM23_Msk         (0x1UL << EXTI_IMR1_IM23_Pos)               /*!< 0x00800000 */
#define EXTI_IMR1_IM23             EXTI_IMR1_IM23_Msk                          /*!< Interrupt Mask on line 23 */
#define EXTI_IMR1_IM24_Pos         (24UL)
#define EXTI_IMR1_IM24_Msk         (0x1UL << EXTI_IMR1_IM24_Pos)               /*!< 0x01000000 */
#define EXTI_IMR1_IM24             EXTI_IMR1_IM24_Msk                          /*!< Interrupt Mask on line 24 */
#define EXTI_IMR1_IM25_Pos         (25UL)
#define EXTI_IMR1_IM25_Msk         (0x1UL << EXTI_IMR1_IM25_Pos)               /*!< 0x02000000 */
#define EXTI_IMR1_IM25             EXTI_IMR1_IM25_Msk                          /*!< Interrupt Mask on line 25 */
#define EXTI_IMR1_IM26_Pos         (26UL)
#define EXTI_IMR1_IM26_Msk         (0x1UL << EXTI_IMR1_IM26_Pos)               /*!< 0x04000000 */
#define EXTI_IMR1_IM26             EXTI_IMR1_IM26_Msk                          /*!< Interrupt Mask on line 26 */
#define EXTI_IMR1_IM27_Pos         (27UL)
#define EXTI_IMR1_IM27_Msk         (0x1UL << EXTI_IMR1_IM27_Pos)               /*!< 0x08000000 */
#define EXTI_IMR1_IM27             EXTI_IMR1_IM27_Msk                          /*!< Interrupt Mask on line 27 */
#define EXTI_IMR1_IM28_Pos         (28UL)
#define EXTI_IMR1_IM28_Msk         (0x1UL << EXTI_IMR1_IM28_Pos)               /*!< 0x10000000 */
#define EXTI_IMR1_IM28             EXTI_IMR1_IM28_Msk                          /*!< Interrupt Mask on line 28 */
#define EXTI_IMR1_IM29_Pos         (29UL)
#define EXTI_IMR1_IM29_Msk         (0x1UL << EXTI_IMR1_IM29_Pos)               /*!< 0x20000000 */
#define EXTI_IMR1_IM29             EXTI_IMR1_IM29_Msk                          /*!< Interrupt Mask on line 29 */
#define EXTI_IMR1_IM30_Pos         (30UL)
#define EXTI_IMR1_IM30_Msk         (0x1UL << EXTI_IMR1_IM30_Pos)               /*!< 0x40000000 */
#define EXTI_IMR1_IM30             EXTI_IMR1_IM30_Msk                          /*!< Interrupt Mask on line 30 */
#define EXTI_IMR1_IM31_Pos         (31UL)
#define EXTI_IMR1_IM31_Msk         (0x1UL << EXTI_IMR1_IM31_Pos)               /*!< 0x80000000 */
#define EXTI_IMR1_IM31             EXTI_IMR1_IM31_Msk                          /*!< Interrupt Mask on line 31 */

/*******************  Bit definition for EXTI_EMR1 register  *******************/
#define EXTI_EMR1_EM_Pos           (0UL)
#define EXTI_EMR1_EM_Msk           (0xFFFFFFFFUL << EXTI_EMR1_EM_Pos)          /*!< 0xFFFFFFFF */
#define EXTI_EMR1_EM               EXTI_EMR1_EM_Msk                            /*!< Event Mask */
#define EXTI_EMR1_EM0_Pos          (0UL)
#define EXTI_EMR1_EM0_Msk          (0x1UL << EXTI_EMR1_EM0_Pos)                /*!< 0x00000001 */
#define EXTI_EMR1_EM0              EXTI_EMR1_EM0_Msk                           /*!< Event Mask on line 0 */
#define EXTI_EMR1_EM1_Pos          (1UL)
#define EXTI_EMR1_EM1_Msk          (0x1UL << EXTI_EMR1_EM1_Pos)                /*!< 0x00000002 */
#define EXTI_EMR1_EM1              EXTI_EMR1_EM1_Msk                           /*!< Event Mask on line 1 */
#define EXTI_EMR1_EM2_Pos          (2UL)
#define EXTI_EMR1_EM2_Msk          (0x1UL << EXTI_EMR1_EM2_Pos)                /*!< 0x00000004 */
#define EXTI_EMR1_EM2              EXTI_EMR1_EM2_Msk                           /*!< Event Mask on line 2 */
#define EXTI_EMR1_EM3_Pos          (3UL)
#define EXTI_EMR1_EM3_Msk          (0x1UL << EXTI_EMR1_EM3_Pos)                /*!< 0x00000008 */
#define EXTI_EMR1_EM3              EXTI_EMR1_EM3_Msk                           /*!< Event Mask on line 3 */
#define EXTI_EMR1_EM4_Pos          (4UL)
#define EXTI_EMR1_EM4_Msk          (0x1UL << EXTI_EMR1_EM4_Pos)                /*!< 0x00000010 */
#define EXTI_EMR1_EM4              EXTI_EMR1_EM4_Msk                           /*!< Event Mask on line 4 */
#define EXTI_EMR1_EM5_Pos          (5UL)
#define EXTI_EMR1_EM5_Msk          (0x1UL << EXTI_EMR1_EM5_Pos)                /*!< 0x00000020 */
#define EXTI_EMR1_EM5              EXTI_EMR1_EM5_Msk                           /*!< Event Mask on line 5 */
#define EXTI_EMR1_EM6_Pos          (6UL)
#define EXTI_EMR1_EM6_Msk          (0x1UL << EXTI_EMR1_EM6_Pos)                /*!< 0x00000040 */
#define EXTI_EMR1_EM6              EXTI_EMR1_EM6_Msk                           /*!< Event Mask on line 6 */
#define EXTI_EMR1_EM7_Pos          (7UL)
#define EXTI_EMR1_EM7_Msk          (0x1UL << EXTI_EMR1_EM7_Pos)                /*!< 0x00000080 */
#define EXTI_EMR1_EM7              EXTI_EMR1_EM7_Msk                           /*!< Event Mask on line 7 */
#define EXTI_EMR1_EM8_Pos          (8UL)
#define EXTI_EMR1_EM8_Msk          (0x1UL << EXTI_EMR1_EM8_Pos)                /*!< 0x00000100 */
#define EXTI_EMR1_EM8              EXTI_EMR1_EM8_Msk                           /*!< Event Mask on line 8 */
#define EXTI_EMR1_EM9_Pos          (9UL)
#define EXTI_EMR1_EM9_Msk          (0x1UL << EXTI_EMR1_EM9_Pos)                /*!< 0x00000200 */
#define EXTI_EMR1_EM9              EXTI_EMR1_EM9_Msk                           /*!< Event Mask on line 9 */
#define EXTI_EMR1_EM10_Pos         (10UL)
#define EXTI_EMR1_EM10_Msk         (0x1UL << EXTI_EMR1_EM10_Pos)               /*!< 0x00000400 */
#define EXTI_EMR1_EM10             EXTI_EMR1_EM10_Msk                          /*!< Event Mask on line 10 */
#define EXTI_EMR1_EM11_Pos         (11UL)
#define EXTI_EMR1_EM11_Msk         (0x1UL << EXTI_EMR1_EM11_Pos)               /*!< 0x00000800 */
#define EXTI_EMR1_EM11             EXTI_EMR1_EM11_Msk                          /*!< Event Mask on line 11 */
#define EXTI_EMR1_EM12_Pos         (12UL)
#define EXTI_EMR1_EM12_Msk         (0x1UL << EXTI_EMR1_EM12_Pos)               /*!< 0x00001000 */
#define EXTI_EMR1_EM12             EXTI_EMR1_EM12_Msk                          /*!< Event Mask on line 12 */
#define EXTI_EMR1_EM13_Pos         (13UL)
#define EXTI_EMR1_EM13_Msk         (0x1UL << EXTI_EMR1_EM13_Pos)               /*!< 0x00002000 */
#define EXTI_EMR1_EM13             EXTI_EMR1_EM13_Msk                          /*!< Event Mask on line 13 */
#define EXTI_EMR1_EM14_Pos         (14UL)
#define EXTI_EMR1_EM14_Msk         (0x1UL << EXTI_EMR1_EM14_Pos)               /*!< 0x00004000 */
#define EXTI_EMR1_EM14             EXTI_EMR1_EM14_Msk                          /*!< Event Mask on line 14 */
#define EXTI_EMR1_EM15_Pos         (15UL)
#define EXTI_EMR1_EM15_Msk         (0x1UL << EXTI_EMR1_EM15_Pos)               /*!< 0x00008000 */
#define EXTI_EMR1_EM15             EXTI_EMR1_EM15_Msk                          /*!< Event Mask on line 15 */
#define EXTI_EMR1_EM16_Pos         (16UL)
#define EXTI_EMR1_EM16_Msk         (0x1UL << EXTI_EMR1_EM16_Pos)               /*!< 0x00010000 */
#define EXTI_EMR1_EM16             EXTI_EMR1_EM16_Msk                          /*!< Event Mask on line 16 */
#define EXTI_EMR1_EM17_Pos         (17UL)
#define EXTI_EMR1_EM17_Msk         (0x1UL << EXTI_EMR1_EM17_Pos)               /*!< 0x00020000 */
#define EXTI_EMR1_EM17             EXTI_EMR1_EM17_Msk                          /*!< Event Mask on line 17 */
#define EXTI_EMR1_EM18_Pos         (18UL)
#define EXTI_EMR1_EM18_Msk         (0x1UL << EXTI_EMR1_EM18_Pos)               /*!< 0x00040000 */
#define EXTI_EMR1_EM18             EXTI_EMR1_EM18_Msk                          /*!< Event Mask on line 18 */
#define EXTI_EMR1_EM19_Pos         (19UL)
#define EXTI_EMR1_EM19_Msk         (0x1UL << EXTI_EMR1_EM19_Pos)               /*!< 0x00080000 */
#define EXTI_EMR1_EM19             EXTI_EMR1_EM19_Msk                          /*!< Event Mask on line 19 */
#define EXTI_EMR1_EM20_Pos         (20UL)
#define EXTI_EMR1_EM20_Msk         (0x1UL << EXTI_EMR1_EM20_Pos)               /*!< 0x00100000 */
#define EXTI_EMR1_EM20             EXTI_EMR1_EM20_Msk                          /*!< Event Mask on line 20 */
#define EXTI_EMR1_EM21_Pos         (21UL)
#define EXTI_EMR1_EM21_Msk         (0x1UL << EXTI_EMR1_EM21_Pos)               /*!< 0x00200000 */
#define EXTI_EMR1_EM21             EXTI_EMR1_EM21_Msk                          /*!< Event Mask on line 21 */
#define EXTI_EMR1_EM22_Pos         (22UL)
#define EXTI_EMR1_EM22_Msk         (0x1UL << EXTI_EMR1_EM22_Pos)               /*!< 0x00400000 */
#define EXTI_EMR1_EM22             EXTI_EMR1_EM22_Msk                          /*!< Event Mask on line 22 */
#define EXTI_EMR1_EM23_Pos         (23UL)
#define EXTI_EMR1_EM23_Msk         (0x1UL << EXTI_EMR1_EM23_Pos)               /*!< 0x00800000 */
#define EXTI_EMR1_EM23             EXTI_EMR1_EM23_Msk                          /*!< Event Mask on line 23 */
#define EXTI_EMR1_EM24_Pos         (24UL)
#define EXTI_EMR1_EM24_Msk         (0x1UL << EXTI_EMR1_EM24_Pos)               /*!< 0x01000000 */
#define EXTI_EMR1_EM24             EXTI_EMR1_EM24_Msk                          /*!< Event Mask on line 24 */
#define EXTI_EMR1_EM25_Pos         (25UL)
#define EXTI_EMR1_EM25_Msk         (0x1UL << EXTI_EMR1_EM25_Pos)               /*!< 0x02000000 */
#define EXTI_EMR1_EM25             EXTI_EMR1_EM25_Msk                          /*!< Event Mask on line 25 */
#define EXTI_EMR1_EM26_Pos         (26UL)
#define EXTI_EMR1_EM26_Msk         (0x1UL << EXTI_EMR1_EM26_Pos)               /*!< 0x04000000 */
#define EXTI_EMR1_EM26             EXTI_EMR1_EM26_Msk                          /*!< Event Mask on line 26 */
#define EXTI_EMR1_EM27_Pos         (27UL)
#define EXTI_EMR1_EM27_Msk         (0x1UL << EXTI_EMR1_EM27_Pos)               /*!< 0x08000000 */
#define EXTI_EMR1_EM27             EXTI_EMR1_EM27_Msk                          /*!< Event Mask on line 27 */
#define EXTI_EMR1_EM28_Pos         (28UL)
#define EXTI_EMR1_EM28_Msk         (0x1UL << EXTI_EMR1_EM28_Pos)               /*!< 0x10000000 */
#define EXTI_EMR1_EM28             EXTI_EMR1_EM28_Msk                          /*!< Event Mask on line 28 */
#define EXTI_EMR1_EM29_Pos         (29UL)
#define EXTI_EMR1_EM29_Msk         (0x1UL << EXTI_EMR1_EM29_Pos)               /*!< 0x20000000 */
#define EXTI_EMR1_EM29             EXTI_EMR1_EM29_Msk                          /*!< Event Mask on line 29 */
#define EXTI_EMR1_EM30_Pos         (30UL)
#define EXTI_EMR1_EM30_Msk         (0x1UL << EXTI_EMR1_EM30_Pos)               /*!< 0x40000000 */
#define EXTI_EMR1_EM30             EXTI_EMR1_EM30_Msk                          /*!< Event Mask on line 30 */
#define EXTI_EMR1_EM31_Pos         (31UL)
#define EXTI_EMR1_EM31_Msk         (0x1UL << EXTI_EMR1_EM31_Pos)               /*!< 0x80000000 */
#define EXTI_EMR1_EM31             EXTI_EMR1_EM31_Msk                          /*!< Event Mask on line 31 */

/*******************  Bit definition for EXTI_IMR2 register  *******************/
#define EXTI_IMR2_IM_Pos           (0UL)
#define EXTI_IMR2_IM_Msk           (0x0000003FUL << EXTI_IMR2_IM_Pos)          /*!< 0xFFFF8FFF */
#define EXTI_IMR2_IM               EXTI_IMR2_IM_Msk                            /*!< Interrupt Mask            */
#define EXTI_IMR2_IM32_Pos         (0UL)
#define EXTI_IMR2_IM32_Msk         (0x1UL << EXTI_IMR2_IM32_Pos)               /*!< 0x00000001 */
#define EXTI_IMR2_IM32             EXTI_IMR2_IM32_Msk                          /*!< Interrupt Mask on line 32 */
#define EXTI_IMR2_IM33_Pos         (1UL)
#define EXTI_IMR2_IM33_Msk         (0x1UL << EXTI_IMR2_IM33_Pos)               /*!< 0x00000002 */
#define EXTI_IMR2_IM33             EXTI_IMR2_IM33_Msk                          /*!< Interrupt Mask on line 33 */
#define EXTI_IMR2_IM34_Pos         (2UL)
#define EXTI_IMR2_IM34_Msk         (0x1UL << EXTI_IMR2_IM34_Pos)               /*!< 0x00000004 */
#define EXTI_IMR2_IM34             EXTI_IMR2_IM34_Msk                          /*!< Interrupt Mask on line 34 */
#define EXTI_IMR2_IM35_Pos         (3UL)
#define EXTI_IMR2_IM35_Msk         (0x1UL << EXTI_IMR2_IM35_Pos)               /*!< 0x00000008 */
#define EXTI_IMR2_IM35             EXTI_IMR2_IM35_Msk                          /*!< Interrupt Mask on line 35 */
#define EXTI_IMR2_IM36_Pos         (4UL)
#define EXTI_IMR2_IM36_Msk         (0x1UL << EXTI_IMR2_IM36_Pos)               /*!< 0x00000010 */
#define EXTI_IMR2_IM36             EXTI_IMR2_IM36_Msk                          /*!< Interrupt Mask on line 36 */
#define EXTI_IMR2_IM37_Pos         (5UL)
#define EXTI_IMR2_IM37_Msk         (0x1UL << EXTI_IMR2_IM37_Pos)               /*!< 0x00000020 */
#define EXTI_IMR2_IM37             EXTI_IMR2_IM37_Msk                          /*!< Interrupt Mask on line 37 */

/*******************  Bit definition for EXTI_EMR2 register  *******************/
#define EXTI_EMR2_EM_Pos           (0UL)
#define EXTI_EMR2_EM_Msk           (0xFFFF8FFFUL << EXTI_EMR2_EM_Pos)          /*!< 0xFFFF8FFF */
#define EXTI_EMR2_EM               EXTI_EMR2_EM_Msk                            /*!< Event Mask           */
#define EXTI_EMR2_EM32_Pos         (0UL)
#define EXTI_EMR2_EM32_Msk         (0x1UL << EXTI_EMR2_EM32_Pos)               /*!< 0x00000001 */
#define EXTI_EMR2_EM32             EXTI_EMR2_EM32_Msk                          /*!< Event Mask on line 32*/
#define EXTI_EMR2_EM33_Pos         (1UL)
#define EXTI_EMR2_EM33_Msk         (0x1UL << EXTI_EMR2_EM33_Pos)               /*!< 0x00000002 */
#define EXTI_EMR2_EM33             EXTI_EMR2_EM33_Msk                          /*!< Event Mask on line 33*/
#define EXTI_EMR2_EM34_Pos         (2UL)
#define EXTI_EMR2_EM34_Msk         (0x1UL << EXTI_EMR2_EM34_Pos)               /*!< 0x00000004 */
#define EXTI_EMR2_EM34             EXTI_EMR2_EM34_Msk                          /*!< Event Mask on line 34*/
#define EXTI_EMR2_EM35_Pos         (3UL)
#define EXTI_EMR2_EM35_Msk         (0x1UL << EXTI_EMR2_EM35_Pos)               /*!< 0x00000008 */
#define EXTI_EMR2_EM35             EXTI_EMR2_EM35_Msk                          /*!< Event Mask on line 35*/
#define EXTI_EMR2_EM36_Pos         (4UL)
#define EXTI_EMR2_EM36_Msk         (0x1UL << EXTI_EMR2_EM36_Pos)               /*!< 0x00000010 */
#define EXTI_EMR2_EM36             EXTI_EMR2_EM36_Msk                          /*!< Event Mask on line 36*/
#define EXTI_EMR2_EM37_Pos         (5UL)
#define EXTI_EMR2_EM37_Msk         (0x1UL << EXTI_EMR2_EM37_Pos)               /*!< 0x00000020 */
#define EXTI_EMR2_EM37             EXTI_EMR2_EM37_Msk                          /*!< Event Mask on line 37*/

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/

/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY_Pos               (0UL)
#define FLASH_ACR_LATENCY_Msk               (0x7UL << FLASH_ACR_LATENCY_Pos) /*!< 0x00000007 */
#define FLASH_ACR_LATENCY                   FLASH_ACR_LATENCY_Msk
#define FLASH_ACR_LATENCY_0                 (0x1UL << FLASH_ACR_LATENCY_Pos) /*!< 0x00000001 */
#define FLASH_ACR_LATENCY_1                 (0x2UL << FLASH_ACR_LATENCY_Pos) /*!< 0x00000002 */
#define FLASH_ACR_LATENCY_2                 (0x4UL << FLASH_ACR_LATENCY_Pos) /*!< 0x00000004 */
#define FLASH_ACR_LATENCY_0WS               (0x00000000UL)
#define FLASH_ACR_LATENCY_1WS               (0x00000001UL)
#define FLASH_ACR_LATENCY_2WS               (0x00000002UL)
#define FLASH_ACR_LATENCY_3WS               (0x00000003UL)
#define FLASH_ACR_LATENCY_4WS               (0x00000004UL)
#define FLASH_ACR_LATENCY_5WS               (0x00000005UL)
#define FLASH_ACR_LATENCY_6WS               (0x00000006UL)
#define FLASH_ACR_LATENCY_7WS               (0x00000007UL)
#define FLASH_ACR_PRFTEN_Pos                (8UL)
#define FLASH_ACR_PRFTEN_Msk                (0x1UL << FLASH_ACR_PRFTEN_Pos)     /*!< 0x00000100 */
#define FLASH_ACR_PRFTEN                    FLASH_ACR_PRFTEN_Msk
#define FLASH_ACR_ICEN_Pos                  (9UL)
#define FLASH_ACR_ICEN_Msk                  (0x1UL << FLASH_ACR_ICEN_Pos)       /*!< 0x00000200 */
#define FLASH_ACR_ICEN                      FLASH_ACR_ICEN_Msk
#define FLASH_ACR_ICRST_Pos                 (11UL)
#define FLASH_ACR_ICRST_Msk                 (0x1UL << FLASH_ACR_ICRST_Pos)      /*!< 0x00000800 */
#define FLASH_ACR_ICRST                     FLASH_ACR_ICRST_Msk
#define FLASH_ACR_EMPTY_Pos                 (16UL)
#define FLASH_ACR_EMPTY_Msk                 (0x1UL << FLASH_ACR_EMPTY_Pos)      /*!< 0x00010000 */
#define FLASH_ACR_EMPTY                     FLASH_ACR_EMPTY_Msk
#define FLASH_ACR_DBG_SWEN_Pos              (18UL)
#define FLASH_ACR_DBG_SWEN_Msk              (0x1UL << FLASH_ACR_DBG_SWEN_Pos)   /*!< 0x00040000 */
#define FLASH_ACR_DBG_SWEN                  FLASH_ACR_DBG_SWEN_Msk

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP_Pos                    (0UL)
#define FLASH_SR_EOP_Msk                    (0x1UL << FLASH_SR_EOP_Pos)             /*!< 0x00000001 */
#define FLASH_SR_EOP                        FLASH_SR_EOP_Msk
#define FLASH_SR_OPERR_Pos                  (1UL)
#define FLASH_SR_OPERR_Msk                  (0x1UL << FLASH_SR_OPERR_Pos)           /*!< 0x00000002 */
#define FLASH_SR_OPERR                      FLASH_SR_OPERR_Msk
#define FLASH_SR_PROGERR_Pos                (3UL)
#define FLASH_SR_PROGERR_Msk                (0x1UL << FLASH_SR_PROGERR_Pos)         /*!< 0x00000008 */
#define FLASH_SR_PROGERR                    FLASH_SR_PROGERR_Msk
#define FLASH_SR_WRPERR_Pos                 (4UL)
#define FLASH_SR_WRPERR_Msk                 (0x1UL << FLASH_SR_WRPERR_Pos)          /*!< 0x00000010 */
#define FLASH_SR_WRPERR                     FLASH_SR_WRPERR_Msk
#define FLASH_SR_PGAERR_Pos                 (5UL)
#define FLASH_SR_PGAERR_Msk                 (0x1UL << FLASH_SR_PGAERR_Pos)          /*!< 0x00000020 */
#define FLASH_SR_PGAERR                     FLASH_SR_PGAERR_Msk
#define FLASH_SR_SIZERR_Pos                 (6UL)
#define FLASH_SR_SIZERR_Msk                 (0x1UL << FLASH_SR_SIZERR_Pos)          /*!< 0x00000040 */
#define FLASH_SR_SIZERR                     FLASH_SR_SIZERR_Msk
#define FLASH_SR_PGSERR_Pos                 (7UL)
#define FLASH_SR_PGSERR_Msk                 (0x1UL << FLASH_SR_PGSERR_Pos)          /*!< 0x00000080 */
#define FLASH_SR_PGSERR                     FLASH_SR_PGSERR_Msk
#define FLASH_SR_MISERR_Pos                 (8UL)
#define FLASH_SR_MISERR_Msk                 (0x1UL << FLASH_SR_MISERR_Pos)          /*!< 0x00000100 */
#define FLASH_SR_MISERR                     FLASH_SR_MISERR_Msk
#define FLASH_SR_FASTERR_Pos                (9UL)
#define FLASH_SR_FASTERR_Msk                (0x1UL << FLASH_SR_FASTERR_Pos)         /*!< 0x00000200 */
#define FLASH_SR_FASTERR                    FLASH_SR_FASTERR_Msk
#define FLASH_SR_HDPOPTWERR_Pos             (11UL)
#define FLASH_SR_HDPOPTWERR_Msk             (0x1UL << FLASH_SR_HDPOPTWERR_Pos)      /*!< 0x00000800 */
#define FLASH_SR_HDPOPTWERR                 FLASH_SR_HDPOPTWERR_Msk
#define FLASH_SR_OEMOPTWERR_Pos             (12UL)
#define FLASH_SR_OEMOPTWERR_Msk             (0x1UL << FLASH_SR_OEMOPTWERR_Pos)      /*!< 0x00001000 */
#define FLASH_SR_OEMOPTWERR                 FLASH_SR_OEMOPTWERR_Msk
#define FLASH_SR_OPTVERR_Pos                (15UL)
#define FLASH_SR_OPTVERR_Msk                (0x1UL << FLASH_SR_OPTVERR_Pos)         /*!< 0x00008000 */
#define FLASH_SR_OPTVERR                    FLASH_SR_OPTVERR_Msk
#define FLASH_SR_BSY1_Pos                   (16UL)
#define FLASH_SR_BSY1_Msk                   (0x1UL << FLASH_SR_BSY1_Pos)            /*!< 0x00010000 */
#define FLASH_SR_BSY1                       FLASH_SR_BSY1_Msk
#define FLASH_SR_CFGBSY_Pos                 (18UL)
#define FLASH_SR_CFGBSY_Msk                 (0x1UL << FLASH_SR_CFGBSY_Pos)          /*!< 0x00040000 */
#define FLASH_SR_CFGBSY                     FLASH_SR_CFGBSY_Msk
#define FLASH_SR_OEM1LOCK_POS               (20UL)
#define FLASH_SR_OEM1LOCK_Msk               (0x1UL << FLASH_SR_OEM1LOCK_POS)        /*!< 0x00100000 */
#define FLASH_SR_OEM1LOCK                   FLASH_SR_OEM1LOCK_Msk
#define FLASH_SR_OEM2LOCK_POS               (21UL)
#define FLASH_SR_OEM2LOCK_Msk               (0x1UL << FLASH_SR_OEM2LOCK_POS)        /*!< 0x00200000 */
#define FLASH_SR_OEM2LOCK                   FLASH_SR_OEM2LOCK_Msk

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG_Pos                     (0UL)
#define FLASH_CR_PG_Msk                     (0x1UL << FLASH_CR_PG_Pos)              /*!< 0x00000001 */
#define FLASH_CR_PG                         FLASH_CR_PG_Msk
#define FLASH_CR_PER_Pos                    (1UL)
#define FLASH_CR_PER_Msk                    (0x1UL << FLASH_CR_PER_Pos)             /*!< 0x00000002 */
#define FLASH_CR_PER                        FLASH_CR_PER_Msk
#define FLASH_CR_MER1_Pos                   (2UL)
#define FLASH_CR_MER1_Msk                   (0x1UL << FLASH_CR_MER1_Pos)            /*!< 0x00000004 */
#define FLASH_CR_MER1                       FLASH_CR_MER1_Msk
#define FLASH_CR_PNB_Pos                    (3UL)
#define FLASH_CR_PNB_Msk                    (0x7FUL << FLASH_CR_PNB_Pos)            /*!< 0x000003F8 */
#define FLASH_CR_PNB                        FLASH_CR_PNB_Msk
#define FLASH_CR_STRT_Pos                   (16UL)
#define FLASH_CR_STRT_Msk                   (0x1UL << FLASH_CR_STRT_Pos)            /*!< 0x00010000 */
#define FLASH_CR_STRT                       FLASH_CR_STRT_Msk
#define FLASH_CR_OPTSTRT_Pos                (17UL)
#define FLASH_CR_OPTSTRT_Msk                (0x1UL << FLASH_CR_OPTSTRT_Pos)         /*!< 0x00020000 */
#define FLASH_CR_OPTSTRT                    FLASH_CR_OPTSTRT_Msk
#define FLASH_CR_FSTPG_Pos                  (18UL)
#define FLASH_CR_FSTPG_Msk                  (0x1UL << FLASH_CR_FSTPG_Pos)           /*!< 0x00040000 */
#define FLASH_CR_FSTPG                      FLASH_CR_FSTPG_Msk
#define FLASH_CR_EOPIE_Pos                  (24UL)
#define FLASH_CR_EOPIE_Msk                  (0x1UL << FLASH_CR_EOPIE_Pos)           /*!< 0x01000000 */
#define FLASH_CR_EOPIE                      FLASH_CR_EOPIE_Msk
#define FLASH_CR_ERRIE_Pos                  (25UL)
#define FLASH_CR_ERRIE_Msk                  (0x1UL << FLASH_CR_ERRIE_Pos)           /*!< 0x02000000 */
#define FLASH_CR_ERRIE                      FLASH_CR_ERRIE_Msk

#define FLASH_CR_OBL_LAUNCH_Pos             (27UL)
#define FLASH_CR_OBL_LAUNCH_Msk             (0x1UL << FLASH_CR_OBL_LAUNCH_Pos)      /*!< 0x08000000 */
#define FLASH_CR_OBL_LAUNCH                 FLASH_CR_OBL_LAUNCH_Msk
#define FLASH_CR_OPTLOCK_Pos                (30UL)
#define FLASH_CR_OPTLOCK_Msk                (0x1UL << FLASH_CR_OPTLOCK_Pos)         /*!< 0x40000000 */
#define FLASH_CR_OPTLOCK                    FLASH_CR_OPTLOCK_Msk
#define FLASH_CR_LOCK_Pos                   (31UL)
#define FLASH_CR_LOCK_Msk                   (0x1UL << FLASH_CR_LOCK_Pos)            /*!< 0x80000000 */
#define FLASH_CR_LOCK                       FLASH_CR_LOCK_Msk

/*******************  Bits definition for FLASH_ECCR register  ****************/
#define FLASH_ECCR_ADDR_ECC_Pos             (0UL)
#define FLASH_ECCR_ADDR_ECC_Msk             (0x3FFFUL << FLASH_ECCR_ADDR_ECC_Pos)   /*!< 0x00003FFF */
#define FLASH_ECCR_ADDR_ECC                 FLASH_ECCR_ADDR_ECC_Msk
#define FLASH_ECCR_SYSF_ECC_Pos             (20UL)
#define FLASH_ECCR_SYSF_ECC_Msk             (0x1UL << FLASH_ECCR_SYSF_ECC_Pos)      /*!< 0x00100000 */
#define FLASH_ECCR_SYSF_ECC                 FLASH_ECCR_SYSF_ECC_Msk
#define FLASH_ECCR_ECCCIE_Pos               (24UL)
#define FLASH_ECCR_ECCCIE_Msk               (0x1UL << FLASH_ECCR_ECCCIE_Pos)        /*!< 0x01000000 */
#define FLASH_ECCR_ECCCIE                   FLASH_ECCR_ECCCIE_Msk
#define FLASH_ECCR_ECCC_Pos                 (30UL)
#define FLASH_ECCR_ECCC_Msk                 (0x1UL << FLASH_ECCR_ECCC_Pos)          /*!< 0x40000000 */
#define FLASH_ECCR_ECCC                     FLASH_ECCR_ECCC_Msk
#define FLASH_ECCR_ECCD_Pos                 (31UL)
#define FLASH_ECCR_ECCD_Msk                 (0x1UL << FLASH_ECCR_ECCD_Pos)          /*!< 0x80000000 */
#define FLASH_ECCR_ECCD                     FLASH_ECCR_ECCD_Msk

/*******************  Bits definition for FLASH_OPTR register  ****************/
#define FLASH_OPTR_RDP_Pos                  (0UL)
#define FLASH_OPTR_RDP_Msk                  (0xFFUL << FLASH_OPTR_RDP_Pos)            /*!< 0x000000FF */
#define FLASH_OPTR_RDP                      FLASH_OPTR_RDP_Msk
#define FLASH_OPTR_BOR_EN_Pos               (8UL)
#define FLASH_OPTR_BOR_EN_Msk               (0x1UL << FLASH_OPTR_BOR_EN_Pos)          /*!< 0x00000100 */
#define FLASH_OPTR_BOR_EN                   FLASH_OPTR_BOR_EN_Msk
#define FLASH_OPTR_BOR_LEV_Pos              (9UL)
#define FLASH_OPTR_BOR_LEV_Msk              (0x3UL << FLASH_OPTR_BOR_LEV_Pos)         /*!< 0x00000600 */
#define FLASH_OPTR_BOR_LEV                  FLASH_OPTR_BOR_LEV_Msk
#define FLASH_OPTR_BOR_LEV_0                (0x1UL << FLASH_OPTR_BOR_LEV_Pos)         /*!< 0x00000200 */
#define FLASH_OPTR_BOR_LEV_1                (0x2UL << FLASH_OPTR_BOR_LEV_Pos)         /*!< 0x00000400 */
#define FLASH_OPTR_nRST_STOP_Pos            (13UL)
#define FLASH_OPTR_nRST_STOP_Msk            (0x1UL << FLASH_OPTR_nRST_STOP_Pos)       /*!< 0x00002000 */
#define FLASH_OPTR_nRST_STOP                FLASH_OPTR_nRST_STOP_Msk
#define FLASH_OPTR_nRST_STDBY_Pos           (14UL)
#define FLASH_OPTR_nRST_STDBY_Msk           (0x1UL << FLASH_OPTR_nRST_STDBY_Pos)      /*!< 0x00004000 */
#define FLASH_OPTR_nRST_STDBY               FLASH_OPTR_nRST_STDBY_Msk
#define FLASH_OPTR_nRST_SHDW_Pos            (15UL)
#define FLASH_OPTR_nRST_SHDW_Msk            (0x1UL << FLASH_OPTR_nRST_SHDW_Pos)       /*!< 0x00008000 */
#define FLASH_OPTR_nRST_SHDW                FLASH_OPTR_nRST_SHDW_Msk
#define FLASH_OPTR_IWDG_SW_Pos              (16UL)
#define FLASH_OPTR_IWDG_SW_Msk              (0x1UL << FLASH_OPTR_IWDG_SW_Pos)         /*!< 0x00010000 */
#define FLASH_OPTR_IWDG_SW                  FLASH_OPTR_IWDG_SW_Msk
#define FLASH_OPTR_IWDG_STOP_Pos            (17UL)
#define FLASH_OPTR_IWDG_STOP_Msk            (0x1UL << FLASH_OPTR_IWDG_STOP_Pos)       /*!< 0x00020000 */
#define FLASH_OPTR_IWDG_STOP                FLASH_OPTR_IWDG_STOP_Msk
#define FLASH_OPTR_IWDG_STDBY_Pos           (18UL)
#define FLASH_OPTR_IWDG_STDBY_Msk           (0x1UL << FLASH_OPTR_IWDG_STDBY_Pos)      /*!< 0x00040000 */
#define FLASH_OPTR_IWDG_STDBY               FLASH_OPTR_IWDG_STDBY_Msk
#define FLASH_OPTR_WWDG_SW_Pos              (19UL)
#define FLASH_OPTR_WWDG_SW_Msk              (0x1UL << FLASH_OPTR_WWDG_SW_Pos)         /*!< 0x00080000 */
#define FLASH_OPTR_WWDG_SW                  FLASH_OPTR_WWDG_SW_Msk
#define FLASH_OPTR_BDRST_Pos                (21UL)
#define FLASH_OPTR_BDRST_Msk                (0x1UL << FLASH_OPTR_BDRST_Pos)            /*!< 0x00200000 */
#define FLASH_OPTR_BDRST                    FLASH_OPTR_BDRST_Msk
#define FLASH_OPTR_RAM_PARITY_CHECK_Pos     (22UL)
#define FLASH_OPTR_RAM_PARITY_CHECK_Msk     (0x1UL << FLASH_OPTR_RAM_PARITY_CHECK_Pos) /*!< 0x00400000 */
#define FLASH_OPTR_RAM_PARITY_CHECK         FLASH_OPTR_RAM_PARITY_CHECK_Msk
#define FLASH_OPTR_BKPSRAM_HW_ERASE_Pos     (23UL)
#define FLASH_OPTR_BKPSRAM_HW_ERASE_Msk     (0x1UL << FLASH_OPTR_BKPSRAM_HW_ERASE_Pos) /*!< 0x00800000 */
#define FLASH_OPTR_BKPSRAM_HW_ERASE         FLASH_OPTR_BKPSRAM_HW_ERASE_Msk
#define FLASH_OPTR_nBOOT_SEL_Pos            (24UL)
#define FLASH_OPTR_nBOOT_SEL_Msk            (0x1UL << FLASH_OPTR_nBOOT_SEL_Pos)        /*!< 0x01000000 */
#define FLASH_OPTR_nBOOT_SEL                FLASH_OPTR_nBOOT_SEL_Msk
#define FLASH_OPTR_nBOOT1_Pos               (25UL)
#define FLASH_OPTR_nBOOT1_Msk               (0x1UL << FLASH_OPTR_nBOOT1_Pos)           /*!< 0x02000000 */
#define FLASH_OPTR_nBOOT1                   FLASH_OPTR_nBOOT1_Msk
#define FLASH_OPTR_nBOOT0_Pos               (26UL)
#define FLASH_OPTR_nBOOT0_Msk               (0x1UL << FLASH_OPTR_nBOOT0_Pos)           /*!< 0x04000000 */
#define FLASH_OPTR_nBOOT0                   FLASH_OPTR_nBOOT0_Msk
#define FLASH_OPTR_NRST_MODE_Pos            (27UL)
#define FLASH_OPTR_NRST_MODE_Msk            (0x3UL << FLASH_OPTR_NRST_MODE_Pos)        /*!< 0x18000000 */
#define FLASH_OPTR_NRST_MODE                FLASH_OPTR_NRST_MODE_Msk
#define FLASH_OPTR_NRST_MODE_0              (0x1UL << FLASH_OPTR_NRST_MODE_Pos)        /*!< 0x08000000 */
#define FLASH_OPTR_NRST_MODE_1              (0x2UL << FLASH_OPTR_NRST_MODE_Pos)        /*!< 0x10000000 */
#define FLASH_OPTR_IRHEN_Pos                (29UL)
#define FLASH_OPTR_IRHEN_Msk                (0x1UL << FLASH_OPTR_IRHEN_Pos)            /*!< 0x20000000 */
#define FLASH_OPTR_IRHEN                    FLASH_OPTR_IRHEN_Msk


/******************  Bits definition for FLASH_WRP1AR register  ***************/
#define FLASH_WRP1AR_WRP1A_STRT_Pos         (0UL)
#define FLASH_WRP1AR_WRP1A_STRT_Msk         (0xFFFFUL << FLASH_WRP1AR_WRP1A_STRT_Pos) /*!< 0x0000FFFF */
#define FLASH_WRP1AR_WRP1A_STRT             FLASH_WRP1AR_WRP1A_STRT_Msk
#define FLASH_WRP1AR_WRP1A_END_Pos          (16UL)
#define FLASH_WRP1AR_WRP1A_END_Msk          (0xFFFFUL << FLASH_WRP1AR_WRP1A_END_Pos) /*!< 0xFFFF0000 */
#define FLASH_WRP1AR_WRP1A_END              FLASH_WRP1AR_WRP1A_END_Msk

/******************  Bits definition for FLASH_WRP1BR register  ***************/
#define FLASH_WRP1BR_WRP1B_STRT_Pos         (0UL)
#define FLASH_WRP1BR_WRP1B_STRT_Msk         (0xFFFFUL << FLASH_WRP1BR_WRP1B_STRT_Pos) /*!< 0x0000FFFF */
#define FLASH_WRP1BR_WRP1B_STRT             FLASH_WRP1BR_WRP1B_STRT_Msk
#define FLASH_WRP1BR_WRP1B_END_Pos          (16UL)
#define FLASH_WRP1BR_WRP1B_END_Msk          (0xFFFFUL << FLASH_WRP1BR_WRP1B_END_Pos) /*!< 0xFFFF0000 */
#define FLASH_WRP1BR_WRP1B_END              FLASH_WRP1BR_WRP1B_END_Msk


/******************  Bits definition for FLASH_SECR register  *****************/
#define FLASH_SECR_HDP1_PEND_Pos            (0UL)
#define FLASH_SECR_HDP1_PEND_Msk            (0x3FUL << FLASH_SECR_HDP1_PEND_Pos)    /*!< 0x0000003F */
#define FLASH_SECR_HDP1_PEND                FLASH_SECR_HDP1_PEND_Msk
#define FLASH_SECR_BOOT_LOCK_Pos            (16UL)
#define FLASH_SECR_BOOT_LOCK_Msk            (0x1UL << FLASH_SECR_BOOT_LOCK_Pos)       /*!< 0x00010000 */
#define FLASH_SECR_BOOT_LOCK                FLASH_SECR_BOOT_LOCK_Msk
#define FLASH_SECR_HDP1EN_Pos               (24UL)
#define FLASH_SECR_HDP1EN_Msk               (0xFFUL << FLASH_SECR_HDP1EN_Pos)       /*!< 0xFF000000 */
#define FLASH_SECR_HDP1EN                   FLASH_SECR_HDP1EN_Msk

/******************  Bits definition for FLASH_OEMKEYSR register  *****************/
#define FLASH_OEMKEYSR_OEM1KEYCRC_Pos       (0UL)
#define FLASH_OEMKEYSR_OEM1KEYCRC_Msk       (0xFFUL << FLASH_OEMKEYSR_OEM1KEYCRC_Pos)     /*!< 0x000000FF */
#define FLASH_OEMKEYSR_OEM1KEYCRC           FLASH_OEMKEYSR_OEM1KEYCRC_Msk
#define FLASH_OEMKEYSR_OEM2KEYCRC_Pos       (16UL)
#define FLASH_OEMKEYSR_OEM2KEYCRC_Msk       (0xFFUL << FLASH_OEMKEYSR_OEM2KEYCRC_Pos)     /*!< 0x00FF0000 */
#define FLASH_OEMKEYSR_OEM2KEYCRC           FLASH_OEMKEYSR_OEM2KEYCRC_Msk

/******************  Bits definition for FLASH_HDPCR register  *****************/
#define FLASH_HDPCR_HDP1_ACCDIS_Pos         (0UL)
#define FLASH_HDPCR_HDP1_ACCDIS_Msk         (0xFFUL << FLASH_HDPCR_HDP1_ACCDIS_Pos)     /*!< 0x000000FF */
#define FLASH_HDPCR_HDP1_ACCDIS             FLASH_HDPCR_HDP1_ACCDIS_Msk
#define FLASH_HDPCR_HDP1EXT_ACCDIS_Pos      (16UL)
#define FLASH_HDPCR_HDP1EXT_ACCDIS_Msk      (0xFFUL << FLASH_HDPCR_HDP1EXT_ACCDIS_Pos)     /*!< 0x000000FF */
#define FLASH_HDPCR_HDP1EXT_ACCDIS          FLASH_HDPCR_HDP1EXT_ACCDIS_Msk

/******************  Bits definition for FLASH_HDPEXTR register  *****************/
#define FLASH_HDPEXTR_HDP1_EXT_Pos          (0UL)
#define FLASH_HDPEXTR_HDP1_EXT_Msk          (0x3FUL << FLASH_HDPEXTR_HDP1_EXT_Pos)     /*!< 0x0000003F */
#define FLASH_HDPEXTR_HDP1_EXT              FLASH_HDPEXTR_HDP1_EXT_Msk

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODE0_Pos           (0UL)
#define GPIO_MODER_MODE0_Msk           (0x3UL << GPIO_MODER_MODE0_Pos)         /*!< 0x00000003 */
#define GPIO_MODER_MODE0               GPIO_MODER_MODE0_Msk
#define GPIO_MODER_MODE0_0             (0x1UL << GPIO_MODER_MODE0_Pos)          /*!< 0x00000001 */
#define GPIO_MODER_MODE0_1             (0x2UL << GPIO_MODER_MODE0_Pos)          /*!< 0x00000002 */

#define GPIO_MODER_MODE1_Pos           (2UL)
#define GPIO_MODER_MODE1_Msk           (0x3UL << GPIO_MODER_MODE1_Pos)         /*!< 0x0000000C */
#define GPIO_MODER_MODE1               GPIO_MODER_MODE1_Msk
#define GPIO_MODER_MODE1_0             (0x1UL << GPIO_MODER_MODE1_Pos)          /*!< 0x00000004 */
#define GPIO_MODER_MODE1_1             (0x2UL << GPIO_MODER_MODE1_Pos)          /*!< 0x00000008 */

#define GPIO_MODER_MODE2_Pos           (4UL)
#define GPIO_MODER_MODE2_Msk           (0x3UL << GPIO_MODER_MODE2_Pos)         /*!< 0x00000030 */
#define GPIO_MODER_MODE2               GPIO_MODER_MODE2_Msk
#define GPIO_MODER_MODE2_0             (0x1UL << GPIO_MODER_MODE2_Pos)          /*!< 0x00000010 */
#define GPIO_MODER_MODE2_1             (0x2UL << GPIO_MODER_MODE2_Pos)          /*!< 0x00000020 */

#define GPIO_MODER_MODE3_Pos           (6UL)
#define GPIO_MODER_MODE3_Msk           (0x3UL << GPIO_MODER_MODE3_Pos)         /*!< 0x000000C0 */
#define GPIO_MODER_MODE3               GPIO_MODER_MODE3_Msk
#define GPIO_MODER_MODE3_0             (0x1UL << GPIO_MODER_MODE3_Pos)          /*!< 0x00000040 */
#define GPIO_MODER_MODE3_1             (0x2UL << GPIO_MODER_MODE3_Pos)          /*!< 0x00000080 */

#define GPIO_MODER_MODE4_Pos           (8UL)
#define GPIO_MODER_MODE4_Msk           (0x3UL << GPIO_MODER_MODE4_Pos)         /*!< 0x00000300 */
#define GPIO_MODER_MODE4               GPIO_MODER_MODE4_Msk
#define GPIO_MODER_MODE4_0             (0x1UL << GPIO_MODER_MODE4_Pos)          /*!< 0x00000100 */
#define GPIO_MODER_MODE4_1             (0x2UL << GPIO_MODER_MODE4_Pos)          /*!< 0x00000200 */

#define GPIO_MODER_MODE5_Pos           (10UL)
#define GPIO_MODER_MODE5_Msk           (0x3UL << GPIO_MODER_MODE5_Pos)         /*!< 0x00000C00 */
#define GPIO_MODER_MODE5               GPIO_MODER_MODE5_Msk
#define GPIO_MODER_MODE5_0             (0x1UL << GPIO_MODER_MODE5_Pos)          /*!< 0x00000400 */
#define GPIO_MODER_MODE5_1             (0x2UL << GPIO_MODER_MODE5_Pos)          /*!< 0x00000800 */

#define GPIO_MODER_MODE6_Pos           (12UL)
#define GPIO_MODER_MODE6_Msk           (0x3UL << GPIO_MODER_MODE6_Pos)         /*!< 0x00003000 */
#define GPIO_MODER_MODE6               GPIO_MODER_MODE6_Msk
#define GPIO_MODER_MODE6_0             (0x1UL << GPIO_MODER_MODE6_Pos)          /*!< 0x00001000 */
#define GPIO_MODER_MODE6_1             (0x2UL << GPIO_MODER_MODE6_Pos)          /*!< 0x00002000 */

#define GPIO_MODER_MODE7_Pos           (14UL)
#define GPIO_MODER_MODE7_Msk           (0x3UL << GPIO_MODER_MODE7_Pos)         /*!< 0x0000C000 */
#define GPIO_MODER_MODE7               GPIO_MODER_MODE7_Msk
#define GPIO_MODER_MODE7_0             (0x1UL << GPIO_MODER_MODE7_Pos)          /*!< 0x00004000 */
#define GPIO_MODER_MODE7_1             (0x2UL << GPIO_MODER_MODE7_Pos)          /*!< 0x00008000 */

#define GPIO_MODER_MODE8_Pos           (16UL)
#define GPIO_MODER_MODE8_Msk           (0x3UL << GPIO_MODER_MODE8_Pos)         /*!< 0x00030000 */
#define GPIO_MODER_MODE8               GPIO_MODER_MODE8_Msk
#define GPIO_MODER_MODE8_0             (0x1UL << GPIO_MODER_MODE8_Pos)          /*!< 0x00010000 */
#define GPIO_MODER_MODE8_1             (0x2UL << GPIO_MODER_MODE8_Pos)          /*!< 0x00020000 */

#define GPIO_MODER_MODE9_Pos           (18UL)
#define GPIO_MODER_MODE9_Msk           (0x3UL << GPIO_MODER_MODE9_Pos)         /*!< 0x000C0000 */
#define GPIO_MODER_MODE9               GPIO_MODER_MODE9_Msk
#define GPIO_MODER_MODE9_0             (0x1UL << GPIO_MODER_MODE9_Pos)          /*!< 0x00040000 */
#define GPIO_MODER_MODE9_1             (0x2UL << GPIO_MODER_MODE9_Pos)          /*!< 0x00080000 */

#define GPIO_MODER_MODE10_Pos          (20UL)
#define GPIO_MODER_MODE10_Msk          (0x3UL << GPIO_MODER_MODE10_Pos)        /*!< 0x00300000 */
#define GPIO_MODER_MODE10              GPIO_MODER_MODE10_Msk
#define GPIO_MODER_MODE10_0            (0x1UL << GPIO_MODER_MODE10_Pos)         /*!< 0x00100000 */
#define GPIO_MODER_MODE10_1            (0x2UL << GPIO_MODER_MODE10_Pos)         /*!< 0x00200000 */

#define GPIO_MODER_MODE11_Pos          (22UL)
#define GPIO_MODER_MODE11_Msk          (0x3UL << GPIO_MODER_MODE11_Pos)        /*!< 0x00C00000 */
#define GPIO_MODER_MODE11              GPIO_MODER_MODE11_Msk
#define GPIO_MODER_MODE11_0            (0x1UL << GPIO_MODER_MODE11_Pos)         /*!< 0x00400000 */
#define GPIO_MODER_MODE11_1            (0x2UL << GPIO_MODER_MODE11_Pos)         /*!< 0x00800000 */

#define GPIO_MODER_MODE12_Pos          (24UL)
#define GPIO_MODER_MODE12_Msk          (0x3UL << GPIO_MODER_MODE12_Pos)        /*!< 0x03000000 */
#define GPIO_MODER_MODE12              GPIO_MODER_MODE12_Msk
#define GPIO_MODER_MODE12_0            (0x1UL << GPIO_MODER_MODE12_Pos)         /*!< 0x01000000 */
#define GPIO_MODER_MODE12_1            (0x2UL << GPIO_MODER_MODE12_Pos)         /*!< 0x02000000 */

#define GPIO_MODER_MODE13_Pos          (26UL)
#define GPIO_MODER_MODE13_Msk          (0x3UL << GPIO_MODER_MODE13_Pos)        /*!< 0x0C000000 */
#define GPIO_MODER_MODE13              GPIO_MODER_MODE13_Msk
#define GPIO_MODER_MODE13_0            (0x1UL << GPIO_MODER_MODE13_Pos)         /*!< 0x04000000 */
#define GPIO_MODER_MODE13_1            (0x2UL << GPIO_MODER_MODE13_Pos)         /*!< 0x08000000 */

#define GPIO_MODER_MODE14_Pos          (28UL)
#define GPIO_MODER_MODE14_Msk          (0x3UL << GPIO_MODER_MODE14_Pos)        /*!< 0x30000000 */
#define GPIO_MODER_MODE14              GPIO_MODER_MODE14_Msk
#define GPIO_MODER_MODE14_0            (0x1UL << GPIO_MODER_MODE14_Pos)         /*!< 0x10000000 */
#define GPIO_MODER_MODE14_1            (0x2UL << GPIO_MODER_MODE14_Pos)         /*!< 0x20000000 */

#define GPIO_MODER_MODE15_Pos          (30UL)
#define GPIO_MODER_MODE15_Msk          (0x3UL << GPIO_MODER_MODE15_Pos)        /*!< 0xC0000000 */
#define GPIO_MODER_MODE15              GPIO_MODER_MODE15_Msk
#define GPIO_MODER_MODE15_0            (0x1UL << GPIO_MODER_MODE15_Pos)         /*!< 0x40000000 */
#define GPIO_MODER_MODE15_1            (0x2UL << GPIO_MODER_MODE15_Pos)         /*!< 0x80000000 */

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT0_Pos            (0UL)
#define GPIO_OTYPER_OT0_Msk            (0x1UL << GPIO_OTYPER_OT0_Pos)          /*!< 0x00000001 */
#define GPIO_OTYPER_OT0                GPIO_OTYPER_OT0_Msk
#define GPIO_OTYPER_OT1_Pos            (1UL)
#define GPIO_OTYPER_OT1_Msk            (0x1UL << GPIO_OTYPER_OT1_Pos)          /*!< 0x00000002 */
#define GPIO_OTYPER_OT1                GPIO_OTYPER_OT1_Msk
#define GPIO_OTYPER_OT2_Pos            (2UL)
#define GPIO_OTYPER_OT2_Msk            (0x1UL << GPIO_OTYPER_OT2_Pos)          /*!< 0x00000004 */
#define GPIO_OTYPER_OT2                GPIO_OTYPER_OT2_Msk
#define GPIO_OTYPER_OT3_Pos            (3UL)
#define GPIO_OTYPER_OT3_Msk            (0x1UL << GPIO_OTYPER_OT3_Pos)          /*!< 0x00000008 */
#define GPIO_OTYPER_OT3                GPIO_OTYPER_OT3_Msk
#define GPIO_OTYPER_OT4_Pos            (4UL)
#define GPIO_OTYPER_OT4_Msk            (0x1UL << GPIO_OTYPER_OT4_Pos)          /*!< 0x00000010 */
#define GPIO_OTYPER_OT4                GPIO_OTYPER_OT4_Msk
#define GPIO_OTYPER_OT5_Pos            (5UL)
#define GPIO_OTYPER_OT5_Msk            (0x1UL << GPIO_OTYPER_OT5_Pos)          /*!< 0x00000020 */
#define GPIO_OTYPER_OT5                GPIO_OTYPER_OT5_Msk
#define GPIO_OTYPER_OT6_Pos            (6UL)
#define GPIO_OTYPER_OT6_Msk            (0x1UL << GPIO_OTYPER_OT6_Pos)          /*!< 0x00000040 */
#define GPIO_OTYPER_OT6                GPIO_OTYPER_OT6_Msk
#define GPIO_OTYPER_OT7_Pos            (7UL)
#define GPIO_OTYPER_OT7_Msk            (0x1UL << GPIO_OTYPER_OT7_Pos)          /*!< 0x00000080 */
#define GPIO_OTYPER_OT7                GPIO_OTYPER_OT7_Msk
#define GPIO_OTYPER_OT8_Pos            (8UL)
#define GPIO_OTYPER_OT8_Msk            (0x1UL << GPIO_OTYPER_OT8_Pos)          /*!< 0x00000100 */
#define GPIO_OTYPER_OT8                GPIO_OTYPER_OT8_Msk
#define GPIO_OTYPER_OT9_Pos            (9UL)
#define GPIO_OTYPER_OT9_Msk            (0x1UL << GPIO_OTYPER_OT9_Pos)          /*!< 0x00000200 */
#define GPIO_OTYPER_OT9                GPIO_OTYPER_OT9_Msk
#define GPIO_OTYPER_OT10_Pos           (10UL)
#define GPIO_OTYPER_OT10_Msk           (0x1UL << GPIO_OTYPER_OT10_Pos)         /*!< 0x00000400 */
#define GPIO_OTYPER_OT10               GPIO_OTYPER_OT10_Msk
#define GPIO_OTYPER_OT11_Pos           (11UL)
#define GPIO_OTYPER_OT11_Msk           (0x1UL << GPIO_OTYPER_OT11_Pos)         /*!< 0x00000800 */
#define GPIO_OTYPER_OT11               GPIO_OTYPER_OT11_Msk
#define GPIO_OTYPER_OT12_Pos           (12UL)
#define GPIO_OTYPER_OT12_Msk           (0x1UL << GPIO_OTYPER_OT12_Pos)         /*!< 0x00001000 */
#define GPIO_OTYPER_OT12               GPIO_OTYPER_OT12_Msk
#define GPIO_OTYPER_OT13_Pos           (13UL)
#define GPIO_OTYPER_OT13_Msk           (0x1UL << GPIO_OTYPER_OT13_Pos)         /*!< 0x00002000 */
#define GPIO_OTYPER_OT13               GPIO_OTYPER_OT13_Msk
#define GPIO_OTYPER_OT14_Pos           (14UL)
#define GPIO_OTYPER_OT14_Msk           (0x1UL << GPIO_OTYPER_OT14_Pos)         /*!< 0x00004000 */
#define GPIO_OTYPER_OT14               GPIO_OTYPER_OT14_Msk
#define GPIO_OTYPER_OT15_Pos           (15UL)
#define GPIO_OTYPER_OT15_Msk           (0x1UL << GPIO_OTYPER_OT15_Pos)         /*!< 0x00008000 */
#define GPIO_OTYPER_OT15               GPIO_OTYPER_OT15_Msk

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDR_OSPEED0_Pos       (0UL)
#define GPIO_OSPEEDR_OSPEED0_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED0_Pos)     /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEED0           GPIO_OSPEEDR_OSPEED0_Msk
#define GPIO_OSPEEDR_OSPEED0_0         (0x1UL << GPIO_OSPEEDR_OSPEED0_Pos)      /*!< 0x00000001 */
#define GPIO_OSPEEDR_OSPEED0_1         (0x2UL << GPIO_OSPEEDR_OSPEED0_Pos)      /*!< 0x00000002 */

#define GPIO_OSPEEDR_OSPEED1_Pos       (2UL)
#define GPIO_OSPEEDR_OSPEED1_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED1_Pos)     /*!< 0x0000000C */
#define GPIO_OSPEEDR_OSPEED1           GPIO_OSPEEDR_OSPEED1_Msk
#define GPIO_OSPEEDR_OSPEED1_0         (0x1UL << GPIO_OSPEEDR_OSPEED1_Pos)      /*!< 0x00000004 */
#define GPIO_OSPEEDR_OSPEED1_1         (0x2UL << GPIO_OSPEEDR_OSPEED1_Pos)      /*!< 0x00000008 */

#define GPIO_OSPEEDR_OSPEED2_Pos       (4UL)
#define GPIO_OSPEEDR_OSPEED2_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED2_Pos)     /*!< 0x00000030 */
#define GPIO_OSPEEDR_OSPEED2           GPIO_OSPEEDR_OSPEED2_Msk
#define GPIO_OSPEEDR_OSPEED2_0         (0x1UL << GPIO_OSPEEDR_OSPEED2_Pos)      /*!< 0x00000010 */
#define GPIO_OSPEEDR_OSPEED2_1         (0x2UL << GPIO_OSPEEDR_OSPEED2_Pos)      /*!< 0x00000020 */

#define GPIO_OSPEEDR_OSPEED3_Pos       (6UL)
#define GPIO_OSPEEDR_OSPEED3_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED3_Pos)     /*!< 0x000000C0 */
#define GPIO_OSPEEDR_OSPEED3           GPIO_OSPEEDR_OSPEED3_Msk
#define GPIO_OSPEEDR_OSPEED3_0         (0x1UL << GPIO_OSPEEDR_OSPEED3_Pos)      /*!< 0x00000040 */
#define GPIO_OSPEEDR_OSPEED3_1         (0x2UL << GPIO_OSPEEDR_OSPEED3_Pos)      /*!< 0x00000080 */

#define GPIO_OSPEEDR_OSPEED4_Pos       (8UL)
#define GPIO_OSPEEDR_OSPEED4_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED4_Pos)     /*!< 0x00000300 */
#define GPIO_OSPEEDR_OSPEED4           GPIO_OSPEEDR_OSPEED4_Msk
#define GPIO_OSPEEDR_OSPEED4_0         (0x1UL << GPIO_OSPEEDR_OSPEED4_Pos)      /*!< 0x00000100 */
#define GPIO_OSPEEDR_OSPEED4_1         (0x2UL << GPIO_OSPEEDR_OSPEED4_Pos)      /*!< 0x00000200 */

#define GPIO_OSPEEDR_OSPEED5_Pos       (10UL)
#define GPIO_OSPEEDR_OSPEED5_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED5_Pos)     /*!< 0x00000C00 */
#define GPIO_OSPEEDR_OSPEED5           GPIO_OSPEEDR_OSPEED5_Msk
#define GPIO_OSPEEDR_OSPEED5_0         (0x1UL << GPIO_OSPEEDR_OSPEED5_Pos)      /*!< 0x00000400 */
#define GPIO_OSPEEDR_OSPEED5_1         (0x2UL << GPIO_OSPEEDR_OSPEED5_Pos)      /*!< 0x00000800 */

#define GPIO_OSPEEDR_OSPEED6_Pos       (12UL)
#define GPIO_OSPEEDR_OSPEED6_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED6_Pos)     /*!< 0x00003000 */
#define GPIO_OSPEEDR_OSPEED6           GPIO_OSPEEDR_OSPEED6_Msk
#define GPIO_OSPEEDR_OSPEED6_0         (0x1UL << GPIO_OSPEEDR_OSPEED6_Pos)      /*!< 0x00001000 */
#define GPIO_OSPEEDR_OSPEED6_1         (0x2UL << GPIO_OSPEEDR_OSPEED6_Pos)      /*!< 0x00002000 */

#define GPIO_OSPEEDR_OSPEED7_Pos       (14UL)
#define GPIO_OSPEEDR_OSPEED7_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED7_Pos)     /*!< 0x0000C000 */
#define GPIO_OSPEEDR_OSPEED7           GPIO_OSPEEDR_OSPEED7_Msk
#define GPIO_OSPEEDR_OSPEED7_0         (0x1UL << GPIO_OSPEEDR_OSPEED7_Pos)      /*!< 0x00004000 */
#define GPIO_OSPEEDR_OSPEED7_1         (0x2UL << GPIO_OSPEEDR_OSPEED7_Pos)      /*!< 0x00008000 */

#define GPIO_OSPEEDR_OSPEED8_Pos       (16UL)
#define GPIO_OSPEEDR_OSPEED8_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED8_Pos)     /*!< 0x00030000 */
#define GPIO_OSPEEDR_OSPEED8           GPIO_OSPEEDR_OSPEED8_Msk
#define GPIO_OSPEEDR_OSPEED8_0         (0x1UL << GPIO_OSPEEDR_OSPEED8_Pos)      /*!< 0x00010000 */
#define GPIO_OSPEEDR_OSPEED8_1         (0x2UL << GPIO_OSPEEDR_OSPEED8_Pos)      /*!< 0x00020000 */

#define GPIO_OSPEEDR_OSPEED9_Pos       (18UL)
#define GPIO_OSPEEDR_OSPEED9_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED9_Pos)     /*!< 0x000C0000 */
#define GPIO_OSPEEDR_OSPEED9           GPIO_OSPEEDR_OSPEED9_Msk
#define GPIO_OSPEEDR_OSPEED9_0         (0x1UL << GPIO_OSPEEDR_OSPEED9_Pos)      /*!< 0x00040000 */
#define GPIO_OSPEEDR_OSPEED9_1         (0x2UL << GPIO_OSPEEDR_OSPEED9_Pos)      /*!< 0x00080000 */

#define GPIO_OSPEEDR_OSPEED10_Pos      (20UL)
#define GPIO_OSPEEDR_OSPEED10_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED10_Pos)    /*!< 0x00300000 */
#define GPIO_OSPEEDR_OSPEED10          GPIO_OSPEEDR_OSPEED10_Msk
#define GPIO_OSPEEDR_OSPEED10_0        (0x1UL << GPIO_OSPEEDR_OSPEED10_Pos)     /*!< 0x00100000 */
#define GPIO_OSPEEDR_OSPEED10_1        (0x2UL << GPIO_OSPEEDR_OSPEED10_Pos)     /*!< 0x00200000 */

#define GPIO_OSPEEDR_OSPEED11_Pos      (22UL)
#define GPIO_OSPEEDR_OSPEED11_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED11_Pos)    /*!< 0x00C00000 */
#define GPIO_OSPEEDR_OSPEED11          GPIO_OSPEEDR_OSPEED11_Msk
#define GPIO_OSPEEDR_OSPEED11_0        (0x1UL << GPIO_OSPEEDR_OSPEED11_Pos)     /*!< 0x00400000 */
#define GPIO_OSPEEDR_OSPEED11_1        (0x2UL << GPIO_OSPEEDR_OSPEED11_Pos)     /*!< 0x00800000 */

#define GPIO_OSPEEDR_OSPEED12_Pos      (24UL)
#define GPIO_OSPEEDR_OSPEED12_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED12_Pos)    /*!< 0x03000000 */
#define GPIO_OSPEEDR_OSPEED12          GPIO_OSPEEDR_OSPEED12_Msk
#define GPIO_OSPEEDR_OSPEED12_0        (0x1UL << GPIO_OSPEEDR_OSPEED12_Pos)     /*!< 0x01000000 */
#define GPIO_OSPEEDR_OSPEED12_1        (0x2UL << GPIO_OSPEEDR_OSPEED12_Pos)     /*!< 0x02000000 */

#define GPIO_OSPEEDR_OSPEED13_Pos      (26UL)
#define GPIO_OSPEEDR_OSPEED13_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED13_Pos)    /*!< 0x0C000000 */
#define GPIO_OSPEEDR_OSPEED13          GPIO_OSPEEDR_OSPEED13_Msk
#define GPIO_OSPEEDR_OSPEED13_0        (0x1UL << GPIO_OSPEEDR_OSPEED13_Pos)     /*!< 0x04000000 */
#define GPIO_OSPEEDR_OSPEED13_1        (0x2UL << GPIO_OSPEEDR_OSPEED13_Pos)     /*!< 0x08000000 */

#define GPIO_OSPEEDR_OSPEED14_Pos      (28UL)
#define GPIO_OSPEEDR_OSPEED14_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED14_Pos)    /*!< 0x30000000 */
#define GPIO_OSPEEDR_OSPEED14          GPIO_OSPEEDR_OSPEED14_Msk
#define GPIO_OSPEEDR_OSPEED14_0        (0x1UL << GPIO_OSPEEDR_OSPEED14_Pos)     /*!< 0x10000000 */
#define GPIO_OSPEEDR_OSPEED14_1        (0x2UL << GPIO_OSPEEDR_OSPEED14_Pos)     /*!< 0x20000000 */

#define GPIO_OSPEEDR_OSPEED15_Pos      (30UL)
#define GPIO_OSPEEDR_OSPEED15_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED15_Pos)    /*!< 0xC0000000 */
#define GPIO_OSPEEDR_OSPEED15          GPIO_OSPEEDR_OSPEED15_Msk
#define GPIO_OSPEEDR_OSPEED15_0        (0x1UL << GPIO_OSPEEDR_OSPEED15_Pos)     /*!< 0x40000000 */
#define GPIO_OSPEEDR_OSPEED15_1        (0x2UL << GPIO_OSPEEDR_OSPEED15_Pos)     /*!< 0x80000000 */

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPD0_Pos           (0UL)
#define GPIO_PUPDR_PUPD0_Msk           (0x3UL << GPIO_PUPDR_PUPD0_Pos)         /*!< 0x00000003 */
#define GPIO_PUPDR_PUPD0               GPIO_PUPDR_PUPD0_Msk
#define GPIO_PUPDR_PUPD0_0             (0x1UL << GPIO_PUPDR_PUPD0_Pos)          /*!< 0x00000001 */
#define GPIO_PUPDR_PUPD0_1             (0x2UL << GPIO_PUPDR_PUPD0_Pos)          /*!< 0x00000002 */

#define GPIO_PUPDR_PUPD1_Pos           (2UL)
#define GPIO_PUPDR_PUPD1_Msk           (0x3UL << GPIO_PUPDR_PUPD1_Pos)         /*!< 0x0000000C */
#define GPIO_PUPDR_PUPD1               GPIO_PUPDR_PUPD1_Msk
#define GPIO_PUPDR_PUPD1_0             (0x1UL << GPIO_PUPDR_PUPD1_Pos)          /*!< 0x00000004 */
#define GPIO_PUPDR_PUPD1_1             (0x2UL << GPIO_PUPDR_PUPD1_Pos)          /*!< 0x00000008 */

#define GPIO_PUPDR_PUPD2_Pos           (4UL)
#define GPIO_PUPDR_PUPD2_Msk           (0x3UL << GPIO_PUPDR_PUPD2_Pos)         /*!< 0x00000030 */
#define GPIO_PUPDR_PUPD2               GPIO_PUPDR_PUPD2_Msk
#define GPIO_PUPDR_PUPD2_0             (0x1UL << GPIO_PUPDR_PUPD2_Pos)          /*!< 0x00000010 */
#define GPIO_PUPDR_PUPD2_1             (0x2UL << GPIO_PUPDR_PUPD2_Pos)          /*!< 0x00000020 */

#define GPIO_PUPDR_PUPD3_Pos           (6UL)
#define GPIO_PUPDR_PUPD3_Msk           (0x3UL << GPIO_PUPDR_PUPD3_Pos)         /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPD3               GPIO_PUPDR_PUPD3_Msk
#define GPIO_PUPDR_PUPD3_0             (0x1UL << GPIO_PUPDR_PUPD3_Pos)          /*!< 0x00000040 */
#define GPIO_PUPDR_PUPD3_1             (0x2UL << GPIO_PUPDR_PUPD3_Pos)          /*!< 0x00000080 */

#define GPIO_PUPDR_PUPD4_Pos           (8UL)
#define GPIO_PUPDR_PUPD4_Msk           (0x3UL << GPIO_PUPDR_PUPD4_Pos)         /*!< 0x00000300 */
#define GPIO_PUPDR_PUPD4               GPIO_PUPDR_PUPD4_Msk
#define GPIO_PUPDR_PUPD4_0             (0x1UL << GPIO_PUPDR_PUPD4_Pos)          /*!< 0x00000100 */
#define GPIO_PUPDR_PUPD4_1             (0x2UL << GPIO_PUPDR_PUPD4_Pos)          /*!< 0x00000200 */

#define GPIO_PUPDR_PUPD5_Pos           (10UL)
#define GPIO_PUPDR_PUPD5_Msk           (0x3UL << GPIO_PUPDR_PUPD5_Pos)         /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPD5               GPIO_PUPDR_PUPD5_Msk
#define GPIO_PUPDR_PUPD5_0             (0x1UL << GPIO_PUPDR_PUPD5_Pos)          /*!< 0x00000400 */
#define GPIO_PUPDR_PUPD5_1             (0x2UL << GPIO_PUPDR_PUPD5_Pos)          /*!< 0x00000800 */

#define GPIO_PUPDR_PUPD6_Pos           (12UL)
#define GPIO_PUPDR_PUPD6_Msk           (0x3UL << GPIO_PUPDR_PUPD6_Pos)         /*!< 0x00003000 */
#define GPIO_PUPDR_PUPD6               GPIO_PUPDR_PUPD6_Msk
#define GPIO_PUPDR_PUPD6_0             (0x1UL << GPIO_PUPDR_PUPD6_Pos)          /*!< 0x00001000 */
#define GPIO_PUPDR_PUPD6_1             (0x2UL << GPIO_PUPDR_PUPD6_Pos)          /*!< 0x00002000 */

#define GPIO_PUPDR_PUPD7_Pos           (14UL)
#define GPIO_PUPDR_PUPD7_Msk           (0x3UL << GPIO_PUPDR_PUPD7_Pos)         /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPD7               GPIO_PUPDR_PUPD7_Msk
#define GPIO_PUPDR_PUPD7_0             (0x1UL << GPIO_PUPDR_PUPD7_Pos)          /*!< 0x00004000 */
#define GPIO_PUPDR_PUPD7_1             (0x2UL << GPIO_PUPDR_PUPD7_Pos)          /*!< 0x00008000 */

#define GPIO_PUPDR_PUPD8_Pos           (16UL)
#define GPIO_PUPDR_PUPD8_Msk           (0x3UL << GPIO_PUPDR_PUPD8_Pos)         /*!< 0x00030000 */
#define GPIO_PUPDR_PUPD8               GPIO_PUPDR_PUPD8_Msk
#define GPIO_PUPDR_PUPD8_0             (0x1UL << GPIO_PUPDR_PUPD8_Pos)          /*!< 0x00010000 */
#define GPIO_PUPDR_PUPD8_1             (0x2UL << GPIO_PUPDR_PUPD8_Pos)          /*!< 0x00020000 */

#define GPIO_PUPDR_PUPD9_Pos           (18UL)
#define GPIO_PUPDR_PUPD9_Msk           (0x3UL << GPIO_PUPDR_PUPD9_Pos)         /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPD9               GPIO_PUPDR_PUPD9_Msk
#define GPIO_PUPDR_PUPD9_0             (0x1UL << GPIO_PUPDR_PUPD9_Pos)          /*!< 0x00040000 */
#define GPIO_PUPDR_PUPD9_1             (0x2UL << GPIO_PUPDR_PUPD9_Pos)          /*!< 0x00080000 */

#define GPIO_PUPDR_PUPD10_Pos          (20UL)
#define GPIO_PUPDR_PUPD10_Msk          (0x3UL << GPIO_PUPDR_PUPD10_Pos)        /*!< 0x00300000 */
#define GPIO_PUPDR_PUPD10              GPIO_PUPDR_PUPD10_Msk
#define GPIO_PUPDR_PUPD10_0            (0x1UL << GPIO_PUPDR_PUPD10_Pos)         /*!< 0x00100000 */
#define GPIO_PUPDR_PUPD10_1            (0x2UL << GPIO_PUPDR_PUPD10_Pos)         /*!< 0x00200000 */

#define GPIO_PUPDR_PUPD11_Pos          (22UL)
#define GPIO_PUPDR_PUPD11_Msk          (0x3UL << GPIO_PUPDR_PUPD11_Pos)        /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPD11              GPIO_PUPDR_PUPD11_Msk
#define GPIO_PUPDR_PUPD11_0            (0x1UL << GPIO_PUPDR_PUPD11_Pos)         /*!< 0x00400000 */
#define GPIO_PUPDR_PUPD11_1            (0x2UL << GPIO_PUPDR_PUPD11_Pos)         /*!< 0x00800000 */

#define GPIO_PUPDR_PUPD12_Pos          (24UL)
#define GPIO_PUPDR_PUPD12_Msk          (0x3UL << GPIO_PUPDR_PUPD12_Pos)        /*!< 0x03000000 */
#define GPIO_PUPDR_PUPD12              GPIO_PUPDR_PUPD12_Msk
#define GPIO_PUPDR_PUPD12_0            (0x1UL << GPIO_PUPDR_PUPD12_Pos)         /*!< 0x01000000 */
#define GPIO_PUPDR_PUPD12_1            (0x2UL << GPIO_PUPDR_PUPD12_Pos)         /*!< 0x02000000 */

#define GPIO_PUPDR_PUPD13_Pos          (26UL)
#define GPIO_PUPDR_PUPD13_Msk          (0x3UL << GPIO_PUPDR_PUPD13_Pos)        /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPD13              GPIO_PUPDR_PUPD13_Msk
#define GPIO_PUPDR_PUPD13_0            (0x1UL << GPIO_PUPDR_PUPD13_Pos)         /*!< 0x04000000 */
#define GPIO_PUPDR_PUPD13_1            (0x2UL << GPIO_PUPDR_PUPD13_Pos)         /*!< 0x08000000 */

#define GPIO_PUPDR_PUPD14_Pos          (28UL)
#define GPIO_PUPDR_PUPD14_Msk          (0x3UL << GPIO_PUPDR_PUPD14_Pos)        /*!< 0x30000000 */
#define GPIO_PUPDR_PUPD14              GPIO_PUPDR_PUPD14_Msk
#define GPIO_PUPDR_PUPD14_0            (0x1UL << GPIO_PUPDR_PUPD14_Pos)         /*!< 0x10000000 */
#define GPIO_PUPDR_PUPD14_1            (0x2UL << GPIO_PUPDR_PUPD14_Pos)         /*!< 0x20000000 */

#define GPIO_PUPDR_PUPD15_Pos          (30UL)
#define GPIO_PUPDR_PUPD15_Msk          (0x3UL << GPIO_PUPDR_PUPD15_Pos)        /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPD15              GPIO_PUPDR_PUPD15_Msk
#define GPIO_PUPDR_PUPD15_0            (0x1UL << GPIO_PUPDR_PUPD15_Pos)         /*!< 0x40000000 */
#define GPIO_PUPDR_PUPD15_1            (0x2UL << GPIO_PUPDR_PUPD15_Pos)         /*!< 0x80000000 */

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_ID0_Pos               (0UL)
#define GPIO_IDR_ID0_Msk               (0x1UL << GPIO_IDR_ID0_Pos)             /*!< 0x00000001 */
#define GPIO_IDR_ID0                   GPIO_IDR_ID0_Msk
#define GPIO_IDR_ID1_Pos               (1UL)
#define GPIO_IDR_ID1_Msk               (0x1UL << GPIO_IDR_ID1_Pos)             /*!< 0x00000002 */
#define GPIO_IDR_ID1                   GPIO_IDR_ID1_Msk
#define GPIO_IDR_ID2_Pos               (2UL)
#define GPIO_IDR_ID2_Msk               (0x1UL << GPIO_IDR_ID2_Pos)             /*!< 0x00000004 */
#define GPIO_IDR_ID2                   GPIO_IDR_ID2_Msk
#define GPIO_IDR_ID3_Pos               (3UL)
#define GPIO_IDR_ID3_Msk               (0x1UL << GPIO_IDR_ID3_Pos)             /*!< 0x00000008 */
#define GPIO_IDR_ID3                   GPIO_IDR_ID3_Msk
#define GPIO_IDR_ID4_Pos               (4UL)
#define GPIO_IDR_ID4_Msk               (0x1UL << GPIO_IDR_ID4_Pos)             /*!< 0x00000010 */
#define GPIO_IDR_ID4                   GPIO_IDR_ID4_Msk
#define GPIO_IDR_ID5_Pos               (5UL)
#define GPIO_IDR_ID5_Msk               (0x1UL << GPIO_IDR_ID5_Pos)             /*!< 0x00000020 */
#define GPIO_IDR_ID5                   GPIO_IDR_ID5_Msk
#define GPIO_IDR_ID6_Pos               (6UL)
#define GPIO_IDR_ID6_Msk               (0x1UL << GPIO_IDR_ID6_Pos)             /*!< 0x00000040 */
#define GPIO_IDR_ID6                   GPIO_IDR_ID6_Msk
#define GPIO_IDR_ID7_Pos               (7UL)
#define GPIO_IDR_ID7_Msk               (0x1UL << GPIO_IDR_ID7_Pos)             /*!< 0x00000080 */
#define GPIO_IDR_ID7                   GPIO_IDR_ID7_Msk
#define GPIO_IDR_ID8_Pos               (8UL)
#define GPIO_IDR_ID8_Msk               (0x1UL << GPIO_IDR_ID8_Pos)             /*!< 0x00000100 */
#define GPIO_IDR_ID8                   GPIO_IDR_ID8_Msk
#define GPIO_IDR_ID9_Pos               (9UL)
#define GPIO_IDR_ID9_Msk               (0x1UL << GPIO_IDR_ID9_Pos)             /*!< 0x00000200 */
#define GPIO_IDR_ID9                   GPIO_IDR_ID9_Msk
#define GPIO_IDR_ID10_Pos              (10UL)
#define GPIO_IDR_ID10_Msk              (0x1UL << GPIO_IDR_ID10_Pos)            /*!< 0x00000400 */
#define GPIO_IDR_ID10                  GPIO_IDR_ID10_Msk
#define GPIO_IDR_ID11_Pos              (11UL)
#define GPIO_IDR_ID11_Msk              (0x1UL << GPIO_IDR_ID11_Pos)            /*!< 0x00000800 */
#define GPIO_IDR_ID11                  GPIO_IDR_ID11_Msk
#define GPIO_IDR_ID12_Pos              (12UL)
#define GPIO_IDR_ID12_Msk              (0x1UL << GPIO_IDR_ID12_Pos)            /*!< 0x00001000 */
#define GPIO_IDR_ID12                  GPIO_IDR_ID12_Msk
#define GPIO_IDR_ID13_Pos              (13UL)
#define GPIO_IDR_ID13_Msk              (0x1UL << GPIO_IDR_ID13_Pos)            /*!< 0x00002000 */
#define GPIO_IDR_ID13                  GPIO_IDR_ID13_Msk
#define GPIO_IDR_ID14_Pos              (14UL)
#define GPIO_IDR_ID14_Msk              (0x1UL << GPIO_IDR_ID14_Pos)            /*!< 0x00004000 */
#define GPIO_IDR_ID14                  GPIO_IDR_ID14_Msk
#define GPIO_IDR_ID15_Pos              (15UL)
#define GPIO_IDR_ID15_Msk              (0x1UL << GPIO_IDR_ID15_Pos)            /*!< 0x00008000 */
#define GPIO_IDR_ID15                  GPIO_IDR_ID15_Msk

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_OD0_Pos               (0UL)
#define GPIO_ODR_OD0_Msk               (0x1UL << GPIO_ODR_OD0_Pos)             /*!< 0x00000001 */
#define GPIO_ODR_OD0                   GPIO_ODR_OD0_Msk
#define GPIO_ODR_OD1_Pos               (1UL)
#define GPIO_ODR_OD1_Msk               (0x1UL << GPIO_ODR_OD1_Pos)             /*!< 0x00000002 */
#define GPIO_ODR_OD1                   GPIO_ODR_OD1_Msk
#define GPIO_ODR_OD2_Pos               (2UL)
#define GPIO_ODR_OD2_Msk               (0x1UL << GPIO_ODR_OD2_Pos)             /*!< 0x00000004 */
#define GPIO_ODR_OD2                   GPIO_ODR_OD2_Msk
#define GPIO_ODR_OD3_Pos               (3UL)
#define GPIO_ODR_OD3_Msk               (0x1UL << GPIO_ODR_OD3_Pos)             /*!< 0x00000008 */
#define GPIO_ODR_OD3                   GPIO_ODR_OD3_Msk
#define GPIO_ODR_OD4_Pos               (4UL)
#define GPIO_ODR_OD4_Msk               (0x1UL << GPIO_ODR_OD4_Pos)             /*!< 0x00000010 */
#define GPIO_ODR_OD4                   GPIO_ODR_OD4_Msk
#define GPIO_ODR_OD5_Pos               (5UL)
#define GPIO_ODR_OD5_Msk               (0x1UL << GPIO_ODR_OD5_Pos)             /*!< 0x00000020 */
#define GPIO_ODR_OD5                   GPIO_ODR_OD5_Msk
#define GPIO_ODR_OD6_Pos               (6UL)
#define GPIO_ODR_OD6_Msk               (0x1UL << GPIO_ODR_OD6_Pos)             /*!< 0x00000040 */
#define GPIO_ODR_OD6                   GPIO_ODR_OD6_Msk
#define GPIO_ODR_OD7_Pos               (7UL)
#define GPIO_ODR_OD7_Msk               (0x1UL << GPIO_ODR_OD7_Pos)             /*!< 0x00000080 */
#define GPIO_ODR_OD7                   GPIO_ODR_OD7_Msk
#define GPIO_ODR_OD8_Pos               (8UL)
#define GPIO_ODR_OD8_Msk               (0x1UL << GPIO_ODR_OD8_Pos)             /*!< 0x00000100 */
#define GPIO_ODR_OD8                   GPIO_ODR_OD8_Msk
#define GPIO_ODR_OD9_Pos               (9UL)
#define GPIO_ODR_OD9_Msk               (0x1UL << GPIO_ODR_OD9_Pos)             /*!< 0x00000200 */
#define GPIO_ODR_OD9                   GPIO_ODR_OD9_Msk
#define GPIO_ODR_OD10_Pos              (10UL)
#define GPIO_ODR_OD10_Msk              (0x1UL << GPIO_ODR_OD10_Pos)            /*!< 0x00000400 */
#define GPIO_ODR_OD10                  GPIO_ODR_OD10_Msk
#define GPIO_ODR_OD11_Pos              (11UL)
#define GPIO_ODR_OD11_Msk              (0x1UL << GPIO_ODR_OD11_Pos)            /*!< 0x00000800 */
#define GPIO_ODR_OD11                  GPIO_ODR_OD11_Msk
#define GPIO_ODR_OD12_Pos              (12UL)
#define GPIO_ODR_OD12_Msk              (0x1UL << GPIO_ODR_OD12_Pos)            /*!< 0x00001000 */
#define GPIO_ODR_OD12                  GPIO_ODR_OD12_Msk
#define GPIO_ODR_OD13_Pos              (13UL)
#define GPIO_ODR_OD13_Msk              (0x1UL << GPIO_ODR_OD13_Pos)            /*!< 0x00002000 */
#define GPIO_ODR_OD13                  GPIO_ODR_OD13_Msk
#define GPIO_ODR_OD14_Pos              (14UL)
#define GPIO_ODR_OD14_Msk              (0x1UL << GPIO_ODR_OD14_Pos)            /*!< 0x00004000 */
#define GPIO_ODR_OD14                  GPIO_ODR_OD14_Msk
#define GPIO_ODR_OD15_Pos              (15UL)
#define GPIO_ODR_OD15_Msk              (0x1UL << GPIO_ODR_OD15_Pos)            /*!< 0x00008000 */
#define GPIO_ODR_OD15                  GPIO_ODR_OD15_Msk

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS0_Pos              (0UL)
#define GPIO_BSRR_BS0_Msk              (0x1UL << GPIO_BSRR_BS0_Pos)            /*!< 0x00000001 */
#define GPIO_BSRR_BS0                  GPIO_BSRR_BS0_Msk
#define GPIO_BSRR_BS1_Pos              (1UL)
#define GPIO_BSRR_BS1_Msk              (0x1UL << GPIO_BSRR_BS1_Pos)            /*!< 0x00000002 */
#define GPIO_BSRR_BS1                  GPIO_BSRR_BS1_Msk
#define GPIO_BSRR_BS2_Pos              (2UL)
#define GPIO_BSRR_BS2_Msk              (0x1UL << GPIO_BSRR_BS2_Pos)            /*!< 0x00000004 */
#define GPIO_BSRR_BS2                  GPIO_BSRR_BS2_Msk
#define GPIO_BSRR_BS3_Pos              (3UL)
#define GPIO_BSRR_BS3_Msk              (0x1UL << GPIO_BSRR_BS3_Pos)            /*!< 0x00000008 */
#define GPIO_BSRR_BS3                  GPIO_BSRR_BS3_Msk
#define GPIO_BSRR_BS4_Pos              (4UL)
#define GPIO_BSRR_BS4_Msk              (0x1UL << GPIO_BSRR_BS4_Pos)            /*!< 0x00000010 */
#define GPIO_BSRR_BS4                  GPIO_BSRR_BS4_Msk
#define GPIO_BSRR_BS5_Pos              (5UL)
#define GPIO_BSRR_BS5_Msk              (0x1UL << GPIO_BSRR_BS5_Pos)            /*!< 0x00000020 */
#define GPIO_BSRR_BS5                  GPIO_BSRR_BS5_Msk
#define GPIO_BSRR_BS6_Pos              (6UL)
#define GPIO_BSRR_BS6_Msk              (0x1UL << GPIO_BSRR_BS6_Pos)            /*!< 0x00000040 */
#define GPIO_BSRR_BS6                  GPIO_BSRR_BS6_Msk
#define GPIO_BSRR_BS7_Pos              (7UL)
#define GPIO_BSRR_BS7_Msk              (0x1UL << GPIO_BSRR_BS7_Pos)            /*!< 0x00000080 */
#define GPIO_BSRR_BS7                  GPIO_BSRR_BS7_Msk
#define GPIO_BSRR_BS8_Pos              (8UL)
#define GPIO_BSRR_BS8_Msk              (0x1UL << GPIO_BSRR_BS8_Pos)            /*!< 0x00000100 */
#define GPIO_BSRR_BS8                  GPIO_BSRR_BS8_Msk
#define GPIO_BSRR_BS9_Pos              (9UL)
#define GPIO_BSRR_BS9_Msk              (0x1UL << GPIO_BSRR_BS9_Pos)            /*!< 0x00000200 */
#define GPIO_BSRR_BS9                  GPIO_BSRR_BS9_Msk
#define GPIO_BSRR_BS10_Pos             (10UL)
#define GPIO_BSRR_BS10_Msk             (0x1UL << GPIO_BSRR_BS10_Pos)           /*!< 0x00000400 */
#define GPIO_BSRR_BS10                 GPIO_BSRR_BS10_Msk
#define GPIO_BSRR_BS11_Pos             (11UL)
#define GPIO_BSRR_BS11_Msk             (0x1UL << GPIO_BSRR_BS11_Pos)           /*!< 0x00000800 */
#define GPIO_BSRR_BS11                 GPIO_BSRR_BS11_Msk
#define GPIO_BSRR_BS12_Pos             (12UL)
#define GPIO_BSRR_BS12_Msk             (0x1UL << GPIO_BSRR_BS12_Pos)           /*!< 0x00001000 */
#define GPIO_BSRR_BS12                 GPIO_BSRR_BS12_Msk
#define GPIO_BSRR_BS13_Pos             (13UL)
#define GPIO_BSRR_BS13_Msk             (0x1UL << GPIO_BSRR_BS13_Pos)           /*!< 0x00002000 */
#define GPIO_BSRR_BS13                 GPIO_BSRR_BS13_Msk
#define GPIO_BSRR_BS14_Pos             (14UL)
#define GPIO_BSRR_BS14_Msk             (0x1UL << GPIO_BSRR_BS14_Pos)           /*!< 0x00004000 */
#define GPIO_BSRR_BS14                 GPIO_BSRR_BS14_Msk
#define GPIO_BSRR_BS15_Pos             (15UL)
#define GPIO_BSRR_BS15_Msk             (0x1UL << GPIO_BSRR_BS15_Pos)           /*!< 0x00008000 */
#define GPIO_BSRR_BS15                 GPIO_BSRR_BS15_Msk
#define GPIO_BSRR_BR0_Pos              (16UL)
#define GPIO_BSRR_BR0_Msk              (0x1UL << GPIO_BSRR_BR0_Pos)            /*!< 0x00010000 */
#define GPIO_BSRR_BR0                  GPIO_BSRR_BR0_Msk
#define GPIO_BSRR_BR1_Pos              (17UL)
#define GPIO_BSRR_BR1_Msk              (0x1UL << GPIO_BSRR_BR1_Pos)            /*!< 0x00020000 */
#define GPIO_BSRR_BR1                  GPIO_BSRR_BR1_Msk
#define GPIO_BSRR_BR2_Pos              (18UL)
#define GPIO_BSRR_BR2_Msk              (0x1UL << GPIO_BSRR_BR2_Pos)            /*!< 0x00040000 */
#define GPIO_BSRR_BR2                  GPIO_BSRR_BR2_Msk
#define GPIO_BSRR_BR3_Pos              (19UL)
#define GPIO_BSRR_BR3_Msk              (0x1UL << GPIO_BSRR_BR3_Pos)            /*!< 0x00080000 */
#define GPIO_BSRR_BR3                  GPIO_BSRR_BR3_Msk
#define GPIO_BSRR_BR4_Pos              (20UL)
#define GPIO_BSRR_BR4_Msk              (0x1UL << GPIO_BSRR_BR4_Pos)            /*!< 0x00100000 */
#define GPIO_BSRR_BR4                  GPIO_BSRR_BR4_Msk
#define GPIO_BSRR_BR5_Pos              (21UL)
#define GPIO_BSRR_BR5_Msk              (0x1UL << GPIO_BSRR_BR5_Pos)            /*!< 0x00200000 */
#define GPIO_BSRR_BR5                  GPIO_BSRR_BR5_Msk
#define GPIO_BSRR_BR6_Pos              (22UL)
#define GPIO_BSRR_BR6_Msk              (0x1UL << GPIO_BSRR_BR6_Pos)            /*!< 0x00400000 */
#define GPIO_BSRR_BR6                  GPIO_BSRR_BR6_Msk
#define GPIO_BSRR_BR7_Pos              (23UL)
#define GPIO_BSRR_BR7_Msk              (0x1UL << GPIO_BSRR_BR7_Pos)            /*!< 0x00800000 */
#define GPIO_BSRR_BR7                  GPIO_BSRR_BR7_Msk
#define GPIO_BSRR_BR8_Pos              (24UL)
#define GPIO_BSRR_BR8_Msk              (0x1UL << GPIO_BSRR_BR8_Pos)            /*!< 0x01000000 */
#define GPIO_BSRR_BR8                  GPIO_BSRR_BR8_Msk
#define GPIO_BSRR_BR9_Pos              (25UL)
#define GPIO_BSRR_BR9_Msk              (0x1UL << GPIO_BSRR_BR9_Pos)            /*!< 0x02000000 */
#define GPIO_BSRR_BR9                  GPIO_BSRR_BR9_Msk
#define GPIO_BSRR_BR10_Pos             (26UL)
#define GPIO_BSRR_BR10_Msk             (0x1UL << GPIO_BSRR_BR10_Pos)           /*!< 0x04000000 */
#define GPIO_BSRR_BR10                 GPIO_BSRR_BR10_Msk
#define GPIO_BSRR_BR11_Pos             (27UL)
#define GPIO_BSRR_BR11_Msk             (0x1UL << GPIO_BSRR_BR11_Pos)           /*!< 0x08000000 */
#define GPIO_BSRR_BR11                 GPIO_BSRR_BR11_Msk
#define GPIO_BSRR_BR12_Pos             (28UL)
#define GPIO_BSRR_BR12_Msk             (0x1UL << GPIO_BSRR_BR12_Pos)           /*!< 0x10000000 */
#define GPIO_BSRR_BR12                 GPIO_BSRR_BR12_Msk
#define GPIO_BSRR_BR13_Pos             (29UL)
#define GPIO_BSRR_BR13_Msk             (0x1UL << GPIO_BSRR_BR13_Pos)           /*!< 0x20000000 */
#define GPIO_BSRR_BR13                 GPIO_BSRR_BR13_Msk
#define GPIO_BSRR_BR14_Pos             (30UL)
#define GPIO_BSRR_BR14_Msk             (0x1UL << GPIO_BSRR_BR14_Pos)           /*!< 0x40000000 */
#define GPIO_BSRR_BR14                 GPIO_BSRR_BR14_Msk
#define GPIO_BSRR_BR15_Pos             (31UL)
#define GPIO_BSRR_BR15_Msk             (0x1UL << GPIO_BSRR_BR15_Pos)           /*!< 0x80000000 */
#define GPIO_BSRR_BR15                 GPIO_BSRR_BR15_Msk

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0_Pos             (0UL)
#define GPIO_LCKR_LCK0_Msk             (0x1UL << GPIO_LCKR_LCK0_Pos)           /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                 GPIO_LCKR_LCK0_Msk
#define GPIO_LCKR_LCK1_Pos             (1UL)
#define GPIO_LCKR_LCK1_Msk             (0x1UL << GPIO_LCKR_LCK1_Pos)           /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                 GPIO_LCKR_LCK1_Msk
#define GPIO_LCKR_LCK2_Pos             (2UL)
#define GPIO_LCKR_LCK2_Msk             (0x1UL << GPIO_LCKR_LCK2_Pos)           /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                 GPIO_LCKR_LCK2_Msk
#define GPIO_LCKR_LCK3_Pos             (3UL)
#define GPIO_LCKR_LCK3_Msk             (0x1UL << GPIO_LCKR_LCK3_Pos)           /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                 GPIO_LCKR_LCK3_Msk
#define GPIO_LCKR_LCK4_Pos             (4UL)
#define GPIO_LCKR_LCK4_Msk             (0x1UL << GPIO_LCKR_LCK4_Pos)           /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                 GPIO_LCKR_LCK4_Msk
#define GPIO_LCKR_LCK5_Pos             (5UL)
#define GPIO_LCKR_LCK5_Msk             (0x1UL << GPIO_LCKR_LCK5_Pos)           /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                 GPIO_LCKR_LCK5_Msk
#define GPIO_LCKR_LCK6_Pos             (6UL)
#define GPIO_LCKR_LCK6_Msk             (0x1UL << GPIO_LCKR_LCK6_Pos)           /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                 GPIO_LCKR_LCK6_Msk
#define GPIO_LCKR_LCK7_Pos             (7UL)
#define GPIO_LCKR_LCK7_Msk             (0x1UL << GPIO_LCKR_LCK7_Pos)           /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                 GPIO_LCKR_LCK7_Msk
#define GPIO_LCKR_LCK8_Pos             (8UL)
#define GPIO_LCKR_LCK8_Msk             (0x1UL << GPIO_LCKR_LCK8_Pos)           /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                 GPIO_LCKR_LCK8_Msk
#define GPIO_LCKR_LCK9_Pos             (9UL)
#define GPIO_LCKR_LCK9_Msk             (0x1UL << GPIO_LCKR_LCK9_Pos)           /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                 GPIO_LCKR_LCK9_Msk
#define GPIO_LCKR_LCK10_Pos            (10UL)
#define GPIO_LCKR_LCK10_Msk            (0x1UL << GPIO_LCKR_LCK10_Pos)          /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                GPIO_LCKR_LCK10_Msk
#define GPIO_LCKR_LCK11_Pos            (11UL)
#define GPIO_LCKR_LCK11_Msk            (0x1UL << GPIO_LCKR_LCK11_Pos)          /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                GPIO_LCKR_LCK11_Msk
#define GPIO_LCKR_LCK12_Pos            (12UL)
#define GPIO_LCKR_LCK12_Msk            (0x1UL << GPIO_LCKR_LCK12_Pos)          /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                GPIO_LCKR_LCK12_Msk
#define GPIO_LCKR_LCK13_Pos            (13UL)
#define GPIO_LCKR_LCK13_Msk            (0x1UL << GPIO_LCKR_LCK13_Pos)          /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                GPIO_LCKR_LCK13_Msk
#define GPIO_LCKR_LCK14_Pos            (14UL)
#define GPIO_LCKR_LCK14_Msk            (0x1UL << GPIO_LCKR_LCK14_Pos)          /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                GPIO_LCKR_LCK14_Msk
#define GPIO_LCKR_LCK15_Pos            (15UL)
#define GPIO_LCKR_LCK15_Msk            (0x1UL << GPIO_LCKR_LCK15_Pos)          /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                GPIO_LCKR_LCK15_Msk
#define GPIO_LCKR_LCKK_Pos             (16UL)
#define GPIO_LCKR_LCKK_Msk             (0x1UL << GPIO_LCKR_LCKK_Pos)           /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                 GPIO_LCKR_LCKK_Msk

/****************** Bit definition for GPIO_AFRL register  ********************/
#define GPIO_AFRL_AFSEL0_Pos           (0UL)
#define GPIO_AFRL_AFSEL0_Msk           (0xFUL << GPIO_AFRL_AFSEL0_Pos)         /*!< 0x0000000F */
#define GPIO_AFRL_AFSEL0               GPIO_AFRL_AFSEL0_Msk
#define GPIO_AFRL_AFSEL0_0             (0x1UL << GPIO_AFRL_AFSEL0_Pos)          /*!< 0x00000001 */
#define GPIO_AFRL_AFSEL0_1             (0x2UL << GPIO_AFRL_AFSEL0_Pos)          /*!< 0x00000002 */
#define GPIO_AFRL_AFSEL0_2             (0x4UL << GPIO_AFRL_AFSEL0_Pos)          /*!< 0x00000004 */
#define GPIO_AFRL_AFSEL0_3             (0x8UL << GPIO_AFRL_AFSEL0_Pos)          /*!< 0x00000008 */
#define GPIO_AFRL_AFSEL1_Pos           (4UL)
#define GPIO_AFRL_AFSEL1_Msk           (0xFUL << GPIO_AFRL_AFSEL1_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRL_AFSEL1               GPIO_AFRL_AFSEL1_Msk
#define GPIO_AFRL_AFSEL1_0             (0x1UL << GPIO_AFRL_AFSEL1_Pos)          /*!< 0x00000010 */
#define GPIO_AFRL_AFSEL1_1             (0x2UL << GPIO_AFRL_AFSEL1_Pos)          /*!< 0x00000020 */
#define GPIO_AFRL_AFSEL1_2             (0x4UL << GPIO_AFRL_AFSEL1_Pos)          /*!< 0x00000040 */
#define GPIO_AFRL_AFSEL1_3             (0x8UL << GPIO_AFRL_AFSEL1_Pos)          /*!< 0x00000080 */
#define GPIO_AFRL_AFSEL2_Pos           (8UL)
#define GPIO_AFRL_AFSEL2_Msk           (0xFUL << GPIO_AFRL_AFSEL2_Pos)         /*!< 0x00000F00 */
#define GPIO_AFRL_AFSEL2               GPIO_AFRL_AFSEL2_Msk
#define GPIO_AFRL_AFSEL2_0             (0x1UL << GPIO_AFRL_AFSEL2_Pos)          /*!< 0x00000100 */
#define GPIO_AFRL_AFSEL2_1             (0x2UL << GPIO_AFRL_AFSEL2_Pos)          /*!< 0x00000200 */
#define GPIO_AFRL_AFSEL2_2             (0x4UL << GPIO_AFRL_AFSEL2_Pos)          /*!< 0x00000400 */
#define GPIO_AFRL_AFSEL2_3             (0x8UL << GPIO_AFRL_AFSEL2_Pos)          /*!< 0x00000800 */
#define GPIO_AFRL_AFSEL3_Pos           (12UL)
#define GPIO_AFRL_AFSEL3_Msk           (0xFUL << GPIO_AFRL_AFSEL3_Pos)         /*!< 0x0000F000 */
#define GPIO_AFRL_AFSEL3               GPIO_AFRL_AFSEL3_Msk
#define GPIO_AFRL_AFSEL3_0             (0x1UL << GPIO_AFRL_AFSEL3_Pos)          /*!< 0x00001000 */
#define GPIO_AFRL_AFSEL3_1             (0x2UL << GPIO_AFRL_AFSEL3_Pos)          /*!< 0x00002000 */
#define GPIO_AFRL_AFSEL3_2             (0x4UL << GPIO_AFRL_AFSEL3_Pos)          /*!< 0x00004000 */
#define GPIO_AFRL_AFSEL3_3             (0x8UL << GPIO_AFRL_AFSEL3_Pos)          /*!< 0x00008000 */
#define GPIO_AFRL_AFSEL4_Pos           (16UL)
#define GPIO_AFRL_AFSEL4_Msk           (0xFUL << GPIO_AFRL_AFSEL4_Pos)         /*!< 0x000F0000 */
#define GPIO_AFRL_AFSEL4               GPIO_AFRL_AFSEL4_Msk
#define GPIO_AFRL_AFSEL4_0             (0x1UL << GPIO_AFRL_AFSEL4_Pos)          /*!< 0x00010000 */
#define GPIO_AFRL_AFSEL4_1             (0x2UL << GPIO_AFRL_AFSEL4_Pos)          /*!< 0x00020000 */
#define GPIO_AFRL_AFSEL4_2             (0x4UL << GPIO_AFRL_AFSEL4_Pos)          /*!< 0x00040000 */
#define GPIO_AFRL_AFSEL4_3             (0x8UL << GPIO_AFRL_AFSEL4_Pos)          /*!< 0x00080000 */
#define GPIO_AFRL_AFSEL5_Pos           (20UL)
#define GPIO_AFRL_AFSEL5_Msk           (0xFUL << GPIO_AFRL_AFSEL5_Pos)         /*!< 0x00F00000 */
#define GPIO_AFRL_AFSEL5               GPIO_AFRL_AFSEL5_Msk
#define GPIO_AFRL_AFSEL5_0             (0x1UL << GPIO_AFRL_AFSEL5_Pos)          /*!< 0x00100000 */
#define GPIO_AFRL_AFSEL5_1             (0x2UL << GPIO_AFRL_AFSEL5_Pos)          /*!< 0x00200000 */
#define GPIO_AFRL_AFSEL5_2             (0x4UL << GPIO_AFRL_AFSEL5_Pos)          /*!< 0x00400000 */
#define GPIO_AFRL_AFSEL5_3             (0x8UL << GPIO_AFRL_AFSEL5_Pos)          /*!< 0x00800000 */
#define GPIO_AFRL_AFSEL6_Pos           (24UL)
#define GPIO_AFRL_AFSEL6_Msk           (0xFUL << GPIO_AFRL_AFSEL6_Pos)         /*!< 0x0F000000 */
#define GPIO_AFRL_AFSEL6               GPIO_AFRL_AFSEL6_Msk
#define GPIO_AFRL_AFSEL6_0             (0x1UL << GPIO_AFRL_AFSEL6_Pos)          /*!< 0x01000000 */
#define GPIO_AFRL_AFSEL6_1             (0x2UL << GPIO_AFRL_AFSEL6_Pos)          /*!< 0x02000000 */
#define GPIO_AFRL_AFSEL6_2             (0x4UL << GPIO_AFRL_AFSEL6_Pos)          /*!< 0x04000000 */
#define GPIO_AFRL_AFSEL6_3             (0x8UL << GPIO_AFRL_AFSEL6_Pos)          /*!< 0x08000000 */
#define GPIO_AFRL_AFSEL7_Pos           (28UL)
#define GPIO_AFRL_AFSEL7_Msk           (0xFUL << GPIO_AFRL_AFSEL7_Pos)         /*!< 0xF0000000 */
#define GPIO_AFRL_AFSEL7               GPIO_AFRL_AFSEL7_Msk
#define GPIO_AFRL_AFSEL7_0             (0x1UL << GPIO_AFRL_AFSEL7_Pos)          /*!< 0x10000000 */
#define GPIO_AFRL_AFSEL7_1             (0x2UL << GPIO_AFRL_AFSEL7_Pos)          /*!< 0x20000000 */
#define GPIO_AFRL_AFSEL7_2             (0x4UL << GPIO_AFRL_AFSEL7_Pos)          /*!< 0x40000000 */
#define GPIO_AFRL_AFSEL7_3             (0x8UL << GPIO_AFRL_AFSEL7_Pos)          /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_AFRL_AFRL0                      GPIO_AFRL_AFSEL0
#define GPIO_AFRL_AFRL1                      GPIO_AFRL_AFSEL1
#define GPIO_AFRL_AFRL2                      GPIO_AFRL_AFSEL2
#define GPIO_AFRL_AFRL3                      GPIO_AFRL_AFSEL3
#define GPIO_AFRL_AFRL4                      GPIO_AFRL_AFSEL4
#define GPIO_AFRL_AFRL5                      GPIO_AFRL_AFSEL5
#define GPIO_AFRL_AFRL6                      GPIO_AFRL_AFSEL6
#define GPIO_AFRL_AFRL7                      GPIO_AFRL_AFSEL7

/****************** Bit definition for GPIO_AFRH register  ********************/
#define GPIO_AFRH_AFSEL8_Pos           (0UL)
#define GPIO_AFRH_AFSEL8_Msk           (0xFUL << GPIO_AFRH_AFSEL8_Pos)         /*!< 0x0000000F */
#define GPIO_AFRH_AFSEL8               GPIO_AFRH_AFSEL8_Msk
#define GPIO_AFRH_AFSEL8_0             (0x1UL << GPIO_AFRH_AFSEL8_Pos)          /*!< 0x00000001 */
#define GPIO_AFRH_AFSEL8_1             (0x2UL << GPIO_AFRH_AFSEL8_Pos)          /*!< 0x00000002 */
#define GPIO_AFRH_AFSEL8_2             (0x4UL << GPIO_AFRH_AFSEL8_Pos)          /*!< 0x00000004 */
#define GPIO_AFRH_AFSEL8_3             (0x8UL << GPIO_AFRH_AFSEL8_Pos)          /*!< 0x00000008 */
#define GPIO_AFRH_AFSEL9_Pos           (4UL)
#define GPIO_AFRH_AFSEL9_Msk           (0xFUL << GPIO_AFRH_AFSEL9_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRH_AFSEL9               GPIO_AFRH_AFSEL9_Msk
#define GPIO_AFRH_AFSEL9_0             (0x1UL << GPIO_AFRH_AFSEL9_Pos)          /*!< 0x00000010 */
#define GPIO_AFRH_AFSEL9_1             (0x2UL << GPIO_AFRH_AFSEL9_Pos)          /*!< 0x00000020 */
#define GPIO_AFRH_AFSEL9_2             (0x4UL << GPIO_AFRH_AFSEL9_Pos)          /*!< 0x00000040 */
#define GPIO_AFRH_AFSEL9_3             (0x8UL << GPIO_AFRH_AFSEL9_Pos)          /*!< 0x00000080 */
#define GPIO_AFRH_AFSEL10_Pos          (8UL)
#define GPIO_AFRH_AFSEL10_Msk          (0xFUL << GPIO_AFRH_AFSEL10_Pos)        /*!< 0x00000F00 */
#define GPIO_AFRH_AFSEL10              GPIO_AFRH_AFSEL10_Msk
#define GPIO_AFRH_AFSEL10_0            (0x1UL << GPIO_AFRH_AFSEL10_Pos)         /*!< 0x00000100 */
#define GPIO_AFRH_AFSEL10_1            (0x2UL << GPIO_AFRH_AFSEL10_Pos)         /*!< 0x00000200 */
#define GPIO_AFRH_AFSEL10_2            (0x4UL << GPIO_AFRH_AFSEL10_Pos)         /*!< 0x00000400 */
#define GPIO_AFRH_AFSEL10_3            (0x8UL << GPIO_AFRH_AFSEL10_Pos)         /*!< 0x00000800 */
#define GPIO_AFRH_AFSEL11_Pos          (12UL)
#define GPIO_AFRH_AFSEL11_Msk          (0xFUL << GPIO_AFRH_AFSEL11_Pos)        /*!< 0x0000F000 */
#define GPIO_AFRH_AFSEL11              GPIO_AFRH_AFSEL11_Msk
#define GPIO_AFRH_AFSEL11_0            (0x1UL << GPIO_AFRH_AFSEL11_Pos)         /*!< 0x00001000 */
#define GPIO_AFRH_AFSEL11_1            (0x2UL << GPIO_AFRH_AFSEL11_Pos)         /*!< 0x00002000 */
#define GPIO_AFRH_AFSEL11_2            (0x4UL << GPIO_AFRH_AFSEL11_Pos)         /*!< 0x00004000 */
#define GPIO_AFRH_AFSEL11_3            (0x8UL << GPIO_AFRH_AFSEL11_Pos)         /*!< 0x00008000 */
#define GPIO_AFRH_AFSEL12_Pos          (16UL)
#define GPIO_AFRH_AFSEL12_Msk          (0xFUL << GPIO_AFRH_AFSEL12_Pos)        /*!< 0x000F0000 */
#define GPIO_AFRH_AFSEL12              GPIO_AFRH_AFSEL12_Msk
#define GPIO_AFRH_AFSEL12_0            (0x1UL << GPIO_AFRH_AFSEL12_Pos)         /*!< 0x00010000 */
#define GPIO_AFRH_AFSEL12_1            (0x2UL << GPIO_AFRH_AFSEL12_Pos)         /*!< 0x00020000 */
#define GPIO_AFRH_AFSEL12_2            (0x4UL << GPIO_AFRH_AFSEL12_Pos)         /*!< 0x00040000 */
#define GPIO_AFRH_AFSEL12_3            (0x8UL << GPIO_AFRH_AFSEL12_Pos)         /*!< 0x00080000 */
#define GPIO_AFRH_AFSEL13_Pos          (20UL)
#define GPIO_AFRH_AFSEL13_Msk          (0xFUL << GPIO_AFRH_AFSEL13_Pos)        /*!< 0x00F00000 */
#define GPIO_AFRH_AFSEL13              GPIO_AFRH_AFSEL13_Msk
#define GPIO_AFRH_AFSEL13_0            (0x1UL << GPIO_AFRH_AFSEL13_Pos)         /*!< 0x00100000 */
#define GPIO_AFRH_AFSEL13_1            (0x2UL << GPIO_AFRH_AFSEL13_Pos)         /*!< 0x00200000 */
#define GPIO_AFRH_AFSEL13_2            (0x4UL << GPIO_AFRH_AFSEL13_Pos)         /*!< 0x00400000 */
#define GPIO_AFRH_AFSEL13_3            (0x8UL << GPIO_AFRH_AFSEL13_Pos)         /*!< 0x00800000 */
#define GPIO_AFRH_AFSEL14_Pos          (24UL)
#define GPIO_AFRH_AFSEL14_Msk          (0xFUL << GPIO_AFRH_AFSEL14_Pos)        /*!< 0x0F000000 */
#define GPIO_AFRH_AFSEL14              GPIO_AFRH_AFSEL14_Msk
#define GPIO_AFRH_AFSEL14_0            (0x1UL << GPIO_AFRH_AFSEL14_Pos)         /*!< 0x01000000 */
#define GPIO_AFRH_AFSEL14_1            (0x2UL << GPIO_AFRH_AFSEL14_Pos)         /*!< 0x02000000 */
#define GPIO_AFRH_AFSEL14_2            (0x4UL << GPIO_AFRH_AFSEL14_Pos)         /*!< 0x04000000 */
#define GPIO_AFRH_AFSEL14_3            (0x8UL << GPIO_AFRH_AFSEL14_Pos)         /*!< 0x08000000 */
#define GPIO_AFRH_AFSEL15_Pos          (28UL)
#define GPIO_AFRH_AFSEL15_Msk          (0xFUL << GPIO_AFRH_AFSEL15_Pos)        /*!< 0xF0000000 */
#define GPIO_AFRH_AFSEL15              GPIO_AFRH_AFSEL15_Msk
#define GPIO_AFRH_AFSEL15_0            (0x1UL << GPIO_AFRH_AFSEL15_Pos)         /*!< 0x10000000 */
#define GPIO_AFRH_AFSEL15_1            (0x2UL << GPIO_AFRH_AFSEL15_Pos)         /*!< 0x20000000 */
#define GPIO_AFRH_AFSEL15_2            (0x4UL << GPIO_AFRH_AFSEL15_Pos)         /*!< 0x40000000 */
#define GPIO_AFRH_AFSEL15_3            (0x8UL << GPIO_AFRH_AFSEL15_Pos)         /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_AFRH_AFRH0                      GPIO_AFRH_AFSEL8
#define GPIO_AFRH_AFRH1                      GPIO_AFRH_AFSEL9
#define GPIO_AFRH_AFRH2                      GPIO_AFRH_AFSEL10
#define GPIO_AFRH_AFRH3                      GPIO_AFRH_AFSEL11
#define GPIO_AFRH_AFRH4                      GPIO_AFRH_AFSEL12
#define GPIO_AFRH_AFRH5                      GPIO_AFRH_AFSEL13
#define GPIO_AFRH_AFRH6                      GPIO_AFRH_AFSEL14
#define GPIO_AFRH_AFRH7                      GPIO_AFRH_AFSEL15

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface (I2C)              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  *******************/
#define I2C_CR1_PE_Pos                      (0UL)
#define I2C_CR1_PE_Msk                      (0x1UL << I2C_CR1_PE_Pos)               /*!< 0x00000001 */
#define I2C_CR1_PE                          I2C_CR1_PE_Msk                          /*!< Peripheral enable */
#define I2C_CR1_TXIE_Pos                    (1UL)
#define I2C_CR1_TXIE_Msk                    (0x1UL << I2C_CR1_TXIE_Pos)             /*!< 0x00000002 */
#define I2C_CR1_TXIE                        I2C_CR1_TXIE_Msk                        /*!< TX interrupt enable */
#define I2C_CR1_RXIE_Pos                    (2UL)
#define I2C_CR1_RXIE_Msk                    (0x1UL << I2C_CR1_RXIE_Pos)             /*!< 0x00000004 */
#define I2C_CR1_RXIE                        I2C_CR1_RXIE_Msk                        /*!< RX interrupt enable */
#define I2C_CR1_ADDRIE_Pos                  (3UL)
#define I2C_CR1_ADDRIE_Msk                  (0x1UL << I2C_CR1_ADDRIE_Pos)           /*!< 0x00000008 */
#define I2C_CR1_ADDRIE                      I2C_CR1_ADDRIE_Msk                      /*!< Address match interrupt enable */
#define I2C_CR1_NACKIE_Pos                  (4UL)
#define I2C_CR1_NACKIE_Msk                  (0x1UL << I2C_CR1_NACKIE_Pos)           /*!< 0x00000010 */
#define I2C_CR1_NACKIE                      I2C_CR1_NACKIE_Msk                      /*!< NACK received interrupt enable */
#define I2C_CR1_STOPIE_Pos                  (5UL)
#define I2C_CR1_STOPIE_Msk                  (0x1UL << I2C_CR1_STOPIE_Pos)           /*!< 0x00000020 */
#define I2C_CR1_STOPIE                      I2C_CR1_STOPIE_Msk                      /*!< STOP detection interrupt enable */
#define I2C_CR1_TCIE_Pos                    (6UL)
#define I2C_CR1_TCIE_Msk                    (0x1UL << I2C_CR1_TCIE_Pos)             /*!< 0x00000040 */
#define I2C_CR1_TCIE                        I2C_CR1_TCIE_Msk                        /*!< Transfer complete interrupt enable */
#define I2C_CR1_ERRIE_Pos                   (7UL)
#define I2C_CR1_ERRIE_Msk                   (0x1UL << I2C_CR1_ERRIE_Pos)            /*!< 0x00000080 */
#define I2C_CR1_ERRIE                       I2C_CR1_ERRIE_Msk                       /*!< Errors interrupt enable */
#define I2C_CR1_DNF_Pos                     (8UL)
#define I2C_CR1_DNF_Msk                     (0xFUL << I2C_CR1_DNF_Pos)              /*!< 0x00000F00 */
#define I2C_CR1_DNF                         I2C_CR1_DNF_Msk                         /*!< Digital noise filter */
#define I2C_CR1_ANFOFF_Pos                  (12UL)
#define I2C_CR1_ANFOFF_Msk                  (0x1UL << I2C_CR1_ANFOFF_Pos)           /*!< 0x00001000 */
#define I2C_CR1_ANFOFF                      I2C_CR1_ANFOFF_Msk                      /*!< Analog noise filter OFF */
#define I2C_CR1_TXDMAEN_Pos                 (14UL)
#define I2C_CR1_TXDMAEN_Msk                 (0x1UL << I2C_CR1_TXDMAEN_Pos)          /*!< 0x00004000 */
#define I2C_CR1_TXDMAEN                     I2C_CR1_TXDMAEN_Msk                     /*!< DMA transmission requests enable */
#define I2C_CR1_RXDMAEN_Pos                 (15UL)
#define I2C_CR1_RXDMAEN_Msk                 (0x1UL << I2C_CR1_RXDMAEN_Pos)          /*!< 0x00008000 */
#define I2C_CR1_RXDMAEN                     I2C_CR1_RXDMAEN_Msk                     /*!< DMA reception requests enable */
#define I2C_CR1_SBC_Pos                     (16UL)
#define I2C_CR1_SBC_Msk                     (0x1UL << I2C_CR1_SBC_Pos)              /*!< 0x00010000 */
#define I2C_CR1_SBC                         I2C_CR1_SBC_Msk                         /*!< Slave byte control */
#define I2C_CR1_NOSTRETCH_Pos               (17UL)
#define I2C_CR1_NOSTRETCH_Msk               (0x1UL << I2C_CR1_NOSTRETCH_Pos)        /*!< 0x00020000 */
#define I2C_CR1_NOSTRETCH                   I2C_CR1_NOSTRETCH_Msk                   /*!< Clock stretching disable */
#define I2C_CR1_WUPEN_Pos                   (18UL)
#define I2C_CR1_WUPEN_Msk                   (0x1UL << I2C_CR1_WUPEN_Pos)            /*!< 0x00040000 */
#define I2C_CR1_WUPEN                       I2C_CR1_WUPEN_Msk                       /*!< Wakeup from STOP enable */
#define I2C_CR1_GCEN_Pos                    (19UL)
#define I2C_CR1_GCEN_Msk                    (0x1UL << I2C_CR1_GCEN_Pos)             /*!< 0x00080000 */
#define I2C_CR1_GCEN                        I2C_CR1_GCEN_Msk                        /*!< General call enable */
#define I2C_CR1_FMP_Pos                     (24UL)
#define I2C_CR1_FMP_Msk                     (0x1UL << I2C_CR1_FMP_Pos)            /*!< 0x01000000 */
#define I2C_CR1_FMP                         I2C_CR1_FMP_Msk                       /*!< Fast-mode Plus 20 mA drive enable */
#define I2C_CR1_ADDRACLR_Pos                (30UL)
#define I2C_CR1_ADDRACLR_Msk                (0x1UL << I2C_CR1_ADDRACLR_Pos)         /*!< 0x40000000 */
#define I2C_CR1_ADDRACLR                    I2C_CR1_ADDRACLR_Msk                    /*!< ADDRACLR enable */
#define I2C_CR1_STOPFACLR_Pos               (31UL)
#define I2C_CR1_STOPFACLR_Msk               (0x1UL << I2C_CR1_STOPFACLR_Pos)        /*!< 0x80000000 */
#define I2C_CR1_STOPFACLR                   I2C_CR1_STOPFACLR_Msk                   /*!< STOPFACLR enable */

/******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_SADD_Pos                    (0UL)
#define I2C_CR2_SADD_Msk                    (0x3FFUL << I2C_CR2_SADD_Pos)           /*!< 0x000003FF */
#define I2C_CR2_SADD                        I2C_CR2_SADD_Msk                        /*!< Slave address (master mode) */
#define I2C_CR2_RD_WRN_Pos                  (10UL)
#define I2C_CR2_RD_WRN_Msk                  (0x1UL << I2C_CR2_RD_WRN_Pos)           /*!< 0x00000400 */
#define I2C_CR2_RD_WRN                      I2C_CR2_RD_WRN_Msk                      /*!< Transfer direction (master mode) */
#define I2C_CR2_ADD10_Pos                   (11UL)
#define I2C_CR2_ADD10_Msk                   (0x1UL << I2C_CR2_ADD10_Pos)            /*!< 0x00000800 */
#define I2C_CR2_ADD10                       I2C_CR2_ADD10_Msk                       /*!< 10-bit addressing mode (master mode) */
#define I2C_CR2_HEAD10R_Pos                 (12UL)
#define I2C_CR2_HEAD10R_Msk                 (0x1UL << I2C_CR2_HEAD10R_Pos)          /*!< 0x00001000 */
#define I2C_CR2_HEAD10R                     I2C_CR2_HEAD10R_Msk                     /*!< 10-bit address header only read direction (master mode) */
#define I2C_CR2_START_Pos                   (13UL)
#define I2C_CR2_START_Msk                   (0x1UL << I2C_CR2_START_Pos)            /*!< 0x00002000 */
#define I2C_CR2_START                       I2C_CR2_START_Msk                       /*!< START generation */
#define I2C_CR2_STOP_Pos                    (14UL)
#define I2C_CR2_STOP_Msk                    (0x1UL << I2C_CR2_STOP_Pos)             /*!< 0x00004000 */
#define I2C_CR2_STOP                        I2C_CR2_STOP_Msk                        /*!< STOP generation (master mode) */
#define I2C_CR2_NACK_Pos                    (15UL)
#define I2C_CR2_NACK_Msk                    (0x1UL << I2C_CR2_NACK_Pos)             /*!< 0x00008000 */
#define I2C_CR2_NACK                        I2C_CR2_NACK_Msk                        /*!< NACK generation (slave mode) */
#define I2C_CR2_NBYTES_Pos                  (16UL)
#define I2C_CR2_NBYTES_Msk                  (0xFFUL << I2C_CR2_NBYTES_Pos)          /*!< 0x00FF0000 */
#define I2C_CR2_NBYTES                      I2C_CR2_NBYTES_Msk                      /*!< Number of bytes */
#define I2C_CR2_RELOAD_Pos                  (24UL)
#define I2C_CR2_RELOAD_Msk                  (0x1UL << I2C_CR2_RELOAD_Pos)           /*!< 0x01000000 */
#define I2C_CR2_RELOAD                      I2C_CR2_RELOAD_Msk                      /*!< NBYTES reload mode */
#define I2C_CR2_AUTOEND_Pos                 (25UL)
#define I2C_CR2_AUTOEND_Msk                 (0x1UL << I2C_CR2_AUTOEND_Pos)          /*!< 0x02000000 */
#define I2C_CR2_AUTOEND                     I2C_CR2_AUTOEND_Msk                     /*!< Automatic end mode (master mode) */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define I2C_OAR1_OA1_Pos                    (0UL)
#define I2C_OAR1_OA1_Msk                    (0x3FFUL << I2C_OAR1_OA1_Pos)           /*!< 0x000003FF */
#define I2C_OAR1_OA1                        I2C_OAR1_OA1_Msk                        /*!< Interface own address 1 */
#define I2C_OAR1_OA1MODE_Pos                (10UL)
#define I2C_OAR1_OA1MODE_Msk                (0x1UL << I2C_OAR1_OA1MODE_Pos)         /*!< 0x00000400 */
#define I2C_OAR1_OA1MODE                    I2C_OAR1_OA1MODE_Msk                    /*!< Own address 1 10-bit mode */
#define I2C_OAR1_OA1EN_Pos                  (15UL)
#define I2C_OAR1_OA1EN_Msk                  (0x1UL << I2C_OAR1_OA1EN_Pos)           /*!< 0x00008000 */
#define I2C_OAR1_OA1EN                      I2C_OAR1_OA1EN_Msk                      /*!< Own address 1 enable */

/*******************  Bit definition for I2C_OAR2 register  ******************/
#define I2C_OAR2_OA2_Pos                    (1UL)
#define I2C_OAR2_OA2_Msk                    (0x7FUL << I2C_OAR2_OA2_Pos)            /*!< 0x000000FE */
#define I2C_OAR2_OA2                        I2C_OAR2_OA2_Msk                        /*!< Interface own address 2 */
#define I2C_OAR2_OA2MSK_Pos                 (8UL)
#define I2C_OAR2_OA2MSK_Msk                 (0x7UL << I2C_OAR2_OA2MSK_Pos)          /*!< 0x00000700 */
#define I2C_OAR2_OA2MSK                     I2C_OAR2_OA2MSK_Msk                     /*!< Own address 2 masks */
#define I2C_OAR2_OA2NOMASK                  (0x00000000UL)                          /*!< No mask                                        */
#define I2C_OAR2_OA2MASK01_Pos              (8UL)
#define I2C_OAR2_OA2MASK01_Msk              (0x1UL << I2C_OAR2_OA2MASK01_Pos)       /*!< 0x00000100 */
#define I2C_OAR2_OA2MASK01                  I2C_OAR2_OA2MASK01_Msk                  /*!< OA2[1] is masked, Only OA2[7:2] are compared   */
#define I2C_OAR2_OA2MASK02_Pos              (9UL)
#define I2C_OAR2_OA2MASK02_Msk              (0x1UL << I2C_OAR2_OA2MASK02_Pos)       /*!< 0x00000200 */
#define I2C_OAR2_OA2MASK02                  I2C_OAR2_OA2MASK02_Msk                  /*!< OA2[2:1] is masked, Only OA2[7:3] are compared */
#define I2C_OAR2_OA2MASK03_Pos              (8UL)
#define I2C_OAR2_OA2MASK03_Msk              (0x3UL << I2C_OAR2_OA2MASK03_Pos)       /*!< 0x00000300 */
#define I2C_OAR2_OA2MASK03                  I2C_OAR2_OA2MASK03_Msk                  /*!< OA2[3:1] is masked, Only OA2[7:4] are compared */
#define I2C_OAR2_OA2MASK04_Pos              (10UL)
#define I2C_OAR2_OA2MASK04_Msk              (0x1UL << I2C_OAR2_OA2MASK04_Pos)       /*!< 0x00000400 */
#define I2C_OAR2_OA2MASK04                  I2C_OAR2_OA2MASK04_Msk                  /*!< OA2[4:1] is masked, Only OA2[7:5] are compared */
#define I2C_OAR2_OA2MASK05_Pos              (8UL)
#define I2C_OAR2_OA2MASK05_Msk              (0x5UL << I2C_OAR2_OA2MASK05_Pos)       /*!< 0x00000500 */
#define I2C_OAR2_OA2MASK05                  I2C_OAR2_OA2MASK05_Msk                  /*!< OA2[5:1] is masked, Only OA2[7:6] are compared */
#define I2C_OAR2_OA2MASK06_Pos              (9UL)
#define I2C_OAR2_OA2MASK06_Msk              (0x3UL << I2C_OAR2_OA2MASK06_Pos)       /*!< 0x00000600 */
#define I2C_OAR2_OA2MASK06                  I2C_OAR2_OA2MASK06_Msk                  /*!< OA2[6:1] is masked, Only OA2[7] are compared   */
#define I2C_OAR2_OA2MASK07_Pos              (8UL)
#define I2C_OAR2_OA2MASK07_Msk              (0x7UL << I2C_OAR2_OA2MASK07_Pos)       /*!< 0x00000700 */
#define I2C_OAR2_OA2MASK07                  I2C_OAR2_OA2MASK07_Msk                  /*!< OA2[7:1] is masked, No comparison is done      */
#define I2C_OAR2_OA2EN_Pos                  (15UL)
#define I2C_OAR2_OA2EN_Msk                  (0x1UL << I2C_OAR2_OA2EN_Pos)           /*!< 0x00008000 */
#define I2C_OAR2_OA2EN                      I2C_OAR2_OA2EN_Msk                      /*!< Own address 2 enable */

/*******************  Bit definition for I2C_TIMINGR register *******************/
#define I2C_TIMINGR_SCLL_Pos                (0UL)
#define I2C_TIMINGR_SCLL_Msk                (0xFFUL << I2C_TIMINGR_SCLL_Pos)        /*!< 0x000000FF */
#define I2C_TIMINGR_SCLL                    I2C_TIMINGR_SCLL_Msk                    /*!< SCL low period (master mode) */
#define I2C_TIMINGR_SCLH_Pos                (8UL)
#define I2C_TIMINGR_SCLH_Msk                (0xFFUL << I2C_TIMINGR_SCLH_Pos)        /*!< 0x0000FF00 */
#define I2C_TIMINGR_SCLH                    I2C_TIMINGR_SCLH_Msk                    /*!< SCL high period (master mode) */
#define I2C_TIMINGR_SDADEL_Pos              (16UL)
#define I2C_TIMINGR_SDADEL_Msk              (0xFUL << I2C_TIMINGR_SDADEL_Pos)       /*!< 0x000F0000 */
#define I2C_TIMINGR_SDADEL                  I2C_TIMINGR_SDADEL_Msk                  /*!< Data hold time */
#define I2C_TIMINGR_SCLDEL_Pos              (20UL)
#define I2C_TIMINGR_SCLDEL_Msk              (0xFUL << I2C_TIMINGR_SCLDEL_Pos)       /*!< 0x00F00000 */
#define I2C_TIMINGR_SCLDEL                  I2C_TIMINGR_SCLDEL_Msk                  /*!< Data setup time */
#define I2C_TIMINGR_PRESC_Pos               (28UL)
#define I2C_TIMINGR_PRESC_Msk               (0xFUL << I2C_TIMINGR_PRESC_Pos)        /*!< 0xF0000000 */
#define I2C_TIMINGR_PRESC                   I2C_TIMINGR_PRESC_Msk                   /*!< Timings prescaler */

/******************  Bit definition for I2C_ISR register  *********************/
#define I2C_ISR_TXE_Pos                     (0UL)
#define I2C_ISR_TXE_Msk                     (0x1UL << I2C_ISR_TXE_Pos)              /*!< 0x00000001 */
#define I2C_ISR_TXE                         I2C_ISR_TXE_Msk                         /*!< Transmit data register empty */
#define I2C_ISR_TXIS_Pos                    (1UL)
#define I2C_ISR_TXIS_Msk                    (0x1UL << I2C_ISR_TXIS_Pos)             /*!< 0x00000002 */
#define I2C_ISR_TXIS                        I2C_ISR_TXIS_Msk                        /*!< Transmit interrupt status */
#define I2C_ISR_RXNE_Pos                    (2UL)
#define I2C_ISR_RXNE_Msk                    (0x1UL << I2C_ISR_RXNE_Pos)             /*!< 0x00000004 */
#define I2C_ISR_RXNE                        I2C_ISR_RXNE_Msk                        /*!< Receive data register not empty */
#define I2C_ISR_ADDR_Pos                    (3UL)
#define I2C_ISR_ADDR_Msk                    (0x1UL << I2C_ISR_ADDR_Pos)             /*!< 0x00000008 */
#define I2C_ISR_ADDR                        I2C_ISR_ADDR_Msk                        /*!< Address matched (slave mode)*/
#define I2C_ISR_NACKF_Pos                   (4UL)
#define I2C_ISR_NACKF_Msk                   (0x1UL << I2C_ISR_NACKF_Pos)            /*!< 0x00000010 */
#define I2C_ISR_NACKF                       I2C_ISR_NACKF_Msk                       /*!< NACK received flag */
#define I2C_ISR_STOPF_Pos                   (5UL)
#define I2C_ISR_STOPF_Msk                   (0x1UL << I2C_ISR_STOPF_Pos)            /*!< 0x00000020 */
#define I2C_ISR_STOPF                       I2C_ISR_STOPF_Msk                       /*!< STOP detection flag */
#define I2C_ISR_TC_Pos                      (6UL)
#define I2C_ISR_TC_Msk                      (0x1UL << I2C_ISR_TC_Pos)               /*!< 0x00000040 */
#define I2C_ISR_TC                          I2C_ISR_TC_Msk                          /*!< Transfer complete (master mode) */
#define I2C_ISR_TCR_Pos                     (7UL)
#define I2C_ISR_TCR_Msk                     (0x1UL << I2C_ISR_TCR_Pos)              /*!< 0x00000080 */
#define I2C_ISR_TCR                         I2C_ISR_TCR_Msk                         /*!< Transfer complete reload */
#define I2C_ISR_BERR_Pos                    (8UL)
#define I2C_ISR_BERR_Msk                    (0x1UL << I2C_ISR_BERR_Pos)             /*!< 0x00000100 */
#define I2C_ISR_BERR                        I2C_ISR_BERR_Msk                        /*!< Bus error */
#define I2C_ISR_ARLO_Pos                    (9UL)
#define I2C_ISR_ARLO_Msk                    (0x1UL << I2C_ISR_ARLO_Pos)             /*!< 0x00000200 */
#define I2C_ISR_ARLO                        I2C_ISR_ARLO_Msk                        /*!< Arbitration lost */
#define I2C_ISR_OVR_Pos                     (10UL)
#define I2C_ISR_OVR_Msk                     (0x1UL << I2C_ISR_OVR_Pos)              /*!< 0x00000400 */
#define I2C_ISR_OVR                         I2C_ISR_OVR_Msk                         /*!< Overrun/Underrun */
#define I2C_ISR_BUSY_Pos                    (15UL)
#define I2C_ISR_BUSY_Msk                    (0x1UL << I2C_ISR_BUSY_Pos)             /*!< 0x00008000 */
#define I2C_ISR_BUSY                        I2C_ISR_BUSY_Msk                        /*!< Bus busy */
#define I2C_ISR_DIR_Pos                     (16UL)
#define I2C_ISR_DIR_Msk                     (0x1UL << I2C_ISR_DIR_Pos)              /*!< 0x00010000 */
#define I2C_ISR_DIR                         I2C_ISR_DIR_Msk                         /*!< Transfer direction (slave mode) */
#define I2C_ISR_ADDCODE_Pos                 (17UL)
#define I2C_ISR_ADDCODE_Msk                 (0x7FUL << I2C_ISR_ADDCODE_Pos)         /*!< 0x00FE0000 */
#define I2C_ISR_ADDCODE                     I2C_ISR_ADDCODE_Msk                     /*!< Address match code (slave mode) */

/******************  Bit definition for I2C_ICR register  *********************/
#define I2C_ICR_ADDRCF_Pos                  (3UL)
#define I2C_ICR_ADDRCF_Msk                  (0x1UL << I2C_ICR_ADDRCF_Pos)           /*!< 0x00000008 */
#define I2C_ICR_ADDRCF                      I2C_ICR_ADDRCF_Msk                      /*!< Address matched clear flag */
#define I2C_ICR_NACKCF_Pos                  (4UL)
#define I2C_ICR_NACKCF_Msk                  (0x1UL << I2C_ICR_NACKCF_Pos)           /*!< 0x00000010 */
#define I2C_ICR_NACKCF                      I2C_ICR_NACKCF_Msk                      /*!< NACK clear flag */
#define I2C_ICR_STOPCF_Pos                  (5UL)
#define I2C_ICR_STOPCF_Msk                  (0x1UL << I2C_ICR_STOPCF_Pos)           /*!< 0x00000020 */
#define I2C_ICR_STOPCF                      I2C_ICR_STOPCF_Msk                      /*!< STOP detection clear flag */
#define I2C_ICR_BERRCF_Pos                  (8UL)
#define I2C_ICR_BERRCF_Msk                  (0x1UL << I2C_ICR_BERRCF_Pos)           /*!< 0x00000100 */
#define I2C_ICR_BERRCF                      I2C_ICR_BERRCF_Msk                      /*!< Bus error clear flag */
#define I2C_ICR_ARLOCF_Pos                  (9UL)
#define I2C_ICR_ARLOCF_Msk                  (0x1UL << I2C_ICR_ARLOCF_Pos)           /*!< 0x00000200 */
#define I2C_ICR_ARLOCF                      I2C_ICR_ARLOCF_Msk                      /*!< Arbitration lost clear flag */
#define I2C_ICR_OVRCF_Pos                   (10UL)
#define I2C_ICR_OVRCF_Msk                   (0x1UL << I2C_ICR_OVRCF_Pos)            /*!< 0x00000400 */
#define I2C_ICR_OVRCF                       I2C_ICR_OVRCF_Msk                       /*!< Overrun/Underrun clear flag */

/******************  Bit definition for I2C_RXDR register  *********************/
#define I2C_RXDR_RXDATA_Pos                 (0UL)
#define I2C_RXDR_RXDATA_Msk                 (0xFFUL << I2C_RXDR_RXDATA_Pos)         /*!< 0x000000FF */
#define I2C_RXDR_RXDATA                     I2C_RXDR_RXDATA_Msk                     /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *********************/
#define I2C_TXDR_TXDATA_Pos                 (0UL)
#define I2C_TXDR_TXDATA_Msk                 (0xFFUL << I2C_TXDR_TXDATA_Pos)         /*!< 0x000000FF */
#define I2C_TXDR_TXDATA                     I2C_TXDR_TXDATA_Msk                     /*!< 8-bit transmit data */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define IWDG_KR_KEY_Pos      (0UL)
#define IWDG_KR_KEY_Msk      (0xFFFFUL << IWDG_KR_KEY_Pos)                     /*!< 0x0000FFFF */
#define IWDG_KR_KEY          IWDG_KR_KEY_Msk                                   /*!<Key value (write only, read 0000h)  */

/*******************  Bit definition for IWDG_PR register  ********************/
#define IWDG_PR_PR_Pos       (0UL)
#define IWDG_PR_PR_Msk       (0xFUL << IWDG_PR_PR_Pos)                          /*!< 0x00000007 */
#define IWDG_PR_PR           IWDG_PR_PR_Msk                                     /*!< PR[3:0] (Prescaler divider)         */
#define IWDG_PR_PR_0         (0x1UL << IWDG_PR_PR_Pos)                          /*!< 0x00000001 */
#define IWDG_PR_PR_1         (0x2UL << IWDG_PR_PR_Pos)                          /*!< 0x00000002 */
#define IWDG_PR_PR_2         (0x4UL << IWDG_PR_PR_Pos)                          /*!< 0x00000004 */
#define IWDG_PR_PR_3         (0x8UL << IWDG_PR_PR_Pos)                          /*!< 0x00000008 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define IWDG_RLR_RL_Pos      (0UL)
#define IWDG_RLR_RL_Msk      (0xFFFUL << IWDG_RLR_RL_Pos)                      /*!< 0x00000FFF */
#define IWDG_RLR_RL          IWDG_RLR_RL_Msk                                   /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_SR register  ********************/
#define IWDG_SR_PVU_Pos      (0UL)
#define IWDG_SR_PVU_Msk      (0x1UL << IWDG_SR_PVU_Pos)                        /*!< 0x00000001 */
#define IWDG_SR_PVU          IWDG_SR_PVU_Msk                                   /*!< Watchdog prescaler value update */
#define IWDG_SR_RVU_Pos      (1UL)
#define IWDG_SR_RVU_Msk      (0x1UL << IWDG_SR_RVU_Pos)                        /*!< 0x00000002 */
#define IWDG_SR_RVU          IWDG_SR_RVU_Msk                                   /*!< Watchdog counter reload value update */
#define IWDG_SR_WVU_Pos      (2UL)
#define IWDG_SR_WVU_Msk      (0x1UL << IWDG_SR_WVU_Pos)                        /*!< 0x00000004 */
#define IWDG_SR_WVU          IWDG_SR_WVU_Msk                                   /*!< Watchdog counter window value update */
#define IWDG_SR_EWU_Pos      (3UL)
#define IWDG_SR_EWU_Msk      (0x1UL << IWDG_SR_EWU_Pos)                        /*!< 0x00000008 */
#define IWDG_SR_EWU          IWDG_SR_EWU_Msk                                   /*!< Watchdog interrupt comparator value update */
#define IWDG_SR_ONF_Pos      (8UL)
#define IWDG_SR_ONF_Msk      (0x1UL << IWDG_SR_ONF_Pos)                        /*!< 0x00000100 */
#define IWDG_SR_ONF          IWDG_SR_ONF_Msk                                   /*!< Watchdog Enable status bit */
#define IWDG_SR_EWIF_Pos     (14UL)
#define IWDG_SR_EWIF_Msk     (0x1UL << IWDG_SR_EWIF_Pos)                       /*!< 0x00004000 */
#define IWDG_SR_EWIF         IWDG_SR_EWIF_Msk                                  /*!< Watchdog early interrupt flag */

/*******************  Bit definition for IWDG_WINR register  ********************/
#define IWDG_WINR_WIN_Pos    (0UL)
#define IWDG_WINR_WIN_Msk    (0xFFFUL << IWDG_WINR_WIN_Pos)                    /*!< 0x00000FFF */
#define IWDG_WINR_WIN        IWDG_WINR_WIN_Msk                                 /*!< Watchdog counter window value */

/******************  Bit definition for IWDG_EWCR register  *******************/
#define IWDG_EWCR_EWIT_Pos                  (0UL)
#define IWDG_EWCR_EWIT_Msk                  (0xFFFUL << IWDG_EWCR_EWIT_Pos)         /*!< 0x00000FFF */
#define IWDG_EWCR_EWIT                      IWDG_EWCR_EWIT_Msk                      /*!< Watchdog early wakeup comparator value */
#define IWDG_EWCR_EWIC_Pos                  (14UL)
#define IWDG_EWCR_EWIC_Msk                  (0x1UL << IWDG_EWCR_EWIC_Pos)           /*!< 0x00004000 */
#define IWDG_EWCR_EWIC                      IWDG_EWCR_EWIC_Msk                      /*!< Watchdog early wakeup comparator value */
#define IWDG_EWCR_EWIE_Pos                  (15UL)
#define IWDG_EWCR_EWIE_Msk                  (0x1UL << IWDG_EWCR_EWIE_Pos)           /*!< 0x00008000 */
#define IWDG_EWCR_EWIE                      IWDG_EWCR_EWIE_Msk                      /*!< Watchdog early wakeup comparator value */

/******************************************************************************/
/*                                                                            */
/*                          LCD Controller (LCD)                              */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for LCD_CR register  *********************/
#define LCD_CR_LCDEN_Pos            (0UL)
#define LCD_CR_LCDEN_Msk            (0x1UL << LCD_CR_LCDEN_Pos)                /*!< 0x00000001 */
#define LCD_CR_LCDEN                LCD_CR_LCDEN_Msk                           /*!< LCD Enable Bit */
#define LCD_CR_VSEL_Pos             (1UL)
#define LCD_CR_VSEL_Msk             (0x1UL << LCD_CR_VSEL_Pos)                 /*!< 0x00000002 */
#define LCD_CR_VSEL                 LCD_CR_VSEL_Msk                            /*!< Voltage source selector Bit */

#define LCD_CR_DUTY_Pos             (2UL)
#define LCD_CR_DUTY_Msk             (0x7UL << LCD_CR_DUTY_Pos)                 /*!< 0x0000001C */
#define LCD_CR_DUTY                 LCD_CR_DUTY_Msk                            /*!< DUTY[2:0] bits (Duty selector) */
#define LCD_CR_DUTY_0               (0x1UL << LCD_CR_DUTY_Pos)                 /*!< 0x00000004 */
#define LCD_CR_DUTY_1               (0x2UL << LCD_CR_DUTY_Pos)                 /*!< 0x00000008 */
#define LCD_CR_DUTY_2               (0x4UL << LCD_CR_DUTY_Pos)                 /*!< 0x00000010 */

#define LCD_CR_BIAS_Pos             (5UL)
#define LCD_CR_BIAS_Msk             (0x3UL << LCD_CR_BIAS_Pos)                 /*!< 0x00000060 */
#define LCD_CR_BIAS                 LCD_CR_BIAS_Msk                            /*!< BIAS[1:0] bits (Bias selector) */
#define LCD_CR_BIAS_0               (0x1UL << LCD_CR_BIAS_Pos)                 /*!< 0x00000020 */
#define LCD_CR_BIAS_1               (0x2UL << LCD_CR_BIAS_Pos)                 /*!< 0x00000040 */

#define LCD_CR_MUX_SEG_Pos          (7UL)
#define LCD_CR_MUX_SEG_Msk          (0x1UL << LCD_CR_MUX_SEG_Pos)              /*!< 0x00000080 */
#define LCD_CR_MUX_SEG              LCD_CR_MUX_SEG_Msk                         /*!< Mux Segment Enable Bit */
#define LCD_CR_BUFEN_Pos            (8UL)
#define LCD_CR_BUFEN_Msk            (0x1UL << LCD_CR_BUFEN_Pos)                /*!< 0x00000100 */
#define LCD_CR_BUFEN                LCD_CR_BUFEN_Msk                           /*!< Voltage output buffer enable */

/*******************  Bit definition for LCD_FCR register  ********************/
#define LCD_FCR_HD_Pos              (0UL)
#define LCD_FCR_HD_Msk              (0x1UL << LCD_FCR_HD_Pos)                  /*!< 0x00000001 */
#define LCD_FCR_HD                  LCD_FCR_HD_Msk                             /*!< High Drive Enable Bit */
#define LCD_FCR_SOFIE_Pos           (1UL)
#define LCD_FCR_SOFIE_Msk           (0x1UL << LCD_FCR_SOFIE_Pos)               /*!< 0x00000002 */
#define LCD_FCR_SOFIE               LCD_FCR_SOFIE_Msk                          /*!< Start of Frame Interrupt Enable Bit */
#define LCD_FCR_UDDIE_Pos           (3UL)
#define LCD_FCR_UDDIE_Msk           (0x1UL << LCD_FCR_UDDIE_Pos)               /*!< 0x00000008 */
#define LCD_FCR_UDDIE               LCD_FCR_UDDIE_Msk                          /*!< Update Display Done Interrupt Enable Bit */

#define LCD_FCR_PON_Pos             (4UL)
#define LCD_FCR_PON_Msk             (0x7UL << LCD_FCR_PON_Pos)                 /*!< 0x00000070 */
#define LCD_FCR_PON                 LCD_FCR_PON_Msk                            /*!< PON[2:0] bits (Pulse ON Duration) */
#define LCD_FCR_PON_0               (0x1UL << LCD_FCR_PON_Pos)                 /*!< 0x00000010 */
#define LCD_FCR_PON_1               (0x2UL << LCD_FCR_PON_Pos)                 /*!< 0x00000020 */
#define LCD_FCR_PON_2               (0x4UL << LCD_FCR_PON_Pos)                 /*!< 0x00000040 */

#define LCD_FCR_DEAD_Pos            (7UL)
#define LCD_FCR_DEAD_Msk            (0x7UL << LCD_FCR_DEAD_Pos)                /*!< 0x00000380 */
#define LCD_FCR_DEAD                LCD_FCR_DEAD_Msk                           /*!< DEAD[2:0] bits (DEAD Time) */
#define LCD_FCR_DEAD_0              (0x1UL << LCD_FCR_DEAD_Pos)                /*!< 0x00000080 */
#define LCD_FCR_DEAD_1              (0x2UL << LCD_FCR_DEAD_Pos)                /*!< 0x00000100 */
#define LCD_FCR_DEAD_2              (0x4UL << LCD_FCR_DEAD_Pos)                /*!< 0x00000200 */

#define LCD_FCR_CC_Pos              (10UL)
#define LCD_FCR_CC_Msk              (0x7UL << LCD_FCR_CC_Pos)                  /*!< 0x00001C00 */
#define LCD_FCR_CC                  LCD_FCR_CC_Msk                             /*!< CC[2:0] bits (Contrast Control) */
#define LCD_FCR_CC_0                (0x1UL << LCD_FCR_CC_Pos)                  /*!< 0x00000400 */
#define LCD_FCR_CC_1                (0x2UL << LCD_FCR_CC_Pos)                  /*!< 0x00000800 */
#define LCD_FCR_CC_2                (0x4UL << LCD_FCR_CC_Pos)                  /*!< 0x00001000 */

#define LCD_FCR_BLINKF_Pos          (13UL)
#define LCD_FCR_BLINKF_Msk          (0x7UL << LCD_FCR_BLINKF_Pos)              /*!< 0x0000E000 */
#define LCD_FCR_BLINKF              LCD_FCR_BLINKF_Msk                         /*!< BLINKF[2:0] bits (Blink Frequency) */
#define LCD_FCR_BLINKF_0            (0x1UL << LCD_FCR_BLINKF_Pos)              /*!< 0x00002000 */
#define LCD_FCR_BLINKF_1            (0x2UL << LCD_FCR_BLINKF_Pos)              /*!< 0x00004000 */
#define LCD_FCR_BLINKF_2            (0x4UL << LCD_FCR_BLINKF_Pos)              /*!< 0x00008000 */

#define LCD_FCR_BLINK_Pos           (16UL)
#define LCD_FCR_BLINK_Msk           (0x3UL << LCD_FCR_BLINK_Pos)               /*!< 0x00030000 */
#define LCD_FCR_BLINK               LCD_FCR_BLINK_Msk                          /*!< BLINK[1:0] bits (Blink Enable) */
#define LCD_FCR_BLINK_0             (0x1UL << LCD_FCR_BLINK_Pos)               /*!< 0x00010000 */
#define LCD_FCR_BLINK_1             (0x2UL << LCD_FCR_BLINK_Pos)               /*!< 0x00020000 */

#define LCD_FCR_DIV_Pos             (18UL)
#define LCD_FCR_DIV_Msk             (0xFUL << LCD_FCR_DIV_Pos)                 /*!< 0x003C0000 */
#define LCD_FCR_DIV                 LCD_FCR_DIV_Msk                            /*!< DIV[3:0] bits (Divider) */
#define LCD_FCR_PS_Pos              (22UL)
#define LCD_FCR_PS_Msk              (0xFUL << LCD_FCR_PS_Pos)                  /*!< 0x03C00000 */
#define LCD_FCR_PS                  LCD_FCR_PS_Msk                             /*!< PS[3:0] bits (Prescaler) */

/*******************  Bit definition for LCD_SR register  *********************/
#define LCD_SR_ENS_Pos              (0UL)
#define LCD_SR_ENS_Msk              (0x1UL << LCD_SR_ENS_Pos)                  /*!< 0x00000001 */
#define LCD_SR_ENS                  LCD_SR_ENS_Msk                             /*!< LCD Enabled Bit */
#define LCD_SR_SOF_Pos              (1UL)
#define LCD_SR_SOF_Msk              (0x1UL << LCD_SR_SOF_Pos)                  /*!< 0x00000002 */
#define LCD_SR_SOF                  LCD_SR_SOF_Msk                             /*!< Start Of Frame Flag Bit */
#define LCD_SR_UDR_Pos              (2UL)
#define LCD_SR_UDR_Msk              (0x1UL << LCD_SR_UDR_Pos)                  /*!< 0x00000004 */
#define LCD_SR_UDR                  LCD_SR_UDR_Msk                             /*!< Update Display Request Bit */
#define LCD_SR_UDD_Pos              (3UL)
#define LCD_SR_UDD_Msk              (0x1UL << LCD_SR_UDD_Pos)                  /*!< 0x00000008 */
#define LCD_SR_UDD                  LCD_SR_UDD_Msk                             /*!< Update Display Done Flag Bit */
#define LCD_SR_RDY_Pos              (4UL)
#define LCD_SR_RDY_Msk              (0x1UL << LCD_SR_RDY_Pos)                  /*!< 0x00000010 */
#define LCD_SR_RDY                  LCD_SR_RDY_Msk                             /*!< Ready Flag Bit */
#define LCD_SR_FCRSR_Pos            (5UL)
#define LCD_SR_FCRSR_Msk            (0x1UL << LCD_SR_FCRSR_Pos)                /*!< 0x00000020 */
#define LCD_SR_FCRSR                LCD_SR_FCRSR_Msk                           /*!< LCD FCR Register Synchronization Flag Bit */

/*******************  Bit definition for LCD_CLR register  ********************/
#define LCD_CLR_SOFC_Pos            (1UL)
#define LCD_CLR_SOFC_Msk            (0x1UL << LCD_CLR_SOFC_Pos)                /*!< 0x00000002 */
#define LCD_CLR_SOFC                LCD_CLR_SOFC_Msk                           /*!< Start Of Frame Flag Clear Bit */
#define LCD_CLR_UDDC_Pos            (3UL)
#define LCD_CLR_UDDC_Msk            (0x1UL << LCD_CLR_UDDC_Pos)                /*!< 0x00000008 */
#define LCD_CLR_UDDC                LCD_CLR_UDDC_Msk                           /*!< Update Display Done Flag Clear Bit */

/*******************  Bit definition for LCD_RAM register  ********************/
#define LCD_RAM_SEGMENT_DATA_Pos    (0UL)
#define LCD_RAM_SEGMENT_DATA_Msk    (0xFFFFFFFFUL << LCD_RAM_SEGMENT_DATA_Pos) /*!< 0xFFFFFFFF */
#define LCD_RAM_SEGMENT_DATA        LCD_RAM_SEGMENT_DATA_Msk                   /*!< Segment Data Bits */

/******************************************************************************/
/*                                                                            */
/*                         Operational Amplifier (OPAMP)                      */
/*                                                                            */
/******************************************************************************/
/*********************  Bit definition for OPAMPx_CSR register  ***************/
#define OPAMP_CSR_OPAEN_Pos                 (0UL)
#define OPAMP_CSR_OPAEN_Msk                 (0x1UL << OPAMP_CSR_OPAEN_Pos)            /*!< 0x00000001 */
#define OPAMP_CSR_OPAEN                     OPAMP_CSR_OPAEN_Msk                       /*!< OPAMP enable */

#define OPAMP_CSR_OPALPM_Pos                (1UL)
#define OPAMP_CSR_OPALPM_Msk                (0x1UL << OPAMP_CSR_OPALPM_Pos)           /*!< 0x00000002 */
#define OPAMP_CSR_OPALPM                    OPAMP_CSR_OPALPM_Msk                      /*!< Operational amplifier Low Power Mode */

#define OPAMP_CSR_OPAMODE_Pos               (2UL)
#define OPAMP_CSR_OPAMODE_Msk               (0x3UL << OPAMP_CSR_OPAMODE_Pos)          /*!< 0x0000000C */
#define OPAMP_CSR_OPAMODE                   OPAMP_CSR_OPAMODE_Msk                     /*!< Operational amplifier PGA mode */
#define OPAMP_CSR_OPAMODE_0                 (0x1UL << OPAMP_CSR_OPAMODE_Pos)          /*!< 0x00000004 */
#define OPAMP_CSR_OPAMODE_1                 (0x2UL << OPAMP_CSR_OPAMODE_Pos)          /*!< 0x00000008 */

#define OPAMP_CSR_PGA_GAIN_Pos              (4UL)
#define OPAMP_CSR_PGA_GAIN_Msk              (0x3UL << OPAMP_CSR_PGA_GAIN_Pos)         /*!< 0x00000030 */
#define OPAMP_CSR_PGA_GAIN                  OPAMP_CSR_PGA_GAIN_Msk                    /*!< Operational amplifier Programmable amplifier gain value */
#define OPAMP_CSR_PGA_GAIN_0                (0x1UL << OPAMP_CSR_PGA_GAIN_Pos)         /*!< 0x00000010 */
#define OPAMP_CSR_PGA_GAIN_1                (0x2UL << OPAMP_CSR_PGA_GAIN_Pos)         /*!< 0x00000020 */

#define OPAMP_CSR_VM_SEL_Pos                (8UL)
#define OPAMP_CSR_VM_SEL_Msk                (0x3UL << OPAMP_CSR_VM_SEL_Pos)           /*!< 0x00000300 */
#define OPAMP_CSR_VM_SEL                    OPAMP_CSR_VM_SEL_Msk                      /*!< Inverting input selection */
#define OPAMP_CSR_VM_SEL_0                  (0x1UL << OPAMP_CSR_VM_SEL_Pos)           /*!< 0x00000100 */
#define OPAMP_CSR_VM_SEL_1                  (0x2UL << OPAMP_CSR_VM_SEL_Pos)           /*!< 0x00000200 */

#define OPAMP_CSR_VP_SEL_Pos                (10UL)
#define OPAMP_CSR_VP_SEL_Msk                (0x1UL << OPAMP_CSR_VP_SEL_Pos)           /*!< 0x00000400 */
#define OPAMP_CSR_VP_SEL                    OPAMP_CSR_VP_SEL_Msk                      /*!< Non inverted input selection */

#define OPAMP_CSR_CALON_Pos                 (12UL)
#define OPAMP_CSR_CALON_Msk                 (0x1UL << OPAMP_CSR_CALON_Pos)            /*!< 0x00001000 */
#define OPAMP_CSR_CALON                     OPAMP_CSR_CALON_Msk                       /*!< Calibration mode enable */

#define OPAMP_CSR_CALSEL_Pos                (13UL)
#define OPAMP_CSR_CALSEL_Msk                (0x1UL << OPAMP_CSR_CALSEL_Pos)           /*!< 0x00002000 */
#define OPAMP_CSR_CALSEL                    OPAMP_CSR_CALSEL_Msk                      /*!< Calibration selection */

#define OPAMP_CSR_USERTRIM_Pos              (14UL)
#define OPAMP_CSR_USERTRIM_Msk              (0x1UL << OPAMP_CSR_USERTRIM_Pos)         /*!< 0x00004000 */
#define OPAMP_CSR_USERTRIM                  OPAMP_CSR_USERTRIM_Msk                    /*!< User trimming enable */

#define OPAMP_CSR_CALOUT_Pos                (15UL)
#define OPAMP_CSR_CALOUT_Msk                (0x1UL << OPAMP_CSR_CALOUT_Pos)           /*!< 0x00008000 */
#define OPAMP_CSR_CALOUT                    OPAMP_CSR_CALOUT_Msk                      /*!< Operational amplifier calibration output */

#define OPAMP_CSR_OPARANGE_Pos              (31UL)
#define OPAMP_CSR_OPARANGE_Msk              (0x1UL << OPAMP_CSR_OPARANGE_Pos)         /*!< 0x80000000 */
#define OPAMP_CSR_OPARANGE                  OPAMP_CSR_OPARANGE_Msk                    /*!< Operational amplifier range setting */

/*******************  Bit definition for OPAMPx_OTR register  ******************/
#define OPAMP_OTR_TRIMOFFSETN_Pos           (0UL)
#define OPAMP_OTR_TRIMOFFSETN_Msk           (0x1FUL << OPAMP_OTR_TRIMOFFSETN_Pos)     /*!< 0x0000001F */
#define OPAMP_OTR_TRIMOFFSETN               OPAMP_OTR_TRIMOFFSETN_Msk                 /*!< Trim for NMOS differential pairs */
#define OPAMP_OTR_TRIMOFFSETP_Pos           (8UL)
#define OPAMP_OTR_TRIMOFFSETP_Msk           (0x1FUL << OPAMP_OTR_TRIMOFFSETP_Pos)     /*!< 0x00001F00 */
#define OPAMP_OTR_TRIMOFFSETP               OPAMP_OTR_TRIMOFFSETP_Msk                 /*!< Trim for PMOS differential pairs */

/*******************  Bit definition for OPAMPx_LPOTR register  ****************/
#define OPAMP_LPOTR_TRIMLPOFFSETN_Pos       (0UL)
#define OPAMP_LPOTR_TRIMLPOFFSETN_Msk       (0x1FUL << OPAMP_LPOTR_TRIMLPOFFSETN_Pos) /*!< 0x0000001F */
#define OPAMP_LPOTR_TRIMLPOFFSETN           OPAMP_LPOTR_TRIMLPOFFSETN_Msk             /*!< Trim for NMOS differential pairs */
#define OPAMP_LPOTR_TRIMLPOFFSETP_Pos       (8UL)
#define OPAMP_LPOTR_TRIMLPOFFSETP_Msk       (0x1FUL << OPAMP_LPOTR_TRIMLPOFFSETP_Pos) /*!< 0x00001F00 */
#define OPAMP_LPOTR_TRIMLPOFFSETP           OPAMP_LPOTR_TRIMLPOFFSETP_Msk             /*!< Trim for PMOS differential pairs */

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for PWR_CR1 register  *******************/
#define PWR_CR1_LPMS_Pos               (0UL)
#define PWR_CR1_LPMS_Msk               (0x7UL << PWR_CR1_LPMS_Pos)             /*!< 0x00000007 */
#define PWR_CR1_LPMS                   PWR_CR1_LPMS_Msk                        /*!< LPMS[2:0] Low-power mode selection field     */
#define PWR_CR1_LPMS_0                 (0x1UL << PWR_CR1_LPMS_Pos)             /*!< 0x00000001 */
#define PWR_CR1_LPMS_1                 (0x2UL << PWR_CR1_LPMS_Pos)             /*!< 0x00000002 */
#define PWR_CR1_LPMS_2                 (0x4UL << PWR_CR1_LPMS_Pos)             /*!< 0x00000004 */

#define PWR_CR1_FPD_STOP_Pos           (3UL)
#define PWR_CR1_FPD_STOP_Msk           (0x1UL << PWR_CR1_FPD_STOP_Pos)         /*!< 0x00000008 */
#define PWR_CR1_FPD_STOP               PWR_CR1_FPD_STOP_Msk                    /*!< Flash power down mode during stop */
#define PWR_CR1_FPD_LPRUN_Pos          (4UL)
#define PWR_CR1_FPD_LPRUN_Msk          (0x1UL << PWR_CR1_FPD_LPRUN_Pos)        /*!< 0x00000010 */
#define PWR_CR1_FPD_LPRUN              PWR_CR1_FPD_LPRUN_Msk                   /*!< Flash power down mode during low power run */
#define PWR_CR1_FPD_LPSLP_Pos          (5UL)
#define PWR_CR1_FPD_LPSLP_Msk          (0x1UL << PWR_CR1_FPD_LPSLP_Pos)        /*!< 0x00000020 */
#define PWR_CR1_FPD_LPSLP              PWR_CR1_FPD_LPSLP_Msk                   /*!< Flash power down mode during low power sleep */
#define PWR_CR1_DBP_Pos                (8UL)
#define PWR_CR1_DBP_Msk                (0x1UL << PWR_CR1_DBP_Pos)              /*!< 0x00000100 */
#define PWR_CR1_DBP                    PWR_CR1_DBP_Msk                         /*!< Disable backup protection */
#define PWR_CR1_VOS_Pos                (9UL)
#define PWR_CR1_VOS_Msk                (0x3UL << PWR_CR1_VOS_Pos)              /*!< 0x00000600 */
#define PWR_CR1_VOS                    PWR_CR1_VOS_Msk                         /*!< VOS[1:0] VOltage Scaling range Selection field     */
#define PWR_CR1_VOS_0                  (0x1UL << PWR_CR1_VOS_Pos)              /*!< 0x00000200 */
#define PWR_CR1_VOS_1                  (0x2UL << PWR_CR1_VOS_Pos)              /*!< 0x00000400 */
#define PWR_CR1_LPR_Pos                (14UL)
#define PWR_CR1_LPR_Msk                (0x1UL << PWR_CR1_LPR_Pos)              /*!< 0x00004000 */
#define PWR_CR1_LPR                    PWR_CR1_LPR_Msk                         /*!< Low Power Run */

/********************  Bit definition for PWR_CR2 register  *******************/
#define PWR_CR2_PVDE_Pos               (0UL)
#define PWR_CR2_PVDE_Msk               (0x1UL << PWR_CR2_PVDE_Pos)             /*!< 0x00000001 */
#define PWR_CR2_PVDE                   PWR_CR2_PVDE_Msk                        /*!< Programmable Voltage detector enable */
#define PWR_CR2_PLS_Pos                (1UL)
#define PWR_CR2_PLS_Msk                (0x7UL << PWR_CR2_PLS_Pos)              /*!< 0x0000000E */
#define PWR_CR2_PLS                    PWR_CR2_PLS_Msk                         /*!< Power Voltage Detector Level selection */
#define PWR_CR2_PLS_0                  (0x1UL << PWR_CR2_PLS_Pos)              /*!< 0x00000002 */
#define PWR_CR2_PLS_1                  (0x2UL << PWR_CR2_PLS_Pos)              /*!< 0x00000004 */
#define PWR_CR2_PLS_2                  (0x4UL << PWR_CR2_PLS_Pos)              /*!< 0x00000008 */
#define PWR_CR2_PVME1_Pos              (4UL)
#define PWR_CR2_PVME1_Msk              (0x1UL << PWR_CR2_PVME1_Pos)            /*!< 0x00000010 */
#define PWR_CR2_PVME1                  PWR_CR2_PVME1_Msk                       /*!< Peripheral Voltage Monitoring Enable of VDDUSB */
#define PWR_CR2_PVME3_Pos              (5UL)
#define PWR_CR2_PVME3_Msk              (0x1UL << PWR_CR2_PVME3_Pos)            /*!< 0x00000020 */
#define PWR_CR2_PVME3                  PWR_CR2_PVME3_Msk                       /*!< Peripheral Voltage Monitoring Enable of VDDADC */
#define PWR_CR2_PVME4_Pos              (6UL)
#define PWR_CR2_PVME4_Msk              (0x1UL << PWR_CR2_PVME4_Pos)            /*!< 0x00000040 */
#define PWR_CR2_PVME4                  PWR_CR2_PVME4_Msk                       /*!< Peripheral Voltage Monitoring Enable of VDDDAC */
#define PWR_CR2_USV_Pos                (10UL)
#define PWR_CR2_USV_Msk                (0x1UL << PWR_CR2_USV_Pos)              /*!< 0x00000400 */
#define PWR_CR2_USV                    PWR_CR2_USV_Msk                         /*!< USB Supply Valid */

/********************  Bit definition for PWR_CR3 register  *******************/
#define PWR_CR3_EWUP1_Pos              (0UL)
#define PWR_CR3_EWUP1_Msk              (0x1UL << PWR_CR3_EWUP1_Pos)            /*!< 0x00000001 */
#define PWR_CR3_EWUP1                  PWR_CR3_EWUP1_Msk                       /*!< Enable external WKUP pin 1 */
#define PWR_CR3_EWUP2_Pos              (1UL)
#define PWR_CR3_EWUP2_Msk              (0x1UL << PWR_CR3_EWUP2_Pos)            /*!< 0x00000002 */
#define PWR_CR3_EWUP2                  PWR_CR3_EWUP2_Msk                       /*!< Enable external WKUP pin 2 */
#define PWR_CR3_EWUP3_Pos              (2UL)
#define PWR_CR3_EWUP3_Msk              (0x1UL << PWR_CR3_EWUP3_Pos)            /*!< 0x00000004 */
#define PWR_CR3_EWUP3                  PWR_CR3_EWUP3_Msk                       /*!< Enable external WKUP pin 3 */
#define PWR_CR3_EWUP4_Pos              (3UL)
#define PWR_CR3_EWUP4_Msk              (0x1UL << PWR_CR3_EWUP4_Pos)            /*!< 0x00000008 */
#define PWR_CR3_EWUP4                  PWR_CR3_EWUP4_Msk                       /*!< Enable external WKUP pin 4 */
#define PWR_CR3_EWUP5_Pos              (4UL)
#define PWR_CR3_EWUP5_Msk              (0x1UL << PWR_CR3_EWUP5_Pos)            /*!< 0x00000010 */
#define PWR_CR3_EWUP5                  PWR_CR3_EWUP5_Msk                       /*!< Enable external WKUP pin 5 */
#define PWR_CR3_EWUP7_Pos              (6UL)
#define PWR_CR3_EWUP7_Msk              (0x1UL << PWR_CR3_EWUP7_Pos)            /*!< 0x00000040 */
#define PWR_CR3_EWUP7                  PWR_CR3_EWUP7_Msk                       /*!< Enable external WKUP pin 7 */

#define PWR_CR3_RRS_Pos                (8UL)
#define PWR_CR3_RRS_Msk                (0x1UL << PWR_CR3_RRS_Pos)              /*!< 0x00000100 */
#define PWR_CR3_RRS                    PWR_CR3_RRS_Msk                         /*!< Ram retention in STANDBY mode */
#define PWR_CR3_ENULP_Pos              (9UL)
#define PWR_CR3_ENULP_Msk              (0x1UL << PWR_CR3_ENULP_Pos)            /*!< 0x00000200 */
#define PWR_CR3_ENULP                  PWR_CR3_ENULP_Msk                       /*!< Enable ULP BORL, BORH and PVD (resistor bridge sampling) for STOP2 and all Standby modes */
#define PWR_CR3_APC_Pos                (10UL)
#define PWR_CR3_APC_Msk                (0x1UL << PWR_CR3_APC_Pos)              /*!< 0x00000400 */
#define PWR_CR3_APC                    PWR_CR3_APC_Msk                         /*!< Apply pull-up and pull-down configuration */
#define PWR_CR3_EIWUL_Pos              (15UL)
#define PWR_CR3_EIWUL_Msk              (0x1UL << PWR_CR3_EIWUL_Pos)            /*!< 0x00008000 */
#define PWR_CR3_EIWUL                  PWR_CR3_EIWUL_Msk                       /*!< Enable Internal Wake-up line */

/********************  Bit definition for PWR_CR4 register  ********************/
#define PWR_CR4_WP1_Pos                (0UL)
#define PWR_CR4_WP1_Msk                (0x1UL << PWR_CR4_WP1_Pos)              /*!< 0x00000001 */
#define PWR_CR4_WP1                    PWR_CR4_WP1_Msk                         /*!< Wake-Up pin 1 polarity */
#define PWR_CR4_WP2_Pos                (1UL)
#define PWR_CR4_WP2_Msk                (0x1UL << PWR_CR4_WP2_Pos)              /*!< 0x00000002 */
#define PWR_CR4_WP2                    PWR_CR4_WP2_Msk                         /*!< Wake-Up pin 2 polarity */
#define PWR_CR4_WP3_Pos                (2UL)
#define PWR_CR4_WP3_Msk                (0x1UL << PWR_CR4_WP3_Pos)              /*!< 0x00000004 */
#define PWR_CR4_WP3                    PWR_CR4_WP3_Msk                         /*!< Wake-Up pin 3 polarity */
#define PWR_CR4_WP4_Pos                (3UL)
#define PWR_CR4_WP4_Msk                (0x1UL << PWR_CR4_WP4_Pos)              /*!< 0x00000008 */
#define PWR_CR4_WP4                    PWR_CR4_WP4_Msk                         /*!< Wake-Up pin 4 polarity */
#define PWR_CR4_WP5_Pos                (4UL)
#define PWR_CR4_WP5_Msk                (0x1UL << PWR_CR4_WP5_Pos)              /*!< 0x00000010 */
#define PWR_CR4_WP5                    PWR_CR4_WP5_Msk                         /*!< Wake-Up pin 5 polarity */
#define PWR_CR4_WP7_Pos                (6UL)
#define PWR_CR4_WP7_Msk                (0x1UL << PWR_CR4_WP7_Pos)              /*!< 0x00000040 */
#define PWR_CR4_WP7                    PWR_CR4_WP7_Msk                         /*!< Wake-Up pin 7 polarity */
#define PWR_CR4_VBE_Pos                (8UL)
#define PWR_CR4_VBE_Msk                (0x1UL << PWR_CR4_VBE_Pos)              /*!< 0x00000100 */
#define PWR_CR4_VBE                    PWR_CR4_VBE_Msk                         /*!< VBAT Battery charging Enable */
#define PWR_CR4_VBRS_Pos               (9UL)
#define PWR_CR4_VBRS_Msk               (0x1UL << PWR_CR4_VBRS_Pos)             /*!< 0x00000200 */
#define PWR_CR4_VBRS                   PWR_CR4_VBRS_Msk                        /*!< VBAT Battery charging Resistor Selection */

/********************  Bit definition for PWR_SR1 register  ********************/
#define PWR_SR1_WUF1_Pos               (0UL)
#define PWR_SR1_WUF1_Msk               (0x1UL << PWR_SR1_WUF1_Pos)             /*!< 0x00000001 */
#define PWR_SR1_WUF1                   PWR_SR1_WUF1_Msk                        /*!< Wakeup Flag 1 */
#define PWR_SR1_WUF2_Pos               (1UL)
#define PWR_SR1_WUF2_Msk               (0x1UL << PWR_SR1_WUF2_Pos)             /*!< 0x00000002 */
#define PWR_SR1_WUF2                   PWR_SR1_WUF2_Msk                        /*!< Wakeup Flag 2 */
#define PWR_SR1_WUF3_Pos               (2UL)
#define PWR_SR1_WUF3_Msk               (0x1UL << PWR_SR1_WUF3_Pos)             /*!< 0x00000004 */
#define PWR_SR1_WUF3                   PWR_SR1_WUF3_Msk                        /*!< Wakeup Flag 3 */
#define PWR_SR1_WUF4_Pos               (3UL)
#define PWR_SR1_WUF4_Msk               (0x1UL << PWR_SR1_WUF4_Pos)             /*!< 0x00000008 */
#define PWR_SR1_WUF4                   PWR_SR1_WUF4_Msk                        /*!< Wakeup Flag 4 */
#define PWR_SR1_WUF5_Pos               (4UL)
#define PWR_SR1_WUF5_Msk               (0x1UL << PWR_SR1_WUF5_Pos)             /*!< 0x00000010 */
#define PWR_SR1_WUF5                   PWR_SR1_WUF5_Msk                        /*!< Wakeup Flag 5 */
#define PWR_SR1_WUF7_Pos               (6UL)
#define PWR_SR1_WUF7_Msk               (0x1UL << PWR_SR1_WUF7_Pos)             /*!< 0x00000040 */
#define PWR_SR1_WUF7                   PWR_SR1_WUF7_Msk                        /*!< Wakeup Flag 7 */

#define PWR_SR1_SBF_Pos                (8UL)
#define PWR_SR1_SBF_Msk                (0x1UL << PWR_SR1_SBF_Pos)              /*!< 0x00000100 */
#define PWR_SR1_SBF                    PWR_SR1_SBF_Msk                         /*!< Standby Flag  */

#define PWR_SR1_STOPF_Pos              (9UL)
#define PWR_SR1_STOPF_Msk              (0x7UL << PWR_SR1_STOPF_Pos)            /*!< 0x00000E00 */
#define PWR_SR1_STOPF                  PWR_SR1_STOPF_Msk                       /*!< STOPF[2:0] Stop Flags */
#define PWR_SR1_STOPF_0                (0x1UL << PWR_SR1_STOPF_Pos)            /*!< 0x00000200 */
#define PWR_SR1_STOPF_1                (0x2UL << PWR_SR1_STOPF_Pos)            /*!< 0x00000400 */
#define PWR_SR1_STOPF_2                (0x4UL << PWR_SR1_STOPF_Pos)            /*!< 0x00000800 */
#define PWR_SR1_WUFI_Pos               (15UL)
#define PWR_SR1_WUFI_Msk               (0x1UL << PWR_SR1_WUFI_Pos)             /*!< 0x00008000 */
#define PWR_SR1_WUFI                   PWR_SR1_WUFI_Msk                        /*!< Wakeup Flag Internal */

/********************  Bit definition for PWR_SR2 register  ********************/
#define PWR_SR2_FLASH_RDY_Pos          (7UL)
#define PWR_SR2_FLASH_RDY_Msk          (0x1UL << PWR_SR2_FLASH_RDY_Pos)        /*!< 0x00000080 */
#define PWR_SR2_FLASH_RDY              PWR_SR2_FLASH_RDY_Msk                   /*!< Flash Ready */
#define PWR_SR2_REGLPS_Pos             (8UL)
#define PWR_SR2_REGLPS_Msk             (0x1UL << PWR_SR2_REGLPS_Pos)           /*!< 0x00000100 */
#define PWR_SR2_REGLPS                 PWR_SR2_REGLPS_Msk                      /*!< Regulator Low Power started    */
#define PWR_SR2_REGLPF_Pos             (9UL)
#define PWR_SR2_REGLPF_Msk             (0x1UL << PWR_SR2_REGLPF_Pos)           /*!< 0x00000200 */
#define PWR_SR2_REGLPF                 PWR_SR2_REGLPF_Msk                      /*!< Regulator Low Power flag    */
#define PWR_SR2_VOSF_Pos               (10UL)
#define PWR_SR2_VOSF_Msk               (0x1UL << PWR_SR2_VOSF_Pos)             /*!< 0x00000400 */
#define PWR_SR2_VOSF                   PWR_SR2_VOSF_Msk                        /*!< VOltage Scaling Flag    */
#define PWR_SR2_PVDO_Pos               (11UL)
#define PWR_SR2_PVDO_Msk               (0x1UL << PWR_SR2_PVDO_Pos)             /*!< 0x00000800 */
#define PWR_SR2_PVDO                   PWR_SR2_PVDO_Msk                        /*!< Power Voltage Detector Output    */
#define PWR_SR2_PVMO1_Pos              (12UL)
#define PWR_SR2_PVMO1_Msk              (0x1UL << PWR_SR2_PVMO1_Pos)            /*!< 0x00001000 */
#define PWR_SR2_PVMO1                  PWR_SR2_PVMO1_Msk                       /*!< PVMO USB & PVME USB    */
#define PWR_SR2_PVMO3_Pos              (14UL)
#define PWR_SR2_PVMO3_Msk              (0x1UL << PWR_SR2_PVMO3_Pos)            /*!< 0x00004000 */
#define PWR_SR2_PVMO3                  PWR_SR2_PVMO3_Msk                       /*!< PVMO ADC & PVME ADC    */
#define PWR_SR2_PVMO4_Pos              (15UL)
#define PWR_SR2_PVMO4_Msk              (0x1UL << PWR_SR2_PVMO4_Pos)            /*!< 0x00008000 */
#define PWR_SR2_PVMO4                  PWR_SR2_PVMO4_Msk                       /*!< PVMO DAC & PVME DAC    */

/********************  Bit definition for PWR_SCR register  ********************/
#define PWR_SCR_CWUF1_Pos              (0UL)
#define PWR_SCR_CWUF1_Msk              (0x1UL << PWR_SCR_CWUF1_Pos)            /*!< 0x00000001 */
#define PWR_SCR_CWUF1                  PWR_SCR_CWUF1_Msk                       /*!< Clear Wake-up Flag 1 */
#define PWR_SCR_CWUF2_Pos              (1UL)
#define PWR_SCR_CWUF2_Msk              (0x1UL << PWR_SCR_CWUF2_Pos)            /*!< 0x00000002 */
#define PWR_SCR_CWUF2                  PWR_SCR_CWUF2_Msk                       /*!< Clear Wake-up Flag 2 */
#define PWR_SCR_CWUF3_Pos              (2UL)
#define PWR_SCR_CWUF3_Msk              (0x1UL << PWR_SCR_CWUF3_Pos)            /*!< 0x00000004 */
#define PWR_SCR_CWUF3                  PWR_SCR_CWUF3_Msk                       /*!< Clear Wake-up Flag 3 */
#define PWR_SCR_CWUF4_Pos              (3UL)
#define PWR_SCR_CWUF4_Msk              (0x1UL << PWR_SCR_CWUF4_Pos)            /*!< 0x00000008 */
#define PWR_SCR_CWUF4                  PWR_SCR_CWUF4_Msk                       /*!< Clear Wake-up Flag 4 */
#define PWR_SCR_CWUF5_Pos              (4UL)
#define PWR_SCR_CWUF5_Msk              (0x1UL << PWR_SCR_CWUF5_Pos)            /*!< 0x00000010 */
#define PWR_SCR_CWUF5                  PWR_SCR_CWUF5_Msk                       /*!< Clear Wake-up Flag 5 */
#define PWR_SCR_CWUF7_Pos              (6UL)
#define PWR_SCR_CWUF7_Msk              (0x1UL << PWR_SCR_CWUF7_Pos)            /*!< 0x00000040 */
#define PWR_SCR_CWUF7                  PWR_SCR_CWUF7_Msk                       /*!< Clear Wake-up Flag 7 */
#define PWR_SCR_CSBF_Pos               (8UL)
#define PWR_SCR_CSBF_Msk               (0x1UL << PWR_SCR_CSBF_Pos)             /*!< 0x00000100 */
#define PWR_SCR_CSBF                   PWR_SCR_CSBF_Msk                        /*!< Clear Standby Flag  */

/********************  Bit definition for PWR_PUCRA register  *****************/
#define PWR_PUCRA_PU0_Pos         (0UL)
#define PWR_PUCRA_PU0_Msk         (0x1UL << PWR_PUCRA_PU0_Pos)                 /*!< 0x00000001 */
#define PWR_PUCRA_PU0             PWR_PUCRA_PU0_Msk                            /*!< Pin PA0 Pull-Up set */
#define PWR_PUCRA_PU1_Pos         (1UL)
#define PWR_PUCRA_PU1_Msk         (0x1UL << PWR_PUCRA_PU1_Pos)                 /*!< 0x00000002 */
#define PWR_PUCRA_PU1             PWR_PUCRA_PU1_Msk                            /*!< Pin PA1 Pull-Up set */
#define PWR_PUCRA_PU2_Pos         (2UL)
#define PWR_PUCRA_PU2_Msk         (0x1UL << PWR_PUCRA_PU2_Pos)                 /*!< 0x00000004 */
#define PWR_PUCRA_PU2             PWR_PUCRA_PU2_Msk                            /*!< Pin PA2 Pull-Up set */
#define PWR_PUCRA_PU3_Pos         (3UL)
#define PWR_PUCRA_PU3_Msk         (0x1UL << PWR_PUCRA_PU3_Pos)                 /*!< 0x00000008 */
#define PWR_PUCRA_PU3             PWR_PUCRA_PU3_Msk                            /*!< Pin PA3 Pull-Up set */
#define PWR_PUCRA_PU4_Pos         (4UL)
#define PWR_PUCRA_PU4_Msk         (0x1UL << PWR_PUCRA_PU4_Pos)                 /*!< 0x00000010 */
#define PWR_PUCRA_PU4             PWR_PUCRA_PU4_Msk                            /*!< Pin PA4 Pull-Up set */
#define PWR_PUCRA_PU5_Pos         (5UL)
#define PWR_PUCRA_PU5_Msk         (0x1UL << PWR_PUCRA_PU5_Pos)                 /*!< 0x00000020 */
#define PWR_PUCRA_PU5             PWR_PUCRA_PU5_Msk                            /*!< Pin PA5 Pull-Up set */
#define PWR_PUCRA_PU6_Pos         (6UL)
#define PWR_PUCRA_PU6_Msk         (0x1UL << PWR_PUCRA_PU6_Pos)                 /*!< 0x00000040 */
#define PWR_PUCRA_PU6             PWR_PUCRA_PU6_Msk                            /*!< Pin PA6 Pull-Up set */
#define PWR_PUCRA_PU7_Pos         (7UL)
#define PWR_PUCRA_PU7_Msk         (0x1UL << PWR_PUCRA_PU7_Pos)                 /*!< 0x00000080 */
#define PWR_PUCRA_PU7             PWR_PUCRA_PU7_Msk                            /*!< Pin PA7 Pull-Up set */
#define PWR_PUCRA_PU8_Pos         (8UL)
#define PWR_PUCRA_PU8_Msk         (0x1UL << PWR_PUCRA_PU8_Pos)                 /*!< 0x00000100 */
#define PWR_PUCRA_PU8             PWR_PUCRA_PU8_Msk                            /*!< Pin PA8 Pull-Up set */
#define PWR_PUCRA_PU9_Pos         (9UL)
#define PWR_PUCRA_PU9_Msk         (0x1UL << PWR_PUCRA_PU9_Pos)                 /*!< 0x00000200 */
#define PWR_PUCRA_PU9             PWR_PUCRA_PU9_Msk                            /*!< Pin PA9 Pull-Up set */
#define PWR_PUCRA_PU10_Pos        (10UL)
#define PWR_PUCRA_PU10_Msk        (0x1UL << PWR_PUCRA_PU10_Pos)                /*!< 0x00000400 */
#define PWR_PUCRA_PU10            PWR_PUCRA_PU10_Msk                           /*!< Pin PA10 Pull-Up set */
#define PWR_PUCRA_PU11_Pos        (11UL)
#define PWR_PUCRA_PU11_Msk        (0x1UL << PWR_PUCRA_PU11_Pos)                /*!< 0x00000800 */
#define PWR_PUCRA_PU11            PWR_PUCRA_PU11_Msk                           /*!< Pin PA11 Pull-Up set */
#define PWR_PUCRA_PU12_Pos        (12UL)
#define PWR_PUCRA_PU12_Msk        (0x1UL << PWR_PUCRA_PU12_Pos)                /*!< 0x00001000 */
#define PWR_PUCRA_PU12            PWR_PUCRA_PU12_Msk                           /*!< Pin PA12 Pull-Up set */
#define PWR_PUCRA_PU13_Pos        (13UL)
#define PWR_PUCRA_PU13_Msk        (0x1UL << PWR_PUCRA_PU13_Pos)                /*!< 0x00002000 */
#define PWR_PUCRA_PU13            PWR_PUCRA_PU13_Msk                           /*!< Pin PA13 Pull-Up set */
#define PWR_PUCRA_PU14_Pos        (14UL)
#define PWR_PUCRA_PU14_Msk        (0x1UL << PWR_PUCRA_PU14_Pos)                /*!< 0x00004000 */
#define PWR_PUCRA_PU14            PWR_PUCRA_PU14_Msk                           /*!< Pin PA14 Pull-Up set */
#define PWR_PUCRA_PU15_Pos        (15UL)
#define PWR_PUCRA_PU15_Msk        (0x1UL << PWR_PUCRA_PU15_Pos)                /*!< 0x00008000 */
#define PWR_PUCRA_PU15            PWR_PUCRA_PU15_Msk                           /*!< Pin PA15 Pull-Up set */

/********************  Bit definition for PWR_PDCRA register  *****************/
#define PWR_PDCRA_PD0_Pos         (0UL)
#define PWR_PDCRA_PD0_Msk         (0x1UL << PWR_PDCRA_PD0_Pos)                 /*!< 0x00000001 */
#define PWR_PDCRA_PD0             PWR_PDCRA_PD0_Msk                            /*!< Pin PA0 Pull-Down set */
#define PWR_PDCRA_PD1_Pos         (1UL)
#define PWR_PDCRA_PD1_Msk         (0x1UL << PWR_PDCRA_PD1_Pos)                 /*!< 0x00000002 */
#define PWR_PDCRA_PD1             PWR_PDCRA_PD1_Msk                            /*!< Pin PA1 Pull-Down set */
#define PWR_PDCRA_PD2_Pos         (2UL)
#define PWR_PDCRA_PD2_Msk         (0x1UL << PWR_PDCRA_PD2_Pos)                 /*!< 0x00000004 */
#define PWR_PDCRA_PD2             PWR_PDCRA_PD2_Msk                            /*!< Pin PA2 Pull-Down set */
#define PWR_PDCRA_PD3_Pos         (3UL)
#define PWR_PDCRA_PD3_Msk         (0x1UL << PWR_PDCRA_PD3_Pos)                 /*!< 0x00000008 */
#define PWR_PDCRA_PD3             PWR_PDCRA_PD3_Msk                            /*!< Pin PA3 Pull-Down set */
#define PWR_PDCRA_PD4_Pos         (4UL)
#define PWR_PDCRA_PD4_Msk         (0x1UL << PWR_PDCRA_PD4_Pos)                 /*!< 0x00000010 */
#define PWR_PDCRA_PD4             PWR_PDCRA_PD4_Msk                            /*!< Pin PA4 Pull-Down set */
#define PWR_PDCRA_PD5_Pos         (5UL)
#define PWR_PDCRA_PD5_Msk         (0x1UL << PWR_PDCRA_PD5_Pos)                 /*!< 0x00000020 */
#define PWR_PDCRA_PD5             PWR_PDCRA_PD5_Msk                            /*!< Pin PA5 Pull-Down set */
#define PWR_PDCRA_PD6_Pos         (6UL)
#define PWR_PDCRA_PD6_Msk         (0x1UL << PWR_PDCRA_PD6_Pos)                 /*!< 0x00000040 */
#define PWR_PDCRA_PD6             PWR_PDCRA_PD6_Msk                            /*!< Pin PA6 Pull-Down set */
#define PWR_PDCRA_PD7_Pos         (7UL)
#define PWR_PDCRA_PD7_Msk         (0x1UL << PWR_PDCRA_PD7_Pos)                 /*!< 0x00000080 */
#define PWR_PDCRA_PD7             PWR_PDCRA_PD7_Msk                            /*!< Pin PA7 Pull-Down set */
#define PWR_PDCRA_PD8_Pos         (8UL)
#define PWR_PDCRA_PD8_Msk         (0x1UL << PWR_PDCRA_PD8_Pos)                 /*!< 0x00000100 */
#define PWR_PDCRA_PD8             PWR_PDCRA_PD8_Msk                            /*!< Pin PA8 Pull-Down set */
#define PWR_PDCRA_PD9_Pos         (9UL)
#define PWR_PDCRA_PD9_Msk         (0x1UL << PWR_PDCRA_PD9_Pos)                 /*!< 0x00000200 */
#define PWR_PDCRA_PD9             PWR_PDCRA_PD9_Msk                            /*!< Pin PA9 Pull-Down set */
#define PWR_PDCRA_PD10_Pos        (10UL)
#define PWR_PDCRA_PD10_Msk        (0x1UL << PWR_PDCRA_PD10_Pos)                /*!< 0x00000400 */
#define PWR_PDCRA_PD10            PWR_PDCRA_PD10_Msk                           /*!< Pin PA10 Pull-Down set */
#define PWR_PDCRA_PD11_Pos        (11UL)
#define PWR_PDCRA_PD11_Msk        (0x1UL << PWR_PDCRA_PD11_Pos)                /*!< 0x00000800 */
#define PWR_PDCRA_PD11            PWR_PDCRA_PD11_Msk                           /*!< Pin PA11 Pull-Down set */
#define PWR_PDCRA_PD12_Pos        (12UL)
#define PWR_PDCRA_PD12_Msk        (0x1UL << PWR_PDCRA_PD12_Pos)                /*!< 0x00001000 */
#define PWR_PDCRA_PD12            PWR_PDCRA_PD12_Msk                           /*!< Pin PA12 Pull-Down set */
#define PWR_PDCRA_PD13_Pos        (13UL)
#define PWR_PDCRA_PD13_Msk        (0x1UL << PWR_PDCRA_PD13_Pos)                /*!< 0x00002000 */
#define PWR_PDCRA_PD13            PWR_PDCRA_PD13_Msk                           /*!< Pin PA13 Pull-Down set */
#define PWR_PDCRA_PD14_Pos        (14UL)
#define PWR_PDCRA_PD14_Msk        (0x1UL << PWR_PDCRA_PD14_Pos)                /*!< 0x00004000 */
#define PWR_PDCRA_PD14            PWR_PDCRA_PD14_Msk                           /*!< Pin PA14 Pull-Down set */
#define PWR_PDCRA_PD15_Pos        (15UL)
#define PWR_PDCRA_PD15_Msk        (0x1UL << PWR_PDCRA_PD15_Pos)                /*!< 0x00008000 */
#define PWR_PDCRA_PD15            PWR_PDCRA_PD15_Msk                           /*!< Pin PA15 Pull-Down set */

/********************  Bit definition for PWR_PUCRB register  *****************/
#define PWR_PUCRB_PU0_Pos         (0UL)
#define PWR_PUCRB_PU0_Msk         (0x1UL << PWR_PUCRB_PU0_Pos)                 /*!< 0x00000001 */
#define PWR_PUCRB_PU0             PWR_PUCRB_PU0_Msk                            /*!< Pin PB0 Pull-Up set */
#define PWR_PUCRB_PU1_Pos         (1UL)
#define PWR_PUCRB_PU1_Msk         (0x1UL << PWR_PUCRB_PU1_Pos)                 /*!< 0x00000002 */
#define PWR_PUCRB_PU1             PWR_PUCRB_PU1_Msk                            /*!< Pin PB1 Pull-Up set */
#define PWR_PUCRB_PU2_Pos         (2UL)
#define PWR_PUCRB_PU2_Msk         (0x1UL << PWR_PUCRB_PU2_Pos)                 /*!< 0x00000004 */
#define PWR_PUCRB_PU2             PWR_PUCRB_PU2_Msk                            /*!< Pin PB2 Pull-Up set */
#define PWR_PUCRB_PU3_Pos         (3UL)
#define PWR_PUCRB_PU3_Msk         (0x1UL << PWR_PUCRB_PU3_Pos)                 /*!< 0x00000008 */
#define PWR_PUCRB_PU3             PWR_PUCRB_PU3_Msk                            /*!< Pin PB3 Pull-Up set */
#define PWR_PUCRB_PU4_Pos         (4UL)
#define PWR_PUCRB_PU4_Msk         (0x1UL << PWR_PUCRB_PU4_Pos)                 /*!< 0x00000010 */
#define PWR_PUCRB_PU4             PWR_PUCRB_PU4_Msk                            /*!< Pin PB4 Pull-Up set */
#define PWR_PUCRB_PU5_Pos         (5UL)
#define PWR_PUCRB_PU5_Msk         (0x1UL << PWR_PUCRB_PU5_Pos)                 /*!< 0x00000020 */
#define PWR_PUCRB_PU5             PWR_PUCRB_PU5_Msk                            /*!< Pin PB5 Pull-Up set */
#define PWR_PUCRB_PU6_Pos         (6UL)
#define PWR_PUCRB_PU6_Msk         (0x1UL << PWR_PUCRB_PU6_Pos)                 /*!< 0x00000040 */
#define PWR_PUCRB_PU6             PWR_PUCRB_PU6_Msk                            /*!< Pin PB6 Pull-Up set */
#define PWR_PUCRB_PU7_Pos         (7UL)
#define PWR_PUCRB_PU7_Msk         (0x1UL << PWR_PUCRB_PU7_Pos)                 /*!< 0x00000080 */
#define PWR_PUCRB_PU7             PWR_PUCRB_PU7_Msk                            /*!< Pin PB7 Pull-Up set */
#define PWR_PUCRB_PU8_Pos         (8UL)
#define PWR_PUCRB_PU8_Msk         (0x1UL << PWR_PUCRB_PU8_Pos)                 /*!< 0x00000100 */
#define PWR_PUCRB_PU8             PWR_PUCRB_PU8_Msk                            /*!< Pin PB8 Pull-Up set */
#define PWR_PUCRB_PU9_Pos         (9UL)
#define PWR_PUCRB_PU9_Msk         (0x1UL << PWR_PUCRB_PU9_Pos)                 /*!< 0x00000200 */
#define PWR_PUCRB_PU9             PWR_PUCRB_PU9_Msk                            /*!< Pin PB9 Pull-Up set */
#define PWR_PUCRB_PU10_Pos        (10UL)
#define PWR_PUCRB_PU10_Msk        (0x1UL << PWR_PUCRB_PU10_Pos)                /*!< 0x00000400 */
#define PWR_PUCRB_PU10            PWR_PUCRB_PU10_Msk                           /*!< Pin PB10 Pull-Up set */
#define PWR_PUCRB_PU11_Pos        (11UL)
#define PWR_PUCRB_PU11_Msk        (0x1UL << PWR_PUCRB_PU11_Pos)                /*!< 0x00000800 */
#define PWR_PUCRB_PU11            PWR_PUCRB_PU11_Msk                           /*!< Pin PB11 Pull-Up set */
#define PWR_PUCRB_PU12_Pos        (12UL)
#define PWR_PUCRB_PU12_Msk        (0x1UL << PWR_PUCRB_PU12_Pos)                /*!< 0x00001000 */
#define PWR_PUCRB_PU12            PWR_PUCRB_PU12_Msk                           /*!< Pin PB12 Pull-Up set */
#define PWR_PUCRB_PU13_Pos        (13UL)
#define PWR_PUCRB_PU13_Msk        (0x1UL << PWR_PUCRB_PU13_Pos)                /*!< 0x00002000 */
#define PWR_PUCRB_PU13            PWR_PUCRB_PU13_Msk                           /*!< Pin PB13 Pull-Up set */
#define PWR_PUCRB_PU14_Pos        (14UL)
#define PWR_PUCRB_PU14_Msk        (0x1UL << PWR_PUCRB_PU14_Pos)                /*!< 0x00004000 */
#define PWR_PUCRB_PU14            PWR_PUCRB_PU14_Msk                           /*!< Pin PB14 Pull-Up set */
#define PWR_PUCRB_PU15_Pos        (15UL)
#define PWR_PUCRB_PU15_Msk        (0x1UL << PWR_PUCRB_PU15_Pos)                /*!< 0x00008000 */
#define PWR_PUCRB_PU15            PWR_PUCRB_PU15_Msk                           /*!< Pin PB15 Pull-Up set */

/********************  Bit definition for PWR_PDCRB register  *****************/
#define PWR_PDCRB_PD0_Pos         (0UL)
#define PWR_PDCRB_PD0_Msk         (0x1UL << PWR_PDCRB_PD0_Pos)                 /*!< 0x00000001 */
#define PWR_PDCRB_PD0             PWR_PDCRB_PD0_Msk                            /*!< Pin PB0 Pull-Down set */
#define PWR_PDCRB_PD1_Pos         (1UL)
#define PWR_PDCRB_PD1_Msk         (0x1UL << PWR_PDCRB_PD1_Pos)                 /*!< 0x00000002 */
#define PWR_PDCRB_PD1             PWR_PDCRB_PD1_Msk                            /*!< Pin PB1 Pull-Down set */
#define PWR_PDCRB_PD2_Pos         (2UL)
#define PWR_PDCRB_PD2_Msk         (0x1UL << PWR_PDCRB_PD2_Pos)                 /*!< 0x00000004 */
#define PWR_PDCRB_PD2             PWR_PDCRB_PD2_Msk                            /*!< Pin PB2 Pull-Down set */
#define PWR_PDCRB_PD3_Pos         (3UL)
#define PWR_PDCRB_PD3_Msk         (0x1UL << PWR_PDCRB_PD3_Pos)                 /*!< 0x00000008 */
#define PWR_PDCRB_PD3             PWR_PDCRB_PD3_Msk                            /*!< Pin PB3 Pull-Down set */
#define PWR_PDCRB_PD4_Pos         (4UL)
#define PWR_PDCRB_PD4_Msk         (0x1UL << PWR_PDCRB_PD4_Pos)                 /*!< 0x00000010 */
#define PWR_PDCRB_PD4             PWR_PDCRB_PD4_Msk                            /*!< Pin PB4 Pull-Down set */
#define PWR_PDCRB_PD5_Pos         (5UL)
#define PWR_PDCRB_PD5_Msk         (0x1UL << PWR_PDCRB_PD5_Pos)                 /*!< 0x00000020 */
#define PWR_PDCRB_PD5             PWR_PDCRB_PD5_Msk                            /*!< Pin PB5 Pull-Down set */
#define PWR_PDCRB_PD6_Pos         (6UL)
#define PWR_PDCRB_PD6_Msk         (0x1UL << PWR_PDCRB_PD6_Pos)                 /*!< 0x00000040 */
#define PWR_PDCRB_PD6             PWR_PDCRB_PD6_Msk                            /*!< Pin PB6 Pull-Down set */
#define PWR_PDCRB_PD7_Pos         (7UL)
#define PWR_PDCRB_PD7_Msk         (0x1UL << PWR_PDCRB_PD7_Pos)                 /*!< 0x00000080 */
#define PWR_PDCRB_PD7             PWR_PDCRB_PD7_Msk                            /*!< Pin PB7 Pull-Down set */
#define PWR_PDCRB_PD8_Pos         (8UL)
#define PWR_PDCRB_PD8_Msk         (0x1UL << PWR_PDCRB_PD8_Pos)                 /*!< 0x00000100 */
#define PWR_PDCRB_PD8             PWR_PDCRB_PD8_Msk                            /*!< Pin PB8 Pull-Down set */
#define PWR_PDCRB_PD9_Pos         (9UL)
#define PWR_PDCRB_PD9_Msk         (0x1UL << PWR_PDCRB_PD9_Pos)                 /*!< 0x00000200 */
#define PWR_PDCRB_PD9             PWR_PDCRB_PD9_Msk                            /*!< Pin PB9 Pull-Down set */
#define PWR_PDCRB_PD10_Pos        (10UL)
#define PWR_PDCRB_PD10_Msk        (0x1UL << PWR_PDCRB_PD10_Pos)                /*!< 0x00000400 */
#define PWR_PDCRB_PD10            PWR_PDCRB_PD10_Msk                           /*!< Pin PB10 Pull-Down set */
#define PWR_PDCRB_PD11_Pos        (11UL)
#define PWR_PDCRB_PD11_Msk        (0x1UL << PWR_PDCRB_PD11_Pos)                /*!< 0x00000800 */
#define PWR_PDCRB_PD11            PWR_PDCRB_PD11_Msk                           /*!< Pin PB11 Pull-Down set */
#define PWR_PDCRB_PD12_Pos        (12UL)
#define PWR_PDCRB_PD12_Msk        (0x1UL << PWR_PDCRB_PD12_Pos)                /*!< 0x00001000 */
#define PWR_PDCRB_PD12            PWR_PDCRB_PD12_Msk                           /*!< Pin PB12 Pull-Down set */
#define PWR_PDCRB_PD13_Pos        (13UL)
#define PWR_PDCRB_PD13_Msk        (0x1UL << PWR_PDCRB_PD13_Pos)                /*!< 0x00002000 */
#define PWR_PDCRB_PD13            PWR_PDCRB_PD13_Msk                           /*!< Pin PB13 Pull-Down set */
#define PWR_PDCRB_PD14_Pos        (14UL)
#define PWR_PDCRB_PD14_Msk        (0x1UL << PWR_PDCRB_PD14_Pos)                /*!< 0x00004000 */
#define PWR_PDCRB_PD14            PWR_PDCRB_PD14_Msk                           /*!< Pin PB14 Pull-Down set */
#define PWR_PDCRB_PD15_Pos        (15UL)
#define PWR_PDCRB_PD15_Msk        (0x1UL << PWR_PDCRB_PD15_Pos)                /*!< 0x00008000 */
#define PWR_PDCRB_PD15            PWR_PDCRB_PD15_Msk                           /*!< Pin PB15 Pull-Down set */

/********************  Bit definition for PWR_PUCRC register  *****************/
#define PWR_PUCRC_PU0_Pos         (0UL)
#define PWR_PUCRC_PU0_Msk         (0x1UL << PWR_PUCRC_PU0_Pos)                 /*!< 0x00000001 */
#define PWR_PUCRC_PU0             PWR_PUCRC_PU0_Msk                            /*!< Pin PC0 Pull-Up set */
#define PWR_PUCRC_PU1_Pos         (1UL)
#define PWR_PUCRC_PU1_Msk         (0x1UL << PWR_PUCRC_PU1_Pos)                 /*!< 0x00000002 */
#define PWR_PUCRC_PU1             PWR_PUCRC_PU1_Msk                            /*!< Pin PC1 Pull-Up set */
#define PWR_PUCRC_PU2_Pos         (2UL)
#define PWR_PUCRC_PU2_Msk         (0x1UL << PWR_PUCRC_PU2_Pos)                 /*!< 0x00000004 */
#define PWR_PUCRC_PU2             PWR_PUCRC_PU2_Msk                            /*!< Pin PC2 Pull-Up set */
#define PWR_PUCRC_PU3_Pos         (3UL)
#define PWR_PUCRC_PU3_Msk         (0x1UL << PWR_PUCRC_PU3_Pos)                 /*!< 0x00000008 */
#define PWR_PUCRC_PU3             PWR_PUCRC_PU3_Msk                            /*!< Pin PC3 Pull-Up set */
#define PWR_PUCRC_PU4_Pos         (4UL)
#define PWR_PUCRC_PU4_Msk         (0x1UL << PWR_PUCRC_PU4_Pos)                 /*!< 0x00000010 */
#define PWR_PUCRC_PU4             PWR_PUCRB_PD4_Msk                            /*!< Pin PC4 Pull-Up set */
#define PWR_PUCRC_PU5_Pos         (5UL)
#define PWR_PUCRC_PU5_Msk         (0x1UL << PWR_PUCRC_PU5_Pos)                 /*!< 0x00000020 */
#define PWR_PUCRC_PU5             PWR_PUCRC_PU5_Msk                            /*!< Pin PC5 Pull-Up set */
#define PWR_PUCRC_PU6_Pos         (6UL)
#define PWR_PUCRC_PU6_Msk         (0x1UL << PWR_PUCRC_PU6_Pos)                 /*!< 0x00000040 */
#define PWR_PUCRC_PU6             PWR_PUCRC_PU6_Msk                            /*!< Pin PC6 Pull-Up set */
#define PWR_PUCRC_PU7_Pos         (7UL)
#define PWR_PUCRC_PU7_Msk         (0x1UL << PWR_PUCRC_PU7_Pos)                 /*!< 0x00000080 */
#define PWR_PUCRC_PU7             PWR_PUCRC_PU7_Msk                            /*!< Pin PC7 Pull-Up set */
#define PWR_PUCRC_PU8_Pos         (8UL)
#define PWR_PUCRC_PU8_Msk         (0x1UL << PWR_PUCRC_PU8_Pos)                 /*!< 0x00000100 */
#define PWR_PUCRC_PU8             PWR_PUCRC_PU8_Msk                            /*!< Pin PC8 Pull-Up set */
#define PWR_PUCRC_PU9_Pos         (9UL)
#define PWR_PUCRC_PU9_Msk         (0x1UL << PWR_PUCRC_PU9_Pos)                 /*!< 0x00000200 */
#define PWR_PUCRC_PU9             PWR_PUCRC_PU9_Msk                            /*!< Pin PC9 Pull-Up set */
#define PWR_PUCRC_PU10_Pos        (10UL)
#define PWR_PUCRC_PU10_Msk        (0x1UL << PWR_PUCRC_PU10_Pos)                /*!< 0x00000400 */
#define PWR_PUCRC_PU10            PWR_PUCRC_PU10_Msk                           /*!< Pin PC10 Pull-Up set */
#define PWR_PUCRC_PU11_Pos        (11UL)
#define PWR_PUCRC_PU11_Msk        (0x1UL << PWR_PUCRC_PU11_Pos)                /*!< 0x00000800 */
#define PWR_PUCRC_PU11            PWR_PUCRC_PU11_Msk                           /*!< Pin PC11 Pull-Up set */
#define PWR_PUCRC_PU12_Pos        (12UL)
#define PWR_PUCRC_PU12_Msk        (0x1UL << PWR_PUCRC_PU12_Pos)                /*!< 0x00001000 */
#define PWR_PUCRC_PU12            PWR_PUCRC_PU12_Msk                           /*!< Pin PC12 Pull-Up set */
#define PWR_PUCRC_PU13_Pos        (13UL)
#define PWR_PUCRC_PU13_Msk        (0x1UL << PWR_PUCRC_PU13_Pos)                /*!< 0x00002000 */
#define PWR_PUCRC_PU13            PWR_PUCRC_PU13_Msk                           /*!< Pin PC13 Pull-Up set */
#define PWR_PUCRC_PU14_Pos        (14UL)
#define PWR_PUCRC_PU14_Msk        (0x1UL << PWR_PUCRC_PU14_Pos)                /*!< 0x00004000 */
#define PWR_PUCRC_PU14            PWR_PUCRC_PU14_Msk                           /*!< Pin PC14 Pull-Up set */
#define PWR_PUCRC_PU15_Pos        (15UL)
#define PWR_PUCRC_PU15_Msk        (0x1UL << PWR_PUCRC_PU15_Pos)                /*!< 0x00008000 */
#define PWR_PUCRC_PU15            PWR_PUCRC_PU15_Msk                           /*!< Pin PC15 Pull-Up set */

/********************  Bit definition for PWR_PDCRC register  *****************/
#define PWR_PDCRC_PD0_Pos         (0UL)
#define PWR_PDCRC_PD0_Msk         (0x1UL << PWR_PDCRC_PD0_Pos)                 /*!< 0x00000001 */
#define PWR_PDCRC_PD0             PWR_PDCRC_PD0_Msk                            /*!< Pin PC0 Pull-Down set */
#define PWR_PDCRC_PD1_Pos         (1UL)
#define PWR_PDCRC_PD1_Msk         (0x1UL << PWR_PDCRC_PD1_Pos)                 /*!< 0x00000002 */
#define PWR_PDCRC_PD1             PWR_PDCRC_PD1_Msk                            /*!< Pin PC1 Pull-Down set */
#define PWR_PDCRC_PD2_Pos         (2UL)
#define PWR_PDCRC_PD2_Msk         (0x1UL << PWR_PDCRC_PD2_Pos)                 /*!< 0x00000004 */
#define PWR_PDCRC_PD2             PWR_PDCRC_PD2_Msk                            /*!< Pin PC2 Pull-Down set */
#define PWR_PDCRC_PD3_Pos         (3UL)
#define PWR_PDCRC_PD3_Msk         (0x1UL << PWR_PDCRC_PD3_Pos)                 /*!< 0x00000008 */
#define PWR_PDCRC_PD3             PWR_PDCRC_PD3_Msk                            /*!< Pin PC3 Pull-Down set */
#define PWR_PDCRC_PD4_Pos         (4UL)
#define PWR_PDCRC_PD4_Msk         (0x1UL << PWR_PDCRC_PD4_Pos)                 /*!< 0x00000010 */
#define PWR_PDCRC_PD4             PWR_PUCRB_PD4_Msk                            /*!< Pin PC4 Pull-Down set */
#define PWR_PDCRC_PD5_Pos         (5UL)
#define PWR_PDCRC_PD5_Msk         (0x1UL << PWR_PDCRC_PD5_Pos)                 /*!< 0x00000020 */
#define PWR_PDCRC_PD5             PWR_PDCRC_PD5_Msk                            /*!< Pin PC5 Pull-Down set */
#define PWR_PDCRC_PD6_Pos         (6UL)
#define PWR_PDCRC_PD6_Msk         (0x1UL << PWR_PDCRC_PD6_Pos)                 /*!< 0x00000040 */
#define PWR_PDCRC_PD6             PWR_PDCRC_PD6_Msk                            /*!< Pin PC6 Pull-Down set */
#define PWR_PDCRC_PD7_Pos         (7UL)
#define PWR_PDCRC_PD7_Msk         (0x1UL << PWR_PDCRC_PD7_Pos)                 /*!< 0x00000080 */
#define PWR_PDCRC_PD7             PWR_PDCRC_PD7_Msk                            /*!< Pin PC7 Pull-Down set */
#define PWR_PDCRC_PD8_Pos         (8UL)
#define PWR_PDCRC_PD8_Msk         (0x1UL << PWR_PDCRC_PD8_Pos)                 /*!< 0x00000100 */
#define PWR_PDCRC_PD8             PWR_PDCRC_PD8_Msk                            /*!< Pin PC8 Pull-Down set */
#define PWR_PDCRC_PD9_Pos         (9UL)
#define PWR_PDCRC_PD9_Msk         (0x1UL << PWR_PDCRC_PD9_Pos)                 /*!< 0x00000200 */
#define PWR_PDCRC_PD9             PWR_PDCRC_PD9_Msk                            /*!< Pin PC9 Pull-Down set */
#define PWR_PDCRC_PD10_Pos        (10UL)
#define PWR_PDCRC_PD10_Msk        (0x1UL << PWR_PDCRC_PD10_Pos)                /*!< 0x00000400 */
#define PWR_PDCRC_PD10            PWR_PDCRC_PD10_Msk                           /*!< Pin PC10 Pull-Down set */
#define PWR_PDCRC_PD11_Pos        (11UL)
#define PWR_PDCRC_PD11_Msk        (0x1UL << PWR_PDCRC_PD11_Pos)                /*!< 0x00000800 */
#define PWR_PDCRC_PD11            PWR_PDCRC_PD11_Msk                           /*!< Pin PC11 Pull-Down set */
#define PWR_PDCRC_PD12_Pos        (12UL)
#define PWR_PDCRC_PD12_Msk        (0x1UL << PWR_PDCRC_PD12_Pos)                /*!< 0x00001000 */
#define PWR_PDCRC_PD12            PWR_PDCRC_PD12_Msk                           /*!< Pin PC12 Pull-Down set */
#define PWR_PDCRC_PD13_Pos        (13UL)
#define PWR_PDCRC_PD13_Msk        (0x1UL << PWR_PDCRC_PD13_Pos)                /*!< 0x00002000 */
#define PWR_PDCRC_PD13            PWR_PDCRC_PD13_Msk                           /*!< Pin PC13 Pull-Down set */
#define PWR_PDCRC_PD14_Pos        (14UL)
#define PWR_PDCRC_PD14_Msk        (0x1UL << PWR_PDCRC_PD14_Pos)                /*!< 0x00004000 */
#define PWR_PDCRC_PD14            PWR_PDCRC_PD14_Msk                           /*!< Pin PC14 Pull-Down set */
#define PWR_PDCRC_PD15_Pos        (15UL)
#define PWR_PDCRC_PD15_Msk        (0x1UL << PWR_PDCRC_PD15_Pos)                /*!< 0x00008000 */
#define PWR_PDCRC_PD15            PWR_PDCRC_PD15_Msk                           /*!< Pin PC15 Pull-Down set */

/********************  Bit definition for PWR_PUCRD register  *****************/
#define PWR_PUCRD_PU0_Pos         (0UL)
#define PWR_PUCRD_PU0_Msk         (0x1UL << PWR_PUCRD_PU0_Pos)                 /*!< 0x00000001 */
#define PWR_PUCRD_PU0             PWR_PUCRD_PU0_Msk                            /*!< Pin PD0 Pull-Up set */
#define PWR_PUCRD_PU1_Pos         (1UL)
#define PWR_PUCRD_PU1_Msk         (0x1UL << PWR_PUCRD_PU1_Pos)                 /*!< 0x00000002 */
#define PWR_PUCRD_PU1             PWR_PUCRD_PU1_Msk                            /*!< Pin PD1 Pull-Up set */
#define PWR_PUCRD_PU2_Pos         (2UL)
#define PWR_PUCRD_PU2_Msk         (0x1UL << PWR_PUCRD_PU2_Pos)                 /*!< 0x00000004 */
#define PWR_PUCRD_PU2             PWR_PUCRD_PU2_Msk                            /*!< Pin PD2 Pull-Up set */
#define PWR_PUCRD_PU3_Pos         (3UL)
#define PWR_PUCRD_PU3_Msk         (0x1UL << PWR_PUCRD_PU3_Pos)                 /*!< 0x00000008 */
#define PWR_PUCRD_PU3             PWR_PUCRD_PU3_Msk                            /*!< Pin PD3 Pull-Up set */
#define PWR_PUCRD_PU4_Pos         (4UL)
#define PWR_PUCRD_PU4_Msk         (0x1UL << PWR_PUCRD_PU4_Pos)                 /*!< 0x00000010 */
#define PWR_PUCRD_PU4             PWR_PUCRD_PU4_Msk                            /*!< Pin PD4 Pull-Up set */
#define PWR_PUCRD_PU5_Pos         (5UL)
#define PWR_PUCRD_PU5_Msk         (0x1UL << PWR_PUCRD_PU5_Pos)                 /*!< 0x00000020 */
#define PWR_PUCRD_PU5             PWR_PUCRD_PU5_Msk                            /*!< Pin PD5 Pull-Up set */
#define PWR_PUCRD_PU6_Pos         (6UL)
#define PWR_PUCRD_PU6_Msk         (0x1UL << PWR_PUCRD_PU6_Pos)                 /*!< 0x00000040 */
#define PWR_PUCRD_PU6             PWR_PUCRD_PU6_Msk                            /*!< Pin PD6 Pull-Up set */
#define PWR_PUCRD_PU8_Pos         (8UL)
#define PWR_PUCRD_PU8_Msk         (0x1UL << PWR_PUCRD_PU8_Pos)                 /*!< 0x00000100 */
#define PWR_PUCRD_PU8             PWR_PUCRD_PU8_Msk                            /*!< Pin PD8 Pull-Up set */
#define PWR_PUCRD_PU9_Pos         (9UL)
#define PWR_PUCRD_PU9_Msk         (0x1UL << PWR_PUCRD_PU9_Pos)                 /*!< 0x00000200 */
#define PWR_PUCRD_PU9             PWR_PUCRD_PU9_Msk                            /*!< Pin PD9 Pull-Up set */
#define PWR_PUCRD_PU10_Pos        (10UL)
#define PWR_PUCRD_PU10_Msk        (0x1UL << PWR_PUCRD_PU10_Pos)                /*!< 0x00000400 */
#define PWR_PUCRD_PU10            PWR_PUCRD_PU10_Msk                           /*!< Pin PD10 Pull-Up set */
#define PWR_PUCRD_PU11_Pos        (11UL)
#define PWR_PUCRD_PU11_Msk        (0x1UL << PWR_PUCRD_PU11_Pos)                /*!< 0x00000800 */
#define PWR_PUCRD_PU11            PWR_PUCRD_PU11_Msk                           /*!< Pin PD11 Pull-Up set */
#define PWR_PUCRD_PU12_Pos        (12UL)
#define PWR_PUCRD_PU12_Msk        (0x1UL << PWR_PUCRD_PU12_Pos)                /*!< 0x00001000 */
#define PWR_PUCRD_PU12            PWR_PUCRD_PU12_Msk                           /*!< Pin PD12 Pull-Up set */
#define PWR_PUCRD_PU13_Pos        (13UL)
#define PWR_PUCRD_PU13_Msk        (0x1UL << PWR_PUCRD_PU13_Pos)                /*!< 0x00002000 */
#define PWR_PUCRD_PU13            PWR_PUCRD_PU13_Msk                           /*!< Pin PD13 Pull-Up set */

/********************  Bit definition for PWR_PDCRD register  *****************/
#define PWR_PDCRD_PD0_Pos         (0UL)
#define PWR_PDCRD_PD0_Msk         (0x1UL << PWR_PDCRD_PD0_Pos)                 /*!< 0x00000001 */
#define PWR_PDCRD_PD0             PWR_PDCRD_PD0_Msk                            /*!< Pin PD0 Pull-Down set */
#define PWR_PDCRD_PD1_Pos         (1UL)
#define PWR_PDCRD_PD1_Msk         (0x1UL << PWR_PDCRD_PD1_Pos)                 /*!< 0x00000002 */
#define PWR_PDCRD_PD1             PWR_PDCRD_PD1_Msk                            /*!< Pin PD1 Pull-Down set */
#define PWR_PDCRD_PD2_Pos         (2UL)
#define PWR_PDCRD_PD2_Msk         (0x1UL << PWR_PDCRD_PD2_Pos)                 /*!< 0x00000004 */
#define PWR_PDCRD_PD2             PWR_PDCRD_PD2_Msk                            /*!< Pin PD2 Pull-Down set */
#define PWR_PDCRD_PD3_Pos         (3UL)
#define PWR_PDCRD_PD3_Msk         (0x1UL << PWR_PDCRD_PD3_Pos)                 /*!< 0x00000008 */
#define PWR_PDCRD_PD3             PWR_PDCRD_PD3_Msk                            /*!< Pin PD3 Pull-Down set */
#define PWR_PDCRD_PD4_Pos         (4UL)
#define PWR_PDCRD_PD4_Msk         (0x1UL << PWR_PDCRD_PD4_Pos)                 /*!< 0x00000010 */
#define PWR_PDCRD_PD4             PWR_PDCRD_PD4_Msk                            /*!< Pin PD4 Pull-Down set */
#define PWR_PDCRD_PD5_Pos         (5UL)
#define PWR_PDCRD_PD5_Msk         (0x1UL << PWR_PDCRD_PD5_Pos)                 /*!< 0x00000020 */
#define PWR_PDCRD_PD5             PWR_PDCRD_PD5_Msk                            /*!< Pin PD5 Pull-Down set */
#define PWR_PDCRD_PD6_Pos         (6UL)
#define PWR_PDCRD_PD6_Msk         (0x1UL << PWR_PDCRD_PD6_Pos)                 /*!< 0x00000040 */
#define PWR_PDCRD_PD6             PWR_PDCRD_PD6_Msk                            /*!< Pin PD6 Pull-Down set */
#define PWR_PDCRD_PD8_Pos         (8UL)
#define PWR_PDCRD_PD8_Msk         (0x1UL << PWR_PDCRD_PD8_Pos)                 /*!< 0x00000100 */
#define PWR_PDCRD_PD8             PWR_PDCRD_PD8_Msk                            /*!< Pin PD8 Pull-Down set */
#define PWR_PDCRD_PD9_Pos         (9UL)
#define PWR_PDCRD_PD9_Msk         (0x1UL << PWR_PDCRD_PD9_Pos)                 /*!< 0x00000200 */
#define PWR_PDCRD_PD9             PWR_PDCRD_PD9_Msk                            /*!< Pin PD9 Pull-Down set */
#define PWR_PDCRD_PD10_Pos        (10UL)
#define PWR_PDCRD_PD10_Msk        (0x1UL << PWR_PDCRD_PD10_Pos)                /*!< 0x00000400 */
#define PWR_PDCRD_PD10            PWR_PDCRD_PD10_Msk                           /*!< Pin PD10 Pull-Down set */
#define PWR_PDCRD_PD11_Pos        (11UL)
#define PWR_PDCRD_PD11_Msk        (0x1UL << PWR_PDCRD_PD11_Pos)                /*!< 0x00000800 */
#define PWR_PDCRD_PD11            PWR_PDCRD_PD11_Msk                           /*!< Pin PD11 Pull-Down set */
#define PWR_PDCRD_PD12_Pos        (12UL)
#define PWR_PDCRD_PD12_Msk        (0x1UL << PWR_PDCRD_PD12_Pos)                /*!< 0x00001000 */
#define PWR_PDCRD_PD12            PWR_PDCRD_PD12_Msk                           /*!< Pin PD12 Pull-Down set */
#define PWR_PDCRD_PD13_Pos        (13UL)
#define PWR_PDCRD_PD13_Msk        (0x1UL << PWR_PDCRD_PD13_Pos)                /*!< 0x00002000 */
#define PWR_PDCRD_PD13            PWR_PDCRD_PD13_Msk                           /*!< Pin PD13 Pull-Down set */

/********************  Bit definition for PWR_PUCRE register  *****************/
#define PWR_PUCRE_PU3_Pos         (3UL)
#define PWR_PUCRE_PU3_Msk         (0x1UL << PWR_PUCRE_PU3_Pos)                 /*!< 0x00000008 */
#define PWR_PUCRE_PU3             PWR_PUCRE_PU3_Msk                            /*!< Pin PE3 Pull-Up set */
#define PWR_PUCRE_PU7_Pos         (7UL)
#define PWR_PUCRE_PU7_Msk         (0x1UL << PWR_PUCRE_PU7_Pos)                 /*!< 0x00000080 */
#define PWR_PUCRE_PU7             PWR_PUCRE_PU7_Msk                            /*!< Pin PE7 Pull-Up set */
#define PWR_PUCRE_PU8_Pos         (8UL)
#define PWR_PUCRE_PU8_Msk         (0x1UL << PWR_PUCRE_PU8_Pos)                 /*!< 0x00000100 */
#define PWR_PUCRE_PU8             PWR_PUCRE_PU8_Msk                            /*!< Pin PE8 Pull-Up set */
#define PWR_PUCRE_PU9_Pos         (9UL)
#define PWR_PUCRE_PU9_Msk         (0x1UL << PWR_PUCRE_PU9_Pos)                 /*!< 0x00000200 */
#define PWR_PUCRE_PU9             PWR_PUCRE_PU9_Msk                            /*!< Pin PE9 Pull-Up set */

/********************  Bit definition for PWR_PDCRE register  *****************/
#define PWR_PDCRE_PD3_Pos         (3UL)
#define PWR_PDCRE_PD3_Msk         (0x1UL << PWR_PDCRE_PD3_Pos)                 /*!< 0x00000008 */
#define PWR_PDCRE_PD3             PWR_PDCRE_PD3_Msk                            /*!< Pin PE3 Pull-Down set */
#define PWR_PDCRE_PD7_Pos         (7UL)
#define PWR_PDCRE_PD7_Msk         (0x1UL << PWR_PDCRE_PD7_Pos)                 /*!< 0x00000080 */
#define PWR_PDCRE_PD7             PWR_PDCRE_PD7_Msk                            /*!< Pin PE7 Pull-Down set */
#define PWR_PDCRE_PD8_Pos         (8UL)
#define PWR_PDCRE_PD8_Msk         (0x1UL << PWR_PDCRE_PD8_Pos)                 /*!< 0x00000100 */
#define PWR_PDCRE_PD8             PWR_PDCRE_PD8_Msk                            /*!< Pin PE8 Pull-Down set */
#define PWR_PDCRE_PD9_Pos         (9UL)
#define PWR_PDCRE_PD9_Msk         (0x1UL << PWR_PDCRE_PD9_Pos)                 /*!< 0x00000200 */
#define PWR_PDCRE_PD9             PWR_PDCRE_PD9_Msk                            /*!< Pin PE9 Pull-Down set */

/********************  Bit definition for PWR_PUCRF register  *****************/
#define PWR_PUCRF_PU0_Pos         (0UL)
#define PWR_PUCRF_PU0_Msk         (0x1UL << PWR_PUCRF_PU0_Pos)                 /*!< 0x00000001 */
#define PWR_PUCRF_PU0             PWR_PUCRF_PU0_Msk                            /*!< Pin PF0 Pull-Up set */
#define PWR_PUCRF_PU1_Pos         (1UL)
#define PWR_PUCRF_PU1_Msk         (0x1UL << PWR_PUCRF_PU1_Pos)                 /*!< 0x00000002 */
#define PWR_PUCRF_PU1             PWR_PUCRF_PU1_Msk                            /*!< Pin PF1 Pull-Up set */
#define PWR_PUCRF_PU2_Pos         (2UL)
#define PWR_PUCRF_PU2_Msk         (0x1UL << PWR_PUCRF_PU2_Pos)                 /*!< 0x00000004 */
#define PWR_PUCRF_PU2             PWR_PUCRF_PU2_Msk                            /*!< Pin PF2 Pull-Up set */
#define PWR_PUCRF_PU3_Pos         (3UL)
#define PWR_PUCRF_PU3_Msk         (0x1UL << PWR_PUCRF_PU3_Pos)                 /*!< 0x00000008 */
#define PWR_PUCRF_PU3             PWR_PUCRF_PU3_Msk                            /*!< Pin PF3 Pull-Up set */

/********************  Bit definition for PWR_PDCRF register  *****************/
#define PWR_PDCRF_PD0_Pos         (0UL)
#define PWR_PDCRF_PD0_Msk         (0x1UL << PWR_PDCRF_PD0_Pos)                 /*!< 0x00000001 */
#define PWR_PDCRF_PD0             PWR_PDCRF_PD0_Msk                            /*!< Pin PF0 Pull-Down set */
#define PWR_PDCRF_PD1_Pos         (1UL)
#define PWR_PDCRF_PD1_Msk         (0x1UL << PWR_PDCRF_PD1_Pos)                 /*!< 0x00000002 */
#define PWR_PDCRF_PD1             PWR_PDCRF_PD1_Msk                            /*!< Pin PF1 Pull-Down set */
#define PWR_PDCRF_PD2_Pos         (2UL)
#define PWR_PDCRF_PD2_Msk         (0x1UL << PWR_PDCRF_PD2_Pos)                 /*!< 0x00000004 */
#define PWR_PDCRF_PD2             PWR_PDCRF_PD2_Msk                            /*!< Pin PF2 Pull-Down set */
#define PWR_PDCRF_PD3_Pos         (3UL)
#define PWR_PDCRF_PD3_Msk         (0x1UL << PWR_PDCRF_PD3_Pos)                 /*!< 0x00000008 */
#define PWR_PDCRF_PD3             PWR_PDCRF_PD3_Msk                            /*!< Pin PF3 Pull-Down set */


/******************************************************************************/
/*                                                                            */
/*                           Reset and Clock Control                          */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_MSION_Pos                     (0UL)
#define RCC_CR_MSION_Msk                     (0x1UL << RCC_CR_MSION_Pos)       /*!< 0x00000001 */
#define RCC_CR_MSION                         RCC_CR_MSION_Msk                  /*!< Internal Multi Speed oscillator (MSI) clock enable */
#define RCC_CR_MSIRDY_Pos                    (1UL)
#define RCC_CR_MSIRDY_Msk                    (0x1UL << RCC_CR_MSIRDY_Pos)      /*!< 0x00000002 */
#define RCC_CR_MSIRDY                        RCC_CR_MSIRDY_Msk                 /*!< Internal Multi Speed oscillator (MSI) clock ready flag */
#define RCC_CR_MSIPLLEN_Pos                  (2UL)
#define RCC_CR_MSIPLLEN_Msk                  (0x1UL << RCC_CR_MSIPLLEN_Pos)    /*!< 0x00000004 */
#define RCC_CR_MSIPLLEN                      RCC_CR_MSIPLLEN_Msk               /*!< Internal Multi Speed oscillator (MSI) PLL enable */
#define RCC_CR_MSIRGSEL_Pos                  (3UL)
#define RCC_CR_MSIRGSEL_Msk                  (0x1UL << RCC_CR_MSIRGSEL_Pos)    /*!< 0x00000008 */
#define RCC_CR_MSIRGSEL                      RCC_CR_MSIRGSEL_Msk               /*!< Internal Multi Speed oscillator (MSI) range selection */

/*!< MSIRANGE configuration : 12 frequency ranges available */
#define RCC_CR_MSIRANGE_Pos                  (4UL)
#define RCC_CR_MSIRANGE_Msk                  (0xFUL << RCC_CR_MSIRANGE_Pos)    /*!< 0x000000F0 */
#define RCC_CR_MSIRANGE                      RCC_CR_MSIRANGE_Msk               /*!< Internal Multi Speed oscillator (MSI) clock Range */
#define RCC_CR_MSIRANGE_0                    (0x0UL << RCC_CR_MSIRANGE_Pos)    /*!< 0x00000000 */
#define RCC_CR_MSIRANGE_1                    (0x1UL << RCC_CR_MSIRANGE_Pos)    /*!< 0x00000010 */
#define RCC_CR_MSIRANGE_2                    (0x2UL << RCC_CR_MSIRANGE_Pos)    /*!< 0x00000020 */
#define RCC_CR_MSIRANGE_3                    (0x3UL << RCC_CR_MSIRANGE_Pos)    /*!< 0x00000030 */
#define RCC_CR_MSIRANGE_4                    (0x4UL << RCC_CR_MSIRANGE_Pos)    /*!< 0x00000040 */
#define RCC_CR_MSIRANGE_5                    (0x5UL << RCC_CR_MSIRANGE_Pos)    /*!< 0x00000050 */
#define RCC_CR_MSIRANGE_6                    (0x6UL << RCC_CR_MSIRANGE_Pos)    /*!< 0x00000060 */
#define RCC_CR_MSIRANGE_7                    (0x7UL << RCC_CR_MSIRANGE_Pos)    /*!< 0x00000070 */
#define RCC_CR_MSIRANGE_8                    (0x8UL << RCC_CR_MSIRANGE_Pos)    /*!< 0x00000080 */
#define RCC_CR_MSIRANGE_9                    (0x9UL << RCC_CR_MSIRANGE_Pos)    /*!< 0x00000090 */
#define RCC_CR_MSIRANGE_10                   (0xAUL << RCC_CR_MSIRANGE_Pos)    /*!< 0x000000A0 */
#define RCC_CR_MSIRANGE_11                   (0xBUL << RCC_CR_MSIRANGE_Pos)    /*!< 0x000000B0 */

#define RCC_CR_HSION_Pos                     (8UL)
#define RCC_CR_HSION_Msk                     (0x1UL << RCC_CR_HSION_Pos)       /*!< 0x00000100 */
#define RCC_CR_HSION                         RCC_CR_HSION_Msk                  /*!< Internal High Speed oscillator (HSI16) clock enable */
#define RCC_CR_HSIKERON_Pos                  (9UL)
#define RCC_CR_HSIKERON_Msk                  (0x1UL << RCC_CR_HSIKERON_Pos)    /*!< 0x00000200 */
#define RCC_CR_HSIKERON                      RCC_CR_HSIKERON_Msk               /*!< Internal High Speed oscillator (HSI16) clock enable for some IPs Kernel */
#define RCC_CR_HSIRDY_Pos                    (10UL)
#define RCC_CR_HSIRDY_Msk                    (0x1UL << RCC_CR_HSIRDY_Pos)      /*!< 0x00000400 */
#define RCC_CR_HSIRDY                        RCC_CR_HSIRDY_Msk                 /*!< Internal High Speed oscillator (HSI16) clock ready flag */
#define RCC_CR_HSIASFS_Pos                   (11UL)
#define RCC_CR_HSIASFS_Msk                   (0x1UL << RCC_CR_HSIASFS_Pos)     /*!< 0x00000800 */
#define RCC_CR_HSIASFS                       RCC_CR_HSIASFS_Msk                /*!< HSI16 Automatic Start from Stop */

#define RCC_CR_HSEON_Pos                     (16UL)
#define RCC_CR_HSEON_Msk                     (0x1UL << RCC_CR_HSEON_Pos)       /*!< 0x00010000 */
#define RCC_CR_HSEON                         RCC_CR_HSEON_Msk                  /*!< External High Speed oscillator (HSE) clock enable */
#define RCC_CR_HSERDY_Pos                    (17UL)
#define RCC_CR_HSERDY_Msk                    (0x1UL << RCC_CR_HSERDY_Pos)      /*!< 0x00020000 */
#define RCC_CR_HSERDY                        RCC_CR_HSERDY_Msk                 /*!< External High Speed oscillator (HSE) clock ready */
#define RCC_CR_HSEBYP_Pos                    (18UL)
#define RCC_CR_HSEBYP_Msk                    (0x1UL << RCC_CR_HSEBYP_Pos)      /*!< 0x00040000 */
#define RCC_CR_HSEBYP                        RCC_CR_HSEBYP_Msk                 /*!< External High Speed oscillator (HSE) clock bypass */
#define RCC_CR_CSSON_Pos                     (19UL)
#define RCC_CR_CSSON_Msk                     (0x1UL << RCC_CR_CSSON_Pos)       /*!< 0x00080000 */
#define RCC_CR_CSSON                         RCC_CR_CSSON_Msk                  /*!< HSE Clock Security System enable */

#define RCC_CR_PLLON_Pos                     (24UL)
#define RCC_CR_PLLON_Msk                     (0x1UL << RCC_CR_PLLON_Pos)       /*!< 0x01000000 */
#define RCC_CR_PLLON                         RCC_CR_PLLON_Msk                  /*!< System PLL clock enable */
#define RCC_CR_PLLRDY_Pos                    (25UL)
#define RCC_CR_PLLRDY_Msk                    (0x1UL << RCC_CR_PLLRDY_Pos)      /*!< 0x02000000 */
#define RCC_CR_PLLRDY                        RCC_CR_PLLRDY_Msk                 /*!< System PLL clock ready */

/********************  Bit definition for RCC_ICSCR register  ***************/
/*!< MSICAL configuration */
#define RCC_ICSCR_MSICAL_Pos                 (0UL)
#define RCC_ICSCR_MSICAL_Msk                 (0xFFUL << RCC_ICSCR_MSICAL_Pos)  /*!< 0x000000FF */
#define RCC_ICSCR_MSICAL                     RCC_ICSCR_MSICAL_Msk              /*!< MSICAL[7:0] bits */
#define RCC_ICSCR_MSICAL_0                   (0x01UL << RCC_ICSCR_MSICAL_Pos)  /*!< 0x00000001 */
#define RCC_ICSCR_MSICAL_1                   (0x02UL << RCC_ICSCR_MSICAL_Pos)  /*!< 0x00000002 */
#define RCC_ICSCR_MSICAL_2                   (0x04UL << RCC_ICSCR_MSICAL_Pos)  /*!< 0x00000004 */
#define RCC_ICSCR_MSICAL_3                   (0x08UL << RCC_ICSCR_MSICAL_Pos)  /*!< 0x00000008 */
#define RCC_ICSCR_MSICAL_4                   (0x10UL << RCC_ICSCR_MSICAL_Pos)  /*!< 0x00000010 */
#define RCC_ICSCR_MSICAL_5                   (0x20UL << RCC_ICSCR_MSICAL_Pos)  /*!< 0x00000020 */
#define RCC_ICSCR_MSICAL_6                   (0x40UL << RCC_ICSCR_MSICAL_Pos)  /*!< 0x00000040 */
#define RCC_ICSCR_MSICAL_7                   (0x80UL << RCC_ICSCR_MSICAL_Pos)  /*!< 0x00000080 */

/*!< MSITRIM configuration */
#define RCC_ICSCR_MSITRIM_Pos                (8UL)
#define RCC_ICSCR_MSITRIM_Msk                (0xFFUL << RCC_ICSCR_MSITRIM_Pos) /*!< 0x0000FF00 */
#define RCC_ICSCR_MSITRIM                    RCC_ICSCR_MSITRIM_Msk             /*!< MSITRIM[7:0] bits */
#define RCC_ICSCR_MSITRIM_0                  (0x01UL << RCC_ICSCR_MSITRIM_Pos) /*!< 0x00000100 */
#define RCC_ICSCR_MSITRIM_1                  (0x02UL << RCC_ICSCR_MSITRIM_Pos) /*!< 0x00000200 */
#define RCC_ICSCR_MSITRIM_2                  (0x04UL << RCC_ICSCR_MSITRIM_Pos) /*!< 0x00000400 */
#define RCC_ICSCR_MSITRIM_3                  (0x08UL << RCC_ICSCR_MSITRIM_Pos) /*!< 0x00000800 */
#define RCC_ICSCR_MSITRIM_4                  (0x10UL << RCC_ICSCR_MSITRIM_Pos) /*!< 0x00001000 */
#define RCC_ICSCR_MSITRIM_5                  (0x20UL << RCC_ICSCR_MSITRIM_Pos) /*!< 0x00002000 */
#define RCC_ICSCR_MSITRIM_6                  (0x40UL << RCC_ICSCR_MSITRIM_Pos) /*!< 0x00004000 */
#define RCC_ICSCR_MSITRIM_7                  (0x80UL << RCC_ICSCR_MSITRIM_Pos) /*!< 0x00008000 */

/*!< HSICAL configuration */
#define RCC_ICSCR_HSICAL_Pos                 (16UL)
#define RCC_ICSCR_HSICAL_Msk                 (0xFFUL << RCC_ICSCR_HSICAL_Pos)  /*!< 0x00FF0000 */
#define RCC_ICSCR_HSICAL                     RCC_ICSCR_HSICAL_Msk              /*!< HSICAL[7:0] bits */
#define RCC_ICSCR_HSICAL_0                   (0x01UL << RCC_ICSCR_HSICAL_Pos)  /*!< 0x00010000 */
#define RCC_ICSCR_HSICAL_1                   (0x02UL << RCC_ICSCR_HSICAL_Pos)  /*!< 0x00020000 */
#define RCC_ICSCR_HSICAL_2                   (0x04UL << RCC_ICSCR_HSICAL_Pos)  /*!< 0x00040000 */
#define RCC_ICSCR_HSICAL_3                   (0x08UL << RCC_ICSCR_HSICAL_Pos)  /*!< 0x00080000 */
#define RCC_ICSCR_HSICAL_4                   (0x10UL << RCC_ICSCR_HSICAL_Pos)  /*!< 0x00100000 */
#define RCC_ICSCR_HSICAL_5                   (0x20UL << RCC_ICSCR_HSICAL_Pos)  /*!< 0x00200000 */
#define RCC_ICSCR_HSICAL_6                   (0x40UL << RCC_ICSCR_HSICAL_Pos)  /*!< 0x00400000 */
#define RCC_ICSCR_HSICAL_7                   (0x80UL << RCC_ICSCR_HSICAL_Pos)  /*!< 0x00800000 */

/*!< HSITRIM configuration */
#define RCC_ICSCR_HSITRIM_Pos                (24UL)
#define RCC_ICSCR_HSITRIM_Msk                (0x7FUL << RCC_ICSCR_HSITRIM_Pos) /*!< 0x7F000000 */
#define RCC_ICSCR_HSITRIM                    RCC_ICSCR_HSITRIM_Msk             /*!< HSITRIM[6:0] bits */
#define RCC_ICSCR_HSITRIM_0                  (0x01UL << RCC_ICSCR_HSITRIM_Pos) /*!< 0x01000000 */
#define RCC_ICSCR_HSITRIM_1                  (0x02UL << RCC_ICSCR_HSITRIM_Pos) /*!< 0x02000000 */
#define RCC_ICSCR_HSITRIM_2                  (0x04UL << RCC_ICSCR_HSITRIM_Pos) /*!< 0x04000000 */
#define RCC_ICSCR_HSITRIM_3                  (0x08UL << RCC_ICSCR_HSITRIM_Pos) /*!< 0x08000000 */
#define RCC_ICSCR_HSITRIM_4                  (0x10UL << RCC_ICSCR_HSITRIM_Pos) /*!< 0x10000000 */
#define RCC_ICSCR_HSITRIM_5                  (0x20UL << RCC_ICSCR_HSITRIM_Pos) /*!< 0x20000000 */
#define RCC_ICSCR_HSITRIM_6                  (0x40UL << RCC_ICSCR_HSITRIM_Pos) /*!< 0x40000000 */

/********************  Bit definition for RCC_CFGR register  ***************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                (0UL)
#define RCC_CFGR_SW_Msk                (0x7UL << RCC_CFGR_SW_Pos)              /*!< 0x00000007 */
#define RCC_CFGR_SW                    RCC_CFGR_SW_Msk                         /*!< SW[2:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0                  (0x1UL << RCC_CFGR_SW_Pos)              /*!< 0x00000001 */
#define RCC_CFGR_SW_1                  (0x2UL << RCC_CFGR_SW_Pos)              /*!< 0x00000002 */
#define RCC_CFGR_SW_2                  (0x4UL << RCC_CFGR_SW_Pos)              /*!< 0x00000004 */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos               (3UL)
#define RCC_CFGR_SWS_Msk               (0x7UL << RCC_CFGR_SWS_Pos)             /*!< 0x00000038 */
#define RCC_CFGR_SWS                   RCC_CFGR_SWS_Msk                        /*!< SWS[2:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0                 (0x1UL << RCC_CFGR_SWS_Pos)             /*!< 0x00000008 */
#define RCC_CFGR_SWS_1                 (0x2UL << RCC_CFGR_SWS_Pos)             /*!< 0x00000010 */
#define RCC_CFGR_SWS_2                 (0x4UL << RCC_CFGR_SWS_Pos)             /*!< 0x00000020 */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos              (8UL)
#define RCC_CFGR_HPRE_Msk              (0xFUL << RCC_CFGR_HPRE_Pos)            /*!< 0x00000F00 */
#define RCC_CFGR_HPRE                  RCC_CFGR_HPRE_Msk                       /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                (0x1UL << RCC_CFGR_HPRE_Pos)            /*!< 0x00000100 */
#define RCC_CFGR_HPRE_1                (0x2UL << RCC_CFGR_HPRE_Pos)            /*!< 0x00000200 */
#define RCC_CFGR_HPRE_2                (0x4UL << RCC_CFGR_HPRE_Pos)            /*!< 0x00000400 */
#define RCC_CFGR_HPRE_3                (0x8UL << RCC_CFGR_HPRE_Pos)            /*!< 0x00000800 */

/*!< PPRE configuration */
#define RCC_CFGR_PPRE_Pos              (12UL)
#define RCC_CFGR_PPRE_Msk              (0x7UL << RCC_CFGR_PPRE_Pos)            /*!< 0x00007000 */
#define RCC_CFGR_PPRE                  RCC_CFGR_PPRE_Msk                       /*!< PRE1[2:0] bits (APB prescaler) */
#define RCC_CFGR_PPRE_0                (0x1UL << RCC_CFGR_PPRE_Pos)            /*!< 0x00001000 */
#define RCC_CFGR_PPRE_1                (0x2UL << RCC_CFGR_PPRE_Pos)            /*!< 0x00002000 */
#define RCC_CFGR_PPRE_2                (0x4UL << RCC_CFGR_PPRE_Pos)            /*!< 0x00004000 */

/*!< STOPWUCK configuration */
#define RCC_CFGR_STOPWUCK_Pos          (15UL)
#define RCC_CFGR_STOPWUCK_Msk          (0x1UL << RCC_CFGR_STOPWUCK_Pos)        /*!< 0x00008000 */
#define RCC_CFGR_STOPWUCK              RCC_CFGR_STOPWUCK_Msk                   /*!< Wake Up from stop and CSS backup clock selection */

/*!< MCOSEL configuration */
#define RCC_CFGR_MCO2SEL_Pos            (16UL)
#define RCC_CFGR_MCO2SEL_Msk            (0xFUL << RCC_CFGR_MCO2SEL_Pos)        /*!< 0x000F0000 */
#define RCC_CFGR_MCO2SEL                RCC_CFGR_MCO2SEL_Msk                   /*!< MCO2SEL [3:0] bits (Clock output selection) */
#define RCC_CFGR_MCO2SEL_0              (0x1UL << RCC_CFGR_MCO2SEL_Pos)        /*!< 0x00010000 */
#define RCC_CFGR_MCO2SEL_1              (0x2UL << RCC_CFGR_MCO2SEL_Pos)        /*!< 0x00020000 */
#define RCC_CFGR_MCO2SEL_2              (0x4UL << RCC_CFGR_MCO2SEL_Pos)        /*!< 0x00040000 */
#define RCC_CFGR_MCO2SEL_3              (0x8UL << RCC_CFGR_MCO2SEL_Pos)        /*!< 0x00080000 */

/*!< MCO Prescaler configuration */
#define RCC_CFGR_MCO2PRE_Pos            (20UL)
#define RCC_CFGR_MCO2PRE_Msk            (0xFUL << RCC_CFGR_MCO2PRE_Pos)        /*!< 0x00F00000 */
#define RCC_CFGR_MCO2PRE                RCC_CFGR_MCO2PRE_Msk                   /*!< MCO1 prescaler [3:0] */
#define RCC_CFGR_MCO2PRE_0              (0x1UL << RCC_CFGR_MCO2PRE_Pos)        /*!< 0x00100000 */
#define RCC_CFGR_MCO2PRE_1              (0x2UL << RCC_CFGR_MCO2PRE_Pos)        /*!< 0x00200000 */
#define RCC_CFGR_MCO2PRE_2              (0x4UL << RCC_CFGR_MCO2PRE_Pos)        /*!< 0x00400000 */
#define RCC_CFGR_MCO2PRE_3              (0x8UL << RCC_CFGR_MCO2PRE_Pos)        /*!< 0x00800000 */

/*!< MCOSEL configuration */
#define RCC_CFGR_MCO1SEL_Pos            (24UL)
#define RCC_CFGR_MCO1SEL_Msk            (0xFUL << RCC_CFGR_MCO1SEL_Pos)        /*!< 0x0F000000 */
#define RCC_CFGR_MCO1SEL                RCC_CFGR_MCO1SEL_Msk                   /*!< MCO1SEL [3:0] bits (Clock output selection) */
#define RCC_CFGR_MCO1SEL_0              (0x1UL << RCC_CFGR_MCO1SEL_Pos)        /*!< 0x01000000 */
#define RCC_CFGR_MCO1SEL_1              (0x2UL << RCC_CFGR_MCO1SEL_Pos)        /*!< 0x02000000 */
#define RCC_CFGR_MCO1SEL_2              (0x4UL << RCC_CFGR_MCO1SEL_Pos)        /*!< 0x04000000 */
#define RCC_CFGR_MCO1SEL_3              (0x8UL << RCC_CFGR_MCO1SEL_Pos)        /*!< 0x08000000 */

/*!< MCO Prescaler configuration */
#define RCC_CFGR_MCO1PRE_Pos            (28UL)
#define RCC_CFGR_MCO1PRE_Msk            (0xFUL << RCC_CFGR_MCO1PRE_Pos)        /*!< 0xF0000000 */
#define RCC_CFGR_MCO1PRE                RCC_CFGR_MCO1PRE_Msk                   /*!< MCO1 prescaler [3:0] */
#define RCC_CFGR_MCO1PRE_0              (0x1UL << RCC_CFGR_MCO1PRE_Pos)        /*!< 0x10000000 */
#define RCC_CFGR_MCO1PRE_1              (0x2UL << RCC_CFGR_MCO1PRE_Pos)        /*!< 0x20000000 */
#define RCC_CFGR_MCO1PRE_2              (0x4UL << RCC_CFGR_MCO1PRE_Pos)        /*!< 0x40000000 */
#define RCC_CFGR_MCO1PRE_3              (0x8UL << RCC_CFGR_MCO1PRE_Pos)        /*!< 0x80000000 */

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLSRC_Pos         (0UL)
#define RCC_PLLCFGR_PLLSRC_Msk         (0x3UL << RCC_PLLCFGR_PLLSRC_Pos)       /*!< 0x00000003 */
#define RCC_PLLCFGR_PLLSRC             RCC_PLLCFGR_PLLSRC_Msk
#define RCC_PLLCFGR_PLLSRC_0           (0x1UL << RCC_PLLCFGR_PLLSRC_Pos)       /*!< 0x00000001 */
#define RCC_PLLCFGR_PLLSRC_1           (0x2UL << RCC_PLLCFGR_PLLSRC_Pos)       /*!< 0x00000002 */

#define RCC_PLLCFGR_PLLM_Pos           (4UL)
#define RCC_PLLCFGR_PLLM_Msk           (0x7UL << RCC_PLLCFGR_PLLM_Pos)         /*!< 0x00000070 */
#define RCC_PLLCFGR_PLLM               RCC_PLLCFGR_PLLM_Msk
#define RCC_PLLCFGR_PLLM_0             (0x1UL << RCC_PLLCFGR_PLLM_Pos)         /*!< 0x00000010 */
#define RCC_PLLCFGR_PLLM_1             (0x2UL << RCC_PLLCFGR_PLLM_Pos)         /*!< 0x00000020 */
#define RCC_PLLCFGR_PLLM_2             (0x4UL << RCC_PLLCFGR_PLLM_Pos)         /*!< 0x00000040 */

#define RCC_PLLCFGR_PLLN_Pos           (8UL)
#define RCC_PLLCFGR_PLLN_Msk           (0x7FUL << RCC_PLLCFGR_PLLN_Pos)        /*!< 0x00007F00 */
#define RCC_PLLCFGR_PLLN               RCC_PLLCFGR_PLLN_Msk
#define RCC_PLLCFGR_PLLN_0             (0x01UL << RCC_PLLCFGR_PLLN_Pos)        /*!< 0x00000100 */
#define RCC_PLLCFGR_PLLN_1             (0x02UL << RCC_PLLCFGR_PLLN_Pos)        /*!< 0x00000200 */
#define RCC_PLLCFGR_PLLN_2             (0x04UL << RCC_PLLCFGR_PLLN_Pos)        /*!< 0x00000400 */
#define RCC_PLLCFGR_PLLN_3             (0x08UL << RCC_PLLCFGR_PLLN_Pos)        /*!< 0x00000800 */
#define RCC_PLLCFGR_PLLN_4             (0x10UL << RCC_PLLCFGR_PLLN_Pos)        /*!< 0x00001000 */
#define RCC_PLLCFGR_PLLN_5             (0x20UL << RCC_PLLCFGR_PLLN_Pos)        /*!< 0x00002000 */
#define RCC_PLLCFGR_PLLN_6             (0x40UL << RCC_PLLCFGR_PLLN_Pos)        /*!< 0x00004000 */

#define RCC_PLLCFGR_PLLPEN_Pos         (16UL)
#define RCC_PLLCFGR_PLLPEN_Msk         (0x1UL << RCC_PLLCFGR_PLLPEN_Pos)       /*!< 0x00010000 */
#define RCC_PLLCFGR_PLLPEN             RCC_PLLCFGR_PLLPEN_Msk
#define RCC_PLLCFGR_PLLP_Pos           (17UL)
#define RCC_PLLCFGR_PLLP_Msk           (0x1FUL << RCC_PLLCFGR_PLLP_Pos)        /*!< 0x003E0000 */
#define RCC_PLLCFGR_PLLP               RCC_PLLCFGR_PLLP_Msk
#define RCC_PLLCFGR_PLLP_0             (0x1UL << RCC_PLLCFGR_PLLP_Pos)         /*!< 0x00020000 */
#define RCC_PLLCFGR_PLLP_1             (0x2UL << RCC_PLLCFGR_PLLP_Pos)         /*!< 0x00040000 */
#define RCC_PLLCFGR_PLLP_2             (0x4UL << RCC_PLLCFGR_PLLP_Pos)         /*!< 0x00080000 */
#define RCC_PLLCFGR_PLLP_3             (0x8UL << RCC_PLLCFGR_PLLP_Pos)         /*!< 0x00100000 */
#define RCC_PLLCFGR_PLLP_4             (0x10UL << RCC_PLLCFGR_PLLP_Pos)        /*!< 0x00200000 */

#define RCC_PLLCFGR_PLLQEN_Pos         (24UL)
#define RCC_PLLCFGR_PLLQEN_Msk         (0x1UL << RCC_PLLCFGR_PLLQEN_Pos)       /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLQEN             RCC_PLLCFGR_PLLQEN_Msk

#define RCC_PLLCFGR_PLLQ_Pos           (25UL)
#define RCC_PLLCFGR_PLLQ_Msk           (0x7UL << RCC_PLLCFGR_PLLQ_Pos)         /*!< 0x0E000000 */
#define RCC_PLLCFGR_PLLQ               RCC_PLLCFGR_PLLQ_Msk
#define RCC_PLLCFGR_PLLQ_0             (0x1UL << RCC_PLLCFGR_PLLQ_Pos)         /*!< 0x02000000 */
#define RCC_PLLCFGR_PLLQ_1             (0x2UL << RCC_PLLCFGR_PLLQ_Pos)         /*!< 0x04000000 */
#define RCC_PLLCFGR_PLLQ_2             (0x4UL << RCC_PLLCFGR_PLLQ_Pos)         /*!< 0x08000000 */

#define RCC_PLLCFGR_PLLREN_Pos         (28UL)
#define RCC_PLLCFGR_PLLREN_Msk         (0x1UL << RCC_PLLCFGR_PLLREN_Pos)       /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLREN             RCC_PLLCFGR_PLLREN_Msk
#define RCC_PLLCFGR_PLLR_Pos           (29UL)
#define RCC_PLLCFGR_PLLR_Msk           (0x7UL << RCC_PLLCFGR_PLLR_Pos)         /*!< 0xE0000000 */
#define RCC_PLLCFGR_PLLR               RCC_PLLCFGR_PLLR_Msk
#define RCC_PLLCFGR_PLLR_0             (0x1UL << RCC_PLLCFGR_PLLR_Pos)         /*!< 0x20000000 */
#define RCC_PLLCFGR_PLLR_1             (0x2UL << RCC_PLLCFGR_PLLR_Pos)         /*!< 0x40000000 */
#define RCC_PLLCFGR_PLLR_2             (0x4UL << RCC_PLLCFGR_PLLR_Pos)         /*!< 0x80000000 */

/********************  Bit definition for RCC_CIER register  ******************/
#define RCC_CIER_LSIRDYIE_Pos          (0UL)
#define RCC_CIER_LSIRDYIE_Msk          (0x1UL << RCC_CIER_LSIRDYIE_Pos)        /*!< 0x00000001 */
#define RCC_CIER_LSIRDYIE              RCC_CIER_LSIRDYIE_Msk
#define RCC_CIER_LSERDYIE_Pos          (1UL)
#define RCC_CIER_LSERDYIE_Msk          (0x1UL << RCC_CIER_LSERDYIE_Pos)        /*!< 0x00000002 */
#define RCC_CIER_LSERDYIE              RCC_CIER_LSERDYIE_Msk
#define RCC_CIER_MSIRDYIE_Pos          (2UL)
#define RCC_CIER_MSIRDYIE_Msk          (0x1UL << RCC_CIER_MSIRDYIE_Pos)        /*!< 0x00000004 */
#define RCC_CIER_MSIRDYIE              RCC_CIER_MSIRDYIE_Msk
#define RCC_CIER_HSIRDYIE_Pos          (3UL)
#define RCC_CIER_HSIRDYIE_Msk          (0x1UL << RCC_CIER_HSIRDYIE_Pos)        /*!< 0x00000008 */
#define RCC_CIER_HSIRDYIE              RCC_CIER_HSIRDYIE_Msk
#define RCC_CIER_HSERDYIE_Pos          (4UL)
#define RCC_CIER_HSERDYIE_Msk          (0x1UL << RCC_CIER_HSERDYIE_Pos)        /*!< 0x00000010 */
#define RCC_CIER_HSERDYIE              RCC_CIER_HSERDYIE_Msk
#define RCC_CIER_PLLRDYIE_Pos          (5UL)
#define RCC_CIER_PLLRDYIE_Msk          (0x1UL << RCC_CIER_PLLRDYIE_Pos)        /*!< 0x00000020 */
#define RCC_CIER_PLLRDYIE              RCC_CIER_PLLRDYIE_Msk

#define RCC_CIER_LSECSSIE_Pos          (9UL)
#define RCC_CIER_LSECSSIE_Msk          (0x1UL << RCC_CIER_LSECSSIE_Pos)        /*!< 0x00000200 */
#define RCC_CIER_LSECSSIE              RCC_CIER_LSECSSIE_Msk
#define RCC_CIER_HSI48RDYIE_Pos        (10UL)
#define RCC_CIER_HSI48RDYIE_Msk        (0x1UL << RCC_CIER_HSI48RDYIE_Pos)      /*!< 0x00000400 */
#define RCC_CIER_HSI48RDYIE            RCC_CIER_HSI48RDYIE_Msk
/********************  Bit definition for RCC_CIFR register  ******************/
#define RCC_CIFR_LSIRDYF_Pos           (0UL)
#define RCC_CIFR_LSIRDYF_Msk           (0x1UL << RCC_CIFR_LSIRDYF_Pos)         /*!< 0x00000001 */
#define RCC_CIFR_LSIRDYF               RCC_CIFR_LSIRDYF_Msk
#define RCC_CIFR_LSERDYF_Pos           (1UL)
#define RCC_CIFR_LSERDYF_Msk           (0x1UL << RCC_CIFR_LSERDYF_Pos)         /*!< 0x00000002 */
#define RCC_CIFR_LSERDYF               RCC_CIFR_LSERDYF_Msk
#define RCC_CIFR_MSIRDYF_Pos           (2UL)
#define RCC_CIFR_MSIRDYF_Msk           (0x1UL << RCC_CIFR_MSIRDYF_Pos)         /*!< 0x00000004 */
#define RCC_CIFR_MSIRDYF               RCC_CIFR_MSIRDYF_Msk
#define RCC_CIFR_HSIRDYF_Pos           (3UL)
#define RCC_CIFR_HSIRDYF_Msk           (0x1UL << RCC_CIFR_HSIRDYF_Pos)         /*!< 0x00000008 */
#define RCC_CIFR_HSIRDYF               RCC_CIFR_HSIRDYF_Msk
#define RCC_CIFR_HSERDYF_Pos           (4UL)
#define RCC_CIFR_HSERDYF_Msk           (0x1UL << RCC_CIFR_HSERDYF_Pos)         /*!< 0x00000010 */
#define RCC_CIFR_HSERDYF               RCC_CIFR_HSERDYF_Msk
#define RCC_CIFR_PLLRDYF_Pos           (5UL)
#define RCC_CIFR_PLLRDYF_Msk           (0x1UL << RCC_CIFR_PLLRDYF_Pos)         /*!< 0x00000020 */
#define RCC_CIFR_PLLRDYF               RCC_CIFR_PLLRDYF_Msk

#define RCC_CIFR_CSSF_Pos              (8UL)
#define RCC_CIFR_CSSF_Msk              (0x1UL << RCC_CIFR_CSSF_Pos)            /*!< 0x00000100 */
#define RCC_CIFR_CSSF                  RCC_CIFR_CSSF_Msk
#define RCC_CIFR_LSECSSF_Pos           (9UL)
#define RCC_CIFR_LSECSSF_Msk           (0x1UL << RCC_CIFR_LSECSSF_Pos)         /*!< 0x00000200 */
#define RCC_CIFR_LSECSSF               RCC_CIFR_LSECSSF_Msk
#define RCC_CIFR_HSI48RDYF_Pos         (10UL)
#define RCC_CIFR_HSI48RDYF_Msk         (0x1UL << RCC_CIFR_HSI48RDYF_Pos)       /*!< 0x00000400 */
#define RCC_CIFR_HSI48RDYF             RCC_CIFR_HSI48RDYF_Msk
/********************  Bit definition for RCC_CICR register  ******************/
#define RCC_CICR_LSIRDYC_Pos           (0UL)
#define RCC_CICR_LSIRDYC_Msk           (0x1UL << RCC_CICR_LSIRDYC_Pos)         /*!< 0x00000001 */
#define RCC_CICR_LSIRDYC               RCC_CICR_LSIRDYC_Msk
#define RCC_CICR_LSERDYC_Pos           (1UL)
#define RCC_CICR_LSERDYC_Msk           (0x1UL << RCC_CICR_LSERDYC_Pos)         /*!< 0x00000002 */
#define RCC_CICR_LSERDYC               RCC_CICR_LSERDYC_Msk
#define RCC_CICR_MSIRDYC_Pos           (2UL)
#define RCC_CICR_MSIRDYC_Msk           (0x1UL << RCC_CICR_MSIRDYC_Pos)         /*!< 0x00000004 */
#define RCC_CICR_MSIRDYC               RCC_CICR_MSIRDYC_Msk
#define RCC_CICR_HSIRDYC_Pos           (3UL)
#define RCC_CICR_HSIRDYC_Msk           (0x1UL << RCC_CICR_HSIRDYC_Pos)         /*!< 0x00000008 */
#define RCC_CICR_HSIRDYC               RCC_CICR_HSIRDYC_Msk
#define RCC_CICR_HSERDYC_Pos           (4UL)
#define RCC_CICR_HSERDYC_Msk           (0x1UL << RCC_CICR_HSERDYC_Pos)         /*!< 0x00000010 */
#define RCC_CICR_HSERDYC               RCC_CICR_HSERDYC_Msk
#define RCC_CICR_PLLRDYC_Pos           (5UL)
#define RCC_CICR_PLLRDYC_Msk           (0x1UL << RCC_CICR_PLLRDYC_Pos)         /*!< 0x00000020 */
#define RCC_CICR_PLLRDYC               RCC_CICR_PLLRDYC_Msk

#define RCC_CICR_CSSC_Pos              (8UL)
#define RCC_CICR_CSSC_Msk              (0x1UL << RCC_CICR_CSSC_Pos)            /*!< 0x00000100 */
#define RCC_CICR_CSSC                  RCC_CICR_CSSC_Msk
#define RCC_CICR_LSECSSC_Pos           (9UL)
#define RCC_CICR_LSECSSC_Msk           (0x1UL << RCC_CICR_LSECSSC_Pos)         /*!< 0x00000200 */
#define RCC_CICR_LSECSSC               RCC_CICR_LSECSSC_Msk
#define RCC_CICR_HSI48RDYC_Pos         (10UL)
#define RCC_CICR_HSI48RDYC_Msk         (0x1UL << RCC_CICR_HSI48RDYC_Pos)       /*!< 0x00000400 */
#define RCC_CICR_HSI48RDYC             RCC_CICR_HSI48RDYC_Msk
/********************  Bit definition for RCC_AHB1RSTR register  ***************/
#define RCC_AHBRSTR_DMA1RST_Pos       (0UL)
#define RCC_AHBRSTR_DMA1RST_Msk       (0x1UL << RCC_AHBRSTR_DMA1RST_Pos)       /*!< 0x00000001 */
#define RCC_AHBRSTR_DMA1RST           RCC_AHBRSTR_DMA1RST_Msk
#define RCC_AHBRSTR_DMA2RST_Pos       (1UL)
#define RCC_AHBRSTR_DMA2RST_Msk       (0x1UL << RCC_AHBRSTR_DMA2RST_Pos)       /*!< 0x00000002 */
#define RCC_AHBRSTR_DMA2RST           RCC_AHBRSTR_DMA2RST_Msk
#define RCC_AHBRSTR_FLASHRST_Pos      (8UL)
#define RCC_AHBRSTR_FLASHRST_Msk      (0x1UL << RCC_AHBRSTR_FLASHRST_Pos)      /*!< 0x00000100 */
#define RCC_AHBRSTR_FLASHRST          RCC_AHBRSTR_FLASHRST_Msk
#define RCC_AHBRSTR_CRCRST_Pos        (12UL)
#define RCC_AHBRSTR_CRCRST_Msk        (0x1UL << RCC_AHBRSTR_CRCRST_Pos)        /*!< 0x00001000 */
#define RCC_AHBRSTR_CRCRST            RCC_AHBRSTR_CRCRST_Msk
#define RCC_AHBRSTR_RNGRST_Pos        (18UL)
#define RCC_AHBRSTR_RNGRST_Msk        (0x1UL << RCC_AHBRSTR_RNGRST_Pos)        /*!< 0x00040000 */
#define RCC_AHBRSTR_RNGRST            RCC_AHBRSTR_RNGRST_Msk
#define RCC_AHBRSTR_TSCRST_Pos        (24UL)
#define RCC_AHBRSTR_TSCRST_Msk        (0x1UL << RCC_AHBRSTR_TSCRST_Pos)        /*!< 0x01000000 */
#define RCC_AHBRSTR_TSCRST            RCC_AHBRSTR_TSCRST_Msk

/********************  Bit definition for RCC_IOPRSTR register  **************/
#define RCC_IOPRSTR_GPIOARST_Pos       (0UL)
#define RCC_IOPRSTR_GPIOARST_Msk       (0x1UL << RCC_IOPRSTR_GPIOARST_Pos)     /*!< 0x00000001 */
#define RCC_IOPRSTR_GPIOARST           RCC_IOPRSTR_GPIOARST_Msk
#define RCC_IOPRSTR_GPIOBRST_Pos       (1UL)
#define RCC_IOPRSTR_GPIOBRST_Msk       (0x1UL << RCC_IOPRSTR_GPIOBRST_Pos)     /*!< 0x00000002 */
#define RCC_IOPRSTR_GPIOBRST           RCC_IOPRSTR_GPIOBRST_Msk
#define RCC_IOPRSTR_GPIOCRST_Pos       (2UL)
#define RCC_IOPRSTR_GPIOCRST_Msk       (0x1UL << RCC_IOPRSTR_GPIOCRST_Pos)     /*!< 0x00000004 */
#define RCC_IOPRSTR_GPIOCRST           RCC_IOPRSTR_GPIOCRST_Msk
#define RCC_IOPRSTR_GPIODRST_Pos       (3UL)
#define RCC_IOPRSTR_GPIODRST_Msk       (0x1UL << RCC_IOPRSTR_GPIODRST_Pos)     /*!< 0x00000008 */
#define RCC_IOPRSTR_GPIODRST           RCC_IOPRSTR_GPIODRST_Msk
#define RCC_IOPRSTR_GPIOERST_Pos       (4UL)
#define RCC_IOPRSTR_GPIOERST_Msk       (0x1UL << RCC_IOPRSTR_GPIOERST_Pos)     /*!< 0x00000010 */
#define RCC_IOPRSTR_GPIOERST           RCC_IOPRSTR_GPIOERST_Msk
#define RCC_IOPRSTR_GPIOFRST_Pos       (5UL)
#define RCC_IOPRSTR_GPIOFRST_Msk       (0x1UL << RCC_IOPRSTR_GPIOFRST_Pos)     /*!< 0x00000020 */
#define RCC_IOPRSTR_GPIOFRST           RCC_IOPRSTR_GPIOFRST_Msk

/********************  Bit definition for RCC_APBRSTR1 register  **************/
#define RCC_APBRSTR1_TIM2RST_Pos       (0UL)
#define RCC_APBRSTR1_TIM2RST_Msk       (0x1UL << RCC_APBRSTR1_TIM2RST_Pos)     /*!< 0x00000001 */
#define RCC_APBRSTR1_TIM2RST           RCC_APBRSTR1_TIM2RST_Msk
#define RCC_APBRSTR1_TIM3RST_Pos       (1UL)
#define RCC_APBRSTR1_TIM3RST_Msk       (0x1UL << RCC_APBRSTR1_TIM3RST_Pos)     /*!< 0x00000002 */
#define RCC_APBRSTR1_TIM3RST           RCC_APBRSTR1_TIM3RST_Msk
#define RCC_APBRSTR1_TIM6RST_Pos       (4UL)
#define RCC_APBRSTR1_TIM6RST_Msk       (0x1UL << RCC_APBRSTR1_TIM6RST_Pos)     /*!< 0x00000010 */
#define RCC_APBRSTR1_TIM6RST           RCC_APBRSTR1_TIM6RST_Msk
#define RCC_APBRSTR1_TIM7RST_Pos       (5UL)
#define RCC_APBRSTR1_TIM7RST_Msk       (0x1UL << RCC_APBRSTR1_TIM7RST_Pos)     /*!< 0x00000020 */
#define RCC_APBRSTR1_TIM7RST           RCC_APBRSTR1_TIM7RST_Msk
#define RCC_APBRSTR1_LPUART2RST_Pos    (7UL)
#define RCC_APBRSTR1_LPUART2RST_Msk    (0x1UL << RCC_APBRSTR1_LPUART2RST_Pos)  /*!< 0x00000080 */
#define RCC_APBRSTR1_LPUART2RST        RCC_APBRSTR1_LPUART2RST_Msk
#define RCC_APBRSTR1_LCDRST_Pos        (9UL)
#define RCC_APBRSTR1_LCDRST_Msk        (0x1UL << RCC_APBRSTR1_LCDRST_Pos)      /*!< 0x00000200 */
#define RCC_APBRSTR1_LCDRST            RCC_APBRSTR1_LCDRST_Msk
#define RCC_APBRSTR1_LPUART3RST_Pos    (12UL)
#define RCC_APBRSTR1_LPUART3RST_Msk    (0x1UL << RCC_APBRSTR1_LPUART3RST_Pos)  /*!< 0x00000100 */
#define RCC_APBRSTR1_LPUART3RST        RCC_APBRSTR1_LPUART3RST_Msk
#define RCC_APBRSTR1_USBRST_Pos        (13UL)
#define RCC_APBRSTR1_USBRST_Msk        (0x1UL << RCC_APBRSTR1_USBRST_Pos)      /*!< 0x00000200 */
#define RCC_APBRSTR1_USBRST            RCC_APBRSTR1_USBRST_Msk
#define RCC_APBRSTR1_SPI2RST_Pos       (14UL)
#define RCC_APBRSTR1_SPI2RST_Msk       (0x1UL << RCC_APBRSTR1_SPI2RST_Pos)     /*!< 0x00004000 */
#define RCC_APBRSTR1_SPI2RST           RCC_APBRSTR1_SPI2RST_Msk
#define RCC_APBRSTR1_SPI3RST_Pos       (15UL)
#define RCC_APBRSTR1_SPI3RST_Msk       (0x1UL << RCC_APBRSTR1_SPI3RST_Pos)     /*!< 0x00008000 */
#define RCC_APBRSTR1_SPI3RST           RCC_APBRSTR1_SPI3RST_Msk
#define RCC_APBRSTR1_CRSRST_Pos        (16UL)
#define RCC_APBRSTR1_CRSRST_Msk        (0x1UL << RCC_APBRSTR1_CRSRST_Pos)      /*!< 0x00010000 */
#define RCC_APBRSTR1_CRSRST            RCC_APBRSTR1_CRSRST_Msk
#define RCC_APBRSTR1_USART2RST_Pos     (17UL)
#define RCC_APBRSTR1_USART2RST_Msk     (0x1UL << RCC_APBRSTR1_USART2RST_Pos)   /*!< 0x00020000 */
#define RCC_APBRSTR1_USART2RST         RCC_APBRSTR1_USART2RST_Msk
#define RCC_APBRSTR1_USART3RST_Pos     (18UL)
#define RCC_APBRSTR1_USART3RST_Msk     (0x1UL << RCC_APBRSTR1_USART3RST_Pos)   /*!< 0x00040000 */
#define RCC_APBRSTR1_USART3RST         RCC_APBRSTR1_USART3RST_Msk
#define RCC_APBRSTR1_USART4RST_Pos     (19UL)
#define RCC_APBRSTR1_USART4RST_Msk     (0x1UL << RCC_APBRSTR1_USART4RST_Pos)   /*!< 0x00080000 */
#define RCC_APBRSTR1_USART4RST         RCC_APBRSTR1_USART4RST_Msk
#define RCC_APBRSTR1_LPUART1RST_Pos    (20UL)
#define RCC_APBRSTR1_LPUART1RST_Msk    (0x1UL << RCC_APBRSTR1_LPUART1RST_Pos)  /*!< 0x00010000 */
#define RCC_APBRSTR1_LPUART1RST        RCC_APBRSTR1_LPUART1RST_Msk
#define RCC_APBRSTR1_I2C1RST_Pos       (21UL)
#define RCC_APBRSTR1_I2C1RST_Msk       (0x1UL << RCC_APBRSTR1_I2C1RST_Pos)     /*!< 0x00200000 */
#define RCC_APBRSTR1_I2C1RST           RCC_APBRSTR1_I2C1RST_Msk
#define RCC_APBRSTR1_I2C2RST_Pos       (22UL)
#define RCC_APBRSTR1_I2C2RST_Msk       (0x1UL << RCC_APBRSTR1_I2C2RST_Pos)     /*!< 0x00400000 */
#define RCC_APBRSTR1_I2C2RST           RCC_APBRSTR1_I2C2RST_Msk
#define RCC_APBRSTR1_I2C3RST_Pos       (23UL)
#define RCC_APBRSTR1_I2C3RST_Msk       (0x1UL << RCC_APBRSTR1_I2C3RST_Pos)     /*!< 0x00800000 */
#define RCC_APBRSTR1_I2C3RST           RCC_APBRSTR1_I2C3RST_Msk
#define RCC_APBRSTR1_OPAMPRST_Pos      (24UL)
#define RCC_APBRSTR1_OPAMPRST_Msk      (0x1UL << RCC_APBRSTR1_OPAMPRST_Pos)    /*!< 0x01000000 */
#define RCC_APBRSTR1_OPAMPRST          RCC_APBRSTR1_OPAMPRST_Msk
#define RCC_APBRSTR1_I2C4RST_Pos       (25UL)
#define RCC_APBRSTR1_I2C4RST_Msk       (0x1UL << RCC_APBRSTR1_I2C4RST_Pos)     /*!< 0x02000000 */
#define RCC_APBRSTR1_I2C4RST           RCC_APBRSTR1_I2C4RST_Msk
#define RCC_APBRSTR1_LPTIM3RST_Pos     (26UL)
#define RCC_APBRSTR1_LPTIM3RST_Msk     (0x1UL << RCC_APBRSTR1_LPTIM3RST_Pos)   /*!< 0x04000000 */
#define RCC_APBRSTR1_LPTIM3RST         RCC_APBRSTR1_LPTIM3RST_Msk
#define RCC_APBRSTR1_PWRRST_Pos        (28UL)
#define RCC_APBRSTR1_PWRRST_Msk        (0x1UL << RCC_APBRSTR1_PWRRST_Pos)      /*!< 0x10000000 */
#define RCC_APBRSTR1_PWRRST            RCC_APBRSTR1_PWRRST_Msk
#define RCC_APBRSTR1_DAC1RST_Pos       (29UL)
#define RCC_APBRSTR1_DAC1RST_Msk       (0x1UL << RCC_APBRSTR1_DAC1RST_Pos)     /*!< 0x20000000 */
#define RCC_APBRSTR1_DAC1RST           RCC_APBRSTR1_DAC1RST_Msk
#define RCC_APBRSTR1_LPTIM2RST_Pos     (30UL)
#define RCC_APBRSTR1_LPTIM2RST_Msk     (0x1UL << RCC_APBRSTR1_LPTIM2RST_Pos)   /*!< 0x40000000 */
#define RCC_APBRSTR1_LPTIM2RST         RCC_APBRSTR1_LPTIM2RST_Msk
#define RCC_APBRSTR1_LPTIM1RST_Pos     (31UL)
#define RCC_APBRSTR1_LPTIM1RST_Msk     (0x1UL << RCC_APBRSTR1_LPTIM1RST_Pos)   /*!< 0x80000000 */
#define RCC_APBRSTR1_LPTIM1RST         RCC_APBRSTR1_LPTIM1RST_Msk

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define RCC_APBRSTR2_SYSCFGRST_Pos     (0UL)
#define RCC_APBRSTR2_SYSCFGRST_Msk     (0x1UL << RCC_APBRSTR2_SYSCFGRST_Pos)   /*!< 0x00000001 */
#define RCC_APBRSTR2_SYSCFGRST         RCC_APBRSTR2_SYSCFGRST_Msk
#define RCC_APBRSTR2_TIM1RST_Pos       (11UL)
#define RCC_APBRSTR2_TIM1RST_Msk       (0x1UL << RCC_APBRSTR2_TIM1RST_Pos)     /*!< 0x00000800 */
#define RCC_APBRSTR2_TIM1RST           RCC_APBRSTR2_TIM1RST_Msk
#define RCC_APBRSTR2_SPI1RST_Pos       (12UL)
#define RCC_APBRSTR2_SPI1RST_Msk       (0x1UL << RCC_APBRSTR2_SPI1RST_Pos)     /*!< 0x00001000 */
#define RCC_APBRSTR2_SPI1RST           RCC_APBRSTR2_SPI1RST_Msk
#define RCC_APBRSTR2_USART1RST_Pos     (14UL)
#define RCC_APBRSTR2_USART1RST_Msk     (0x1UL << RCC_APBRSTR2_USART1RST_Pos)   /*!< 0x00004000 */
#define RCC_APBRSTR2_USART1RST         RCC_APBRSTR2_USART1RST_Msk
#define RCC_APBRSTR2_TIM15RST_Pos      (16UL)
#define RCC_APBRSTR2_TIM15RST_Msk      (0x1UL << RCC_APBRSTR2_TIM15RST_Pos)    /*!< 0x00010000 */
#define RCC_APBRSTR2_TIM15RST          RCC_APBRSTR2_TIM15RST_Msk
#define RCC_APBRSTR2_TIM16RST_Pos      (17UL)
#define RCC_APBRSTR2_TIM16RST_Msk      (0x1UL << RCC_APBRSTR2_TIM16RST_Pos)    /*!< 0x00020000 */
#define RCC_APBRSTR2_TIM16RST          RCC_APBRSTR2_TIM16RST_Msk
#define RCC_APBRSTR2_ADCRST_Pos        (20UL)
#define RCC_APBRSTR2_ADCRST_Msk        (0x1UL << RCC_APBRSTR2_ADCRST_Pos)      /*!< 0x00100000 */
#define RCC_APBRSTR2_ADCRST            RCC_APBRSTR2_ADCRST_Msk

/********************  Bit definition for RCC_AHBENR register  ****************/
#define RCC_AHBENR_DMA1EN_Pos          (0UL)
#define RCC_AHBENR_DMA1EN_Msk          (0x1UL << RCC_AHBENR_DMA1EN_Pos)        /*!< 0x00000001 */
#define RCC_AHBENR_DMA1EN              RCC_AHBENR_DMA1EN_Msk
#define RCC_AHBENR_DMA2EN_Pos          (1UL)
#define RCC_AHBENR_DMA2EN_Msk          (0x1UL << RCC_AHBENR_DMA2EN_Pos)        /*!< 0x00000002 */
#define RCC_AHBENR_DMA2EN              RCC_AHBENR_DMA2EN_Msk
#define RCC_AHBENR_FLASHEN_Pos         (8UL)
#define RCC_AHBENR_FLASHEN_Msk         (0x1UL << RCC_AHBENR_FLASHEN_Pos)       /*!< 0x00000100 */
#define RCC_AHBENR_FLASHEN             RCC_AHBENR_FLASHEN_Msk
#define RCC_AHBENR_CRCEN_Pos           (12UL)
#define RCC_AHBENR_CRCEN_Msk           (0x1UL << RCC_AHBENR_CRCEN_Pos)         /*!< 0x00001000 */
#define RCC_AHBENR_CRCEN               RCC_AHBENR_CRCEN_Msk
#define RCC_AHBENR_RNGEN_Pos           (18UL)
#define RCC_AHBENR_RNGEN_Msk           (0x1UL << RCC_AHBENR_RNGEN_Pos)         /*!< 0x00040000 */
#define RCC_AHBENR_RNGEN               RCC_AHBENR_RNGEN_Msk
#define RCC_AHBENR_TSCEN_Pos           (24UL)
#define RCC_AHBENR_TSCEN_Msk           (0x1UL << RCC_AHBENR_TSCEN_Pos)         /*!< 0x01000000 */
#define RCC_AHBENR_TSCEN               RCC_AHBENR_TSCEN_Msk

/********************  Bit definition for RCC_IOPENR register  ****************/
#define RCC_IOPENR_GPIOAEN_Pos         (0UL)
#define RCC_IOPENR_GPIOAEN_Msk         (0x1UL << RCC_IOPENR_GPIOAEN_Pos)       /*!< 0x00000001 */
#define RCC_IOPENR_GPIOAEN             RCC_IOPENR_GPIOAEN_Msk
#define RCC_IOPENR_GPIOBEN_Pos         (1UL)
#define RCC_IOPENR_GPIOBEN_Msk         (0x1UL << RCC_IOPENR_GPIOBEN_Pos)       /*!< 0x00000002 */
#define RCC_IOPENR_GPIOBEN             RCC_IOPENR_GPIOBEN_Msk
#define RCC_IOPENR_GPIOCEN_Pos         (2UL)
#define RCC_IOPENR_GPIOCEN_Msk         (0x1UL << RCC_IOPENR_GPIOCEN_Pos)       /*!< 0x00000004 */
#define RCC_IOPENR_GPIOCEN             RCC_IOPENR_GPIOCEN_Msk
#define RCC_IOPENR_GPIODEN_Pos         (3UL)
#define RCC_IOPENR_GPIODEN_Msk         (0x1UL << RCC_IOPENR_GPIODEN_Pos)       /*!< 0x00000008 */
#define RCC_IOPENR_GPIODEN             RCC_IOPENR_GPIODEN_Msk
#define RCC_IOPENR_GPIOEEN_Pos         (4UL)
#define RCC_IOPENR_GPIOEEN_Msk         (0x1UL << RCC_IOPENR_GPIOEEN_Pos)       /*!< 0x00000010 */
#define RCC_IOPENR_GPIOEEN             RCC_IOPENR_GPIOEEN_Msk
#define RCC_IOPENR_GPIOFEN_Pos         (5UL)
#define RCC_IOPENR_GPIOFEN_Msk         (0x1UL << RCC_IOPENR_GPIOFEN_Pos)       /*!< 0x00000020 */
#define RCC_IOPENR_GPIOFEN             RCC_IOPENR_GPIOFEN_Msk

/********************  Bit definition for RCC_DBGCFGR register  ****************/
#define RCC_DBGCFGR_DBGEN_Pos          (0UL)
#define RCC_DBGCFGR_DBGEN_Msk          (0x1UL << RCC_DBGCFGR_DBGEN_Pos)        /*!< 0x00000001 */
#define RCC_DBGCFGR_DBGEN              RCC_DBGCFGR_DBGEN_Msk
#define RCC_DBGCFGR_DBGRST_Pos         (1UL)
#define RCC_DBGCFGR_DBGRST_Msk         (0x1UL << RCC_DBGCFGR_DBGRST_Pos)       /*!< 0x00000002 */
#define RCC_DBGCFGR_DBGRST             RCC_DBGCFGR_DBGRST_Msk

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define RCC_APBENR1_TIM2EN_Pos         (0UL)
#define RCC_APBENR1_TIM2EN_Msk         (0x1UL << RCC_APBENR1_TIM2EN_Pos)       /*!< 0x00000001 */
#define RCC_APBENR1_TIM2EN             RCC_APBENR1_TIM2EN_Msk
#define RCC_APBENR1_TIM3EN_Pos         (1UL)
#define RCC_APBENR1_TIM3EN_Msk         (0x1UL << RCC_APBENR1_TIM3EN_Pos)       /*!< 0x00000002 */
#define RCC_APBENR1_TIM3EN             RCC_APBENR1_TIM3EN_Msk
#define RCC_APBENR1_TIM6EN_Pos         (4UL)
#define RCC_APBENR1_TIM6EN_Msk         (0x1UL << RCC_APBENR1_TIM6EN_Pos)       /*!< 0x00000010 */
#define RCC_APBENR1_TIM6EN             RCC_APBENR1_TIM6EN_Msk
#define RCC_APBENR1_TIM7EN_Pos         (5UL)
#define RCC_APBENR1_TIM7EN_Msk         (0x1UL << RCC_APBENR1_TIM7EN_Pos)       /*!< 0x00000020 */
#define RCC_APBENR1_TIM7EN             RCC_APBENR1_TIM7EN_Msk
#define RCC_APBENR1_LPUART2EN_Pos      (7UL)
#define RCC_APBENR1_LPUART2EN_Msk      (0x1UL << RCC_APBENR1_LPUART2EN_Pos)    /*!< 0x00000080 */
#define RCC_APBENR1_LPUART2EN          RCC_APBENR1_LPUART2EN_Msk
#define RCC_APBENR1_LCDEN_Pos          (9UL)
#define RCC_APBENR1_LCDEN_Msk          (0x1UL << RCC_APBENR1_LCDEN_Pos)        /*!< 0x00000200 */
#define RCC_APBENR1_LCDEN              RCC_APBENR1_LCDEN_Msk
#define RCC_APBENR1_RTCAPBEN_Pos       (10UL)
#define RCC_APBENR1_RTCAPBEN_Msk       (0x1UL << RCC_APBENR1_RTCAPBEN_Pos)     /*!< 0x00000400 */
#define RCC_APBENR1_RTCAPBEN           RCC_APBENR1_RTCAPBEN_Msk
#define RCC_APBENR1_WWDGEN_Pos         (11UL)
#define RCC_APBENR1_WWDGEN_Msk         (0x1UL << RCC_APBENR1_WWDGEN_Pos)       /*!< 0x00000800 */
#define RCC_APBENR1_WWDGEN             RCC_APBENR1_WWDGEN_Msk
#define RCC_APBENR1_LPUART3EN_Pos      (12UL)
#define RCC_APBENR1_LPUART3EN_Msk      (0x1UL << RCC_APBENR1_LPUART3EN_Pos)    /*!< 0x00001000 */
#define RCC_APBENR1_LPUART3EN          RCC_APBENR1_LPUART3EN_Msk
#define RCC_APBENR1_USBEN_Pos          (13UL)
#define RCC_APBENR1_USBEN_Msk          (0x1UL << RCC_APBENR1_USBEN_Pos)        /*!< 0x00002000 */
#define RCC_APBENR1_USBEN              RCC_APBENR1_USBEN_Msk
#define RCC_APBENR1_SPI2EN_Pos         (14UL)
#define RCC_APBENR1_SPI2EN_Msk         (0x1UL << RCC_APBENR1_SPI2EN_Pos)       /*!< 0x00004000 */
#define RCC_APBENR1_SPI2EN             RCC_APBENR1_SPI2EN_Msk
#define RCC_APBENR1_SPI3EN_Pos         (15UL)
#define RCC_APBENR1_SPI3EN_Msk         (0x1UL << RCC_APBENR1_SPI3EN_Pos)       /*!< 0x00008000 */
#define RCC_APBENR1_SPI3EN             RCC_APBENR1_SPI3EN_Msk
#define RCC_APBENR1_CRSEN_Pos          (16UL)
#define RCC_APBENR1_CRSEN_Msk          (0x1UL << RCC_APBENR1_CRSEN_Pos)        /*!< 0x00010000 */
#define RCC_APBENR1_CRSEN              RCC_APBENR1_CRSEN_Msk
#define RCC_APBENR1_USART2EN_Pos       (17UL)
#define RCC_APBENR1_USART2EN_Msk       (0x1UL << RCC_APBENR1_USART2EN_Pos)     /*!< 0x00020000 */
#define RCC_APBENR1_USART2EN           RCC_APBENR1_USART2EN_Msk
#define RCC_APBENR1_USART3EN_Pos       (18UL)
#define RCC_APBENR1_USART3EN_Msk       (0x1UL << RCC_APBENR1_USART3EN_Pos)     /*!< 0x00040000 */
#define RCC_APBENR1_USART3EN           RCC_APBENR1_USART3EN_Msk
#define RCC_APBENR1_USART4EN_Pos       (19UL)
#define RCC_APBENR1_USART4EN_Msk       (0x1UL << RCC_APBENR1_USART4EN_Pos)     /*!< 0x00080000 */
#define RCC_APBENR1_USART4EN           RCC_APBENR1_USART4EN_Msk
#define RCC_APBENR1_LPUART1EN_Pos      (20UL)
#define RCC_APBENR1_LPUART1EN_Msk      (0x1UL << RCC_APBENR1_LPUART1EN_Pos)    /*!< 0x00010000 */
#define RCC_APBENR1_LPUART1EN          RCC_APBENR1_LPUART1EN_Msk
#define RCC_APBENR1_I2C1EN_Pos         (21UL)
#define RCC_APBENR1_I2C1EN_Msk         (0x1UL << RCC_APBENR1_I2C1EN_Pos)       /*!< 0x00200000 */
#define RCC_APBENR1_I2C1EN             RCC_APBENR1_I2C1EN_Msk
#define RCC_APBENR1_I2C2EN_Pos         (22UL)
#define RCC_APBENR1_I2C2EN_Msk         (0x1UL << RCC_APBENR1_I2C2EN_Pos)       /*!< 0x00400000 */
#define RCC_APBENR1_I2C2EN             RCC_APBENR1_I2C2EN_Msk
#define RCC_APBENR1_I2C3EN_Pos         (23UL)
#define RCC_APBENR1_I2C3EN_Msk         (0x1UL << RCC_APBENR1_I2C3EN_Pos)       /*!< 0x00800000 */
#define RCC_APBENR1_I2C3EN             RCC_APBENR1_I2C3EN_Msk
#define RCC_APBENR1_OPAMPEN_Pos        (24UL)
#define RCC_APBENR1_OPAMPEN_Msk        (0x1UL << RCC_APBENR1_OPAMPEN_Pos)      /*!< 0x01000000 */
#define RCC_APBENR1_OPAMPEN            RCC_APBENR1_OPAMPEN_Msk
#define RCC_APBENR1_I2C4EN_Pos         (25UL)
#define RCC_APBENR1_I2C4EN_Msk         (0x1UL << RCC_APBENR1_I2C4EN_Pos)       /*!< 0x02000000 */
#define RCC_APBENR1_I2C4EN             RCC_APBENR1_I2C4EN_Msk
#define RCC_APBENR1_LPTIM3EN_Pos       (26UL)
#define RCC_APBENR1_LPTIM3EN_Msk       (0x1UL << RCC_APBENR1_LPTIM3EN_Pos)     /*!< 0x04000000 */
#define RCC_APBENR1_LPTIM3EN           RCC_APBENR1_LPTIM3EN_Msk
#define RCC_APBENR1_PWREN_Pos          (28UL)
#define RCC_APBENR1_PWREN_Msk          (0x1UL << RCC_APBENR1_PWREN_Pos)        /*!< 0x10000000 */
#define RCC_APBENR1_PWREN              RCC_APBENR1_PWREN_Msk
#define RCC_APBENR1_DAC1EN_Pos         (29UL)
#define RCC_APBENR1_DAC1EN_Msk         (0x1UL << RCC_APBENR1_DAC1EN_Pos)       /*!< 0x20000000 */
#define RCC_APBENR1_DAC1EN             RCC_APBENR1_DAC1EN_Msk
#define RCC_APBENR1_LPTIM2EN_Pos       (30UL)
#define RCC_APBENR1_LPTIM2EN_Msk       (0x1UL << RCC_APBENR1_LPTIM2EN_Pos)     /*!< 0x40000000 */
#define RCC_APBENR1_LPTIM2EN           RCC_APBENR1_LPTIM2EN_Msk
#define RCC_APBENR1_LPTIM1EN_Pos       (31UL)
#define RCC_APBENR1_LPTIM1EN_Msk       (0x1UL << RCC_APBENR1_LPTIM1EN_Pos)     /*!< 0x80000000 */
#define RCC_APBENR1_LPTIM1EN           RCC_APBENR1_LPTIM1EN_Msk

/********************  Bit definition for RCC_APB2ENR register  **************/
#define RCC_APBENR2_SYSCFGEN_Pos       (0UL)
#define RCC_APBENR2_SYSCFGEN_Msk       (0x1UL << RCC_APBENR2_SYSCFGEN_Pos)     /*!< 0x00000001 */
#define RCC_APBENR2_SYSCFGEN           RCC_APBENR2_SYSCFGEN_Msk
#define RCC_APBENR2_TIM1EN_Pos         (11UL)
#define RCC_APBENR2_TIM1EN_Msk         (0x1UL << RCC_APBENR2_TIM1EN_Pos)       /*!< 0x00000800 */
#define RCC_APBENR2_TIM1EN             RCC_APBENR2_TIM1EN_Msk
#define RCC_APBENR2_SPI1EN_Pos         (12UL)
#define RCC_APBENR2_SPI1EN_Msk         (0x1UL << RCC_APBENR2_SPI1EN_Pos)       /*!< 0x00001000 */
#define RCC_APBENR2_SPI1EN             RCC_APBENR2_SPI1EN_Msk
#define RCC_APBENR2_USART1EN_Pos       (14UL)
#define RCC_APBENR2_USART1EN_Msk       (0x1UL << RCC_APBENR2_USART1EN_Pos)     /*!< 0x00004000 */
#define RCC_APBENR2_USART1EN           RCC_APBENR2_USART1EN_Msk
#define RCC_APBENR2_TIM15EN_Pos        (16UL)
#define RCC_APBENR2_TIM15EN_Msk        (0x1UL << RCC_APBENR2_TIM15EN_Pos)      /*!< 0x00008000 */
#define RCC_APBENR2_TIM15EN            RCC_APBENR2_TIM15EN_Msk
#define RCC_APBENR2_TIM16EN_Pos        (17UL)
#define RCC_APBENR2_TIM16EN_Msk        (0x1UL << RCC_APBENR2_TIM16EN_Pos)      /*!< 0x00020000 */
#define RCC_APBENR2_TIM16EN            RCC_APBENR2_TIM16EN_Msk
#define RCC_APBENR2_ADCEN_Pos          (20UL)
#define RCC_APBENR2_ADCEN_Msk          (0x1UL << RCC_APBENR2_ADCEN_Pos)        /*!< 0x00100000 */
#define RCC_APBENR2_ADCEN              RCC_APBENR2_ADCEN_Msk

/********************  Bit definition for RCC_AHBSMENR register  *************/
#define RCC_AHBSMENR_DMA1SMEN_Pos      (0UL)
#define RCC_AHBSMENR_DMA1SMEN_Msk      (0x1UL << RCC_AHBSMENR_DMA1SMEN_Pos)    /*!< 0x00000001 */
#define RCC_AHBSMENR_DMA1SMEN          RCC_AHBSMENR_DMA1SMEN_Msk
#define RCC_AHBSMENR_DMA2SMEN_Pos      (1UL)
#define RCC_AHBSMENR_DMA2SMEN_Msk      (0x1UL << RCC_AHBSMENR_DMA2SMEN_Pos)    /*!< 0x00000002 */
#define RCC_AHBSMENR_DMA2SMEN          RCC_AHBSMENR_DMA2SMEN_Msk
#define RCC_AHBSMENR_FLASHSMEN_Pos     (8UL)
#define RCC_AHBSMENR_FLASHSMEN_Msk     (0x1UL << RCC_AHBSMENR_FLASHSMEN_Pos)   /*!< 0x00000100 */
#define RCC_AHBSMENR_FLASHSMEN         RCC_AHBSMENR_FLASHSMEN_Msk
#define RCC_AHBSMENR_SRAM1SMEN_Pos     (9UL)
#define RCC_AHBSMENR_SRAM1SMEN_Msk     (0x1UL << RCC_AHBSMENR_SRAM1SMEN_Pos)   /*!< 0x00000200 */
#define RCC_AHBSMENR_SRAM1SMEN         RCC_AHBSMENR_SRAM1SMEN_Msk
#define RCC_AHBSMENR_CRCSMEN_Pos       (12UL)
#define RCC_AHBSMENR_CRCSMEN_Msk       (0x1UL << RCC_AHBSMENR_CRCSMEN_Pos)     /*!< 0x00001000 */
#define RCC_AHBSMENR_CRCSMEN           RCC_AHBSMENR_CRCSMEN_Msk
#define RCC_AHBSMENR_RNGSMEN_Pos       (18UL)
#define RCC_AHBSMENR_RNGSMEN_Msk       (0x1UL << RCC_AHBSMENR_RNGSMEN_Pos)     /*!< 0x00040000 */
#define RCC_AHBSMENR_RNGSMEN           RCC_AHBSMENR_RNGSMEN_Msk
#define RCC_AHBSMENR_TSCSMEN_Pos       (24UL)
#define RCC_AHBSMENR_TSCSMEN_Msk       (0x1UL << RCC_AHBSMENR_TSCSMEN_Pos)     /*!< 0x01000000 */
#define RCC_AHBSMENR_TSCSMEN           RCC_AHBSMENR_TSCSMEN_Msk

/********************  Bit definition for RCC_IOPSMENR register  *************/
#define RCC_IOPSMENR_GPIOASMEN_Pos     (0UL)
#define RCC_IOPSMENR_GPIOASMEN_Msk     (0x1UL << RCC_IOPSMENR_GPIOASMEN_Pos)   /*!< 0x00000001 */
#define RCC_IOPSMENR_GPIOASMEN         RCC_IOPSMENR_GPIOASMEN_Msk
#define RCC_IOPSMENR_GPIOBSMEN_Pos     (1UL)
#define RCC_IOPSMENR_GPIOBSMEN_Msk     (0x1UL << RCC_IOPSMENR_GPIOBSMEN_Pos)   /*!< 0x00000002 */
#define RCC_IOPSMENR_GPIOBSMEN         RCC_IOPSMENR_GPIOBSMEN_Msk
#define RCC_IOPSMENR_GPIOCSMEN_Pos     (2UL)
#define RCC_IOPSMENR_GPIOCSMEN_Msk     (0x1UL << RCC_IOPSMENR_GPIOCSMEN_Pos)   /*!< 0x00000004 */
#define RCC_IOPSMENR_GPIOCSMEN         RCC_IOPSMENR_GPIOCSMEN_Msk
#define RCC_IOPSMENR_GPIODSMEN_Pos     (3UL)
#define RCC_IOPSMENR_GPIODSMEN_Msk     (0x1UL << RCC_IOPSMENR_GPIODSMEN_Pos)   /*!< 0x00000008 */
#define RCC_IOPSMENR_GPIODSMEN         RCC_IOPSMENR_GPIODSMEN_Msk
#define RCC_IOPSMENR_GPIOESMEN_Pos     (4UL)
#define RCC_IOPSMENR_GPIOESMEN_Msk     (0x1UL << RCC_IOPSMENR_GPIOESMEN_Pos)   /*!< 0x00000010 */
#define RCC_IOPSMENR_GPIOESMEN         RCC_IOPSMENR_GPIOESMEN_Msk
#define RCC_IOPSMENR_GPIOFSMEN_Pos     (5UL)
#define RCC_IOPSMENR_GPIOFSMEN_Msk     (0x1UL << RCC_IOPSMENR_GPIOFSMEN_Pos)   /*!< 0x00000020 */
#define RCC_IOPSMENR_GPIOFSMEN         RCC_IOPSMENR_GPIOFSMEN_Msk

/********************  Bit definition for RCC_APB1SMENR register  *************/
#define RCC_APBSMENR1_TIM2SMEN_Pos     (0UL)
#define RCC_APBSMENR1_TIM2SMEN_Msk     (0x1UL << RCC_APBSMENR1_TIM2SMEN_Pos)   /*!< 0x00000001 */
#define RCC_APBSMENR1_TIM2SMEN         RCC_APBSMENR1_TIM2SMEN_Msk
#define RCC_APBSMENR1_TIM3SMEN_Pos     (1UL)
#define RCC_APBSMENR1_TIM3SMEN_Msk     (0x1UL << RCC_APBSMENR1_TIM3SMEN_Pos)   /*!< 0x00000002 */
#define RCC_APBSMENR1_TIM3SMEN         RCC_APBSMENR1_TIM3SMEN_Msk
#define RCC_APBSMENR1_TIM6SMEN_Pos     (4UL)
#define RCC_APBSMENR1_TIM6SMEN_Msk     (0x1UL << RCC_APBSMENR1_TIM6SMEN_Pos)   /*!< 0x00000010 */
#define RCC_APBSMENR1_TIM6SMEN         RCC_APBSMENR1_TIM6SMEN_Msk
#define RCC_APBSMENR1_TIM7SMEN_Pos     (5UL)
#define RCC_APBSMENR1_TIM7SMEN_Msk     (0x1UL << RCC_APBSMENR1_TIM7SMEN_Pos)   /*!< 0x00000020 */
#define RCC_APBSMENR1_TIM7SMEN         RCC_APBSMENR1_TIM7SMEN_Msk
#define RCC_APBSMENR1_LPUART2SMEN_Pos  (7UL)
#define RCC_APBSMENR1_LPUART2SMEN_Msk  (0x1UL << RCC_APBSMENR1_LPUART2SMEN_Pos)/*!< 0x00000080 */
#define RCC_APBSMENR1_LPUART2SMEN      RCC_APBSMENR1_LPUART2SMEN_Msk
#define RCC_APBSMENR1_LCDSMEN_Pos      (9UL)
#define RCC_APBSMENR1_LCDSMEN_Msk      (0x1UL << RCC_APBSMENR1_LCDSMEN_Pos)    /*!< 0x00000200 */
#define RCC_APBSMENR1_LCDSMEN          RCC_APBSMENR1_LCDSMEN_Msk
#define RCC_APBSMENR1_RTCAPBSMEN_Pos   (10UL)
#define RCC_APBSMENR1_RTCAPBSMEN_Msk   (0x1UL << RCC_APBSMENR1_RTCAPBSMEN_Pos) /*!< 0x00000400 */
#define RCC_APBSMENR1_RTCAPBSMEN       RCC_APBSMENR1_RTCAPBSMEN_Msk
#define RCC_APBSMENR1_WWDGSMEN_Pos     (11UL)
#define RCC_APBSMENR1_WWDGSMEN_Msk     (0x1UL << RCC_APBSMENR1_WWDGSMEN_Pos)   /*!< 0x00000800 */
#define RCC_APBSMENR1_WWDGSMEN         RCC_APBSMENR1_WWDGSMEN_Msk
#define RCC_APBSMENR1_LPUART3SMEN_Pos  (12UL)
#define RCC_APBSMENR1_LPUART3SMEN_Msk  (0x1UL << RCC_APBSMENR1_LPUART3SMEN_Pos)/*!< 0x00000100 */
#define RCC_APBSMENR1_LPUART3SMEN      RCC_APBSMENR1_LPUART3SMEN_Msk
#define RCC_APBSMENR1_USBSMEN_Pos      (13UL)
#define RCC_APBSMENR1_USBSMEN_Msk      (0x1UL << RCC_APBSMENR1_USBSMEN_Pos)    /*!< 0x00000200 */
#define RCC_APBSMENR1_USBSMEN          RCC_APBSMENR1_USBSMEN_Msk
#define RCC_APBSMENR1_SPI2SMEN_Pos     (14UL)
#define RCC_APBSMENR1_SPI2SMEN_Msk     (0x1UL << RCC_APBSMENR1_SPI2SMEN_Pos)   /*!< 0x00004000 */
#define RCC_APBSMENR1_SPI2SMEN         RCC_APBSMENR1_SPI2SMEN_Msk
#define RCC_APBSMENR1_SPI3SMEN_Pos     (15UL)
#define RCC_APBSMENR1_SPI3SMEN_Msk     (0x1UL << RCC_APBSMENR1_SPI3SMEN_Pos)   /*!< 0x00008000 */
#define RCC_APBSMENR1_SPI3SMEN         RCC_APBSMENR1_SPI3SMEN_Msk
#define RCC_APBSMENR1_CRSSMEN_Pos      (16UL)
#define RCC_APBSMENR1_CRSSMEN_Msk      (0x1UL << RCC_APBSMENR1_CRSSMEN_Pos)    /*!< 0x00010000 */
#define RCC_APBSMENR1_CRSSMEN          RCC_APBSMENR1_CRSSMEN_Msk
#define RCC_APBSMENR1_USART2SMEN_Pos   (17UL)
#define RCC_APBSMENR1_USART2SMEN_Msk   (0x1UL << RCC_APBSMENR1_USART2SMEN_Pos) /*!< 0x00020000 */
#define RCC_APBSMENR1_USART2SMEN       RCC_APBSMENR1_USART2SMEN_Msk
#define RCC_APBSMENR1_USART3SMEN_Pos   (18UL)
#define RCC_APBSMENR1_USART3SMEN_Msk   (0x1UL << RCC_APBSMENR1_USART3SMEN_Pos) /*!< 0x00040000 */
#define RCC_APBSMENR1_USART3SMEN       RCC_APBSMENR1_USART3SMEN_Msk
#define RCC_APBSMENR1_USART4SMEN_Pos   (19UL)
#define RCC_APBSMENR1_USART4SMEN_Msk   (0x1UL << RCC_APBSMENR1_USART4SMEN_Pos) /*!< 0x00080000 */
#define RCC_APBSMENR1_USART4SMEN       RCC_APBSMENR1_USART4SMEN_Msk
#define RCC_APBSMENR1_LPUART1SMEN_Pos  (20UL)
#define RCC_APBSMENR1_LPUART1SMEN_Msk  (0x1UL << RCC_APBSMENR1_LPUART1SMEN_Pos)/*!< 0x00100000 */
#define RCC_APBSMENR1_LPUART1SMEN      RCC_APBSMENR1_LPUART1SMEN_Msk
#define RCC_APBSMENR1_I2C1SMEN_Pos     (21UL)
#define RCC_APBSMENR1_I2C1SMEN_Msk     (0x1UL << RCC_APBSMENR1_I2C1SMEN_Pos)   /*!< 0x00200000 */
#define RCC_APBSMENR1_I2C1SMEN         RCC_APBSMENR1_I2C1SMEN_Msk
#define RCC_APBSMENR1_I2C2SMEN_Pos     (22UL)
#define RCC_APBSMENR1_I2C2SMEN_Msk     (0x1UL << RCC_APBSMENR1_I2C2SMEN_Pos)   /*!< 0x00400000 */
#define RCC_APBSMENR1_I2C2SMEN         RCC_APBSMENR1_I2C2SMEN_Msk
#define RCC_APBSMENR1_I2C3SMEN_Pos     (23UL)
#define RCC_APBSMENR1_I2C3SMEN_Msk     (0x1UL << RCC_APBSMENR1_I2C3SMEN_Pos)   /*!< 0x00800000 */
#define RCC_APBSMENR1_I2C3SMEN         RCC_APBSMENR1_I2C3SMEN_Msk
#define RCC_APBSMENR1_OPAMPSMEN_Pos    (24UL)
#define RCC_APBSMENR1_OPAMPSMEN_Msk    (0x1UL << RCC_APBSMENR1_OPAMPSMEN_Pos)  /*!< 0x01000000 */
#define RCC_APBSMENR1_OPAMPSMEN        RCC_APBSMENR1_OPAMPSMEN_Msk
#define RCC_APBSMENR1_I2C4SMEN_Pos     (25UL)
#define RCC_APBSMENR1_I2C4SMEN_Msk     (0x1UL << RCC_APBSMENR1_I2C4SMEN_Pos)   /*!< 0x02000000 */
#define RCC_APBSMENR1_I2C4SMEN         RCC_APBSMENR1_I2C4SMEN_Msk
#define RCC_APBSMENR1_LPTIM3SMEN_Pos   (26UL)
#define RCC_APBSMENR1_LPTIM3SMEN_Msk   (0x1UL << RCC_APBSMENR1_LPTIM3SMEN_Pos) /*!< 0x04000000 */
#define RCC_APBSMENR1_LPTIM3SMEN       RCC_APBSMENR1_LPTIM3SMEN_Msk
#define RCC_APBSMENR1_PWRSMEN_Pos      (28UL)
#define RCC_APBSMENR1_PWRSMEN_Msk      (0x1UL << RCC_APBSMENR1_PWRSMEN_Pos)    /*!< 0x10000000 */
#define RCC_APBSMENR1_PWRSMEN          RCC_APBSMENR1_PWRSMEN_Msk
#define RCC_APBSMENR1_DAC1SMEN_Pos     (29UL)
#define RCC_APBSMENR1_DAC1SMEN_Msk     (0x1UL << RCC_APBSMENR1_DAC1SMEN_Pos)   /*!< 0x20000000 */
#define RCC_APBSMENR1_DAC1SMEN         RCC_APBSMENR1_DAC1SMEN_Msk
#define RCC_APBSMENR1_LPTIM2SMEN_Pos   (30UL)
#define RCC_APBSMENR1_LPTIM2SMEN_Msk   (0x1UL << RCC_APBSMENR1_LPTIM2SMEN_Pos) /*!< 0x40000000 */
#define RCC_APBSMENR1_LPTIM2SMEN       RCC_APBSMENR1_LPTIM2SMEN_Msk
#define RCC_APBSMENR1_LPTIM1SMEN_Pos   (31UL)
#define RCC_APBSMENR1_LPTIM1SMEN_Msk   (0x1UL << RCC_APBSMENR1_LPTIM1SMEN_Pos) /*!< 0x80000000 */
#define RCC_APBSMENR1_LPTIM1SMEN       RCC_APBSMENR1_LPTIM1SMEN_Msk

/********************  Bit definition for RCC_APBSMENR2 register  *************/
#define RCC_APBSMENR2_SYSCFGSMEN_Pos   (0UL)
#define RCC_APBSMENR2_SYSCFGSMEN_Msk   (0x1UL << RCC_APBSMENR2_SYSCFGSMEN_Pos) /*!< 0x00000001 */
#define RCC_APBSMENR2_SYSCFGSMEN       RCC_APBSMENR2_SYSCFGSMEN_Msk
#define RCC_APBSMENR2_TIM1SMEN_Pos     (11UL)
#define RCC_APBSMENR2_TIM1SMEN_Msk     (0x1UL << RCC_APBSMENR2_TIM1SMEN_Pos)   /*!< 0x00000800 */
#define RCC_APBSMENR2_TIM1SMEN         RCC_APBSMENR2_TIM1SMEN_Msk
#define RCC_APBSMENR2_SPI1SMEN_Pos     (12UL)
#define RCC_APBSMENR2_SPI1SMEN_Msk     (0x1UL << RCC_APBSMENR2_SPI1SMEN_Pos)   /*!< 0x00001000 */
#define RCC_APBSMENR2_SPI1SMEN         RCC_APBSMENR2_SPI1SMEN_Msk
#define RCC_APBSMENR2_USART1SMEN_Pos   (14UL)
#define RCC_APBSMENR2_USART1SMEN_Msk   (0x1UL << RCC_APBSMENR2_USART1SMEN_Pos) /*!< 0x00004000 */
#define RCC_APBSMENR2_USART1SMEN       RCC_APBSMENR2_USART1SMEN_Msk
#define RCC_APBSMENR2_TIM15SMEN_Pos    (16UL)
#define RCC_APBSMENR2_TIM15SMEN_Msk    (0x1UL << RCC_APBSMENR2_TIM15SMEN_Pos)  /*!< 0x00010000 */
#define RCC_APBSMENR2_TIM15SMEN        RCC_APBSMENR2_TIM15SMEN_Msk
#define RCC_APBSMENR2_TIM16SMEN_Pos    (17UL)
#define RCC_APBSMENR2_TIM16SMEN_Msk    (0x1UL << RCC_APBSMENR2_TIM16SMEN_Pos)  /*!< 0x00020000 */
#define RCC_APBSMENR2_TIM16SMEN        RCC_APBSMENR2_TIM16SMEN_Msk
#define RCC_APBSMENR2_ADCSMEN_Pos      (20UL)
#define RCC_APBSMENR2_ADCSMEN_Msk      (0x1UL << RCC_APBSMENR2_ADCSMEN_Pos)    /*!< 0x00100000 */
#define RCC_APBSMENR2_ADCSMEN          RCC_APBSMENR2_ADCSMEN_Msk

/********************  Bit definition for RCC_CCIPR register  ******************/
#define RCC_CCIPR_USART1SEL_Pos        (0UL)
#define RCC_CCIPR_USART1SEL_Msk        (0x3UL << RCC_CCIPR_USART1SEL_Pos)      /*!< 0x00000003 */
#define RCC_CCIPR_USART1SEL            RCC_CCIPR_USART1SEL_Msk
#define RCC_CCIPR_USART1SEL_0          (0x1UL << RCC_CCIPR_USART1SEL_Pos)      /*!< 0x00000001 */
#define RCC_CCIPR_USART1SEL_1          (0x2UL << RCC_CCIPR_USART1SEL_Pos)      /*!< 0x00000002 */
#define RCC_CCIPR_USART2SEL_Pos        (2UL)
#define RCC_CCIPR_USART2SEL_Msk        (0x3UL << RCC_CCIPR_USART2SEL_Pos)      /*!< 0x0000000C */
#define RCC_CCIPR_USART2SEL            RCC_CCIPR_USART2SEL_Msk
#define RCC_CCIPR_USART2SEL_0          (0x1UL << RCC_CCIPR_USART2SEL_Pos)      /*!< 0x00000004 */
#define RCC_CCIPR_USART2SEL_1          (0x2UL << RCC_CCIPR_USART2SEL_Pos)      /*!< 0x00000008 */
#define RCC_CCIPR_LPUART3SEL_Pos       (6UL)
#define RCC_CCIPR_LPUART3SEL_Msk       (0x3UL << RCC_CCIPR_LPUART3SEL_Pos)     /*!< 0x000000C0 */
#define RCC_CCIPR_LPUART3SEL           RCC_CCIPR_LPUART3SEL_Msk
#define RCC_CCIPR_LPUART3SEL_0         (0x1UL << RCC_CCIPR_LPUART3SEL_Pos)     /*!< 0x00000040 */
#define RCC_CCIPR_LPUART3SEL_1         (0x2UL << RCC_CCIPR_LPUART3SEL_Pos)     /*!< 0x00000080 */
#define RCC_CCIPR_LPUART2SEL_Pos       (8UL)
#define RCC_CCIPR_LPUART2SEL_Msk       (0x3UL << RCC_CCIPR_LPUART2SEL_Pos)     /*!< 0x00000300 */
#define RCC_CCIPR_LPUART2SEL           RCC_CCIPR_LPUART2SEL_Msk
#define RCC_CCIPR_LPUART2SEL_0         (0x1UL << RCC_CCIPR_LPUART2SEL_Pos)     /*!< 0x00000100 */
#define RCC_CCIPR_LPUART2SEL_1         (0x2UL << RCC_CCIPR_LPUART2SEL_Pos)     /*!< 0x00000200 */
#define RCC_CCIPR_LPUART1SEL_Pos       (10UL)
#define RCC_CCIPR_LPUART1SEL_Msk       (0x3UL << RCC_CCIPR_LPUART1SEL_Pos)     /*!< 0x00000C00 */
#define RCC_CCIPR_LPUART1SEL           RCC_CCIPR_LPUART1SEL_Msk
#define RCC_CCIPR_LPUART1SEL_0         (0x1UL << RCC_CCIPR_LPUART1SEL_Pos)     /*!< 0x00000400 */
#define RCC_CCIPR_LPUART1SEL_1         (0x2UL << RCC_CCIPR_LPUART1SEL_Pos)     /*!< 0x00000800 */
#define RCC_CCIPR_I2C1SEL_Pos          (12UL)
#define RCC_CCIPR_I2C1SEL_Msk          (0x3UL << RCC_CCIPR_I2C1SEL_Pos)        /*!< 0x00003000 */
#define RCC_CCIPR_I2C1SEL              RCC_CCIPR_I2C1SEL_Msk
#define RCC_CCIPR_I2C1SEL_0            (0x1UL << RCC_CCIPR_I2C1SEL_Pos)        /*!< 0x00001000 */
#define RCC_CCIPR_I2C1SEL_1            (0x2UL << RCC_CCIPR_I2C1SEL_Pos)        /*!< 0x00002000 */
#define RCC_CCIPR_I2C3SEL_Pos          (16UL)
#define RCC_CCIPR_I2C3SEL_Msk          (0x3UL << RCC_CCIPR_I2C3SEL_Pos)        /*!< 0x00030000 */
#define RCC_CCIPR_I2C3SEL              RCC_CCIPR_I2C3SEL_Msk
#define RCC_CCIPR_I2C3SEL_0            (0x1UL << RCC_CCIPR_I2C3SEL_Pos)        /*!< 0x00010000 */
#define RCC_CCIPR_I2C3SEL_1            (0x2UL << RCC_CCIPR_I2C3SEL_Pos)        /*!< 0x00020000 */
#define RCC_CCIPR_LPTIM1SEL_Pos        (18UL)
#define RCC_CCIPR_LPTIM1SEL_Msk        (0x3UL << RCC_CCIPR_LPTIM1SEL_Pos)      /*!< 0x000C0000 */
#define RCC_CCIPR_LPTIM1SEL            RCC_CCIPR_LPTIM1SEL_Msk
#define RCC_CCIPR_LPTIM1SEL_0          (0x1UL << RCC_CCIPR_LPTIM1SEL_Pos)      /*!< 0x00040000 */
#define RCC_CCIPR_LPTIM1SEL_1          (0x2UL << RCC_CCIPR_LPTIM1SEL_Pos)      /*!< 0x00080000 */
#define RCC_CCIPR_LPTIM2SEL_Pos        (20UL)
#define RCC_CCIPR_LPTIM2SEL_Msk        (0x3UL << RCC_CCIPR_LPTIM2SEL_Pos)      /*!< 0x00300000 */
#define RCC_CCIPR_LPTIM2SEL            RCC_CCIPR_LPTIM2SEL_Msk
#define RCC_CCIPR_LPTIM2SEL_0          (0x1UL << RCC_CCIPR_LPTIM2SEL_Pos)      /*!< 0x00100000 */
#define RCC_CCIPR_LPTIM2SEL_1          (0x2UL << RCC_CCIPR_LPTIM2SEL_Pos)      /*!< 0x00200000 */
#define RCC_CCIPR_LPTIM3SEL_Pos        (22UL)
#define RCC_CCIPR_LPTIM3SEL_Msk        (0x3UL << RCC_CCIPR_LPTIM3SEL_Pos)      /*!< 0x00C0000 */
#define RCC_CCIPR_LPTIM3SEL            RCC_CCIPR_LPTIM3SEL_Msk
#define RCC_CCIPR_LPTIM3SEL_0          (0x1UL << RCC_CCIPR_LPTIM3SEL_Pos)      /*!< 0x00400000 */
#define RCC_CCIPR_LPTIM3SEL_1          (0x2UL << RCC_CCIPR_LPTIM3SEL_Pos)      /*!< 0x00800000 */
#define RCC_CCIPR_TIM1SEL_Pos          (24UL)
#define RCC_CCIPR_TIM1SEL_Msk          (0x1UL << RCC_CCIPR_TIM1SEL_Pos)        /*!< 0x01000000 */
#define RCC_CCIPR_TIM1SEL              RCC_CCIPR_TIM1SEL_Msk
#define RCC_CCIPR_TIM15SEL_Pos         (25UL)
#define RCC_CCIPR_TIM15SEL_Msk         (0x1UL << RCC_CCIPR_TIM15SEL_Pos)       /*!< 0x02000000 */
#define RCC_CCIPR_TIM15SEL             RCC_CCIPR_TIM15SEL_Msk
#define RCC_CCIPR_CLK48SEL_Pos         (26UL)
#define RCC_CCIPR_CLK48SEL_Msk         (0x3UL << RCC_CCIPR_CLK48SEL_Pos)         /*!< 0x0C000000 */
#define RCC_CCIPR_CLK48SEL             RCC_CCIPR_CLK48SEL_Msk
#define RCC_CCIPR_CLK48SEL_0           (0x1UL << RCC_CCIPR_CLK48SEL_Pos)       /*!< 0x04000000 */
#define RCC_CCIPR_CLK48SEL_1           (0x2UL << RCC_CCIPR_CLK48SEL_Pos)       /*!< 0x08000000 */
#define RCC_CCIPR_ADCSEL_Pos           (28UL)
#define RCC_CCIPR_ADCSEL_Msk           (0x3UL << RCC_CCIPR_ADCSEL_Pos)         /*!< 0x30000000 */
#define RCC_CCIPR_ADCSEL               RCC_CCIPR_ADCSEL_Msk
#define RCC_CCIPR_ADCSEL_0             (0x1UL << RCC_CCIPR_ADCSEL_Pos)         /*!< 0x10000000 */
#define RCC_CCIPR_ADCSEL_1             (0x2UL << RCC_CCIPR_ADCSEL_Pos)         /*!< 0x20000000 */

/********************  Bit definition for RCC_BDCR register  ******************/
#define RCC_BDCR_LSEON_Pos             (0UL)
#define RCC_BDCR_LSEON_Msk             (0x1UL << RCC_BDCR_LSEON_Pos)           /*!< 0x00000001 */
#define RCC_BDCR_LSEON                 RCC_BDCR_LSEON_Msk
#define RCC_BDCR_LSERDY_Pos            (1UL)
#define RCC_BDCR_LSERDY_Msk            (0x1UL << RCC_BDCR_LSERDY_Pos)          /*!< 0x00000002 */
#define RCC_BDCR_LSERDY                RCC_BDCR_LSERDY_Msk
#define RCC_BDCR_LSEBYP_Pos            (2UL)
#define RCC_BDCR_LSEBYP_Msk            (0x1UL << RCC_BDCR_LSEBYP_Pos)          /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP                RCC_BDCR_LSEBYP_Msk
#define RCC_BDCR_LSEDRV_Pos            (3UL)
#define RCC_BDCR_LSEDRV_Msk            (0x3UL << RCC_BDCR_LSEDRV_Pos)          /*!< 0x00000018 */
#define RCC_BDCR_LSEDRV                RCC_BDCR_LSEDRV_Msk
#define RCC_BDCR_LSEDRV_0              (0x1UL << RCC_BDCR_LSEDRV_Pos)          /*!< 0x00000008 */
#define RCC_BDCR_LSEDRV_1              (0x2UL << RCC_BDCR_LSEDRV_Pos)          /*!< 0x00000010 */
#define RCC_BDCR_LSECSSON_Pos          (5UL)
#define RCC_BDCR_LSECSSON_Msk          (0x1UL << RCC_BDCR_LSECSSON_Pos)        /*!< 0x00000020 */
#define RCC_BDCR_LSECSSON              RCC_BDCR_LSECSSON_Msk
#define RCC_BDCR_LSECSSD_Pos           (6UL)
#define RCC_BDCR_LSECSSD_Msk           (0x1UL << RCC_BDCR_LSECSSD_Pos)         /*!< 0x00000040 */
#define RCC_BDCR_LSECSSD               RCC_BDCR_LSECSSD_Msk
#define RCC_BDCR_LSESYSEN_Pos          (7UL)
#define RCC_BDCR_LSESYSEN_Msk          (0x1UL << RCC_BDCR_LSESYSEN_Pos)        /*!< 0x00000080 */
#define RCC_BDCR_LSESYSEN              RCC_BDCR_LSESYSEN_Msk
#define RCC_BDCR_RTCSEL_Pos            (8UL)
#define RCC_BDCR_RTCSEL_Msk            (0x3UL << RCC_BDCR_RTCSEL_Pos)          /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL                RCC_BDCR_RTCSEL_Msk
#define RCC_BDCR_RTCSEL_0              (0x1UL << RCC_BDCR_RTCSEL_Pos)          /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1              (0x2UL << RCC_BDCR_RTCSEL_Pos)          /*!< 0x00000200 */
#define RCC_BDCR_LSESYSRDY_Pos         (11UL)
#define RCC_BDCR_LSESYSRDY_Msk         (0x1UL << RCC_BDCR_LSESYSRDY_Pos)       /*!< 0x00000800 */
#define RCC_BDCR_LSESYSRDY             RCC_BDCR_LSESYSRDY_Msk
#define RCC_BDCR_RTCEN_Pos             (15UL)
#define RCC_BDCR_RTCEN_Msk             (0x1UL << RCC_BDCR_RTCEN_Pos)           /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                 RCC_BDCR_RTCEN_Msk
#define RCC_BDCR_BDRST_Pos             (16UL)
#define RCC_BDCR_BDRST_Msk             (0x1UL << RCC_BDCR_BDRST_Pos)           /*!< 0x00010000 */
#define RCC_BDCR_BDRST                 RCC_BDCR_BDRST_Msk
#define RCC_BDCR_LSCOEN_Pos            (24UL)
#define RCC_BDCR_LSCOEN_Msk            (0x1UL << RCC_BDCR_LSCOEN_Pos)          /*!< 0x01000000 */
#define RCC_BDCR_LSCOEN                RCC_BDCR_LSCOEN_Msk
#define RCC_BDCR_LSCOSEL_Pos           (25UL)
#define RCC_BDCR_LSCOSEL_Msk           (0x1UL << RCC_BDCR_LSCOSEL_Pos)         /*!< 0x02000000 */
#define RCC_BDCR_LSCOSEL               RCC_BDCR_LSCOSEL_Msk

/********************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos              (0UL)
#define RCC_CSR_LSION_Msk              (0x1UL << RCC_CSR_LSION_Pos)            /*!< 0x00000001 */
#define RCC_CSR_LSION                  RCC_CSR_LSION_Msk
#define RCC_CSR_LSIRDY_Pos             (1UL)
#define RCC_CSR_LSIRDY_Msk             (0x1UL << RCC_CSR_LSIRDY_Pos)           /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                 RCC_CSR_LSIRDY_Msk
#define RCC_CSR_LSIPREDIV_Pos          (2UL)
#define RCC_CSR_LSIPREDIV_Msk          (0x1UL << RCC_CSR_LSIPREDIV_Pos)        /*!< 0x00000004 */
#define RCC_CSR_LSIPREDIV              RCC_CSR_LSIPREDIV_Msk
#define RCC_CSR_MSISTBYRG_Pos          (8UL)
#define RCC_CSR_MSISTBYRG_Msk          (0xFUL << RCC_CSR_MSISTBYRG_Pos)        /*!< 0x00000F00 */
#define RCC_CSR_MSISTBYRG              RCC_CSR_MSISTBYRG_Msk
#define RCC_CSR_MSISTBYRG_1            (0x4UL << RCC_CSR_MSISTBYRG_Pos)        /*!< 0x00000400 */
#define RCC_CSR_MSISTBYRG_2            (0x5UL << RCC_CSR_MSISTBYRG_Pos)        /*!< 0x00000500 */
#define RCC_CSR_MSISTBYRG_4            (0x6UL << RCC_CSR_MSISTBYRG_Pos)        /*!< 0x00000600 */
#define RCC_CSR_MSISTBYRG_8            (0x7UL << RCC_CSR_MSISTBYRG_Pos)        /*!< 0x00000700 */
#define RCC_CSR_RMVF_Pos               (23UL)
#define RCC_CSR_RMVF_Msk               (0x1UL << RCC_CSR_RMVF_Pos)             /*!< 0x00800000 */
#define RCC_CSR_RMVF                   RCC_CSR_RMVF_Msk
#define RCC_CSR_OBLRSTF_Pos            (25UL)
#define RCC_CSR_OBLRSTF_Msk            (0x1UL << RCC_CSR_OBLRSTF_Pos)          /*!< 0x02000000 */
#define RCC_CSR_OBLRSTF                RCC_CSR_OBLRSTF_Msk
#define RCC_CSR_PINRSTF_Pos            (26UL)
#define RCC_CSR_PINRSTF_Msk            (0x1UL << RCC_CSR_PINRSTF_Pos)          /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                RCC_CSR_PINRSTF_Msk
#define RCC_CSR_PWRRSTF_Pos            (27UL)
#define RCC_CSR_PWRRSTF_Msk            (0x1UL << RCC_CSR_PWRRSTF_Pos)          /*!< 0x08000000 */
#define RCC_CSR_PWRRSTF                RCC_CSR_PWRRSTF_Msk
#define RCC_CSR_SFTRSTF_Pos            (28UL)
#define RCC_CSR_SFTRSTF_Msk            (0x1UL << RCC_CSR_SFTRSTF_Pos)          /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                RCC_CSR_SFTRSTF_Msk
#define RCC_CSR_IWDGRSTF_Pos           (29UL)
#define RCC_CSR_IWDGRSTF_Msk           (0x1UL << RCC_CSR_IWDGRSTF_Pos)         /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF               RCC_CSR_IWDGRSTF_Msk
#define RCC_CSR_WWDGRSTF_Pos           (30UL)
#define RCC_CSR_WWDGRSTF_Msk           (0x1UL << RCC_CSR_WWDGRSTF_Pos)         /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF               RCC_CSR_WWDGRSTF_Msk
#define RCC_CSR_LPWRRSTF_Pos           (31UL)
#define RCC_CSR_LPWRRSTF_Msk           (0x1UL << RCC_CSR_LPWRRSTF_Pos)         /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF               RCC_CSR_LPWRRSTF_Msk
/********************  Bit definition for RCC_CRRCR register  *****************/
#define RCC_CRRCR_HSI48ON_Pos          (0UL)
#define RCC_CRRCR_HSI48ON_Msk          (0x1UL << RCC_CRRCR_HSI48ON_Pos)        /*!< 0x00000000 */
#define RCC_CRRCR_HSI48ON              RCC_CRRCR_HSI48ON_Msk                   /*!< System HSI48 clock enable */
#define RCC_CRRCR_HSI48RDY_Pos         (1UL)
#define RCC_CRRCR_HSI48RDY_Msk         (0x1UL << RCC_CRRCR_HSI48RDY_Pos)       /*!< 0x00000001 */
#define RCC_CRRCR_HSI48RDY             RCC_CRRCR_HSI48RDY_Msk                  /*!< System HSI48 clock ready */

/*!< HSI48CAL configuration */
#define RCC_CRRCR_HSI48CAL_Pos         (7UL)
#define RCC_CRRCR_HSI48CAL_Msk         (0x1FFUL << RCC_CRRCR_HSI48CAL_Pos)     /*!< 0x0000FF80 */
#define RCC_CRRCR_HSI48CAL             RCC_CRRCR_HSI48CAL_Msk                  /*!< RC48CAL[8:0] bits */
#define RCC_CRRCR_HSI48CAL_0           (0x001UL << RCC_CRRCR_HSI48CAL_Pos)     /*!< 0x00000080 */
#define RCC_CRRCR_HSI48CAL_1           (0x002UL << RCC_CRRCR_HSI48CAL_Pos)     /*!< 0x00000100 */
#define RCC_CRRCR_HSI48CAL_2           (0x004UL << RCC_CRRCR_HSI48CAL_Pos)     /*!< 0x00000200 */
#define RCC_CRRCR_HSI48CAL_3           (0x008UL << RCC_CRRCR_HSI48CAL_Pos)     /*!< 0x00000400 */
#define RCC_CRRCR_HSI48CAL_4           (0x010UL << RCC_CRRCR_HSI48CAL_Pos)     /*!< 0x00000800 */
#define RCC_CRRCR_HSI48CAL_5           (0x020UL << RCC_CRRCR_HSI48CAL_Pos)     /*!< 0x00001000 */
#define RCC_CRRCR_HSI48CAL_6           (0x040UL << RCC_CRRCR_HSI48CAL_Pos)     /*!< 0x00002000 */
#define RCC_CRRCR_HSI48CAL_7           (0x080UL << RCC_CRRCR_HSI48CAL_Pos)     /*!< 0x00004000 */
#define RCC_CRRCR_HSI48CAL_8           (0x100UL << RCC_CRRCR_HSI48CAL_Pos)     /*!< 0x00008000 */
/******************************************************************************/
/*                                                                            */
/*                                    RNG                                     */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RNG_CR register  *******************/
#define RNG_CR_RNGEN_Pos                    (2UL)
#define RNG_CR_RNGEN_Msk                    (0x1UL << RNG_CR_RNGEN_Pos)             /*!< 0x00000004 */
#define RNG_CR_RNGEN                        RNG_CR_RNGEN_Msk
#define RNG_CR_IE_Pos                       (3UL)
#define RNG_CR_IE_Msk                       (0x1UL << RNG_CR_IE_Pos)                /*!< 0x00000008 */
#define RNG_CR_IE                           RNG_CR_IE_Msk
#define RNG_CR_CED_Pos                      (5UL)
#define RNG_CR_CED_Msk                      (0x1UL << RNG_CR_CED_Pos)               /*!< 0x00000020 */
#define RNG_CR_CED                          RNG_CR_CED_Msk
#define RNG_CR_ARDIS_Pos                    (7UL)
#define RNG_CR_ARDIS_Msk                    (0x1UL << RNG_CR_ARDIS_Pos)
#define RNG_CR_ARDIS                        RNG_CR_ARDIS_Msk
#define RNG_CR_RNG_CONFIG3_Pos              (8UL)
#define RNG_CR_RNG_CONFIG3_Msk              (0xFUL << RNG_CR_RNG_CONFIG3_Pos)
#define RNG_CR_RNG_CONFIG3                  RNG_CR_RNG_CONFIG3_Msk
#define RNG_CR_NISTC_Pos                    (12UL)
#define RNG_CR_NISTC_Msk                    (0x1UL << RNG_CR_NISTC_Pos)
#define RNG_CR_NISTC                        RNG_CR_NISTC_Msk
#define RNG_CR_RNG_CONFIG2_Pos              (13UL)
#define RNG_CR_RNG_CONFIG2_Msk              (0x7UL << RNG_CR_RNG_CONFIG2_Pos)
#define RNG_CR_RNG_CONFIG2                  RNG_CR_RNG_CONFIG2_Msk
#define RNG_CR_CLKDIV_Pos                   (16UL)
#define RNG_CR_CLKDIV_Msk                   (0xFUL << RNG_CR_CLKDIV_Pos)
#define RNG_CR_CLKDIV                       RNG_CR_CLKDIV_Msk
#define RNG_CR_CLKDIV_0                     (0x1UL << RNG_CR_CLKDIV_Pos)            /*!< 0x00010000 */
#define RNG_CR_CLKDIV_1                     (0x2UL << RNG_CR_CLKDIV_Pos)            /*!< 0x00020000 */
#define RNG_CR_CLKDIV_2                     (0x4UL << RNG_CR_CLKDIV_Pos)            /*!< 0x00040000 */
#define RNG_CR_CLKDIV_3                     (0x8UL << RNG_CR_CLKDIV_Pos)            /*!< 0x00080000 */
#define RNG_CR_RNG_CONFIG1_Pos              (20UL)
#define RNG_CR_RNG_CONFIG1_Msk              (0x3FUL << RNG_CR_RNG_CONFIG1_Pos)
#define RNG_CR_RNG_CONFIG1                  RNG_CR_RNG_CONFIG1_Msk
#define RNG_CR_CONDRST_Pos                  (30UL)
#define RNG_CR_CONDRST_Msk                  (0x1UL << RNG_CR_CONDRST_Pos)
#define RNG_CR_CONDRST                      RNG_CR_CONDRST_Msk
#define RNG_CR_CONFIGLOCK_Pos               (31UL)
#define RNG_CR_CONFIGLOCK_Msk               (0x1UL << RNG_CR_CONFIGLOCK_Pos)
#define RNG_CR_CONFIGLOCK                   RNG_CR_CONFIGLOCK_Msk

/********************  Bits definition for RNG_SR register  *******************/
#define RNG_SR_DRDY_Pos     (0UL)
#define RNG_SR_DRDY_Msk     (0x1UL << RNG_SR_DRDY_Pos)                         /*!< 0x00000001 */
#define RNG_SR_DRDY         RNG_SR_DRDY_Msk
#define RNG_SR_CECS_Pos     (1UL)
#define RNG_SR_CECS_Msk     (0x1UL << RNG_SR_CECS_Pos)                         /*!< 0x00000002 */
#define RNG_SR_CECS         RNG_SR_CECS_Msk
#define RNG_SR_SECS_Pos     (2UL)
#define RNG_SR_SECS_Msk     (0x1UL << RNG_SR_SECS_Pos)                         /*!< 0x00000004 */
#define RNG_SR_SECS         RNG_SR_SECS_Msk
#define RNG_SR_CEIS_Pos     (5UL)
#define RNG_SR_CEIS_Msk     (0x1UL << RNG_SR_CEIS_Pos)                         /*!< 0x00000020 */
#define RNG_SR_CEIS         RNG_SR_CEIS_Msk
#define RNG_SR_SEIS_Pos     (6UL)
#define RNG_SR_SEIS_Msk     (0x1UL << RNG_SR_SEIS_Pos)                         /*!< 0x00000040 */
#define RNG_SR_SEIS         RNG_SR_SEIS_Msk

/********************  Bits definition for RNG_HTCR register  *******************/
#define RNG_HTCR_HTCFG_Pos                  (0UL)
#define RNG_HTCR_HTCFG_Msk                  (0xFFFFFFFFUL << RNG_HTCR_HTCFG_Pos)    /*!< 0xFFFFFFFF */
#define RNG_HTCR_HTCFG                      RNG_HTCR_HTCFG_Msk

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RTC_TR register  *******************/
#define RTC_TR_SU_Pos                       (0UL)
#define RTC_TR_SU_Msk                       (0xFUL << RTC_TR_SU_Pos)                /*!< 0x0000000F */
#define RTC_TR_SU                           RTC_TR_SU_Msk
#define RTC_TR_SU_0                         (0x1UL << RTC_TR_SU_Pos)                /*!< 0x00000001 */
#define RTC_TR_SU_1                         (0x2UL << RTC_TR_SU_Pos)                /*!< 0x00000002 */
#define RTC_TR_SU_2                         (0x4UL << RTC_TR_SU_Pos)                /*!< 0x00000004 */
#define RTC_TR_SU_3                         (0x8UL << RTC_TR_SU_Pos)                /*!< 0x00000008 */
#define RTC_TR_ST_Pos                       (4UL)
#define RTC_TR_ST_Msk                       (0x7UL << RTC_TR_ST_Pos)                /*!< 0x00000070 */
#define RTC_TR_ST                           RTC_TR_ST_Msk
#define RTC_TR_ST_0                         (0x1UL << RTC_TR_ST_Pos)                /*!< 0x00000010 */
#define RTC_TR_ST_1                         (0x2UL << RTC_TR_ST_Pos)                /*!< 0x00000020 */
#define RTC_TR_ST_2                         (0x4UL << RTC_TR_ST_Pos)                /*!< 0x00000040 */
#define RTC_TR_MNU_Pos                      (8UL)
#define RTC_TR_MNU_Msk                      (0xFUL << RTC_TR_MNU_Pos)               /*!< 0x00000F00 */
#define RTC_TR_MNU                          RTC_TR_MNU_Msk
#define RTC_TR_MNU_0                        (0x1UL << RTC_TR_MNU_Pos)               /*!< 0x00000100 */
#define RTC_TR_MNU_1                        (0x2UL << RTC_TR_MNU_Pos)               /*!< 0x00000200 */
#define RTC_TR_MNU_2                        (0x4UL << RTC_TR_MNU_Pos)               /*!< 0x00000400 */
#define RTC_TR_MNU_3                        (0x8UL << RTC_TR_MNU_Pos)               /*!< 0x00000800 */
#define RTC_TR_MNT_Pos                      (12UL)
#define RTC_TR_MNT_Msk                      (0x7UL << RTC_TR_MNT_Pos)               /*!< 0x00007000 */
#define RTC_TR_MNT                          RTC_TR_MNT_Msk
#define RTC_TR_MNT_0                        (0x1UL << RTC_TR_MNT_Pos)               /*!< 0x00001000 */
#define RTC_TR_MNT_1                        (0x2UL << RTC_TR_MNT_Pos)               /*!< 0x00002000 */
#define RTC_TR_MNT_2                        (0x4UL << RTC_TR_MNT_Pos)               /*!< 0x00004000 */
#define RTC_TR_HU_Pos                       (16UL)
#define RTC_TR_HU_Msk                       (0xFUL << RTC_TR_HU_Pos)                /*!< 0x000F0000 */
#define RTC_TR_HU                           RTC_TR_HU_Msk
#define RTC_TR_HU_0                         (0x1UL << RTC_TR_HU_Pos)                /*!< 0x00010000 */
#define RTC_TR_HU_1                         (0x2UL << RTC_TR_HU_Pos)                /*!< 0x00020000 */
#define RTC_TR_HU_2                         (0x4UL << RTC_TR_HU_Pos)                /*!< 0x00040000 */
#define RTC_TR_HU_3                         (0x8UL << RTC_TR_HU_Pos)                /*!< 0x00080000 */
#define RTC_TR_HT_Pos                       (20UL)
#define RTC_TR_HT_Msk                       (0x3UL << RTC_TR_HT_Pos)                /*!< 0x00300000 */
#define RTC_TR_HT                           RTC_TR_HT_Msk
#define RTC_TR_HT_0                         (0x1UL << RTC_TR_HT_Pos)                /*!< 0x00100000 */
#define RTC_TR_HT_1                         (0x2UL << RTC_TR_HT_Pos)                /*!< 0x00200000 */
#define RTC_TR_PM_Pos                       (22UL)
#define RTC_TR_PM_Msk                       (0x1UL << RTC_TR_PM_Pos)                /*!< 0x00400000 */
#define RTC_TR_PM                           RTC_TR_PM_Msk

/********************  Bits definition for RTC_DR register  *******************/
#define RTC_DR_DU_Pos                       (0UL)
#define RTC_DR_DU_Msk                       (0xFUL << RTC_DR_DU_Pos)                /*!< 0x0000000F */
#define RTC_DR_DU                           RTC_DR_DU_Msk
#define RTC_DR_DU_0                         (0x1UL << RTC_DR_DU_Pos)                /*!< 0x00000001 */
#define RTC_DR_DU_1                         (0x2UL << RTC_DR_DU_Pos)                /*!< 0x00000002 */
#define RTC_DR_DU_2                         (0x4UL << RTC_DR_DU_Pos)                /*!< 0x00000004 */
#define RTC_DR_DU_3                         (0x8UL << RTC_DR_DU_Pos)                /*!< 0x00000008 */
#define RTC_DR_DT_Pos                       (4UL)
#define RTC_DR_DT_Msk                       (0x3UL << RTC_DR_DT_Pos)                /*!< 0x00000030 */
#define RTC_DR_DT                           RTC_DR_DT_Msk
#define RTC_DR_DT_0                         (0x1UL << RTC_DR_DT_Pos)                /*!< 0x00000010 */
#define RTC_DR_DT_1                         (0x2UL << RTC_DR_DT_Pos)                /*!< 0x00000020 */
#define RTC_DR_MU_Pos                       (8UL)
#define RTC_DR_MU_Msk                       (0xFUL << RTC_DR_MU_Pos)                /*!< 0x00000F00 */
#define RTC_DR_MU                           RTC_DR_MU_Msk
#define RTC_DR_MU_0                         (0x1UL << RTC_DR_MU_Pos)                /*!< 0x00000100 */
#define RTC_DR_MU_1                         (0x2UL << RTC_DR_MU_Pos)                /*!< 0x00000200 */
#define RTC_DR_MU_2                         (0x4UL << RTC_DR_MU_Pos)                /*!< 0x00000400 */
#define RTC_DR_MU_3                         (0x8UL << RTC_DR_MU_Pos)                /*!< 0x00000800 */
#define RTC_DR_MT_Pos                       (12UL)
#define RTC_DR_MT_Msk                       (0x1UL << RTC_DR_MT_Pos)                /*!< 0x00001000 */
#define RTC_DR_MT                           RTC_DR_MT_Msk
#define RTC_DR_WDU_Pos                      (13UL)
#define RTC_DR_WDU_Msk                      (0x7UL << RTC_DR_WDU_Pos)               /*!< 0x0000E000 */
#define RTC_DR_WDU                          RTC_DR_WDU_Msk
#define RTC_DR_WDU_0                        (0x1UL << RTC_DR_WDU_Pos)               /*!< 0x00002000 */
#define RTC_DR_WDU_1                        (0x2UL << RTC_DR_WDU_Pos)               /*!< 0x00004000 */
#define RTC_DR_WDU_2                        (0x4UL << RTC_DR_WDU_Pos)               /*!< 0x00008000 */
#define RTC_DR_YU_Pos                       (16UL)
#define RTC_DR_YU_Msk                       (0xFUL << RTC_DR_YU_Pos)                /*!< 0x000F0000 */
#define RTC_DR_YU                           RTC_DR_YU_Msk
#define RTC_DR_YU_0                         (0x1UL << RTC_DR_YU_Pos)                /*!< 0x00010000 */
#define RTC_DR_YU_1                         (0x2UL << RTC_DR_YU_Pos)                /*!< 0x00020000 */
#define RTC_DR_YU_2                         (0x4UL << RTC_DR_YU_Pos)                /*!< 0x00040000 */
#define RTC_DR_YU_3                         (0x8UL << RTC_DR_YU_Pos)                /*!< 0x00080000 */
#define RTC_DR_YT_Pos                       (20UL)
#define RTC_DR_YT_Msk                       (0xFUL << RTC_DR_YT_Pos)                /*!< 0x00F00000 */
#define RTC_DR_YT                           RTC_DR_YT_Msk
#define RTC_DR_YT_0                         (0x1UL << RTC_DR_YT_Pos)                /*!< 0x00100000 */
#define RTC_DR_YT_1                         (0x2UL << RTC_DR_YT_Pos)                /*!< 0x00200000 */
#define RTC_DR_YT_2                         (0x4UL << RTC_DR_YT_Pos)                /*!< 0x00400000 */
#define RTC_DR_YT_3                         (0x8UL << RTC_DR_YT_Pos)                /*!< 0x00800000 */

/********************  Bits definition for RTC_SSR register  ******************/
#define RTC_SSR_SS_Pos                      (0UL)
#define RTC_SSR_SS_Msk                      (0xFFFFFFFFUL << RTC_SSR_SS_Pos)        /*!< 0xFFFFFFFF */
#define RTC_SSR_SS                          RTC_SSR_SS_Msk

/********************  Bits definition for RTC_ICSR register  ******************/
#define RTC_ICSR_WUTWF_Pos                  (2UL)
#define RTC_ICSR_WUTWF_Msk                  (0x1UL << RTC_ICSR_WUTWF_Pos)           /*!< 0x00000004 */
#define RTC_ICSR_WUTWF                      RTC_ICSR_WUTWF_Msk
#define RTC_ICSR_SHPF_Pos                   (3UL)
#define RTC_ICSR_SHPF_Msk                   (0x1UL << RTC_ICSR_SHPF_Pos)            /*!< 0x00000008 */
#define RTC_ICSR_SHPF                       RTC_ICSR_SHPF_Msk
#define RTC_ICSR_INITS_Pos                  (4UL)
#define RTC_ICSR_INITS_Msk                  (0x1UL << RTC_ICSR_INITS_Pos)           /*!< 0x00000010 */
#define RTC_ICSR_INITS                      RTC_ICSR_INITS_Msk
#define RTC_ICSR_RSF_Pos                    (5UL)
#define RTC_ICSR_RSF_Msk                    (0x1UL << RTC_ICSR_RSF_Pos)             /*!< 0x00000020 */
#define RTC_ICSR_RSF                        RTC_ICSR_RSF_Msk
#define RTC_ICSR_INITF_Pos                  (6UL)
#define RTC_ICSR_INITF_Msk                  (0x1UL << RTC_ICSR_INITF_Pos)           /*!< 0x00000040 */
#define RTC_ICSR_INITF                      RTC_ICSR_INITF_Msk
#define RTC_ICSR_INIT_Pos                   (7UL)
#define RTC_ICSR_INIT_Msk                   (0x1UL << RTC_ICSR_INIT_Pos)            /*!< 0x00000080 */
#define RTC_ICSR_INIT                       RTC_ICSR_INIT_Msk
#define RTC_ICSR_BIN_Pos                    (8UL)
#define RTC_ICSR_BIN_Msk                    (0x3UL << RTC_ICSR_BIN_Pos)             /*!< 0x00000300 */
#define RTC_ICSR_BIN                        RTC_ICSR_BIN_Msk
#define RTC_ICSR_BIN_0                      (0x1UL << RTC_ICSR_BIN_Pos)             /*!< 0x00000100 */
#define RTC_ICSR_BIN_1                      (0x2UL << RTC_ICSR_BIN_Pos)             /*!< 0x00000200 */
#define RTC_ICSR_BCDU_Pos                   (10UL)
#define RTC_ICSR_BCDU_Msk                   (0x7UL << RTC_ICSR_BCDU_Pos)            /*!< 0x00001C00 */
#define RTC_ICSR_BCDU                       RTC_ICSR_BCDU_Msk
#define RTC_ICSR_BCDU_0                     (0x1UL << RTC_ICSR_BCDU_Pos)            /*!< 0x00000400 */
#define RTC_ICSR_BCDU_1                     (0x2UL << RTC_ICSR_BCDU_Pos)            /*!< 0x00000800 */
#define RTC_ICSR_BCDU_2                     (0x4UL << RTC_ICSR_BCDU_Pos)            /*!< 0x00001000 */
#define RTC_ICSR_RECALPF_Pos                (16UL)
#define RTC_ICSR_RECALPF_Msk                (0x1UL << RTC_ICSR_RECALPF_Pos)         /*!< 0x00010000 */
#define RTC_ICSR_RECALPF                    RTC_ICSR_RECALPF_Msk

/********************  Bits definition for RTC_PRER register  *****************/
#define RTC_PRER_PREDIV_S_Pos               (0UL)
#define RTC_PRER_PREDIV_S_Msk               (0x7FFFUL << RTC_PRER_PREDIV_S_Pos)     /*!< 0x00007FFF */
#define RTC_PRER_PREDIV_S                   RTC_PRER_PREDIV_S_Msk
#define RTC_PRER_PREDIV_A_Pos               (16UL)
#define RTC_PRER_PREDIV_A_Msk               (0x7FUL << RTC_PRER_PREDIV_A_Pos)       /*!< 0x007F0000 */
#define RTC_PRER_PREDIV_A                   RTC_PRER_PREDIV_A_Msk

/********************  Bits definition for RTC_WUTR register  *****************/
#define RTC_WUTR_WUT_Pos                    (0UL)
#define RTC_WUTR_WUT_Msk                    (0xFFFFUL << RTC_WUTR_WUT_Pos)          /*!< 0x0000FFFF */
#define RTC_WUTR_WUT                        RTC_WUTR_WUT_Msk
#define RTC_WUTR_WUTOCLR_Pos                (16UL)
#define RTC_WUTR_WUTOCLR_Msk                (0xFFFFUL << RTC_WUTR_WUTOCLR_Pos)      /*!< 0x0000FFFF */
#define RTC_WUTR_WUTOCLR                    RTC_WUTR_WUTOCLR_Msk

/********************  Bits definition for RTC_CR register  *******************/
#define RTC_CR_WUCKSEL_Pos                  (0UL)
#define RTC_CR_WUCKSEL_Msk                  (0x7UL << RTC_CR_WUCKSEL_Pos)           /*!< 0x00000007 */
#define RTC_CR_WUCKSEL                      RTC_CR_WUCKSEL_Msk
#define RTC_CR_WUCKSEL_0                    (0x1UL << RTC_CR_WUCKSEL_Pos)           /*!< 0x00000001 */
#define RTC_CR_WUCKSEL_1                    (0x2UL << RTC_CR_WUCKSEL_Pos)           /*!< 0x00000002 */
#define RTC_CR_WUCKSEL_2                    (0x4UL << RTC_CR_WUCKSEL_Pos)           /*!< 0x00000004 */
#define RTC_CR_TSEDGE_Pos                   (3UL)
#define RTC_CR_TSEDGE_Msk                   (0x1UL << RTC_CR_TSEDGE_Pos)            /*!< 0x00000008 */
#define RTC_CR_TSEDGE                       RTC_CR_TSEDGE_Msk
#define RTC_CR_REFCKON_Pos                  (4UL)
#define RTC_CR_REFCKON_Msk                  (0x1UL << RTC_CR_REFCKON_Pos)           /*!< 0x00000010 */
#define RTC_CR_REFCKON                      RTC_CR_REFCKON_Msk
#define RTC_CR_BYPSHAD_Pos                  (5UL)
#define RTC_CR_BYPSHAD_Msk                  (0x1UL << RTC_CR_BYPSHAD_Pos)           /*!< 0x00000020 */
#define RTC_CR_BYPSHAD                      RTC_CR_BYPSHAD_Msk
#define RTC_CR_FMT_Pos                      (6UL)
#define RTC_CR_FMT_Msk                      (0x1UL << RTC_CR_FMT_Pos)               /*!< 0x00000040 */
#define RTC_CR_FMT                          RTC_CR_FMT_Msk
#define RTC_CR_SSRUIE_Pos                   (7UL)
#define RTC_CR_SSRUIE_Msk                   (0x1UL << RTC_CR_SSRUIE_Pos)            /*!< 0x00000080 */
#define RTC_CR_SSRUIE                       RTC_CR_SSRUIE_Msk
#define RTC_CR_ALRAE_Pos                    (8UL)
#define RTC_CR_ALRAE_Msk                    (0x1UL << RTC_CR_ALRAE_Pos)             /*!< 0x00000100 */
#define RTC_CR_ALRAE                        RTC_CR_ALRAE_Msk
#define RTC_CR_ALRBE_Pos                    (9UL)
#define RTC_CR_ALRBE_Msk                    (0x1UL << RTC_CR_ALRBE_Pos)             /*!< 0x00000200 */
#define RTC_CR_ALRBE                        RTC_CR_ALRBE_Msk
#define RTC_CR_WUTE_Pos                     (10UL)
#define RTC_CR_WUTE_Msk                     (0x1UL << RTC_CR_WUTE_Pos)              /*!< 0x00000400 */
#define RTC_CR_WUTE                         RTC_CR_WUTE_Msk
#define RTC_CR_TSE_Pos                      (11UL)
#define RTC_CR_TSE_Msk                      (0x1UL << RTC_CR_TSE_Pos)               /*!< 0x00000800 */
#define RTC_CR_TSE                          RTC_CR_TSE_Msk
#define RTC_CR_ALRAIE_Pos                   (12UL)
#define RTC_CR_ALRAIE_Msk                   (0x1UL << RTC_CR_ALRAIE_Pos)            /*!< 0x00001000 */
#define RTC_CR_ALRAIE                       RTC_CR_ALRAIE_Msk
#define RTC_CR_ALRBIE_Pos                   (13UL)
#define RTC_CR_ALRBIE_Msk                   (0x1UL << RTC_CR_ALRBIE_Pos)            /*!< 0x00002000 */
#define RTC_CR_ALRBIE                       RTC_CR_ALRBIE_Msk
#define RTC_CR_WUTIE_Pos                    (14UL)
#define RTC_CR_WUTIE_Msk                    (0x1UL << RTC_CR_WUTIE_Pos)             /*!< 0x00004000 */
#define RTC_CR_WUTIE                        RTC_CR_WUTIE_Msk
#define RTC_CR_TSIE_Pos                     (15UL)
#define RTC_CR_TSIE_Msk                     (0x1UL << RTC_CR_TSIE_Pos)              /*!< 0x00008000 */
#define RTC_CR_TSIE                         RTC_CR_TSIE_Msk
#define RTC_CR_ADD1H_Pos                    (16UL)
#define RTC_CR_ADD1H_Msk                    (0x1UL << RTC_CR_ADD1H_Pos)             /*!< 0x00010000 */
#define RTC_CR_ADD1H                        RTC_CR_ADD1H_Msk
#define RTC_CR_SUB1H_Pos                    (17UL)
#define RTC_CR_SUB1H_Msk                    (0x1UL << RTC_CR_SUB1H_Pos)             /*!< 0x00020000 */
#define RTC_CR_SUB1H                        RTC_CR_SUB1H_Msk
#define RTC_CR_BKP_Pos                      (18UL)
#define RTC_CR_BKP_Msk                      (0x1UL << RTC_CR_BKP_Pos)               /*!< 0x00040000 */
#define RTC_CR_BKP                          RTC_CR_BKP_Msk
#define RTC_CR_COSEL_Pos                    (19UL)
#define RTC_CR_COSEL_Msk                    (0x1UL << RTC_CR_COSEL_Pos)             /*!< 0x00080000 */
#define RTC_CR_COSEL                        RTC_CR_COSEL_Msk
#define RTC_CR_POL_Pos                      (20UL)
#define RTC_CR_POL_Msk                      (0x1UL << RTC_CR_POL_Pos)               /*!< 0x00100000 */
#define RTC_CR_POL                          RTC_CR_POL_Msk
#define RTC_CR_OSEL_Pos                     (21UL)
#define RTC_CR_OSEL_Msk                     (0x3UL << RTC_CR_OSEL_Pos)              /*!< 0x00600000 */
#define RTC_CR_OSEL                         RTC_CR_OSEL_Msk
#define RTC_CR_OSEL_0                       (0x1UL << RTC_CR_OSEL_Pos)              /*!< 0x00200000 */
#define RTC_CR_OSEL_1                       (0x2UL << RTC_CR_OSEL_Pos)              /*!< 0x00400000 */
#define RTC_CR_COE_Pos                      (23UL)
#define RTC_CR_COE_Msk                      (0x1UL << RTC_CR_COE_Pos)               /*!< 0x00800000 */
#define RTC_CR_COE                          RTC_CR_COE_Msk
#define RTC_CR_ITSE_Pos                     (24UL)
#define RTC_CR_ITSE_Msk                     (0x1UL << RTC_CR_ITSE_Pos)              /*!< 0x01000000 */
#define RTC_CR_ITSE                         RTC_CR_ITSE_Msk                         /*!<Timestamp on internal event enable  */
#define RTC_CR_TAMPTS_Pos                   (25UL)
#define RTC_CR_TAMPTS_Msk                   (0x1UL << RTC_CR_TAMPTS_Pos)            /*!< 0x02000000 */
#define RTC_CR_TAMPTS                       RTC_CR_TAMPTS_Msk                       /*!<Activate timestamp on tamper detection event  */
#define RTC_CR_TAMPOE_Pos                   (26UL)
#define RTC_CR_TAMPOE_Msk                   (0x1UL << RTC_CR_TAMPOE_Pos)            /*!< 0x04000000 */
#define RTC_CR_TAMPOE                       RTC_CR_TAMPOE_Msk                       /*!<Tamper detection output enable on TAMPALARM  */
#define RTC_CR_ALRAFCLR_Pos                 (27UL)
#define RTC_CR_ALRAFCLR_Msk                 (0x1UL << RTC_CR_ALRAFCLR_Pos)          /*!< 0x8000000 */
#define RTC_CR_ALRAFCLR                     RTC_CR_ALRAFCLR_Msk                     /*!<Alarm A mask */
#define RTC_CR_ALRBFCLR_Pos                 (28UL)
#define RTC_CR_ALRBFCLR_Msk                 (0x1UL << RTC_CR_ALRBFCLR_Pos)          /*!< 0x10000000 */
#define RTC_CR_ALRBFCLR                     RTC_CR_ALRBFCLR_Msk                     /*!<Alarm B mask */
#define RTC_CR_TAMPALRM_PU_Pos              (29UL)
#define RTC_CR_TAMPALRM_PU_Msk              (0x1UL << RTC_CR_TAMPALRM_PU_Pos)       /*!< 0x20000000 */
#define RTC_CR_TAMPALRM_PU                  RTC_CR_TAMPALRM_PU_Msk                  /*!<TAMPALARM output pull-up config */
#define RTC_CR_TAMPALRM_TYPE_Pos            (30UL)
#define RTC_CR_TAMPALRM_TYPE_Msk            (0x1UL << RTC_CR_TAMPALRM_TYPE_Pos)     /*!< 0x40000000 */
#define RTC_CR_TAMPALRM_TYPE                RTC_CR_TAMPALRM_TYPE_Msk                /*!<TAMPALARM output type  */
#define RTC_CR_OUT2EN_Pos                   (31UL)
#define RTC_CR_OUT2EN_Msk                   (0x1UL << RTC_CR_OUT2EN_Pos)            /*!< 0x80000000 */
#define RTC_CR_OUT2EN                       RTC_CR_OUT2EN_Msk                       /*!<RTC_OUT2 output enable */

/********************  Bits definition for RTC_WPR register  ******************/
#define RTC_WPR_KEY_Pos                     (0UL)
#define RTC_WPR_KEY_Msk                     (0xFFUL << RTC_WPR_KEY_Pos)             /*!< 0x000000FF */
#define RTC_WPR_KEY                         RTC_WPR_KEY_Msk

/********************  Bits definition for RTC_CALR register  *****************/
#define RTC_CALR_CALM_Pos                   (0UL)
#define RTC_CALR_CALM_Msk                   (0x1FFUL << RTC_CALR_CALM_Pos)          /*!< 0x000001FF */
#define RTC_CALR_CALM                       RTC_CALR_CALM_Msk
#define RTC_CALR_CALM_0                     (0x001UL << RTC_CALR_CALM_Pos)          /*!< 0x00000001 */
#define RTC_CALR_CALM_1                     (0x002UL << RTC_CALR_CALM_Pos)          /*!< 0x00000002 */
#define RTC_CALR_CALM_2                     (0x004UL << RTC_CALR_CALM_Pos)          /*!< 0x00000004 */
#define RTC_CALR_CALM_3                     (0x008UL << RTC_CALR_CALM_Pos)          /*!< 0x00000008 */
#define RTC_CALR_CALM_4                     (0x010UL << RTC_CALR_CALM_Pos)          /*!< 0x00000010 */
#define RTC_CALR_CALM_5                     (0x020UL << RTC_CALR_CALM_Pos)          /*!< 0x00000020 */
#define RTC_CALR_CALM_6                     (0x040UL << RTC_CALR_CALM_Pos)          /*!< 0x00000040 */
#define RTC_CALR_CALM_7                     (0x080UL << RTC_CALR_CALM_Pos)          /*!< 0x00000080 */
#define RTC_CALR_CALM_8                     (0x100UL << RTC_CALR_CALM_Pos)          /*!< 0x00000100 */
#define RTC_CALR_LPCAL_Pos                  (12UL)
#define RTC_CALR_LPCAL_Msk                  (0x1UL << RTC_CALR_LPCAL_Pos)           /*!< 0x00001000 */
#define RTC_CALR_LPCAL                      RTC_CALR_LPCAL_Msk
#define RTC_CALR_CALW16_Pos                 (13UL)
#define RTC_CALR_CALW16_Msk                 (0x1UL << RTC_CALR_CALW16_Pos)          /*!< 0x00002000 */
#define RTC_CALR_CALW16                     RTC_CALR_CALW16_Msk
#define RTC_CALR_CALW8_Pos                  (14UL)
#define RTC_CALR_CALW8_Msk                  (0x1UL << RTC_CALR_CALW8_Pos)           /*!< 0x00004000 */
#define RTC_CALR_CALW8                      RTC_CALR_CALW8_Msk
#define RTC_CALR_CALP_Pos                   (15UL)
#define RTC_CALR_CALP_Msk                   (0x1UL << RTC_CALR_CALP_Pos)            /*!< 0x00008000 */
#define RTC_CALR_CALP                       RTC_CALR_CALP_Msk

/********************  Bits definition for RTC_SHIFTR register  ***************/
#define RTC_SHIFTR_SUBFS_Pos                (0UL)
#define RTC_SHIFTR_SUBFS_Msk                (0x7FFFUL << RTC_SHIFTR_SUBFS_Pos)      /*!< 0x00007FFF */
#define RTC_SHIFTR_SUBFS                    RTC_SHIFTR_SUBFS_Msk
#define RTC_SHIFTR_ADD1S_Pos                (31UL)
#define RTC_SHIFTR_ADD1S_Msk                (0x1UL << RTC_SHIFTR_ADD1S_Pos)         /*!< 0x80000000 */
#define RTC_SHIFTR_ADD1S                    RTC_SHIFTR_ADD1S_Msk

/********************  Bits definition for RTC_TSTR register  *****************/
#define RTC_TSTR_SU_Pos                     (0UL)
#define RTC_TSTR_SU_Msk                     (0xFUL << RTC_TSTR_SU_Pos)              /*!< 0x0000000F */
#define RTC_TSTR_SU                         RTC_TSTR_SU_Msk
#define RTC_TSTR_SU_0                       (0x1UL << RTC_TSTR_SU_Pos)              /*!< 0x00000001 */
#define RTC_TSTR_SU_1                       (0x2UL << RTC_TSTR_SU_Pos)              /*!< 0x00000002 */
#define RTC_TSTR_SU_2                       (0x4UL << RTC_TSTR_SU_Pos)              /*!< 0x00000004 */
#define RTC_TSTR_SU_3                       (0x8UL << RTC_TSTR_SU_Pos)              /*!< 0x00000008 */
#define RTC_TSTR_ST_Pos                     (4UL)
#define RTC_TSTR_ST_Msk                     (0x7UL << RTC_TSTR_ST_Pos)              /*!< 0x00000070 */
#define RTC_TSTR_ST                         RTC_TSTR_ST_Msk
#define RTC_TSTR_ST_0                       (0x1UL << RTC_TSTR_ST_Pos)              /*!< 0x00000010 */
#define RTC_TSTR_ST_1                       (0x2UL << RTC_TSTR_ST_Pos)              /*!< 0x00000020 */
#define RTC_TSTR_ST_2                       (0x4UL << RTC_TSTR_ST_Pos)              /*!< 0x00000040 */
#define RTC_TSTR_MNU_Pos                    (8UL)
#define RTC_TSTR_MNU_Msk                    (0xFUL << RTC_TSTR_MNU_Pos)             /*!< 0x00000F00 */
#define RTC_TSTR_MNU                        RTC_TSTR_MNU_Msk
#define RTC_TSTR_MNU_0                      (0x1UL << RTC_TSTR_MNU_Pos)             /*!< 0x00000100 */
#define RTC_TSTR_MNU_1                      (0x2UL << RTC_TSTR_MNU_Pos)             /*!< 0x00000200 */
#define RTC_TSTR_MNU_2                      (0x4UL << RTC_TSTR_MNU_Pos)             /*!< 0x00000400 */
#define RTC_TSTR_MNU_3                      (0x8UL << RTC_TSTR_MNU_Pos)             /*!< 0x00000800 */
#define RTC_TSTR_MNT_Pos                    (12UL)
#define RTC_TSTR_MNT_Msk                    (0x7UL << RTC_TSTR_MNT_Pos)             /*!< 0x00007000 */
#define RTC_TSTR_MNT                        RTC_TSTR_MNT_Msk
#define RTC_TSTR_MNT_0                      (0x1UL << RTC_TSTR_MNT_Pos)             /*!< 0x00001000 */
#define RTC_TSTR_MNT_1                      (0x2UL << RTC_TSTR_MNT_Pos)             /*!< 0x00002000 */
#define RTC_TSTR_MNT_2                      (0x4UL << RTC_TSTR_MNT_Pos)             /*!< 0x00004000 */
#define RTC_TSTR_HU_Pos                     (16UL)
#define RTC_TSTR_HU_Msk                     (0xFUL << RTC_TSTR_HU_Pos)              /*!< 0x000F0000 */
#define RTC_TSTR_HU                         RTC_TSTR_HU_Msk
#define RTC_TSTR_HU_0                       (0x1UL << RTC_TSTR_HU_Pos)              /*!< 0x00010000 */
#define RTC_TSTR_HU_1                       (0x2UL << RTC_TSTR_HU_Pos)              /*!< 0x00020000 */
#define RTC_TSTR_HU_2                       (0x4UL << RTC_TSTR_HU_Pos)              /*!< 0x00040000 */
#define RTC_TSTR_HU_3                       (0x8UL << RTC_TSTR_HU_Pos)              /*!< 0x00080000 */
#define RTC_TSTR_HT_Pos                     (20UL)
#define RTC_TSTR_HT_Msk                     (0x3UL << RTC_TSTR_HT_Pos)              /*!< 0x00300000 */
#define RTC_TSTR_HT                         RTC_TSTR_HT_Msk
#define RTC_TSTR_HT_0                       (0x1UL << RTC_TSTR_HT_Pos)              /*!< 0x00100000 */
#define RTC_TSTR_HT_1                       (0x2UL << RTC_TSTR_HT_Pos)              /*!< 0x00200000 */
#define RTC_TSTR_PM_Pos                     (22UL)
#define RTC_TSTR_PM_Msk                     (0x1UL << RTC_TSTR_PM_Pos)              /*!< 0x00400000 */
#define RTC_TSTR_PM                         RTC_TSTR_PM_Msk

/********************  Bits definition for RTC_TSDR register  *****************/
#define RTC_TSDR_DU_Pos                     (0UL)
#define RTC_TSDR_DU_Msk                     (0xFUL << RTC_TSDR_DU_Pos)              /*!< 0x0000000F */
#define RTC_TSDR_DU                         RTC_TSDR_DU_Msk
#define RTC_TSDR_DU_0                       (0x1UL << RTC_TSDR_DU_Pos)              /*!< 0x00000001 */
#define RTC_TSDR_DU_1                       (0x2UL << RTC_TSDR_DU_Pos)              /*!< 0x00000002 */
#define RTC_TSDR_DU_2                       (0x4UL << RTC_TSDR_DU_Pos)              /*!< 0x00000004 */
#define RTC_TSDR_DU_3                       (0x8UL << RTC_TSDR_DU_Pos)              /*!< 0x00000008 */
#define RTC_TSDR_DT_Pos                     (4UL)
#define RTC_TSDR_DT_Msk                     (0x3UL << RTC_TSDR_DT_Pos)              /*!< 0x00000030 */
#define RTC_TSDR_DT                         RTC_TSDR_DT_Msk
#define RTC_TSDR_DT_0                       (0x1UL << RTC_TSDR_DT_Pos)              /*!< 0x00000010 */
#define RTC_TSDR_DT_1                       (0x2UL << RTC_TSDR_DT_Pos)              /*!< 0x00000020 */
#define RTC_TSDR_MU_Pos                     (8UL)
#define RTC_TSDR_MU_Msk                     (0xFUL << RTC_TSDR_MU_Pos)              /*!< 0x00000F00 */
#define RTC_TSDR_MU                         RTC_TSDR_MU_Msk
#define RTC_TSDR_MU_0                       (0x1UL << RTC_TSDR_MU_Pos)              /*!< 0x00000100 */
#define RTC_TSDR_MU_1                       (0x2UL << RTC_TSDR_MU_Pos)              /*!< 0x00000200 */
#define RTC_TSDR_MU_2                       (0x4UL << RTC_TSDR_MU_Pos)              /*!< 0x00000400 */
#define RTC_TSDR_MU_3                       (0x8UL << RTC_TSDR_MU_Pos)              /*!< 0x00000800 */
#define RTC_TSDR_MT_Pos                     (12UL)
#define RTC_TSDR_MT_Msk                     (0x1UL << RTC_TSDR_MT_Pos)              /*!< 0x00001000 */
#define RTC_TSDR_MT                         RTC_TSDR_MT_Msk
#define RTC_TSDR_WDU_Pos                    (13UL)
#define RTC_TSDR_WDU_Msk                    (0x7UL << RTC_TSDR_WDU_Pos)             /*!< 0x0000E000 */
#define RTC_TSDR_WDU                        RTC_TSDR_WDU_Msk
#define RTC_TSDR_WDU_0                      (0x1UL << RTC_TSDR_WDU_Pos)             /*!< 0x00002000 */
#define RTC_TSDR_WDU_1                      (0x2UL << RTC_TSDR_WDU_Pos)             /*!< 0x00004000 */
#define RTC_TSDR_WDU_2                      (0x4UL << RTC_TSDR_WDU_Pos)             /*!< 0x00008000 */

/********************  Bits definition for RTC_TSSSR register  ****************/
#define RTC_TSSSR_SS_Pos                    (0UL)
#define RTC_TSSSR_SS_Msk                    (0xFFFFFFFFUL << RTC_TSSSR_SS_Pos)      /*!< 0xFFFFFFFF */
#define RTC_TSSSR_SS                        RTC_TSSSR_SS_Msk                        /*!< rtc timestamp sub second > */

/********************  Bits definition for RTC_ALRMAR register  ***************/
#define RTC_ALRMAR_SU_Pos                   (0UL)
#define RTC_ALRMAR_SU_Msk                   (0xFUL << RTC_ALRMAR_SU_Pos)            /*!< 0x0000000F */
#define RTC_ALRMAR_SU                       RTC_ALRMAR_SU_Msk
#define RTC_ALRMAR_SU_0                     (0x1UL << RTC_ALRMAR_SU_Pos)            /*!< 0x00000001 */
#define RTC_ALRMAR_SU_1                     (0x2UL << RTC_ALRMAR_SU_Pos)            /*!< 0x00000002 */
#define RTC_ALRMAR_SU_2                     (0x4UL << RTC_ALRMAR_SU_Pos)            /*!< 0x00000004 */
#define RTC_ALRMAR_SU_3                     (0x8UL << RTC_ALRMAR_SU_Pos)            /*!< 0x00000008 */
#define RTC_ALRMAR_ST_Pos                   (4UL)
#define RTC_ALRMAR_ST_Msk                   (0x7UL << RTC_ALRMAR_ST_Pos)            /*!< 0x00000070 */
#define RTC_ALRMAR_ST                       RTC_ALRMAR_ST_Msk
#define RTC_ALRMAR_ST_0                     (0x1UL << RTC_ALRMAR_ST_Pos)            /*!< 0x00000010 */
#define RTC_ALRMAR_ST_1                     (0x2UL << RTC_ALRMAR_ST_Pos)            /*!< 0x00000020 */
#define RTC_ALRMAR_ST_2                     (0x4UL << RTC_ALRMAR_ST_Pos)            /*!< 0x00000040 */
#define RTC_ALRMAR_MSK1_Pos                 (7UL)
#define RTC_ALRMAR_MSK1_Msk                 (0x1UL << RTC_ALRMAR_MSK1_Pos)          /*!< 0x00000080 */
#define RTC_ALRMAR_MSK1                     RTC_ALRMAR_MSK1_Msk
#define RTC_ALRMAR_MNU_Pos                  (8UL)
#define RTC_ALRMAR_MNU_Msk                  (0xFUL << RTC_ALRMAR_MNU_Pos)           /*!< 0x00000F00 */
#define RTC_ALRMAR_MNU                      RTC_ALRMAR_MNU_Msk
#define RTC_ALRMAR_MNU_0                    (0x1UL << RTC_ALRMAR_MNU_Pos)           /*!< 0x00000100 */
#define RTC_ALRMAR_MNU_1                    (0x2UL << RTC_ALRMAR_MNU_Pos)           /*!< 0x00000200 */
#define RTC_ALRMAR_MNU_2                    (0x4UL << RTC_ALRMAR_MNU_Pos)           /*!< 0x00000400 */
#define RTC_ALRMAR_MNU_3                    (0x8UL << RTC_ALRMAR_MNU_Pos)           /*!< 0x00000800 */
#define RTC_ALRMAR_MNT_Pos                  (12UL)
#define RTC_ALRMAR_MNT_Msk                  (0x7UL << RTC_ALRMAR_MNT_Pos)           /*!< 0x00007000 */
#define RTC_ALRMAR_MNT                      RTC_ALRMAR_MNT_Msk
#define RTC_ALRMAR_MNT_0                    (0x1UL << RTC_ALRMAR_MNT_Pos)           /*!< 0x00001000 */
#define RTC_ALRMAR_MNT_1                    (0x2UL << RTC_ALRMAR_MNT_Pos)           /*!< 0x00002000 */
#define RTC_ALRMAR_MNT_2                    (0x4UL << RTC_ALRMAR_MNT_Pos)           /*!< 0x00004000 */
#define RTC_ALRMAR_MSK2_Pos                 (15UL)
#define RTC_ALRMAR_MSK2_Msk                 (0x1UL << RTC_ALRMAR_MSK2_Pos)          /*!< 0x00008000 */
#define RTC_ALRMAR_MSK2                     RTC_ALRMAR_MSK2_Msk
#define RTC_ALRMAR_HU_Pos                   (16UL)
#define RTC_ALRMAR_HU_Msk                   (0xFUL << RTC_ALRMAR_HU_Pos)            /*!< 0x000F0000 */
#define RTC_ALRMAR_HU                       RTC_ALRMAR_HU_Msk
#define RTC_ALRMAR_HU_0                     (0x1UL << RTC_ALRMAR_HU_Pos)            /*!< 0x00010000 */
#define RTC_ALRMAR_HU_1                     (0x2UL << RTC_ALRMAR_HU_Pos)            /*!< 0x00020000 */
#define RTC_ALRMAR_HU_2                     (0x4UL << RTC_ALRMAR_HU_Pos)            /*!< 0x00040000 */
#define RTC_ALRMAR_HU_3                     (0x8UL << RTC_ALRMAR_HU_Pos)            /*!< 0x00080000 */
#define RTC_ALRMAR_HT_Pos                   (20UL)
#define RTC_ALRMAR_HT_Msk                   (0x3UL << RTC_ALRMAR_HT_Pos)            /*!< 0x00300000 */
#define RTC_ALRMAR_HT                       RTC_ALRMAR_HT_Msk
#define RTC_ALRMAR_HT_0                     (0x1UL << RTC_ALRMAR_HT_Pos)            /*!< 0x00100000 */
#define RTC_ALRMAR_HT_1                     (0x2UL << RTC_ALRMAR_HT_Pos)            /*!< 0x00200000 */
#define RTC_ALRMAR_PM_Pos                   (22UL)
#define RTC_ALRMAR_PM_Msk                   (0x1UL << RTC_ALRMAR_PM_Pos)            /*!< 0x00400000 */
#define RTC_ALRMAR_PM                       RTC_ALRMAR_PM_Msk
#define RTC_ALRMAR_MSK3_Pos                 (23UL)
#define RTC_ALRMAR_MSK3_Msk                 (0x1UL << RTC_ALRMAR_MSK3_Pos)          /*!< 0x00800000 */
#define RTC_ALRMAR_MSK3                     RTC_ALRMAR_MSK3_Msk
#define RTC_ALRMAR_DU_Pos                   (24UL)
#define RTC_ALRMAR_DU_Msk                   (0xFUL << RTC_ALRMAR_DU_Pos)            /*!< 0x0F000000 */
#define RTC_ALRMAR_DU                       RTC_ALRMAR_DU_Msk
#define RTC_ALRMAR_DU_0                     (0x1UL << RTC_ALRMAR_DU_Pos)            /*!< 0x01000000 */
#define RTC_ALRMAR_DU_1                     (0x2UL << RTC_ALRMAR_DU_Pos)            /*!< 0x02000000 */
#define RTC_ALRMAR_DU_2                     (0x4UL << RTC_ALRMAR_DU_Pos)            /*!< 0x04000000 */
#define RTC_ALRMAR_DU_3                     (0x8UL << RTC_ALRMAR_DU_Pos)            /*!< 0x08000000 */
#define RTC_ALRMAR_DT_Pos                   (28UL)
#define RTC_ALRMAR_DT_Msk                   (0x3UL << RTC_ALRMAR_DT_Pos)            /*!< 0x30000000 */
#define RTC_ALRMAR_DT                       RTC_ALRMAR_DT_Msk
#define RTC_ALRMAR_DT_0                     (0x1UL << RTC_ALRMAR_DT_Pos)            /*!< 0x10000000 */
#define RTC_ALRMAR_DT_1                     (0x2UL << RTC_ALRMAR_DT_Pos)            /*!< 0x20000000 */
#define RTC_ALRMAR_WDSEL_Pos                (30UL)
#define RTC_ALRMAR_WDSEL_Msk                (0x1UL << RTC_ALRMAR_WDSEL_Pos)         /*!< 0x40000000 */
#define RTC_ALRMAR_WDSEL                    RTC_ALRMAR_WDSEL_Msk
#define RTC_ALRMAR_MSK4_Pos                 (31UL)
#define RTC_ALRMAR_MSK4_Msk                 (0x1UL << RTC_ALRMAR_MSK4_Pos)          /*!< 0x80000000 */
#define RTC_ALRMAR_MSK4                     RTC_ALRMAR_MSK4_Msk

/********************  Bits definition for RTC_ALRMASSR register  *************/
#define RTC_ALRMASSR_SS_Pos                 (0UL)
#define RTC_ALRMASSR_SS_Msk                 (0x7FFFUL << RTC_ALRMASSR_SS_Pos)       /*!< 0x00007FFF */
#define RTC_ALRMASSR_SS                     RTC_ALRMASSR_SS_Msk
#define RTC_ALRMASSR_MASKSS_Pos             (24UL)
#define RTC_ALRMASSR_MASKSS_Msk             (0x3FUL << RTC_ALRMASSR_MASKSS_Pos)     /*!< 0x3F000000 */
#define RTC_ALRMASSR_MASKSS                 RTC_ALRMASSR_MASKSS_Msk
#define RTC_ALRMASSR_MASKSS_0               (0x1UL << RTC_ALRMASSR_MASKSS_Pos)      /*!< 0x01000000 */
#define RTC_ALRMASSR_MASKSS_1               (0x2UL << RTC_ALRMASSR_MASKSS_Pos)      /*!< 0x02000000 */
#define RTC_ALRMASSR_MASKSS_2               (0x4UL << RTC_ALRMASSR_MASKSS_Pos)      /*!< 0x04000000 */
#define RTC_ALRMASSR_MASKSS_3               (0x8UL << RTC_ALRMASSR_MASKSS_Pos)      /*!< 0x08000000 */
#define RTC_ALRMASSR_MASKSS_4               (0x10UL << RTC_ALRMASSR_MASKSS_Pos)     /*!< 0x10000000 */
#define RTC_ALRMASSR_MASKSS_5               (0x20UL << RTC_ALRMASSR_MASKSS_Pos)     /*!< 0x20000000 */
#define RTC_ALRMASSR_SSCLR_Pos              (31UL)
#define RTC_ALRMASSR_SSCLR_Msk              (0x1UL << RTC_ALRMASSR_SSCLR_Pos)       /*!< 0x80000000 */
#define RTC_ALRMASSR_SSCLR                  RTC_ALRMASSR_SSCLR_Msk

/********************  Bits definition for RTC_ALRMBR register  ***************/
#define RTC_ALRMBR_SU_Pos                   (0UL)
#define RTC_ALRMBR_SU_Msk                   (0xFUL << RTC_ALRMBR_SU_Pos)            /*!< 0x0000000F */
#define RTC_ALRMBR_SU                       RTC_ALRMBR_SU_Msk
#define RTC_ALRMBR_SU_0                     (0x1UL << RTC_ALRMBR_SU_Pos)            /*!< 0x00000001 */
#define RTC_ALRMBR_SU_1                     (0x2UL << RTC_ALRMBR_SU_Pos)            /*!< 0x00000002 */
#define RTC_ALRMBR_SU_2                     (0x4UL << RTC_ALRMBR_SU_Pos)            /*!< 0x00000004 */
#define RTC_ALRMBR_SU_3                     (0x8UL << RTC_ALRMBR_SU_Pos)            /*!< 0x00000008 */
#define RTC_ALRMBR_ST_Pos                   (4UL)
#define RTC_ALRMBR_ST_Msk                   (0x7UL << RTC_ALRMBR_ST_Pos)            /*!< 0x00000070 */
#define RTC_ALRMBR_ST                       RTC_ALRMBR_ST_Msk
#define RTC_ALRMBR_ST_0                     (0x1UL << RTC_ALRMBR_ST_Pos)            /*!< 0x00000010 */
#define RTC_ALRMBR_ST_1                     (0x2UL << RTC_ALRMBR_ST_Pos)            /*!< 0x00000020 */
#define RTC_ALRMBR_ST_2                     (0x4UL << RTC_ALRMBR_ST_Pos)            /*!< 0x00000040 */
#define RTC_ALRMBR_MSK1_Pos                 (7UL)
#define RTC_ALRMBR_MSK1_Msk                 (0x1UL << RTC_ALRMBR_MSK1_Pos)          /*!< 0x00000080 */
#define RTC_ALRMBR_MSK1                     RTC_ALRMBR_MSK1_Msk
#define RTC_ALRMBR_MNU_Pos                  (8UL)
#define RTC_ALRMBR_MNU_Msk                  (0xFUL << RTC_ALRMBR_MNU_Pos)           /*!< 0x00000F00 */
#define RTC_ALRMBR_MNU                      RTC_ALRMBR_MNU_Msk
#define RTC_ALRMBR_MNU_0                    (0x1UL << RTC_ALRMBR_MNU_Pos)           /*!< 0x00000100 */
#define RTC_ALRMBR_MNU_1                    (0x2UL << RTC_ALRMBR_MNU_Pos)           /*!< 0x00000200 */
#define RTC_ALRMBR_MNU_2                    (0x4UL << RTC_ALRMBR_MNU_Pos)           /*!< 0x00000400 */
#define RTC_ALRMBR_MNU_3                    (0x8UL << RTC_ALRMBR_MNU_Pos)           /*!< 0x00000800 */
#define RTC_ALRMBR_MNT_Pos                  (12UL)
#define RTC_ALRMBR_MNT_Msk                  (0x7UL << RTC_ALRMBR_MNT_Pos)           /*!< 0x00007000 */
#define RTC_ALRMBR_MNT                      RTC_ALRMBR_MNT_Msk
#define RTC_ALRMBR_MNT_0                    (0x1UL << RTC_ALRMBR_MNT_Pos)           /*!< 0x00001000 */
#define RTC_ALRMBR_MNT_1                    (0x2UL << RTC_ALRMBR_MNT_Pos)           /*!< 0x00002000 */
#define RTC_ALRMBR_MNT_2                    (0x4UL << RTC_ALRMBR_MNT_Pos)           /*!< 0x00004000 */
#define RTC_ALRMBR_MSK2_Pos                 (15UL)
#define RTC_ALRMBR_MSK2_Msk                 (0x1UL << RTC_ALRMBR_MSK2_Pos)          /*!< 0x00008000 */
#define RTC_ALRMBR_MSK2                     RTC_ALRMBR_MSK2_Msk
#define RTC_ALRMBR_HU_Pos                   (16UL)
#define RTC_ALRMBR_HU_Msk                   (0xFUL << RTC_ALRMBR_HU_Pos)            /*!< 0x000F0000 */
#define RTC_ALRMBR_HU                       RTC_ALRMBR_HU_Msk
#define RTC_ALRMBR_HU_0                     (0x1UL << RTC_ALRMBR_HU_Pos)            /*!< 0x00010000 */
#define RTC_ALRMBR_HU_1                     (0x2UL << RTC_ALRMBR_HU_Pos)            /*!< 0x00020000 */
#define RTC_ALRMBR_HU_2                     (0x4UL << RTC_ALRMBR_HU_Pos)            /*!< 0x00040000 */
#define RTC_ALRMBR_HU_3                     (0x8UL << RTC_ALRMBR_HU_Pos)            /*!< 0x00080000 */
#define RTC_ALRMBR_HT_Pos                   (20UL)
#define RTC_ALRMBR_HT_Msk                   (0x3UL << RTC_ALRMBR_HT_Pos)            /*!< 0x00300000 */
#define RTC_ALRMBR_HT                       RTC_ALRMBR_HT_Msk
#define RTC_ALRMBR_HT_0                     (0x1UL << RTC_ALRMBR_HT_Pos)            /*!< 0x00100000 */
#define RTC_ALRMBR_HT_1                     (0x2UL << RTC_ALRMBR_HT_Pos)            /*!< 0x00200000 */
#define RTC_ALRMBR_PM_Pos                   (22UL)
#define RTC_ALRMBR_PM_Msk                   (0x1UL << RTC_ALRMBR_PM_Pos)            /*!< 0x00400000 */
#define RTC_ALRMBR_PM                       RTC_ALRMBR_PM_Msk
#define RTC_ALRMBR_MSK3_Pos                 (23UL)
#define RTC_ALRMBR_MSK3_Msk                 (0x1UL << RTC_ALRMBR_MSK3_Pos)          /*!< 0x00800000 */
#define RTC_ALRMBR_MSK3                     RTC_ALRMBR_MSK3_Msk
#define RTC_ALRMBR_DU_Pos                   (24UL)
#define RTC_ALRMBR_DU_Msk                   (0xFUL << RTC_ALRMBR_DU_Pos)            /*!< 0x0F000000 */
#define RTC_ALRMBR_DU                       RTC_ALRMBR_DU_Msk
#define RTC_ALRMBR_DU_0                     (0x1UL << RTC_ALRMBR_DU_Pos)            /*!< 0x01000000 */
#define RTC_ALRMBR_DU_1                     (0x2UL << RTC_ALRMBR_DU_Pos)            /*!< 0x02000000 */
#define RTC_ALRMBR_DU_2                     (0x4UL << RTC_ALRMBR_DU_Pos)            /*!< 0x04000000 */
#define RTC_ALRMBR_DU_3                     (0x8UL << RTC_ALRMBR_DU_Pos)            /*!< 0x08000000 */
#define RTC_ALRMBR_DT_Pos                   (28UL)
#define RTC_ALRMBR_DT_Msk                   (0x3UL << RTC_ALRMBR_DT_Pos)            /*!< 0x30000000 */
#define RTC_ALRMBR_DT                       RTC_ALRMBR_DT_Msk
#define RTC_ALRMBR_DT_0                     (0x1UL << RTC_ALRMBR_DT_Pos)            /*!< 0x10000000 */
#define RTC_ALRMBR_DT_1                     (0x2UL << RTC_ALRMBR_DT_Pos)            /*!< 0x20000000 */
#define RTC_ALRMBR_WDSEL_Pos                (30UL)
#define RTC_ALRMBR_WDSEL_Msk                (0x1UL << RTC_ALRMBR_WDSEL_Pos)         /*!< 0x40000000 */
#define RTC_ALRMBR_WDSEL                    RTC_ALRMBR_WDSEL_Msk
#define RTC_ALRMBR_MSK4_Pos                 (31UL)
#define RTC_ALRMBR_MSK4_Msk                 (0x1UL << RTC_ALRMBR_MSK4_Pos)          /*!< 0x80000000 */
#define RTC_ALRMBR_MSK4                     RTC_ALRMBR_MSK4_Msk

/********************  Bits definition for RTC_ALRMBSSR register  *************/
#define RTC_ALRMBSSR_SS_Pos                 (0UL)
#define RTC_ALRMBSSR_SS_Msk                 (0x7FFFUL << RTC_ALRMBSSR_SS_Pos)       /*!< 0x00007FFF */
#define RTC_ALRMBSSR_SS                     RTC_ALRMBSSR_SS_Msk
#define RTC_ALRMBSSR_MASKSS_Pos             (24UL)
#define RTC_ALRMBSSR_MASKSS_Msk             (0x3FUL << RTC_ALRMBSSR_MASKSS_Pos)     /*!< 0x3F000000 */
#define RTC_ALRMBSSR_MASKSS                 RTC_ALRMBSSR_MASKSS_Msk
#define RTC_ALRMBSSR_MASKSS_0               (0x1UL << RTC_ALRMBSSR_MASKSS_Pos)      /*!< 0x01000000 */
#define RTC_ALRMBSSR_MASKSS_1               (0x2UL << RTC_ALRMBSSR_MASKSS_Pos)      /*!< 0x02000000 */
#define RTC_ALRMBSSR_MASKSS_2               (0x4UL << RTC_ALRMBSSR_MASKSS_Pos)      /*!< 0x04000000 */
#define RTC_ALRMBSSR_MASKSS_3               (0x8UL << RTC_ALRMBSSR_MASKSS_Pos)      /*!< 0x08000000 */
#define RTC_ALRMBSSR_MASKSS_4               (0x10UL << RTC_ALRMBSSR_MASKSS_Pos)     /*!< 0x10000000 */
#define RTC_ALRMBSSR_MASKSS_5               (0x20UL << RTC_ALRMBSSR_MASKSS_Pos)     /*!< 0x20000000 */
#define RTC_ALRMBSSR_SSCLR_Pos              (31UL)
#define RTC_ALRMBSSR_SSCLR_Msk              (0x1UL << RTC_ALRMBSSR_SSCLR_Pos)       /*!< 0x80000000 */
#define RTC_ALRMBSSR_SSCLR                  RTC_ALRMBSSR_SSCLR_Msk

/********************  Bits definition for RTC_SR register  *******************/
#define RTC_SR_ALRAF_Pos                    (0UL)
#define RTC_SR_ALRAF_Msk                    (0x1UL << RTC_SR_ALRAF_Pos)             /*!< 0x00000001 */
#define RTC_SR_ALRAF                        RTC_SR_ALRAF_Msk
#define RTC_SR_ALRBF_Pos                    (1UL)
#define RTC_SR_ALRBF_Msk                    (0x1UL << RTC_SR_ALRBF_Pos)             /*!< 0x00000002 */
#define RTC_SR_ALRBF                        RTC_SR_ALRBF_Msk
#define RTC_SR_WUTF_Pos                     (2UL)
#define RTC_SR_WUTF_Msk                     (0x1UL << RTC_SR_WUTF_Pos)              /*!< 0x00000004 */
#define RTC_SR_WUTF                         RTC_SR_WUTF_Msk
#define RTC_SR_TSF_Pos                      (3UL)
#define RTC_SR_TSF_Msk                      (0x1UL << RTC_SR_TSF_Pos)               /*!< 0x00000008 */
#define RTC_SR_TSF                          RTC_SR_TSF_Msk
#define RTC_SR_TSOVF_Pos                    (4UL)
#define RTC_SR_TSOVF_Msk                    (0x1UL << RTC_SR_TSOVF_Pos)             /*!< 0x00000010 */
#define RTC_SR_TSOVF                        RTC_SR_TSOVF_Msk
#define RTC_SR_ITSF_Pos                     (5UL)
#define RTC_SR_ITSF_Msk                     (0x1UL << RTC_SR_ITSF_Pos)              /*!< 0x00000020 */
#define RTC_SR_ITSF                         RTC_SR_ITSF_Msk
#define RTC_SR_SSRUF_Pos                    (6UL)
#define RTC_SR_SSRUF_Msk                    (0x1UL << RTC_SR_SSRUF_Pos)             /*!< 0x00000040 */
#define RTC_SR_SSRUF                        RTC_SR_SSRUF_Msk

/********************  Bits definition for RTC_MISR register  *****************/
#define RTC_MISR_ALRAMF_Pos                 (0UL)
#define RTC_MISR_ALRAMF_Msk                 (0x1UL << RTC_MISR_ALRAMF_Pos)          /*!< 0x00000001 */
#define RTC_MISR_ALRAMF                     RTC_MISR_ALRAMF_Msk
#define RTC_MISR_ALRBMF_Pos                 (1UL)
#define RTC_MISR_ALRBMF_Msk                 (0x1UL << RTC_MISR_ALRBMF_Pos)          /*!< 0x00000002 */
#define RTC_MISR_ALRBMF                     RTC_MISR_ALRBMF_Msk
#define RTC_MISR_WUTMF_Pos                  (2UL)
#define RTC_MISR_WUTMF_Msk                  (0x1UL << RTC_MISR_WUTMF_Pos)           /*!< 0x00000004 */
#define RTC_MISR_WUTMF                      RTC_MISR_WUTMF_Msk
#define RTC_MISR_TSMF_Pos                   (3UL)
#define RTC_MISR_TSMF_Msk                   (0x1UL << RTC_MISR_TSMF_Pos)            /*!< 0x00000008 */
#define RTC_MISR_TSMF                       RTC_MISR_TSMF_Msk
#define RTC_MISR_TSOVMF_Pos                 (4UL)
#define RTC_MISR_TSOVMF_Msk                 (0x1UL << RTC_MISR_TSOVMF_Pos)          /*!< 0x00000010 */
#define RTC_MISR_TSOVMF                     RTC_MISR_TSOVMF_Msk
#define RTC_MISR_ITSMF_Pos                  (5UL)
#define RTC_MISR_ITSMF_Msk                  (0x1UL << RTC_MISR_ITSMF_Pos)           /*!< 0x00000020 */
#define RTC_MISR_ITSMF                      RTC_MISR_ITSMF_Msk
#define RTC_MISR_SSRUMF_Pos                 (6UL)
#define RTC_MISR_SSRUMF_Msk                 (0x1UL << RTC_MISR_SSRUMF_Pos)          /*!< 0x00000040 */
#define RTC_MISR_SSRUMF                     RTC_MISR_SSRUMF_Msk

/********************  Bits definition for RTC_SCR register  ******************/
#define RTC_SCR_CALRAF_Pos                  (0UL)
#define RTC_SCR_CALRAF_Msk                  (0x1UL << RTC_SCR_CALRAF_Pos)           /*!< 0x00000001 */
#define RTC_SCR_CALRAF                      RTC_SCR_CALRAF_Msk
#define RTC_SCR_CALRBF_Pos                  (1UL)
#define RTC_SCR_CALRBF_Msk                  (0x1UL << RTC_SCR_CALRBF_Pos)           /*!< 0x00000002 */
#define RTC_SCR_CALRBF                      RTC_SCR_CALRBF_Msk
#define RTC_SCR_CWUTF_Pos                   (2UL)
#define RTC_SCR_CWUTF_Msk                   (0x1UL << RTC_SCR_CWUTF_Pos)            /*!< 0x00000004 */
#define RTC_SCR_CWUTF                       RTC_SCR_CWUTF_Msk
#define RTC_SCR_CTSF_Pos                    (3UL)
#define RTC_SCR_CTSF_Msk                    (0x1UL << RTC_SCR_CTSF_Pos)             /*!< 0x00000008 */
#define RTC_SCR_CTSF                        RTC_SCR_CTSF_Msk
#define RTC_SCR_CTSOVF_Pos                  (4UL)
#define RTC_SCR_CTSOVF_Msk                  (0x1UL << RTC_SCR_CTSOVF_Pos)           /*!< 0x00000010 */
#define RTC_SCR_CTSOVF                      RTC_SCR_CTSOVF_Msk
#define RTC_SCR_CITSF_Pos                   (5UL)
#define RTC_SCR_CITSF_Msk                   (0x1UL << RTC_SCR_CITSF_Pos)            /*!< 0x00000020 */
#define RTC_SCR_CITSF                       RTC_SCR_CITSF_Msk
#define RTC_SCR_CSSRUF_Pos                  (6UL)
#define RTC_SCR_CSSRUF_Msk                  (0x1UL << RTC_SCR_CSSRUF_Pos)           /*!< 0x00000040 */
#define RTC_SCR_CSSRUF                      RTC_SCR_CSSRUF_Msk

/********************  Bits definition for RTC_ALRABINR register  ******************/
#define RTC_ALRABINR_SS_Pos                 (0UL)
#define RTC_ALRABINR_SS_Msk                 (0xFFFFFFFFUL << RTC_ALRABINR_SS_Pos)   /*!< 0xFFFFFFFF */
#define RTC_ALRABINR_SS                     RTC_ALRABINR_SS_Msk

/********************  Bits definition for RTC_ALRBBINR register  ******************/
#define RTC_ALRBBINR_SS_Pos                 (0UL)
#define RTC_ALRBBINR_SS_Msk                 (0xFFFFFFFFUL << RTC_ALRBBINR_SS_Pos)   /*!< 0xFFFFFFFF */
#define RTC_ALRBBINR_SS                     RTC_ALRBBINR_SS_Msk

/******************************************************************************/
/*                                                                            */
/*                     Tamper and backup register (TAMP)                      */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for TAMP_CR1 register  *****************/
#define TAMP_CR1_TAMP1E_Pos          (0UL)
#define TAMP_CR1_TAMP1E_Msk          (0x1UL << TAMP_CR1_TAMP1E_Pos)            /*!< 0x00000001 */
#define TAMP_CR1_TAMP1E              TAMP_CR1_TAMP1E_Msk
#define TAMP_CR1_TAMP2E_Pos          (1UL)
#define TAMP_CR1_TAMP2E_Msk          (0x1UL << TAMP_CR1_TAMP2E_Pos)            /*!< 0x00000002 */
#define TAMP_CR1_TAMP2E              TAMP_CR1_TAMP2E_Msk
#define TAMP_CR1_TAMP3E_Pos          (2UL)
#define TAMP_CR1_TAMP3E_Msk          (0x1UL << TAMP_CR1_TAMP3E_Pos)            /*!< 0x00000004 */
#define TAMP_CR1_TAMP3E              TAMP_CR1_TAMP3E_Msk
#define TAMP_CR1_TAMP4E_Pos          (3UL)
#define TAMP_CR1_TAMP4E_Msk          (0x1UL << TAMP_CR1_TAMP4E_Pos)            /*!< 0x00000008 */
#define TAMP_CR1_TAMP4E              TAMP_CR1_TAMP4E_Msk
#define TAMP_CR1_TAMP5E_Pos          (4UL)
#define TAMP_CR1_TAMP5E_Msk          (0x1UL << TAMP_CR1_TAMP5E_Pos)            /*!< 0x00000010 */
#define TAMP_CR1_TAMP5E              TAMP_CR1_TAMP5E_Msk
#define TAMP_CR1_ITAMP3E_Pos         (18UL)
#define TAMP_CR1_ITAMP3E_Msk         (0x1UL << TAMP_CR1_ITAMP3E_Pos)           /*!< 0x00040000 */
#define TAMP_CR1_ITAMP3E             TAMP_CR1_ITAMP3E_Msk
#define TAMP_CR1_ITAMP4E_Pos         (19UL)
#define TAMP_CR1_ITAMP4E_Msk         (0x1UL << TAMP_CR1_ITAMP4E_Pos)           /*!< 0x00080000 */
#define TAMP_CR1_ITAMP4E             TAMP_CR1_ITAMP4E_Msk
#define TAMP_CR1_ITAMP5E_Pos         (20UL)
#define TAMP_CR1_ITAMP5E_Msk         (0x1UL << TAMP_CR1_ITAMP5E_Pos)           /*!< 0x00100000 */
#define TAMP_CR1_ITAMP5E             TAMP_CR1_ITAMP5E_Msk
#define TAMP_CR1_ITAMP6E_Pos         (21UL)
#define TAMP_CR1_ITAMP6E_Msk         (0x1UL << TAMP_CR1_ITAMP6E_Pos)           /*!< 0x00200000 */
#define TAMP_CR1_ITAMP6E             TAMP_CR1_ITAMP6E_Msk

/********************  Bits definition for TAMP_CR2 register  *****************/
#define TAMP_CR2_TAMP1POM_Pos        (0UL)
#define TAMP_CR2_TAMP1POM_Msk        (0x1UL << TAMP_CR2_TAMP1POM_Pos)      /*!< 0x00000001 */
#define TAMP_CR2_TAMP1POM            TAMP_CR2_TAMP1POM_Msk
#define TAMP_CR2_TAMP2POM_Pos        (1UL)
#define TAMP_CR2_TAMP2POM_Msk        (0x1UL << TAMP_CR2_TAMP2POM_Pos)      /*!< 0x00000002 */
#define TAMP_CR2_TAMP2POM            TAMP_CR2_TAMP2POM_Msk
#define TAMP_CR2_TAMP3POM_Pos        (2UL)
#define TAMP_CR2_TAMP3POM_Msk        (0x1UL << TAMP_CR2_TAMP3POM_Pos)      /*!< 0x00000004 */
#define TAMP_CR2_TAMP3POM            TAMP_CR2_TAMP3POM_Msk
#define TAMP_CR2_TAMP4POM_Pos        (3UL)
#define TAMP_CR2_TAMP4POM_Msk        (0x1UL << TAMP_CR2_TAMP4POM_Pos)      /*!< 0x00000004 */
#define TAMP_CR2_TAMP4POM            TAMP_CR2_TAMP4POM_Msk
#define TAMP_CR2_TAMP5POM_Pos        (4UL)
#define TAMP_CR2_TAMP5POM_Msk        (0x1UL << TAMP_CR2_TAMP5POM_Pos)      /*!< 0x00000004 */
#define TAMP_CR2_TAMP5POM            TAMP_CR2_TAMP5POM_Msk
#define TAMP_CR2_TAMP1MSK_Pos        (16UL)
#define TAMP_CR2_TAMP1MSK_Msk        (0x1UL << TAMP_CR2_TAMP1MSK_Pos)          /*!< 0x00010000 */
#define TAMP_CR2_TAMP1MSK            TAMP_CR2_TAMP1MSK_Msk
#define TAMP_CR2_TAMP2MSK_Pos        (17UL)
#define TAMP_CR2_TAMP2MSK_Msk        (0x1UL << TAMP_CR2_TAMP2MSK_Pos)          /*!< 0x00020000 */
#define TAMP_CR2_TAMP2MSK            TAMP_CR2_TAMP2MSK_Msk
#define TAMP_CR2_TAMP3MSK_Pos        (18UL)
#define TAMP_CR2_TAMP3MSK_Msk        (0x1UL << TAMP_CR2_TAMP3MSK_Pos)          /*!< 0x00040000 */
#define TAMP_CR2_TAMP3MSK            TAMP_CR2_TAMP3MSK_Msk
#define TAMP_CR2_BKBLOCK_Pos         (22UL)
#define TAMP_CR2_BKBLOCK_Msk         (0x1UL << TAMP_CR2_BKBLOCK_Pos)           /*!< 0x00400000 */
#define TAMP_CR2_BKBLOCK             TAMP_CR2_BKBLOCK_Msk
#define TAMP_CR2_BKERASE_Pos         (23UL)
#define TAMP_CR2_BKERASE_Msk         (0x1UL << TAMP_CR2_BKERASE_Pos)           /*!< 0x00800000 */
#define TAMP_CR2_BKERASE             TAMP_CR2_BKERASE_Msk
#define TAMP_CR2_TAMP1TRG_Pos        (24UL)
#define TAMP_CR2_TAMP1TRG_Msk        (0x1UL << TAMP_CR2_TAMP1TRG_Pos)          /*!< 0x01000000 */
#define TAMP_CR2_TAMP1TRG            TAMP_CR2_TAMP1TRG_Msk
#define TAMP_CR2_TAMP2TRG_Pos        (25UL)
#define TAMP_CR2_TAMP2TRG_Msk        (0x1UL << TAMP_CR2_TAMP2TRG_Pos)          /*!< 0x02000000 */
#define TAMP_CR2_TAMP2TRG            TAMP_CR2_TAMP2TRG_Msk
#define TAMP_CR2_TAMP3TRG_Pos        (26UL)
#define TAMP_CR2_TAMP3TRG_Msk        (0x1UL << TAMP_CR2_TAMP3TRG_Pos)          /*!< 0x04000000 */
#define TAMP_CR2_TAMP3TRG            TAMP_CR2_TAMP3TRG_Msk
#define TAMP_CR2_TAMP4TRG_Pos        (27UL)
#define TAMP_CR2_TAMP4TRG_Msk        (0x1UL << TAMP_CR2_TAMP4TRG_Pos)          /*!< 0x04000000 */
#define TAMP_CR2_TAMP4TRG            TAMP_CR2_TAMP4TRG_Msk
#define TAMP_CR2_TAMP5TRG_Pos        (28UL)
#define TAMP_CR2_TAMP5TRG_Msk        (0x1UL << TAMP_CR2_TAMP5TRG_Pos)          /*!< 0x04000000 */
#define TAMP_CR2_TAMP5TRG            TAMP_CR2_TAMP5TRG_Msk

/********************  Bits definition for TAMP_CR3 register  *****************/
#define TAMP_CR3_ITAMP3POM_Pos             (2UL)
#define TAMP_CR3_ITAMP3POM_Msk             (0x1UL << TAMP_CR3_ITAMP3POM_Pos)      /*!< 0x00000004 */
#define TAMP_CR3_ITAMP3POM                 TAMP_CR3_ITAMP3POM_Msk
#define TAMP_CR3_ITAMP4POM_Pos             (3UL)
#define TAMP_CR3_ITAMP4POM_Msk             (0x1UL << TAMP_CR3_ITAMP4POM_Pos)      /*!< 0x00000008 */
#define TAMP_CR3_ITAMP4POM                 TAMP_CR3_ITAMP4POM_Msk
#define TAMP_CR3_ITAMP5POM_Pos             (4UL)
#define TAMP_CR3_ITAMP5POM_Msk             (0x1UL << TAMP_CR3_ITAMP5POM_Pos)      /*!< 0x00000010 */
#define TAMP_CR3_ITAMP5POM                 TAMP_CR3_ITAMP5POM_Msk
#define TAMP_CR3_ITAMP6POM_Pos             (5UL)
#define TAMP_CR3_ITAMP6POM_Msk             (0x1UL << TAMP_CR3_ITAMP6POM_Pos)      /*!< 0x00000020 */
#define TAMP_CR3_ITAMP6POM                 TAMP_CR3_ITAMP6POM_Msk

/********************  Bits definition for TAMP_FLTCR register  ***************/
#define TAMP_FLTCR_TAMPFREQ_Pos      (0UL)
#define TAMP_FLTCR_TAMPFREQ_Msk      (0x7UL << TAMP_FLTCR_TAMPFREQ_Pos)         /*!< 0x00000007 */
#define TAMP_FLTCR_TAMPFREQ          TAMP_FLTCR_TAMPFREQ_Msk
#define TAMP_FLTCR_TAMPFREQ_0        (0x1UL << TAMP_FLTCR_TAMPFREQ_Pos)         /*!< 0x00000001 */
#define TAMP_FLTCR_TAMPFREQ_1        (0x2UL << TAMP_FLTCR_TAMPFREQ_Pos)         /*!< 0x00000002 */
#define TAMP_FLTCR_TAMPFREQ_2        (0x4UL << TAMP_FLTCR_TAMPFREQ_Pos)         /*!< 0x00000004 */
#define TAMP_FLTCR_TAMPFLT_Pos       (3UL)
#define TAMP_FLTCR_TAMPFLT_Msk       (0x3UL << TAMP_FLTCR_TAMPFLT_Pos)          /*!< 0x00000018 */
#define TAMP_FLTCR_TAMPFLT           TAMP_FLTCR_TAMPFLT_Msk
#define TAMP_FLTCR_TAMPFLT_0         (0x1UL << TAMP_FLTCR_TAMPFLT_Pos)          /*!< 0x00000008 */
#define TAMP_FLTCR_TAMPFLT_1         (0x2UL << TAMP_FLTCR_TAMPFLT_Pos)          /*!< 0x00000010 */
#define TAMP_FLTCR_TAMPPRCH_Pos      (5UL)
#define TAMP_FLTCR_TAMPPRCH_Msk      (0x3UL << TAMP_FLTCR_TAMPPRCH_Pos)         /*!< 0x00000060 */
#define TAMP_FLTCR_TAMPPRCH          TAMP_FLTCR_TAMPPRCH_Msk
#define TAMP_FLTCR_TAMPPRCH_0        (0x1UL << TAMP_FLTCR_TAMPPRCH_Pos)         /*!< 0x00000020 */
#define TAMP_FLTCR_TAMPPRCH_1        (0x2UL << TAMP_FLTCR_TAMPPRCH_Pos)         /*!< 0x00000040 */
#define TAMP_FLTCR_TAMPPUDIS_Pos     (7UL)
#define TAMP_FLTCR_TAMPPUDIS_Msk     (0x1UL << TAMP_FLTCR_TAMPPUDIS_Pos)        /*!< 0x00000080 */
#define TAMP_FLTCR_TAMPPUDIS         TAMP_FLTCR_TAMPPUDIS_Msk

/********************  Bits definition for TAMP_IER register  *****************/
#define TAMP_IER_TAMP1IE_Pos         (0UL)
#define TAMP_IER_TAMP1IE_Msk         (0x1UL << TAMP_IER_TAMP1IE_Pos)           /*!< 0x00000001 */
#define TAMP_IER_TAMP1IE             TAMP_IER_TAMP1IE_Msk
#define TAMP_IER_TAMP2IE_Pos         (1UL)
#define TAMP_IER_TAMP2IE_Msk         (0x1UL << TAMP_IER_TAMP2IE_Pos)           /*!< 0x00000002 */
#define TAMP_IER_TAMP2IE             TAMP_IER_TAMP2IE_Msk
#define TAMP_IER_TAMP3IE_Pos         (2UL)
#define TAMP_IER_TAMP3IE_Msk         (0x1UL << TAMP_IER_TAMP3IE_Pos)           /*!< 0x00000004 */
#define TAMP_IER_TAMP3IE             TAMP_IER_TAMP3IE_Msk
#define TAMP_IER_TAMP4IE_Pos         (3UL)
#define TAMP_IER_TAMP4IE_Msk         (0x1UL << TAMP_IER_TAMP4IE_Pos)           /*!< 0x00000004 */
#define TAMP_IER_TAMP4IE             TAMP_IER_TAMP4IE_Msk
#define TAMP_IER_TAMP5IE_Pos         (4UL)
#define TAMP_IER_TAMP5IE_Msk         (0x1UL << TAMP_IER_TAMP5IE_Pos)           /*!< 0x00000004 */
#define TAMP_IER_TAMP5IE             TAMP_IER_TAMP5IE_Msk
#define TAMP_IER_ITAMP3IE_Pos        (18UL)
#define TAMP_IER_ITAMP3IE_Msk        (0x1UL << TAMP_IER_ITAMP3IE_Pos)          /*!< 0x00040000 */
#define TAMP_IER_ITAMP3IE            TAMP_IER_ITAMP3IE_Msk
#define TAMP_IER_ITAMP4IE_Pos        (19UL)
#define TAMP_IER_ITAMP4IE_Msk        (0x1UL << TAMP_IER_ITAMP4IE_Pos)          /*!< 0x00080000 */
#define TAMP_IER_ITAMP4IE            TAMP_IER_ITAMP4IE_Msk
#define TAMP_IER_ITAMP5IE_Pos        (20UL)
#define TAMP_IER_ITAMP5IE_Msk        (0x1UL << TAMP_IER_ITAMP5IE_Pos)          /*!< 0x00100000 */
#define TAMP_IER_ITAMP5IE            TAMP_IER_ITAMP5IE_Msk
#define TAMP_IER_ITAMP6IE_Pos        (21UL)
#define TAMP_IER_ITAMP6IE_Msk        (0x1UL << TAMP_IER_ITAMP6IE_Pos)          /*!< 0x00200000 */
#define TAMP_IER_ITAMP6IE            TAMP_IER_ITAMP6IE_Msk

/********************  Bits definition for TAMP_SR register  *****************/
#define TAMP_SR_TAMP1F_Pos           (0UL)
#define TAMP_SR_TAMP1F_Msk           (0x1UL << TAMP_SR_TAMP1F_Pos)             /*!< 0x00000001 */
#define TAMP_SR_TAMP1F               TAMP_SR_TAMP1F_Msk
#define TAMP_SR_TAMP2F_Pos           (1UL)
#define TAMP_SR_TAMP2F_Msk           (0x1UL << TAMP_SR_TAMP2F_Pos)             /*!< 0x00000002 */
#define TAMP_SR_TAMP2F               TAMP_SR_TAMP2F_Msk
#define TAMP_SR_TAMP3F_Pos           (2UL)
#define TAMP_SR_TAMP3F_Msk           (0x1UL << TAMP_SR_TAMP3F_Pos)             /*!< 0x00000004 */
#define TAMP_SR_TAMP3F               TAMP_SR_TAMP3F_Msk
#define TAMP_SR_TAMP4F_Pos           (3UL)
#define TAMP_SR_TAMP4F_Msk           (0x1UL << TAMP_SR_TAMP4F_Pos)             /*!< 0x00000004 */
#define TAMP_SR_TAMP4F               TAMP_SR_TAMP4F_Msk
#define TAMP_SR_TAMP5F_Pos           (4UL)
#define TAMP_SR_TAMP5F_Msk           (0x1UL << TAMP_SR_TAMP5F_Pos)             /*!< 0x00000004 */
#define TAMP_SR_TAMP5F               TAMP_SR_TAMP5F_Msk
#define TAMP_SR_ITAMP3F_Pos          (18UL)
#define TAMP_SR_ITAMP3F_Msk          (0x1UL << TAMP_SR_ITAMP3F_Pos)            /*!< 0x00040000 */
#define TAMP_SR_ITAMP3F              TAMP_SR_ITAMP3F_Msk
#define TAMP_SR_ITAMP4F_Pos          (19UL)
#define TAMP_SR_ITAMP4F_Msk          (0x1UL << TAMP_SR_ITAMP4F_Pos)            /*!< 0x00080000 */
#define TAMP_SR_ITAMP4F              TAMP_SR_ITAMP4F_Msk
#define TAMP_SR_ITAMP5F_Pos          (20UL)
#define TAMP_SR_ITAMP5F_Msk          (0x1UL << TAMP_SR_ITAMP5F_Pos)            /*!< 0x00100000 */
#define TAMP_SR_ITAMP5F              TAMP_SR_ITAMP5F_Msk
#define TAMP_SR_ITAMP6F_Pos          (21UL)
#define TAMP_SR_ITAMP6F_Msk          (0x1UL << TAMP_SR_ITAMP6F_Pos)            /*!< 0x00200000 */
#define TAMP_SR_ITAMP6F              TAMP_SR_ITAMP6F_Msk

/********************  Bits definition for TAMP_MISR register  ************ *****/
#define TAMP_MISR_TAMP1MF_Pos        (0UL)
#define TAMP_MISR_TAMP1MF_Msk        (0x1UL << TAMP_MISR_TAMP1MF_Pos)           /*!< 0x00000001 */
#define TAMP_MISR_TAMP1MF            TAMP_MISR_TAMP1MF_Msk
#define TAMP_MISR_TAMP2MF_Pos        (1UL)
#define TAMP_MISR_TAMP2MF_Msk        (0x1UL << TAMP_MISR_TAMP2MF_Pos)           /*!< 0x00000002 */
#define TAMP_MISR_TAMP2MF            TAMP_MISR_TAMP2MF_Msk
#define TAMP_MISR_TAMP3MF_Pos        (2UL)
#define TAMP_MISR_TAMP3MF_Msk        (0x1UL << TAMP_MISR_TAMP3MF_Pos)          /*!< 0x00000004 */
#define TAMP_MISR_TAMP3MF            TAMP_MISR_TAMP3MF_Msk
#define TAMP_MISR_TAMP4MF_Pos        (3UL)
#define TAMP_MISR_TAMP4MF_Msk        (0x1UL << TAMP_MISR_TAMP4MF_Pos)          /*!< 0x00000004 */
#define TAMP_MISR_TAMP4MF            TAMP_MISR_TAMP4MF_Msk
#define TAMP_MISR_TAMP5MF_Pos        (4UL)
#define TAMP_MISR_TAMP5MF_Msk        (0x1UL << TAMP_MISR_TAMP5MF_Pos)          /*!< 0x00000004 */
#define TAMP_MISR_TAMP5MF            TAMP_MISR_TAMP5MF_Msk
#define TAMP_MISR_ITAMP3MF_Pos       (18UL)
#define TAMP_MISR_ITAMP3MF_Msk       (0x1UL << TAMP_MISR_ITAMP3MF_Pos)         /*!< 0x00040000 */
#define TAMP_MISR_ITAMP3MF           TAMP_MISR_ITAMP3MF_Msk
#define TAMP_MISR_ITAMP4MF_Pos       (19UL)
#define TAMP_MISR_ITAMP4MF_Msk       (0x1UL << TAMP_MISR_ITAMP4MF_Pos)         /*!< 0x00080000 */
#define TAMP_MISR_ITAMP4MF           TAMP_MISR_ITAMP4MF_Msk
#define TAMP_MISR_ITAMP5MF_Pos       (20UL)
#define TAMP_MISR_ITAMP5MF_Msk       (0x1UL << TAMP_MISR_ITAMP5MF_Pos)         /*!< 0x00100000 */
#define TAMP_MISR_ITAMP5MF           TAMP_MISR_ITAMP5MF_Msk
#define TAMP_MISR_ITAMP6MF_Pos       (21UL)
#define TAMP_MISR_ITAMP6MF_Msk       (0x1UL << TAMP_MISR_ITAMP6MF_Pos)         /*!< 0x00200000 */
#define TAMP_MISR_ITAMP6MF           TAMP_MISR_ITAMP6MF_Msk

/********************  Bits definition for TAMP_SCR register  *****************/
#define TAMP_SCR_CTAMP1F_Pos         (0UL)
#define TAMP_SCR_CTAMP1F_Msk         (0x1UL << TAMP_SCR_CTAMP1F_Pos)            /*!< 0x00000001 */
#define TAMP_SCR_CTAMP1F             TAMP_SCR_CTAMP1F_Msk
#define TAMP_SCR_CTAMP2F_Pos         (1UL)
#define TAMP_SCR_CTAMP2F_Msk         (0x1UL << TAMP_SCR_CTAMP2F_Pos)            /*!< 0x00000002 */
#define TAMP_SCR_CTAMP2F             TAMP_SCR_CTAMP2F_Msk
#define TAMP_SCR_CTAMP3F_Pos         (2UL)
#define TAMP_SCR_CTAMP3F_Msk         (0x1UL << TAMP_SCR_CTAMP3F_Pos)           /*!< 0x00000004 */
#define TAMP_SCR_CTAMP3F             TAMP_SCR_CTAMP3F_Msk
#define TAMP_SCR_CTAMP4F_Pos         (3UL)
#define TAMP_SCR_CTAMP4F_Msk         (0x1UL << TAMP_SCR_CTAMP4F_Pos)           /*!< 0x00000004 */
#define TAMP_SCR_CTAMP4F             TAMP_SCR_CTAMP4F_Msk
#define TAMP_SCR_CTAMP5F_Pos         (4UL)
#define TAMP_SCR_CTAMP5F_Msk         (0x1UL << TAMP_SCR_CTAMP5F_Pos)           /*!< 0x00000004 */
#define TAMP_SCR_CTAMP5F             TAMP_SCR_CTAMP5F_Msk
#define TAMP_SCR_CITAMP3F_Pos        (18UL)
#define TAMP_SCR_CITAMP3F_Msk        (0x1UL << TAMP_SCR_CITAMP3F_Pos)          /*!< 0x00040000 */
#define TAMP_SCR_CITAMP3F            TAMP_SCR_CITAMP3F_Msk
#define TAMP_SCR_CITAMP4F_Pos        (19UL)
#define TAMP_SCR_CITAMP4F_Msk        (0x1UL << TAMP_SCR_CITAMP4F_Pos)          /*!< 0x00080000 */
#define TAMP_SCR_CITAMP4F            TAMP_SCR_CITAMP4F_Msk
#define TAMP_SCR_CITAMP5F_Pos        (20UL)
#define TAMP_SCR_CITAMP5F_Msk        (0x1UL << TAMP_SCR_CITAMP5F_Pos)          /*!< 0x00100000 */
#define TAMP_SCR_CITAMP5F            TAMP_SCR_CITAMP5F_Msk
#define TAMP_SCR_CITAMP6F_Pos        (21UL)
#define TAMP_SCR_CITAMP6F_Msk        (0x1UL << TAMP_SCR_CITAMP6F_Pos)          /*!< 0x00200000 */
#define TAMP_SCR_CITAMP6F            TAMP_SCR_CITAMP6F_Msk

/********************  Bits definition for TAMP_BKP0R register  ***************/
#define TAMP_BKP0R_Pos               (0UL)
#define TAMP_BKP0R_Msk               (0xFFFFFFFFUL << TAMP_BKP0R_Pos)           /*!< 0xFFFFFFFF */
#define TAMP_BKP0R                   TAMP_BKP0R_Msk

/********************  Bits definition for TAMP_BKP1R register  ****************/
#define TAMP_BKP1R_Pos               (0UL)
#define TAMP_BKP1R_Msk               (0xFFFFFFFFUL << TAMP_BKP1R_Pos)           /*!< 0xFFFFFFFF */
#define TAMP_BKP1R                   TAMP_BKP1R_Msk

/********************  Bits definition for TAMP_BKP2R register  ****************/
#define TAMP_BKP2R_Pos               (0UL)
#define TAMP_BKP2R_Msk               (0xFFFFFFFFUL << TAMP_BKP2R_Pos)           /*!< 0xFFFFFFFF */
#define TAMP_BKP2R                   TAMP_BKP2R_Msk

/********************  Bits definition for TAMP_BKP3R register  ****************/
#define TAMP_BKP3R_Pos               (0UL)
#define TAMP_BKP3R_Msk               (0xFFFFFFFFUL << TAMP_BKP3R_Pos)           /*!< 0xFFFFFFFF */
#define TAMP_BKP3R                   TAMP_BKP3R_Msk

/********************  Bits definition for TAMP_BKP4R register  ****************/
#define TAMP_BKP4R_Pos               (0UL)
#define TAMP_BKP4R_Msk               (0xFFFFFFFFUL << TAMP_BKP4R_Pos)           /*!< 0xFFFFFFFF */
#define TAMP_BKP4R                   TAMP_BKP4R_Msk

/********************  Bits definition for TAMP_BKP5R register  ****************/
#define TAMP_BKP5R_Pos               (0UL)
#define TAMP_BKP5R_Msk               (0xFFFFFFFFUL << TAMP_BKP5R_Pos)          /*!< 0xFFFFFFFF */
#define TAMP_BKP5R                   TAMP_BKP5R_Msk

/********************  Bits definition for TAMP_BKP6R register  ****************/
#define TAMP_BKP6R_Pos               (0UL)
#define TAMP_BKP6R_Msk               (0xFFFFFFFFUL << TAMP_BKP6R_Pos)          /*!< 0xFFFFFFFF */
#define TAMP_BKP6R                   TAMP_BKP6R_Msk

/********************  Bits definition for TAMP_BKP7R register  ****************/
#define TAMP_BKP7R_Pos               (0UL)
#define TAMP_BKP7R_Msk               (0xFFFFFFFFUL << TAMP_BKP7R_Pos)          /*!< 0xFFFFFFFF */
#define TAMP_BKP7R                   TAMP_BKP7R_Msk

/********************  Bits definition for TAMP_BKP8R register  ****************/
#define TAMP_BKP8R_Pos               (0UL)
#define TAMP_BKP8R_Msk               (0xFFFFFFFFUL << TAMP_BKP8R_Pos)          /*!< 0xFFFFFFFF */
#define TAMP_BKP8R                   TAMP_BKP8R_Msk

/******************** Number of backup registers ******************************/
#define TAMP_BKP_NUMBER_Pos             (4UL)
#define TAMP_BKP_NUMBER_Msk             (0x1UL << TAMP_BKP_NUMBER_Pos)         /*!< 0x00000080 */
#define TAMP_BKP_NUMBER                 TAMP_BKP_NUMBER_Msk                    /*!< 9 BKPREG  */

/******************************************************************************/
/*                                                                            */
/*                   Serial Peripheral Interface (SPI)                        */
/*                                                                            */
/******************************************************************************/
/*
 * @brief Specific device feature definitions
 */

/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA_Pos            (0UL)
#define SPI_CR1_CPHA_Msk            (0x1UL << SPI_CR1_CPHA_Pos)                /*!< 0x00000001 */
#define SPI_CR1_CPHA                SPI_CR1_CPHA_Msk                           /*!<Clock Phase      */
#define SPI_CR1_CPOL_Pos            (1UL)
#define SPI_CR1_CPOL_Msk            (0x1UL << SPI_CR1_CPOL_Pos)                /*!< 0x00000002 */
#define SPI_CR1_CPOL                SPI_CR1_CPOL_Msk                           /*!<Clock Polarity   */
#define SPI_CR1_MSTR_Pos            (2UL)
#define SPI_CR1_MSTR_Msk            (0x1UL << SPI_CR1_MSTR_Pos)                /*!< 0x00000004 */
#define SPI_CR1_MSTR                SPI_CR1_MSTR_Msk                           /*!<Master Selection */

#define SPI_CR1_BR_Pos              (3UL)
#define SPI_CR1_BR_Msk              (0x7UL << SPI_CR1_BR_Pos)                  /*!< 0x00000038 */
#define SPI_CR1_BR                  SPI_CR1_BR_Msk                             /*!<BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                (0x1UL << SPI_CR1_BR_Pos)                  /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2UL << SPI_CR1_BR_Pos)                  /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4UL << SPI_CR1_BR_Pos)                  /*!< 0x00000020 */

#define SPI_CR1_SPE_Pos             (6UL)
#define SPI_CR1_SPE_Msk             (0x1UL << SPI_CR1_SPE_Pos)                 /*!< 0x00000040 */
#define SPI_CR1_SPE                 SPI_CR1_SPE_Msk                            /*!<SPI Enable                          */
#define SPI_CR1_LSBFIRST_Pos        (7UL)
#define SPI_CR1_LSBFIRST_Msk        (0x1UL << SPI_CR1_LSBFIRST_Pos)            /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST            SPI_CR1_LSBFIRST_Msk                       /*!<Frame Format                        */
#define SPI_CR1_SSI_Pos             (8UL)
#define SPI_CR1_SSI_Msk             (0x1UL << SPI_CR1_SSI_Pos)                 /*!< 0x00000100 */
#define SPI_CR1_SSI                 SPI_CR1_SSI_Msk                            /*!<Internal slave select               */
#define SPI_CR1_SSM_Pos             (9UL)
#define SPI_CR1_SSM_Msk             (0x1UL << SPI_CR1_SSM_Pos)                 /*!< 0x00000200 */
#define SPI_CR1_SSM                 SPI_CR1_SSM_Msk                            /*!<Software slave management           */
#define SPI_CR1_RXONLY_Pos          (10UL)
#define SPI_CR1_RXONLY_Msk          (0x1UL << SPI_CR1_RXONLY_Pos)              /*!< 0x00000400 */
#define SPI_CR1_RXONLY              SPI_CR1_RXONLY_Msk                         /*!<Receive only                        */
#define SPI_CR1_CRCL_Pos            (11UL)
#define SPI_CR1_CRCL_Msk            (0x1UL << SPI_CR1_CRCL_Pos)                /*!< 0x00000800 */
#define SPI_CR1_CRCL                SPI_CR1_CRCL_Msk                           /*!< CRC Length */
#define SPI_CR1_CRCNEXT_Pos         (12UL)
#define SPI_CR1_CRCNEXT_Msk         (0x1UL << SPI_CR1_CRCNEXT_Pos)             /*!< 0x00001000 */
#define SPI_CR1_CRCNEXT             SPI_CR1_CRCNEXT_Msk                        /*!<Transmit CRC next                   */
#define SPI_CR1_CRCEN_Pos           (13UL)
#define SPI_CR1_CRCEN_Msk           (0x1UL << SPI_CR1_CRCEN_Pos)               /*!< 0x00002000 */
#define SPI_CR1_CRCEN               SPI_CR1_CRCEN_Msk                          /*!<Hardware CRC calculation enable     */
#define SPI_CR1_BIDIOE_Pos          (14UL)
#define SPI_CR1_BIDIOE_Msk          (0x1UL << SPI_CR1_BIDIOE_Pos)              /*!< 0x00004000 */
#define SPI_CR1_BIDIOE              SPI_CR1_BIDIOE_Msk                         /*!<Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_Pos        (15UL)
#define SPI_CR1_BIDIMODE_Msk        (0x1UL << SPI_CR1_BIDIMODE_Pos)            /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE            SPI_CR1_BIDIMODE_Msk                       /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_RXDMAEN_Pos         (0UL)
#define SPI_CR2_RXDMAEN_Msk         (0x1UL << SPI_CR2_RXDMAEN_Pos)             /*!< 0x00000001 */
#define SPI_CR2_RXDMAEN             SPI_CR2_RXDMAEN_Msk                        /*!< Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN_Pos         (1UL)
#define SPI_CR2_TXDMAEN_Msk         (0x1UL << SPI_CR2_TXDMAEN_Pos)             /*!< 0x00000002 */
#define SPI_CR2_TXDMAEN             SPI_CR2_TXDMAEN_Msk                        /*!< Tx Buffer DMA Enable */
#define SPI_CR2_SSOE_Pos            (2UL)
#define SPI_CR2_SSOE_Msk            (0x1UL << SPI_CR2_SSOE_Pos)                /*!< 0x00000004 */
#define SPI_CR2_SSOE                SPI_CR2_SSOE_Msk                           /*!< SS Output Enable */
#define SPI_CR2_NSSP_Pos            (3UL)
#define SPI_CR2_NSSP_Msk            (0x1UL << SPI_CR2_NSSP_Pos)                /*!< 0x00000008 */
#define SPI_CR2_NSSP                SPI_CR2_NSSP_Msk                           /*!< NSS pulse management Enable */
#define SPI_CR2_FRF_Pos             (4UL)
#define SPI_CR2_FRF_Msk             (0x1UL << SPI_CR2_FRF_Pos)                 /*!< 0x00000010 */
#define SPI_CR2_FRF                 SPI_CR2_FRF_Msk                            /*!< Frame Format Enable */
#define SPI_CR2_ERRIE_Pos           (5UL)
#define SPI_CR2_ERRIE_Msk           (0x1UL << SPI_CR2_ERRIE_Pos)               /*!< 0x00000020 */
#define SPI_CR2_ERRIE               SPI_CR2_ERRIE_Msk                          /*!< Error Interrupt Enable */
#define SPI_CR2_RXNEIE_Pos          (6UL)
#define SPI_CR2_RXNEIE_Msk          (0x1UL << SPI_CR2_RXNEIE_Pos)              /*!< 0x00000040 */
#define SPI_CR2_RXNEIE              SPI_CR2_RXNEIE_Msk                         /*!< RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE_Pos           (7UL)
#define SPI_CR2_TXEIE_Msk           (0x1UL << SPI_CR2_TXEIE_Pos)               /*!< 0x00000080 */
#define SPI_CR2_TXEIE               SPI_CR2_TXEIE_Msk                          /*!< Tx buffer Empty Interrupt Enable */
#define SPI_CR2_DS_Pos              (8UL)
#define SPI_CR2_DS_Msk              (0xFUL << SPI_CR2_DS_Pos)                  /*!< 0x00000F00 */
#define SPI_CR2_DS                  SPI_CR2_DS_Msk                             /*!< DS[3:0] Data Size */
#define SPI_CR2_DS_0                (0x1UL << SPI_CR2_DS_Pos)                  /*!< 0x00000100 */
#define SPI_CR2_DS_1                (0x2UL << SPI_CR2_DS_Pos)                  /*!< 0x00000200 */
#define SPI_CR2_DS_2                (0x4UL << SPI_CR2_DS_Pos)                  /*!< 0x00000400 */
#define SPI_CR2_DS_3                (0x8UL << SPI_CR2_DS_Pos)                  /*!< 0x00000800 */
#define SPI_CR2_FRXTH_Pos           (12UL)
#define SPI_CR2_FRXTH_Msk           (0x1UL << SPI_CR2_FRXTH_Pos)               /*!< 0x00001000 */
#define SPI_CR2_FRXTH               SPI_CR2_FRXTH_Msk                          /*!< FIFO reception Threshold */
#define SPI_CR2_LDMARX_Pos          (13UL)
#define SPI_CR2_LDMARX_Msk          (0x1UL << SPI_CR2_LDMARX_Pos)              /*!< 0x00002000 */
#define SPI_CR2_LDMARX              SPI_CR2_LDMARX_Msk                         /*!< Last DMA transfer for reception */
#define SPI_CR2_LDMATX_Pos          (14UL)
#define SPI_CR2_LDMATX_Msk          (0x1UL << SPI_CR2_LDMATX_Pos)              /*!< 0x00004000 */
#define SPI_CR2_LDMATX              SPI_CR2_LDMATX_Msk                         /*!< Last DMA transfer for transmission */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE_Pos             (0UL)
#define SPI_SR_RXNE_Msk             (0x1UL << SPI_SR_RXNE_Pos)                 /*!< 0x00000001 */
#define SPI_SR_RXNE                 SPI_SR_RXNE_Msk                            /*!< Receive buffer Not Empty */
#define SPI_SR_TXE_Pos              (1UL)
#define SPI_SR_TXE_Msk              (0x1UL << SPI_SR_TXE_Pos)                  /*!< 0x00000002 */
#define SPI_SR_TXE                  SPI_SR_TXE_Msk                             /*!< Transmit buffer Empty */
#define SPI_SR_CHSIDE_Pos           (2UL)
#define SPI_SR_CHSIDE_Msk           (0x1UL << SPI_SR_CHSIDE_Pos)               /*!< 0x00000004 */
#define SPI_SR_CHSIDE               SPI_SR_CHSIDE_Msk                          /*!< Channel side */
#define SPI_SR_UDR_Pos              (3UL)
#define SPI_SR_UDR_Msk              (0x1UL << SPI_SR_UDR_Pos)                  /*!< 0x00000008 */
#define SPI_SR_UDR                  SPI_SR_UDR_Msk                             /*!< Underrun flag */
#define SPI_SR_CRCERR_Pos           (4UL)
#define SPI_SR_CRCERR_Msk           (0x1UL << SPI_SR_CRCERR_Pos)               /*!< 0x00000010 */
#define SPI_SR_CRCERR               SPI_SR_CRCERR_Msk                          /*!< CRC Error flag */
#define SPI_SR_MODF_Pos             (5UL)
#define SPI_SR_MODF_Msk             (0x1UL << SPI_SR_MODF_Pos)                 /*!< 0x00000020 */
#define SPI_SR_MODF                 SPI_SR_MODF_Msk                            /*!< Mode fault */
#define SPI_SR_OVR_Pos              (6UL)
#define SPI_SR_OVR_Msk              (0x1UL << SPI_SR_OVR_Pos)                  /*!< 0x00000040 */
#define SPI_SR_OVR                  SPI_SR_OVR_Msk                             /*!< Overrun flag */
#define SPI_SR_BSY_Pos              (7UL)
#define SPI_SR_BSY_Msk              (0x1UL << SPI_SR_BSY_Pos)                  /*!< 0x00000080 */
#define SPI_SR_BSY                  SPI_SR_BSY_Msk                             /*!< Busy flag */
#define SPI_SR_FRE_Pos              (8UL)
#define SPI_SR_FRE_Msk              (0x1UL << SPI_SR_FRE_Pos)                  /*!< 0x00000100 */
#define SPI_SR_FRE                  SPI_SR_FRE_Msk                             /*!< TI frame format error */
#define SPI_SR_FRLVL_Pos            (9UL)
#define SPI_SR_FRLVL_Msk            (0x3UL << SPI_SR_FRLVL_Pos)                /*!< 0x00000600 */
#define SPI_SR_FRLVL                SPI_SR_FRLVL_Msk                           /*!< FIFO Reception Level */
#define SPI_SR_FRLVL_0              (0x1UL << SPI_SR_FRLVL_Pos)                /*!< 0x00000200 */
#define SPI_SR_FRLVL_1              (0x2UL << SPI_SR_FRLVL_Pos)                /*!< 0x00000400 */
#define SPI_SR_FTLVL_Pos            (11UL)
#define SPI_SR_FTLVL_Msk            (0x3UL << SPI_SR_FTLVL_Pos)                /*!< 0x00001800 */
#define SPI_SR_FTLVL                SPI_SR_FTLVL_Msk                           /*!< FIFO Transmission Level */
#define SPI_SR_FTLVL_0              (0x1UL << SPI_SR_FTLVL_Pos)                /*!< 0x00000800 */
#define SPI_SR_FTLVL_1              (0x2UL << SPI_SR_FTLVL_Pos)                /*!< 0x00001000 */

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR_Pos               (0UL)
#define SPI_DR_DR_Msk               (0xFFFFUL << SPI_DR_DR_Pos)                /*!< 0x0000FFFF */
#define SPI_DR_DR                   SPI_DR_DR_Msk                              /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define SPI_CRCPR_CRCPOLY_Pos       (0UL)
#define SPI_CRCPR_CRCPOLY_Msk       (0xFFFFUL << SPI_CRCPR_CRCPOLY_Pos)        /*!< 0x0000FFFF */
#define SPI_CRCPR_CRCPOLY           SPI_CRCPR_CRCPOLY_Msk                      /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define SPI_RXCRCR_RXCRC_Pos        (0UL)
#define SPI_RXCRCR_RXCRC_Msk        (0xFFFFUL << SPI_RXCRCR_RXCRC_Pos)         /*!< 0x0000FFFF */
#define SPI_RXCRCR_RXCRC            SPI_RXCRCR_RXCRC_Msk                       /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define SPI_TXCRCR_TXCRC_Pos        (0UL)
#define SPI_TXCRCR_TXCRC_Msk        (0xFFFFUL << SPI_TXCRCR_TXCRC_Pos)         /*!< 0x0000FFFF */
#define SPI_TXCRCR_TXCRC            SPI_TXCRCR_TXCRC_Msk                       /*!<Tx CRC Register         */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for SYSCFG_CFGR1 register  ******************/
#define SYSCFG_CFGR1_MEM_MODE_Pos       (0UL)
#define SYSCFG_CFGR1_MEM_MODE_Msk       (0x3UL << SYSCFG_CFGR1_MEM_MODE_Pos)     /*!< 0x00000003 */
#define SYSCFG_CFGR1_MEM_MODE           SYSCFG_CFGR1_MEM_MODE_Msk                /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_CFGR1_MEM_MODE_0         (0x1UL << SYSCFG_CFGR1_MEM_MODE_Pos)     /*!< 0x00000001 */
#define SYSCFG_CFGR1_MEM_MODE_1         (0x2UL << SYSCFG_CFGR1_MEM_MODE_Pos)     /*!< 0x00000002 */
#define SYSCFG_CFGR1_PA11_RMP_Pos       (3UL)
#define SYSCFG_CFGR1_PA11_RMP_Msk       (0x1UL << SYSCFG_CFGR1_PA11_RMP_Pos)     /*!< 0x00000008 */
#define SYSCFG_CFGR1_PA11_RMP           SYSCFG_CFGR1_PA11_RMP_Msk                /*!< PA11 Remap */
#define SYSCFG_CFGR1_PA12_RMP_Pos       (4UL)
#define SYSCFG_CFGR1_PA12_RMP_Msk       (0x1UL << SYSCFG_CFGR1_PA12_RMP_Pos)     /*!< 0x00000010 */
#define SYSCFG_CFGR1_PA12_RMP           SYSCFG_CFGR1_PA12_RMP_Msk                /*!< PA12 Remap */
#define SYSCFG_CFGR1_IR_POL_Pos         (5UL)
#define SYSCFG_CFGR1_IR_POL_Msk         (0x1UL << SYSCFG_CFGR1_IR_POL_Pos)       /*!< 0x00000020 */
#define SYSCFG_CFGR1_IR_POL             SYSCFG_CFGR1_IR_POL_Msk                  /*!< IROut Polarity Selection */
#define SYSCFG_CFGR1_IR_MOD_Pos         (6UL)
#define SYSCFG_CFGR1_IR_MOD_Msk         (0x3UL << SYSCFG_CFGR1_IR_MOD_Pos)       /*!< 0x000000C0 */
#define SYSCFG_CFGR1_IR_MOD             SYSCFG_CFGR1_IR_MOD_Msk                  /*!< IRDA Modulation Envelope signal source selection */
#define SYSCFG_CFGR1_IR_MOD_0           (0x1UL << SYSCFG_CFGR1_IR_MOD_Pos)       /*!< 0x00000040 */
#define SYSCFG_CFGR1_IR_MOD_1           (0x2UL << SYSCFG_CFGR1_IR_MOD_Pos)       /*!< 0x00000080 */
#define SYSCFG_CFGR1_BOOSTEN_Pos        (8UL)
#define SYSCFG_CFGR1_BOOSTEN_Msk        (0x1UL << SYSCFG_CFGR1_BOOSTEN_Pos)      /*!< 0x00000100 */
#define SYSCFG_CFGR1_BOOSTEN            SYSCFG_CFGR1_BOOSTEN_Msk                 /*!< I/O analog switch voltage booster enable */
#define SYSCFG_CFGR1_I2C_PB6_FMP_Pos    (16UL)
#define SYSCFG_CFGR1_I2C_PB6_FMP_Msk    (0x1UL << SYSCFG_CFGR1_I2C_PB6_FMP_Pos)  /*!< 0x00010000 */
#define SYSCFG_CFGR1_I2C_PB6_FMP        SYSCFG_CFGR1_I2C_PB6_FMP_Msk             /*!< I2C PB6 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB7_FMP_Pos    (17UL)
#define SYSCFG_CFGR1_I2C_PB7_FMP_Msk    (0x1UL << SYSCFG_CFGR1_I2C_PB7_FMP_Pos)  /*!< 0x00020000 */
#define SYSCFG_CFGR1_I2C_PB7_FMP        SYSCFG_CFGR1_I2C_PB7_FMP_Msk             /*!< I2C PB7 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB8_FMP_Pos    (18UL)
#define SYSCFG_CFGR1_I2C_PB8_FMP_Msk    (0x1UL << SYSCFG_CFGR1_I2C_PB8_FMP_Pos)  /*!< 0x00040000 */
#define SYSCFG_CFGR1_I2C_PB8_FMP        SYSCFG_CFGR1_I2C_PB8_FMP_Msk             /*!< I2C PB8 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB9_FMP_Pos    (19UL)
#define SYSCFG_CFGR1_I2C_PB9_FMP_Msk    (0x1UL << SYSCFG_CFGR1_I2C_PB9_FMP_Pos)  /*!< 0x00080000 */
#define SYSCFG_CFGR1_I2C_PB9_FMP        SYSCFG_CFGR1_I2C_PB9_FMP_Msk             /*!< I2C PB9 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PA9_FMP_Pos    (22UL)
#define SYSCFG_CFGR1_I2C_PA9_FMP_Msk    (0x1UL << SYSCFG_CFGR1_I2C_PA9_FMP_Pos)  /*!< 0x00400000 */
#define SYSCFG_CFGR1_I2C_PA9_FMP        SYSCFG_CFGR1_I2C_PA9_FMP_Msk             /*!< Enable Fast Mode Plus on PA9  */
#define SYSCFG_CFGR1_I2C_PA10_FMP_Pos   (23UL)
#define SYSCFG_CFGR1_I2C_PA10_FMP_Msk   (0x1UL << SYSCFG_CFGR1_I2C_PA10_FMP_Pos) /*!< 0x00800000 */
#define SYSCFG_CFGR1_I2C_PA10_FMP       SYSCFG_CFGR1_I2C_PA10_FMP_Msk            /*!< Enable Fast Mode Plus on PA10 */
#define SYSCFG_CFGR1_I2C3_FMP_Pos       (24UL)
#define SYSCFG_CFGR1_I2C3_FMP_Msk       (0x1UL << SYSCFG_CFGR1_I2C3_FMP_Pos)     /*!< 0x01000000 */
#define SYSCFG_CFGR1_I2C3_FMP           SYSCFG_CFGR1_I2C3_FMP_Msk                /*!< Enable Fast Mode Plus for I2C3 */

/******************  Bit definition for SYSCFG_CFGR2 register  ****************/
#define SYSCFG_CFGR2_CCL_Pos            (0UL)
#define SYSCFG_CFGR2_CCL_Msk            (0x1UL << SYSCFG_CFGR2_CCL_Pos)         /*!< 0x00000001 */
#define SYSCFG_CFGR2_CCL                SYSCFG_CFGR2_CCL_Msk                    /*!< Enables and locks the LOCKUP (Hardfault) output of CortexM0 with Break Input */
#define SYSCFG_CFGR2_SPL_Pos            (1UL)
#define SYSCFG_CFGR2_SPL_Msk            (0x1UL << SYSCFG_CFGR2_SPL_Pos)         /*!< 0x00000002 */
#define SYSCFG_CFGR2_SPL                SYSCFG_CFGR2_SPL_Msk                    /*!< Enables and locks the SRAM Parity error signal with TIMERs Break Input */
#define SYSCFG_CFGR2_PVDL_Pos           (2UL)
#define SYSCFG_CFGR2_PVDL_Msk           (0x1UL << SYSCFG_CFGR2_PVDL_Pos)        /*!< 0x00000004 */
#define SYSCFG_CFGR2_PVDL               SYSCFG_CFGR2_PVDL_Msk                   /*!< Enables and locks the PVD connection with TIMERs Break Input */
#define SYSCFG_CFGR2_ECCL_Pos           (3UL)
#define SYSCFG_CFGR2_ECCL_Msk           (0x1UL << SYSCFG_CFGR2_ECCL_Pos)        /*!< 0x00000008 */
#define SYSCFG_CFGR2_ECCL               SYSCFG_CFGR2_ECCL_Msk                   /*!< Enables and locks the FLASH ECC error with TIMERs Break Input */
#define SYSCFG_CFGR2_BKPL_Pos           (4UL)
#define SYSCFG_CFGR2_BKPL_Msk           (0x1UL << SYSCFG_CFGR2_BKPL_Pos)        /*!< 0x00000010 */
#define SYSCFG_CFGR2_BKPL               SYSCFG_CFGR2_BKPL_Msk                   /*!< Enables and locks the BackUp SRAM Parity error signal with TIMERs Break Input */
#define SYSCFG_CFGR2_BKPF_Pos           (7UL)
#define SYSCFG_CFGR2_BKPF_Msk           (0x1UL << SYSCFG_CFGR2_BKPF_Pos)        /*!< 0x00000080 */
#define SYSCFG_CFGR2_BKPF               SYSCFG_CFGR2_BKPF_Msk                   /*!< Saves the occurrence of the BackUpSRAM Parity Error */
#define SYSCFG_CFGR2_SPF_Pos            (8UL)
#define SYSCFG_CFGR2_SPF_Msk            (0x1UL << SYSCFG_CFGR2_SPF_Pos)         /*!< 0x00000100 */
#define SYSCFG_CFGR2_SPF                SYSCFG_CFGR2_SPF_Msk                    /*!< Saves the occurrence of the SRAM Parity Error */

/******************  Bit definition for SYSCFG_SCSR register  ****************/
#define SYSCFG_SCSR_SRAM2ER_Pos         (0UL)
#define SYSCFG_SCSR_SRAM2ER_Msk         (0x1UL << SYSCFG_SCSR_SRAM2ER_Pos)       /*!< 0x00000001 */
#define SYSCFG_SCSR_SRAM2ER             SYSCFG_SCSR_SRAM2ER_Msk                  /*!< Starts a hardware BackUpSRAM erase operation */
#define SYSCFG_SCSR_SRAM2BSY_Pos        (1UL)
#define SYSCFG_SCSR_SRAM2BSY_Msk        (0x1UL << SYSCFG_SCSR_SRAM2BSY_Pos)      /*!< 0x00000002 */
#define SYSCFG_SCSR_SRAM2BSY            SYSCFG_SCSR_SRAM2BSY_Msk                 /*!< BackUpSRAM Erase Ongoing Status Flag*/

/******************  Bit definition for SYSCFG_SER register  ****************/
#define SYSCFG_SKR_KEY_Pos              (0UL)
#define SYSCFG_SKR_KEY_Msk              (0xFFUL << SYSCFG_SKR_KEY_Pos)           /*!< 0x000000FF */
#define SYSCFG_SKR_KEY                  SYSCFG_SKR_KEY_Msk                       /*!< Write BackUpSRAM Key for software erase */

/******************  Bit definition for SYSCFG_TSCCR register  ****************/
#define SYSCFG_TSCCR_G2IO1_Pos          (0UL)
#define SYSCFG_TSCCR_G2IO1_Msk          (0x1UL << SYSCFG_TSCCR_G2IO1_Pos)         /*!< 0x00000001 */
#define SYSCFG_TSCCR_G2IO1              SYSCFG_TSCCR_G2IO1_Msk                    /*!< Enable the comparator mode with the IO1 of Group2 (PB4 on INPLUS of COMP2) */
#define SYSCFG_TSCCR_G2IO3_Pos          (1UL)
#define SYSCFG_TSCCR_G2IO3_Msk          (0x1UL << SYSCFG_TSCCR_G2IO3_Pos)         /*!< 0x00000002 */
#define SYSCFG_TSCCR_G2IO3              SYSCFG_TSCCR_G2IO3_Msk                    /*!< Enable the comparator mode with the IO3 of Group2 (PB6 on INPLUS of COMP2) */
#define SYSCFG_TSCCR_G4IO1_Pos          (2UL)
#define SYSCFG_TSCCR_G4IO1_Msk          (0x1UL << SYSCFG_TSCCR_G4IO1_Pos)         /*!< 0x00000004 */
#define SYSCFG_TSCCR_G4IO1              SYSCFG_TSCCR_G4IO1_Msk                    /*!< Enable the comparator mode with the IO1 of Group4 (PC6 on INPLUS of COMP1) */
#define SYSCFG_TSCCR_G6IO1_Pos          (3UL)
#define SYSCFG_TSCCR_G6IO1_Msk          (0x1UL << SYSCFG_TSCCR_G6IO1_Pos)         /*!< 0x00000008 */
#define SYSCFG_TSCCR_G6IO1              SYSCFG_TSCCR_G6IO1_Msk                    /*!< Enable the comparator mode with the IO1 of Group6 (PD10 on INPLUS of COMP2) */
#define SYSCFG_TSCCR_G7IO2_Pos          (4UL)
#define SYSCFG_TSCCR_G7IO2_Msk          (0x1UL << SYSCFG_TSCCR_G7IO2_Pos)         /*!< 0x00000010 */
#define SYSCFG_TSCCR_G7IO2              SYSCFG_TSCCR_G7IO2_Msk                    /*!< Enable the comparator mode with the IO2 of Group7 (PA9 on INPLUS of COMP1) */
#define SYSCFG_TSCCR_TSCIOCTRL_Pos      (5UL)
#define SYSCFG_TSCCR_TSCIOCTRL_Msk      (0x1UL << SYSCFG_TSCCR_TSCIOCTRL_Pos)     /*!< 0x00000020 */
#define SYSCFG_TSCCR_TSCIOCTRL          SYSCFG_TSCCR_TSCIOCTRL_Msk                /*!< Program Comparator mode (programmed via alternate function) */

/*****************  Bit definition for SYSCFG_ITLINEx ISR Wrapper register  ****************/
#define SYSCFG_ITLINE0_SR_WWDG_Pos         (0UL)
#define SYSCFG_ITLINE0_SR_WWDG_Msk         (0x1UL << SYSCFG_ITLINE0_SR_WWDG_Pos)       /*!< 0x00000001 */
#define SYSCFG_ITLINE0_SR_WWDG             SYSCFG_ITLINE0_SR_WWDG_Msk                  /*!< WWDG interrupt */

#define SYSCFG_ITLINE1_SR_PVDOUT_Pos       (0UL)
#define SYSCFG_ITLINE1_SR_PVDOUT_Msk       (0x1UL << SYSCFG_ITLINE1_SR_PVDOUT_Pos)     /*!< 0x00000001 */
#define SYSCFG_ITLINE1_SR_PVDOUT           SYSCFG_ITLINE1_SR_PVDOUT_Msk                /*!< PVDOUT interrupt */
#define SYSCFG_ITLINE1_SR_PVMOUT1_Pos      (1UL)
#define SYSCFG_ITLINE1_SR_PVMOUT1_Msk      (0x1UL << SYSCFG_ITLINE1_SR_PVMOUT1_Pos)    /*!< 0x00000002 */
#define SYSCFG_ITLINE1_SR_PVMOUT1          SYSCFG_ITLINE1_SR_PVMOUT1_Msk               /*!< VDDUSB interrupt */
#define SYSCFG_ITLINE1_SR_PVMOUT3_Pos      (2UL)
#define SYSCFG_ITLINE1_SR_PVMOUT3_Msk      (0x1UL << SYSCFG_ITLINE1_SR_PVMOUT3_Pos)    /*!< 0x00000004 */
#define SYSCFG_ITLINE1_SR_PVMOUT3          SYSCFG_ITLINE1_SR_PVMOUT3_Msk               /*!< VDDADC interrupt */
#define SYSCFG_ITLINE1_SR_PVMOUT4_Pos      (3)
#define SYSCFG_ITLINE1_SR_PVMOUT4_Msk      (0x1UL << SYSCFG_ITLINE1_SR_PVMOUT4_Pos)    /*!< 0x00000008 */
#define SYSCFG_ITLINE1_SR_PVMOUT4          SYSCFG_ITLINE1_SR_PVMOUT4_Msk               /*!< VDDADC interrupt */

#define SYSCFG_ITLINE2_SR_TAMPER_Pos       (0UL)
#define SYSCFG_ITLINE2_SR_TAMPER_Msk       (0x1UL << SYSCFG_ITLINE2_SR_TAMPER_Pos)     /*!< 0x00000001 */
#define SYSCFG_ITLINE2_SR_TAMPER           SYSCFG_ITLINE2_SR_TAMPER_Msk                /*!< TAMPER interrupt */
#define SYSCFG_ITLINE2_SR_RTC_Pos          (1UL)
#define SYSCFG_ITLINE2_SR_RTC_Msk          (0x1UL << SYSCFG_ITLINE2_SR_RTC_Pos)        /*!< 0x00000001 */
#define SYSCFG_ITLINE2_SR_RTC              SYSCFG_ITLINE2_SR_RTC_Msk                   /*!< RTC interrupt */

#define SYSCFG_ITLINE3_SR_FLASH_ECC_Pos    (0UL)
#define SYSCFG_ITLINE3_SR_FLASH_ECC_Msk    (0x1UL << SYSCFG_ITLINE3_SR_FLASH_ECC_Pos)  /*!< 0x00000001 */
#define SYSCFG_ITLINE3_SR_FLASH_ECC        SYSCFG_ITLINE3_SR_FLASH_ECC_Msk             /*!< FLASH ECC interrupt */
#define SYSCFG_ITLINE3_SR_FLASH_ITF_Pos    (1UL)
#define SYSCFG_ITLINE3_SR_FLASH_ITF_Msk    (0x1UL << SYSCFG_ITLINE3_SR_FLASH_ITF_Pos)  /*!< 0x00000002 */
#define SYSCFG_ITLINE3_SR_FLASH_ITF        SYSCFG_ITLINE3_SR_FLASH_ITF_Msk             /*!< FLASH ITF interrupt */

#define SYSCFG_ITLINE4_SR_RCC_Pos          (0UL)
#define SYSCFG_ITLINE4_SR_RCC_Msk          (0x1UL << SYSCFG_ITLINE4_SR_RCC_Pos)        /*!< 0x00000001 */
#define SYSCFG_ITLINE4_SR_RCC              SYSCFG_ITLINE4_SR_RCC_Msk                   /*!< RCC interrupt */
#define SYSCFG_ITLINE4_SR_CRS_Pos          (1UL)
#define SYSCFG_ITLINE4_SR_CRS_Msk          (0x1UL << SYSCFG_ITLINE4_SR_CRS_Pos)        /*!< 0x00000002 */
#define SYSCFG_ITLINE4_SR_CRS              SYSCFG_ITLINE4_SR_CRS_Msk                   /*!< CRS interrupt */
#define SYSCFG_ITLINE5_SR_EXTI0_Pos        (0UL)
#define SYSCFG_ITLINE5_SR_EXTI0_Msk        (0x1UL << SYSCFG_ITLINE5_SR_EXTI0_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE5_SR_EXTI0            SYSCFG_ITLINE5_SR_EXTI0_Msk                 /*!< External Interrupt 0 */
#define SYSCFG_ITLINE5_SR_EXTI1_Pos        (1UL)
#define SYSCFG_ITLINE5_SR_EXTI1_Msk        (0x1UL << SYSCFG_ITLINE5_SR_EXTI1_Pos)      /*!< 0x00000002 */
#define SYSCFG_ITLINE5_SR_EXTI1            SYSCFG_ITLINE5_SR_EXTI1_Msk                 /*!< External Interrupt 1 */

#define SYSCFG_ITLINE6_SR_EXTI2_Pos        (0UL)
#define SYSCFG_ITLINE6_SR_EXTI2_Msk        (0x1UL << SYSCFG_ITLINE6_SR_EXTI2_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE6_SR_EXTI2            SYSCFG_ITLINE6_SR_EXTI2_Msk                 /*!< External Interrupt 2 */
#define SYSCFG_ITLINE6_SR_EXTI3_Pos        (1UL)
#define SYSCFG_ITLINE6_SR_EXTI3_Msk        (0x1UL << SYSCFG_ITLINE6_SR_EXTI3_Pos)      /*!< 0x00000002 */
#define SYSCFG_ITLINE6_SR_EXTI3            SYSCFG_ITLINE6_SR_EXTI3_Msk                 /*!< External Interrupt 3 */

#define SYSCFG_ITLINE7_SR_EXTI4_Pos        (0UL)
#define SYSCFG_ITLINE7_SR_EXTI4_Msk        (0x1UL << SYSCFG_ITLINE7_SR_EXTI4_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE7_SR_EXTI4            SYSCFG_ITLINE7_SR_EXTI4_Msk                 /*!< External Interrupt 4 */
#define SYSCFG_ITLINE7_SR_EXTI5_Pos        (1UL)
#define SYSCFG_ITLINE7_SR_EXTI5_Msk        (0x1UL << SYSCFG_ITLINE7_SR_EXTI5_Pos)      /*!< 0x00000002 */
#define SYSCFG_ITLINE7_SR_EXTI5            SYSCFG_ITLINE7_SR_EXTI5_Msk                 /*!< External Interrupt 5 */
#define SYSCFG_ITLINE7_SR_EXTI6_Pos        (2UL)
#define SYSCFG_ITLINE7_SR_EXTI6_Msk        (0x1UL << SYSCFG_ITLINE7_SR_EXTI6_Pos)      /*!< 0x00000004 */
#define SYSCFG_ITLINE7_SR_EXTI6            SYSCFG_ITLINE7_SR_EXTI6_Msk                 /*!< External Interrupt 6 */
#define SYSCFG_ITLINE7_SR_EXTI7_Pos        (3UL)
#define SYSCFG_ITLINE7_SR_EXTI7_Msk        (0x1UL << SYSCFG_ITLINE7_SR_EXTI7_Pos)      /*!< 0x00000008 */
#define SYSCFG_ITLINE7_SR_EXTI7            SYSCFG_ITLINE7_SR_EXTI7_Msk                 /*!< External Interrupt 7 */
#define SYSCFG_ITLINE7_SR_EXTI8_Pos        (4UL)
#define SYSCFG_ITLINE7_SR_EXTI8_Msk        (0x1UL << SYSCFG_ITLINE7_SR_EXTI8_Pos)      /*!< 0x00000010 */
#define SYSCFG_ITLINE7_SR_EXTI8            SYSCFG_ITLINE7_SR_EXTI8_Msk                 /*!< External Interrupt 8 */
#define SYSCFG_ITLINE7_SR_EXTI9_Pos        (5UL)
#define SYSCFG_ITLINE7_SR_EXTI9_Msk        (0x1UL << SYSCFG_ITLINE7_SR_EXTI9_Pos)      /*!< 0x00000020 */
#define SYSCFG_ITLINE7_SR_EXTI9            SYSCFG_ITLINE7_SR_EXTI9_Msk                 /*!< External Interrupt 9 */
#define SYSCFG_ITLINE7_SR_EXTI10_Pos       (6UL)
#define SYSCFG_ITLINE7_SR_EXTI10_Msk       (0x1UL << SYSCFG_ITLINE7_SR_EXTI10_Pos)     /*!< 0x00000040 */
#define SYSCFG_ITLINE7_SR_EXTI10           SYSCFG_ITLINE7_SR_EXTI10_Msk                /*!< External Interrupt 10 */
#define SYSCFG_ITLINE7_SR_EXTI11_Pos       (7UL)
#define SYSCFG_ITLINE7_SR_EXTI11_Msk       (0x1UL << SYSCFG_ITLINE7_SR_EXTI11_Pos)     /*!< 0x00000080 */
#define SYSCFG_ITLINE7_SR_EXTI11           SYSCFG_ITLINE7_SR_EXTI11_Msk                /*!< External Interrupt 11 */
#define SYSCFG_ITLINE7_SR_EXTI12_Pos       (8UL)
#define SYSCFG_ITLINE7_SR_EXTI12_Msk       (0x1UL << SYSCFG_ITLINE7_SR_EXTI12_Pos)     /*!< 0x00000100 */
#define SYSCFG_ITLINE7_SR_EXTI12           SYSCFG_ITLINE7_SR_EXTI12_Msk                /*!< External Interrupt 12 */
#define SYSCFG_ITLINE7_SR_EXTI13_Pos       (9UL)
#define SYSCFG_ITLINE7_SR_EXTI13_Msk       (0x1UL << SYSCFG_ITLINE7_SR_EXTI13_Pos)     /*!< 0x00000200 */
#define SYSCFG_ITLINE7_SR_EXTI13           SYSCFG_ITLINE7_SR_EXTI13_Msk                /*!< External Interrupt 13 */
#define SYSCFG_ITLINE7_SR_EXTI14_Pos       (10UL)
#define SYSCFG_ITLINE7_SR_EXTI14_Msk       (0x1UL << SYSCFG_ITLINE7_SR_EXTI14_Pos)     /*!< 0x00000400 */
#define SYSCFG_ITLINE7_SR_EXTI14           SYSCFG_ITLINE7_SR_EXTI14_Msk                /*!< External Interrupt 14 */
#define SYSCFG_ITLINE7_SR_EXTI15_Pos       (11UL)
#define SYSCFG_ITLINE7_SR_EXTI15_Msk       (0x1UL << SYSCFG_ITLINE7_SR_EXTI15_Pos)     /*!< 0x00000800 */
#define SYSCFG_ITLINE7_SR_EXTI15           SYSCFG_ITLINE7_SR_EXTI15_Msk                /*!< External Interrupt 15 */

#define SYSCFG_ITLINE8_SR_USBFS_Pos        (0UL)
#define SYSCFG_ITLINE8_SR_USBFS_Msk        (0x1UL << SYSCFG_ITLINE8_SR_USBFS_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE8_SR_USBFS            SYSCFG_ITLINE8_SR_USBFS_Msk                 /*!< USBFS Interrupt */

#define SYSCFG_ITLINE9_SR_DMA1_CH1_Pos     (0UL)
#define SYSCFG_ITLINE9_SR_DMA1_CH1_Msk     (0x1UL << SYSCFG_ITLINE9_SR_DMA1_CH1_Pos)   /*!< 0x00000001 */
#define SYSCFG_ITLINE9_SR_DMA1_CH1         SYSCFG_ITLINE9_SR_DMA1_CH1_Msk              /*!< DMA1 Channel 1 Interrupt */

#define SYSCFG_ITLINE10_SR_DMA1_CH2_Pos    (0UL)
#define SYSCFG_ITLINE10_SR_DMA1_CH2_Msk    (0x1UL << SYSCFG_ITLINE10_SR_DMA1_CH2_Pos)  /*!< 0x00000001 */
#define SYSCFG_ITLINE10_SR_DMA1_CH2        SYSCFG_ITLINE10_SR_DMA1_CH2_Msk             /*!< DMA1 Channel 2 Interrupt */
#define SYSCFG_ITLINE10_SR_DMA1_CH3_Pos    (1UL)
#define SYSCFG_ITLINE10_SR_DMA1_CH3_Msk    (0x1UL << SYSCFG_ITLINE10_SR_DMA1_CH3_Pos)  /*!< 0x00000002 */
#define SYSCFG_ITLINE10_SR_DMA1_CH3        SYSCFG_ITLINE10_SR_DMA1_CH3_Msk             /*!< DMA1 Channel 3 Interrupt */

#define SYSCFG_ITLINE11_SR_DMAMUX_Pos      (0UL)
#define SYSCFG_ITLINE11_SR_DMAMUX_Msk      (0x1UL << SYSCFG_ITLINE11_SR_DMAMUX_Pos)    /*!< 0x00000001 */
#define SYSCFG_ITLINE11_SR_DMAMUX          SYSCFG_ITLINE11_SR_DMAMUX_Msk               /*!< DMAMUX Interrupt */
#define SYSCFG_ITLINE11_SR_DMA1_CH4_Pos    (1UL)
#define SYSCFG_ITLINE11_SR_DMA1_CH4_Msk    (0x1UL << SYSCFG_ITLINE11_SR_DMA1_CH4_Pos)  /*!< 0x00000002 */
#define SYSCFG_ITLINE11_SR_DMA1_CH4        SYSCFG_ITLINE11_SR_DMA1_CH4_Msk             /*!< DMA1 Channel 4 Interrupt */
#define SYSCFG_ITLINE11_SR_DMA1_CH5_Pos    (2UL)
#define SYSCFG_ITLINE11_SR_DMA1_CH5_Msk    (0x1UL << SYSCFG_ITLINE11_SR_DMA1_CH5_Pos)  /*!< 0x00000004 */
#define SYSCFG_ITLINE11_SR_DMA1_CH5        SYSCFG_ITLINE11_SR_DMA1_CH5_Msk             /*!< DMA1 Channel 5 Interrupt */
#define SYSCFG_ITLINE11_SR_DMA1_CH6_Pos    (3UL)
#define SYSCFG_ITLINE11_SR_DMA1_CH6_Msk    (0x1UL << SYSCFG_ITLINE11_SR_DMA1_CH6_Pos)  /*!< 0x00000008 */
#define SYSCFG_ITLINE11_SR_DMA1_CH6        SYSCFG_ITLINE11_SR_DMA1_CH6_Msk             /*!< DMA1 Channel 6 Interrupt */
#define SYSCFG_ITLINE11_SR_DMA1_CH7_Pos    (4UL)
#define SYSCFG_ITLINE11_SR_DMA1_CH7_Msk    (0x1UL << SYSCFG_ITLINE11_SR_DMA1_CH7_Pos)  /*!< 0x00000010 */
#define SYSCFG_ITLINE11_SR_DMA1_CH7        SYSCFG_ITLINE11_SR_DMA1_CH7_Msk             /*!< DMA1 Channel 7 Interrupt */
#define SYSCFG_ITLINE11_SR_DMA2_CH1_Pos    (5UL)
#define SYSCFG_ITLINE11_SR_DMA2_CH1_Msk    (0x1UL << SYSCFG_ITLINE11_SR_DMA2_CH1_Pos)  /*!< 0x00000020 */
#define SYSCFG_ITLINE11_SR_DMA2_CH1        SYSCFG_ITLINE11_SR_DMA2_CH1_Msk             /*!< DMA1 Channel 8 Interrupt */
#define SYSCFG_ITLINE11_SR_DMA2_CH2_Pos    (6UL)
#define SYSCFG_ITLINE11_SR_DMA2_CH2_Msk    (0x1UL << SYSCFG_ITLINE11_SR_DMA2_CH2_Pos)  /*!< 0x00000040 */
#define SYSCFG_ITLINE11_SR_DMA2_CH2        SYSCFG_ITLINE11_SR_DMA2_CH2_Msk             /*!< DMA1 Channel 9 Interrupt */
#define SYSCFG_ITLINE11_SR_DMA2_CH3_Pos    (7UL)
#define SYSCFG_ITLINE11_SR_DMA2_CH3_Msk    (0x1UL << SYSCFG_ITLINE11_SR_DMA2_CH3_Pos)  /*!< 0x00000080 */
#define SYSCFG_ITLINE11_SR_DMA2_CH3        SYSCFG_ITLINE11_SR_DMA2_CH3_Msk             /*!< DMA1 Channel 10 Interrupt */
#define SYSCFG_ITLINE11_SR_DMA2_CH4_Pos    (8UL)
#define SYSCFG_ITLINE11_SR_DMA2_CH4_Msk    (0x1UL << SYSCFG_ITLINE11_SR_DMA2_CH4_Pos)  /*!< 0x00000100 */
#define SYSCFG_ITLINE11_SR_DMA2_CH4        SYSCFG_ITLINE11_SR_DMA2_CH4_Msk             /*!< DMA1 Channel 11 Interrupt */
#define SYSCFG_ITLINE11_SR_DMA2_CH5_Pos    (9UL)
#define SYSCFG_ITLINE11_SR_DMA2_CH5_Msk    (0x1UL << SYSCFG_ITLINE11_SR_DMA2_CH5_Pos)  /*!< 0x00000200 */
#define SYSCFG_ITLINE11_SR_DMA2_CH5        SYSCFG_ITLINE11_SR_DMA2_CH5_Msk             /*!< DMA1 Channel 12 Interrupt */

#define SYSCFG_ITLINE12_SR_ADC_Pos         (0UL)
#define SYSCFG_ITLINE12_SR_ADC_Msk         (0x1UL << SYSCFG_ITLINE12_SR_ADC_Pos)       /*!< 0x00000001 */
#define SYSCFG_ITLINE12_SR_ADC             SYSCFG_ITLINE12_SR_ADC_Msk                  /*!< ADC Interrupt */
#define SYSCFG_ITLINE12_SR_COMP1_Pos       (1UL)
#define SYSCFG_ITLINE12_SR_COMP1_Msk       (0x1UL << SYSCFG_ITLINE12_SR_COMP1_Pos)     /*!< 0x00000002 */
#define SYSCFG_ITLINE12_SR_COMP1           SYSCFG_ITLINE12_SR_COMP1_Msk                /*!< COMP1 Interrupt -> exti[17] */
#define SYSCFG_ITLINE12_SR_COMP2_Pos       (2UL)
#define SYSCFG_ITLINE12_SR_COMP2_Msk       (0x1UL << SYSCFG_ITLINE12_SR_COMP2_Pos)     /*!< 0x00000004 */
#define SYSCFG_ITLINE12_SR_COMP2           SYSCFG_ITLINE12_SR_COMP2_Msk                /*!< COMP2 Interrupt -> exti[18] */

#define SYSCFG_ITLINE13_SR_TIM1_CCU_Pos    (0UL)
#define SYSCFG_ITLINE13_SR_TIM1_CCU_Msk    (0x1UL << SYSCFG_ITLINE13_SR_TIM1_CCU_Pos)  /*!< 0x00000001 */
#define SYSCFG_ITLINE13_SR_TIM1_CCU        SYSCFG_ITLINE13_SR_TIM1_CCU_Msk             /*!< TIM1 CCU Interrupt */
#define SYSCFG_ITLINE13_SR_TIM1_TRG_Pos    (1UL)
#define SYSCFG_ITLINE13_SR_TIM1_TRG_Msk    (0x1UL << SYSCFG_ITLINE13_SR_TIM1_TRG_Pos)  /*!< 0x00000002 */
#define SYSCFG_ITLINE13_SR_TIM1_TRG        SYSCFG_ITLINE13_SR_TIM1_TRG_Msk             /*!< TIM1 TRG Interrupt */
#define SYSCFG_ITLINE13_SR_TIM1_UPD_Pos    (2UL)
#define SYSCFG_ITLINE13_SR_TIM1_UPD_Msk    (0x1UL << SYSCFG_ITLINE13_SR_TIM1_UPD_Pos)  /*!< 0x00000004 */
#define SYSCFG_ITLINE13_SR_TIM1_UPD        SYSCFG_ITLINE13_SR_TIM1_UPD_Msk             /*!< TIM1 UPD Interrupt */
#define SYSCFG_ITLINE13_SR_TIM1_BRK_Pos    (3UL)
#define SYSCFG_ITLINE13_SR_TIM1_BRK_Msk    (0x1UL << SYSCFG_ITLINE13_SR_TIM1_BRK_Pos)  /*!< 0x00000008 */
#define SYSCFG_ITLINE13_SR_TIM1_BRK        SYSCFG_ITLINE13_SR_TIM1_BRK_Msk             /*!< TIM1 BRK Interrupt */

#define SYSCFG_ITLINE14_SR_TIM1_CC1_Pos    (0UL)
#define SYSCFG_ITLINE14_SR_TIM1_CC1_Msk    (0x1UL << SYSCFG_ITLINE14_SR_TIM1_CC1_Pos)  /*!< 0x00000001 */
#define SYSCFG_ITLINE14_SR_TIM1_CC1        SYSCFG_ITLINE14_SR_TIM1_CC1_Msk             /*!< TIM1 CC1 Interrupt */
#define SYSCFG_ITLINE14_SR_TIM1_CC2_Pos    (1UL)
#define SYSCFG_ITLINE14_SR_TIM1_CC2_Msk    (0x1UL << SYSCFG_ITLINE14_SR_TIM1_CC2_Pos)  /*!< 0x00000002 */
#define SYSCFG_ITLINE14_SR_TIM1_CC2        SYSCFG_ITLINE14_SR_TIM1_CC2_Msk             /*!< TIM1 CC2 Interrupt */
#define SYSCFG_ITLINE14_SR_TIM1_CC3_Pos    (2UL)
#define SYSCFG_ITLINE14_SR_TIM1_CC3_Msk    (0x1UL << SYSCFG_ITLINE14_SR_TIM1_CC3_Pos)  /*!< 0x00000004 */
#define SYSCFG_ITLINE14_SR_TIM1_CC3        SYSCFG_ITLINE14_SR_TIM1_CC3_Msk             /*!< TIM1 CC3 Interrupt */
#define SYSCFG_ITLINE14_SR_TIM1_CC4_Pos    (3UL)
#define SYSCFG_ITLINE14_SR_TIM1_CC4_Msk    (0x1UL << SYSCFG_ITLINE14_SR_TIM1_CC4_Pos)  /*!< 0x00000008 */
#define SYSCFG_ITLINE14_SR_TIM1_CC4        SYSCFG_ITLINE14_SR_TIM1_CC4_Msk             /*!< TIM1 CC4 Interrupt */

#define SYSCFG_ITLINE15_SR_TIM2_Pos        (0UL)
#define SYSCFG_ITLINE15_SR_TIM2_Msk        (0x1UL << SYSCFG_ITLINE15_SR_TIM2_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE15_SR_TIM2            SYSCFG_ITLINE15_SR_TIM2_Msk                 /*!< TIM2 GLB Interrupt */

#define SYSCFG_ITLINE16_SR_TIM3_Pos        (0UL)
#define SYSCFG_ITLINE16_SR_TIM3_Msk        (0x1UL << SYSCFG_ITLINE16_SR_TIM3_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE16_SR_TIM3            SYSCFG_ITLINE16_SR_TIM3_Msk                 /*!< TIM3 GLB Interrupt */

#define SYSCFG_ITLINE17_SR_TIM6_Pos        (0UL)
#define SYSCFG_ITLINE17_SR_TIM6_Msk        (0x1UL << SYSCFG_ITLINE17_SR_TIM6_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE17_SR_TIM6            SYSCFG_ITLINE17_SR_TIM6_Msk                 /*!< TIM6 GLB Interrupt */
#define SYSCFG_ITLINE17_SR_DAC_Pos         (1UL)
#define SYSCFG_ITLINE17_SR_DAC_Msk         (0x1UL << SYSCFG_ITLINE17_SR_DAC_Pos)       /*!< 0x00000002 */
#define SYSCFG_ITLINE17_SR_DAC             SYSCFG_ITLINE17_SR_DAC_Msk                  /*!< DAC Interrupt */
#define SYSCFG_ITLINE17_SR_LPTIM1_Pos      (2UL)
#define SYSCFG_ITLINE17_SR_LPTIM1_Msk      (0x1UL << SYSCFG_ITLINE17_SR_LPTIM1_Pos)    /*!< 0x00000004 */
#define SYSCFG_ITLINE17_SR_LPTIM1          SYSCFG_ITLINE17_SR_LPTIM1_Msk               /*!< LPTIM1 -> exti[24] Interrupt */

#define SYSCFG_ITLINE18_SR_TIM7_Pos        (0UL)
#define SYSCFG_ITLINE18_SR_TIM7_Msk        (0x1UL << SYSCFG_ITLINE18_SR_TIM7_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE18_SR_TIM7            SYSCFG_ITLINE18_SR_TIM7_Msk                 /*!< TIM7 GLB Interrupt */
#define SYSCFG_ITLINE18_SR_LPTIM2_Pos      (1UL)
#define SYSCFG_ITLINE18_SR_LPTIM2_Msk      (0x1UL << SYSCFG_ITLINE18_SR_LPTIM2_Pos)    /*!< 0x00000002 */
#define SYSCFG_ITLINE18_SR_LPTIM2          SYSCFG_ITLINE18_SR_LPTIM2_Msk               /*!< LPTIM2 -> exti[25] Interrupt */

#define SYSCFG_ITLINE19_SR_TIM15_Pos       (0UL)
#define SYSCFG_ITLINE19_SR_TIM15_Msk       (0x1UL << SYSCFG_ITLINE19_SR_TIM15_Pos)     /*!< 0x00000001 */
#define SYSCFG_ITLINE19_SR_TIM15           SYSCFG_ITLINE19_SR_TIM15_Msk                /*!< TIM15 GLB Interrupt */
#define SYSCFG_ITLINE19_SR_LPTIM3_Pos      (1UL)
#define SYSCFG_ITLINE19_SR_LPTIM3_Msk      (0x1UL << SYSCFG_ITLINE19_SR_LPTIM3_Pos)    /*!< 0x00000002 */
#define SYSCFG_ITLINE19_SR_LPTIM3          SYSCFG_ITLINE19_SR_LPTIM3_Msk               /*!< LPTIM3 GLB Interrupt  -> exti [26]*/

#define SYSCFG_ITLINE20_SR_TIM16_Pos       (0UL)
#define SYSCFG_ITLINE20_SR_TIM16_Msk       (0x1UL << SYSCFG_ITLINE20_SR_TIM16_Pos)     /*!< 0x00000001 */
#define SYSCFG_ITLINE20_SR_TIM16           SYSCFG_ITLINE20_SR_TIM16_Msk                /*!< TIM16 GLB Interrupt */

#define SYSCFG_ITLINE21_SR_TSC_MCE_Pos     (0UL)
#define SYSCFG_ITLINE21_SR_TSC_MCE_Msk     (0x1UL << SYSCFG_ITLINE21_SR_TSC_MCE_Pos)   /*!< 0x00000001 */
#define SYSCFG_ITLINE21_SR_TSC_MCE         SYSCFG_ITLINE21_SR_TSC_MCE_Msk              /*!< TSC_MCE Interrupt */
#define SYSCFG_ITLINE21_SR_TSC_EOA_Pos     (1UL)
#define SYSCFG_ITLINE21_SR_TSC_EOA_Msk     (0x1UL << SYSCFG_ITLINE21_SR_TSC_EOA_Pos)   /*!< 0x00000001 */
#define SYSCFG_ITLINE21_SR_TSC_EOA         SYSCFG_ITLINE21_SR_TSC_EOA_Msk              /*!< TSC_MCE Interrupt */

#define SYSCFG_ITLINE22_SR_LCD_Pos         (0UL)
#define SYSCFG_ITLINE22_SR_LCD_Msk         (0x1UL << SYSCFG_ITLINE22_SR_LCD_Pos)       /*!< 0x00000001 */
#define SYSCFG_ITLINE22_SR_LCD             SYSCFG_ITLINE22_SR_LCD_Msk                  /*!< LCD GLB Interrupt */

#define SYSCFG_ITLINE23_SR_I2C1_Pos        (0UL)
#define SYSCFG_ITLINE23_SR_I2C1_Msk        (0x1UL << SYSCFG_ITLINE23_SR_I2C1_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE23_SR_I2C1            SYSCFG_ITLINE23_SR_I2C1_Msk                 /*!< I2C1 GLB Interrupt */

#define SYSCFG_ITLINE24_SR_I2C2_Pos        (0UL)
#define SYSCFG_ITLINE24_SR_I2C2_Msk        (0x1UL << SYSCFG_ITLINE24_SR_I2C2_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE24_SR_I2C2            SYSCFG_ITLINE24_SR_I2C2_Msk                 /*!< I2C2 GLB Interrupt */
#define SYSCFG_ITLINE24_SR_I2C4_Pos        (1UL)
#define SYSCFG_ITLINE24_SR_I2C4_Msk        (0x1UL << SYSCFG_ITLINE24_SR_I2C4_Pos)      /*!< 0x00000002 */
#define SYSCFG_ITLINE24_SR_I2C4            SYSCFG_ITLINE24_SR_I2C4_Msk                 /*!< I2C3 GLB Interrupt */
#define SYSCFG_ITLINE24_SR_I2C3_Pos        (2UL)
#define SYSCFG_ITLINE24_SR_I2C3_Msk        (0x1UL << SYSCFG_ITLINE24_SR_I2C3_Pos)      /*!< 0x00000004 */
#define SYSCFG_ITLINE24_SR_I2C3            SYSCFG_ITLINE24_SR_I2C3_Msk                 /*!< I2C3 GLB Interrupt  -> exti[23]*/

#define SYSCFG_ITLINE25_SR_SPI1_Pos        (0UL)
#define SYSCFG_ITLINE25_SR_SPI1_Msk        (0x1UL << SYSCFG_ITLINE25_SR_SPI1_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE25_SR_SPI1            SYSCFG_ITLINE25_SR_SPI1_Msk                 /*!< SPI1 Interrupt */

#define SYSCFG_ITLINE26_SR_SPI2_Pos        (0UL)
#define SYSCFG_ITLINE26_SR_SPI2_Msk        (0x1UL << SYSCFG_ITLINE26_SR_SPI2_Pos)      /*!< 0x00000001 */
#define SYSCFG_ITLINE26_SR_SPI2            SYSCFG_ITLINE26_SR_SPI2_Msk                 /*!< SPI2  Interrupt */
#define SYSCFG_ITLINE26_SR_SPI3_Pos        (1UL)
#define SYSCFG_ITLINE26_SR_SPI3_Msk        (0x1UL << SYSCFG_ITLINE26_SR_SPI3_Pos)      /*!< 0x00000002 */
#define SYSCFG_ITLINE26_SR_SPI3            SYSCFG_ITLINE26_SR_SPI3_Msk                 /*!< SPI3  Interrupt */

#define SYSCFG_ITLINE27_SR_USART1_Pos      (0UL)
#define SYSCFG_ITLINE27_SR_USART1_Msk      (0x1UL << SYSCFG_ITLINE27_SR_USART1_Pos)    /*!< 0x00000001 */
#define SYSCFG_ITLINE27_SR_USART1          SYSCFG_ITLINE27_SR_USART1_Msk               /*!< USART1 GLB Interrupt */

#define SYSCFG_ITLINE28_SR_USART2_Pos      (0UL)
#define SYSCFG_ITLINE28_SR_USART2_Msk      (0x1UL << SYSCFG_ITLINE28_SR_USART2_Pos)    /*!< 0x00000001 */
#define SYSCFG_ITLINE28_SR_USART2          SYSCFG_ITLINE28_SR_USART2_Msk               /*!< USART2 GLB Interrupt */
#define SYSCFG_ITLINE28_SR_LPUART2_Pos     (1UL)
#define SYSCFG_ITLINE28_SR_LPUART2_Msk     (0x1UL << SYSCFG_ITLINE28_SR_LPUART2_Pos)   /*!< 0x00000002 */
#define SYSCFG_ITLINE28_SR_LPUART2         SYSCFG_ITLINE28_SR_LPUART2_Msk              /*!< LPUART2 GLB Interrupt -> exti[31] */

#define SYSCFG_ITLINE29_SR_USART3_Pos      (0UL)
#define SYSCFG_ITLINE29_SR_USART3_Msk      (0x1UL << SYSCFG_ITLINE29_SR_USART3_Pos)    /*!< 0x00000001 */
#define SYSCFG_ITLINE29_SR_USART3          SYSCFG_ITLINE29_SR_USART3_Msk               /*!< USART3 GLB Interrupt */
#define SYSCFG_ITLINE29_SR_LPUART1_Pos     (1UL)
#define SYSCFG_ITLINE29_SR_LPUART1_Msk     (0x1UL << SYSCFG_ITLINE29_SR_LPUART1_Pos)   /*!< 0x00000002 */
#define SYSCFG_ITLINE29_SR_LPUART1         SYSCFG_ITLINE29_SR_LPUART1_Msk              /*!< LPUART1 GLB Interrupt -> exti[30] */

#define SYSCFG_ITLINE30_SR_USART4_Pos      (0UL)
#define SYSCFG_ITLINE30_SR_USART4_Msk      (0x1UL << SYSCFG_ITLINE30_SR_USART4_Pos)    /*!< 0x00000001 */
#define SYSCFG_ITLINE30_SR_USART4          SYSCFG_ITLINE30_SR_USART4_Msk               /*!< USART4 GLB Interrupt */
#define SYSCFG_ITLINE30_SR_LPUART3_Pos     (1UL)
#define SYSCFG_ITLINE30_SR_LPUART3_Msk     (0x1UL << SYSCFG_ITLINE30_SR_LPUART3_Pos)   /*!< 0x00000002 */
#define SYSCFG_ITLINE30_SR_LPUART3         SYSCFG_ITLINE30_SR_LPUART3_Msk              /*!< LPUART3 GLB Interrupt */

#define SYSCFG_ITLINE31_SR_RNG_Pos         (0UL)
#define SYSCFG_ITLINE31_SR_RNG_Msk         (0x1UL << SYSCFG_ITLINE31_SR_RNG_Pos)       /*!< 0x00000001 */
#define SYSCFG_ITLINE31_SR_RNG             SYSCFG_ITLINE31_SR_RNG_Msk                  /*!< RNG Interrupt */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define TIM_CR1_CEN_Pos           (0UL)
#define TIM_CR1_CEN_Msk           (0x1UL << TIM_CR1_CEN_Pos)                   /*!< 0x00000001 */
#define TIM_CR1_CEN               TIM_CR1_CEN_Msk                              /*!<Counter enable */
#define TIM_CR1_UDIS_Pos          (1UL)
#define TIM_CR1_UDIS_Msk          (0x1UL << TIM_CR1_UDIS_Pos)                  /*!< 0x00000002 */
#define TIM_CR1_UDIS              TIM_CR1_UDIS_Msk                             /*!<Update disable */
#define TIM_CR1_URS_Pos           (2UL)
#define TIM_CR1_URS_Msk           (0x1UL << TIM_CR1_URS_Pos)                   /*!< 0x00000004 */
#define TIM_CR1_URS               TIM_CR1_URS_Msk                              /*!<Update request source */
#define TIM_CR1_OPM_Pos           (3UL)
#define TIM_CR1_OPM_Msk           (0x1UL << TIM_CR1_OPM_Pos)                   /*!< 0x00000008 */
#define TIM_CR1_OPM               TIM_CR1_OPM_Msk                              /*!<One pulse mode */
#define TIM_CR1_DIR_Pos           (4UL)
#define TIM_CR1_DIR_Msk           (0x1UL << TIM_CR1_DIR_Pos)                   /*!< 0x00000010 */
#define TIM_CR1_DIR               TIM_CR1_DIR_Msk                              /*!<Direction */

#define TIM_CR1_CMS_Pos           (5UL)
#define TIM_CR1_CMS_Msk           (0x3UL << TIM_CR1_CMS_Pos)                   /*!< 0x00000060 */
#define TIM_CR1_CMS               TIM_CR1_CMS_Msk                              /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0             (0x1UL << TIM_CR1_CMS_Pos)                   /*!< 0x00000020 */
#define TIM_CR1_CMS_1             (0x2UL << TIM_CR1_CMS_Pos)                   /*!< 0x00000040 */

#define TIM_CR1_ARPE_Pos          (7UL)
#define TIM_CR1_ARPE_Msk          (0x1UL << TIM_CR1_ARPE_Pos)                  /*!< 0x00000080 */
#define TIM_CR1_ARPE              TIM_CR1_ARPE_Msk                             /*!<Auto-reload preload enable */

#define TIM_CR1_CKD_Pos           (8UL)
#define TIM_CR1_CKD_Msk           (0x3UL << TIM_CR1_CKD_Pos)                   /*!< 0x00000300 */
#define TIM_CR1_CKD               TIM_CR1_CKD_Msk                              /*!<CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0             (0x1UL << TIM_CR1_CKD_Pos)                   /*!< 0x00000100 */
#define TIM_CR1_CKD_1             (0x2UL << TIM_CR1_CKD_Pos)                   /*!< 0x00000200 */

#define TIM_CR1_UIFREMAP_Pos      (11UL)
#define TIM_CR1_UIFREMAP_Msk      (0x1UL << TIM_CR1_UIFREMAP_Pos)              /*!< 0x00000800 */
#define TIM_CR1_UIFREMAP          TIM_CR1_UIFREMAP_Msk                         /*!<Update interrupt flag remap */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define TIM_CR2_CCPC_Pos          (0UL)
#define TIM_CR2_CCPC_Msk          (0x1UL << TIM_CR2_CCPC_Pos)                  /*!< 0x00000001 */
#define TIM_CR2_CCPC              TIM_CR2_CCPC_Msk                             /*!<Capture/Compare Preloaded Control */
#define TIM_CR2_CCUS_Pos          (2UL)
#define TIM_CR2_CCUS_Msk          (0x1UL << TIM_CR2_CCUS_Pos)                  /*!< 0x00000004 */
#define TIM_CR2_CCUS              TIM_CR2_CCUS_Msk                             /*!<Capture/Compare Control Update Selection */
#define TIM_CR2_CCDS_Pos          (3UL)
#define TIM_CR2_CCDS_Msk          (0x1UL << TIM_CR2_CCDS_Pos)                  /*!< 0x00000008 */
#define TIM_CR2_CCDS              TIM_CR2_CCDS_Msk                             /*!<Capture/Compare DMA Selection */

#define TIM_CR2_MMS_Pos           (4UL)
#define TIM_CR2_MMS_Msk           (0x7UL << TIM_CR2_MMS_Pos)                   /*!< 0x00000070 */
#define TIM_CR2_MMS               TIM_CR2_MMS_Msk                              /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0             (0x1UL << TIM_CR2_MMS_Pos)                   /*!< 0x00000010 */
#define TIM_CR2_MMS_1             (0x2UL << TIM_CR2_MMS_Pos)                   /*!< 0x00000020 */
#define TIM_CR2_MMS_2             (0x4UL << TIM_CR2_MMS_Pos)                   /*!< 0x00000040 */

#define TIM_CR2_TI1S_Pos          (7UL)
#define TIM_CR2_TI1S_Msk          (0x1UL << TIM_CR2_TI1S_Pos)                  /*!< 0x00000080 */
#define TIM_CR2_TI1S              TIM_CR2_TI1S_Msk                             /*!<TI1 Selection */
#define TIM_CR2_OIS1_Pos          (8UL)
#define TIM_CR2_OIS1_Msk          (0x1UL << TIM_CR2_OIS1_Pos)                  /*!< 0x00000100 */
#define TIM_CR2_OIS1              TIM_CR2_OIS1_Msk                             /*!<Output Idle state 1 (OC1 output) */
#define TIM_CR2_OIS1N_Pos         (9UL)
#define TIM_CR2_OIS1N_Msk         (0x1UL << TIM_CR2_OIS1N_Pos)                 /*!< 0x00000200 */
#define TIM_CR2_OIS1N             TIM_CR2_OIS1N_Msk                            /*!<Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2_Pos          (10UL)
#define TIM_CR2_OIS2_Msk          (0x1UL << TIM_CR2_OIS2_Pos)                  /*!< 0x00000400 */
#define TIM_CR2_OIS2              TIM_CR2_OIS2_Msk                             /*!<Output Idle state 2 (OC2 output) */
#define TIM_CR2_OIS2N_Pos         (11UL)
#define TIM_CR2_OIS2N_Msk         (0x1UL << TIM_CR2_OIS2N_Pos)                 /*!< 0x00000800 */
#define TIM_CR2_OIS2N             TIM_CR2_OIS2N_Msk                            /*!<Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3_Pos          (12UL)
#define TIM_CR2_OIS3_Msk          (0x1UL << TIM_CR2_OIS3_Pos)                  /*!< 0x00001000 */
#define TIM_CR2_OIS3              TIM_CR2_OIS3_Msk                             /*!<Output Idle state 3 (OC3 output) */
#define TIM_CR2_OIS3N_Pos         (13UL)
#define TIM_CR2_OIS3N_Msk         (0x1UL << TIM_CR2_OIS3N_Pos)                 /*!< 0x00002000 */
#define TIM_CR2_OIS3N             TIM_CR2_OIS3N_Msk                            /*!<Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4_Pos          (14UL)
#define TIM_CR2_OIS4_Msk          (0x1UL << TIM_CR2_OIS4_Pos)                  /*!< 0x00004000 */
#define TIM_CR2_OIS4              TIM_CR2_OIS4_Msk                             /*!<Output Idle state 4 (OC4 output) */
#define TIM_CR2_OIS5_Pos          (16UL)
#define TIM_CR2_OIS5_Msk          (0x1UL << TIM_CR2_OIS5_Pos)                  /*!< 0x00010000 */
#define TIM_CR2_OIS5              TIM_CR2_OIS5_Msk                             /*!<Output Idle state 5 (OC5 output) */
#define TIM_CR2_OIS6_Pos          (18UL)
#define TIM_CR2_OIS6_Msk          (0x1UL << TIM_CR2_OIS6_Pos)                  /*!< 0x00040000 */
#define TIM_CR2_OIS6              TIM_CR2_OIS6_Msk                             /*!<Output Idle state 6 (OC6 output) */

#define TIM_CR2_MMS2_Pos          (20UL)
#define TIM_CR2_MMS2_Msk          (0xFUL << TIM_CR2_MMS2_Pos)                  /*!< 0x00F00000 */
#define TIM_CR2_MMS2              TIM_CR2_MMS2_Msk                             /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS2_0            (0x1UL << TIM_CR2_MMS2_Pos)                  /*!< 0x00100000 */
#define TIM_CR2_MMS2_1            (0x2UL << TIM_CR2_MMS2_Pos)                  /*!< 0x00200000 */
#define TIM_CR2_MMS2_2            (0x4UL << TIM_CR2_MMS2_Pos)                  /*!< 0x00400000 */
#define TIM_CR2_MMS2_3            (0x8UL << TIM_CR2_MMS2_Pos)                  /*!< 0x00800000 */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define TIM_SMCR_SMS_Pos          (0UL)
#define TIM_SMCR_SMS_Msk          (0x10007UL << TIM_SMCR_SMS_Pos)              /*!< 0x00010007 */
#define TIM_SMCR_SMS              TIM_SMCR_SMS_Msk                             /*!<SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCR_SMS_0            (0x00001UL << TIM_SMCR_SMS_Pos)              /*!< 0x00000001 */
#define TIM_SMCR_SMS_1            (0x00002UL << TIM_SMCR_SMS_Pos)              /*!< 0x00000002 */
#define TIM_SMCR_SMS_2            (0x00004UL << TIM_SMCR_SMS_Pos)              /*!< 0x00000004 */
#define TIM_SMCR_SMS_3            (0x10000UL << TIM_SMCR_SMS_Pos)              /*!< 0x00010000 */

#define TIM_SMCR_OCCS_Pos         (3UL)
#define TIM_SMCR_OCCS_Msk         (0x1UL << TIM_SMCR_OCCS_Pos)                 /*!< 0x00000008 */
#define TIM_SMCR_OCCS             TIM_SMCR_OCCS_Msk                            /*!< OCREF clear selection */

#define TIM_SMCR_TS_Pos           (4UL)
#define TIM_SMCR_TS_Msk           (0x30007UL << TIM_SMCR_TS_Pos)               /*!< 0x00300070 */
#define TIM_SMCR_TS               TIM_SMCR_TS_Msk                              /*!<TS[2:0] bits (Trigger selection) */
#define TIM_SMCR_TS_0             (0x00001UL << TIM_SMCR_TS_Pos)               /*!< 0x00000010 */
#define TIM_SMCR_TS_1             (0x00002UL << TIM_SMCR_TS_Pos)               /*!< 0x00000020 */
#define TIM_SMCR_TS_2             (0x00004UL << TIM_SMCR_TS_Pos)               /*!< 0x00000040 */
#define TIM_SMCR_TS_3             (0x10000UL << TIM_SMCR_TS_Pos)               /*!< 0x00100000 */
#define TIM_SMCR_TS_4             (0x20000UL << TIM_SMCR_TS_Pos)               /*!< 0x00200000 */

#define TIM_SMCR_MSM_Pos          (7UL)
#define TIM_SMCR_MSM_Msk          (0x1UL << TIM_SMCR_MSM_Pos)                  /*!< 0x00000080 */
#define TIM_SMCR_MSM              TIM_SMCR_MSM_Msk                             /*!<Master/slave mode */

#define TIM_SMCR_ETF_Pos          (8UL)
#define TIM_SMCR_ETF_Msk          (0xFUL << TIM_SMCR_ETF_Pos)                  /*!< 0x00000F00 */
#define TIM_SMCR_ETF              TIM_SMCR_ETF_Msk                             /*!<ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0            (0x1UL << TIM_SMCR_ETF_Pos)                  /*!< 0x00000100 */
#define TIM_SMCR_ETF_1            (0x2UL << TIM_SMCR_ETF_Pos)                  /*!< 0x00000200 */
#define TIM_SMCR_ETF_2            (0x4UL << TIM_SMCR_ETF_Pos)                  /*!< 0x00000400 */
#define TIM_SMCR_ETF_3            (0x8UL << TIM_SMCR_ETF_Pos)                  /*!< 0x00000800 */

#define TIM_SMCR_ETPS_Pos         (12UL)
#define TIM_SMCR_ETPS_Msk         (0x3UL << TIM_SMCR_ETPS_Pos)                 /*!< 0x00003000 */
#define TIM_SMCR_ETPS             TIM_SMCR_ETPS_Msk                            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0           (0x1UL << TIM_SMCR_ETPS_Pos)                 /*!< 0x00001000 */
#define TIM_SMCR_ETPS_1           (0x2UL << TIM_SMCR_ETPS_Pos)                 /*!< 0x00002000 */

#define TIM_SMCR_ECE_Pos          (14UL)
#define TIM_SMCR_ECE_Msk          (0x1UL << TIM_SMCR_ECE_Pos)                  /*!< 0x00004000 */
#define TIM_SMCR_ECE              TIM_SMCR_ECE_Msk                             /*!<External clock enable */
#define TIM_SMCR_ETP_Pos          (15UL)
#define TIM_SMCR_ETP_Msk          (0x1UL << TIM_SMCR_ETP_Pos)                  /*!< 0x00008000 */
#define TIM_SMCR_ETP              TIM_SMCR_ETP_Msk                             /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define TIM_DIER_UIE_Pos          (0UL)
#define TIM_DIER_UIE_Msk          (0x1UL << TIM_DIER_UIE_Pos)                  /*!< 0x00000001 */
#define TIM_DIER_UIE              TIM_DIER_UIE_Msk                             /*!<Update interrupt enable */
#define TIM_DIER_CC1IE_Pos        (1UL)
#define TIM_DIER_CC1IE_Msk        (0x1UL << TIM_DIER_CC1IE_Pos)                /*!< 0x00000002 */
#define TIM_DIER_CC1IE            TIM_DIER_CC1IE_Msk                           /*!<Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE_Pos        (2UL)
#define TIM_DIER_CC2IE_Msk        (0x1UL << TIM_DIER_CC2IE_Pos)                /*!< 0x00000004 */
#define TIM_DIER_CC2IE            TIM_DIER_CC2IE_Msk                           /*!<Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE_Pos        (3UL)
#define TIM_DIER_CC3IE_Msk        (0x1UL << TIM_DIER_CC3IE_Pos)                /*!< 0x00000008 */
#define TIM_DIER_CC3IE            TIM_DIER_CC3IE_Msk                           /*!<Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE_Pos        (4UL)
#define TIM_DIER_CC4IE_Msk        (0x1UL << TIM_DIER_CC4IE_Pos)                /*!< 0x00000010 */
#define TIM_DIER_CC4IE            TIM_DIER_CC4IE_Msk                           /*!<Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE_Pos        (5UL)
#define TIM_DIER_COMIE_Msk        (0x1UL << TIM_DIER_COMIE_Pos)                /*!< 0x00000020 */
#define TIM_DIER_COMIE            TIM_DIER_COMIE_Msk                           /*!<COM interrupt enable */
#define TIM_DIER_TIE_Pos          (6UL)
#define TIM_DIER_TIE_Msk          (0x1UL << TIM_DIER_TIE_Pos)                  /*!< 0x00000040 */
#define TIM_DIER_TIE              TIM_DIER_TIE_Msk                             /*!<Trigger interrupt enable */
#define TIM_DIER_BIE_Pos          (7UL)
#define TIM_DIER_BIE_Msk          (0x1UL << TIM_DIER_BIE_Pos)                  /*!< 0x00000080 */
#define TIM_DIER_BIE              TIM_DIER_BIE_Msk                             /*!<Break interrupt enable */
#define TIM_DIER_UDE_Pos          (8UL)
#define TIM_DIER_UDE_Msk          (0x1UL << TIM_DIER_UDE_Pos)                  /*!< 0x00000100 */
#define TIM_DIER_UDE              TIM_DIER_UDE_Msk                             /*!<Update DMA request enable */
#define TIM_DIER_CC1DE_Pos        (9UL)
#define TIM_DIER_CC1DE_Msk        (0x1UL << TIM_DIER_CC1DE_Pos)                /*!< 0x00000200 */
#define TIM_DIER_CC1DE            TIM_DIER_CC1DE_Msk                           /*!<Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE_Pos        (10UL)
#define TIM_DIER_CC2DE_Msk        (0x1UL << TIM_DIER_CC2DE_Pos)                /*!< 0x00000400 */
#define TIM_DIER_CC2DE            TIM_DIER_CC2DE_Msk                           /*!<Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE_Pos        (11UL)
#define TIM_DIER_CC3DE_Msk        (0x1UL << TIM_DIER_CC3DE_Pos)                /*!< 0x00000800 */
#define TIM_DIER_CC3DE            TIM_DIER_CC3DE_Msk                           /*!<Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE_Pos        (12UL)
#define TIM_DIER_CC4DE_Msk        (0x1UL << TIM_DIER_CC4DE_Pos)                /*!< 0x00001000 */
#define TIM_DIER_CC4DE            TIM_DIER_CC4DE_Msk                           /*!<Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE_Pos        (13UL)
#define TIM_DIER_COMDE_Msk        (0x1UL << TIM_DIER_COMDE_Pos)                /*!< 0x00002000 */
#define TIM_DIER_COMDE            TIM_DIER_COMDE_Msk                           /*!<COM DMA request enable */
#define TIM_DIER_TDE_Pos          (14UL)
#define TIM_DIER_TDE_Msk          (0x1UL << TIM_DIER_TDE_Pos)                  /*!< 0x00004000 */
#define TIM_DIER_TDE              TIM_DIER_TDE_Msk                             /*!<Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  ********************/
#define TIM_SR_UIF_Pos            (0UL)
#define TIM_SR_UIF_Msk            (0x1UL << TIM_SR_UIF_Pos)                    /*!< 0x00000001 */
#define TIM_SR_UIF                TIM_SR_UIF_Msk                               /*!<Update interrupt Flag */
#define TIM_SR_CC1IF_Pos          (1UL)
#define TIM_SR_CC1IF_Msk          (0x1UL << TIM_SR_CC1IF_Pos)                  /*!< 0x00000002 */
#define TIM_SR_CC1IF              TIM_SR_CC1IF_Msk                             /*!<Capture/Compare 1 interrupt Flag */
#define TIM_SR_CC2IF_Pos          (2UL)
#define TIM_SR_CC2IF_Msk          (0x1UL << TIM_SR_CC2IF_Pos)                  /*!< 0x00000004 */
#define TIM_SR_CC2IF              TIM_SR_CC2IF_Msk                             /*!<Capture/Compare 2 interrupt Flag */
#define TIM_SR_CC3IF_Pos          (3UL)
#define TIM_SR_CC3IF_Msk          (0x1UL << TIM_SR_CC3IF_Pos)                  /*!< 0x00000008 */
#define TIM_SR_CC3IF              TIM_SR_CC3IF_Msk                             /*!<Capture/Compare 3 interrupt Flag */
#define TIM_SR_CC4IF_Pos          (4UL)
#define TIM_SR_CC4IF_Msk          (0x1UL << TIM_SR_CC4IF_Pos)                  /*!< 0x00000010 */
#define TIM_SR_CC4IF              TIM_SR_CC4IF_Msk                             /*!<Capture/Compare 4 interrupt Flag */
#define TIM_SR_COMIF_Pos          (5UL)
#define TIM_SR_COMIF_Msk          (0x1UL << TIM_SR_COMIF_Pos)                  /*!< 0x00000020 */
#define TIM_SR_COMIF              TIM_SR_COMIF_Msk                             /*!<COM interrupt Flag */
#define TIM_SR_TIF_Pos            (6UL)
#define TIM_SR_TIF_Msk            (0x1UL << TIM_SR_TIF_Pos)                    /*!< 0x00000040 */
#define TIM_SR_TIF                TIM_SR_TIF_Msk                               /*!<Trigger interrupt Flag */
#define TIM_SR_BIF_Pos            (7UL)
#define TIM_SR_BIF_Msk            (0x1UL << TIM_SR_BIF_Pos)                    /*!< 0x00000080 */
#define TIM_SR_BIF                TIM_SR_BIF_Msk                               /*!<Break interrupt Flag */
#define TIM_SR_B2IF_Pos           (8UL)
#define TIM_SR_B2IF_Msk           (0x1UL << TIM_SR_B2IF_Pos)                   /*!< 0x00000100 */
#define TIM_SR_B2IF               TIM_SR_B2IF_Msk                              /*!<Break 2 interrupt Flag */
#define TIM_SR_CC1OF_Pos          (9UL)
#define TIM_SR_CC1OF_Msk          (0x1UL << TIM_SR_CC1OF_Pos)                  /*!< 0x00000200 */
#define TIM_SR_CC1OF              TIM_SR_CC1OF_Msk                             /*!<Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF_Pos          (10UL)
#define TIM_SR_CC2OF_Msk          (0x1UL << TIM_SR_CC2OF_Pos)                  /*!< 0x00000400 */
#define TIM_SR_CC2OF              TIM_SR_CC2OF_Msk                             /*!<Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF_Pos          (11UL)
#define TIM_SR_CC3OF_Msk          (0x1UL << TIM_SR_CC3OF_Pos)                  /*!< 0x00000800 */
#define TIM_SR_CC3OF              TIM_SR_CC3OF_Msk                             /*!<Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF_Pos          (12UL)
#define TIM_SR_CC4OF_Msk          (0x1UL << TIM_SR_CC4OF_Pos)                  /*!< 0x00001000 */
#define TIM_SR_CC4OF              TIM_SR_CC4OF_Msk                             /*!<Capture/Compare 4 Overcapture Flag */
#define TIM_SR_SBIF_Pos           (13UL)
#define TIM_SR_SBIF_Msk           (0x1UL << TIM_SR_SBIF_Pos)                   /*!< 0x00002000 */
#define TIM_SR_SBIF               TIM_SR_SBIF_Msk                              /*!<System Break interrupt Flag */
#define TIM_SR_CC5IF_Pos          (16UL)
#define TIM_SR_CC5IF_Msk          (0x1UL << TIM_SR_CC5IF_Pos)                  /*!< 0x00010000 */
#define TIM_SR_CC5IF              TIM_SR_CC5IF_Msk                             /*!<Capture/Compare 5 interrupt Flag */
#define TIM_SR_CC6IF_Pos          (17UL)
#define TIM_SR_CC6IF_Msk          (0x1UL << TIM_SR_CC6IF_Pos)                  /*!< 0x00020000 */
#define TIM_SR_CC6IF              TIM_SR_CC6IF_Msk                             /*!<Capture/Compare 6 interrupt Flag */


/*******************  Bit definition for TIM_EGR register  ********************/
#define TIM_EGR_UG_Pos            (0UL)
#define TIM_EGR_UG_Msk            (0x1UL << TIM_EGR_UG_Pos)                    /*!< 0x00000001 */
#define TIM_EGR_UG                TIM_EGR_UG_Msk                               /*!<Update Generation */
#define TIM_EGR_CC1G_Pos          (1UL)
#define TIM_EGR_CC1G_Msk          (0x1UL << TIM_EGR_CC1G_Pos)                  /*!< 0x00000002 */
#define TIM_EGR_CC1G              TIM_EGR_CC1G_Msk                             /*!<Capture/Compare 1 Generation */
#define TIM_EGR_CC2G_Pos          (2UL)
#define TIM_EGR_CC2G_Msk          (0x1UL << TIM_EGR_CC2G_Pos)                  /*!< 0x00000004 */
#define TIM_EGR_CC2G              TIM_EGR_CC2G_Msk                             /*!<Capture/Compare 2 Generation */
#define TIM_EGR_CC3G_Pos          (3UL)
#define TIM_EGR_CC3G_Msk          (0x1UL << TIM_EGR_CC3G_Pos)                  /*!< 0x00000008 */
#define TIM_EGR_CC3G              TIM_EGR_CC3G_Msk                             /*!<Capture/Compare 3 Generation */
#define TIM_EGR_CC4G_Pos          (4UL)
#define TIM_EGR_CC4G_Msk          (0x1UL << TIM_EGR_CC4G_Pos)                  /*!< 0x00000010 */
#define TIM_EGR_CC4G              TIM_EGR_CC4G_Msk                             /*!<Capture/Compare 4 Generation */
#define TIM_EGR_COMG_Pos          (5UL)
#define TIM_EGR_COMG_Msk          (0x1UL << TIM_EGR_COMG_Pos)                  /*!< 0x00000020 */
#define TIM_EGR_COMG              TIM_EGR_COMG_Msk                             /*!<Capture/Compare Control Update Generation */
#define TIM_EGR_TG_Pos            (6UL)
#define TIM_EGR_TG_Msk            (0x1UL << TIM_EGR_TG_Pos)                    /*!< 0x00000040 */
#define TIM_EGR_TG                TIM_EGR_TG_Msk                               /*!<Trigger Generation */
#define TIM_EGR_BG_Pos            (7UL)
#define TIM_EGR_BG_Msk            (0x1UL << TIM_EGR_BG_Pos)                    /*!< 0x00000080 */
#define TIM_EGR_BG                TIM_EGR_BG_Msk                               /*!<Break Generation */
#define TIM_EGR_B2G_Pos           (8UL)
#define TIM_EGR_B2G_Msk           (0x1UL << TIM_EGR_B2G_Pos)                   /*!< 0x00000100 */
#define TIM_EGR_B2G               TIM_EGR_B2G_Msk                              /*!<Break 2 Generation */


/******************  Bit definition for TIM_CCMR1 register  *******************/
#define TIM_CCMR1_CC1S_Pos        (0UL)
#define TIM_CCMR1_CC1S_Msk        (0x3UL << TIM_CCMR1_CC1S_Pos)                /*!< 0x00000003 */
#define TIM_CCMR1_CC1S            TIM_CCMR1_CC1S_Msk                           /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0          (0x1UL << TIM_CCMR1_CC1S_Pos)                /*!< 0x00000001 */
#define TIM_CCMR1_CC1S_1          (0x2UL << TIM_CCMR1_CC1S_Pos)                /*!< 0x00000002 */

#define TIM_CCMR1_OC1FE_Pos       (2UL)
#define TIM_CCMR1_OC1FE_Msk       (0x1UL << TIM_CCMR1_OC1FE_Pos)               /*!< 0x00000004 */
#define TIM_CCMR1_OC1FE           TIM_CCMR1_OC1FE_Msk                          /*!<Output Compare 1 Fast enable */
#define TIM_CCMR1_OC1PE_Pos       (3UL)
#define TIM_CCMR1_OC1PE_Msk       (0x1UL << TIM_CCMR1_OC1PE_Pos)               /*!< 0x00000008 */
#define TIM_CCMR1_OC1PE           TIM_CCMR1_OC1PE_Msk                          /*!<Output Compare 1 Preload enable */

#define TIM_CCMR1_OC1M_Pos        (4UL)
#define TIM_CCMR1_OC1M_Msk        (0x1007UL << TIM_CCMR1_OC1M_Pos)             /*!< 0x00010070 */
#define TIM_CCMR1_OC1M            TIM_CCMR1_OC1M_Msk                           /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1M_0          (0x0001UL << TIM_CCMR1_OC1M_Pos)             /*!< 0x00000010 */
#define TIM_CCMR1_OC1M_1          (0x0002UL << TIM_CCMR1_OC1M_Pos)             /*!< 0x00000020 */
#define TIM_CCMR1_OC1M_2          (0x0004UL << TIM_CCMR1_OC1M_Pos)             /*!< 0x00000040 */
#define TIM_CCMR1_OC1M_3          (0x1000UL << TIM_CCMR1_OC1M_Pos)             /*!< 0x00010000 */

#define TIM_CCMR1_OC1CE_Pos       (7UL)
#define TIM_CCMR1_OC1CE_Msk       (0x1UL << TIM_CCMR1_OC1CE_Pos)               /*!< 0x00000080 */
#define TIM_CCMR1_OC1CE           TIM_CCMR1_OC1CE_Msk                          /*!<Output Compare 1 Clear Enable */

#define TIM_CCMR1_CC2S_Pos        (8UL)
#define TIM_CCMR1_CC2S_Msk        (0x3UL << TIM_CCMR1_CC2S_Pos)                /*!< 0x00000300 */
#define TIM_CCMR1_CC2S            TIM_CCMR1_CC2S_Msk                           /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0          (0x1UL << TIM_CCMR1_CC2S_Pos)                /*!< 0x00000100 */
#define TIM_CCMR1_CC2S_1          (0x2UL << TIM_CCMR1_CC2S_Pos)                /*!< 0x00000200 */

#define TIM_CCMR1_OC2FE_Pos       (10UL)
#define TIM_CCMR1_OC2FE_Msk       (0x1UL << TIM_CCMR1_OC2FE_Pos)               /*!< 0x00000400 */
#define TIM_CCMR1_OC2FE           TIM_CCMR1_OC2FE_Msk                          /*!<Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE_Pos       (11UL)
#define TIM_CCMR1_OC2PE_Msk       (0x1UL << TIM_CCMR1_OC2PE_Pos)               /*!< 0x00000800 */
#define TIM_CCMR1_OC2PE           TIM_CCMR1_OC2PE_Msk                          /*!<Output Compare 2 Preload enable */

#define TIM_CCMR1_OC2M_Pos        (12UL)
#define TIM_CCMR1_OC2M_Msk        (0x1007UL << TIM_CCMR1_OC2M_Pos)             /*!< 0x01007000 */
#define TIM_CCMR1_OC2M            TIM_CCMR1_OC2M_Msk                           /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2M_0          (0x0001UL << TIM_CCMR1_OC2M_Pos)             /*!< 0x00001000 */
#define TIM_CCMR1_OC2M_1          (0x0002UL << TIM_CCMR1_OC2M_Pos)             /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_2          (0x0004UL << TIM_CCMR1_OC2M_Pos)             /*!< 0x00004000 */
#define TIM_CCMR1_OC2M_3          (0x1000UL << TIM_CCMR1_OC2M_Pos)             /*!< 0x01000000 */

#define TIM_CCMR1_OC2CE_Pos       (15UL)
#define TIM_CCMR1_OC2CE_Msk       (0x1UL << TIM_CCMR1_OC2CE_Pos)               /*!< 0x00008000 */
#define TIM_CCMR1_OC2CE           TIM_CCMR1_OC2CE_Msk                          /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/
#define TIM_CCMR1_IC1PSC_Pos      (2UL)
#define TIM_CCMR1_IC1PSC_Msk      (0x3UL << TIM_CCMR1_IC1PSC_Pos)              /*!< 0x0000000C */
#define TIM_CCMR1_IC1PSC          TIM_CCMR1_IC1PSC_Msk                         /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0        (0x1UL << TIM_CCMR1_IC1PSC_Pos)              /*!< 0x00000004 */
#define TIM_CCMR1_IC1PSC_1        (0x2UL << TIM_CCMR1_IC1PSC_Pos)              /*!< 0x00000008 */

#define TIM_CCMR1_IC1F_Pos        (4UL)
#define TIM_CCMR1_IC1F_Msk        (0xFUL << TIM_CCMR1_IC1F_Pos)                /*!< 0x000000F0 */
#define TIM_CCMR1_IC1F            TIM_CCMR1_IC1F_Msk                           /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMR1_IC1F_0          (0x1UL << TIM_CCMR1_IC1F_Pos)                /*!< 0x00000010 */
#define TIM_CCMR1_IC1F_1          (0x2UL << TIM_CCMR1_IC1F_Pos)                /*!< 0x00000020 */
#define TIM_CCMR1_IC1F_2          (0x4UL << TIM_CCMR1_IC1F_Pos)                /*!< 0x00000040 */
#define TIM_CCMR1_IC1F_3          (0x8UL << TIM_CCMR1_IC1F_Pos)                /*!< 0x00000080 */

#define TIM_CCMR1_IC2PSC_Pos      (10UL)
#define TIM_CCMR1_IC2PSC_Msk      (0x3UL << TIM_CCMR1_IC2PSC_Pos)              /*!< 0x00000C00 */
#define TIM_CCMR1_IC2PSC          TIM_CCMR1_IC2PSC_Msk                         /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2PSC_0        (0x1UL << TIM_CCMR1_IC2PSC_Pos)              /*!< 0x00000400 */
#define TIM_CCMR1_IC2PSC_1        (0x2UL << TIM_CCMR1_IC2PSC_Pos)              /*!< 0x00000800 */

#define TIM_CCMR1_IC2F_Pos        (12UL)
#define TIM_CCMR1_IC2F_Msk        (0xFUL << TIM_CCMR1_IC2F_Pos)                /*!< 0x0000F000 */
#define TIM_CCMR1_IC2F            TIM_CCMR1_IC2F_Msk                           /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_CCMR1_IC2F_0          (0x1UL << TIM_CCMR1_IC2F_Pos)                /*!< 0x00001000 */
#define TIM_CCMR1_IC2F_1          (0x2UL << TIM_CCMR1_IC2F_Pos)                /*!< 0x00002000 */
#define TIM_CCMR1_IC2F_2          (0x4UL << TIM_CCMR1_IC2F_Pos)                /*!< 0x00004000 */
#define TIM_CCMR1_IC2F_3          (0x8UL << TIM_CCMR1_IC2F_Pos)                /*!< 0x00008000 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define TIM_CCMR2_CC3S_Pos        (0UL)
#define TIM_CCMR2_CC3S_Msk        (0x3UL << TIM_CCMR2_CC3S_Pos)                /*!< 0x00000003 */
#define TIM_CCMR2_CC3S            TIM_CCMR2_CC3S_Msk                           /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMR2_CC3S_0          (0x1UL << TIM_CCMR2_CC3S_Pos)                /*!< 0x00000001 */
#define TIM_CCMR2_CC3S_1          (0x2UL << TIM_CCMR2_CC3S_Pos)                /*!< 0x00000002 */

#define TIM_CCMR2_OC3FE_Pos       (2UL)
#define TIM_CCMR2_OC3FE_Msk       (0x1UL << TIM_CCMR2_OC3FE_Pos)               /*!< 0x00000004 */
#define TIM_CCMR2_OC3FE           TIM_CCMR2_OC3FE_Msk                          /*!<Output Compare 3 Fast enable */
#define TIM_CCMR2_OC3PE_Pos       (3UL)
#define TIM_CCMR2_OC3PE_Msk       (0x1UL << TIM_CCMR2_OC3PE_Pos)               /*!< 0x00000008 */
#define TIM_CCMR2_OC3PE           TIM_CCMR2_OC3PE_Msk                          /*!<Output Compare 3 Preload enable */

#define TIM_CCMR2_OC3M_Pos        (4UL)
#define TIM_CCMR2_OC3M_Msk        (0x1007UL << TIM_CCMR2_OC3M_Pos)             /*!< 0x00010070 */
#define TIM_CCMR2_OC3M            TIM_CCMR2_OC3M_Msk                           /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0          (0x0001UL << TIM_CCMR2_OC3M_Pos)             /*!< 0x00000010 */
#define TIM_CCMR2_OC3M_1          (0x0002UL << TIM_CCMR2_OC3M_Pos)             /*!< 0x00000020 */
#define TIM_CCMR2_OC3M_2          (0x0004UL << TIM_CCMR2_OC3M_Pos)             /*!< 0x00000040 */
#define TIM_CCMR2_OC3M_3          (0x1000UL << TIM_CCMR2_OC3M_Pos)             /*!< 0x00010000 */

#define TIM_CCMR2_OC3CE_Pos       (7UL)
#define TIM_CCMR2_OC3CE_Msk       (0x1UL << TIM_CCMR2_OC3CE_Pos)               /*!< 0x00000080 */
#define TIM_CCMR2_OC3CE           TIM_CCMR2_OC3CE_Msk                          /*!<Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S_Pos        (8UL)
#define TIM_CCMR2_CC4S_Msk        (0x3UL << TIM_CCMR2_CC4S_Pos)                /*!< 0x00000300 */
#define TIM_CCMR2_CC4S            TIM_CCMR2_CC4S_Msk                           /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0          (0x1UL << TIM_CCMR2_CC4S_Pos)                /*!< 0x00000100 */
#define TIM_CCMR2_CC4S_1          (0x2UL << TIM_CCMR2_CC4S_Pos)                /*!< 0x00000200 */

#define TIM_CCMR2_OC4FE_Pos       (10UL)
#define TIM_CCMR2_OC4FE_Msk       (0x1UL << TIM_CCMR2_OC4FE_Pos)               /*!< 0x00000400 */
#define TIM_CCMR2_OC4FE           TIM_CCMR2_OC4FE_Msk                          /*!<Output Compare 4 Fast enable */
#define TIM_CCMR2_OC4PE_Pos       (11UL)
#define TIM_CCMR2_OC4PE_Msk       (0x1UL << TIM_CCMR2_OC4PE_Pos)               /*!< 0x00000800 */
#define TIM_CCMR2_OC4PE           TIM_CCMR2_OC4PE_Msk                          /*!<Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M_Pos        (12UL)
#define TIM_CCMR2_OC4M_Msk        (0x1007UL << TIM_CCMR2_OC4M_Pos)             /*!< 0x01007000 */
#define TIM_CCMR2_OC4M            TIM_CCMR2_OC4M_Msk                           /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0          (0x0001UL << TIM_CCMR2_OC4M_Pos)             /*!< 0x00001000 */
#define TIM_CCMR2_OC4M_1          (0x0002UL << TIM_CCMR2_OC4M_Pos)             /*!< 0x00002000 */
#define TIM_CCMR2_OC4M_2          (0x0004UL << TIM_CCMR2_OC4M_Pos)             /*!< 0x00004000 */
#define TIM_CCMR2_OC4M_3          (0x1000UL << TIM_CCMR2_OC4M_Pos)             /*!< 0x01000000 */

#define TIM_CCMR2_OC4CE_Pos       (15UL)
#define TIM_CCMR2_OC4CE_Msk       (0x1UL << TIM_CCMR2_OC4CE_Pos)               /*!< 0x00008000 */
#define TIM_CCMR2_OC4CE           TIM_CCMR2_OC4CE_Msk                          /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/
#define TIM_CCMR2_IC3PSC_Pos      (2UL)
#define TIM_CCMR2_IC3PSC_Msk      (0x3UL << TIM_CCMR2_IC3PSC_Pos)              /*!< 0x0000000C */
#define TIM_CCMR2_IC3PSC          TIM_CCMR2_IC3PSC_Msk                         /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0        (0x1UL << TIM_CCMR2_IC3PSC_Pos)              /*!< 0x00000004 */
#define TIM_CCMR2_IC3PSC_1        (0x2UL << TIM_CCMR2_IC3PSC_Pos)              /*!< 0x00000008 */

#define TIM_CCMR2_IC3F_Pos        (4UL)
#define TIM_CCMR2_IC3F_Msk        (0xFUL << TIM_CCMR2_IC3F_Pos)                /*!< 0x000000F0 */
#define TIM_CCMR2_IC3F            TIM_CCMR2_IC3F_Msk                           /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0          (0x1UL << TIM_CCMR2_IC3F_Pos)                /*!< 0x00000010 */
#define TIM_CCMR2_IC3F_1          (0x2UL << TIM_CCMR2_IC3F_Pos)                /*!< 0x00000020 */
#define TIM_CCMR2_IC3F_2          (0x4UL << TIM_CCMR2_IC3F_Pos)                /*!< 0x00000040 */
#define TIM_CCMR2_IC3F_3          (0x8UL << TIM_CCMR2_IC3F_Pos)                /*!< 0x00000080 */

#define TIM_CCMR2_IC4PSC_Pos      (10UL)
#define TIM_CCMR2_IC4PSC_Msk      (0x3UL << TIM_CCMR2_IC4PSC_Pos)              /*!< 0x00000C00 */
#define TIM_CCMR2_IC4PSC          TIM_CCMR2_IC4PSC_Msk                         /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4PSC_0        (0x1UL << TIM_CCMR2_IC4PSC_Pos)              /*!< 0x00000400 */
#define TIM_CCMR2_IC4PSC_1        (0x2UL << TIM_CCMR2_IC4PSC_Pos)              /*!< 0x00000800 */

#define TIM_CCMR2_IC4F_Pos        (12UL)
#define TIM_CCMR2_IC4F_Msk        (0xFUL << TIM_CCMR2_IC4F_Pos)                /*!< 0x0000F000 */
#define TIM_CCMR2_IC4F            TIM_CCMR2_IC4F_Msk                           /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMR2_IC4F_0          (0x1UL << TIM_CCMR2_IC4F_Pos)                /*!< 0x00001000 */
#define TIM_CCMR2_IC4F_1          (0x2UL << TIM_CCMR2_IC4F_Pos)                /*!< 0x00002000 */
#define TIM_CCMR2_IC4F_2          (0x4UL << TIM_CCMR2_IC4F_Pos)                /*!< 0x00004000 */
#define TIM_CCMR2_IC4F_3          (0x8UL << TIM_CCMR2_IC4F_Pos)                /*!< 0x00008000 */

/******************  Bit definition for TIM_CCMR3 register  *******************/
#define TIM_CCMR3_OC5FE_Pos       (2UL)
#define TIM_CCMR3_OC5FE_Msk       (0x1UL << TIM_CCMR3_OC5FE_Pos)               /*!< 0x00000004 */
#define TIM_CCMR3_OC5FE           TIM_CCMR3_OC5FE_Msk                          /*!<Output Compare 5 Fast enable */
#define TIM_CCMR3_OC5PE_Pos       (3UL)
#define TIM_CCMR3_OC5PE_Msk       (0x1UL << TIM_CCMR3_OC5PE_Pos)               /*!< 0x00000008 */
#define TIM_CCMR3_OC5PE           TIM_CCMR3_OC5PE_Msk                          /*!<Output Compare 5 Preload enable */

#define TIM_CCMR3_OC5M_Pos        (4UL)
#define TIM_CCMR3_OC5M_Msk        (0x1007UL << TIM_CCMR3_OC5M_Pos)             /*!< 0x00010070 */
#define TIM_CCMR3_OC5M            TIM_CCMR3_OC5M_Msk                           /*!<OC5M[3:0] bits (Output Compare 5 Mode) */
#define TIM_CCMR3_OC5M_0          (0x0001UL << TIM_CCMR3_OC5M_Pos)             /*!< 0x00000010 */
#define TIM_CCMR3_OC5M_1          (0x0002UL << TIM_CCMR3_OC5M_Pos)             /*!< 0x00000020 */
#define TIM_CCMR3_OC5M_2          (0x0004UL << TIM_CCMR3_OC5M_Pos)             /*!< 0x00000040 */
#define TIM_CCMR3_OC5M_3          (0x1000UL << TIM_CCMR3_OC5M_Pos)             /*!< 0x00010000 */

#define TIM_CCMR3_OC5CE_Pos       (7UL)
#define TIM_CCMR3_OC5CE_Msk       (0x1UL << TIM_CCMR3_OC5CE_Pos)               /*!< 0x00000080 */
#define TIM_CCMR3_OC5CE           TIM_CCMR3_OC5CE_Msk                          /*!<Output Compare 5 Clear Enable */

#define TIM_CCMR3_OC6FE_Pos       (10UL)
#define TIM_CCMR3_OC6FE_Msk       (0x1UL << TIM_CCMR3_OC6FE_Pos)               /*!< 0x00000400 */
#define TIM_CCMR3_OC6FE           TIM_CCMR3_OC6FE_Msk                          /*!<Output Compare 6 Fast enable */
#define TIM_CCMR3_OC6PE_Pos       (11UL)
#define TIM_CCMR3_OC6PE_Msk       (0x1UL << TIM_CCMR3_OC6PE_Pos)               /*!< 0x00000800 */
#define TIM_CCMR3_OC6PE           TIM_CCMR3_OC6PE_Msk                          /*!<Output Compare 6 Preload enable */

#define TIM_CCMR3_OC6M_Pos        (12UL)
#define TIM_CCMR3_OC6M_Msk        (0x1007UL << TIM_CCMR3_OC6M_Pos)             /*!< 0x01007000 */
#define TIM_CCMR3_OC6M            TIM_CCMR3_OC6M_Msk                           /*!<OC6M[3:0] bits (Output Compare 6 Mode) */
#define TIM_CCMR3_OC6M_0          (0x0001UL << TIM_CCMR3_OC6M_Pos)             /*!< 0x00001000 */
#define TIM_CCMR3_OC6M_1          (0x0002UL << TIM_CCMR3_OC6M_Pos)             /*!< 0x00002000 */
#define TIM_CCMR3_OC6M_2          (0x0004UL << TIM_CCMR3_OC6M_Pos)             /*!< 0x00004000 */
#define TIM_CCMR3_OC6M_3          (0x1000UL << TIM_CCMR3_OC6M_Pos)             /*!< 0x01000000 */

#define TIM_CCMR3_OC6CE_Pos       (15UL)
#define TIM_CCMR3_OC6CE_Msk       (0x1UL << TIM_CCMR3_OC6CE_Pos)               /*!< 0x00008000 */
#define TIM_CCMR3_OC6CE           TIM_CCMR3_OC6CE_Msk                          /*!<Output Compare 6 Clear Enable */

/*******************  Bit definition for TIM_CCER register  *******************/
#define TIM_CCER_CC1E_Pos         (0UL)
#define TIM_CCER_CC1E_Msk         (0x1UL << TIM_CCER_CC1E_Pos)                 /*!< 0x00000001 */
#define TIM_CCER_CC1E             TIM_CCER_CC1E_Msk                            /*!<Capture/Compare 1 output enable */
#define TIM_CCER_CC1P_Pos         (1UL)
#define TIM_CCER_CC1P_Msk         (0x1UL << TIM_CCER_CC1P_Pos)                 /*!< 0x00000002 */
#define TIM_CCER_CC1P             TIM_CCER_CC1P_Msk                            /*!<Capture/Compare 1 output Polarity */
#define TIM_CCER_CC1NE_Pos        (2UL)
#define TIM_CCER_CC1NE_Msk        (0x1UL << TIM_CCER_CC1NE_Pos)                /*!< 0x00000004 */
#define TIM_CCER_CC1NE            TIM_CCER_CC1NE_Msk                           /*!<Capture/Compare 1 Complementary output enable */
#define TIM_CCER_CC1NP_Pos        (3UL)
#define TIM_CCER_CC1NP_Msk        (0x1UL << TIM_CCER_CC1NP_Pos)                /*!< 0x00000008 */
#define TIM_CCER_CC1NP            TIM_CCER_CC1NP_Msk                           /*!<Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E_Pos         (4UL)
#define TIM_CCER_CC2E_Msk         (0x1UL << TIM_CCER_CC2E_Pos)                 /*!< 0x00000010 */
#define TIM_CCER_CC2E             TIM_CCER_CC2E_Msk                            /*!<Capture/Compare 2 output enable */
#define TIM_CCER_CC2P_Pos         (5UL)
#define TIM_CCER_CC2P_Msk         (0x1UL << TIM_CCER_CC2P_Pos)                 /*!< 0x00000020 */
#define TIM_CCER_CC2P             TIM_CCER_CC2P_Msk                            /*!<Capture/Compare 2 output Polarity */
#define TIM_CCER_CC2NE_Pos        (6UL)
#define TIM_CCER_CC2NE_Msk        (0x1UL << TIM_CCER_CC2NE_Pos)                /*!< 0x00000040 */
#define TIM_CCER_CC2NE            TIM_CCER_CC2NE_Msk                           /*!<Capture/Compare 2 Complementary output enable */
#define TIM_CCER_CC2NP_Pos        (7UL)
#define TIM_CCER_CC2NP_Msk        (0x1UL << TIM_CCER_CC2NP_Pos)                /*!< 0x00000080 */
#define TIM_CCER_CC2NP            TIM_CCER_CC2NP_Msk                           /*!<Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E_Pos         (8UL)
#define TIM_CCER_CC3E_Msk         (0x1UL << TIM_CCER_CC3E_Pos)                 /*!< 0x00000100 */
#define TIM_CCER_CC3E             TIM_CCER_CC3E_Msk                            /*!<Capture/Compare 3 output enable */
#define TIM_CCER_CC3P_Pos         (9UL)
#define TIM_CCER_CC3P_Msk         (0x1UL << TIM_CCER_CC3P_Pos)                 /*!< 0x00000200 */
#define TIM_CCER_CC3P             TIM_CCER_CC3P_Msk                            /*!<Capture/Compare 3 output Polarity */
#define TIM_CCER_CC3NE_Pos        (10UL)
#define TIM_CCER_CC3NE_Msk        (0x1UL << TIM_CCER_CC3NE_Pos)                /*!< 0x00000400 */
#define TIM_CCER_CC3NE            TIM_CCER_CC3NE_Msk                           /*!<Capture/Compare 3 Complementary output enable */
#define TIM_CCER_CC3NP_Pos        (11UL)
#define TIM_CCER_CC3NP_Msk        (0x1UL << TIM_CCER_CC3NP_Pos)                /*!< 0x00000800 */
#define TIM_CCER_CC3NP            TIM_CCER_CC3NP_Msk                           /*!<Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E_Pos         (12UL)
#define TIM_CCER_CC4E_Msk         (0x1UL << TIM_CCER_CC4E_Pos)                 /*!< 0x00001000 */
#define TIM_CCER_CC4E             TIM_CCER_CC4E_Msk                            /*!<Capture/Compare 4 output enable */
#define TIM_CCER_CC4P_Pos         (13UL)
#define TIM_CCER_CC4P_Msk         (0x1UL << TIM_CCER_CC4P_Pos)                 /*!< 0x00002000 */
#define TIM_CCER_CC4P             TIM_CCER_CC4P_Msk                            /*!<Capture/Compare 4 output Polarity */
#define TIM_CCER_CC4NP_Pos        (15UL)
#define TIM_CCER_CC4NP_Msk        (0x1UL << TIM_CCER_CC4NP_Pos)                /*!< 0x00008000 */
#define TIM_CCER_CC4NP            TIM_CCER_CC4NP_Msk                           /*!<Capture/Compare 4 Complementary output Polarity */
#define TIM_CCER_CC5E_Pos         (16UL)
#define TIM_CCER_CC5E_Msk         (0x1UL << TIM_CCER_CC5E_Pos)                 /*!< 0x00010000 */
#define TIM_CCER_CC5E             TIM_CCER_CC5E_Msk                            /*!<Capture/Compare 5 output enable */
#define TIM_CCER_CC5P_Pos         (17UL)
#define TIM_CCER_CC5P_Msk         (0x1UL << TIM_CCER_CC5P_Pos)                 /*!< 0x00020000 */
#define TIM_CCER_CC5P             TIM_CCER_CC5P_Msk                            /*!<Capture/Compare 5 output Polarity */
#define TIM_CCER_CC6E_Pos         (20UL)
#define TIM_CCER_CC6E_Msk         (0x1UL << TIM_CCER_CC6E_Pos)                 /*!< 0x00100000 */
#define TIM_CCER_CC6E             TIM_CCER_CC6E_Msk                            /*!<Capture/Compare 6 output enable */
#define TIM_CCER_CC6P_Pos         (21UL)
#define TIM_CCER_CC6P_Msk         (0x1UL << TIM_CCER_CC6P_Pos)                 /*!< 0x00200000 */
#define TIM_CCER_CC6P             TIM_CCER_CC6P_Msk                            /*!<Capture/Compare 6 output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define TIM_CNT_CNT_Pos           (0UL)
#define TIM_CNT_CNT_Msk           (0xFFFFFFFFUL << TIM_CNT_CNT_Pos)            /*!< 0xFFFFFFFF */
#define TIM_CNT_CNT               TIM_CNT_CNT_Msk                              /*!<Counter Value */
#define TIM_CNT_UIFCPY_Pos        (31UL)
#define TIM_CNT_UIFCPY_Msk        (0x1UL << TIM_CNT_UIFCPY_Pos)                /*!< 0x80000000 */
#define TIM_CNT_UIFCPY            TIM_CNT_UIFCPY_Msk                           /*!<Update interrupt flag copy (if UIFREMAP=1) */

/*******************  Bit definition for TIM_PSC register  ********************/
#define TIM_PSC_PSC_Pos           (0UL)
#define TIM_PSC_PSC_Msk           (0xFFFFUL << TIM_PSC_PSC_Pos)                /*!< 0x0000FFFF */
#define TIM_PSC_PSC               TIM_PSC_PSC_Msk                              /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  ********************/
#define TIM_ARR_ARR_Pos           (0UL)
#define TIM_ARR_ARR_Msk           (0xFFFFFFFFUL << TIM_ARR_ARR_Pos)            /*!< 0xFFFFFFFF */
#define TIM_ARR_ARR               TIM_ARR_ARR_Msk                              /*!<Actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define TIM_RCR_REP_Pos           (0UL)
#define TIM_RCR_REP_Msk           (0xFFFFUL << TIM_RCR_REP_Pos)                /*!< 0x0000FFFF */
#define TIM_RCR_REP               TIM_RCR_REP_Msk                              /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define TIM_CCR1_CCR1_Pos         (0UL)
#define TIM_CCR1_CCR1_Msk         (0xFFFFUL << TIM_CCR1_CCR1_Pos)              /*!< 0x0000FFFF */
#define TIM_CCR1_CCR1             TIM_CCR1_CCR1_Msk                            /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define TIM_CCR2_CCR2_Pos         (0UL)
#define TIM_CCR2_CCR2_Msk         (0xFFFFUL << TIM_CCR2_CCR2_Pos)              /*!< 0x0000FFFF */
#define TIM_CCR2_CCR2             TIM_CCR2_CCR2_Msk                            /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define TIM_CCR3_CCR3_Pos         (0UL)
#define TIM_CCR3_CCR3_Msk         (0xFFFFUL << TIM_CCR3_CCR3_Pos)              /*!< 0x0000FFFF */
#define TIM_CCR3_CCR3             TIM_CCR3_CCR3_Msk                            /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define TIM_CCR4_CCR4_Pos         (0UL)
#define TIM_CCR4_CCR4_Msk         (0xFFFFUL << TIM_CCR4_CCR4_Pos)              /*!< 0x0000FFFF */
#define TIM_CCR4_CCR4             TIM_CCR4_CCR4_Msk                            /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_CCR5 register  *******************/
#define TIM_CCR5_CCR5_Pos         (0UL)
#define TIM_CCR5_CCR5_Msk         (0xFFFFFFFFUL << TIM_CCR5_CCR5_Pos)          /*!< 0xFFFFFFFF */
#define TIM_CCR5_CCR5             TIM_CCR5_CCR5_Msk                            /*!<Capture/Compare 5 Value */
#define TIM_CCR5_GC5C1_Pos        (29UL)
#define TIM_CCR5_GC5C1_Msk        (0x1UL << TIM_CCR5_GC5C1_Pos)                /*!< 0x20000000 */
#define TIM_CCR5_GC5C1            TIM_CCR5_GC5C1_Msk                           /*!<Group Channel 5 and Channel 1 */
#define TIM_CCR5_GC5C2_Pos        (30UL)
#define TIM_CCR5_GC5C2_Msk        (0x1UL << TIM_CCR5_GC5C2_Pos)                /*!< 0x40000000 */
#define TIM_CCR5_GC5C2            TIM_CCR5_GC5C2_Msk                           /*!<Group Channel 5 and Channel 2 */
#define TIM_CCR5_GC5C3_Pos        (31UL)
#define TIM_CCR5_GC5C3_Msk        (0x1UL << TIM_CCR5_GC5C3_Pos)                /*!< 0x80000000 */
#define TIM_CCR5_GC5C3            TIM_CCR5_GC5C3_Msk                           /*!<Group Channel 5 and Channel 3 */

/*******************  Bit definition for TIM_CCR6 register  *******************/
#define TIM_CCR6_CCR6_Pos         (0UL)
#define TIM_CCR6_CCR6_Msk         (0xFFFFUL << TIM_CCR6_CCR6_Pos)              /*!< 0x0000FFFF */
#define TIM_CCR6_CCR6             TIM_CCR6_CCR6_Msk                            /*!<Capture/Compare 6 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define TIM_BDTR_DTG_Pos          (0UL)
#define TIM_BDTR_DTG_Msk          (0xFFUL << TIM_BDTR_DTG_Pos)                 /*!< 0x000000FF */
#define TIM_BDTR_DTG              TIM_BDTR_DTG_Msk                             /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0            (0x01UL << TIM_BDTR_DTG_Pos)                 /*!< 0x00000001 */
#define TIM_BDTR_DTG_1            (0x02UL << TIM_BDTR_DTG_Pos)                 /*!< 0x00000002 */
#define TIM_BDTR_DTG_2            (0x04UL << TIM_BDTR_DTG_Pos)                 /*!< 0x00000004 */
#define TIM_BDTR_DTG_3            (0x08UL << TIM_BDTR_DTG_Pos)                 /*!< 0x00000008 */
#define TIM_BDTR_DTG_4            (0x10UL << TIM_BDTR_DTG_Pos)                 /*!< 0x00000010 */
#define TIM_BDTR_DTG_5            (0x20UL << TIM_BDTR_DTG_Pos)                 /*!< 0x00000020 */
#define TIM_BDTR_DTG_6            (0x40UL << TIM_BDTR_DTG_Pos)                 /*!< 0x00000040 */
#define TIM_BDTR_DTG_7            (0x80UL << TIM_BDTR_DTG_Pos)                 /*!< 0x00000080 */

#define TIM_BDTR_LOCK_Pos         (8UL)
#define TIM_BDTR_LOCK_Msk         (0x3UL << TIM_BDTR_LOCK_Pos)                 /*!< 0x00000300 */
#define TIM_BDTR_LOCK             TIM_BDTR_LOCK_Msk                            /*!<LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0           (0x1UL << TIM_BDTR_LOCK_Pos)                 /*!< 0x00000100 */
#define TIM_BDTR_LOCK_1           (0x2UL << TIM_BDTR_LOCK_Pos)                 /*!< 0x00000200 */

#define TIM_BDTR_OSSI_Pos         (10UL)
#define TIM_BDTR_OSSI_Msk         (0x1UL << TIM_BDTR_OSSI_Pos)                 /*!< 0x00000400 */
#define TIM_BDTR_OSSI             TIM_BDTR_OSSI_Msk                            /*!<Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR_Pos         (11UL)
#define TIM_BDTR_OSSR_Msk         (0x1UL << TIM_BDTR_OSSR_Pos)                 /*!< 0x00000800 */
#define TIM_BDTR_OSSR             TIM_BDTR_OSSR_Msk                            /*!<Off-State Selection for Run mode */
#define TIM_BDTR_BKE_Pos          (12UL)
#define TIM_BDTR_BKE_Msk          (0x1UL << TIM_BDTR_BKE_Pos)                  /*!< 0x00001000 */
#define TIM_BDTR_BKE              TIM_BDTR_BKE_Msk                             /*!<Break enable for Break 1 */
#define TIM_BDTR_BKP_Pos          (13UL)
#define TIM_BDTR_BKP_Msk          (0x1UL << TIM_BDTR_BKP_Pos)                  /*!< 0x00002000 */
#define TIM_BDTR_BKP              TIM_BDTR_BKP_Msk                             /*!<Break Polarity for Break 1 */
#define TIM_BDTR_AOE_Pos          (14UL)
#define TIM_BDTR_AOE_Msk          (0x1UL << TIM_BDTR_AOE_Pos)                  /*!< 0x00004000 */
#define TIM_BDTR_AOE              TIM_BDTR_AOE_Msk                             /*!<Automatic Output enable */
#define TIM_BDTR_MOE_Pos          (15UL)
#define TIM_BDTR_MOE_Msk          (0x1UL << TIM_BDTR_MOE_Pos)                  /*!< 0x00008000 */
#define TIM_BDTR_MOE              TIM_BDTR_MOE_Msk                             /*!<Main Output enable */

#define TIM_BDTR_BKF_Pos          (16UL)
#define TIM_BDTR_BKF_Msk          (0xFUL << TIM_BDTR_BKF_Pos)                  /*!< 0x000F0000 */
#define TIM_BDTR_BKF              TIM_BDTR_BKF_Msk                             /*!<Break Filter for Break 1 */
#define TIM_BDTR_BK2F_Pos         (20UL)
#define TIM_BDTR_BK2F_Msk         (0xFUL << TIM_BDTR_BK2F_Pos)                 /*!< 0x00F00000 */
#define TIM_BDTR_BK2F             TIM_BDTR_BK2F_Msk                            /*!<Break Filter for Break 2 */

#define TIM_BDTR_BK2E_Pos         (24UL)
#define TIM_BDTR_BK2E_Msk         (0x1UL << TIM_BDTR_BK2E_Pos)                 /*!< 0x01000000 */
#define TIM_BDTR_BK2E             TIM_BDTR_BK2E_Msk                            /*!<Break enable for Break 2 */
#define TIM_BDTR_BK2P_Pos         (25UL)
#define TIM_BDTR_BK2P_Msk         (0x1UL << TIM_BDTR_BK2P_Pos)                 /*!< 0x02000000 */
#define TIM_BDTR_BK2P             TIM_BDTR_BK2P_Msk                            /*!<Break Polarity for Break 2 */

#define TIM_BDTR_BKDSRM_Pos       (26UL)
#define TIM_BDTR_BKDSRM_Msk       (0x1UL << TIM_BDTR_BKDSRM_Pos)               /*!< 0x04000000 */
#define TIM_BDTR_BKDSRM           TIM_BDTR_BKDSRM_Msk                          /*!<Break disarming/re-arming */
#define TIM_BDTR_BK2DSRM_Pos      (27UL)
#define TIM_BDTR_BK2DSRM_Msk      (0x1UL << TIM_BDTR_BK2DSRM_Pos)              /*!< 0x08000000 */
#define TIM_BDTR_BK2DSRM          TIM_BDTR_BK2DSRM_Msk                         /*!<Break2 disarming/re-arming */

#define TIM_BDTR_BKBID_Pos        (28UL)
#define TIM_BDTR_BKBID_Msk        (0x1UL << TIM_BDTR_BKBID_Pos)                /*!< 0x10000000 */
#define TIM_BDTR_BKBID            TIM_BDTR_BKBID_Msk                           /*!<Break BIDirectional */
#define TIM_BDTR_BK2BID_Pos       (29UL)
#define TIM_BDTR_BK2BID_Msk       (0x1UL << TIM_BDTR_BK2BID_Pos)               /*!< 0x20000000 */
#define TIM_BDTR_BK2BID           TIM_BDTR_BK2BID_Msk                          /*!<Break2 BIDirectional */

/*******************  Bit definition for TIM_DCR register  ********************/
#define TIM_DCR_DBA_Pos           (0UL)
#define TIM_DCR_DBA_Msk           (0x1FUL << TIM_DCR_DBA_Pos)                  /*!< 0x0000001F */
#define TIM_DCR_DBA               TIM_DCR_DBA_Msk                              /*!<DBA[4:0] bits (DMA Base Address) */
#define TIM_DCR_DBA_0             (0x01UL << TIM_DCR_DBA_Pos)                  /*!< 0x00000001 */
#define TIM_DCR_DBA_1             (0x02UL << TIM_DCR_DBA_Pos)                  /*!< 0x00000002 */
#define TIM_DCR_DBA_2             (0x04UL << TIM_DCR_DBA_Pos)                  /*!< 0x00000004 */
#define TIM_DCR_DBA_3             (0x08UL << TIM_DCR_DBA_Pos)                  /*!< 0x00000008 */
#define TIM_DCR_DBA_4             (0x10UL << TIM_DCR_DBA_Pos)                  /*!< 0x00000010 */

#define TIM_DCR_DBL_Pos           (8UL)
#define TIM_DCR_DBL_Msk           (0x1FUL << TIM_DCR_DBL_Pos)                  /*!< 0x00001F00 */
#define TIM_DCR_DBL               TIM_DCR_DBL_Msk                              /*!<DBL[4:0] bits (DMA Burst Length) */
#define TIM_DCR_DBL_0             (0x01UL << TIM_DCR_DBL_Pos)                  /*!< 0x00000100 */
#define TIM_DCR_DBL_1             (0x02UL << TIM_DCR_DBL_Pos)                  /*!< 0x00000200 */
#define TIM_DCR_DBL_2             (0x04UL << TIM_DCR_DBL_Pos)                  /*!< 0x00000400 */
#define TIM_DCR_DBL_3             (0x08UL << TIM_DCR_DBL_Pos)                  /*!< 0x00000800 */
#define TIM_DCR_DBL_4             (0x10UL << TIM_DCR_DBL_Pos)                  /*!< 0x00001000 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define TIM_DMAR_DMAB_Pos         (0UL)
#define TIM_DMAR_DMAB_Msk         (0xFFFFUL << TIM_DMAR_DMAB_Pos)              /*!< 0x0000FFFF */
#define TIM_DMAR_DMAB             TIM_DMAR_DMAB_Msk                            /*!<DMA register for burst accesses */

/*******************  Bit definition for TIM1_OR register  ********************/
#define TIM_OR1_OCREF_CLR_Pos     (0UL)
#define TIM_OR1_OCREF_CLR_Msk     (0x1UL << TIM_OR1_OCREF_CLR_Pos)            /*!< 0x00000001 */
#define TIM_OR1_OCREF_CLR         TIM_OR1_OCREF_CLR_Msk                       /*!<OCREF clear input selection */

/*******************  Bit definition for TIM_AF1 register  *******************/
#define TIM_AF1_BKINE_Pos        (0UL)
#define TIM_AF1_BKINE_Msk        (0x1UL << TIM_AF1_BKINE_Pos)                /*!< 0x00000001 */
#define TIM_AF1_BKINE            TIM_AF1_BKINE_Msk                           /*!<BRK BKIN input enable */
#define TIM_AF1_BKCMP1E_Pos      (1UL)
#define TIM_AF1_BKCMP1E_Msk      (0x1UL << TIM_AF1_BKCMP1E_Pos)              /*!< 0x00000002 */
#define TIM_AF1_BKCMP1E          TIM_AF1_BKCMP1E_Msk                         /*!<BRK COMP1 enable */
#define TIM_AF1_BKCMP2E_Pos      (2UL)
#define TIM_AF1_BKCMP2E_Msk      (0x1UL << TIM_AF1_BKCMP2E_Pos)              /*!< 0x00000004 */
#define TIM_AF1_BKCMP2E          TIM_AF1_BKCMP2E_Msk                         /*!<BRK COMP2 enable */
#define TIM_AF1_BKINP_Pos        (9UL)
#define TIM_AF1_BKINP_Msk        (0x1UL << TIM_AF1_BKINP_Pos)                /*!< 0x00000200 */
#define TIM_AF1_BKINP            TIM_AF1_BKINP_Msk                           /*!<BRK BKIN input polarity */
#define TIM_AF1_BKCMP1P_Pos      (10UL)
#define TIM_AF1_BKCMP1P_Msk      (0x1UL << TIM_AF1_BKCMP1P_Pos)              /*!< 0x00000400 */
#define TIM_AF1_BKCMP1P          TIM_AF1_BKCMP1P_Msk                         /*!<BRK COMP1 input polarity */
#define TIM_AF1_BKCMP2P_Pos      (11UL)
#define TIM_AF1_BKCMP2P_Msk      (0x1UL << TIM_AF1_BKCMP2P_Pos)              /*!< 0x00000800 */
#define TIM_AF1_BKCMP2P          TIM_AF1_BKCMP2P_Msk                         /*!<BRK COMP2 input polarity */

#define TIM_AF1_ETRSEL_Pos       (14UL)
#define TIM_AF1_ETRSEL_Msk       (0xFUL << TIM_AF1_ETRSEL_Pos)               /*!< 0x0003C000 */
#define TIM_AF1_ETRSEL           TIM_AF1_ETRSEL_Msk                          /*!<ETRSEL[3:0] bits (TIM1 ETR source selection) */
#define TIM_AF1_ETRSEL_0         (0x1UL << TIM_AF1_ETRSEL_Pos)               /*!< 0x00004000 */
#define TIM_AF1_ETRSEL_1         (0x2UL << TIM_AF1_ETRSEL_Pos)               /*!< 0x00008000 */
#define TIM_AF1_ETRSEL_2         (0x4UL << TIM_AF1_ETRSEL_Pos)               /*!< 0x00010000 */
#define TIM_AF1_ETRSEL_3         (0x8UL << TIM_AF1_ETRSEL_Pos)               /*!< 0x00020000 */

/*******************  Bit definition for TIM_AF2 register  *******************/
#define TIM_AF2_BK2INE_Pos       (0UL)
#define TIM_AF2_BK2INE_Msk       (0x1UL << TIM_AF2_BK2INE_Pos)               /*!< 0x00000001 */
#define TIM_AF2_BK2INE           TIM_AF2_BK2INE_Msk                          /*!<BRK2 BKIN2 input enable */
#define TIM_AF2_BK2CMP1E_Pos     (1UL)
#define TIM_AF2_BK2CMP1E_Msk     (0x1UL << TIM_AF2_BK2CMP1E_Pos)             /*!< 0x00000002 */
#define TIM_AF2_BK2CMP1E         TIM_AF2_BK2CMP1E_Msk                        /*!<BRK2 COMP1 enable */
#define TIM_AF2_BK2CMP2E_Pos     (2UL)
#define TIM_AF2_BK2CMP2E_Msk     (0x1UL << TIM_AF2_BK2CMP2E_Pos)             /*!< 0x00000004 */
#define TIM_AF2_BK2CMP2E         TIM_AF2_BK2CMP2E_Msk                        /*!<BRK2 COMP2 enable */
#define TIM_AF2_BK2INP_Pos       (9UL)
#define TIM_AF2_BK2INP_Msk       (0x1UL << TIM_AF2_BK2INP_Pos)               /*!< 0x00000200 */
#define TIM_AF2_BK2INP           TIM_AF2_BK2INP_Msk                          /*!<BRK2 BKIN2 input polarity */
#define TIM_AF2_BK2CMP1P_Pos     (10UL)
#define TIM_AF2_BK2CMP1P_Msk     (0x1UL << TIM_AF2_BK2CMP1P_Pos)             /*!< 0x00000400 */
#define TIM_AF2_BK2CMP1P         TIM_AF2_BK2CMP1P_Msk                        /*!<BRK2 COMP1 input polarity */
#define TIM_AF2_BK2CMP2P_Pos     (11UL)
#define TIM_AF2_BK2CMP2P_Msk     (0x1UL << TIM_AF2_BK2CMP2P_Pos)             /*!< 0x00000800 */
#define TIM_AF2_BK2CMP2P         TIM_AF2_BK2CMP2P_Msk                        /*!<BRK2 COMP2 input polarity */

/*******************  Bit definition for TIM_TISEL register  *********************/
#define TIM_TISEL_TI1SEL_Pos      (0UL)
#define TIM_TISEL_TI1SEL_Msk      (0xFUL << TIM_TISEL_TI1SEL_Pos)              /*!< 0x0000000F */
#define TIM_TISEL_TI1SEL          TIM_TISEL_TI1SEL_Msk                         /*!<TI1SEL[3:0] bits (TIM TI1 SEL)*/
#define TIM_TISEL_TI1SEL_0        (0x1UL << TIM_TISEL_TI1SEL_Pos)              /*!< 0x00000001 */
#define TIM_TISEL_TI1SEL_1        (0x2UL << TIM_TISEL_TI1SEL_Pos)              /*!< 0x00000002 */
#define TIM_TISEL_TI1SEL_2        (0x4UL << TIM_TISEL_TI1SEL_Pos)              /*!< 0x00000004 */
#define TIM_TISEL_TI1SEL_3        (0x8UL << TIM_TISEL_TI1SEL_Pos)              /*!< 0x00000008 */

#define TIM_TISEL_TI2SEL_Pos      (8UL)
#define TIM_TISEL_TI2SEL_Msk      (0xFUL << TIM_TISEL_TI2SEL_Pos)              /*!< 0x00000F00 */
#define TIM_TISEL_TI2SEL          TIM_TISEL_TI2SEL_Msk                         /*!<TI2SEL[3:0] bits (TIM TI2 SEL)*/
#define TIM_TISEL_TI2SEL_0        (0x1UL << TIM_TISEL_TI2SEL_Pos)              /*!< 0x00000100 */
#define TIM_TISEL_TI2SEL_1        (0x2UL << TIM_TISEL_TI2SEL_Pos)              /*!< 0x00000200 */
#define TIM_TISEL_TI2SEL_2        (0x4UL << TIM_TISEL_TI2SEL_Pos)              /*!< 0x00000400 */
#define TIM_TISEL_TI2SEL_3        (0x8UL << TIM_TISEL_TI2SEL_Pos)              /*!< 0x00000800 */

#define TIM_TISEL_TI3SEL_Pos      (16UL)
#define TIM_TISEL_TI3SEL_Msk      (0xFUL << TIM_TISEL_TI3SEL_Pos)              /*!< 0x000F0000 */
#define TIM_TISEL_TI3SEL          TIM_TISEL_TI3SEL_Msk                         /*!<TI3SEL[3:0] bits (TIM TI3 SEL)*/
#define TIM_TISEL_TI3SEL_0        (0x1UL << TIM_TISEL_TI3SEL_Pos)              /*!< 0x00010000 */
#define TIM_TISEL_TI3SEL_1        (0x2UL << TIM_TISEL_TI3SEL_Pos)              /*!< 0x00020000 */
#define TIM_TISEL_TI3SEL_2        (0x4UL << TIM_TISEL_TI3SEL_Pos)              /*!< 0x00040000 */
#define TIM_TISEL_TI3SEL_3        (0x8UL << TIM_TISEL_TI3SEL_Pos)              /*!< 0x00080000 */

#define TIM_TISEL_TI4SEL_Pos      (24UL)
#define TIM_TISEL_TI4SEL_Msk      (0xFUL << TIM_TISEL_TI4SEL_Pos)              /*!< 0x0F000000 */
#define TIM_TISEL_TI4SEL          TIM_TISEL_TI4SEL_Msk                         /*!<TI4SEL[3:0] bits (TIM TI4 SEL)*/
#define TIM_TISEL_TI4SEL_0        (0x1UL << TIM_TISEL_TI4SEL_Pos)              /*!< 0x01000000 */
#define TIM_TISEL_TI4SEL_1        (0x2UL << TIM_TISEL_TI4SEL_Pos)              /*!< 0x02000000 */
#define TIM_TISEL_TI4SEL_2        (0x4UL << TIM_TISEL_TI4SEL_Pos)              /*!< 0x04000000 */
#define TIM_TISEL_TI4SEL_3        (0x8UL << TIM_TISEL_TI4SEL_Pos)              /*!< 0x08000000 */

/******************************************************************************/
/*                                                                            */
/*                         Low Power Timer (LPTIM)                            */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for LPTIM_ISR register  *******************/
#define LPTIM_ISR_CC1IF_Pos         (0UL)
#define LPTIM_ISR_CC1IF_Msk         (0x1UL << LPTIM_ISR_CC1IF_Pos)             /*!< 0x00000001 */
#define LPTIM_ISR_CC1IF             LPTIM_ISR_CC1IF_Msk                        /*!< Capture/Compare 1 interrupt flag */
#define LPTIM_ISR_ARRM_Pos          (1UL)
#define LPTIM_ISR_ARRM_Msk          (0x1UL << LPTIM_ISR_ARRM_Pos)              /*!< 0x00000002 */
#define LPTIM_ISR_ARRM              LPTIM_ISR_ARRM_Msk                         /*!< Autoreload match */
#define LPTIM_ISR_EXTTRIG_Pos       (2UL)
#define LPTIM_ISR_EXTTRIG_Msk       (0x1UL << LPTIM_ISR_EXTTRIG_Pos)           /*!< 0x00000004 */
#define LPTIM_ISR_EXTTRIG           LPTIM_ISR_EXTTRIG_Msk                      /*!< External trigger edge event */
#define LPTIM_ISR_CMP1OK_Pos        (3UL)
#define LPTIM_ISR_CMP1OK_Msk        (0x1UL << LPTIM_ISR_CMP1OK_Pos)            /*!< 0x00000008 */
#define LPTIM_ISR_CMP1OK            LPTIM_ISR_CMP1OK_Msk                       /*!< Compare register 1 update OK */
#define LPTIM_ISR_ARROK_Pos         (4UL)
#define LPTIM_ISR_ARROK_Msk         (0x1UL << LPTIM_ISR_ARROK_Pos)             /*!< 0x00000010 */
#define LPTIM_ISR_ARROK             LPTIM_ISR_ARROK_Msk                        /*!< Autoreload register update OK */
#define LPTIM_ISR_UP_Pos            (5UL)
#define LPTIM_ISR_UP_Msk            (0x1UL << LPTIM_ISR_UP_Pos)                /*!< 0x00000020 */
#define LPTIM_ISR_UP                LPTIM_ISR_UP_Msk                           /*!< Counter direction change down to up */
#define LPTIM_ISR_DOWN_Pos          (6UL)
#define LPTIM_ISR_DOWN_Msk          (0x1UL << LPTIM_ISR_DOWN_Pos)              /*!< 0x00000040 */
#define LPTIM_ISR_DOWN              LPTIM_ISR_DOWN_Msk                         /*!< Counter direction change up to down */
#define LPTIM_ISR_UE_Pos            (7UL)
#define LPTIM_ISR_UE_Msk            (0x1UL << LPTIM_ISR_UE_Pos)                /*!< 0x00000080 */
#define LPTIM_ISR_UE                LPTIM_ISR_UE_Msk                           /*!< Update event */
#define LPTIM_ISR_REPOK_Pos         (8UL)
#define LPTIM_ISR_REPOK_Msk         (0x1UL << LPTIM_ISR_REPOK_Pos)             /*!< 0x00000100 */
#define LPTIM_ISR_REPOK             LPTIM_ISR_REPOK_Msk                        /*!< Repetition register update OK */
#define LPTIM_ISR_CC2IF_Pos         (9UL)
#define LPTIM_ISR_CC2IF_Msk         (0x1UL << LPTIM_ISR_CC2IF_Pos)             /*!< 0x00000200 */
#define LPTIM_ISR_CC2IF             LPTIM_ISR_CC2IF_Msk                        /*!< Capture/Compare 2 interrupt flag */
#define LPTIM_ISR_CC3IF_Pos         (10UL)
#define LPTIM_ISR_CC3IF_Msk         (0x1UL << LPTIM_ISR_CC3IF_Pos)             /*!< 0x00000400 */
#define LPTIM_ISR_CC3IF             LPTIM_ISR_CC3IF_Msk                        /*!< Capture/Compare 3 interrupt flag */
#define LPTIM_ISR_CC4IF_Pos         (11UL)
#define LPTIM_ISR_CC4IF_Msk         (0x1UL << LPTIM_ISR_CC4IF_Pos)             /*!< 0x00000800 */
#define LPTIM_ISR_CC4IF             LPTIM_ISR_CC4IF_Msk                        /*!< Capture/Compare 4 interrupt flag */
#define LPTIM_ISR_CC1OF_Pos         (12UL)
#define LPTIM_ISR_CC1OF_Msk         (0x1UL << LPTIM_ISR_CC1OF_Pos)            /*!< 0x00001000 */
#define LPTIM_ISR_CC1OF             LPTIM_ISR_CC1OF_Msk                       /*!< Capture/Compare 1 over-capture flag */
#define LPTIM_ISR_CC2OF_Pos         (13UL)
#define LPTIM_ISR_CC2OF_Msk         (0x1UL << LPTIM_ISR_CC2OF_Pos)            /*!< 0x00002000 */
#define LPTIM_ISR_CC2OF             LPTIM_ISR_CC2OF_Msk                       /*!< Capture/Compare 2 over-capture flag */
#define LPTIM_ISR_CC3OF_Pos         (14UL)
#define LPTIM_ISR_CC3OF_Msk         (0x1UL << LPTIM_ISR_CC3OF_Pos)            /*!< 0x00004000 */
#define LPTIM_ISR_CC3OF             LPTIM_ISR_CC3OF_Msk                       /*!< Capture/Compare 3 over-capture flag */
#define LPTIM_ISR_CC4OF_Pos         (15UL)
#define LPTIM_ISR_CC4OF_Msk         (0x1UL << LPTIM_ISR_CC4OF_Pos)            /*!< 0x00008000 */
#define LPTIM_ISR_CC4OF             LPTIM_ISR_CC4OF_Msk                       /*!< Capture/Compare 4 over-capture flag */
#define LPTIM_ISR_CMP2OK_Pos        (19UL)
#define LPTIM_ISR_CMP2OK_Msk        (0x1UL << LPTIM_ISR_CMP2OK_Pos)          /*!< 0x00080000 */
#define LPTIM_ISR_CMP2OK            LPTIM_ISR_CMP2OK_Msk                     /*!< Compare register 2 update OK */
#define LPTIM_ISR_CMP3OK_Pos        (20UL)
#define LPTIM_ISR_CMP3OK_Msk        (0x1UL << LPTIM_ISR_CMP3OK_Pos)          /*!< 0x00100000 */
#define LPTIM_ISR_CMP3OK            LPTIM_ISR_CMP3OK_Msk                     /*!< Compare register 3 update OK */
#define LPTIM_ISR_CMP4OK_Pos        (21UL)
#define LPTIM_ISR_CMP4OK_Msk        (0x1UL << LPTIM_ISR_CMP4OK_Pos)          /*!< 0x00200000 */
#define LPTIM_ISR_CMP4OK            LPTIM_ISR_CMP4OK_Msk                     /*!< Compare register 4 update OK */
#define LPTIM_ISR_DIEROK_Pos        (24UL)
#define LPTIM_ISR_DIEROK_Msk        (0x1UL << LPTIM_ISR_DIEROK_Pos)          /*!< 0x01000000 */
#define LPTIM_ISR_DIEROK            LPTIM_ISR_DIEROK_Msk                     /*!< DMA & interrupt enable update OK */

/******************  Bit definition for LPTIM_ICR register  *******************/
#define LPTIM_ICR_CC1CF_Pos         (0UL)
#define LPTIM_ICR_CC1CF_Msk         (0x1UL << LPTIM_ICR_CC1CF_Pos)             /*!< 0x00000001 */
#define LPTIM_ICR_CC1CF             LPTIM_ICR_CC1CF_Msk                        /*!< Capture/Compare 1 clear flag  */
#define LPTIM_ICR_ARRMCF_Pos        (1UL)
#define LPTIM_ICR_ARRMCF_Msk        (0x1UL << LPTIM_ICR_ARRMCF_Pos)            /*!< 0x00000002 */
#define LPTIM_ICR_ARRMCF            LPTIM_ICR_ARRMCF_Msk                       /*!< Autoreload match clear flag */
#define LPTIM_ICR_EXTTRIGCF_Pos     (2UL)
#define LPTIM_ICR_EXTTRIGCF_Msk     (0x1UL << LPTIM_ICR_EXTTRIGCF_Pos)         /*!< 0x00000004 */
#define LPTIM_ICR_EXTTRIGCF         LPTIM_ICR_EXTTRIGCF_Msk                    /*!< External trigger edge event clear flag */
#define LPTIM_ICR_CMP1OKCF_Pos      (3UL)
#define LPTIM_ICR_CMP1OKCF_Msk      (0x1UL << LPTIM_ICR_CMP1OKCF_Pos)          /*!< 0x00000008 */
#define LPTIM_ICR_CMP1OKCF          LPTIM_ICR_CMP1OKCF_Msk                     /*!< Compare register 1 update OK clear flag */
#define LPTIM_ICR_ARROKCF_Pos       (4UL)
#define LPTIM_ICR_ARROKCF_Msk       (0x1UL << LPTIM_ICR_ARROKCF_Pos)           /*!< 0x00000010 */
#define LPTIM_ICR_ARROKCF           LPTIM_ICR_ARROKCF_Msk                      /*!< Autoreload register update OK clear flag */
#define LPTIM_ICR_UPCF_Pos          (5UL)
#define LPTIM_ICR_UPCF_Msk          (0x1UL << LPTIM_ICR_UPCF_Pos)              /*!< 0x00000020 */
#define LPTIM_ICR_UPCF              LPTIM_ICR_UPCF_Msk                         /*!< Counter direction change down to up clear flag */
#define LPTIM_ICR_DOWNCF_Pos        (6UL)
#define LPTIM_ICR_DOWNCF_Msk        (0x1UL << LPTIM_ICR_DOWNCF_Pos)            /*!< 0x00000040 */
#define LPTIM_ICR_DOWNCF            LPTIM_ICR_DOWNCF_Msk                       /*!< Counter direction change up to down clear flag */
#define LPTIM_ICR_UECF_Pos          (7UL)
#define LPTIM_ICR_UECF_Msk          (0x1UL << LPTIM_ICR_UECF_Pos)              /*!< 0x00000080 */
#define LPTIM_ICR_UECF              LPTIM_ICR_UECF_Msk                         /*!< Update event clear flag */
#define LPTIM_ICR_REPOKCF_Pos       (8UL)
#define LPTIM_ICR_REPOKCF_Msk       (0x1UL << LPTIM_ICR_REPOKCF_Pos)           /*!< 0x00000100 */
#define LPTIM_ICR_REPOKCF           LPTIM_ICR_REPOKCF_Msk                      /*!< Repetition register update OK clear flag */
#define LPTIM_ICR_CC2CF_Pos         (9UL)
#define LPTIM_ICR_CC2CF_Msk         (0x1UL << LPTIM_ICR_CC2CF_Pos)             /*!< 0x00000200 */
#define LPTIM_ICR_CC2CF             LPTIM_ICR_CC2CF_Msk                        /*!< Capture/Compare 2 clear flag  */
#define LPTIM_ICR_CC3CF_Pos         (10UL)
#define LPTIM_ICR_CC3CF_Msk         (0x1UL << LPTIM_ICR_CC3CF_Pos)             /*!< 0x00000400 */
#define LPTIM_ICR_CC3CF             LPTIM_ICR_CC3CF_Msk                        /*!< Capture/Compare 3 clear flag  */
#define LPTIM_ICR_CC4CF_Pos         (11UL)
#define LPTIM_ICR_CC4CF_Msk         (0x1UL << LPTIM_ICR_CC4CF_Pos)             /*!< 0x00000800 */
#define LPTIM_ICR_CC4CF             LPTIM_ICR_CC4CF_Msk                        /*!< Capture/Compare 4 clear flag  */
#define LPTIM_ICR_CC1OCF_Pos        (12UL)
#define LPTIM_ICR_CC1OCF_Msk        (0x1UL << LPTIM_ICR_CC1OCF_Pos)            /*!< 0x00001000 */
#define LPTIM_ICR_CC1OCF            LPTIM_ICR_CC1OCF_Msk                       /*!< Capture/Compare 1 over-capture clear flag */
#define LPTIM_ICR_CC2OCF_Pos        (13UL)
#define LPTIM_ICR_CC2OCF_Msk        (0x1UL << LPTIM_ICR_CC2OCF_Pos)            /*!< 0x00002000 */
#define LPTIM_ICR_CC2OCF            LPTIM_ICR_CC2OCF_Msk                       /*!< Capture/Compare 2 over-capture clear flag */
#define LPTIM_ICR_CC3OCF_Pos        (14UL)
#define LPTIM_ICR_CC3OCF_Msk        (0x1UL << LPTIM_ICR_CC3OCF_Pos)            /*!< 0x00004000 */
#define LPTIM_ICR_CC3OCF            LPTIM_ICR_CC3OCF_Msk                       /*!< Capture/Compare 3 over-capture clear flag */
#define LPTIM_ICR_CC4OCF_Pos        (15UL)
#define LPTIM_ICR_CC4OCF_Msk        (0x1UL << LPTIM_ICR_CC4OCF_Pos)            /*!< 0x00008000 */
#define LPTIM_ICR_CC4OCF            LPTIM_ICR_CC4OCF_Msk                       /*!< Capture/Compare 4 over-capture clear flag */
#define LPTIM_ICR_CMP2OKCF_Pos      (19UL)
#define LPTIM_ICR_CMP2OKCF_Msk      (0x1UL << LPTIM_ICR_CMP2OKCF_Pos)          /*!< 0x00080000 */
#define LPTIM_ICR_CMP2OKCF          LPTIM_ICR_CMP2OKCF_Msk                     /*!< Compare register 2 update OK clear flag */
#define LPTIM_ICR_CMP3OKCF_Pos      (20UL)
#define LPTIM_ICR_CMP3OKCF_Msk      (0x1UL << LPTIM_ICR_CMP3OKCF_Pos)          /*!< 0x00100000 */
#define LPTIM_ICR_CMP3OKCF          LPTIM_ICR_CMP3OKCF_Msk                     /*!< Compare register 3 update OK clear flag */
#define LPTIM_ICR_CMP4OKCF_Pos      (21UL)
#define LPTIM_ICR_CMP4OKCF_Msk      (0x1UL << LPTIM_ICR_CMP4OKCF_Pos)          /*!< 0x00200000 */
#define LPTIM_ICR_CMP4OKCF          LPTIM_ICR_CMP4OKCF_Msk                     /*!< Compare register 4 update OK clear flag */
#define LPTIM_ICR_DIEROKCF_Pos      (24UL)
#define LPTIM_ICR_DIEROKCF_Msk      (0x1UL << LPTIM_ICR_DIEROKCF_Pos)          /*!< 0x01000000 */
#define LPTIM_ICR_DIEROKCF          LPTIM_ICR_DIEROKCF_Msk                     /*!< DMA & interrupt enable update OK clear flag */

/******************  Bit definition for LPTIM_DIER register *******************/
#define LPTIM_DIER_CC1IE_Pos         (0UL)
#define LPTIM_DIER_CC1IE_Msk         (0x1UL << LPTIM_DIER_CC1IE_Pos)           /*!< 0x00000001 */
#define LPTIM_DIER_CC1IE             LPTIM_DIER_CC1IE_Msk                      /*!< Compare/Compare 1 interrupt enable */
#define LPTIM_DIER_ARRMIE_Pos        (1UL)
#define LPTIM_DIER_ARRMIE_Msk        (0x1UL << LPTIM_DIER_ARRMIE_Pos)          /*!< 0x00000002 */
#define LPTIM_DIER_ARRMIE            LPTIM_DIER_ARRMIE_Msk                     /*!< Autoreload match interrupt enable */
#define LPTIM_DIER_EXTTRIGIE_Pos     (2UL)
#define LPTIM_DIER_EXTTRIGIE_Msk     (0x1UL << LPTIM_DIER_EXTTRIGIE_Pos)       /*!< 0x00000004 */
#define LPTIM_DIER_EXTTRIGIE         LPTIM_DIER_EXTTRIGIE_Msk                  /*!< External trigger edge event interrupt enable */
#define LPTIM_DIER_CMP1OKIE_Pos      (3UL)
#define LPTIM_DIER_CMP1OKIE_Msk      (0x1UL << LPTIM_DIER_CMP1OKIE_Pos)        /*!< 0x00000008 */
#define LPTIM_DIER_CMP1OKIE          LPTIM_DIER_CMP1OKIE_Msk                   /*!< Compare register 1 update OK interrupt enable */
#define LPTIM_DIER_ARROKIE_Pos       (4UL)
#define LPTIM_DIER_ARROKIE_Msk       (0x1UL << LPTIM_DIER_ARROKIE_Pos)         /*!< 0x00000010 */
#define LPTIM_DIER_ARROKIE           LPTIM_DIER_ARROKIE_Msk                    /*!< Autoreload register update OK interrupt enable */
#define LPTIM_DIER_UPIE_Pos          (5UL)
#define LPTIM_DIER_UPIE_Msk          (0x1UL << LPTIM_DIER_UPIE_Pos)            /*!< 0x00000020 */
#define LPTIM_DIER_UPIE              LPTIM_DIER_UPIE_Msk                       /*!< Counter direction change down to up interrupt enable */
#define LPTIM_DIER_DOWNIE_Pos        (6UL)
#define LPTIM_DIER_DOWNIE_Msk        (0x1UL << LPTIM_DIER_DOWNIE_Pos)          /*!< 0x00000040 */
#define LPTIM_DIER_DOWNIE            LPTIM_DIER_DOWNIE_Msk                     /*!< Counter direction change up to down interrupt enable */
#define LPTIM_DIER_UEIE_Pos          (7UL)
#define LPTIM_DIER_UEIE_Msk          (0x1UL << LPTIM_DIER_UEIE_Pos)            /*!< 0x00000080 */
#define LPTIM_DIER_UEIE              LPTIM_DIER_UEIE_Msk                       /*!< Update event interrupt enable */
#define LPTIM_DIER_REPOKIE_Pos       (8UL)
#define LPTIM_DIER_REPOKIE_Msk       (0x1UL << LPTIM_DIER_REPOKIE_Pos)         /*!< 0x00000100 */
#define LPTIM_DIER_REPOKIE           LPTIM_DIER_REPOKIE_Msk                    /*!< Repetition register update OK interrupt enable */
#define LPTIM_DIER_CC2IE_Pos         (9UL)
#define LPTIM_DIER_CC2IE_Msk         (0x1UL << LPTIM_DIER_CC2IE_Pos)           /*!< 0x00000200 */
#define LPTIM_DIER_CC2IE             LPTIM_DIER_CC2IE_Msk                      /*!< Capture/Compare 2 interrupt interrupt enable */
#define LPTIM_DIER_CC3IE_Pos         (10UL)
#define LPTIM_DIER_CC3IE_Msk         (0x1UL << LPTIM_DIER_CC3IE_Pos)           /*!< 0x00000400 */
#define LPTIM_DIER_CC3IE             LPTIM_DIER_CC3IE_Msk                      /*!< Capture/Compare 3 interrupt interrupt enable */
#define LPTIM_DIER_CC4IE_Pos         (11UL)
#define LPTIM_DIER_CC4IE_Msk         (0x1UL << LPTIM_DIER_CC4IE_Pos)           /*!< 0x00000800 */
#define LPTIM_DIER_CC4IE             LPTIM_DIER_CC4IE_Msk                      /*!< Capture/Compare 4 interrupt interrupt enable */
#define LPTIM_DIER_CC1OIE_Pos        (12UL)
#define LPTIM_DIER_CC1OIE_Msk        (0x1UL << LPTIM_DIER_CC1OIE_Pos)          /*!< 0x00001000 */
#define LPTIM_DIER_CC1OIE            LPTIM_DIER_CC1OIE_Msk                     /*!< Capture/Compare 1 over-capture interrupt enable */
#define LPTIM_DIER_CC2OIE_Pos        (13UL)
#define LPTIM_DIER_CC2OIE_Msk        (0x1UL << LPTIM_DIER_CC2OIE_Pos)          /*!< 0x00002000 */
#define LPTIM_DIER_CC2OIE            LPTIM_DIER_CC2OIE_Msk                     /*!< Capture/Compare 2 over-capture interrupt enable */
#define LPTIM_DIER_CC3OIE_Pos        (14UL)
#define LPTIM_DIER_CC3OIE_Msk        (0x1UL << LPTIM_DIER_CC3OIE_Pos)          /*!< 0x00004000 */
#define LPTIM_DIER_CC3OIE            LPTIM_DIER_CC3OIE_Msk                     /*!< Capture/Compare 3 over-capture interrupt enable */
#define LPTIM_DIER_CC4OIE_Pos        (15UL)
#define LPTIM_DIER_CC4OIE_Msk        (0x1UL << LPTIM_DIER_CC4OIE_Pos)          /*!< 0x00008000 */
#define LPTIM_DIER_CC4OIE            LPTIM_DIER_CC4OIE_Msk                     /*!< Capture/Compare 4 over-capture interrupt enable */
#define LPTIM_DIER_CC1DE_Pos         (16UL)
#define LPTIM_DIER_CC1DE_Msk         (0x1UL << LPTIM_DIER_CC1DE_Pos)           /*!< 0x00010000 */
#define LPTIM_DIER_CC1DE             LPTIM_DIER_CC1DE_Msk                      /*!< Capture/Compare 1 DMA request enable */
#define LPTIM_DIER_CMP2OKIE_Pos      (19UL)
#define LPTIM_DIER_CMP2OKIE_Msk      (0x1UL << LPTIM_DIER_CMP2OKIE_Pos)        /*!< 0x00080000 */
#define LPTIM_DIER_CMP2OKIE          LPTIM_DIER_CMP2OKIE_Msk                   /*!< Compare register 2 update OK interrupt enable */
#define LPTIM_DIER_CMP3OKIE_Pos      (20UL)
#define LPTIM_DIER_CMP3OKIE_Msk      (0x1UL << LPTIM_DIER_CMP3OKIE_Pos)        /*!< 0x00100000 */
#define LPTIM_DIER_CMP3OKIE          LPTIM_DIER_CMP3OKIE_Msk                   /*!< Compare register 3 update OK interrupt enable */
#define LPTIM_DIER_CMP4OKIE_Pos      (21UL)
#define LPTIM_DIER_CMP4OKIE_Msk      (0x1UL << LPTIM_DIER_CMP4OKIE_Pos)        /*!< 0x00200000 */
#define LPTIM_DIER_CMP4OKIE          LPTIM_DIER_CMP4OKIE_Msk                   /*!< Compare register 4 update OK interrupt enable */
#define LPTIM_DIER_UEDE_Pos          (23UL)
#define LPTIM_DIER_UEDE_Msk          (0x1UL << LPTIM_DIER_UEDE_Pos)            /*!< 0x00800000 */
#define LPTIM_DIER_UEDE              LPTIM_DIER_UEDE_Msk                       /*!< Update event DMA request enable */
#define LPTIM_DIER_CC2DE_Pos         (25UL)
#define LPTIM_DIER_CC2DE_Msk         (0x1UL << LPTIM_DIER_CC2DE_Pos)           /*!< 0x02000000 */
#define LPTIM_DIER_CC2DE             LPTIM_DIER_CC2DE_Msk                      /*!< Capture/Compare 2 DMA request enable */
#define LPTIM_DIER_CC3DE_Pos         (26UL)
#define LPTIM_DIER_CC3DE_Msk         (0x1UL << LPTIM_DIER_CC3DE_Pos)           /*!< 0x04000000 */
#define LPTIM_DIER_CC3DE             LPTIM_DIER_CC3DE_Msk                      /*!< Capture/Compare 3 DMA request enable */
#define LPTIM_DIER_CC4DE_Pos         (27UL)
#define LPTIM_DIER_CC4DE_Msk         (0x1UL << LPTIM_DIER_CC4DE_Pos)           /*!< 0x08000000 */
#define LPTIM_DIER_CC4DE             LPTIM_DIER_CC4DE_Msk                      /*!< Capture/Compare 4 DMA request enable */

/******************  Bit definition for LPTIM_CFGR register *******************/
#define LPTIM_CFGR_CKSEL_Pos        (0UL)
#define LPTIM_CFGR_CKSEL_Msk        (0x1UL << LPTIM_CFGR_CKSEL_Pos)            /*!< 0x00000001 */
#define LPTIM_CFGR_CKSEL            LPTIM_CFGR_CKSEL_Msk                       /*!< Clock selector */
#define LPTIM_CFGR_CKPOL_Pos        (1UL)
#define LPTIM_CFGR_CKPOL_Msk        (0x3UL << LPTIM_CFGR_CKPOL_Pos)            /*!< 0x00000006 */
#define LPTIM_CFGR_CKPOL            LPTIM_CFGR_CKPOL_Msk                       /*!< CKPOL[1:0] bits (Clock polarity) */
#define LPTIM_CFGR_CKPOL_0          (0x1UL << LPTIM_CFGR_CKPOL_Pos)             /*!< 0x00000002 */
#define LPTIM_CFGR_CKPOL_1          (0x2UL << LPTIM_CFGR_CKPOL_Pos)             /*!< 0x00000004 */
#define LPTIM_CFGR_CKFLT_Pos        (3UL)
#define LPTIM_CFGR_CKFLT_Msk        (0x3UL << LPTIM_CFGR_CKFLT_Pos)            /*!< 0x00000018 */
#define LPTIM_CFGR_CKFLT            LPTIM_CFGR_CKFLT_Msk                       /*!< CKFLT[1:0] bits (Configurable digital filter for external clock) */
#define LPTIM_CFGR_CKFLT_0          (0x1UL << LPTIM_CFGR_CKFLT_Pos)             /*!< 0x00000008 */
#define LPTIM_CFGR_CKFLT_1          (0x2UL << LPTIM_CFGR_CKFLT_Pos)             /*!< 0x00000010 */
#define LPTIM_CFGR_TRGFLT_Pos       (6UL)
#define LPTIM_CFGR_TRGFLT_Msk       (0x3UL << LPTIM_CFGR_TRGFLT_Pos)           /*!< 0x000000C0 */
#define LPTIM_CFGR_TRGFLT           LPTIM_CFGR_TRGFLT_Msk                      /*!< TRGFLT[1:0] bits (Configurable digital filter for trigger) */
#define LPTIM_CFGR_TRGFLT_0         (0x1UL << LPTIM_CFGR_TRGFLT_Pos)            /*!< 0x00000040 */
#define LPTIM_CFGR_TRGFLT_1         (0x2UL << LPTIM_CFGR_TRGFLT_Pos)            /*!< 0x00000080 */
#define LPTIM_CFGR_PRESC_Pos        (9UL)
#define LPTIM_CFGR_PRESC_Msk        (0x7UL << LPTIM_CFGR_PRESC_Pos)            /*!< 0x00000E00 */
#define LPTIM_CFGR_PRESC            LPTIM_CFGR_PRESC_Msk                       /*!< PRESC[2:0] bits (Clock prescaler) */
#define LPTIM_CFGR_PRESC_0          (0x1UL << LPTIM_CFGR_PRESC_Pos)             /*!< 0x00000200 */
#define LPTIM_CFGR_PRESC_1          (0x2UL << LPTIM_CFGR_PRESC_Pos)             /*!< 0x00000400 */
#define LPTIM_CFGR_PRESC_2          (0x4UL << LPTIM_CFGR_PRESC_Pos)             /*!< 0x00000800 */
#define LPTIM_CFGR_TRIGSEL_Pos      (13UL)
#define LPTIM_CFGR_TRIGSEL_Msk      (0x7UL << LPTIM_CFGR_TRIGSEL_Pos)          /*!< 0x0000E000 */
#define LPTIM_CFGR_TRIGSEL          LPTIM_CFGR_TRIGSEL_Msk                     /*!< TRIGSEL[2:0]] bits (Trigger selector) */
#define LPTIM_CFGR_TRIGSEL_0        (0x1UL << LPTIM_CFGR_TRIGSEL_Pos)           /*!< 0x00002000 */
#define LPTIM_CFGR_TRIGSEL_1        (0x2UL << LPTIM_CFGR_TRIGSEL_Pos)           /*!< 0x00004000 */
#define LPTIM_CFGR_TRIGSEL_2        (0x4UL << LPTIM_CFGR_TRIGSEL_Pos)           /*!< 0x00008000 */
#define LPTIM_CFGR_TRIGEN_Pos       (17UL)
#define LPTIM_CFGR_TRIGEN_Msk       (0x3UL << LPTIM_CFGR_TRIGEN_Pos)           /*!< 0x00060000 */
#define LPTIM_CFGR_TRIGEN           LPTIM_CFGR_TRIGEN_Msk                      /*!< TRIGEN[1:0] bits (Trigger enable and polarity) */
#define LPTIM_CFGR_TRIGEN_0         (0x1UL << LPTIM_CFGR_TRIGEN_Pos)            /*!< 0x00020000 */
#define LPTIM_CFGR_TRIGEN_1         (0x2UL << LPTIM_CFGR_TRIGEN_Pos)            /*!< 0x00040000 */
#define LPTIM_CFGR_TIMOUT_Pos       (19UL)
#define LPTIM_CFGR_TIMOUT_Msk       (0x1UL << LPTIM_CFGR_TIMOUT_Pos)           /*!< 0x00080000 */
#define LPTIM_CFGR_TIMOUT           LPTIM_CFGR_TIMOUT_Msk                      /*!< Timout enable */
#define LPTIM_CFGR_WAVE_Pos         (20UL)
#define LPTIM_CFGR_WAVE_Msk         (0x1UL << LPTIM_CFGR_WAVE_Pos)             /*!< 0x00100000 */
#define LPTIM_CFGR_WAVE             LPTIM_CFGR_WAVE_Msk                        /*!< Waveform shape */
#define LPTIM_CFGR_WAVPOL_Pos       (21UL)
#define LPTIM_CFGR_WAVPOL_Msk       (0x1UL << LPTIM_CFGR_WAVPOL_Pos)           /*!< 0x00200000 */
#define LPTIM_CFGR_WAVPOL           LPTIM_CFGR_WAVPOL_Msk                      /*!< Waveform shape */
#define LPTIM_CFGR_PRELOAD_Pos      (22UL)
#define LPTIM_CFGR_PRELOAD_Msk      (0x1UL << LPTIM_CFGR_PRELOAD_Pos)          /*!< 0x00400000 */
#define LPTIM_CFGR_PRELOAD          LPTIM_CFGR_PRELOAD_Msk                     /*!< Reg update mode */
#define LPTIM_CFGR_COUNTMODE_Pos    (23UL)
#define LPTIM_CFGR_COUNTMODE_Msk    (0x1UL << LPTIM_CFGR_COUNTMODE_Pos)        /*!< 0x00800000 */
#define LPTIM_CFGR_COUNTMODE        LPTIM_CFGR_COUNTMODE_Msk                   /*!< Counter mode enable */
#define LPTIM_CFGR_ENC_Pos          (24UL)
#define LPTIM_CFGR_ENC_Msk          (0x1UL << LPTIM_CFGR_ENC_Pos)              /*!< 0x01000000 */
#define LPTIM_CFGR_ENC              LPTIM_CFGR_ENC_Msk                         /*!< Encoder mode enable */

/******************  Bit definition for LPTIM_CR register  ********************/
#define LPTIM_CR_ENABLE_Pos         (0UL)
#define LPTIM_CR_ENABLE_Msk         (0x1UL << LPTIM_CR_ENABLE_Pos)             /*!< 0x00000001 */
#define LPTIM_CR_ENABLE             LPTIM_CR_ENABLE_Msk                        /*!< LPTIMer enable */
#define LPTIM_CR_SNGSTRT_Pos        (1UL)
#define LPTIM_CR_SNGSTRT_Msk        (0x1UL << LPTIM_CR_SNGSTRT_Pos)            /*!< 0x00000002 */
#define LPTIM_CR_SNGSTRT            LPTIM_CR_SNGSTRT_Msk                       /*!< Timer start in single mode */
#define LPTIM_CR_CNTSTRT_Pos        (2UL)
#define LPTIM_CR_CNTSTRT_Msk        (0x1UL << LPTIM_CR_CNTSTRT_Pos)            /*!< 0x00000004 */
#define LPTIM_CR_CNTSTRT            LPTIM_CR_CNTSTRT_Msk                       /*!< Timer start in continuous mode */
#define LPTIM_CR_COUNTRST_Pos       (3UL)
#define LPTIM_CR_COUNTRST_Msk       (0x1UL << LPTIM_CR_COUNTRST_Pos)           /*!< 0x00000008 */
#define LPTIM_CR_COUNTRST           LPTIM_CR_COUNTRST_Msk                      /*!< Timer Counter reset in synchronous mode*/
#define LPTIM_CR_RSTARE_Pos         (4UL)
#define LPTIM_CR_RSTARE_Msk         (0x1UL << LPTIM_CR_RSTARE_Pos)             /*!< 0x00000010 */
#define LPTIM_CR_RSTARE             LPTIM_CR_RSTARE_Msk                        /*!< Timer Counter reset after read enable (asynchronously)*/

/******************  Bit definition for LPTIM_CCR1 register  ******************/
#define LPTIM_CCR1_CCR1_Pos         (0UL)
#define LPTIM_CCR1_CCR1_Msk         (0xFFFFUL << LPTIM_CCR1_CCR1_Pos)          /*!< 0x0000FFFF */
#define LPTIM_CCR1_CCR1             LPTIM_CCR1_CCR1_Msk                        /*!< Compare register 1 */

/******************  Bit definition for LPTIM_ARR register  *******************/
#define LPTIM_ARR_ARR_Pos           (0UL)
#define LPTIM_ARR_ARR_Msk           (0xFFFFUL << LPTIM_ARR_ARR_Pos)            /*!< 0x0000FFFF */
#define LPTIM_ARR_ARR               LPTIM_ARR_ARR_Msk                          /*!< Auto reload register */

/******************  Bit definition for LPTIM_CNT register  *******************/
#define LPTIM_CNT_CNT_Pos           (0UL)
#define LPTIM_CNT_CNT_Msk           (0xFFFFUL << LPTIM_CNT_CNT_Pos)            /*!< 0x0000FFFF */
#define LPTIM_CNT_CNT               LPTIM_CNT_CNT_Msk                          /*!< Counter register */

/******************  Bit definition for LPTIM_CFGR2 register  *****************/
#define LPTIM_CFGR2_IN1SEL_Pos      (0UL)
#define LPTIM_CFGR2_IN1SEL_Msk      (0x3UL << LPTIM_CFGR2_IN1SEL_Pos)          /*!< 0x00000003 */
#define LPTIM_CFGR2_IN1SEL          LPTIM_CFGR2_IN1SEL_Msk                     /*!< IN1SEL[1:0] bits (Remap selection) */
#define LPTIM_CFGR2_IN1SEL_0        (0x1UL << LPTIM_CFGR2_IN1SEL_Pos)           /*!< 0x00000001 */
#define LPTIM_CFGR2_IN1SEL_1        (0x2UL << LPTIM_CFGR2_IN1SEL_Pos)           /*!< 0x00000002 */
#define LPTIM_CFGR2_IN2SEL_Pos      (4UL)
#define LPTIM_CFGR2_IN2SEL_Msk      (0x3UL << LPTIM_CFGR2_IN2SEL_Pos)          /*!< 0x00000030 */
#define LPTIM_CFGR2_IN2SEL          LPTIM_CFGR2_IN2SEL_Msk                     /*!< IN2SEL[5:4] bits (Remap selection) */
#define LPTIM_CFGR2_IN2SEL_0        (0x1UL << LPTIM_CFGR2_IN2SEL_Pos)           /*!< 0x00000010 */
#define LPTIM_CFGR2_IN2SEL_1        (0x2UL << LPTIM_CFGR2_IN2SEL_Pos)           /*!< 0x00000020 */
#define LPTIM_CFGR2_IC1SEL_Pos      (16UL)
#define LPTIM_CFGR2_IC1SEL_Msk      (0x3UL << LPTIM_CFGR2_IC1SEL_Pos)          /*!< 0x00030000 */
#define LPTIM_CFGR2_IC1SEL          LPTIM_CFGR2_IC1SEL_Msk                     /*!< IC1SEL[17:16] bits */
#define LPTIM_CFGR2_IC1SEL_0        (0x1UL << LPTIM_CFGR2_IC1SEL_Pos)          /*!< 0x00010000 */
#define LPTIM_CFGR2_IC1SEL_1        (0x2UL << LPTIM_CFGR2_IC1SEL_Pos)          /*!< 0x00020000 */
#define LPTIM_CFGR2_IC2SEL_Pos      (20UL)
#define LPTIM_CFGR2_IC2SEL_Msk      (0x3UL << LPTIM_CFGR2_IC2SEL_Pos)          /*!< 0x00300000 */
#define LPTIM_CFGR2_IC2SEL          LPTIM_CFGR2_IC2SEL_Msk                     /*!< IC2SEL[21:20] bits */
#define LPTIM_CFGR2_IC2SEL_0        (0x1UL << LPTIM_CFGR2_IC2SEL_Pos)          /*!< 0x00100000 */
#define LPTIM_CFGR2_IC2SEL_1        (0x2UL << LPTIM_CFGR2_IC2SEL_Pos)          /*!< 0x00200000 */
#define LPTIM_CFGR2_IC3SEL_Pos      (24UL)
#define LPTIM_CFGR2_IC3SEL_Msk      (0x3UL << LPTIM_CFGR2_IC3SEL_Pos)          /*!< 0x03000000 */
#define LPTIM_CFGR2_IC3SEL          LPTIM_CFGR2_IC3SEL_Msk                     /*!< IC3SEL[25:24] bits */
#define LPTIM_CFGR2_IC3SEL_0        (0x1UL << LPTIM_CFGR2_IC3SEL_Pos)          /*!< 0x01000000 */
#define LPTIM_CFGR2_IC3SEL_1        (0x2UL << LPTIM_CFGR2_IC3SEL_Pos)          /*!< 0x02000000 */
#define LPTIM_CFGR2_IC4SEL_Pos      (28UL)
#define LPTIM_CFGR2_IC4SEL_Msk      (0x3UL << LPTIM_CFGR2_IC4SEL_Pos)          /*!< 0x30000000 */
#define LPTIM_CFGR2_IC4SEL          LPTIM_CFGR2_IC4SEL_Msk                     /*!< IC4SEL[29:28] bits */
#define LPTIM_CFGR2_IC4SEL_0        (0x1UL << LPTIM_CFGR2_IC4SEL_Pos)          /*!< 0x10000000 */
#define LPTIM_CFGR2_IC4SEL_1        (0x2UL << LPTIM_CFGR2_IC4SEL_Pos)          /*!< 0x20000000 */

/******************  Bit definition for LPTIM_RCR register  *******************/
#define LPTIM_RCR_REP_Pos           (0UL)
#define LPTIM_RCR_REP_Msk           (0xFFUL << LPTIM_RCR_REP_Pos)              /*!< 0x000000FF */
#define LPTIM_RCR_REP               LPTIM_RCR_REP_Msk                          /*!< Repetition register value */

/*****************  Bit definition for LPTIM_CCMR1 register  ******************/
#define LPTIM_CCMR1_CC1SEL_Pos      (0UL)
#define LPTIM_CCMR1_CC1SEL_Msk      (0x1UL << LPTIM_CCMR1_CC1SEL_Pos)           /*!< 0x00000001 */
#define LPTIM_CCMR1_CC1SEL          LPTIM_CCMR1_CC1SEL_Msk                      /*!< Capture/Compare 1 selection */
#define LPTIM_CCMR1_CC1E_Pos        (1UL)
#define LPTIM_CCMR1_CC1E_Msk        (0x1UL << LPTIM_CCMR1_CC1E_Pos)             /*!< 0x00000002 */
#define LPTIM_CCMR1_CC1E            LPTIM_CCMR1_CC1E_Msk                        /*!< Capture/Compare 1 output enable */
#define LPTIM_CCMR1_CC1P_Pos        (2UL)
#define LPTIM_CCMR1_CC1P_Msk        (0x3UL << LPTIM_CCMR1_CC1P_Pos)             /*!< 0x0000000C */
#define LPTIM_CCMR1_CC1P            LPTIM_CCMR1_CC1P_Msk                        /*!< Capture/Compare 1 output polarity */
#define LPTIM_CCMR1_CC1P_0          (0x1UL << LPTIM_CCMR1_CC1P_Pos)             /*!< 0x00000004 */
#define LPTIM_CCMR1_CC1P_1          (0x2UL << LPTIM_CCMR1_CC1P_Pos)             /*!< 0x00000008 */
#define LPTIM_CCMR1_IC1PSC_Pos      (8UL)
#define LPTIM_CCMR1_IC1PSC_Msk      (0x3UL << LPTIM_CCMR1_IC1PSC_Pos)           /*!< 0x00000300 */
#define LPTIM_CCMR1_IC1PSC          LPTIM_CCMR1_IC1PSC_Msk                      /*!< Input capture 1 prescaler */
#define LPTIM_CCMR1_IC1PSC_0        (0x1UL << LPTIM_CCMR1_IC1PSC_Pos)           /*!< 0x00000100 */
#define LPTIM_CCMR1_IC1PSC_1        (0x2UL << LPTIM_CCMR1_IC1PSC_Pos)           /*!< 0x00000200 */
#define LPTIM_CCMR1_IC1F_Pos        (12UL)
#define LPTIM_CCMR1_IC1F_Msk        (0x3UL << LPTIM_CCMR1_IC1F_Pos)             /*!< 0x00003000 */
#define LPTIM_CCMR1_IC1F            LPTIM_CCMR1_IC1F_Msk                        /*!< Input capture 1 filter */
#define LPTIM_CCMR1_IC1F_0          (0x1UL << LPTIM_CCMR1_IC1F_Pos)             /*!< 0x00001000 */
#define LPTIM_CCMR1_IC1F_1          (0x2UL << LPTIM_CCMR1_IC1F_Pos)             /*!< 0x00002000 */
#define LPTIM_CCMR1_CC2SEL_Pos      (16UL)
#define LPTIM_CCMR1_CC2SEL_Msk      (0x1UL << LPTIM_CCMR1_CC2SEL_Pos)           /*!< 0x00010000 */
#define LPTIM_CCMR1_CC2SEL          LPTIM_CCMR1_CC2SEL_Msk                      /*!< Capture/Compare 2 selection */
#define LPTIM_CCMR1_CC2E_Pos        (17UL)
#define LPTIM_CCMR1_CC2E_Msk        (0x1UL << LPTIM_CCMR1_CC2E_Pos)             /*!< 0x00020000 */
#define LPTIM_CCMR1_CC2E            LPTIM_CCMR1_CC2E_Msk                        /*!< Capture/Compare 2 output enable */
#define LPTIM_CCMR1_CC2P_Pos        (18UL)
#define LPTIM_CCMR1_CC2P_Msk        (0x3UL << LPTIM_CCMR1_CC2P_Pos)             /*!< 0x000C0000 */
#define LPTIM_CCMR1_CC2P            LPTIM_CCMR1_CC2P_Msk                        /*!< Capture/Compare 2 output polarity */
#define LPTIM_CCMR1_CC2P_0          (0x1UL << LPTIM_CCMR1_CC2P_Pos)             /*!< 0x00040000 */
#define LPTIM_CCMR1_CC2P_1          (0x2UL << LPTIM_CCMR1_CC2P_Pos)             /*!< 0x00080000 */
#define LPTIM_CCMR1_IC2PSC_Pos      (24UL)
#define LPTIM_CCMR1_IC2PSC_Msk      (0x3UL << LPTIM_CCMR1_IC2PSC_Pos)           /*!< 0x03000000 */
#define LPTIM_CCMR1_IC2PSC          LPTIM_CCMR1_IC2PSC_Msk                      /*!< Input capture 2 prescaler */
#define LPTIM_CCMR1_IC2PSC_0        (0x1UL << LPTIM_CCMR1_IC2PSC_Pos)           /*!< 0x01000000 */
#define LPTIM_CCMR1_IC2PSC_1        (0x2UL << LPTIM_CCMR1_IC2PSC_Pos)           /*!< 0x02000000 */
#define LPTIM_CCMR1_IC2F_Pos        (28UL)
#define LPTIM_CCMR1_IC2F_Msk        (0x3UL << LPTIM_CCMR1_IC2F_Pos)             /*!< 0x30000000 */
#define LPTIM_CCMR1_IC2F            LPTIM_CCMR1_IC2F_Msk                        /*!< Input capture 2 filter */
#define LPTIM_CCMR1_IC2F_0          (0x1UL << LPTIM_CCMR1_IC2F_Pos)             /*!< 0x10000000 */
#define LPTIM_CCMR1_IC2F_1          (0x2UL << LPTIM_CCMR1_IC2F_Pos)             /*!< 0x20000000 */

/*****************  Bit definition for LPTIM_CCMR2 register  ******************/
#define LPTIM_CCMR2_CC3SEL_Pos      (0UL)
#define LPTIM_CCMR2_CC3SEL_Msk      (0x1UL << LPTIM_CCMR2_CC3SEL_Pos)           /*!< 0x00000001 */
#define LPTIM_CCMR2_CC3SEL          LPTIM_CCMR2_CC3SEL_Msk                      /*!< Capture/Compare 3 selection */
#define LPTIM_CCMR2_CC3E_Pos        (1UL)
#define LPTIM_CCMR2_CC3E_Msk        (0x1UL << LPTIM_CCMR2_CC3E_Pos)             /*!< 0x00000002 */
#define LPTIM_CCMR2_CC3E            LPTIM_CCMR2_CC3E_Msk                        /*!< Capture/Compare 3 output enable */
#define LPTIM_CCMR2_CC3P_Pos        (2UL)
#define LPTIM_CCMR2_CC3P_Msk        (0x3UL << LPTIM_CCMR2_CC3P_Pos)             /*!< 0x0000000C */
#define LPTIM_CCMR2_CC3P            LPTIM_CCMR2_CC3P_Msk                        /*!< Capture/Compare 3 output polarity */
#define LPTIM_CCMR2_CC3P_0          (0x1UL << LPTIM_CCMR2_CC3P_Pos)             /*!< 0x00000004 */
#define LPTIM_CCMR2_CC3P_1          (0x2UL << LPTIM_CCMR2_CC3P_Pos)             /*!< 0x00000008 */
#define LPTIM_CCMR2_IC3PSC_Pos      (8UL)
#define LPTIM_CCMR2_IC3PSC_Msk      (0x3UL << LPTIM_CCMR2_IC3PSC_Pos)           /*!< 0x00000300 */
#define LPTIM_CCMR2_IC3PSC          LPTIM_CCMR2_IC3PSC_Msk                      /*!< Input capture 3 prescaler */
#define LPTIM_CCMR2_IC3PSC_0        (0x1UL << LPTIM_CCMR2_IC3PSC_Pos)           /*!< 0x00000100 */
#define LPTIM_CCMR2_IC3PSC_1        (0x2UL << LPTIM_CCMR2_IC3PSC_Pos)           /*!< 0x00000200 */
#define LPTIM_CCMR2_IC3F_Pos        (12UL)
#define LPTIM_CCMR2_IC3F_Msk        (0x3UL << LPTIM_CCMR2_IC3F_Pos)             /*!< 0x00003000 */
#define LPTIM_CCMR2_IC3F            LPTIM_CCMR2_IC3F_Msk                        /*!< Input capture 3 filter */
#define LPTIM_CCMR2_IC3F_0          (0x1UL << LPTIM_CCMR2_IC3F_Pos)             /*!< 0x00001000 */
#define LPTIM_CCMR2_IC3F_1          (0x2UL << LPTIM_CCMR2_IC3F_Pos)             /*!< 0x00002000 */
#define LPTIM_CCMR2_CC4SEL_Pos      (16UL)
#define LPTIM_CCMR2_CC4SEL_Msk      (0x1UL << LPTIM_CCMR2_CC4SEL_Pos)           /*!< 0x00010000 */
#define LPTIM_CCMR2_CC4SEL          LPTIM_CCMR2_CC4SEL_Msk                      /*!< Capture/Compare 4 selection */
#define LPTIM_CCMR2_CC4E_Pos        (17UL)
#define LPTIM_CCMR2_CC4E_Msk        (0x1UL << LPTIM_CCMR2_CC4E_Pos)             /*!< 0x00020000 */
#define LPTIM_CCMR2_CC4E            LPTIM_CCMR2_CC4E_Msk                        /*!< Capture/Compare 4 output enable */
#define LPTIM_CCMR2_CC4P_Pos        (18UL)
#define LPTIM_CCMR2_CC4P_Msk        (0x3UL << LPTIM_CCMR2_CC4P_Pos)             /*!< 0x000C0000 */
#define LPTIM_CCMR2_CC4P            LPTIM_CCMR2_CC4P_Msk                        /*!< Capture/Compare 4 output polarity */
#define LPTIM_CCMR2_CC4P_0          (0x1UL << LPTIM_CCMR2_CC4P_Pos)             /*!< 0x00040000 */
#define LPTIM_CCMR2_CC4P_1          (0x2UL << LPTIM_CCMR2_CC4P_Pos)             /*!< 0x00080000 */
#define LPTIM_CCMR2_IC4PSC_Pos      (24UL)
#define LPTIM_CCMR2_IC4PSC_Msk      (0x3UL << LPTIM_CCMR2_IC4PSC_Pos)           /*!< 0x03000000 */
#define LPTIM_CCMR2_IC4PSC          LPTIM_CCMR2_IC4PSC_Msk                      /*!< Input capture 4 prescaler */
#define LPTIM_CCMR2_IC4PSC_0        (0x1UL << LPTIM_CCMR2_IC4PSC_Pos)           /*!< 0x01000000 */
#define LPTIM_CCMR2_IC4PSC_1        (0x2UL << LPTIM_CCMR2_IC4PSC_Pos)           /*!< 0x02000000 */
#define LPTIM_CCMR2_IC4F_Pos        (28UL)
#define LPTIM_CCMR2_IC4F_Msk        (0x3UL << LPTIM_CCMR2_IC4F_Pos)             /*!< 0x30000000 */
#define LPTIM_CCMR2_IC4F            LPTIM_CCMR2_IC4F_Msk                        /*!< Input capture 4 filter */
#define LPTIM_CCMR2_IC4F_0          (0x1UL << LPTIM_CCMR2_IC4F_Pos)             /*!< 0x10000000 */
#define LPTIM_CCMR2_IC4F_1          (0x2UL << LPTIM_CCMR2_IC4F_Pos)             /*!< 0x20000000 */

/******************  Bit definition for LPTIM_CCR2 register  ******************/
#define LPTIM_CCR2_CCR2_Pos         (0UL)
#define LPTIM_CCR2_CCR2_Msk         (0xFFFFUL << LPTIM_CCR2_CCR2_Pos)          /*!< 0x0000FFFF */
#define LPTIM_CCR2_CCR2             LPTIM_CCR2_CCR2_Msk                        /*!< Compare register 2 */

/******************  Bit definition for LPTIM_CCR3 register  ******************/
#define LPTIM_CCR3_CCR3_Pos         (0UL)
#define LPTIM_CCR3_CCR3_Msk         (0xFFFFUL << LPTIM_CCR3_CCR3_Pos)          /*!< 0x0000FFFF */
#define LPTIM_CCR3_CCR3             LPTIM_CCR3_CCR3_Msk                        /*!< Compare register 3 */

/******************  Bit definition for LPTIM_CCR4 register  ******************/
#define LPTIM_CCR4_CCR4_Pos         (0UL)
#define LPTIM_CCR4_CCR4_Msk         (0xFFFFUL << LPTIM_CCR4_CCR4_Pos)          /*!< 0x0000FFFF */
#define LPTIM_CCR4_CCR4             LPTIM_CCR4_CCR4_Msk                        /*!< Compare register 4 */

/******************************************************************************/
/*                                                                            */
/*                      Analog Comparators (COMP)                             */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for COMPx_CSR register  *******************/
#define COMP_CSR_EN_Pos                (0UL)
#define COMP_CSR_EN_Msk                (0x1UL << COMP_CSR_EN_Pos)              /*!< 0x00000001 */
#define COMP_CSR_EN                    COMP_CSR_EN_Msk                         /*!< COMPx enable bit */
#define COMP_CSR_INMSEL_Pos            (4UL)
#define COMP_CSR_INMSEL_Msk            (0xFUL << COMP_CSR_INMSEL_Pos)          /*!< 0x000000F0 */
#define COMP_CSR_INMSEL                COMP_CSR_INMSEL_Msk                     /*!< COMPx input minus selection bit  */
#define COMP_CSR_INMSEL_0              (0x1UL << COMP_CSR_INMSEL_Pos)          /*!< 0x00000010 */
#define COMP_CSR_INMSEL_1              (0x2UL << COMP_CSR_INMSEL_Pos)          /*!< 0x00000020 */
#define COMP_CSR_INMSEL_2              (0x4UL << COMP_CSR_INMSEL_Pos)          /*!< 0x00000040 */
#define COMP_CSR_INMSEL_3              (0x8UL << COMP_CSR_INMSEL_Pos)          /*!< 0x00000080 */
#define COMP_CSR_INPSEL_Pos            (8UL)
#define COMP_CSR_INPSEL_Msk            (0x7UL << COMP_CSR_INPSEL_Pos)          /*!< 0x00000700 */
#define COMP_CSR_INPSEL                COMP_CSR_INPSEL_Msk                     /*!< COMPx input plus selection bit */
#define COMP_CSR_INPSEL_0              (0x1UL << COMP_CSR_INPSEL_Pos)          /*!< 0x00000100 */
#define COMP_CSR_INPSEL_1              (0x2UL << COMP_CSR_INPSEL_Pos)          /*!< 0x00000200 */
#define COMP_CSR_INPSEL_2              (0x4UL << COMP_CSR_INPSEL_Pos)          /*!< 0x00000400 */
#define COMP_CSR_WINMODE_Pos           (11UL)
#define COMP_CSR_WINMODE_Msk           (0x1UL << COMP_CSR_WINMODE_Pos)         /*!< 0x00000800 */
#define COMP_CSR_WINMODE               COMP_CSR_WINMODE_Msk                    /*!< COMPx Windows mode selection bit */
#define COMP_CSR_WINOUT_Pos            (14UL)
#define COMP_CSR_WINOUT_Msk            (0x1UL << COMP_CSR_WINOUT_Pos)          /*!< 0x00004000 */
#define COMP_CSR_WINOUT                COMP_CSR_WINOUT_Msk                     /*!< COMPx polarity selection bit */
#define COMP_CSR_POLARITY_Pos          (15UL)
#define COMP_CSR_POLARITY_Msk          (0x1UL << COMP_CSR_POLARITY_Pos)        /*!< 0x00008000 */
#define COMP_CSR_POLARITY              COMP_CSR_POLARITY_Msk                   /*!< COMPx polarity selection bit */
#define COMP_CSR_HYST_Pos              (16UL)
#define COMP_CSR_HYST_Msk              (0x3UL << COMP_CSR_HYST_Pos)            /*!< 0x00030000 */
#define COMP_CSR_HYST                  COMP_CSR_HYST_Msk                       /*!< COMPx hysteresis selection bits */
#define COMP_CSR_HYST_0                (0x1UL << COMP_CSR_HYST_Pos)            /*!< 0x00010000 */
#define COMP_CSR_HYST_1                (0x2UL << COMP_CSR_HYST_Pos)            /*!< 0x00020000 */
#define COMP_CSR_PWRMODE_Pos           (18UL)
#define COMP_CSR_PWRMODE_Msk           (0x3UL << COMP_CSR_PWRMODE_Pos)         /*!< 0x000C0000 */
#define COMP_CSR_PWRMODE               COMP_CSR_PWRMODE_Msk                    /*!< COMPx Power Mode of the comparator */
#define COMP_CSR_PWRMODE_0             (0x1UL << COMP_CSR_PWRMODE_Pos)         /*!< 0x00010000 */
#define COMP_CSR_PWRMODE_1             (0x2UL << COMP_CSR_PWRMODE_Pos)         /*!< 0x00020000 */
#define COMP_CSR_BLANKSEL_Pos          (20UL)
#define COMP_CSR_BLANKSEL_Msk          (0x1FUL << COMP_CSR_BLANKSEL_Pos)       /*!< 0x01F00000 */
#define COMP_CSR_BLANKSEL              COMP_CSR_BLANKSEL_Msk                   /*!< COMPx blanking source selection bits */
#define COMP_CSR_BLANKSEL_0            (0x1UL << COMP_CSR_BLANKSEL_Pos)        /*!< 0x00100000 */
#define COMP_CSR_BLANKSEL_1            (0x2UL << COMP_CSR_BLANKSEL_Pos)        /*!< 0x00200000 */
#define COMP_CSR_BLANKSEL_2            (0x4UL << COMP_CSR_BLANKSEL_Pos)        /*!< 0x00400000 */
#define COMP_CSR_BLANKSEL_3            (0x8UL << COMP_CSR_BLANKSEL_Pos)        /*!< 0x00800000 */
#define COMP_CSR_BLANKSEL_4            (0x10UL << COMP_CSR_BLANKSEL_Pos)       /*!< 0x01000000 */
#define COMP_CSR_VALUE_Pos             (30UL)
#define COMP_CSR_VALUE_Msk             (0x1UL << COMP_CSR_VALUE_Pos)           /*!< 0x04000000 */
#define COMP_CSR_VALUE                 COMP_CSR_VALUE_Msk                      /*!< Comparator output level */
#define COMP_CSR_LOCK_Pos              (31UL)
#define COMP_CSR_LOCK_Msk              (0x1UL << COMP_CSR_LOCK_Pos)            /*!< 0x80000000 */
#define COMP_CSR_LOCK                  COMP_CSR_LOCK_Msk                       /*!< COMPx Lock Bit */

/******************************************************************************/
/*                                                                            */
/*                          Touch Sensing Controller (TSC)                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TSC_CR register  *********************/
#define TSC_CR_TSCE_Pos          (0UL)
#define TSC_CR_TSCE_Msk          (0x1UL << TSC_CR_TSCE_Pos)                    /*!< 0x00000001 */
#define TSC_CR_TSCE              TSC_CR_TSCE_Msk                               /*!<Touch sensing controller enable */
#define TSC_CR_START_Pos         (1UL)
#define TSC_CR_START_Msk         (0x1UL << TSC_CR_START_Pos)                   /*!< 0x00000002 */
#define TSC_CR_START             TSC_CR_START_Msk                              /*!<Start acquisition */
#define TSC_CR_AM_Pos            (2UL)
#define TSC_CR_AM_Msk            (0x1UL << TSC_CR_AM_Pos)                      /*!< 0x00000004 */
#define TSC_CR_AM                TSC_CR_AM_Msk                                 /*!<Acquisition mode */
#define TSC_CR_SYNCPOL_Pos       (3UL)
#define TSC_CR_SYNCPOL_Msk       (0x1UL << TSC_CR_SYNCPOL_Pos)                 /*!< 0x00000008 */
#define TSC_CR_SYNCPOL           TSC_CR_SYNCPOL_Msk                            /*!<Synchronization pin polarity */
#define TSC_CR_IODEF_Pos         (4UL)
#define TSC_CR_IODEF_Msk         (0x1UL << TSC_CR_IODEF_Pos)                   /*!< 0x00000010 */
#define TSC_CR_IODEF             TSC_CR_IODEF_Msk                              /*!<IO default mode */

#define TSC_CR_MCV_Pos           (5UL)
#define TSC_CR_MCV_Msk           (0x7UL << TSC_CR_MCV_Pos)                     /*!< 0x000000E0 */
#define TSC_CR_MCV               TSC_CR_MCV_Msk                                /*!<MCV[2:0] bits (Max Count Value) */
#define TSC_CR_MCV_0             (0x1UL << TSC_CR_MCV_Pos)                     /*!< 0x00000020 */
#define TSC_CR_MCV_1             (0x2UL << TSC_CR_MCV_Pos)                     /*!< 0x00000040 */
#define TSC_CR_MCV_2             (0x4UL << TSC_CR_MCV_Pos)                     /*!< 0x00000080 */

#define TSC_CR_PGPSC_Pos         (12UL)
#define TSC_CR_PGPSC_Msk         (0x7UL << TSC_CR_PGPSC_Pos)                   /*!< 0x00007000 */
#define TSC_CR_PGPSC             TSC_CR_PGPSC_Msk                              /*!<PGPSC[2:0] bits (Pulse Generator Prescaler) */
#define TSC_CR_PGPSC_0           (0x1UL << TSC_CR_PGPSC_Pos)                   /*!< 0x00001000 */
#define TSC_CR_PGPSC_1           (0x2UL << TSC_CR_PGPSC_Pos)                   /*!< 0x00002000 */
#define TSC_CR_PGPSC_2           (0x4UL << TSC_CR_PGPSC_Pos)                   /*!< 0x00004000 */

#define TSC_CR_SSPSC_Pos         (15UL)
#define TSC_CR_SSPSC_Msk         (0x1UL << TSC_CR_SSPSC_Pos)                   /*!< 0x00008000 */
#define TSC_CR_SSPSC             TSC_CR_SSPSC_Msk                              /*!<Spread Spectrum Prescaler */
#define TSC_CR_SSE_Pos           (16UL)
#define TSC_CR_SSE_Msk           (0x1UL << TSC_CR_SSE_Pos)                     /*!< 0x00010000 */
#define TSC_CR_SSE               TSC_CR_SSE_Msk                                /*!<Spread Spectrum Enable */

#define TSC_CR_SSD_Pos           (17UL)
#define TSC_CR_SSD_Msk           (0x7FUL << TSC_CR_SSD_Pos)                    /*!< 0x00FE0000 */
#define TSC_CR_SSD               TSC_CR_SSD_Msk                                /*!<SSD[6:0] bits (Spread Spectrum Deviation) */
#define TSC_CR_SSD_0             (0x01UL << TSC_CR_SSD_Pos)                    /*!< 0x00020000 */
#define TSC_CR_SSD_1             (0x02UL << TSC_CR_SSD_Pos)                    /*!< 0x00040000 */
#define TSC_CR_SSD_2             (0x04UL << TSC_CR_SSD_Pos)                    /*!< 0x00080000 */
#define TSC_CR_SSD_3             (0x08UL << TSC_CR_SSD_Pos)                    /*!< 0x00100000 */
#define TSC_CR_SSD_4             (0x10UL << TSC_CR_SSD_Pos)                    /*!< 0x00200000 */
#define TSC_CR_SSD_5             (0x20UL << TSC_CR_SSD_Pos)                    /*!< 0x00400000 */
#define TSC_CR_SSD_6             (0x40UL << TSC_CR_SSD_Pos)                    /*!< 0x00800000 */

#define TSC_CR_CTPL_Pos          (24UL)
#define TSC_CR_CTPL_Msk          (0xFUL << TSC_CR_CTPL_Pos)                    /*!< 0x0F000000 */
#define TSC_CR_CTPL              TSC_CR_CTPL_Msk                               /*!<CTPL[3:0] bits (Charge Transfer pulse low) */
#define TSC_CR_CTPL_0            (0x1UL << TSC_CR_CTPL_Pos)                    /*!< 0x01000000 */
#define TSC_CR_CTPL_1            (0x2UL << TSC_CR_CTPL_Pos)                    /*!< 0x02000000 */
#define TSC_CR_CTPL_2            (0x4UL << TSC_CR_CTPL_Pos)                    /*!< 0x04000000 */
#define TSC_CR_CTPL_3            (0x8UL << TSC_CR_CTPL_Pos)                    /*!< 0x08000000 */

#define TSC_CR_CTPH_Pos          (28UL)
#define TSC_CR_CTPH_Msk          (0xFUL << TSC_CR_CTPH_Pos)                    /*!< 0xF0000000 */
#define TSC_CR_CTPH              TSC_CR_CTPH_Msk                               /*!<CTPH[3:0] bits (Charge Transfer pulse high) */
#define TSC_CR_CTPH_0            (0x1UL << TSC_CR_CTPH_Pos)                    /*!< 0x10000000 */
#define TSC_CR_CTPH_1            (0x2UL << TSC_CR_CTPH_Pos)                    /*!< 0x20000000 */
#define TSC_CR_CTPH_2            (0x4UL << TSC_CR_CTPH_Pos)                    /*!< 0x40000000 */
#define TSC_CR_CTPH_3            (0x8UL << TSC_CR_CTPH_Pos)                    /*!< 0x80000000 */

/*******************  Bit definition for TSC_IER register  ********************/
#define TSC_IER_EOAIE_Pos        (0UL)
#define TSC_IER_EOAIE_Msk        (0x1UL << TSC_IER_EOAIE_Pos)                  /*!< 0x00000001 */
#define TSC_IER_EOAIE            TSC_IER_EOAIE_Msk                             /*!<End of acquisition interrupt enable */
#define TSC_IER_MCEIE_Pos        (1UL)
#define TSC_IER_MCEIE_Msk        (0x1UL << TSC_IER_MCEIE_Pos)                  /*!< 0x00000002 */
#define TSC_IER_MCEIE            TSC_IER_MCEIE_Msk                             /*!<Max count error interrupt enable */

/*******************  Bit definition for TSC_ICR register  ********************/
#define TSC_ICR_EOAIC_Pos        (0UL)
#define TSC_ICR_EOAIC_Msk        (0x1UL << TSC_ICR_EOAIC_Pos)                  /*!< 0x00000001 */
#define TSC_ICR_EOAIC            TSC_ICR_EOAIC_Msk                             /*!<End of acquisition interrupt clear */
#define TSC_ICR_MCEIC_Pos        (1UL)
#define TSC_ICR_MCEIC_Msk        (0x1UL << TSC_ICR_MCEIC_Pos)                  /*!< 0x00000002 */
#define TSC_ICR_MCEIC            TSC_ICR_MCEIC_Msk                             /*!<Max count error interrupt clear */

/*******************  Bit definition for TSC_ISR register  ********************/
#define TSC_ISR_EOAF_Pos         (0UL)
#define TSC_ISR_EOAF_Msk         (0x1UL << TSC_ISR_EOAF_Pos)                   /*!< 0x00000001 */
#define TSC_ISR_EOAF             TSC_ISR_EOAF_Msk                              /*!<End of acquisition flag */
#define TSC_ISR_MCEF_Pos         (1UL)
#define TSC_ISR_MCEF_Msk         (0x1UL << TSC_ISR_MCEF_Pos)                   /*!< 0x00000002 */
#define TSC_ISR_MCEF             TSC_ISR_MCEF_Msk                              /*!<Max count error flag */

/*******************  Bit definition for TSC_IOHCR register  ******************/
#define TSC_IOHCR_G1_IO1_Pos     (0UL)
#define TSC_IOHCR_G1_IO1_Msk     (0x1UL << TSC_IOHCR_G1_IO1_Pos)               /*!< 0x00000001 */
#define TSC_IOHCR_G1_IO1         TSC_IOHCR_G1_IO1_Msk                          /*!<GROUP1_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO2_Pos     (1UL)
#define TSC_IOHCR_G1_IO2_Msk     (0x1UL << TSC_IOHCR_G1_IO2_Pos)               /*!< 0x00000002 */
#define TSC_IOHCR_G1_IO2         TSC_IOHCR_G1_IO2_Msk                          /*!<GROUP1_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO3_Pos     (2UL)
#define TSC_IOHCR_G1_IO3_Msk     (0x1UL << TSC_IOHCR_G1_IO3_Pos)               /*!< 0x00000004 */
#define TSC_IOHCR_G1_IO3         TSC_IOHCR_G1_IO3_Msk                          /*!<GROUP1_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO4_Pos     (3UL)
#define TSC_IOHCR_G1_IO4_Msk     (0x1UL << TSC_IOHCR_G1_IO4_Pos)               /*!< 0x00000008 */
#define TSC_IOHCR_G1_IO4         TSC_IOHCR_G1_IO4_Msk                          /*!<GROUP1_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO1_Pos     (4UL)
#define TSC_IOHCR_G2_IO1_Msk     (0x1UL << TSC_IOHCR_G2_IO1_Pos)               /*!< 0x00000010 */
#define TSC_IOHCR_G2_IO1         TSC_IOHCR_G2_IO1_Msk                          /*!<GROUP2_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO2_Pos     (5UL)
#define TSC_IOHCR_G2_IO2_Msk     (0x1UL << TSC_IOHCR_G2_IO2_Pos)               /*!< 0x00000020 */
#define TSC_IOHCR_G2_IO2         TSC_IOHCR_G2_IO2_Msk                          /*!<GROUP2_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO3_Pos     (6UL)
#define TSC_IOHCR_G2_IO3_Msk     (0x1UL << TSC_IOHCR_G2_IO3_Pos)               /*!< 0x00000040 */
#define TSC_IOHCR_G2_IO3         TSC_IOHCR_G2_IO3_Msk                          /*!<GROUP2_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO4_Pos     (7UL)
#define TSC_IOHCR_G2_IO4_Msk     (0x1UL << TSC_IOHCR_G2_IO4_Pos)               /*!< 0x00000080 */
#define TSC_IOHCR_G2_IO4         TSC_IOHCR_G2_IO4_Msk                          /*!<GROUP2_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO1_Pos     (8UL)
#define TSC_IOHCR_G3_IO1_Msk     (0x1UL << TSC_IOHCR_G3_IO1_Pos)               /*!< 0x00000100 */
#define TSC_IOHCR_G3_IO1         TSC_IOHCR_G3_IO1_Msk                          /*!<GROUP3_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO2_Pos     (9UL)
#define TSC_IOHCR_G3_IO2_Msk     (0x1UL << TSC_IOHCR_G3_IO2_Pos)               /*!< 0x00000200 */
#define TSC_IOHCR_G3_IO2         TSC_IOHCR_G3_IO2_Msk                          /*!<GROUP3_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO3_Pos     (10UL)
#define TSC_IOHCR_G3_IO3_Msk     (0x1UL << TSC_IOHCR_G3_IO3_Pos)               /*!< 0x00000400 */
#define TSC_IOHCR_G3_IO3         TSC_IOHCR_G3_IO3_Msk                          /*!<GROUP3_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO4_Pos     (11UL)
#define TSC_IOHCR_G3_IO4_Msk     (0x1UL << TSC_IOHCR_G3_IO4_Pos)               /*!< 0x00000800 */
#define TSC_IOHCR_G3_IO4         TSC_IOHCR_G3_IO4_Msk                          /*!<GROUP3_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO1_Pos     (12UL)
#define TSC_IOHCR_G4_IO1_Msk     (0x1UL << TSC_IOHCR_G4_IO1_Pos)               /*!< 0x00001000 */
#define TSC_IOHCR_G4_IO1         TSC_IOHCR_G4_IO1_Msk                          /*!<GROUP4_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO2_Pos     (13UL)
#define TSC_IOHCR_G4_IO2_Msk     (0x1UL << TSC_IOHCR_G4_IO2_Pos)               /*!< 0x00002000 */
#define TSC_IOHCR_G4_IO2         TSC_IOHCR_G4_IO2_Msk                          /*!<GROUP4_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO3_Pos     (14UL)
#define TSC_IOHCR_G4_IO3_Msk     (0x1UL << TSC_IOHCR_G4_IO3_Pos)               /*!< 0x00004000 */
#define TSC_IOHCR_G4_IO3         TSC_IOHCR_G4_IO3_Msk                          /*!<GROUP4_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO4_Pos     (15UL)
#define TSC_IOHCR_G4_IO4_Msk     (0x1UL << TSC_IOHCR_G4_IO4_Pos)               /*!< 0x00008000 */
#define TSC_IOHCR_G4_IO4         TSC_IOHCR_G4_IO4_Msk                          /*!<GROUP4_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO1_Pos     (16UL)
#define TSC_IOHCR_G5_IO1_Msk     (0x1UL << TSC_IOHCR_G5_IO1_Pos)               /*!< 0x00010000 */
#define TSC_IOHCR_G5_IO1         TSC_IOHCR_G5_IO1_Msk                          /*!<GROUP5_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO2_Pos     (17UL)
#define TSC_IOHCR_G5_IO2_Msk     (0x1UL << TSC_IOHCR_G5_IO2_Pos)               /*!< 0x00020000 */
#define TSC_IOHCR_G5_IO2         TSC_IOHCR_G5_IO2_Msk                          /*!<GROUP5_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO3_Pos     (18UL)
#define TSC_IOHCR_G5_IO3_Msk     (0x1UL << TSC_IOHCR_G5_IO3_Pos)               /*!< 0x00040000 */
#define TSC_IOHCR_G5_IO3         TSC_IOHCR_G5_IO3_Msk                          /*!<GROUP5_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO4_Pos     (19UL)
#define TSC_IOHCR_G5_IO4_Msk     (0x1UL << TSC_IOHCR_G5_IO4_Pos)               /*!< 0x00080000 */
#define TSC_IOHCR_G5_IO4         TSC_IOHCR_G5_IO4_Msk                          /*!<GROUP5_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO1_Pos     (20UL)
#define TSC_IOHCR_G6_IO1_Msk     (0x1UL << TSC_IOHCR_G6_IO1_Pos)               /*!< 0x00100000 */
#define TSC_IOHCR_G6_IO1         TSC_IOHCR_G6_IO1_Msk                          /*!<GROUP6_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO2_Pos     (21UL)
#define TSC_IOHCR_G6_IO2_Msk     (0x1UL << TSC_IOHCR_G6_IO2_Pos)               /*!< 0x00200000 */
#define TSC_IOHCR_G6_IO2         TSC_IOHCR_G6_IO2_Msk                          /*!<GROUP6_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO3_Pos     (22UL)
#define TSC_IOHCR_G6_IO3_Msk     (0x1UL << TSC_IOHCR_G6_IO3_Pos)               /*!< 0x00400000 */
#define TSC_IOHCR_G6_IO3         TSC_IOHCR_G6_IO3_Msk                          /*!<GROUP6_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO4_Pos     (23UL)
#define TSC_IOHCR_G6_IO4_Msk     (0x1UL << TSC_IOHCR_G6_IO4_Pos)               /*!< 0x00800000 */
#define TSC_IOHCR_G6_IO4         TSC_IOHCR_G6_IO4_Msk                          /*!<GROUP6_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO1_Pos     (24UL)
#define TSC_IOHCR_G7_IO1_Msk     (0x1UL << TSC_IOHCR_G7_IO1_Pos)               /*!< 0x01000000 */
#define TSC_IOHCR_G7_IO1         TSC_IOHCR_G7_IO1_Msk                          /*!<GROUP7_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO2_Pos     (25UL)
#define TSC_IOHCR_G7_IO2_Msk     (0x1UL << TSC_IOHCR_G7_IO2_Pos)               /*!< 0x02000000 */
#define TSC_IOHCR_G7_IO2         TSC_IOHCR_G7_IO2_Msk                          /*!<GROUP7_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO3_Pos     (26UL)
#define TSC_IOHCR_G7_IO3_Msk     (0x1UL << TSC_IOHCR_G7_IO3_Pos)               /*!< 0x04000000 */
#define TSC_IOHCR_G7_IO3         TSC_IOHCR_G7_IO3_Msk                          /*!<GROUP7_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO4_Pos     (27UL)
#define TSC_IOHCR_G7_IO4_Msk     (0x1UL << TSC_IOHCR_G7_IO4_Pos)               /*!< 0x08000000 */
#define TSC_IOHCR_G7_IO4         TSC_IOHCR_G7_IO4_Msk                          /*!<GROUP7_IO4 schmitt trigger hysteresis mode */

/*******************  Bit definition for TSC_IOASCR register  *****************/
#define TSC_IOASCR_G1_IO1_Pos    (0UL)
#define TSC_IOASCR_G1_IO1_Msk    (0x1UL << TSC_IOASCR_G1_IO1_Pos)              /*!< 0x00000001 */
#define TSC_IOASCR_G1_IO1        TSC_IOASCR_G1_IO1_Msk                         /*!<GROUP1_IO1 analog switch enable */
#define TSC_IOASCR_G1_IO2_Pos    (1UL)
#define TSC_IOASCR_G1_IO2_Msk    (0x1UL << TSC_IOASCR_G1_IO2_Pos)              /*!< 0x00000002 */
#define TSC_IOASCR_G1_IO2        TSC_IOASCR_G1_IO2_Msk                         /*!<GROUP1_IO2 analog switch enable */
#define TSC_IOASCR_G1_IO3_Pos    (2UL)
#define TSC_IOASCR_G1_IO3_Msk    (0x1UL << TSC_IOASCR_G1_IO3_Pos)              /*!< 0x00000004 */
#define TSC_IOASCR_G1_IO3        TSC_IOASCR_G1_IO3_Msk                         /*!<GROUP1_IO3 analog switch enable */
#define TSC_IOASCR_G1_IO4_Pos    (3UL)
#define TSC_IOASCR_G1_IO4_Msk    (0x1UL << TSC_IOASCR_G1_IO4_Pos)              /*!< 0x00000008 */
#define TSC_IOASCR_G1_IO4        TSC_IOASCR_G1_IO4_Msk                         /*!<GROUP1_IO4 analog switch enable */
#define TSC_IOASCR_G2_IO1_Pos    (4UL)
#define TSC_IOASCR_G2_IO1_Msk    (0x1UL << TSC_IOASCR_G2_IO1_Pos)              /*!< 0x00000010 */
#define TSC_IOASCR_G2_IO1        TSC_IOASCR_G2_IO1_Msk                         /*!<GROUP2_IO1 analog switch enable */
#define TSC_IOASCR_G2_IO2_Pos    (5UL)
#define TSC_IOASCR_G2_IO2_Msk    (0x1UL << TSC_IOASCR_G2_IO2_Pos)              /*!< 0x00000020 */
#define TSC_IOASCR_G2_IO2        TSC_IOASCR_G2_IO2_Msk                         /*!<GROUP2_IO2 analog switch enable */
#define TSC_IOASCR_G2_IO3_Pos    (6UL)
#define TSC_IOASCR_G2_IO3_Msk    (0x1UL << TSC_IOASCR_G2_IO3_Pos)              /*!< 0x00000040 */
#define TSC_IOASCR_G2_IO3        TSC_IOASCR_G2_IO3_Msk                         /*!<GROUP2_IO3 analog switch enable */
#define TSC_IOASCR_G2_IO4_Pos    (7UL)
#define TSC_IOASCR_G2_IO4_Msk    (0x1UL << TSC_IOASCR_G2_IO4_Pos)              /*!< 0x00000080 */
#define TSC_IOASCR_G2_IO4        TSC_IOASCR_G2_IO4_Msk                         /*!<GROUP2_IO4 analog switch enable */
#define TSC_IOASCR_G3_IO1_Pos    (8UL)
#define TSC_IOASCR_G3_IO1_Msk    (0x1UL << TSC_IOASCR_G3_IO1_Pos)              /*!< 0x00000100 */
#define TSC_IOASCR_G3_IO1        TSC_IOASCR_G3_IO1_Msk                         /*!<GROUP3_IO1 analog switch enable */
#define TSC_IOASCR_G3_IO2_Pos    (9UL)
#define TSC_IOASCR_G3_IO2_Msk    (0x1UL << TSC_IOASCR_G3_IO2_Pos)              /*!< 0x00000200 */
#define TSC_IOASCR_G3_IO2        TSC_IOASCR_G3_IO2_Msk                         /*!<GROUP3_IO2 analog switch enable */
#define TSC_IOASCR_G3_IO3_Pos    (10UL)
#define TSC_IOASCR_G3_IO3_Msk    (0x1UL << TSC_IOASCR_G3_IO3_Pos)              /*!< 0x00000400 */
#define TSC_IOASCR_G3_IO3        TSC_IOASCR_G3_IO3_Msk                         /*!<GROUP3_IO3 analog switch enable */
#define TSC_IOASCR_G3_IO4_Pos    (11UL)
#define TSC_IOASCR_G3_IO4_Msk    (0x1UL << TSC_IOASCR_G3_IO4_Pos)              /*!< 0x00000800 */
#define TSC_IOASCR_G3_IO4        TSC_IOASCR_G3_IO4_Msk                         /*!<GROUP3_IO4 analog switch enable */
#define TSC_IOASCR_G4_IO1_Pos    (12UL)
#define TSC_IOASCR_G4_IO1_Msk    (0x1UL << TSC_IOASCR_G4_IO1_Pos)              /*!< 0x00001000 */
#define TSC_IOASCR_G4_IO1        TSC_IOASCR_G4_IO1_Msk                         /*!<GROUP4_IO1 analog switch enable */
#define TSC_IOASCR_G4_IO2_Pos    (13UL)
#define TSC_IOASCR_G4_IO2_Msk    (0x1UL << TSC_IOASCR_G4_IO2_Pos)              /*!< 0x00002000 */
#define TSC_IOASCR_G4_IO2        TSC_IOASCR_G4_IO2_Msk                         /*!<GROUP4_IO2 analog switch enable */
#define TSC_IOASCR_G4_IO3_Pos    (14UL)
#define TSC_IOASCR_G4_IO3_Msk    (0x1UL << TSC_IOASCR_G4_IO3_Pos)              /*!< 0x00004000 */
#define TSC_IOASCR_G4_IO3        TSC_IOASCR_G4_IO3_Msk                         /*!<GROUP4_IO3 analog switch enable */
#define TSC_IOASCR_G4_IO4_Pos    (15UL)
#define TSC_IOASCR_G4_IO4_Msk    (0x1UL << TSC_IOASCR_G4_IO4_Pos)              /*!< 0x00008000 */
#define TSC_IOASCR_G4_IO4        TSC_IOASCR_G4_IO4_Msk                         /*!<GROUP4_IO4 analog switch enable */
#define TSC_IOASCR_G5_IO1_Pos    (16UL)
#define TSC_IOASCR_G5_IO1_Msk    (0x1UL << TSC_IOASCR_G5_IO1_Pos)              /*!< 0x00010000 */
#define TSC_IOASCR_G5_IO1        TSC_IOASCR_G5_IO1_Msk                         /*!<GROUP5_IO1 analog switch enable */
#define TSC_IOASCR_G5_IO2_Pos    (17UL)
#define TSC_IOASCR_G5_IO2_Msk    (0x1UL << TSC_IOASCR_G5_IO2_Pos)              /*!< 0x00020000 */
#define TSC_IOASCR_G5_IO2        TSC_IOASCR_G5_IO2_Msk                         /*!<GROUP5_IO2 analog switch enable */
#define TSC_IOASCR_G5_IO3_Pos    (18UL)
#define TSC_IOASCR_G5_IO3_Msk    (0x1UL << TSC_IOASCR_G5_IO3_Pos)              /*!< 0x00040000 */
#define TSC_IOASCR_G5_IO3        TSC_IOASCR_G5_IO3_Msk                         /*!<GROUP5_IO3 analog switch enable */
#define TSC_IOASCR_G5_IO4_Pos    (19UL)
#define TSC_IOASCR_G5_IO4_Msk    (0x1UL << TSC_IOASCR_G5_IO4_Pos)              /*!< 0x00080000 */
#define TSC_IOASCR_G5_IO4        TSC_IOASCR_G5_IO4_Msk                         /*!<GROUP5_IO4 analog switch enable */
#define TSC_IOASCR_G6_IO1_Pos    (20UL)
#define TSC_IOASCR_G6_IO1_Msk    (0x1UL << TSC_IOASCR_G6_IO1_Pos)              /*!< 0x00100000 */
#define TSC_IOASCR_G6_IO1        TSC_IOASCR_G6_IO1_Msk                         /*!<GROUP6_IO1 analog switch enable */
#define TSC_IOASCR_G6_IO2_Pos    (21UL)
#define TSC_IOASCR_G6_IO2_Msk    (0x1UL << TSC_IOASCR_G6_IO2_Pos)              /*!< 0x00200000 */
#define TSC_IOASCR_G6_IO2        TSC_IOASCR_G6_IO2_Msk                         /*!<GROUP6_IO2 analog switch enable */
#define TSC_IOASCR_G6_IO3_Pos    (22UL)
#define TSC_IOASCR_G6_IO3_Msk    (0x1UL << TSC_IOASCR_G6_IO3_Pos)              /*!< 0x00400000 */
#define TSC_IOASCR_G6_IO3        TSC_IOASCR_G6_IO3_Msk                         /*!<GROUP6_IO3 analog switch enable */
#define TSC_IOASCR_G6_IO4_Pos    (23UL)
#define TSC_IOASCR_G6_IO4_Msk    (0x1UL << TSC_IOASCR_G6_IO4_Pos)              /*!< 0x00800000 */
#define TSC_IOASCR_G6_IO4        TSC_IOASCR_G6_IO4_Msk                         /*!<GROUP6_IO4 analog switch enable */
#define TSC_IOASCR_G7_IO1_Pos    (24UL)
#define TSC_IOASCR_G7_IO1_Msk    (0x1UL << TSC_IOASCR_G7_IO1_Pos)              /*!< 0x01000000 */
#define TSC_IOASCR_G7_IO1        TSC_IOASCR_G7_IO1_Msk                         /*!<GROUP7_IO1 analog switch enable */
#define TSC_IOASCR_G7_IO2_Pos    (25UL)
#define TSC_IOASCR_G7_IO2_Msk    (0x1UL << TSC_IOASCR_G7_IO2_Pos)              /*!< 0x02000000 */
#define TSC_IOASCR_G7_IO2        TSC_IOASCR_G7_IO2_Msk                         /*!<GROUP7_IO2 analog switch enable */
#define TSC_IOASCR_G7_IO3_Pos    (26UL)
#define TSC_IOASCR_G7_IO3_Msk    (0x1UL << TSC_IOASCR_G7_IO3_Pos)              /*!< 0x04000000 */
#define TSC_IOASCR_G7_IO3        TSC_IOASCR_G7_IO3_Msk                         /*!<GROUP7_IO3 analog switch enable */
#define TSC_IOASCR_G7_IO4_Pos    (27UL)
#define TSC_IOASCR_G7_IO4_Msk    (0x1UL << TSC_IOASCR_G7_IO4_Pos)              /*!< 0x08000000 */
#define TSC_IOASCR_G7_IO4        TSC_IOASCR_G7_IO4_Msk                         /*!<GROUP7_IO4 analog switch enable */

/*******************  Bit definition for TSC_IOSCR register  ******************/
#define TSC_IOSCR_G1_IO1_Pos     (0UL)
#define TSC_IOSCR_G1_IO1_Msk     (0x1UL << TSC_IOSCR_G1_IO1_Pos)               /*!< 0x00000001 */
#define TSC_IOSCR_G1_IO1         TSC_IOSCR_G1_IO1_Msk                          /*!<GROUP1_IO1 sampling mode */
#define TSC_IOSCR_G1_IO2_Pos     (1UL)
#define TSC_IOSCR_G1_IO2_Msk     (0x1UL << TSC_IOSCR_G1_IO2_Pos)               /*!< 0x00000002 */
#define TSC_IOSCR_G1_IO2         TSC_IOSCR_G1_IO2_Msk                          /*!<GROUP1_IO2 sampling mode */
#define TSC_IOSCR_G1_IO3_Pos     (2UL)
#define TSC_IOSCR_G1_IO3_Msk     (0x1UL << TSC_IOSCR_G1_IO3_Pos)               /*!< 0x00000004 */
#define TSC_IOSCR_G1_IO3         TSC_IOSCR_G1_IO3_Msk                          /*!<GROUP1_IO3 sampling mode */
#define TSC_IOSCR_G1_IO4_Pos     (3UL)
#define TSC_IOSCR_G1_IO4_Msk     (0x1UL << TSC_IOSCR_G1_IO4_Pos)               /*!< 0x00000008 */
#define TSC_IOSCR_G1_IO4         TSC_IOSCR_G1_IO4_Msk                          /*!<GROUP1_IO4 sampling mode */
#define TSC_IOSCR_G2_IO1_Pos     (4UL)
#define TSC_IOSCR_G2_IO1_Msk     (0x1UL << TSC_IOSCR_G2_IO1_Pos)               /*!< 0x00000010 */
#define TSC_IOSCR_G2_IO1         TSC_IOSCR_G2_IO1_Msk                          /*!<GROUP2_IO1 sampling mode */
#define TSC_IOSCR_G2_IO2_Pos     (5UL)
#define TSC_IOSCR_G2_IO2_Msk     (0x1UL << TSC_IOSCR_G2_IO2_Pos)               /*!< 0x00000020 */
#define TSC_IOSCR_G2_IO2         TSC_IOSCR_G2_IO2_Msk                          /*!<GROUP2_IO2 sampling mode */
#define TSC_IOSCR_G2_IO3_Pos     (6UL)
#define TSC_IOSCR_G2_IO3_Msk     (0x1UL << TSC_IOSCR_G2_IO3_Pos)               /*!< 0x00000040 */
#define TSC_IOSCR_G2_IO3         TSC_IOSCR_G2_IO3_Msk                          /*!<GROUP2_IO3 sampling mode */
#define TSC_IOSCR_G2_IO4_Pos     (7UL)
#define TSC_IOSCR_G2_IO4_Msk     (0x1UL << TSC_IOSCR_G2_IO4_Pos)               /*!< 0x00000080 */
#define TSC_IOSCR_G2_IO4         TSC_IOSCR_G2_IO4_Msk                          /*!<GROUP2_IO4 sampling mode */
#define TSC_IOSCR_G3_IO1_Pos     (8UL)
#define TSC_IOSCR_G3_IO1_Msk     (0x1UL << TSC_IOSCR_G3_IO1_Pos)               /*!< 0x00000100 */
#define TSC_IOSCR_G3_IO1         TSC_IOSCR_G3_IO1_Msk                          /*!<GROUP3_IO1 sampling mode */
#define TSC_IOSCR_G3_IO2_Pos     (9UL)
#define TSC_IOSCR_G3_IO2_Msk     (0x1UL << TSC_IOSCR_G3_IO2_Pos)               /*!< 0x00000200 */
#define TSC_IOSCR_G3_IO2         TSC_IOSCR_G3_IO2_Msk                          /*!<GROUP3_IO2 sampling mode */
#define TSC_IOSCR_G3_IO3_Pos     (10UL)
#define TSC_IOSCR_G3_IO3_Msk     (0x1UL << TSC_IOSCR_G3_IO3_Pos)               /*!< 0x00000400 */
#define TSC_IOSCR_G3_IO3         TSC_IOSCR_G3_IO3_Msk                          /*!<GROUP3_IO3 sampling mode */
#define TSC_IOSCR_G3_IO4_Pos     (11UL)
#define TSC_IOSCR_G3_IO4_Msk     (0x1UL << TSC_IOSCR_G3_IO4_Pos)               /*!< 0x00000800 */
#define TSC_IOSCR_G3_IO4         TSC_IOSCR_G3_IO4_Msk                          /*!<GROUP3_IO4 sampling mode */
#define TSC_IOSCR_G4_IO1_Pos     (12UL)
#define TSC_IOSCR_G4_IO1_Msk     (0x1UL << TSC_IOSCR_G4_IO1_Pos)               /*!< 0x00001000 */
#define TSC_IOSCR_G4_IO1         TSC_IOSCR_G4_IO1_Msk                          /*!<GROUP4_IO1 sampling mode */
#define TSC_IOSCR_G4_IO2_Pos     (13UL)
#define TSC_IOSCR_G4_IO2_Msk     (0x1UL << TSC_IOSCR_G4_IO2_Pos)               /*!< 0x00002000 */
#define TSC_IOSCR_G4_IO2         TSC_IOSCR_G4_IO2_Msk                          /*!<GROUP4_IO2 sampling mode */
#define TSC_IOSCR_G4_IO3_Pos     (14UL)
#define TSC_IOSCR_G4_IO3_Msk     (0x1UL << TSC_IOSCR_G4_IO3_Pos)               /*!< 0x00004000 */
#define TSC_IOSCR_G4_IO3         TSC_IOSCR_G4_IO3_Msk                          /*!<GROUP4_IO3 sampling mode */
#define TSC_IOSCR_G4_IO4_Pos     (15UL)
#define TSC_IOSCR_G4_IO4_Msk     (0x1UL << TSC_IOSCR_G4_IO4_Pos)               /*!< 0x00008000 */
#define TSC_IOSCR_G4_IO4         TSC_IOSCR_G4_IO4_Msk                          /*!<GROUP4_IO4 sampling mode */
#define TSC_IOSCR_G5_IO1_Pos     (16UL)
#define TSC_IOSCR_G5_IO1_Msk     (0x1UL << TSC_IOSCR_G5_IO1_Pos)               /*!< 0x00010000 */
#define TSC_IOSCR_G5_IO1         TSC_IOSCR_G5_IO1_Msk                          /*!<GROUP5_IO1 sampling mode */
#define TSC_IOSCR_G5_IO2_Pos     (17UL)
#define TSC_IOSCR_G5_IO2_Msk     (0x1UL << TSC_IOSCR_G5_IO2_Pos)               /*!< 0x00020000 */
#define TSC_IOSCR_G5_IO2         TSC_IOSCR_G5_IO2_Msk                          /*!<GROUP5_IO2 sampling mode */
#define TSC_IOSCR_G5_IO3_Pos     (18UL)
#define TSC_IOSCR_G5_IO3_Msk     (0x1UL << TSC_IOSCR_G5_IO3_Pos)               /*!< 0x00040000 */
#define TSC_IOSCR_G5_IO3         TSC_IOSCR_G5_IO3_Msk                          /*!<GROUP5_IO3 sampling mode */
#define TSC_IOSCR_G5_IO4_Pos     (19UL)
#define TSC_IOSCR_G5_IO4_Msk     (0x1UL << TSC_IOSCR_G5_IO4_Pos)               /*!< 0x00080000 */
#define TSC_IOSCR_G5_IO4         TSC_IOSCR_G5_IO4_Msk                          /*!<GROUP5_IO4 sampling mode */
#define TSC_IOSCR_G6_IO1_Pos     (20UL)
#define TSC_IOSCR_G6_IO1_Msk     (0x1UL << TSC_IOSCR_G6_IO1_Pos)               /*!< 0x00100000 */
#define TSC_IOSCR_G6_IO1         TSC_IOSCR_G6_IO1_Msk                          /*!<GROUP6_IO1 sampling mode */
#define TSC_IOSCR_G6_IO2_Pos     (21UL)
#define TSC_IOSCR_G6_IO2_Msk     (0x1UL << TSC_IOSCR_G6_IO2_Pos)               /*!< 0x00200000 */
#define TSC_IOSCR_G6_IO2         TSC_IOSCR_G6_IO2_Msk                          /*!<GROUP6_IO2 sampling mode */
#define TSC_IOSCR_G6_IO3_Pos     (22UL)
#define TSC_IOSCR_G6_IO3_Msk     (0x1UL << TSC_IOSCR_G6_IO3_Pos)               /*!< 0x00400000 */
#define TSC_IOSCR_G6_IO3         TSC_IOSCR_G6_IO3_Msk                          /*!<GROUP6_IO3 sampling mode */
#define TSC_IOSCR_G6_IO4_Pos     (23UL)
#define TSC_IOSCR_G6_IO4_Msk     (0x1UL << TSC_IOSCR_G6_IO4_Pos)               /*!< 0x00800000 */
#define TSC_IOSCR_G6_IO4         TSC_IOSCR_G6_IO4_Msk                          /*!<GROUP6_IO4 sampling mode */
#define TSC_IOSCR_G7_IO1_Pos     (24UL)
#define TSC_IOSCR_G7_IO1_Msk     (0x1UL << TSC_IOSCR_G7_IO1_Pos)               /*!< 0x01000000 */
#define TSC_IOSCR_G7_IO1         TSC_IOSCR_G7_IO1_Msk                          /*!<GROUP7_IO1 sampling mode */
#define TSC_IOSCR_G7_IO2_Pos     (25UL)
#define TSC_IOSCR_G7_IO2_Msk     (0x1UL << TSC_IOSCR_G7_IO2_Pos)               /*!< 0x02000000 */
#define TSC_IOSCR_G7_IO2         TSC_IOSCR_G7_IO2_Msk                          /*!<GROUP7_IO2 sampling mode */
#define TSC_IOSCR_G7_IO3_Pos     (26UL)
#define TSC_IOSCR_G7_IO3_Msk     (0x1UL << TSC_IOSCR_G7_IO3_Pos)               /*!< 0x04000000 */
#define TSC_IOSCR_G7_IO3         TSC_IOSCR_G7_IO3_Msk                          /*!<GROUP7_IO3 sampling mode */
#define TSC_IOSCR_G7_IO4_Pos     (27UL)
#define TSC_IOSCR_G7_IO4_Msk     (0x1UL << TSC_IOSCR_G7_IO4_Pos)               /*!< 0x08000000 */
#define TSC_IOSCR_G7_IO4         TSC_IOSCR_G7_IO4_Msk                          /*!<GROUP7_IO4 sampling mode */

/*******************  Bit definition for TSC_IOCCR register  ******************/
#define TSC_IOCCR_G1_IO1_Pos     (0UL)
#define TSC_IOCCR_G1_IO1_Msk     (0x1UL << TSC_IOCCR_G1_IO1_Pos)               /*!< 0x00000001 */
#define TSC_IOCCR_G1_IO1         TSC_IOCCR_G1_IO1_Msk                          /*!<GROUP1_IO1 channel mode */
#define TSC_IOCCR_G1_IO2_Pos     (1UL)
#define TSC_IOCCR_G1_IO2_Msk     (0x1UL << TSC_IOCCR_G1_IO2_Pos)               /*!< 0x00000002 */
#define TSC_IOCCR_G1_IO2         TSC_IOCCR_G1_IO2_Msk                          /*!<GROUP1_IO2 channel mode */
#define TSC_IOCCR_G1_IO3_Pos     (2UL)
#define TSC_IOCCR_G1_IO3_Msk     (0x1UL << TSC_IOCCR_G1_IO3_Pos)               /*!< 0x00000004 */
#define TSC_IOCCR_G1_IO3         TSC_IOCCR_G1_IO3_Msk                          /*!<GROUP1_IO3 channel mode */
#define TSC_IOCCR_G1_IO4_Pos     (3UL)
#define TSC_IOCCR_G1_IO4_Msk     (0x1UL << TSC_IOCCR_G1_IO4_Pos)               /*!< 0x00000008 */
#define TSC_IOCCR_G1_IO4         TSC_IOCCR_G1_IO4_Msk                          /*!<GROUP1_IO4 channel mode */
#define TSC_IOCCR_G2_IO1_Pos     (4UL)
#define TSC_IOCCR_G2_IO1_Msk     (0x1UL << TSC_IOCCR_G2_IO1_Pos)               /*!< 0x00000010 */
#define TSC_IOCCR_G2_IO1         TSC_IOCCR_G2_IO1_Msk                          /*!<GROUP2_IO1 channel mode */
#define TSC_IOCCR_G2_IO2_Pos     (5UL)
#define TSC_IOCCR_G2_IO2_Msk     (0x1UL << TSC_IOCCR_G2_IO2_Pos)               /*!< 0x00000020 */
#define TSC_IOCCR_G2_IO2         TSC_IOCCR_G2_IO2_Msk                          /*!<GROUP2_IO2 channel mode */
#define TSC_IOCCR_G2_IO3_Pos     (6UL)
#define TSC_IOCCR_G2_IO3_Msk     (0x1UL << TSC_IOCCR_G2_IO3_Pos)               /*!< 0x00000040 */
#define TSC_IOCCR_G2_IO3         TSC_IOCCR_G2_IO3_Msk                          /*!<GROUP2_IO3 channel mode */
#define TSC_IOCCR_G2_IO4_Pos     (7UL)
#define TSC_IOCCR_G2_IO4_Msk     (0x1UL << TSC_IOCCR_G2_IO4_Pos)               /*!< 0x00000080 */
#define TSC_IOCCR_G2_IO4         TSC_IOCCR_G2_IO4_Msk                          /*!<GROUP2_IO4 channel mode */
#define TSC_IOCCR_G3_IO1_Pos     (8UL)
#define TSC_IOCCR_G3_IO1_Msk     (0x1UL << TSC_IOCCR_G3_IO1_Pos)               /*!< 0x00000100 */
#define TSC_IOCCR_G3_IO1         TSC_IOCCR_G3_IO1_Msk                          /*!<GROUP3_IO1 channel mode */
#define TSC_IOCCR_G3_IO2_Pos     (9UL)
#define TSC_IOCCR_G3_IO2_Msk     (0x1UL << TSC_IOCCR_G3_IO2_Pos)               /*!< 0x00000200 */
#define TSC_IOCCR_G3_IO2         TSC_IOCCR_G3_IO2_Msk                          /*!<GROUP3_IO2 channel mode */
#define TSC_IOCCR_G3_IO3_Pos     (10UL)
#define TSC_IOCCR_G3_IO3_Msk     (0x1UL << TSC_IOCCR_G3_IO3_Pos)               /*!< 0x00000400 */
#define TSC_IOCCR_G3_IO3         TSC_IOCCR_G3_IO3_Msk                          /*!<GROUP3_IO3 channel mode */
#define TSC_IOCCR_G3_IO4_Pos     (11UL)
#define TSC_IOCCR_G3_IO4_Msk     (0x1UL << TSC_IOCCR_G3_IO4_Pos)               /*!< 0x00000800 */
#define TSC_IOCCR_G3_IO4         TSC_IOCCR_G3_IO4_Msk                          /*!<GROUP3_IO4 channel mode */
#define TSC_IOCCR_G4_IO1_Pos     (12UL)
#define TSC_IOCCR_G4_IO1_Msk     (0x1UL << TSC_IOCCR_G4_IO1_Pos)               /*!< 0x00001000 */
#define TSC_IOCCR_G4_IO1         TSC_IOCCR_G4_IO1_Msk                          /*!<GROUP4_IO1 channel mode */
#define TSC_IOCCR_G4_IO2_Pos     (13UL)
#define TSC_IOCCR_G4_IO2_Msk     (0x1UL << TSC_IOCCR_G4_IO2_Pos)               /*!< 0x00002000 */
#define TSC_IOCCR_G4_IO2         TSC_IOCCR_G4_IO2_Msk                          /*!<GROUP4_IO2 channel mode */
#define TSC_IOCCR_G4_IO3_Pos     (14UL)
#define TSC_IOCCR_G4_IO3_Msk     (0x1UL << TSC_IOCCR_G4_IO3_Pos)               /*!< 0x00004000 */
#define TSC_IOCCR_G4_IO3         TSC_IOCCR_G4_IO3_Msk                          /*!<GROUP4_IO3 channel mode */
#define TSC_IOCCR_G4_IO4_Pos     (15UL)
#define TSC_IOCCR_G4_IO4_Msk     (0x1UL << TSC_IOCCR_G4_IO4_Pos)               /*!< 0x00008000 */
#define TSC_IOCCR_G4_IO4         TSC_IOCCR_G4_IO4_Msk                          /*!<GROUP4_IO4 channel mode */
#define TSC_IOCCR_G5_IO1_Pos     (16UL)
#define TSC_IOCCR_G5_IO1_Msk     (0x1UL << TSC_IOCCR_G5_IO1_Pos)               /*!< 0x00010000 */
#define TSC_IOCCR_G5_IO1         TSC_IOCCR_G5_IO1_Msk                          /*!<GROUP5_IO1 channel mode */
#define TSC_IOCCR_G5_IO2_Pos     (17UL)
#define TSC_IOCCR_G5_IO2_Msk     (0x1UL << TSC_IOCCR_G5_IO2_Pos)               /*!< 0x00020000 */
#define TSC_IOCCR_G5_IO2         TSC_IOCCR_G5_IO2_Msk                          /*!<GROUP5_IO2 channel mode */
#define TSC_IOCCR_G5_IO3_Pos     (18UL)
#define TSC_IOCCR_G5_IO3_Msk     (0x1UL << TSC_IOCCR_G5_IO3_Pos)               /*!< 0x00040000 */
#define TSC_IOCCR_G5_IO3         TSC_IOCCR_G5_IO3_Msk                          /*!<GROUP5_IO3 channel mode */
#define TSC_IOCCR_G5_IO4_Pos     (19UL)
#define TSC_IOCCR_G5_IO4_Msk     (0x1UL << TSC_IOCCR_G5_IO4_Pos)               /*!< 0x00080000 */
#define TSC_IOCCR_G5_IO4         TSC_IOCCR_G5_IO4_Msk                          /*!<GROUP5_IO4 channel mode */
#define TSC_IOCCR_G6_IO1_Pos     (20UL)
#define TSC_IOCCR_G6_IO1_Msk     (0x1UL << TSC_IOCCR_G6_IO1_Pos)               /*!< 0x00100000 */
#define TSC_IOCCR_G6_IO1         TSC_IOCCR_G6_IO1_Msk                          /*!<GROUP6_IO1 channel mode */
#define TSC_IOCCR_G6_IO2_Pos     (21UL)
#define TSC_IOCCR_G6_IO2_Msk     (0x1UL << TSC_IOCCR_G6_IO2_Pos)               /*!< 0x00200000 */
#define TSC_IOCCR_G6_IO2         TSC_IOCCR_G6_IO2_Msk                          /*!<GROUP6_IO2 channel mode */
#define TSC_IOCCR_G6_IO3_Pos     (22UL)
#define TSC_IOCCR_G6_IO3_Msk     (0x1UL << TSC_IOCCR_G6_IO3_Pos)               /*!< 0x00400000 */
#define TSC_IOCCR_G6_IO3         TSC_IOCCR_G6_IO3_Msk                          /*!<GROUP6_IO3 channel mode */
#define TSC_IOCCR_G6_IO4_Pos     (23UL)
#define TSC_IOCCR_G6_IO4_Msk     (0x1UL << TSC_IOCCR_G6_IO4_Pos)               /*!< 0x00800000 */
#define TSC_IOCCR_G6_IO4         TSC_IOCCR_G6_IO4_Msk                          /*!<GROUP6_IO4 channel mode */
#define TSC_IOCCR_G7_IO1_Pos     (24UL)
#define TSC_IOCCR_G7_IO1_Msk     (0x1UL << TSC_IOCCR_G7_IO1_Pos)               /*!< 0x01000000 */
#define TSC_IOCCR_G7_IO1         TSC_IOCCR_G7_IO1_Msk                          /*!<GROUP7_IO1 channel mode */
#define TSC_IOCCR_G7_IO2_Pos     (25UL)
#define TSC_IOCCR_G7_IO2_Msk     (0x1UL << TSC_IOCCR_G7_IO2_Pos)               /*!< 0x02000000 */
#define TSC_IOCCR_G7_IO2         TSC_IOCCR_G7_IO2_Msk                          /*!<GROUP7_IO2 channel mode */
#define TSC_IOCCR_G7_IO3_Pos     (26UL)
#define TSC_IOCCR_G7_IO3_Msk     (0x1UL << TSC_IOCCR_G7_IO3_Pos)               /*!< 0x04000000 */
#define TSC_IOCCR_G7_IO3         TSC_IOCCR_G7_IO3_Msk                          /*!<GROUP7_IO3 channel mode */
#define TSC_IOCCR_G7_IO4_Pos     (27UL)
#define TSC_IOCCR_G7_IO4_Msk     (0x1UL << TSC_IOCCR_G7_IO4_Pos)               /*!< 0x08000000 */
#define TSC_IOCCR_G7_IO4         TSC_IOCCR_G7_IO4_Msk                          /*!<GROUP7_IO4 channel mode */

/*******************  Bit definition for TSC_IOGCSR register  *****************/
#define TSC_IOGCSR_G1E_Pos       (0UL)
#define TSC_IOGCSR_G1E_Msk       (0x1UL << TSC_IOGCSR_G1E_Pos)                 /*!< 0x00000001 */
#define TSC_IOGCSR_G1E           TSC_IOGCSR_G1E_Msk                            /*!<Analog IO GROUP1 enable */
#define TSC_IOGCSR_G2E_Pos       (1UL)
#define TSC_IOGCSR_G2E_Msk       (0x1UL << TSC_IOGCSR_G2E_Pos)                 /*!< 0x00000002 */
#define TSC_IOGCSR_G2E           TSC_IOGCSR_G2E_Msk                            /*!<Analog IO GROUP2 enable */
#define TSC_IOGCSR_G3E_Pos       (2UL)
#define TSC_IOGCSR_G3E_Msk       (0x1UL << TSC_IOGCSR_G3E_Pos)                 /*!< 0x00000004 */
#define TSC_IOGCSR_G3E           TSC_IOGCSR_G3E_Msk                            /*!<Analog IO GROUP3 enable */
#define TSC_IOGCSR_G4E_Pos       (3UL)
#define TSC_IOGCSR_G4E_Msk       (0x1UL << TSC_IOGCSR_G4E_Pos)                 /*!< 0x00000008 */
#define TSC_IOGCSR_G4E           TSC_IOGCSR_G4E_Msk                            /*!<Analog IO GROUP4 enable */
#define TSC_IOGCSR_G5E_Pos       (4UL)
#define TSC_IOGCSR_G5E_Msk       (0x1UL << TSC_IOGCSR_G5E_Pos)                 /*!< 0x00000010 */
#define TSC_IOGCSR_G5E           TSC_IOGCSR_G5E_Msk                            /*!<Analog IO GROUP5 enable */
#define TSC_IOGCSR_G6E_Pos       (5UL)
#define TSC_IOGCSR_G6E_Msk       (0x1UL << TSC_IOGCSR_G6E_Pos)                 /*!< 0x00000020 */
#define TSC_IOGCSR_G6E           TSC_IOGCSR_G6E_Msk                            /*!<Analog IO GROUP6 enable */
#define TSC_IOGCSR_G7E_Pos       (6UL)
#define TSC_IOGCSR_G7E_Msk       (0x1UL << TSC_IOGCSR_G7E_Pos)                 /*!< 0x00000040 */
#define TSC_IOGCSR_G7E           TSC_IOGCSR_G7E_Msk                            /*!<Analog IO GROUP7 enable */
#define TSC_IOGCSR_G1S_Pos       (16UL)
#define TSC_IOGCSR_G1S_Msk       (0x1UL << TSC_IOGCSR_G1S_Pos)                 /*!< 0x00010000 */
#define TSC_IOGCSR_G1S           TSC_IOGCSR_G1S_Msk                            /*!<Analog IO GROUP1 status */
#define TSC_IOGCSR_G2S_Pos       (17UL)
#define TSC_IOGCSR_G2S_Msk       (0x1UL << TSC_IOGCSR_G2S_Pos)                 /*!< 0x00020000 */
#define TSC_IOGCSR_G2S           TSC_IOGCSR_G2S_Msk                            /*!<Analog IO GROUP2 status */
#define TSC_IOGCSR_G3S_Pos       (18UL)
#define TSC_IOGCSR_G3S_Msk       (0x1UL << TSC_IOGCSR_G3S_Pos)                 /*!< 0x00040000 */
#define TSC_IOGCSR_G3S           TSC_IOGCSR_G3S_Msk                            /*!<Analog IO GROUP3 status */
#define TSC_IOGCSR_G4S_Pos       (19UL)
#define TSC_IOGCSR_G4S_Msk       (0x1UL << TSC_IOGCSR_G4S_Pos)                 /*!< 0x00080000 */
#define TSC_IOGCSR_G4S           TSC_IOGCSR_G4S_Msk                            /*!<Analog IO GROUP4 status */
#define TSC_IOGCSR_G5S_Pos       (20UL)
#define TSC_IOGCSR_G5S_Msk       (0x1UL << TSC_IOGCSR_G5S_Pos)                 /*!< 0x00100000 */
#define TSC_IOGCSR_G5S           TSC_IOGCSR_G5S_Msk                            /*!<Analog IO GROUP5 status */
#define TSC_IOGCSR_G6S_Pos       (21UL)
#define TSC_IOGCSR_G6S_Msk       (0x1UL << TSC_IOGCSR_G6S_Pos)                 /*!< 0x00200000 */
#define TSC_IOGCSR_G6S           TSC_IOGCSR_G6S_Msk                            /*!<Analog IO GROUP6 status */
#define TSC_IOGCSR_G7S_Pos       (22UL)
#define TSC_IOGCSR_G7S_Msk       (0x1UL << TSC_IOGCSR_G7S_Pos)                 /*!< 0x00400000 */
#define TSC_IOGCSR_G7S           TSC_IOGCSR_G7S_Msk                            /*!<Analog IO GROUP7 status */

/*******************  Bit definition for TSC_IOGXCR register  *****************/
#define TSC_IOGXCR_CNT_Pos       (0UL)
#define TSC_IOGXCR_CNT_Msk       (0x3FFFUL << TSC_IOGXCR_CNT_Pos)              /*!< 0x00003FFF */
#define TSC_IOGXCR_CNT           TSC_IOGXCR_CNT_Msk                            /*!<CNT[13:0] bits (Counter value) */

/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter (USART)       */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_UE_Pos                (0UL)
#define USART_CR1_UE_Msk                (0x1UL << USART_CR1_UE_Pos)            /*!< 0x00000001 */
#define USART_CR1_UE                    USART_CR1_UE_Msk                       /*!< USART Enable */
#define USART_CR1_UESM_Pos              (1UL)
#define USART_CR1_UESM_Msk              (0x1UL << USART_CR1_UESM_Pos)          /*!< 0x00000002 */
#define USART_CR1_UESM                  USART_CR1_UESM_Msk                     /*!< USART Enable in STOP Mode */
#define USART_CR1_RE_Pos                (2UL)
#define USART_CR1_RE_Msk                (0x1UL << USART_CR1_RE_Pos)            /*!< 0x00000004 */
#define USART_CR1_RE                    USART_CR1_RE_Msk                       /*!< Receiver Enable */
#define USART_CR1_TE_Pos                (3UL)
#define USART_CR1_TE_Msk                (0x1UL << USART_CR1_TE_Pos)            /*!< 0x00000008 */
#define USART_CR1_TE                    USART_CR1_TE_Msk                       /*!< Transmitter Enable */
#define USART_CR1_IDLEIE_Pos            (4UL)
#define USART_CR1_IDLEIE_Msk            (0x1UL << USART_CR1_IDLEIE_Pos)        /*!< 0x00000010 */
#define USART_CR1_IDLEIE                USART_CR1_IDLEIE_Msk                   /*!< IDLE Interrupt Enable */
#define USART_CR1_RXNEIE_RXFNEIE_Pos    (5UL)
#define USART_CR1_RXNEIE_RXFNEIE_Msk    (0x1UL << USART_CR1_RXNEIE_RXFNEIE_Pos) /*!< 0x00000020 */
#define USART_CR1_RXNEIE_RXFNEIE        USART_CR1_RXNEIE_RXFNEIE_Msk           /*!< RXNE and RX FIFO Not Empty Interrupt Enable */
#define USART_CR1_TCIE_Pos              (6UL)
#define USART_CR1_TCIE_Msk              (0x1UL << USART_CR1_TCIE_Pos)          /*!< 0x00000040 */
#define USART_CR1_TCIE                  USART_CR1_TCIE_Msk                     /*!< Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE_TXFNFIE_Pos     (7UL)
#define USART_CR1_TXEIE_TXFNFIE_Msk     (0x1UL << USART_CR1_TXEIE_TXFNFIE_Pos) /*!< 0x00000080 */
#define USART_CR1_TXEIE_TXFNFIE         USART_CR1_TXEIE_TXFNFIE_Msk            /*!< TXE and TX FIFO Not Full Interrupt Enable */
#define USART_CR1_PEIE_Pos              (8UL)
#define USART_CR1_PEIE_Msk              (0x1UL << USART_CR1_PEIE_Pos)          /*!< 0x00000100 */
#define USART_CR1_PEIE                  USART_CR1_PEIE_Msk                     /*!< PE Interrupt Enable */
#define USART_CR1_PS_Pos                (9UL)
#define USART_CR1_PS_Msk                (0x1UL << USART_CR1_PS_Pos)            /*!< 0x00000200 */
#define USART_CR1_PS                    USART_CR1_PS_Msk                       /*!< Parity Selection */
#define USART_CR1_PCE_Pos               (10UL)
#define USART_CR1_PCE_Msk               (0x1UL << USART_CR1_PCE_Pos)           /*!< 0x00000400 */
#define USART_CR1_PCE                   USART_CR1_PCE_Msk                      /*!< Parity Control Enable */
#define USART_CR1_WAKE_Pos              (11UL)
#define USART_CR1_WAKE_Msk              (0x1UL << USART_CR1_WAKE_Pos)          /*!< 0x00000800 */
#define USART_CR1_WAKE                  USART_CR1_WAKE_Msk                     /*!< Receiver Wakeup method */
#define USART_CR1_M_Pos                 (12UL)
#define USART_CR1_M_Msk                 (0x10001UL << USART_CR1_M_Pos)         /*!< 0x10001000 */
#define USART_CR1_M                     USART_CR1_M_Msk                        /*!< Word length */
#define USART_CR1_M0_Pos                (12UL)
#define USART_CR1_M0_Msk                (0x1UL << USART_CR1_M0_Pos)            /*!< 0x00001000 */
#define USART_CR1_M0                    USART_CR1_M0_Msk                       /*!< Word length - Bit 0 */
#define USART_CR1_MME_Pos               (13UL)
#define USART_CR1_MME_Msk               (0x1UL << USART_CR1_MME_Pos)           /*!< 0x00002000 */
#define USART_CR1_MME                   USART_CR1_MME_Msk                      /*!< Mute Mode Enable */
#define USART_CR1_CMIE_Pos              (14UL)
#define USART_CR1_CMIE_Msk              (0x1UL << USART_CR1_CMIE_Pos)          /*!< 0x00004000 */
#define USART_CR1_CMIE                  USART_CR1_CMIE_Msk                     /*!< Character match interrupt enable */
#define USART_CR1_OVER8_Pos             (15UL)
#define USART_CR1_OVER8_Msk             (0x1UL << USART_CR1_OVER8_Pos)         /*!< 0x00008000 */
#define USART_CR1_OVER8                 USART_CR1_OVER8_Msk                    /*!< Oversampling by 8-bit or 16-bit mode */
#define USART_CR1_DEDT_Pos              (16UL)
#define USART_CR1_DEDT_Msk              (0x1FUL << USART_CR1_DEDT_Pos)         /*!< 0x001F0000 */
#define USART_CR1_DEDT                  USART_CR1_DEDT_Msk                     /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
#define USART_CR1_DEDT_0                (0x01UL << USART_CR1_DEDT_Pos)          /*!< 0x00010000 */
#define USART_CR1_DEDT_1                (0x02UL << USART_CR1_DEDT_Pos)          /*!< 0x00020000 */
#define USART_CR1_DEDT_2                (0x04UL << USART_CR1_DEDT_Pos)          /*!< 0x00040000 */
#define USART_CR1_DEDT_3                (0x08UL << USART_CR1_DEDT_Pos)          /*!< 0x00080000 */
#define USART_CR1_DEDT_4                (0x10UL << USART_CR1_DEDT_Pos)          /*!< 0x00100000 */
#define USART_CR1_DEAT_Pos              (21UL)
#define USART_CR1_DEAT_Msk              (0x1FUL << USART_CR1_DEAT_Pos)         /*!< 0x03E00000 */
#define USART_CR1_DEAT                  USART_CR1_DEAT_Msk                     /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
#define USART_CR1_DEAT_0                (0x01UL << USART_CR1_DEAT_Pos)          /*!< 0x00200000 */
#define USART_CR1_DEAT_1                (0x02UL << USART_CR1_DEAT_Pos)          /*!< 0x00400000 */
#define USART_CR1_DEAT_2                (0x04UL << USART_CR1_DEAT_Pos)          /*!< 0x00800000 */
#define USART_CR1_DEAT_3                (0x08UL << USART_CR1_DEAT_Pos)          /*!< 0x01000000 */
#define USART_CR1_DEAT_4                (0x10UL << USART_CR1_DEAT_Pos)          /*!< 0x02000000 */
#define USART_CR1_RTOIE_Pos             (26UL)
#define USART_CR1_RTOIE_Msk             (0x1UL << USART_CR1_RTOIE_Pos)         /*!< 0x04000000 */
#define USART_CR1_RTOIE                 USART_CR1_RTOIE_Msk                    /*!< Receive Time Out interrupt enable */
#define USART_CR1_EOBIE_Pos             (27UL)
#define USART_CR1_EOBIE_Msk             (0x1UL << USART_CR1_EOBIE_Pos)         /*!< 0x08000000 */
#define USART_CR1_EOBIE                 USART_CR1_EOBIE_Msk                    /*!< End of Block interrupt enable */
#define USART_CR1_M1_Pos                (28UL)
#define USART_CR1_M1_Msk                (0x1UL << USART_CR1_M1_Pos)            /*!< 0x10000000 */
#define USART_CR1_M1                    USART_CR1_M1_Msk                       /*!< Word length - Bit 1 */
#define USART_CR1_FIFOEN_Pos            (29UL)
#define USART_CR1_FIFOEN_Msk            (0x1UL << USART_CR1_FIFOEN_Pos)        /*!< 0x20000000 */
#define USART_CR1_FIFOEN                USART_CR1_FIFOEN_Msk                   /*!< FIFO mode enable */
#define USART_CR1_TXFEIE_Pos            (30UL)
#define USART_CR1_TXFEIE_Msk            (0x1UL << USART_CR1_TXFEIE_Pos)        /*!< 0x40000000 */
#define USART_CR1_TXFEIE                USART_CR1_TXFEIE_Msk                   /*!< TXFIFO empty interrupt enable */
#define USART_CR1_RXFFIE_Pos            (31UL)
#define USART_CR1_RXFFIE_Msk            (0x1UL << USART_CR1_RXFFIE_Pos)        /*!< 0x80000000 */
#define USART_CR1_RXFFIE                USART_CR1_RXFFIE_Msk                   /*!< RXFIFO Full interrupt enable */

/* Legacy define */
#define  USART_CR1_RXNEIE  USART_CR1_RXNEIE_RXFNEIE
#define  USART_CR1_TXEIE   USART_CR1_TXEIE_TXFNFIE

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_SLVEN_Pos             (0UL)
#define USART_CR2_SLVEN_Msk             (0x1UL << USART_CR2_SLVEN_Pos)         /*!< 0x00000001 */
#define USART_CR2_SLVEN                 USART_CR2_SLVEN_Msk                    /*!< Synchronous Slave mode Enable */
#define USART_CR2_DIS_NSS_Pos           (3UL)
#define USART_CR2_DIS_NSS_Msk           (0x1UL << USART_CR2_DIS_NSS_Pos)       /*!< 0x00000008 */
#define USART_CR2_DIS_NSS               USART_CR2_DIS_NSS_Msk                  /*!< Negative Slave Select (NSS) pin management */
#define USART_CR2_ADDM7_Pos             (4UL)
#define USART_CR2_ADDM7_Msk             (0x1UL << USART_CR2_ADDM7_Pos)         /*!< 0x00000010 */
#define USART_CR2_ADDM7                 USART_CR2_ADDM7_Msk                    /*!< 7-bit or 4-bit Address Detection */
#define USART_CR2_LBDL_Pos              (5UL)
#define USART_CR2_LBDL_Msk              (0x1UL << USART_CR2_LBDL_Pos)          /*!< 0x00000020 */
#define USART_CR2_LBDL                  USART_CR2_LBDL_Msk                     /*!< LIN Break Detection Length */
#define USART_CR2_LBDIE_Pos             (6UL)
#define USART_CR2_LBDIE_Msk             (0x1UL << USART_CR2_LBDIE_Pos)         /*!< 0x00000040 */
#define USART_CR2_LBDIE                 USART_CR2_LBDIE_Msk                    /*!< LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL_Pos              (8UL)
#define USART_CR2_LBCL_Msk              (0x1UL << USART_CR2_LBCL_Pos)          /*!< 0x00000100 */
#define USART_CR2_LBCL                  USART_CR2_LBCL_Msk                     /*!< Last Bit Clock pulse */
#define USART_CR2_CPHA_Pos              (9UL)
#define USART_CR2_CPHA_Msk              (0x1UL << USART_CR2_CPHA_Pos)          /*!< 0x00000200 */
#define USART_CR2_CPHA                  USART_CR2_CPHA_Msk                     /*!< Clock Phase */
#define USART_CR2_CPOL_Pos              (10UL)
#define USART_CR2_CPOL_Msk              (0x1UL << USART_CR2_CPOL_Pos)          /*!< 0x00000400 */
#define USART_CR2_CPOL                  USART_CR2_CPOL_Msk                     /*!< Clock Polarity */
#define USART_CR2_CLKEN_Pos             (11UL)
#define USART_CR2_CLKEN_Msk             (0x1UL << USART_CR2_CLKEN_Pos)         /*!< 0x00000800 */
#define USART_CR2_CLKEN                 USART_CR2_CLKEN_Msk                    /*!< Clock Enable */
#define USART_CR2_STOP_Pos              (12UL)
#define USART_CR2_STOP_Msk              (0x3UL << USART_CR2_STOP_Pos)          /*!< 0x00003000 */
#define USART_CR2_STOP                  USART_CR2_STOP_Msk                     /*!< STOP[1:0] bits (STOP bits) */
#define USART_CR2_STOP_0                (0x1UL << USART_CR2_STOP_Pos)           /*!< 0x00001000 */
#define USART_CR2_STOP_1                (0x2UL << USART_CR2_STOP_Pos)           /*!< 0x00002000 */
#define USART_CR2_LINEN_Pos             (14UL)
#define USART_CR2_LINEN_Msk             (0x1UL << USART_CR2_LINEN_Pos)         /*!< 0x00004000 */
#define USART_CR2_LINEN                 USART_CR2_LINEN_Msk                    /*!< LIN mode enable */
#define USART_CR2_SWAP_Pos              (15UL)
#define USART_CR2_SWAP_Msk              (0x1UL << USART_CR2_SWAP_Pos)          /*!< 0x00008000 */
#define USART_CR2_SWAP                  USART_CR2_SWAP_Msk                     /*!< SWAP TX/RX pins */
#define USART_CR2_RXINV_Pos             (16UL)
#define USART_CR2_RXINV_Msk             (0x1UL << USART_CR2_RXINV_Pos)         /*!< 0x00010000 */
#define USART_CR2_RXINV                 USART_CR2_RXINV_Msk                    /*!< RX pin active level inversion */
#define USART_CR2_TXINV_Pos             (17UL)
#define USART_CR2_TXINV_Msk             (0x1UL << USART_CR2_TXINV_Pos)         /*!< 0x00020000 */
#define USART_CR2_TXINV                 USART_CR2_TXINV_Msk                    /*!< TX pin active level inversion */
#define USART_CR2_DATAINV_Pos           (18UL)
#define USART_CR2_DATAINV_Msk           (0x1UL << USART_CR2_DATAINV_Pos)       /*!< 0x00040000 */
#define USART_CR2_DATAINV               USART_CR2_DATAINV_Msk                  /*!< Binary data inversion */
#define USART_CR2_MSBFIRST_Pos          (19UL)
#define USART_CR2_MSBFIRST_Msk          (0x1UL << USART_CR2_MSBFIRST_Pos)      /*!< 0x00080000 */
#define USART_CR2_MSBFIRST              USART_CR2_MSBFIRST_Msk                 /*!< Most Significant Bit First */
#define USART_CR2_ABREN_Pos             (20UL)
#define USART_CR2_ABREN_Msk             (0x1UL << USART_CR2_ABREN_Pos)         /*!< 0x00100000 */
#define USART_CR2_ABREN                 USART_CR2_ABREN_Msk                    /*!< Auto Baud-Rate Enable*/
#define USART_CR2_ABRMODE_Pos           (21UL)
#define USART_CR2_ABRMODE_Msk           (0x3UL << USART_CR2_ABRMODE_Pos)       /*!< 0x00600000 */
#define USART_CR2_ABRMODE               USART_CR2_ABRMODE_Msk                  /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
#define USART_CR2_ABRMODE_0             (0x1UL << USART_CR2_ABRMODE_Pos)        /*!< 0x00200000 */
#define USART_CR2_ABRMODE_1             (0x2UL << USART_CR2_ABRMODE_Pos)        /*!< 0x00400000 */
#define USART_CR2_RTOEN_Pos             (23UL)
#define USART_CR2_RTOEN_Msk             (0x1UL << USART_CR2_RTOEN_Pos)         /*!< 0x00800000 */
#define USART_CR2_RTOEN                 USART_CR2_RTOEN_Msk                    /*!< Receiver Time-Out enable */
#define USART_CR2_ADD_Pos               (24UL)
#define USART_CR2_ADD_Msk               (0xFFUL << USART_CR2_ADD_Pos)          /*!< 0xFF000000 */
#define USART_CR2_ADD                   USART_CR2_ADD_Msk                      /*!< Address of the USART node */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE_Pos               (0UL)
#define USART_CR3_EIE_Msk               (0x1UL << USART_CR3_EIE_Pos)           /*!< 0x00000001 */
#define USART_CR3_EIE                   USART_CR3_EIE_Msk                      /*!< Error Interrupt Enable */
#define USART_CR3_IREN_Pos              (1UL)
#define USART_CR3_IREN_Msk              (0x1UL << USART_CR3_IREN_Pos)          /*!< 0x00000002 */
#define USART_CR3_IREN                  USART_CR3_IREN_Msk                     /*!< IrDA mode Enable */
#define USART_CR3_IRLP_Pos              (2UL)
#define USART_CR3_IRLP_Msk              (0x1UL << USART_CR3_IRLP_Pos)          /*!< 0x00000004 */
#define USART_CR3_IRLP                  USART_CR3_IRLP_Msk                     /*!< IrDA Low-Power */
#define USART_CR3_HDSEL_Pos             (3UL)
#define USART_CR3_HDSEL_Msk             (0x1UL << USART_CR3_HDSEL_Pos)         /*!< 0x00000008 */
#define USART_CR3_HDSEL                 USART_CR3_HDSEL_Msk                    /*!< Half-Duplex Selection */
#define USART_CR3_NACK_Pos              (4UL)
#define USART_CR3_NACK_Msk              (0x1UL << USART_CR3_NACK_Pos)          /*!< 0x00000010 */
#define USART_CR3_NACK                  USART_CR3_NACK_Msk                     /*!< SmartCard NACK enable */
#define USART_CR3_SCEN_Pos              (5UL)
#define USART_CR3_SCEN_Msk              (0x1UL << USART_CR3_SCEN_Pos)          /*!< 0x00000020 */
#define USART_CR3_SCEN                  USART_CR3_SCEN_Msk                     /*!< SmartCard mode enable */
#define USART_CR3_DMAR_Pos              (6UL)
#define USART_CR3_DMAR_Msk              (0x1UL << USART_CR3_DMAR_Pos)          /*!< 0x00000040 */
#define USART_CR3_DMAR                  USART_CR3_DMAR_Msk                     /*!< DMA Enable Receiver */
#define USART_CR3_DMAT_Pos              (7UL)
#define USART_CR3_DMAT_Msk              (0x1UL << USART_CR3_DMAT_Pos)          /*!< 0x00000080 */
#define USART_CR3_DMAT                  USART_CR3_DMAT_Msk                     /*!< DMA Enable Transmitter */
#define USART_CR3_RTSE_Pos              (8UL)
#define USART_CR3_RTSE_Msk              (0x1UL << USART_CR3_RTSE_Pos)          /*!< 0x00000100 */
#define USART_CR3_RTSE                  USART_CR3_RTSE_Msk                     /*!< RTS Enable */
#define USART_CR3_CTSE_Pos              (9UL)
#define USART_CR3_CTSE_Msk              (0x1UL << USART_CR3_CTSE_Pos)          /*!< 0x00000200 */
#define USART_CR3_CTSE                  USART_CR3_CTSE_Msk                     /*!< CTS Enable */
#define USART_CR3_CTSIE_Pos             (10UL)
#define USART_CR3_CTSIE_Msk             (0x1UL << USART_CR3_CTSIE_Pos)         /*!< 0x00000400 */
#define USART_CR3_CTSIE                 USART_CR3_CTSIE_Msk                    /*!< CTS Interrupt Enable */
#define USART_CR3_ONEBIT_Pos            (11UL)
#define USART_CR3_ONEBIT_Msk            (0x1UL << USART_CR3_ONEBIT_Pos)        /*!< 0x00000800 */
#define USART_CR3_ONEBIT                USART_CR3_ONEBIT_Msk                   /*!< One sample bit method enable */
#define USART_CR3_OVRDIS_Pos            (12UL)
#define USART_CR3_OVRDIS_Msk            (0x1UL << USART_CR3_OVRDIS_Pos)        /*!< 0x00001000 */
#define USART_CR3_OVRDIS                USART_CR3_OVRDIS_Msk                   /*!< Overrun Disable */
#define USART_CR3_DDRE_Pos              (13UL)
#define USART_CR3_DDRE_Msk              (0x1UL << USART_CR3_DDRE_Pos)          /*!< 0x00002000 */
#define USART_CR3_DDRE                  USART_CR3_DDRE_Msk                     /*!< DMA Disable on Reception Error */
#define USART_CR3_DEM_Pos               (14UL)
#define USART_CR3_DEM_Msk               (0x1UL << USART_CR3_DEM_Pos)           /*!< 0x00004000 */
#define USART_CR3_DEM                   USART_CR3_DEM_Msk                      /*!< Driver Enable Mode */
#define USART_CR3_DEP_Pos               (15UL)
#define USART_CR3_DEP_Msk               (0x1UL << USART_CR3_DEP_Pos)           /*!< 0x00008000 */
#define USART_CR3_DEP                   USART_CR3_DEP_Msk                      /*!< Driver Enable Polarity Selection */
#define USART_CR3_SCARCNT_Pos           (17UL)
#define USART_CR3_SCARCNT_Msk           (0x7UL << USART_CR3_SCARCNT_Pos)       /*!< 0x000E0000 */
#define USART_CR3_SCARCNT               USART_CR3_SCARCNT_Msk                  /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
#define USART_CR3_SCARCNT_0             (0x1UL << USART_CR3_SCARCNT_Pos)        /*!< 0x00020000 */
#define USART_CR3_SCARCNT_1             (0x2UL << USART_CR3_SCARCNT_Pos)        /*!< 0x00040000 */
#define USART_CR3_SCARCNT_2             (0x4UL << USART_CR3_SCARCNT_Pos)        /*!< 0x00080000 */
#define USART_CR3_WUS_Pos               (20UL)
#define USART_CR3_WUS_Msk               (0x3UL << USART_CR3_WUS_Pos)           /*!< 0x00300000 */
#define USART_CR3_WUS                   USART_CR3_WUS_Msk                      /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
#define USART_CR3_WUS_0                 (0x1UL << USART_CR3_WUS_Pos)            /*!< 0x00100000 */
#define USART_CR3_WUS_1                 (0x2UL << USART_CR3_WUS_Pos)            /*!< 0x00200000 */
#define USART_CR3_WUFIE_Pos             (22UL)
#define USART_CR3_WUFIE_Msk             (0x1UL << USART_CR3_WUFIE_Pos)         /*!< 0x00400000 */
#define USART_CR3_WUFIE                 USART_CR3_WUFIE_Msk                    /*!< Wake Up Interrupt Enable */
#define USART_CR3_TXFTIE_Pos            (23UL)
#define USART_CR3_TXFTIE_Msk            (0x1UL << USART_CR3_TXFTIE_Pos)        /*!< 0x00800000 */
#define USART_CR3_TXFTIE                USART_CR3_TXFTIE_Msk                   /*!< TXFIFO threshold interrupt enable */
#define USART_CR3_TCBGTIE_Pos           (24UL)
#define USART_CR3_TCBGTIE_Msk           (0x1UL << USART_CR3_TCBGTIE_Pos)       /*!< 0x01000000 */
#define USART_CR3_TCBGTIE               USART_CR3_TCBGTIE_Msk                  /*!< Transmission Complete before guard time, interrupt enable */
#define USART_CR3_RXFTCFG_Pos           (25UL)
#define USART_CR3_RXFTCFG_Msk           (0x7UL << USART_CR3_RXFTCFG_Pos)       /*!< 0x0E000000 */
#define USART_CR3_RXFTCFG               USART_CR3_RXFTCFG_Msk                  /*!< RXFTCFG [2:0]Receive FIFO threshold configuration */
#define USART_CR3_RXFTCFG_0             (0x1UL << USART_CR3_RXFTCFG_Pos)        /*!< 0x02000000 */
#define USART_CR3_RXFTCFG_1             (0x2UL << USART_CR3_RXFTCFG_Pos)        /*!< 0x04000000 */
#define USART_CR3_RXFTCFG_2             (0x4UL << USART_CR3_RXFTCFG_Pos)        /*!< 0x08000000 */
#define USART_CR3_RXFTIE_Pos            (28UL)
#define USART_CR3_RXFTIE_Msk            (0x1UL << USART_CR3_RXFTIE_Pos)        /*!< 0x10000000 */
#define USART_CR3_RXFTIE                USART_CR3_RXFTIE_Msk                   /*!< RXFIFO threshold interrupt enable */
#define USART_CR3_TXFTCFG_Pos           (29UL)
#define USART_CR3_TXFTCFG_Msk           (0x7UL << USART_CR3_TXFTCFG_Pos)       /*!< 0xE0000000 */
#define USART_CR3_TXFTCFG               USART_CR3_TXFTCFG_Msk                  /*!< TXFIFO [2:0] threshold configuration */
#define USART_CR3_TXFTCFG_0             (0x1UL << USART_CR3_TXFTCFG_Pos)        /*!< 0x20000000 */
#define USART_CR3_TXFTCFG_1             (0x2UL << USART_CR3_TXFTCFG_Pos)        /*!< 0x40000000 */
#define USART_CR3_TXFTCFG_2             (0x4UL << USART_CR3_TXFTCFG_Pos)        /*!< 0x80000000 */

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_FRACTION_Pos      (0UL)
#define USART_BRR_DIV_FRACTION_Msk      (0xFUL << USART_BRR_DIV_FRACTION_Pos)  /*!< 0x0000000F */
#define USART_BRR_DIV_FRACTION          USART_BRR_DIV_FRACTION_Msk             /*!< Fraction of USARTDIV */
#define USART_BRR_DIV_MANTISSA_Pos      (4UL)
#define USART_BRR_DIV_MANTISSA_Msk      (0xFFFUL << USART_BRR_DIV_MANTISSA_Pos) /*!< 0x0000FFF0 */
#define USART_BRR_DIV_MANTISSA          USART_BRR_DIV_MANTISSA_Msk             /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC_Pos              (0UL)
#define USART_GTPR_PSC_Msk              (0xFFUL << USART_GTPR_PSC_Pos)         /*!< 0x000000FF */
#define USART_GTPR_PSC                  USART_GTPR_PSC_Msk                     /*!< PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_GT_Pos               (8UL)
#define USART_GTPR_GT_Msk               (0xFFUL << USART_GTPR_GT_Pos)          /*!< 0x0000FF00 */
#define USART_GTPR_GT                   USART_GTPR_GT_Msk                      /*!< GT[7:0] bits (Guard time value) */

/*******************  Bit definition for USART_RTOR register  *****************/
#define USART_RTOR_RTO_Pos              (0UL)
#define USART_RTOR_RTO_Msk              (0xFFFFFFUL << USART_RTOR_RTO_Pos)     /*!< 0x00FFFFFF */
#define USART_RTOR_RTO                  USART_RTOR_RTO_Msk                     /*!< Receiver Time Out Value */
#define USART_RTOR_BLEN_Pos             (24UL)
#define USART_RTOR_BLEN_Msk             (0xFFUL << USART_RTOR_BLEN_Pos)        /*!< 0xFF000000 */
#define USART_RTOR_BLEN                 USART_RTOR_BLEN_Msk                    /*!< Block Length */

/*******************  Bit definition for USART_RQR register  ******************/
#define USART_RQR_ABRRQ_Pos             (0UL)
#define USART_RQR_ABRRQ_Msk             (0x1UL << USART_RQR_ABRRQ_Pos)         /*!< 0x00000001 */
#define USART_RQR_ABRRQ                 USART_RQR_ABRRQ_Msk                    /*!< Auto-Baud Rate Request */
#define USART_RQR_SBKRQ_Pos             (1UL)
#define USART_RQR_SBKRQ_Msk             (0x1UL << USART_RQR_SBKRQ_Pos)         /*!< 0x00000002 */
#define USART_RQR_SBKRQ                 USART_RQR_SBKRQ_Msk                    /*!< Send Break Request */
#define USART_RQR_MMRQ_Pos              (2UL)
#define USART_RQR_MMRQ_Msk              (0x1UL << USART_RQR_MMRQ_Pos)          /*!< 0x00000004 */
#define USART_RQR_MMRQ                  USART_RQR_MMRQ_Msk                     /*!< Mute Mode Request */
#define USART_RQR_RXFRQ_Pos             (3UL)
#define USART_RQR_RXFRQ_Msk             (0x1UL << USART_RQR_RXFRQ_Pos)         /*!< 0x00000008 */
#define USART_RQR_RXFRQ                 USART_RQR_RXFRQ_Msk                    /*!< Receive Data flush Request */
#define USART_RQR_TXFRQ_Pos             (4UL)
#define USART_RQR_TXFRQ_Msk             (0x1UL << USART_RQR_TXFRQ_Pos)         /*!< 0x00000010 */
#define USART_RQR_TXFRQ                 USART_RQR_TXFRQ_Msk                    /*!< Transmit data flush Request */

/*******************  Bit definition for USART_ISR register  ******************/
#define USART_ISR_PE_Pos                (0UL)
#define USART_ISR_PE_Msk                (0x1UL << USART_ISR_PE_Pos)            /*!< 0x00000001 */
#define USART_ISR_PE                    USART_ISR_PE_Msk                       /*!< Parity Error */
#define USART_ISR_FE_Pos                (1UL)
#define USART_ISR_FE_Msk                (0x1UL << USART_ISR_FE_Pos)            /*!< 0x00000002 */
#define USART_ISR_FE                    USART_ISR_FE_Msk                       /*!< Framing Error */
#define USART_ISR_NE_Pos                (2UL)
#define USART_ISR_NE_Msk                (0x1UL << USART_ISR_NE_Pos)            /*!< 0x00000004 */
#define USART_ISR_NE                    USART_ISR_NE_Msk                       /*!< Noise detected Flag */
#define USART_ISR_ORE_Pos               (3UL)
#define USART_ISR_ORE_Msk               (0x1UL << USART_ISR_ORE_Pos)           /*!< 0x00000008 */
#define USART_ISR_ORE                   USART_ISR_ORE_Msk                      /*!< OverRun Error */
#define USART_ISR_IDLE_Pos              (4UL)
#define USART_ISR_IDLE_Msk              (0x1UL << USART_ISR_IDLE_Pos)          /*!< 0x00000010 */
#define USART_ISR_IDLE                  USART_ISR_IDLE_Msk                     /*!< IDLE line detected */
#define USART_ISR_RXNE_RXFNE_Pos        (5UL)
#define USART_ISR_RXNE_RXFNE_Msk        (0x1UL << USART_ISR_RXNE_RXFNE_Pos)    /*!< 0x00000020 */
#define USART_ISR_RXNE_RXFNE            USART_ISR_RXNE_RXFNE_Msk               /*!< Read Data Register or RX FIFO Not Empty */
#define USART_ISR_TC_Pos                (6UL)
#define USART_ISR_TC_Msk                (0x1UL << USART_ISR_TC_Pos)            /*!< 0x00000040 */
#define USART_ISR_TC                    USART_ISR_TC_Msk                       /*!< Transmission Complete */
#define USART_ISR_TXE_TXFNF_Pos         (7UL)
#define USART_ISR_TXE_TXFNF_Msk         (0x1UL << USART_ISR_TXE_TXFNF_Pos)     /*!< 0x00000080 */
#define USART_ISR_TXE_TXFNF             USART_ISR_TXE_TXFNF_Msk                /*!< Transmit Data Register Empty or TX FIFO Not Full Flag */
#define USART_ISR_LBDF_Pos              (8UL)
#define USART_ISR_LBDF_Msk              (0x1UL << USART_ISR_LBDF_Pos)          /*!< 0x00000100 */
#define USART_ISR_LBDF                  USART_ISR_LBDF_Msk                     /*!< LIN Break Detection Flag */
#define USART_ISR_CTSIF_Pos             (9UL)
#define USART_ISR_CTSIF_Msk             (0x1UL << USART_ISR_CTSIF_Pos)         /*!< 0x00000200 */
#define USART_ISR_CTSIF                 USART_ISR_CTSIF_Msk                    /*!< CTS interrupt flag */
#define USART_ISR_CTS_Pos               (10UL)
#define USART_ISR_CTS_Msk               (0x1UL << USART_ISR_CTS_Pos)           /*!< 0x00000400 */
#define USART_ISR_CTS                   USART_ISR_CTS_Msk                      /*!< CTS flag */
#define USART_ISR_RTOF_Pos              (11UL)
#define USART_ISR_RTOF_Msk              (0x1UL << USART_ISR_RTOF_Pos)          /*!< 0x00000800 */
#define USART_ISR_RTOF                  USART_ISR_RTOF_Msk                     /*!< Receiver Time Out */
#define USART_ISR_EOBF_Pos              (12UL)
#define USART_ISR_EOBF_Msk              (0x1UL << USART_ISR_EOBF_Pos)          /*!< 0x00001000 */
#define USART_ISR_EOBF                  USART_ISR_EOBF_Msk                     /*!< End Of Block Flag */
#define USART_ISR_UDR_Pos               (13UL)
#define USART_ISR_UDR_Msk               (0x1UL << USART_ISR_UDR_Pos)           /*!< 0x00002000 */
#define USART_ISR_UDR                   USART_ISR_UDR_Msk                      /*!< SPI slave underrun error flag */
#define USART_ISR_ABRE_Pos              (14UL)
#define USART_ISR_ABRE_Msk              (0x1UL << USART_ISR_ABRE_Pos)          /*!< 0x00004000 */
#define USART_ISR_ABRE                  USART_ISR_ABRE_Msk                     /*!< Auto-Baud Rate Error */
#define USART_ISR_ABRF_Pos              (15UL)
#define USART_ISR_ABRF_Msk              (0x1UL << USART_ISR_ABRF_Pos)          /*!< 0x00008000 */
#define USART_ISR_ABRF                  USART_ISR_ABRF_Msk                     /*!< Auto-Baud Rate Flag */
#define USART_ISR_BUSY_Pos              (16UL)
#define USART_ISR_BUSY_Msk              (0x1UL << USART_ISR_BUSY_Pos)          /*!< 0x00010000 */
#define USART_ISR_BUSY                  USART_ISR_BUSY_Msk                     /*!< Busy Flag */
#define USART_ISR_CMF_Pos               (17UL)
#define USART_ISR_CMF_Msk               (0x1UL << USART_ISR_CMF_Pos)           /*!< 0x00020000 */
#define USART_ISR_CMF                   USART_ISR_CMF_Msk                      /*!< Character Match Flag */
#define USART_ISR_SBKF_Pos              (18UL)
#define USART_ISR_SBKF_Msk              (0x1UL << USART_ISR_SBKF_Pos)          /*!< 0x00040000 */
#define USART_ISR_SBKF                  USART_ISR_SBKF_Msk                     /*!< Send Break Flag */
#define USART_ISR_RWU_Pos               (19UL)
#define USART_ISR_RWU_Msk               (0x1UL << USART_ISR_RWU_Pos)           /*!< 0x00080000 */
#define USART_ISR_RWU                   USART_ISR_RWU_Msk                      /*!< Receive Wake Up from mute mode Flag */
#define USART_ISR_WUF_Pos               (20UL)
#define USART_ISR_WUF_Msk               (0x1UL << USART_ISR_WUF_Pos)           /*!< 0x00100000 */
#define USART_ISR_WUF                   USART_ISR_WUF_Msk                      /*!< Wake Up from stop mode Flag */
#define USART_ISR_TEACK_Pos             (21UL)
#define USART_ISR_TEACK_Msk             (0x1UL << USART_ISR_TEACK_Pos)         /*!< 0x00200000 */
#define USART_ISR_TEACK                 USART_ISR_TEACK_Msk                    /*!< Transmit Enable Acknowledge Flag */
#define USART_ISR_REACK_Pos             (22UL)
#define USART_ISR_REACK_Msk             (0x1UL << USART_ISR_REACK_Pos)         /*!< 0x00400000 */
#define USART_ISR_REACK                 USART_ISR_REACK_Msk                    /*!< Receive Enable Acknowledge Flag */
#define USART_ISR_TXFE_Pos              (23UL)
#define USART_ISR_TXFE_Msk              (0x1UL << USART_ISR_TXFE_Pos)          /*!< 0x00800000 */
#define USART_ISR_TXFE                  USART_ISR_TXFE_Msk                     /*!< TXFIFO Empty */
#define USART_ISR_RXFF_Pos              (24UL)
#define USART_ISR_RXFF_Msk              (0x1UL << USART_ISR_RXFF_Pos)          /*!< 0x01000000 */
#define USART_ISR_RXFF                  USART_ISR_RXFF_Msk                     /*!< RXFIFO Full Flag */
#define USART_ISR_TCBGT_Pos             (25UL)
#define USART_ISR_TCBGT_Msk             (0x1UL << USART_ISR_TCBGT_Pos)         /*!< 0x02000000 */
#define USART_ISR_TCBGT                 USART_ISR_TCBGT_Msk                    /*!< Transmission complete before guard time Flag */
#define USART_ISR_RXFT_Pos              (26UL)
#define USART_ISR_RXFT_Msk              (0x1UL << USART_ISR_RXFT_Pos)          /*!< 0x04000000 */
#define USART_ISR_RXFT                  USART_ISR_RXFT_Msk                     /*!< RXFIFO threshold Flag */
#define USART_ISR_TXFT_Pos              (27UL)
#define USART_ISR_TXFT_Msk              (0x1UL << USART_ISR_TXFT_Pos)          /*!< 0x08000000 */
#define USART_ISR_TXFT                  USART_ISR_TXFT_Msk                     /*!< TXFIFO threshold Flag */

/*******************  Bit definition for USART_ICR register  ******************/
#define USART_ICR_PECF_Pos              (0UL)
#define USART_ICR_PECF_Msk              (0x1UL << USART_ICR_PECF_Pos)          /*!< 0x00000001 */
#define USART_ICR_PECF                  USART_ICR_PECF_Msk                     /*!< Parity Error Clear Flag */
#define USART_ICR_FECF_Pos              (1UL)
#define USART_ICR_FECF_Msk              (0x1UL << USART_ICR_FECF_Pos)          /*!< 0x00000002 */
#define USART_ICR_FECF                  USART_ICR_FECF_Msk                     /*!< Framing Error Clear Flag */
#define USART_ICR_NECF_Pos              (2UL)
#define USART_ICR_NECF_Msk              (0x1UL << USART_ICR_NECF_Pos)          /*!< 0x00000004 */
#define USART_ICR_NECF                  USART_ICR_NECF_Msk                     /*!< Noise detected Clear Flag */
#define USART_ICR_ORECF_Pos             (3UL)
#define USART_ICR_ORECF_Msk             (0x1UL << USART_ICR_ORECF_Pos)         /*!< 0x00000008 */
#define USART_ICR_ORECF                 USART_ICR_ORECF_Msk                    /*!< OverRun Error Clear Flag */
#define USART_ICR_IDLECF_Pos            (4UL)
#define USART_ICR_IDLECF_Msk            (0x1UL << USART_ICR_IDLECF_Pos)        /*!< 0x00000010 */
#define USART_ICR_IDLECF                USART_ICR_IDLECF_Msk                   /*!< IDLE line detected Clear Flag */
#define USART_ICR_TXFECF_Pos            (5UL)
#define USART_ICR_TXFECF_Msk            (0x1UL << USART_ICR_TXFECF_Pos)        /*!< 0x00000020 */
#define USART_ICR_TXFECF                USART_ICR_TXFECF_Msk                   /*!< TXFIFO empty clear flag */
#define USART_ICR_TCCF_Pos              (6UL)
#define USART_ICR_TCCF_Msk              (0x1UL << USART_ICR_TCCF_Pos)          /*!< 0x00000040 */
#define USART_ICR_TCCF                  USART_ICR_TCCF_Msk                     /*!< Transmission Complete Clear Flag */
#define USART_ICR_TCBGTCF_Pos           (7UL)
#define USART_ICR_TCBGTCF_Msk           (0x1UL << USART_ICR_TCBGTCF_Pos)       /*!< 0x00000080 */
#define USART_ICR_TCBGTCF               USART_ICR_TCBGTCF_Msk                  /*!< Transmission complete before guard time Clear Flag */
#define USART_ICR_LBDCF_Pos             (8UL)
#define USART_ICR_LBDCF_Msk             (0x1UL << USART_ICR_LBDCF_Pos)         /*!< 0x00000100 */
#define USART_ICR_LBDCF                 USART_ICR_LBDCF_Msk                    /*!< LIN Break Detection Clear Flag */
#define USART_ICR_CTSCF_Pos             (9UL)
#define USART_ICR_CTSCF_Msk             (0x1UL << USART_ICR_CTSCF_Pos)         /*!< 0x00000200 */
#define USART_ICR_CTSCF                 USART_ICR_CTSCF_Msk                    /*!< CTS Interrupt Clear Flag */
#define USART_ICR_RTOCF_Pos             (11UL)
#define USART_ICR_RTOCF_Msk             (0x1UL << USART_ICR_RTOCF_Pos)         /*!< 0x00000800 */
#define USART_ICR_RTOCF                 USART_ICR_RTOCF_Msk                    /*!< Receiver Time Out Clear Flag */
#define USART_ICR_EOBCF_Pos             (12UL)
#define USART_ICR_EOBCF_Msk             (0x1UL << USART_ICR_EOBCF_Pos)         /*!< 0x00001000 */
#define USART_ICR_EOBCF                 USART_ICR_EOBCF_Msk                    /*!< End Of Block Clear Flag */
#define USART_ICR_UDRCF_Pos             (13UL)
#define USART_ICR_UDRCF_Msk             (0x1UL << USART_ICR_UDRCF_Pos)         /*!< 0x00002000 */
#define USART_ICR_UDRCF                 USART_ICR_UDRCF_Msk                    /*!< SPI slave underrun clear flag */
#define USART_ICR_CMCF_Pos              (17UL)
#define USART_ICR_CMCF_Msk              (0x1UL << USART_ICR_CMCF_Pos)          /*!< 0x00020000 */
#define USART_ICR_CMCF                  USART_ICR_CMCF_Msk                     /*!< Character Match Clear Flag */
#define USART_ICR_WUCF_Pos              (20UL)
#define USART_ICR_WUCF_Msk              (0x1UL << USART_ICR_WUCF_Pos)          /*!< 0x00100000 */
#define USART_ICR_WUCF                  USART_ICR_WUCF_Msk                     /*!< Wake Up from stop mode Clear Flag */

/*******************  Bit definition for USART_RDR register  ******************/
#define USART_RDR_RDR_Pos               (0UL)
#define USART_RDR_RDR_Msk               (0x1FFUL << USART_RDR_RDR_Pos)         /*!< 0x000001FF */
#define USART_RDR_RDR                   USART_RDR_RDR_Msk                      /*!< RDR[8:0] bits (Receive Data value) */

/*******************  Bit definition for USART_TDR register  ******************/
#define USART_TDR_TDR_Pos               (0UL)
#define USART_TDR_TDR_Msk               (0x1FFUL << USART_TDR_TDR_Pos)         /*!< 0x000001FF */
#define USART_TDR_TDR                   USART_TDR_TDR_Msk                      /*!< TDR[8:0] bits (Transmit Data value) */

/*******************  Bit definition for USART_PRESC register  ******************/
#define USART_PRESC_PRESCALER_Pos       (0UL)
#define USART_PRESC_PRESCALER_Msk       (0xFUL << USART_PRESC_PRESCALER_Pos)   /*!< 0x0000000F */
#define USART_PRESC_PRESCALER           USART_PRESC_PRESCALER_Msk              /*!< PRESCALER[3:0] bits (Clock prescaler) */
#define USART_PRESC_PRESCALER_0         (0x1UL << USART_PRESC_PRESCALER_Pos)    /*!< 0x00000001 */
#define USART_PRESC_PRESCALER_1         (0x2UL << USART_PRESC_PRESCALER_Pos)    /*!< 0x00000002 */
#define USART_PRESC_PRESCALER_2         (0x4UL << USART_PRESC_PRESCALER_Pos)    /*!< 0x00000004 */
#define USART_PRESC_PRESCALER_3         (0x8UL << USART_PRESC_PRESCALER_Pos)    /*!< 0x00000008 */

/******************************************************************************/
/*                                                                            */
/*                                 VREFBUF                                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for VREFBUF_CSR register  ****************/
#define VREFBUF_CSR_ENVR_Pos    (0UL)
#define VREFBUF_CSR_ENVR_Msk    (0x1UL << VREFBUF_CSR_ENVR_Pos)                /*!< 0x00000001 */
#define VREFBUF_CSR_ENVR        VREFBUF_CSR_ENVR_Msk                           /*!<Voltage reference buffer enable */
#define VREFBUF_CSR_HIZ_Pos     (1UL)
#define VREFBUF_CSR_HIZ_Msk     (0x1UL << VREFBUF_CSR_HIZ_Pos)                 /*!< 0x00000002 */
#define VREFBUF_CSR_HIZ         VREFBUF_CSR_HIZ_Msk                            /*!<High impedance mode             */
#define VREFBUF_CSR_VRS_Pos     (2UL)
#define VREFBUF_CSR_VRS_Msk     (0x1UL << VREFBUF_CSR_VRS_Pos)                 /*!< 0x00000004 */
#define VREFBUF_CSR_VRS         VREFBUF_CSR_VRS_Msk                            /*!<Voltage reference scale         */
#define VREFBUF_CSR_VRR_Pos     (3UL)
#define VREFBUF_CSR_VRR_Msk     (0x1UL << VREFBUF_CSR_VRR_Pos)                 /*!< 0x00000008 */
#define VREFBUF_CSR_VRR         VREFBUF_CSR_VRR_Msk                            /*!<Voltage reference buffer ready  */

/*******************  Bit definition for VREFBUF_CCR register  ******************/
#define VREFBUF_CCR_TRIM_Pos    (0UL)
#define VREFBUF_CCR_TRIM_Msk    (0x3FUL << VREFBUF_CCR_TRIM_Pos)               /*!< 0x0000003F */
#define VREFBUF_CCR_TRIM        VREFBUF_CCR_TRIM_Msk                           /*!<TRIM[5:0] bits (Trimming code)  */

/******************************************************************************/
/*                                                                            */
/*           Single Wire Protocol Master Interface (SWPMI)                    */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for SWPMI_CR register   ********************/
#define SWPMI_CR_RXDMA_Pos       (0UL)
#define SWPMI_CR_RXDMA_Msk       (0x1UL << SWPMI_CR_RXDMA_Pos)                 /*!< 0x00000001 */
#define SWPMI_CR_RXDMA           SWPMI_CR_RXDMA_Msk                            /*!<Reception DMA enable                                 */
#define SWPMI_CR_TXDMA_Pos       (1UL)
#define SWPMI_CR_TXDMA_Msk       (0x1UL << SWPMI_CR_TXDMA_Pos)                 /*!< 0x00000002 */
#define SWPMI_CR_TXDMA           SWPMI_CR_TXDMA_Msk                            /*!<Transmission DMA enable                              */
#define SWPMI_CR_RXMODE_Pos      (2UL)
#define SWPMI_CR_RXMODE_Msk      (0x1UL << SWPMI_CR_RXMODE_Pos)                /*!< 0x00000004 */
#define SWPMI_CR_RXMODE          SWPMI_CR_RXMODE_Msk                           /*!<Reception buffering mode                             */
#define SWPMI_CR_TXMODE_Pos      (3UL)
#define SWPMI_CR_TXMODE_Msk      (0x1UL << SWPMI_CR_TXMODE_Pos)                /*!< 0x00000008 */
#define SWPMI_CR_TXMODE          SWPMI_CR_TXMODE_Msk                           /*!<Transmission buffering mode                          */
#define SWPMI_CR_LPBK_Pos        (4UL)
#define SWPMI_CR_LPBK_Msk        (0x1UL << SWPMI_CR_LPBK_Pos)                  /*!< 0x00000010 */
#define SWPMI_CR_LPBK            SWPMI_CR_LPBK_Msk                             /*!<Loopback mode enable                                 */
#define SWPMI_CR_SWPACT_Pos      (5UL)
#define SWPMI_CR_SWPACT_Msk      (0x1UL << SWPMI_CR_SWPACT_Pos)                /*!< 0x00000020 */
#define SWPMI_CR_SWPACT          SWPMI_CR_SWPACT_Msk                           /*!<Single wire protocol master interface activate       */
#define SWPMI_CR_DEACT_Pos       (10UL)
#define SWPMI_CR_DEACT_Msk       (0x1UL << SWPMI_CR_DEACT_Pos)                 /*!< 0x00000400 */
#define SWPMI_CR_DEACT           SWPMI_CR_DEACT_Msk                            /*!<Single wire protocol master interface deactivate     */
#define SWPMI_CR_SWPEN_Pos       (11UL)
#define SWPMI_CR_SWPEN_Msk       (0x1UL << SWPMI_CR_SWPEN_Pos)                 /*!< 0x00000800 */
#define SWPMI_CR_SWPEN           SWPMI_CR_SWPEN_Msk                            /*!<Single wire protocol master transceiver enable       */

/*******************  Bit definition for SWPMI_BRR register  ********************/
#define SWPMI_BRR_BR_Pos         (0UL)
#define SWPMI_BRR_BR_Msk         (0xFFUL << SWPMI_BRR_BR_Pos)                  /*!< 0x000000FF */
#define SWPMI_BRR_BR             SWPMI_BRR_BR_Msk                              /*!<BR[7:0] bits (Bitrate prescaler) */

/*******************  Bit definition for SWPMI_ISR register  ********************/
#define SWPMI_ISR_RXBFF_Pos      (0UL)
#define SWPMI_ISR_RXBFF_Msk      (0x1UL << SWPMI_ISR_RXBFF_Pos)                /*!< 0x00000001 */
#define SWPMI_ISR_RXBFF          SWPMI_ISR_RXBFF_Msk                           /*!<Receive buffer full flag        */
#define SWPMI_ISR_TXBEF_Pos      (1UL)
#define SWPMI_ISR_TXBEF_Msk      (0x1UL << SWPMI_ISR_TXBEF_Pos)                /*!< 0x00000002 */
#define SWPMI_ISR_TXBEF          SWPMI_ISR_TXBEF_Msk                           /*!<Transmit buffer empty flag      */
#define SWPMI_ISR_RXBERF_Pos     (2UL)
#define SWPMI_ISR_RXBERF_Msk     (0x1UL << SWPMI_ISR_RXBERF_Pos)               /*!< 0x00000004 */
#define SWPMI_ISR_RXBERF         SWPMI_ISR_RXBERF_Msk                          /*!<Receive CRC error flag          */
#define SWPMI_ISR_RXOVRF_Pos     (3UL)
#define SWPMI_ISR_RXOVRF_Msk     (0x1UL << SWPMI_ISR_RXOVRF_Pos)               /*!< 0x00000008 */
#define SWPMI_ISR_RXOVRF         SWPMI_ISR_RXOVRF_Msk                          /*!<Receive overrun error flag      */
#define SWPMI_ISR_TXUNRF_Pos     (4UL)
#define SWPMI_ISR_TXUNRF_Msk     (0x1UL << SWPMI_ISR_TXUNRF_Pos)               /*!< 0x00000010 */
#define SWPMI_ISR_TXUNRF         SWPMI_ISR_TXUNRF_Msk                          /*!<Transmit underrun error flag    */
#define SWPMI_ISR_RXNE_Pos       (5UL)
#define SWPMI_ISR_RXNE_Msk       (0x1UL << SWPMI_ISR_RXNE_Pos)                 /*!< 0x00000020 */
#define SWPMI_ISR_RXNE           SWPMI_ISR_RXNE_Msk                            /*!<Receive data register not empty */
#define SWPMI_ISR_TXE_Pos        (6UL)
#define SWPMI_ISR_TXE_Msk        (0x1UL << SWPMI_ISR_TXE_Pos)                  /*!< 0x00000040 */
#define SWPMI_ISR_TXE            SWPMI_ISR_TXE_Msk                             /*!<Transmit data register empty    */
#define SWPMI_ISR_TCF_Pos        (7UL)
#define SWPMI_ISR_TCF_Msk        (0x1UL << SWPMI_ISR_TCF_Pos)                  /*!< 0x00000080 */
#define SWPMI_ISR_TCF            SWPMI_ISR_TCF_Msk                             /*!<Transfer complete flag          */
#define SWPMI_ISR_SRF_Pos        (8UL)
#define SWPMI_ISR_SRF_Msk        (0x1UL << SWPMI_ISR_SRF_Pos)                  /*!< 0x00000100 */
#define SWPMI_ISR_SRF            SWPMI_ISR_SRF_Msk                             /*!<Slave resume flag               */
#define SWPMI_ISR_SUSP_Pos       (9UL)
#define SWPMI_ISR_SUSP_Msk       (0x1UL << SWPMI_ISR_SUSP_Pos)                 /*!< 0x00000200 */
#define SWPMI_ISR_SUSP           SWPMI_ISR_SUSP_Msk                            /*!<SUSPEND flag                    */
#define SWPMI_ISR_DEACTF_Pos     (10UL)
#define SWPMI_ISR_DEACTF_Msk     (0x1UL << SWPMI_ISR_DEACTF_Pos)               /*!< 0x00000400 */
#define SWPMI_ISR_DEACTF         SWPMI_ISR_DEACTF_Msk                          /*!<DEACTIVATED flag                */
#define SWPMI_ISR_RDYF_Pos       (11UL)
#define SWPMI_ISR_RDYF_Msk       (0x1UL << SWPMI_ISR_RDYF_Pos)                 /*!< 0x00000800 */
#define SWPMI_ISR_RDYF           SWPMI_ISR_RDYF_Msk                            /*!<Transceiver ready flag          */

/*******************  Bit definition for SWPMI_ICR register  ********************/
#define SWPMI_ICR_CRXBFF_Pos     (0UL)
#define SWPMI_ICR_CRXBFF_Msk     (0x1UL << SWPMI_ICR_CRXBFF_Pos)               /*!< 0x00000001 */
#define SWPMI_ICR_CRXBFF         SWPMI_ICR_CRXBFF_Msk                          /*!<Clear receive buffer full flag       */
#define SWPMI_ICR_CTXBEF_Pos     (1UL)
#define SWPMI_ICR_CTXBEF_Msk     (0x1UL << SWPMI_ICR_CTXBEF_Pos)               /*!< 0x00000002 */
#define SWPMI_ICR_CTXBEF         SWPMI_ICR_CTXBEF_Msk                          /*!<Clear transmit buffer empty flag     */
#define SWPMI_ICR_CRXBERF_Pos    (2UL)
#define SWPMI_ICR_CRXBERF_Msk    (0x1UL << SWPMI_ICR_CRXBERF_Pos)              /*!< 0x00000004 */
#define SWPMI_ICR_CRXBERF        SWPMI_ICR_CRXBERF_Msk                         /*!<Clear receive CRC error flag         */
#define SWPMI_ICR_CRXOVRF_Pos    (3UL)
#define SWPMI_ICR_CRXOVRF_Msk    (0x1UL << SWPMI_ICR_CRXOVRF_Pos)              /*!< 0x00000008 */
#define SWPMI_ICR_CRXOVRF        SWPMI_ICR_CRXOVRF_Msk                         /*!<Clear receive overrun error flag     */
#define SWPMI_ICR_CTXUNRF_Pos    (4UL)
#define SWPMI_ICR_CTXUNRF_Msk    (0x1UL << SWPMI_ICR_CTXUNRF_Pos)              /*!< 0x00000010 */
#define SWPMI_ICR_CTXUNRF        SWPMI_ICR_CTXUNRF_Msk                         /*!<Clear transmit underrun error flag   */
#define SWPMI_ICR_CTCF_Pos       (7UL)
#define SWPMI_ICR_CTCF_Msk       (0x1UL << SWPMI_ICR_CTCF_Pos)                 /*!< 0x00000080 */
#define SWPMI_ICR_CTCF           SWPMI_ICR_CTCF_Msk                            /*!<Clear transfer complete flag         */
#define SWPMI_ICR_CSRF_Pos       (8UL)
#define SWPMI_ICR_CSRF_Msk       (0x1UL << SWPMI_ICR_CSRF_Pos)                 /*!< 0x00000100 */
#define SWPMI_ICR_CSRF           SWPMI_ICR_CSRF_Msk                            /*!<Clear slave resume flag              */
#define SWPMI_ICR_CRDYF_Pos      (11UL)
#define SWPMI_ICR_CRDYF_Msk      (0x1UL << SWPMI_ICR_CRDYF_Pos)                /*!< 0x00000800 */
#define SWPMI_ICR_CRDYF          SWPMI_ICR_CRDYF_Msk                           /*!<Clear transceiver ready flag         */

/*******************  Bit definition for SWPMI_IER register  ********************/
#define SWPMI_IER_RXBFIE_Pos     (0UL)
#define SWPMI_IER_RXBFIE_Msk     (0x1UL << SWPMI_IER_RXBFIE_Pos)               /*!< 0x00000001 */
#define SWPMI_IER_RXBFIE         SWPMI_IER_RXBFIE_Msk                          /*!<Receive buffer full interrupt enable        */
#define SWPMI_IER_TXBEIE_Pos     (1UL)
#define SWPMI_IER_TXBEIE_Msk     (0x1UL << SWPMI_IER_TXBEIE_Pos)               /*!< 0x00000002 */
#define SWPMI_IER_TXBEIE         SWPMI_IER_TXBEIE_Msk                          /*!<Transmit buffer empty interrupt enable      */
#define SWPMI_IER_RXBERIE_Pos    (2UL)
#define SWPMI_IER_RXBERIE_Msk    (0x1UL << SWPMI_IER_RXBERIE_Pos)              /*!< 0x00000004 */
#define SWPMI_IER_RXBERIE        SWPMI_IER_RXBERIE_Msk                         /*!<Receive CRC error interrupt enable          */
#define SWPMI_IER_RXOVRIE_Pos    (3UL)
#define SWPMI_IER_RXOVRIE_Msk    (0x1UL << SWPMI_IER_RXOVRIE_Pos)              /*!< 0x00000008 */
#define SWPMI_IER_RXOVRIE        SWPMI_IER_RXOVRIE_Msk                         /*!<Receive overrun error interrupt enable      */
#define SWPMI_IER_TXUNRIE_Pos    (4UL)
#define SWPMI_IER_TXUNRIE_Msk    (0x1UL << SWPMI_IER_TXUNRIE_Pos)              /*!< 0x00000010 */
#define SWPMI_IER_TXUNRIE        SWPMI_IER_TXUNRIE_Msk                         /*!<Transmit underrun error interrupt enable    */
#define SWPMI_IER_RIE_Pos        (5UL)
#define SWPMI_IER_RIE_Msk        (0x1UL << SWPMI_IER_RIE_Pos)                  /*!< 0x00000020 */
#define SWPMI_IER_RIE            SWPMI_IER_RIE_Msk                             /*!<Receive interrupt enable                    */
#define SWPMI_IER_TIE_Pos        (6UL)
#define SWPMI_IER_TIE_Msk        (0x1UL << SWPMI_IER_TIE_Pos)                  /*!< 0x00000040 */
#define SWPMI_IER_TIE            SWPMI_IER_TIE_Msk                             /*!<Transmit interrupt enable                   */
#define SWPMI_IER_TCIE_Pos       (7UL)
#define SWPMI_IER_TCIE_Msk       (0x1UL << SWPMI_IER_TCIE_Pos)                 /*!< 0x00000080 */
#define SWPMI_IER_TCIE           SWPMI_IER_TCIE_Msk                            /*!<Transmit complete interrupt enable          */
#define SWPMI_IER_SRIE_Pos       (8UL)
#define SWPMI_IER_SRIE_Msk       (0x1UL << SWPMI_IER_SRIE_Pos)                 /*!< 0x00000100 */
#define SWPMI_IER_SRIE           SWPMI_IER_SRIE_Msk                            /*!<Slave resume interrupt enable               */
#define SWPMI_IER_RDYIE_Pos      (11UL)
#define SWPMI_IER_RDYIE_Msk      (0x1UL << SWPMI_IER_RDYIE_Pos)                /*!< 0x00000800 */
#define SWPMI_IER_RDYIE          SWPMI_IER_RDYIE_Msk                           /*!<Transceiver ready interrupt enable          */

/*******************  Bit definition for SWPMI_RFL register  ********************/
#define SWPMI_RFL_RFL_Pos        (0UL)
#define SWPMI_RFL_RFL_Msk        (0x1FUL << SWPMI_RFL_RFL_Pos)                 /*!< 0x0000001F */
#define SWPMI_RFL_RFL            SWPMI_RFL_RFL_Msk                             /*!<RFL[4:0] bits (Receive Frame length) */
#define SWPMI_RFL_RFL_0_1        ((uint32_t)0x00000003)                        /*!<RFL[1:0] bits (number of relevant bytes for the last SWPMI_RDR register read.) */

/*******************  Bit definition for SWPMI_TDR register  ********************/
#define SWPMI_TDR_TD_Pos         (0UL)
#define SWPMI_TDR_TD_Msk         (0xFFFFFFFFUL << SWPMI_TDR_TD_Pos)            /*!< 0xFFFFFFFF */
#define SWPMI_TDR_TD             SWPMI_TDR_TD_Msk                              /*!<Transmit Data Register         */

/*******************  Bit definition for SWPMI_RDR register  ********************/
#define SWPMI_RDR_RD_Pos         (0UL)
#define SWPMI_RDR_RD_Msk         (0xFFFFFFFFUL << SWPMI_RDR_RD_Pos)            /*!< 0xFFFFFFFF */
#define SWPMI_RDR_RD             SWPMI_RDR_RD_Msk                              /*!<Receive Data Register           */


/*******************  Bit definition for SWPMI_OR register  ********************/
#define SWPMI_OR_TBYP_Pos        (0UL)
#define SWPMI_OR_TBYP_Msk        (0x1UL << SWPMI_OR_TBYP_Pos)                  /*!< 0x00000001 */
#define SWPMI_OR_TBYP            SWPMI_OR_TBYP_Msk                             /*!<SWP Transceiver Bypass */
#define SWPMI_OR_CLASS_Pos       (1UL)
#define SWPMI_OR_CLASS_Msk       (0x1UL << SWPMI_OR_CLASS_Pos)                 /*!< 0x00000002 */
#define SWPMI_OR_CLASS           SWPMI_OR_CLASS_Msk                            /*!<SWP CLASS selection */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define WWDG_CR_T_Pos           (0UL)
#define WWDG_CR_T_Msk           (0x7FUL << WWDG_CR_T_Pos)                      /*!< 0x0000007F */
#define WWDG_CR_T               WWDG_CR_T_Msk                                  /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define WWDG_CR_T_0             (0x01UL << WWDG_CR_T_Pos)                       /*!< 0x00000001 */
#define WWDG_CR_T_1             (0x02UL << WWDG_CR_T_Pos)                       /*!< 0x00000002 */
#define WWDG_CR_T_2             (0x04UL << WWDG_CR_T_Pos)                       /*!< 0x00000004 */
#define WWDG_CR_T_3             (0x08UL << WWDG_CR_T_Pos)                       /*!< 0x00000008 */
#define WWDG_CR_T_4             (0x10UL << WWDG_CR_T_Pos)                       /*!< 0x00000010 */
#define WWDG_CR_T_5             (0x20UL << WWDG_CR_T_Pos)                       /*!< 0x00000020 */
#define WWDG_CR_T_6             (0x40UL << WWDG_CR_T_Pos)                       /*!< 0x00000040 */

#define WWDG_CR_WDGA_Pos        (7UL)
#define WWDG_CR_WDGA_Msk        (0x1UL << WWDG_CR_WDGA_Pos)                    /*!< 0x00000080 */
#define WWDG_CR_WDGA            WWDG_CR_WDGA_Msk                               /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define WWDG_CFR_W_Pos          (0UL)
#define WWDG_CFR_W_Msk          (0x7FUL << WWDG_CFR_W_Pos)                     /*!< 0x0000007F */
#define WWDG_CFR_W              WWDG_CFR_W_Msk                                 /*!<W[6:0] bits (7-bit window value) */
#define WWDG_CFR_W_0            (0x01UL << WWDG_CFR_W_Pos)                      /*!< 0x00000001 */
#define WWDG_CFR_W_1            (0x02UL << WWDG_CFR_W_Pos)                      /*!< 0x00000002 */
#define WWDG_CFR_W_2            (0x04UL << WWDG_CFR_W_Pos)                      /*!< 0x00000004 */
#define WWDG_CFR_W_3            (0x08UL << WWDG_CFR_W_Pos)                      /*!< 0x00000008 */
#define WWDG_CFR_W_4            (0x10UL << WWDG_CFR_W_Pos)                      /*!< 0x00000010 */
#define WWDG_CFR_W_5            (0x20UL << WWDG_CFR_W_Pos)                      /*!< 0x00000020 */
#define WWDG_CFR_W_6            (0x40UL << WWDG_CFR_W_Pos)                      /*!< 0x00000040 */

#define WWDG_CFR_EWI_Pos        (9UL)
#define WWDG_CFR_EWI_Msk        (0x1UL << WWDG_CFR_EWI_Pos)                    /*!< 0x00000200 */
#define WWDG_CFR_EWI            WWDG_CFR_EWI_Msk                               /*!<Early Wakeup Interrupt */

#define WWDG_CFR_WDGTB_Pos      (11UL)
#define WWDG_CFR_WDGTB_Msk      (0x7UL << WWDG_CFR_WDGTB_Pos)                  /*!< 0x00003800 */
#define WWDG_CFR_WDGTB          WWDG_CFR_WDGTB_Msk                             /*!<WDGTB[2:0] bits (Timer Base) */
#define WWDG_CFR_WDGTB_0        (0x1UL << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000800 */
#define WWDG_CFR_WDGTB_1        (0x2UL << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00001000 */
#define WWDG_CFR_WDGTB_2        (0x4UL << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00002000 */

/*******************  Bit definition for WWDG_SR register  ********************/
#define WWDG_SR_EWIF_Pos        (0UL)
#define WWDG_SR_EWIF_Msk        (0x1UL << WWDG_SR_EWIF_Pos)                    /*!< 0x00000001 */
#define WWDG_SR_EWIF            WWDG_SR_EWIF_Msk                               /*!<Early Wakeup Interrupt Flag */


/******************************************************************************/
/*                                                                            */
/*                                DBG                                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for DBGMCU_IDCODE register  *************/
#define DBGMCU_IDCODE_DEV_ID_Pos          (0UL)
#define DBGMCU_IDCODE_DEV_ID_Msk          (0xFFFUL << DBGMCU_IDCODE_DEV_ID_Pos)  /*!< 0x00000FFF */
#define DBGMCU_IDCODE_DEV_ID              DBGMCU_IDCODE_DEV_ID_Msk
#define DBGMCU_IDCODE_REV_ID_Pos          (16UL)
#define DBGMCU_IDCODE_REV_ID_Msk          (0xFFFFUL << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0xFFFF0000 */
#define DBGMCU_IDCODE_REV_ID              DBGMCU_IDCODE_REV_ID_Msk

/********************  Bit definition for DBGMCU_CR register  *****************/
#define DBGMCU_CR_DBG_STOP_Pos           (1UL)
#define DBGMCU_CR_DBG_STOP_Msk           (0x1UL << DBGMCU_CR_DBG_STOP_Pos)     /*!< 0x00000002 */
#define DBGMCU_CR_DBG_STOP               DBGMCU_CR_DBG_STOP_Msk
#define DBGMCU_CR_DBG_STANDBY_Pos        (2UL)
#define DBGMCU_CR_DBG_STANDBY_Msk        (0x1UL << DBGMCU_CR_DBG_STANDBY_Pos)  /*!< 0x00000004 */
#define DBGMCU_CR_DBG_STANDBY            DBGMCU_CR_DBG_STANDBY_Msk

/********************  Bit definition for DBGMCU_APB1FZR register  ***********/
#define DBGMCU_APBFZ1_DBG_TIM2_STOP_Pos                  (0UL)
#define DBGMCU_APBFZ1_DBG_TIM2_STOP_Msk                  (0x1UL << DBGMCU_APBFZ1_DBG_TIM2_STOP_Pos)  /*!< 0x00000001 */
#define DBGMCU_APBFZ1_DBG_TIM2_STOP                      DBGMCU_APBFZ1_DBG_TIM2_STOP_Msk
#define DBGMCU_APBFZ1_DBG_TIM3_STOP_Pos                  (1UL)
#define DBGMCU_APBFZ1_DBG_TIM3_STOP_Msk                  (0x1UL << DBGMCU_APBFZ1_DBG_TIM3_STOP_Pos)  /*!< 0x00000002 */
#define DBGMCU_APBFZ1_DBG_TIM3_STOP                      DBGMCU_APBFZ1_DBG_TIM3_STOP_Msk
#define DBGMCU_APBFZ1_DBG_TIM4_STOP_Pos                  (2UL)
#define DBGMCU_APBFZ1_DBG_TIM4_STOP_Msk                  (0x1UL << DBGMCU_APBFZ1_DBG_TIM4_STOP_Pos)  /*!< 0x00000004 */
#define DBGMCU_APBFZ1_DBG_TIM4_STOP                      DBGMCU_APBFZ1_DBG_TIM4_STOP_Msk
#define DBGMCU_APBFZ1_DBG_TIM6_STOP_Pos                  (4UL)
#define DBGMCU_APBFZ1_DBG_TIM6_STOP_Msk                  (0x1UL << DBGMCU_APBFZ1_DBG_TIM6_STOP_Pos)  /*!< 0x00000010 */
#define DBGMCU_APBFZ1_DBG_TIM6_STOP                      DBGMCU_APBFZ1_DBG_TIM6_STOP_Msk
#define DBGMCU_APBFZ1_DBG_TIM7_STOP_Pos                  (5UL)
#define DBGMCU_APBFZ1_DBG_TIM7_STOP_Msk                  (0x1UL << DBGMCU_APBFZ1_DBG_TIM7_STOP_Pos)  /*!< 0x00000020 */
#define DBGMCU_APBFZ1_DBG_TIM7_STOP                      DBGMCU_APBFZ1_DBG_TIM7_STOP_Msk
#define DBGMCU_APBFZ1_DBG_RTC_STOP_Pos                   (10UL)
#define DBGMCU_APBFZ1_DBG_RTC_STOP_Msk                   (0x1UL << DBGMCU_APBFZ1_DBG_RTC_STOP_Pos)   /*!< 0x00000400 */
#define DBGMCU_APBFZ1_DBG_RTC_STOP                       DBGMCU_APBFZ1_DBG_RTC_STOP_Msk
#define DBGMCU_APBFZ1_DBG_WWDG_STOP_Pos                  (11UL)
#define DBGMCU_APBFZ1_DBG_WWDG_STOP_Msk                  (0x1UL << DBGMCU_APBFZ1_DBG_WWDG_STOP_Pos)  /*!< 0x00000800 */
#define DBGMCU_APBFZ1_DBG_WWDG_STOP                      DBGMCU_APBFZ1_DBG_WWDG_STOP_Msk
#define DBGMCU_APBFZ1_DBG_IWDG_STOP_Pos                  (12UL)
#define DBGMCU_APBFZ1_DBG_IWDG_STOP_Msk                  (0x1UL << DBGMCU_APBFZ1_DBG_IWDG_STOP_Pos) /*!< 0x00001000 */
#define DBGMCU_APBFZ1_DBG_IWDG_STOP                      DBGMCU_APBFZ1_DBG_IWDG_STOP_Msk
#define DBGMCU_APBFZ1_DBG_I2C3_STOP_Pos                  (21UL)
#define DBGMCU_APBFZ1_DBG_I2C3_STOP_Msk                  (0x1UL << DBGMCU_APBFZ1_DBG_I2C3_STOP_Pos) /*!< 0x00200000 */
#define DBGMCU_APBFZ1_DBG_I2C3_STOP                      DBGMCU_APBFZ1_DBG_I2C3_STOP_Msk
#define DBGMCU_APBFZ1_DBG_I2C1_STOP_Pos                  (22UL)
#define DBGMCU_APBFZ1_DBG_I2C1_STOP_Msk                  (0x1UL << DBGMCU_APBFZ1_DBG_I2C1_STOP_Pos) /*!< 0x00400000 */
#define DBGMCU_APBFZ1_DBG_I2C1_STOP                      DBGMCU_APBFZ1_DBG_I2C1_STOP_Msk
#define DBGMCU_APBFZ1_DBG_LPTIM2_STOP_Pos                (30UL)
#define DBGMCU_APBFZ1_DBG_LPTIM2_STOP_Msk                (0x1UL << DBGMCU_APBFZ1_DBG_LPTIM2_STOP_Pos) /*!< 0x40000000 */
#define DBGMCU_APBFZ1_DBG_LPTIM2_STOP                    DBGMCU_APBFZ1_DBG_LPTIM2_STOP_Msk
#define DBGMCU_APBFZ1_DBG_LPTIM1_STOP_Pos                (31UL)
#define DBGMCU_APBFZ1_DBG_LPTIM1_STOP_Msk                (0x1UL << DBGMCU_APBFZ1_DBG_LPTIM1_STOP_Pos) /*!< 0x80000000 */
#define DBGMCU_APBFZ1_DBG_LPTIM1_STOP                    DBGMCU_APBFZ1_DBG_LPTIM1_STOP_Msk

/********************  Bit definition for DBGMCU_APB2FZR register  ************/
#define DBGMCU_APBFZ2_DBG_TIM1_STOP_Pos                  (11UL)
#define DBGMCU_APBFZ2_DBG_TIM1_STOP_Msk                  (0x1UL << DBGMCU_APBFZ2_DBG_TIM1_STOP_Pos)  /*!< 0x00000800 */
#define DBGMCU_APBFZ2_DBG_TIM1_STOP                      DBGMCU_APBFZ2_DBG_TIM1_STOP_Msk
#define DBGMCU_APBFZ2_DBG_TIM14_STOP_Pos                 (15UL)
#define DBGMCU_APBFZ2_DBG_TIM14_STOP_Msk                 (0x1UL << DBGMCU_APBFZ2_DBG_TIM14_STOP_Pos) /*!< 0x00008000 */
#define DBGMCU_APBFZ2_DBG_TIM14_STOP                     DBGMCU_APBFZ2_DBG_TIM14_STOP_Msk
#define DBGMCU_APBFZ2_DBG_TIM15_STOP_Pos                 (16UL)
#define DBGMCU_APBFZ2_DBG_TIM15_STOP_Msk                 (0x1UL << DBGMCU_APBFZ2_DBG_TIM15_STOP_Pos) /*!< 0x00010000 */
#define DBGMCU_APBFZ2_DBG_TIM15_STOP                     DBGMCU_APBFZ2_DBG_TIM15_STOP_Msk
#define DBGMCU_APBFZ2_DBG_TIM16_STOP_Pos                 (17UL)
#define DBGMCU_APBFZ2_DBG_TIM16_STOP_Msk                 (0x1UL << DBGMCU_APBFZ2_DBG_TIM16_STOP_Pos) /*!< 0x00020000 */
#define DBGMCU_APBFZ2_DBG_TIM16_STOP                     DBGMCU_APBFZ2_DBG_TIM16_STOP_Msk
#define DBGMCU_APBFZ2_DBG_LPTIM3_STOP_Pos                (18UL)
#define DBGMCU_APBFZ2_DBG_LPTIM3_STOP_Msk                (0x1UL << DBGMCU_APBFZ2_DBG_LPTIM3_STOP_Pos) /*!< 0x00040000 */
#define DBGMCU_APBFZ2_DBG_LPTIM3_STOP                    DBGMCU_APBFZ2_DBG_LPTIM3_STOP_Msk

/********************  Bit definition for DBGMCU_SR register  ************/
#define DBGMCU_SR_AP1_PRESENT_Pos                     (0UL)
#define DBGMCU_SR_AP1_PRESENT_Msk                     (0x1UL << DBGMCU_SR_AP1_PRESENT_Pos)           /*!< 0x00000001 */
#define DBGMCU_SR_AP1_PRESENT                         DBGMCU_SR_AP1_PRESENT_Msk
#define DBGMCU_SR_AP0_PRESENT_Pos                     (1UL)
#define DBGMCU_SR_AP0_PRESENT_Msk                     (0x1UL << DBGMCU_SR_AP0_PRESENT_Pos)           /*!< 0x00000002 */
#define DBGMCU_SR_AP0_PRESENT                         DBGMCU_SR_AP0_PRESENT_Msk
#define DBGMCU_SR_AP1_ENABLED_Pos                     (16UL)
#define DBGMCU_SR_AP1_ENABLED_Msk                     (0x1UL << DBGMCU_SR_AP1_ENABLED_Pos)           /*!< 0x00010000 */
#define DBGMCU_SR_AP1_ENABLED                         DBGMCU_SR_AP1_ENABLED_Msk
#define DBGMCU_SR_AP0_ENABLED_Pos                     (17UL)
#define DBGMCU_SR_AP0_ENABLED_Msk                     (0x1UL << DBGMCU_SR_AP0_ENABLED_Pos)           /*!< 0x00020000 */
#define DBGMCU_SR_AP0_ENABLED                         DBGMCU_SR_AP0_ENABLED_Msk

/********************  Bit definition for DBGMCU_DBG_AUTH_HOST register  ************/
#define DBGMCU_DBG_AUTH_HOST_AP1_MESSAGE_Pos          (0UL)
#define DBGMCU_DBG_AUTH_HOST_AP1_MESSAGE_Msk          (0xFFFFFFFFUL << DBGMCU_DBG_AUTH_HOST_AP1_MESSAGE_Pos)  /*!< 0xFFFFFFFF */
#define DBGMCU_DBG_AUTH_HOST_AP1_MESSAGE              DBGMCU_DBG_AUTH_HOST_AP1_MESSAGE_Msk

/********************  Bit definition for DBGMCU_DBG_AUTH_DEVICE register  ************/
#define DBGMCU_DBG_AUTH_DEVICE_MESSAGE_Pos            (0UL)
#define DBGMCU_DBG_AUTH_DEVICE_MESSAGE_Msk            (0xFFFFFFFFUL << DBGMCU_DBG_AUTH_DEVICE_MESSAGE_Pos)  /*!< 0xFFFFFFFF */
#define DBGMCU_DBG_AUTH_DEVICE_MESSAGE                DBGMCU_DBG_AUTH_DEVICE_MESSAGE_Msk

/********************  Bit definition for DBGMCU_PIDR4 register  ************/
#define DBGMCU_PIDR4_JEP106CON_Pos                   (0UL)
#define DBGMCU_PIDR4_JEP106CON_Msk                   (0xFUL << DBGMCU_PIDR4_JEP106CON_Pos)                  /*!< 0x0000000F */
#define DBGMCU_PIDR4_JEP106CON                       DBGMCU_PIDR4_JEP106CON_Msk
#define DBGMCU_PIDR4_SIZE_Pos                        (4UL)
#define DBGMCU_PIDR4_SIZE_Msk                        (0xFUL << DBGMCU_PIDR4_SIZE_Pos)                       /*!< 0x0000000F */
#define DBGMCU_PIDR4_SIZE                            DBGMCU_PIDR4_SIZE_Msk

/********************  Bit definition for DBGMCU_PIDR0 register  ************/
#define DBGMCU_PIDR0_PARTNUM_Pos                     (0UL)
#define DBGMCU_PIDR0_PARTNUM_Msk                     (0xFFUL << DBGMCU_PIDR0_PARTNUM_Pos)                   /*!< 0x000000FF */
#define DBGMCU_PIDR0_PARTNUM                         DBGMCU_PIDR0_PARTNUM_Msk

/********************  Bit definition for DBGMCU_PIDR1 register  ************/
#define DBGMCU_PIDR1_PARTNUM_Pos                     (0UL)
#define DBGMCU_PIDR1_PARTNUM_Msk                     (0xFUL << DBGMCU_PIDR1_PARTNUM_Pos)                    /*!< 0x0000000F */
#define DBGMCU_PIDR1_PARTNUM                         DBGMCU_PIDR1_PARTNUM_Msk
#define DBGMCU_PIDR1_JEP106ID_Pos                    (4UL)
#define DBGMCU_PIDR1_JEP106ID_Msk                    (0xFUL << DBGMCU_PIDR1_JEP106ID_Pos)                   /*!< 0x0000000F */
#define DBGMCU_PIDR1_JEP106ID                        DBGMCU_PIDR1_JEP106ID_Msk

/********************  Bit definition for DBGMCU_PIDR2 register  ************/
#define DBGMCU_PIDR2_JEP106ID_Pos                     (0UL)
#define DBGMCU_PIDR2_JEP106ID_Msk                     (0x7UL << DBGMCU_PIDR2_JEP106ID_Pos)                    /*!< 0x00000007 */
#define DBGMCU_PIDR2_JEP106ID                         DBGMCU_PIDR2_JEP106ID_Msk
#define DBGMCU_PIDR2_JEDEC_Pos                        (4UL)
#define DBGMCU_PIDR2_JEDEC_Msk                        (0x1UL << DBGMCU_PIDR2_JEDEC_Pos)                       /*!< 0x00000008 */
#define DBGMCU_PIDR2_JEDEC                            DBGMCU_PIDR2_JEDEC_Msk
#define DBGMCU_PIDR2_REVISION_Pos                     (4UL)
#define DBGMCU_PIDR2_REVISION_Msk                     (0xFUL << DBGMCU_PIDR2_REVISION_Pos)                    /*!< 0x000000F0 */
#define DBGMCU_PIDR2_REVISION                         DBGMCU_PIDR2_REVISION_Msk

/********************  Bit definition for DBGMCU_PIDR3 register  ************/
#define DBGMCU_PIDR3_CMOD_Pos                         (0UL)
#define DBGMCU_PIDR3_CMOD_Msk                         (0xFUL << DBGMCU_PIDR3_CMOD_Pos)                            /*!< 0x0000000F */
#define DBGMCU_PIDR3_CMOD                             DBGMCU_PIDR3_CMOD_Msk
#define DBGMCU_PIDR3_REVAND_Pos                       (4UL)
#define DBGMCU_PIDR3_REVAND_Msk                       (0xFUL << DBGMCU_PIDR3_REVAND_Pos)                          /*!< 0x000000F0 */
#define DBGMCU_PIDR3_REVAND                           DBGMCU_PIDR3_REVAND_Msk

/********************  Bit definition for DBGMCU_CIDR0 register  ************/
#define DBGMCU_CIDR0_PREAMBLE_Pos                     (0UL)
#define DBGMCU_CIDR0_PREAMBLE_Msk                     (0xFFUL << DBGMCU_CIDR0_PREAMBLE_Pos)                    /*!< 0x000000FF */
#define DBGMCU_CIDR0_PREAMBLE                         DBGMCU_CIDR0_PREAMBLE_Msk

/********************  Bit definition for DBGMCU_CIDR1 register  ************/
#define DBGMCU_CIDR1_PREAMBLE_Pos                     (0UL)
#define DBGMCU_CIDR1_PREAMBLE_Msk                     (0xFUL << DBGMCU_CIDR1_PREAMBLE_Pos)                    /*!< 0x0000000F */
#define DBGMCU_CIDR1_PREAMBLE                         DBGMCU_CIDR1_PREAMBLE_Msk
#define DBGMCU_CIDR1_CLASS_Pos                        (4UL)
#define DBGMCU_CIDR1_CLASS_Msk                        (0xFUL << DBGMCU_CIDR1_CLASS_Pos)                       /*!< 0x000000F0 */
#define DBGMCU_CIDR1_CLASS                            DBGMCU_CIDR1_CLASS_Msk

/********************  Bit definition for DBGMCU_CIDR2 register  ************/
#define DBGMCU_CIDR2_PREAMBLE_Pos                     (0UL)
#define DBGMCU_CIDR2_PREAMBLE_Msk                     (0xFFUL << DBGMCU_CIDR2_PREAMBLE_Pos)                    /*!< 0x000000FF */
#define DBGMCU_CIDR2_PREAMBLE                         DBGMCU_CIDR2_PREAMBLE_Msk

/********************  Bit definition for DBGMCU_CIDR3 register  ************/
#define DBGMCU_CIDR3_PREAMBLE_Pos                     (0UL)
#define DBGMCU_CIDR3_PREAMBLE_Msk                     (0xFFUL << DBGMCU_CIDR3_PREAMBLE_Pos)                    /*!< 0x000000FF */
#define DBGMCU_CIDR3_PREAMBLE                         DBGMCU_CIDR3_PREAMBLE_Msk

/******************************************************************************/
/*                                                                            */
/*                         USB Dual Role Device FS Endpoint registers         */
/*                                                                            */
/******************************************************************************/

/******************  Bits definition for USB_DRD_CNTR register  *******************/
#define USB_CNTR_HOST_Pos               (31UL)
#define USB_CNTR_HOST_Msk               (0x1UL << USB_CNTR_HOST_Pos)    /*!< 0x80000000 */
#define USB_CNTR_HOST                   USB_CNTR_HOST_Msk               /*!< Host Mode  */
#define USB_CNTR_THR512M_Pos            (16UL)
#define USB_CNTR_THR512M_Msk            (0x1UL << USB_CNTR_THR512M_Pos)  /*!< 0x00010000 */
#define USB_CNTR_THR512M                USB_CNTR_THR512M_Msk             /*!< 512byte Threshold interrupt mask */
#define USB_CNTR_CTRM_Pos               (15UL)
#define USB_CNTR_CTRM_Msk               (0x1UL << USB_CNTR_CTRM_Pos)    /*!< 0x00008000 */
#define USB_CNTR_CTRM                   USB_CNTR_CTRM_Msk               /*!< Correct Transfer Mask */
#define USB_CNTR_PMAOVRM_Pos            (14UL)
#define USB_CNTR_PMAOVRM_Msk            (0x1UL << USB_CNTR_PMAOVRM_Pos) /*!< 0x00004000 */
#define USB_CNTR_PMAOVRM                USB_CNTR_PMAOVRM_Msk            /*!< DMA OVeR/underrun Mask */
#define USB_CNTR_ERRM_Pos               (13UL)
#define USB_CNTR_ERRM_Msk               (0x1UL << USB_CNTR_ERRM_Pos)    /*!< 0x00002000 */
#define USB_CNTR_ERRM                   USB_CNTR_ERRM_Msk               /*!< ERRor Mask */
#define USB_CNTR_WKUPM_Pos              (12UL)
#define USB_CNTR_WKUPM_Msk              (0x1UL << USB_CNTR_WKUPM_Pos)   /*!< 0x00001000 */
#define USB_CNTR_WKUPM                  USB_CNTR_WKUPM_Msk              /*!< WaKe UP Mask */
#define USB_CNTR_SUSPM_Pos              (11UL)
#define USB_CNTR_SUSPM_Msk              (0x1UL << USB_CNTR_SUSPM_Pos)   /*!< 0x00000800 */
#define USB_CNTR_SUSPM                  USB_CNTR_SUSPM_Msk              /*!< SUSPend Mask */
#define USB_CNTR_RESETM_Pos             (10UL)
#define USB_CNTR_RESETM_Msk             (0x1UL << USB_CNTR_RESETM_Pos)  /*!< 0x00000400 */
#define USB_CNTR_RESETM                 USB_CNTR_RESETM_Msk             /*!< RESET Mask */
#define USB_CNTR_DCON                   USB_CNTR_RESETM_Msk             /*!< Disconnection Connection Mask */
#define USB_CNTR_SOFM_Pos               (9UL)
#define USB_CNTR_SOFM_Msk               (0x1UL << USB_CNTR_SOFM_Pos)    /*!< 0x00000200 */
#define USB_CNTR_SOFM                   USB_CNTR_SOFM_Msk               /*!< Start Of Frame Mask */
#define USB_CNTR_ESOFM_Pos              (8UL)
#define USB_CNTR_ESOFM_Msk              (0x1UL << USB_CNTR_ESOFM_Pos)   /*!< 0x00000100 */
#define USB_CNTR_ESOFM                  USB_CNTR_ESOFM_Msk              /*!< Expected Start Of Frame Mask */
#define USB_CNTR_L1REQM_Pos             (7UL)
#define USB_CNTR_L1REQM_Msk             (0x1UL << USB_CNTR_L1REQM_Pos)  /*!< 0x00000080 */
#define USB_CNTR_L1REQM                 USB_CNTR_L1REQM_Msk             /*!< LPM L1 state request interrupt Mask */
#define USB_CNTR_L1XACT_Pos             (6UL)
#define USB_CNTR_L1XACT_Msk             (0x1UL << USB_CNTR_L1XACT_Pos)  /*!< 0x00000040 */
#define USB_CNTR_L1XACT                 USB_CNTR_L1XACT_Msk             /*!< Host LPM L1 transaction request Mask */
#define USB_CNTR_L1RES_Pos              (5UL)
#define USB_CNTR_L1RES_Msk              (0x1UL << USB_CNTR_L1RES_Pos)   /*!< 0x00000020 */
#define USB_CNTR_L1RES                  USB_CNTR_L1RES_Msk              /*!< LPM L1 Resume request/ Remote Wakeup Mask */
#define USB_CNTR_L2RES_Pos              (4UL)
#define USB_CNTR_L2RES_Msk              (0x1UL << USB_CNTR_L2RES_Pos)   /*!< 0x00000010 */
#define USB_CNTR_L2RES                  USB_CNTR_L2RES_Msk              /*!< L2 Remote Wakeup / Resume driver Mask */
#define USB_CNTR_SUSPEN_Pos             (3UL)
#define USB_CNTR_SUSPEN_Msk             (0x1UL << USB_CNTR_SUSPEN_Pos)  /*!< 0x00000008 */
#define USB_CNTR_SUSPEN                 USB_CNTR_SUSPEN_Msk             /*!< Suspend state enable Mask */
#define USB_CNTR_SUSPRDY_Pos            (2UL)
#define USB_CNTR_SUSPRDY_Msk            (0x1UL << USB_CNTR_SUSPRDY_Pos) /*!< 0x00000004 */
#define USB_CNTR_SUSPRDY                USB_CNTR_SUSPRDY_Msk            /*!< Suspend state effective Mask */
#define USB_CNTR_PDWN_Pos               (1UL)
#define USB_CNTR_PDWN_Msk               (0x1UL << USB_CNTR_PDWN_Pos)    /*!< 0x00000002 */
#define USB_CNTR_PDWN                   USB_CNTR_PDWN_Msk               /*!< Power DoWN Mask */
#define USB_CNTR_USBRST_Pos             (0UL)
#define USB_CNTR_USBRST_Msk             (0x1UL << USB_CNTR_USBRST_Pos)  /*!< 0x00000001 */
#define USB_CNTR_USBRST                 USB_CNTR_USBRST_Msk             /*!< USB Reset Mask */

/******************  Bits definition for USB_DRD_ISTR register  *******************/
#define USB_ISTR_IDN_Pos                (0UL)
#define USB_ISTR_IDN_Msk                (0xFUL << USB_ISTR_IDN_Pos)     /*!< 0x0000000F */
#define USB_ISTR_IDN                    USB_ISTR_IDN_Msk                /*!< EndPoint IDentifier (read-only bit) Mask */
#define USB_ISTR_DIR_Pos                (4UL)
#define USB_ISTR_DIR_Msk                (0x1UL << USB_ISTR_DIR_Pos)     /*!< 0x00000010 */
#define USB_ISTR_DIR                    USB_ISTR_DIR_Msk                /*!< DIRection of transaction (read-only bit) Mask */
#define USB_ISTR_L1REQ_Pos              (7UL)
#define USB_ISTR_L1REQ_Msk              (0x1UL << USB_ISTR_L1REQ_Pos)   /*!< 0x00000080 */
#define USB_ISTR_L1REQ                  USB_ISTR_L1REQ_Msk              /*!< LPM L1 state request Mask */
#define USB_ISTR_ESOF_Pos               (8UL)
#define USB_ISTR_ESOF_Msk               (0x1UL << USB_ISTR_ESOF_Pos)    /*!< 0x00000100 */
#define USB_ISTR_ESOF                   USB_ISTR_ESOF_Msk               /*!< Expected Start Of Frame (clear-only bit) Mask */
#define USB_ISTR_SOF_Pos                (9UL)
#define USB_ISTR_SOF_Msk                (0x1UL << USB_ISTR_SOF_Pos)     /*!< 0x00000200 */
#define USB_ISTR_SOF                    USB_ISTR_SOF_Msk                /*!< Start Of Frame (clear-only bit) Mask */
#define USB_ISTR_RESET_Pos              (10UL)
#define USB_ISTR_RESET_Msk              (0x1UL << USB_ISTR_RESET_Pos)   /*!< 0x00000400 */
#define USB_ISTR_RESET                  USB_ISTR_RESET_Msk              /*!< RESET Mask */
#define USB_ISTR_DCON_Pos               (10UL)
#define USB_ISTR_DCON_Msk               (0x1UL << USB_ISTR_DCON_Pos)    /*!< 0x00000400 */
#define USB_ISTR_DCON                   USB_ISTR_DCON_Msk               /*!< HOST MODE-Device Connection or disconnection Mask */
#define USB_ISTR_SUSP_Pos               (11UL)
#define USB_ISTR_SUSP_Msk               (0x1UL << USB_ISTR_SUSP_Pos)    /*!< 0x00000800 */
#define USB_ISTR_SUSP                   USB_ISTR_SUSP_Msk               /*!< SUSPend (clear-only bit) Mask */
#define USB_ISTR_WKUP_Pos               (12UL)
#define USB_ISTR_WKUP_Msk               (0x1UL << USB_ISTR_WKUP_Pos)    /*!< 0x00001000 */
#define USB_ISTR_WKUP                   USB_ISTR_WKUP_Msk               /*!< WaKe UP (clear-only bit) Mask */
#define USB_ISTR_ERR_Pos                (13UL)
#define USB_ISTR_ERR_Msk                (0x1UL << USB_ISTR_ERR_Pos)     /*!< 0x00002000 */
#define USB_ISTR_ERR                    USB_ISTR_ERR_Msk                /*!< ERRor (clear-only bit) Mask */
#define USB_ISTR_PMAOVR_Pos             (14UL)
#define USB_ISTR_PMAOVR_Msk             (0x1UL << USB_ISTR_PMAOVR_Pos)  /*!< 0x00004000 */
#define USB_ISTR_PMAOVR                 USB_ISTR_PMAOVR_Msk             /*!< PMA OVeR/underrun (clear-only bit) Mask */
#define USB_ISTR_CTR_Pos                (15UL)
#define USB_ISTR_CTR_Msk                (0x1UL << USB_ISTR_CTR_Pos)     /*!< 0x00008000 */
#define USB_ISTR_CTR                    USB_ISTR_CTR_Msk                /*!< Correct TRansfer (clear-only bit) Mask */
#define USB_ISTR_THR512_Pos             (16UL)
#define USB_ISTR_THR512_Msk             (0x1UL << USB_ISTR_THR512_Pos)  /*!< 0x00010000 */
#define USB_ISTR_THR512                 USB_ISTR_THR512_Msk             /*!< 512byte threshold interrupt (used with isochrnous single buffer ) */
#define USB_ISTR_DCON_STAT_Pos          (29UL)
#define USB_ISTR_DCON_STAT_Msk          (0x1UL << USB_ISTR_DCON_STAT_Pos)/*!< 0x20000000 */
#define USB_ISTR_DCON_STAT              USB_ISTR_DCON_STAT_Msk           /*!< Device Connection status (connected/Disconnected) don't cause an interrupt */
#define USB_ISTR_LS_DCONN_Pos           (30UL)
#define USB_ISTR_LS_DCONN_Msk           (0x1UL << USB_ISTR_LS_DCONN_Pos)/*!< 0x40000000 */
#define USB_ISTR_LS_DCONN               USB_ISTR_LS_DCONN_Msk           /*!< LS_DCONN Mask */

/******************  Bits definition for USB_DRD_FNR register  ********************/
#define USB_FNR_FN_Pos                  (0UL)
#define USB_FNR_FN_Msk                  (0x7FFUL << USB_FNR_FN_Pos)     /*!< 0x000007FF */
#define USB_FNR_FN                      USB_FNR_FN_Msk                  /*!< Frame Number Mask */
#define USB_FNR_LSOF_Pos                (11UL)
#define USB_FNR_LSOF_Msk                (0x3UL << USB_FNR_LSOF_Pos)     /*!< 0x00001800 */
#define USB_FNR_LSOF                    USB_FNR_LSOF_Msk                /*!< Lost SOF  Mask */
#define USB_FNR_LCK_Pos                 (13UL)
#define USB_FNR_LCK_Msk                 (0x1UL << USB_FNR_LCK_Pos)      /*!< 0x00002000 */
#define USB_FNR_LCK                     USB_FNR_LCK_Msk                 /*!< LoCKed Mask */
#define USB_FNR_RXDM_Pos                (14UL)
#define USB_FNR_RXDM_Msk                (0x1UL << USB_FNR_RXDM_Pos)     /*!< 0x00004000 */
#define USB_FNR_RXDM                    USB_FNR_RXDM_Msk                /*!< status of D- data line Mask */
#define USB_FNR_RXDP_Pos                (15UL)
#define USB_FNR_RXDP_Msk                (0x1UL << USB_FNR_RXDP_Pos)     /*!< 0x00008000 */
#define USB_FNR_RXDP                    USB_FNR_RXDP_Msk                /*!< status of D+ data line Mask */

/******************  Bits definition for USB_DRD_DADDR register    ****************/
#define USB_DADDR_ADD_Pos               (0UL)
#define USB_DADDR_ADD_Msk               (0x7FUL << USB_DADDR_ADD_Pos)   /*!< 0x0000007F */
#define USB_DADDR_ADD                   USB_DADDR_ADD_Msk               /*!<  ADD[6:0] bits (Device Address)Mask */
#define USB_DADDR_ADD0_Pos              (0UL)
#define USB_DADDR_ADD0_Msk              (0x1UL << USB_DADDR_ADD0_Pos)   /*!< 0x00000001 */
#define USB_DADDR_ADD0                  USB_DADDR_ADD0_Msk              /*!< Bit 0 Mask */
#define USB_DADDR_ADD1_Pos              (1UL)
#define USB_DADDR_ADD1_Msk              (0x1UL << USB_DADDR_ADD1_Pos)   /*!< 0x00000002 */
#define USB_DADDR_ADD1                  USB_DADDR_ADD1_Msk              /*!< Bit 1 Mask */
#define USB_DADDR_ADD2_Pos              (2UL)
#define USB_DADDR_ADD2_Msk              (0x1UL << USB_DADDR_ADD2_Pos)   /*!< 0x00000004 */
#define USB_DADDR_ADD2                  USB_DADDR_ADD2_Msk              /*!< Bit 2 Mask */
#define USB_DADDR_ADD3_Pos              (3UL)
#define USB_DADDR_ADD3_Msk              (0x1UL << USB_DADDR_ADD3_Pos)   /*!< 0x00000008 */
#define USB_DADDR_ADD3                  USB_DADDR_ADD3_Msk              /*!< Bit 3 Mask */
#define USB_DADDR_ADD4_Pos              (4UL)
#define USB_DADDR_ADD4_Msk              (0x1UL << USB_DADDR_ADD4_Pos)   /*!< 0x00000010 */
#define USB_DADDR_ADD4                  USB_DADDR_ADD4_Msk              /*!< Bit 4 Mask */
#define USB_DADDR_ADD5_Pos              (5UL)
#define USB_DADDR_ADD5_Msk              (0x1UL << USB_DADDR_ADD5_Pos)   /*!< 0x00000020 */
#define USB_DADDR_ADD5                  USB_DADDR_ADD5_Msk              /*!< Bit 5 Mask */
#define USB_DADDR_ADD6_Pos              (6UL)
#define USB_DADDR_ADD6_Msk              (0x1UL << USB_DADDR_ADD6_Pos)   /*!< 0x00000040 */
#define USB_DADDR_ADD6                  USB_DADDR_ADD6_Msk              /*!< Bit 6 Mask */
#define USB_DADDR_EF_Pos                (7UL)
#define USB_DADDR_EF_Msk                (0x1UL << USB_DADDR_EF_Pos)     /*!< 0x00000080 */
#define USB_DADDR_EF                    USB_DADDR_EF_Msk                /*!< Enable Function Mask */

/******************  Bit definition for USB_DRD_BTABLE register  ******************/
#define USB_BTABLE_BTABLE_Pos          (3UL)
#define USB_BTABLE_BTABLE_Msk          (0xFFF8UL << USB_BTABLE_BTABLE_Pos)/*!< 0x00000000 */
#define USB_BTABLE_BTABLE              USB_BTABLE_BTABLE_Msk              /*!< Buffer Table Mask */

/*******************  Bit definition for LPMCSR register  *********************/
#define USB_LPMCSR_LMPEN_Pos           (0UL)
#define USB_LPMCSR_LMPEN_Msk           (0x1UL << USB_LPMCSR_LMPEN_Pos)  /*!< 0x00000001 */
#define USB_LPMCSR_LMPEN               USB_LPMCSR_LMPEN_Msk             /*!< LPM support enable Mask */
#define USB_LPMCSR_LPMACK_Pos          (1UL)
#define USB_LPMCSR_LPMACK_Msk          (0x1UL << USB_LPMCSR_LPMACK_Pos) /*!< 0x00000002 */
#define USB_LPMCSR_LPMACK              USB_LPMCSR_LPMACK_Msk            /*!< LPM Token acknowledge enable Mask */
#define USB_LPMCSR_REMWAKE_Pos         (3UL)
#define USB_LPMCSR_REMWAKE_Msk         (0x1UL << USB_LPMCSR_REMWAKE_Pos)/*!< 0x00000008 */
#define USB_LPMCSR_REMWAKE             USB_LPMCSR_REMWAKE_Msk           /*!< bRemoteWake value received with last ACKed LPM Token Mask */
#define USB_LPMCSR_BESL_Pos            (4UL)
#define USB_LPMCSR_BESL_Msk            (0xFUL << USB_LPMCSR_BESL_Pos)   /*!< 0x000000F0 */
#define USB_LPMCSR_BESL                USB_LPMCSR_BESL_Msk              /*!< BESL value received with last ACKed LPM Token Mask */

/******************  Bits definition for USB_DRD_BCDR register  *******************/
#define USB_BCDR_BCDEN_Pos             (0UL)
#define USB_BCDR_BCDEN_Msk             (0x1UL << USB_BCDR_BCDEN_Pos)    /*!< 0x00000001 */
#define USB_BCDR_BCDEN                 USB_BCDR_BCDEN_Msk               /*!< Battery charging detector (BCD) enable Mask */
#define USB_BCDR_DCDEN_Pos             (1UL)
#define USB_BCDR_DCDEN_Msk             (0x1UL << USB_BCDR_DCDEN_Pos)    /*!< 0x00000002 */
#define USB_BCDR_DCDEN                 USB_BCDR_DCDEN_Msk               /*!< Data contact detection (DCD) mode enable Mask */
#define USB_BCDR_PDEN_Pos              (2UL)
#define USB_BCDR_PDEN_Msk              (0x1UL << USB_BCDR_PDEN_Pos)     /*!< 0x00000004 */
#define USB_BCDR_PDEN                  USB_BCDR_PDEN_Msk                /*!< Primary detection (PD) mode enable Mask */
#define USB_BCDR_SDEN_Pos              (3UL)
#define USB_BCDR_SDEN_Msk              (0x1UL << USB_BCDR_SDEN_Pos)     /*!< 0x00000008 */
#define USB_BCDR_SDEN                  USB_BCDR_SDEN_Msk                /*!< Secondary detection (SD) mode enable Mask */
#define USB_BCDR_DCDET_Pos             (4UL)
#define USB_BCDR_DCDET_Msk             (0x1UL << USB_BCDR_DCDET_Pos)    /*!< 0x00000010 */
#define USB_BCDR_DCDET                 USB_BCDR_DCDET_Msk               /*!< Data contact detection (DCD) status Mask */
#define USB_BCDR_PDET_Pos              (5UL)
#define USB_BCDR_PDET_Msk              (0x1UL << USB_BCDR_PDET_Pos)     /*!< 0x00000020 */
#define USB_BCDR_PDET                  USB_BCDR_PDET_Msk                /*!< Primary detection (PD) status Mask */
#define USB_BCDR_SDET_Pos              (6UL)
#define USB_BCDR_SDET_Msk              (0x1UL << USB_BCDR_SDET_Pos)     /*!< 0x00000040 */
#define USB_BCDR_SDET                  USB_BCDR_SDET_Msk                /*!< Secondary detection (SD) status Mask */
#define USB_BCDR_PS2DET_Pos            (7UL)
#define USB_BCDR_PS2DET_Msk            (0x1UL << USB_BCDR_PS2DET_Pos)   /*!< 0x00000080 */
#define USB_BCDR_PS2DET                USB_BCDR_PS2DET_Msk              /*!< PS2 port or proprietary charger detected Mask */
#define USB_BCDR_DPPU_Pos              (15UL)
#define USB_BCDR_DPPU_Msk              (0x1UL << USB_BCDR_DPPU_Pos)     /*!< 0x00008000 */
#define USB_BCDR_DPPU                  USB_BCDR_DPPU_Msk                /*!< DP Pull-up Enable Mask */
#define USB_BCDR_DPPD_Pos              (15UL)
#define USB_BCDR_DPPD_Msk              (0x1UL << USB_BCDR_DPPD_Pos)     /*!< 0x00008000 */
#define USB_BCDR_DPPD                  USB_BCDR_DPPD_Msk                /*!< DP Pull-Down Enable Mask */

/******************  Bits definition for USB_DRD_CHEP register  *******************/
#define USB_CHEP_ERRRX_Pos             (26UL)
#define USB_CHEP_ERRRX_Msk             (0x01UL << USB_CHEP_ERRRX_Pos)   /*!< 0x04000000 */
#define USB_CHEP_ERRRX                 USB_CHEP_ERRRX_Msk               /*!< Receive error */
#define USB_EP_ERRRX                   USB_CHEP_ERRRX_Msk               /*!< EP Receive error */
#define USB_CH_ERRRX                   USB_CHEP_ERRRX_Msk               /*!< CH Receive error */
#define USB_CHEP_ERRTX_Pos             (25UL)
#define USB_CHEP_ERRTX_Msk             (0x01UL << USB_CHEP_ERRTX_Pos)   /*!< 0x02000000 */
#define USB_CHEP_ERRTX                 USB_CHEP_ERRTX_Msk               /*!< Transmit error */
#define USB_EP_ERRTX                   USB_CHEP_ERRTX_Msk               /*!< EP Transmit error */
#define USB_CH_ERRTX                   USB_CHEP_ERRTX_Msk               /*!< CH Transmit error */
#define USB_CHEP_LSEP_Pos              (24UL)
#define USB_CHEP_LSEP_Msk              (0x01UL << USB_CHEP_LSEP_Pos)    /*!< 0x01000000 */
#define USB_CHEP_LSEP                  USB_CHEP_LSEP_Msk                /*!< Low Speed Endpoint (host with Hub Only) */
#define USB_CHEP_NAK_Pos               (23UL)
#define USB_CHEP_NAK_Msk               (0x01UL << USB_CHEP_NAK_Pos)     /*!< 0x00800000 */
#define USB_CHEP_NAK                   USB_CHEP_NAK_Msk                 /*!< Previous NAK detected */
#define USB_CHEP_DEVADDR_Pos           (16UL)
#define USB_CHEP_DEVADDR_Msk           (0x7FU << USB_CHEP_DEVADDR_Pos)  /*!< 0x7F000000 */
#define USB_CHEP_DEVADDR               USB_CHEP_DEVADDR_Msk             /* Target Endpoint address*/
#define USB_CHEP_VTRX_Pos              (15UL)
#define USB_CHEP_VTRX_Msk              (0x1UL << USB_CHEP_VTRX_Pos)     /*!< 0x00008000 */
#define USB_CHEP_VTRX                  USB_CHEP_VTRX_Msk                /*!< USB valid transaction received Mask */
#define USB_EP_VTRX                    USB_CHEP_VTRX_Msk                /*!< USB Endpoint valid transaction received Mask */
#define USB_CH_VTRX                    USB_CHEP_VTRX_Msk                /*!< USB valid Channel transaction received Mask */
#define USB_CHEP_DTOG_RX_Pos           (14UL)
#define USB_CHEP_DTOG_RX_Msk           (0x1UL << USB_CHEP_DTOG_RX_Pos)  /*!< 0x00004000 */
#define USB_CHEP_DTOG_RX               USB_CHEP_DTOG_RX_Msk             /*!< Data Toggle, for reception transfers Mask */
#define USB_EP_DTOG_RX                 USB_CHEP_DTOG_RX_Msk             /*!< EP Data Toggle, for reception transfers Mask */
#define USB_CH_DTOG_RX                 USB_CHEP_DTOG_RX_Msk             /*!< CH Data Toggle, for reception transfers Mask */
#define USB_CHEP_RX_STRX_Pos           (12UL)
#define USB_CHEP_RX_STRX_Msk           (0x3UL << USB_CHEP_RX_STRX_Pos)  /*!< 0x00003000 */
#define USB_CHEP_RX_STRX               USB_CHEP_RX_STRX_Msk             /*!< Status bits, for reception transfers Mask */
#define USB_EP_RX_STRX                 USB_CHEP_RX_STRX_Msk             /*!< Status bits, for EP reception transfers Mask */
#define USB_CH_RX_STRX                 USB_CHEP_RX_STRX_Msk             /*!< Status bits, for CH reception transfers Mask */
#define USB_CHEP_SETUP_Pos             (11UL)
#define USB_CHEP_SETUP_Msk             (0x1UL << USB_CHEP_SETUP_Pos)    /*!< 0x00000800 */
#define USB_CHEP_SETUP                 USB_CHEP_SETUP_Msk               /*!< Setup transaction completed Mask */
#define USB_EP_SETUP                   USB_CHEP_SETUP_Msk               /*!< EP Setup transaction completed Mask */
#define USB_CH_SETUP                   USB_CHEP_SETUP_Msk               /*!< CH Setup transaction completed Mask */
#define USB_CHEP_UTYPE_Pos             (9UL)
#define USB_CHEP_UTYPE_Msk             (0x3UL << USB_CHEP_UTYPE_Pos)    /*!< 0x00000600 */
#define USB_CHEP_UTYPE                 USB_CHEP_UTYPE_Msk               /*!< USB type of transaction Mask */
#define USB_EP_UTYPE                   USB_CHEP_UTYPE_Msk               /*!< USB type of EP transaction Mask */
#define USB_CH_UTYPE                   USB_CHEP_UTYPE_Msk               /*!< USB type of CH transaction Mask */
#define USB_CHEP_KIND_Pos              (8UL)
#define USB_CHEP_KIND_Msk              (0x1UL << USB_CHEP_KIND_Pos)     /*!< 0x00000100 */
#define USB_CHEP_KIND                  USB_CHEP_KIND_Msk                /*!< EndPoint KIND Mask */
#define USB_EP_KIND                    USB_CHEP_KIND_Msk                /*!< EndPoint KIND Mask */
#define USB_CH_KIND                    USB_CHEP_KIND_Msk                /*!< Channel KIND Mask */
#define USB_CHEP_VTTX_Pos              (7UL)
#define USB_CHEP_VTTX_Msk              (0x1UL << USB_CHEP_VTTX_Pos)     /*!< 0x00000080 */
#define USB_CHEP_VTTX                  USB_CHEP_VTTX_Msk                /*!< Valid USB transaction transmitted Mask */
#define USB_EP_VTTX                    USB_CHEP_VTTX_Msk                /*!< USB Endpoint valid transaction transmitted Mask */
#define USB_CH_VTTX                    USB_CHEP_VTTX_Msk                /*!< USB valid Channel transaction transmitted Mask */
#define USB_CHEP_DTOG_TX_Pos           (6UL)
#define USB_CHEP_DTOG_TX_Msk           (0x1UL << USB_CHEP_DTOG_TX_Pos)  /*!< 0x00000040 */
#define USB_CHEP_DTOG_TX               USB_CHEP_DTOG_TX_Msk             /*!< Data Toggle, for transmission transfers Mask */
#define USB_EP_DTOG_TX                 USB_CHEP_DTOG_TX_Msk             /*!< EP Data Toggle, for transmission transfers Mask */
#define USB_CH_DTOG_TX                 USB_CHEP_DTOG_TX_Msk             /*!< CH Data Toggle, for transmission transfers Mask */
#define USB_CHEP_TX_STTX_Pos           (4UL)
#define USB_CHEP_TX_STTX_Msk           (0x3UL << USB_CHEP_TX_STTX_Pos)  /*!< 0x00000030 */
#define USB_CHEP_TX_STTX               USB_CHEP_TX_STTX_Msk             /*!< Status bits, for transmission transfers Mask */
#define USB_EP_TX_STTX                 USB_CHEP_TX_STTX_Msk             /*!< Status bits, for EP transmission transfers Mask */
#define USB_CH_TX_STTX                 USB_CHEP_TX_STTX_Msk             /*!< Status bits, for CH transmission transfers Mask */
#define USB_CHEP_ADDR_Pos              (0UL)
#define USB_CHEP_ADDR_Msk              (0xFUL << USB_CHEP_ADDR_Pos)     /*!< 0x0000000F */
#define USB_CHEP_ADDR                  USB_CHEP_ADDR_Msk                /*!< Endpoint address Mask */

/* EndPoint Register MASK (no toggle fields) */
#define USB_CHEP_REG_MASK                          (USB_CHEP_ERRRX | USB_CHEP_ERRTX | USB_CHEP_LSEP | \
                                                    USB_CHEP_DEVADDR | USB_CHEP_VTRX | USB_CHEP_SETUP | \
                                                    USB_CHEP_UTYPE | USB_CHEP_KIND | USB_CHEP_VTTX | USB_CHEP_ADDR | \
                                                    USB_CHEP_NAK) /* 0x07FF8F8F */

#define USB_CHEP_TX_DTOGMASK                       (USB_CHEP_TX_STTX | USB_CHEP_REG_MASK)
#define USB_CHEP_RX_DTOGMASK                       (USB_CHEP_RX_STRX | USB_CHEP_REG_MASK)

#define USB_CHEP_TX_DTOG1                          (0x00000010UL)           /*!< Channel/EndPoint TX Data Toggle bit1 */
#define USB_CHEP_TX_DTOG2                          (0x00000020UL)           /*!< Channel/EndPoint TX Data Toggle bit2 */
#define USB_CHEP_RX_DTOG1                          (0x00001000UL)           /*!< Channel/EndPoint RX Data Toggle bit1 */
#define USB_CHEP_RX_DTOG2                          (0x00002000UL)           /*!< Channel/EndPoint RX Data Toggle bit1 */

/*!< EP_TYPE[1:0] Channel/EndPoint TYPE */
#define USB_EP_TYPE_MASK                           (0x00000600UL)           /*!< Channel/EndPoint TYPE Mask */
#define USB_EP_BULK                                (0x00000000UL)           /*!< Channel/EndPoint BULK */
#define USB_EP_CONTROL                             (0x00000200UL)           /*!< Channel/EndPoint CONTROL */
#define USB_EP_ISOCHRONOUS                         (0x00000400UL)           /*!< Channel/EndPoint ISOCHRONOUS */
#define USB_EP_INTERRUPT                           (0x00000600UL)           /*!< Channel/EndPoint INTERRUPT */

#define USB_EP_T_MASK                              ((~USB_EP_UTYPE) & USB_CHEP_REG_MASK) /* =0x898F */
#define USB_CH_T_MASK                              ((~USB_CH_UTYPE) & USB_CHEP_REG_MASK) /* =0x898F */

#define USB_EP_KIND_MASK                           ((~USB_EP_KIND) & USB_CHEP_REG_MASK) /*!< EP_KIND EndPoint KIND */
#define USB_CH_KIND_MASK                           ((~USB_CH_KIND) & USB_CHEP_REG_MASK) /*!< EP_KIND EndPoint KIND */

/*!< STAT_TX[1:0] STATus for TX transfer */
#define USB_EP_TX_DIS                              (0x00000000UL)           /*!< EndPoint TX Disabled */
#define USB_EP_TX_STALL                            (0x00000010UL)           /*!< EndPoint TX STALLed */
#define USB_EP_TX_NAK                              (0x00000020UL)           /*!< EndPoint TX NAKed */
#define USB_EP_TX_VALID                            (0x00000030UL)           /*!< EndPoint TX VALID */

#define USB_CH_TX_DIS                              (0x00000000UL)           /*!< Channel TX Disabled */
#define USB_CH_TX_STALL                            (0x00000010UL)           /*!< Channel TX STALLed */
#define USB_CH_TX_NAK                              (0x00000020UL)           /*!< Channel TX NAKed */
#define USB_CH_TX_VALID                            (0x00000030UL)           /*!< Channel TX VALID */

#define USB_EP_TX_ACK_SBUF                         (0x00000000UL)           /*!< ACK single buffer mode */
#define USB_EP_TX_ACK_DBUF                         (0x00000030UL)           /*!< ACK Double buffer mode */

#define USB_CH_TX_ACK_SBUF                         (0x00000000UL)           /*!< ACK single buffer mode */
#define USB_CH_TX_ACK_DBUF                         (0x00000030UL)           /*!< ACK Double buffer mode */

/*!< STAT_RX[1:0] STATus for RX transfer */
#define USB_EP_RX_DIS                              (0x00000000UL)           /*!< EndPoint RX Disabled */
#define USB_EP_RX_STALL                            (0x00001000UL)           /*!< EndPoint RX STALLed */
#define USB_EP_RX_NAK                              (0x00002000UL)           /*!< EndPoint RX NAKed */
#define USB_EP_RX_VALID                            (0x00003000UL)           /*!< EndPoint RX VALID */

#define USB_EP_RX_ACK_SBUF                         (0x00000000UL)           /*!< ACK single buffer mode */
#define USB_EP_RX_ACK_DBUF                         (0x00003000UL)           /*!< ACK Double buffer mode */



#define USB_CH_RX_DIS                              (0x00000000UL)           /*!< EndPoint RX Disabled */
#define USB_CH_RX_STALL                            (0x00001000UL)           /*!< EndPoint RX STALLed */
#define USB_CH_RX_NAK                              (0x00002000UL)           /*!< Channel RX NAKed */
#define USB_CH_RX_VALID                            (0x00003000UL)           /*!< Channel RX VALID */

#define USB_CH_RX_ACK_SBUF                         (0x00000000UL)           /*!< ACK single buffer mode */
#define USB_CH_RX_ACK_DBUF                         (0x00003000UL)           /*!< ACK Double buffer mode */

/*! <used For Double Buffer Enable Disable */
#define USB_CHEP_DB_MSK                            (0xFFFF0F0FUL)

/*Buffer Descriptor Mask*/
#define USB_PMA_TXBD_ADDMSK                        (0xFFFF0000UL)
#define USB_PMA_TXBD_COUNTMSK                      (0x0000FFFFUL)
#define USB_PMA_RXBD_ADDMSK                        (0xFFFF0000UL)
#define USB_PMA_RXBD_COUNTMSK                      (0x03FFFFFFUL)
/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup Exported_macros
  * @{
  */

/******************************* ADC Instances ********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)
#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC1_COMMON)


/******************************** COMP Instances ******************************/
#define IS_COMP_ALL_INSTANCE(INSTANCE) (((INSTANCE) == COMP1) || \
                                        ((INSTANCE) == COMP2))

#define IS_COMP_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == COMP12_COMMON)

/******************** COMP Instances with window mode capability **************/
#define IS_COMP_WINDOWMODE_INSTANCE(INSTANCE) (((INSTANCE) == COMP1) || \
                                               ((INSTANCE) == COMP2))
                                               
/******************************* CRC Instances ********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)

/******************************* DAC Instances ********************************/
#define IS_DAC_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DAC1))

/******************************** DMA Instances *******************************/
#define IS_DMA_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Channel1) || \
                                       ((INSTANCE) == DMA1_Channel2) || \
                                       ((INSTANCE) == DMA1_Channel3) || \
                                       ((INSTANCE) == DMA1_Channel4) || \
                                       ((INSTANCE) == DMA1_Channel5) || \
                                       ((INSTANCE) == DMA1_Channel6) || \
                                       ((INSTANCE) == DMA1_Channel7) || \
                                       ((INSTANCE) == DMA2_Channel1) || \
                                       ((INSTANCE) == DMA2_Channel2) || \
                                       ((INSTANCE) == DMA2_Channel3) || \
                                       ((INSTANCE) == DMA2_Channel4) || \
                                       ((INSTANCE) == DMA2_Channel5))

/****************************** DMA STREAM Instances ***************************/
#define IS_DMA_STREAM_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Stream0)   || \
                                          ((INSTANCE) == DMA1_Stream1)   || \
                                          ((INSTANCE) == DMA1_Stream2)   || \
                                          ((INSTANCE) == DMA1_Stream3)   || \
                                          ((INSTANCE) == DMA1_Stream4)   || \
                                          ((INSTANCE) == DMA1_Stream5)   || \
                                          ((INSTANCE) == DMA1_Stream6)   || \
                                          ((INSTANCE) == DMA1_Stream7)   || \
                                          ((INSTANCE) == DMA2_Stream0)   || \
                                          ((INSTANCE) == DMA2_Stream1)   || \
                                          ((INSTANCE) == DMA2_Stream2)   || \
                                          ((INSTANCE) == DMA2_Stream3)   || \
                                          ((INSTANCE) == DMA2_Stream4)   || \
                                          ((INSTANCE) == DMA2_Stream5)   || \
                                          ((INSTANCE) == DMA2_Stream6)   || \
                                          ((INSTANCE) == DMA2_Stream7))

/******************************** DMAMUX Instances ****************************/
#define IS_DMAMUX_ALL_INSTANCE(INSTANCE) ((INSTANCE) == DMAMUX1)

#define IS_DMAMUX_REQUEST_GEN_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMAMUX_RequestGenerator0) || \
                                                      ((INSTANCE) == DMAMUX_RequestGenerator1) || \
                                                      ((INSTANCE) == DMAMUX_RequestGenerator2) || \
                                                      ((INSTANCE) == DMAMUX_RequestGenerator3))

/******************************* GPIO Instances *******************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                        ((INSTANCE) == GPIOB) || \
                                        ((INSTANCE) == GPIOC) || \
                                        ((INSTANCE) == GPIOD) || \
                                        ((INSTANCE) == GPIOE) || \
                                        ((INSTANCE) == GPIOF))
/******************************* GPIO AF Instances ****************************/
#define IS_GPIO_AF_INSTANCE(INSTANCE)   IS_GPIO_ALL_INSTANCE(INSTANCE)

/**************************** GPIO Lock Instances *****************************/
#define IS_GPIO_LOCK_INSTANCE(INSTANCE) IS_GPIO_ALL_INSTANCE(INSTANCE)

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                       ((INSTANCE) == I2C2) || \
                                       ((INSTANCE) == I2C3) || \
                                       ((INSTANCE) == I2C4))
/************** I2C Instances : wakeup capability from stop modes *************/
#define IS_I2C_WAKEUP_FROMSTOP_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                                   ((INSTANCE) == I2C3))

/****************************** LTDC Instances ********************************/
#define IS_LTDC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == LTDC)

/******************************* RNG Instances ********************************/
#define IS_RNG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RNG)

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)


/******************************** SMBUS Instances *****************************/
#define IS_SMBUS_INSTANCE(INSTANCE)  (((INSTANCE) == I2C1)  || \
                                      ((INSTANCE) == I2C3))

/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || \
                                       ((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/****************** LPTIM Instances : All supported instances *****************/
#define IS_LPTIM_INSTANCE(INSTANCE)     (((INSTANCE) == LPTIM1) ||\
                                         ((INSTANCE) == LPTIM2) ||\
                                         ((INSTANCE) == LPTIM3))

/****************** LPTIM Instances : DMA supported instances *****************/
#define IS_LPTIM_DMA_INSTANCE(INSTANCE)     (((INSTANCE) == LPTIM1) ||\
                                             ((INSTANCE) == LPTIM2) ||\
                                             ((INSTANCE) == LPTIM3))

/************* LPTIM Instances : at least 1 capture/compare channel ***********/
#define IS_LPTIM_CC1_INSTANCE(INSTANCE)     (((INSTANCE) == LPTIM1) ||\
                                             ((INSTANCE) == LPTIM2) ||\
                                             ((INSTANCE) == LPTIM3))


/************* LPTIM Instances : at least 2 capture/compare channel ***********/
#define IS_LPTIM_CC2_INSTANCE(INSTANCE)     (((INSTANCE) == LPTIM1) ||\
                                             ((INSTANCE) == LPTIM2) ||\
                                             ((INSTANCE) == LPTIM3))

/************* LPTIM Instances : at least 3 capture/compare channel ***********/
#define IS_LPTIM_CC3_INSTANCE(INSTANCE)     (((INSTANCE) == LPTIM1) ||\
                                             ((INSTANCE) == LPTIM3))


/************* LPTIM Instances : at least 4 capture/compare channel ***********/
#define IS_LPTIM_CC4_INSTANCE(INSTANCE)     (((INSTANCE) == LPTIM1) ||\
                                             ((INSTANCE) == LPTIM3))

/****************** LPTIM Instances : supporting encoder interface **************/
#define IS_LPTIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)    (((INSTANCE) == LPTIM1) ||\
                                                          ((INSTANCE) == LPTIM3))

/****************** LPTIM Instances : supporting Input Capture **************/
#define IS_LPTIM_INPUT_CAPTURE_INSTANCE(INSTANCE)    (((INSTANCE) == LPTIM1) ||\
                                                      ((INSTANCE) == LPTIM2) ||\
                                                      ((INSTANCE) == LPTIM3))

/****************** TIM Instances : All supported instances *******************/
#define IS_TIM_INSTANCE(INSTANCE)       (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3)   || \
                                         ((INSTANCE) == TIM6)   || \
                                         ((INSTANCE) == TIM7)   || \
                                         ((INSTANCE) == TIM15)  || \
                                         ((INSTANCE) == TIM16))

/****************** TIM Instances : supporting 32 bits counter ****************/
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) ((INSTANCE) == TIM2)

/****************** TIM Instances : supporting the break function *************/
#define IS_TIM_BREAK_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)    || \
                                            ((INSTANCE) == TIM15)   || \
                                            ((INSTANCE) == TIM16))

/************** TIM Instances : supporting Break source selection *************/
#define IS_TIM_BREAKSOURCE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                               ((INSTANCE) == TIM15)  || \
                                               ((INSTANCE) == TIM16))

/****************** TIM Instances : supporting 2 break inputs *****************/
#define IS_TIM_BKIN2_INSTANCE(INSTANCE)    ((INSTANCE) == TIM1)

/************* TIM Instances : at least 1 capture/compare channel *************/
#define IS_TIM_CC1_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3)   || \
                                         ((INSTANCE) == TIM15)  || \
                                         ((INSTANCE) == TIM16))

/************ TIM Instances : at least 2 capture/compare channels *************/
#define IS_TIM_CC2_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3)   || \
                                         ((INSTANCE) == TIM15))

/************ TIM Instances : at least 3 capture/compare channels *************/
#define IS_TIM_CC3_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3))

/************ TIM Instances : at least 4 capture/compare channels *************/
#define IS_TIM_CC4_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3))

/****************** TIM Instances : at least 5 capture/compare channels *******/
#define IS_TIM_CC5_INSTANCE(INSTANCE)   ((INSTANCE) == TIM1)

/****************** TIM Instances : at least 6 capture/compare channels *******/
#define IS_TIM_CC6_INSTANCE(INSTANCE)   ((INSTANCE) == TIM1)

/************ TIM Instances : DMA requests generation (TIMx_DIER.COMDE) *******/
#define IS_TIM_CCDMA_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16))

/****************** TIM Instances : DMA requests generation (TIMx_DIER.UDE) ***/
#define IS_TIM_DMA_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM3)   || \
                                            ((INSTANCE) == TIM6)   || \
                                            ((INSTANCE) == TIM7)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16))

/************ TIM Instances : DMA requests generation (TIMx_DIER.CCxDE) *******/
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM3)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16))

/******************** TIM Instances : DMA burst feature ***********************/
#define IS_TIM_DMABURST_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM3)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16))

/******************* TIM Instances : output(s) available **********************/
#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
    ((((INSTANCE) == TIM1) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4) ||          \
      ((CHANNEL) == TIM_CHANNEL_5) ||          \
      ((CHANNEL) == TIM_CHANNEL_6)))           \
     ||                                        \
     (((INSTANCE) == TIM2) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
     ||                                        \
     (((INSTANCE) == TIM3) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
     ||                                        \
     (((INSTANCE) == TIM15) &&                 \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2)))           \
     ||                                        \
     (((INSTANCE) == TIM16) &&                 \
     (((CHANNEL) == TIM_CHANNEL_1))))

/****************** TIM Instances : supporting complementary output(s) ********/
#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3)))            \
    ||                                          \
    (((INSTANCE) == TIM15) &&                   \
     ((CHANNEL) == TIM_CHANNEL_1))              \
    ||                                          \
    (((INSTANCE) == TIM16) &&                   \
     ((CHANNEL) == TIM_CHANNEL_1)))

/****************** TIM Instances : supporting clock division *****************/
#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)    || \
                                                    ((INSTANCE) == TIM2)    || \
                                                    ((INSTANCE) == TIM3)    || \
                                                    ((INSTANCE) == TIM15)   || \
                                                    ((INSTANCE) == TIM16))

/****** TIM Instances : supporting external clock mode 1 for ETRF input *******/
#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3))

/****** TIM Instances : supporting external clock mode 2 for ETRF input *******/
#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3))

/****************** TIM Instances : supporting external clock mode 1 for TIX inputs*/
#define IS_TIM_CLOCKSOURCE_TIX_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting internal trigger inputs(ITRX) *******/
#define IS_TIM_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE)     (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting combined 3-phase PWM mode ******/
#define IS_TIM_COMBINED3PHASEPWM_INSTANCE(INSTANCE)    ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting commutation event generation ***/
#define IS_TIM_COMMUTATION_EVENT_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                                     ((INSTANCE) == TIM15)  || \
                                                     ((INSTANCE) == TIM16))

/****************** TIM Instances : supporting counting mode selection ********/
#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3))

/****************** TIM Instances : supporting encoder interface **************/
#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1)  || \
                                                      ((INSTANCE) == TIM2)  || \
                                                      ((INSTANCE) == TIM3))

/****************** TIM Instances : supporting Hall sensor interface **********/
#define IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                                         ((INSTANCE) == TIM2)   || \
                                                         ((INSTANCE) == TIM3))

/**************** TIM Instances : external trigger input available ************/
#define IS_TIM_ETR_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM3))

/************* TIM Instances : supporting ETR source selection ***************/
#define IS_TIM_ETRSEL_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)  || \
                                             ((INSTANCE) == TIM2)  || \
                                             ((INSTANCE) == TIM3))

/****** TIM Instances : Master mode available (TIMx_CR2.MMS available )********/
#define IS_TIM_MASTER_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM3)  || \
                                            ((INSTANCE) == TIM6)  || \
                                            ((INSTANCE) == TIM7)  || \
                                            ((INSTANCE) == TIM15))

/*********** TIM Instances : Slave mode available (TIMx_SMCR available )*******/
#define IS_TIM_SLAVE_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM3)  || \
                                            ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting OCxREF clear *******************/
#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE)        (((INSTANCE) == TIM1) || \
                                                       ((INSTANCE) == TIM2) || \
                                                       ((INSTANCE) == TIM3))

/******** TIM Instances : supporting bitfield OCCS in SMCR register ***********/
#define IS_TIM_OCCS_INSTANCE(INSTANCE)                (((INSTANCE) == TIM1)  || \
                                                       ((INSTANCE) == TIM2)  || \
                                                       ((INSTANCE) == TIM3))

/****************** TIM Instances : remapping capability **********************/
#define IS_TIM_REMAP_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM3))

/****************** TIM Instances : supporting repetition counter *************/
#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1)  || \
                                                       ((INSTANCE) == TIM15) || \
                                                       ((INSTANCE) == TIM16))

/****************** TIM Instances : supporting ADC triggering through TRGO2 ***/
#define IS_TIM_TRGO2_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1))

/******************* TIM Instances : Timer input XOR function *****************/
#define IS_TIM_XOR_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM3)   || \
                                            ((INSTANCE) == TIM15))

/******************* TIM Instances : Timer input selection ********************/
#define IS_TIM_TISEL_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3)   || \
                                         ((INSTANCE) == TIM15)  || \
                                         ((INSTANCE) == TIM16))

/************ TIM Instances : Advanced timers  ********************************/
#define IS_TIM_ADVANCED_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1))

/****************************** TSC Instances *********************************/
#define IS_TSC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == TSC)

/******************** USART Instances : Synchronous mode **********************/
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                     ((INSTANCE) == USART2) || \
                                     ((INSTANCE) == USART3) || \
                                     ((INSTANCE) == USART4))


/******************** USART Instances : SPI slave mode ************************/
#define IS_UART_SPI_SLAVE_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                              ((INSTANCE) == USART2) || \
                                              ((INSTANCE) == USART3) || \
                                              ((INSTANCE) == USART4))

/******************** UART Instances : Asynchronous mode **********************/
#define IS_UART_INSTANCE(INSTANCE) (((INSTANCE) == USART1)  || \
                                    ((INSTANCE) == USART2)  || \
                                    ((INSTANCE) == USART3)  || \
                                    ((INSTANCE) == USART4))

/******************** UART Instances : FIFO mode.******************************/
#define IS_UART_FIFO_INSTANCE(INSTANCE) (((INSTANCE) == USART1)  || \
                                         ((INSTANCE) == USART2)  || \
                                         ((INSTANCE) == LPUART1) || \
                                         ((INSTANCE) == LPUART2) || \
                                         ((INSTANCE) == LPUART3))

/****************** UART Instances : Auto Baud Rate detection *****************/
#define IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                                            ((INSTANCE) == USART2))

/*********************** UART Instances : Driver Enable ***********************/
#define IS_UART_DRIVER_ENABLE_INSTANCE(INSTANCE) (((INSTANCE) == USART1)  || \
                                                  ((INSTANCE) == USART2)  || \
                                                  ((INSTANCE) == USART3)  || \
                                                  ((INSTANCE) == USART4)  || \
                                                  ((INSTANCE) == LPUART1) || \
                                                  ((INSTANCE) == LPUART2) || \
                                                  ((INSTANCE) == LPUART3))

/********************* UART Instances : Half-Duplex mode **********************/
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE) (((INSTANCE) == USART1)  || \
                                               ((INSTANCE) == USART2)  || \
                                               ((INSTANCE) == USART3)  || \
                                               ((INSTANCE) == USART4)  || \
                                               ((INSTANCE) == LPUART1) || \
                                               ((INSTANCE) == LPUART2) || \
                                               ((INSTANCE) == LPUART3))

/******************* UART Instances : Hardware Flow control *******************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1)  || \
                                           ((INSTANCE) == USART2)  || \
                                           ((INSTANCE) == USART3)  || \
                                           ((INSTANCE) == USART4)  || \
                                           ((INSTANCE) == LPUART1) || \
                                           ((INSTANCE) == LPUART2) || \
                                           ((INSTANCE) == LPUART3))

/******************** UART Instances : Wake-up from Stop mode **********************/
#define IS_UART_WAKEUP_FROMSTOP_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                                      ((INSTANCE) == USART2) || \
                                                      ((INSTANCE) == LPUART1)|| \
                                                      ((INSTANCE) == LPUART2)|| \
                                                      ((INSTANCE) == LPUART3))

/************************* UART Instances : LIN mode **************************/
#define IS_UART_LIN_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                        ((INSTANCE) == USART2))

/************************* UART Instances : IRDA mode *************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2))

/********************* USART Instances : Smard card mode **********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                         ((INSTANCE) == USART2))

/****************************** LPUART Instance *******************************/
#define IS_LPUART_INSTANCE(INSTANCE) (((INSTANCE) == LPUART1) || \
                                      ((INSTANCE) == LPUART2) || \
                                      ((INSTANCE) == LPUART3))

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/******************************* USB DRD FS PCD Instances *************************/
#define IS_PCD_ALL_INSTANCE(INSTANCE) ((INSTANCE) == USB_DRD_FS)

/****************************** WWDG Instances ********************************/
#define IS_WWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDG)

/****************************** OPAMP Instances *******************************/
#define IS_OPAMP_ALL_INSTANCE(INSTANCE) ((INSTANCE) == OPAMP1)

#define IS_OPAMP_COMMON_INSTANCE(COMMON_INSTANCE) ((COMMON_INSTANCE) == OPAMP_COMMON)

/****************** LCD Instances : All supported instances *****************/
#define IS_LCD_ALL_INSTANCE(INSTANCE)    ((INSTANCE) == LCD)

/******************************************************************************/



/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32U073xx_H */
/**
  * @}
  */
