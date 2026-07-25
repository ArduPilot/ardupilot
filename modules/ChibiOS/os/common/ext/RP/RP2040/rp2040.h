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
 * @file    RP/rp2040.h
 * @brief   CMSIS device file for the RP2040.
 *
 * @defgroup CMSIS_RP2040 RP2040 Device File
 * @ingroup CMSIS_DEVICE
 * @{
 */
    
#ifndef RP2040_H
#define RP2040_H

#include <stdint.h>

#define __CM0PLUS_REV                   0U
#define __MPU_PRESENT                   1U
#define __VTOR_PRESENT                  1U
#define __NVIC_PRIO_BITS                2U
#define __Vendor_SysTickConfig          0U

/**
 * @brief   Interrupt vector numbers.
 */
typedef enum {
  NonMaskableInt_IRQn                   = -14,
  HardFault_IRQn                        = -13,
  SVC_IRQn                              = -5,
  PendSV_IRQn                           = -2,
  SysTick_IRQn                          = -1,
  Vector_0n                             = 0,
  Vector_1n                             = 1,
  Vector_2n                             = 2,
  Vector_3n                             = 3,
  Vector_4n                             = 4,
  Vector_5n                             = 5,
  Vector_6n                             = 6,
  Vector_7n                             = 7,
  Vector_8n                             = 8,
  Vector_9n                             = 9,
  Vector_10n                            = 10,
  Vector_11n                            = 11,
  Vector_12n                            = 12,
  Vector_13n                            = 13,
  Vector_14n                            = 14,
  Vector_15n                            = 15,
  Vector_16n                            = 16,
  Vector_17n                            = 17,
  Vector_18n                            = 18,
  Vector_19n                            = 19,
  Vector_20n                            = 20,
  Vector_21n                            = 21,
  Vector_22n                            = 22,
  Vector_23n                            = 23,
  Vector_24n                            = 24,
  Vector_25n                            = 25,
  Vector_26n                            = 26,
  Vector_27n                            = 27,
  Vector_28n                            = 28,
  Vector_29n                            = 29,
  Vector_30n                            = 30,
  Vector_31n                            = 31
} IRQn_Type;

#include "core_cm0plus.h"
#include "system_rp2040.h"

/**
 * @name    Peripheral structures
 * @{
 */
typedef struct {
  __IO uint32_t         READ_ADDR;
  __IO uint32_t         WRITE_ADDR;
  __IO uint32_t         TRANS_COUNT;
  __IO uint32_t         CTRL_TRIG;
  __IO uint32_t         AL1_CTRL;
  __IO uint32_t         AL1_READ_ADDR;
  __IO uint32_t         AL1_WRITE_ADDR;
  __IO uint32_t         AL1_TRANSFER_COUNT_TRIG;
  __IO uint32_t         AL2_CTRL;
  __IO uint32_t         AL2_TRANSFER_COUNT;
  __IO uint32_t         AL2_READ_ADDR;
  __IO uint32_t         AL2_WRITE_ADDR_TRIG;
  __IO uint32_t         AL3_CTRL;
  __IO uint32_t         AL3_WRITE_ADDR;
  __IO uint32_t         AL3_TRANSFER_COUNT;
  __IO uint32_t         AL3_READ_ADDR_TRIG;
} DMA_Channel_Typedef;

typedef struct {
  __I  uint32_t         CTDREQ;
  __I  uint32_t         TCR;
  __I  uint32_t         resvd8[14];
} DMA_Debug_Typedef;

typedef struct {
  DMA_Channel_Typedef   CH[12];
  __I  uint32_t         resvd300[64];
  __IO uint32_t         INTR;
  __IO uint32_t         INTE0;
  __IO uint32_t         INTF0;
  __IO uint32_t         INTS0;
  __I  uint32_t         resvd410;
  __IO uint32_t         INTE1;
  __IO uint32_t         INTF1;
  __IO uint32_t         INTS1;
  __IO uint32_t         TIMER[4];
  __IO uint32_t         MULTI_CHAN_TRIGGER;
  __IO uint32_t         SNIFF_CTRL;
  __IO uint32_t         SNIFF_DATA;
  __I  uint32_t         resvd43C;
  __I  uint32_t         FIFO_LEVELS;
  __IO uint32_t         CHAN_ABORT;
  __I  uint32_t         N_CHANNELS;
  __I  uint32_t         resvd44C[237];
  DMA_Debug_Typedef     CH_DBG[12];
  __I  uint32_t         resvdB00[320];
  struct {
    DMA_Channel_Typedef CH[12];
    __I  uint32_t       resvd300[64];
    __IO uint32_t       INTR;
    __IO uint32_t       INTE0;
    __IO uint32_t       INTF0;
    __IO uint32_t       INTS0;
    __I  uint32_t       resvd410;
    __IO uint32_t       INTE1;
    __IO uint32_t       INTF1;
    __IO uint32_t       INTS1;
    __IO uint32_t       TIMER[4];
    __IO uint32_t       MULTI_CHAN_TRIGGER;
    __IO uint32_t       SNIFF_CTRL;
    __IO uint32_t       SNIFF_DATA;
    __I  uint32_t       resvd43C;
    __I  uint32_t       FIFO_LEVELS;
    __IO uint32_t       CHAN_ABORT;
    __I  uint32_t       N_CHANNELS;
    __I  uint32_t       resvd44C[237];
    DMA_Debug_Typedef   CH_DBG[12];
    __I  uint32_t       resvdB00[320];
  } XOR;
  struct {
    DMA_Channel_Typedef CH[12];
    __I  uint32_t       resvd300[64];
    __IO uint32_t       INTR;
    __IO uint32_t       INTE0;
    __IO uint32_t       INTF0;
    __IO uint32_t       INTS0;
    __I  uint32_t       resvd410;
    __IO uint32_t       INTE1;
    __IO uint32_t       INTF1;
    __IO uint32_t       INTS1;
    __IO uint32_t       TIMER[4];
    __IO uint32_t       MULTI_CHAN_TRIGGER;
    __IO uint32_t       SNIFF_CTRL;
    __IO uint32_t       SNIFF_DATA;
    __I  uint32_t       resvd43C;
    __I  uint32_t       FIFO_LEVELS;
    __IO uint32_t       CHAN_ABORT;
    __I  uint32_t       N_CHANNELS;
    __I  uint32_t       resvd44C[237];
    DMA_Debug_Typedef   CH_DBG[12];
    __I  uint32_t       resvdB00[320];
  } SET;
  struct {
    DMA_Channel_Typedef CH[12];
    __I  uint32_t       resvd300[64];
    __IO uint32_t       INTR;
    __IO uint32_t       INTE0;
    __IO uint32_t       INTF0;
    __IO uint32_t       INTS0;
    __I  uint32_t       resvd410;
    __IO uint32_t       INTE1;
    __IO uint32_t       INTF1;
    __IO uint32_t       INTS1;
    __IO uint32_t       TIMER[4];
    __IO uint32_t       MULTI_CHAN_TRIGGER;
    __IO uint32_t       SNIFF_CTRL;
    __IO uint32_t       SNIFF_DATA;
    __I  uint32_t       resvd43C;
    __I  uint32_t       FIFO_LEVELS;
    __IO uint32_t       CHAN_ABORT;
    __I  uint32_t       N_CHANNELS;
    __I  uint32_t       resvd44C[237];
    DMA_Debug_Typedef   CH_DBG[12];
    __I  uint32_t       resvdB00[320];
  } CLR;
} DMA_TypeDef;

typedef struct {
  struct {
    __I  uint32_t       STATUS;
    __IO uint32_t       CTRL;
  } GPIO[30];
  __IO uint32_t         INTR[4];
  struct {
    __IO uint32_t       INTE[4];
    __IO uint32_t       INTF[4];
    __I  uint32_t       INTS[4];
  } PROC[2];
  __IO uint32_t         DW_INTE[4];
  __IO uint32_t         DW_INTF[4];
  __I  uint32_t         DW_INTS[4];
} IOUSER_TypeDef;

typedef struct {
  struct {
    __I  uint32_t       STATUS;
    __IO uint32_t       CTRL;
  } GPIO[6];
  __IO uint32_t         INTR[1];
  struct {
    __IO uint32_t       INTE[1];
    __IO uint32_t       INTF[1];
    __I  uint32_t       INTS[1];
  } PROC[2];
  __IO uint32_t         DW_INTE[1];
  __IO uint32_t         DW_INTF[1];
  __I  uint32_t         DW_INTS[1];
} IOQSPI_TypeDef;

typedef struct {
  __IO uint32_t         VOLTAGE_SELECT;
  __IO uint32_t         GPIO[30];
  __IO uint32_t         SWCLK;
  __IO uint32_t         SWD;
} PADS_TypeDef;

typedef struct {
  __IO uint32_t         FRCE_ON;
  __IO uint32_t         FRCE_OFF;
  __IO uint32_t         WDSEL;
  __IO uint32_t         DONE;
  __I  uint32_t         resvd10[1020];
  struct {
    __IO uint32_t       FRCE_ON;
    __IO uint32_t       FRCE_OFF;
    __IO uint32_t       WDSEL;
    __IO uint32_t       DONE;
    __I  uint32_t       resvd10[1020];
  } XOR;
  struct {
    __IO uint32_t       FRCE_ON;
    __IO uint32_t       FRCE_OFF;
    __IO uint32_t       WDSEL;
    __IO uint32_t       DONE;
    __I  uint32_t       resvd10[1020];
  } SET;
  struct {
    __IO uint32_t       FRCE_ON;
    __IO uint32_t       FRCE_OFF;
    __IO uint32_t       WDSEL;
    __IO uint32_t       DONE;
    __I  uint32_t       resvd10[1020];
  } CLR;
} PSM_TypeDef;

typedef struct {
  __IO uint32_t         RESET;
  __IO uint32_t         WDSEL;
  __I  uint32_t         RESET_DONE;
} RESETS_TypeDef;

typedef struct {
  __I  uint32_t         CPUID;
  __I  uint32_t         GPIO_IN;
  __I  uint32_t         GPIO_HI_IN;
  __I  uint32_t         resvdC;
  __IO uint32_t         GPIO_OUT;
  __IO uint32_t         GPIO_OUT_SET;
  __IO uint32_t         GPIO_OUT_CLR;
  __IO uint32_t         GPIO_OUT_XOR;
  __IO uint32_t         GPIO_OE;
  __IO uint32_t         GPIO_OE_SET;
  __IO uint32_t         GPIO_OE_CLR;
  __IO uint32_t         GPIO_OE_XOR;
  __IO uint32_t         GPIO_HI_OUT;
  __IO uint32_t         GPIO_HI_OUT_SET;
  __IO uint32_t         GPIO_HI_OUT_CLR;
  __IO uint32_t         GPIO_HI_OUT_XOR;
  __IO uint32_t         GPIO_HI_OE;
  __IO uint32_t         GPIO_HI_OE_SET;
  __IO uint32_t         GPIO_HI_OE_CLR;
  __IO uint32_t         GPIO_HI_OE_XOR;
  __IO uint32_t         FIFO_ST;
  __IO uint32_t         FIFO_WR;
  __I  uint32_t         FIFO_RD;
  __IO uint32_t         SPINLOCK_ST;
  __IO uint32_t         DIV_UDIVIDEND;
  __IO uint32_t         DIV_UDIVISOR;
  __IO uint32_t         DIV_SDIVIDEND;
  __IO uint32_t         DIV_SDIVISOR;
  __IO uint32_t         DIV_QUOTIENT;
  __IO uint32_t         DIV_REMAINDER;
  __I  uint32_t         DIV_CSR;
  __I  uint32_t         resvd7C;
  struct {
    __IO uint32_t       ACCUM0;
    __IO uint32_t       ACCUM1;
    __IO uint32_t       BASE0;
    __IO uint32_t       BASE1;
    __IO uint32_t       BASE2;
    __I  uint32_t       POP_LANE0;
    __I  uint32_t       POP_LANE1;
    __I  uint32_t       POP_FULL;
    __I  uint32_t       PEEK_LANE0;
    __I  uint32_t       PEEK_LANE1;
    __I  uint32_t       PEEK_FULL;
    __IO uint32_t       CTRL_LANE0;
    __IO uint32_t       CTRL_LANE1;
    __IO uint32_t       ACCUM0_ADD;
    __IO uint32_t       ACCUM1_ADD;
    __IO uint32_t       BASE_1AND0;
  } INTERP[2];
  __IO uint32_t         SPINLOCK[32];       /* NUM_SPIN_LOCKS */
} SIO_TypeDef;

typedef struct {
  __IO uint32_t         SSPCR0;
  __IO uint32_t         SSPCR1;
  __IO uint32_t         SSPDR;
  __I  uint32_t         SSPSR;
  __IO uint32_t         SSPCPSR;
  __IO uint32_t         SSPIMSC;
  __I  uint32_t         SSPRIS;
  __I  uint32_t         SSPMIS;
  __IO uint32_t         SSPICR;
  __IO uint32_t         SSPDMACR;
  __I  uint32_t         resvd28[1006];
  __I  uint32_t         SSPPERIPHID0;
  __I  uint32_t         SSPPERIPHID1;
  __I  uint32_t         SSPPERIPHID2;
  __I  uint32_t         SSPPERIPHID3;
  __I  uint32_t         SSPPCELLID0;
  __I  uint32_t         SSPPCELLID1;
  __I  uint32_t         SSPPCELLID2;
  __I  uint32_t         SSPPCELLID3;
  struct {
    __IO uint32_t       SSPCR0;
    __IO uint32_t       SSPCR1;
    __IO uint32_t       SSPDR;
    __I  uint32_t       SSPSR;
    __IO uint32_t       SSPCPSR;
    __IO uint32_t       SSPIMSC;
    __I  uint32_t       SSPRIS;
    __I  uint32_t       SSPMIS;
    __IO uint32_t       SSPICR;
    __IO uint32_t       SSPDMACR;
    __I  uint32_t       resvd28[1006];
    __I  uint32_t       SSPPERIPHID0;
    __I  uint32_t       SSPPERIPHID1;
    __I  uint32_t       SSPPERIPHID2;
    __I  uint32_t       SSPPERIPHID3;
    __I  uint32_t       SSPPCELLID0;
    __I  uint32_t       SSPPCELLID1;
    __I  uint32_t       SSPPCELLID2;
    __I  uint32_t       SSPPCELLID3;
  } XOR;
  struct {
    __IO uint32_t       SSPCR0;
    __IO uint32_t       SSPCR1;
    __IO uint32_t       SSPDR;
    __I  uint32_t       SSPSR;
    __IO uint32_t       SSPCPSR;
    __IO uint32_t       SSPIMSC;
    __I  uint32_t       SSPRIS;
    __I  uint32_t       SSPMIS;
    __IO uint32_t       SSPICR;
    __IO uint32_t       SSPDMACR;
    __I  uint32_t       resvd28[1006];
    __I  uint32_t       SSPPERIPHID0;
    __I  uint32_t       SSPPERIPHID1;
    __I  uint32_t       SSPPERIPHID2;
    __I  uint32_t       SSPPERIPHID3;
    __I  uint32_t       SSPPCELLID0;
    __I  uint32_t       SSPPCELLID1;
    __I  uint32_t       SSPPCELLID2;
    __I  uint32_t       SSPPCELLID3;
  } SET;
  struct {
    __IO uint32_t       SSPCR0;
    __IO uint32_t       SSPCR1;
    __IO uint32_t       SSPDR;
    __I  uint32_t       SSPSR;
    __IO uint32_t       SSPCPSR;
    __IO uint32_t       SSPIMSC;
    __I  uint32_t       SSPRIS;
    __I  uint32_t       SSPMIS;
    __IO uint32_t       SSPICR;
    __IO uint32_t       SSPDMACR;
    __I  uint32_t       resvd28[1006];
    __I  uint32_t       SSPPERIPHID0;
    __I  uint32_t       SSPPERIPHID1;
    __I  uint32_t       SSPPERIPHID2;
    __I  uint32_t       SSPPERIPHID3;
    __I  uint32_t       SSPPCELLID0;
    __I  uint32_t       SSPPCELLID1;
    __I  uint32_t       SSPPCELLID2;
    __I  uint32_t       SSPPCELLID3;
  } CLR;
} SPI_TypeDef;

typedef struct {
  __IO uint32_t         TIMEHW;
  __IO uint32_t         TIMELW;
  __I  uint32_t         TIMEHR;
  __I  uint32_t         TIMELR;
  __IO uint32_t         ALARM[4];           /* NUM_TIMERS */
  __IO uint32_t         ARMED;
  __I  uint32_t         TIMERAWH;
  __I  uint32_t         TIMERAWL;
  __IO uint32_t         DBGPAUSE;
  __IO uint32_t         PAUSE;
  __IO uint32_t         INTR;
  __IO uint32_t         INTE;
  __IO uint32_t         INTF;
  __I  uint32_t         INTS;
  __I  uint32_t         resvd44[1007];
  struct {
    __IO uint32_t         TIMEHW;
    __IO uint32_t         TIMELW;
    __I  uint32_t         TIMEHR;
    __I  uint32_t         TIMELR;
    __IO uint32_t         ALARM[4];           /* NUM_TIMERS */
    __IO uint32_t         ARMED;
    __I  uint32_t         TIMERAWH;
    __I  uint32_t         TIMERAWL;
    __IO uint32_t         DBGPAUSE;
    __IO uint32_t         PAUSE;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __I  uint32_t         INTS;
    __I  uint32_t         resvd44[1007];
  } XOR;
  struct {
    __IO uint32_t         TIMEHW;
    __IO uint32_t         TIMELW;
    __I  uint32_t         TIMEHR;
    __I  uint32_t         TIMELR;
    __IO uint32_t         ALARM[4];           /* NUM_TIMERS */
    __IO uint32_t         ARMED;
    __I  uint32_t         TIMERAWH;
    __I  uint32_t         TIMERAWL;
    __IO uint32_t         DBGPAUSE;
    __IO uint32_t         PAUSE;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __I  uint32_t         INTS;
    __I  uint32_t         resvd44[1007];
  } SET;
  struct {
    __IO uint32_t         TIMEHW;
    __IO uint32_t         TIMELW;
    __I  uint32_t         TIMEHR;
    __I  uint32_t         TIMELR;
    __IO uint32_t         ALARM[4];           /* NUM_TIMERS */
    __IO uint32_t         ARMED;
    __I  uint32_t         TIMERAWH;
    __I  uint32_t         TIMERAWL;
    __IO uint32_t         DBGPAUSE;
    __IO uint32_t         PAUSE;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __I  uint32_t         INTS;
    __I  uint32_t         resvd44[1007];
  } CLR;
} TIMER_TypeDef;

typedef struct {
  __IO uint32_t         UARTDR;
  __IO uint32_t         UARTRSR;
  __I  uint32_t         resvd8;
  __I  uint32_t         resvdC;
  __I  uint32_t         resvd10;
  __I  uint32_t         resvd14;
  __I  uint32_t         UARTFR;
  __I  uint32_t         resvd1C;
  __IO uint32_t         UARTILPR;
  __IO uint32_t         UARTIBRD;
  __IO uint32_t         UARTFBRD;
  __IO uint32_t         UARTLCR_H;
  __IO uint32_t         UARTCR;
  __IO uint32_t         UARTIFLS;
  __IO uint32_t         UARTIMSC;
  __I  uint32_t         UARTRIS;
  __I  uint32_t         UARTMIS;
  __IO uint32_t         UARTICR;
  __IO uint32_t         UARTDMACR;
  __I  uint32_t         resvd4C[997];
  __I  uint32_t         UARTPERIPHID0;
  __I  uint32_t         UARTPERIPHID1;
  __I  uint32_t         UARTPERIPHID2;
  __I  uint32_t         UARTPERIPHID3;
  __I  uint32_t         UARTPCELLID0;
  __I  uint32_t         UARTPCELLID1;
  __I  uint32_t         UARTPCELLID2;
  __I  uint32_t         UARTPCELLID3;
  struct {
    __IO uint32_t       UARTDR;
    __IO uint32_t       UARTRSR;
    __I  uint32_t       resvd8;
    __I  uint32_t       resvdC;
    __I  uint32_t       resvd10;
    __I  uint32_t       resvd14;
    __I  uint32_t       UARTFR;
    __I  uint32_t       resvd1C;
    __IO uint32_t       UARTILPR;
    __IO uint32_t       UARTIBRD;
    __IO uint32_t       UARTFBRD;
    __IO uint32_t       UARTLCR_H;
    __IO uint32_t       UARTCR;
    __IO uint32_t       UARTIFLS;
    __IO uint32_t       UARTIMSC;
    __I  uint32_t       UARTRIS;
    __I  uint32_t       UARTMIS;
    __IO uint32_t       UARTICR;
    __IO uint32_t       UARTDMACR;
    __I  uint32_t       resvd4C[997];
    __I  uint32_t       UARTPERIPHID0;
    __I  uint32_t       UARTPERIPHID1;
    __I  uint32_t       UARTPERIPHID2;
    __I  uint32_t       UARTPERIPHID3;
    __I  uint32_t       UARTPCELLID0;
    __I  uint32_t       UARTPCELLID1;
    __I  uint32_t       UARTPCELLID2;
    __I  uint32_t       UARTPCELLID3;
  } XOR;
  struct {
    __IO uint32_t       UARTDR;
    __IO uint32_t       UARTRSR;
    __I  uint32_t       resvd8;
    __I  uint32_t       resvdC;
    __I  uint32_t       resvd10;
    __I  uint32_t       resvd14;
    __I  uint32_t       UARTFR;
    __I  uint32_t       resvd1C;
    __IO uint32_t       UARTILPR;
    __IO uint32_t       UARTIBRD;
    __IO uint32_t       UARTFBRD;
    __IO uint32_t       UARTLCR_H;
    __IO uint32_t       UARTCR;
    __IO uint32_t       UARTIFLS;
    __IO uint32_t       UARTIMSC;
    __I  uint32_t       UARTRIS;
    __I  uint32_t       UARTMIS;
    __IO uint32_t       UARTICR;
    __IO uint32_t       UARTDMACR;
    __I  uint32_t       resvd4C[997];
    __I  uint32_t       UARTPERIPHID0;
    __I  uint32_t       UARTPERIPHID1;
    __I  uint32_t       UARTPERIPHID2;
    __I  uint32_t       UARTPERIPHID3;
    __I  uint32_t       UARTPCELLID0;
    __I  uint32_t       UARTPCELLID1;
    __I  uint32_t       UARTPCELLID2;
    __I  uint32_t       UARTPCELLID3;
  } SET;
  struct {
    __IO uint32_t       UARTDR;
    __IO uint32_t       UARTRSR;
    __I  uint32_t       resvd8;
    __I  uint32_t       resvdC;
    __I  uint32_t       resvd10;
    __I  uint32_t       resvd14;
    __I  uint32_t       UARTFR;
    __I  uint32_t       resvd1C;
    __IO uint32_t       UARTILPR;
    __IO uint32_t       UARTIBRD;
    __IO uint32_t       UARTFBRD;
    __IO uint32_t       UARTLCR_H;
    __IO uint32_t       UARTCR;
    __IO uint32_t       UARTIFLS;
    __IO uint32_t       UARTIMSC;
    __I  uint32_t       UARTRIS;
    __I  uint32_t       UARTMIS;
    __IO uint32_t       UARTICR;
    __IO uint32_t       UARTDMACR;
    __I  uint32_t       resvd4C[997];
    __I  uint32_t       UARTPERIPHID0;
    __I  uint32_t       UARTPERIPHID1;
    __I  uint32_t       UARTPERIPHID2;
    __I  uint32_t       UARTPERIPHID3;
    __I  uint32_t       UARTPCELLID0;
    __I  uint32_t       UARTPCELLID1;
    __I  uint32_t       UARTPCELLID2;
    __I  uint32_t       UARTPCELLID3;
  } CLR;
} UART_TypeDef;

typedef struct {
  __IO uint32_t         CLKDIVM1;
  __IO uint32_t         SETUP0;
  __IO uint32_t         SETUP1;
  __IO uint32_t         CTRL;
  __IO uint32_t         IRQSETUP0;
  __IO uint32_t         IRQSETUP1;
  __IO uint32_t         RTC1;
  __IO uint32_t         RTC0;
  __IO uint32_t         INTR;
  __IO uint32_t         INTE;
  __IO uint32_t         INTF;
  __IO uint32_t         INTS;
  __I  uint32_t         resvd30[1012];
  struct {
    __IO uint32_t         CLKDIVM1;
    __IO uint32_t         SETUP0;
    __IO uint32_t         SETUP1;
    __IO uint32_t         CTRL;
    __IO uint32_t         IRQSETUP0;
    __IO uint32_t         IRQSETUP1;
    __IO uint32_t         RTC1;
    __IO uint32_t         RTC0;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __IO uint32_t         INTS;
    __I  uint32_t         resvd30[1012];
  } XOR;
  struct {
    __IO uint32_t         CLKDIVM1;
    __IO uint32_t         SETUP0;
    __IO uint32_t         SETUP1;
    __IO uint32_t         CTRL;
    __IO uint32_t         IRQSETUP0;
    __IO uint32_t         IRQSETUP1;
    __IO uint32_t         RTC1;
    __IO uint32_t         RTC0;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __IO uint32_t         INTS;
    __I  uint32_t         resvd30[1012];
  } SET;
  struct {
    __IO uint32_t         CLKDIVM1;
    __IO uint32_t         SETUP0;
    __IO uint32_t         SETUP1;
    __IO uint32_t         CTRL;
    __IO uint32_t         IRQSETUP0;
    __IO uint32_t         IRQSETUP1;
    __IO uint32_t         RTC1;
    __IO uint32_t         RTC0;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __IO uint32_t         INTS;
    __I  uint32_t         resvd30[1012];
  } CLR;
} RTC_TypeDef;

typedef struct {
  __IO uint32_t         CON;
  __IO uint32_t         TAR;
  __IO uint32_t         SAR;
  __I  uint32_t         resvdC;
  __IO uint32_t         DATACMD;
  __IO uint32_t         SSSCLHCNT;
  __IO uint32_t         SSSCLLCNT;
  __IO uint32_t         FSSCLHCNT;
  __IO uint32_t         FSSCLLCNT;
  __I  uint32_t         resvd24[2];
  __IO uint32_t         INTRSTAT;
  __IO uint32_t         INTRMASK;
  __IO uint32_t         RAWINTRSTAT;
  __IO uint32_t         RXTL;
  __IO uint32_t         TXTL;
  __IO uint32_t         CLRINTR;
  __IO uint32_t         CLRRXUNDER;
  __IO uint32_t         CLRRXOVER;
  __IO uint32_t         CLRTXOVER;
  __IO uint32_t         CLRRDREQ;
  __IO uint32_t         CLRTXABRT;
  __IO uint32_t         CLRRXDONE;
  __IO uint32_t         CLRACTIVITY;
  __IO uint32_t         CLRSTOPDET;
  __IO uint32_t         CLRSTARTDET;
  __IO uint32_t         CLRGENCALL;
  __IO uint32_t         ENABLE;
  __IO uint32_t         STATUS;
  __IO uint32_t         TXFLR;
  __IO uint32_t         RXFLR;
  __IO uint32_t         SDAHOLD;
  __IO uint32_t         TXABRTSOURCE;
  __IO uint32_t         SLVDATANACKONLY;
  __IO uint32_t         DMACR;
  __IO uint32_t         DMATDLR;
  __IO uint32_t         DMARDLR;
  __IO uint32_t         SDASETUP;
  __IO uint32_t         ACKGENERALCALL;
  __IO uint32_t         ENABLESTATUS;
  __IO uint32_t         FSSPKLEN;
  __I  uint32_t         resvdA4;
  __IO uint32_t         CLRRESTARTDET;
  __I  uint32_t         resvdac[981];
  struct {
    __IO uint32_t         CON;
    __IO uint32_t         TAR;
    __IO uint32_t         SAR;
    __I  uint32_t         resvdC;
    __IO uint32_t         DATACMD;
    __IO uint32_t         SSSCLHCNT;
    __IO uint32_t         SSSCLLCNT;
    __IO uint32_t         FSSCLHCNT;
    __IO uint32_t         FSSCLLCNT;
    __I  uint32_t         resvd24[2];
    __IO uint32_t         INTRSTAT;
    __IO uint32_t         INTRMASK;
    __IO uint32_t         RAWINTRSTAT;
    __IO uint32_t         RXTL;
    __IO uint32_t         TXTL;
    __IO uint32_t         CLRINTR;
    __IO uint32_t         CLRRXUNDER;
    __IO uint32_t         CLRRXOVER;
    __IO uint32_t         CLRTXOVER;
    __IO uint32_t         CLRRDREQ;
    __IO uint32_t         CLRTXABRT;
    __IO uint32_t         CLRRXDONE;
    __IO uint32_t         CLRACTIVITY;
    __IO uint32_t         CLRSTOPDET;
    __IO uint32_t         CLRSTARTDET;
    __IO uint32_t         CLRGENCALL;
    __IO uint32_t         ENABLE;
    __IO uint32_t         STATUS;
    __IO uint32_t         TXFLR;
    __IO uint32_t         RXFLR;
    __IO uint32_t         SDAHOLD;
    __IO uint32_t         TXABRTSOURCE;
    __IO uint32_t         SLVDATANACKONLY;
    __IO uint32_t         DMACR;
    __IO uint32_t         DMATDLR;
    __IO uint32_t         DMARDLR;
    __IO uint32_t         SDASETUP;
    __IO uint32_t         ACKGENERALCALL;
    __IO uint32_t         ENABLESTATUS;
    __IO uint32_t         FSSPKLEN;
    __I  uint32_t         resvdA4;
    __IO uint32_t         CLRRESTARTDET;
    __I  uint32_t         resvdac[981];
  } XOR;
  struct {
    __IO uint32_t         CON;
    __IO uint32_t         TAR;
    __IO uint32_t         SAR;
    __I  uint32_t         resvdC;
    __IO uint32_t         DATACMD;
    __IO uint32_t         SSSCLHCNT;
    __IO uint32_t         SSSCLLCNT;
    __IO uint32_t         FSSCLHCNT;
    __IO uint32_t         FSSCLLCNT;
    __I  uint32_t         resvd24[2];
    __IO uint32_t         INTRSTAT;
    __IO uint32_t         INTRMASK;
    __IO uint32_t         RAWINTRSTAT;
    __IO uint32_t         RXTL;
    __IO uint32_t         TXTL;
    __IO uint32_t         CLRINTR;
    __IO uint32_t         CLRRXUNDER;
    __IO uint32_t         CLRRXOVER;
    __IO uint32_t         CLRTXOVER;
    __IO uint32_t         CLRRDREQ;
    __IO uint32_t         CLRTXABRT;
    __IO uint32_t         CLRRXDONE;
    __IO uint32_t         CLRACTIVITY;
    __IO uint32_t         CLRSTOPDET;
    __IO uint32_t         CLRSTARTDET;
    __IO uint32_t         CLRGENCALL;
    __IO uint32_t         ENABLE;
    __IO uint32_t         STATUS;
    __IO uint32_t         TXFLR;
    __IO uint32_t         RXFLR;
    __IO uint32_t         SDAHOLD;
    __IO uint32_t         TXABRTSOURCE;
    __IO uint32_t         SLVDATANACKONLY;
    __IO uint32_t         DMACR;
    __IO uint32_t         DMATDLR;
    __IO uint32_t         DMARDLR;
    __IO uint32_t         SDASETUP;
    __IO uint32_t         ACKGENERALCALL;
    __IO uint32_t         ENABLESTATUS;
    __IO uint32_t         FSSPKLEN;
    __I  uint32_t         resvdA4;
    __IO uint32_t         CLRRESTARTDET;
    __I  uint32_t         resvdac[981];
  } SET;
  struct {
    __IO uint32_t         CON;
    __IO uint32_t         TAR;
    __IO uint32_t         SAR;
    __I  uint32_t         resvdC;
    __IO uint32_t         DATACMD;
    __IO uint32_t         SSSCLHCNT;
    __IO uint32_t         SSSCLLCNT;
    __IO uint32_t         FSSCLHCNT;
    __IO uint32_t         FSSCLLCNT;
    __I  uint32_t         resvd24[2];
    __IO uint32_t         INTRSTAT;
    __IO uint32_t         INTRMASK;
    __IO uint32_t         RAWINTRSTAT;
    __IO uint32_t         RXTL;
    __IO uint32_t         TXTL;
    __IO uint32_t         CLRINTR;
    __IO uint32_t         CLRRXUNDER;
    __IO uint32_t         CLRRXOVER;
    __IO uint32_t         CLRTXOVER;
    __IO uint32_t         CLRRDREQ;
    __IO uint32_t         CLRTXABRT;
    __IO uint32_t         CLRRXDONE;
    __IO uint32_t         CLRACTIVITY;
    __IO uint32_t         CLRSTOPDET;
    __IO uint32_t         CLRSTARTDET;
    __IO uint32_t         CLRGENCALL;
    __IO uint32_t         ENABLE;
    __IO uint32_t         STATUS;
    __IO uint32_t         TXFLR;
    __IO uint32_t         RXFLR;
    __IO uint32_t         SDAHOLD;
    __IO uint32_t         TXABRTSOURCE;
    __IO uint32_t         SLVDATANACKONLY;
    __IO uint32_t         DMACR;
    __IO uint32_t         DMATDLR;
    __IO uint32_t         DMARDLR;
    __IO uint32_t         SDASETUP;
    __IO uint32_t         ACKGENERALCALL;
    __IO uint32_t         ENABLESTATUS;
    __IO uint32_t         FSSPKLEN;
    __I  uint32_t         resvdA4;
    __IO uint32_t         CLRRESTARTDET;
    __I  uint32_t         resvdac[981];
  } CLR;
} I2C_TypeDef;

typedef struct {
  __IO uint32_t         CS;
  __IO uint32_t         RESULT;
  __IO uint32_t         FCS;
  __IO uint32_t         FIFO;
  __IO uint32_t         DIV;
  __IO uint32_t         INTR;
  __IO uint32_t         INTE;
  __IO uint32_t         INTF;
  __IO uint32_t         INTS;
  __I  uint32_t         resvd24[1015];
struct {
    __IO uint32_t         CS;
    __IO uint32_t         RESULT;
    __IO uint32_t         FCS;
    __IO uint32_t         FIFO;
    __IO uint32_t         DIV;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __IO uint32_t         INTS;
    __I  uint32_t         resvd24[1015];
  } XOR;
struct {
    __IO uint32_t         CS;
    __IO uint32_t         RESULT;
    __IO uint32_t         FCS;
    __IO uint32_t         FIFO;
    __IO uint32_t         DIV;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __IO uint32_t         INTS;
    __I  uint32_t         resvd24[1015];
  } SET;
struct {
    __IO uint32_t         CS;
    __IO uint32_t         RESULT;
    __IO uint32_t         FCS;
    __IO uint32_t         FIFO;
    __IO uint32_t         DIV;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __IO uint32_t         INTS;
    __I  uint32_t         resvd24[1015];
  } CLR;
} ADC_TypeDef;

typedef struct {
  struct {
    __IO uint32_t       CSR;
    __IO uint32_t       DIV;
    __IO uint32_t       CTR;
    __IO uint32_t       CC;
    __IO uint32_t       TOP;
  } CH[8];
  __IO uint32_t         EN;
  __IO uint32_t         INTR;
  __IO uint32_t         INTE;
  __IO uint32_t         INTF;
  __I  uint32_t         INTS;
  __I  uint32_t         resvdpwm[979];
  struct {
    struct {
      __IO uint32_t     CSR;
      __IO uint32_t     DIV;
      __IO uint32_t     CTR;
      __IO uint32_t     CC;
      __IO uint32_t     TOP;
    } CH[8];
    __IO uint32_t       EN;
    __IO uint32_t       INTR;
    __IO uint32_t       INTE;
    __IO uint32_t       INTF;
    __I  uint32_t       INTS;
    __I  uint32_t       resvdpwm[979];
  } XOR;
  struct {
    struct {
      __IO uint32_t     CSR;
      __IO uint32_t     DIV;
      __IO uint32_t     CTR;
      __IO uint32_t     CC;
      __IO uint32_t     TOP;
    } CH[8];
    __IO uint32_t       EN;
    __IO uint32_t       INTR;
    __IO uint32_t       INTE;
    __IO uint32_t       INTF;
    __I  uint32_t       INTS;
    __I  uint32_t       resvdpwm[979];
  } SET;
  struct {
    struct {
      __IO uint32_t     CSR;
      __IO uint32_t     DIV;
      __IO uint32_t     CTR;
      __IO uint32_t     CC;
      __IO uint32_t     TOP;
    } CH[8];
    __IO uint32_t       EN;
    __IO uint32_t       INTR;
    __IO uint32_t       INTE;
    __IO uint32_t       INTF;
    __I  uint32_t       INTS;
    __I  uint32_t       resvdpwm[979];
  } CLR;
} PWM_TypeDef;

typedef struct {
  __IO uint32_t         DEVADDRCTRL;
  __IO uint32_t         INTEPADDRCTRL[15]; /* USB_HOST_INTERRUPT_ENDPOINTS */
  __IO uint32_t         MAINCTRL;
  __IO uint32_t         SOFRW;
  __I  uint32_t         SOFRD;
  __IO uint32_t         SIECTRL;
  __IO uint32_t         SIESTATUS;
  __IO uint32_t         INTEPCTRL;
  __IO uint32_t         BUFSTATUS;
  __IO uint32_t         BUFCPUSHOULDHANDLE;
  __IO uint32_t         ABORT;
  __IO uint32_t         ABORTDONE;
  __IO uint32_t         EPSTALLARM;
  __IO uint32_t         NAKPOLL;
  __IO uint32_t         EPNAKSTALLSTATUS;
  __IO uint32_t         MUXING;
  __IO uint32_t         PWR;
  __IO uint32_t         PHYDIRECT;
  __IO uint32_t         PHYDIRECTOVERRIDE;
  __IO uint32_t         PHYTRIM;
  __IO uint32_t         LINESTATETUNING;
  __IO uint32_t         INTR;
  __IO uint32_t         INTE;
  __IO uint32_t         INTF;
  __IO uint32_t         INTS;
  __I  uint32_t         resvd9c[985];
  struct {
    __IO uint32_t         DEVADDRCTRL;
    __IO uint32_t         INTEPADDRCTRL[15]; /* USB_HOST_INTERRUPT_ENDPOINTS */
    __IO uint32_t         MAINCTRL;
    __IO uint32_t         SOFRW;
    __I  uint32_t         SOFRD;
    __IO uint32_t         SIECTRL;
    __IO uint32_t         SIESTATUS;
    __IO uint32_t         INTEPCTRL;
    __IO uint32_t         BUFSTATUS;
    __IO uint32_t         BUFCPUSHOULDHANDLE;
    __IO uint32_t         ABORT;
    __IO uint32_t         ABORTDONE;
    __IO uint32_t         EPSTALLARM;
    __IO uint32_t         NAKPOLL;
    __IO uint32_t         EPNAKSTALLSTATUS;
    __IO uint32_t         MUXING;
    __IO uint32_t         PWR;
    __IO uint32_t         PHYDIRECT;
    __IO uint32_t         PHYDIRECTOVERRIDE;
    __IO uint32_t         PHYTRIM;
    __IO uint32_t         LINESTATETUNING;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __IO uint32_t         INTS;
    __I  uint32_t         resvd9c[985];
  } XOR;
  struct {
    __IO uint32_t         DEVADDRCTRL;
    __IO uint32_t         INTEPADDRCTRL[15]; /* USB_HOST_INTERRUPT_ENDPOINTS */
    __IO uint32_t         MAINCTRL;
    __IO uint32_t         SOFRW;
    __I  uint32_t         SOFRD;
    __IO uint32_t         SIECTRL;
    __IO uint32_t         SIESTATUS;
    __IO uint32_t         INTEPCTRL;
    __IO uint32_t         BUFSTATUS;
    __IO uint32_t         BUFCPUSHOULDHANDLE;
    __IO uint32_t         ABORT;
    __IO uint32_t         ABORTDONE;
    __IO uint32_t         EPSTALLARM;
    __IO uint32_t         NAKPOLL;
    __IO uint32_t         EPNAKSTALLSTATUS;
    __IO uint32_t         MUXING;
    __IO uint32_t         PWR;
    __IO uint32_t         PHYDIRECT;
    __IO uint32_t         PHYDIRECTOVERRIDE;
    __IO uint32_t         PHYTRIM;
    __IO uint32_t         LINESTATETUNING;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __IO uint32_t         INTS;
    __I  uint32_t         resvd9c[985];
  } SET;
  struct {
    __IO uint32_t         DEVADDRCTRL;
    __IO uint32_t         INTEPADDRCTRL[15]; /* USB_HOST_INTERRUPT_ENDPOINTS */
    __IO uint32_t         MAINCTRL;
    __IO uint32_t         SOFRW;
    __I  uint32_t         SOFRD;
    __IO uint32_t         SIECTRL;
    __IO uint32_t         SIESTATUS;
    __IO uint32_t         INTEPCTRL;
    __IO uint32_t         BUFSTATUS;
    __IO uint32_t         BUFCPUSHOULDHANDLE;
    __IO uint32_t         ABORT;
    __IO uint32_t         ABORTDONE;
    __IO uint32_t         EPSTALLARM;
    __IO uint32_t         NAKPOLL;
    __IO uint32_t         EPNAKSTALLSTATUS;
    __IO uint32_t         MUXING;
    __IO uint32_t         PWR;
    __IO uint32_t         PHYDIRECT;
    __IO uint32_t         PHYDIRECTOVERRIDE;
    __IO uint32_t         PHYTRIM;
    __IO uint32_t         LINESTATETUNING;
    __IO uint32_t         INTR;
    __IO uint32_t         INTE;
    __IO uint32_t         INTF;
    __IO uint32_t         INTS;
    __I  uint32_t         resvd9c[985];
  } CLR;
} USB_TypeDef;

typedef struct {
  __IO uint32_t         CTRL;
  __O  uint32_t         LOAD;
  __I  uint32_t         REASON;
  __IO uint32_t         SCRATCH[8];
  __IO uint32_t         TICK;
  __I  uint32_t         resvd30[1012];
  struct {
    __IO uint32_t         CTRL;
    __O  uint32_t         LOAD;
    __I  uint32_t         REASON;
    __IO uint32_t         SCRATCH[8];
    __IO uint32_t         TICK;
    __I  uint32_t         resvd30[1012];
  } XOR;
  struct {
    __IO uint32_t         CTRL;
    __O  uint32_t         LOAD;
    __I  uint32_t         REASON;
    __IO uint32_t         SCRATCH[8];
    __IO uint32_t         TICK;
    __I  uint32_t         resvd30[1012];
  } SET;
  struct {
    __IO uint32_t         CTRL;
    __O  uint32_t         LOAD;
    __I  uint32_t         REASON;
    __IO uint32_t         SCRATCH[8];
    __IO uint32_t         TICK;
    __I  uint32_t         resvd30[1012];
  } CLR;
} WATCHDOG_TypeDef;

typedef struct {
  __IO uint32_t         CTRL;
  __I  uint32_t         FSTAT;
  __IO uint32_t         FDEBUG;
  __I  uint32_t         FLEVEL;
  __O  uint32_t         TXF[4];            /* NUM_PIO_STATE_MACHINES */
  __I  uint32_t         RXF[4];            /* NUM_PIO_STATE_MACHINES */
  __IO uint32_t         IRQ;
  __O  uint32_t         IRQFORCE;
  __IO uint32_t         INPUTSYNCBYPASS;
  __IO uint32_t         DBGPADOUT;
  __IO uint32_t         DBGPADOE;
  __IO uint32_t         DBGCFGINFO;
  __O  uint32_t         INSTRMEM[32];
  struct {
    __IO uint32_t       CLKDIV;
    __IO uint32_t       EXECCTRL;
    __IO uint32_t       SHIFTCTRL;
    __I  uint32_t       ADDR;
    __IO uint32_t       INSTR;
    __IO uint32_t       PINCTRL;
  } SM[4];                                /* NUM_PIO_STATE_MACHINES */
  __IO uint32_t         INTR;
  __IO uint32_t         INTE0;
  __IO uint32_t         INTF0;
  __I  uint32_t         INTS0;
  __IO uint32_t         INTE1;
  __IO uint32_t         INTF1;
  __I  uint32_t         INTS1;
  __I  uint32_t         resvd144[943];
  struct {
  __IO uint32_t         CTRL;
  __I  uint32_t         FSTAT;
  __IO uint32_t         FDEBUG;
  __I  uint32_t         FLEVEL;
  __O  uint32_t         TXF[4];            /* NUM_PIO_STATE_MACHINES */
  __I  uint32_t         RXF[4];            /* NUM_PIO_STATE_MACHINES */
  __IO uint32_t         IRQ;
  __O  uint32_t         IRQFORCE;
  __IO uint32_t         INPUTSYNCBYPASS;
  __IO uint32_t         DBGPADOUT;
  __IO uint32_t         DBGPADOE;
  __IO uint32_t         DBGCFGINFO;
  __O  uint32_t         INSTRMEM[32];
  struct {
    __IO uint32_t       CLKDIV;
    __IO uint32_t       EXECCTRL;
    __IO uint32_t       SHIFTCTRL;
    __I  uint32_t       ADDR;
    __IO uint32_t       INSTR;
    __IO uint32_t       PINCTRL;
  } SM[4];                                /* NUM_PIO_STATE_MACHINES */
  __IO uint32_t         INTR;
  __IO uint32_t         INTE0;
  __IO uint32_t         INTF0;
  __I  uint32_t         INTS0;
  __IO uint32_t         INTE1;
  __IO uint32_t         INTF1;
  __I  uint32_t         INTS1;
  __I  uint32_t         resvd144[943];
  } XOR;
  struct {
  __IO uint32_t         CTRL;
  __I  uint32_t         FSTAT;
  __IO uint32_t         FDEBUG;
  __I  uint32_t         FLEVEL;
  __O  uint32_t         TXF[4];            /* NUM_PIO_STATE_MACHINES */
  __I  uint32_t         RXF[4];            /* NUM_PIO_STATE_MACHINES */
  __IO uint32_t         IRQ;
  __O  uint32_t         IRQFORCE;
  __IO uint32_t         INPUTSYNCBYPASS;
  __IO uint32_t         DBGPADOUT;
  __IO uint32_t         DBGPADOE;
  __IO uint32_t         DBGCFGINFO;
  __O  uint32_t         INSTRMEM[32];
  struct {
    __IO uint32_t       CLKDIV;
    __IO uint32_t       EXECCTRL;
    __IO uint32_t       SHIFTCTRL;
    __I  uint32_t       ADDR;
    __IO uint32_t       INSTR;
    __IO uint32_t       PINCTRL;
  } SM[4];                                /* NUM_PIO_STATE_MACHINES */
  __IO uint32_t         INTR;
  __IO uint32_t         INTE0;
  __IO uint32_t         INTF0;
  __I  uint32_t         INTS0;
  __IO uint32_t         INTE1;
  __IO uint32_t         INTF1;
  __I  uint32_t         INTS1;
  __I  uint32_t         resvd144[943];
  } SET;
  struct {
  __IO uint32_t         CTRL;
  __I  uint32_t         FSTAT;
  __IO uint32_t         FDEBUG;
  __I  uint32_t         FLEVEL;
  __O  uint32_t         TXF[4];            /* NUM_PIO_STATE_MACHINES */
  __I  uint32_t         RXF[4];            /* NUM_PIO_STATE_MACHINES */
  __IO uint32_t         IRQ;
  __O  uint32_t         IRQFORCE;
  __IO uint32_t         INPUTSYNCBYPASS;
  __IO uint32_t         DBGPADOUT;
  __IO uint32_t         DBGPADOE;
  __IO uint32_t         DBGCFGINFO;
  __O  uint32_t         INSTRMEM[32];
  struct {
    __IO uint32_t       CLKDIV;
    __IO uint32_t       EXECCTRL;
    __IO uint32_t       SHIFTCTRL;
    __I  uint32_t       ADDR;
    __IO uint32_t       INSTR;
    __IO uint32_t       PINCTRL;
  } SM[4];                                /* NUM_PIO_STATE_MACHINES */
  __IO uint32_t         INTR;
  __IO uint32_t         INTE0;
  __IO uint32_t         INTF0;
  __I  uint32_t         INTS0;
  __IO uint32_t         INTE1;
  __IO uint32_t         INTF1;
  __I  uint32_t         INTS1;
  __I  uint32_t         resvd144[943];
  } CLR;
} PIO_TypeDef;
/** @} */

/**
 * @name    Base addresses
 * @{
 */
#define __APBPERIPH_BASE                  0x40000000U
#define __AHBPERIPH_BASE                  0x50000000U
#define __IOPORT_BASE                     0xD0000000U

#define __RESETS_BASE                     (__APBPERIPH_BASE + 0x0000C000U)
#define __IOUSER0_BASE                    (__APBPERIPH_BASE + 0x00014000U)
#define __IOQSPI_BASE                     (__APBPERIPH_BASE + 0x00018000U)
#define __PADSUSER0_BASE                  (__APBPERIPH_BASE + 0x0001C000U)
#define __PADSQSPI_BASE                   (__APBPERIPH_BASE + 0x00020000U)
#define __PSM_BASE                        (__APBPERIPH_BASE + 0x00010000U)
#define __TIMER_BASE                      (__APBPERIPH_BASE + 0x00054000U)
#define __UART0_BASE                      (__APBPERIPH_BASE + 0x00034000U)
#define __UART1_BASE                      (__APBPERIPH_BASE + 0x00038000U)
#define __SPI0_BASE                       (__APBPERIPH_BASE + 0x0003C000U)
#define __SPI1_BASE                       (__APBPERIPH_BASE + 0x00040000U)
#define __I2C0_BASE                       (__APBPERIPH_BASE + 0x00044000U)
#define __I2C1_BASE                       (__APBPERIPH_BASE + 0x00048000U)
#define __ADC_BASE                        (__APBPERIPH_BASE + 0x0004C000U)
#define __PWM_BASE                        (__APBPERIPH_BASE + 0x00050000U)
#define __WATCHDOG_BASE                   (__APBPERIPH_BASE + 0x00058000U)
#define __RTC_BASE                        (__APBPERIPH_BASE + 0x0005C000U)

#define __DMA_BASE                        (__AHBPERIPH_BASE + 0x00000000U)
#define __USB_BASE                        (__AHBPERIPH_BASE + 0x00110000U)
#define __PIO0_BASE                       (__AHBPERIPH_BASE + 0x00200000U)
#define __PIO1_BASE                       (__AHBPERIPH_BASE + 0x00300000U)

#define __SIO_BASE                        (__IOPORT_BASE    + 0x00000000U)
/** @} */

/**
 * @name    Peripherals
 * @{
 */
#define DMA                               ((DMA_TypeDef *)    __DMA_BASE)
#define IO_BANK0                          ((IOUSER_TypeDef *) __IOUSER0_BASE)
#define IO_QSPI                           ((IOUSER_TypeDef *) __IOQSPI_BASE)
#define PADS_BANK0                        ((PADS_TypeDef *)   __PADSUSER0_BASE)
#define PADS_QSPI                         ((PADS_TypeDef *)   __PADSQSPI_BASE)
#define PSM                               ((PSM_TypeDef *)    __PSM_BASE)
#define RESETS                            ((RESETS_TypeDef *) __RESETS_BASE)
#define SIO                               ((SIO_TypeDef *)    __SIO_BASE)
#define TIMER                             ((TIMER_TypeDef *)  __TIMER_BASE)
#define UART0                             ((UART_TypeDef *)   __UART0_BASE)
#define UART1                             ((UART_TypeDef *)   __UART1_BASE)
#define RTC                               ((RTC_TypeDef *)    __RTC_BASE)
#define SPI0                              ((SPI_TypeDef *)    __SPI0_BASE)
#define SPI1                              ((SPI_TypeDef *)    __SPI1_BASE)
#define I2C0                              ((I2C_TypeDef *)    __I2C0_BASE)
#define I2C1                              ((I2C_TypeDef *)    __I2C1_BASE)
#define ADC                               ((ADC_TypeDef *)    __ADC_BASE)
#define PWM                               ((PWM_TypeDef *)    __PWM_BASE)
#define PIO0                              ((PIO_TypeDef *)    __PIO0_BASE)
#define PIO1                              ((PIO_TypeDef *)    __PIO1_BASE)
#define USB                               ((USB_TypeDef *)    __USB_BASE)
#define WATCHDOG                          ((WATCHDOG_TypeDef *) __WATCHDOG_BASE)
/** @} */

/**
 * @name    DMA bits definitions
 * @{
 */
#define DMA_CTRL_TRIG_AHB_ERROR           (1U << 31)
#define DMA_CTRL_TRIG_READ_ERROR          (1U << 30)
#define DMA_CTRL_TRIG_WRITE_ERROR         (1U << 29)
#define DMA_CTRL_TRIG_BUSY                (1U << 24)
#define DMA_CTRL_TRIG_SNIFF_EN            (1U << 23)
#define DMA_CTRL_TRIG_BSWAP               (1U << 22)
#define DMA_CTRL_TRIG_IRQ_QUIET           (1U << 21)
#define DMA_CTRL_TRIG_TREQ_SEL_Pos        15U
#define DMA_CTRL_TRIG_TREQ_SEL_Msk        (0x3FU << DMA_CTRL_TRIG_TREQ_SEL_Pos)
#define DMA_CTRL_TRIG_TREQ_SEL(n)         ((n) << DMA_CTRL_TRIG_TREQ_SEL_Pos)
#define DMA_CTRL_TRIG_TREQ_PIO0_TX0       DMA_CTRL_TRIG_TREQ_SEL(0x00U)
#define DMA_CTRL_TRIG_TREQ_PIO0_TX1       DMA_CTRL_TRIG_TREQ_SEL(0x01U)
#define DMA_CTRL_TRIG_TREQ_PIO0_TX2       DMA_CTRL_TRIG_TREQ_SEL(0x02U)
#define DMA_CTRL_TRIG_TREQ_PIO0_TX3       DMA_CTRL_TRIG_TREQ_SEL(0x03U)
#define DMA_CTRL_TRIG_TREQ_PIO0_RX0       DMA_CTRL_TRIG_TREQ_SEL(0x04U)
#define DMA_CTRL_TRIG_TREQ_PIO0_RX1       DMA_CTRL_TRIG_TREQ_SEL(0x05U)
#define DMA_CTRL_TRIG_TREQ_PIO0_RX2       DMA_CTRL_TRIG_TREQ_SEL(0x06U)
#define DMA_CTRL_TRIG_TREQ_PIO0_RX3       DMA_CTRL_TRIG_TREQ_SEL(0x07U)
#define DMA_CTRL_TRIG_TREQ_PIO1_TX0       DMA_CTRL_TRIG_TREQ_SEL(0x08U)
#define DMA_CTRL_TRIG_TREQ_PIO1_TX1       DMA_CTRL_TRIG_TREQ_SEL(0x09U)
#define DMA_CTRL_TRIG_TREQ_PIO1_TX2       DMA_CTRL_TRIG_TREQ_SEL(0x0AU)
#define DMA_CTRL_TRIG_TREQ_PIO1_TX3       DMA_CTRL_TRIG_TREQ_SEL(0x0BU)
#define DMA_CTRL_TRIG_TREQ_PIO1_RX0       DMA_CTRL_TRIG_TREQ_SEL(0x0CU)
#define DMA_CTRL_TRIG_TREQ_PIO1_RX1       DMA_CTRL_TRIG_TREQ_SEL(0x0DU)
#define DMA_CTRL_TRIG_TREQ_PIO1_RX2       DMA_CTRL_TRIG_TREQ_SEL(0x0EU)
#define DMA_CTRL_TRIG_TREQ_PIO1_RX3       DMA_CTRL_TRIG_TREQ_SEL(0x0FU)
#define DMA_CTRL_TRIG_TREQ_SPI0_TX        DMA_CTRL_TRIG_TREQ_SEL(0x10U)
#define DMA_CTRL_TRIG_TREQ_SPI0_RX        DMA_CTRL_TRIG_TREQ_SEL(0x11U)
#define DMA_CTRL_TRIG_TREQ_SPI1_TX        DMA_CTRL_TRIG_TREQ_SEL(0x12U)
#define DMA_CTRL_TRIG_TREQ_SPI1_RX        DMA_CTRL_TRIG_TREQ_SEL(0x13U)
#define DMA_CTRL_TRIG_TREQ_UART0_TX       DMA_CTRL_TRIG_TREQ_SEL(0x14U)
#define DMA_CTRL_TRIG_TREQ_UART0_RX       DMA_CTRL_TRIG_TREQ_SEL(0x15U)
#define DMA_CTRL_TRIG_TREQ_UART1_TX       DMA_CTRL_TRIG_TREQ_SEL(0x16U)
#define DMA_CTRL_TRIG_TREQ_UART1_RX       DMA_CTRL_TRIG_TREQ_SEL(0x17U)
#define DMA_CTRL_TRIG_TREQ_PWM_WRAP0      DMA_CTRL_TRIG_TREQ_SEL(0x18U)
#define DMA_CTRL_TRIG_TREQ_PWM_WRAP1      DMA_CTRL_TRIG_TREQ_SEL(0x19U)
#define DMA_CTRL_TRIG_TREQ_PWM_WRAP2      DMA_CTRL_TRIG_TREQ_SEL(0x1AU)
#define DMA_CTRL_TRIG_TREQ_PWM_WRAP3      DMA_CTRL_TRIG_TREQ_SEL(0x1BU)
#define DMA_CTRL_TRIG_TREQ_PWM_WRAP4      DMA_CTRL_TRIG_TREQ_SEL(0x1CU)
#define DMA_CTRL_TRIG_TREQ_PWM_WRAP5      DMA_CTRL_TRIG_TREQ_SEL(0x1DU)
#define DMA_CTRL_TRIG_TREQ_PWM_WRAP6      DMA_CTRL_TRIG_TREQ_SEL(0x1EU)
#define DMA_CTRL_TRIG_TREQ_PWM_WRAP7      DMA_CTRL_TRIG_TREQ_SEL(0x1FU)
#define DMA_CTRL_TRIG_TREQ_I2C0_TX        DMA_CTRL_TRIG_TREQ_SEL(0x20U)
#define DMA_CTRL_TRIG_TREQ_I2C0_RX        DMA_CTRL_TRIG_TREQ_SEL(0x21U)
#define DMA_CTRL_TRIG_TREQ_I2C1_TX        DMA_CTRL_TRIG_TREQ_SEL(0x22U)
#define DMA_CTRL_TRIG_TREQ_I2C1_RX        DMA_CTRL_TRIG_TREQ_SEL(0x23U)
#define DMA_CTRL_TRIG_TREQ_ADC            DMA_CTRL_TRIG_TREQ_SEL(0x24U)
#define DMA_CTRL_TRIG_TREQ_XIP_STREAM     DMA_CTRL_TRIG_TREQ_SEL(0x25U)
#define DMA_CTRL_TRIG_TREQ_XIP_SSITX      DMA_CTRL_TRIG_TREQ_SEL(0x26U)
#define DMA_CTRL_TRIG_TREQ_XIP_SSIRX      DMA_CTRL_TRIG_TREQ_SEL(0x27U)
#define DMA_CTRL_TRIG_TREQ_TIMER0         DMA_CTRL_TRIG_TREQ_SEL(0x3BU)
#define DMA_CTRL_TRIG_TREQ_TIMER1         DMA_CTRL_TRIG_TREQ_SEL(0x3CU)
#define DMA_CTRL_TRIG_TREQ_TIMER2         DMA_CTRL_TRIG_TREQ_SEL(0x3DU)
#define DMA_CTRL_TRIG_TREQ_TIMER3         DMA_CTRL_TRIG_TREQ_SEL(0x3EU)
#define DMA_CTRL_TRIG_TREQ_PERMANENT      DMA_CTRL_TRIG_TREQ_SEL(0x3FU)
#define DMA_CTRL_TRIG_CHAIN_TO_Pos        11U
#define DMA_CTRL_TRIG_CHAIN_TO_Msk        (15U << DMA_CTRL_TRIG_CHAIN_TO_Pos)
#define DMA_CTRL_TRIG_CHAIN_TO(n)         ((n) << DMA_CTRL_TRIG_CHAIN_TO_Pos)
#define DMA_CTRL_TRIG_RING_SEL            (1U << 10)
#define DMA_CTRL_TRIG_RING_SIZE_Pos       6U
#define DMA_CTRL_TRIG_RING_SIZE_Msk       (15U << DMA_CTRL_TRIG_RING_SIZE_Pos)
#define DMA_CTRL_TRIG_RING_SIZE(n)        ((n) << DMA_CTRL_TRIG_RING_SIZE_Pos)
#define DMA_CTRL_TRIG_INCR_WRITE          (1U << 5)
#define DMA_CTRL_TRIG_INCR_READ           (1U << 4)
#define DMA_CTRL_TRIG_DATA_SIZE_Pos       2U
#define DMA_CTRL_TRIG_DATA_SIZE_Msk       (3U << DMA_CTRL_TRIG_DATA_SIZE_Pos)
#define DMA_CTRL_TRIG_DATA_SIZE(n)        ((n) << DMA_CTRL_TRIG_DATA_SIZE_Pos)
#define DMA_CTRL_TRIG_DATA_SIZE_BYTE      DMA_CTRL_TRIG_DATA_SIZE(0U)
#define DMA_CTRL_TRIG_DATA_SIZE_HWORD     DMA_CTRL_TRIG_DATA_SIZE(1U)
#define DMA_CTRL_TRIG_DATA_SIZE_WORD      DMA_CTRL_TRIG_DATA_SIZE(2U)
#define DMA_CTRL_TRIG_HIGH_PRIORITY       (1U << 1)
#define DMA_CTRL_TRIG_PRIORITY(n)         ((n) << 1)
#define DMA_CTRL_TRIG_EN                  (1U << 0)
/** @} */

/**
 * @name    PSM bits definitions (dangerous)
 * @{
 */
#define PSM_ANY_PROC1                     (1U << 16)
#define PSM_ANY_PROC0                     (1U << 15)
#define PSM_ANY_SIO                       (1U << 14)
#define PSM_ANY_VREG_AND_CHIP_RESET       (1U << 13)
#define PSM_ANY_XIP                       (1U << 12)
#define PSM_ANY_SRAM5                     (1U << 11)
#define PSM_ANY_SRAM4                     (1U << 10)
#define PSM_ANY_SRAM3                     (1U << 9)
#define PSM_ANY_SRAM2                     (1U << 8)
#define PSM_ANY_SRAM1                     (1U << 7)
#define PSM_ANY_SRAM0                     (1U << 6)
#define PSM_ANY_ROM                       (1U << 5)
#define PSM_ANY_BUSFABRIC                 (1U << 4)
#define PSM_ANY_RESETS                    (1U << 3)
#define PSM_ANY_CLOCKS                    (1U << 2)
#define PSM_ANY_XOSC                      (1U << 1)
#define PSM_ANY_ROSC                      (1U << 0)
/** @} */

/**
 * @name    RESETS bits definitions
 * @{
 */
#define RESETS_ALLREG_USBCTRL             (1U << 24)
#define RESETS_ALLREG_UART1               (1U << 23)
#define RESETS_ALLREG_UART0               (1U << 22)
#define RESETS_ALLREG_TIMER               (1U << 21)
#define RESETS_ALLREG_TBMAN               (1U << 20)
#define RESETS_ALLREG_SYSINFO             (1U << 19)
#define RESETS_ALLREG_SYSCFG              (1U << 18)
#define RESETS_ALLREG_SPI1                (1U << 17)
#define RESETS_ALLREG_SPI0                (1U << 16)
#define RESETS_ALLREG_RTC                 (1U << 15)
#define RESETS_ALLREG_PWM                 (1U << 14)
#define RESETS_ALLREG_PLL_USB             (1U << 13)
#define RESETS_ALLREG_PLL_SYS             (1U << 12)
#define RESETS_ALLREG_PIO1                (1U << 11)
#define RESETS_ALLREG_PIO0                (1U << 10)
#define RESETS_ALLREG_PADS_QSPI           (1U << 9)
#define RESETS_ALLREG_PADS_BANK0          (1U << 8)
#define RESETS_ALLREG_JTAG                (1U << 7)
#define RESETS_ALLREG_IO_QSPI             (1U << 6)
#define RESETS_ALLREG_IO_BANK0            (1U << 5)
#define RESETS_ALLREG_I2C1                (1U << 4)
#define RESETS_ALLREG_I2C0                (1U << 3)
#define RESETS_ALLREG_DMA                 (1U << 2)
#define RESETS_ALLREG_BUSCTRL             (1U << 1)
#define RESETS_ALLREG_ADC                 (1U << 0)
/** @} */

/**
 * @name    SIO bits definitions
 * @{
 */
#define SIO_FIFO_ST_VLD_Pos               0U
#define SIO_FIFO_ST_VLD_Msk               (1U << SIO_FIFO_ST_VLD_Pos)
#define SIO_FIFO_ST_VLD                   SIO_FIFO_ST_VLD_Msk
#define SIO_FIFO_ST_RDY_Pos               1U
#define SIO_FIFO_ST_RDY_Msk               (1U << SIO_FIFO_ST_RDY_Pos)
#define SIO_FIFO_ST_RDY                   SIO_FIFO_ST_RDY_Msk
#define SIO_FIFO_ST_WOF_Pos               2U
#define SIO_FIFO_ST_WOF_Msk               (1U << SIO_FIFO_ST_WOF_Pos)
#define SIO_FIFO_ST_WOF                   SIO_FIFO_ST_WOF_Msk
#define SIO_FIFO_ST_ROE_Pos               3U
#define SIO_FIFO_ST_ROE_Msk               (1U << SIO_FIFO_ST_ROE_Pos)
#define SIO_FIFO_ST_ROE                   SIO_FIFO_ST_ROE_Msk
/** @} */

/**
 * @name    SPI bits definitions
 * @{
 */
#define SPI_SSPCR0_SCR_Pos                8U
#define SPI_SSPCR0_SCR_Msk                (255U << SPI_SSPCR0_SCR_Pos)
#define SPI_SSPCR0_SCR(n)                 ((n) << SPI_SSPCR0_SCR_Pos)
#define SPI_SSPCR0_SPH_Pos                7U
#define SPI_SSPCR0_SPH_Msk                (1U << SPI_SSPCR0_SPH_Pos)
#define SPI_SSPCR0_SPH                    SPI_SSPCR0_SPH_Msk
#define SPI_SSPCR0_SPO_Pos                6U
#define SPI_SSPCR0_SPO_Msk                (1U << SPI_SSPCR0_SPO_Pos)
#define SPI_SSPCR0_SPO                    SPI_SSPCR0_SPO_Msk
#define SPI_SSPCR0_FRF_Pos                4U
#define SPI_SSPCR0_FRF_Msk                (3U << SPI_SSPCR0_FRF_Pos)
#define SPI_SSPCR0_FRF(n)                 ((n) << SPI_SSPCR0_FRF_Pos)
#define SPI_SSPCR0_FRF_MOTOROLA           SPI_SSPCR0_FRF(0U)
#define SPI_SSPCR0_FRF_TI                 SPI_SSPCR0_FRF(1U)
#define SPI_SSPCR0_FRF_NATIONAL           SPI_SSPCR0_FRF(2U)
#define SPI_SSPCR0_DSS_Pos                0U
#define SPI_SSPCR0_DSS_Msk                (15U << SPI_SSPCR0_DSS_Pos)
#define SPI_SSPCR0_DSS(n)                 ((n) << SPI_SSPCR0_DSS_Pos)
#define SPI_SSPCR0_DSS_4BIT               SPI_SSPCR0_DSS(3U)
#define SPI_SSPCR0_DSS_5BIT               SPI_SSPCR0_DSS(4U)
#define SPI_SSPCR0_DSS_6BIT               SPI_SSPCR0_DSS(5U)
#define SPI_SSPCR0_DSS_7BIT               SPI_SSPCR0_DSS(6U)
#define SPI_SSPCR0_DSS_8BIT               SPI_SSPCR0_DSS(7U)
#define SPI_SSPCR0_DSS_9BIT               SPI_SSPCR0_DSS(8U)
#define SPI_SSPCR0_DSS_10BIT              SPI_SSPCR0_DSS(9U)
#define SPI_SSPCR0_DSS_11BIT              SPI_SSPCR0_DSS(10U)
#define SPI_SSPCR0_DSS_12BIT              SPI_SSPCR0_DSS(11U)
#define SPI_SSPCR0_DSS_13BIT              SPI_SSPCR0_DSS(12U)
#define SPI_SSPCR0_DSS_14BIT              SPI_SSPCR0_DSS(13U)
#define SPI_SSPCR0_DSS_15BIT              SPI_SSPCR0_DSS(14U)
#define SPI_SSPCR0_DSS_16BIT              SPI_SSPCR0_DSS(15U)

#define SPI_SSPCR1_SOD_Pos                3U
#define SPI_SSPCR1_SOD_Msk                (1U << SPI_SSPCR1_SOD_Pos)
#define SPI_SSPCR1_SOD                    SPI_SSPCR1_SOD_Msk
#define SPI_SSPCR1_MS_Pos                 2U
#define SPI_SSPCR1_MS_Msk                 (1U << SPI_SSPCR1_MS_Pos)
#define SPI_SSPCR1_MS                     SPI_SSPCR1_MS_Msk
#define SPI_SSPCR1_SSE_Pos                1U
#define SPI_SSPCR1_SSE_Msk                (1U << SPI_SSPCR1_SSE_Pos)
#define SPI_SSPCR1_SSE                    SPI_SSPCR1_SSE_Msk
#define SPI_SSPCR1_LBM_Pos                0U
#define SPI_SSPCR1_LBM_Msk                (1U << SPI_SSPCR1_LBM_Pos)
#define SPI_SSPCR1_LBM                    SPI_SSPCR1_LBM_Msk

#define SPI_SSPSR_BSY_Pos                 4U
#define SPI_SSPSR_BSY_Msk                 (1U << SPI_SSPSR_BSY_Pos)
#define SPI_SSPSR_BSY                     SPI_SSPSR_BSY_Msk
#define SPI_SSPSR_RFF_Pos                 3U
#define SPI_SSPSR_RFF_Msk                 (1U << SPI_SSPSR_RFF_Pos)
#define SPI_SSPSR_RFF                     SPI_SSPSR_RFF_Msk
#define SPI_SSPSR_RNE_Pos                 2U
#define SPI_SSPSR_RNE_Msk                 (1U << SPI_SSPSR_RNE_Pos)
#define SPI_SSPSR_RNE                     SPI_SSPSR_RNE_Msk
#define SPI_SSPSR_TNF_Pos                 1U
#define SPI_SSPSR_TNF_Msk                 (1U << SPI_SSPSR_TNF_Pos)
#define SPI_SSPSR_TNF                     SPI_SSPSR_TNF_Msk
#define SPI_SSPSR_TFE_Pos                 0U
#define SPI_SSPSR_TFE_Msk                 (1U << SPI_SSPSR_TFE_Pos)
#define SPI_SSPSR_TFE                     SPI_SSPSR_TFE_Msk

#define SPI_SSPCPSR_CPSDVSR_Pos           0U
#define SPI_SSPCPSR_CPSDVSR_Msk           (255U << SPI_SSPCPSR_CPSDVSR_Pos)
#define SPI_SSPCPSR_CPSDVSR(n)            ((n) << SPI_SSPCPSR_CPSDVSR_Msk)

#define SPI_SSPIMSC_TXIM_Pos              3U
#define SPI_SSPIMSC_TXIM_Msk              (1U << SPI_SSPIMSC_TXIM_Pos)
#define SPI_SSPIMSC_TXIM                  SPI_SSPIMSC_TXIM_Msk
#define SPI_SSPIMSC_RXIM_Pos              2U
#define SPI_SSPIMSC_RXIM_Msk              (1U << SPI_SSPIMSC_RXIM_Pos)
#define SPI_SSPIMSC_RXIM                  SPI_SSPIMSC_RXIM_Msk
#define SPI_SSPIMSC_RTIM_Pos              1U
#define SPI_SSPIMSC_RTIM_Msk              (1U << SPI_SSPIMSC_RTIM_Pos)
#define SPI_SSPIMSC_RTIM                  SPI_SSPIMSC_RTIM_Msk
#define SPI_SSPIMSC_RORIM_Pos             0U
#define SPI_SSPIMSC_RORIM_Msk             (1U << SPI_SSPIMSC_RORIM_Pos)
#define SPI_SSPIMSC_RORIM                 SPI_SSPIMSC_RORIM_Msk

#define SPI_SSPRIS_TXRIS_Pos              3U
#define SPI_SSPRIS_TXRIS_Msk              (1U << SPI_SSPRIS_TXRIS_Pos)
#define SPI_SSPRIS_TXRIS                  SPI_SSPRIS_TXRIS_Msk
#define SPI_SSPRIS_RXRIS_Pos              2U
#define SPI_SSPRIS_RXRIS_Msk              (1U << SPI_SSPRIS_RXRIS_Pos)
#define SPI_SSPRIS_RXRIS                  SPI_SSPRIS_RXRIS_Msk
#define SPI_SSPRIS_RTRIS_Pos              1U
#define SPI_SSPRIS_RTRIS_Msk              (1U << SPI_SSPRIS_RTRIS_Pos)
#define SPI_SSPRIS_RTRIS                  SPI_SSPRIS_RTRIS_Msk
#define SPI_SSPRIS_RORRIS_Pos             0U
#define SPI_SSPRIS_RORRIS_Msk             (1U << SPI_SSPRIS_RORRIS_Pos)
#define SPI_SSPRIS_RORRIS                 SPI_SSPRIS_RORRIS_Msk

#define SPI_SSPMIS_TXMIS_Pos              3U
#define SPI_SSPMIS_TXMIS_Msk              (1U << SPI_SSPMIS_TXMIS_Pos)
#define SPI_SSPMIS_TXMIS                  SPI_SSPMIS_TXMIS_Msk
#define SPI_SSPMIS_RXMIS_Pos              2U
#define SPI_SSPMIS_RXMIS_Msk              (1U << SPI_SSPMIS_RXMIS_Pos)
#define SPI_SSPMIS_RXMIS                  SPI_SSPMIS_RXMIS_Msk
#define SPI_SSPMIS_RTMIS_Pos              1U
#define SPI_SSPMIS_RTMIS_Msk              (1U << SPI_SSPMIS_RTMIS_Pos)
#define SPI_SSPMIS_RTMIS                  SPI_SSPMIS_RTMIS_Msk
#define SPI_SSPMIS_RORMIS_Pos             0U
#define SPI_SSPMIS_RORMIS_Msk             (1U << SPI_SSPMIS_RORMIS_Pos)
#define SPI_SSPMIS_RORMIS                 SPI_SSPMIS_RORMIS_Msk

#define SPI_SSPICR_RTIC_Pos               1U
#define SPI_SSPICR_RTIC_Msk               (1U << SPI_SSPICR_RTIC_Pos)
#define SPI_SSPICR_RTIC                   SPI_SSPICR_RTIC_Msk
#define SPI_SSPICR_RORIC_Pos              0U
#define SPI_SSPICR_RORIC_Msk              (1U << SPI_SSPICR_RORIC_Pos)
#define SPI_SSPICR_RORIC                  SPI_SSPICR_RORIC_Msk

#define SPI_SSPDMACR_TXDMAE_Pos           1U
#define SPI_SSPDMACR_TXDMAE_Msk           (1U << SPI_SSPDMACR_TXDMAE_Pos)
#define SPI_SSPDMACR_TXDMAE               SPI_SSPDMACR_TXDMAE_Msk
#define SPI_SSPDMACR_RXDMAE_Pos           0U
#define SPI_SSPDMACR_RXDMAE_Msk           (1U << SPI_SSPDMACR_RXDMAE_Pos)
#define SPI_SSPDMACR_RXDMAE               SPI_SSPDMACR_RXDMAE_Msk
/** @} */

/**
 * @name    TIMER bits definitions
 * @{
 */
#define TIMER_ARMED_ALARM0_Pos            0U
#define TIMER_ARMED_ALARM0_Msk            (1U << TIMER_ARMED_ALARM0_Pos)
#define TIMER_ARMED_ALARM0                TIMER_ARMED_ALARM0_Msk
#define TIMER_ARMED_ALARM1_Pos            1U
#define TIMER_ARMED_ALARM1_Msk            (1U << TIMER_ARMED_ALARM1_Pos)
#define TIMER_ARMED_ALARM1                TIMER_ARMED_ALARM1_Msk
#define TIMER_ARMED_ALARM2_Pos            2U
#define TIMER_ARMED_ALARM2_Msk            (1U << TIMER_ARMED_ALARM2_Pos)
#define TIMER_ARMED_ALARM2                TIMER_ARMED_ALARM2_Msk
#define TIMER_ARMED_ALARM3_Pos            3U
#define TIMER_ARMED_ALARM3_Msk            (1U << TIMER_ARMED_ALARM3_Pos)
#define TIMER_ARMED_ALARM3                TIMER_ARMED_ALARM3_Msk

#define TIMER_DBGPAUSE_DBG0_Pos           1U
#define TIMER_DBGPAUSE_DBG0_Msk           (1U << TIMER_DBGPAUSE_DBG0_Pos)
#define TIMER_DBGPAUSE_DBG0               TIMER_DBGPAUSE_DBG0_Msk
#define TIMER_DBGPAUSE_DBG1_Pos           2U
#define TIMER_DBGPAUSE_DBG1_Msk           (1U << TIMER_DBGPAUSE_DBG1_Pos)
#define TIMER_DBGPAUSE_DBG1               TIMER_DBGPAUSE_DBG1_Msk

#define TIMER_PAUSE_PAUSE_Pos             0U
#define TIMER_PAUSE_PAUSE_Msk             (1U << TIMER_PAUSE_PAUSE_Pos)
#define TIMER_PAUSE_PAUSE                 TIMER_PAUSE_PAUSE_Msk

#define TIMER_INTR_ALARM0_Pos             0U
#define TIMER_INTR_ALARM0_Msk             (1U << TIMER_INTR_ALARM0_Pos)
#define TIMER_INTR_ALARM0                 TIMER_INTR_ALARM0_Msk
#define TIMER_INTR_ALARM1_Pos             1U
#define TIMER_INTR_ALARM1_Msk             (1U << TIMER_INTR_ALARM1_Pos)
#define TIMER_INTR_ALARM1                 TIMER_INTR_ALARM1_Msk
#define TIMER_INTR_ALARM2_Pos             2U
#define TIMER_INTR_ALARM2_Msk             (1U << TIMER_INTR_ALARM2_Pos)
#define TIMER_INTR_ALARM2                 TIMER_INTR_ALARM2_Msk
#define TIMER_INTR_ALARM3_Pos             3U
#define TIMER_INTR_ALARM3_Msk             (1U << TIMER_INTR_ALARM3_Pos)
#define TIMER_INTR_ALARM3                 TIMER_INTR_ALARM3_Msk

#define TIMER_INTE_ALARM0_Pos             0U
#define TIMER_INTE_ALARM0_Msk             (1U << TIMER_INTE_ALARM0_Pos)
#define TIMER_INTE_ALARM0                 TIMER_INTE_ALARM0_Msk
#define TIMER_INTE_ALARM1_Pos             1U
#define TIMER_INTE_ALARM1_Msk             (1U << TIMER_INTE_ALARM1_Pos)
#define TIMER_INTE_ALARM1                 TIMER_INTE_ALARM1_Msk
#define TIMER_INTE_ALARM2_Pos             2U
#define TIMER_INTE_ALARM2_Msk             (1U << TIMER_INTE_ALARM2_Pos)
#define TIMER_INTE_ALARM2                 TIMER_INTE_ALARM2_Msk
#define TIMER_INTE_ALARM3_Pos             3U
#define TIMER_INTE_ALARM3_Msk             (1U << TIMER_INTE_ALARM3_Pos)
#define TIMER_INTE_ALARM3                 TIMER_INTE_ALARM3_Msk

#define TIMER_INTF_ALARM0_Pos             0U
#define TIMER_INTF_ALARM0_Msk             (1U << TIMER_INTF_ALARM0_Pos)
#define TIMER_INTF_ALARM0                 TIMER_INTF_ALARM0_Msk
#define TIMER_INTF_ALARM1_Pos             1U
#define TIMER_INTF_ALARM1_Msk             (1U << TIMER_INTF_ALARM1_Pos)
#define TIMER_INTF_ALARM1                 TIMER_INTF_ALARM1_Msk
#define TIMER_INTF_ALARM2_Pos             2U
#define TIMER_INTF_ALARM2_Msk             (1U << TIMER_INTF_ALARM2_Pos)
#define TIMER_INTF_ALARM2                 TIMER_INTF_ALARM2_Msk
#define TIMER_INTF_ALARM3_Pos             3U
#define TIMER_INTF_ALARM3_Msk             (1U << TIMER_INTF_ALARM3_Pos)
#define TIMER_INTF_ALARM3                 TIMER_INTF_ALARM3_Msk

#define TIMER_INTS_ALARM0_Pos             0U
#define TIMER_INTS_ALARM0_Msk             (1U << TIMER_INTS_ALARM0_Pos)
#define TIMER_INTS_ALARM0                 TIMER_INTS_ALARM0_Msk
#define TIMER_INTS_ALARM1_Pos             1U
#define TIMER_INTS_ALARM1_Msk             (1U << TIMER_INTS_ALARM1_Pos)
#define TIMER_INTS_ALARM1                 TIMER_INTS_ALARM1_Msk
#define TIMER_INTS_ALARM2_Pos             2U
#define TIMER_INTS_ALARM2_Msk             (1U << TIMER_INTS_ALARM2_Pos)
#define TIMER_INTS_ALARM2                 TIMER_INTS_ALARM2_Msk
#define TIMER_INTS_ALARM3_Pos             3U
#define TIMER_INTS_ALARM3_Msk             (1U << TIMER_INTS_ALARM3_Pos)
#define TIMER_INTS_ALARM3                 TIMER_INTS_ALARM3_Msk
/** @} */

/**
 * @name    UART bits definitions
 * @{
 */
#define UART_UARTDR_OE_Pos                11U
#define UART_UARTDR_OE_Msk                (1U << UART_UARTDR_OE_Pos)
#define UART_UARTDR_OE                    UART_UARTDR_OE_Msk
#define UART_UARTDR_BE_Pos                10U
#define UART_UARTDR_BE_Msk                (1U << UART_UARTDR_BE_Pos)
#define UART_UARTDR_BE                    UART_UARTDR_BE_Msk
#define UART_UARTDR_PE_Pos                9U
#define UART_UARTDR_PE_Msk                (1U << UART_UARTDR_PE_Pos)
#define UART_UARTDR_PE                    UART_UARTDR_PE_Msk
#define UART_UARTDR_FE_Pos                8U
#define UART_UARTDR_FE_Msk                (1U << UART_UARTDR_FE_Pos)
#define UART_UARTDR_FE                    UART_UARTDR_FE_Msk

#define UART_UARTRSR_OE_Pos               3U
#define UART_UARTRSR_OE_Msk               (1U << UART_UARTRSR_OE_Pos)
#define UART_UARTRSR_OE                   UART_UARTRSR_OE_Msk
#define UART_UARTRSR_BE_Pos               2U
#define UART_UARTRSR_BE_Msk               (1U << UART_UARTRSR_BE_Pos)
#define UART_UARTRSR_BE                   UART_UARTRSR_BE_Msk
#define UART_UARTRSR_PE_Pos               1U
#define UART_UARTRSR_PE_Msk               (1U << UART_UARTRSR_PE_Pos)
#define UART_UARTRSR_PE                   UART_UARTRSR_PE_Msk
#define UART_UARTRSR_FE_Pos               0U
#define UART_UARTRSR_FE_Msk               (1U << UART_UARTRSR_FE_Pos)
#define UART_UARTRSR_FE                   UART_UARTRSR_FE_Msk

#define UART_UARTFR_RI_Pos                8U
#define UART_UARTFR_RI_Msk                (1U << UART_UARTFR_RI_Pos)
#define UART_UARTFR_RI                    UART_UARTFR_RI_Msk
#define UART_UARTFR_TXFE_Pos              7U
#define UART_UARTFR_TXFE_Msk              (1U << UART_UARTFR_TXFE_Pos)
#define UART_UARTFR_TXFE                  UART_UARTFR_TXFE_Msk
#define UART_UARTFR_RXFF_Pos              6U
#define UART_UARTFR_RXFF_Msk              (1U << UART_UARTFR_RXFF_Pos)
#define UART_UARTFR_RXFF                  UART_UARTFR_RXFF_Msk
#define UART_UARTFR_TXFF_Pos              5U
#define UART_UARTFR_TXFF_Msk              (1U << UART_UARTFR_TXFF_Pos)
#define UART_UARTFR_TXFF                  UART_UARTFR_TXFF_Msk
#define UART_UARTFR_RXFE_Pos              4U
#define UART_UARTFR_RXFE_Msk              (1U << UART_UARTFR_RXFE_Pos)
#define UART_UARTFR_RXFE                  UART_UARTFR_RXFE_Msk
#define UART_UARTFR_BUSY_Pos              3U
#define UART_UARTFR_BUSY_Msk              (1U << UART_UARTFR_BUSY_Pos)
#define UART_UARTFR_BUSY                  UART_UARTFR_BUSY_Msk
#define UART_UARTFR_DCD_Pos               2U
#define UART_UARTFR_DCD_Msk               (1U << UART_UARTFR_DCD_Pos)
#define UART_UARTFR_DCD                   UART_UARTFR_DCD_Msk
#define UART_UARTFR_DSR_Pos               1U
#define UART_UARTFR_DSR_Msk               (1U << UART_UARTFR_DSR_Pos)
#define UART_UARTFR_DSR                   UART_UARTFR_DSR_Msk
#define UART_UARTFR_CTS_Pos               0U
#define UART_UARTFR_CTS_Msk               (1U << UART_UARTFR_CTS_Pos)
#define UART_UARTFR_CTS                   UART_UARTFR_CTS_Msk

#define UART_UARTILPR_ILPDVSR_Pos         0U
#define UART_UARTILPR_ILPDVSR_Msk         (255U << UART_UARTILPR_ILPDVSR_Pos)
#define UART_UARTILPR_ILPDVSR(n)          ((n) << UART_UARTILPR_ILPDVSR_Pos)

#define UART_UARTIBRD_BAUD_DIVINT_Pos     0U
#define UART_UARTIBRD_BAUD_DIVINT_Msk     (0xFFFFU << UART_UARTIBRD_BAUD_DIVINT_Pos)
#define UART_UARTIBRD_BAUD_DIVINT(n)      ((n) << UART_UARTIBRD_BAUD_DIVINT_Pos)

#define UART_UARTFBRD_BAUD_DIVFRAC_Pos    0U
#define UART_UARTFBRD_BAUD_DIVFRAC_Msk    (63U << UART_UARTFBRD_BAUD_DIVFRAC_Pos)
#define UART_UARTFBRD_BAUD_DIVFRAC(n)     ((n) << UART_UARTFBRD_BAUD_DIVFRAC_Pos)

#define UART_UARTLCR_H_SPS_Pos            7U
#define UART_UARTLCR_H_SPS_Msk            (1U << UART_UARTLCR_H_SPS_Pos)
#define UART_UARTLCR_H_SPS                UART_UARTLCR_H_SPS_Msk
#define UART_UARTLCR_H_WLEN_Pos           5U
#define UART_UARTLCR_H_WLEN_Msk           (1U << UART_UARTLCR_H_WLEN_Pos)
#define UART_UARTLCR_H_WLEN(n)            ((n) << UART_UARTLCR_H_WLEN_Pos)
#define UART_UARTLCR_H_WLEN_5BITS         UART_UARTLCR_H_WLEN(0U)
#define UART_UARTLCR_H_WLEN_6BITS         UART_UARTLCR_H_WLEN(1U)
#define UART_UARTLCR_H_WLEN_7BITS         UART_UARTLCR_H_WLEN(2U)
#define UART_UARTLCR_H_WLEN_8BITS         UART_UARTLCR_H_WLEN(3U)
#define UART_UARTLCR_H_FEN_Pos            4U
#define UART_UARTLCR_H_FEN_Msk            (1U << UART_UARTLCR_H_FEN_Pos)
#define UART_UARTLCR_H_FEN                UART_UARTLCR_H_FEN_Msk
#define UART_UARTLCR_H_STP2_Pos           3U
#define UART_UARTLCR_H_STP2_Msk           (1U << UART_UARTLCR_H_STP2_Pos)
#define UART_UARTLCR_H_STP2               UART_UARTLCR_H_STP2_Msk
#define UART_UARTLCR_H_EPS_Pos            2U
#define UART_UARTLCR_H_EPS_Msk            (1U << UART_UARTLCR_H_EPS_Pos)
#define UART_UARTLCR_H_EPS                UART_UARTLCR_H_EPS_Msk
#define UART_UARTLCR_H_PEN_Pos            1U
#define UART_UARTLCR_H_PEN_Msk            (1U << UART_UARTLCR_H_PEN_Pos)
#define UART_UARTLCR_H_PEN                UART_UARTLCR_H_PEN_Msk
#define UART_UARTLCR_H_BRK_Pos            0U
#define UART_UARTLCR_H_BRK_Msk            (1U << UART_UARTLCR_H_BRK_Pos)
#define UART_UARTLCR_H_BRK                UART_UARTLCR_H_BRK_Msk

#define UART_UARTCR_CTSEN_Pos             15U
#define UART_UARTCR_CTSEN_Msk             (1U << UART_UARTCR_CTSEN_Pos)
#define UART_UARTCR_CTSEN                 UART_UARTCR_CTSEN_Msk
#define UART_UARTCR_RTSEN_Pos             14U
#define UART_UARTCR_RTSEN_Msk             (1U << UART_UARTCR_RTSEN_Pos)
#define UART_UARTCR_RTSEN                 UART_UARTCR_RTSEN_Msk
#define UART_UARTCR_OUT2_Pos              13U
#define UART_UARTCR_OUT2_Msk              (1U << UART_UARTCR_OUT2_Pos)
#define UART_UARTCR_OUT2                  UART_UARTCR_OUT2_Msk
#define UART_UARTCR_OUT1_Pos              12U
#define UART_UARTCR_OUT1_Msk              (1U << UART_UARTCR_OUT1_Pos)
#define UART_UARTCR_OUT1                  UART_UARTCR_OUT1_Msk
#define UART_UARTCR_RTS_Pos               11U
#define UART_UARTCR_RTS_Msk               (1U << UART_UARTCR_RTS_Pos)
#define UART_UARTCR_RTS                   UART_UARTCR_RTS_Msk
#define UART_UARTCR_DTR_Pos               10U
#define UART_UARTCR_DTR_Msk               (1U << UART_UARTCR_DTR_Pos)
#define UART_UARTCR_DTR                   UART_UARTCR_DTR_Msk
#define UART_UARTCR_RXE_Pos               9U
#define UART_UARTCR_RXE_Msk               (1U << UART_UARTCR_RXE_Pos)
#define UART_UARTCR_RXE                   UART_UARTCR_RXE_Msk
#define UART_UARTCR_TXE_Pos               8U
#define UART_UARTCR_TXE_Msk               (1U << UART_UARTCR_TXE_Pos)
#define UART_UARTCR_TXE                   UART_UARTCR_TXE_Msk
#define UART_UARTCR_LBE_Pos               7U
#define UART_UARTCR_LBE_Msk               (1U << UART_UARTCR_LBE_Pos)
#define UART_UARTCR_LBE                   UART_UARTCR_LBE_Msk
#define UART_UARTCR_SIRLP_Pos             2U
#define UART_UARTCR_SIRLP_Msk             (1U << UART_UARTCR_SIRLP_Pos)
#define UART_UARTCR_SIRLP                 UART_UARTCR_SIRLP_Msk
#define UART_UARTCR_SIREN_Pos             1U
#define UART_UARTCR_SIREN_Msk             (1U << UART_UARTCR_SIREN_Pos)
#define UART_UARTCR_SIREN                 UART_UARTCR_SIREN_Msk
#define UART_UARTCR_UARTEN_Pos            0U
#define UART_UARTCR_UARTEN_Msk            (1U << UART_UARTCR_UARTEN_Pos)
#define UART_UARTCR_UARTEN                UART_UARTCR_UARTEN_Msk

#define UART_UARTIFLS_RXIFLSEL_Pos        3U
#define UART_UARTIFLS_RXIFLSEL_Msk        (1U << UART_UARTIFLS_RXIFLSEL_Pos)
#define UART_UARTIFLS_RXIFLSEL(n)         ((n) << UART_UARTIFLS_RXIFLSEL_Pos)
#define UART_UARTIFLS_RXIFLSEL_1_8F       UART_UARTIFLS_RXIFLSEL(0U)
#define UART_UARTIFLS_RXIFLSEL_1_4F       UART_UARTIFLS_RXIFLSEL(1U)
#define UART_UARTIFLS_RXIFLSEL_1_2F       UART_UARTIFLS_RXIFLSEL(2U)
#define UART_UARTIFLS_RXIFLSEL_3_4F       UART_UARTIFLS_RXIFLSEL(3U)
#define UART_UARTIFLS_RXIFLSEL_7_8F       UART_UARTIFLS_RXIFLSEL(4U)

#define UART_UARTIFLS_TXIFLSEL_Pos        3U
#define UART_UARTIFLS_TXIFLSEL_Msk        (1U << UART_UARTIFLS_TXIFLSEL_Pos)
#define UART_UARTIFLS_TXIFLSEL(n)         ((n) << UART_UARTIFLS_TXIFLSEL_Pos)
#define UART_UARTIFLS_TXIFLSEL_1_8E       UART_UARTIFLS_TXIFLSEL(0U)
#define UART_UARTIFLS_TXIFLSEL_1_4E       UART_UARTIFLS_TXIFLSEL(1U)
#define UART_UARTIFLS_TXIFLSEL_1_2E       UART_UARTIFLS_TXIFLSEL(2U)
#define UART_UARTIFLS_TXIFLSEL_3_4E       UART_UARTIFLS_TXIFLSEL(3U)
#define UART_UARTIFLS_TXIFLSEL_7_8E       UART_UARTIFLS_TXIFLSEL(4U)

#define UART_UARTIMSC_OEIM_Pos            10U
#define UART_UARTIMSC_OEIM_Msk            (1U << UART_UARTIMSC_OEIM_Pos)
#define UART_UARTIMSC_OEIM                UART_UARTIMSC_OEIM_Msk
#define UART_UARTIMSC_BEIM_Pos            9U
#define UART_UARTIMSC_BEIM_Msk            (1U << UART_UARTIMSC_BEIM_Pos)
#define UART_UARTIMSC_BEIM                UART_UARTIMSC_BEIM_Msk
#define UART_UARTIMSC_PEIM_Pos            8U
#define UART_UARTIMSC_PEIM_Msk            (1U << UART_UARTIMSC_PEIM_Pos)
#define UART_UARTIMSC_PEIM                UART_UARTIMSC_PEIM_Msk
#define UART_UARTIMSC_FEIM_Pos            7U
#define UART_UARTIMSC_FEIM_Msk            (1U << UART_UARTIMSC_FEIM_Pos)
#define UART_UARTIMSC_FEIM                UART_UARTIMSC_FEIM_Msk
#define UART_UARTIMSC_RTIM_Pos            6U
#define UART_UARTIMSC_RTIM_Msk            (1U << UART_UARTIMSC_RTIM_Pos)
#define UART_UARTIMSC_RTIM                UART_UARTIMSC_RTIM_Msk
#define UART_UARTIMSC_TXIM_Pos            5U
#define UART_UARTIMSC_TXIM_Msk            (1U << UART_UARTIMSC_TXIM_Pos)
#define UART_UARTIMSC_TXIM                UART_UARTIMSC_TXIM_Msk
#define UART_UARTIMSC_RXIM_Pos            4U
#define UART_UARTIMSC_RXIM_Msk            (1U << UART_UARTIMSC_RXIM_Pos)
#define UART_UARTIMSC_RXIM                UART_UARTIMSC_RXIM_Msk
#define UART_UARTIMSC_DSRMIM_Pos          3U
#define UART_UARTIMSC_DSRMIM_Msk          (1U << UART_UARTIMSC_DSRMIM_Pos)
#define UART_UARTIMSC_DSRMIM              UART_UARTIMSC_DSRMIM_Msk
#define UART_UARTIMSC_DCDMIM_Pos          2U
#define UART_UARTIMSC_DCDMIM_Msk          (1U << UART_UARTIMSC_DCDMIM_Pos)
#define UART_UARTIMSC_DCDMIM              UART_UARTIMSC_DCDMIM_Msk
#define UART_UARTIMSC_CTSMIM_Pos          1U
#define UART_UARTIMSC_CTSMIM_Msk          (1U << UART_UARTIMSC_CTSMIM_Pos)
#define UART_UARTIMSC_CTSMIM              UART_UARTIMSC_CTSMIM_Msk
#define UART_UARTIMSC_RIMIM_Pos           0U
#define UART_UARTIMSC_RIMIM_Msk           (1U << UART_UARTIMSC_RIMIM_Pos)
#define UART_UARTIMSC_RIMIM               UART_UARTIMSC_RIMIM_Msk

#define UART_UARTRIS_OERIS_Pos            10U
#define UART_UARTRIS_OERIS_Msk            (1U << UART_UARTRIS_OERIS_Pos)
#define UART_UARTRIS_OERIS                UART_UARTRIS_OERIS_Msk
#define UART_UARTRIS_BERIS_Pos            9U
#define UART_UARTRIS_BERIS_Msk            (1U << UART_UARTRIS_BERIS_Pos)
#define UART_UARTRIS_BERIS                UART_UARTRIS_BERIS_Msk
#define UART_UARTRIS_PERIS_Pos            8U
#define UART_UARTRIS_PERIS_Msk            (1U << UART_UARTRIS_PERIS_Pos)
#define UART_UARTRIS_PERIS                UART_UARTRIS_PERIS_Msk
#define UART_UARTRIS_FERIS_Pos            7U
#define UART_UARTRIS_FERIS_Msk            (1U << UART_UARTRIS_FERIS_Pos)
#define UART_UARTRIS_FERIS                UART_UARTRIS_FERIS_Msk
#define UART_UARTRIS_RTRIS_Pos            6U
#define UART_UARTRIS_RTRIS_Msk            (1U << UART_UARTRIS_RTRIS_Pos)
#define UART_UARTRIS_RTRIS                UART_UARTRIS_RTRIS_Msk
#define UART_UARTRIS_TXRIS_Pos            5U
#define UART_UARTRIS_TXRIS_Msk            (1U << UART_UARTRIS_TXRIS_Pos)
#define UART_UARTRIS_TXRIS                UART_UARTRIS_TXRIS_Msk
#define UART_UARTRIS_RXRIS_Pos            4U
#define UART_UARTRIS_RXRIS_Msk            (1U << UART_UARTRIS_RXRIS_Pos)
#define UART_UARTRIS_RXRIS                UART_UARTRIS_RXRIS_Msk
#define UART_UARTRIS_DSRRMIS_Pos          3U
#define UART_UARTRIS_DSRRMIS_Msk          (1U << UART_UARTRIS_DSRRMIS_Pos)
#define UART_UARTRIS_DSRRMIS              UART_UARTRIS_DSRRMIS_Msk
#define UART_UARTRIS_DCDRMIS_Pos          2U
#define UART_UARTRIS_DCDRMIS_Msk          (1U << UART_UARTRIS_DCDRMIS_Pos)
#define UART_UARTRIS_DCDRMIS              UART_UARTRIS_DCDRMIS_Msk
#define UART_UARTRIS_CTSRMIS_Pos          1U
#define UART_UARTRIS_CTSRMIS_Msk          (1U << UART_UARTRIS_CTSRMIS_Pos)
#define UART_UARTRIS_CTSRMIS              UART_UARTRIS_CTSRMIS_Msk
#define UART_UARTRIS_RIRMIS_Pos           0U
#define UART_UARTRIS_RIRMIS_Msk           (1U << UART_UARTRIS_RIRMIS_Pos)
#define UART_UARTRIS_RIRMIS               UART_UARTIMSC_RIRMIS_Msk

#define UART_UARTMIS_OEMIS_Pos            10U
#define UART_UARTMIS_OEMIS_Msk            (1U << UART_UARTMIS_OEMIS_Pos)
#define UART_UARTMIS_OEMIS                UART_UARTMIS_OEMIS_Msk
#define UART_UARTMIS_BEMIS_Pos            9U
#define UART_UARTMIS_BEMIS_Msk            (1U << UART_UARTMIS_BEMIS_Pos)
#define UART_UARTMIS_BEMIS                UART_UARTMIS_BEMIS_Msk
#define UART_UARTMIS_PEMIS_Pos            8U
#define UART_UARTMIS_PEMIS_Msk            (1U << UART_UARTMIS_PEMIS_Pos)
#define UART_UARTMIS_PEMIS                UART_UARTMIS_PEMIS_Msk
#define UART_UARTMIS_FEMIS_Pos            7U
#define UART_UARTMIS_FEMIS_Msk            (1U << UART_UARTMIS_FEMIS_Pos)
#define UART_UARTMIS_FEMIS                UART_UARTMIS_FEMIS_Msk
#define UART_UARTMIS_RTMIS_Pos            6U
#define UART_UARTMIS_RTMIS_Msk            (1U << UART_UARTMIS_RTMIS_Pos)
#define UART_UARTMIS_RTMIS                UART_UARTMIS_RTMIS_Msk
#define UART_UARTMIS_TXMIS_Pos            5U
#define UART_UARTMIS_TXMIS_Msk            (1U << UART_UARTMIS_TXMIS_Pos)
#define UART_UARTMIS_TXMIS                UART_UARTMIS_TXMIS_Msk
#define UART_UARTMIS_RXMIS_Pos            4U
#define UART_UARTMIS_RXMIS_Msk            (1U << UART_UARTMIS_RXMIS_Pos)
#define UART_UARTMIS_RXMIS                UART_UARTMIS_RXMIS_Msk
#define UART_UARTMIS_DSRMMIS_Pos          3U
#define UART_UARTMIS_DSRMMIS_Msk          (1U << UART_UARTMIS_DSRMMIS_Pos)
#define UART_UARTMIS_DSRRMIS              UART_UARTMIS_DSRMMIS_Msk
#define UART_UARTMIS_DCDMMIS_Pos          2U
#define UART_UARTMIS_DCDMMIS_Msk          (1U << UART_UARTMIS_DCDMMIS_Pos)
#define UART_UARTMIS_DCDMMIS              UART_UARTMIS_DCDMMIS_Msk
#define UART_UARTMIS_CTSMMIS_Pos          1U
#define UART_UARTMIS_CTSMMIS_Msk          (1U << UART_UARTMIS_CTSMMIS_Pos)
#define UART_UARTMIS_CTSMMIS              UART_UARTMIS_CTSMMIS_Msk
#define UART_UARTMIS_RIMMIS_Pos           0U
#define UART_UARTMIS_RIMMIS_Msk           (1U << UART_UARTMIS_RIMMIS_Pos)
#define UART_UARTMIS_RIMMIS               UART_UARTIMSC_RIMMIS_Msk

#define UART_UARTICR_OEIC_Pos             10U
#define UART_UARTICR_OEIC_Msk             (1U << UART_UARTICR_OEIC_Pos)
#define UART_UARTICR_OEIC                 UART_UARTICR_OEIC_Msk
#define UART_UARTICR_BEIC_Pos             9U
#define UART_UARTICR_BEIC_Msk             (1U << UART_UARTICR_BEIC_Pos)
#define UART_UARTICR_BEIC                 UART_UARTICR_BEIC_Msk
#define UART_UARTICR_PEIC_Pos             8U
#define UART_UARTICR_PEIC_Msk             (1U << UART_UARTICR_PEIC_Pos)
#define UART_UARTICR_PEIC                 UART_UARTICR_PEIC_Msk
#define UART_UARTICR_FEIC_Pos             7U
#define UART_UARTICR_FEIC_Msk             (1U << UART_UARTICR_FEIC_Pos)
#define UART_UARTICR_FEIC                 UART_UARTICR_FEIC_Msk
#define UART_UARTICR_RTIC_Pos             6U
#define UART_UARTICR_RTIC_Msk             (1U << UART_UARTICR_RTIC_Pos)
#define UART_UARTICR_RTIC                 UART_UARTICR_RTIC_Msk
#define UART_UARTICR_TXIC_Pos             5U
#define UART_UARTICR_TXIC_Msk             (1U << UART_UARTICR_TXIC_Pos)
#define UART_UARTICR_TXIC                 UART_UARTICR_TXIC_Msk
#define UART_UARTICR_RXIC_Pos             4U
#define UART_UARTICR_RXIC_Msk             (1U << UART_UARTICR_RXIC_Pos)
#define UART_UARTICR_RXIC                 UART_UARTICR_RXIC_Msk
#define UART_UARTICR_DSRMIC_Pos           3U
#define UART_UARTICR_DSRMIC_Msk           (1U << UART_UARTICR_DSRMIC_Pos)
#define UART_UARTICR_DSRMIC               UART_UARTICR_DSRMIC_Msk
#define UART_UARTICR_DCDMIC_Pos           2U
#define UART_UARTICR_DCDMIC_Msk           (1U << UART_UARTICR_DCDMIC_Pos)
#define UART_UARTICR_DCDMIC               UART_UARTICR_DCDMIC_Msk
#define UART_UARTICR_CTSMIC_Pos           1U
#define UART_UARTICR_CTSMIC_Msk           (1U << UART_UARTICR_CTSMIC_Pos)
#define UART_UARTICR_CTSMIC               UART_UARTICR_CTSMIC_Msk
#define UART_UARTICR_RIMIC_Pos            0U
#define UART_UARTICR_RIMIC_Msk            (1U << UART_UARTICR_RIMIC_Pos)
#define UART_UARTICR_RIMIC                UART_UARTICR_RIMIC_Msk

#define UART_UARTDMACR_DMAONERR_Pos       2U
#define UART_UARTDMACR_DMAONERR_Msk       (1U << UART_UARTDMACR_DMAONERR_Pos)
#define UART_UARTDMACR_DMAONERR           UART_UARTDMACR_DMAONERR_Msk
#define UART_UARTDMACR_TXDMAE_Pos         1U
#define UART_UARTDMACR_TXDMAE_Msk         (1U << UART_UARTDMACR_TXDMAE_Pos)
#define UART_UARTDMACR_TXDMAE             UART_UARTDMACR_TXDMAE_Msk
#define UART_UARTDMACR_RXDMAE_Pos         0U
#define UART_UARTDMACR_RXDMAE_Msk         (1U << UART_UARTDMACR_RXDMAE_Pos)
#define UART_UARTDMACR_RXDMAE             UART_UARTDMACR_RXDMAE_Msk
/** @} */

/**
 * @name    RTC bits definitions
 * @{
 */
#define RTC_CLKDIV_M1_Pos                 0U
#define RTC_CLKDIV_M1_Msk                 (0xFFFFU << RTC_CLKDIV_M1_Pos)
#define RTC_CLKDIV_M1                     RTC_CLKDIV_M1_Msk

#define RTC_SETUP_0_YEAR_Pos              12U
#define RTC_SETUP_0_YEAR_Msk              (0xFFFU << RTC_SETUP_0_YEAR_Pos)
#define RTC_SETUP_0_YEAR(n)               ((n) << RTC_SETUP_0_YEAR_Pos)
#define RTC_SETUP_0_MONTH_Pos             8U
#define RTC_SETUP_0_MONTH_Msk             (0xFU << RTC_SETUP_0_MONTH_Pos)
#define RTC_SETUP_0_MONTH(n)              ((n) << RTC_SETUP_0_MONTH_Pos)
#define RTC_SETUP_0_DAY_Pos               0U
#define RTC_SETUP_0_DAY_Msk               (0x1FU << RTC_SETUP_0_DAY_Pos)
#define RTC_SETUP_0_DAY(n)                ((n) << RTC_SETUP_0_DAY_Pos)

#define RTC_SETUP_1_DOTW_Pos              24U
#define RTC_SETUP_1_DOTW_Msk              (0x7U << RTC_SETUP_1_DOTW_Pos)
#define RTC_SETUP_1_DOTW(n)               ((n) << RTC_SETUP_1_DOTW_Pos)
#define RTC_SETUP_1_HOUR_Pos              16U
#define RTC_SETUP_1_HOUR_Msk              (0x1FU << RTC_SETUP_1_HOUR_Pos)
#define RTC_SETUP_1_HOUR(n)               ((n) << RTC_SETUP_1_HOUR_Pos)
#define RTC_SETUP_1_MIN_Pos               8U
#define RTC_SETUP_1_MIN_Msk               (0x3FU << RTC_SETUP_1_MIN_Pos)
#define RTC_SETUP_1_MIN(n)                ((n) << RTC_SETUP_1_MIN_Pos)
#define RTC_SETUP_1_SEC_Pos               0U
#define RTC_SETUP_1_SEC_Msk               (0x3FU << RTC_SETUP_1_SEC_Pos)
#define RTC_SETUP_1_SEC(n)                ((n) << RTC_SETUP_1_SEC_Pos)

#define RTC_CTRL_FORCE_NOTLEAPYEAR_Pos    8U
#define RTC_CTRL_FORCE_NOTLEAPYEAR_Msk    (1U << RTC_CTRL_FORCE_NOTLEAPYEAR_Pos)
#define RTC_CTRL_FORCE_NOTLEAPYEAR        RTC_CTRL_FORCE_NOTLEAPYEAR_Msk

#define RTC_CTRL_LOAD_Pos                 4U
#define RTC_CTRL_LOAD_Msk                 (1U << RTC_CTRL_LOAD_Pos)
#define RTC_CTRL_LOAD                     RTC_CTRL_LOAD_Msk

#define RTC_CTRL_RTC_ACTIVE_Pos           1U
#define RTC_CTRL_RTC_ACTIVE_Msk           (1U << RTC_CTRL_RTC_ACTIVE_Pos)
#define RTC_CTRL_RTC_ACTIVE               RTC_CTRL_RTC_ACTIVE_Msk

#define RTC_CTRL_RTC_ENABLE_Pos           0U
#define RTC_CTRL_RTC_ENABLE_Msk           (1U << RTC_CTRL_RTC_ENABLE_Pos)
#define RTC_CTRL_RTC_ENABLE               RTC_CTRL_RTC_ENABLE_Msk

#define RTC_IRQ_SETUP_0_MATCH_ACTIVE_Pos  29U
#define RTC_IRQ_SETUP_0_MATCH_ACTIVE_Msk  (1U << RTC_IRQ_SETUP_0_MATCH_ACTIVE_Pos)
#define RTC_IRQ_SETUP_0_MATCH_ACTIVE      RTC_IRQ_SETUP_0_MATCH_ACTIVE_Msk
#define RTC_IRQ_SETUP_0_MATCH_ENA_Pos     28U
#define RTC_IRQ_SETUP_0_MATCH_ENA_Msk     (1U << RTC_IRQ_SETUP_0_MATCH_ENA_Pos)
#define RTC_IRQ_SETUP_0_MATCH_ENA         RTC_IRQ_SETUP_0_MATCH_ENA_Msk
#define RTC_IRQ_SETUP_0_YEAR_ENA_Pos      26U
#define RTC_IRQ_SETUP_0_YEAR_ENA_Msk      (1U << RTC_IRQ_SETUP_0_YEAR_ENA_Pos)
#define RTC_IRQ_SETUP_0_YEAR_ENA          RTC_IRQ_SETUP_0_YEAR_ENA_Msk
#define RTC_IRQ_SETUP_0_MONTH_ENA_Pos     25U
#define RTC_IRQ_SETUP_0_MONTH_ENA_Msk     (1U << RTC_IRQ_SETUP_0_MONTH_ENA_Pos)
#define RTC_IRQ_SETUP_0_MONTH_ENA         RTC_IRQ_SETUP_0_MONTH_ENA_Msk
#define RTC_IRQ_SETUP_0_DAY_ENA_Pos       24U
#define RTC_IRQ_SETUP_0_DAY_ENA_Msk       (1U << RTC_IRQ_SETUP_0_DAY_ENA_Pos)
#define RTC_IRQ_SETUP_0_DAY_ENA           RTC_IRQ_SETUP_0_DAY_ENA_Msk

#define RTC_IRQ_SETUP_0_YEAR_Pos          12U
#define RTC_IRQ_SETUP_0_YEAR_Msk          (0xFFFU << RTC_IRQ_SETUP_0_YEAR_Pos)
#define RTC_IRQ_SETUP_0_YEAR(n)           ((n) << RTC_IRQ_SETUP_0_YEAR_Pos)
#define RTC_IRQ_SETUP_0_MONTH_Pos         8U
#define RTC_IRQ_SETUP_0_MONTH_Msk         (0xFU << RTC_IRQ_SETUP_0_MONTH_Pos)
#define RTC_IRQ_SETUP_0_MONTH(n)          ((n) << RTC_IRQ_SETUP_0_MONTH_Pos)
#define RTC_IRQ_SETUP_0_DAY_Pos           0U
#define RTC_IRQ_SETUP_0_DAY_Msk           (0x1FU << RTC_IRQ_SETUP_0_DAY_Pos)
#define RTC_IRQ_SETUP_0_DAY(n)            ((n) << RTC_IRQ_SETUP_0_DAY_Pos)

#define RTC_IRQ_SETUP_1_DOTW_ENA_Pos      31U
#define RTC_IRQ_SETUP_1_DOTW_ENA_Msk      (1U << RTC_IRQ_SETUP_1_DOTW_ENA_Pos)
#define RTC_IRQ_SETUP_1_DOTW_ENA          RTC_IRQ_SETUP_1_DOTW_ENA_Msk
#define RTC_IRQ_SETUP_1_HOUR_ENA_Pos      30U
#define RTC_IRQ_SETUP_1_HOUR_ENA_Msk      (1U << RTC_IRQ_SETUP_1_HOUR_ENA_Pos)
#define RTC_IRQ_SETUP_1_HOUR_ENA          RTC_IRQ_SETUP_1_HOUR_ENA_Msk
#define RTC_IRQ_SETUP_1_MIN_ENA_Pos       29U
#define RTC_IRQ_SETUP_1_MIN_ENA_Msk       (1U << RTC_IRQ_SETUP_1_MIN_ENA_Pos)
#define RTC_IRQ_SETUP_1_MIN_ENA           RTC_IRQ_SETUP_1_MIN_ENA_Msk
#define RTC_IRQ_SETUP_1_SEC_ENA_Pos       28U
#define RTC_IRQ_SETUP_1_SEC_ENA_Msk       (1U << RTC_IRQ_SETUP_1_SEC_ENA_Pos)
#define RTC_IRQ_SETUP_1_SEC_ENA           RTC_IRQ_SETUP_1_SEC_ENA_Msk

#define RTC_IRQ_SETUP_1_DOTW_Pos          24U
#define RTC_IRQ_SETUP_1_DOTW_Msk          (0x7U << RTC_IRQ_SETUP_1_DOTW_Pos)
#define RTC_IRQ_SETUP_1_DOTW(n)           ((n) << RTC_IRQ_SETUP_1_DOTW_Pos)
#define RTC_IRQ_SETUP_1_HOUR_Pos          16U
#define RTC_IRQ_SETUP_1_HOUR_Msk          (0x1FU << RTC_IRQ_SETUP_1_HOUR_Pos)
#define RTC_IRQ_SETUP_1_HOUR(n)           ((n) << RTC_IRQ_SETUP_1_HOUR_Pos)
#define RTC_IRQ_SETUP_1_MIN_Pos           8U
#define RTC_IRQ_SETUP_1_MIN_Msk           (0x3FU << RTC_IRQ_SETUP_1_MIN_Pos)
#define RTC_IRQ_SETUP_1_MIN(n)            ((n) << RTC_IRQ_SETUP_1_MIN_Pos)
#define RTC_IRQ_SETUP_1_SEC_Pos           0U
#define RTC_IRQ_SETUP_1_SEC_Msk           (0x3FU << RTC_IRQ_SETUP_1_SEC_Pos)
#define RTC_IRQ_SETUP_1_SEC(n)            ((n) << RTC_IRQ_SETUP_1_SEC_Pos)

#define RTC_INTR_RTC_Pos                  0U
#define RTC_INTR_RTC_Msk                  (1U << RTC_INTR_RTC_Pos)
#define RTC_INTR_RTC                      RTC_INTR_RTC_Msk

#define RTC_INTE_RTC_Pos                  0U
#define RTC_INTE_RTC_Msk                  (1U << RTC_INTE_RTC_Pos)
#define RTC_INTE_RTC                      RTC_INTE_RTC_Msk

#define RTC_INTF_RTC_Pos                  0U
#define RTC_INTF_RTC_Msk                  (1U << RTC_INTF_RTC_Pos)
#define RTC_INTF_RTC                      RTC_INTF_RTC_Msk

#define RTC_INTS_RTC_Pos                  0U
#define RTC_INTS_RTC_Msk                  (1U << RTC_INTS_RTC_Pos)
#define RTC_INTS_RTC                      RTC_INTS_RTC_Msk

/* Normalisation of RTC0 & RTC1 fields read from RTC. */
#define RTC_RTC_1_YEAR_Pos                12U
#define RTC_RTC_1_YEAR_Msk                (0xFFFU << RTC_RTC_1_YEAR_Pos)
#define RTC_RTC_1_YEAR(n)                 ((n & RTC_RTC_1_YEAR_Msk) >>       \
                                            RTC_RTC_1_YEAR_Pos)
#define RTC_RTC_1_MONTH_Pos               8U
#define RTC_RTC_1_MONTH_Msk               (0xFU << RTC_RTC_1_MONTH_Pos)
#define RTC_RTC_1_MONTH(n)                ((n & RTC_RTC_1_MONTH_Msk) >>      \
                                            RTC_RTC_1_MONTH_Pos)
#define RTC_RTC_1_DAY_Pos                 0U
#define RTC_RTC_1_DAY_Msk                 (0x1FU << RTC_RTC_1_DAY_Pos)
#define RTC_RTC_1_DAY(n)                  ((n & RTC_RTC_1_DAY_Msk) >>        \
                                            RTC_RTC_1_DAY_Pos)
#define RTC_RTC_0_DOTW_Pos                24U
#define RTC_RTC_0_DOTW_Msk                (0x7U << RTC_RTC_0_DOTW_Pos)
#define RTC_RTC_0_DOTW(n)                 ((n & RTC_RTC_0_DOTW_Msk) >>       \
                                            RTC_RTC_0_DOTW_Pos)
#define RTC_RTC_0_HOUR_Pos                16U
#define RTC_RTC_0_HOUR_Msk                (0x1FU << RTC_RTC_0_HOUR_Pos)
#define RTC_RTC_0_HOUR(n)                 ((n & RTC_RTC_0_HOUR_Msk) >>       \
                                            RTC_RTC_0_HOUR_Pos)
#define RTC_RTC_0_MIN_Pos                 8U
#define RTC_RTC_0_MIN_Msk                 (0x3FU << RTC_RTC_0_MIN_Pos)
#define RTC_RTC_0_MIN(n)                  ((n & RTC_RTC_0_MIN_Msk) >>        \
                                            RTC_RTC_0_MIN_Pos)
#define RTC_RTC_0_SEC_Pos                 0U
#define RTC_RTC_0_SEC_Msk                 (0x3FU << RTC_RTC_0_SEC_Pos)
#define RTC_RTC_0_SEC(n)                  ((n & RTC_RTC_0_SEC_Msk) >>        \
                                            RTC_RTC_0_SEC_Pos)
/** @} */

/**
 * @name    WATCHDOG bits definitions
 * @{
 */
#define WATCHDOG_CTRL_TRIGGER_Pos         31U
#define WATCHDOG_CTRL_TRIGGER_Msk         (1U << WATCHDOG_CTRL_TRIGGER_Pos)
#define WATCHDOG_CTRL_TRIGGER             WATCHDOG_CTRL_TRIGGER_Msk
#define WATCHDOG_CTRL_ENABLE_Pos          30U
#define WATCHDOG_CTRL_ENABLE_Msk          (1U << WATCHDOG_CTRL_ENABLE_Pos)
#define WATCHDOG_CTRL_ENABLE              WATCHDOG_CTRL_ENABLE_Msk
#define WATCHDOG_CTRL_PAUSE_DBG1_Pos      26U
#define WATCHDOG_CTRL_PAUSE_DBG1_Msk      (1U << WATCHDOG_CTRL_PAUSE_DBG1_Pos)
#define WATCHDOG_CTRL_PAUSE_DBG1          WATCHDOG_CTRL_PAUSE_DBG1_Msk
#define WATCHDOG_CTRL_PAUSE_DBG0_Pos      25U
#define WATCHDOG_CTRL_PAUSE_DBG0_Msk      (1U << WATCHDOG_CTRL_PAUSE_DBG0_Pos)
#define WATCHDOG_CTRL_PAUSE_DBG0          WATCHDOG_CTRL_PAUSE_DBG0_Msk
#define WATCHDOG_CTRL_PAUSE_JTAG_Pos      24U
#define WATCHDOG_CTRL_PAUSE_JTAG_Msk      (1U << WATCHDOG_CTRL_PAUSE_JTAG_Pos)
#define WATCHDOG_CTRL_PAUSE_JTAG          WATCHDOG_CTRL_PAUSE_JTAG_Msk
#define WATCHDOG_CTRL_TIME_Pos            0U
#define WATCHDOG_CTRL_TIME_Msk            (0xFFFFFFU << WATCHDOG_CTRL_TIME_Pos)
#define WATCHDOG_CTRL_TIME                WATCHDOG_CTRL_TIME_Msk

#define WATCHDOG_LOAD_Pos                 0U
#define WATCHDOG_LOAD_Msk                 (0xFFFFFFU << WATCHDOG_LOAD_Pos)
#define WATCHDOG_LOAD                     WATCHDOG_LOAD_Msk

#define WATCHDOG_REASON_FORCE_Pos         1U
#define WATCHDOG_REASON_FORCE_Msk         (1U << WATCHDOG_REASON_FORCE_Pos)
#define WATCHDOG_REASON_FORCE             WATCHDOG_REASON_FORCE_Msk
#define WATCHDOG_REASON_TIMER_Pos         0U
#define WATCHDOG_REASON_TIMER_Msk         (1U << WATCHDOG_REASON_TIMER_Pos)
#define WATCHDOG_REASON_TIMER             WATCHDOG_REASON_TIMER_Msk

#define WATCHDOG_TICK_COUNT_Pos           11U
#define WATCHDOG_TICK_COUNT_Msk           (0xFF800U << WATCHDOG_TICK_COUNT_Pos)
#define WATCHDOG_TICK_COUNT               WATCHDOG_TICK_COUNT_Msk
#define WATCHDOG_TICK_RUNNING_Pos         10U
#define WATCHDOG_TICK_RUNNING_Msk         (1U << WATCHDOG_TICK_RUNNING_Pos)
#define WATCHDOG_TICK_RUNNING             WATCHDOG_TICK_RUNNING_Msk
#define WATCHDOG_TICK_ENABLE_Pos          9U
#define WATCHDOG_TICK_ENABLE_Msk          (1U << WATCHDOG_TICK_ENABLE_Pos)
#define WATCHDOG_TICK_ENABLE              WATCHDOG_TICK_ENABLE_Msk
#define WATCHDOG_TICK_CYCLES_Pos          0U
#define WATCHDOG_TICK_CYCLES_Msk          (0x1FFU << WATCHDOG_TICK_CYCLES_Pos)
#define WATCHDOG_TICK_CYCLES              WATCHDOG_TICK_CYCLES_Msk
/** @} */

/**
 * @name    I2C bits definitions
 * @{
 */
#define I2C_IC_CON_STOP_DET_IF_MASTER_ACTIVE_Pos         10U
#define I2C_IC_CON_STOP_DET_IF_MASTER_ACTIVE_Msk         (1U << I2C_IC_CON_STOP_DET_IF_MASTER_ACTIVE_Pos)
#define I2C_IC_CON_STOP_DET_IF_MASTER_ACTIVE             I2C_IC_CON_STOP_DET_IF_MASTER_ACTIVE_Msk
#define I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_Pos             9U
#define I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_Msk             (1U << I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_Pos)
#define I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL                 I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_Msk
#define I2C_IC_CON_TX_EMPTY_CTRL_Pos                     8U
#define I2C_IC_CON_TX_EMPTY_CTRL_Msk                     (1U << I2C_IC_CON_TX_EMPTY_CTRL_Pos)
#define I2C_IC_CON_TX_EMPTY_CTRL                         I2C_IC_CON_TX_EMPTY_CTRL_Msk
#define I2C_IC_CON_STOP_DET_IFADDRESSED_Pos              7U
#define I2C_IC_CON_STOP_DET_IFADDRESSED_Msk              (1U << I2C_IC_CON_STOP_DET_IFADDRESSED_Pos)
#define I2C_IC_CON_STOP_DET_IFADDRESSED                  I2C_IC_CON_STOP_DET_IFADDRESSED_Msk
#define I2C_IC_CON_IC_SLAVE_DISABLE_Pos                  6U
#define I2C_IC_CON_IC_SLAVE_DISABLE_Msk                  (1U << I2C_IC_CON_IC_SLAVE_DISABLE_Pos)
#define I2C_IC_CON_IC_SLAVE_DISABLE                      I2C_IC_CON_IC_SLAVE_DISABLE_Msk
#define I2C_IC_CON_IC_RESTART_EN_Pos                     5U
#define I2C_IC_CON_IC_RESTART_EN_Msk                     (1U << I2C_IC_CON_IC_RESTART_EN_Pos)
#define I2C_IC_CON_IC_RESTART_EN                         I2C_IC_CON_IC_RESTART_EN_Msk
#define I2C_IC_CON_IC_10BITADDR_MASTER_Pos               4U
#define I2C_IC_CON_IC_10BITADDR_MASTER_Msk               (1U << I2C_IC_CON_IC_10BITADDR_MASTER_Pos)
#define I2C_IC_CON_IC_10BITADDR_MASTER                   I2C_IC_CON_IC_10BITADDR_MASTER_Msk
#define I2C_IC_CON_IC_10BITADDR_SLAVE_Pos                3U
#define I2C_IC_CON_IC_10BITADDR_SLAVE_Msk                (1U << I2C_IC_CON_IC_10BITADDR_SLAVE_Pos)
#define I2C_IC_CON_IC_10BITADDR_SLAVE                    I2C_IC_CON_IC_10BITADDR_SLAVE_Msk
#define I2C_IC_CON_SPEED_Pos                             1U
#define I2C_IC_CON_SPEED_Msk                             (3U << I2C_IC_CON_SPEED_Pos)
#define I2C_IC_CON_SPEED                                 I2C_IC_CON_SPEED_Msk
#define I2C_IC_CON_MASTER_MODE_Pos                       0U
#define I2C_IC_CON_MASTER_MODE_Msk                       (1U << I2C_IC_CON_MASTER_MODE_Pos)
#define I2C_IC_CON_MASTER_MODE                           I2C_IC_CON_MASTER_MODE_Msk

#define I2C_IC_TAR_SPECIAL_Pos                           11U
#define I2C_IC_TAR_SPECIAL_Msk                           (1U << I2C_IC_TAR_SPECIAL_Pos)
#define I2C_IC_TAR_SPECIAL                               I2C_IC_TAR_SPECIAL_Msk
#define I2C_IC_TAR_GC_OR_START_Pos                       10U
#define I2C_IC_TAR_GC_OR_START_Msk                       (1U << I2C_IC_TAR_GC_OR_START_Pos)
#define I2C_IC_TAR_GC_OR_START                           I2C_IC_TAR_GC_OR_START_Msk
#define I2C_IC_TAR_IC_TAR_Pos                            0U
#define I2C_IC_TAR_IC_TAR_Msk                            (0x3FFU << I2C_IC_TAR_IC_TAR_Pos)
#define I2C_IC_TAR_IC_TAR                                I2C_IC_TAR_IC_TAR_Msk

#define I2C_IC_SAR_IC_SAR_Pos                            0U
#define I2C_IC_SAR_IC_SAR_Msk                            (0x3FFU << I2C_IC_SAR_IC_SAR_Pos)
#define I2C_IC_SAR_IC_SAR                                I2C_IC_SAR_IC_SAR_Msk

#define I2C_IC_DATA_CMD_FIRST_DATA_BYTE_Pos              11U
#define I2C_IC_DATA_CMD_FIRST_DATA_BYTE_Msk              (1U << I2C_IC_DATA_CMD_FIRST_DATA_BYTE_Pos)
#define I2C_IC_DATA_CMD_FIRST_DATA_BYTE                  I2C_IC_DATA_CMD_FIRST_DATA_BYTE_Msk
#define I2C_IC_DATA_CMD_RESTART_Pos                      10U
#define I2C_IC_DATA_CMD_RESTART_Msk                      (1U << I2C_IC_DATA_CMD_RESTART_Pos)
#define I2C_IC_DATA_CMD_RESTART                          I2C_IC_DATA_CMD_RESTART_Msk
#define I2C_IC_DATA_CMD_STOP_Pos                         9U
#define I2C_IC_DATA_CMD_STOP_Msk                         (1U << I2C_IC_DATA_CMD_STOP_Pos)
#define I2C_IC_DATA_CMD_STOP                             I2C_IC_DATA_CMD_STOP_Msk
#define I2C_IC_DATA_CMD_CMD_Pos                          8U
#define I2C_IC_DATA_CMD_CMD_Msk                          (1U << I2C_IC_DATA_CMD_CMD_Pos)
#define I2C_IC_DATA_CMD_CMD                              I2C_IC_DATA_CMD_CMD_Msk
#define I2C_IC_DATA_CMD_DAT_Pos                          0U
#define I2C_IC_DATA_CMD_DAT_Msk                          (0xFFU << I2C_IC_DATA_CMD_DAT_Pos)
#define I2C_IC_DATA_CMD_DAT                              I2C_IC_DATA_CMD_DAT_Msk

#define I2C_IC_SS_SCL_HCNT_Pos                           0U
#define I2C_IC_SS_SCL_HCNT_Msk                           (0xFFFFU << I2C_IC_SS_SCL_HCNT_Pos)
#define I2C_IC_SS_SCL_HCNT                               I2C_IC_SS_SCL_HCNT_Msk

#define I2C_IC_SS_SCL_LCNT_Pos                           0U
#define I2C_IC_SS_SCL_LCNT_Msk                           (0xFFFFU << I2C_IC_SS_SCL_LCNT_Pos)
#define I2C_IC_SS_SCL_LCNT                               I2C_IC_SS_SCL_LCNT_Msk

#define I2C_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT_Pos            0U
#define I2C_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT_Msk            (0xFFFFU << I2C_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT_Pos)
#define I2C_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT                I2C_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT_Msk

#define I2C_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT_Pos            0U
#define I2C_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT_Msk            (0xFFFFU << I2C_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT_Pos)
#define I2C_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT                I2C_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT_Msk

#define I2C_IC_FS_SCL_HCNT_Pos                           0U
#define I2C_IC_FS_SCL_HCNT_Msk                           (0xFFFFU << I2C_IC_FS_SCL_HCNT_Pos)
#define I2C_IC_FS_SCL_HCNT                               I2C_IC_FS_SCL_HCNT_Msk

#define I2C_IC_FS_SCL_LCNT_Pos                           0U
#define I2C_IC_FS_SCL_LCNT_Msk                           (0xFFFFU << I2C_IC_FS_SCL_LCNT_Pos)
#define I2C_IC_FS_SCL_LCNT                               I2C_IC_FS_SCL_LCNT_Msk

#define I2C_IC_FS_SCL_HCNT_IC_FS_SCL_HCNT_Pos            0U
#define I2C_IC_FS_SCL_HCNT_IC_FS_SCL_HCNT_Msk            (0xFFFFU << I2C_IC_FS_SCL_HCNT_IC_FS_SCL_HCNT_Pos)
#define I2C_IC_FS_SCL_HCNT_IC_FS_SCL_HCNT                I2C_IC_FS_SCL_HCNT_IC_FS_SCL_HCNT_Msk

#define I2C_IC_FS_SCL_LCNT_IC_FS_SCL_LCNT_Pos            0U
#define I2C_IC_FS_SCL_LCNT_IC_FS_SCL_LCNT_Msk            (0xFFFFU << I2C_IC_FS_SCL_LCNT_IC_FS_SCL_LCNT_Pos)
#define I2C_IC_FS_SCL_LCNT_IC_FS_SCL_LCNT                I2C_IC_FS_SCL_LCNT_IC_FS_SCL_LCNT_Msk

#define I2C_IC_INTR_STAT_R_MASTER_ON_HOLD_Pos            13U
#define I2C_IC_INTR_STAT_R_MASTER_ON_HOLD_Msk            (1U << I2C_IC_INTR_STAT_R_MASTER_ON_HOLD_Pos)
#define I2C_IC_INTR_STAT_R_MASTER_ON_HOLD                I2C_IC_INTR_STAT_R_MASTER_ON_HOLD_Msk
#define I2C_IC_INTR_STAT_R_RESTART_DET_Pos               12U
#define I2C_IC_INTR_STAT_R_RESTART_DET_Msk               (1U << I2C_IC_INTR_STAT_R_RESTART_DET_Pos)
#define I2C_IC_INTR_STAT_R_RESTART_DET                   I2C_IC_INTR_STAT_R_RESTART_DET_Msk
#define I2C_IC_INTR_STAT_R_GEN_CALL_Pos                  11U
#define I2C_IC_INTR_STAT_R_GEN_CALL_Msk                  (1U << I2C_IC_INTR_STAT_R_GEN_CALL_Pos)
#define I2C_IC_INTR_STAT_R_GEN_CALL                      I2C_IC_INTR_STAT_R_GEN_CALL_Msk
#define I2C_IC_INTR_STAT_R_START_DET_Pos                 10U
#define I2C_IC_INTR_STAT_R_START_DET_Msk                 (1U << I2C_IC_INTR_STAT_R_START_DET_Pos)
#define I2C_IC_INTR_STAT_R_START_DET                     I2C_IC_INTR_STAT_R_START_DET_Msk
#define I2C_IC_INTR_STAT_R_STOP_DET_Pos                  9U
#define I2C_IC_INTR_STAT_R_STOP_DET_Msk                  (1U << I2C_IC_INTR_STAT_R_STOP_DET_Pos)
#define I2C_IC_INTR_STAT_R_STOP_DET                      I2C_IC_INTR_STAT_R_STOP_DET_Msk
#define I2C_IC_INTR_STAT_R_ACTIVITY_Pos                  8U
#define I2C_IC_INTR_STAT_R_ACTIVITY_Msk                  (1U << I2C_IC_INTR_STAT_R_ACTIVITY_Pos)
#define I2C_IC_INTR_STAT_R_ACTIVITY                      I2C_IC_INTR_STAT_R_ACTIVITY_Msk
#define I2C_IC_INTR_STAT_R_RX_DONE_Pos                   7U
#define I2C_IC_INTR_STAT_R_RX_DONE_Msk                   (1U << I2C_IC_INTR_STAT_R_RX_DONE_Pos)
#define I2C_IC_INTR_STAT_R_RX_DONE                       I2C_IC_INTR_STAT_R_RX_DONE_Msk
#define I2C_IC_INTR_STAT_R_TX_ABRT_Pos                   6U
#define I2C_IC_INTR_STAT_R_TX_ABRT_Msk                   (1U << I2C_IC_INTR_STAT_R_TX_ABRT_Pos)
#define I2C_IC_INTR_STAT_R_TX_ABRT                       I2C_IC_INTR_STAT_R_TX_ABRT_Msk
#define I2C_IC_INTR_STAT_R_RD_REQ_Pos                    5U
#define I2C_IC_INTR_STAT_R_RD_REQ_Msk                    (1U << I2C_IC_INTR_STAT_R_RD_REQ_Pos)
#define I2C_IC_INTR_STAT_R_RD_REQ                        I2C_IC_INTR_STAT_R_RD_REQ_Msk
#define I2C_IC_INTR_STAT_R_TX_EMPTY_Pos                  4U
#define I2C_IC_INTR_STAT_R_TX_EMPTY_Msk                  (1U << I2C_IC_INTR_STAT_R_TX_EMPTY_Pos)
#define I2C_IC_INTR_STAT_R_TX_EMPTY                      I2C_IC_INTR_STAT_R_TX_EMPTY_Msk
#define I2C_IC_INTR_STAT_R_TX_OVER_Pos                   3U
#define I2C_IC_INTR_STAT_R_TX_OVER_Msk                   (1U << I2C_IC_INTR_STAT_R_TX_OVER_Pos)
#define I2C_IC_INTR_STAT_R_TX_OVER                       I2C_IC_INTR_STAT_R_TX_OVER_Msk
#define I2C_IC_INTR_STAT_R_RX_FULL_Pos                   2U
#define I2C_IC_INTR_STAT_R_RX_FULL_Msk                   (1U << I2C_IC_INTR_STAT_R_RX_FULL_Pos)
#define I2C_IC_INTR_STAT_R_RX_FULL                       I2C_IC_INTR_STAT_R_RX_FULL_Msk
#define I2C_IC_INTR_STAT_R_RX_OVER_Pos                   1U
#define I2C_IC_INTR_STAT_R_RX_OVER_Msk                   (1U << I2C_IC_INTR_STAT_R_RX_OVER_Pos)
#define I2C_IC_INTR_STAT_R_RX_OVER                       I2C_IC_INTR_STAT_R_RX_OVER_Msk
#define I2C_IC_INTR_STAT_R_RX_UNDER_Pos                  0U
#define I2C_IC_INTR_STAT_R_RX_UNDER_Msk                  (1U << I2C_IC_INTR_STAT_R_RX_UNDER_Pos)
#define I2C_IC_INTR_STAT_R_RX_UNDER                      I2C_IC_INTR_STAT_R_RX_UNDER_Msk

#define I2C_IC_INTR_MASK_M_MASTER_ON_HOLD_Pos            13U
#define I2C_IC_INTR_MASK_M_MASTER_ON_HOLD_Msk            (1U << I2C_IC_INTR_MASK_M_MASTER_ON_HOLD_Pos)
#define I2C_IC_INTR_MASK_M_MASTER_ON_HOLD                I2C_IC_INTR_MASK_M_MASTER_ON_HOLD_Msk
#define I2C_IC_INTR_MASK_M_MASTER_ON_HOLD_READ_ONLY      I2C_IC_INTR_MASK_M_MASTER_ON_HOLD
#define I2C_IC_INTR_MASK_M_RESTART_DET_Pos               12U
#define I2C_IC_INTR_MASK_M_RESTART_DET_Msk               (1U << I2C_IC_INTR_MASK_M_RESTART_DET_Pos)
#define I2C_IC_INTR_MASK_M_RESTART_DET                   I2C_IC_INTR_MASK_M_RESTART_DET_Msk
#define I2C_IC_INTR_MASK_M_GEN_CALL_Pos                  11U
#define I2C_IC_INTR_MASK_M_GEN_CALL_Msk                  (1U << I2C_IC_INTR_MASK_M_GEN_CALL_Pos)
#define I2C_IC_INTR_MASK_M_GEN_CALL                      I2C_IC_INTR_MASK_M_GEN_CALL_Msk
#define I2C_IC_INTR_MASK_M_START_DET_Pos                 10U
#define I2C_IC_INTR_MASK_M_START_DET_Msk                 (1U << I2C_IC_INTR_MASK_M_START_DET_Pos)
#define I2C_IC_INTR_MASK_M_START_DET                     I2C_IC_INTR_MASK_M_START_DET_Msk
#define I2C_IC_INTR_MASK_M_STOP_DET_Pos                  9U
#define I2C_IC_INTR_MASK_M_STOP_DET_Msk                  (1U << I2C_IC_INTR_MASK_M_STOP_DET_Pos)
#define I2C_IC_INTR_MASK_M_STOP_DET                      I2C_IC_INTR_MASK_M_STOP_DET_Msk
#define I2C_IC_INTR_MASK_M_ACTIVITY_Pos                  8U
#define I2C_IC_INTR_MASK_M_ACTIVITY_Msk                  (1U << I2C_IC_INTR_MASK_M_ACTIVITY_Pos)
#define I2C_IC_INTR_MASK_M_ACTIVITY                      I2C_IC_INTR_MASK_M_ACTIVITY_Msk
#define I2C_IC_INTR_MASK_M_RX_DONE_Pos                   7U
#define I2C_IC_INTR_MASK_M_RX_DONE_Msk                   (1U << I2C_IC_INTR_MASK_M_RX_DONE_Pos)
#define I2C_IC_INTR_MASK_M_RX_DONE                       I2C_IC_INTR_MASK_M_RX_DONE_Msk
#define I2C_IC_INTR_MASK_M_TX_ABRT_Pos                   6U
#define I2C_IC_INTR_MASK_M_TX_ABRT_Msk                   (1U << I2C_IC_INTR_MASK_M_TX_ABRT_Pos)
#define I2C_IC_INTR_MASK_M_TX_ABRT                       I2C_IC_INTR_MASK_M_TX_ABRT_Msk
#define I2C_IC_INTR_MASK_M_RD_REQ_Pos                    5U
#define I2C_IC_INTR_MASK_M_RD_REQ_Msk                    (1U << I2C_IC_INTR_MASK_M_RD_REQ_Pos)
#define I2C_IC_INTR_MASK_M_RD_REQ                        I2C_IC_INTR_MASK_M_RD_REQ_Msk
#define I2C_IC_INTR_MASK_M_TX_EMPTY_Pos                  4U
#define I2C_IC_INTR_MASK_M_TX_EMPTY_Msk                  (1U << I2C_IC_INTR_MASK_M_TX_EMPTY_Pos)
#define I2C_IC_INTR_MASK_M_TX_EMPTY                      I2C_IC_INTR_MASK_M_TX_EMPTY_Msk
#define I2C_IC_INTR_MASK_M_TX_OVER_Pos                   3U
#define I2C_IC_INTR_MASK_M_TX_OVER_Msk                   (1U << I2C_IC_INTR_MASK_M_TX_OVER_Pos)
#define I2C_IC_INTR_MASK_M_TX_OVER                       I2C_IC_INTR_MASK_M_TX_OVER_Msk
#define I2C_IC_INTR_MASK_M_RX_FULL_Pos                   2U
#define I2C_IC_INTR_MASK_M_RX_FULL_Msk                   (1U << I2C_IC_INTR_MASK_M_RX_FULL_Pos)
#define I2C_IC_INTR_MASK_M_RX_FULL                       I2C_IC_INTR_MASK_M_RX_FULL_Msk
#define I2C_IC_INTR_MASK_M_RX_OVER_Pos                   1U
#define I2C_IC_INTR_MASK_M_RX_OVER_Msk                   (1U << I2C_IC_INTR_MASK_M_RX_OVER_Pos)
#define I2C_IC_INTR_MASK_M_RX_OVER                       I2C_IC_INTR_MASK_M_RX_OVER_Msk
#define I2C_IC_INTR_MASK_M_RX_UNDER_Pos                  0U
#define I2C_IC_INTR_MASK_M_RX_UNDER_Msk                  (1U << I2C_IC_INTR_MASK_M_RX_UNDER_Pos)
#define I2C_IC_INTR_MASK_M_RX_UNDER                      I2C_IC_INTR_MASK_M_RX_UNDER_Msk
#define I2C_IC_RAW_INTR_STAT_MASTER_ON_HOLD_Pos          13U
#define I2C_IC_RAW_INTR_STAT_MASTER_ON_HOLD_Msk          (1U << I2C_IC_RAW_INTR_STAT_MASTER_ON_HOLD_Pos)
#define I2C_IC_RAW_INTR_STAT_MASTER_ON_HOLD              I2C_IC_RAW_INTR_STAT_MASTER_ON_HOLD_Msk
#define I2C_IC_INTR_STAT_M_RESTART_DET_Pos               12U
#define I2C_IC_INTR_STAT_M_RESTART_DET_Msk               (1U << I2C_IC_INTR_STAT_M_RESTART_DET_Pos)
#define I2C_IC_INTR_STAT_M_RESTART_DET                   I2C_IC_INTR_STAT_M_RESTART_DET_Msk
#define I2C_IC_INTR_STAT_M_GEN_CALL_Pos                  11U
#define I2C_IC_INTR_STAT_M_GEN_CALL_Msk                  (1U << I2C_IC_INTR_STAT_M_GEN_CALL_Pos)
#define I2C_IC_INTR_STAT_M_GEN_CALL                      I2C_IC_INTR_STAT_M_GEN_CALL_Msk
#define I2C_IC_INTR_STAT_M_START_DET_Pos                 10U
#define I2C_IC_INTR_STAT_M_START_DET_Msk                 (1U << I2C_IC_INTR_STAT_M_START_DET_Pos)
#define I2C_IC_INTR_STAT_M_START_DET                     I2C_IC_INTR_STAT_M_START_DET_Msk
#define I2C_IC_INTR_STAT_M_STOP_DET_Pos                  9U
#define I2C_IC_INTR_STAT_M_STOP_DET_Msk                  (1U << I2C_IC_INTR_STAT_M_STOP_DET_Pos)
#define I2C_IC_INTR_STAT_M_STOP_DET                      I2C_IC_INTR_STAT_M_STOP_DET_Msk
#define I2C_IC_INTR_STAT_M_ACTIVITY_Pos                  8U
#define I2C_IC_INTR_STAT_M_ACTIVITY_Msk                  (1U << I2C_IC_INTR_STAT_M_ACTIVITY_Pos)
#define I2C_IC_INTR_STAT_M_ACTIVITY                      I2C_IC_INTR_STAT_M_ACTIVITY_Msk
#define I2C_IC_INTR_STAT_M_RX_DONE_Pos                   7U
#define I2C_IC_INTR_STAT_M_RX_DONE_Msk                   (1U << I2C_IC_INTR_STAT_M_RX_DONE_Pos)
#define I2C_IC_INTR_STAT_M_RX_DONE                       I2C_IC_INTR_STAT_M_RX_DONE_Msk
#define I2C_IC_INTR_STAT_M_TX_ABRT_Pos                   6U
#define I2C_IC_INTR_STAT_M_TX_ABRT_Msk                   (1U << I2C_IC_INTR_STAT_M_TX_ABRT_Pos)
#define I2C_IC_INTR_STAT_M_TX_ABRT                       I2C_IC_INTR_STAT_M_TX_ABRT_Msk
#define I2C_IC_INTR_STAT_M_RD_REQ_Pos                    5U
#define I2C_IC_INTR_STAT_M_RD_REQ_Msk                    (1U << I2C_IC_INTR_STAT_M_RD_REQ_Pos)
#define I2C_IC_INTR_STAT_M_RD_REQ                        I2C_IC_INTR_STAT_M_RD_REQ_Msk
#define I2C_IC_INTR_STAT_M_TX_EMPTY_Pos                  4U
#define I2C_IC_INTR_STAT_M_TX_EMPTY_Msk                  (1U << I2C_IC_INTR_STAT_M_TX_EMPTY_Pos)
#define I2C_IC_INTR_STAT_M_TX_EMPTY                      I2C_IC_INTR_STAT_M_TX_EMPTY_Msk
#define I2C_IC_INTR_STAT_M_TX_OVER_Pos                   3U
#define I2C_IC_INTR_STAT_M_TX_OVER_Msk                   (1U << I2C_IC_INTR_STAT_M_TX_OVER_Pos)
#define I2C_IC_INTR_STAT_M_TX_OVER                       I2C_IC_INTR_STAT_M_TX_OVER_Msk
#define I2C_IC_INTR_STAT_M_RX_FULL_Pos                   2U
#define I2C_IC_INTR_STAT_M_RX_FULL_Msk                   (1U << I2C_IC_INTR_STAT_M_RX_FULL_Pos)
#define I2C_IC_INTR_STAT_M_RX_FULL                       I2C_IC_INTR_STAT_M_RX_FULL_Msk
#define I2C_IC_INTR_STAT_M_RX_OVER_Pos                   1U
#define I2C_IC_INTR_STAT_M_RX_OVER_Msk                   (1U << I2C_IC_INTR_STAT_M_RX_OVER_Pos)
#define I2C_IC_INTR_STAT_M_RX_OVER                       I2C_IC_INTR_STAT_M_RX_OVER_Msk
#define I2C_IC_INTR_STAT_M_RX_UNDER_Pos                  0U
#define I2C_IC_INTR_STAT_M_RX_UNDER_Msk                  (1U << I2C_IC_INTR_STAT_M_RX_UNDER_Pos)
#define I2C_IC_INTR_STAT_M_RX_UNDER                      I2C_IC_INTR_STAT_M_RX_UNDER_Msk

#define I2C_IC_RX_TL_RX_TL_Pos                           0U
#define I2C_IC_RX_TL_RX_TL_Msk                           (0xFFU << I2C_IC_RX_TL_RX_TL_Pos)
#define I2C_IC_RX_TL_RX_TL                               I2C_IC_RX_TL_RX_TL_Msk

#define I2C_IC_TX_TL_TX_TL_Pos                           0U
#define I2C_IC_TX_TL_TX_TL_Msk                           (0xFFU << I2C_IC_TX_TL_TX_TL_Pos)
#define I2C_IC_TX_TL_TX_TL                               I2C_IC_TX_TL_TX_TL_Msk

#define I2C_IC_ENABLE_TX_CMD_BLOCK_Pos                   2U
#define I2C_IC_ENABLE_TX_CMD_BLOCK_Msk                   (1U << I2C_IC_ENABLE_TX_CMD_BLOCK_Pos)
#define I2C_IC_ENABLE_TX_CMD_BLOCK                       I2C_IC_ENABLE_TX_CMD_BLOCK_Msk
#define I2C_IC_ENABLE_ABORT_Pos                          1U
#define I2C_IC_ENABLE_ABORT_Msk                          (1U << I2C_IC_ENABLE_ABORT_Pos)
#define I2C_IC_ENABLE_ABORT                              I2C_IC_ENABLE_ABORT_Msk
#define I2C_IC_ENABLE_ENABLE_Pos                         0U
#define I2C_IC_ENABLE_ENABLE_Msk                         (1U << I2C_IC_ENABLE_ENABLE_Pos)
#define I2C_IC_ENABLE_ENABLE                             I2C_IC_ENABLE_ENABLE_Msk

#define I2C_IC_STATUS_SLV_ACTIVITY_Pos                   6U
#define I2C_IC_STATUS_SLV_ACTIVITY_Msk                   (1U << I2C_IC_STATUS_SLV_ACTIVITY_Pos)
#define I2C_IC_STATUS_SLV_ACTIVITY                       I2C_IC_STATUS_SLV_ACTIVITY_Msk
#define I2C_IC_STATUS_MST_ACTIVITY_Pos                   5U
#define I2C_IC_STATUS_MST_ACTIVITY_Msk                   (1U << I2C_IC_STATUS_MST_ACTIVITY_Pos)
#define I2C_IC_STATUS_MST_ACTIVITY                       I2C_IC_STATUS_MST_ACTIVITY_Msk
#define I2C_IC_STATUS_RFF_Pos                            4U
#define I2C_IC_STATUS_RFF_Msk                            (1U << I2C_IC_STATUS_RFF_Pos)
#define I2C_IC_STATUS_RFF                                I2C_IC_STATUS_RFF_Msk
#define I2C_IC_STATUS_RFNE_Pos                           3U
#define I2C_IC_STATUS_RFNE_Msk                           (1U << I2C_IC_STATUS_RFNE_Pos)
#define I2C_IC_STATUS_RFNE                               I2C_IC_STATUS_RFNE_Msk
#define I2C_IC_STATUS_TFE_Pos                            2U
#define I2C_IC_STATUS_TFE_Msk                            (1U << I2C_IC_STATUS_TFE_Pos)
#define I2C_IC_STATUS_TFE                                I2C_IC_STATUS_TFE_Msk
#define I2C_IC_STATUS_TFNF_Pos                           1U
#define I2C_IC_STATUS_TFNF_Msk                           (1U << I2C_IC_STATUS_TFNF_Pos)
#define I2C_IC_STATUS_TFNF                               I2C_IC_STATUS_TFNF_Msk
#define I2C_IC_STATUS_ACTIVITY_Pos                       0U
#define I2C_IC_STATUS_ACTIVITY_Msk                       (1U << I2C_IC_STATUS_ACTIVITY_Pos)
#define I2C_IC_STATUS_ACTIVITY                           I2C_IC_STATUS_ACTIVITY_Msk

#define I2C_IC_TXFLR_TXFLR_Pos                           0U
#define I2C_IC_TXFLR_TXFLR_Msk                           (0x1FU << I2C_IC_TXFLR_TXFLR_Pos)
#define I2C_IC_TXFLR_TXFLR                               I2C_IC_TXFLR_TXFLR_Msk

#define I2C_IC_RXFLR_RXFLR_Pos                           0U
#define I2C_IC_RXFLR_RXFLR_Msk                           (0x1FU << I2C_IC_RXFLR_RXFLR_Pos)
#define I2C_IC_RXFLR_RXFLR                               I2C_IC_RXFLR_RXFLR_Msk

#define I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_Pos               16U
#define I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_Msk               (0xFFU << I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_Pos)
#define I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD                   I2C_IC_SDA_HOLD_IC_SDA_RX_HOLD_Msk
#define I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_Pos               0U
#define I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_Msk               (0xFFU << I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_Pos)
#define I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD                   I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_Msk

#define I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_Pos           23U
#define I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_Msk           (0xFF8U << I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_Pos)
#define I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT               I2C_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_Pos         16U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_Msk         (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT             I2C_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX_Pos        15U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX_Msk        (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX            I2C_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST_Pos       14U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST_Msk       (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST           I2C_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO_Pos   13U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO_Msk   (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO       I2C_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO_Msk
#define I2C_IC_TX_ABRT_SOURCE_ARB_LOST_Pos               12U
#define I2C_IC_TX_ABRT_SOURCE_ARB_LOST_Msk               (1U << I2C_IC_TX_ABRT_SOURCE_ARB_LOST_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ARB_LOST                   I2C_IC_TX_ABRT_SOURCE_ARB_LOST_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_Pos        11U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_Msk        (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS            I2C_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_10B_RD_NORSTRT_Pos    10U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_10B_RD_NORSTRT_Msk    (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_10B_RD_NORSTRT_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_10B_RD_NORSTRT        I2C_IC_TX_ABRT_SOURCE_ABRT_10B_RD_NORSTRT_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_NORSTRT_Pos     9U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_NORSTRT_Msk     (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_NORSTRT_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_NORSTRT         I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_NORSTRT_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_HS_NORSTRT_Pos        8U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_HS_NORSTRT_Msk        (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_HS_NORSTRT_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_HS_NORSTRT            I2C_IC_TX_ABRT_SOURCE_ABRT_HS_NORSTRT_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_ACKDET_Pos      7U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_ACKDET_Msk      (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_ACKDET_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_ACKDET          I2C_IC_TX_ABRT_SOURCE_ABRT_SBYTE_ACKDET_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_HS_ACKDET_Pos         6U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_HS_ACKDET_Msk         (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_HS_ACKDET_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_HS_ACKDET             I2C_IC_TX_ABRT_SOURCE_ABRT_HS_ACKDET_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_READ_Pos        5U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_READ_Msk        (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_READ_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_READ            I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_READ_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_Pos       4U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_Msk       (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK           I2C_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_Pos      3U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_Msk      (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK          I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK_Pos     2U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK_Msk     (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK         I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK_Pos     1U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK_Msk     (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK         I2C_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK_Msk
#define I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_Pos     0U
#define I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_Msk     (1U << I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_Pos)
#define I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK         I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_Msk

#define I2C_IC_DMA_CR_TDMAE_Pos                          1U
#define I2C_IC_DMA_CR_TDMAE_Msk                          (1U << I2C_IC_DMA_CR_TDMAE_Pos)
#define I2C_IC_DMA_CR_TDMAE                              I2C_IC_DMA_CR_TDMAE_Msk
#define I2C_IC_DMA_CR_RDMAE_Pos                          0U
#define I2C_IC_DMA_CR_RDMAE_Msk                          (1U << I2C_IC_DMA_CR_RDMAE_Pos)
#define I2C_IC_DMA_CR_RDMAE                              I2C_IC_DMA_CR_RDMAE_Msk

#define I2C_IC_DMA_TDLR_DMATDL_Pos                       0U
#define I2C_IC_DMA_TDLR_DMATDL_Msk                       (0xFU << I2C_IC_DMA_TDLR_DMATDL_Pos)
#define I2C_IC_DMA_TDLR_DMATDL                           I2C_IC_DMA_TDLR_DMATDL_Msk

#define I2C_IC_DMA_RDLR_DMARDL_Pos                       0U
#define I2C_IC_DMA_RDLR_DMARDL_Msk                       (0xFU << I2C_IC_DMA_RDLR_DMARDL_Pos)
#define I2C_IC_DMA_RDLR_DMARDL                           I2C_IC_DMA_RDLR_DMARDL_Msk

#define I2C_IC_SDA_SETUP_SDA_SETUP_Pos                   0U
#define I2C_IC_SDA_SETUP_SDA_SETUP_Msk                   (0xFFU << I2C_IC_SDA_SETUP_SDA_SETUP_Pos)
#define I2C_IC_SDA_SETUP_SDA_SETUP                       I2C_IC_SDA_SETUP_SDA_SETUP_Msk

#define I2C_IC_ACK_GENERAL_CALL_ACK_GEN_CALL_Pos         0U
#define I2C_IC_ACK_GENERAL_CALL_ACK_GEN_CALL_Msk         (1U << I2C_IC_ACK_GENERAL_CALL_ACK_GEN_CALL_Pos)
#define I2C_IC_ACK_GENERAL_CALL_ACK_GEN_CALL             I2C_IC_ACK_GENERAL_CALL_ACK_GEN_CALL_Msk

#define I2C_IC_ENABLE_STATUS_SLV_RX_DATA_LOST_Pos        2U
#define I2C_IC_ENABLE_STATUS_SLV_RX_DATA_LOST_Msk        (1U << I2C_IC_ENABLE_STATUS_SLV_RX_DATA_LOST_Pos)
#define I2C_IC_ENABLE_STATUS_SLV_RX_DATA_LOST            I2C_IC_ENABLE_STATUS_SLV_RX_DATA_LOST_Msk
#define I2C_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY_Pos 1U
#define I2C_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY_Msk (1U << I2C_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY_Pos)
#define I2C_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY     I2C_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY_Msk
#define I2C_IC_ENABLE_STATUS_IC_EN_Pos                   0U
#define I2C_IC_ENABLE_STATUS_IC_EN_Msk                   (1U << I2C_IC_ENABLE_STATUS_IC_EN_Pos)
#define I2C_IC_ENABLE_STATUS_IC_EN                       I2C_IC_ENABLE_STATUS_IC_EN_Msk

#define I2C_IC_FS_SPKLEN_IC_FS_SPKLEN_Pos                0U
#define I2C_IC_FS_SPKLEN_IC_FS_SPKLEN_Msk                (0xFFU << I2C_IC_FS_SPKLEN_IC_FS_SPKLEN_Pos)
#define I2C_IC_FS_SPKLEN_IC_FS_SPKLEN                    I2C_IC_FS_SPKLEN_IC_FS_SPKLEN_Msk

#define I2C_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_Pos          16U
#define I2C_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_Msk          (0xFFU << I2C_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_Pos)
#define I2C_IC_COMP_PARAM_1_TX_BUFFER_DEPTH              I2C_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_Msk
#define I2C_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_Pos          0U
#define I2C_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_Msk          (0xFFU << I2C_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_Pos)
#define I2C_IC_COMP_PARAM_1_RX_BUFFER_DEPTH              I2C_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_Msk

#define I2C_IC_COMP_PARAM_1_ADD_ENCODED_PARAMS_Pos       7U
#define I2C_IC_COMP_PARAM_1_ADD_ENCODED_PARAMS_Msk       (1U << I2C_IC_COMP_PARAM_1_ADD_ENCODED_PARAMS_Pos)
#define I2C_IC_COMP_PARAM_1_ADD_ENCODED_PARAMS           I2C_IC_COMP_PARAM_1_ADD_ENCODED_PARAMS_Msk
#define I2C_IC_COMP_PARAM_1_HAS_DMA_Pos                  6U
#define I2C_IC_COMP_PARAM_1_HAS_DMA_Msk                  (1U << I2C_IC_COMP_PARAM_1_HAS_DMA_Pos)
#define I2C_IC_COMP_PARAM_1_HAS_DMA                      I2C_IC_COMP_PARAM_1_HAS_DMA_Msk
#define I2C_IC_COMP_PARAM_1_INTR_IO_Pos                  5U
#define I2C_IC_COMP_PARAM_1_INTR_IO_Msk                  (1U << I2C_IC_COMP_PARAM_1_INTR_IO_Pos)
#define I2C_IC_COMP_PARAM_1_INTR_IO                      I2C_IC_COMP_PARAM_1_INTR_IO_Msk
#define I2C_IC_COMP_PARAM_1_HC_COUNT_VALUES_Pos          4U
#define I2C_IC_COMP_PARAM_1_HC_COUNT_VALUES_Msk          (1U << I2C_IC_COMP_PARAM_1_HC_COUNT_VALUES_Pos)
#define I2C_IC_COMP_PARAM_1_HC_COUNT_VALUES              I2C_IC_COMP_PARAM_1_HC_COUNT_VALUES_Msk
#define I2C_IC_COMP_PARAM_1_MAX_SPEED_MODE_Pos           2U
#define I2C_IC_COMP_PARAM_1_MAX_SPEED_MODE_Msk           (3U << I2C_IC_COMP_PARAM_1_MAX_SPEED_MODE_Pos)
#define I2C_IC_COMP_PARAM_1_MAX_SPEED_MODE               I2C_IC_COMP_PARAM_1_MAX_SPEED_MODE_Msk
#define I2C_IC_COMP_PARAM_1_APB_DATA_WIDTH_Pos           0U
#define I2C_IC_COMP_PARAM_1_APB_DATA_WIDTH_Msk           (3U << I2C_IC_COMP_PARAM_1_APB_DATA_WIDTH_Pos)
#define I2C_IC_COMP_PARAM_1_APB_DATA_WIDTH               I2C_IC_COMP_PARAM_1_APB_DATA_WIDTH_Msk
/** @} */

/**
 * @name    PWM bits definitions
 * @{
 */
#define PWM_CSR_PH_ADV_Pos                               7U
#define PWM_CSR_PH_ADV_Msk                               (1U << PWM_CSR_PH_ADV_Pos)
#define PWM_CSR_PH_ADV                                   PWM_CSR_PH_ADV_Msk
#define PWM_CSR_PH_RET_Pos                               6U
#define PWM_CSR_PH_RET_Msk                               (1U << PWM_CSR_PH_RET_Pos)
#define PWM_CSR_PH_RET                                   PWM_CSR_PH_RET_Msk
#define PWM_CSR_DIVMODE_Pos                              4U
#define PWM_CSR_DIVMODE_Msk                              (0x3U << PWM_CSR_DIVMODE_Pos)
#define PWM_CSR_DIVMODE(n)                               ((n) << PWM_CSR_DIVMODE_Pos)
#define PWM_CSR_B_INV_Pos                                3U
#define PWM_CSR_B_INV_Msk                                (1U << PWM_CSR_B_INV_Pos)
#define PWM_CSR_B_INV                                    PWM_CSR_B_INV_Msk
#define PWM_CSR_A_INV_Pos                                2U
#define PWM_CSR_A_INV_Msk                                (1U << PWM_CSR_A_INV_Pos)
#define PWM_CSR_A_INV                                    PWM_CSR_A_INV_Msk
#define PWM_CSR_PH_CORRECT_Pos                           1U
#define PWM_CSR_PH_CORRECT_Msk                           (1U << PWM_CSR_PH_CORRECT_Pos)
#define PWM_CSR_PH_CORRECT                               PWM_CSR_PH_CORRECT_Msk
#define PWM_CSR_EN_Pos                                   0U
#define PWM_CSR_EN_Msk                                   (1U << PWM_CSR_EN_Pos)
#define PWM_CSR_EN                                       PWM_CSR_EN_Msk
#define PWM_CSR_PH_CORRECT_PHASE_CORRECT_MODULATION      (1U << PWM_CSR_PH_CORRECT_Pos)
#define PWM_CSR_PH_CORRECT_TRAILING_EDGE                 (0U << PWM_CSR_PH_CORRECT_Pos)
#define PWM_CSR_DIVMODE_FREE                             PWM_CSR_DIVMODE(0U)
#define PWM_CSR_DIVMODE_GATED                            PWM_CSR_DIVMODE(1U)
#define PWM_CSR_DIVMODE_RISING_EDGE                      PWM_CSR_DIVMODE(2U)
#define PWM_CSR_DIVMODE_FALING_EDGE                      PWM_CSR_DIVMODE(3U)

#define PWM_DIV_INT_Pos                                  4U
#define PWM_DIV_INT_Msk                                  (0xFFU << PWM_DIV_INT_Pos)
#define PWM_DIV_INT                                      PWM_DIV_INT_Msk
#define PWM_DIV_FRAC_Pos                                 0U
#define PWM_DIV_FRAC_Msk                                 (0xFU << PWM_DIV_FRAC_Pos)
#define PWM_DIV_FRAC                                     PWM_DIV_FRAC_Msk

#define PWM_CTR_COUNTER_Pos                              0U
#define PWM_CTR_COUNTER_Msk                              (0xFFFFU << PWM_CTR_COUNTER_Pos)
#define PWM_CTR_COUNTER                                  PWM_CTR_COUNTER_Msk

#define PWM_CC_B_Pos                                     16U
#define PWM_CC_B_Msk                                     (0xFFFFU << PWM_CC_B_Pos)
#define PWM_CC_B                                         PWM_CC_B_Msk
#define PWM_CC_A_Pos                                     0U
#define PWM_CC_A_Msk                                     (0xFFFFU << PWM_CC_A_Pos)
#define PWM_CC_A                                         PWM_CC_A_Msk

#define PWM_TOP_VALUE_Pos                                0U
#define PWM_TOP_VALUE_Msk                                (0xFFFFU << PWM_TOP_VALUE_Pos)
#define PWM_TOP_VALUE                                    PWM_TOP_VALUE_Msk

#define PWM_EN_CH7_Pos                                   7U
#define PWM_EN_CH7_Msk                                   (1U << PWM_EN_CH7_Pos)
#define PWM_EN_CH7                                       PWM_EN_CH7_Msk
#define PWM_EN_CH6_Pos                                   6U
#define PWM_EN_CH6_Msk                                   (1U << PWM_EN_CH6_Pos)
#define PWM_EN_CH6                                       PWM_EN_CH6_Msk
#define PWM_EN_CH5_Pos                                   5U
#define PWM_EN_CH5_Msk                                   (1U << PWM_EN_CH5_Pos)
#define PWM_EN_CH5                                       PWM_EN_CH5_Msk
#define PWM_EN_CH4_Pos                                   4U
#define PWM_EN_CH4_Msk                                   (1U << PWM_EN_CH4_Pos)
#define PWM_EN_CH4                                       PWM_EN_CH4_Msk
#define PWM_EN_CH3_Pos                                   3U
#define PWM_EN_CH3_Msk                                   (1U << PWM_EN_CH3_Pos)
#define PWM_EN_CH3                                       PWM_EN_CH3_Msk
#define PWM_EN_CH2_Pos                                   2U
#define PWM_EN_CH2_Msk                                   (1U << PWM_EN_CH2_Pos)
#define PWM_EN_CH2                                       PWM_EN_CH2_Msk
#define PWM_EN_CH1_Pos                                   1U
#define PWM_EN_CH1_Msk                                   (1U << PWM_EN_CH1_Pos)
#define PWM_EN_CH1                                       PWM_EN_CH1_Msk
#define PWM_EN_CH0_Pos                                   0U
#define PWM_EN_CH0_Msk                                   (1U << PWM_EN_CH0_Pos)
#define PWM_EN_CH0                                       PWM_EN_CH0_Msk

#define PWM_INTR_CH7_Pos                                 7U
#define PWM_INTR_CH7_Msk                                 (1U << PWM_INTR_CH7_Pos)
#define PWM_INTR_CH7                                     PWM_INTR_CH7_Msk
#define PWM_INTR_CH6_Pos                                 6U
#define PWM_INTR_CH6_Msk                                 (1U << PWM_INTR_CH6_Pos)
#define PWM_INTR_CH6                                     PWM_INTR_CH6_Msk
#define PWM_INTR_CH5_Pos                                 5U
#define PWM_INTR_CH5_Msk                                 (1U << PWM_INTR_CH5_Pos)
#define PWM_INTR_CH5                                     PWM_INTR_CH5_Msk
#define PWM_INTR_CH4_Pos                                 4U
#define PWM_INTR_CH4_Msk                                 (1U << PWM_INTR_CH4_Pos)
#define PWM_INTR_CH4                                     PWM_INTR_CH4_Msk
#define PWM_INTR_CH3_Pos                                 3U
#define PWM_INTR_CH3_Msk                                 (1U << PWM_INTR_CH3_Pos)
#define PWM_INTR_CH3                                     PWM_INTR_CH3_Msk
#define PWM_INTR_CH2_Pos                                 2U
#define PWM_INTR_CH2_Msk                                 (1U << PWM_INTR_CH2_Pos)
#define PWM_INTR_CH2                                     PWM_INTR_CH2_Msk
#define PWM_INTR_CH1_Pos                                 1U
#define PWM_INTR_CH1_Msk                                 (1U << PWM_INTR_CH1_Pos)
#define PWM_INTR_CH1                                     PWM_INTR_CH1_Msk
#define PWM_INTR_CH0_Pos                                 0U
#define PWM_INTR_CH0_Msk                                 (1U << PWM_INTR_CH0_Pos)
#define PWM_INTR_CH0                                     PWM_INTR_CH0_Msk
#define PWM_INTR_CH(n)                                   (1U << n)

#define PWM_INTE_CH7_Pos                                 7U
#define PWM_INTE_CH7_Msk                                 (1U << PWM_INTE_CH7_Pos)
#define PWM_INTE_CH7                                     PWM_INTE_CH7_Msk
#define PWM_INTE_CH6_Pos                                 6U
#define PWM_INTE_CH6_Msk                                 (1U << PWM_INTE_CH6_Pos)
#define PWM_INTE_CH6                                     PWM_INTE_CH6_Msk
#define PWM_INTE_CH5_Pos                                 5U
#define PWM_INTE_CH5_Msk                                 (1U << PWM_INTE_CH5_Pos)
#define PWM_INTE_CH5                                     PWM_INTE_CH5_Msk
#define PWM_INTE_CH4_Pos                                 4U
#define PWM_INTE_CH4_Msk                                 (1U << PWM_INTE_CH4_Pos)
#define PWM_INTE_CH4                                     PWM_INTE_CH4_Msk
#define PWM_INTE_CH3_Pos                                 3U
#define PWM_INTE_CH3_Msk                                 (1U << PWM_INTE_CH3_Pos)
#define PWM_INTE_CH3                                     PWM_INTE_CH3_Msk
#define PWM_INTE_CH2_Pos                                 2U
#define PWM_INTE_CH2_Msk                                 (1U << PWM_INTE_CH2_Pos)
#define PWM_INTE_CH2                                     PWM_INTE_CH2_Msk
#define PWM_INTE_CH1_Pos                                 1U
#define PWM_INTE_CH1_Msk                                 (1U << PWM_INTE_CH1_Pos)
#define PWM_INTE_CH1                                     PWM_INTE_CH1_Msk
#define PWM_INTE_CH0_Pos                                 0U
#define PWM_INTE_CH0_Msk                                 (1U << PWM_INTE_CH0_Pos)
#define PWM_INTE_CH0                                     PWM_INTE_CH0_Msk
#define PWM_INTE_CH(n)                                   (1U << n)

#define PWM_INTF_CH7_Pos                                 7U
#define PWM_INTF_CH7_Msk                                 (1U << PWM_INTF_CH7_Pos)
#define PWM_INTF_CH7                                     PWM_INTF_CH7_Msk
#define PWM_INTF_CH6_Pos                                 6U
#define PWM_INTF_CH6_Msk                                 (1U << PWM_INTF_CH6_Pos)
#define PWM_INTF_CH6                                     PWM_INTF_CH6_Msk
#define PWM_INTF_CH5_Pos                                 5U
#define PWM_INTF_CH5_Msk                                 (1U << PWM_INTF_CH5_Pos)
#define PWM_INTF_CH5                                     PWM_INTF_CH5_Msk
#define PWM_INTF_CH4_Pos                                 4U
#define PWM_INTF_CH4_Msk                                 (1U << PWM_INTF_CH4_Pos)
#define PWM_INTF_CH4                                     PWM_INTF_CH4_Msk
#define PWM_INTF_CH3_Pos                                 3U
#define PWM_INTF_CH3_Msk                                 (1U << PWM_INTF_CH3_Pos)
#define PWM_INTF_CH3                                     PWM_INTF_CH3_Msk
#define PWM_INTF_CH2_Pos                                 2U
#define PWM_INTF_CH2_Msk                                 (1U << PWM_INTF_CH2_Pos)
#define PWM_INTF_CH2                                     PWM_INTF_CH2_Msk
#define PWM_INTF_CH1_Pos                                 1U
#define PWM_INTF_CH1_Msk                                 (1U << PWM_INTF_CH1_Pos)
#define PWM_INTF_CH1                                     PWM_INTF_CH1_Msk
#define PWM_INTF_CH0_Pos                                 0U
#define PWM_INTF_CH0_Msk                                 (1U << PWM_INTF_CH0_Pos)
#define PWM_INTF_CH0                                     PWM_INTF_CH0_Msk
#define PWM_INTF_CH(n)                                   (1U << n)

#define PWM_INTS_CH7_Pos                                 7U
#define PWM_INTS_CH7_Msk                                 (1U << PWM_INTS_CH7_Pos)
#define PWM_INTS_CH7                                     PWM_INTS_CH7_Msk
#define PWM_INTS_CH6_Pos                                 6U
#define PWM_INTS_CH6_Msk                                 (1U << PWM_INTS_CH6_Pos)
#define PWM_INTS_CH6                                     PWM_INTS_CH6_Msk
#define PWM_INTS_CH5_Pos                                 5U
#define PWM_INTS_CH5_Msk                                 (1U << PWM_INTS_CH5_Pos)
#define PWM_INTS_CH5                                     PWM_INTS_CH5_Msk
#define PWM_INTS_CH4_Pos                                 4U
#define PWM_INTS_CH4_Msk                                 (1U << PWM_INTS_CH4_Pos)
#define PWM_INTS_CH4                                     PWM_INTS_CH4_Msk
#define PWM_INTS_CH3_Pos                                 3U
#define PWM_INTS_CH3_Msk                                 (1U << PWM_INTS_CH3_Pos)
#define PWM_INTS_CH3                                     PWM_INTS_CH3_Msk
#define PWM_INTS_CH2_Pos                                 2U
#define PWM_INTS_CH2_Msk                                 (1U << PWM_INTS_CH2_Pos)
#define PWM_INTS_CH2                                     PWM_INTS_CH2_Msk
#define PWM_INTS_CH1_Pos                                 1U
#define PWM_INTS_CH1_Msk                                 (1U << PWM_INTS_CH1_Pos)
#define PWM_INTS_CH1                                     PWM_INTS_CH1_Msk
#define PWM_INTS_CH0_Pos                                 0U
#define PWM_INTS_CH0_Msk                                 (1U << PWM_INTS_CH0_Pos)
#define PWM_INTS_CH0                                     PWM_INTS_CH0_Msk
#define PWM_INTS_CH(n)                                   (1U << n)
/** @} */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RP2040_H */

/** @} */
