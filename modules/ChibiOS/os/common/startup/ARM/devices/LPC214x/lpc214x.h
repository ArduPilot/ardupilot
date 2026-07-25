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
 * @file    lpc214x.h
 * @brief   LPC214x register definitions.
 */

#ifndef LPC214X_H
#define LPC214X_H

typedef volatile uint8_t IOREG8;
typedef volatile uint16_t IOREG16;
typedef volatile uint32_t IOREG32;

/*
 * System.
 */
#define MEMMAP                  (*((IOREG32 *)0xE01FC040))
#define PCON                    (*((IOREG32 *)0xE01FC0C0))
#define PCONP                   (*((IOREG32 *)0xE01FC0C4))
#define VPBDIV                  (*((IOREG32 *)0xE01FC100))
#define EXTINT                  (*((IOREG32 *)0xE01FC140))
#define INTWAKE                 (*((IOREG32 *)0xE01FC144))
#define EXTMODE                 (*((IOREG32 *)0xE01FC148))
#define EXTPOLAR                (*((IOREG32 *)0xE01FC14C))
#define RSID                    (*((IOREG32 *)0xE01FC180))
#define CSPR                    (*((IOREG32 *)0xE01FC184))
#define SCS                     (*((IOREG32 *)0xE01FC1A0))

#define VPD_D4                  0
#define VPD_D1                  1
#define VPD_D2                  2
#define VPD_RESERVED            3

#define PCTIM0                  (1 << 1)
#define PCTIM1                  (1 << 2)
#define PCUART0                 (1 << 3)
#define PCUART1                 (1 << 4)
#define PCPWM0                  (1 << 5)
#define PCI2C0                  (1 << 7)
#define PCSPI0                  (1 << 8)
#define PCRTC                   (1 << 9)
#define PCSPI1                  (1 << 10)
#define PCAD0                   (1 << 12)
#define PCI2C1                  (1 << 19)
#define PCAD1                   (1 << 20)
#define PCUSB                   (1 << 31)
#define PCALL                   (PCTIM0 | PCTIM1 | PCUART0 | PCUART1 | \
                                 PCPWM0 | PCI2C0 | PCSPI0 | PCRTC | PCSPI1 | \
                                 PCAD0 | PCI2C1 | PCAD1 | PCUSB)

#define EINT0                   1
#define EINT1                   2
#define EINT2                   4
#define EINT3                   8

#define EXTWAKE0                1
#define EXTWAKE1                2
#define EXTWAKE2                4
#define EXTWAKE3                8
#define USBWAKE                 0x20
#define BODWAKE                 0x4000
#define RTCWAKE                 0x8000

#define EXTMODE0                1
#define EXTMODE1                2
#define EXTMODE2                4
#define EXTMODE3                8

#define EXTPOLAR0               1
#define EXTPOLAR1               2
#define EXTPOLAR2               4
#define EXTPOLAR3               8

typedef struct {
  IOREG32       PLL_CON;
  IOREG32       PLL_CFG;
  IOREG32       PLL_STAT;
  IOREG32       PLL_FEED;
} PLL;

#define PLL0Base                ((PLL *)0xE01FC080)
#define PLL1Base                ((PLL *)0xE01FC0A0)
#define PLL0CON                 (PLL0Base->PLL_CON)
#define PLL0CFG                 (PLL0Base->PLL_CFG)
#define PLL0STAT                (PLL0Base->PLL_STAT)
#define PLL0FEED                (PLL0Base->PLL_FEED)
#define PLL1CON                 (PLL1Base->PLL_CON)
#define PLL1CFG                 (PLL1Base->PLL_CFG)
#define PLL1STAT                (PLL1Base->PLL_STAT)
#define PLL1FEED                (PLL1Base->PLL_FEED)

/*
 * Pins.
 */
typedef struct {
  IOREG32       PS_SEL0;
  IOREG32       PS_SEL1;
  IOREG32       _dummy[3];
  IOREG32       PS_SEL2;
} PS;

#define PSBase                  ((PS *)0xE002C000)
#define PINSEL0                 (PSBase->PS_SEL0)
#define PINSEL1                 (PSBase->PS_SEL1)
#define PINSEL2                 (PSBase->PS_SEL2)

/*
 * VIC
 */
#define SOURCE_WDT              0
#define SOURCE_ARMCore0         2
#define SOURCE_ARMCore1         3
#define SOURCE_Timer0           4
#define SOURCE_Timer1           5
#define SOURCE_UART0            6
#define SOURCE_UART1            7
#define SOURCE_PWM0             8
#define SOURCE_I2C0             9
#define SOURCE_SPI0             10
#define SOURCE_SPI1             11
#define SOURCE_PLL              12
#define SOURCE_RTC              13
#define SOURCE_EINT0            14
#define SOURCE_EINT1            15
#define SOURCE_EINT2            16
#define SOURCE_EINT3            17
#define SOURCE_ADC0             18
#define SOURCE_I2C1             19
#define SOURCE_BOD              20
#define SOURCE_ADC1             21
#define SOURCE_USB              22

#define INTMASK(n)              (1 << (n))
#define ALLINTMASK (INTMASK(SOURCE_WDT) | INTMASK(SOURCE_ARMCore0) | \
                    INTMASK(SOURCE_ARMCore1) | INTMASK(SOURCE_Timer0) | \
                    INTMASK(SOURCE_Timer1) | INTMASK(SOURCE_UART0) | \
                    INTMASK(SOURCE_UART1) | INTMASK(SOURCE_PWM0) | \
                    INTMASK(SOURCE_I2C0) | INTMASK(SOURCE_SPI0) | \
                    INTMASK(SOURCE_SPI1) | INTMASK(SOURCE_PLL) | \
                    INTMASK(SOURCE_RTC) | INTMASK(SOURCE_EINT0) | \
                    INTMASK(SOURCE_EINT1) | INTMASK(SOURCE_EINT2) | \
                    INTMASK(SOURCE_EINT3) | INTMASK(SOURCE_ADC0) | \
                    INTMASK(SOURCE_I2C1) | INTMASK(SOURCE_BOD) | \
                    INTMASK(SOURCE_ADC1) | INTMASK(SOURCE_USB))

typedef struct {
  IOREG32       VIC_IRQStatus;
  IOREG32       VIC_FIQStatus;
  IOREG32       VIC_RawIntr;
  IOREG32       VIC_IntSelect;
  IOREG32       VIC_IntEnable;
  IOREG32       VIC_IntEnClear;
  IOREG32       VIC_SoftInt;
  IOREG32       VIC_SoftIntClear;
  IOREG32       VIC_Protection;
  IOREG32       unused1[3];
  IOREG32       VIC_VectAddr;
  IOREG32       VIC_DefVectAddr;
  IOREG32       unused2[50];
  IOREG32       VIC_VectAddrs[16];
  IOREG32       unused3[48];
  IOREG32       VIC_VectCntls[16];
} VIC;

#define VICBase                 ((VIC *)0xFFFFF000)
#define VICVectorsBase          ((IOREG32 *)0xFFFFF100)
#define VICControlsBase         ((IOREG32 *)0xFFFFF200)

#define VICIRQStatus            (VICBase->VIC_IRQStatus)
#define VICFIQStatus            (VICBase->VIC_FIQStatus)
#define VICRawIntr              (VICBase->VIC_RawIntr)
#define VICIntSelect            (VICBase->VIC_IntSelect)
#define VICIntEnable            (VICBase->VIC_IntEnable)
#define VICIntEnClear           (VICBase->VIC_IntEnClear)
#define VICSoftInt              (VICBase->VIC_SoftInt)
#define VICSoftIntClear         (VICBase->VIC_SoftIntClear)
#define VICProtection           (VICBase->VIC_Protection)
#define VICVectAddr             (VICBase->VIC_VectAddr)
#define VICDefVectAddr          (VICBase->VIC_DefVectAddr)

#define VICVectAddrs(n)         (VICBase->VIC_VectAddrs[n])
#define VICVectCntls(n)         (VICBase->VIC_VectCntls[n])

/*
 * MAM.
 */
typedef struct {
  IOREG32       MAM_Control;
  IOREG32       MAM_Timing;
} MAM;

#define MAMBase                 ((MAM *)0xE01FC000)
#define MAMCR                   (MAMBase->MAM_Control)
#define MAMTIM                  (MAMBase->MAM_Timing)

/*
 * GPIO - FIO.
 */
typedef struct {
  IOREG32       IO_PIN;
  IOREG32       IO_SET;
  IOREG32       IO_DIR;
  IOREG32       IO_CLR;
} GPIO;

#define GPIO0Base                ((GPIO *)0xE0028000)
#define IO0PIN                   (GPIO0Base->IO_PIN)
#define IO0SET                   (GPIO0Base->IO_SET)
#define IO0DIR                   (GPIO0Base->IO_DIR)
#define IO0CLR                   (GPIO0Base->IO_CLR)

#define GPIO1Base                ((GPIO *)0xE0028010)
#define IO1PIN                   (GPIO1Base->IO_PIN)
#define IO1SET                   (GPIO1Base->IO_SET)
#define IO1DIR                   (GPIO1Base->IO_DIR)
#define IO1CLR                   (GPIO1Base->IO_CLR)

typedef struct {
  IOREG32       FIO_DIR;
  IOREG32       unused1;
  IOREG32       unused2;
  IOREG32       unused3;
  IOREG32       FIO_MASK;
  IOREG32       FIO_PIN;
  IOREG32       FIO_SET;
  IOREG32       FIO_CLR;
} FIO;

#define FIO0Base                ((FIO *)0x3FFFC000)
#define FIO0DIR                 (FIO0Base->FIO_DIR)
#define FIO0MASK                (FIO0Base->FIO_MASK)
#define FIO0PIN                 (FIO0Base->FIO_PIN)
#define FIO0SET                 (FIO0Base->FIO_SET)
#define FIO0CLR                 (FIO0Base->FIO_CLR)

#define FIO1Base                ((FIO *)0x3FFFC020)
#define FIO1DIR                 (FIO1Base->FIO_DIR)
#define FIO1MASK                (FIO1Base->FIO_MASK)
#define FIO1PIN                 (FIO1Base->FIO_PIN)
#define FIO1SET                 (FIO1Base->FIO_SET)
#define FIO1CLR                 (FIO1Base->FIO_CLR)

/*
 * UART.
 */
typedef struct {
  union {
    IOREG32     UART_RBR;
    IOREG32     UART_THR;
    IOREG32     UART_DLL;
  };
  union {
    IOREG32     UART_IER;
    IOREG32     UART_DLM;
  };
  union {
    IOREG32     UART_IIR;
    IOREG32     UART_FCR;
  };
  IOREG32       UART_LCR;
  IOREG32       UART_MCR;
  IOREG32       UART_LSR;
  IOREG32       unused18;
  IOREG32       UART_SCR;
  IOREG32       UART_ACR;
  IOREG32       unused24;
  IOREG32       UART_FDR;
  IOREG32       unused2C;
  IOREG32       UART_TER;
} UART;

#define U0Base                  ((UART *)0xE000C000)
#define U0RBR                   (U0Base->UART_RBR)
#define U0THR                   (U0Base->UART_THR)
#define U0DLL                   (U0Base->UART_DLL)
#define U0IER                   (U0Base->UART_IER)
#define U0DLM                   (U0Base->UART_DLM)
#define U0IIR                   (U0Base->UART_IIR)
#define U0FCR                   (U0Base->UART_FCR)
#define U0LCR                   (U0Base->UART_LCR)
#define U0LSR                   (U0Base->UART_LSR)
#define U0SCR                   (U0Base->UART_SCR)
#define U0ACR                   (U0Base->UART_ACR)
#define U0FDR                   (U0Base->UART_FDR)
#define U0TER                   (U0Base->UART_TER)

#define U1Base                  ((UART *)0xE0010000)
#define U1RBR                   (U1Base->UART_RBR)
#define U1THR                   (U1Base->UART_THR)
#define U1DLL                   (U1Base->UART_DLL)
#define U1IER                   (U1Base->UART_IER)
#define U1DLM                   (U1Base->UART_DLM)
#define U1IIR                   (U1Base->UART_IIR)
#define U1FCR                   (U1Base->UART_FCR)
#define U1MCR                   (U1Base->UART_MCR)
#define U1LCR                   (U1Base->UART_LCR)
#define U1LSR                   (U1Base->UART_LSR)
#define U1SCR                   (U1Base->UART_SCR)
#define U1ACR                   (U1Base->UART_ACR)
#define U1FDR                   (U1Base->UART_FDR)
#define U1TER                   (U1Base->UART_TER)

#define IIR_SRC_MASK    0x0F
#define IIR_SRC_NONE    0x01
#define IIR_SRC_TX      0x02
#define IIR_SRC_RX      0x04
#define IIR_SRC_ERROR   0x06
#define IIR_SRC_TIMEOUT 0x0C

#define IER_RBR         1
#define IER_THRE        2
#define IER_STATUS      4

#define IIR_INT_PENDING 1

#define LCR_WL5         0
#define LCR_WL6         1
#define LCR_WL7         2
#define LCR_WL8         3
#define LCR_STOP1       0
#define LCR_STOP2       4
#define LCR_NOPARITY    0
#define LCR_PARITYODD   0x08
#define LCR_PARITYEVEN  0x18
#define LCR_PARITYONE   0x28
#define LCR_PARITYZERO  0x38
#define LCR_BREAK_ON    0x40
#define LCR_DLAB        0x80

#define FCR_ENABLE      1
#define FCR_RXRESET     2
#define FCR_TXRESET     4
#define FCR_TRIGGER0    0
#define FCR_TRIGGER1    0x40
#define FCR_TRIGGER2    0x80
#define FCR_TRIGGER3    0xC0

#define LSR_RBR_FULL    1
#define LSR_OVERRUN     2
#define LSR_PARITY      4
#define LSR_FRAMING     8
#define LSR_BREAK       0x10
#define LSR_THRE        0x20
#define LSR_TEMT        0x40
#define LSR_RXFE        0x80

#define TER_ENABLE      0x80

/*
 * SSP.
 */
typedef struct {
  IOREG32       SSP_CR0;
  IOREG32       SSP_CR1;
  IOREG32       SSP_DR;
  IOREG32       SSP_SR;
  IOREG32       SSP_CPSR;
  IOREG32       SSP_IMSC;
  IOREG32       SSP_RIS;
  IOREG32       SSP_MIS;
  IOREG32       SSP_ICR;
} SSP;

#define SSPBase                 ((SSP *)0xE0068000)
#define SSPCR0                  (SSPBase->SSP_CR0)
#define SSPCR1                  (SSPBase->SSP_CR1)
#define SSPDR                   (SSPBase->SSP_DR)
#define SSPSR                   (SSPBase->SSP_SR)
#define SSPCPSR                 (SSPBase->SSP_CPSR)
#define SSPIMSC                 (SSPBase->SSP_IMSC)
#define SSPRIS                  (SSPBase->SSP_RIS)
#define SSPMIS                  (SSPBase->SSP_MIS)
#define SSPICR                  (SSPBase->SSP_ICR)

#define CR0_DSSMASK             0x0F
#define CR0_DSS4BIT             3
#define CR0_DSS5BIT             4
#define CR0_DSS6BIT             5
#define CR0_DSS7BIT             6
#define CR0_DSS8BIT             7
#define CR0_DSS9BIT             8
#define CR0_DSS10BIT            9
#define CR0_DSS11BIT            0xA
#define CR0_DSS12BIT            0xB
#define CR0_DSS13BIT            0xC
#define CR0_DSS14BIT            0xD
#define CR0_DSS15BIT            0xE
#define CR0_DSS16BIT            0xF
#define CR0_FRFSPI              0
#define CR0_FRFSSI              0x10
#define CR0_FRFMW               0x20
#define CR0_CPOL                0x40
#define CR0_CPHA                0x80
#define CR0_CLOCKRATE(n)        ((n) << 8)

#define CR1_LBM                 1
#define CR1_SSE                 2
#define CR1_MS                  4
#define CR1_SOD                 8

#define SR_TFE                  1
#define SR_TNF                  2
#define SR_RNE                  4
#define SR_RFF                  8
#define SR_BSY                  0x10

#define IMSC_ROR                1
#define IMSC_RT                 2
#define IMSC_RX                 4
#define IMSC_TX                 8

#define RIS_ROR                 1
#define RIS_RT                  2
#define RIS_RX                  4
#define RIS_TX                  8

#define MIS_ROR                 1
#define MIS_RT                  2
#define MIS_RX                  4
#define MIS_TX                  8

#define ICR_ROR                 1
#define ICR_RT                  2

/*
 * Timers/Counters.
 */
typedef struct {
  IOREG32       TC_IR;
  IOREG32       TC_TCR;
  IOREG32       TC_TC;
  IOREG32       TC_PR;
  IOREG32       TC_PC;
  IOREG32       TC_MCR;
  IOREG32       TC_MR0;
  IOREG32       TC_MR1;
  IOREG32       TC_MR2;
  IOREG32       TC_MR3;
  IOREG32       TC_CCR;
  IOREG32       TC_CR0;
  IOREG32       TC_CR1;
  IOREG32       TC_CR2;
  IOREG32       TC_CR3;
  IOREG32       TC_EMR;
  IOREG32       TC_CTCR;
} TC;

#define T0Base                  ((TC *)0xE0004000)
#define T0IR                    (T0Base->TC_IR)
#define T0TCR                   (T0Base->TC_TCR)
#define T0TC                    (T0Base->TC_TC)
#define T0PR                    (T0Base->TC_PR)
#define T0PC                    (T0Base->TC_PC)
#define T0MCR                   (T0Base->TC_MCR)
#define T0MR0                   (T0Base->TC_MR0)
#define T0MR1                   (T0Base->TC_MR1)
#define T0MR2                   (T0Base->TC_MR2)
#define T0MR3                   (T0Base->TC_MR3)
#define T0CCR                   (T0Base->TC_CCR)
#define T0CR0                   (T0Base->TC_CR0)
#define T0CR1                   (T0Base->TC_CR1)
#define T0CR2                   (T0Base->TC_CR2)
#define T0CR3                   (T0Base->TC_CR3)
#define T0EMR                   (T0Base->TC_EMR)
#define T0CTCR                  (T0Base->TC_CTCR)

#define T1Base                  ((TC *)0xE0008000)
#define T1IR                    (T1Base->TC_IR)
#define T1TCR                   (T1Base->TC_TCR)
#define T1TC                    (T1Base->TC_TC)
#define T1PR                    (T1Base->TC_PR)
#define T1PC                    (T1Base->TC_PC)
#define T1MCR                   (T1Base->TC_MCR)
#define T1MR0                   (T1Base->TC_MR0)
#define T1MR1                   (T1Base->TC_MR1)
#define T1MR2                   (T1Base->TC_MR2)
#define T1MR3                   (T1Base->TC_MR3)
#define T1CCR                   (T1Base->TC_CCR)
#define T1CR0                   (T1Base->TC_CR0)
#define T1CR1                   (T1Base->TC_CR1)
#define T1CR2                   (T1Base->TC_CR2)
#define T1CR3                   (T1Base->TC_CR3)
#define T1EMR                   (T1Base->TC_EMR)
#define T1CTCR                  (T1Base->TC_CTCR)

/*
 * Watchdog.
 */
typedef struct {
  IOREG32       WD_MOD;
  IOREG32       WD_TC;
  IOREG32       WD_FEED;
  IOREG32       WD_TV;
} WD;

#define WDBase                  ((WD *)0xE0000000)
#define WDMOD                   (WDBase->WD_MOD)
#define WDTC                    (WDBase->WD_TC)
#define WDFEED                  (WDBase->WD_FEED)
#define WDTV                    (WDBase->WD_TV)

/*
 * DAC.
 */
#define DACR                    (*((IOREG32 *)0xE006C000))

#endif /* LPC214X_H */

