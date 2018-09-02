/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Bit definitions were copied from NuttX STM32 CAN driver.
 *
 * With modifications for Ardupilot CAN driver
 * Copyright (C) 2017 Eugene Shamaev
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <stdint.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <chip/stm32_tim.h>
#include <syslog.h>
#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/mm.h>
#include <pthread.h>

#ifndef UAVCAN_CPP_VERSION
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION < UAVCAN_CPP11
// #undef'ed at the end of this file
# define constexpr const
#endif

namespace VRBRAIN {
namespace bxcan {

# define CAN_IRQ_ATTACH(irq, handler)                          \
   do {                                                      \
        const int res = irq_attach(irq, handler);          \
        (void)res;                                         \
        assert(res >= 0);                                  \
        up_enable_irq(irq);                                \
    } while(0)

struct TxMailboxType {
    volatile uint32_t TIR;
    volatile uint32_t TDTR;
    volatile uint32_t TDLR;
    volatile uint32_t TDHR;
};

struct RxMailboxType {
    volatile uint32_t RIR;
    volatile uint32_t RDTR;
    volatile uint32_t RDLR;
    volatile uint32_t RDHR;
};

struct FilterRegisterType {
    volatile uint32_t FR1;
    volatile uint32_t FR2;
};

struct CanType {
    volatile uint32_t MCR; /*!< CAN master control register,         Address offset: 0x00          */
    volatile uint32_t MSR; /*!< CAN master status register,          Address offset: 0x04          */
    volatile uint32_t TSR; /*!< CAN transmit status register,        Address offset: 0x08          */
    volatile uint32_t RF0R; /*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
    volatile uint32_t RF1R; /*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
    volatile uint32_t IER; /*!< CAN interrupt enable register,       Address offset: 0x14          */
    volatile uint32_t ESR; /*!< CAN error status register,           Address offset: 0x18          */
    volatile uint32_t BTR; /*!< CAN bit timing register,             Address offset: 0x1C          */
    uint32_t RESERVED0[88]; /*!< Reserved, 0x020 - 0x17F                                            */
    TxMailboxType TxMailbox[3]; /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
    RxMailboxType RxMailbox[2]; /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
    uint32_t RESERVED1[12]; /*!< Reserved, 0x1D0 - 0x1FF                                            */
    volatile uint32_t FMR; /*!< CAN filter master register,          Address offset: 0x200         */
    volatile uint32_t FM1R; /*!< CAN filter mode register,            Address offset: 0x204         */
    uint32_t RESERVED2; /*!< Reserved, 0x208                                                    */
    volatile uint32_t FS1R; /*!< CAN filter scale register,           Address offset: 0x20C         */
    uint32_t RESERVED3; /*!< Reserved, 0x210                                                    */
    volatile uint32_t FFA1R; /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
    uint32_t RESERVED4; /*!< Reserved, 0x218                                                    */
    volatile uint32_t FA1R; /*!< CAN filter activation register,      Address offset: 0x21C         */
    uint32_t RESERVED5[8]; /*!< Reserved, 0x220-0x23F                                              */
    FilterRegisterType FilterRegister[28]; /*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
};

/**
 * CANx register sets
 */
CanType* const Can[2] = { reinterpret_cast<CanType*>(STM32_CAN1_BASE), reinterpret_cast<CanType*>(STM32_CAN2_BASE) };

/* CAN master control register */

constexpr unsigned long MCR_INRQ = (1U << 0); /* Bit 0: Initialization Request */
constexpr unsigned long MCR_SLEEP = (1U << 1); /* Bit 1: Sleep Mode Request */
constexpr unsigned long MCR_TXFP = (1U << 2); /* Bit 2: Transmit FIFO Priority */
constexpr unsigned long MCR_RFLM = (1U << 3); /* Bit 3: Receive FIFO Locked Mode */
constexpr unsigned long MCR_NART = (1U << 4); /* Bit 4: No Automatic Retransmission */
constexpr unsigned long MCR_AWUM = (1U << 5); /* Bit 5: Automatic Wakeup Mode */
constexpr unsigned long MCR_ABOM = (1U << 6); /* Bit 6: Automatic Bus-Off Management */
constexpr unsigned long MCR_TTCM = (1U << 7); /* Bit 7: Time Triggered Communication Mode Enable */
constexpr unsigned long MCR_RESET = (1U << 15);/* Bit 15: bxCAN software master reset */
constexpr unsigned long MCR_DBF = (1U << 16);/* Bit 16: Debug freeze */

/* CAN master status register */

constexpr unsigned long MSR_INAK = (1U << 0); /* Bit 0: Initialization Acknowledge */
constexpr unsigned long MSR_SLAK = (1U << 1); /* Bit 1: Sleep Acknowledge */
constexpr unsigned long MSR_ERRI = (1U << 2); /* Bit 2: Error Interrupt */
constexpr unsigned long MSR_WKUI = (1U << 3); /* Bit 3: Wakeup Interrupt */
constexpr unsigned long MSR_SLAKI = (1U << 4); /* Bit 4: Sleep acknowledge interrupt */
constexpr unsigned long MSR_TXM = (1U << 8); /* Bit 8: Transmit Mode */
constexpr unsigned long MSR_RXM = (1U << 9); /* Bit 9: Receive Mode */
constexpr unsigned long MSR_SAMP = (1U << 10);/* Bit 10: Last Sample Point */
constexpr unsigned long MSR_RX = (1U << 11);/* Bit 11: CAN Rx Signal */

/* CAN transmit status register */

constexpr unsigned long TSR_RQCP0 = (1U << 0); /* Bit 0: Request Completed Mailbox 0 */
constexpr unsigned long TSR_TXOK0 = (1U << 1); /* Bit 1 : Transmission OK of Mailbox 0 */
constexpr unsigned long TSR_ALST0 = (1U << 2); /* Bit 2 : Arbitration Lost for Mailbox 0 */
constexpr unsigned long TSR_TERR0 = (1U << 3); /* Bit 3 : Transmission Error of Mailbox 0 */
constexpr unsigned long TSR_ABRQ0 = (1U << 7); /* Bit 7 : Abort Request for Mailbox 0 */
constexpr unsigned long TSR_RQCP1 = (1U << 8); /* Bit 8 : Request Completed Mailbox 1 */
constexpr unsigned long TSR_TXOK1 = (1U << 9); /* Bit 9 : Transmission OK of Mailbox 1 */
constexpr unsigned long TSR_ALST1 = (1U << 10);/* Bit 10 : Arbitration Lost for Mailbox 1 */
constexpr unsigned long TSR_TERR1 = (1U << 11);/* Bit 11 : Transmission Error of Mailbox 1 */
constexpr unsigned long TSR_ABRQ1 = (1U << 15);/* Bit 15 : Abort Request for Mailbox 1 */
constexpr unsigned long TSR_RQCP2 = (1U << 16);/* Bit 16 : Request Completed Mailbox 2 */
constexpr unsigned long TSR_TXOK2 = (1U << 17);/* Bit 17 : Transmission OK of Mailbox 2 */
constexpr unsigned long TSR_ALST2 = (1U << 18);/* Bit 18: Arbitration Lost for Mailbox 2 */
constexpr unsigned long TSR_TERR2 = (1U << 19);/* Bit 19: Transmission Error of Mailbox 2 */
constexpr unsigned long TSR_ABRQ2 = (1U << 23);/* Bit 23: Abort Request for Mailbox 2 */
constexpr unsigned long TSR_CODE_SHIFT = (24U); /* Bits 25-24: Mailbox Code */
constexpr unsigned long TSR_CODE_MASK = (3U << TSR_CODE_SHIFT);
constexpr unsigned long TSR_TME0 = (1U << 26);/* Bit 26: Transmit Mailbox 0 Empty */
constexpr unsigned long TSR_TME1 = (1U << 27);/* Bit 27: Transmit Mailbox 1 Empty */
constexpr unsigned long TSR_TME2 = (1U << 28);/* Bit 28: Transmit Mailbox 2 Empty */
constexpr unsigned long TSR_LOW0 = (1U << 29);/* Bit 29: Lowest Priority Flag for Mailbox 0 */
constexpr unsigned long TSR_LOW1 = (1U << 30);/* Bit 30: Lowest Priority Flag for Mailbox 1 */
constexpr unsigned long TSR_LOW2 = (1U << 31);/* Bit 31: Lowest Priority Flag for Mailbox 2 */

/* CAN receive FIFO 0/1 registers */

constexpr unsigned long RFR_FMP_SHIFT = (0U); /* Bits 1-0: FIFO Message Pending */
constexpr unsigned long RFR_FMP_MASK = (3U << RFR_FMP_SHIFT);
constexpr unsigned long RFR_FULL = (1U << 3); /* Bit 3: FIFO 0 Full */
constexpr unsigned long RFR_FOVR = (1U << 4); /* Bit 4: FIFO 0 Overrun */
constexpr unsigned long RFR_RFOM = (1U << 5); /* Bit 5: Release FIFO 0 Output Mailbox */

/* CAN interrupt enable register */

constexpr unsigned long IER_TMEIE = (1U << 0); /* Bit 0: Transmit Mailbox Empty Interrupt Enable */
constexpr unsigned long IER_FMPIE0 = (1U << 1); /* Bit 1: FIFO Message Pending Interrupt Enable */
constexpr unsigned long IER_FFIE0 = (1U << 2); /* Bit 2: FIFO Full Interrupt Enable */
constexpr unsigned long IER_FOVIE0 = (1U << 3); /* Bit 3: FIFO Overrun Interrupt Enable */
constexpr unsigned long IER_FMPIE1 = (1U << 4); /* Bit 4: FIFO Message Pending Interrupt Enable */
constexpr unsigned long IER_FFIE1 = (1U << 5); /* Bit 5: FIFO Full Interrupt Enable */
constexpr unsigned long IER_FOVIE1 = (1U << 6); /* Bit 6: FIFO Overrun Interrupt Enable */
constexpr unsigned long IER_EWGIE = (1U << 8); /* Bit 8: Error Warning Interrupt Enable */
constexpr unsigned long IER_EPVIE = (1U << 9); /* Bit 9: Error Passive Interrupt Enable */
constexpr unsigned long IER_BOFIE = (1U << 10);/* Bit 10: Bus-Off Interrupt Enable */
constexpr unsigned long IER_LECIE = (1U << 11);/* Bit 11: Last Error Code Interrupt Enable */
constexpr unsigned long IER_ERRIE = (1U << 15);/* Bit 15: Error Interrupt Enable */
constexpr unsigned long IER_WKUIE = (1U << 16);/* Bit 16: Wakeup Interrupt Enable */
constexpr unsigned long IER_SLKIE = (1U << 17);/* Bit 17: Sleep Interrupt Enable */

/* CAN error status register */

constexpr unsigned long ESR_EWGF = (1U << 0); /* Bit 0: Error Warning Flag */
constexpr unsigned long ESR_EPVF = (1U << 1); /* Bit 1: Error Passive Flag */
constexpr unsigned long ESR_BOFF = (1U << 2); /* Bit 2: Bus-Off Flag */
constexpr unsigned long ESR_LEC_SHIFT = (4U); /* Bits 6-4: Last Error Code */
constexpr unsigned long ESR_LEC_MASK = (7U << ESR_LEC_SHIFT);
constexpr unsigned long ESR_NOERROR = (0U << ESR_LEC_SHIFT);/* 000: No Error */
constexpr unsigned long ESR_STUFFERROR = (1U << ESR_LEC_SHIFT);/* 001: Stuff Error */
constexpr unsigned long ESR_FORMERROR = (2U << ESR_LEC_SHIFT);/* 010: Form Error */
constexpr unsigned long ESR_ACKERROR = (3U << ESR_LEC_SHIFT);/* 011: Acknowledgment Error */
constexpr unsigned long ESR_BRECERROR = (4U << ESR_LEC_SHIFT);/* 100: Bit recessive Error */
constexpr unsigned long ESR_BDOMERROR = (5U << ESR_LEC_SHIFT);/* 101: Bit dominant Error */
constexpr unsigned long ESR_CRCERRPR = (6U << ESR_LEC_SHIFT);/* 110: CRC Error */
constexpr unsigned long ESR_SWERROR = (7U << ESR_LEC_SHIFT);/* 111: Set by software */
constexpr unsigned long ESR_TEC_SHIFT = (16U); /* Bits 23-16: LS byte of the 9-bit Transmit Error Counter */
constexpr unsigned long ESR_TEC_MASK = (0xFFU << ESR_TEC_SHIFT);
constexpr unsigned long ESR_REC_SHIFT = (24U); /* Bits 31-24: Receive Error Counter */
constexpr unsigned long ESR_REC_MASK = (0xFFU << ESR_REC_SHIFT);

/* CAN bit timing register */

constexpr unsigned long BTR_BRP_SHIFT = (0U); /* Bits 9-0: Baud Rate Prescaler */
constexpr unsigned long BTR_BRP_MASK = (0x03FFU << BTR_BRP_SHIFT);
constexpr unsigned long BTR_TS1_SHIFT = (16U); /* Bits 19-16: Time Segment 1 */
constexpr unsigned long BTR_TS1_MASK = (0x0FU << BTR_TS1_SHIFT);
constexpr unsigned long BTR_TS2_SHIFT = (20U); /* Bits 22-20: Time Segment 2 */
constexpr unsigned long BTR_TS2_MASK = (7U << BTR_TS2_SHIFT);
constexpr unsigned long BTR_SJW_SHIFT = (24U); /* Bits 25-24: Resynchronization Jump Width */
constexpr unsigned long BTR_SJW_MASK = (3U << BTR_SJW_SHIFT);
constexpr unsigned long BTR_LBKM = (1U << 30);/* Bit 30: Loop Back Mode (Debug);*/
constexpr unsigned long BTR_SILM = (1U << 31);/* Bit 31: Silent Mode (Debug);*/

constexpr unsigned long BTR_BRP_MAX = (1024U); /* Maximum BTR value (without decrement);*/
constexpr unsigned long BTR_TSEG1_MAX = (16U); /* Maximum TSEG1 value (without decrement);*/
constexpr unsigned long BTR_TSEG2_MAX = (8U); /* Maximum TSEG2 value (without decrement);*/

/* TX mailbox identifier register */

constexpr unsigned long TIR_TXRQ = (1U << 0); /* Bit 0: Transmit Mailbox Request */
constexpr unsigned long TIR_RTR = (1U << 1); /* Bit 1: Remote Transmission Request */
constexpr unsigned long TIR_IDE = (1U << 2); /* Bit 2: Identifier Extension */
constexpr unsigned long TIR_EXID_SHIFT = (3U); /* Bit 3-31: Extended Identifier */
constexpr unsigned long TIR_EXID_MASK = (0x1FFFFFFFU << TIR_EXID_SHIFT);
constexpr unsigned long TIR_STID_SHIFT = (21U); /* Bits 21-31: Standard Identifier */
constexpr unsigned long TIR_STID_MASK = (0x07FFU << TIR_STID_SHIFT);

/* Mailbox data length control and time stamp register */

constexpr unsigned long TDTR_DLC_SHIFT = (0U); /* Bits 3:0: Data Length Code */
constexpr unsigned long TDTR_DLC_MASK = (0x0FU << TDTR_DLC_SHIFT);
constexpr unsigned long TDTR_TGT = (1U << 8); /* Bit 8: Transmit Global Time */
constexpr unsigned long TDTR_TIME_SHIFT = (16U); /* Bits 31:16: Message Time Stamp */
constexpr unsigned long TDTR_TIME_MASK = (0xFFFFU << TDTR_TIME_SHIFT);

/* Mailbox data low register */

constexpr unsigned long TDLR_DATA0_SHIFT = (0U); /* Bits 7-0: Data Byte 0 */
constexpr unsigned long TDLR_DATA0_MASK = (0xFFU << TDLR_DATA0_SHIFT);
constexpr unsigned long TDLR_DATA1_SHIFT = (8U); /* Bits 15-8: Data Byte 1 */
constexpr unsigned long TDLR_DATA1_MASK = (0xFFU << TDLR_DATA1_SHIFT);
constexpr unsigned long TDLR_DATA2_SHIFT = (16U); /* Bits 23-16: Data Byte 2 */
constexpr unsigned long TDLR_DATA2_MASK = (0xFFU << TDLR_DATA2_SHIFT);
constexpr unsigned long TDLR_DATA3_SHIFT = (24U); /* Bits 31-24: Data Byte 3 */
constexpr unsigned long TDLR_DATA3_MASK = (0xFFU << TDLR_DATA3_SHIFT);

/* Mailbox data high register */

constexpr unsigned long TDHR_DATA4_SHIFT = (0U); /* Bits 7-0: Data Byte 4 */
constexpr unsigned long TDHR_DATA4_MASK = (0xFFU << TDHR_DATA4_SHIFT);
constexpr unsigned long TDHR_DATA5_SHIFT = (8U); /* Bits 15-8: Data Byte 5 */
constexpr unsigned long TDHR_DATA5_MASK = (0xFFU << TDHR_DATA5_SHIFT);
constexpr unsigned long TDHR_DATA6_SHIFT = (16U); /* Bits 23-16: Data Byte 6 */
constexpr unsigned long TDHR_DATA6_MASK = (0xFFU << TDHR_DATA6_SHIFT);
constexpr unsigned long TDHR_DATA7_SHIFT = (24U); /* Bits 31-24: Data Byte 7 */
constexpr unsigned long TDHR_DATA7_MASK = (0xFFU << TDHR_DATA7_SHIFT);

/* Rx FIFO mailbox identifier register */

constexpr unsigned long RIR_RTR = (1U << 1); /* Bit 1: Remote Transmission Request */
constexpr unsigned long RIR_IDE = (1U << 2); /* Bit 2: Identifier Extension */
constexpr unsigned long RIR_EXID_SHIFT = (3U); /* Bit 3-31: Extended Identifier */
constexpr unsigned long RIR_EXID_MASK = (0x1FFFFFFFU << RIR_EXID_SHIFT);
constexpr unsigned long RIR_STID_SHIFT = (21U); /* Bits 21-31: Standard Identifier */
constexpr unsigned long RIR_STID_MASK = (0x07FFU << RIR_STID_SHIFT);

/* Receive FIFO mailbox data length control and time stamp register */

constexpr unsigned long RDTR_DLC_SHIFT = (0U); /* Bits 3:0: Data Length Code */
constexpr unsigned long RDTR_DLC_MASK = (0x0FU << RDTR_DLC_SHIFT);
constexpr unsigned long RDTR_FM_SHIFT = (8U); /* Bits 15-8: Filter Match Index */
constexpr unsigned long RDTR_FM_MASK = (0xFFU << RDTR_FM_SHIFT);
constexpr unsigned long RDTR_TIME_SHIFT = (16U); /* Bits 31:16: Message Time Stamp */
constexpr unsigned long RDTR_TIME_MASK = (0xFFFFU << RDTR_TIME_SHIFT);

/* Receive FIFO mailbox data low register */

constexpr unsigned long RDLR_DATA0_SHIFT = (0U); /* Bits 7-0: Data Byte 0 */
constexpr unsigned long RDLR_DATA0_MASK = (0xFFU << RDLR_DATA0_SHIFT);
constexpr unsigned long RDLR_DATA1_SHIFT = (8U); /* Bits 15-8: Data Byte 1 */
constexpr unsigned long RDLR_DATA1_MASK = (0xFFU << RDLR_DATA1_SHIFT);
constexpr unsigned long RDLR_DATA2_SHIFT = (16U); /* Bits 23-16: Data Byte 2 */
constexpr unsigned long RDLR_DATA2_MASK = (0xFFU << RDLR_DATA2_SHIFT);
constexpr unsigned long RDLR_DATA3_SHIFT = (24U); /* Bits 31-24: Data Byte 3 */
constexpr unsigned long RDLR_DATA3_MASK = (0xFFU << RDLR_DATA3_SHIFT);

/* Receive FIFO mailbox data high register */

constexpr unsigned long RDHR_DATA4_SHIFT = (0U); /* Bits 7-0: Data Byte 4 */
constexpr unsigned long RDHR_DATA4_MASK = (0xFFU << RDHR_DATA4_SHIFT);
constexpr unsigned long RDHR_DATA5_SHIFT = (8U); /* Bits 15-8: Data Byte 5 */
constexpr unsigned long RDHR_DATA5_MASK = (0xFFU << RDHR_DATA5_SHIFT);
constexpr unsigned long RDHR_DATA6_SHIFT = (16U); /* Bits 23-16: Data Byte 6 */
constexpr unsigned long RDHR_DATA6_MASK = (0xFFU << RDHR_DATA6_SHIFT);
constexpr unsigned long RDHR_DATA7_SHIFT = (24U); /* Bits 31-24: Data Byte 7 */
constexpr unsigned long RDHR_DATA7_MASK = (0xFFU << RDHR_DATA7_SHIFT);

/* CAN filter master register */

constexpr unsigned long FMR_FINIT = (1U << 0); /* Bit 0:  Filter Init Mode */
}
}

#if UAVCAN_CPP_VERSION < UAVCAN_CPP11
# undef constexpr
#endif
