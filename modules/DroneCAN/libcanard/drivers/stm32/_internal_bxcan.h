/*
 * Copyright (c) 2017 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#ifndef CANARD_STM32_BXCAN_H
#define CANARD_STM32_BXCAN_H

#include <stdint.h>


typedef struct
{
    volatile uint32_t TIR;
    volatile uint32_t TDTR;
    volatile uint32_t TDLR;
    volatile uint32_t TDHR;
} CanardSTM32TxMailboxType;

typedef struct
{
    volatile uint32_t RIR;
    volatile uint32_t RDTR;
    volatile uint32_t RDLR;
    volatile uint32_t RDHR;
} CanardSTM32RxMailboxType;

typedef struct
{
    volatile uint32_t FR1;
    volatile uint32_t FR2;
} CanardSTM32FilterRegisterType;

typedef struct
{
    volatile uint32_t             MCR;                  ///< CAN master control register         0x000
    volatile uint32_t             MSR;                  ///< CAN master status register          0x004
    volatile uint32_t             TSR;                  ///< CAN transmit status register        0x008
    volatile uint32_t             RF0R;                 ///< CAN receive FIFO 0 register         0x00C
    volatile uint32_t             RF1R;                 ///< CAN receive FIFO 1 register         0x010
    volatile uint32_t             IER;                  ///< CAN interrupt enable register       0x014
    volatile uint32_t             ESR;                  ///< CAN error status register           0x018
    volatile uint32_t             BTR;                  ///< CAN bit timing register             0x01C
    const    uint32_t             RESERVED0[88];        ///< Reserved                            0x020-0x17F
    CanardSTM32TxMailboxType      TxMailbox[3];         ///< CAN Tx MailBox                      0x180-0x1AC
    CanardSTM32RxMailboxType      RxMailbox[2];         ///< CAN FIFO MailBox                    0x1B0-0x1CC
    const    uint32_t             RESERVED1[12];        ///< Reserved                            0x1D0-0x1FF
    volatile uint32_t             FMR;                  ///< CAN filter master register          0x200
    volatile uint32_t             FM1R;                 ///< CAN filter mode register            0x204
    const    uint32_t             RESERVED2;            ///< Reserved                            0x208
    volatile uint32_t             FS1R;                 ///< CAN filter scale register           0x20C
    const    uint32_t             RESERVED3;            ///< Reserved                            0x210
    volatile uint32_t             FFA1R;                ///< CAN filter FIFO assignment register 0x214
    const    uint32_t             RESERVED4;            ///< Reserved                            0x218
    volatile uint32_t             FA1R;                 ///< CAN filter activation register      0x21C
    const    uint32_t             RESERVED5[8];         ///< Reserved                            0x220-0x23F
    CanardSTM32FilterRegisterType FilterRegister[28];   ///< CAN Filter Register                 0x240-0x31C
} CanardSTM32CANType;

/**
 * CANx instances
 */
#define CANARD_STM32_CAN1       ((volatile CanardSTM32CANType*)0x40006400U)
#define CANARD_STM32_CAN2       ((volatile CanardSTM32CANType*)0x40006800U)

// CAN master control register

#define CANARD_STM32_CAN_MCR_INRQ              (1U << 0U)  // Bit 0: Initialization Request
#define CANARD_STM32_CAN_MCR_SLEEP             (1U << 1U)  // Bit 1: Sleep Mode Request
#define CANARD_STM32_CAN_MCR_TXFP              (1U << 2U)  // Bit 2: Transmit FIFO Priority
#define CANARD_STM32_CAN_MCR_RFLM              (1U << 3U)  // Bit 3: Receive FIFO Locked Mode
#define CANARD_STM32_CAN_MCR_NART              (1U << 4U)  // Bit 4: No Automatic Retransmission
#define CANARD_STM32_CAN_MCR_AWUM              (1U << 5U)  // Bit 5: Automatic Wakeup Mode
#define CANARD_STM32_CAN_MCR_ABOM              (1U << 6U)  // Bit 6: Automatic Bus-Off Management
#define CANARD_STM32_CAN_MCR_TTCM              (1U << 7U)  // Bit 7: Time Triggered Communication Mode Enable
#define CANARD_STM32_CAN_MCR_RESET             (1U << 15U) // Bit 15: bxCAN software master reset
#define CANARD_STM32_CAN_MCR_DBF               (1U << 16U) // Bit 16: Debug freeze

// CAN master status register

#define CANARD_STM32_CAN_MSR_INAK              (1U << 0U)  // Bit 0: Initialization Acknowledge
#define CANARD_STM32_CAN_MSR_SLAK              (1U << 1U)  // Bit 1: Sleep Acknowledge
#define CANARD_STM32_CAN_MSR_ERRI              (1U << 2U)  // Bit 2: Error Interrupt
#define CANARD_STM32_CAN_MSR_WKUI              (1U << 3U)  // Bit 3: Wakeup Interrupt
#define CANARD_STM32_CAN_MSR_SLAKI             (1U << 4U)  // Bit 4: Sleep acknowledge interrupt
#define CANARD_STM32_CAN_MSR_TXM               (1U << 8U)  // Bit 8: Transmit Mode
#define CANARD_STM32_CAN_MSR_RXM               (1U << 9U)  // Bit 9: Receive Mode
#define CANARD_STM32_CAN_MSR_SAMP              (1U << 10U) // Bit 10: Last Sample Point
#define CANARD_STM32_CAN_MSR_RX                (1U << 11U) // Bit 11: CAN Rx Signal

// CAN transmit status register

#define CANARD_STM32_CAN_TSR_RQCP0             (1U << 0U)  // Bit 0: Request Completed Mailbox 0
#define CANARD_STM32_CAN_TSR_TXOK0             (1U << 1U)  // Bit 1 : Transmission OK of Mailbox 0
#define CANARD_STM32_CAN_TSR_ALST0             (1U << 2U)  // Bit 2 : Arbitration Lost for Mailbox 0
#define CANARD_STM32_CAN_TSR_TERR0             (1U << 3U)  // Bit 3 : Transmission Error of Mailbox 0
#define CANARD_STM32_CAN_TSR_ABRQ0             (1U << 7U)  // Bit 7 : Abort Request for Mailbox 0
#define CANARD_STM32_CAN_TSR_RQCP1             (1U << 8U)  // Bit 8 : Request Completed Mailbox 1
#define CANARD_STM32_CAN_TSR_TXOK1             (1U << 9U)  // Bit 9 : Transmission OK of Mailbox 1
#define CANARD_STM32_CAN_TSR_ALST1             (1U << 10U) // Bit 10 : Arbitration Lost for Mailbox 1
#define CANARD_STM32_CAN_TSR_TERR1             (1U << 11U) // Bit 11 : Transmission Error of Mailbox 1
#define CANARD_STM32_CAN_TSR_ABRQ1             (1U << 15U) // Bit 15 : Abort Request for Mailbox 1
#define CANARD_STM32_CAN_TSR_RQCP2             (1U << 16U) // Bit 16 : Request Completed Mailbox 2
#define CANARD_STM32_CAN_TSR_TXOK2             (1U << 17U) // Bit 17 : Transmission OK of Mailbox 2
#define CANARD_STM32_CAN_TSR_ALST2             (1U << 18U) // Bit 18: Arbitration Lost for Mailbox 2
#define CANARD_STM32_CAN_TSR_TERR2             (1U << 19U) // Bit 19: Transmission Error of Mailbox 2
#define CANARD_STM32_CAN_TSR_ABRQ2             (1U << 23U) // Bit 23: Abort Request for Mailbox 2
#define CANARD_STM32_CAN_TSR_CODE_SHIFT        (24U)       // Bits 25-24: Mailbox Code
#define CANARD_STM32_CAN_TSR_CODE_MASK         (3U << CANARD_STM32_CAN_TSR_CODE_SHIFT)
#define CANARD_STM32_CAN_TSR_TME0              (1U << 26U) // Bit 26: Transmit Mailbox 0 Empty
#define CANARD_STM32_CAN_TSR_TME1              (1U << 27U) // Bit 27: Transmit Mailbox 1 Empty
#define CANARD_STM32_CAN_TSR_TME2              (1U << 28U) // Bit 28: Transmit Mailbox 2 Empty
#define CANARD_STM32_CAN_TSR_LOW0              (1U << 29U) // Bit 29: Lowest Priority Flag for Mailbox 0
#define CANARD_STM32_CAN_TSR_LOW1              (1U << 30U) // Bit 30: Lowest Priority Flag for Mailbox 1
#define CANARD_STM32_CAN_TSR_LOW2              (1U << 31U) // Bit 31: Lowest Priority Flag for Mailbox 2

// CAN receive FIFO 0/1 registers

#define CANARD_STM32_CAN_RFR_FMP_SHIFT         (0U)        // Bits 1-0: FIFO Message Pending
#define CANARD_STM32_CAN_RFR_FMP_MASK          (3U << CANARD_STM32_CAN_RFR_FMP_SHIFT)
#define CANARD_STM32_CAN_RFR_FULL              (1U << 3U)  // Bit 3: FIFO 0 Full
#define CANARD_STM32_CAN_RFR_FOVR              (1U << 4U)  // Bit 4: FIFO 0 Overrun
#define CANARD_STM32_CAN_RFR_RFOM              (1U << 5U)  // Bit 5: Release FIFO 0 Output Mailbox

// CAN interrupt enable register

#define CANARD_STM32_CAN_IER_TMEIE             (1U << 0U)  // Bit 0: Transmit Mailbox Empty Interrupt Enable
#define CANARD_STM32_CAN_IER_FMPIE0            (1U << 1U)  // Bit 1: FIFO Message Pending Interrupt Enable
#define CANARD_STM32_CAN_IER_FFIE0             (1U << 2U)  // Bit 2: FIFO Full Interrupt Enable
#define CANARD_STM32_CAN_IER_FOVIE0            (1U << 3U)  // Bit 3: FIFO Overrun Interrupt Enable
#define CANARD_STM32_CAN_IER_FMPIE1            (1U << 4U)  // Bit 4: FIFO Message Pending Interrupt Enable
#define CANARD_STM32_CAN_IER_FFIE1             (1U << 5U)  // Bit 5: FIFO Full Interrupt Enable
#define CANARD_STM32_CAN_IER_FOVIE1            (1U << 6U)  // Bit 6: FIFO Overrun Interrupt Enable
#define CANARD_STM32_CAN_IER_EWGIE             (1U << 8U)  // Bit 8: Error Warning Interrupt Enable
#define CANARD_STM32_CAN_IER_EPVIE             (1U << 9U)  // Bit 9: Error Passive Interrupt Enable
#define CANARD_STM32_CAN_IER_BOFIE             (1U << 10U) // Bit 10: Bus-Off Interrupt Enable
#define CANARD_STM32_CAN_IER_LECIE             (1U << 11U) // Bit 11: Last Error Code Interrupt Enable
#define CANARD_STM32_CAN_IER_ERRIE             (1U << 15U) // Bit 15: Error Interrupt Enable
#define CANARD_STM32_CAN_IER_WKUIE             (1U << 16U) // Bit 16: Wakeup Interrupt Enable
#define CANARD_STM32_CAN_IER_SLKIE             (1U << 17U) // Bit 17: Sleep Interrupt Enable

// CAN error status register

#define CANARD_STM32_CAN_ESR_EWGF              (1U << 0U)  // Bit 0: Error Warning Flag
#define CANARD_STM32_CAN_ESR_EPVF              (1U << 1U)  // Bit 1: Error Passive Flag
#define CANARD_STM32_CAN_ESR_BOFF              (1U << 2U)  // Bit 2: Bus-Off Flag
#define CANARD_STM32_CAN_ESR_LEC_SHIFT         (4U)        // Bits 6-4: Last Error Code
#define CANARD_STM32_CAN_ESR_LEC_MASK          (7U << CANARD_STM32_CAN_ESR_LEC_SHIFT)
#define CANARD_STM32_CAN_ESR_NOERROR           (0U << CANARD_STM32_CAN_ESR_LEC_SHIFT) // 000: No Error
#define CANARD_STM32_CAN_ESR_STUFFERROR        (1U << CANARD_STM32_CAN_ESR_LEC_SHIFT) // 001: Stuff Error
#define CANARD_STM32_CAN_ESR_FORMERROR         (2U << CANARD_STM32_CAN_ESR_LEC_SHIFT) // 010: Form Error
#define CANARD_STM32_CAN_ESR_ACKERROR          (3U << CANARD_STM32_CAN_ESR_LEC_SHIFT) // 011: Acknowledgment Error
#define CANARD_STM32_CAN_ESR_BRECERROR         (4U << CANARD_STM32_CAN_ESR_LEC_SHIFT) // 100: Bit recessive Error
#define CANARD_STM32_CAN_ESR_BDOMERROR         (5U << CANARD_STM32_CAN_ESR_LEC_SHIFT) // 101: Bit dominant Error
#define CANARD_STM32_CAN_ESR_CRCERRPR          (6U << CANARD_STM32_CAN_ESR_LEC_SHIFT) // 110: CRC Error
#define CANARD_STM32_CAN_ESR_SWERROR           (7U << CANARD_STM32_CAN_ESR_LEC_SHIFT) // 111: Set by software
#define CANARD_STM32_CAN_ESR_TEC_SHIFT         (16U)       // Bits 23-16: LS byte of the 9-bit Transmit Error Counter
#define CANARD_STM32_CAN_ESR_TEC_MASK          (0xFFU << CANARD_STM32_CAN_ESR_TEC_SHIFT)
#define CANARD_STM32_CAN_ESR_REC_SHIFT         (24U)       // Bits 31-24: Receive Error Counter
#define CANARD_STM32_CAN_ESR_REC_MASK          (0xFFU << CANARD_STM32_CAN_ESR_REC_SHIFT)

// CAN bit timing register

#define CANARD_STM32_CAN_BTR_BRP_SHIFT         (0U)        // Bits 9-0: Baud Rate Prescaler
#define CANARD_STM32_CAN_BTR_BRP_MASK          (0x03FFU << CANARD_STM32_CAN_BTR_BRP_SHIFT)
#define CANARD_STM32_CAN_BTR_TS1_SHIFT         (16U)       // Bits 19-16: Time Segment 1
#define CANARD_STM32_CAN_BTR_TS1_MASK          (0x0FU <<  CANARD_STM32_CAN_BTR_TS1_SHIFT)
#define CANARD_STM32_CAN_BTR_TS2_SHIFT         (20U)       // Bits 22-20: Time Segment 2
#define CANARD_STM32_CAN_BTR_TS2_MASK          (7U << CANARD_STM32_CAN_BTR_TS2_SHIFT)
#define CANARD_STM32_CAN_BTR_SJW_SHIFT         (24U)       // Bits 25-24: Resynchronization Jump Width
#define CANARD_STM32_CAN_BTR_SJW_MASK          (3U << CANARD_STM32_CAN_BTR_SJW_SHIFT)
#define CANARD_STM32_CAN_BTR_LBKM              (1U << 30U) // Bit 30: Loop Back Mode (Debug);
#define CANARD_STM32_CAN_BTR_SILM              (1U << 31U) // Bit 31: Silent Mode (Debug);

#define CANARD_STM32_CAN_BTR_BRP_MAX           (1024U)     // Maximum BTR value (without decrement);
#define CANARD_STM32_CAN_BTR_TSEG1_MAX         (16U)       // Maximum TSEG1 value (without decrement);
#define CANARD_STM32_CAN_BTR_TSEG2_MAX         (8U)        // Maximum TSEG2 value (without decrement);

// TX mailbox identifier register

#define CANARD_STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define CANARD_STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define CANARD_STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define CANARD_STM32_CAN_TIR_EXID_SHIFT        (3U)        // Bit 3-31: Extended Identifier
#define CANARD_STM32_CAN_TIR_EXID_MASK         (0x1FFFFFFFU << CANARD_STM32_CAN_TIR_EXID_SHIFT)
#define CANARD_STM32_CAN_TIR_STID_SHIFT        (21U)       // Bits 21-31: Standard Identifier
#define CANARD_STM32_CAN_TIR_STID_MASK         (0x07FFU << CANARD_STM32_CAN_TIR_STID_SHIFT)

// Mailbox data length control and time stamp register

#define CANARD_STM32_CAN_TDTR_DLC_SHIFT        (0U)        // Bits 3:0: Data Length Code
#define CANARD_STM32_CAN_TDTR_DLC_MASK         (0x0FU << CANARD_STM32_CAN_TDTR_DLC_SHIFT)
#define CANARD_STM32_CAN_TDTR_TGT              (1U << 8U)  // Bit 8: Transmit Global Time
#define CANARD_STM32_CAN_TDTR_TIME_SHIFT       (16U)       // Bits 31:16: Message Time Stamp
#define CANARD_STM32_CAN_TDTR_TIME_MASK        (0xFFFFU << CANARD_STM32_CAN_TDTR_TIME_SHIFT)

// Rx FIFO mailbox identifier register

#define CANARD_STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define CANARD_STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define CANARD_STM32_CAN_RIR_EXID_SHIFT        (3U)        // Bit 3-31: Extended Identifier
#define CANARD_STM32_CAN_RIR_EXID_MASK         (0x1FFFFFFFU << CANARD_STM32_CAN_RIR_EXID_SHIFT)
#define CANARD_STM32_CAN_RIR_STID_SHIFT        (21U)       // Bits 21-31: Standard Identifier
#define CANARD_STM32_CAN_RIR_STID_MASK         (0x07FFU << CANARD_STM32_CAN_RIR_STID_SHIFT)

// Receive FIFO mailbox data length control and time stamp register

#define CANARD_STM32_CAN_RDTR_DLC_SHIFT        (0U)        // Bits 3:0: Data Length Code
#define CANARD_STM32_CAN_RDTR_DLC_MASK         (0x0FU << CANARD_STM32_CAN_RDTR_DLC_SHIFT)
#define CANARD_STM32_CAN_RDTR_FM_SHIFT         (8U)        // Bits 15-8: Filter Match Index
#define CANARD_STM32_CAN_RDTR_FM_MASK          (0xFFU << CANARD_STM32_CAN_RDTR_FM_SHIFT)
#define CANARD_STM32_CAN_RDTR_TIME_SHIFT       (16U)       // Bits 31:16: Message Time Stamp
#define CANARD_STM32_CAN_RDTR_TIME_MASK        (0xFFFFU << CANARD_STM32_CAN_RDTR_TIME_SHIFT)

// CAN filter master register

#define CANARD_STM32_CAN_FMR_FINIT             (1U << 0U)  // Bit 0:  Filter Init Mode

#endif // CANARD_STM32_BXCAN_H
