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
 * @file    hal_mii.h
 * @brief   MII macros and structures.
 *
 * @addtogroup MII
 * @{
 */

#ifndef MII_H
#define MII_H

/**
 * @name    Generic MII registers
 * @{
 */
#define MII_BMCR                0x00    /**< Basic mode control register.   */
#define MII_BMSR                0x01    /**< Basic mode status register.    */
#define MII_PHYSID1             0x02    /**< PHYS ID 1.                     */
#define MII_PHYSID2             0x03    /**< PHYS ID 2.                     */
#define MII_ADVERTISE           0x04    /**< Advertisement control reg.     */
#define MII_LPA                 0x05    /**< Link partner ability reg.      */
#define MII_EXPANSION           0x06    /**< Expansion register.            */
#define MII_ANNPTR              0x07    /**< 1000BASE-T control.            */
#define MII_CTRL1000            0x09    /**< 1000BASE-T control.            */
#define MII_STAT1000            0x0a    /**< 1000BASE-T status.             */
#define MII_ESTATUS             0x0f    /**< Extended Status.               */
#define MII_PHYSTS              0x10    /**< PHY Status register.           */
#define MII_MICR                0x11    /**< MII Interrupt ctrl register.   */
#define MII_DCOUNTER            0x12    /**< Disconnect counter.            */
#define MII_FCSCOUNTER          0x13    /**< False carrier counter.         */
#define MII_NWAYTEST            0x14    /**< N-way auto-neg test reg.       */
#define MII_RERRCOUNTER         0x15    /**< Receive error counter.         */
#define MII_SREVISION           0x16    /**< Silicon revision.              */
#define MII_RESV1               0x17    /**< Reserved.                      */
#define MII_LBRERROR            0x18    /**< Lpback, rx, bypass error.      */
#define MII_PHYADDR             0x19    /**< PHY address.                   */
#define MII_RESV2               0x1a    /**< Reserved.                      */
#define MII_TPISTATUS           0x1b    /**< TPI status for 10Mbps.         */
#define MII_NCONFIG             0x1c    /**< Network interface config.      */
/** @} */

/**
 * @name    Basic mode control register
 * @{
 */
#define BMCR_RESV               0x007f  /**< Unused.                        */
#define BMCR_CTST               0x0080  /**< Collision test.                */
#define BMCR_FULLDPLX           0x0100  /**< Full duplex.                   */
#define BMCR_ANRESTART          0x0200  /**< Auto negotiation restart.      */
#define BMCR_ISOLATE            0x0400  /**< Disconnect DP83840 from MII.   */
#define BMCR_PDOWN              0x0800  /**< Powerdown.                     */
#define BMCR_ANENABLE           0x1000  /**< Enable auto negotiation.       */
#define BMCR_SPEED100           0x2000  /**< Select 100Mbps.                */
#define BMCR_LOOPBACK           0x4000  /**< TXD loopback bit.              */
#define BMCR_RESET              0x8000  /**< Reset.                         */
/** @} */

/**
 * @name    Basic mode status register
 * @{
 */
#define BMSR_ERCAP              0x0001  /**< Ext-reg capability.            */
#define BMSR_JCD                0x0002  /**< Jabber detected.               */
#define BMSR_LSTATUS            0x0004  /**< Link status.                   */
#define BMSR_ANEGCAPABLE        0x0008  /**< Able to do auto-negotiation.   */
#define BMSR_RFAULT             0x0010  /**< Remote fault detected.         */
#define BMSR_ANEGCOMPLETE       0x0020  /**< Auto-negotiation complete.     */
#define BMSR_MFPRESUPPCAP       0x0040  /**< Able to suppress preamble.     */
#define BMSR_RESV               0x0780  /**< Unused.                        */
#define BMSR_10HALF             0x0800  /**< Can do 10mbps, half-duplex.    */
#define BMSR_10FULL             0x1000  /**< Can do 10mbps, full-duplex.    */
#define BMSR_100HALF            0x2000  /**< Can do 100mbps, half-duplex.   */
#define BMSR_100FULL            0x4000  /**< Can do 100mbps, full-duplex.   */
#define BMSR_100BASE4           0x8000  /**< Can do 100mbps, 4k packets.    */
/** @} */

/**
 * @name    Advertisement control register
 * @{
 */
#define ADVERTISE_SLCT          0x001f  /**< Selector bits.                 */
#define ADVERTISE_CSMA          0x0001  /**< Only selector supported.       */
#define ADVERTISE_10HALF        0x0020  /**< Try for 10mbps half-duplex.    */
#define ADVERTISE_10FULL        0x0040  /**< Try for 10mbps full-duplex.    */
#define ADVERTISE_100HALF       0x0080  /**< Try for 100mbps half-duplex.   */
#define ADVERTISE_100FULL       0x0100  /**< Try for 100mbps full-duplex.   */
#define ADVERTISE_100BASE4      0x0200  /**< Try for 100mbps 4k packets.    */
#define ADVERTISE_PAUSE_CAP     0x0400  /**< Try for pause.                 */
#define ADVERTISE_PAUSE_ASYM    0x0800  /**< Try for asymetric pause.       */
#define ADVERTISE_RESV          0x1000  /**< Unused.                        */
#define ADVERTISE_RFAULT        0x2000  /**< Say we can detect faults.      */
#define ADVERTISE_LPACK         0x4000  /**< Ack link partners response.    */
#define ADVERTISE_NPAGE         0x8000  /**< Next page bit.                 */

#define ADVERTISE_FULL (ADVERTISE_100FULL | ADVERTISE_10FULL | \
                        ADVERTISE_CSMA)
#define ADVERTISE_ALL (ADVERTISE_10HALF | ADVERTISE_10FULL | \
                       ADVERTISE_100HALF | ADVERTISE_100FULL)
/** @} */

/**
 * @name    Link partner ability register
 * @{
 */
#define LPA_SLCT                0x001f  /**< Same as advertise selector.    */
#define LPA_10HALF              0x0020  /**< Can do 10mbps half-duplex.     */
#define LPA_10FULL              0x0040  /**< Can do 10mbps full-duplex.     */
#define LPA_100HALF             0x0080  /**< Can do 100mbps half-duplex.    */
#define LPA_100FULL             0x0100  /**< Can do 100mbps full-duplex.    */
#define LPA_100BASE4            0x0200  /**< Can do 100mbps 4k packets.     */
#define LPA_PAUSE_CAP           0x0400  /**< Can pause.                     */
#define LPA_PAUSE_ASYM          0x0800  /**< Can pause asymetrically.       */
#define LPA_RESV                0x1000  /**< Unused.                        */
#define LPA_RFAULT              0x2000  /**< Link partner faulted.          */
#define LPA_LPACK               0x4000  /**< Link partner acked us.         */
#define LPA_NPAGE               0x8000  /**< Next page bit.                 */

#define LPA_DUPLEX              (LPA_10FULL | LPA_100FULL)
#define LPA_100                 (LPA_100FULL | LPA_100HALF | LPA_100BASE4)
/** @} */

/**
 * @name    Expansion register for auto-negotiation
 * @{
 */
#define EXPANSION_NWAY          0x0001  /**< Can do N-way auto-nego.        */
#define EXPANSION_LCWP          0x0002  /**< Got new RX page code word.     */
#define EXPANSION_ENABLENPAGE   0x0004  /**< This enables npage words.      */
#define EXPANSION_NPCAPABLE     0x0008  /**< Link partner supports npage.   */
#define EXPANSION_MFAULTS       0x0010  /**< Multiple faults detected.      */
#define EXPANSION_RESV          0xffe0  /**< Unused.                        */
/** @} */

/**
 * @name    N-way test register
 * @{
 */
#define NWAYTEST_RESV1          0x00ff  /**< Unused.                        */
#define NWAYTEST_LOOPBACK       0x0100  /**< Enable loopback for N-way.     */
#define NWAYTEST_RESV2          0xfe00  /**< Unused.                        */
/** @} */

/**
 * @name    PHY identifiers
 * @{
 */
#define MII_DM9161_ID           0x0181b8a0
#define MII_AM79C875_ID         0x00225540
#define MII_KSZ8081_ID          0x00221560
#define MII_KS8721_ID           0x00221610
#define MII_STE101P_ID          0x00061C50
#define MII_DP83848I_ID         0x20005C90
#define MII_LAN8710A_ID         0x0007C0F1
#define MII_LAN8720_ID          0x0007C0F0
#define MII_LAN8742A_ID         0x0007C130
#define MII_LAN8740A_ID         0x0007C110
#define MII_DP83825I_ID         0x2000A140
/** @} */

#endif /* MII_H */

/** @} */
