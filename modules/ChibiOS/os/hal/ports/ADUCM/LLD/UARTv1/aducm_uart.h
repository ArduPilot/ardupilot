/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

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
 * @file    UARTv1/aducm_uart.h
 * @brief   ADUCM UART units common header.
 * @note    This file requires definitions from the ADI ADUCM header file.
 *
 * @{
 */

#ifndef ADUCM_UART_H
#define ADUCM_UART_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* Discarded definitions from the ADI headers, the Serial/UART drivers use its
   own definitions in order to have an unified handling for all devices.
   Unfortunately the ADI headers have no uniform definitions for the same
   objects across the various sub-families.*/
#if !defined(pADI_UART0)
#define pADI_UART0                          pADI_UART
#endif

/**
 * @name    UART ports definitions
 * @{
 */
#define UART0                               ((aducm_uart_t *)pADI_UART0)
#define UART1                               ((aducm_uart_t *)pADI_UART1)
/** @} */

/**
 * @name    UART Data register bits definitions
 * @{
 */
#define ADUCM_UART_DATA_MASK                0x00FFU
/** @} */

/**
 * @name    UART Interrupt Enable register bits definitions
 * @{
 */
#define ADUCM_UART_IEN_MASK                 0x003FU
#define ADUCM_UART_IEN_ERBFI                (1U << 0)
#define ADUCM_UART_IEN_ETBEI                (1U << 1)
#define ADUCM_UART_IEN_ELSI                 (1U << 2)
#define ADUCM_UART_IEN_EDSSI                (1U << 3)
#define ADUCM_UART_IEN_EDMAT                (1U << 4)
#define ADUCM_UART_IEN_EDMAR                (1U << 5)
/** @} */

/**
 * @name    UART Interrupt Identification register bits definitions
 * @{
 */
#define ADUCM_UART_IIR_MASK                 0x0007U
#define ADUCM_UART_IIR_NINT                 (1U << 0)
#define ADUCM_UART_IIR_STA_MODEM            (0U << 1)
#define ADUCM_UART_IIR_STA_TX_EMPTY         (1U << 1)
#define ADUCM_UART_IIR_STA_RX_FULL          (2U << 1)
#define ADUCM_UART_IIR_STA_LINE             (3U << 1)
/** @} */

/**
 * @name    UART Line Control register bits definitions
 * @{
 */
#define ADUCM_UART_LCR_MASK                 0x007FU
#define ADUCM_UART_LCR_WLS_5_BITS           (0U << 0)
#define ADUCM_UART_LCR_WLS_6_BITS           (1U << 0)
#define ADUCM_UART_LCR_WLS_7_BITS           (2U << 0)
#define ADUCM_UART_LCR_WLS_8_BITS           (3U << 0)
#define ADUCM_UART_LCR_STOP                 (1U << 2)
#define ADUCM_UART_LCR_PEN                  (1U << 3)
#define ADUCM_UART_LCR_EPS                  (1U << 4)
#define ADUCM_UART_LCR_SP                   (1U << 5)
#define ADUCM_UART_LCR_BRK                  (1U << 6)
/** @} */

/**
 * @name    UART Modem Control register bits definitions
 * @{
 */
#define ADUCM_UART_MCR_MASK                 0x0013U
#define ADUCM_UART_MCR_DTR                  (1U << 0)
#define ADUCM_UART_MCR_RTS                  (1U << 1)
#define ADUCM_UART_MCR_LOOPBACK             (1U << 4)
/** @} */

/**
 * @name    UART Line Status register bits definitions
 * @{
 */
#define ADUCM_UART_LSR_MASK                 0x007FU
#define ADUCM_UART_LSR_DR                   (1U << 0)
#define ADUCM_UART_LSR_OE                   (1U << 1)
#define ADUCM_UART_LSR_PE                   (1U << 2)
#define ADUCM_UART_LSR_FE                   (1U << 3)
#define ADUCM_UART_LSR_BI                   (1U << 4)
#define ADUCM_UART_LSR_THRE                 (1U << 5)
#define ADUCM_UART_LSR_TEMT                 (1U << 6)
/** @} */

/**
 * @name    UART Modem Status register bits definitions
 * @{
 */
#define ADUCM_UART_MSR_MASK                 0x00FFU
#define ADUCM_UART_MSR_DCTS                 (1U << 0)
#define ADUCM_UART_MSR_DDSR                 (1U << 1)
#define ADUCM_UART_MSR_TERI                 (1U << 2)
#define ADUCM_UART_MSR_DDCD                 (1U << 3)
#define ADUCM_UART_MSR_CTS                  (1U << 4)
#define ADUCM_UART_MSR_DSR                  (1U << 5)
#define ADUCM_UART_MSR_RI                   (1U << 6)
#define ADUCM_UART_MSR_DCD                  (1U << 7)
/** @} */

/**
 * @name    UART Fractional Baud Rate Divider register bits definitions
 * @{
 */
#define ADUCM_UART_FBR_MASK                 0x9FFFU
#define ADUCM_UART_FBR_DIVN_MASK            0x07FFU
#define ADUCM_UART_FBR_DIVN(n)              ((n) << 0)
#define ADUCM_UART_FBR_DIVM_MASK            0x1800U
#define ADUCM_UART_FBR_DIVM(m)              ((m) << 11)
#define ADUCM_UART_FBR_ENABLE               (1U << 15)
/** @} */

/**
 * @name    UART Divider register bits definitions
 * @{
 */
#define ADUCM_UART_DIV_MASK                 0xFFFFU
/** @} */

/**
 * @name    UART Control register bits definitions
 * @{
 */
#define ADUCM_UART_CTL_MASK                 0x0001U
#define ADUCM_UART_CTL_DISABLE              (1U << 0)
#define ADUCM_UART_CTL_ENABLE               (0U << 0)
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

/**
 * @brief   ADUCM UARTv1 registers block.
 */
typedef struct {
  volatile uint32_t     DATA;
  volatile uint32_t     IEN;
  volatile uint32_t     IIR;
  volatile uint32_t     LCR;
  volatile uint32_t     MCR;
  volatile uint32_t     LSR;
  volatile uint32_t     MSR;
  volatile uint32_t     SCR;
  volatile uint32_t     FCR;
  volatile uint32_t     FBR;
  volatile uint32_t     DIV;
  volatile uint32_t     LCR2;
  volatile uint32_t     CTL;
  volatile uint32_t     RFC;
  volatile uint32_t     TFC;
  volatile uint32_t     RSC;
  volatile uint32_t     ACR;
  volatile uint32_t     ASRL;
  volatile uint32_t     ASRH;
} aducm_uart_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* ADUCM_UART_H */

/** @} */
