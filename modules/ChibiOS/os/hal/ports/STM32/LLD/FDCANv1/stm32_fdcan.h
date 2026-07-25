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
 * @file    FDCANv1/stm32_fdcan.h
 * @brief   STM32 FDCAN helpers header.
 *
 * @addtogroup STM32_
 * @{
 */

#ifndef STM32_FDCAN_H
#define STM32_FDCAN_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    NBTP register additional macros
 * @{
 */
#define FDCAN_CONFIG_NBTP_NTSEG2_Pos     (0)
#define FDCAN_CONFIG_NBTP_NTSEG2_Msk     (0x7Fu << FDCAN_CONFIG_NBTP_NTSEG2_Pos)
#define FDCAN_CONFIG_NBTP_NTSEG2(n)      ((FDCAN_CONFIG_NBTP_NTSEG2_Msk & ((n) << FDCAN_CONFIG_NBTP_NTSEG2_Pos)))
#define FDCAN_CONFIG_NBTP_NTSEG1_Pos     (8)
#define FDCAN_CONFIG_NBTP_NTSEG1_Msk     (0xFFu << FDCAN_CONFIG_NBTP_NTSEG1_Pos)
#define FDCAN_CONFIG_NBTP_NTSEG1(n)      ((FDCAN_CONFIG_NBTP_NTSEG1_Msk & ((n) << FDCAN_CONFIG_NBTP_NTSEG1_Pos)))
#define FDCAN_CONFIG_NBTP_NBRP_Pos       (16)
#define FDCAN_CONFIG_NBTP_NBRP_Msk       (0x1FFu << FDCAN_CONFIG_NBTP_NBRP_Pos)
#define FDCAN_CONFIG_NBTP_NBRP(n)        ((FDCAN_CONFIG_NBTP_NBRP_Msk & ((n) << FDCAN_CONFIG_NBTP_NBRP_Pos)))
#define FDCAN_CONFIG_NBTP_NSJW_Pos       (25)
#define FDCAN_CONFIG_NBTP_NSJW_Msk       (0x7Fu << FDCAN_CONFIG_NBTP_NSJW_Pos)
#define FDCAN_CONFIG_NBTP_NSJW(n)        ((FDCAN_CONFIG_NBTP_NSJW_Msk & ((n) << FDCAN_CONFIG_NBTP_NSJW_Pos)))
/** @} */

/**
 * @name    DBTP register additional macros
 * @{
 */
#define FDCAN_CONFIG_DBTP_DSJW_Pos       (0)
#define FDCAN_CONFIG_DBTP_DSJW_Msk       (0xFu << FDCAN_CONFIG_DBTP_DSJW_Pos)
#define FDCAN_CONFIG_DBTP_DSJW(n)        ((FDCAN_CONFIG_DBTP_DSJW_Msk & ((n) << FDCAN_CONFIG_DBTP_DSJW_Pos)))
#define FDCAN_CONFIG_DBTP_DTSEG2_Pos     (4)
#define FDCAN_CONFIG_DBTP_DTSEG2_Msk     (0xFu << FDCAN_CONFIG_DBTP_DTSEG2_Pos)
#define FDCAN_CONFIG_DBTP_DTSEG2(n)      ((FDCAN_CONFIG_DBTP_DTSEG2_Msk & ((n) << FDCAN_CONFIG_DBTP_DTSEG2_Pos)))
#define FDCAN_CONFIG_DBTP_DTSEG1_Pos     (8)
#define FDCAN_CONFIG_DBTP_DTSEG1_Msk     (0x1Fu << FDCAN_CONFIG_DBTP_DTSEG1_Pos)
#define FDCAN_CONFIG_DBTP_DTSEG1(n)      ((FDCAN_CONFIG_DBTP_DTSEG1_Msk & ((n) << FDCAN_CONFIG_DBTP_DTSEG1_Pos)))
#define FDCAN_CONFIG_DBTP_DBRP_Pos       (16)
#define FDCAN_CONFIG_DBTP_DBRP_Msk       (0x1Fu << FDCAN_CONFIG_DBTP_DBRP_Pos)
#define FDCAN_CONFIG_DBTP_DBRP(n)        ((FDCAN_CONFIG_DBTP_DBRP_Msk & ((n) << FDCAN_CONFIG_DBTP_DBRP_Pos)))
#define FDCAN_CONFIG_DBTP_TDC            (1u << 23)
/** @} */

/**
 * @name    CCCR register additional macros
 * @{
 */
#define FDCAN_CONFIG_CCCR_DAR            (1u << 6)
#define FDCAN_CONFIG_CCCR_TEST_MODE      (1u << 7)
#define FDCAN_CONFIG_CCCR_FDCAN_MODE     (1u << 8)
#define FDCAN_CONFIG_CCCR_BRSE           (1u << 9)
#define FDCAN_CONFIG_CCCR_NISO           (1u << 15)
/** @} */

/**
 * @name    TEST register additional macros
 * @{
 */
#define FDCAN_CONFIG_TEST_LBCK_MODE      (1u << 4)
#define FDCAN_CONFIG_TEST_TX_Pos         (5)
#define FDCAN_CONFIG_TEST_TX_Msk         (0x3u << FDCAN_CONFIG_TEST_TX_Pos)
#define FDCAN_CONFIG_TEST_TX(n)          ((FDCAN_CONFIG_TEST_TX_Msk & ((n) << FDCAN_CONFIG_TEST_TX_Pos)))
#define FDCAN_CONFIG_TEST_RX             (1u << 7)
/** @} */

/**
 * @name    RXGFC) register additional macros
 * @{
 */
#define FDCAN_CONFIG_RXGFC_RRFE            (1u << 0)
#define FDCAN_CONFIG_RXGFC_RRFS            (1u << 1)
#define FDCAN_CONFIG_RXGFC_ANFE_Pos        (2)
#define FDCAN_CONFIG_RXGFC_ANFE_Msk        (0x3u << FDCAN_CONFIG_RXGFC_ANFE_Pos)
#define FDCAN_CONFIG_RXGFC_ANFE(n)         ((FDCAN_CONFIG_RXGFC_ANFE_Msk & ((n) << FDCAN_CONFIG_RXGFC_ANFE_Pos)))
#define   FDCAN_CONFIG_RXGFC_ANFE_RX_0     (0x0u << 2)
#define   FDCAN_CONFIG_RXGFC_ANFE_RX_1     (0x1u << 2)
#define   FDCAN_CONFIG_RXGFC_ANFE_REJECT   (0x2u << 2)
#define FDCAN_CONFIG_RXGFC_ANFS_Pos        (4)
#define FDCAN_CONFIG_RXGFC_ANFS_Msk        (0x3u << FDCAN_CONFIG_RXGFC_ANFS_Pos)
#define FDCAN_CONFIG_RXGFC_ANFS(n)         ((FDCAN_CONFIG_RXGFC_ANFS_Msk & ((n) << FDCAN_CONFIG_RXGFC_ANFS_Pos)))
#define   FDCAN_CONFIG_RXGFC_ANFS_RX_0     (0x0u << 4)
#define   FDCAN_CONFIG_RXGFC_ANFS_RX_1     (0x1u << 4)
#define   FDCAN_CONFIG_RXGFC_ANFS_REJECT   (0x2u << 4)
#define FDCAN_CONFIG_RXGFC_F1OM            (1u << 8)
#define FDCAN_CONFIG_RXGFC_F0OM            (1u << 9)
#define FDCAN_CONFIG_RXGFC_LSS_Pos         (16)
#define FDCAN_CONFIG_RXGFC_LSS_Msk         (0x1Fu << FDCAN_CONFIG_RXGFC_LSS_Pos)
#define FDCAN_CONFIG_RXGFC_LSS(n)          ((FDCAN_CONFIG_RXGFC_LSS_Msk & ((n) << FDCAN_CONFIG_RXGFC_LSS_Pos)))
#define FDCAN_CONFIG_RXGFC_LSE_Pos         (24)
#define FDCAN_CONFIG_RXGFC_LSE_Msk         (0xFu << FDCAN_CONFIG_RXGFC_LSE_Pos)
#define FDCAN_CONFIG_RXGFC_LSE(n)          ((FDCAN_CONFIG_RXGFC_LSE_Msk & ((n) << FDCAN_CONFIG_RXGFC_LSE_Pos)))
/** @} */

/**
 * @name    TXBC register additional macros
 * @{
 */
#define FDCAN_CONFIG_TXBC_TFQM            (1u << 24)
/** @} */

/**
 * @name    TXBC register additional macros
 * @{
 */
#define FDCAN_CONFIG_CKDIV_PDIV_Pos        (0)
#define FDCAN_CONFIG_CKDIV_PDIV_Msk        (0xFu << FDCAN_CONFIG_CKDIV_PDIV_Pos)
#define FDCAN_CONFIG_CKDIV_PDIV(n)         ((FDCAN_CONFIG_CKDIV_PDIV_Msk & ((n) << FDCAN_CONFIG_CKDIV_PDIV_Pos)))
#define   FDCAN_CONFIG_CKDIV_PDIV_1        (0x0u << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_2        (0x1u << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_4        (0x2u << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_6        (0x3u << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_8        (0x4u << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_10       (0x5u << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_12       (0x6u << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_14       (0x7u << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_16       (0x8u << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_18       (0x9u << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_20       (0xAu << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_22       (0xBu << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_24       (0xCu << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_26       (0xDu << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_28       (0xEu << 0)
#define   FDCAN_CONFIG_CKDIV_PDIV_30       (0xFu << 0)
/** @} */

/**
 * @name    FDCAN standard message ID filter element help macros.
 * @{
 */
#define FDCAN_STD_FILTER_SFID2_Pos         (0)
#define FDCAN_STD_FILTER_SFID2_Msk         (0x7FFu << FDCAN_STD_FILTER_SFID2_Pos)
#define FDCAN_STD_FILTER_SFID2(n)          ((FDCAN_STD_FILTER_SFID2_Msk & ((n) << FDCAN_STD_FILTER_SFID2_Pos)))
#define FDCAN_STD_FILTER_SFID1_Pos         (16)
#define FDCAN_STD_FILTER_SFID1_Msk         (0x7FFu << FDCAN_STD_FILTER_SFID1_Pos)
#define FDCAN_STD_FILTER_SFID1(n)          ((FDCAN_STD_FILTER_SFID1_Msk & ((n) << FDCAN_STD_FILTER_SFID1_Pos)))
#define FDCAN_STD_FILTER_SFEC_Pos          (27)
#define FDCAN_STD_FILTER_SFEC_Msk          (0x7u << FDCAN_STD_FILTER_SFEC_Pos)
#define FDCAN_STD_FILTER_SFEC(n)           ((FDCAN_STD_FILTER_SFEC_Msk & ((n) << FDCAN_STD_FILTER_SFEC_Pos)))
#define FDCAN_STD_FILTER_SFT_Pos           (30)
#define FDCAN_STD_FILTER_SFT_Msk           (0x3u << FDCAN_STD_FILTER_SFT_Pos)
#define FDCAN_STD_FILTER_SFT(n)            ((FDCAN_STD_FILTER_SFT_Msk & ((n) << FDCAN_STD_FILTER_SFT_Pos)))
/** @} */

/**
 * @name    FDCAN Extended message ID filter element help macros.
 * @{
 */
#define FDCAN_EXT_FILTER_EFID1_Pos         (0)
#define FDCAN_EXT_FILTER_EFID1_Msk         (0x1FFFFFFFu << FDCAN_EXT_FILTER_EFID1_Pos)
#define FDCAN_EXT_FILTER_EFID1(n)          ((FDCAN_EXT_FILTER_EFID1_Msk & ((n) << FDCAN_EXT_FILTER_EFID1_Pos)))
#define FDCAN_EXT_FILTER_EFEC_Pos          (29)
#define FDCAN_EXT_FILTER_EFEC_Msk          (0x7u << FDCAN_EXT_FILTER_EFEC_Pos)
#define FDCAN_EXT_FILTER_EFEC(n)           ((FDCAN_EXT_FILTER_EFEC_Msk & ((n) << FDCAN_EXT_FILTER_EFEC_Pos)))
#define FDCAN_EXT_FILTER_EFID2_Pos         (0)
#define FDCAN_EXT_FILTER_EFID2_Msk         (0x1FFFFFFFu << FDCAN_EXT_FILTER_EFID2_Pos)
#define FDCAN_EXT_FILTER_EFID2(n)          ((FDCAN_EXT_FILTER_EFID2_Msk & ((n) << FDCAN_EXT_FILTER_EFID2_Pos)))
#define FDCAN_EXT_FILTER_EFT_Pos           (30)
#define FDCAN_EXT_FILTER_EFT_Msk           (0x2u << FDCAN_EXT_FILTER_EFT_Pos)
#define FDCAN_EXT_FILTER_EFT(n)            ((FDCAN_EXT_FILTER_EFT_Msk & ((n) << FDCAN_EXT_FILTER_EFT_Pos)))
/** @} */

/**
 * @name    FDCAN transmitter delay compensation register.
 * @{
 */
#define FDCAN_CONFIG_TDCR_TDCF_Pos       (0)
#define FDCAN_CONFIG_TDCR_TDCF_Msk       (0x7Fu << FDCAN_CONFIG_TDCR_TDCF_Pos)
#define FDCAN_CONFIG_TDCR_TDCF(n)        ((FDCAN_CONFIG_TDCR_TDCF_Msk & ((n) << FDCAN_CONFIG_TDCR_TDCF_Pos)))
#define FDCAN_CONFIG_TDCR_TDCO_Pos       (8)
#define FDCAN_CONFIG_TDCR_TDCO_Msk       (0x7Fu << FDCAN_CONFIG_TDCR_TDCO_Pos)
#define FDCAN_CONFIG_TDCR_TDCO(n)        ((FDCAN_CONFIG_TDCR_TDCO_Msk & ((n) << FDCAN_CONFIG_TDCR_TDCO_Pos)))
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

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32_FDCAN_H */

/** @} */
