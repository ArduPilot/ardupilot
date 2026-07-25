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
 * @file    FDCANv2/stm32_fdcan.h
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
#define FDCAN_CONFIG_CCCR_CAN_MODE       (0u << 8)
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
 * @name    GFC register additional macros
 * @{
 */
#define FDCAN_CONFIG_GFC_RRFE            (1u << 0)
#define FDCAN_CONFIG_GFC_RRFS            (1u << 1)
#define FDCAN_CONFIG_GFC_ANFE_Pos        (2)
#define FDCAN_CONFIG_GFC_ANFE_Msk        (0x3u << FDCAN_CONFIG_GFC_ANFE_Pos)
#define FDCAN_CONFIG_GFC_ANFE(n)         ((FDCAN_CONFIG_GFC_ANFE_Msk & ((n) << FDCAN_CONFIG_GFC_ANFE_Pos)))
#define   FDCAN_CONFIG_GFC_ANFE_RX_0     (0x0u << 2)
#define   FDCAN_CONFIG_GFC_ANFE_RX_1     (0x1u << 2)
#define   FDCAN_CONFIG_GFC_ANFE_REJECT   (0x2u << 2)
#define FDCAN_CONFIG_GFC_ANFS_Pos        (4)
#define FDCAN_CONFIG_GFC_ANFS_Msk        (0x3u << FDCAN_CONFIG_GFC_ANFS_Pos)
#define FDCAN_CONFIG_GFC_ANFS(n)         ((FDCAN_CONFIG_GFC_ANFS_Msk & ((n) << FDCAN_CONFIG_GFC_ANFS_Pos)))
#define   FDCAN_CONFIG_GFC_ANFS_RX_0     (0x0u << 4)
#define   FDCAN_CONFIG_GFC_ANFS_RX_1     (0x1u << 4)
#define   FDCAN_CONFIG_GFC_ANFS_REJECT   (0x2u << 4)
/** @} */

/**
 * @name    SIDFC register additional macros
 * @{
 */
#define FDCAN_CONFIG_SIDFC_FLSSA_Pos     (2)
#define FDCAN_CONFIG_SIDFC_FLSSA_Msk     (0x3FFFu << FDCAN_CONFIG_SIDFC_FLSSA_Pos)
#define FDCAN_CONFIG_SIDFC_FLSSA(n)      ((FDCAN_CONFIG_SIDFC_FLSSA_Msk & ((n) << FDCAN_CONFIG_SIDFC_FLSSA_Pos)))
#define FDCAN_CONFIG_SIDFC_LSS_Pos       (16)
#define FDCAN_CONFIG_SIDFC_LSS_Msk       (0xFFu << FDCAN_CONFIG_SIDFC_LSS_Pos)
#define FDCAN_CONFIG_SIDFC_LSS(n)        ((FDCAN_CONFIG_SIDFC_LSS_Msk & ((n) << FDCAN_CONFIG_SIDFC_LSS_Pos)))
/** @} */

/**
 * @name    XIDFC register additional macros
 * @{
 */
#define FDCAN_CONFIG_XIDFC_FLESA_Pos     (2)
#define FDCAN_CONFIG_XIDFC_FLESA_Msk     (0x3FFFu << FDCAN_CONFIG_XIDFC_FLESA_Pos)
#define FDCAN_CONFIG_XIDFC_FLESA(n)      ((FDCAN_CONFIG_XIDFC_FLESA_Msk & ((n) << FDCAN_CONFIG_XIDFC_FLESA_Pos)))
#define FDCAN_CONFIG_XIDFC_LSE_Pos       (16)
#define FDCAN_CONFIG_XIDFC_LSE_Msk       (0x7Fu << FDCAN_CONFIG_XIDFC_LSE_Pos)
#define FDCAN_CONFIG_XIDFC_LSE(n)        ((FDCAN_CONFIG_XIDFC_LSE_Msk & ((n) << FDCAN_CONFIG_XIDFC_LSE_Pos)))
/** @} */

/**
 * @name    RXF0C register additional macros
 * @{
 */
#define FDCAN_CONFIG_RXF0C_F0SA_Pos      (2)
#define FDCAN_CONFIG_RXF0C_F0SA_Msk      (0x3FFFu << FDCAN_CONFIG_RXF0C_F0SA_Pos)
#define FDCAN_CONFIG_RXF0C_F0SA(n)       ((FDCAN_CONFIG_RXF0C_F0SA_Msk & ((n) << FDCAN_CONFIG_RXF0C_F0SA_Pos)))
#define FDCAN_CONFIG_RXF0C_F0S_Pos       (16)
#define FDCAN_CONFIG_RXF0C_F0S_Msk       (0x7Fu << FDCAN_CONFIG_RXF0C_F0S_Pos)
#define FDCAN_CONFIG_RXF0C_F0S(n)        ((FDCAN_CONFIG_RXF0C_F0S_Msk & ((n) << FDCAN_CONFIG_RXF0C_F0S_Pos)))
#define FDCAN_CONFIG_RXF0C_F0WM_Pos      (24)
#define FDCAN_CONFIG_RXF0C_F0WM_Msk      (0x7Fu << FDCAN_CONFIG_RXF0C_F0WM_Pos)
#define FDCAN_CONFIG_RXF0C_F0WM(n)       ((FDCAN_CONFIG_RXF0C_F0WM_Msk & ((n) << FDCAN_CONFIG_RXF0C_F0WM_Pos)))
/** @} */

/**
 * @name    RXF1C register additional macros
 * @{
 */
#define FDCAN_CONFIG_RXF1C_F1SA_Pos      (2)
#define FDCAN_CONFIG_RXF1C_F1SA_Msk      (0x3FFFu << FDCAN_CONFIG_RXF1C_F1SA_Pos)
#define FDCAN_CONFIG_RXF1C_F1SA(n)       ((FDCAN_CONFIG_RXF1C_F1SA_Msk & ((n) << FDCAN_CONFIG_RXF1C_F1SA_Pos)))
#define FDCAN_CONFIG_RXF1C_F1S_Pos       (16)
#define FDCAN_CONFIG_RXF1C_F1S_Msk       (0x7Fu << FDCAN_CONFIG_RXF1C_F1S_Pos)
#define FDCAN_CONFIG_RXF1C_F1S(n)        ((FDCAN_CONFIG_RXF1C_F1S_Msk & ((n) << FDCAN_CONFIG_RXF1C_F1S_Pos)))
#define FDCAN_CONFIG_RXF1C_F1WM_Pos      (24)
#define FDCAN_CONFIG_RXF1C_F1WM_Msk      (0x7Fu << FDCAN_CONFIG_RXF1C_F1WM_Pos)
#define FDCAN_CONFIG_RXF1C_F1WM(n)       ((FDCAN_CONFIG_RXF1C_F1WM_Msk & ((n) << FDCAN_CONFIG_RXF1C_F1WM_Pos)))
/** @} */

/**
 * @name    RXBC register additional macros
 * @{
 */
#define FDCAN_CONFIG_RXBC_RBSA_Pos       (2)
#define FDCAN_CONFIG_RXBC_RBSA_Msk       (0x3FFFu << FDCAN_CONFIG_RXBC_RBSA_Pos)
#define FDCAN_CONFIG_RXBC_RBSA(n)        ((FDCAN_CONFIG_RXBC_RBSA_Msk & ((n) << FDCAN_CONFIG_RXBC_RBSA_Pos)))
/** @} */

/**
 * @name    TXEFC register additional macros
 * @{
 */
#define FDCAN_CONFIG_TXEFC_EFSA_Pos       (2)
#define FDCAN_CONFIG_TXEFC_EFSA_Msk       (0x3FFFu << FDCAN_CONFIG_TXEFC_EFSA_Pos)
#define FDCAN_CONFIG_TXEFC_EFSA(n)        ((FDCAN_CONFIG_TXEFC_EFSA_Msk & ((n) << FDCAN_CONFIG_TXEFC_EFSA_Pos)))
#define FDCAN_CONFIG_TXEFC_EFS_Pos        (16)
#define FDCAN_CONFIG_TXEFC_EFS_Msk        (0x3Fu << FDCAN_CONFIG_TXEFC_EFS_Pos)
#define FDCAN_CONFIG_TXEFC_EFS(n)         ((FDCAN_CONFIG_TXEFC_EFS_Msk & ((n) << FDCAN_CONFIG_TXEFC_EFS_Pos)))
/** @} */

/**
 * @name    TXBC register additional macros
 * @{
 */
#define FDCAN_CONFIG_TXBC_TBSA_Pos        (2)
#define FDCAN_CONFIG_TXBC_TBSA_Msk        (0x3FFFu << FDCAN_CONFIG_TXBC_TBSA_Pos)
#define FDCAN_CONFIG_TXBC_TBSA(n)         ((FDCAN_CONFIG_TXBC_TBSA_Msk & ((n) << FDCAN_CONFIG_TXBC_TBSA_Pos)))
#define FDCAN_CONFIG_TXBC_TFQS_Pos        (24)
#define FDCAN_CONFIG_TXBC_TFQS_Msk        (0x3Fu << FDCAN_CONFIG_TXBC_TFQS_Pos)
#define FDCAN_CONFIG_TXBC_TFQS(n)         ((FDCAN_CONFIG_TXBC_TFQS_Msk & ((n) << FDCAN_CONFIG_TXBC_TFQS_Pos)))

/**
 * @name    TXESC register additional macros
 * @{
 */
#define FDCAN_CONFIG_TXESC_TBDS_Pos        (0)
#define FDCAN_CONFIG_TXESC_TBDS_Msk        (0x7u << FDCAN_CONFIG_TXESC_TBDS_Pos)
#define FDCAN_CONFIG_TXESC_TBDS(n)         ((FDCAN_CONFIG_TXESC_TBDS_Msk & ((n) << FDCAN_CONFIG_TXESC_TBDS_Pos)))
#define   FDCAN_CONFIG_TXESC_TBDS_8BDF     (0x0u << 0)
#define   FDCAN_CONFIG_TXESC_TBDS_64BDF    (0x7u << 0)
/** @} */

/**
 * @name    RXESC register additional macros
 * @{
 */
#define FDCAN_CONFIG_RXESC_F0DS_Pos        (0)
#define FDCAN_CONFIG_RXESC_F0DS_Msk        (0x7u << FDCAN_CONFIG_RXESC_F0DS_Pos)
#define FDCAN_CONFIG_RXESC_F0DS(n)         ((FDCAN_CONFIG_RXESC_F0DS_Msk & ((n) << FDCAN_CONFIG_RXESC_F0DS_Pos)))
#define   FDCAN_CONFIG_RXESC_F0DS_8BDF     (0x0u << 0)
#define   FDCAN_CONFIG_RXESC_F0DS_64BDF    (0x7u << 0)
#define FDCAN_CONFIG_RXESC_F1DS_Pos        (4)
#define FDCAN_CONFIG_RXESC_F1DS_Msk        (0x7u << FDCAN_CONFIG_RXESC_F1DS_Pos)
#define FDCAN_CONFIG_RXESC_F1DS(n)         ((FDCAN_CONFIG_RXESC_F1DS_Msk & ((n) << FDCAN_CONFIG_RXESC_F1DS_Pos)))
#define   FDCAN_CONFIG_RXESC_F1DS_8BDF     (0x0u << 4)
#define   FDCAN_CONFIG_RXESC_F1DS_64BDF    (0x7u << 4)
#define FDCAN_CONFIG_RXESC_RBDS_Pos        (8)
#define FDCAN_CONFIG_RXESC_RBDS_Msk        (0x7u << FDCAN_CONFIG_RXESC_RBDS_Pos)
#define FDCAN_CONFIG_RXESC_RBDS(n)         ((FDCAN_CONFIG_RXESC_RBDS_Msk & ((n) << FDCAN_CONFIG_RXESC_RBDS_Pos)))
#define   FDCAN_CONFIG_RXESC_RBDS_8BDF     (0x0u << 8)
#define   FDCAN_CONFIG_RXESC_RBDS_64BDF    (0x7u << 8)
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
