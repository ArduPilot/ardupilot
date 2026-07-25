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
 * @file    hal_mmcsd.c
 * @brief   MMC/SD cards common code.
 *
 * @addtogroup MMCSD
 * @{
 */

#include "hal.h"

#if (HAL_USE_MMC_SPI == TRUE) || (HAL_USE_SDC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Gets a bit field from a words array.
 * @note    The bit zero is the LSb of the first word.
 *
 * @param[in] data      pointer to the words array
 * @param[in] end       bit offset of the last bit of the field, inclusive
 * @param[in] start     bit offset of the first bit of the field, inclusive
 *
 * @return              The bits field value, left aligned.
 *
 * @notapi
 */
uint32_t _mmcsd_get_slice(const uint32_t *data,
                          uint32_t end,
                          uint32_t start) {
  unsigned startidx, endidx, startoff;
  uint32_t endmask;

  osalDbgCheck((end >= start) && ((end - start) < 32U));

  startidx = start / 32U;
  startoff = start % 32U;
  endidx   = end / 32U;
  endmask  = ((uint32_t)1U << ((end % 32U) + 1U)) - 1U;

  /* One or two pieces?*/
  if (startidx < endidx) {
    return (data[startidx] >> startoff) |               /* Two pieces case. */
           ((data[endidx] & endmask) << (32U - startoff));
  }
  return (data[startidx] & endmask) >> startoff;        /* One piece case.  */
}

/**
 * @brief   Extract card capacity from a CSD.
 * @details The capacity is returned as number of available blocks.
 *
 * @param[in] csd       the CSD record
 *
 * @return              The card capacity.
 * @retval 0            CSD format error
 *
 * @notapi
 */
uint32_t _mmcsd_get_capacity(const uint32_t *csd) {
  uint32_t a, b, c;

  osalDbgCheck(NULL != csd);

  switch (_mmcsd_get_slice(csd, MMCSD_CSD_10_CSD_STRUCTURE_SLICE)) {
  case 0:
    /* CSD version 1.0 */
    a = _mmcsd_get_slice(csd, MMCSD_CSD_10_C_SIZE_SLICE);
    b = _mmcsd_get_slice(csd, MMCSD_CSD_10_C_SIZE_MULT_SLICE);
    c = _mmcsd_get_slice(csd, MMCSD_CSD_10_READ_BL_LEN_SLICE);
    return ((a + 1U) << (b + 2U)) << (c - 9U);  /* 2^9 == MMCSD_BLOCK_SIZE. */
  case 1:
    /* CSD version 2.0.*/
    return 1024U * (_mmcsd_get_slice(csd, MMCSD_CSD_20_C_SIZE_SLICE) + 1U);
  default:
    /* Reserved value detected.*/
    break;
  }
  return 0U;
}

/**
 * @brief   Extract MMC card capacity from EXT_CSD.
 * @details The capacity is returned as number of available blocks.
 *
 * @param[in] ext_csd   the extended CSD record
 *
 * @return              The card capacity.
 *
 * @notapi
 */
uint32_t _mmcsd_get_capacity_ext(const uint8_t *ext_csd) {

  osalDbgCheck(NULL != ext_csd);

  return ((uint32_t)ext_csd[215] << 24U) +
         ((uint32_t)ext_csd[214] << 16U) +
         ((uint32_t)ext_csd[213] << 8U)  +
         (uint32_t)ext_csd[212];
}

/**
 * @brief   Unpacks SDC CID array in structure.
 *
 * @param[in] sdcp      pointer to the @p MMCSDBlockDevice object
 * @param[out] cidsdc   pointer to the @p unpacked_sdc_cid_t object
 *
 * @notapi
 */
void _mmcsd_unpack_sdc_cid(const MMCSDBlockDevice *sdcp,
                           unpacked_sdc_cid_t *cidsdc) {
  const uint32_t *cid;

  osalDbgCheck((NULL != sdcp) && (NULL != cidsdc));

  cid = sdcp->cid;
  cidsdc->crc    = (uint8_t) _mmcsd_get_slice(cid, MMCSD_CID_SDC_CRC_SLICE);
  cidsdc->mdt_y  = (uint16_t)_mmcsd_get_slice(cid, MMCSD_CID_SDC_MDT_Y_SLICE) +
                             2000U;
  cidsdc->mdt_m  = (uint8_t) _mmcsd_get_slice(cid, MMCSD_CID_SDC_MDT_M_SLICE);
  cidsdc->mid    = (uint8_t) _mmcsd_get_slice(cid, MMCSD_CID_SDC_MID_SLICE);
  cidsdc->oid    = (uint16_t)_mmcsd_get_slice(cid, MMCSD_CID_SDC_OID_SLICE);
  cidsdc->pnm[4] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_SDC_PNM0_SLICE);
  cidsdc->pnm[3] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_SDC_PNM1_SLICE);
  cidsdc->pnm[2] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_SDC_PNM2_SLICE);
  cidsdc->pnm[1] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_SDC_PNM3_SLICE);
  cidsdc->pnm[0] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_SDC_PNM4_SLICE);
  cidsdc->prv_n  = (uint8_t) _mmcsd_get_slice(cid, MMCSD_CID_SDC_PRV_N_SLICE);
  cidsdc->prv_m  = (uint8_t) _mmcsd_get_slice(cid, MMCSD_CID_SDC_PRV_M_SLICE);
  cidsdc->psn    =           _mmcsd_get_slice(cid, MMCSD_CID_SDC_PSN_SLICE);
}

/**
 * @brief   Unpacks MMC CID array in structure.
 *
 * @param[in] sdcp      pointer to the @p MMCSDBlockDevice object
 * @param[out] cidmmc   pointer to the @p unpacked_mmc_cid_t object
 *
 * @notapi
 */
void _mmcsd_unpack_mmc_cid(const MMCSDBlockDevice *sdcp,
                           unpacked_mmc_cid_t *cidmmc) {
  const uint32_t *cid;

  osalDbgCheck((NULL != sdcp) && (NULL != cidmmc));

  cid = sdcp->cid;
  cidmmc->crc    = (uint8_t) _mmcsd_get_slice(cid, MMCSD_CID_MMC_CRC_SLICE);
  cidmmc->mdt_y  = (uint16_t)_mmcsd_get_slice(cid, MMCSD_CID_MMC_MDT_Y_SLICE) +
                             1997U;
  cidmmc->mdt_m  = (uint8_t) _mmcsd_get_slice(cid, MMCSD_CID_MMC_MDT_M_SLICE);
  cidmmc->mid    = (uint8_t) _mmcsd_get_slice(cid, MMCSD_CID_MMC_MID_SLICE);
  cidmmc->oid    = (uint16_t)_mmcsd_get_slice(cid, MMCSD_CID_MMC_OID_SLICE);
  cidmmc->pnm[5] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_MMC_PNM0_SLICE);
  cidmmc->pnm[4] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_MMC_PNM1_SLICE);
  cidmmc->pnm[3] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_MMC_PNM2_SLICE);
  cidmmc->pnm[2] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_MMC_PNM3_SLICE);
  cidmmc->pnm[1] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_MMC_PNM4_SLICE);
  cidmmc->pnm[0] = (char)    _mmcsd_get_slice(cid, MMCSD_CID_MMC_PNM5_SLICE);
  cidmmc->prv_n  = (uint8_t) _mmcsd_get_slice(cid, MMCSD_CID_MMC_PRV_N_SLICE);
  cidmmc->prv_m  = (uint8_t) _mmcsd_get_slice(cid, MMCSD_CID_MMC_PRV_M_SLICE);
  cidmmc->psn    =           _mmcsd_get_slice(cid, MMCSD_CID_MMC_PSN_SLICE);
}

/**
 * @brief   Unpacks MMC CSD array in structure.
 *
 * @param[in] sdcp      pointer to the @p MMCSDBlockDevice object
 * @param[out] csdmmc   pointer to the @p unpacked_mmc_csd_t object
 *
 * @notapi
 */
void _mmcsd_unpack_csd_mmc(const MMCSDBlockDevice *sdcp,
                           unpacked_mmc_csd_t *csdmmc) {
  const uint32_t *csd;

  osalDbgCheck((NULL != sdcp) && (NULL != csdmmc));

  csd = sdcp->csd;
  csdmmc->c_size             = (uint16_t)_mmcsd_get_slice(csd, MMCSD_CSD_MMC_C_SIZE_SLICE);
  csdmmc->c_size_mult        = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_C_SIZE_MULT_SLICE);
  csdmmc->ccc                = (uint16_t)_mmcsd_get_slice(csd, MMCSD_CSD_MMC_CCC_SLICE);
  csdmmc->copy               = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_COPY_SLICE);
  csdmmc->crc                = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_CRC_SLICE);
  csdmmc->csd_structure      = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_CSD_STRUCTURE_SLICE);
  csdmmc->dsr_imp            = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_DSR_IMP_SLICE);
  csdmmc->ecc                = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_ECC_SLICE);
  csdmmc->erase_grp_mult     = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_ERASE_GRP_MULT_SLICE);
  csdmmc->erase_grp_size     = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_ERASE_GRP_SIZE_SLICE);
  csdmmc->file_format        = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_FILE_FORMAT_SLICE);
  csdmmc->file_format_grp    = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_FILE_FORMAT_GRP_SLICE);
  csdmmc->nsac               = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_NSAC_SLICE);
  csdmmc->perm_write_protect = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_PERM_WRITE_PROTECT_SLICE);
  csdmmc->r2w_factor         = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_R2W_FACTOR_SLICE);
  csdmmc->read_bl_len        = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_READ_BL_LEN_SLICE);
  csdmmc->read_bl_partial    = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_READ_BL_PARTIAL_SLICE);
  csdmmc->read_blk_misalign  = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_READ_BLK_MISALIGN_SLICE);
  csdmmc->spec_vers          = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_SPEC_VERS_SLICE);
  csdmmc->taac               = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_TAAC_SLICE);
  csdmmc->tmp_write_protect  = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_TMP_WRITE_PROTECT_SLICE);
  csdmmc->tran_speed         = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_TRAN_SPEED_SLICE);
  csdmmc->vdd_r_curr_max     = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_VDD_R_CURR_MAX_SLICE);
  csdmmc->vdd_r_curr_min     = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_VDD_R_CURR_MIN_SLICE);
  csdmmc->vdd_w_curr_max     = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_VDD_W_CURR_MAX_SLICE);
  csdmmc->vdd_w_curr_min     = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_VDD_W_CURR_MIN_SLICE);
  csdmmc->wp_grp_enable      = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_WP_GRP_ENABLE_SLICE);
  csdmmc->wp_grp_size        = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_WP_GRP_SIZE_SLICE);
  csdmmc->write_bl_len       = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_WRITE_BL_LEN_SLICE);
  csdmmc->write_bl_partial   = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_WRITE_BL_PARTIAL_SLICE);
  csdmmc->write_blk_misalign = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_MMC_WRITE_BLK_MISALIGN_SLICE);
}

/**
 * @brief   Unpacks SDC CSD v1.0 array in structure.
 *
 * @param[in] sdcp      pointer to the @p MMCSDBlockDevice object
 * @param[out] csd10    pointer to the @p unpacked_sdc_csd_10_t object
 *
 * @notapi
 */
void _mmcsd_unpack_csd_v10(const MMCSDBlockDevice *sdcp,
                           unpacked_sdc_csd_10_t *csd10) {
  const uint32_t *csd;

  osalDbgCheck(NULL != sdcp);

  csd = sdcp->csd;
  csd10->c_size              = (uint16_t)_mmcsd_get_slice(csd, MMCSD_CSD_10_C_SIZE_SLICE);
  csd10->c_size_mult         = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_C_SIZE_MULT_SLICE);
  csd10->ccc                 = (uint16_t)_mmcsd_get_slice(csd, MMCSD_CSD_10_CCC_SLICE);
  csd10->copy                = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_COPY_SLICE);
  csd10->crc                 = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_CRC_SLICE);
  csd10->csd_structure       = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_CSD_STRUCTURE_SLICE);
  csd10->dsr_imp             = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_DSR_IMP_SLICE);
  csd10->erase_blk_en        = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_ERASE_BLK_EN_SLICE);
  csd10->erase_sector_size   = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_ERASE_SECTOR_SIZE_SLICE);
  csd10->file_format         = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_FILE_FORMAT_SLICE);
  csd10->file_format_grp     = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_FILE_FORMAT_GRP_SLICE);
  csd10->nsac                = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_NSAC_SLICE);
  csd10->perm_write_protect  = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_PERM_WRITE_PROTECT_SLICE);
  csd10->r2w_factor          = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_R2W_FACTOR_SLICE);
  csd10->read_bl_len         = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_READ_BL_LEN_SLICE);
  csd10->read_bl_partial     = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_READ_BL_PARTIAL_SLICE);
  csd10->read_blk_misalign   = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_READ_BLK_MISALIGN_SLICE);
  csd10->taac                = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_TAAC_SLICE);
  csd10->tmp_write_protect   = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_TMP_WRITE_PROTECT_SLICE);
  csd10->tran_speed          = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_TRANS_SPEED_SLICE);
  csd10->wp_grp_enable       = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_WP_GRP_ENABLE_SLICE);
  csd10->wp_grp_size         = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_WP_GRP_SIZE_SLICE);
  csd10->write_bl_len        = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_WRITE_BL_LEN_SLICE);
  csd10->write_bl_partial    = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_WRITE_BL_PARTIAL_SLICE);
  csd10->write_blk_misalign  = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_10_WRITE_BLK_MISALIGN_SLICE);
}

/**
 * @brief   Unpacks SDC CSD v2.0 array in structure.
 *
 * @param[in] sdcp      pointer to the @p MMCSDBlockDevice object
 * @param[out] csd20    pointer to the @p unpacked_sdc_csd_20_t object
 *
 * @notapi
 */
void _mmcsd_unpack_csd_v20(const MMCSDBlockDevice *sdcp,
                           unpacked_sdc_csd_20_t *csd20) {
  const uint32_t *csd;

  osalDbgCheck(NULL != sdcp);

  csd = sdcp->csd;
  csd20->c_size              =           _mmcsd_get_slice(csd, MMCSD_CSD_20_C_SIZE_SLICE);
  csd20->crc                 = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_CRC_SLICE);
  csd20->ccc                 = (uint16_t)_mmcsd_get_slice(csd, MMCSD_CSD_20_CCC_SLICE);
  csd20->copy                = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_COPY_SLICE);
  csd20->csd_structure       = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_CSD_STRUCTURE_SLICE);
  csd20->dsr_imp             = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_DSR_IMP_SLICE);
  csd20->erase_blk_en        = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_ERASE_BLK_EN_SLICE);
  csd20->file_format         = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_FILE_FORMAT_SLICE);
  csd20->file_format_grp     = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_FILE_FORMAT_GRP_SLICE);
  csd20->nsac                = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_NSAC_SLICE);
  csd20->perm_write_protect  = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_PERM_WRITE_PROTECT_SLICE);
  csd20->r2w_factor          = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_R2W_FACTOR_SLICE);
  csd20->read_bl_len         = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_READ_BL_LEN_SLICE);
  csd20->read_bl_partial     = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_READ_BL_PARTIAL_SLICE);
  csd20->read_blk_misalign   = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_READ_BLK_MISALIGN_SLICE);
  csd20->erase_sector_size   = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_ERASE_SECTOR_SIZE_SLICE);
  csd20->taac                = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_TAAC_SLICE);
  csd20->tmp_write_protect   = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_TMP_WRITE_PROTECT_SLICE);
  csd20->tran_speed          = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_TRANS_SPEED_SLICE);
  csd20->wp_grp_enable       = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_WP_GRP_ENABLE_SLICE);
  csd20->wp_grp_size         = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_WP_GRP_SIZE_SLICE);
  csd20->write_bl_len        = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_WRITE_BL_LEN_SLICE);
  csd20->write_bl_partial    = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_WRITE_BL_PARTIAL_SLICE);
  csd20->write_blk_misalign  = (uint8_t) _mmcsd_get_slice(csd, MMCSD_CSD_20_WRITE_BLK_MISALIGN_SLICE);
}

#endif /* (HAL_USE_MMC_SPI == TRUE) || (HAL_USE_SDC == TRUE) */

/** @} */
