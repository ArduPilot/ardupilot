/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    hal_mfs.c
 * @brief   Managed Flash Storage module code.
 * @details This module manages a flash partition as a generic storage where
 *          arbitrary data records can be created, updated, deleted and
 *          retrieved.<br>
 *          A managed partition is composed of two banks of equal size, a
 *          bank is composed of one or more erasable sectors, a sector is
 *          divided in writable pages.<br>
 *          The module handles flash wear leveling and recovery of damaged
 *          banks (where possible) caused by power loss during operations.
 *          Both operations are transparent to the user.
 *
 * @addtogroup HAL_MFS
 * @{
 */

#include <string.h>

#include "hal.h"

#include "hal_mfs.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Data record size aligned.
 */
#define ALIGNED_REC_SIZE(n)                                                 \
  (flash_offset_t)MFS_ALIGN_NEXT(sizeof (mfs_data_header_t) + (size_t)(n))

/**
 * @brief   Data record header size aligned.
 */
#define ALIGNED_DHDR_SIZE                                                   \
  ALIGNED_REC_SIZE(0)

/**
 * @brief   Aligned size of a type.
 */
#define ALIGNED_SIZEOF(t)                                                   \
  (((sizeof (t) - 1U) | MFS_ALIGN_MASK) + 1U)

/**
 * @brief   Combines two values (0..3) in one (0..15).
 */
#define PAIR(a, b) (((unsigned)(a) << 2U) | (unsigned)(b))

/**
 * @brief   Error check helper.
 */
#define RET_ON_ERROR(err) do {                                              \
  mfs_error_t e = (err);                                                    \
  if (e != MFS_NO_ERROR) {                                                  \
    return e;                                                               \
  }                                                                         \
} while (false)

#if (MFS_USE_FLASH_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
static void mfs_flash_acquire(MFSDriver *mfsp) {
  flash_error_t ferr;

  ferr = flashAcquireExclusive(mfsp->config->flashp);
  osalDbgAssert(ferr == FLASH_NO_ERROR, "flash exclusive access failed");
}

static void mfs_flash_release(MFSDriver *mfsp) {
  flash_error_t ferr;

  ferr = flashReleaseExclusive(mfsp->config->flashp);
  osalDbgAssert(ferr == FLASH_NO_ERROR, "flash exclusive release failed");
}

#else
#define mfs_flash_acquire(mfsp)  (void)(mfsp)
#define mfs_flash_release(mfsp)  (void)(mfsp)
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static const uint16_t crc16_table[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

uint16_t crc16(uint16_t crc, const uint8_t *data, size_t n) {

  while (n > 0U) {
    crc = (crc << 8U) ^ crc16_table[(crc >> 8U) ^ (uint16_t)*data];
    data++;
    n--;
  }

  return crc;
}

static void mfs_state_reset(MFSDriver *mfsp) {
  unsigned i;

  mfsp->current_bank    = MFS_BANK_0;
  mfsp->current_counter = 0U;
  mfsp->next_offset     = 0U;
  mfsp->used_space      = 0U;

#if (MFS_CFG_TRANSACTION_MAX > 0)
  mfsp->tr_nops = 0U;
  mfsp->tr_next_offset = 0U;
  mfsp->tr_limit_offset = 0U;
#endif

  for (i = 0; i < MFS_CFG_MAX_RECORDS; i++) {
    mfsp->descriptors[i].offset = 0U;
    mfsp->descriptors[i].size   = 0U;
  }
}

static flash_offset_t mfs_flash_get_bank_offset(MFSDriver *mfsp,
                                                mfs_bank_t bank) {

  return bank == MFS_BANK_0 ? flashGetSectorOffset(mfsp->config->flashp,
                                                   mfsp->config->bank0_start) :
                              flashGetSectorOffset(mfsp->config->flashp,
                                                   mfsp->config->bank1_start);
}

/**
 * @brief   Flash read.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] offset    flash offset
 * @param[in] n         number of bytes to be read
 * @param[out] rp       pointer to the data buffer
 * @return              The operation status.
 *
 * @notapi
 */
static mfs_error_t mfs_flash_read(MFSDriver *mfsp, flash_offset_t offset,
                                  size_t n, uint8_t *rp) {
  flash_error_t ferr;
  mfs_error_t err;

  mfs_flash_acquire(mfsp);

  ferr = flashRead(mfsp->config->flashp, offset, n, rp);
  if (ferr != FLASH_NO_ERROR) {
    mfsp->state = MFS_ERROR;
    err = MFS_ERR_FLASH_FAILURE;
  }
  else {
    err = MFS_NO_ERROR;
  }

  mfs_flash_release(mfsp);

  return err;
}

/**
 * @brief   Flash write.
 * @note    If the option @p MFS_CFG_WRITE_VERIFY is enabled then the flash
 *          is also read back for verification.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] offset    flash offset
 * @param[in] n         number of bytes to be written
 * @param[in] wp        pointer to the data buffer
 * @return              The operation status.
 *
 * @notapi
 */
static mfs_error_t mfs_flash_write(MFSDriver *mfsp,
                                   flash_offset_t offset,
                                   size_t n,
                                   const uint8_t *wp) {
  flash_error_t ferr;

  mfs_flash_acquire(mfsp);

  ferr = flashProgram(mfsp->config->flashp, offset, n, wp);
  if (ferr != FLASH_NO_ERROR) {
    mfsp->state = MFS_ERROR;
    mfs_flash_release(mfsp);
    return MFS_ERR_FLASH_FAILURE;
  }

  mfs_flash_release(mfsp);

#if MFS_CFG_WRITE_VERIFY == TRUE
  /* Verifying the written data by reading it back and comparing.*/
  while (n > 0U) {
    size_t chunk = n <= MFS_CFG_BUFFER_SIZE ? n : MFS_CFG_BUFFER_SIZE;

    RET_ON_ERROR(mfs_flash_read(mfsp, offset, chunk, mfsp->ncbuf->data8));

    if (memcmp((void *)mfsp->ncbuf->data8, (void *)wp, chunk)) {
      mfsp->state = MFS_ERROR;
      return MFS_ERR_FLASH_FAILURE;
    }
    n -= chunk;
    offset += (flash_offset_t)chunk;
    wp += chunk;
  }
#endif

  return MFS_NO_ERROR;
}

/**
 * @brief   Flash copy.
 * @note    If the option @p MFS_CFG_WRITE_VERIFY is enabled then the flash
 *          is also read back for verification.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] doffset   destination flash offset
 * @param[in] soffset   source flash offset
 * @param[in] n         number of bytes to be copied
 * @return              The operation status.
 *
 * @notapi
 */
static mfs_error_t mfs_flash_copy(MFSDriver *mfsp,
                                  flash_offset_t doffset,
                                  flash_offset_t soffset,
                                  uint32_t n) {

  /* Splitting the operation in smaller operations because the buffer is
     small.*/
  while (n > 0U) {
    /* Data size that can be written in a single program page operation.*/
    size_t chunk = (size_t)(((doffset | (MFS_CFG_BUFFER_SIZE - 1U)) + 1U) -
                            doffset);
    if (chunk > n) {
      chunk = n;
    }

    RET_ON_ERROR(mfs_flash_read(mfsp, soffset, chunk, mfsp->ncbuf->data8));
    RET_ON_ERROR(mfs_flash_write(mfsp, doffset, chunk, mfsp->ncbuf->data8));

    /* Next page.*/
    soffset += chunk;
    doffset += chunk;
    n       -= chunk;
  }

  return MFS_NO_ERROR;
}

/**
 * @brief   Erases and verifies all sectors belonging to a bank.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] bank      bank to be erased
 * @return              The operation status.
 *
 * @notapi
 */
static mfs_error_t mfs_bank_erase(MFSDriver *mfsp, mfs_bank_t bank) {
  flash_sector_t sector, end;

  if (bank == MFS_BANK_0) {
    sector = mfsp->config->bank0_start;
    end    = mfsp->config->bank0_start + mfsp->config->bank0_sectors;
  }
  else {
    sector = mfsp->config->bank1_start;
    end    = mfsp->config->bank1_start + mfsp->config->bank1_sectors;
  }

  while (sector < end) {
    flash_error_t ferr;

    mfs_flash_acquire(mfsp);

    ferr = flashStartEraseSector(mfsp->config->flashp, sector);
    if (ferr != FLASH_NO_ERROR) {
      mfsp->state = MFS_ERROR;
      mfs_flash_release(mfsp);
      return MFS_ERR_FLASH_FAILURE;
    }
    ferr = flashWaitErase(mfsp->config->flashp);
    if (ferr != FLASH_NO_ERROR) {
      mfsp->state = MFS_ERROR;
      mfs_flash_release(mfsp);
      return MFS_ERR_FLASH_FAILURE;
    }
    ferr = flashVerifyErase(mfsp->config->flashp, sector);
    if (ferr != FLASH_NO_ERROR) {
      mfsp->state = MFS_ERROR;
      mfs_flash_release(mfsp);
      return MFS_ERR_FLASH_FAILURE;
    }

    mfs_flash_release(mfsp);

    sector++;
  }

  return MFS_NO_ERROR;
}

/**
 * @brief   Erases and verifies all sectors belonging to a bank.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] bank      bank to be verified
 * @return              The operation status.
 *
 * @notapi
 */
static mfs_error_t mfs_bank_verify_erase(MFSDriver *mfsp, mfs_bank_t bank) {
  flash_sector_t sector, end;

  if (bank == MFS_BANK_0) {
    sector = mfsp->config->bank0_start;
    end    = mfsp->config->bank0_start + mfsp->config->bank0_sectors;
  }
  else {
    sector = mfsp->config->bank1_start;
    end    = mfsp->config->bank1_start + mfsp->config->bank1_sectors;
  }

  while (sector < end) {
    flash_error_t ferr;

    mfs_flash_acquire(mfsp);

    ferr = flashVerifyErase(mfsp->config->flashp, sector);
    if (ferr == FLASH_ERROR_VERIFY) {
      mfs_flash_release(mfsp);
      return MFS_ERR_NOT_ERASED;
    }
    if (ferr != FLASH_NO_ERROR) {
      mfsp->state = MFS_ERROR;
      mfs_flash_release(mfsp);
      return MFS_ERR_FLASH_FAILURE;
    }

    mfs_flash_release(mfsp);

    sector++;
  }

  return MFS_NO_ERROR;
}

/**
 * @brief   Writes the validation header in a bank.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] bank      bank to be validated
 * @param[in] cnt       value for the flash usage counter
 * @return              The operation status.
 *
 * @notapi
 */
static mfs_error_t mfs_bank_write_header(MFSDriver *mfsp,
                                         mfs_bank_t bank,
                                         uint32_t cnt) {
  flash_sector_t sector;

  if (bank == MFS_BANK_0) {
    sector = mfsp->config->bank0_start;
  }
  else {
    sector = mfsp->config->bank1_start;
  }

  mfsp->ncbuf->bhdr.fields.magic1    = MFS_BANK_MAGIC_1;
  mfsp->ncbuf->bhdr.fields.magic2    = MFS_BANK_MAGIC_2;
  mfsp->ncbuf->bhdr.fields.counter   = cnt;
  mfsp->ncbuf->bhdr.fields.reserved1 = (uint16_t)mfsp->config->erased;
  mfsp->ncbuf->bhdr.fields.crc       = crc16(0xFFFFU,
                                             mfsp->ncbuf->bhdr.hdr8,
                                             sizeof (mfs_bank_header_t) - sizeof (uint16_t));

  return mfs_flash_write(mfsp,
                         flashGetSectorOffset(mfsp->config->flashp, sector),
                         sizeof (mfs_bank_header_t),
                         mfsp->ncbuf->bhdr.hdr8);
}

/**
 * @brief   Checks integrity of the header in the shared buffer.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @return              The header state.
 *
 * @notapi
 */
static mfs_bank_state_t mfs_bank_check_header(MFSDriver *mfsp) {
  uint16_t crc;

  if ((mfsp->ncbuf->bhdr.hdr32[0] == mfsp->config->erased) &&
      (mfsp->ncbuf->bhdr.hdr32[1] == mfsp->config->erased) &&
      (mfsp->ncbuf->bhdr.hdr32[2] == mfsp->config->erased) &&
      (mfsp->ncbuf->bhdr.hdr32[3] == mfsp->config->erased)) {
    return MFS_BANK_ERASED;
  }

  /* Checking header fields integrity.*/
  if ((mfsp->ncbuf->bhdr.fields.magic1 != MFS_BANK_MAGIC_1) ||
      (mfsp->ncbuf->bhdr.fields.magic2 != MFS_BANK_MAGIC_2) ||
      (mfsp->ncbuf->bhdr.fields.counter == mfsp->config->erased) ||
      (mfsp->ncbuf->bhdr.fields.reserved1 != (uint16_t)mfsp->config->erased)) {
    return MFS_BANK_GARBAGE;
  }

  /* Verifying header CRC.*/
  crc = crc16(0xFFFFU, mfsp->ncbuf->bhdr.hdr8,
              sizeof (mfs_bank_header_t) - sizeof (uint16_t));
  if (crc != mfsp->ncbuf->bhdr.fields.crc) {
    return MFS_BANK_GARBAGE;
  }

  return MFS_BANK_OK;
}

/**
 * @brief   Scans blocks searching for records.
 * @note    The block integrity is strongly checked.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] bank      the bank identifier
 * @param[out] wflagp   warning flag on anomalies
 *
 * @return              The operation status.
 *
 * @notapi
 */
static mfs_error_t mfs_bank_scan_records(MFSDriver *mfsp,
                                         mfs_bank_t bank,
                                         bool *wflagp) {
  flash_offset_t hdr_offset, start_offset, end_offset;

  /* No warning by default.*/
  *wflagp = false;

  /* Boundaries.*/
  start_offset = mfs_flash_get_bank_offset(mfsp, bank);
  hdr_offset   = start_offset + (flash_offset_t)ALIGNED_SIZEOF(mfs_bank_header_t);
  end_offset   = start_offset + mfsp->config->bank_size;

  /* Scanning records until there is there is not enough space left for an
     header.*/
  while (hdr_offset < end_offset - ALIGNED_DHDR_SIZE) {
    mfs_data_header_t dhdr;
    uint16_t crc;
    flash_offset_t data_offset;
    flash_offset_t data_available;

    /* Reading the current record header.*/
    RET_ON_ERROR(mfs_flash_read(mfsp, hdr_offset,
                                sizeof (mfs_data_header_t),
                                mfsp->ncbuf->data8));

    /* Checking if the found header is in erased state.*/
    if ((mfsp->ncbuf->data32[0] == mfsp->config->erased) &&
        (mfsp->ncbuf->data32[1] == mfsp->config->erased) &&
        (mfsp->ncbuf->data32[2] == mfsp->config->erased)) {
      break;
    }

    data_offset = hdr_offset + (flash_offset_t)sizeof (mfs_data_header_t);
    if (data_offset > end_offset) {
      *wflagp = true;
      break;
    }
    data_available = end_offset - data_offset;

    /* It is not erased so checking for integrity.*/
    if ((mfsp->ncbuf->dhdr.fields.magic1 != MFS_HEADER_MAGIC_1) ||
        (mfsp->ncbuf->dhdr.fields.magic2 != MFS_HEADER_MAGIC_2) ||
        (mfsp->ncbuf->dhdr.fields.id < 1U) ||
        (mfsp->ncbuf->dhdr.fields.id > (uint32_t)MFS_CFG_MAX_RECORDS) ||
        (mfsp->ncbuf->dhdr.fields.size > data_available)) {
      *wflagp = true;
      break;
    }

    /* Copying the non-cached buffer locally.*/
    dhdr = mfsp->ncbuf->dhdr;

    /* Finally checking the CRC, we need to perform it in chunks because
       we have a limited buffer.*/
    crc = 0xFFFFU;
    if (dhdr.fields.size > 0U) {
      uint32_t total = dhdr.fields.size;

      while (total > 0U) {
        uint32_t chunk = total > MFS_CFG_BUFFER_SIZE ? MFS_CFG_BUFFER_SIZE :
                                                       total;

        /* Reading the data chunk.*/
        RET_ON_ERROR(mfs_flash_read(mfsp, data_offset, chunk,
                                    mfsp->ncbuf->data8));

        /* CRC on the read data chunk.*/
        crc = crc16(crc, &mfsp->ncbuf->data8[0], chunk);

        /* Next chunk.*/
        data_offset += chunk;
        total -= chunk;
      }
    }
    if (crc != dhdr.fields.crc) {
      /* If the CRC is invalid then this record is ignored but scanning
         continues because there could be more valid records afterward.*/
      *wflagp = true;
    }
    else {
      /* Zero-sized records are erase markers.*/
      if (dhdr.fields.size == 0U) {
        mfsp->descriptors[dhdr.fields.id - 1U].offset = 0U;
        mfsp->descriptors[dhdr.fields.id - 1U].size   = 0U;
      }
      else {
        mfsp->descriptors[dhdr.fields.id - 1U].offset = hdr_offset;
        mfsp->descriptors[dhdr.fields.id - 1U].size   = dhdr.fields.size;
      }
    }

    /* On the next header.*/
    hdr_offset = hdr_offset + ALIGNED_REC_SIZE(dhdr.fields.size);
  }

  /* Next writable offset.*/
  mfsp->next_offset = hdr_offset;

  return MFS_NO_ERROR;
}

/**
 * @brief   Determines the state of a bank.
 * @note    This function does not test the bank integrity by scanning
 *          the data area, it just checks the header.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] bank      bank to be checked
 * @param[out] statep   bank state, it can be:
 *                      - MFS_BANK_ERASED
 *                      - MFS_BANK_GARBAGE
 *                      - MFS_BANK_OK
 * @param[out] cntp     bank counter
 * @return              The operation status.
 *
 * @notapi
 */
static mfs_error_t mfs_bank_get_state(MFSDriver *mfsp,
                                      mfs_bank_t bank,
                                      mfs_bank_state_t *statep,
                                      uint32_t *cntp) {

  /* Reading the current bank header.*/
  RET_ON_ERROR(mfs_flash_read(mfsp, mfs_flash_get_bank_offset(mfsp, bank),
                              sizeof (mfs_bank_header_t),
                              mfsp->ncbuf->data8));

  /* Getting the counter regardless of the bank state, it is only valid if
     the state is MFS_BANK_OK.*/
  *cntp = mfsp->ncbuf->bhdr.fields.counter;

  /* Checking just the header.*/
  *statep = mfs_bank_check_header(mfsp);
  if (*statep == MFS_BANK_ERASED) {
    mfs_error_t err;

    /* Checking if the bank is really all erased.*/
    err = mfs_bank_verify_erase(mfsp, bank);
    if (err == MFS_ERR_NOT_ERASED) {
      *statep = MFS_BANK_GARBAGE;
    }
  }

  return MFS_NO_ERROR;
}

/**
 * @brief   Enforces a garbage collection.
 * @details Storage data is compacted into a single bank.
 *
 * @param[out] mfsp     pointer to the @p MFSDriver object
 * @return              The operation status.
 *
 * @notapi
 */
static mfs_error_t mfs_garbage_collect(MFSDriver *mfsp) {
  unsigned i;
  mfs_bank_t sbank, dbank;
  flash_offset_t dest_offset;

  sbank = mfsp->current_bank;
  if (sbank == MFS_BANK_0) {
    dbank = MFS_BANK_1;
  }
  else {
    dbank = MFS_BANK_0;
  }

  /* Write address.*/
  dest_offset = mfs_flash_get_bank_offset(mfsp, dbank) +
                ALIGNED_SIZEOF(mfs_bank_header_t);

  /* Copying the most recent record instances only.*/
  for (i = 0; i < MFS_CFG_MAX_RECORDS; i++) {
    uint32_t totsize = ALIGNED_REC_SIZE(mfsp->descriptors[i].size);
    if (mfsp->descriptors[i].offset != 0) {
      RET_ON_ERROR(mfs_flash_copy(mfsp, dest_offset,
                                  mfsp->descriptors[i].offset,
                                  totsize));
      mfsp->descriptors[i].offset = dest_offset;
      dest_offset += totsize;
    }
  }

  /* New current bank.*/
  mfsp->current_bank = dbank;
  mfsp->current_counter += 1U;
  mfsp->next_offset = dest_offset;

  /* The header is written after the data.*/
  RET_ON_ERROR(mfs_bank_write_header(mfsp, dbank, mfsp->current_counter));

  /* The source bank is erased last.*/
  RET_ON_ERROR(mfs_bank_erase(mfsp, sbank));

  return MFS_NO_ERROR;
}

/**
 * @brief   Performs a flash partition mount attempt.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @return              The operation status.
 *
 * @api
 */
static mfs_error_t mfs_try_mount(MFSDriver *mfsp) {
  mfs_bank_state_t sts0, sts1;
  mfs_bank_t bank;
  uint32_t cnt0 = 0, cnt1 = 0;
  bool w1 = false, w2 = false;

  /* Resetting the bank state.*/
  mfs_state_reset(mfsp);

  /* Assessing the state of the two banks.*/
  RET_ON_ERROR(mfs_bank_get_state(mfsp, MFS_BANK_0, &sts0, &cnt0));
  RET_ON_ERROR(mfs_bank_get_state(mfsp, MFS_BANK_1, &sts1, &cnt1));

  /* Handling all possible scenarios, each one requires its own recovery
     strategy.*/
  switch (PAIR(sts0, sts1)) {

  case PAIR(MFS_BANK_ERASED, MFS_BANK_ERASED):
    /* Both banks erased, first initialization.*/
    RET_ON_ERROR(mfs_bank_write_header(mfsp, MFS_BANK_0, 1));
    bank = MFS_BANK_0;
    break;

  case PAIR(MFS_BANK_OK, MFS_BANK_OK):
    /* Both banks appear to be valid but one must be newer, erasing the
       older one.*/
    if (cnt0 > cnt1) {
      /* Bank 0 is newer.*/
      RET_ON_ERROR(mfs_bank_erase(mfsp, MFS_BANK_1));
      bank = MFS_BANK_0;
    }
    else {
      /* Bank 1 is newer.*/
      RET_ON_ERROR(mfs_bank_erase(mfsp, MFS_BANK_0));
      bank = MFS_BANK_1;
    }
    w1 = true;
    break;

  case PAIR(MFS_BANK_GARBAGE, MFS_BANK_GARBAGE):
    /* Both banks are unreadable, reinitializing.*/
    RET_ON_ERROR(mfs_bank_erase(mfsp, MFS_BANK_0));
    RET_ON_ERROR(mfs_bank_erase(mfsp, MFS_BANK_1));
    RET_ON_ERROR(mfs_bank_write_header(mfsp, MFS_BANK_0, 1));
    bank = MFS_BANK_0;
    w1 = true;
    break;

  case PAIR(MFS_BANK_ERASED, MFS_BANK_OK):
    /* Normal situation, bank one is used.*/
    bank = MFS_BANK_1;
    break;

  case PAIR(MFS_BANK_OK, MFS_BANK_ERASED):
    /* Normal situation, bank zero is used.*/
    bank = MFS_BANK_0;
    break;

  case PAIR(MFS_BANK_ERASED, MFS_BANK_GARBAGE):
    /* Bank zero is erased, bank one is not readable.*/
    RET_ON_ERROR(mfs_bank_erase(mfsp, MFS_BANK_1));
    RET_ON_ERROR(mfs_bank_write_header(mfsp, MFS_BANK_0, 1));
    bank = MFS_BANK_0;
    w1 = true;
    break;

  case PAIR(MFS_BANK_GARBAGE, MFS_BANK_ERASED):
    /* Bank zero is not readable, bank one is erased.*/
    RET_ON_ERROR(mfs_bank_erase(mfsp, MFS_BANK_0));
    RET_ON_ERROR(mfs_bank_write_header(mfsp, MFS_BANK_1, 1));
    bank = MFS_BANK_1;
    w1 = true;
    break;

  case PAIR(MFS_BANK_OK, MFS_BANK_GARBAGE):
    /* Bank zero is normal, bank one is unreadable.*/
    RET_ON_ERROR(mfs_bank_erase(mfsp, MFS_BANK_1));
    bank = MFS_BANK_0;
    w1 = true;
    break;

  case PAIR(MFS_BANK_GARBAGE, MFS_BANK_OK):
    /* Bank zero is unreadable, bank one is normal.*/
    RET_ON_ERROR(mfs_bank_erase(mfsp, MFS_BANK_0));
    bank = MFS_BANK_1;
    w1 = true;
    break;

  default:
    return MFS_ERR_INTERNAL;
  }

  /* Mounting the bank.*/
  {
    unsigned i;

    /* Reading the bank header again.*/
    RET_ON_ERROR(mfs_flash_read(mfsp, mfs_flash_get_bank_offset(mfsp, bank),
                                sizeof (mfs_bank_header_t),
                                mfsp->ncbuf->data8));

    /* Checked again for extra safety.*/
    if (mfs_bank_check_header(mfsp) != MFS_BANK_OK) {
      return MFS_ERR_INTERNAL;
    }

    /* Storing the bank data.*/
    mfsp->current_bank    = bank;
    mfsp->current_counter = mfsp->ncbuf->bhdr.fields.counter;

    /* Scanning for the most recent instance of all records.*/
    RET_ON_ERROR(mfs_bank_scan_records(mfsp, bank, &w2));

    /* Calculating the effective used size.*/
    mfsp->used_space = ALIGNED_SIZEOF(mfs_bank_header_t);
    for (i = 0; i < MFS_CFG_MAX_RECORDS; i++) {
      if (mfsp->descriptors[i].offset != 0U) {
        mfsp->used_space += ALIGNED_REC_SIZE(mfsp->descriptors[i].size);
      }
    }
  }

  /* In case of detected problems then a garbage collection is performed in
     order to repair/remove anomalies.*/
  if (w2) {
    RET_ON_ERROR(mfs_garbage_collect(mfsp));
  }

  return (w1 || w2) ? MFS_WARN_REPAIR : MFS_NO_ERROR;
}

/**
 * @brief   Configures and activates a MFS driver.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @return              The operation status.
 * @retval MFS_NO_ERROR             if the operation has been successfully
 *                                  completed.
 * @retval MFS_WARN_GC              if the operation triggered a garbage
 *                                  collection.
 * @retval MFS_ERR_FLASH_FAILURE    if the flash memory is unusable because HW
 *                                  failures. Makes the driver enter the
 *                                  @p MFS_ERROR state.
 * @retval MFS_ERR_INTERNAL         if an internal logic failure is detected.
 *
 * @api
 */
mfs_error_t mfs_mount(MFSDriver *mfsp) {
  unsigned i;

  /* Resetting previous state.*/
  mfs_state_reset(mfsp);

  /* Attempting to mount the managed partition.*/
  for (i = 0; i < MFS_CFG_MAX_REPAIR_ATTEMPTS; i++) {
    mfs_error_t err;

    err = mfs_try_mount(mfsp);
    if (err == MFS_ERR_INTERNAL) {
      /* Special case, do not retry on internal errors but report
         immediately.*/
      mfsp->state = MFS_ERROR;
      return err;
    }
    if (!MFS_IS_ERROR(err)) {
      mfsp->state  = MFS_READY;
      return err;
    }
  }

  /* Driver start failed.*/
  mfsp->state = MFS_ERROR;
  return MFS_ERR_FLASH_FAILURE;
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes an instance.
 *
 * @param[out] mfsp     pointer to the @p MFSDriver object
 * @param[in] ncbuf     pointer to an @p mfs_nocache_buffer_t buffer structure
 *
 * @init
 */
void mfsObjectInit(MFSDriver *mfsp, mfs_nocache_buffer_t *ncbuf) {

  osalDbgCheck(mfsp != NULL);

  mfsp->state  = MFS_STOP;
  mfsp->config = NULL;
  mfsp->ncbuf  = ncbuf;
}

/**
 * @brief   Configures and activates a MFS driver.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] config    pointer to the configuration
 * @return              The operation status.
 * @retval MFS_NO_ERROR             if the operation has been
 *                                  completed.
 * @retval MFS_WARN_GC              if the operation triggered a garbage
 *                                  collection.
 * @retval MFS_ERR_FLASH_FAILURE    if the flash memory is unusable because HW
 *                                  failures. Makes the driver enter the
 *                                  @p MFS_ERROR state.
 * @retval MFS_ERR_INTERNAL         if an internal logic failure is detected.
 *
 * @api
 */
mfs_error_t mfsStart(MFSDriver *mfsp, const MFSConfig *config) {

  osalDbgCheck((mfsp != NULL) && (config != NULL));
  osalDbgAssert((mfsp->state == MFS_STOP) || (mfsp->state == MFS_READY) ||
                (mfsp->state == MFS_ERROR), "invalid state");

  /* Storing configuration.*/
  mfsp->config = config;

  return mfs_mount(mfsp);
} 

/**
 * @brief   Deactivates a MFS driver.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 *
 * @api
 */
void mfsStop(MFSDriver *mfsp) {

  osalDbgCheck(mfsp != NULL);
  osalDbgAssert((mfsp->state == MFS_STOP) || (mfsp->state == MFS_READY) ||
                (mfsp->state == MFS_ERROR), "invalid state");

  mfsp->config = NULL;
  mfsp->state = MFS_STOP;
}

/**
 * @brief   Destroys the state of the managed storage by erasing the flash.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @return              The operation status.
 * @retval MFS_ERR_INV_STATE        if the driver is in not in @p MFS_READY
 *                                  state.
 * @retval MFS_NO_ERROR             if the operation has been successfully
 *                                  completed.
 * @retval MFS_ERR_FLASH_FAILURE    if the flash memory is unusable because HW
 *                                  failures. Makes the driver enter the
 *                                  @p MFS_ERROR state.
 * @retval MFS_ERR_INTERNAL         if an internal logic failure is detected.
 *
 * @api
 */
mfs_error_t mfsErase(MFSDriver *mfsp) {

  osalDbgCheck(mfsp != NULL);

  if (mfsp->state != MFS_READY) {
    return MFS_ERR_INV_STATE;
  }

  RET_ON_ERROR(mfs_bank_erase(mfsp, MFS_BANK_0));
  RET_ON_ERROR(mfs_bank_erase(mfsp, MFS_BANK_1));

  return mfs_mount(mfsp);
}

/**
 * @brief   Retrieves and reads a data record.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] id        record numeric identifier, the valid range is between
 *                      @p 1 and @p MFS_CFG_MAX_RECORDS
 * @param[in,out] np    on input is the maximum buffer size, on return it is
 *                      the size of the data copied into the buffer
 * @param[out] buffer   pointer to a buffer for record data
 * @return              The operation status.
 * @retval MFS_NO_ERROR             if the operation has been successfully
 *                                  completed.
 * @retval MFS_ERR_INV_STATE        if the driver is in not in @p MFS_READY
 *                                  state.
 * @retval MFS_ERR_INV_SIZE         if the passed buffer is not large enough to
 *                                  contain the record data.
 * @retval MFS_ERR_NOT_FOUND        if the specified id does not exists.
 * @retval MFS_ERR_FLASH_FAILURE    if the flash memory is unusable because HW
 *                                  failures. Makes the driver enter the
 *                                  @p MFS_ERROR state.
 * @retval MFS_ERR_INTERNAL         if an internal logic failure is detected.
 *
 * @api
 */
mfs_error_t mfsReadRecord(MFSDriver *mfsp, mfs_id_t id,
                          size_t *np, uint8_t *buffer) {
  uint16_t crc;

  osalDbgCheck((mfsp != NULL) &&
               (id >= 1U) && (id <= (mfs_id_t)MFS_CFG_MAX_RECORDS) &&
               (np != NULL) && (*np > 0U) && (buffer != NULL));

  if ((mfsp->state != MFS_READY) && (mfsp->state != MFS_TRANSACTION)) {
    return MFS_ERR_INV_STATE;
  }

  /* Checking if the requested record actually exists.*/
  if (mfsp->descriptors[id - 1U].offset == 0U) {
    return MFS_ERR_NOT_FOUND;
  }

  /* Making sure to not overflow the buffer.*/
  if (*np < mfsp->descriptors[id - 1U].size) {
    return MFS_ERR_INV_SIZE;
  }

  /* Header read from flash.*/
  RET_ON_ERROR(mfs_flash_read(mfsp,
                              mfsp->descriptors[id - 1U].offset,
                              sizeof (mfs_data_header_t),
                              mfsp->ncbuf->data8));

  /* Data read from flash.*/
  *np = mfsp->descriptors[id - 1U].size;
  RET_ON_ERROR(mfs_flash_read(mfsp,
                              mfsp->descriptors[id - 1U].offset + sizeof (mfs_data_header_t),
                              *np,
                              buffer));

  /* Checking CRC.*/
  crc = crc16(0xFFFFU, buffer, *np);
  if (crc != mfsp->ncbuf->dhdr.fields.crc) {
    mfsp->state = MFS_ERROR;
    return MFS_ERR_FLASH_FAILURE;
  }

  return MFS_NO_ERROR;
}

/**
 * @brief   Creates or updates a data record.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] id        record numeric identifier, the valid range is between
 *                      @p 1 and @p MFS_CFG_MAX_RECORDS
 * @param[in] n         size of data to be written, it cannot be zero
 * @param[in] buffer    pointer to a buffer for record data
 * @return              The operation status.
 * @retval MFS_NO_ERROR             if the operation has been successfully
 *                                  completed.
 * @retval MFS_WARN_GC              if the operation triggered a garbage
 *                                  collection.
 * @retval MFS_ERR_INV_STATE        if the driver is in not in @p MFS_READY
 *                                  state.
 * @retval MFS_ERR_OUT_OF_MEM       if there is not enough flash space for the
 *                                  operation.
 * @retval MFS_ERR_TRANSACTION_NUM  if the transaction operations buffer space
 *                                  has been exceeded.
 * @retval MFS_ERR_TRANSACTION_SIZE if the transaction allocated space
 *                                  has been exceeded.
 * @retval MFS_ERR_FLASH_FAILURE    if the flash memory is unusable because HW
 *                                  failures. Makes the driver enter the
 *                                  @p MFS_ERROR state.
 * @retval MFS_ERR_INTERNAL         if an internal logic failure is detected.
 *
 * @api
 */
mfs_error_t mfsWriteRecord(MFSDriver *mfsp, mfs_id_t id,
                           size_t n, const uint8_t *buffer) {
  flash_offset_t free, asize, rspace;

  osalDbgCheck((mfsp != NULL) &&
               (id >= 1U) && (id <= (mfs_id_t)MFS_CFG_MAX_RECORDS) &&
               (n > (size_t)0) && (n <= (size_t)MFS_CFG_MAX_RECORD_SIZE) &&
               (buffer != NULL));

  /* Aligned record size.*/
  asize = ALIGNED_REC_SIZE(n);

  /* Normal mode code path.*/
  if (mfsp->state == MFS_READY) {
    bool warning = false;

    /* If the required space is beyond the available (compacted) block
       size then an error is returned.
       NOTE: The space for one extra header is reserved in order to allow
       for an erase operation after the space has been fully allocated.*/
    rspace = ALIGNED_DHDR_SIZE + asize;
    if (rspace > mfsp->config->bank_size - mfsp->used_space) {
      return MFS_ERR_OUT_OF_MEM;
    }

    /* Checking for immediately (not compacted) available space.*/
    free = (mfs_flash_get_bank_offset(mfsp, mfsp->current_bank) +
            mfsp->config->bank_size) - mfsp->next_offset;
    if (rspace > free) {
      /* We need to perform a garbage collection, there is enough space
         but it has to be freed.*/
      warning = true;
      RET_ON_ERROR(mfs_garbage_collect(mfsp));
    }

    /* Writing the data header without the magic, it will be written last.*/
    mfsp->ncbuf->dhdr.fields.id     = (uint16_t)id;
    mfsp->ncbuf->dhdr.fields.size   = (uint32_t)n;
    mfsp->ncbuf->dhdr.fields.crc    = crc16(0xFFFFU, buffer, n);
    RET_ON_ERROR(mfs_flash_write(mfsp,
                                 mfsp->next_offset + (sizeof (uint32_t) * 2U),
                                 sizeof (mfs_data_header_t) - (sizeof (uint32_t) * 2U),
                                 mfsp->ncbuf->data8 + (sizeof (uint32_t) * 2U)));

    /* Writing the data part.*/
    RET_ON_ERROR(mfs_flash_write(mfsp,
                                 mfsp->next_offset + sizeof (mfs_data_header_t),
                                 n,
                                 buffer));

    /* Finally writing the magic number, it seals the operation.*/
    mfsp->ncbuf->dhdr.fields.magic1 = (uint32_t)MFS_HEADER_MAGIC_1;
    mfsp->ncbuf->dhdr.fields.magic2 = (uint32_t)MFS_HEADER_MAGIC_2;
    RET_ON_ERROR(mfs_flash_write(mfsp,
                                 mfsp->next_offset,
                                 sizeof (uint32_t) * 2U,
                                 mfsp->ncbuf->data8));

    /* The size of the old record instance, if present, must be subtracted
       to the total used size.*/
    if (mfsp->descriptors[id - 1U].offset != 0U) {
      mfsp->used_space -= ALIGNED_REC_SIZE(mfsp->descriptors[id - 1U].size);
    }

    /* Adjusting bank-related metadata.*/
    mfsp->descriptors[id - 1U].offset = mfsp->next_offset;
    mfsp->descriptors[id - 1U].size   = (uint32_t)n;
    mfsp->next_offset += asize;
    mfsp->used_space  += asize;

    return warning ? MFS_WARN_GC : MFS_NO_ERROR;
  }

#if MFS_CFG_TRANSACTION_MAX > 0
  /* Transaction mode code path.*/
  if (mfsp->state == MFS_TRANSACTION) {
    mfs_transaction_op_t *top;

    /* Checking if the maximum number of operations in a transaction is
       Exceeded.*/
    if (mfsp->tr_nops >= MFS_CFG_TRANSACTION_MAX) {
      return MFS_ERR_TRANSACTION_NUM;
    }

    /* If the required space is greater than the space allocated for the
       transaction then error.
       Note, the condition check is written in a defensive way.*/
    rspace = asize;
    if ((mfsp->tr_next_offset > mfsp->tr_limit_offset) ||
        (rspace > mfsp->tr_limit_offset - mfsp->tr_next_offset)) {
      return MFS_ERR_TRANSACTION_SIZE;
    }

    /* Writing the data header without the magic, it will be written last.*/
    mfsp->ncbuf->dhdr.fields.id     = (uint16_t)id;
    mfsp->ncbuf->dhdr.fields.size   = (uint32_t)n;
    mfsp->ncbuf->dhdr.fields.crc    = crc16(0xFFFFU, buffer, n);
    RET_ON_ERROR(mfs_flash_write(mfsp,
                                 mfsp->tr_next_offset + (sizeof (uint32_t) * 2U),
                                 sizeof (mfs_data_header_t) - (sizeof (uint32_t) * 2U),
                                 mfsp->ncbuf->data8 + (sizeof (uint32_t) * 2U)));

    /* Writing the data part.*/
    RET_ON_ERROR(mfs_flash_write(mfsp,
                                 mfsp->tr_next_offset + sizeof (mfs_data_header_t),
                                 n,
                                 buffer));

    /* Adding a transaction operation record.*/
    top = &mfsp->tr_ops[mfsp->tr_nops];
    top->offset = mfsp->tr_next_offset;
    top->size   = n;
    top->id     = id;

    /* Number of records and next write position updated.*/
    mfsp->tr_nops++;
    mfsp->tr_next_offset += asize;

    return MFS_NO_ERROR;
  }
#endif /* MFS_CFG_TRANSACTION_MAX > 0 */

  /* Invalid state.*/
  return MFS_ERR_INV_STATE;
}

/**
 * @brief   Erases a data record.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] id        record numeric identifier, the valid range is between
 *                      @p 1 and @p MFS_CFG_MAX_RECORDS
 * @return              The operation status.
 * @retval MFS_NO_ERROR             if the operation has been successfully
 *                                  completed.
 * @retval MFS_WARN_GC              if the operation triggered a garbage
 *                                  collection.
 * @retval MFS_ERR_INV_STATE        if the driver is in not in @p MFS_READY
 *                                  state.
 * @retval MFS_ERR_OUT_OF_MEM       if there is not enough flash space for the
 *                                  operation.
 * @retval MFS_ERR_TRANSACTION_NUM  if the transaction operations buffer space
 *                                  has been exceeded.
 * @retval MFS_ERR_TRANSACTION_SIZE if the transaction allocated space
 *                                  has been exceeded.
 * @retval MFS_ERR_FLASH_FAILURE    if the flash memory is unusable because HW
 *                                  failures. Makes the driver enter the
 *                                  @p MFS_ERROR state.
 * @retval MFS_ERR_INTERNAL         if an internal logic failure is detected.
 *
 * @api
 */
mfs_error_t mfsEraseRecord(MFSDriver *mfsp, mfs_id_t id) {
  flash_offset_t free, asize, rspace;

  osalDbgCheck((mfsp != NULL) &&
               (id >= 1U) && (id <= (mfs_id_t)MFS_CFG_MAX_RECORDS));

  /* Aligned record size.*/
  asize = ALIGNED_DHDR_SIZE;

  /* Normal mode code path.*/
  if (mfsp->state == MFS_READY) {
    bool warning = false;

    /* Checking if the requested record actually exists.*/
    if (mfsp->descriptors[id - 1U].offset == 0U) {
      return MFS_ERR_NOT_FOUND;
    }

    /* If the required space is beyond the available (compacted) block
       size then an internal error is returned, it should never happen.*/
    rspace = asize;
    if (rspace > mfsp->config->bank_size - mfsp->used_space) {
      return MFS_ERR_INTERNAL;
    }

    /* Checking for immediately (not compacted) available space.*/
    free = (mfs_flash_get_bank_offset(mfsp, mfsp->current_bank) +
            mfsp->config->bank_size) - mfsp->next_offset;
    if (rspace > free) {
      /* We need to perform a garbage collection, there is enough space
         but it has to be freed.*/
      warning = true;
      RET_ON_ERROR(mfs_garbage_collect(mfsp));
    }

    /* Writing the data header with size set to zero, it means that the
       record is logically erased.*/
    mfsp->ncbuf->dhdr.fields.magic1 = (uint32_t)MFS_HEADER_MAGIC_1;
    mfsp->ncbuf->dhdr.fields.magic2 = (uint32_t)MFS_HEADER_MAGIC_2;
    mfsp->ncbuf->dhdr.fields.id     = (uint16_t)id;
    mfsp->ncbuf->dhdr.fields.size   = (uint32_t)0;
    mfsp->ncbuf->dhdr.fields.crc    = (uint16_t)0xFFFF;
    RET_ON_ERROR(mfs_flash_write(mfsp,
                                 mfsp->next_offset,
                                 sizeof (mfs_data_header_t),
                                 mfsp->ncbuf->data8));

    /* Adjusting bank-related metadata.*/
    mfsp->used_space  -= ALIGNED_REC_SIZE(mfsp->descriptors[id - 1U].size);
    mfsp->next_offset += asize;
    mfsp->descriptors[id - 1U].offset = 0U;
    mfsp->descriptors[id - 1U].size   = 0U;

    return warning ? MFS_WARN_GC : MFS_NO_ERROR;
  }

#if MFS_CFG_TRANSACTION_MAX > 0
  /* Transaction mode code path.*/
  if (mfsp->state == MFS_TRANSACTION) {
    mfs_transaction_op_t *top;

    /* Checking if the requested record actually exists.*/
    if (mfsp->descriptors[id - 1U].offset == 0U) {
      return MFS_ERR_NOT_FOUND;
    }

    /* Checking if the maximum number of operations in a transaction is
       Exceeded.*/
    if (mfsp->tr_nops >= MFS_CFG_TRANSACTION_MAX) {
      return MFS_ERR_TRANSACTION_NUM;
    }

    /* If the required space is greater than the space allocated for the
       transaction then error.
       Note, the condition check is written in a defensive way.*/
    rspace = asize;
    if ((mfsp->tr_next_offset > mfsp->tr_limit_offset) ||
        (rspace > mfsp->tr_limit_offset - mfsp->tr_next_offset)) {
      return MFS_ERR_TRANSACTION_SIZE;
    }

    /* Writing the data header with size set to zero, it means that the
       record is logically erased. Note, the magic number is not set.*/
    mfsp->ncbuf->dhdr.fields.id     = (uint16_t)id;
    mfsp->ncbuf->dhdr.fields.size   = (uint32_t)0;
    mfsp->ncbuf->dhdr.fields.crc    = (uint16_t)0xFFFF;
    RET_ON_ERROR(mfs_flash_write(mfsp,
                                 mfsp->tr_next_offset + (sizeof (uint32_t) * 2U),
                                 sizeof (mfs_data_header_t) - (sizeof (uint32_t) * 2U),
                                 mfsp->ncbuf->data8 + (sizeof (uint32_t) * 2U)));

    /* Adding a transaction operation record.*/
    top = &mfsp->tr_ops[mfsp->tr_nops];
    top->offset = mfsp->tr_next_offset;
    top->size   = 0U;
    top->id     = id;

    /* Number of records and next write position updated.*/
    mfsp->tr_nops++;
    mfsp->tr_next_offset += asize;

    return MFS_NO_ERROR;
  }
#endif /* MFS_CFG_TRANSACTION_MAX > 0 */

  return MFS_ERR_INV_STATE;
}

/**
 * @brief   Enforces a garbage collection operation.
 * @details Garbage collection involves: integrity check, optionally repairs,
 *          obsolete data removal, data compaction and a flash bank swap.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @return              The operation status.
 * @retval MFS_NO_ERROR             if the operation has been successfully
 *                                  completed.
 * @retval MFS_ERR_INV_STATE        if the driver is in not in @p MFS_READY
 *                                  state.
 * @retval MFS_ERR_FLASH_FAILURE    if the flash memory is unusable because HW
 *                                  failures. Makes the driver enter the
 *                                  @p MFS_ERROR state.
 * @retval MFS_ERR_INTERNAL         if an internal logic failure is detected.
 *
 * @api
 */
mfs_error_t mfsPerformGarbageCollection(MFSDriver *mfsp) {

  osalDbgCheck(mfsp != NULL);

  if (mfsp->state != MFS_READY) {
    return MFS_ERR_INV_STATE;
  }

  return mfs_garbage_collect(mfsp);
}

#if (MFS_CFG_TRANSACTION_MAX > 0) || defined(__DOXYGEN__)
/**
 * @brief   Puts the driver in transaction mode.
 * @note    The parameters @p n and @p size are used to make an
 *          estimation of the space required for the transaction to succeed.
 *          Note that the estimated size must include also the extra space
 *          required by alignment enforcement option. If the estimated size
 *          is wrong (by defect) what could happen is that there is a failure
 *          in the middle of a transaction and a roll-back would be required.
 *  @note   The conditions for starting a transaction are:
 *          - The driver must be started.
 *          - There must be enough compacted storage to accommodate the whole
 *            transaction. If the required space is available but it is not
 *            compacted then a garbage collect operation is performed.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @param[in] size      estimated total size of written records in transaction,
 *                      this includes, data, headers and alignment gaps
 * @return              The operation status.
 * @retval MFS_NO_ERROR             if the operation has been successfully
 *                                  completed.
 * @retval MFS_ERR_INV_STATE        if the driver is in not in @p MFS_READY
 *                                  state.
 * @retval MFS_ERR_FLASH_FAILURE    if the flash memory is unusable because HW
 *                                  failures. Makes the driver enter the
 *                                  @p MFS_ERROR state.
 * @retval MFS_ERR_INTERNAL         if an internal logic failure is detected.
 *
 * @api
 */
mfs_error_t mfsStartTransaction(MFSDriver *mfsp, size_t size) {
  flash_offset_t free, tspace, rspace;

  osalDbgCheck((mfsp != NULL) && (size > ALIGNED_DHDR_SIZE));

  /* The driver must be in ready mode.*/
  if (mfsp->state != MFS_READY) {
    return MFS_ERR_INV_STATE;
  }

  /* Estimating the required contiguous compacted space.*/
  tspace = (flash_offset_t)MFS_ALIGN_NEXT(size);
  rspace = tspace + ALIGNED_DHDR_SIZE;

  /* If the required space is beyond the available (compacted) block
     size then an error is returned.*/
  if (rspace > mfsp->config->bank_size - mfsp->used_space) {
    return MFS_ERR_OUT_OF_MEM;
  }

  /* Checking for immediately (not compacted) available space.*/
  free = (mfs_flash_get_bank_offset(mfsp, mfsp->current_bank) +
          mfsp->config->bank_size) - mfsp->next_offset;
  if (rspace > free) {
    /* We need to perform a garbage collection, there is enough space
       but it has to be freed.*/
    RET_ON_ERROR(mfs_garbage_collect(mfsp));
  }

  /* Entering transaction mode.*/
  mfsp->state = MFS_TRANSACTION;

  /* Initializing transaction state.*/
  mfsp->tr_next_offset = mfsp->next_offset;
  mfsp->tr_nops        = 0U;
  mfsp->tr_limit_offset = mfsp->tr_next_offset + tspace;

  return MFS_NO_ERROR;
}

/**
 * @brief   A transaction is committed and finalized atomically.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @return              The operation status.
 * @retval MFS_NO_ERROR             if the operation has been successfully
 *                                  completed.
 * @retval MFS_ERR_INV_STATE        if the driver is in not in @p MFS_TRANSACTION
 *                                  state.
 * @retval MFS_ERR_FLASH_FAILURE    if the flash memory is unusable because HW
 *                                  failures. Makes the driver enter the
 *                                  @p MFS_ERROR state.
 * @retval MFS_ERR_INTERNAL         if an internal logic failure is detected.
 *
 * @api
 */
mfs_error_t mfsCommitTransaction(MFSDriver *mfsp) {
  mfs_transaction_op_t *top;

  osalDbgCheck(mfsp != NULL);

  /* The driver must be in transaction mode.*/
  if (mfsp->state != MFS_TRANSACTION) {
    return MFS_ERR_INV_STATE;
  }

  /* Scanning all buffered operations in reverse order.*/
  mfsp->ncbuf->dhdr.fields.magic1 = (uint32_t)MFS_HEADER_MAGIC_1;
  mfsp->ncbuf->dhdr.fields.magic2 = (uint32_t)MFS_HEADER_MAGIC_2;
  top = &mfsp->tr_ops[mfsp->tr_nops];
  while (top > &mfsp->tr_ops[0]) {
    /* On the previous element.*/
    top--;

    /* Finalizing the operation by writing the magic number.*/
    RET_ON_ERROR(mfs_flash_write(mfsp,
                                 top->offset,
                                 sizeof (uint32_t) * 2U,
                                 mfsp->ncbuf->data8));
  }

  /* Transaction fully committed by writing the last (first in transaction)
     magic number, now updating the internal state using the buffered data.*/
  mfsp->next_offset = mfsp->tr_next_offset;
  while (top < &mfsp->tr_ops[mfsp->tr_nops]) {
    unsigned i = (unsigned)top->id - 1U;

    /* The calculation is a bit different depending on write or erase record
       operations.*/
    if (top->size > 0U) {
      /* It is a write.*/
      if (mfsp->descriptors[i].offset != 0U) {
        /* The size of the old record instance, if present, must be subtracted
           to the total used size.*/
        mfsp->used_space -= ALIGNED_REC_SIZE(mfsp->descriptors[i].size);
      }

      /* Adjusting bank-related metadata.*/
      mfsp->used_space           += ALIGNED_REC_SIZE(top->size);
      mfsp->descriptors[i].offset = top->offset;
      mfsp->descriptors[i].size   = top->size;
    }
    else {
      /* It is an erase.*/
      mfsp->used_space           -= ALIGNED_REC_SIZE(mfsp->descriptors[i].size);
      mfsp->descriptors[i].offset = 0U;
      mfsp->descriptors[i].size   = 0U;
    }

    /* On the next element.*/
    top++;
  }

  /* Returning to ready mode.*/
  mfsp->state = MFS_READY;

  return MFS_NO_ERROR;
}

/**
 * @brief   A transaction is rolled back atomically.
 * @details This function performs a garbage collection in order to discard
 *          all written data that has not been finalized.
 *
 * @param[in] mfsp      pointer to the @p MFSDriver object
 * @return              The operation status.
 * @retval MFS_NO_ERROR             if the operation has been successfully
 *                                  completed.
 * @retval MFS_ERR_INV_STATE        if the driver is in not in @p MFS_TRANSACTION
 *                                  state.
 * @retval MFS_ERR_FLASH_FAILURE    if the flash memory is unusable because HW
 *                                  failures. Makes the driver enter the
 *                                  @p MFS_ERROR state.
 * @retval MFS_ERR_INTERNAL         if an internal logic failure is detected.
 *
 * @api
 */
mfs_error_t mfsRollbackTransaction(MFSDriver *mfsp) {
  mfs_error_t err;

  osalDbgCheck(mfsp != NULL);

  if (mfsp->state != MFS_TRANSACTION) {
    return MFS_ERR_INV_STATE;
  }

  /* Returning to ready mode.*/
  mfsp->state = MFS_READY;

  /* If no operations have been performed then there is no need to perform
     a garbage collection.*/
  if (mfsp->tr_nops > 0U) {
    err = mfs_garbage_collect(mfsp);
  }
  else {
    err = MFS_NO_ERROR;
  }

  return err;
}
#endif /* MFS_CFG_TRANSACTION_MAX > 0 */

/** @} */
