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
 * @file    sb/host/sbposix.c
 * @brief   ARM SandBox host Posix API code.
 *
 * @addtogroup ARM_SANDBOX_HOSTAPI
 * @{
 */

#include "ch.h"
#include "sb.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

uint32_t sb_posix_open(const char *pathname, uint32_t flags) {

  (void)pathname;
  (void)flags;

  return SB_ERR_ENOENT;
}

uint32_t sb_posix_close(uint32_t fd) {

  if ((fd == 0U) || (fd == 1U) || (fd == 2U)) {

    return SB_ERR_NOERROR;
  }

  return SB_ERR_EBADFD;
}

uint32_t sb_posix_read(uint32_t fd, uint8_t *buf, size_t count) {
  sb_class_t *sbcp = (sb_class_t *)chThdGetSelfX()->ctx.syscall.p;

  if (!sb_is_valid_write_range(sbcp, (void *)buf, count)) {
    return SB_ERR_EFAULT;
  }

  if (fd == 0U) {
    SandboxStream *ssp = sbcp->config->stdin_stream;

    if ((count == 0U) || (ssp == NULL)) {
      return 0U;
    }

    return (uint32_t)ssp->vmt->read((void *)ssp, buf, count);
  }

  return SB_ERR_EBADFD;
}

uint32_t sb_posix_write(uint32_t fd, const uint8_t *buf, size_t count) {
  sb_class_t *sbcp = (sb_class_t *)chThdGetSelfX()->ctx.syscall.p;

  if (!sb_is_valid_read_range(sbcp, (const void *)buf, count)) {
    return SB_ERR_EFAULT;
  }

  if (fd == 1U) {
    SandboxStream *ssp = sbcp->config->stdout_stream;

    if ((count == 0U) || (ssp == NULL)) {
      return 0U;
    }

    return (uint32_t)ssp->vmt->write((void *)ssp, buf, count);
  }

  if (fd == 2U) {
    SandboxStream *ssp = sbcp->config->stderr_stream;

    if ((count == 0U) || (ssp == NULL)) {
      return 0U;
    }

    return (uint32_t)ssp->vmt->write((void *)ssp, buf, count);
  }

  return SB_ERR_EBADFD;
}

uint32_t sb_posix_lseek(uint32_t fd, uint32_t offset, uint32_t whence) {

  (void)offset;
  (void)whence;

  if ((fd == 0U) || (fd == 1U) || (fd == 2U)) {

    return SB_ERR_ESPIPE;
  }

  return SB_ERR_EBADFD;
}

/** @} */
