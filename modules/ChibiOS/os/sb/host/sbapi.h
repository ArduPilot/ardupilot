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
 * @file    sb/host/sbapi.h
 * @brief   ARM SandBox host API macros and structures.
 *
 * @addtogroup ARM_SANDBOX_HOSTAPI
 * @{
 */

#ifndef SBAPI_H
#define SBAPI_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a syscall handler.
 */
typedef void (*port_syscall_t)(struct port_extctx *ectx);

/**
 * @brief   Sandbox Stream interface methods.
 * @note    Is intentionally compatible with HAL streams but we have to
 *          duplicate is because we don't want dependencies with HAL in
 *          this subsystem.
 */
struct SandboxStreamVMT {
  /**
   * @brief   Object instance offset.
   */
  size_t instance_offset;
  /**
   * @brief   Stream write buffer method.
   */
  size_t (*write)(void *instance, const uint8_t *bp, size_t n);
  /**
   * @brief   Stream read buffer method.
   */
  size_t (*read)(void *instance, uint8_t *bp, size_t n);
  /**
   * @brief   Channel put method, blocking.
   */
  msg_t (*put)(void *instance, uint8_t b);
  /**
   * @brief   Channel get method, blocking.
   */
  msg_t (*get)(void *instance);
};

/**
 * @brief   Sandbox Stream class.
 * @note    Is intentionally compatible with HAL streams but we have to
 *          duplicate is because we don't want dependencies with HAL in
 *          this subsystem.
 */
typedef struct {
  /**
   * @brief Virtual Methods Table.
   */
  const struct SandboxStreamVMT *vmt;
} SandboxStream;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void sb_api_stdio(struct port_extctx *ectxp);
  void sb_api_exit(struct port_extctx *ctxp);
  void sb_api_get_systime(struct port_extctx *ctxp);
  void sb_api_get_frequency(struct port_extctx *ctxp);
  void sb_api_sleep(struct port_extctx *ctxp);
  void sb_api_sleep_until_windowed(struct port_extctx *ctxp);
  void sb_api_wait_message(struct port_extctx *ctxp);
  void sb_api_reply_message(struct port_extctx *ctxp);
  void sb_api_wait_one_timeout(struct port_extctx *ctxp);
  void sb_api_wait_any_timeout(struct port_extctx *ctxp);
  void sb_api_wait_all_timeout(struct port_extctx *ctxp);
  void sb_api_broadcast_flags(struct port_extctx *ctxp);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif /* SBAPI_H */

/** @} */
