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
 * @file    hal_buffered_sio.c
 * @brief   Buffered SIO Driver code.
 *
 * @addtogroup HAL_BUFFERED_SIO
 * @{
 */

#include "hal.h"

#if (HAL_USE_SIO == TRUE) || defined(__DOXYGEN__)

#include "hal_buffered_sio.h"

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

static void __bsio_push_data(BufferedSIODriver *bsiop) {

  while (!sioIsTXFullX(bsiop->siop)) {
    msg_t msg;

    msg = oqGetI(&bsiop->oqueue);
    if (msg < MSG_OK) {
      chnAddFlagsI((BufferedSerial *)bsiop, CHN_OUTPUT_EMPTY);
      return;
    }
    sioPutX(bsiop->siop, (uint_fast16_t)msg);
  }
}

static void __bsio_pop_data(BufferedSIODriver *bsiop) {

  /* RX FIFO needs to be fully emptied or SIO will not generate more RX FIFO
     events.*/
  while (!sioIsRXEmptyX(bsiop->siop)) {
    bsIncomingDataI((BufferedSerial *)bsiop, sioGetX(bsiop->siop));
  }
}

static void __bsio_default_cb(SIODriver *siop) {
  BufferedSIODriver *bsiop = (BufferedSIODriver *)siop->arg;
  sioevents_t events;

  if (bsiop == NULL) {
    return;
  }

  osalSysLockFromISR();

  /* Drain/fill FIFOs before re-enabling data interrupts in the LLD.
     NOTE: this assumes status/error flags are not cleared by data reads,
     otherwise non-data events could be lost before sioGetAndClearEventsX(). */
  if (!sioIsRXEmptyX(siop)) {
    __bsio_pop_data(bsiop);
  }
  if (!sioIsTXFullX(siop)) {
    __bsio_push_data(bsiop);
  }

  /* Posting the non-data SIO events as channel event flags, the masks are
     made to match.*/
  events = sioGetAndClearEventsX(siop);
  chnAddFlagsI(bsiop, (eventflags_t)(events & ~SIO_EV_ALL_DATA));

  osalSysUnlockFromISR();
}

static void __bsio_onotify(io_queue_t *qp) {

  __bsio_push_data((BufferedSIODriver *)qp->q_link);
}

/*
 * Interface implementation.
 */
static size_t __bsio_write(void *ip, const uint8_t *bp, size_t n) {

  return __buffered_serial_write_impl(ip, bp, n);
}

static size_t __bsio_read(void *ip, uint8_t *bp, size_t n) {

  return __buffered_serial_read_impl(ip, bp, n);
}

static msg_t __bsio_put(void *ip, uint8_t b) {

  return __buffered_serial_put_impl(ip, b);
}

static msg_t __bsio_get(void *ip) {

  return __buffered_serial_get_impl(ip);
}

static msg_t __bsio_putt(void *ip, uint8_t b, sysinterval_t timeout) {

  return __buffered_serial_put_timeout_impl(ip, b, timeout);
}

static msg_t __bsio_gett(void *ip, sysinterval_t timeout) {

  return __buffered_serial_get_timeout_impl(ip, timeout);
}

static size_t __bsio_writet(void *ip, const uint8_t *bp, size_t n,
                            sysinterval_t timeout) {

  return __buffered_serial_write_timeout_impl(ip, bp, n, timeout);
}

static size_t __bsio_readt(void *ip, uint8_t *bp, size_t n,
                           sysinterval_t timeout) {

  return __buffered_serial_read_timeout_impl(ip, bp, n, timeout);
}

static msg_t __bsio_ctl(void *ip, unsigned int operation, void *arg) {

  return sioControlX(((BufferedSIODriver *)ip)->siop, operation, arg);
}

static const struct BufferedSIODriverVMT vmt = {
  (size_t)0,
  __bsio_write, __bsio_read, __bsio_put,    __bsio_get,
  __bsio_putt,  __bsio_gett, __bsio_writet, __bsio_readt,
  __bsio_ctl
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes a generic serial driver object.
 * @details The HW dependent part of the initialization has to be performed
 *          outside, usually in the hardware initialization code.
 *
 * @param[out] bsiop     pointer to a @p BufferedSIODriver structure
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] ib        pointer to the input buffer
 * @param[in] ibsize    size of the input buffer
 * @param[in] ob        pointer to the output buffer
 * @param[in] obsize    size of the output buffer
 *
 * @init
 */
void bsioObjectInit(BufferedSIODriver *bsiop, SIODriver *siop,
                   uint8_t *ib, size_t ibsize,
                   uint8_t *ob, size_t obsize) {

  __buffered_serial_objinit_impl((void *)bsiop, (const void *)&vmt,
                                 ib, ibsize, NULL, NULL,
                                 ob, obsize, __bsio_onotify, (void *)bsiop);
  bsiop->siop = siop;
  siop->arg   = (void *)bsiop;
}

/**
 * @brief   Configures and starts the driver.
 *
 * @param[out] bsiop     pointer to a @p BufferedSIODriver structure
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 * @return              The operation status.
 *
 * @api
 */
msg_t bsioStart(BufferedSIODriver *bsiop, const BufferedSIOConfig *config) {
  msg_t msg;

  osalDbgCheck(bsiop != NULL);

  osalDbgAssert((bsiop->state == BS_STOP) || (bsiop->state == BS_READY),
                "invalid state");

  msg = sioStart(bsiop->siop, config);
  if (msg == HAL_RET_SUCCESS) {
    osalSysLock();
    bsiop->siop->arg = (void *)bsiop;
    sioSetCallbackX(bsiop->siop, &__bsio_default_cb);
    sioWriteEnableFlagsX(bsiop->siop, SIO_EV_ALL_EVENTS);
    bsiop->state = BS_READY;
    osalSysUnlock();
  }
  else {
    bsiop->state = BS_STOP;
  }

  return msg;
}

/**
 * @brief   Stops the driver.
 * @details Any thread waiting on the driver's queues will be awakened with
 *          the message @p MSG_RESET.
 *
 * @param[out] bsiop     pointer to a @p BufferedSIODriver structure
 *
 * @api
 */
void bsioStop(BufferedSIODriver *bsiop) {

  osalDbgCheck(bsiop != NULL);

  osalDbgAssert((bsiop->state == BS_STOP) || (bsiop->state == BS_READY),
                "invalid state");

  /* Stopping undelying SIO driver.*/
  sioStop(bsiop->siop);

  bsiop->state = BS_STOP;

  osalSysLock();
  oqResetI(&bsiop->oqueue); /* TODO should go in the upper class.*/
  iqResetI(&bsiop->iqueue);
  osalOsRescheduleS();
  osalSysUnlock();
}

#endif /* HAL_USE_SIO == TRUE */

/** @} */
