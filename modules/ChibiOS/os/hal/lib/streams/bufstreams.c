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

/*
    This file was contributed by Alex Lewontin.
 */

/**
 * @file    bufstreams.c
 * @brief   Buffered streams code.
 *
 * @addtogroup HAL_BUFFERED_STREAMS
 * @details Wrapper for BaseSequentialStreams that allows some ungetting.
 * @{
 */

#include <string.h>

#include "hal.h"
#include "bufstreams.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static size_t _writes(void *ip, const uint8_t *bp, size_t n) {
  BufferedStreamAdapter *bsap = ip;
  return bsap->bssp->vmt->write(bsap->bssp, bp, n);
}

static size_t _reads(void *ip, uint8_t *bp, size_t n) {
  BufferedStreamAdapter* bsap = ip;
  size_t          buffered;

  if (n < (BUFSTREAM_BUFFER_SIZE - bsap->ndx)) {
    buffered = n;
  } else {
    buffered = BUFSTREAM_BUFFER_SIZE - bsap->ndx;
  }

  memcpy(bp, bsap->buffer + (BUFSTREAM_BUFFER_SIZE - buffered), buffered);
  bsap->ndx += buffered;

  return buffered + bsap->bssp->vmt->read(bsap->bssp, bp + buffered, n - buffered);
}

static msg_t _put(void *ip, uint8_t b) {
  BufferedStreamAdapter* bsap = ip;
  return bsap->bssp->vmt->put(bsap->bssp, b);
}

static msg_t _get(void *ip) {
  BufferedStreamAdapter* bsap = ip;

  if (bsap->ndx == BUFSTREAM_BUFFER_SIZE) {
    return bsap->bssp->vmt->get(bsap->bssp);
  }

  return bsap->buffer[bsap->ndx++];
}

static msg_t _unget(void* ip, uint8_t b) {
  BufferedStreamAdapter* bsap = ip;

  if (((int8_t)b == STM_RESET) || (bsap->ndx == 0)) {
    return STM_RESET;
  }

  bsap->buffer[--(bsap->ndx)] = b;

  return b;
}

static const struct BufferedStreamAdapterVMT vmt = {
  (size_t)0, _writes, _reads, _put, _get, _unget
};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Buffered stream adapter object initialization.
 *
 * @param[out] bsap      pointer to the @p BufferedStream object to be
 *                       initialized
 * @param[in] bssp       pointer to a @p BaseSequentialStream fulfilling
 *                       object to be wrapped
 */
void bsaObjectInit(BufferedStreamAdapter *bsap, BaseSequentialStream* bssp) {

  bsap->vmt    = &vmt;
  bsap->bssp   = bssp;
  memset(bsap->buffer, STM_RESET, BUFSTREAM_BUFFER_SIZE);
  bsap->ndx    = BUFSTREAM_BUFFER_SIZE;
}

/** @} */
