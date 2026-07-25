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
 * @file    memstreams.c
 * @brief   Memory streams code.
 *
 * @addtogroup HAL_MEMORY_STREAMS
 * @details Memory buffers handled as streams.
 * @{
 */

#include <string.h>

#include "hal.h"
#include "memstreams.h"

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
  MemoryStream *msp = ip;

  if (msp->size - msp->eos < n)
    n = msp->size - msp->eos;
  memcpy(msp->buffer + msp->eos, bp, n);
  msp->eos += n;
  return n;
}

static size_t _reads(void *ip, uint8_t *bp, size_t n) {
  MemoryStream *msp = ip;

  if (msp->eos - msp->offset < n)
    n = msp->eos - msp->offset;
  memcpy(bp, msp->buffer + msp->offset, n);
  msp->offset += n;
  return n;
}

static msg_t _put(void *ip, uint8_t b) {
  MemoryStream *msp = ip;

  if (msp->size - msp->eos <= 0)
    return MSG_RESET;
  *(msp->buffer + msp->eos) = b;
  msp->eos += 1;
  return MSG_OK;
}

static msg_t _get(void *ip) {
  uint8_t b;
  MemoryStream *msp = ip;

  if (msp->eos - msp->offset <= 0)
    return MSG_RESET;
  b = *(msp->buffer + msp->offset);
  msp->offset += 1;
  return b;
}

static msg_t _unget(void* ip, uint8_t b)
{
  MemoryStream* msp = ip;

  if (msp->offset <= 0)
    return MSG_RESET;
  msp->offset -= 1;
  *(msp->buffer + msp->offset) = b;

  return MSG_OK;
}

static const struct MemStreamVMT vmt = {(size_t)0, _writes, _reads, _put, _get, _unget};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Memory stream object initialization.
 *
 * @param[out] msp      pointer to the @p MemoryStream object to be initialized
 * @param[in] buffer    pointer to the memory buffer for the memory stream
 * @param[in] size      total size of the memory stream buffer
 * @param[in] eos       initial End Of Stream offset. Normally you need to
 *                      put this to zero for RAM buffers or equal to @p size
 *                      for ROM streams.
 */
void msObjectInit(MemoryStream *msp, uint8_t *buffer,
                  size_t size, size_t eos) {

  msp->vmt    = &vmt;
  msp->buffer = buffer;
  msp->size   = size;
  msp->eos    = eos;
  msp->offset = 0;
}

/** @} */
