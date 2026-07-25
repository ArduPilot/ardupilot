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
 * @file    nullstreams.c
 * @brief   Null streams code.
 *
 * @addtogroup HAL_NULL_STREAMS
 * @details A null streams.
 * @{
 */

#include "hal.h"
#include "nullstreams.h"

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

static size_t writes(void *ip, const uint8_t *bp, size_t n) {

  (void)ip;
  (void)bp;

  return n;
}

static size_t reads(void *ip, uint8_t *bp, size_t n) {

  (void)ip;
  (void)bp;
  (void)n;

  return 0;
}

static msg_t put(void *ip, uint8_t b) {

  (void)ip;
  (void)b;

  return MSG_OK;
}

static msg_t get(void *ip) {

  (void)ip;

  return 4;
}

static msg_t unget(void* ip, uint8_t b)
{

  (void)ip;
  (void)b;

  return MSG_OK;
}

static const struct NullStreamVMT vmt = {(size_t)0, writes, reads, put, get, unget};

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Null stream object initialization.
 *
 * @param[out] nsp      pointer to the @p NullStream object to be initialized
 */
void nullObjectInit(NullStream *nsp) {

  nsp->vmt = &vmt;
}

/** @} */
