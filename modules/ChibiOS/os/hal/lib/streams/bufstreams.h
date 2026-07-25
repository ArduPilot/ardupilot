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
 * @file    bufstreams.h
 * @brief   Buffered streams structures and macros.

 * @addtogroup HAL_BUFFERED_STREAMS
 * @{
 */

#ifndef BUFSTREAMS_H
#define BUFSTREAMS_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Buffer size for unget.
 */
#if !defined(BUFSTREAM_BUFFER_SIZE) || defined(__DOXYGEN__)
#define BUFSTREAM_BUFFER_SIZE           1
#endif

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   @p BufferedStreamAdapter specific data.
 */
#define _buffered_stream_adapter_data                                       \
  _base_buffered_stream_data                                                \
  /* Pointer to a wrapped BaseSequentialStream object */                    \
  BaseSequentialStream* bssp;                                               \
  /* Stream buffer */                                                       \
  uint8_t               buffer[BUFSTREAM_BUFFER_SIZE];                      \
  /* Index in the stream buffer */                                          \
  size_t                ndx;

/**
 * @brief   @p BufferedStreamAdapter virtual methods table.
 */
struct BufferedStreamAdapterVMT {
  _base_buffered_stream_methods
};

/**
 * @extends BufferedStream
 *
 * @brief Buffered stream adapter object.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BufferedStreamAdapterVMT *vmt;
  _buffered_stream_adapter_data
} BufferedStreamAdapter;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void bsaObjectInit(BufferedStreamAdapter *bsap, BaseSequentialStream* bssp);
#ifdef __cplusplus
}
#endif

#endif /* MEMSTREAMS_H */

/** @} */
