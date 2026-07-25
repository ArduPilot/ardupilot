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
 * @file    nullstreams.h
 * @brief   Null streams structures and macros.
 
 * @addtogroup HAL_NULL_STREAMS
 * @{
 */

#ifndef NULLSTREAMS_H
#define NULLSTREAMS_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

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
 * @brief   @p NullStream specific data.
 */
#define _null_stream_data                                                   \
  _base_buffered_stream_data

/**
 * @brief   @p NullStream virtual methods table.
 */
struct NullStreamVMT {
  _base_buffered_stream_methods
};

/**
 * @extends BaseBufferedStream
 *
 * @brief   Null stream object.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct NullStreamVMT *vmt;
  _null_stream_data
} NullStream;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void nullObjectInit(NullStream *nsp);
#ifdef __cplusplus
}
#endif

#endif /* NULLSTREAMS_H */

/** @} */
