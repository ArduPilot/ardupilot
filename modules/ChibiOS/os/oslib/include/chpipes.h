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
 * @file    oslib/include/chpipes.h
 * @brief   Pipes macros and structures.
 *
 * @addtogroup oslib_pipes
 * @{
 */

#ifndef CHPIPES_H
#define CHPIPES_H

#if (CH_CFG_USE_PIPES == TRUE) || defined(__DOXYGEN__)

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
 * @brief   Structure representing a pipe object.
 */
typedef struct {
  uint8_t               *buffer;        /**< @brief Pointer to the pipe
                                                    buffer.                 */
  uint8_t               *top;           /**< @brief Pointer to the location
                                                    after the buffer.       */
  uint8_t               *wrptr;         /**< @brief Write pointer.          */
  uint8_t               *rdptr;         /**< @brief Read pointer.           */
  size_t                cnt;            /**< @brief Bytes in the pipe.      */
  bool                  reset;          /**< @brief True if in reset state. */
  thread_reference_t    wtr;            /**< @brief Waiting writer.         */
  thread_reference_t    rtr;            /**< @brief Waiting reader.         */
#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
  mutex_t               cmtx;           /**< @brief Common access mutex.    */
  mutex_t               wmtx;           /**< @brief Write access mutex.     */
  mutex_t               rmtx;           /**< @brief Read access mutex.      */
#else
  semaphore_t           csem;           /**< @brief Common access semaphore.*/
  semaphore_t           wsem;           /**< @brief Write access semaphore. */
  semaphore_t           rsem;           /**< @brief Read access semaphore.  */
#endif
} pipe_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Data part of a static pipe initializer.
 * @details This macro should be used when statically initializing a
 *          pipe that is part of a bigger structure.
 *
 * @param[in] name      the name of the pipe variable
 * @param[in] buffer    pointer to the pipe buffer array of @p uint8_t
 * @param[in] size      number of @p uint8_t elements in the buffer array
 */
#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
#define __PIPE_DATA(name, buffer, size) {                                   \
  (uint8_t *)(buffer),                                                      \
  (uint8_t *)(buffer) + size,                                               \
  (uint8_t *)(buffer),                                                      \
  (uint8_t *)(buffer),                                                      \
  (size_t)0,                                                                \
  false,                                                                    \
  NULL,                                                                     \
  NULL,                                                                     \
  __MUTEX_DATA(name.cmtx),                                                  \
  __MUTEX_DATA(name.wmtx),                                                  \
  __MUTEX_DATA(name.rmtx),                                                  \
}
#else /* CH_CFG_USE_MUTEXES == FALSE */
#define __PIPE_DATA(name, buffer, size) {                                   \
  (uint8_t *)(buffer),                                                      \
  (uint8_t *)(buffer) + size,                                               \
  (uint8_t *)(buffer),                                                      \
  (uint8_t *)(buffer),                                                      \
  (size_t)0,                                                                \
  false,                                                                    \
  NULL,                                                                     \
  NULL,                                                                     \
  __SEMAPHORE_DATA(name.csem, (cnt_t)1),                                    \
  __SEMAPHORE_DATA(name.wsem, (cnt_t)1),                                    \
  __SEMAPHORE_DATA(name.rsem, (cnt_t)1),                                    \
}
#endif /* CH_CFG_USE_MUTEXES == FALSE */

/**
 * @brief   Static pipe initializer.
 * @details Statically initialized pipes require no explicit
 *          initialization using @p chPipeObjectInit().
 *
 * @param[in] name      the name of the pipe variable
 * @param[in] buffer    pointer to the pipe buffer array of @p uint8_t
 * @param[in] size      number of @p uint8_t elements in the buffer array
 */
#define PIPE_DECL(name, buffer, size)                                       \
  pipe_t name = __PIPE_DATA(name, buffer, size)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void chPipeObjectInit(pipe_t *pp, uint8_t *buf, size_t n);
  void chPipeReset(pipe_t *pp);
  size_t chPipeWriteTimeout(pipe_t *pp, const uint8_t *bp,
                            size_t n, sysinterval_t timeout);
  size_t chPipeReadTimeout(pipe_t *pp, uint8_t *bp,
                           size_t n, sysinterval_t timeout);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Returns the pipe buffer size as number of bytes.
 *
 * @param[in] pp        the pointer to an initialized @p pipe_t object
 * @return              The size of the pipe.
 *
 * @api
 */
static inline size_t chPipeGetSize(const pipe_t *pp) {

  /*lint -save -e9033 [10.8] Perfectly safe pointers
    arithmetic.*/
  return (size_t)(pp->top - pp->buffer);
  /*lint -restore*/
}

/**
 * @brief   Returns the number of used byte slots into a pipe.
 *
 * @param[in] pp        the pointer to an initialized @p pipe_t object
 * @return              The number of queued bytes.
 *
 * @api
 */
static inline size_t chPipeGetUsedCount(const pipe_t *pp) {

  return pp->cnt;
}

/**
 * @brief   Returns the number of free byte slots into a pipe.
 *
 * @param[in] pp        the pointer to an initialized @p pipe_t object
 * @return              The number of empty byte slots.
 *
 * @api
 */
static inline size_t chPipeGetFreeCount(const pipe_t *pp) {

  return chPipeGetSize(pp) - chPipeGetUsedCount(pp);
}

/**
 * @brief   Terminates the reset state.
 *
 * @param[in] pp        the pointer to an initialized @p pipe_t object
 *
 * @api
 */
static inline void chPipeResume(pipe_t *pp) {

  pp->reset = false;
}

#endif /* CH_CFG_USE_PIPES == TRUE */

#endif /* CHPIPES_H */

/** @} */
