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
 * @file    oslib/src/chmempools.c
 * @brief   Memory Pools code.
 *
 * @addtogroup oslib_mempools
 * @details Memory Pools related APIs and services.
 *          <h2>Operation mode</h2>
 *          The Memory Pools APIs allow to allocate/free fixed size objects in
 *          <b>constant time</b> and reliably without memory fragmentation
 *          problems.<br>
 *          Memory Pools do not enforce any alignment constraint on the
 *          contained object however the objects must be properly aligned
 *          to contain a pointer to void.
 * @pre     In order to use the memory pools APIs the @p CH_CFG_USE_MEMPOOLS option
 *          must be enabled in @p chconf.h.
 * @note    Compatible with RT and NIL.
 * @{
 */

#include "ch.h"

#if (CH_CFG_USE_MEMPOOLS == TRUE) || defined(__DOXYGEN__)

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

/**
 * @brief   Initializes an empty memory pool.
 *
 * @param[out] mp       pointer to a @p memory_pool_t structure
 * @param[in] size      the size of the objects contained in this memory pool,
 *                      the minimum accepted size is the size of a pointer to
 *                      void.
 * @param[in] align     required memory alignment
 * @param[in] provider  memory provider function for the memory pool or
 *                      @p NULL if the pool is not allowed to grow
 *                      automatically
 *
 * @init
 */
void chPoolObjectInitAligned(memory_pool_t *mp, size_t size,
                             unsigned align, memgetfunc_t provider) {

  chDbgCheck((mp != NULL) &&
             (size >= sizeof(void *)) &&
             (align >= PORT_NATURAL_ALIGN) &&
             MEM_IS_VALID_ALIGNMENT(align));

  mp->next = NULL;
  mp->object_size = size;
  mp->align = align;
  mp->provider = provider;
}

/**
 * @brief   Loads a memory pool with an array of static objects.
 * @pre     The memory pool must already be initialized.
 * @pre     The array elements must be of the right size for the specified
 *          memory pool.
 * @pre     The array elements size must be a multiple of the alignment
 *          requirement for the pool.
 * @post    The memory pool contains the elements of the input array.
 *
 * @param[in] mp        pointer to a @p memory_pool_t structure
 * @param[in] p         pointer to the array first element
 * @param[in] n         number of elements in the array
 *
 * @api
 */
void chPoolLoadArray(memory_pool_t *mp, void *p, size_t n) {

  chDbgCheck((mp != NULL) && (n != 0U));

  while (n != 0U) {
    chPoolAdd(mp, p);
    /*lint -save -e9087 [11.3] Safe cast.*/
    p = (void *)(((uint8_t *)p) + mp->object_size);
    /*lint -restore*/
    n--;
  }
}

/**
 * @brief   Allocates an object from a memory pool.
 * @pre     The memory pool must already be initialized.
 *
 * @param[in] mp        pointer to a @p memory_pool_t structure
 * @return              The pointer to the allocated object.
 * @retval NULL         if pool is empty.
 *
 * @iclass
 */
void *chPoolAllocI(memory_pool_t *mp) {
  void *objp;

  chDbgCheckClassI();
  chDbgCheck(mp != NULL);

  objp = mp->next;
  /*lint -save -e9013 [15.7] There is no else because it is not needed.*/
  if (objp != NULL) {
    mp->next = mp->next->next;
  }
  else if (mp->provider != NULL) {
    objp = mp->provider(mp->object_size, mp->align);

    chDbgAssert(MEM_IS_ALIGNED(objp, mp->align),
                "returned object not aligned");
  }
  /*lint -restore*/

  return objp;
}

/**
 * @brief   Allocates an object from a memory pool.
 * @pre     The memory pool must already be initialized.
 *
 * @param[in] mp        pointer to a @p memory_pool_t structure
 * @return              The pointer to the allocated object.
 * @retval NULL         if pool is empty.
 *
 * @api
 */
void *chPoolAlloc(memory_pool_t *mp) {
  void *objp;

  chSysLock();
  objp = chPoolAllocI(mp);
  chSysUnlock();

  return objp;
}

/**
 * @brief   Releases an object into a memory pool.
 * @pre     The memory pool must already be initialized.
 * @pre     The freed object must be of the right size for the specified
 *          memory pool.
 * @pre     The added object must be properly aligned.
 *
 * @param[in] mp        pointer to a @p memory_pool_t structure
 * @param[in] objp      the pointer to the object to be released
 *
 * @iclass
 */
void chPoolFreeI(memory_pool_t *mp, void *objp) {
  struct pool_header *php = objp;

  chDbgCheckClassI();
  chDbgCheck((mp != NULL) &&
             (objp != NULL) &&
             MEM_IS_ALIGNED(objp, mp->align));

  php->next = mp->next;
  mp->next = php;
}

/**
 * @brief   Releases an object into a memory pool.
 * @pre     The memory pool must already be initialized.
 * @pre     The freed object must be of the right size for the specified
 *          memory pool.
 * @pre     The added object must be properly aligned.
 *
 * @param[in] mp        pointer to a @p memory_pool_t structure
 * @param[in] objp      the pointer to the object to be released
 *
 * @api
 */
void chPoolFree(memory_pool_t *mp, void *objp) {

  chSysLock();
  chPoolFreeI(mp, objp);
  chSysUnlock();
}

#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Initializes an empty guarded memory pool.
 *
 * @param[out] gmp      pointer to a @p guarded_memory_pool_t structure
 * @param[in] size      the size of the objects contained in this guarded
 *                      memory pool, the minimum accepted size is the size
 *                      of a pointer to void.
 * @param[in] align     required memory alignment
 *
 * @init
 */
void chGuardedPoolObjectInitAligned(guarded_memory_pool_t *gmp,
                                    size_t size,
                                    unsigned align) {

  chPoolObjectInitAligned(&gmp->pool, size, align, NULL);
  chSemObjectInit(&gmp->sem, (cnt_t)0);
}

/**
 * @brief   Loads a guarded memory pool with an array of static objects.
 * @pre     The guarded memory pool must already be initialized.
 * @pre     The array elements must be of the right size for the specified
 *          guarded memory pool.
 * @post    The guarded memory pool contains the elements of the input array.
 *
 * @param[in] gmp       pointer to a @p guarded_memory_pool_t structure
 * @param[in] p         pointer to the array first element
 * @param[in] n         number of elements in the array
 *
 * @api
 */
void chGuardedPoolLoadArray(guarded_memory_pool_t *gmp, void *p, size_t n) {

  chDbgCheck((gmp != NULL) && (n != 0U));

  while (n != 0U) {
    chGuardedPoolAdd(gmp, p);
    /*lint -save -e9087 [11.3] Safe cast.*/
    p = (void *)(((uint8_t *)p) + gmp->pool.object_size);
    /*lint -restore*/
    n--;
  }
}

/**
 * @brief   Allocates an object from a guarded memory pool.
 * @pre     The guarded memory pool must already be initialized.
 *
 * @param[in] gmp       pointer to a @p guarded_memory_pool_t structure
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The pointer to the allocated object.
 * @retval NULL         if the operation timed out.
 *
 * @sclass
 */
void *chGuardedPoolAllocTimeoutS(guarded_memory_pool_t *gmp,
                                 sysinterval_t timeout) {
  msg_t msg;

  msg = chSemWaitTimeoutS(&gmp->sem, timeout);
  if (msg != MSG_OK) {
    return NULL;
  }

  return chPoolAllocI(&gmp->pool);
}

/**
 * @brief   Allocates an object from a guarded memory pool.
 * @pre     The guarded memory pool must already be initialized.
 *
 * @param[in] gmp       pointer to a @p guarded_memory_pool_t structure
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The pointer to the allocated object.
 * @retval NULL         if the operation timed out.
 *
 * @api
 */
void *chGuardedPoolAllocTimeout(guarded_memory_pool_t *gmp,
                                sysinterval_t timeout) {
  void *p;

  chSysLock();
  p = chGuardedPoolAllocTimeoutS(gmp, timeout);
  chSysUnlock();

  return p;
}

/**
 * @brief   Releases an object into a guarded memory pool.
 * @pre     The guarded memory pool must already be initialized.
 * @pre     The freed object must be of the right size for the specified
 *          guarded memory pool.
 * @pre     The added object must be properly aligned.
 *
 * @param[in] gmp       pointer to a @p guarded_memory_pool_t structure
 * @param[in] objp      the pointer to the object to be released
 *
 * @api
 */
void chGuardedPoolFree(guarded_memory_pool_t *gmp, void *objp) {

  chSysLock();
  chGuardedPoolFreeI(gmp, objp);
  chSchRescheduleS();
  chSysUnlock();
}
#endif

#endif /* CH_CFG_USE_MEMPOOLS == TRUE */

/** @} */
