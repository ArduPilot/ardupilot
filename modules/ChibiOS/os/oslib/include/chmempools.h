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
 * @file    oslib/include/chmempools.h
 * @brief   Memory Pools macros and structures.
 *
 * @addtogroup oslib_mempools
 * @{
 */

#ifndef CHMEMPOOLS_H
#define CHMEMPOOLS_H

#if (CH_CFG_USE_MEMPOOLS == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if CH_CFG_USE_MEMCORE == FALSE
#error "CH_CFG_USE_MEMPOOLS requires CH_CFG_USE_MEMCORE"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Memory pool free object header.
 */
struct pool_header {
  struct pool_header    *next;          /**< @brief Pointer to the next pool
                                                    header in the list.     */
};

/**
 * @brief   Memory pool descriptor.
 */
typedef struct {
  struct pool_header    *next;          /**< @brief Pointer to the header.  */
  size_t                object_size;    /**< @brief Memory pool objects
                                                    size.                   */
  unsigned              align;          /**< @brief Required alignment.     */
  memgetfunc_t          provider;       /**< @brief Memory blocks provider
                                                    for this pool.          */
} memory_pool_t;

#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Guarded memory pool descriptor.
 */
typedef struct {
  semaphore_t           sem;            /**< @brief Counter semaphore guarding
                                                    the memory pool.        */
  memory_pool_t         pool;           /**< @brief The memory pool itself. */
} guarded_memory_pool_t;
#endif /* CH_CFG_USE_SEMAPHORES == TRUE */

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Data part of a static memory pool initializer.
 * @details This macro should be used when statically initializing a
 *          memory pool that is part of a bigger structure.
 *
 * @param[in] name      the name of the memory pool variable
 * @param[in] size      size of the memory pool contained objects
 * @param[in] align     required memory alignment
 * @param[in] provider  memory provider function for the memory pool
 */
#define __MEMORYPOOL_DATA(name, size, align, provider)                      \
  {NULL, size, align, provider}

/**
 * @brief   Static memory pool initializer.
 * @details Statically initialized memory pools require no explicit
 *          initialization using @p chPoolInit().
 *
 * @param[in] name      the name of the memory pool variable
 * @param[in] size      size of the memory pool contained objects
 * @param[in] align     required memory alignment
 * @param[in] provider  memory provider function for the memory pool or @p NULL
 *                      if the pool is not allowed to grow automatically
 */
#define MEMORYPOOL_DECL(name, size, align, provider)                        \
  memory_pool_t name = __MEMORYPOOL_DATA(name, size, align, provider)

#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Data part of a static guarded memory pool initializer.
 * @details This macro should be used when statically initializing a
 *          memory pool that is part of a bigger structure.
 *
 * @param[in] name      the name of the memory pool variable
 * @param[in] size      size of the memory pool contained objects
 * @param[in] align     required memory alignment
 */
#define __GUARDEDMEMORYPOOL_DATA(name, size, align) {                       \
  __SEMAPHORE_DATA(name.sem, (cnt_t)0),                                     \
  __MEMORYPOOL_DATA(NULL, size, align, NULL)                                \
}

/**
 * @brief   Static guarded memory pool initializer.
 * @details Statically initialized guarded memory pools require no explicit
 *          initialization using @p chGuardedPoolInit().
 *
 * @param[in] name      the name of the guarded memory pool variable
 * @param[in] size      size of the memory pool contained objects
 * @param[in] align     required memory alignment
 */
#define GUARDEDMEMORYPOOL_DECL(name, size, align)                           \
  guarded_memory_pool_t name = __GUARDEDMEMORYPOOL_DATA(name, size, align)
#endif /* CH_CFG_USE_SEMAPHORES == TRUE */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void chPoolObjectInitAligned(memory_pool_t *mp, size_t size,
                               unsigned align, memgetfunc_t provider);
  void chPoolLoadArray(memory_pool_t *mp, void *p, size_t n);
  void *chPoolAllocI(memory_pool_t *mp);
  void *chPoolAlloc(memory_pool_t *mp);
  void chPoolFreeI(memory_pool_t *mp, void *objp);
  void chPoolFree(memory_pool_t *mp, void *objp);
#if CH_CFG_USE_SEMAPHORES == TRUE
  void chGuardedPoolObjectInitAligned(guarded_memory_pool_t *gmp,
                                      size_t size,
                                      unsigned align);
  void chGuardedPoolLoadArray(guarded_memory_pool_t *gmp, void *p, size_t n);
  void *chGuardedPoolAllocTimeoutS(guarded_memory_pool_t *gmp,
                                   sysinterval_t timeout);
  void *chGuardedPoolAllocTimeout(guarded_memory_pool_t *gmp,
                                  sysinterval_t timeout);
  void chGuardedPoolFree(guarded_memory_pool_t *gmp, void *objp);
#endif
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Initializes an empty memory pool.
 *
 * @param[out] mp       pointer to a @p memory_pool_t structure
 * @param[in] size      the size of the objects contained in this memory pool,
 *                      the minimum accepted size is the size of a pointer to
 *                      void.
 * @param[in] provider  memory provider function for the memory pool or
 *                      @p NULL if the pool is not allowed to grow
 *                      automatically
 *
 * @init
 */
static inline void chPoolObjectInit(memory_pool_t *mp,
                                    size_t size,
                                    memgetfunc_t provider) {

  chPoolObjectInitAligned(mp, size, PORT_NATURAL_ALIGN, provider);
}

/**
 * @brief   Adds an object to a memory pool.
 * @pre     The memory pool must be already been initialized.
 * @pre     The added object must be of the right size for the specified
 *          memory pool.
 * @pre     The added object must be properly aligned.
 * @note    This function is just an alias for @p chPoolFree() and has been
 *          added for clarity.
 *
 * @param[in] mp        pointer to a @p memory_pool_t structure
 * @param[in] objp      the pointer to the object to be added
 *
 * @api
 */
static inline void chPoolAdd(memory_pool_t *mp, void *objp) {

  chPoolFree(mp, objp);
}

/**
 * @brief   Adds an object to a memory pool.
 * @pre     The memory pool must be already been initialized.
 * @pre     The added object must be of the right size for the specified
 *          memory pool.
 * @pre     The added object must be properly aligned.
 * @note    This function is just an alias for @p chPoolFreeI() and has been
 *          added for clarity.
 *
 * @param[in] mp        pointer to a @p memory_pool_t structure
 * @param[in] objp      the pointer to the object to be added
 *
 * @iclass
 */
static inline void chPoolAddI(memory_pool_t *mp, void *objp) {

  chPoolFreeI(mp, objp);
}

#if (CH_CFG_USE_SEMAPHORES == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Initializes an empty guarded memory pool.
 *
 * @param[out] gmp      pointer to a @p guarded_memory_pool_t structure
 * @param[in] size      the size of the objects contained in this guarded
 *                      memory pool, the minimum accepted size is the size
 *                      of a pointer to void.
 *
 * @init
 */
static inline void chGuardedPoolObjectInit(guarded_memory_pool_t *gmp,
                                           size_t size) {

  chGuardedPoolObjectInitAligned(gmp, size, PORT_NATURAL_ALIGN);
}

/**
 * @brief   Gets the count of objects in a guarded memory pool.
 * @pre     The guarded memory pool must be already been initialized.
 *
 * @param[in] gmp       pointer to a @p guarded_memory_pool_t structure
 * @return              The counter of the guard semaphore.
 *
 * @iclass
 */
static inline cnt_t chGuardedPoolGetCounterI(guarded_memory_pool_t *gmp) {

  return chSemGetCounterI(&gmp->sem);
}

/**
 * @brief   Allocates an object from a guarded memory pool.
 * @pre     The guarded memory pool must be already been initialized.
 *
 * @param[in] gmp       pointer to a @p guarded_memory_pool_t structure
 * @return              The pointer to the allocated object.
 * @retval NULL         if the pool is empty.
 *
 * @iclass
 */
static inline void *chGuardedPoolAllocI(guarded_memory_pool_t *gmp) {
  void *p;

  if (chSemGetCounterI(&gmp->sem) > (cnt_t)0) {

    chSemFastWaitI(&gmp->sem);
    p = chPoolAllocI(&gmp->pool);
  }
  else {
    p = NULL;
  }

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
 * @iclass
 */
static inline void chGuardedPoolFreeI(guarded_memory_pool_t *gmp, void *objp) {

  chPoolFreeI(&gmp->pool, objp);
  chSemSignalI(&gmp->sem);
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
 * @sclass
 */
static inline void chGuardedPoolFreeS(guarded_memory_pool_t *gmp, void *objp) {

  chGuardedPoolFreeI(gmp, objp);
  chSchRescheduleS();
}

/**
 * @brief   Adds an object to a guarded memory pool.
 * @pre     The guarded memory pool must be already been initialized.
 * @pre     The added object must be of the right size for the specified
 *          guarded memory pool.
 * @pre     The added object must be properly aligned.
 * @note    This function is just an alias for @p chGuardedPoolFree() and
 *          has been added for clarity.
 *
 * @param[in] gmp       pointer to a @p guarded_memory_pool_t structure
 * @param[in] objp      the pointer to the object to be added
 *
 * @api
 */
static inline void chGuardedPoolAdd(guarded_memory_pool_t *gmp, void *objp) {

  chGuardedPoolFree(gmp, objp);
}

/**
 * @brief   Adds an object to a guarded memory pool.
 * @pre     The guarded memory pool must be already been initialized.
 * @pre     The added object must be of the right size for the specified
 *          guarded memory pool.
 * @pre     The added object must be properly aligned.
 * @note    This function is just an alias for @p chGuardedPoolFreeI() and
 *          has been added for clarity.
 *
 * @param[in] gmp       pointer to a @p guarded_memory_pool_t structure
 * @param[in] objp      the pointer to the object to be added
 *
 * @iclass
 */
static inline void chGuardedPoolAddI(guarded_memory_pool_t *gmp, void *objp) {

  chGuardedPoolFreeI(gmp, objp);
}

/**
 * @brief   Adds an object to a guarded memory pool.
 * @pre     The guarded memory pool must be already been initialized.
 * @pre     The added object must be of the right size for the specified
 *          guarded memory pool.
 * @pre     The added object must be properly aligned.
 * @note    This function is just an alias for @p chGuardedPoolFreeI() and
 *          has been added for clarity.
 *
 * @param[in] gmp       pointer to a @p guarded_memory_pool_t structure
 * @param[in] objp      the pointer to the object to be added
 *
 * @sclass
 */
static inline void chGuardedPoolAddS(guarded_memory_pool_t *gmp, void *objp) {

  chGuardedPoolFreeS(gmp, objp);
}
#endif /* CH_CFG_USE_SEMAPHORES == TRUE */

#endif /* CH_CFG_USE_MEMPOOLS == TRUE */

#endif /* CHMEMPOOLS_H */

/** @} */
