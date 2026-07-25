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
 * @file    oslib/include/chmemcore.h
 * @brief   Core memory manager macros and structures.
 *
 * @addtogroup oslib_memcore
 * @{
 */

#ifndef CHMEMCORE_H
#define CHMEMCORE_H

#if (CH_CFG_USE_MEMCORE == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Managed RAM size.
 * @details Size of the RAM area to be managed by the OS. If set to zero
 *          then the whole available RAM is used. The core memory is made
 *          available to the heap allocator and/or can be used directly through
 *          the simplified core memory allocator.
 *
 * @note    In order to let the OS manage the whole RAM the linker script must
 *          provide the @p __heap_base__ and @p __heap_end__ symbols.
 * @note    Requires @p CH_CFG_USE_MEMCORE.
 */
#if !defined(CH_CFG_MEMCORE_SIZE) || defined(__DOXYGEN__)
#define CH_CFG_MEMCORE_SIZE                 0
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if CH_CFG_MEMCORE_SIZE < 0
#error "invalid CH_CFG_MEMCORE_SIZE value specified"
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Memory get function.
 */
typedef void *(*memgetfunc_t)(size_t size, unsigned align);

/**
 * @brief   Enhanced memory get function.
 */
typedef void *(*memgetfunc2_t)(size_t size, unsigned align, size_t offset);

/**
 * @brief   Type of memory core object.
 */
typedef struct {
  /**
   * @brief   Next free address.
   */
  uint8_t *basemem;
  /**
   * @brief   Final address.
   */
  uint8_t *topmem;
} memcore_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Allocates a memory block.
 * @note    This is a generic form with unspecified allocation position.
 *
 * @iclass
 */
#define chCoreAllocAlignedWithOffsetI chCoreAllocFromTopI

/**
 * @brief   Allocates a memory block.
 * @note    This is a generic form with unspecified allocation position.
 *
 * @api
 */
#define chCoreAllocAlignedWithOffset chCoreAllocFromTop

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern memcore_t ch_memcore;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void __core_init(void);
  void *chCoreAllocFromBaseI(size_t size, unsigned align, size_t offset);
  void *chCoreAllocFromTopI(size_t size, unsigned align, size_t offset);
  void *chCoreAllocFromBase(size_t size, unsigned align, size_t offset);
  void *chCoreAllocFromTop(size_t size, unsigned align, size_t offset);
  size_t chCoreGetStatusX(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Allocates a memory block.
 * @details The allocated block is guaranteed to be properly aligned to the
 *          specified alignment.
 * @note    This is a generic form with unspecified allocation position.
 *
 * @param[in] size      the size of the block to be allocated.
 * @param[in] align     desired memory alignment
 * @return              A pointer to the allocated memory block.
 * @retval NULL         allocation failed, core memory exhausted.
 *
 * @iclass
 */
static inline void *chCoreAllocAlignedI(size_t size, unsigned align) {

  return chCoreAllocAlignedWithOffsetI(size, align, 0U);
}

/**
 * @brief   Allocates a memory block.
 * @details The allocated block is guaranteed to be properly aligned to the
 *          specified alignment.
 * @note    This is a generic form with unspecified allocation position.
 *
 * @param[in] size      the size of the block to be allocated
 * @param[in] align     desired memory alignment
 * @return              A pointer to the allocated memory block.
 * @retval NULL         allocation failed, core memory exhausted.
 *
 * @api
 */
static inline void *chCoreAllocAligned(size_t size, unsigned align) {

  return chCoreAllocAlignedWithOffset(size, align, 0U);
}

/**
 * @brief   Allocates a memory block.
 * @details The allocated block is guaranteed to be properly aligned for a
 *          pointer data type.
 * @note    This is a generic form with unspecified allocation position.
 *
 * @param[in] size      the size of the block to be allocated.
 * @return              A pointer to the allocated memory block.
 * @retval NULL         allocation failed, core memory exhausted.
 *
 * @iclass
 */
static inline void *chCoreAllocI(size_t size) {

  return chCoreAllocAlignedWithOffsetI(size, PORT_NATURAL_ALIGN, 0U);
}

/**
 * @brief   Allocates a memory block.
 * @details The allocated block is guaranteed to be properly aligned for a
 *          pointer data type.
 * @note    This is a generic form with unspecified allocation position.
 *
 * @param[in] size      the size of the block to be allocated.
 * @return              A pointer to the allocated memory block.
 * @retval NULL         allocation failed, core memory exhausted.
 *
 * @api
 */
static inline void *chCoreAlloc(size_t size) {

  return chCoreAllocAlignedWithOffset(size, PORT_NATURAL_ALIGN, 0U);
}

#endif /* CH_CFG_USE_MEMCORE == TRUE */

#endif /* CHMEMCORE_H */

/** @} */
