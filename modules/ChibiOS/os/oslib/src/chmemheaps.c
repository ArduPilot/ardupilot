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
 * @file    oslib/src/chmemheaps.c
 * @brief   Memory heaps code.
 *
 * @addtogroup oslib_memheaps
 * @details Heap Allocator related APIs.
 *          <h2>Operation mode</h2>
 *          The heap allocator implements a first-fit strategy and its APIs
 *          are functionally equivalent to the usual @p malloc() and @p free()
 *          library functions. The main difference is that the OS heap APIs
 *          are guaranteed to be thread safe and there is the ability to
 *          return memory blocks aligned to arbitrary powers of two.<br>
 * @pre     In order to use the heap APIs the @p CH_CFG_USE_HEAP option must
 *          be enabled in @p chconf.h.
 * @note    Compatible with RT and NIL.
 * @{
 */

#include "ch.h"

#if (CH_CFG_USE_HEAP == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*
 * Defaults on the best synchronization mechanism available.
 */
#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
#define H_LOCK(h)       chMtxLock(&(h)->mtx)
#define H_UNLOCK(h)     chMtxUnlock(&(h)->mtx)
#else
#define H_LOCK(h)       (void) chSemWait(&(h)->sem)
#define H_UNLOCK(h)     chSemSignal(&(h)->sem)
#endif

#define H_BLOCK(hp)     ((hp) + 1U)

#define H_LIMIT(hp)     (H_BLOCK(hp) + H_PAGES(hp))

#define H_NEXT(hp)      ((hp)->free.next)

#define H_PAGES(hp)     ((hp)->free.pages)

#define H_HEAP(hp)      ((hp)->used.heap)

#define H_SIZE(hp)      ((hp)->used.size)

/*
 * Number of pages between two pointers in a MISRA-compatible way.
 */
#define NPAGES(p1, p2)                                                      \
  /*lint -save -e9033 [10.8] The cast is safe.*/                            \
  ((size_t)((p1) - (p2)))                                                   \
  /*lint -restore*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/**
 * @brief   Default heap descriptor.
 */
static memory_heap_t default_heap;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes the default heap.
 *
 * @notapi
 */
void __heap_init(void) {

  default_heap.provider = chCoreAllocAlignedWithOffset;
  H_NEXT(&default_heap.header) = NULL;
  H_PAGES(&default_heap.header) = 0;
#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
  chMtxObjectInit(&default_heap.mtx);
#else
  chSemObjectInit(&default_heap.sem, (cnt_t)1);
#endif
}

/**
 * @brief   Initializes a memory heap from a static memory area.
 * @note    The heap buffer base and size are adjusted if the passed buffer
 *          is not aligned to @p CH_HEAP_ALIGNMENT. This mean that the
 *          effective heap size can be less than @p size.
 *
 * @param[out] heapp    pointer to the memory heap descriptor to be initialized
 * @param[in] buf       heap buffer base
 * @param[in] size      heap size
 *
 * @init
 */
void chHeapObjectInit(memory_heap_t *heapp, void *buf, size_t size) {
  heap_header_t *hp = (heap_header_t *)MEM_ALIGN_NEXT(buf, CH_HEAP_ALIGNMENT);

  chDbgCheck((heapp != NULL) && (size > 0U));

  /* Adjusting the size in case the initial block was not correctly
     aligned.*/
  /*lint -save -e9033 [10.8] Required cast operations.*/
  size -= (size_t)((uint8_t *)hp - (uint8_t *)buf);
  /*lint -restore*/

  /* Initializing the heap header.*/
  heapp->provider = NULL;
  H_NEXT(&heapp->header) = hp;
  H_PAGES(&heapp->header) = 0;
  H_NEXT(hp) = NULL;
  H_PAGES(hp) = (size - sizeof (heap_header_t)) / CH_HEAP_ALIGNMENT;
#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
  chMtxObjectInit(&heapp->mtx);
#else
  chSemObjectInit(&heapp->sem, (cnt_t)1);
#endif
}

/**
 * @brief   Allocates a block of memory from the heap by using the first-fit
 *          algorithm.
 * @details The allocated block is guaranteed to be properly aligned to the
 *          specified alignment.
 *
 * @param[in] heapp     pointer to a heap descriptor or @p NULL in order to
 *                      access the default heap.
 * @param[in] size      the size of the block to be allocated. Note that the
 *                      allocated block may be a bit bigger than the requested
 *                      size for alignment and fragmentation reasons.
 * @param[in] align     desired memory alignment
 * @return              A pointer to the aligned allocated block.
 * @retval NULL         if the block cannot be allocated.
 *
 * @api
 */
void *chHeapAllocAligned(memory_heap_t *heapp, size_t size, unsigned align) {
  heap_header_t *qp, *hp, *ahp;
  size_t pages;

  chDbgCheck((size > 0U) && MEM_IS_VALID_ALIGNMENT(align));

  /* If an heap is not specified then the default system header is used.*/
  if (heapp == NULL) {
    heapp = &default_heap;
  }

  /* Minimum alignment is constrained by the heap header structure size.*/
  if (align < CH_HEAP_ALIGNMENT) {
    align = CH_HEAP_ALIGNMENT;
  }

  /* Size is converted in number of elementary allocation units, checking
     for overflow in the alignment rounding.*/
  {
    size_t asize = MEM_ALIGN_NEXT(size, CH_HEAP_ALIGNMENT);
    if (asize < size) {
      return NULL;
    }
    pages = asize / CH_HEAP_ALIGNMENT;
  }

  /* Taking heap mutex/semaphore.*/
  H_LOCK(heapp);

  /* Start of the free blocks list.*/
  qp = &heapp->header;
  while (H_NEXT(qp) != NULL) {

    /* Next free block.*/
    hp = H_NEXT(qp);

    /* Pointer aligned to the requested alignment.*/
    ahp = (heap_header_t *)MEM_ALIGN_NEXT(H_BLOCK(hp), align) - 1U;

    if ((ahp < H_LIMIT(hp)) && (pages <= NPAGES(H_LIMIT(hp), ahp + 1U))) {
      /* The block is large enough to contain a correctly aligned area
         of sufficient size.*/

      if (ahp > hp) {
        /* The block is not properly aligned, must split it.*/
        size_t bpages;

        bpages = NPAGES(H_LIMIT(hp), H_BLOCK(ahp));
        H_PAGES(hp) = NPAGES(ahp, H_BLOCK(hp));
        if (bpages > pages) {
          /* The block is bigger than required, must split the excess.*/
          heap_header_t *fp;

          /* Creating the excess block.*/
          fp = H_BLOCK(ahp) + pages;
          H_PAGES(fp) = (bpages - pages) - 1U;

          /* Linking the excess block.*/
          H_NEXT(fp) = H_NEXT(hp);
          H_NEXT(hp) = fp;
        }

        hp = ahp;
      }
      else {
        /* The block is already properly aligned.*/

        if (H_PAGES(hp) == pages) {
          /* Exact size, getting the whole block.*/
          H_NEXT(qp) = H_NEXT(hp);
        }
        else {
          /* The block is bigger than required, must split the excess.*/
          heap_header_t *fp;

          fp = H_BLOCK(hp) + pages;
          H_NEXT(fp) = H_NEXT(hp);
          H_PAGES(fp) = NPAGES(H_LIMIT(hp), H_BLOCK(fp));
          H_NEXT(qp) = fp;
        }
      }

      /* Setting in the block owner heap and size.*/
      H_SIZE(hp) = size;
      H_HEAP(hp) = heapp;

      /* Releasing heap mutex/semaphore.*/
      H_UNLOCK(heapp);

      /*lint -save -e9087 [11.3] Safe cast.*/
      return (void *)H_BLOCK(hp);
      /*lint -restore*/
    }

    /* Next in the free blocks list.*/
    qp = hp;
  }

  /* Releasing heap mutex/semaphore.*/
  H_UNLOCK(heapp);

  /* More memory is required, tries to get it from the associated provider
     else fails.*/
  if (heapp->provider != NULL) {
    ahp = heapp->provider(pages * CH_HEAP_ALIGNMENT,
                          align,
                          sizeof (heap_header_t));
    if (ahp != NULL) {
      hp = ahp - 1U;
      H_HEAP(hp) = heapp;
      H_SIZE(hp) = size;

      /*lint -save -e9087 [11.3] Safe cast.*/
      return (void *)ahp;
      /*lint -restore*/
    }
  }

  return NULL;
}

/**
 * @brief   Frees a previously allocated memory block.
 *
 * @param[in] p         pointer to the memory block to be freed
 *
 * @api
 */
void chHeapFree(void *p) {
  heap_header_t *qp, *hp;
  memory_heap_t *heapp;

  chDbgCheck((p != NULL) && MEM_IS_ALIGNED(p, CH_HEAP_ALIGNMENT));

  /*lint -save -e9087 [11.3] Safe cast.*/
  hp = (heap_header_t *)p - 1U;
  /*lint -restore*/
  heapp = H_HEAP(hp);
  qp = &heapp->header;

  /* Size is converted in number of elementary allocation units.*/
  H_PAGES(hp) = MEM_ALIGN_NEXT(H_SIZE(hp),
                               CH_HEAP_ALIGNMENT) / CH_HEAP_ALIGNMENT;

  /* Taking heap mutex/semaphore.*/
  H_LOCK(heapp);

  while (true) {
    chDbgAssert((hp < qp) || (hp >= H_LIMIT(qp)), "within free block");

    if (((qp == &heapp->header) || (hp > qp)) &&
        ((H_NEXT(qp) == NULL) || (hp < H_NEXT(qp)))) {
      /* Insertion after qp.*/
      H_NEXT(hp) = H_NEXT(qp);
      H_NEXT(qp) = hp;
      /* Verifies if the newly inserted block should be merged.*/
      if (H_LIMIT(hp) == H_NEXT(hp)) {
        /* Merge with the next block.*/
        H_PAGES(hp) += H_PAGES(H_NEXT(hp)) + 1U;
        H_NEXT(hp) = H_NEXT(H_NEXT(hp));
      }
      if ((H_LIMIT(qp) == hp)) {
        /* Merge with the previous block.*/
        H_PAGES(qp) += H_PAGES(hp) + 1U;
        H_NEXT(qp) = H_NEXT(hp);
      }
      break;
    }
    qp = H_NEXT(qp);
  }

  /* Releasing heap mutex/semaphore.*/
  H_UNLOCK(heapp);
}

/**
 * @brief   Reports the heap status.
 * @note    This function is meant to be used in the test suite, it should
 *          not be really useful for the application code.
 *
 * @param[in] heapp     pointer to a heap descriptor or @p NULL in order to
 *                      access the default heap.
 * @param[in] totalp    pointer to a variable that will receive the total
 *                      fragmented free space or @p NULL
 * @param[in] largestp  pointer to a variable that will receive the largest
 *                      free free block found space or @p NULL
 * @return              The number of fragments in the heap.
 *
 * @api
 */
size_t chHeapStatus(memory_heap_t *heapp, size_t *totalp, size_t *largestp) {
  heap_header_t *qp;
  size_t n, tpages, lpages;

  if (heapp == NULL) {
    heapp = &default_heap;
  }

  H_LOCK(heapp);
  tpages = 0U;
  lpages = 0U;
  n = 0U;
  qp = &heapp->header;
  while (H_NEXT(qp) != NULL) {
    size_t pages = H_PAGES(H_NEXT(qp));

    /* Updating counters.*/
    n++;
    tpages += pages;
    if (pages > lpages) {
      lpages = pages;
    }

    qp = H_NEXT(qp);
  }

  /* Writing out fragmented free memory.*/
  if (totalp != NULL) {
    *totalp = tpages * CH_HEAP_ALIGNMENT;
  }

  /* Writing out unfragmented free memory.*/
  if (largestp != NULL) {
    *largestp = lpages * CH_HEAP_ALIGNMENT;
  }
  H_UNLOCK(heapp);

  return n;
}

#endif /* CH_CFG_USE_HEAP == TRUE */

/** @} */
