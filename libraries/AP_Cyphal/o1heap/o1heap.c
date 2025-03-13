// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions
// of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Copyright (c) 2020 Pavel Kirienko
// Authors: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "o1heap.h"
#include <assert.h>
#include <limits.h>

// ---------------------------------------- BUILD CONFIGURATION OPTIONS ----------------------------------------

/// Define this macro to include build configuration header. This is an alternative to the -D compiler flag.
/// Usage example with CMake: "-DO1HEAP_CONFIG_HEADER=\"${CMAKE_CURRENT_SOURCE_DIR}/my_o1heap_config.h\""
#ifdef O1HEAP_CONFIG_HEADER
#    include O1HEAP_CONFIG_HEADER
#endif

/// The assertion macro defaults to the standard assert().
/// It can be overridden to manually suppress assertion checks or use a different error handling policy.
#ifndef O1HEAP_ASSERT
// Intentional violation of MISRA: the assertion check macro cannot be replaced with a function definition.
#    define O1HEAP_ASSERT(x) assert(x)  // NOSONAR
#endif

/// Allow usage of compiler intrinsics for branch annotation and CLZ.
#ifndef O1HEAP_USE_INTRINSICS
#    define O1HEAP_USE_INTRINSICS 1
#endif

/// Branch probability annotations are used to improve the worst case execution time (WCET). They are entirely optional.
#if O1HEAP_USE_INTRINSICS && !defined(O1HEAP_LIKELY)
#    if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
// Intentional violation of MISRA: branch hinting macro cannot be replaced with a function definition.
#        define O1HEAP_LIKELY(x) __builtin_expect((x), 1)  // NOSONAR
#    endif
#endif
#ifndef O1HEAP_LIKELY
#    define O1HEAP_LIKELY(x) x
#endif

/// This option is used for testing only. Do not use in production.
#ifndef O1HEAP_PRIVATE
#    define O1HEAP_PRIVATE static inline
#endif

/// Count leading zeros (CLZ) is used for fast computation of binary logarithm (which needs to be done very often).
/// Most of the modern processors (including the embedded ones) implement dedicated hardware support for fast CLZ
/// computation, which is available via compiler intrinsics. The default implementation will automatically use
/// the intrinsics for some of the compilers; for others it will default to the slow software emulation,
/// which can be overridden by the user via O1HEAP_CONFIG_HEADER. The library guarantees that the argument is positive.
#if O1HEAP_USE_INTRINSICS && !defined(O1HEAP_CLZ)
#    if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#        define O1HEAP_CLZ __builtin_clzl
#    endif
#endif
#ifndef O1HEAP_CLZ
O1HEAP_PRIVATE uint_fast8_t O1HEAP_CLZ(const size_t x)
{
    O1HEAP_ASSERT(x > 0);
    size_t       t = ((size_t) 1U) << ((sizeof(size_t) * CHAR_BIT) - 1U);
    uint_fast8_t r = 0;
    while ((x & t) == 0)
    {
        t >>= 1U;
        r++;
    }
    return r;
}
#endif

// ---------------------------------------- INTERNAL DEFINITIONS ----------------------------------------

#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 199901L)
#    error "Unsupported language: ISO C99 or a newer version is required."
#endif

#if __STDC_VERSION__ < 201112L
// Intentional violation of MISRA: static assertion macro cannot be replaced with a function definition.
#    define static_assert(x, ...) typedef char _static_assert_gl(_static_assertion_, __LINE__)[(x) ? 1 : -1]  // NOSONAR
#    define _static_assert_gl(a, b) _static_assert_gl_impl(a, b)                                              // NOSONAR
// Intentional violation of MISRA: the paste operator ## cannot be avoided in this context.
#    define _static_assert_gl_impl(a, b) a##b  // NOSONAR
#endif

/// The overhead is at most O1HEAP_ALIGNMENT bytes large,
/// then follows the user data which shall keep the next fragment aligned.
#define FRAGMENT_SIZE_MIN (O1HEAP_ALIGNMENT * 2U)

/// This is risky, handle with care: if the allocation amount plus per-fragment overhead exceeds 2**(b-1),
/// where b is the pointer bit width, then ceil(log2(amount)) yields b; then 2**b causes an integer overflow.
/// To avoid this, we put a hard limit on fragment size (which is amount + per-fragment overhead): 2**(b-1)
#define FRAGMENT_SIZE_MAX ((SIZE_MAX >> 1U) + 1U)

/// Normally we should subtract log2(FRAGMENT_SIZE_MIN) but log2 is bulky to compute using the preprocessor only.
/// We will certainly end up with unused bins this way, but it is cheap to ignore.
#define NUM_BINS_MAX (sizeof(size_t) * CHAR_BIT)

static_assert((O1HEAP_ALIGNMENT & (O1HEAP_ALIGNMENT - 1U)) == 0U, "Not a power of 2");
static_assert((FRAGMENT_SIZE_MIN & (FRAGMENT_SIZE_MIN - 1U)) == 0U, "Not a power of 2");
static_assert((FRAGMENT_SIZE_MAX & (FRAGMENT_SIZE_MAX - 1U)) == 0U, "Not a power of 2");

typedef struct Fragment Fragment;

typedef struct FragmentHeader
{
    Fragment* next;
    Fragment* prev;
    size_t    size;
    bool      used;
} FragmentHeader;
static_assert(sizeof(FragmentHeader) <= O1HEAP_ALIGNMENT, "Memory layout error");

struct Fragment
{
    FragmentHeader header;
    // Everything past the header may spill over into the allocatable space. The header survives across alloc/free.
    Fragment* next_free;  // Next free fragment in the bin; NULL in the last one.
    Fragment* prev_free;  // Same but points back; NULL in the first one.
};
static_assert(sizeof(Fragment) <= FRAGMENT_SIZE_MIN, "Memory layout error");

struct O1HeapInstance
{
    Fragment* bins[NUM_BINS_MAX];  ///< Smallest fragments are in the bin at index 0.
    size_t    nonempty_bin_mask;   ///< Bit 1 represents a non-empty bin; bin at index 0 is for the smallest fragments.

    O1HeapDiagnostics diagnostics;
};

/// The amount of space allocated for the heap instance.
/// Its size is padded up to O1HEAP_ALIGNMENT to ensure correct alignment of the allocation arena that follows.
#define INSTANCE_SIZE_PADDED ((sizeof(O1HeapInstance) + O1HEAP_ALIGNMENT - 1U) & ~(O1HEAP_ALIGNMENT - 1U))

static_assert(INSTANCE_SIZE_PADDED >= sizeof(O1HeapInstance), "Invalid instance footprint computation");
static_assert((INSTANCE_SIZE_PADDED % O1HEAP_ALIGNMENT) == 0U, "Invalid instance footprint computation");

/// Undefined for zero argument.
O1HEAP_PRIVATE uint_fast8_t log2Floor(const size_t x)
{
    O1HEAP_ASSERT(x > 0);
    // NOLINTNEXTLINE redundant cast to the same type.
    return (uint_fast8_t) (((sizeof(x) * CHAR_BIT) - 1U) - ((uint_fast8_t) O1HEAP_CLZ(x)));
}

/// Special case: if the argument is zero, returns zero.
O1HEAP_PRIVATE uint_fast8_t log2Ceil(const size_t x)
{
    // NOLINTNEXTLINE redundant cast to the same type.
    return (x <= 1U) ? 0U : (uint_fast8_t) ((sizeof(x) * CHAR_BIT) - ((uint_fast8_t) O1HEAP_CLZ(x - 1U)));
}

/// Raise 2 into the specified power.
/// You might be tempted to do something like (1U << power). WRONG! We humans are prone to forgetting things.
/// If you forget to cast your 1U to size_t or ULL, you may end up with undefined behavior.
O1HEAP_PRIVATE size_t pow2(const uint_fast8_t power)
{
    return ((size_t) 1U) << power;
}

/// This is equivalent to pow2(log2Ceil(x)). Undefined for x<2.
O1HEAP_PRIVATE size_t roundUpToPowerOf2(const size_t x)
{
    O1HEAP_ASSERT(x >= 2U);
    // NOLINTNEXTLINE redundant cast to the same type.
    return ((size_t) 1U) << ((sizeof(x) * CHAR_BIT) - ((uint_fast8_t) O1HEAP_CLZ(x - 1U)));
}

/// Links two fragments so that their next/prev pointers point to each other; left goes before right.
O1HEAP_PRIVATE void interlink(Fragment* const left, Fragment* const right)
{
    if (O1HEAP_LIKELY(left != NULL))
    {
        left->header.next = right;
    }
    if (O1HEAP_LIKELY(right != NULL))
    {
        right->header.prev = left;
    }
}

/// Adds a new fragment into the appropriate bin and updates the lookup mask.
O1HEAP_PRIVATE void rebin(O1HeapInstance* const handle, Fragment* const fragment)
{
    O1HEAP_ASSERT(handle != NULL);
    O1HEAP_ASSERT(fragment != NULL);
    O1HEAP_ASSERT(fragment->header.size >= FRAGMENT_SIZE_MIN);
    O1HEAP_ASSERT((fragment->header.size % FRAGMENT_SIZE_MIN) == 0U);
    const uint_fast8_t idx = log2Floor(fragment->header.size / FRAGMENT_SIZE_MIN);  // Round DOWN when inserting.
    O1HEAP_ASSERT(idx < NUM_BINS_MAX);
    // Add the new fragment to the beginning of the bin list.
    // I.e., each allocation will be returning the most-recently-used fragment -- good for caching.
    fragment->next_free = handle->bins[idx];
    fragment->prev_free = NULL;
    if (O1HEAP_LIKELY(handle->bins[idx] != NULL))
    {
        handle->bins[idx]->prev_free = fragment;
    }
    handle->bins[idx] = fragment;
    handle->nonempty_bin_mask |= pow2(idx);
}

/// Removes the specified fragment from its bin.
O1HEAP_PRIVATE void unbin(O1HeapInstance* const handle, const Fragment* const fragment)
{
    O1HEAP_ASSERT(handle != NULL);
    O1HEAP_ASSERT(fragment != NULL);
    O1HEAP_ASSERT(fragment->header.size >= FRAGMENT_SIZE_MIN);
    O1HEAP_ASSERT((fragment->header.size % FRAGMENT_SIZE_MIN) == 0U);
    const uint_fast8_t idx = log2Floor(fragment->header.size / FRAGMENT_SIZE_MIN);  // Round DOWN when removing.
    O1HEAP_ASSERT(idx < NUM_BINS_MAX);
    // Remove the bin from the free fragment list.
    if (O1HEAP_LIKELY(fragment->next_free != NULL))
    {
        fragment->next_free->prev_free = fragment->prev_free;
    }
    if (O1HEAP_LIKELY(fragment->prev_free != NULL))
    {
        fragment->prev_free->next_free = fragment->next_free;
    }
    // Update the bin header.
    if (O1HEAP_LIKELY(handle->bins[idx] == fragment))
    {
        O1HEAP_ASSERT(fragment->prev_free == NULL);
        handle->bins[idx] = fragment->next_free;
        if (O1HEAP_LIKELY(handle->bins[idx] == NULL))
        {
            handle->nonempty_bin_mask &= ~pow2(idx);
        }
    }
}

// ---------------------------------------- PUBLIC API IMPLEMENTATION ----------------------------------------

O1HeapInstance* o1heapInit(void* const base, const size_t size)
{
    O1HeapInstance* out = NULL;
    if ((base != NULL) && ((((size_t) base) % O1HEAP_ALIGNMENT) == 0U) &&
        (size >= (INSTANCE_SIZE_PADDED + FRAGMENT_SIZE_MIN)))
    {
        // Allocate the core heap metadata structure in the beginning of the arena.
        O1HEAP_ASSERT(((size_t) base) % sizeof(O1HeapInstance*) == 0U);
        out                    = (O1HeapInstance*) base;
        out->nonempty_bin_mask = 0U;
        for (size_t i = 0; i < NUM_BINS_MAX; i++)
        {
            out->bins[i] = NULL;
        }

        // Limit and align the capacity.
        size_t capacity = size - INSTANCE_SIZE_PADDED;
        if (capacity > FRAGMENT_SIZE_MAX)
        {
            capacity = FRAGMENT_SIZE_MAX;
        }
        while ((capacity % FRAGMENT_SIZE_MIN) != 0)
        {
            O1HEAP_ASSERT(capacity > 0U);
            capacity--;
        }
        O1HEAP_ASSERT((capacity % FRAGMENT_SIZE_MIN) == 0);
        O1HEAP_ASSERT((capacity >= FRAGMENT_SIZE_MIN) && (capacity <= FRAGMENT_SIZE_MAX));

        // Initialize the root fragment.
        Fragment* const frag = (Fragment*) (void*) (((char*) base) + INSTANCE_SIZE_PADDED);
        O1HEAP_ASSERT((((size_t) frag) % O1HEAP_ALIGNMENT) == 0U);
        frag->header.next = NULL;
        frag->header.prev = NULL;
        frag->header.size = capacity;
        frag->header.used = false;
        frag->next_free   = NULL;
        frag->prev_free   = NULL;
        rebin(out, frag);
        O1HEAP_ASSERT(out->nonempty_bin_mask != 0U);

        // Initialize the diagnostics.
        out->diagnostics.capacity          = capacity;
        out->diagnostics.allocated         = 0U;
        out->diagnostics.peak_allocated    = 0U;
        out->diagnostics.peak_request_size = 0U;
        out->diagnostics.oom_count         = 0U;
    }

    return out;
}

void* o1heapAllocate(O1HeapInstance* const handle, const size_t amount)
{
    O1HEAP_ASSERT(handle != NULL);
    O1HEAP_ASSERT(handle->diagnostics.capacity <= FRAGMENT_SIZE_MAX);
    void* out = NULL;

    // If the amount approaches approx. SIZE_MAX/2, an undetected integer overflow may occur.
    // To avoid that, we do not attempt allocation if the amount exceeds the hard limit.
    // We perform multiple redundant checks to account for a possible unaccounted overflow.
    if (O1HEAP_LIKELY((amount > 0U) && (amount <= (handle->diagnostics.capacity - O1HEAP_ALIGNMENT))))
    {
        // Add the header size and align the allocation size to the power of 2.
        // See "Timing-Predictable Memory Allocation In Hard Real-Time Systems", Herter, page 27.
        const size_t fragment_size = roundUpToPowerOf2(amount + O1HEAP_ALIGNMENT);
        O1HEAP_ASSERT(fragment_size <= FRAGMENT_SIZE_MAX);
        O1HEAP_ASSERT(fragment_size >= FRAGMENT_SIZE_MIN);
        O1HEAP_ASSERT(fragment_size >= amount + O1HEAP_ALIGNMENT);
        O1HEAP_ASSERT((fragment_size & (fragment_size - 1U)) == 0U);  // Is power of 2.

        const uint_fast8_t optimal_bin_index = log2Ceil(fragment_size / FRAGMENT_SIZE_MIN);  // Use CEIL when fetching.
        O1HEAP_ASSERT(optimal_bin_index < NUM_BINS_MAX);
        const size_t candidate_bin_mask = ~(pow2(optimal_bin_index) - 1U);

        // Find the smallest non-empty bin we can use.
        const size_t suitable_bins     = handle->nonempty_bin_mask & candidate_bin_mask;
        const size_t smallest_bin_mask = suitable_bins & ~(suitable_bins - 1U);  // Clear all bits but the lowest.
        if (O1HEAP_LIKELY(smallest_bin_mask != 0))
        {
            O1HEAP_ASSERT((smallest_bin_mask & (smallest_bin_mask - 1U)) == 0U);  // Is power of 2.
            const uint_fast8_t bin_index = log2Floor(smallest_bin_mask);
            O1HEAP_ASSERT(bin_index >= optimal_bin_index);
            O1HEAP_ASSERT(bin_index < NUM_BINS_MAX);

            // The bin we found shall not be empty, otherwise it's a state divergence (memory corruption?).
            Fragment* const frag = handle->bins[bin_index];
            O1HEAP_ASSERT(frag != NULL);
            O1HEAP_ASSERT(frag->header.size >= fragment_size);
            O1HEAP_ASSERT((frag->header.size % FRAGMENT_SIZE_MIN) == 0U);
            O1HEAP_ASSERT(!frag->header.used);
            unbin(handle, frag);

            // Split the fragment if it is too large.
            const size_t leftover = frag->header.size - fragment_size;
            frag->header.size     = fragment_size;
            O1HEAP_ASSERT(leftover < handle->diagnostics.capacity);  // Overflow check.
            O1HEAP_ASSERT(leftover % FRAGMENT_SIZE_MIN == 0U);       // Alignment check.
            if (O1HEAP_LIKELY(leftover >= FRAGMENT_SIZE_MIN))
            {
                Fragment* const new_frag = (Fragment*) (void*) (((char*) frag) + fragment_size);
                O1HEAP_ASSERT(((size_t) new_frag) % O1HEAP_ALIGNMENT == 0U);
                new_frag->header.size = leftover;
                new_frag->header.used = false;
                interlink(new_frag, frag->header.next);
                interlink(frag, new_frag);
                rebin(handle, new_frag);
            }

            // Update the diagnostics.
            O1HEAP_ASSERT((handle->diagnostics.allocated % FRAGMENT_SIZE_MIN) == 0U);
            handle->diagnostics.allocated += fragment_size;
            O1HEAP_ASSERT(handle->diagnostics.allocated <= handle->diagnostics.capacity);
            if (O1HEAP_LIKELY(handle->diagnostics.peak_allocated < handle->diagnostics.allocated))
            {
                handle->diagnostics.peak_allocated = handle->diagnostics.allocated;
            }

            // Finalize the fragment we just allocated.
            O1HEAP_ASSERT(frag->header.size >= amount + O1HEAP_ALIGNMENT);
            frag->header.used = true;

            out = ((char*) frag) + O1HEAP_ALIGNMENT;
        }
    }

    // Update the diagnostics.
    if (O1HEAP_LIKELY(handle->diagnostics.peak_request_size < amount))
    {
        handle->diagnostics.peak_request_size = amount;
    }
    if (O1HEAP_LIKELY((out == NULL) && (amount > 0U)))
    {
        handle->diagnostics.oom_count++;
    }

    return out;
}

void o1heapFree(O1HeapInstance* const handle, void* const pointer)
{
    O1HEAP_ASSERT(handle != NULL);
    O1HEAP_ASSERT(handle->diagnostics.capacity <= FRAGMENT_SIZE_MAX);
    if (O1HEAP_LIKELY(pointer != NULL))  // NULL pointer is a no-op.
    {
        Fragment* const frag = (Fragment*) (void*) (((char*) pointer) - O1HEAP_ALIGNMENT);

        // Check for heap corruption in debug builds.
        O1HEAP_ASSERT(((size_t) frag) % sizeof(Fragment*) == 0U);
        O1HEAP_ASSERT(((size_t) frag) >= (((size_t) handle) + INSTANCE_SIZE_PADDED));
        O1HEAP_ASSERT(((size_t) frag) <=
                      (((size_t) handle) + INSTANCE_SIZE_PADDED + handle->diagnostics.capacity - FRAGMENT_SIZE_MIN));
        O1HEAP_ASSERT(frag->header.used);  // Catch double-free
        O1HEAP_ASSERT(((size_t) frag->header.next) % sizeof(Fragment*) == 0U);
        O1HEAP_ASSERT(((size_t) frag->header.prev) % sizeof(Fragment*) == 0U);
        O1HEAP_ASSERT(frag->header.size >= FRAGMENT_SIZE_MIN);
        O1HEAP_ASSERT(frag->header.size <= handle->diagnostics.capacity);
        O1HEAP_ASSERT((frag->header.size % FRAGMENT_SIZE_MIN) == 0U);

        // Even if we're going to drop the fragment later, mark it free anyway to prevent double-free.
        frag->header.used = false;

        // Update the diagnostics. It must be done before merging because it invalidates the fragment size information.
        O1HEAP_ASSERT(handle->diagnostics.allocated >= frag->header.size);  // Heap corruption check.
        handle->diagnostics.allocated -= frag->header.size;

        // Merge with siblings and insert the returned fragment into the appropriate bin and update metadata.
        Fragment* const prev       = frag->header.prev;
        Fragment* const next       = frag->header.next;
        const bool      join_left  = (prev != NULL) && (!prev->header.used);
        const bool      join_right = (next != NULL) && (!next->header.used);
        if (join_left && join_right)  // [ prev ][ this ][ next ] => [ ------- prev ------- ]
        {
            unbin(handle, prev);
            unbin(handle, next);
            prev->header.size += frag->header.size + next->header.size;
            frag->header.size = 0;  // Invalidate the dropped fragment headers to prevent double-free.
            next->header.size = 0;
            O1HEAP_ASSERT((prev->header.size % FRAGMENT_SIZE_MIN) == 0U);
            interlink(prev, next->header.next);
            rebin(handle, prev);
        }
        else if (join_left)  // [ prev ][ this ][ next ] => [ --- prev --- ][ next ]
        {
            unbin(handle, prev);
            prev->header.size += frag->header.size;
            frag->header.size = 0;
            O1HEAP_ASSERT((prev->header.size % FRAGMENT_SIZE_MIN) == 0U);
            interlink(prev, next);
            rebin(handle, prev);
        }
        else if (join_right)  // [ prev ][ this ][ next ] => [ prev ][ --- this --- ]
        {
            unbin(handle, next);
            frag->header.size += next->header.size;
            next->header.size = 0;
            O1HEAP_ASSERT((frag->header.size % FRAGMENT_SIZE_MIN) == 0U);
            interlink(frag, next->header.next);
            rebin(handle, frag);
        }
        else
        {
            rebin(handle, frag);
        }
    }
}

bool o1heapDoInvariantsHold(const O1HeapInstance* const handle)
{
    O1HEAP_ASSERT(handle != NULL);
    bool valid = true;

    // Check the bin mask consistency.
    for (size_t i = 0; i < NUM_BINS_MAX; i++)  // Dear compiler, feel free to unroll this loop.
    {
        const bool mask_bit_set = (handle->nonempty_bin_mask & pow2((uint_fast8_t) i)) != 0U;
        const bool bin_nonempty = handle->bins[i] != NULL;
        valid                   = valid && (mask_bit_set == bin_nonempty);
    }

    // Create a local copy of the diagnostics struct.
    const O1HeapDiagnostics diag = handle->diagnostics;

    // Capacity check.
    valid = valid && (diag.capacity <= FRAGMENT_SIZE_MAX) && (diag.capacity >= FRAGMENT_SIZE_MIN) &&
            ((diag.capacity % FRAGMENT_SIZE_MIN) == 0U);

    // Allocation info check.
    valid = valid && (diag.allocated <= diag.capacity) && ((diag.allocated % FRAGMENT_SIZE_MIN) == 0U) &&
            (diag.peak_allocated <= diag.capacity) && (diag.peak_allocated >= diag.allocated) &&
            ((diag.peak_allocated % FRAGMENT_SIZE_MIN) == 0U);

    // Peak request check
    valid = valid && ((diag.peak_request_size < diag.capacity) || (diag.oom_count > 0U));
    if (diag.peak_request_size == 0U)
    {
        valid = valid && (diag.peak_allocated == 0U) && (diag.allocated == 0U) && (diag.oom_count == 0U);
    }
    else
    {
        valid = valid &&  // Overflow on summation is possible but safe to ignore.
                (((diag.peak_request_size + O1HEAP_ALIGNMENT) <= diag.peak_allocated) || (diag.oom_count > 0U));
    }

    return valid;
}

O1HeapDiagnostics o1heapGetDiagnostics(const O1HeapInstance* const handle)
{
    O1HEAP_ASSERT(handle != NULL);
    const O1HeapDiagnostics out = handle->diagnostics;
    return out;
}
