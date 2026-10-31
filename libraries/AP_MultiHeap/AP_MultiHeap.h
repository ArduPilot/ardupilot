/*
  multiple heap interface, allowing for an allocator that uses
  multiple underlying heaps to cope with multiple memory regions on
  STM32 boards
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Util.h>

#include <stdint.h>
#include <stdbool.h>

class MultiHeap {
public:
    /*
      allocate/deallocate heaps
     */
    // max_capacity bounds backing arena capacity, not live allocation bytes.
    // Zero preserves the uncapped scripting heap policy.
    bool create(uint32_t total_size, uint8_t max_heaps, bool allow_expansion, uint32_t reserve_size,
                uint32_t max_capacity = 0);
    void destroy(void);

    // return true if the heap is available for operations
    bool available(void) const;

    // allocate memory within heaps
    void *allocate(uint32_t size);
    void deallocate(void *ptr);

    // change allocated size of a pointer - this operates in a similar
    // fashion to realloc, but requires an (accurate!) old_size value
    // when ptr is not NULL. This is guaranteed by the lua scripting
    // allocation API
    void *change_size(void *ptr, uint32_t old_size, uint32_t new_size);

    /*
      get the size that we have expanded to. Used by error reporting in scripting
     */
    uint32_t get_expansion_size(void) const {
        return expanded_to;
    }
    enum ExpansionFailure : uint8_t { Capacity = 1, Arenas = 2, Memory = 4 };
    uint8_t get_expansion_failures() const { return expansion_failures; }
    uint32_t get_size() const { return sum_size; }
    // Caller must exclude concurrent allocation, expansion and destruction.
    uint32_t get_free_size() const;

private:
    struct Heap {
        void *hp;
    };
    struct Heap *heaps;

    uint8_t num_heaps;
    bool allow_expansion;
    uint32_t reserve_size;
    uint32_t sum_size;
    uint32_t max_capacity;
    uint8_t expansion_failures;
    uint32_t expanded_to;

    // we only do heap expansion if the last allocation failed this
    // encourages the lua scripting engine to garbage collect to
    // re-use memory when possible
    bool last_failed;


    /*
      low level allocation functions
     */
    /*
      heap functions used by lua scripting
    */
    // allocate a new heap
    void *heap_create(uint32_t size);

    // destroy a heap
    void heap_destroy(void *ptr);

    // allocate some memory on a specific heap
    void *heap_allocate(void *heap, uint32_t size);

    // free some memory that was allocated by heap_allocate. The implementation must
    // be able to determine which heap the allocation was from using the pointer
    void heap_free(void *ptr);
    uint32_t heap_free_size(void *heap) const;
};
