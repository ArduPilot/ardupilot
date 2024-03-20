/*
  multiple heap interface, allowing for an allocator that uses
  multiple underlying heaps to cope with multiple memory regions on
  STM32 boards
 */

class MultiHeap {
public:
    /*
      allocate/deallocate heaps
     */
    bool create(uint32_t total_size, uint8_t max_heaps, bool allow_expansion, uint32_t reserve_size);
    void destroy(void);

    // return true if the heap is available for operations
    bool available(void) const;

    // allocate memory within heaps
    void *allocate(uint32_t size);
    void deallocate(void *ptr);

    // change allocated size of a pointer - this requires the old
    // size, unlike realloc()
    void *change_size(void *ptr, uint32_t old_size, uint32_t new_size);

    struct alloc_header {
        void *heap;
#ifdef HAL_DEBUG_BUILD
        uint32_t size;
#endif
    };

private:

    void *allocate_from_heap(void *heap, uint32_t size);

    uint8_t num_heaps;
    void **heaps;
    bool allow_expansion;
    uint32_t reserve_size;
};
