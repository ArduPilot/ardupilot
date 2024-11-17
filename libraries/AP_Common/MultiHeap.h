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

private:
    struct Heap {
        void *hp;
    };
    struct Heap *heaps;

    uint8_t num_heaps;
    bool allow_expansion;
    uint32_t reserve_size;
    uint32_t sum_size;
    uint32_t expanded_to;

    // we only do heap expansion if the last allocation failed this
    // encourages the lua scripting engine to garbage collect to
    // re-use memory when possible
    bool last_failed;
};
