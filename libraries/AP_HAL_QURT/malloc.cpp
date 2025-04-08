/*
  wrappers around core memory allocation functions from libc
*/

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include <new>

const std::nothrow_t std::nothrow;

extern "C" {
    void *fc_heap_alloc(size_t size);
    void fc_heap_free(void* ptr);

    void *__wrap_malloc(size_t size);
    void __wrap_free(void *ptr);
    void *__wrap_calloc(size_t nmemb, size_t size);
    void *__wrap_realloc(void *ptr, size_t size);
}

#define HEAP_HEADER_MAGIC 0x1763247F

typedef struct {
    size_t size;
    uint32_t magic;
} heap_header_t;

void *__wrap_malloc(size_t size)
{
    // fc_heap_alloc guarantees zero filled memory
    auto *ret = (heap_header_t *)fc_heap_alloc(size+sizeof(heap_header_t));
    if (ret == nullptr) {
        return nullptr;
    }
    ret->size = size;
    ret->magic = HEAP_HEADER_MAGIC;
    return (void*)(ret+1);
}

void __wrap_free(void *ptr)
{
    if (ptr == nullptr) {
        return;
    }
    auto *h = ((heap_header_t *)ptr)-1;
    if (h->magic != HEAP_HEADER_MAGIC) {
        AP_HAL::panic("free: bad memory header 0x%x", unsigned(h->magic));
    }
    fc_heap_free((void*)h);
}

void *__wrap_calloc(size_t nmemb, size_t size)
{
    return __wrap_malloc(nmemb*size);
}

void *__wrap_realloc(void *ptr, size_t size)
{
    if (size == 0) {
        // a free
        __wrap_free(ptr);
        return nullptr;
    }
    if (ptr == nullptr) {
        // new allocation
        return __wrap_malloc(size);
    }

    auto *h = ((heap_header_t *)ptr)-1;
    if (h->magic != HEAP_HEADER_MAGIC) {
        AP_HAL::panic("realloc: bad memory header 0x%x", unsigned(h->magic));
    }

    void *ret = __wrap_malloc(size);
    if (ret == nullptr) {
        return nullptr;
    }
    const size_t copy_size = size > h->size? h->size : size;
    memcpy(ret, ptr, copy_size);
    __wrap_free(ptr);

    return ret;
}

