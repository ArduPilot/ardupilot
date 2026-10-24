// SPDX-License-Identifier: GPL-3.0-or-later
#include "Heap.h"

#if AP_MODULE_ENABLED

namespace AP_Module
{

Heap::~Heap()
{
    clear();
    _heap.destroy();
}

bool Heap::create(uint32_t size, bool allow_expansion, uint32_t max_capacity)
{
    WITH_SEMAPHORE(_sem);
    constexpr uint8_t max_arenas = 10;
    return size >= 256 && size <= 16U * 1024 * 1024 &&
           _heap.create(size, max_arenas, allow_expansion, 20 * 1024, max_capacity);
}

void *Heap::allocate(uint32_t size, uint32_t alignment)
{
    if (size == 0 || alignment == 0 || alignment > 4096 || (alignment & (alignment - 1)) != 0) {
        return nullptr;
    }
    const uint32_t overhead = sizeof(Allocation) + alignment - 1;
    if (size > UINT32_MAX - overhead) {
        return nullptr;
    }
    const uint32_t charge = size + overhead;
    WITH_SEMAPHORE(_sem);
    auto *allocation = static_cast<Allocation *>(_heap.allocate(charge));
    if (allocation == nullptr) {
        // MultiHeap deliberately fails once before expanding so Lua can try GC.
        // Native modules have no GC retry: make a second attempt before OOM.
        // MultiHeap still enforces its armed-only expansion and memory reserve.
        allocation = static_cast<Allocation *>(_heap.allocate(charge));
    }
    if (allocation == nullptr) {
        return nullptr;
    }
    const uintptr_t start = reinterpret_cast<uintptr_t>(allocation + 1);
    allocation->pointer = reinterpret_cast<void *>((start + alignment - 1) & ~uintptr_t(alignment - 1));
    allocation->charge = charge;
    allocation->next = _allocations;
    _allocations = allocation;
    _stats.used += charge;
    if (_stats.used > _stats.peak) {
        _stats.peak = _stats.used;
    }
    _stats.allocations++;
    return allocation->pointer;
}

bool Heap::deallocate(void *pointer)
{
    if (pointer == nullptr) {
        return true;
    }
    WITH_SEMAPHORE(_sem);
    for (Allocation **link = &_allocations; *link != nullptr; link = &(*link)->next) {
        Allocation *allocation = *link;
        if (allocation->pointer == pointer) {
            *link = allocation->next;
            _stats.used -= allocation->charge;
            _stats.allocations--;
            _heap.deallocate(allocation);
            return true;
        }
    }
    return false;
}

void Heap::clear()
{
    WITH_SEMAPHORE(_sem);
    while (_allocations != nullptr) {
        Allocation *allocation = _allocations;
        _allocations = allocation->next;
        _heap.deallocate(allocation);
    }
    _stats.used = 0;
    _stats.allocations = 0;
}

Heap::Stats Heap::stats(bool include_free)
{
    WITH_SEMAPHORE(_sem);
    _stats.expanded_to = _heap.get_expansion_size();
    _stats.expansion_failures = _heap.get_expansion_failures();
    _stats.capacity = _heap.get_size();
    _stats.free = include_free ? _heap.get_free_size() : 0;
    return _stats;
}

}
#endif // AP_MODULE_ENABLED
