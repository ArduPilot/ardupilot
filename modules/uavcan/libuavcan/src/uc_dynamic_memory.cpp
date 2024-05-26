/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/dynamic_memory.hpp>

namespace uavcan
{
/*
 * LimitedPoolAllocator
 */
void* LimitedPoolAllocator::allocate(std::size_t size)
{
    if (used_blocks_ < max_blocks_)
    {
        used_blocks_++;
        return allocator_.allocate(size);
    }
    else
    {
        return UAVCAN_NULLPTR;
    }
}

void LimitedPoolAllocator::deallocate(const void* ptr)
{
    allocator_.deallocate(ptr);

    UAVCAN_ASSERT(used_blocks_ > 0);
    if (used_blocks_ > 0)
    {
        used_blocks_--;
    }
}

uint16_t LimitedPoolAllocator::getBlockCapacity() const
{
    return min(max_blocks_, allocator_.getBlockCapacity());
}

}
