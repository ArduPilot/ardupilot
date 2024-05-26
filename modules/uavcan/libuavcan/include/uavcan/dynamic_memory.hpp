/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_DYNAMIC_MEMORY_HPP_INCLUDED
#define UAVCAN_DYNAMIC_MEMORY_HPP_INCLUDED

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <uavcan/std.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/util/placement_new.hpp>
#include <uavcan/build_config.hpp>

namespace uavcan
{
/**
 * This interface is used by other library components that need dynamic memory.
 */
class UAVCAN_EXPORT IPoolAllocator
{
public:
    virtual ~IPoolAllocator() { }

    virtual void* allocate(std::size_t size) = 0;
    virtual void deallocate(const void* ptr) = 0;

    /**
     * Returns the maximum number of blocks this allocator can allocate.
     */
    virtual uint16_t getBlockCapacity() const = 0;
};

/**
 * Classic implementation of a pool allocator (Meyers).
 *
 * The allocator can be made thread-safe (optional) by means of providing a RAII-lock type via the second template
 * argument. The allocator uses the lock only to access the shared state, therefore critical sections are only a few
 * cycles long, which implies that it should be acceptable to use hardware IRQ disabling instead of a mutex for
 * performance reasons. For example, an IRQ-based RAII-lock type can be implemented as follows:
 *     struct RaiiSynchronizer
 *     {
 *         RaiiSynchronizer()  { __disable_irq(); }
 *         ~RaiiSynchronizer() { __enable_irq(); }
 *     };
 */
template <std::size_t PoolSize,
          uint8_t BlockSize,
          typename RaiiSynchronizer = char>
class UAVCAN_EXPORT PoolAllocator : public IPoolAllocator,
                                    Noncopyable
{
    union Node
    {
        uint8_t data[BlockSize];
        Node* next;
    };

    Node* free_list_;
    union
    {
         uint8_t bytes[PoolSize];
         long double _aligner1;
         long long _aligner2;
         Node _aligner3;
    } pool_;

    uint16_t used_;
    uint16_t max_used_;

public:
    static const uint16_t NumBlocks = PoolSize / BlockSize;

    PoolAllocator();

    virtual void* allocate(std::size_t size) override;
    virtual void deallocate(const void* ptr) override;

    virtual uint16_t getBlockCapacity() const override { return NumBlocks; }

    /**
     * Return the number of blocks that are currently allocated/unallocated.
     */
    uint16_t getNumUsedBlocks() const
    {
        RaiiSynchronizer lock;
        (void)lock;
        return used_;
    }
    uint16_t getNumFreeBlocks() const
    {
        RaiiSynchronizer lock;
        (void)lock;
        return static_cast<uint16_t>(NumBlocks - used_);
    }

    /**
     * Returns the maximum number of blocks that were ever allocated at the same time.
     */
    uint16_t getPeakNumUsedBlocks() const
    {
        RaiiSynchronizer lock;
        (void)lock;
        return max_used_;
    }
};

/**
 * Limits the maximum number of blocks that can be allocated in a given allocator.
 */
class LimitedPoolAllocator : public IPoolAllocator
{
    IPoolAllocator& allocator_;
    const uint16_t max_blocks_;
    uint16_t used_blocks_;

public:
    LimitedPoolAllocator(IPoolAllocator& allocator, std::size_t max_blocks)
        : allocator_(allocator)
        , max_blocks_(static_cast<uint16_t>(min<std::size_t>(max_blocks, 0xFFFFU)))
        , used_blocks_(0)
    {
        UAVCAN_ASSERT(max_blocks_ > 0);
    }

    virtual void* allocate(std::size_t size) override;
    virtual void deallocate(const void* ptr) override;

    virtual uint16_t getBlockCapacity() const override;
};

// ----------------------------------------------------------------------------

/*
 * PoolAllocator<>
 */
template <std::size_t PoolSize, uint8_t BlockSize, typename RaiiSynchronizer>
const uint16_t PoolAllocator<PoolSize, BlockSize, RaiiSynchronizer>::NumBlocks;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

template <std::size_t PoolSize, uint8_t BlockSize, typename RaiiSynchronizer>
PoolAllocator<PoolSize, BlockSize, RaiiSynchronizer>::PoolAllocator() :
    free_list_(reinterpret_cast<Node*>(pool_.bytes)),
    used_(0),
    max_used_(0)
{
    // The limit is imposed by the width of the pool usage tracking variables.
    StaticAssert<((PoolSize / BlockSize) <= 0xFFFFU)>::check();

    (void)std::memset(pool_.bytes, 0, PoolSize);
    for (unsigned i = 0; (i + 1) < (NumBlocks - 1 + 1); i++) // -Werror=type-limits
    {
        // coverity[dead_error_line : FALSE]
        free_list_[i].next = free_list_ + i + 1;
    }
    free_list_[NumBlocks - 1].next = UAVCAN_NULLPTR;
}
#pragma GCC diagnostic pop

template <std::size_t PoolSize, uint8_t BlockSize, typename RaiiSynchronizer>
void* PoolAllocator<PoolSize, BlockSize, RaiiSynchronizer>::allocate(std::size_t size)
{
    if (free_list_ == UAVCAN_NULLPTR || size > BlockSize)
    {
        return UAVCAN_NULLPTR;
    }

    RaiiSynchronizer lock;
    (void)lock;

    void* pmem = free_list_;
    free_list_ = free_list_->next;

    // Statistics
    UAVCAN_ASSERT(used_ < NumBlocks);
    used_++;
    if (used_ > max_used_)
    {
        max_used_ = used_;
    }

    return pmem;
}

template <std::size_t PoolSize, uint8_t BlockSize, typename RaiiSynchronizer>
void PoolAllocator<PoolSize, BlockSize, RaiiSynchronizer>::deallocate(const void* ptr)
{
    if (ptr == UAVCAN_NULLPTR)
    {
        return;
    }

    RaiiSynchronizer lock;
    (void)lock;

    Node* p = static_cast<Node*>(const_cast<void*>(ptr));
    p->next = free_list_;
    free_list_ = p;

    // Statistics
    UAVCAN_ASSERT(used_ > 0);
    used_--;
}

}

#endif // UAVCAN_DYNAMIC_MEMORY_HPP_INCLUDED
