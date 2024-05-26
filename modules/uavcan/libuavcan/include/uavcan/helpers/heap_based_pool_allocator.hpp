/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_HELPERS_HEAP_BASED_POOL_ALLOCATOR_HPP_INCLUDED
#define UAVCAN_HELPERS_HEAP_BASED_POOL_ALLOCATOR_HPP_INCLUDED

#include <cstdlib>
#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/dynamic_memory.hpp>

namespace uavcan
{
/**
 * A special-purpose implementation of a pool allocator that keeps the pool in the heap using malloc()/free().
 * The pool grows dynamically, ad-hoc, thus using as little memory as possible.
 *
 * Allocated blocks will not be freed back automatically, but there are two ways to force their deallocation:
 *  - Call @ref shrink() - this method frees all blocks that are unused at the moment.
 *  - Destroy the object - the desctructor calls @ref shrink().
 *
 * The pool can be limited in growth with hard and soft limits.
 * The soft limit defines the value that will be reported via @ref IPoolAllocator::getBlockCapacity().
 * The hard limit defines the maximum number of blocks that can be allocated from heap.
 * Typically, the hard limit should be equal or greater than the soft limit.
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
template <std::size_t BlockSize,
          typename RaiiSynchronizer = char>
class UAVCAN_EXPORT HeapBasedPoolAllocator : public IPoolAllocator,
                                             Noncopyable
{
    union Node
    {
        Node* next;
    private:
        uint8_t data[BlockSize];
        long double _aligner1;
        long long _aligner2;
    };

    const uint16_t capacity_soft_limit_;
    const uint16_t capacity_hard_limit_;

    uint16_t num_reserved_blocks_;
    uint16_t num_allocated_blocks_;

    Node* reserve_;

public:
    /**
     * The allocator initializes with empty reserve, so first allocations will be served from heap.
     *
     * @param block_capacity_soft_limit     Block capacity that will be reported via @ref getBlockCapacity().
     *
     * @param block_capacity_hard_limit     Real block capacity limit; the number of allocated blocks will never
     *                                      exceed this value. Hard limit should be higher than soft limit.
     *                                      Default value is two times the soft limit.
     */
    HeapBasedPoolAllocator(uint16_t block_capacity_soft_limit,
                           uint16_t block_capacity_hard_limit = 0) :
        capacity_soft_limit_(block_capacity_soft_limit),
        capacity_hard_limit_((block_capacity_hard_limit > 0) ? block_capacity_hard_limit :
                             static_cast<uint16_t>(min(static_cast<uint32_t>(block_capacity_soft_limit) * 2U,
                                                       static_cast<uint32_t>(NumericTraits<uint16_t>::max())))),
        num_reserved_blocks_(0),
        num_allocated_blocks_(0),
        reserve_(UAVCAN_NULLPTR)
    { }

    /**
     * The destructor de-allocates all blocks that are currently in the reserve.
     * BLOCKS THAT ARE CURRENTLY HELD BY THE APPLICATION WILL LEAK.
     */
    ~HeapBasedPoolAllocator()
    {
        shrink();
#if UAVCAN_DEBUG
        if (num_allocated_blocks_ > 0)
        {
            UAVCAN_TRACE("HeapBasedPoolAllocator", "%u BLOCKS LEAKED", num_allocated_blocks_);
        }
#endif
    }

    /**
     * Takes a block from the reserve, unless it's empty.
     * In the latter case, allocates a new block in the heap.
     */
    virtual void* allocate(std::size_t size) override
    {
        if (size > BlockSize)
        {
            return UAVCAN_NULLPTR;
        }

        {
            RaiiSynchronizer lock;
            (void)lock;

            Node* const p = reserve_;

            if (UAVCAN_LIKELY(p != UAVCAN_NULLPTR))
            {
                reserve_ = reserve_->next;
                num_allocated_blocks_++;
                return p;
            }

            if (num_reserved_blocks_ >= capacity_hard_limit_)   // Hard limit reached, no further allocations
            {
                return UAVCAN_NULLPTR;
            }
        }

        // Unlikely branch
        void* const m = std::malloc(sizeof(Node));
        if (m != UAVCAN_NULLPTR)
        {
            RaiiSynchronizer lock;
            (void)lock;

            num_reserved_blocks_++;
            num_allocated_blocks_++;
        }
        return m;
    }

    /**
     * Puts the block back to reserve.
     * The block will not be free()d automatically; see @ref shrink().
     */
    virtual void deallocate(const void* ptr) override
    {
        if (ptr != UAVCAN_NULLPTR)
        {
            RaiiSynchronizer lock;
            (void)lock;

            Node* const node = static_cast<Node*>(const_cast<void*>(ptr));
            node->next = reserve_;
            reserve_ = node;

            num_allocated_blocks_--;
        }
    }

    /**
     * The soft limit.
     */
    virtual uint16_t getBlockCapacity() const override { return capacity_soft_limit_; }

    /**
     * The hard limit.
     */
    uint16_t getBlockCapacityHardLimit() const { return capacity_hard_limit_; }

    /**
     * Frees all blocks that are not in use at the moment.
     * Post-condition is getNumAllocatedBlocks() == getNumReservedBlocks().
     */
    void shrink()
    {
        Node* p = UAVCAN_NULLPTR;
        for (;;)
        {
            {
                RaiiSynchronizer lock;
                (void)lock;
                // Removing from reserve and updating the counter.
                p = reserve_;
                if (p != UAVCAN_NULLPTR)
                {
                    reserve_ = reserve_->next;
                    num_reserved_blocks_--;
                }
                else
                {
                    break;
                }
            }
            // Then freeing, having left the critical section.
            std::free(p);
        }
    }

    /**
     * Number of blocks that are currently in use by the application.
     */
    uint16_t getNumAllocatedBlocks() const
    {
        RaiiSynchronizer lock;
        (void)lock;
        return num_allocated_blocks_;
    }

    /**
     * Number of blocks that are acquired from the heap.
     */
    uint16_t getNumReservedBlocks() const
    {
        RaiiSynchronizer lock;
        (void)lock;
        return num_reserved_blocks_;
    }
};

}

#endif
