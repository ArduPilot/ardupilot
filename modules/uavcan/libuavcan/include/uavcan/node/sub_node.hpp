/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_SUB_NODE_NODE_HPP_INCLUDED
#define UAVCAN_SUB_NODE_NODE_HPP_INCLUDED

#include <cassert>
#include <uavcan/build_config.hpp>
#include <uavcan/node/abstract_node.hpp>

#if UAVCAN_TINY
# error "This functionality is not available in tiny mode"
#endif

namespace uavcan
{
/**
 * This node object can be used in multiprocess UAVCAN nodes.
 * Please refer to the @ref Node<> for documentation concerning the template arguments; refer to the tutorials
 * to lean how to use libuavcan in multiprocess applications.
 */
template <std::size_t MemPoolSize = 0>
class UAVCAN_EXPORT SubNode : public INode
{
    typedef typename
        Select<(MemPoolSize > 0),
               PoolAllocator<MemPoolSize, MemPoolBlockSize>, // If pool size is specified, use default allocator
               IPoolAllocator&                               // Otherwise use reference to user-provided allocator
              >::Result Allocator;

    Allocator pool_allocator_;
    Scheduler scheduler_;

    uint64_t internal_failure_cnt_;

protected:
    virtual void registerInternalFailure(const char* msg)
    {
        internal_failure_cnt_++;
        UAVCAN_TRACE("Node", "Internal failure: %s", msg);
        (void)msg;
    }

public:
    /**
     * This overload is only valid if MemPoolSize > 0.
     */
    SubNode(ICanDriver& can_driver,
            ISystemClock& system_clock) :
        scheduler_(can_driver, pool_allocator_, system_clock),
        internal_failure_cnt_(0)
    { }

    /**
     * This overload is only valid if MemPoolSize == 0.
     */
    SubNode(ICanDriver& can_driver,
            ISystemClock& system_clock,
            IPoolAllocator& allocator) :
        pool_allocator_(allocator),
        scheduler_(can_driver, pool_allocator_, system_clock),
        internal_failure_cnt_(0)
    { }

    virtual typename RemoveReference<Allocator>::Type& getAllocator() { return pool_allocator_; }

    virtual Scheduler& getScheduler() { return scheduler_; }
    virtual const Scheduler& getScheduler() const { return scheduler_; }

    uint64_t getInternalFailureCount() const { return internal_failure_cnt_; }
};

}

#endif // Include guard
