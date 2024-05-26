/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_SERVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_SERVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/dynamic_node_id_server/allocation_request_manager.hpp>
#include <uavcan/protocol/dynamic_node_id_server/node_discoverer.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{

class AbstractServer : protected IAllocationRequestHandler
                     , protected INodeDiscoveryHandler
{
    UniqueID own_unique_id_;
    MonotonicTime started_at_;

protected:
    INode& node_;
    IEventTracer& tracer_;
    AllocationRequestManager allocation_request_manager_;
    NodeDiscoverer node_discoverer_;

    AbstractServer(INode& node,
                   IEventTracer& tracer) :
       node_(node),
       tracer_(tracer),
       allocation_request_manager_(node, tracer, *this),
       node_discoverer_(node, tracer, *this)
    { }

    const UniqueID& getOwnUniqueID() const { return own_unique_id_; }

    int init(const UniqueID& own_unique_id, const TransferPriority priority)
    {
        int res = 0;

        own_unique_id_ = own_unique_id;

        res = allocation_request_manager_.init(priority);
        if (res < 0)
        {
            return res;
        }

        res = node_discoverer_.init(priority);
        if (res < 0)
        {
            return res;
        }

        started_at_ = node_.getMonotonicTime();

        return 0;
    }

public:
    /**
     * This can be used to guess if there are any un-allocated dynamic nodes left in the network.
     */
    bool guessIfAllDynamicNodesAreAllocated(
        const MonotonicDuration& allocation_activity_timeout =
            MonotonicDuration::fromMSec(Allocation::MAX_REQUEST_PERIOD_MS * 2),
        const MonotonicDuration& min_uptime = MonotonicDuration::fromMSec(6000)) const
    {
        const MonotonicTime ts = node_.getMonotonicTime();

        /*
         * If uptime is not large enough, the allocator may be unaware about some nodes yet.
         */
        const MonotonicDuration uptime = ts - started_at_;
        if (uptime < max(allocation_activity_timeout, min_uptime))
        {
            return false;
        }

        /*
         * If there are any undiscovered nodes, assume that allocation is still happening.
         */
        if (node_discoverer_.hasUnknownNodes())
        {
            return false;
        }

        /*
         * Lastly, check if there wasn't any allocation messages detected on the bus in the specified amount of time.
         */
        const MonotonicDuration since_allocation_activity =
            ts - allocation_request_manager_.getTimeOfLastAllocationActivity();
        if (since_allocation_activity < allocation_activity_timeout)
        {
            return false;
        }

        return true;
    }

    /**
     * This is useful for debugging/testing/monitoring.
     */
    const NodeDiscoverer& getNodeDiscoverer() const { return node_discoverer_; }
};

}
}

#endif // Include guard
