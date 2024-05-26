/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/allocation_request_manager.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include "event_tracer.hpp"
#include "../helpers.hpp"


using uavcan::dynamic_node_id_server::UniqueID;

class AllocationRequestHandler : public uavcan::dynamic_node_id_server::IAllocationRequestHandler
{
    std::vector<std::pair<UniqueID, uavcan::NodeID> > requests_;

public:
    bool can_followup;

    AllocationRequestHandler() : can_followup(false) { }

    virtual void handleAllocationRequest(const UniqueID& unique_id, uavcan::NodeID preferred_node_id)
    {
        requests_.push_back(std::pair<UniqueID, uavcan::NodeID>(unique_id, preferred_node_id));
    }

    virtual bool canPublishFollowupAllocationResponse() const
    {
        return can_followup;
    }

    bool matchAndPopLastRequest(const UniqueID& unique_id, uavcan::NodeID preferred_node_id)
    {
        if (requests_.empty())
        {
            std::cout << "No pending requests" << std::endl;
            return false;
        }

        const std::pair<UniqueID, uavcan::NodeID> pair = requests_.at(requests_.size() - 1U);
        requests_.pop_back();

        if (pair.first != unique_id)
        {
            std::cout << "Unique ID mismatch" << std::endl;
            return false;
        }

        if (pair.second != preferred_node_id)
        {
            std::cout << "Node ID mismatch (" << pair.second.get() << ", " << preferred_node_id.get() << ")"
                << std::endl;
            return false;
        }

        return true;
    }

    void reset() { requests_.clear(); }
};


TEST(dynamic_node_id_server_AllocationRequestManager, Basic)
{
    using namespace uavcan::protocol::dynamic_node_id;
    using namespace uavcan::protocol::dynamic_node_id::server;
    using namespace uavcan::dynamic_node_id_server;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<Allocation> _reg1;

    // Node A is Allocator, Node B is Allocatee
    InterlinkedTestNodesWithSysClock nodes(uavcan::NodeID(10), uavcan::NodeID::Broadcast);

    uavcan::DynamicNodeIDClient client(nodes.b);

    /*
     * Client initialization
     */
    uavcan::protocol::HardwareVersion::FieldTypes::unique_id unique_id;
    for (uavcan::uint8_t i = 0; i < unique_id.size(); i++)
    {
        unique_id[i] = i;
    }
    const uavcan::NodeID PreferredNodeID = 42;
    ASSERT_LE(0, client.start(unique_id, PreferredNodeID));

    /*
     * Request manager initialization
     */
    EventTracer tracer;
    AllocationRequestHandler handler;
    handler.can_followup = true;

    AllocationRequestManager manager(nodes.a, tracer, handler);

    ASSERT_LE(0, manager.init(uavcan::TransferPriority::OneHigherThanLowest));

    /*
     * Allocation
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2000));

    ASSERT_TRUE(handler.matchAndPopLastRequest(unique_id, PreferredNodeID));

    ASSERT_LE(0, manager.broadcastAllocationResponse(unique_id, PreferredNodeID));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));

    /*
     * Checking the client
     */
    ASSERT_TRUE(client.isAllocationComplete());

    ASSERT_EQ(PreferredNodeID, client.getAllocatedNodeID());
}
