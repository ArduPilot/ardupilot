/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/centralized.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include "../../helpers.hpp"
#include "../event_tracer.hpp"
#include "../../helpers.hpp"
#include "../memory_storage_backend.hpp"

using uavcan::dynamic_node_id_server::UniqueID;


TEST(dynamic_node_id_server_centralized_Server, Basic)
{
    using namespace uavcan::dynamic_node_id_server;
    using namespace uavcan::protocol::dynamic_node_id;
    using namespace uavcan::protocol::dynamic_node_id::server;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<Allocation> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg3;

    EventTracer tracer;
    MemoryStorageBackend storage;

    // Node A is Allocator, Node B is Allocatee
    InterlinkedTestNodesWithSysClock nodes(uavcan::NodeID(10), uavcan::NodeID::Broadcast);

    UniqueID own_unique_id;
    own_unique_id[0] = 0xAA;
    own_unique_id[3] = 0xCC;
    own_unique_id[7] = 0xEE;
    own_unique_id[9] = 0xBB;

    /*
     * Server
     */
    uavcan::dynamic_node_id_server::CentralizedServer server(nodes.a, storage, tracer);

    ASSERT_LE(0, server.init(own_unique_id));

    ASSERT_EQ(1, server.getNumAllocations());   // Server's own node ID

    /*
     * Client
     */
    uavcan::DynamicNodeIDClient client(nodes.b);
    uavcan::protocol::HardwareVersion::FieldTypes::unique_id unique_id;
    for (uavcan::uint8_t i = 0; i < unique_id.size(); i++)
    {
        unique_id[i] = i;
    }
    const uavcan::NodeID PreferredNodeID = 42;
    ASSERT_LE(0, client.start(unique_id, PreferredNodeID));

    /*
     * Fire
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(15000));

    ASSERT_TRUE(client.isAllocationComplete());
    ASSERT_EQ(PreferredNodeID, client.getAllocatedNodeID());

    ASSERT_EQ(2, server.getNumAllocations());   // Server's own node ID + client
}


TEST(dynamic_node_id_server_centralized, ObjectSizes)
{
    using namespace uavcan::dynamic_node_id_server;
    std::cout << "centralized::Storage: " << sizeof(centralized::Storage) << std::endl;
    std::cout << "centralized::Server:  " << sizeof(centralized::Server) << std::endl;
    std::cout << "NodeDiscoverer:       " << sizeof(NodeDiscoverer) << std::endl;
}
