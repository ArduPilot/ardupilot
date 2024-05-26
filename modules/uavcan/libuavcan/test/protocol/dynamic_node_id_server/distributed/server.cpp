/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#if __GNUC__
# pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif

#include <gtest/gtest.h>
#include <memory>
#include <uavcan/protocol/dynamic_node_id_server/distributed.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include "../event_tracer.hpp"
#include "../../helpers.hpp"
#include "../memory_storage_backend.hpp"

using uavcan::dynamic_node_id_server::UniqueID;


class CommitHandler : public uavcan::dynamic_node_id_server::distributed::IRaftLeaderMonitor
{
    const std::string id_;

    virtual void handleLogCommitOnLeader(const uavcan::protocol::dynamic_node_id::server::Entry& entry)
    {
        std::cout << "ENTRY COMMITTED [" << id_ << "]\n" << entry << std::endl;
    }

    virtual void handleLocalLeadershipChange(bool local_node_is_leader)
    {
        std::cout << "I AM LEADER [" << id_ << "]: " << (local_node_is_leader ? "YES" : "NOT ANYMORE") << std::endl;
    }

public:
    CommitHandler(const std::string& id) : id_(id) { }
};


TEST(dynamic_node_id_server_RaftCore, Basic)
{
    using namespace uavcan::dynamic_node_id_server::distributed;
    using namespace uavcan::protocol::dynamic_node_id::server;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<Discovery> _reg1;
    uavcan::DefaultDataTypeRegistrator<AppendEntries> _reg2;
    uavcan::DefaultDataTypeRegistrator<RequestVote> _reg3;

    EventTracer tracer_a("a");
    EventTracer tracer_b("b");
    MemoryStorageBackend storage_a;
    MemoryStorageBackend storage_b;
    CommitHandler commit_handler_a("a");
    CommitHandler commit_handler_b("b");

    InterlinkedTestNodesWithSysClock nodes;

    std::unique_ptr<RaftCore> raft_a(new RaftCore(nodes.a, storage_a, tracer_a, commit_handler_a));
    std::unique_ptr<RaftCore> raft_b(new RaftCore(nodes.b, storage_b, tracer_b, commit_handler_b));

    /*
     * Initialization
     */
    ASSERT_LE(0, raft_a->init(2, uavcan::TransferPriority::OneHigherThanLowest));
    ASSERT_LE(0, raft_b->init(2, uavcan::TransferPriority::OneHigherThanLowest));

    /*
     * Running and trying not to fall
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(9000));

    // Either must become a leader
    ASSERT_TRUE(raft_a->isLeader() || raft_b->isLeader());

    ASSERT_EQ(0, raft_a->getCommitIndex());
    ASSERT_EQ(0, raft_b->getCommitIndex());

    /*
     * Adding some stuff
     */
    Entry::FieldTypes::unique_id unique_id;
    uavcan::fill_n(unique_id.begin(), 16, uint8_t(0xAA));

    (raft_a->isLeader() ? raft_a : raft_b)->appendLog(unique_id, uavcan::NodeID(1));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(6000));

    ASSERT_EQ(1, raft_a->getCommitIndex());
    ASSERT_EQ(1, raft_b->getCommitIndex());

    /*
     * Terminating the leader
     */
    raft_a.reset();

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(6000));

    /*
     * Reinitializing the leader - current Follower will become the new Leader
     */
    storage_a.reset();

    raft_a.reset(new RaftCore(nodes.a, storage_a, tracer_a, commit_handler_a));
    ASSERT_LE(0, raft_a->init(2, uavcan::TransferPriority::OneHigherThanLowest));
    ASSERT_EQ(0, raft_a->getCommitIndex());

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(9000));

    ASSERT_FALSE(raft_a->isLeader());
    ASSERT_TRUE(raft_b->isLeader());

    ASSERT_EQ(1, raft_a->getCommitIndex());
    ASSERT_EQ(1, raft_b->getCommitIndex());
}


TEST(dynamic_node_id_server_Server, Basic)
{
    using namespace uavcan::dynamic_node_id_server;
    using namespace uavcan::protocol::dynamic_node_id;
    using namespace uavcan::protocol::dynamic_node_id::server;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<Discovery> _reg1;
    uavcan::DefaultDataTypeRegistrator<AppendEntries> _reg2;
    uavcan::DefaultDataTypeRegistrator<RequestVote> _reg3;
    uavcan::DefaultDataTypeRegistrator<Allocation> _reg4;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg5;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg6;

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
    DistributedServer server(nodes.a, storage, tracer);

    ASSERT_LE(0, server.init(own_unique_id, 1));

    ASSERT_EQ(0, server.getNumAllocations());

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


TEST(dynamic_node_id_server, ObjectSizes)
{
    using namespace uavcan;
    using namespace uavcan::protocol::dynamic_node_id::server;
    using namespace uavcan::dynamic_node_id_server;

    std::cout << "distributed::Log:             " << sizeof(distributed::Log) << std::endl;
    std::cout << "distributed::PersistentState: " << sizeof(distributed::PersistentState) << std::endl;
    std::cout << "distributed::ClusterManager:  " << sizeof(distributed::ClusterManager) << std::endl;
    std::cout << "distributed::RaftCore:        " << sizeof(distributed::RaftCore) << std::endl;
    std::cout << "distributed::Server:          " << sizeof(distributed::Server) << std::endl;
    std::cout << "AllocationRequestManager:     " << sizeof(AllocationRequestManager) << std::endl;

    std::cout << "ServiceServer<AppendEntries>:  " << sizeof(ServiceServer<AppendEntries>) << std::endl;
    std::cout << "ServiceClient<AppendEntries>:  " << sizeof(ServiceClient<AppendEntries>) << std::endl;
    std::cout << "ServiceServer<RequestVote>:    " << sizeof(ServiceServer<RequestVote>) << std::endl;
    std::cout << "ServiceClient<RequestVote>:    " << sizeof(ServiceClient<RequestVote>) << std::endl;
}
