/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_server/distributed/cluster_manager.hpp>
#include "../event_tracer.hpp"
#include "../../helpers.hpp"
#include "../memory_storage_backend.hpp"

TEST(dynamic_node_id_server_ClusterManager, Initialization)
{
    using namespace uavcan::dynamic_node_id_server::distributed;

    const unsigned MaxClusterSize =
        uavcan::protocol::dynamic_node_id::server::Discovery::FieldTypes::known_nodes::MaxSize;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::dynamic_node_id::server::Discovery> _reg1;

    EventTracer tracer;

    /*
     * Simple initialization
     */
    {
        MemoryStorageBackend storage;
        Log log(storage, tracer);
        InterlinkedTestNodesWithSysClock nodes;

        ClusterManager mgr(nodes.a, storage, log, tracer);

        // Too big
        ASSERT_GT(0, mgr.init(MaxClusterSize + 1, uavcan::TransferPriority::OneHigherThanLowest));
        ASSERT_EQ(0, storage.getNumKeys());

        // OK
        ASSERT_LE(0, mgr.init(5, uavcan::TransferPriority::OneHigherThanLowest));
        ASSERT_EQ(1, storage.getNumKeys());
        ASSERT_EQ("5", storage.get("cluster_size"));

        // Testing other states
        ASSERT_EQ(0, mgr.getNumKnownServers());
        ASSERT_EQ(5, mgr.getClusterSize());
        ASSERT_EQ(3, mgr.getQuorumSize());
        ASSERT_FALSE(mgr.getRemoteServerNodeIDAtIndex(0).isValid());
    }
    /*
     * Recovery from the storage
     */
    {
        MemoryStorageBackend storage;
        Log log(storage, tracer);
        InterlinkedTestNodesWithSysClock nodes;

        ClusterManager mgr(nodes.a, storage, log, tracer);

        // Not configured
        ASSERT_GT(0, mgr.init(0, uavcan::TransferPriority::OneHigherThanLowest));
        ASSERT_EQ(0, storage.getNumKeys());

        // OK
        storage.set("cluster_size", "5");
        ASSERT_LE(0, mgr.init(0, uavcan::TransferPriority::OneHigherThanLowest));
        ASSERT_EQ(1, storage.getNumKeys());
    }
}


TEST(dynamic_node_id_server_ClusterManager, OneServer)
{
    using namespace uavcan::dynamic_node_id_server::distributed;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::dynamic_node_id::server::Discovery> _reg1;

    EventTracer tracer;
    MemoryStorageBackend storage;
    Log log(storage, tracer);
    InterlinkedTestNodesWithSysClock nodes;

    ClusterManager mgr(nodes.a, storage, log, tracer);

    /*
     * Pub and sub
     */
    SubscriberWithCollector<uavcan::protocol::dynamic_node_id::server::Discovery> sub(nodes.b);
    uavcan::Publisher<uavcan::protocol::dynamic_node_id::server::Discovery> pub(nodes.b);

    ASSERT_LE(0, sub.start());
    ASSERT_LE(0, pub.init());

    /*
     * Starting
     */
    ASSERT_LE(0, mgr.init(1, uavcan::TransferPriority::OneHigherThanLowest));

    ASSERT_EQ(0, mgr.getNumKnownServers());
    ASSERT_TRUE(mgr.isClusterDiscovered());

    ASSERT_EQ(0, nodes.a.internal_failure_count);

    /*
     * Broadcasting discovery with wrong cluster size, it will be reported as internal failure
     */
    uavcan::protocol::dynamic_node_id::server::Discovery msg;
    msg.configured_cluster_size = 2;
    msg.known_nodes.push_back(2U);
    ASSERT_LE(0, pub.broadcast(msg));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));

    ASSERT_EQ(1, nodes.a.internal_failure_count);

    /*
     * Discovery rate limiting test
     */
    ASSERT_FALSE(sub.collector.msg.get());

    msg = uavcan::protocol::dynamic_node_id::server::Discovery();
    msg.configured_cluster_size = 1;              // Correct value
    ASSERT_LE(0, pub.broadcast(msg));             // List of known nodes is empty, intentionally

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));
    ASSERT_FALSE(sub.collector.msg.get());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(1, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(1, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    sub.collector.msg.reset();

    // Rinse repeat
    ASSERT_LE(0, pub.broadcast(msg));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));
    ASSERT_FALSE(sub.collector.msg.get());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(1, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(1, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    sub.collector.msg.reset();
}


TEST(dynamic_node_id_server_ClusterManager, ThreeServers)
{
    using namespace uavcan::dynamic_node_id_server::distributed;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::dynamic_node_id::server::Discovery> _reg1;

    EventTracer tracer;
    MemoryStorageBackend storage;
    Log log(storage, tracer);
    InterlinkedTestNodesWithSysClock nodes;

    ClusterManager mgr(nodes.a, storage, log, tracer);

    /*
     * Pub and sub
     */
    SubscriberWithCollector<uavcan::protocol::dynamic_node_id::server::Discovery> sub(nodes.b);
    uavcan::Publisher<uavcan::protocol::dynamic_node_id::server::Discovery> pub(nodes.b);

    ASSERT_LE(0, sub.start());
    ASSERT_LE(0, pub.init());

    /*
     * Starting
     */
    ASSERT_LE(0, mgr.init(3, uavcan::TransferPriority::OneHigherThanLowest));

    ASSERT_EQ(0, mgr.getNumKnownServers());
    ASSERT_FALSE(mgr.isClusterDiscovered());

    /*
     * Discovery publishing rate check
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));
    ASSERT_FALSE(sub.collector.msg.get());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(3, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(1, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    sub.collector.msg.reset();

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(100));
    ASSERT_FALSE(sub.collector.msg.get());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(3, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(1, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    sub.collector.msg.reset();

    /*
     * Discovering other nodes
     */
    uavcan::protocol::dynamic_node_id::server::Discovery msg;
    msg.configured_cluster_size = 3;
    msg.known_nodes.push_back(2U);
    ASSERT_LE(0, pub.broadcast(msg));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1050));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(3, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(2, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    ASSERT_EQ(2, sub.collector.msg->known_nodes[1]);
    sub.collector.msg.reset();

    ASSERT_FALSE(mgr.isClusterDiscovered());

    // This will complete the discovery
    msg.known_nodes.push_back(127U);
    ASSERT_LE(0, pub.broadcast(msg));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1050));
    ASSERT_TRUE(sub.collector.msg.get());
    ASSERT_EQ(3, sub.collector.msg->configured_cluster_size);
    ASSERT_EQ(3, sub.collector.msg->known_nodes.size());
    ASSERT_EQ(1, sub.collector.msg->known_nodes[0]);
    ASSERT_EQ(2, sub.collector.msg->known_nodes[1]);
    ASSERT_EQ(127, sub.collector.msg->known_nodes[2]);
    sub.collector.msg.reset();

    // Making sure discovery is now terminated
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1500));
    ASSERT_FALSE(sub.collector.msg.get());

    /*
     * Checking Raft states
     */
    ASSERT_EQ(uavcan::NodeID(2),   mgr.getRemoteServerNodeIDAtIndex(0));
    ASSERT_EQ(uavcan::NodeID(127), mgr.getRemoteServerNodeIDAtIndex(1));
    ASSERT_EQ(uavcan::NodeID(),    mgr.getRemoteServerNodeIDAtIndex(2));

    ASSERT_EQ(0, mgr.getServerMatchIndex(2));
    ASSERT_EQ(0, mgr.getServerMatchIndex(127));

    ASSERT_EQ(log.getLastIndex() + 1, mgr.getServerNextIndex(2));
    ASSERT_EQ(log.getLastIndex() + 1, mgr.getServerNextIndex(127));

    mgr.setServerMatchIndex(2, 10);
    ASSERT_EQ(10, mgr.getServerMatchIndex(2));

    mgr.incrementServerNextIndexBy(2, 5);
    ASSERT_EQ(log.getLastIndex() + 1 + 5, mgr.getServerNextIndex(2));
    mgr.decrementServerNextIndex(2);
    ASSERT_EQ(log.getLastIndex() + 1 + 5 - 1, mgr.getServerNextIndex(2));

    mgr.resetAllServerIndices();

    ASSERT_EQ(0, mgr.getServerMatchIndex(2));
    ASSERT_EQ(0, mgr.getServerMatchIndex(127));

    ASSERT_EQ(log.getLastIndex() + 1, mgr.getServerNextIndex(2));
    ASSERT_EQ(log.getLastIndex() + 1, mgr.getServerNextIndex(127));
}
