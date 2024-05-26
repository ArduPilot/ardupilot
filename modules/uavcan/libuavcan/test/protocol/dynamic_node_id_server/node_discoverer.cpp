/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#if __GNUC__
# pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif

#include <gtest/gtest.h>
#include <vector>
#include <uavcan/protocol/dynamic_node_id_server/node_discoverer.hpp>
#include <uavcan/node/publisher.hpp>
#include "event_tracer.hpp"
#include "get_node_info_mock_server.hpp"
#include "../helpers.hpp"

using namespace uavcan::dynamic_node_id_server;


class NodeDiscoveryHandler : public uavcan::dynamic_node_id_server::INodeDiscoveryHandler
{
public:
    struct NodeInfo
    {
        UniqueID unique_id;
        uavcan::NodeID node_id;
        bool committed;

        NodeInfo() : committed(false) { }
    };

    bool can_discover;
    std::vector<NodeInfo> nodes;

    NodeDiscoveryHandler() : can_discover(false) { }

    virtual bool canDiscoverNewNodes() const
    {
        return can_discover;
    }

    virtual NodeAwareness checkNodeAwareness(uavcan::NodeID node_id) const
    {
        const NodeInfo* const ni = const_cast<NodeDiscoveryHandler*>(this)->findNode(node_id);
        if (ni == UAVCAN_NULLPTR)
        {
            return NodeAwarenessUnknown;
        }
        return ni->committed ? NodeAwarenessKnownAndCommitted : NodeAwarenessKnownButNotCommitted;
    }

    virtual void handleNewNodeDiscovery(const UniqueID* unique_id_or_null, uavcan::NodeID node_id)
    {
        NodeInfo info;
        if (unique_id_or_null != UAVCAN_NULLPTR)
        {
            info.unique_id = *unique_id_or_null;
        }
        info.node_id = node_id;
        nodes.push_back(info);
    }

    NodeInfo* findNode(const UniqueID& unique_id)
    {
        for (unsigned i = 0; i < nodes.size(); i++)
        {
            if (nodes.at(i).unique_id == unique_id)
            {
                return &nodes.at(i);
            }
        }
        return UAVCAN_NULLPTR;
    }

    NodeInfo* findNode(const uavcan::NodeID& node_id)
    {
        for (unsigned i = 0; i < nodes.size(); i++)
        {
            if (nodes.at(i).node_id == node_id)
            {
                return &nodes.at(i);
            }
        }
        return UAVCAN_NULLPTR;
    }
};


TEST(dynamic_node_id_server_NodeDiscoverer, Basic)
{
    using namespace uavcan::protocol::dynamic_node_id::server;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;

    EventTracer tracer;
    InterlinkedTestNodesWithSysClock nodes;
    NodeDiscoveryHandler handler;

    NodeDiscoverer disc(nodes.a, tracer, handler);

    /*
     * Initialization
     */
    ASSERT_LE(0, disc.init(uavcan::TransferPriority::OneHigherThanLowest));

    ASSERT_FALSE(disc.hasUnknownNodes());

    /*
     * Publishing NodeStatus, discovery is disabled
     */
    std::cout << "!!! Publishing NodeStatus, discovery is disabled" << std::endl;
    handler.can_discover = false;

    uavcan::Publisher<uavcan::protocol::NodeStatus> node_status_pub(nodes.b);
    ASSERT_LE(0, node_status_pub.init());

    uavcan::protocol::NodeStatus node_status;
    node_status.uptime_sec = 0;
    ASSERT_LE(0, node_status_pub.broadcast(node_status));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1100));

    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNewNodeFound));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryTimerStart));   // The timer runs as long as there are unknown nodes
    ASSERT_EQ(0, tracer.countEvents(TraceDiscoveryGetNodeInfoRequest)); // Querying is disabled!
    ASSERT_EQ(0, tracer.countEvents(TraceDiscoveryGetNodeInfoFailure));
    ASSERT_TRUE(disc.hasUnknownNodes());

    /*
     * Enabling discovery - the querying will continue despite the fact that NodeStatus messages are not arriving
     */
    std::cout << "!!! Enabling discovery" << std::endl;
    handler.can_discover = true;

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1150));

    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNewNodeFound));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryTimerStart));
    ASSERT_EQ(0, tracer.countEvents(TraceDiscoveryTimerStop));
    ASSERT_EQ(2, tracer.countEvents(TraceDiscoveryGetNodeInfoRequest));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryGetNodeInfoFailure));
    ASSERT_TRUE(disc.hasUnknownNodes());

    /*
     * Publishing NodeStatus
     */
    std::cout << "!!! Publishing NodeStatus" << std::endl;

    node_status.uptime_sec += 5U;
    ASSERT_LE(0, node_status_pub.broadcast(node_status));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1250));

    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNewNodeFound));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryTimerStart));
    ASSERT_EQ(0, tracer.countEvents(TraceDiscoveryTimerStop));
    ASSERT_EQ(3, tracer.countEvents(TraceDiscoveryGetNodeInfoRequest));
    ASSERT_EQ(2, tracer.countEvents(TraceDiscoveryGetNodeInfoFailure));
    ASSERT_TRUE(disc.hasUnknownNodes());

    /*
     * Publishing NodeStatus, discovery is enabled, GetNodeInfo mock server is initialized
     */
    std::cout << "!!! Publishing NodeStatus, discovery is enabled, GetNodeInfo mock server is initialized" << std::endl;

    GetNodeInfoMockServer get_node_info_server(nodes.b);
    get_node_info_server.response.hardware_version.unique_id[0] = 123;  // Arbitrary data
    get_node_info_server.response.hardware_version.unique_id[6] = 213;
    get_node_info_server.response.hardware_version.unique_id[14] = 52;
    ASSERT_LE(0, get_node_info_server.start());

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));

    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNewNodeFound));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryTimerStart));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryTimerStop));
    ASSERT_EQ(4, tracer.countEvents(TraceDiscoveryGetNodeInfoRequest));
    ASSERT_EQ(3, tracer.countEvents(TraceDiscoveryGetNodeInfoFailure));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNodeFinalized));
    ASSERT_FALSE(disc.hasUnknownNodes());

    /*
     * Checking the results
     */
    ASSERT_TRUE(handler.findNode(get_node_info_server.response.hardware_version.unique_id));
    ASSERT_EQ(2, handler.findNode(get_node_info_server.response.hardware_version.unique_id)->node_id.get());
}


TEST(dynamic_node_id_server_NodeDiscoverer, RestartAndMaxAttempts)
{
    using namespace uavcan::protocol::dynamic_node_id::server;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;

    EventTracer tracer;
    InterlinkedTestNodesWithSysClock nodes;
    NodeDiscoveryHandler handler;

    NodeDiscoverer disc(nodes.a, tracer, handler);

    /*
     * Initialization
     */
    ASSERT_LE(0, disc.init(uavcan::TransferPriority::OneHigherThanLowest));

    ASSERT_FALSE(disc.hasUnknownNodes());

    /*
     * Publishing NodeStatus once to trigger querying
     * Querying for 2 seconds, no responses will be sent (there's no server)
     */
    handler.can_discover = true;

    uavcan::Publisher<uavcan::protocol::NodeStatus> node_status_pub(nodes.b);
    ASSERT_LE(0, node_status_pub.init());

    uavcan::protocol::NodeStatus node_status;
    node_status.uptime_sec = 10;                        // Nonzero
    ASSERT_LE(0, node_status_pub.broadcast(node_status));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(3100));

    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNewNodeFound));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryTimerStart));
    ASSERT_EQ(0, tracer.countEvents(TraceDiscoveryTimerStop));
    ASSERT_EQ(4, tracer.countEvents(TraceDiscoveryGetNodeInfoRequest));
    ASSERT_EQ(3, tracer.countEvents(TraceDiscoveryGetNodeInfoFailure));
    ASSERT_EQ(0, tracer.countEvents(TraceDiscoveryNodeFinalized));
    ASSERT_EQ(0, tracer.countEvents(TraceDiscoveryNodeRestartDetected));
    ASSERT_TRUE(disc.hasUnknownNodes());

    /*
     * Emulating node restart
     */
    node_status.uptime_sec = 9;                         // Less than previous
    ASSERT_LE(0, node_status_pub.broadcast(node_status));

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(3100));

    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNewNodeFound));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryTimerStart));
    ASSERT_EQ(0, tracer.countEvents(TraceDiscoveryTimerStop));
    ASSERT_EQ(7, tracer.countEvents(TraceDiscoveryGetNodeInfoRequest));
    ASSERT_EQ(6, tracer.countEvents(TraceDiscoveryGetNodeInfoFailure));
    ASSERT_EQ(0, tracer.countEvents(TraceDiscoveryNodeFinalized));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNodeRestartDetected));
    ASSERT_TRUE(disc.hasUnknownNodes());

    /*
     * Waiting for timeout
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(3100));

    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNewNodeFound));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryTimerStart));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryTimerStop));
    ASSERT_EQ(8, tracer.countEvents(TraceDiscoveryGetNodeInfoRequest));
    ASSERT_EQ(8, tracer.countEvents(TraceDiscoveryGetNodeInfoFailure));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNodeFinalized));
    ASSERT_EQ(1, tracer.countEvents(TraceDiscoveryNodeRestartDetected));
    ASSERT_FALSE(disc.hasUnknownNodes());

    /*
     * Checking the results
     */
    ASSERT_TRUE(handler.findNode(UniqueID()));
    ASSERT_EQ(2, handler.findNode(UniqueID())->node_id.get());
}


TEST(dynamic_node_id_server_NodeDiscoverer, Sizes)
{
    using namespace uavcan;

    std::cout << "BitSet<NodeID::Max + 1>:              " << sizeof(BitSet<NodeID::Max + 1>) << std::endl;
    std::cout << "ServiceClient<protocol::GetNodeInfo>: " << sizeof(ServiceClient<protocol::GetNodeInfo>) << std::endl;
    std::cout << "protocol::GetNodeInfo::Response:      " << sizeof(protocol::GetNodeInfo::Response) << std::endl;
    std::cout << "Subscriber<protocol::NodeStatus>:     " << sizeof(Subscriber<protocol::NodeStatus>) << std::endl;
}
