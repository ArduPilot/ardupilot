/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#if __GNUC__
# pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif

#include <memory>
#include <gtest/gtest.h>
#include <uavcan/protocol/node_info_retriever.hpp>
#include <uavcan/protocol/node_status_provider.hpp>
#include "helpers.hpp"

static void publishNodeStatus(PairableCanDriver& can, uavcan::NodeID node_id,
                              uavcan::uint32_t uptime_sec, uavcan::TransferID tid)
{
    uavcan::protocol::NodeStatus msg;
    msg.health     = uavcan::protocol::NodeStatus::HEALTH_OK;
    msg.mode       = uavcan::protocol::NodeStatus::MODE_OPERATIONAL;
    msg.uptime_sec = uptime_sec;
    emulateSingleFrameBroadcastTransfer(can, node_id, msg, tid);
}


struct NodeInfoListener : public uavcan::INodeInfoListener
{
    std::unique_ptr<uavcan::protocol::GetNodeInfo::Response> last_node_info;
    uavcan::NodeID last_node_id;
    unsigned status_message_cnt;
    unsigned status_change_cnt;
    unsigned info_unavailable_cnt;

    NodeInfoListener()
        : status_message_cnt(0)
        , status_change_cnt(0)
        , info_unavailable_cnt(0)
    { }

    virtual void handleNodeInfoRetrieved(uavcan::NodeID node_id,
                                         const uavcan::protocol::GetNodeInfo::Response& node_info)
    {
        last_node_id = node_id;
        std::cout << node_info << std::endl;
        last_node_info.reset(new uavcan::protocol::GetNodeInfo::Response(node_info));
    }

    virtual void handleNodeInfoUnavailable(uavcan::NodeID node_id)
    {
        std::cout << "NODE INFO FOR " << int(node_id.get()) << " IS UNAVAILABLE" << std::endl;
        last_node_id = node_id;
        info_unavailable_cnt++;
    }

    virtual void handleNodeStatusChange(const uavcan::NodeStatusMonitor::NodeStatusChangeEvent& event)
    {
        std::cout << "NODE " << int(event.node_id.get()) << " STATUS CHANGE: "
                  << event.old_status.toString() << " --> " << event.status.toString() << std::endl;
        status_change_cnt++;
    }

    virtual void handleNodeStatusMessage(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
    {
        std::cout << msg << std::endl;
        status_message_cnt++;
    }
};


TEST(NodeInfoRetriever, Basic)
{
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;

    InterlinkedTestNodesWithSysClock nodes;

    uavcan::NodeInfoRetriever retr(nodes.a);
    std::cout << "sizeof(uavcan::NodeInfoRetriever): " << sizeof(uavcan::NodeInfoRetriever) << std::endl;
    std::cout << "sizeof(uavcan::ServiceClient<uavcan::protocol::GetNodeInfo>): "
        << sizeof(uavcan::ServiceClient<uavcan::protocol::GetNodeInfo>) << std::endl;

    std::unique_ptr<uavcan::NodeStatusProvider> provider(new uavcan::NodeStatusProvider(nodes.b));

    NodeInfoListener listener;

    /*
     * Initialization
     */
    ASSERT_LE(0, retr.start());

    retr.removeListener(&listener);     // Does nothing
    retr.addListener(&listener);
    retr.addListener(&listener);
    retr.addListener(&listener);
    ASSERT_EQ(1, retr.getNumListeners());

    uavcan::protocol::HardwareVersion hwver;
    hwver.unique_id[0] = 123;
    hwver.unique_id[4] = 213;
    hwver.unique_id[8] = 45;

    provider->setName("Ivan");
    provider->setHardwareVersion(hwver);

    ASSERT_LE(0, provider->startAndPublish());

    ASSERT_FALSE(retr.isRetrievingInProgress());
    ASSERT_EQ(0, retr.getNumPendingRequests());

    EXPECT_EQ(40, retr.getRequestInterval().toMSec());  // Default

    /*
     * Waiting for discovery
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(50));
    ASSERT_TRUE(retr.isRetrievingInProgress());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1500));
    ASSERT_FALSE(retr.isRetrievingInProgress());

    ASSERT_EQ(2, listener.status_message_cnt);
    ASSERT_EQ(1, listener.status_change_cnt);
    ASSERT_EQ(0, listener.info_unavailable_cnt);
    ASSERT_TRUE(listener.last_node_info.get());
    ASSERT_EQ(uavcan::NodeID(2), listener.last_node_id);
    ASSERT_EQ("Ivan", listener.last_node_info->name);
    ASSERT_TRUE(hwver == listener.last_node_info->hardware_version);

    provider.reset();   // Moving the provider out of the way; its entry will timeout meanwhile

    /*
     * Declaring a bunch of different nodes that don't support GetNodeInfo
     */
    ASSERT_FALSE(retr.isRetrievingInProgress());

    retr.setNumRequestAttempts(3);

    uavcan::TransferID tid;

    publishNodeStatus(nodes.can_a, uavcan::NodeID(10), 10, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(11), 10, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(12), 10, tid);

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_LE(1, retr.getNumPendingRequests());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_LE(2, retr.getNumPendingRequests());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_EQ(3, retr.getNumPendingRequests());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(retr.isRetrievingInProgress());

    tid.increment();
    publishNodeStatus(nodes.can_a, uavcan::NodeID(10), 11, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(11), 11, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(12), 11, tid);

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_LE(1, retr.getNumPendingRequests());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_LE(2, retr.getNumPendingRequests());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_EQ(3, retr.getNumPendingRequests());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(retr.isRetrievingInProgress());

    tid.increment();
    publishNodeStatus(nodes.can_a, uavcan::NodeID(10), 12, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(11), 12, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(12), 10, tid);     // Reset

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_LE(1, retr.getNumPendingRequests());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_LE(2, retr.getNumPendingRequests());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_EQ(3, retr.getNumPendingRequests());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1000));
    ASSERT_TRUE(retr.isRetrievingInProgress());

    EXPECT_EQ(11, listener.status_message_cnt);
    EXPECT_EQ(5, listener.status_change_cnt);           // node 2 online/offline + 3 test nodes above
    EXPECT_EQ(2, listener.info_unavailable_cnt);

    tid.increment();
    publishNodeStatus(nodes.can_a, uavcan::NodeID(12), 11, tid);

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_EQ(1, retr.getNumPendingRequests());
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(40));
    ASSERT_EQ(1, retr.getNumPendingRequests());                         // Still one because two went offline
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1200));
    ASSERT_TRUE(retr.isRetrievingInProgress());

    tid.increment();
    publishNodeStatus(nodes.can_a, uavcan::NodeID(12), 12, tid);

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1200));
    ASSERT_FALSE(retr.isRetrievingInProgress());                // Out of attempts, stopping
    ASSERT_EQ(0, retr.getNumPendingRequests());

    EXPECT_EQ(13, listener.status_message_cnt);
    EXPECT_EQ(7, listener.status_change_cnt);        // node 2 online/offline + 2 test nodes above online/offline + 1
    EXPECT_EQ(3, listener.info_unavailable_cnt);

    /*
     * Forcing the class to forget everything
     */
    std::cout << "Invalidation" << std::endl;

    retr.invalidateAll();

    ASSERT_FALSE(retr.isRetrievingInProgress());
    ASSERT_EQ(0, retr.getNumPendingRequests());

    tid.increment();
    publishNodeStatus(nodes.can_a, uavcan::NodeID(10), 60, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(11), 60, tid);
    publishNodeStatus(nodes.can_a, uavcan::NodeID(12), 60, tid);

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(200));

    ASSERT_TRUE(retr.isRetrievingInProgress());
    ASSERT_EQ(3, retr.getNumPendingRequests());
}


TEST(NodeInfoRetriever, MaxConcurrentRequests)
{
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;

    InterlinkedTestNodesWithSysClock nodes;

    uavcan::NodeInfoRetriever retr(nodes.a);
    std::cout << "sizeof(uavcan::NodeInfoRetriever): " << sizeof(uavcan::NodeInfoRetriever) << std::endl;
    std::cout << "sizeof(uavcan::ServiceClient<uavcan::protocol::GetNodeInfo>): "
        << sizeof(uavcan::ServiceClient<uavcan::protocol::GetNodeInfo>) << std::endl;

    NodeInfoListener listener;

    /*
     * Initialization
     */
    ASSERT_LE(0, retr.start());

    retr.addListener(&listener);
    ASSERT_EQ(1, retr.getNumListeners());

    ASSERT_FALSE(retr.isRetrievingInProgress());
    ASSERT_EQ(0, retr.getNumPendingRequests());

    ASSERT_EQ(40, retr.getRequestInterval().toMSec());

    const unsigned MaxPendingRequests = 26;             // See class docs
    const unsigned MinPendingRequestsAtFullLoad = 12;

    /*
     * Sending a lot of requests, making sure that the number of concurrent calls does not exceed the specified limit.
     */
    for (uint8_t node_id = 1U; node_id <= 127U; node_id++)
    {
        publishNodeStatus(nodes.can_a, node_id, 0, uavcan::TransferID());
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
        ASSERT_GE(MaxPendingRequests, retr.getNumPendingRequests());
        ASSERT_TRUE(retr.isRetrievingInProgress());
    }

    ASSERT_GE(MaxPendingRequests, retr.getNumPendingRequests());
    ASSERT_LE(MinPendingRequestsAtFullLoad, retr.getNumPendingRequests());

    for (int i = 0; i < 8; i++)      // Approximate
    {
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(35));
        std::cout << "!!! SPIN " << i << " COMPLETE" << std::endl;

        ASSERT_GE(MaxPendingRequests, retr.getNumPendingRequests());
        ASSERT_LE(MinPendingRequestsAtFullLoad, retr.getNumPendingRequests());

        ASSERT_TRUE(retr.isRetrievingInProgress());
    }

    ASSERT_LT(0, retr.getNumPendingRequests());
    ASSERT_TRUE(retr.isRetrievingInProgress());

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(5000));

    ASSERT_EQ(0, retr.getNumPendingRequests());
    ASSERT_FALSE(retr.isRetrievingInProgress());
}
