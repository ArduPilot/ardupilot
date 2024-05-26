/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/node_status_provider.hpp>
#include "helpers.hpp"

static void publishNodeStatus(CanDriverMock& can, uavcan::NodeID node_id,
                              uavcan::uint8_t health, uavcan::uint8_t mode,
                              uavcan::uint32_t uptime_sec, uavcan::TransferID tid)
{
    uavcan::protocol::NodeStatus msg;
    msg.health     = health;
    msg.mode       = mode;
    msg.uptime_sec = uptime_sec;
    emulateSingleFrameBroadcastTransfer(can, node_id, msg, tid);
}


static void shortSpin(TestNode& node)
{
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));
}


TEST(NodeStatusMonitor, Basic)
{
    using uavcan::protocol::NodeStatus;
    using uavcan::NodeID;

    SystemClockMock clock_mock(100);
    clock_mock.monotonic_auto_advance = 1000;

    CanDriverMock can(2, clock_mock);

    TestNode node(can, clock_mock, 64);

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg1;

    uavcan::NodeStatusMonitor nsm(node);
    ASSERT_LE(0, nsm.start());

    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    /*
     * Empty NSM, no nodes were registered yet
     */
    ASSERT_FALSE(nsm.findNodeWithWorstHealth().isValid());

    ASSERT_FALSE(nsm.isNodeKnown(uavcan::NodeID(123)));
    ASSERT_EQ(NodeStatus::MODE_OFFLINE, nsm.getNodeStatus(uavcan::NodeID(123)).mode);

    /*
     * Some new status messages
     */
    publishNodeStatus(can, 10, NodeStatus::HEALTH_OK, NodeStatus::MODE_OPERATIONAL, 12, 0);
    shortSpin(node);
    ASSERT_EQ(NodeID(10), nsm.findNodeWithWorstHealth());

    publishNodeStatus(can, 9,  NodeStatus::HEALTH_WARNING, NodeStatus::MODE_INITIALIZATION, 0, 0);
    shortSpin(node);
    ASSERT_EQ(NodeID(9), nsm.findNodeWithWorstHealth());

    publishNodeStatus(can, 11, NodeStatus::HEALTH_CRITICAL, NodeStatus::MODE_MAINTENANCE, 999, 0);
    shortSpin(node);
    ASSERT_EQ(NodeID(11), nsm.findNodeWithWorstHealth());

    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(10)));
    ASSERT_EQ(NodeStatus::MODE_OPERATIONAL, nsm.getNodeStatus(uavcan::NodeID(10)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_OK, nsm.getNodeStatus(uavcan::NodeID(10)).health);

    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(9)));
    ASSERT_EQ(NodeStatus::MODE_INITIALIZATION, nsm.getNodeStatus(uavcan::NodeID(9)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_WARNING, nsm.getNodeStatus(uavcan::NodeID(9)).health);

    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(11)));
    ASSERT_EQ(NodeStatus::MODE_MAINTENANCE, nsm.getNodeStatus(uavcan::NodeID(11)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_CRITICAL, nsm.getNodeStatus(uavcan::NodeID(11)).health);

    /*
     * Timeout
     */
    std::cout << "Starting timeout test, current monotime is " << clock_mock.monotonic << std::endl;

    clock_mock.advance(500000);
    shortSpin(node);
    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(10)));
    ASSERT_EQ(NodeStatus::MODE_OPERATIONAL, nsm.getNodeStatus(uavcan::NodeID(10)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_OK, nsm.getNodeStatus(uavcan::NodeID(10)).health);

    clock_mock.advance(500000);
    shortSpin(node);
    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(9)));
    ASSERT_EQ(NodeStatus::MODE_INITIALIZATION, nsm.getNodeStatus(uavcan::NodeID(9)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_WARNING, nsm.getNodeStatus(uavcan::NodeID(9)).health);

    clock_mock.advance(500000);
    shortSpin(node);
    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(11)));
    ASSERT_EQ(NodeStatus::MODE_MAINTENANCE, nsm.getNodeStatus(uavcan::NodeID(11)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_CRITICAL, nsm.getNodeStatus(uavcan::NodeID(11)).health);

    /*
     * Will timeout now
     */
    clock_mock.advance(4000000);
    shortSpin(node);

    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(10)));
    ASSERT_EQ(NodeStatus::MODE_OFFLINE, nsm.getNodeStatus(uavcan::NodeID(10)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_OK, nsm.getNodeStatus(uavcan::NodeID(10)).health);

    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(9)));
    ASSERT_EQ(NodeStatus::MODE_OFFLINE, nsm.getNodeStatus(uavcan::NodeID(9)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_WARNING, nsm.getNodeStatus(uavcan::NodeID(9)).health);

    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(11)));
    ASSERT_EQ(NodeStatus::MODE_OFFLINE, nsm.getNodeStatus(uavcan::NodeID(11)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_CRITICAL, nsm.getNodeStatus(uavcan::NodeID(11)).health);

    /*
     * Recovering one node, adding two extra
     */
    publishNodeStatus(can, 11, NodeStatus::HEALTH_WARNING, NodeStatus::MODE_OPERATIONAL, 999, 1);
    shortSpin(node);

    publishNodeStatus(can, 127, NodeStatus::HEALTH_WARNING, NodeStatus::MODE_OPERATIONAL, 9999, 1);
    shortSpin(node);

    publishNodeStatus(can, 1, NodeStatus::HEALTH_OK, NodeStatus::MODE_OPERATIONAL, 1234, 1);
    shortSpin(node);

    /*
     * Making sure OFFLINE is still worst status
     */
    ASSERT_EQ(NodeID(9), nsm.findNodeWithWorstHealth());

    /*
     * Final validation
     */
    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(10)));
    ASSERT_EQ(NodeStatus::MODE_OFFLINE, nsm.getNodeStatus(uavcan::NodeID(10)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_OK, nsm.getNodeStatus(uavcan::NodeID(10)).health);

    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(9)));
    ASSERT_EQ(NodeStatus::MODE_OFFLINE, nsm.getNodeStatus(uavcan::NodeID(9)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_WARNING, nsm.getNodeStatus(uavcan::NodeID(9)).health);

    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(11)));
    ASSERT_EQ(NodeStatus::MODE_OPERATIONAL, nsm.getNodeStatus(uavcan::NodeID(11)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_WARNING, nsm.getNodeStatus(uavcan::NodeID(11)).health);

    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(127)));
    ASSERT_EQ(NodeStatus::MODE_OPERATIONAL, nsm.getNodeStatus(uavcan::NodeID(127)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_WARNING, nsm.getNodeStatus(uavcan::NodeID(127)).health);

    ASSERT_TRUE(nsm.isNodeKnown(uavcan::NodeID(1)));
    ASSERT_EQ(NodeStatus::MODE_OPERATIONAL, nsm.getNodeStatus(uavcan::NodeID(1)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_OK, nsm.getNodeStatus(uavcan::NodeID(1)).health);

    /*
     * Forgetting
     */
    nsm.forgetNode(127);
    ASSERT_FALSE(nsm.isNodeKnown(uavcan::NodeID(127)));
    ASSERT_EQ(NodeStatus::MODE_OFFLINE, nsm.getNodeStatus(uavcan::NodeID(127)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_CRITICAL, nsm.getNodeStatus(uavcan::NodeID(127)).health);

    nsm.forgetNode(9);
    ASSERT_FALSE(nsm.isNodeKnown(uavcan::NodeID(9)));
    ASSERT_EQ(NodeStatus::MODE_OFFLINE, nsm.getNodeStatus(uavcan::NodeID(9)).mode);
    ASSERT_EQ(NodeStatus::HEALTH_CRITICAL, nsm.getNodeStatus(uavcan::NodeID(9)).health);
}
