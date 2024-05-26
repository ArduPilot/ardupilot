/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include "helpers.hpp"


TEST(DynamicNodeIDClient, Basic)
{
    // Node A is Allocator, Node B is Allocatee
    InterlinkedTestNodesWithSysClock nodes(uavcan::NodeID(10), uavcan::NodeID::Broadcast);

    uavcan::DynamicNodeIDClient dnidac(nodes.b);

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::dynamic_node_id::Allocation> _reg1;
    (void)_reg1;

    /*
     * Client initialization
     */
    uavcan::protocol::HardwareVersion::FieldTypes::unique_id unique_id;

    ASSERT_LE(-uavcan::ErrInvalidParam, dnidac.start(unique_id));  // Empty hardware version is not allowed

    for (uavcan::uint8_t i = 0; i < unique_id.size(); i++)
    {
        unique_id[i] = i;
    }

    ASSERT_LE(-uavcan::ErrInvalidParam, dnidac.start(unique_id, uavcan::NodeID()));

    const uavcan::NodeID PreferredNodeID = 42;
    ASSERT_LE(0, dnidac.start(unique_id, PreferredNodeID));

    ASSERT_FALSE(dnidac.getAllocatedNodeID().isValid());
    ASSERT_FALSE(dnidac.getAllocatorNodeID().isValid());
    ASSERT_FALSE(dnidac.isAllocationComplete());

    /*
     * Subscriber (server emulation)
     */
    SubscriberWithCollector<uavcan::protocol::dynamic_node_id::Allocation> dynid_sub(nodes.a);
    ASSERT_LE(0, dynid_sub.start());
    dynid_sub.subscriber.allowAnonymousTransfers();

    /*
     * Monitoring requests
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1500));
    ASSERT_TRUE(dynid_sub.collector.msg.get());
    std::cout << "First-stage request:\n" << *dynid_sub.collector.msg << std::endl;
    ASSERT_EQ(PreferredNodeID.get(), dynid_sub.collector.msg->node_id);
    ASSERT_TRUE(dynid_sub.collector.msg->first_part_of_unique_id);
    ASSERT_TRUE(uavcan::equal(dynid_sub.collector.msg->unique_id.begin(),
                              dynid_sub.collector.msg->unique_id.end(),
                              unique_id.begin()));
    dynid_sub.collector.msg.reset();

    // Second - rate is no lower than 0.5 Hz
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1500));
    ASSERT_TRUE(dynid_sub.collector.msg.get());
    dynid_sub.collector.msg.reset();

    ASSERT_FALSE(dnidac.getAllocatedNodeID().isValid());
    ASSERT_FALSE(dnidac.getAllocatorNodeID().isValid());
    ASSERT_FALSE(dnidac.isAllocationComplete());

    /*
     * Publisher (server emulation)
     */
    uavcan::Publisher<uavcan::protocol::dynamic_node_id::Allocation> dynid_pub(nodes.a);
    ASSERT_LE(0, dynid_pub.init());

    /*
     * Sending some some Allocation messages - the timer will keep restarting
     */
    for (int i = 0; i < 10; i++)
    {
        uavcan::protocol::dynamic_node_id::Allocation msg;  // Contents of the message doesn't matter
        ASSERT_LE(0, dynid_pub.broadcast(msg));
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(210));
        ASSERT_FALSE(dynid_sub.collector.msg.get());
    }

    /*
     * Responding with partially matching unique ID - the client will respond with second-stage request immediately
     */
    const uint8_t BytesPerRequest = uavcan::protocol::dynamic_node_id::Allocation::MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;
    {
        uavcan::protocol::dynamic_node_id::Allocation msg;
        msg.unique_id.resize(BytesPerRequest);
        uavcan::copy(unique_id.begin(), unique_id.begin() + BytesPerRequest, msg.unique_id.begin());

        std::cout << "First-stage offer:\n" << msg << std::endl;

        ASSERT_FALSE(dynid_sub.collector.msg.get());
        ASSERT_LE(0, dynid_pub.broadcast(msg));
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(500));

        ASSERT_TRUE(dynid_sub.collector.msg.get());
        std::cout << "Second-stage request:\n" << *dynid_sub.collector.msg << std::endl;
        ASSERT_EQ(PreferredNodeID.get(), dynid_sub.collector.msg->node_id);
        ASSERT_FALSE(dynid_sub.collector.msg->first_part_of_unique_id);
        ASSERT_TRUE(uavcan::equal(dynid_sub.collector.msg->unique_id.begin(),
                                  dynid_sub.collector.msg->unique_id.end(),
                                  unique_id.begin() + BytesPerRequest));
        dynid_sub.collector.msg.reset();
    }

    /*
     * Responding with second-stage offer, expecting the last request back
     */
    {
        uavcan::protocol::dynamic_node_id::Allocation msg;
        msg.unique_id.resize(BytesPerRequest * 2);
        uavcan::copy(unique_id.begin(), unique_id.begin() + BytesPerRequest * 2, msg.unique_id.begin());

        std::cout << "Second-stage offer:\n" << msg << std::endl;

        ASSERT_FALSE(dynid_sub.collector.msg.get());
        ASSERT_LE(0, dynid_pub.broadcast(msg));
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(500));

        ASSERT_TRUE(dynid_sub.collector.msg.get());
        std::cout << "Last request:\n" << *dynid_sub.collector.msg << std::endl;
        ASSERT_EQ(PreferredNodeID.get(), dynid_sub.collector.msg->node_id);
        ASSERT_FALSE(dynid_sub.collector.msg->first_part_of_unique_id);
        ASSERT_TRUE(uavcan::equal(dynid_sub.collector.msg->unique_id.begin(),
                                  dynid_sub.collector.msg->unique_id.end(),
                                  unique_id.begin() + BytesPerRequest * 2));
        dynid_sub.collector.msg.reset();
    }

    ASSERT_FALSE(dnidac.getAllocatedNodeID().isValid());
    ASSERT_FALSE(dnidac.getAllocatorNodeID().isValid());
    ASSERT_FALSE(dnidac.isAllocationComplete());

    /*
     * Now we have full unique ID for this client received, and it is possible to grant allocation
     */
    {
        uavcan::protocol::dynamic_node_id::Allocation msg;
        msg.unique_id.resize(16);
        msg.node_id = 72;
        uavcan::copy(unique_id.begin(), unique_id.end(), msg.unique_id.begin());

        ASSERT_FALSE(dynid_sub.collector.msg.get());
        ASSERT_LE(0, dynid_pub.broadcast(msg));
        nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2000));
        ASSERT_FALSE(dynid_sub.collector.msg.get());
    }

    ASSERT_EQ(uavcan::NodeID(72), dnidac.getAllocatedNodeID());
    ASSERT_EQ(uavcan::NodeID(10), dnidac.getAllocatorNodeID());
    ASSERT_TRUE(dnidac.isAllocationComplete());
}


TEST(DynamicNodeIDClient, NonPassiveMode)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::DynamicNodeIDClient dnidac(nodes.b);

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::dynamic_node_id::Allocation> _reg1;
    (void)_reg1;

    uavcan::protocol::HardwareVersion::FieldTypes::unique_id unique_id;
    for (uavcan::uint8_t i = 0; i < unique_id.size(); i++)
    {
        unique_id[i] = i;
    }

    ASSERT_LE(-uavcan::ErrLogic, dnidac.start(unique_id));
}
