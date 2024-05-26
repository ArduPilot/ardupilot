/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/node/publisher.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include "helpers.hpp"


TEST(GlobalTimeSyncSlave, Basic)
{
    InterlinkedTestNodesWithClockMock nodes(64, 65);

    SystemClockMock& slave_clock = nodes.clock_a;
    SystemClockMock& master_clock = nodes.clock_b;

    slave_clock.advance(1000000);
    master_clock.advance(1000000);

    master_clock.monotonic_auto_advance = slave_clock.monotonic_auto_advance = 1000;
    master_clock.preserve_utc = slave_clock.preserve_utc = true;
    slave_clock.utc = 0; // Not set yet

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GlobalTimeSync> _reg1;

    uavcan::GlobalTimeSyncSlave gtss(nodes.a);
    uavcan::Publisher<uavcan::protocol::GlobalTimeSync> gts_pub(nodes.b);

    ASSERT_LE(0, gtss.start());
    ASSERT_FALSE(gtss.isActive());
    ASSERT_FALSE(gtss.getMasterNodeID().isValid());

    /*
     * Empty broadcast
     * The slave must only register the timestamp and adjust nothing
     */
    uavcan::protocol::GlobalTimeSync gts;
    gts.previous_transmission_timestamp_usec = 0;
    gts_pub.broadcast(gts);
    gts.previous_transmission_timestamp_usec = master_clock.utc;
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_EQ(0, slave_clock.utc);
    ASSERT_EQ(1000000, master_clock.utc);
    std::cout << "Master mono=" << master_clock.monotonic << " utc=" << master_clock.utc << std::endl;
    std::cout << "Slave  mono=" << slave_clock.monotonic  << " utc=" << slave_clock.utc << std::endl;

    ASSERT_FALSE(gtss.isActive());
    ASSERT_FALSE(gtss.getMasterNodeID().isValid());

    /*
     * Follow-up broadcast with proper time
     * Slave must adjust now
     */
    gts_pub.broadcast(gts);
    gts.previous_transmission_timestamp_usec = master_clock.utc;
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(1000000, slave_clock.utc);
    ASSERT_EQ(1000000, master_clock.utc);
    std::cout << "Master mono=" << master_clock.monotonic << " utc=" << master_clock.utc << std::endl;
    std::cout << "Slave  mono=" << slave_clock.monotonic  << " utc=" << slave_clock.utc << std::endl;

    master_clock.utc += 1000000;
    slave_clock.utc += 1000000;

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(nodes.b.getNodeID(), gtss.getMasterNodeID());

    /*
     * Next follow-up, slave is synchronized now
     * Will update
     */
    gts_pub.broadcast(gts);
    gts.previous_transmission_timestamp_usec = master_clock.utc;
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(2000000, slave_clock.utc);
    ASSERT_EQ(2000000, master_clock.utc);

    master_clock.utc += 1000000;
    slave_clock.utc += 1000000;

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(nodes.b.getNodeID(), gtss.getMasterNodeID());

    /*
     * Next follow-up, slave is synchronized now
     * Will adjust
     */
    gts_pub.broadcast(gts);
    gts.previous_transmission_timestamp_usec = master_clock.utc;
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(3000000, slave_clock.utc);
    ASSERT_EQ(3000000, master_clock.utc);

    master_clock.utc += 1000000;
    slave_clock.utc += 1000000;
    ASSERT_EQ(4000000, slave_clock.utc);
    ASSERT_EQ(4000000, master_clock.utc);

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(nodes.b.getNodeID(), gtss.getMasterNodeID());

    /*
     * Another master
     * This one has higher priority, so it will be preferred
     */
    SystemClockMock master2_clock(100);
    master2_clock.monotonic_auto_advance = 1000;
    master2_clock.preserve_utc = true;
    PairableCanDriver master2_can(master2_clock);
    master2_can.others.insert(&nodes.can_a);
    TestNode master2_node(master2_can, master2_clock, 8);

    uavcan::Publisher<uavcan::protocol::GlobalTimeSync> gts_pub2(master2_node);

    /*
     * Update step, no adjustment yet
     */
    gts.previous_transmission_timestamp_usec = 0;
    gts_pub2.broadcast(gts);
    gts.previous_transmission_timestamp_usec = master2_clock.utc;
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(4000000, slave_clock.utc);
    ASSERT_EQ(100, master2_clock.utc);

    master2_clock.utc += 1000000;

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(master2_node.getNodeID(), gtss.getMasterNodeID());

    /*
     * Adjustment
     */
    gts_pub2.broadcast(gts);
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(100, slave_clock.utc);

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(master2_node.getNodeID(), gtss.getMasterNodeID());

    /*
     * Another master will be ignored now
     */
    gts.previous_transmission_timestamp_usec = 99999999;
    // Update
    gts_pub.broadcast(gts);
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(100, slave_clock.utc);
    // Adjust
    gts_pub.broadcast(gts);
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(100, slave_clock.utc);

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(master2_node.getNodeID(), gtss.getMasterNodeID());

    /*
     * Timeout
     */
    slave_clock.advance(100000000);

    ASSERT_FALSE(gtss.isActive());
    ASSERT_FALSE(gtss.getMasterNodeID().isValid());
}


#if !defined(BYTE_ORDER) || !defined(LITTLE_ENDIAN) || (BYTE_ORDER != LITTLE_ENDIAN)
# error "This test cannot be executed on this platform"
#endif

static uavcan::Frame makeSyncMsg(uavcan::uint64_t usec, uavcan::NodeID snid, uavcan::TransferID tid)
{
    uavcan::Frame frame(uavcan::protocol::GlobalTimeSync::DefaultDataTypeID, uavcan::TransferTypeMessageBroadcast,
                        snid, uavcan::NodeID::Broadcast, tid);
    frame.setStartOfTransfer(true);
    frame.setEndOfTransfer(true);
    EXPECT_EQ(7, frame.setPayload(reinterpret_cast<uint8_t*>(&usec), 7)); // Assuming little endian!!!
    return frame;
}

static void broadcastSyncMsg(CanIfaceMock& iface, uavcan::uint64_t usec, uavcan::NodeID snid, uavcan::TransferID tid)
{
    const uavcan::Frame frame = makeSyncMsg(usec, snid, tid);
    uavcan::CanFrame can_frame;
    ASSERT_TRUE(frame.compile(can_frame));
    iface.pushRx(can_frame);
}


TEST(GlobalTimeSyncSlave, Validation)
{
    SystemClockMock slave_clock;
    slave_clock.monotonic = 1000000;
    slave_clock.preserve_utc = true;

    CanDriverMock slave_can(3, slave_clock);
    for (uint8_t i = 0; i < slave_can.getNumIfaces(); i++)
    {
        slave_can.ifaces.at(i).enable_utc_timestamping = true;
    }

    TestNode node(slave_can, slave_clock, 64);

    uavcan::GlobalTimeSyncSlave gtss(node);
    uavcan::Publisher<uavcan::protocol::GlobalTimeSync> gts_pub(node);

    ASSERT_LE(0, gtss.start());
    ASSERT_FALSE(gtss.isActive());
    ASSERT_FALSE(gtss.getMasterNodeID().isValid());
    ASSERT_EQ(0, slave_clock.utc);

    /*
     * Update/adjust/update
     */
    broadcastSyncMsg(slave_can.ifaces.at(0), 0, 8, 0);    // Locked on this
    broadcastSyncMsg(slave_can.ifaces.at(1), 2000, 8, 0); // Ignored
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    broadcastSyncMsg(slave_can.ifaces.at(0), 1000, 8, 1); // Adjust 1000 ahead
    broadcastSyncMsg(slave_can.ifaces.at(1), 2000, 8, 1); // Ignored
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(8, gtss.getMasterNodeID().get());
    ASSERT_EQ(1000, slave_clock.utc);

    broadcastSyncMsg(slave_can.ifaces.at(0), 2000, 8, 2); // Update
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    ASSERT_EQ(1000, slave_clock.utc);
    std::cout << slave_clock.monotonic << std::endl;

    /*
     * TID jump simulates a frame loss
     */
    broadcastSyncMsg(slave_can.ifaces.at(0), 3000, 8, 4); // Adjustment skipped - expected TID 3, update instead
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(8, gtss.getMasterNodeID().get());
    ASSERT_EQ(1000, slave_clock.utc);
    std::cout << slave_clock.monotonic << std::endl;

    /*
     * Valid adjustment - continuing from TID 4
     */
    broadcastSyncMsg(slave_can.ifaces.at(0), 3000, 8, 5);  // Slave UTC was 1000, master reports 3000 --> shift ahead
    broadcastSyncMsg(slave_can.ifaces.at(1), 2000, 8, 5);
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(8, gtss.getMasterNodeID().get());
    ASSERT_EQ(3000, slave_clock.utc);
    std::cout << slave_clock.monotonic << std::endl;

    /*
     * Update, then very long delay with correct TID
     */
    broadcastSyncMsg(slave_can.ifaces.at(0), 2000, 8, 6); // Valid update, slave UTC is 3000
    broadcastSyncMsg(slave_can.ifaces.at(1), 2000, 8, 6);
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    slave_clock.monotonic += 5000000;

    broadcastSyncMsg(slave_can.ifaces.at(0), 5000, 8, 7); // Adjustment skipped
    broadcastSyncMsg(slave_can.ifaces.at(1), 2000, 8, 7);
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    broadcastSyncMsg(slave_can.ifaces.at(0), 5000, 8, 8); // Valid adjustment now
    broadcastSyncMsg(slave_can.ifaces.at(1), 2000, 8, 8);
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(8, gtss.getMasterNodeID().get());
    ASSERT_EQ(5000, slave_clock.utc);
    std::cout << slave_clock.monotonic << std::endl;
}


TEST(GlobalTimeSyncSlave, Suppression)
{
    SystemClockMock slave_clock;
    slave_clock.monotonic = 1000000;
    slave_clock.preserve_utc = true;

    CanDriverMock slave_can(3, slave_clock);
    for (uint8_t i = 0; i < slave_can.getNumIfaces(); i++)
    {
        slave_can.ifaces.at(i).enable_utc_timestamping = true;
    }

    TestNode node(slave_can, slave_clock, 64);

    uavcan::GlobalTimeSyncSlave gtss(node);
    uavcan::Publisher<uavcan::protocol::GlobalTimeSync> gts_pub(node);

    ASSERT_LE(0, gtss.start());
    ASSERT_EQ(0, slave_clock.utc);

    gtss.suppress(true);

    broadcastSyncMsg(slave_can.ifaces.at(0), 0, 8, 0);    // Locked on this
    broadcastSyncMsg(slave_can.ifaces.at(1), 2000, 8, 0); // Ignored
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    broadcastSyncMsg(slave_can.ifaces.at(0), 1000, 8, 1); // Adjust 1000 ahead
    broadcastSyncMsg(slave_can.ifaces.at(1), 2000, 8, 1); // Ignored
    ASSERT_LE(0, node.spin(uavcan::MonotonicDuration::fromMSec(10)));

    ASSERT_TRUE(gtss.isActive());
    ASSERT_EQ(8, gtss.getMasterNodeID().get());
    ASSERT_EQ(0, slave_clock.utc);                  // The clock shall not be asjusted
}
