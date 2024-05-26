/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/global_time_sync_master.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include "helpers.hpp"

struct GlobalTimeSyncMasterTestNode
{
    SystemClockDriver clock;
    PairableCanDriver can;
    TestNode node;

    GlobalTimeSyncMasterTestNode(uavcan::NodeID nid)
        : can(clock)
        , node(can, clock, nid)
    { }
};

struct GlobalTimeSyncTestNetwork
{
    GlobalTimeSyncMasterTestNode slave;
    GlobalTimeSyncMasterTestNode master_low;
    GlobalTimeSyncMasterTestNode master_high;

    GlobalTimeSyncTestNetwork()
        : slave(64)
        , master_low(120)
        , master_high(8)
    {
        slave.can.others.insert(&master_low.can);
        master_low.can.others.insert(&slave.can);
        master_high.can.others.insert(&slave.can);
    }

    void spinAll(uavcan::MonotonicDuration duration = uavcan::MonotonicDuration::fromMSec(9))
    {
        assert(!duration.isNegative());
        unsigned nspins3 = unsigned(duration.toMSec() / 3);
        nspins3 = nspins3 ? nspins3 : 2;
        while (nspins3 --> 0)
        {
            ASSERT_LE(0, slave.node.spin(uavcan::MonotonicDuration::fromMSec(1)));
            ASSERT_LE(0, master_low.node.spin(uavcan::MonotonicDuration::fromMSec(1)));
            ASSERT_LE(0, master_high.node.spin(uavcan::MonotonicDuration::fromMSec(1)));
        }
    }
};

TEST(GlobalTimeSyncMaster, Basic)
{
    GlobalTimeSyncTestNetwork nwk;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GlobalTimeSync> _reg1;

    uavcan::GlobalTimeSyncSlave slave(nwk.slave.node);
    uavcan::GlobalTimeSyncMaster master_low(nwk.master_low.node);
    uavcan::GlobalTimeSyncMaster master_high(nwk.master_high.node);

    ASSERT_FALSE(master_low.isInitialized());

    ASSERT_LE(0, slave.start());
    ASSERT_LE(0, master_low.init());
    ASSERT_LE(0, master_high.init());

    ASSERT_TRUE(master_low.isInitialized());
    ASSERT_FALSE(slave.isActive());

    /*
     * Simple synchronization
     */
    ASSERT_LE(0, master_low.publish());  // Update
    nwk.spinAll();

    usleep(400000);
    ASSERT_LE(0, master_low.publish());  // Adjustment
    nwk.spinAll();

    // Synchronization complete.
    ASSERT_TRUE(areTimestampsClose(nwk.slave.clock.getUtc(), nwk.master_low.clock.getUtc()));
    ASSERT_TRUE(slave.isActive());
    ASSERT_EQ(nwk.master_low.node.getNodeID(), slave.getMasterNodeID());

    /*
     * Moving clocks forward and re-syncing with another master
     */
    static const uavcan::UtcDuration OneDay = uavcan::UtcDuration::fromMSec(24 * 3600 * 1000);
    nwk.master_high.clock.utc_adjustment = OneDay;

    usleep(400000);
    ASSERT_LE(0, master_low.publish());   // Update from the old master
    nwk.spinAll();

    ASSERT_LE(0, master_high.publish());  // Update from the new master
    nwk.spinAll();

    usleep(400000);
    ASSERT_LE(0, master_low.publish());   // Adjustment from the old master (ignored now)
    ASSERT_LE(0, master_high.publish());  // Adjustment from the new master (accepted)
    nwk.spinAll();

    // Synchronization complete.
    ASSERT_TRUE(areTimestampsClose(nwk.slave.clock.getUtc(), nwk.master_high.clock.getUtc()));
    ASSERT_FALSE(areTimestampsClose(nwk.slave.clock.getUtc(), nwk.master_low.clock.getUtc()));
    ASSERT_TRUE(slave.isActive());
    ASSERT_EQ(nwk.master_high.node.getNodeID(), slave.getMasterNodeID());

    /*
     * Frequent calls to publish()
     */
    ASSERT_LE(0, master_low.publish()); // Dropped
    ASSERT_LE(0, master_low.publish()); // Dropped
    ASSERT_LE(0, master_low.publish()); // Dropped

    ASSERT_TRUE(nwk.slave.can.read_queue.empty());

    usleep(400000);
    ASSERT_LE(0, master_low.publish()); // Accepted
    ASSERT_FALSE(nwk.slave.can.read_queue.empty());

    nwk.spinAll();

    // Synchronization did not change
    ASSERT_TRUE(areTimestampsClose(nwk.slave.clock.getUtc(), nwk.master_high.clock.getUtc()));
    ASSERT_FALSE(areTimestampsClose(nwk.slave.clock.getUtc(), nwk.master_low.clock.getUtc()));
    ASSERT_TRUE(slave.isActive());
    ASSERT_EQ(nwk.master_high.node.getNodeID(), slave.getMasterNodeID());
}
