/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include "can.hpp"

static bool rxFrameEquals(const uavcan::CanRxFrame& rxframe, const uavcan::CanFrame& frame,
                          uint64_t timestamp_usec, int iface_index)
{
    if (static_cast<const uavcan::CanFrame&>(rxframe) != frame)
    {
        std::cout << "Frame mismatch:\n"
                  << "    " << rxframe.toString(uavcan::CanFrame::StrAligned) << "\n"
                  << "    " << frame.toString(uavcan::CanFrame::StrAligned) << std::endl;
    }
    return (static_cast<const uavcan::CanFrame&>(rxframe) == frame) &&
           (rxframe.ts_mono == uavcan::MonotonicTime::fromUSec(timestamp_usec)) &&
           (rxframe.iface_index == iface_index);
}

TEST(CanIOManager, Reception)
{
    // Memory
    uavcan::PoolAllocator<sizeof(uavcan::CanTxQueue::Entry) * 4, sizeof(uavcan::CanTxQueue::Entry)> pool;

    // Platform interface
    SystemClockMock clockmock;
    CanDriverMock driver(2, clockmock);

    // IO Manager
    uavcan::CanIOManager iomgr(driver, pool, clockmock);
    ASSERT_EQ(2, iomgr.getNumIfaces());

    /*
     * Empty, will time out
     */
    uavcan::CanRxFrame frame;
    uavcan::CanIOFlags flags = uavcan::CanIOFlags();
    EXPECT_EQ(0, iomgr.receive(frame, tsMono(100), flags));
    EXPECT_EQ(0, flags);
    EXPECT_EQ(100, clockmock.monotonic);
    EXPECT_EQ(100, clockmock.utc);

    /*
     * Non empty from multiple ifaces
     */
    const uavcan::CanFrame frames[2][3] = {
        { makeCanFrame(1, "a0", EXT),    makeCanFrame(99, "a1", EXT),  makeCanFrame(803, "a2", STD) },
        { makeCanFrame(6341, "b0", EXT), makeCanFrame(196, "b1", STD), makeCanFrame(73, "b2", EXT) },
    };

    clockmock.advance(10);
    driver.ifaces.at(0).pushRx(frames[0][0]);  // Timestamp 110
    driver.ifaces.at(1).pushRx(frames[1][0]);
    clockmock.advance(10);
    driver.ifaces.at(0).pushRx(frames[0][1]);  // Timestamp 120
    driver.ifaces.at(1).pushRx(frames[1][1]);
    clockmock.advance(10);
    driver.ifaces.at(0).pushRx(frames[0][2]);  // Timestamp 130
    driver.ifaces.at(1).pushRx(frames[1][2]);
    clockmock.advance(10);

    EXPECT_EQ(1, iomgr.receive(frame, uavcan::MonotonicTime(), flags));
    EXPECT_TRUE(rxFrameEquals(frame, frames[0][0], 110, 0));
    EXPECT_EQ(0, flags);

    EXPECT_EQ(1, iomgr.receive(frame, uavcan::MonotonicTime(), flags));
    EXPECT_TRUE(rxFrameEquals(frame, frames[0][1], 120, 0));
    EXPECT_EQ(0, flags);

    EXPECT_EQ(1, iomgr.receive(frame, uavcan::MonotonicTime(), flags));
    EXPECT_TRUE(rxFrameEquals(frame, frames[0][2], 130, 0));
    EXPECT_EQ(0, flags);

    EXPECT_EQ(1, iomgr.receive(frame, uavcan::MonotonicTime(), flags));
    EXPECT_TRUE(rxFrameEquals(frame, frames[1][0], 110, 1));
    EXPECT_EQ(0, flags);

    EXPECT_EQ(1, iomgr.receive(frame, uavcan::MonotonicTime(), flags));
    EXPECT_TRUE(rxFrameEquals(frame, frames[1][1], 120, 1));
    EXPECT_EQ(0, flags);

    EXPECT_EQ(1, iomgr.receive(frame, uavcan::MonotonicTime(), flags));
    EXPECT_TRUE(rxFrameEquals(frame, frames[1][2], 130, 1));
    EXPECT_EQ(0, flags);

    EXPECT_EQ(0, iomgr.receive(frame, uavcan::MonotonicTime(), flags));  // Will time out
    EXPECT_EQ(0, flags);

    /*
     * Perf counters
     */
    driver.select_failure = true;
    EXPECT_EQ(-uavcan::ErrDriver, iomgr.receive(frame, uavcan::MonotonicTime(), flags));

    driver.select_failure = false;
    driver.ifaces.at(1).pushRx(frames[0][0]);
    driver.ifaces.at(1).rx_failure = true;
    EXPECT_EQ(-uavcan::ErrDriver, iomgr.receive(frame, uavcan::MonotonicTime(), flags));

    driver.ifaces.at(0).num_errors = 9000;
    driver.ifaces.at(1).num_errors = 100500;
    EXPECT_EQ(9000, iomgr.getIfacePerfCounters(0).errors);
    EXPECT_EQ(100500, iomgr.getIfacePerfCounters(1).errors);

    EXPECT_EQ(3, iomgr.getIfacePerfCounters(0).frames_rx);
    EXPECT_EQ(3, iomgr.getIfacePerfCounters(1).frames_rx);

    EXPECT_EQ(0, iomgr.getIfacePerfCounters(0).frames_tx);
    EXPECT_EQ(0, iomgr.getIfacePerfCounters(1).frames_tx);
}

TEST(CanIOManager, Transmission)
{
    using uavcan::CanIOManager;
    using uavcan::CanTxQueue;

    // Memory
    uavcan::PoolAllocator<sizeof(CanTxQueue::Entry) * 4, sizeof(CanTxQueue::Entry)> pool;

    // Platform interface
    SystemClockMock clockmock;
    CanDriverMock driver(2, clockmock);

    // IO Manager
    CanIOManager iomgr(driver, pool, clockmock, 9999);
    ASSERT_EQ(2, iomgr.getNumIfaces());

    const int ALL_IFACES_MASK = 3;

    const uavcan::CanFrame frames[] = {
        makeCanFrame(1, "a0", EXT),    makeCanFrame(99, "a1", EXT),  makeCanFrame(803, "a2", STD)
    };

    uavcan::CanIOFlags flags = uavcan::CanIOFlags();

    /*
     * Simple transmission
     */
    EXPECT_EQ(2, iomgr.send(frames[0], tsMono(100), tsMono(0), ALL_IFACES_MASK, CanTxQueue::Volatile, flags));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[0], 100));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 100));
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[0]));
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[0]));

    EXPECT_EQ(1, iomgr.send(frames[1], tsMono(200), tsMono(100), 2, CanTxQueue::Persistent, flags));  // To #1 only
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[1], 200));
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(uavcan::CanFrame()));
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[1]));

    EXPECT_EQ(0, clockmock.monotonic);
    EXPECT_EQ(0, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(0, iomgr.getIfacePerfCounters(0).errors);
    EXPECT_EQ(0, iomgr.getIfacePerfCounters(1).errors);

    /*
     * TX Queue basics
     */
    EXPECT_EQ(0, pool.getNumUsedBlocks());

    // Sending to both, #0 blocked
    driver.ifaces.at(0).writeable = false;
    EXPECT_LT(0, iomgr.send(frames[0], tsMono(201), tsMono(200), ALL_IFACES_MASK, CanTxQueue::Persistent, flags));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 201));
    EXPECT_EQ(200, clockmock.monotonic);
    EXPECT_EQ(200, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(1, pool.getNumUsedBlocks());          // One frame went into TX queue, and will expire soon
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[0]));          // This one will persist
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(uavcan::CanFrame())); // This will drop off on the second select()

    // Sending to both, both blocked
    driver.ifaces.at(1).writeable = false;
    EXPECT_EQ(0, iomgr.send(frames[1], tsMono(777), tsMono(300), ALL_IFACES_MASK, CanTxQueue::Volatile, flags));
    EXPECT_EQ(3, pool.getNumUsedBlocks());          // Total 3 frames in TX queue now
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[0])); // Still 0
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[1])); // 1!!

    // Sending to #0, both blocked
    EXPECT_EQ(0, iomgr.send(frames[2], tsMono(888), tsMono(400), 1, CanTxQueue::Persistent, flags));
    EXPECT_EQ(400, clockmock.monotonic);
    EXPECT_EQ(400, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(4, pool.getNumUsedBlocks());
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[0]));
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[1]));

    // At this time TX queues are containing the following data:
    // iface 0: frames[0] (EXPIRED), frames[1], frames[2]
    // iface 1: frames[1]

    // Sending to #1, both writeable
    driver.ifaces.at(0).writeable = true;
    driver.ifaces.at(1).writeable = true;
    // One frame per each iface will be sent:
    EXPECT_LT(0, iomgr.send(frames[0], tsMono(999), tsMono(500), 2, CanTxQueue::Persistent, flags));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[1], 777));   // Note that frame[0] on iface #0 has expired
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 999));   // In different order due to prioritization
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[0]));       // Expired but still will be reported
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[0]));

    // Calling receive() to flush the rest two frames
    uavcan::CanRxFrame dummy_rx_frame;
    EXPECT_EQ(0, iomgr.receive(dummy_rx_frame, tsMono(0), flags));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[2], 888));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[1], 777));
    ASSERT_EQ(0, flags);
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[2]));
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[1]));

    // Final checks
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(0, pool.getNumUsedBlocks());              // Make sure the memory was properly released
    EXPECT_EQ(1, iomgr.getIfacePerfCounters(0).errors); // This is because of expired frame[0]
    EXPECT_EQ(0, iomgr.getIfacePerfCounters(1).errors);

    /*
     * TX Queue updates from receive() call
     */
    driver.ifaces.at(0).writeable = false;
    driver.ifaces.at(1).writeable = false;

    // Sending 5 frames, one will be rejected
    EXPECT_EQ(0, iomgr.send(frames[2], tsMono(2222), tsMono(1000), ALL_IFACES_MASK, CanTxQueue::Persistent, flags));
    EXPECT_EQ(0, iomgr.send(frames[0], tsMono(3333), tsMono(1100), 2, CanTxQueue::Persistent, flags));
    // One frame kicked here:
    EXPECT_EQ(0, iomgr.send(frames[1], tsMono(4444), tsMono(1200), ALL_IFACES_MASK, CanTxQueue::Volatile, flags));

    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[1]));
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[0]));

    // State checks
    EXPECT_EQ(4, pool.getNumUsedBlocks());          // TX queue is full
    EXPECT_EQ(1200, clockmock.monotonic);
    EXPECT_EQ(1200, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());

    // Preparing the driver mock for receive() call
    driver.ifaces.at(0).writeable = true;
    driver.ifaces.at(1).writeable = true;
    const uavcan::CanFrame rx_frames[] = { makeCanFrame(123, "rx0", STD), makeCanFrame(321, "rx1", EXT) };
    driver.ifaces.at(0).pushRx(rx_frames[0]);
    driver.ifaces.at(1).pushRx(rx_frames[1]);

    // This shall transmit _some_ frames now, at least one per iface (exact number can be changed - it will be OK)
    uavcan::CanRxFrame rx_frame;
    EXPECT_EQ(1, iomgr.receive(rx_frame, tsMono(0), flags));                         // Non-blocking
    EXPECT_TRUE(rxFrameEquals(rx_frame, rx_frames[0], 1200, 0));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[1], 4444));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 3333));
    ASSERT_EQ(0, flags);
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[1]));
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[0]));

    EXPECT_EQ(1, iomgr.receive(rx_frame, tsMono(0), flags));
    EXPECT_TRUE(rxFrameEquals(rx_frame, rx_frames[1], 1200, 1));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[2], 2222));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[2], 2222));  // Iface #1, frame[1] was rejected (VOLATILE)
    ASSERT_EQ(0, flags);
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[2]));
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[2]));

    // State checks
    EXPECT_EQ(0, pool.getNumUsedBlocks());          // TX queue is empty
    EXPECT_EQ(1200, clockmock.monotonic);
    EXPECT_EQ(1200, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(1, iomgr.getIfacePerfCounters(0).errors);
    EXPECT_EQ(1, iomgr.getIfacePerfCounters(1).errors); // This is because of rejected frame[1]

    /*
     * Error handling
     */
    // Select failure
    driver.select_failure = true;
    EXPECT_EQ(-uavcan::ErrDriver, iomgr.receive(rx_frame, tsMono(2000), flags));
    EXPECT_EQ(-uavcan::ErrDriver,
              iomgr.send(frames[0], tsMono(2100), tsMono(2000), ALL_IFACES_MASK, CanTxQueue::Volatile, flags));
    EXPECT_EQ(1200, clockmock.monotonic);
    EXPECT_EQ(1200, clockmock.utc);
    ASSERT_EQ(0, flags);
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[0]));
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[0]));

    // Transmission failure
    driver.select_failure = false;
    driver.ifaces.at(0).writeable = true;
    driver.ifaces.at(1).writeable = true;
    driver.ifaces.at(0).tx_failure = true;
    driver.ifaces.at(1).tx_failure = true;
    // Non-blocking - return < 0
    EXPECT_GE(0, iomgr.send(frames[0], tsMono(2200), tsMono(0), ALL_IFACES_MASK, CanTxQueue::Persistent, flags));
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(frames[0]));
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(frames[0]));

    ASSERT_EQ(2, pool.getNumUsedBlocks());               // Untransmitted frames will be buffered

    // Failure removed - transmission shall proceed
    driver.ifaces.at(0).tx_failure = false;
    driver.ifaces.at(1).tx_failure = false;
    EXPECT_EQ(0, iomgr.receive(rx_frame, tsMono(2500), flags));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[0], 2200));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 2200));
    EXPECT_EQ(0, pool.getNumUsedBlocks());               // All transmitted
    ASSERT_EQ(0, flags);
    EXPECT_TRUE(driver.ifaces.at(0).matchPendingTx(uavcan::CanFrame()));        // Last call will be receive-only,
    EXPECT_TRUE(driver.ifaces.at(1).matchPendingTx(uavcan::CanFrame()));        // hence empty TX

    /*
     * Perf counters
     */
    EXPECT_EQ(1, iomgr.getIfacePerfCounters(0).frames_rx);
    EXPECT_EQ(1, iomgr.getIfacePerfCounters(1).frames_rx);

    EXPECT_EQ(6, iomgr.getIfacePerfCounters(0).frames_tx);
    EXPECT_EQ(8, iomgr.getIfacePerfCounters(1).frames_tx);
}

TEST(CanIOManager, Loopback)
{
    using uavcan::CanIOManager;
    using uavcan::CanTxQueue;
    using uavcan::CanFrame;
    using uavcan::CanRxFrame;

    // Memory
    uavcan::PoolAllocator<sizeof(CanTxQueue::Entry) * 4, sizeof(CanTxQueue::Entry)> pool;

    // Platform interface
    SystemClockMock clockmock;
    CanDriverMock driver(2, clockmock);

    // IO Manager
    CanIOManager iomgr(driver, pool, clockmock);
    ASSERT_EQ(2, iomgr.getNumIfaces());

    CanFrame fr1;
    fr1.id = 123 | CanFrame::FlagEFF;

    CanFrame fr2;
    fr2.id = 456 | CanFrame::FlagEFF;

    CanRxFrame rfr1;
    CanRxFrame rfr2;

    uavcan::CanIOFlags flags = 0;
    ASSERT_EQ(1, iomgr.send(fr1, tsMono(1000), tsMono(0), 1, CanTxQueue::Volatile, uavcan::CanIOFlagLoopback));
    ASSERT_LE(0, iomgr.receive(rfr1, tsMono(100), flags));
    ASSERT_EQ(uavcan::CanIOFlagLoopback, flags);
    ASSERT_TRUE(rfr1 == fr1);

    flags = 0;
    ASSERT_EQ(1, iomgr.send(fr1, tsMono(1000), tsMono(0), 1, CanTxQueue::Volatile, uavcan::CanIOFlagLoopback));
    ASSERT_EQ(1, iomgr.send(fr2, tsMono(1000), tsMono(0), 1, CanTxQueue::Persistent, uavcan::CanIOFlagLoopback));
    ASSERT_LE(0, iomgr.receive(rfr1, tsMono(100), flags));
    ASSERT_EQ(uavcan::CanIOFlagLoopback, flags);
    ASSERT_LE(0, iomgr.receive(rfr2, tsMono(100), flags));
    ASSERT_EQ(uavcan::CanIOFlagLoopback, flags);
    ASSERT_TRUE(rfr1 == fr1);
    ASSERT_TRUE(rfr2 == fr2);

    /*
     * Perf counters
     * Loopback frames are not registered as RX
     */
    EXPECT_EQ(0, iomgr.getIfacePerfCounters(0).frames_rx);
    EXPECT_EQ(0, iomgr.getIfacePerfCounters(1).frames_rx);

    EXPECT_EQ(3, iomgr.getIfacePerfCounters(0).frames_tx);
    EXPECT_EQ(0, iomgr.getIfacePerfCounters(1).frames_tx);
}

TEST(CanIOManager, Size)
{
    std::cout << sizeof(uavcan::CanIOManager) << std::endl;
}
