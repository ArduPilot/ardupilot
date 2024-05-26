/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/transport/can_io.hpp>
#include "can.hpp"


static int getQueueLength(uavcan::CanTxQueue& queue)
{
    const uavcan::CanTxQueue::Entry* p = queue.peek();
    int length = 0;
    while (p)
    {
        length++;
        p = p->getNextListNode();
    }
    return length;
}

static bool isInQueue(uavcan::CanTxQueue& queue, const uavcan::CanFrame& frame)
{
    const uavcan::CanTxQueue::Entry* p = queue.peek();
    while (p)
    {
        if (frame == p->frame)
        {
            return true;
        }
        p = p->getNextListNode();
    }
    return false;
}

TEST(CanTxQueue, Qos)
{
    const uavcan::CanIOFlags flags = 0;
    uavcan::CanTxQueue::Entry e1(makeCanFrame(100, "", EXT), tsMono(1000), uavcan::CanTxQueue::Volatile, flags);
    uavcan::CanTxQueue::Entry e2(makeCanFrame(100, "", EXT), tsMono(1000), uavcan::CanTxQueue::Volatile, flags);

    EXPECT_FALSE(e1.qosHigherThan(e2));
    EXPECT_FALSE(e2.qosHigherThan(e1));
    EXPECT_FALSE(e1.qosLowerThan(e2));
    EXPECT_FALSE(e2.qosLowerThan(e1));

    e2.qos = uavcan::CanTxQueue::Persistent;

    EXPECT_FALSE(e1.qosHigherThan(e2));
    EXPECT_TRUE(e2.qosHigherThan(e1));
    EXPECT_TRUE(e1.qosLowerThan(e2));
    EXPECT_FALSE(e2.qosLowerThan(e1));

    e1.qos = uavcan::CanTxQueue::Persistent;
    e1.frame.id -= 1;

    EXPECT_TRUE(e1.qosHigherThan(e2));
    EXPECT_FALSE(e2.qosHigherThan(e1));
    EXPECT_FALSE(e1.qosLowerThan(e2));
    EXPECT_TRUE(e2.qosLowerThan(e1));
}

TEST(CanTxQueue, TxQueue)
{
    using uavcan::CanTxQueue;
    using uavcan::CanFrame;

    ASSERT_GE(40, sizeof(CanTxQueue::Entry)); // should be true for any platforms, though not required

    uavcan::PoolAllocator<40 * 4, 40> pool;

    SystemClockMock clockmock;

    CanTxQueue queue(pool, clockmock, 99999);
    EXPECT_TRUE(queue.isEmpty());

    const uavcan::CanIOFlags flags = 0;

    // Descending priority
    const CanFrame f0 = makeCanFrame(0, "f0", EXT);
    const CanFrame f1 = makeCanFrame(10, "f1", EXT);
    const CanFrame f2 = makeCanFrame(20, "f2", EXT);
    const CanFrame f3 = makeCanFrame(100, "f3", EXT);
    const CanFrame f4 = makeCanFrame(10000, "f4", EXT);
    const CanFrame f5 = makeCanFrame(99999, "f5", EXT);
    const CanFrame f5a = makeCanFrame(99999, "f5a", EXT);
    const CanFrame f6 = makeCanFrame(999999, "f6", EXT);

    /*
     * Priority insertion
     */
    queue.push(f4, tsMono(100), CanTxQueue::Persistent, flags);
    EXPECT_FALSE(queue.isEmpty());
    EXPECT_EQ(1, pool.getNumUsedBlocks());
    EXPECT_EQ(f4, queue.peek()->frame);
    EXPECT_TRUE(queue.topPriorityHigherOrEqual(f5));
    EXPECT_TRUE(queue.topPriorityHigherOrEqual(f4)); // Equal
    EXPECT_FALSE(queue.topPriorityHigherOrEqual(f3));

    queue.push(f3, tsMono(200), CanTxQueue::Persistent, flags);
    EXPECT_EQ(f3, queue.peek()->frame);

    queue.push(f0, tsMono(300), CanTxQueue::Volatile, flags);
    EXPECT_EQ(f0, queue.peek()->frame);

    queue.push(f1, tsMono(400), CanTxQueue::Volatile, flags);
    EXPECT_EQ(f0, queue.peek()->frame);              // Still f0, since it is highest
    EXPECT_TRUE(queue.topPriorityHigherOrEqual(f0)); // Equal
    EXPECT_TRUE(queue.topPriorityHigherOrEqual(f1));

    // Out of free memory now

    EXPECT_EQ(0, queue.getRejectedFrameCount());
    EXPECT_EQ(4, getQueueLength(queue));
    EXPECT_TRUE(isInQueue(queue, f0));
    EXPECT_TRUE(isInQueue(queue, f1));
    EXPECT_TRUE(isInQueue(queue, f3));
    EXPECT_TRUE(isInQueue(queue, f4));

    const CanTxQueue::Entry* p = queue.peek();
    while (p)
    {
        std::cout << p->toString() << std::endl;
        p = p->getNextListNode();
    }

    /*
     * QoS
     */
    EXPECT_FALSE(isInQueue(queue, f2));
    queue.push(f2, tsMono(100), CanTxQueue::Volatile, flags);     // Non preempting, will be rejected
    EXPECT_FALSE(isInQueue(queue, f2));

    queue.push(f2, tsMono(500), CanTxQueue::Persistent, flags);   // Will override f1 (f3 and f4 are presistent)
    EXPECT_TRUE(isInQueue(queue, f2));
    EXPECT_FALSE(isInQueue(queue, f1));
    EXPECT_EQ(4, getQueueLength(queue));
    EXPECT_EQ(2, queue.getRejectedFrameCount());
    EXPECT_EQ(f0, queue.peek()->frame);            // Check the priority

    queue.push(f5, tsMono(600), CanTxQueue::Persistent, flags);   // Will override f0 (rest are presistent)
    EXPECT_TRUE(isInQueue(queue, f5));
    EXPECT_FALSE(isInQueue(queue, f0));
    EXPECT_EQ(f2, queue.peek()->frame);            // Check the priority

    // No volatile frames left now

    queue.push(f5a, tsMono(700), CanTxQueue::Persistent, flags);   // Will override f5 (same frame, same QoS)
    EXPECT_TRUE(isInQueue(queue, f5a));
    EXPECT_FALSE(isInQueue(queue, f5));

    queue.push(f6, tsMono(700), CanTxQueue::Persistent, flags);    // Will be rejected (lowest QoS)
    EXPECT_FALSE(isInQueue(queue, f6));

    EXPECT_FALSE(queue.topPriorityHigherOrEqual(f0));
    EXPECT_TRUE(queue.topPriorityHigherOrEqual(f2));   // Equal
    EXPECT_TRUE(queue.topPriorityHigherOrEqual(f5a));
    EXPECT_EQ(4, getQueueLength(queue));
    EXPECT_EQ(4, pool.getNumUsedBlocks());
    EXPECT_EQ(5, queue.getRejectedFrameCount());
    EXPECT_TRUE(isInQueue(queue, f2));
    EXPECT_TRUE(isInQueue(queue, f3));
    EXPECT_TRUE(isInQueue(queue, f4));
    EXPECT_TRUE(isInQueue(queue, f5a));
    EXPECT_EQ(f2, queue.peek()->frame);            // Check the priority

    /*
     * Expiration
     */
    clockmock.monotonic = 101;
    queue.push(f0, tsMono(800), CanTxQueue::Volatile, flags);     // Will replace f4 which is expired now
    EXPECT_TRUE(isInQueue(queue, f0));
    EXPECT_FALSE(isInQueue(queue, f4));
    EXPECT_EQ(6, queue.getRejectedFrameCount());

    clockmock.monotonic = 1001;
    queue.push(f5, tsMono(2000), CanTxQueue::Volatile, flags);    // Entire queue is expired
    EXPECT_TRUE(isInQueue(queue, f5));
    EXPECT_EQ(1, getQueueLength(queue));           // Just one entry left - f5
    EXPECT_EQ(1, pool.getNumUsedBlocks());         // Make sure there is no leaks
    EXPECT_EQ(10, queue.getRejectedFrameCount());

    queue.push(f0, tsMono(1000), CanTxQueue::Persistent, flags);  // This entry is already expired
    EXPECT_EQ(1, getQueueLength(queue));
    EXPECT_EQ(1, pool.getNumUsedBlocks());
    EXPECT_EQ(11, queue.getRejectedFrameCount());

    /*
     * Removing
     */
    queue.push(f4, tsMono(5000), CanTxQueue::Volatile, flags);
    EXPECT_EQ(2, getQueueLength(queue));
    EXPECT_TRUE(isInQueue(queue, f4));
    EXPECT_EQ(f4, queue.peek()->frame);

    CanTxQueue::Entry* entry = queue.peek();
    EXPECT_TRUE(entry);
    queue.remove(entry);
    EXPECT_FALSE(entry);

    EXPECT_FALSE(isInQueue(queue, f4));
    EXPECT_TRUE(isInQueue(queue, f5));

    entry = queue.peek();
    EXPECT_TRUE(entry);
    queue.remove(entry);
    EXPECT_FALSE(entry);

    EXPECT_FALSE(isInQueue(queue, f5));

    EXPECT_EQ(0, getQueueLength(queue));           // Final state checks
    EXPECT_EQ(0, pool.getNumUsedBlocks());
    EXPECT_EQ(11, queue.getRejectedFrameCount());
    EXPECT_FALSE(queue.peek());
    EXPECT_FALSE(queue.topPriorityHigherOrEqual(f0));
}
