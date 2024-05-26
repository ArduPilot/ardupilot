/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/dynamic_memory.hpp>

TEST(DynamicMemory, Basic)
{
    uavcan::PoolAllocator<128, 32> pool32;
    EXPECT_EQ(4, pool32.getNumFreeBlocks());
    EXPECT_EQ(0, pool32.getPeakNumUsedBlocks());
    const void* ptr1 = pool32.allocate(16);
    ASSERT_TRUE(ptr1);
    EXPECT_EQ(1, pool32.getNumUsedBlocks());
    EXPECT_FALSE(pool32.allocate(120));
    EXPECT_EQ(1, pool32.getNumUsedBlocks());
    pool32.deallocate(ptr1);
    EXPECT_EQ(0, pool32.getNumUsedBlocks());
    EXPECT_EQ(1, pool32.getPeakNumUsedBlocks());
}

TEST(DynamicMemory, OutOfMemory)
{
    uavcan::PoolAllocator<64, 32> pool32;

    EXPECT_EQ(2, pool32.getNumFreeBlocks());
    EXPECT_EQ(0, pool32.getNumUsedBlocks());
    EXPECT_EQ(0, pool32.getPeakNumUsedBlocks());

    const void* ptr1 = pool32.allocate(32);
    ASSERT_TRUE(ptr1);
    EXPECT_EQ(1, pool32.getNumUsedBlocks());
    EXPECT_EQ(1, pool32.getPeakNumUsedBlocks());

    const void* ptr2 = pool32.allocate(32);
    ASSERT_TRUE(ptr2);
    EXPECT_EQ(2, pool32.getNumUsedBlocks());
    EXPECT_EQ(2, pool32.getPeakNumUsedBlocks());

    ASSERT_FALSE(pool32.allocate(32));        // No free blocks left --> UAVCAN_NULLPTR
    EXPECT_EQ(2, pool32.getNumUsedBlocks());
    EXPECT_EQ(0, pool32.getNumFreeBlocks());
    EXPECT_EQ(2, pool32.getPeakNumUsedBlocks());
}

TEST(DynamicMemory, LimitedPoolAllocator)
{
    uavcan::PoolAllocator<128, 32> pool32;
    uavcan::LimitedPoolAllocator lim(pool32, 2);

    EXPECT_EQ(2, lim.getBlockCapacity());
    EXPECT_EQ(0, pool32.getPeakNumUsedBlocks());

    const void* ptr1 = lim.allocate(1);
    const void* ptr2 = lim.allocate(1);
    const void* ptr3 = lim.allocate(1);

    EXPECT_TRUE(ptr1);
    EXPECT_TRUE(ptr2);
    EXPECT_FALSE(ptr3);

    lim.deallocate(ptr2);
    const void* ptr4 = lim.allocate(1);
    lim.deallocate(ptr1);
    const void* ptr5 = lim.allocate(1);
    const void* ptr6 = lim.allocate(1);

    EXPECT_TRUE(ptr4);
    EXPECT_TRUE(ptr5);
    EXPECT_FALSE(ptr6);

    EXPECT_EQ(2, pool32.getPeakNumUsedBlocks());
}
