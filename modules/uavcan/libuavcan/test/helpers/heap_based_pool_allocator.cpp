/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#ifdef __linux__
#include <malloc.h>

#else
#include <stdlib.h>
#endif

TEST(HeapBasedPoolAllocator, Basic)
{
#ifdef __linux__
    std::cout << ">>> HEAP BEFORE:" << std::endl;
    malloc_stats();
#endif

    uavcan::HeapBasedPoolAllocator<uavcan::MemPoolBlockSize> al(0xEEEE);

    ASSERT_EQ(0, al.getNumReservedBlocks());
    ASSERT_EQ(0, al.getNumAllocatedBlocks());

    ASSERT_EQ(0xEEEE, al.getBlockCapacity());
    ASSERT_EQ(0xFFFF, al.getBlockCapacityHardLimit());

    void* a = al.allocate(10);
    void* b = al.allocate(10);
    void* c = al.allocate(10);
    void* d = al.allocate(10);

    ASSERT_EQ(4, al.getNumReservedBlocks());
    ASSERT_EQ(4, al.getNumAllocatedBlocks());

    al.deallocate(a);
    ASSERT_EQ(4, al.getNumReservedBlocks());
    ASSERT_EQ(3, al.getNumAllocatedBlocks());

    al.deallocate(b);
    ASSERT_EQ(4, al.getNumReservedBlocks());
    ASSERT_EQ(2, al.getNumAllocatedBlocks());

    al.deallocate(c);
    ASSERT_EQ(4, al.getNumReservedBlocks());
    ASSERT_EQ(1, al.getNumAllocatedBlocks());

    a = al.allocate(10);
    ASSERT_EQ(4, al.getNumReservedBlocks());
    ASSERT_EQ(2, al.getNumAllocatedBlocks());
    ASSERT_EQ(c, a);

    al.deallocate(a);
    ASSERT_EQ(4, al.getNumReservedBlocks());
    ASSERT_EQ(1, al.getNumAllocatedBlocks());

    al.shrink();
    ASSERT_EQ(1, al.getNumReservedBlocks());
    ASSERT_EQ(1, al.getNumAllocatedBlocks());

    al.deallocate(d);
    ASSERT_EQ(1, al.getNumReservedBlocks());
    ASSERT_EQ(0, al.getNumAllocatedBlocks());

    al.shrink();
    ASSERT_EQ(0, al.getNumReservedBlocks());
    ASSERT_EQ(0, al.getNumAllocatedBlocks());

#ifdef __linux__
    std::cout << ">>> HEAP AFTER:" << std::endl;
    malloc_stats();
#endif
}


TEST(HeapBasedPoolAllocator, Limits)
{
    uavcan::HeapBasedPoolAllocator<uavcan::MemPoolBlockSize> al(2);

    ASSERT_EQ(2, al.getBlockCapacity());
    ASSERT_EQ(4, al.getBlockCapacityHardLimit());

    ASSERT_EQ(0, al.getNumReservedBlocks());
    ASSERT_EQ(0, al.getNumAllocatedBlocks());

    void* a = al.allocate(10);
    void* b = al.allocate(10);
    void* c = al.allocate(10);
    void* d = al.allocate(10);

    ASSERT_TRUE(a);
    ASSERT_TRUE(b);
    ASSERT_TRUE(c);
    ASSERT_TRUE(d);

    ASSERT_FALSE(al.allocate(10));

    ASSERT_EQ(4, al.getNumReservedBlocks());
    ASSERT_EQ(4, al.getNumAllocatedBlocks());

    al.deallocate(a);
    al.deallocate(b);
    al.deallocate(c);
    al.deallocate(d);

    ASSERT_EQ(4, al.getNumReservedBlocks());
    ASSERT_EQ(0, al.getNumAllocatedBlocks());
}

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

#include <thread>
#include <mutex>

struct RaiiSynchronizer
{
    static std::mutex mutex;
    std::lock_guard<std::mutex> guard{mutex};
};

std::mutex RaiiSynchronizer::mutex;

TEST(HeapBasedPoolAllocator, Concurrency)
{
#ifdef __linux__
    std::cout << ">>> HEAP BEFORE:" << std::endl;
    malloc_stats();

#endif
    uavcan::HeapBasedPoolAllocator<uavcan::MemPoolBlockSize, RaiiSynchronizer> al(1000);

    ASSERT_EQ(1000, al.getBlockCapacity());
    ASSERT_EQ(2000, al.getBlockCapacityHardLimit());

    volatile bool terminate = false;

    /*
     * Starting the testing threads
     */
    std::thread threads[3];

    for (auto& x : threads)
    {
        x = std::thread([&al, &terminate]()
        {
            while (!terminate)
            {
                auto a = al.allocate(1);
                auto b = al.allocate(1);
                auto c = al.allocate(1);
                al.deallocate(al.allocate(1));
                al.deallocate(a);
                al.deallocate(b);
                al.deallocate(c);
            }
        });
    }

    /*
     * Running the threads for some time, then terminating
     */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    terminate = true;
    std::cout << "Terminating workers..." << std::endl;

    for (auto& x : threads)
    {
        x.join();
    }
    std::cout << "All workers joined" << std::endl;

    /*
     * Now, there must not be any leaked memory, because the worker threads deallocate everything before completion.
     */
    std::cout << "Allocated: " << al.getNumAllocatedBlocks() << std::endl;
    std::cout << "Reserved:  " << al.getNumReservedBlocks() << std::endl;

#ifdef __linux__
    std::cout << ">>> HEAP BEFORE SHRINK:" << std::endl;
    malloc_stats();

#endif
    al.shrink();

#ifdef __linux__
    std::cout << ">>> HEAP AFTER SHRINK:" << std::endl;
    malloc_stats();
#endif
}

#endif
