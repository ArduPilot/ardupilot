#include <AP_gtest.h>
#include <AP_MultiHeap/AP_MultiHeap.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(MultiHeap, Tests)
{
    static MultiHeap h;

    EXPECT_TRUE(h.create(150000, 10, true, 10000));
    EXPECT_TRUE(h.available());

    const uint32_t max_allocs = 1000;
    struct alloc {
        void *ptr;
        uint32_t size;
    };
    auto *allocs = new alloc[max_allocs];
    allocs[0].size = 640;
    allocs[0].ptr = h.allocate(allocs[0].size);
    for (uint32_t i=1; i<2; i++) {
        auto &a = allocs[i];
        a.size = 30;
        a.ptr = h.allocate(a.size);
    }
    allocs[0].ptr = h.change_size(allocs[0].ptr, allocs[0].size, 50);
    allocs[0].size = 50;

    for (uint32_t i=0; i<5000; i++) {
        uint16_t idx = get_random16() % max_allocs;
        auto &a = allocs[idx];
        if (a.ptr == nullptr) {
            const uint16_t size = get_random16() % 150;
            a.ptr = h.allocate(size);
            //printf("a.ptr=%p %u -> %u\n", a.ptr, a.size, size);
            EXPECT_TRUE(size==0?a.ptr == nullptr : a.ptr != nullptr);
            a.size = size;
        } else {
            const uint16_t size = get_random16() % 150;
            a.ptr = h.change_size(a.ptr, a.size, size);
            //printf("a.ptr=%p %u -> %u\n", a.ptr, a.size, size);
            EXPECT_TRUE(size==0?a.ptr == nullptr : a.ptr != nullptr);
            a.size = size;
        }
    }
    for (uint32_t i=0; i<max_allocs; i++) {
        auto &a = allocs[i];
        h.deallocate(a.ptr);
        a.size = 0;
    }
    h.destroy();
    delete[] allocs;
}

TEST(MultiHeap, DestroyUnavailableHeap)
{
    MultiHeap heap{};
    // This has the same unavailable state as failing to allocate the first
    // backing heap: the descriptor array exists, but heaps[0].hp is null.
    ASSERT_TRUE(heap.create(0, 1, false, 0));
    EXPECT_FALSE(heap.available());
    heap.destroy();
    ASSERT_TRUE(heap.create(1024, 1, false, 0));
    EXPECT_TRUE(heap.available());
    heap.destroy();
}

TEST(MultiHeap, DestroyUnusedDescriptors)
{
    MultiHeap heap{};
    for (uint8_t i = 0; i < 100; i++) {
        // One backing heap satisfies this request. Remaining descriptors must
        // be initialized before destroy() inspects every possible heap.
        ASSERT_TRUE(heap.create(1024, 8, false, 0));
        void *allocation = heap.allocate(16);
        ASSERT_NE(allocation, nullptr);
        heap.deallocate(allocation);
        heap.destroy();
    }
}

TEST(MultiHeap, CapAndArenaExhaustionAreDistinct)
{
    MultiHeap heap{};
    const bool previous = hal.util->get_soft_armed();
    hal.util->set_soft_armed(true);
    EXPECT_TRUE(heap.create(4096, 1, true, 0, 65536));
    EXPECT_EQ(heap.allocate(8192), nullptr); // initial GC opportunity
    EXPECT_EQ(heap.allocate(8192), nullptr); // no descriptor for growth
    EXPECT_EQ(heap.get_expansion_failures(), MultiHeap::Arenas);
    EXPECT_EQ(heap.get_expansion_size(), 0U);
    heap.destroy();
    hal.util->set_soft_armed(previous);
}

AP_GTEST_MAIN()
