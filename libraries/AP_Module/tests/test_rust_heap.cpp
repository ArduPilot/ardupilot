// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/Heap.h>
#include <AP_Scripting/AP_Scripting_config.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_MODULE_ENABLED
class ArmedState
{
public:
    explicit ArmedState(bool armed) : _previous(hal.util->get_soft_armed())
    {
        hal.util->set_soft_armed(armed);
    }
    ~ArmedState()
    {
        hal.util->set_soft_armed(_previous);
    }
private:
    bool _previous;
};

TEST(RustHeap, AlignmentBoundsAndCleanup)
{
    AP_Module::Heap heap;
    ASSERT_TRUE(heap.create(32768));
    EXPECT_FALSE(heap.create(32768));
    EXPECT_EQ(heap.allocate(0, 8), nullptr);
    EXPECT_EQ(heap.allocate(1, 0), nullptr);
    EXPECT_EQ(heap.allocate(1, 3), nullptr);
    EXPECT_EQ(heap.allocate(1, 8192), nullptr);
    EXPECT_EQ(heap.allocate(UINT32_MAX, 8), nullptr);
    for (uint32_t alignment = 1; alignment <= 4096; alignment *= 2) {
        void *pointer = heap.allocate(37, alignment);
        ASSERT_NE(pointer, nullptr);
        EXPECT_EQ(reinterpret_cast<uintptr_t>(pointer) % alignment, 0U);
        memset(pointer, 0xA5, 37);
    }
    EXPECT_EQ(heap.stats().allocations, 13U);
    EXPECT_GT(heap.stats().used, 13U * 37);
    heap.clear();
    EXPECT_EQ(heap.stats().used, 0U);
    EXPECT_EQ(heap.stats().allocations, 0U);
    EXPECT_GT(heap.stats().peak, 0U);
}

TEST(RustHeap, ExhaustionDoesNotExpand)
{
    ArmedState armed(true);
    AP_Module::Heap heap;
    ASSERT_TRUE(heap.create(4096, false));
    void *pointers[64] {};
    uint8_t count = 0;
    while (count < ARRAY_SIZE(pointers)) {
        void *pointer = heap.allocate(100, 16);
        if (pointer == nullptr) {
            break;
        }
        pointers[count++] = pointer;
    }
    EXPECT_GT(count, 0U);
    EXPECT_LT(count, ARRAY_SIZE(pointers));
    EXPECT_LE(heap.stats().used, 4096U);
    // MultiHeap's second failed allocation normally permits expansion.
    EXPECT_EQ(heap.allocate(100, 16), nullptr);
    EXPECT_EQ(heap.allocate(100, 16), nullptr);
    int foreign;
    EXPECT_FALSE(heap.deallocate(&foreign));
    for (uint8_t i = 0; i < count; i++) {
        ASSERT_TRUE(heap.deallocate(pointers[i]));
        EXPECT_FALSE(heap.deallocate(pointers[i]));
    }
    EXPECT_EQ(heap.stats().used, 0U);
    void *pointer = heap.allocate(100, 16);
    ASSERT_NE(pointer, nullptr);
    // Destruction must reclaim an outstanding allocation on the host backend.
}

TEST(RustHeap, DiagnosticCapacityAndFreeSpace)
{
    AP_Module::Heap heap;
    EXPECT_EQ(heap.stats().capacity, 0U);
    EXPECT_EQ(heap.stats(true).free, 0U);
    ASSERT_TRUE(heap.create(4096, false));
    const auto empty = heap.stats(true);
    EXPECT_EQ(empty.capacity, 4096U);
    EXPECT_GT(empty.free, 0U);
    EXPECT_LE(empty.free, empty.capacity);
    void *pointer = heap.allocate(512, 16);
    ASSERT_NE(pointer, nullptr);
    const auto used = heap.stats(true);
    EXPECT_EQ(used.capacity, empty.capacity);
    EXPECT_LT(used.free, empty.free);
    EXPECT_GE(used.capacity - used.free, used.used);
    ASSERT_TRUE(heap.deallocate(pointer));
    EXPECT_EQ(heap.stats(true).free, empty.free);
}

TEST(RustHeap, ExpandsOnFirstRustRequestOnlyWhenArmed)
{
#if AP_SCRIPTING_HEAP_EXPANSION_ALWAYS_ENABLED
    GTEST_SKIP() << "Build explicitly permits expansion while disarmed";
#endif
    ArmedState armed(false);
    AP_Module::Heap heap;
    ASSERT_TRUE(heap.create(4096));
    EXPECT_EQ(heap.allocate(8192, 64), nullptr);
    EXPECT_EQ(heap.stats().expanded_to, 0U);
    // A successful allocation resets MultiHeap's first-failure/GC state.
    auto *small = heap.allocate(16, 8);
    ASSERT_NE(small, nullptr);
    hal.util->set_soft_armed(true);
    auto *large = heap.allocate(8192, 64);
    ASSERT_NE(large, nullptr);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(large) % 64, 0U);
    EXPECT_GT(heap.stats().expanded_to, 4096U);
    EXPECT_EQ(heap.stats().allocations, 2U);
    // Expanded capacity stays usable after disarming, but cannot grow further.
    hal.util->set_soft_armed(false);
    EXPECT_EQ(heap.allocate(65536, 8), nullptr);
    ASSERT_TRUE(heap.deallocate(large));
    ASSERT_NE(heap.allocate(8192, 64), nullptr);
    heap.clear();
    EXPECT_EQ(heap.stats().used, 0U);
    EXPECT_GT(heap.stats().expanded_to, 4096U);
}

TEST(RustHeap, BoundedBackingCapacity)
{
    ArmedState armed(true);
    AP_Module::Heap heap;
    // Admission must reject the original request, not shrink it to the cap.
    EXPECT_FALSE(heap.create(4096, true, 4095));
    ASSERT_TRUE(heap.create(4096, true, 12288));
    auto *p = heap.allocate(5000, 16);
    ASSERT_NE(p, nullptr);
    // The remaining 8 KiB is smaller than the usual 30 KiB growth chunk.
    EXPECT_EQ(heap.stats().expanded_to, 12288U);
    EXPECT_EQ(heap.stats().capacity, 12288U);
    EXPECT_LT(heap.stats(true).free, heap.stats().capacity);
    EXPECT_EQ(heap.allocate(9000, 16), nullptr);
    EXPECT_NE(heap.stats().expansion_failures & MultiHeap::Capacity, 0);
    ASSERT_TRUE(heap.deallocate(p));
    // Freeing live bytes does not lower the backing-arena capacity charge.
    EXPECT_EQ(heap.allocate(9000, 16), nullptr);
    EXPECT_EQ(heap.stats().expanded_to, 12288U);
    ASSERT_NE(heap.allocate(5000, 16), nullptr);
}

TEST(RustHeap, ExpansionPreservesSystemReserve)
{
    ArmedState armed(true);
    AP_Module::Heap heap;
    ASSERT_TRUE(heap.create(4096));
    // Available memory itself is too large a request once the reserve and
    // backing allocator overhead are considered, even after the retry.
    EXPECT_EQ(heap.allocate(hal.util->available_memory(), 8), nullptr);
    EXPECT_EQ(heap.stats().expanded_to, 0U);
}
#else
TEST(RustHeap, RequiresEnabledBuild)
{
    GTEST_SKIP() << "Configure with --extra-hwdef libraries/AP_Module/tests/enabled.hwdef";
}
#endif // AP_MODULE_ENABLED

AP_GTEST_MAIN()
