// SPDX-License-Identifier: GPL-3.0-or-later
// Exercise the real ChibiOS allocator under Renode, without a vehicle.
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/Heap.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

volatile uint32_t rust_heap_test_result;
volatile uint32_t rust_heap_test_cycles;
volatile uint32_t rust_heap_test_before;
volatile uint32_t rust_heap_test_after;

#if AP_MODULE_ENABLED
static uint32_t exercise()
{
    AP_Module::Heap heap;
    AP_Module::Heap fixed;
    hal.util->set_soft_armed(false);
    if (!heap.create(4096) || !fixed.create(4096, false)) {
        return 2;
    }
    if (heap.allocate(8192, 64) != nullptr || heap.stats().expanded_to != 0) {
        return 3;
    }
    // Reset the GC first-failure state before testing the single Rust call.
    if (heap.allocate(16, 8) == nullptr) {
        return 4;
    }
    hal.util->set_soft_armed(true);
    void *large = heap.allocate(8192, 64);
    if (large == nullptr || uintptr_t(large) % 64 != 0 || heap.stats().expanded_to <= 4096) {
        return 5;
    }
    if (fixed.allocate(8192, 64) != nullptr || fixed.stats().expanded_to != 0) {
        return 6;
    }
    if (heap.allocate(hal.util->available_memory(), 8) != nullptr) {
        return 7;
    }
    hal.util->set_soft_armed(false);
    if (heap.allocate(65536, 8) != nullptr || !heap.deallocate(large) || heap.allocate(8192, 64) == nullptr) {
        return 8;
    }
    // Destruction must also reclaim allocations not explicitly freed here.
    return 0;
}
#endif

void setup();
void loop();

void setup()
{
    rust_heap_test_result = 1;
#if AP_MODULE_ENABLED
    // Let background startup work finish before comparing free memory.
    hal.scheduler->delay(1000);
    // Warm the allocator before measuring: moving core memory to the heap
    // introduces a free-block header excluded from available_memory().
    const uint32_t warmup = exercise();
    hal.util->set_soft_armed(false);
    if (warmup != 0) {
        rust_heap_test_result = warmup;
        return;
    }
    rust_heap_test_before = hal.util->available_memory();
    for (unsigned i = 0; i < 100; i++) {
        const uint32_t result = exercise();
        hal.util->set_soft_armed(false);
        if (result != 0) {
            rust_heap_test_result = result;
            return;
        }
        rust_heap_test_cycles++;
    }
    rust_heap_test_after = hal.util->available_memory();
    rust_heap_test_result = 0x600DF00D;
#else
    rust_heap_test_result = 0xBAD0;
#endif
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
