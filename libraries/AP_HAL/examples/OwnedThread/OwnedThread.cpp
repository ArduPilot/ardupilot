// SPDX-License-Identifier: GPL-3.0-or-later
// Exercise the real HAL thread lifecycle, including ChibiOS under Renode.
#include <AP_HAL/AP_HAL.h>
#include <atomic>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// Symbols inspected by the emulator; failures retain a stage-specific value.
volatile uint32_t owned_thread_test_result;
volatile uint32_t owned_thread_test_cycles;
volatile uint32_t owned_thread_test_memory_before;
volatile uint32_t owned_thread_test_memory_after;

#if HAL_OWNED_THREADS_ENABLED
class OwnedThreadExample
{
public:
    void start()
    {
        owned_thread_test_result = 1;
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&OwnedThreadExample::manage, void),
                                          "owned-manager", 4096, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0)) {
            owned_thread_test_result = 2;
        }
    }

private:
    void worker()
    {
        while (!_release.load()) {
            hal.scheduler->delay(1);
        }
        _answer.store(42);
    }

    bool exercise(bool immediate)
    {
        _answer.store(0);
        _release.store(immediate);
        auto *thread = hal.scheduler->thread_create_owned(FUNCTOR_BIND_MEMBER(&OwnedThreadExample::worker, void),
                       "owned-worker", 4096, AP_HAL::Scheduler::PRIORITY_SCRIPTING, immediate ? 1 : 0);
        if (thread == nullptr) {
            owned_thread_test_result = 3;
            return false;
        }
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        // A higher-priority ready thread preempts this single-core creator.
        if (immediate && _answer.load() != 42) {
            owned_thread_test_result = 9;
            return false;
        }
#endif
        if (!immediate && AP_HAL::Scheduler::thread_reap(thread)) {
            owned_thread_test_result = 4;
            return false;
        }
        if (thread->stack_size() < 4096 || thread->allocation_size() <= thread->stack_size()) {
            owned_thread_test_result = 5;
            return false;
        }
        _release.store(true);
        const uint32_t start_ms = AP_HAL::millis();
        while (!AP_HAL::Scheduler::thread_reap(thread)) {
            if (AP_HAL::millis() - start_ms > 1000) {
                owned_thread_test_result = 6;
                return false;
            }
            hal.scheduler->delay(1);
        }
        if (thread != nullptr || _answer.load() != 42 || AP_HAL::Scheduler::thread_reap(thread)) {
            owned_thread_test_result = 7;
            return false;
        }
        return true;
    }

    void manage()
    {
        // Let startup allocations settle, then warm up before measuring.
        hal.scheduler->delay(200);
        bool passed = exercise(false);
        owned_thread_test_memory_before = hal.util->available_memory();
        for (uint32_t i = 0; i < 100; i++) {
            if (!passed || !exercise((i & 1) != 0)) {
                passed = false;
                break;
            }
            owned_thread_test_cycles = i + 1;
        }
        owned_thread_test_memory_after = hal.util->available_memory();
        // Permit small unrelated HAL allocations, not leaked working areas.
        if (passed && owned_thread_test_memory_before > owned_thread_test_memory_after &&
            owned_thread_test_memory_before - owned_thread_test_memory_after > 1024) {
            owned_thread_test_result = 8;
            passed = false;
        }
        if (passed) {
            owned_thread_test_result = 0x600DF00D;
        }
        // The fire-and-forget manager retains its own stack for this example.
        while (true) {
            hal.scheduler->delay(1000);
        }
    }

    std::atomic<bool> _release{false};
    std::atomic<uint32_t> _answer{0};
};
static OwnedThreadExample example;
#endif

void setup();
void loop();

void setup()
{
#if HAL_OWNED_THREADS_ENABLED
    example.start();
#else
    owned_thread_test_result = 0xBAD0;
#endif
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
