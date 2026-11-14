// SPDX-License-Identifier: GPL-3.0-or-later
// Destructive validation: run only in an emulator or on a bench without actuators.
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/ExpandingString.h>
#include <atomic>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

volatile uint32_t stack_guard_test_result;
volatile uint32_t stack_guard_test_cycles;
volatile uint32_t stack_guard_test_before;
volatile uint32_t stack_guard_test_after;
volatile uint32_t stack_guard_test_mode;
volatile uint32_t stack_guard_test_depth;
volatile uint32_t stack_guard_test_heartbeat;
volatile uint32_t stack_guard_test_start;
volatile uint32_t stack_guard_test_waiters;
volatile uint32_t stack_guard_test_faults;

#if HAL_OWNED_THREADS_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS.h>
#include <AP_HAL_ChibiOS/Semaphores.h>
#include <ch.h>
#if HAL_STACK_GUARD_ENABLED
class StackGuardExample
{
public:
    void start()
    {
        stack_guard_test_depth = 0;
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&StackGuardExample::manage, void),
                                          "guard-manager", 8192, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 1)) {
            stack_guard_test_result = 2;
        }
    }
private:
    static void fault(void *context)
    {
        static_cast<StackGuardExample *>(context)->_faults.fetch_add(1);
        stack_guard_test_faults++;
    }
    static uint32_t __attribute__((noinline, optimize("O0"))) recurse(uint32_t n)
    {
        volatile uint32_t words[32];
        words[n % 32] = n;
        stack_guard_test_depth = n;
        return recurse(n + 1) + words[n % 32];
    }
    void waiter()
    {
        _waiter_started.store(true, std::memory_order_release);
        _mutex.take_blocking();
        _waiter_acquired.fetch_add(1, std::memory_order_relaxed);
        stack_guard_test_waiters++;
        _mutex.give();
    }
    static void __attribute__((noreturn, noinline)) host_call_fault(uintptr_t sp, uintptr_t target, uint32_t mode)
    {
        // Model a stack fault after control has crossed into a host callback.
        __asm volatile("mov sp, %0\nstr %2, [%1]\nb ." : : "r"(sp), "r"(target), "r"(mode) : "memory");
        __builtin_unreachable();
    }
    void worker()
    {
        if (stack_guard_test_mode == 2) {
            _mutex.take_blocking();
            _mutex_held.store(true, std::memory_order_release);
            while (!_waiter_started.load(std::memory_order_acquire)) {
                hal.scheduler->delay(1);
            }
        }
        stack_guard_test_result = 4;
        const uint32_t mode = stack_guard_test_mode;
        if (mode != 0) {
            const auto *guard = chThdGetSelfX()->stack_guard;
            const uintptr_t sp = guard->base + HAL_STACK_GUARD_BYTES + 256U;
            uintptr_t target = guard->base;
            if (mode == 4) {
                __asm volatile("vmov s0, %0" : : "r"(mode) : "s0");
            }
            if (mode == 5) {
                // Protect a different 1 KiB stack region. The resulting
                // MemManage fault is real but is not attributable to the
                // registered guard, so recovery must be refused.
                target += 2U * HAL_STACK_GUARD_BYTES;
                mpuConfigureRegion(HAL_STACK_GUARD_MPU_REGION, target,
                                   MPU_RASR_ATTR_AP_NA_NA | MPU_RASR_ATTR_XN |
                                   MPU_RASR_SIZE_1K | MPU_RASR_ENABLE);
            }
            if (mode == 6) {
                chSysLock();
            }
            // Controlled access with enough space for a complete exception
            // frame, independent of compiler frame layout. Never resume this
            // C++ frame after replacing its SP; only the exit trampoline runs.
            if (mode == 1) {
                host_call_fault(sp, target, mode);
            }
            __asm volatile("mov sp, %0\nstr %1, [%2]\nb ." : : "r"(sp), "r"(mode), "r"(target) : "memory");
            __builtin_unreachable();
        }
        recurse(0);
        stack_guard_test_result = 5; // must not return
    }
    bool exercise()
    {
        const uint32_t before = _faults.load();
        const uint32_t waiters_before = _waiter_acquired.load();
        _mutex_held.store(false, std::memory_order_relaxed);
        _waiter_started.store(false, std::memory_order_relaxed);
        auto *thread = hal.scheduler->thread_create_owned_guarded(
                           FUNCTOR_BIND_MEMBER(&StackGuardExample::worker, void), "guard-worker", 4096,
                           AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0, {this, fault});
        if (thread == nullptr) {
            stack_guard_test_result = 3;
            return false;
        }
        AP_HAL::Scheduler::OwnedThread *waiter_thread = nullptr;
        if (stack_guard_test_mode == 2) {
            while (!_mutex_held.load(std::memory_order_acquire)) {
                hal.scheduler->delay(1);
            }
            // Higher priority guarantees it queues on the held mutex before
            // the worker resumes and faults. UnlockAll must hand ownership to
            // this waiter and may reschedule from the recovery trampoline.
            waiter_thread = hal.scheduler->thread_create_owned(
                                FUNCTOR_BIND_MEMBER(&StackGuardExample::waiter, void), "guard-waiter", 2048,
                                AP_HAL::Scheduler::PRIORITY_SCRIPTING, 2);
            if (waiter_thread == nullptr) {
                _waiter_started.store(true, std::memory_order_release);
            }
        }
        const bool waiter_created = waiter_thread != nullptr;
        while (!AP_HAL::Scheduler::thread_reap(thread)) {
            hal.scheduler->delay(1);
        }
        if (waiter_thread != nullptr) {
            while (!AP_HAL::Scheduler::thread_reap(waiter_thread)) {
                hal.scheduler->delay(1);
            }
        }
        return _faults.load() == before + 1 &&
               (stack_guard_test_mode != 2 ||
                (waiter_created && _waiter_acquired.load() == waiters_before + 1));
    }
    void manage()
    {
        while (!stack_guard_test_start) {
            hal.scheduler->delay(10);
        }
        if (!exercise()) {
            return;
        }
        stack_guard_test_before = hal.util->available_memory();
        for (unsigned i = 0; i < 100; i++) {
            if (!exercise()) {
                return;
            }
            stack_guard_test_cycles = i + 1;
        }
        stack_guard_test_after = hal.util->available_memory();
        ExpandingString threads;
        hal.util->thread_info(threads);
        stack_guard_test_result = stack_guard_test_before == stack_guard_test_after ? 0x600DF00D : 6;
        while (true) {
            hal.scheduler->delay(1000);
        }
    }
    ChibiOS::Semaphore _mutex;
    std::atomic<uint32_t> _faults{0};
    std::atomic<uint32_t> _waiter_acquired{0};
    std::atomic<bool> _mutex_held{false};
    std::atomic<bool> _waiter_started{false};
};
static StackGuardExample example;
#endif
#endif

void setup();
void loop();
void setup()
{
    stack_guard_test_result = 1;
#if HAL_OWNED_THREADS_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && HAL_STACK_GUARD_ENABLED
    example.start();
#else
    stack_guard_test_result = 0xBAD0;
#endif
}
void loop()
{
    stack_guard_test_heartbeat++;
    hal.scheduler->delay(10);
}
AP_HAL_MAIN();
