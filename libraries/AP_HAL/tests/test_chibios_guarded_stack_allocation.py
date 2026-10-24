# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise the production guarded-stack allocator with a cold core provider.

Core-provider alignment gaps cannot be freed, unlike fixed-heap fragments.
The model accounts for those gaps separately from the owned heap block.
"""

import shutil
import subprocess

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]


def test_chibios_guarded_stack_allocation(tmp_path):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('Requires a host C++ compiler')
    scheduler = (ROOT / 'libraries/AP_HAL_ChibiOS/Scheduler.cpp').read_text()
    start = scheduler.index('    bool start_guarded(')
    method = scheduler[start:scheduler.index('\n#endif', start)]
    start = scheduler.index('    ~OwnedThreadChibiOS() override')
    destructor = scheduler[start:scheduler.index('\n#if HAL_STACK_GUARD_ENABLED\n    bool start_guarded(', start)]
    destructor = destructor.replace('~OwnedThreadChibiOS() override', '~Probe()')
    start = scheduler.index('    uint32_t allocation_size() const override', scheduler.index('class OwnedThreadChibiOS'))
    accounting = scheduler[start:scheduler.index('\nprivate:', start)].replace(' const override', ' const')
    source = r'''
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <unordered_map>
constexpr size_t HAL_STACK_GUARD_BYTES = 1024;
constexpr uint32_t MEM_REGION_FLAG_FAST = 1;
using stkalign_t = uint64_t;
using tprio_t = int;
using memory_heap_t = int;
struct memory_region {
    uint32_t flags;
};
struct AP_StackGuard {
    uintptr_t base, thread;
    void *context;
    void (*fault)(void *);
    bool overflow;
};
struct thread_t {
    AP_StackGuard *stack_guard;
};
struct thread_descriptor_t {
    const char *name;
    stkalign_t *base, *end;
    int priority;
    void (*entry)(void *);
    void *context;
};
#define THD_WORKING_AREA_SIZE(n) (size_t(n) + 256)
#define THD_DESCRIPTOR(...) {__VA_ARGS__}
#define MPU_CTRL_PRIVDEFENA 4
#define MSG_OK 0
namespace AP_HAL
{
struct Scheduler {
    struct StackFaultHandler {
        void *context;
        void (*fault)(void *);
    };
};
} // namespace AP_HAL
static memory_heap_t heaps[2];
static memory_region regions[2];
static unsigned count = 1;
static bool fail = false;
static bool fail_fixed = false;
static std::unordered_map<void *, size_t> allocations;
alignas(1024) static uint8_t core[131072];
static size_t consumed, recovered;
struct heap_header_t {
    uint8_t bytes[16];
};
size_t chHeapGetSize(void *p)
{
    return allocations.at(p) -
           (uintptr_t(p) >= uintptr_t(core) && uintptr_t(p) < uintptr_t(core + sizeof(core)) ? 16 : 0);
}
static thread_t tcb;
uint8_t malloc_get_heaps(memory_heap_t **h, const memory_region **r)
{
    *h = heaps;
    *r = regions;
    return count;
}
void *chHeapAllocAligned(memory_heap_t *heap, size_t size, unsigned alignment)
{
    if (fail || (heap && fail_fixed)) {
        return nullptr;
    }
    if (heap) {
        void *p = std::aligned_alloc(alignment, (size + alignment - 1) & ~size_t(alignment - 1));
        assert(p);
        allocations[p] = size;
        return p;
    }
    // The real default provider owns only the requested block and its header.
    const size_t header = 16;
    size_t aligned = (consumed + header + alignment - 1) & ~size_t(alignment - 1);
    assert(aligned + size < sizeof(core));
    void *p = core + aligned;
    consumed = aligned + size;
    allocations[p] = size + header;
    return p;
}
void *chHeapAlloc(memory_heap_t *heap, size_t size) { return chHeapAllocAligned(heap, size, 16); }
void chHeapFree(void *p)
{
    assert(allocations.count(p));
    if (uintptr_t(p) >= uintptr_t(core) && uintptr_t(p) < uintptr_t(core + sizeof(core))) {
        recovered += allocations.at(p);
    } else {
        std::free(p);
    }
    allocations.erase(p);
}
thread_t *chThdCreateSuspended(const thread_descriptor_t *td)
{
    assert(uintptr_t(td->base) % HAL_STACK_GUARD_BYTES == 0);
    assert(size_t(td->end - td->base) * sizeof(stkalign_t) == THD_WORKING_AREA_SIZE(4096));
    bool owned = false;
    for (const auto &a : allocations) {
        auto p = uintptr_t(a.first);
        if (uintptr_t(td->base) >= p + HAL_STACK_GUARD_BYTES &&
            uintptr_t(td->end) <= p + chHeapGetSize(a.first)) {
            owned = true;
        }
    }
    assert(owned);
    return &tcb;
}
void chSysLock() {}
void chSysUnlock() {}
void mpuEnable(unsigned) {}
void chSchWakeupS(thread_t *, int) {}
struct Probe {
    uint32_t _stack_size = 4096;
    void *_guard_allocation = nullptr;
    thread_t *_thread = nullptr;
    AP_StackGuard _guard{};
    static void trampoline(void *) {}
    DESTRUCTOR
    METHOD
    ACCOUNTING
};
static void fault(void *) {}
int main()
{
    for (unsigned mode = 0; mode < 4; mode++) {
        // Deliberately cold, misaligned default core and a fixed-heap option.
        consumed = 32;
        recovered = 0;
        count = mode == 0 ? 1 : 2;
        regions[0].flags = 0;
        regions[1].flags = MEM_REGION_FLAG_FAST;
        {
            Probe p;
            assert(!p.start_guarded("missing handler", 1, {nullptr, nullptr}));
            assert(allocations.empty());
            fail = mode == 3;
            fail_fixed = mode == 2;
            bool ok = p.start_guarded("probe", 1, {nullptr, fault});
            assert(ok != fail);
            if (ok) {
                assert(p._guard.base % HAL_STACK_GUARD_BYTES == 0);
                const size_t header = mode == 1 ? sizeof(heap_header_t) : 0;
                assert(p.allocation_size() == allocations.at(p._guard_allocation) + header + sizeof(p));
            }
        }
        assert(allocations.empty());
        if (mode == 0 || mode == 2) {
            assert(consumed - 32 == recovered);
        } else {
            assert(consumed == 32);
        }
        fail = false;
    }
}
'''
    source = source.replace('DESTRUCTOR', destructor).replace('METHOD', method).replace('ACCOUNTING', accounting)
    path = tmp_path / 'probe.cpp'
    path.write_text(source)
    executable = tmp_path / 'probe'
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-DHAL_STACK_GUARD_ENABLED=1',
                    '-fsanitize=address,undefined', '-g',
                    str(path), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
