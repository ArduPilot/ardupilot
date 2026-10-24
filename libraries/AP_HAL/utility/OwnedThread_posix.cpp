// SPDX-License-Identifier: GPL-3.0-or-later
#include "OwnedThread_posix.h"

#if HAL_OWNED_THREADS_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#include <limits.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_Common/ExpandingString.h>
#include <AP_HAL/Semaphores.h>
static HAL_Semaphore thread_info_sem;
OwnedThreadPosix *OwnedThreadPosix::_info_threads;
#endif

OwnedThreadPosix::~OwnedThreadPosix()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // Exclude snapshots before unmapping a joined worker's stack. Failed
    // creations were never registered and simply fall through this search.
    WITH_SEMAPHORE(thread_info_sem);
    for (OwnedThreadPosix **p = &_info_threads; *p != nullptr; p = &(*p)->_info_next) {
        if (*p == this) {
            *p = _info_next;
            break;
        }
    }
#endif
    if (_mapping != nullptr) {
        munmap(_mapping, _mapping_size);
    }
}

AP_HAL::Scheduler::OwnedThread *OwnedThreadPosix::create(AP_HAL::MemberProc proc, const char *name,
        uint32_t stack_size, int policy, int priority)
{
    const long page = sysconf(_SC_PAGESIZE);
    // Bound arithmetic and reject nonsensical requests before allocating.
    if (!proc || stack_size == 0 || stack_size > 16U * 1024 * 1024 || page <= 0 || page > 65536) {
        return nullptr;
    }
    if (stack_size < size_t(PTHREAD_STACK_MIN)) {
        stack_size = PTHREAD_STACK_MIN;
    }
    stack_size = ((stack_size + page - 1) / page) * page;
    auto *thread = NEW_NOTHROW OwnedThreadPosix(proc);
    if (thread == nullptr) {
        return nullptr;
    }
    thread->_stack_size = stack_size;
    thread->_mapping_size = stack_size + 2 * page;
    thread->_mapping = mmap(nullptr, thread->_mapping_size, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (thread->_mapping == MAP_FAILED) {
        thread->_mapping = nullptr;
        delete thread;
        return nullptr;
    }
    void *stack = static_cast<uint8_t *>(thread->_mapping) + page;
    if (mprotect(stack, stack_size, PROT_READ | PROT_WRITE) != 0) {
        delete thread;
        return nullptr;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    thread->_stack = stack;
    // SITL diagnostics need a high-water mark; painting commits these pages.
    memset(stack, stackfill, stack_size);
    strncpy(thread->_name, name == nullptr ? "unnamed" : name, sizeof(thread->_name) - 1);
    thread->_priority = priority;
#endif
    pthread_attr_t attr;
    if (pthread_attr_init(&attr) != 0) {
        delete thread;
        return nullptr;
    }
    const sched_param param{priority};
    // Explicit scheduling prevents inheriting the creator's realtime priority.
    const bool attributes_ok = pthread_attr_setstack(&attr, stack, stack_size) == 0 &&
                               pthread_attr_setguardsize(&attr, 0) == 0 &&
                               pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) == 0 &&
                               pthread_attr_setschedpolicy(&attr, policy) == 0 &&
                               pthread_attr_setschedparam(&attr, &param) == 0;
#if defined(__linux__)
    timespec created;
    if (clock_gettime(CLOCK_MONOTONIC, &created) == 0) {
        thread->_created_ns = uint64_t(created.tv_sec) * 1000000000ULL + created.tv_nsec;
    }
#endif
    const int result = attributes_ok ? pthread_create(&thread->_thread, &attr, trampoline, thread) : -1;
    pthread_attr_destroy(&attr);
    if (result != 0) {
        delete thread;
        return nullptr;
    }
#if !defined(__APPLE__) && !defined(__OpenBSD__) && !defined(__EMSCRIPTEN__)
    if (name != nullptr) {
        pthread_setname_np(thread->_thread, name);
    }
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    {
        WITH_SEMAPHORE(thread_info_sem);
        thread->_info_next = _info_threads;
        _info_threads = thread;
    }
#endif
    return thread;
}

#if defined(__linux__)
bool OwnedThreadPosix::cpu_load(float &percent) const
{
    clockid_t clock;
    timespec cpu, now;
    if (_created_ns == 0 || pthread_getcpuclockid(_thread, &clock) != 0 ||
        clock_gettime(clock, &cpu) != 0 || clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return false;
    }
    const uint64_t elapsed = uint64_t(now.tv_sec) * 1000000000ULL + now.tv_nsec - _created_ns;
    if (elapsed == 0) {
        return false;
    }
    // Host wall time, not the accelerated SITL clock. Lifetime average avoids
    // one reader consuming another reader's sampling window.
    percent = 100.0f * float(uint64_t(cpu.tv_sec) * 1000000000ULL + cpu.tv_nsec) / float(elapsed);
    return true;
}
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void OwnedThreadPosix::thread_info(ExpandingString &str)
{
    WITH_SEMAPHORE(thread_info_sem);
    for (auto *p = _info_threads; p != nullptr; p = p->_info_next) {
        const volatile uint8_t *stack = static_cast<const uint8_t *>(p->_stack);
        uint32_t free = 0;
        while (free < p->_stack_size && stack[free] == stackfill) {
            free++;
        }
        str.printf("%-13.13s PRI=%3d sp=0x%llx STACK=%u/%u%s\n", p->_name, p->_priority,
                   (unsigned long long)uintptr_t(p->_stack), unsigned(free), unsigned(p->_stack_size),
                   p->_finished.load(std::memory_order_acquire) ? " EXITED" : "");
    }
}
#endif

void *OwnedThreadPosix::trampoline(void *context)
{
    auto *thread = static_cast<OwnedThreadPosix *>(context);
    thread->_proc();
    thread->_finished.store(true, std::memory_order_release);
    return nullptr;
}

bool OwnedThreadPosix::join_if_finished()
{
    if (!_finished.load(std::memory_order_acquire) || pthread_equal(pthread_self(), _thread)) {
        return false;
    }
    return pthread_join(_thread, nullptr) == 0;
}
#endif
