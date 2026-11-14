// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <AP_HAL/Scheduler.h>

#if HAL_OWNED_THREADS_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#include <atomic>
#include <pthread.h>

class ExpandingString;

// Shared POSIX backend for the Linux and SITL schedulers.
class OwnedThreadPosix final : public AP_HAL::Scheduler::OwnedThread
{
public:
    static OwnedThread *create(AP_HAL::MemberProc proc, const char *name, uint32_t stack_size,
                               int policy, int priority);
    uint32_t stack_size() const override
    {
        return _stack_size;
    }
    uint32_t allocation_size() const override
    {
        return _mapping_size + sizeof(*this);
    }
#if defined(__linux__)
    bool cpu_load(float &percent) const override;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static void thread_info(ExpandingString &str);
#endif

private:
    explicit OwnedThreadPosix(AP_HAL::MemberProc proc) : _proc(proc) {}
    ~OwnedThreadPosix() override;
    bool join_if_finished() override;
    static void *trampoline(void *context);

    AP_HAL::MemberProc _proc;
    pthread_t _thread{};
    std::atomic<bool> _finished{false};
    void *_mapping = nullptr;
    uint32_t _mapping_size = 0;
    uint32_t _stack_size = 0;
#if defined(__linux__)
    uint64_t _created_ns = 0;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    OwnedThreadPosix *_info_next = nullptr;
    static OwnedThreadPosix *_info_threads;
    void *_stack = nullptr;
    char _name[32] {};
    int _priority = 0;
    static constexpr uint8_t stackfill = 0xEB;
#endif
};
#endif
