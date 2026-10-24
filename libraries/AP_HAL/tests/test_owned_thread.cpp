// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if HAL_OWNED_THREADS_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#include <AP_HAL/utility/OwnedThread_posix.h>
#include <atomic>
#include <unistd.h>
#include <AP_Common/ExpandingString.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <thread>
#endif

class OwnedThreadTask
{
public:
    ~OwnedThreadTask()
    {
        release.store(true);
        if (thread != nullptr) {
            if (!reap()) {
                // Do not destroy callback context while a stuck worker uses it.
                abort();
            }
        }
    }

    bool start(uint32_t size = 65536, AP_HAL::Scheduler::priority_base base = AP_HAL::Scheduler::PRIORITY_SCRIPTING,
               int16_t priority = 0)
    {
        thread = hal.scheduler->thread_create_owned(FUNCTOR_BIND_MEMBER(&OwnedThreadTask::run, void),
                 "owned-test", size, base, priority);
        return thread != nullptr;
    }

    bool reap()
    {
        for (uint32_t i = 0; i < 10000; i++) {
            if (AP_HAL::Scheduler::thread_reap(thread)) {
                return true;
            }
            usleep(1000);
        }
        return false;
    }

    void run()
    {
        pthread_attr_t attr;
        void *stack;
        size_t size;
        if (pthread_getattr_np(pthread_self(), &attr) == 0) {
            if (pthread_attr_getstack(&attr, &stack, &size) == 0) {
                actual_stack_size = size;
                actual_stack_address = uintptr_t(stack);
            }
            pthread_attr_destroy(&attr);
        }
        entered.store(true);
        while (!release.load()) {
            usleep(1000);
        }
        result = 42;
    }

    AP_HAL::Scheduler::OwnedThread *thread = nullptr;
    std::atomic<bool> release{false};
    std::atomic<bool> entered{false};
    uint32_t actual_stack_size = 0;
    uintptr_t actual_stack_address = 0;
    uint32_t result = 0;
};

TEST(OwnedThread, RunningThreadsRetainResources)
{
    OwnedThreadTask first, second;
    ASSERT_TRUE(first.start());
    ASSERT_TRUE(second.start());
    auto *first_handle = first.thread;
    auto *second_handle = second.thread;
    EXPECT_FALSE(AP_HAL::Scheduler::thread_reap(first.thread));
    EXPECT_FALSE(AP_HAL::Scheduler::thread_reap(second.thread));
    EXPECT_EQ(first.thread, first_handle);
    EXPECT_EQ(second.thread, second_handle);
    const uint32_t stack_size = first.thread->stack_size();
    EXPECT_GE(stack_size, 65536U);
    EXPECT_GT(first.thread->allocation_size(), stack_size);
    first.release.store(true);
    ASSERT_TRUE(first.reap());
    EXPECT_EQ(first.thread, nullptr);
    EXPECT_EQ(first.result, 42U);
    EXPECT_EQ(first.actual_stack_size, stack_size);
    EXPECT_FALSE(AP_HAL::Scheduler::thread_reap(first.thread));
    EXPECT_FALSE(AP_HAL::Scheduler::thread_reap(second.thread));
    second.release.store(true);
    ASSERT_TRUE(second.reap());
    EXPECT_EQ(second.result, 42U);
}

TEST(OwnedThread, AbsolutePriorityBounds)
{
    const uint8_t minimum = hal.scheduler->thread_priority_min();
    const uint8_t maximum = hal.scheduler->thread_priority_max();
    ASSERT_GE(minimum, 1);
    ASSERT_GT(maximum, minimum);
    OwnedThreadTask task;
    EXPECT_FALSE(task.start(65536, AP_HAL::Scheduler::PRIORITY_ABSOLUTE, minimum - 1));
    EXPECT_FALSE(task.start(65536, AP_HAL::Scheduler::PRIORITY_ABSOLUTE, maximum + 1));
    EXPECT_FALSE(task.start(65536, AP_HAL::Scheduler::PRIORITY_ABSOLUTE, -1));
    ASSERT_TRUE(task.start(65536, AP_HAL::Scheduler::PRIORITY_ABSOLUTE, minimum));
    task.release.store(true);
    ASSERT_TRUE(task.reap());
}

TEST(OwnedThread, RepeatedImmediateExitAndMinimumStack)
{
    for (uint32_t i = 0; i < 100; i++) {
        OwnedThreadTask task;
        task.release.store(true);
        ASSERT_TRUE(task.start(1024));
        const uint32_t reserved = task.thread->stack_size();
        EXPECT_GE(reserved, uint32_t(PTHREAD_STACK_MIN));
        ASSERT_TRUE(task.reap());
        EXPECT_EQ(task.result, 42U);
        EXPECT_EQ(task.actual_stack_size, reserved);
    }
}

TEST(OwnedThread, RejectInvalidRequests)
{
    OwnedThreadTask task;
    EXPECT_FALSE(task.start(0));
    EXPECT_FALSE(task.start(UINT32_MAX));
    EXPECT_EQ(hal.scheduler->thread_create_owned(AP_HAL::MemberProc{}, "empty", 65536,
              AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0), nullptr);
    // Exercise cleanup after allocation but before successful pthread_create.
    const auto proc = FUNCTOR_BIND(&task, &OwnedThreadTask::run, void);
    for (uint32_t i = 0; i < 100; i++) {
        EXPECT_EQ(OwnedThreadPosix::create(proc, "invalid-policy", 65536, SCHED_OTHER, 99), nullptr);
    }
    ASSERT_TRUE(task.start());
    task.release.store(true);
    EXPECT_TRUE(task.reap());
}

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
TEST(OwnedThread, NonRealtimeCallerDoesNotRequireRealtimePrivileges)
{
    if (sched_getscheduler(0) != SCHED_OTHER) {
        GTEST_SKIP() << "Requires a non-realtime test process";
    }
    OwnedThreadTask task;
    task.release.store(true);
    ASSERT_TRUE(task.start());
    EXPECT_TRUE(task.reap());
    EXPECT_EQ(task.result, 42U);
}
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
TEST(OwnedThread, SnapshotTracksOwnedThreadLifetime)
{
    OwnedThreadTask task;
    ASSERT_TRUE(task.start(65537));
    for (unsigned i = 0; i < 1000 && !task.entered.load(); i++) {
        usleep(1000);
    }
    ASSERT_TRUE(task.entered.load());
    ExpandingString running;
    OwnedThreadPosix::thread_info(running);
    ASSERT_NE(running.get_string(), nullptr);
    const char *line = strstr(running.get_string(), "owned-test");
    ASSERT_NE(line, nullptr);
    uint32_t free, total;
    unsigned long long address;
    ASSERT_EQ(sscanf(line, "%*s PRI=%*d sp=%llx STACK=%u/%u", &address, &free, &total), 3);
    EXPECT_EQ(address, task.actual_stack_address);
    EXPECT_EQ(total, task.thread->stack_size());
    EXPECT_GT(free, 0U);
    EXPECT_LT(free, total);
    task.release.store(true);
    ASSERT_TRUE(task.reap());
    ExpandingString reaped;
    OwnedThreadPosix::thread_info(reaped);
    EXPECT_TRUE(reaped.get_string() == nullptr || strstr(reaped.get_string(), "owned-test") == nullptr);
}

TEST(OwnedThread, SnapshotsDuringCreationAndReaping)
{
    std::atomic<bool> stop{false};
    std::atomic<uint32_t> snapshots{0};
    std::thread reader([&]() {
        while (!stop.load()) {
            ExpandingString snapshot;
            OwnedThreadPosix::thread_info(snapshot);
            snapshots.fetch_add(1);
            usleep(100);
        }
    });
    for (unsigned i = 0; i < 1000 && snapshots.load() == 0; i++) {
        usleep(1000);
    }
    // Join the reader before any fatal test assertion can return.
    for (unsigned i = 0; i < 100; i++) {
        OwnedThreadTask task;
        if (!task.start()) {
            ADD_FAILURE() << "Could not create worker";
            break;
        }
        task.release.store(true);
        EXPECT_TRUE(task.reap());
    }
    stop.store(true);
    reader.join();
    EXPECT_GT(snapshots.load(), 0U);
}
#endif
#else
TEST(OwnedThread, RequiresEnabledPosixBuild)
{
    GTEST_SKIP() << "Requires HAL_OWNED_THREADS_ENABLED on Linux/SITL";
}
#endif

AP_GTEST_MAIN()
