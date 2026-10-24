// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/Semaphores.h>
#include <new>
#include <thread>

class InspectSemaphore : public HALSITL::Semaphore
{
public:
    uint8_t recursion() const
    {
        return take_count;
    }

    bool unowned() const
    {
        return owner == (pthread_t)-1;
    }
};

TEST(SITLSemaphore, InitializesRecursionWithoutZeroedStorage)
{
    alignas(InspectSemaphore) uint8_t storage[sizeof(InspectSemaphore)];
    memset(storage, 0xA5, sizeof(storage));
    auto *semaphore = new (storage) InspectSemaphore;
    EXPECT_EQ(semaphore->recursion(), 0U);
    EXPECT_TRUE(semaphore->unowned());
    ASSERT_TRUE(semaphore->take_nonblocking());
    EXPECT_EQ(semaphore->recursion(), 1U);
    EXPECT_FALSE(semaphore->unowned());
    ASSERT_TRUE(semaphore->take_nonblocking());
    EXPECT_EQ(semaphore->recursion(), 2U);
    EXPECT_TRUE(semaphore->give());
    EXPECT_EQ(semaphore->recursion(), 1U);
    semaphore->check_owner();
    EXPECT_TRUE(semaphore->give());
    EXPECT_EQ(semaphore->recursion(), 0U);
    EXPECT_TRUE(semaphore->unowned());
    semaphore->~InspectSemaphore();
}

TEST(SITLSemaphore, RecursiveLockExcludesOtherThreadsUntilFullyReleased)
{
    InspectSemaphore semaphore;
    ASSERT_TRUE(semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER));
    ASSERT_TRUE(semaphore.take_nonblocking());
    for (uint8_t remaining = 2; remaining > 0; remaining--) {
        bool acquired = false;
        std::thread contender([&]() {
            acquired = semaphore.take_nonblocking();
            if (acquired) {
                semaphore.give();
            }
        });
        contender.join();
        EXPECT_FALSE(acquired);
        EXPECT_EQ(semaphore.recursion(), remaining);
        semaphore.check_owner();
        EXPECT_TRUE(semaphore.give());
    }
    EXPECT_TRUE(semaphore.unowned());

    bool acquired = false;
    std::thread next_owner([&]() {
        acquired = semaphore.take_nonblocking();
        if (acquired) {
            semaphore.check_owner();
            semaphore.give();
        }
    });
    next_owner.join();
    EXPECT_TRUE(acquired);
    EXPECT_TRUE(semaphore.unowned());
    EXPECT_EQ(semaphore.recursion(), 0U);
}
#else
TEST(SITLSemaphore, RequiresSITL)
{
    GTEST_SKIP() << "Requires SITL";
}
#endif

AP_GTEST_MAIN()
