#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Common/Bitmask.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

template<int N>
void bitmask_tests(void)
{
    Bitmask<N> x;
    EXPECT_EQ(0, x.count());
    EXPECT_EQ(-1, x.first_set());
    x.set(5);
    EXPECT_EQ(1, x.count());
    EXPECT_EQ(5, x.first_set());
    x.clear(5);
    EXPECT_EQ(0, x.count());
    EXPECT_EQ(-1, x.first_set());

    EXPECT_EQ(-1, x.first_set());
    x.set(N-7);
    EXPECT_EQ(N-7, x.first_set());
    x.clear(N-7);
    EXPECT_EQ(-1, x.first_set());

    EXPECT_EQ(-1, x.first_set());
    x.set(0);
    x.set(5);
    x.set(6);
    x.set(N-1);
    EXPECT_EQ(4, x.count());
    EXPECT_EQ(0, x.first_set());
    EXPECT_EQ(0, x.first_set());
    x.clear(0);
    EXPECT_EQ(5, x.first_set());
    EXPECT_EQ(5, x.first_set());
    x.clear(5);
    EXPECT_EQ(6, x.first_set());
    EXPECT_EQ(6, x.first_set());
    x.clear(6);
    EXPECT_EQ(N-1, x.first_set());
    EXPECT_EQ(N-1, x.first_set());
    x.clear(N-1);
    EXPECT_EQ(-1, x.first_set());

    Bitmask<N> x2;
    x2 = x;

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    x.set(N+1);
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    EXPECT_EXIT(x.set(N+1), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bitmask_range");
#endif

    for (uint8_t i=0; i<N; i++) {
        EXPECT_EQ(x2.get(i), x.get(i));
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    EXPECT_EQ(x2.get(N+1), x.get(N+1));
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    EXPECT_EXIT(x2.get(N+1), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bitmask_range");
#endif
}

TEST(Bitmask, Tests)
{
    bitmask_tests<49>();
}

template<int N>
void bitmask_setall(void)
{
    Bitmask<N> x;
    EXPECT_EQ(-1, x.first_set());
    EXPECT_EQ(false, x.get(N-4));
    x.setall();
    EXPECT_EQ(0, x.first_set());
    x.clear(0);
    EXPECT_EQ(1, x.first_set());
    x.clear(1);
    EXPECT_EQ(2, x.first_set());
    EXPECT_EQ(true, x.get(N-4));
    EXPECT_EQ(false, x.empty());
    x.clearall();
    EXPECT_EQ(-1, x.first_set());
    EXPECT_EQ(false, x.get(N-4));
    EXPECT_EQ(true, x.empty());
}

TEST(Bitmask, SetAll)
{
    bitmask_setall<49>();
}

template<int N>
void bitmask_assignment(void)
{
    Bitmask<N> x;
    x.set(0);
    x.set(5);
    x.set(6);
    x.set(N-1);

    Bitmask<N> y;
    y = x;
    x.clear(0);
    EXPECT_EQ(true, y.get(0));
    EXPECT_EQ(true, y.get(5));
    EXPECT_EQ(true, y.get(6));
    EXPECT_EQ(true, y.get(N-1));
}

TEST(Bitmask, Assignment)
{
    bitmask_assignment<49>();
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
