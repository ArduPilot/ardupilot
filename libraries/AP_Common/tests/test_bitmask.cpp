#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Common/Bitmask.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(Bitmask, Tests)
{
    Bitmask<49> x;
    EXPECT_EQ(0, x.count());
    EXPECT_EQ(-1, x.first_set());
    x.set(5);
    EXPECT_EQ(1, x.count());
    EXPECT_EQ(5, x.first_set());
    x.clear(5);
    EXPECT_EQ(0, x.count());
    EXPECT_EQ(-1, x.first_set());

    EXPECT_EQ(-1, x.first_set());
    x.set(42);
    EXPECT_EQ(42, x.first_set());
    x.clear(42);
    EXPECT_EQ(-1, x.first_set());

    EXPECT_EQ(-1, x.first_set());
    x.set(0);
    x.set(5);
    x.set(6);
    x.set(48);
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
    EXPECT_EQ(48, x.first_set());
    EXPECT_EQ(48, x.first_set());
    x.clear(48);
    EXPECT_EQ(-1, x.first_set());

    Bitmask<49> x2;
    x2 = x;

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    x.set(50);
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    EXPECT_EXIT(x.set(50), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bitmask_range");
#endif

    for (uint8_t i=0; i<49; i++) {
        EXPECT_EQ(x2.get(i), x.get(i));
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    EXPECT_EQ(x2.get(50), x.get(50));
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    EXPECT_EXIT(x2.get(50), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bitmask_range");
#endif
}

TEST(Bitmask, SetAll)
{
    Bitmask<49> x;
    EXPECT_EQ(-1, x.first_set());
    EXPECT_EQ(false, x.get(45));
    x.setall();
    EXPECT_EQ(0, x.first_set());
    x.clear(0);
    EXPECT_EQ(1, x.first_set());
    x.clear(1);
    EXPECT_EQ(2, x.first_set());
    EXPECT_EQ(true, x.get(45));
    EXPECT_EQ(false, x.empty());
    x.clearall();
    EXPECT_EQ(-1, x.first_set());
    EXPECT_EQ(false, x.get(45));
    EXPECT_EQ(true, x.empty());
}

TEST(Bitmask, Assignment)
{
    Bitmask<49> x;
    x.set(0);
    x.set(5);
    x.set(6);
    x.set(48);

    Bitmask<49> y;
    y = x;
    x.clear(0);
    EXPECT_EQ(true, y.get(0));
    EXPECT_EQ(true, y.get(5));
    EXPECT_EQ(true, y.get(6));
    EXPECT_EQ(true, y.get(48));
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
