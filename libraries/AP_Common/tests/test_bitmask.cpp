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
    x.clear(N+1);
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    EXPECT_EXIT(x.clear(N+1), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bitmask_range");
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    x.set(N+1);
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    EXPECT_EXIT(x.set(N+1), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bitmask_range");
#endif

    for (uint8_t i=0; i<N; i++) {
        EXPECT_EQ(x2.get(i), x.get(i));
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    EXPECT_EQ(false, x.get(N+1));
    EXPECT_EQ(false, x2.get(N+1));
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    EXPECT_EXIT(x2.get(N+1), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bitmask_range");
#endif
}

// bitmasks are composed of 32 bit words, so test those boundaries
TEST(Bitmask, Tests31) { bitmask_tests<31>(); }
TEST(Bitmask, Tests32) { bitmask_tests<32>(); }
TEST(Bitmask, Tests33) { bitmask_tests<33>(); }
TEST(Bitmask, Tests47) { bitmask_tests<47>(); }
TEST(Bitmask, Tests48) { bitmask_tests<48>(); }
TEST(Bitmask, Tests49) { bitmask_tests<49>(); }
TEST(Bitmask, Tests63) { bitmask_tests<63>(); }
TEST(Bitmask, Tests64) { bitmask_tests<64>(); }
TEST(Bitmask, Tests65) { bitmask_tests<65>(); }

template<int N>
void bitmask_setall(void)
{
    Bitmask<N> x;
    EXPECT_EQ(-1, x.first_set());
    EXPECT_EQ(false, x.get(N-4));
    EXPECT_EQ(0, x.count());
    x.setall();
    EXPECT_EQ(0, x.first_set());
    EXPECT_EQ(N, x.count());
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
    EXPECT_EQ(0, x.count());
}

TEST(Bitmask, SetAll31) { bitmask_setall<31>(); }
TEST(Bitmask, SetAll32) { bitmask_setall<32>(); }
TEST(Bitmask, SetAll33) { bitmask_setall<33>(); }
TEST(Bitmask, SetAll47) { bitmask_setall<47>(); }
TEST(Bitmask, SetAll48) { bitmask_setall<48>(); }
TEST(Bitmask, SetAll49) { bitmask_setall<49>(); }
TEST(Bitmask, SetAll63) { bitmask_setall<63>(); }
TEST(Bitmask, SetAll64) { bitmask_setall<64>(); }
TEST(Bitmask, SetAll65) { bitmask_setall<65>(); }

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
    EXPECT_EQ(true, x == y);
    x.clear(0);
    EXPECT_EQ(true, y.get(0));
    EXPECT_EQ(true, y.get(5));
    EXPECT_EQ(true, y.get(6));
    EXPECT_EQ(true, y.get(N-1));
}

TEST(Bitmask, Assignment31) { bitmask_assignment<31>(); }
TEST(Bitmask, Assignment32) { bitmask_assignment<32>(); }
TEST(Bitmask, Assignment33) { bitmask_assignment<33>(); }
TEST(Bitmask, Assignment47) { bitmask_assignment<47>(); }
TEST(Bitmask, Assignment48) { bitmask_assignment<48>(); }
TEST(Bitmask, Assignment49) { bitmask_assignment<49>(); }
TEST(Bitmask, Assignment63) { bitmask_assignment<63>(); }
TEST(Bitmask, Assignment64) { bitmask_assignment<64>(); }
TEST(Bitmask, Assignment65) { bitmask_assignment<65>(); }

AP_GTEST_PANIC()
AP_GTEST_MAIN()
