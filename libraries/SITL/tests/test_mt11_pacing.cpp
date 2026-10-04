#include <AP_gtest.h>
#include <SITL/SIM_MT11.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_SIM_MT11_ENABLED

class MT11PacingTest : public SITL::MT11 {
public:
    using MT11::next_frame_time_us;
};

TEST(MT11Pacing, SchedulingJitterDoesNotAccumulate)
{
    uint64_t deadline = 1000000;
    for (uint32_t frame = 1; frame <= 300; frame++) {
        // Alternate lateness to model a scheduler that does not run at 30 Hz.
        deadline = MT11PacingTest::next_frame_time_us(deadline, deadline + (frame % 7) * 500);
        EXPECT_EQ(deadline, 1000000ULL + frame * 33333ULL);
    }
}

TEST(MT11Pacing, ResynchronizesAfterBackpressure)
{
    const uint64_t now = 5000000;
    const uint64_t next = MT11PacingTest::next_frame_time_us(1000000, now);
    EXPECT_EQ(next, now + 33333);
    EXPECT_EQ(MT11PacingTest::next_frame_time_us(next, next + 1000), next + 33333);
}

#endif // AP_SIM_MT11_ENABLED

AP_GTEST_MAIN()
