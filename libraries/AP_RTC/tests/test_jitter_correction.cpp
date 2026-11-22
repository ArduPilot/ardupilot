/*
  Unit tests for JitterCorrection.
 */

#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/JitterCorrection.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// a 5Hz GPS whose messages arrive 120ms after the PPS edge they are timed to
static constexpr uint64_t PERIOD_US = 200000;
static constexpr uint64_t LAG_US = 120000;
static constexpr uint64_t START_US = 10000000;

// JitterCorrection relies on zeroed memory, as firmware objects are, so each
// test allocates it with NEW_NOTHROW
struct Gps {
    JitterCorrection jitter;
    uint64_t t_us = START_US;   // local time of the next fix's PPS edge

    // feed one fix, timed to its PPS edge or to its message arrival
    uint64_t feed(bool pps) {
        const uint64_t local_us = pps ? t_us : t_us + LAG_US;
        const uint64_t estimate_us = jitter.correct_offboard_timestamp_usec(t_us, local_us, pps);
        t_us += PERIOD_US;
        return estimate_us;
    }
};

class JitterCorrectionTest : public testing::Test {
protected:
    void SetUp() override {
        gps_ptr = NEW_NOTHROW Gps;
        ASSERT_NE(nullptr, gps_ptr);
    }
    void TearDown() override { delete gps_ptr; }
    Gps *gps_ptr;
};

TEST_F(JitterCorrectionTest, exact_from_pps)
{
    Gps &gps = *gps_ptr;
    for (uint8_t i=0; i<10; i++) {
        const uint64_t t_us = gps.t_us;
        EXPECT_EQ(t_us, gps.feed(true));
        EXPECT_TRUE(gps.jitter.last_estimate_exact());
    }
}

TEST_F(JitterCorrectionTest, arrival_timed_not_exact)
{
    Gps &gps = *gps_ptr;
    for (uint8_t i=0; i<10; i++) {
        gps.feed(false);
        EXPECT_FALSE(gps.jitter.last_estimate_exact());
    }
}

// after PPS stops, samples stay referenced to the PPS edge until the
// convergence window re-takes its minimum from arrival times
TEST_F(JitterCorrectionTest, pps_stops)
{
    Gps &gps = *gps_ptr;
    for (uint8_t i=0; i<150; i++) {
        gps.feed(true);
    }
    // 50 PPS samples into the second window of 100
    for (uint8_t i=0; i<50; i++) {
        const uint64_t t_us = gps.t_us;
        EXPECT_EQ(t_us, gps.feed(false));
        EXPECT_TRUE(gps.jitter.last_estimate_exact());
    }
    // the window that ends with arrival timed samples only switches the offset
    for (uint8_t i=0; i<100; i++) {
        gps.feed(false);
    }
    const uint64_t t_us = gps.t_us;
    EXPECT_EQ(t_us + LAG_US, gps.feed(false));
    EXPECT_FALSE(gps.jitter.last_estimate_exact());
}

// a sample timed to its arrival still uses an offset set by a PPS sample in the same window
TEST_F(JitterCorrectionTest, pps_resumes_after_gap)
{
    Gps &gps = *gps_ptr;
    gps.feed(true);
    gps.feed(false);
    EXPECT_TRUE(gps.jitter.last_estimate_exact());
    gps.feed(true);
    EXPECT_TRUE(gps.jitter.last_estimate_exact());
}

// arrival timed samples are later than PPS timed ones, so PPS takes over at once
TEST_F(JitterCorrectionTest, pps_starts)
{
    Gps &gps = *gps_ptr;
    for (uint8_t i=0; i<10; i++) {
        gps.feed(false);
    }
    const uint64_t t_us = gps.t_us;
    EXPECT_EQ(t_us, gps.feed(true));
    EXPECT_TRUE(gps.jitter.last_estimate_exact());
}

// an offset clamped to the maximum lag comes from no sample
TEST_F(JitterCorrectionTest, clamped_not_exact)
{
    JitterCorrection &jitter = gps_ptr->jitter;
    jitter.correct_offboard_timestamp_usec(START_US, START_US, true);
    EXPECT_TRUE(jitter.last_estimate_exact());
    // the remote clock falls 1s behind
    jitter.correct_offboard_timestamp_usec(START_US + 200000, START_US + 1200000, true);
    EXPECT_FALSE(jitter.last_estimate_exact());
}

AP_GTEST_MAIN()
