#include <AP_gtest.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/RCOutput.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class PrescalerParameterizedTestFixture :public ::testing::TestWithParam<uint32_t> {
protected:
    void test_prescaler(uint32_t clock, uint32_t target_rate, bool is_dshot)
    {
        const uint32_t prescaler = AP_HAL::RCOutput::calculate_bitrate_prescaler(clock, target_rate, is_dshot);
        // we would like at most a 1% discrepancy in target versus actual
        const float rate_delta = fabsf(float(clock / prescaler) - target_rate) / target_rate;
        EXPECT_TRUE(rate_delta < 0.20f);
    }
};

TEST_P(PrescalerParameterizedTestFixture, DShot150) {
    test_prescaler(GetParam(), 150000 * 20, true);
}

TEST_P(PrescalerParameterizedTestFixture, DShot300) {
    test_prescaler(GetParam(), 300000 * 20, true);
}

TEST_P(PrescalerParameterizedTestFixture, DShot600) {
    test_prescaler(GetParam(), 600000 * 20, true);
}

TEST_P(PrescalerParameterizedTestFixture, DShot1200) {
    test_prescaler(GetParam(), 1200000 * 20, true);
}

TEST_P(PrescalerParameterizedTestFixture, Passthrough) {
    test_prescaler(GetParam(), 19200 * 10, false);
}

TEST_P(PrescalerParameterizedTestFixture, NeoPixel) {
    test_prescaler(GetParam(), 800000 * 20, false);
}

TEST_P(PrescalerParameterizedTestFixture, ProfiLED) {
    test_prescaler(GetParam(), 1500000 * 20, false);
}

INSTANTIATE_TEST_CASE_P(
        prescaler_Test,
        PrescalerParameterizedTestFixture,
        ::testing::Values(
                200000000,  // H743
                216000000,  // F745
                108000000,  // F745
                 84000000,  // F405
                168000000   // F405
        ));

AP_GTEST_MAIN()