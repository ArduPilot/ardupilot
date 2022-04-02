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
        // hack because CUAV_GPS does not support dshot
        const float expected_delta = clock > 50000000 ? 0.13f : 0.35f;
        EXPECT_TRUE(rate_delta < expected_delta);
    }

    void test_prescaler_neopixel(uint32_t clock)
    {
        const uint32_t target_rate = 800000 * AP_HAL::RCOutput::NEOP_BIT_WIDTH_TICKS;
        const uint32_t prescaler = AP_HAL::RCOutput::calculate_bitrate_prescaler(clock, target_rate, false);
        const uint32_t actual_rate = clock / prescaler;

        const float bit_1_width_us = 1000000.0f * AP_HAL::RCOutput::NEOP_BIT_1_TICKS / actual_rate;
        const float bit_0_width_us = 1000000.0f * AP_HAL::RCOutput::NEOP_BIT_0_TICKS / actual_rate;

        // timing requirements from WS2812B spec
        EXPECT_TRUE(bit_1_width_us < (0.85f + 0.15f) && bit_1_width_us > (0.85f - 0.15f));
        EXPECT_TRUE(bit_0_width_us < (0.4f + 0.15f) && bit_0_width_us > (0.4f - 0.15f));
        EXPECT_TRUE((bit_0_width_us + bit_1_width_us) < (1.25f + 0.6f) && (bit_0_width_us + bit_1_width_us) > (1.25f - 0.6f));
    }
};

TEST_P(PrescalerParameterizedTestFixture, DShot150) {
    test_prescaler(GetParam(), 150000 * AP_HAL::RCOutput::DSHOT_BIT_WIDTH_TICKS, true);
}

TEST_P(PrescalerParameterizedTestFixture, DShot300) {
    test_prescaler(GetParam(), 300000 * AP_HAL::RCOutput::DSHOT_BIT_WIDTH_TICKS, true);
}

TEST_P(PrescalerParameterizedTestFixture, DShot600) {
    test_prescaler(GetParam(), 600000 * AP_HAL::RCOutput::DSHOT_BIT_WIDTH_TICKS, true);
}

TEST_P(PrescalerParameterizedTestFixture, DShot1200) {
    test_prescaler(GetParam(), 1200000 * AP_HAL::RCOutput::DSHOT_BIT_WIDTH_TICKS, true);
}

TEST_P(PrescalerParameterizedTestFixture, Passthrough) {
    test_prescaler(GetParam(), 19200 * 10, false);
}

TEST_P(PrescalerParameterizedTestFixture, NeoPixel) {
    test_prescaler_neopixel(GetParam());
}

TEST_P(PrescalerParameterizedTestFixture, ProfiLED) {
    test_prescaler(GetParam(), 1500000 * AP_HAL::RCOutput::NEOP_BIT_WIDTH_TICKS, false);
}

INSTANTIATE_TEST_CASE_P(
        prescaler_Test,
        PrescalerParameterizedTestFixture,
        ::testing::Values(
                200000000,  // H743
                216000000,  // F745
                108000000,  // F745
                 84000000,  // F405
                168000000,  // F405
                 50000000   // CUAV_GPS
        ));

AP_GTEST_MAIN()