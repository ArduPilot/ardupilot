#include <AP_gtest.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/RCOutput.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

enum Type {
    DSHOT,
    NEOPIXEL,
    NONE
};

class PrescalerParameterizedTestFixture :public ::testing::TestWithParam<uint32_t> {
protected:

    struct TestResult {
        uint32_t clock, target, rate, prescaler;
        Type type;
    };

    static TestResult test_results[];
    static uint32_t test_index;

    void test_prescaler(uint32_t clock, uint32_t target_rate, bool is_dshot)
    {
        const uint32_t prescaler = AP_HAL::RCOutput::calculate_bitrate_prescaler(clock, target_rate, is_dshot);
        // we would like at most a 1% discrepancy in target versus actual
        const float rate_delta = fabsf(float(clock / (prescaler + 1)) - target_rate) / target_rate;
        // hack because CUAV_GPS does not support dshot
        const float expected_delta = clock > 50000000 ? 0.03f : 0.05f;
        ::printf("Clock: %uMHz, Target: %uKHz, Rate: %uKHz, prescaler: %u, error: %.1f%%, Dshot: %i\n",
                  clock/1000000, target_rate/1000, (clock/(prescaler+1))/1000, prescaler, rate_delta * 100.f, is_dshot);
        // assert the output of expected results
        EXPECT_EQ(test_results[test_index].clock, clock);
        EXPECT_EQ(test_results[test_index].target, target_rate);
        EXPECT_EQ(test_results[test_index].rate, clock/(prescaler+1));
        EXPECT_EQ(test_results[test_index].prescaler, prescaler);
        EXPECT_EQ(test_results[test_index].type, is_dshot?DSHOT:NONE);
        test_index++;
        EXPECT_TRUE(rate_delta < expected_delta);
        if (is_dshot) {
            EXPECT_TRUE(fabs(clock/(prescaler+1.0f)-target_rate) < fabsf(clock/(prescaler+2.0f)-target_rate));
            EXPECT_TRUE(fabs(clock/(float(prescaler))-target_rate) > fabsf(clock/(prescaler+1.0f)-target_rate));
        }
    }

    void test_prescaler_neopixel(uint32_t clock)
    {
        const uint32_t target_rate = 800000 * AP_HAL::RCOutput::NEOP_BIT_WIDTH_TICKS;
        const uint32_t prescaler = AP_HAL::RCOutput::calculate_bitrate_prescaler(clock, target_rate, false);
        const uint32_t actual_rate = clock / prescaler;

        ::printf("NeoPixel Clock: %uMHz, Target: %uKHz, Rate: %uKHz, prescaler: %u\n",
                  clock/1000000, target_rate/1000, (clock/(prescaler+1))/1000, prescaler);

        const float bit_1_width_us = 1000000.0f * AP_HAL::RCOutput::NEOP_BIT_1_TICKS / actual_rate;
        const float bit_0_width_us = 1000000.0f * AP_HAL::RCOutput::NEOP_BIT_0_TICKS / actual_rate;
        // assert the output of expected results
        EXPECT_EQ(test_results[test_index].clock, clock);
        EXPECT_EQ(test_results[test_index].target, target_rate);
        EXPECT_EQ(test_results[test_index].rate, clock/(prescaler+1));
        EXPECT_EQ(test_results[test_index].prescaler, prescaler);
        EXPECT_EQ(test_results[test_index].type, NEOPIXEL);
        test_index++;

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

uint32_t PrescalerParameterizedTestFixture::test_index = 0;

PrescalerParameterizedTestFixture::TestResult PrescalerParameterizedTestFixture::test_results[] = {
    { 200000000, 1200000, 1197604, 166, DSHOT },
    { 216000000, 1200000, 1200000, 179, DSHOT },
    { 108000000, 1200000, 1200000, 89, DSHOT },
    { 84000000, 1200000, 1200000, 69, DSHOT },
    { 168000000, 1200000, 1200000, 139, DSHOT },
    { 50000000, 1200000, 1190476, 41, DSHOT },
    { 200000000, 2400000, 2409638, 82, DSHOT },
    { 216000000, 2400000, 2400000, 89, DSHOT },
    { 108000000, 2400000, 2400000, 44, DSHOT },
    { 84000000, 2400000, 2400000, 34, DSHOT },
    { 168000000, 2400000, 2400000, 69, DSHOT },
    { 50000000, 2400000, 2380952, 20, DSHOT },
    { 200000000, 4800000, 4761904, 41, DSHOT },
    { 216000000, 4800000, 4800000, 44, DSHOT },
    { 108000000, 4800000, 4695652, 22, DSHOT },
    { 84000000, 4800000, 4666666, 17, DSHOT },
    { 168000000, 4800000, 4800000, 34, DSHOT },
    { 50000000, 4800000, 5000000, 9, DSHOT },
    { 200000000, 9600000, 9523809, 20, DSHOT },
    { 216000000, 9600000, 9391304, 22, DSHOT },
    { 108000000, 9600000, 9818181, 10, DSHOT },
    { 84000000, 9600000, 9333333, 8, DSHOT },
    { 168000000, 9600000, 9333333, 17, DSHOT },
    { 50000000, 9600000, 10000000, 4, DSHOT },
    { 200000000, 192000, 191938, 1041, NONE },
    { 216000000, 192000, 192000, 1124, NONE },
    { 108000000, 192000, 192170, 561, NONE },
    { 84000000, 192000, 192219, 436, NONE },
    { 168000000, 192000, 192000, 874, NONE },
    { 50000000, 192000, 192307, 259, NONE },
    { 200000000, 6400000, 6451612, 30, NEOPIXEL },
    { 216000000, 6400000, 6352941, 33, NEOPIXEL },
    { 108000000, 6400000, 6352941, 16, NEOPIXEL },
    { 84000000, 6400000, 6461538, 12, NEOPIXEL },
    { 168000000, 6400000, 6461538, 25, NEOPIXEL },
    { 50000000, 6400000, 6250000, 7, NEOPIXEL },
    { 200000000, 12000000, 11764705, 16, NONE },
    { 216000000, 12000000, 12000000, 17, NONE },
    { 108000000, 12000000, 12000000, 8, NONE },
    { 84000000, 12000000, 12000000, 6, NONE },
    { 168000000, 12000000, 12000000, 13, NONE },
    { 50000000, 12000000, 12500000, 3, NONE }
};