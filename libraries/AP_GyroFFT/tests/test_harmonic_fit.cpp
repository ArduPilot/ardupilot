#include <AP_gtest.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/Util.h>
#include <AP_GyroFFT/AP_GyroFFT.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(harmonic_fit_Test, Basic)
{
#if HAL_GYROFFT_ENABLED
    float freqs[] { 176.9, 57.2, 228.7 };
    uint8_t harmonics;
    float harmonic = AP_GyroFFT::calculate_notch_frequency(freqs, 3, 10, harmonics);

    printf("FFT: Found peaks at %.1f/%.1f/%.1fHz\n", freqs[0], freqs[1], freqs[2]);
    printf("FFT: Selected %.1fHz %d\n", harmonic, harmonics);

    EXPECT_TRUE(is_equal(harmonic, 57.2f));
#endif
}

AP_GTEST_MAIN()
