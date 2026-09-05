#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Declination/AP_Declination.h>

/*
  The WMM tables are sampled every 10 degrees and read with a bilinear
  interpolation, which uses the next sample along in both latitude and
  longitude.  The indexes are constrained so that the +1 stays inside
  the table; sweep the whole sampled area, and the edges in particular,
  to keep it that way.  Run under the address sanitizer to check the
  indexing rather than just the results.
 */
TEST(Declination, sampled_area)
{
    for (float lat = -90; lat <= 90; lat += 5) {
        for (float lon = -180; lon <= 180; lon += 5) {
            float intensity_gauss = 0, declination_deg = 0, inclination_deg = 0;
            AP_Declination::get_mag_field_ef(lat, lon, intensity_gauss, declination_deg, inclination_deg);
            EXPECT_TRUE(isfinite(declination_deg));
            EXPECT_TRUE(isfinite(inclination_deg));
            EXPECT_TRUE(isfinite(intensity_gauss));
            EXPECT_LE(fabsf(declination_deg), 180);
            EXPECT_LE(fabsf(inclination_deg), 90);
            EXPECT_GT(intensity_gauss, 0);

            // get_declination is the same lookup, so it must agree
            EXPECT_FLOAT_EQ(AP_Declination::get_declination(lat, lon), declination_deg);
        }
    }
}

/*
  positions off the ends of the table must be clamped onto it rather
  than indexing past the end
 */
TEST(Declination, outside_sampled_area)
{
    const struct {
        float lat;
        float lon;
    } positions[] {
        { -91, 0 }, { 91, 0 },
        { 0, -181 }, { 0, 181 },
        { 91, 181 }, { -91, -181 },
        { 1000, 1000 }, { -1000, -1000 },
    };
    for (const auto &p : positions) {
        float intensity_gauss = 0, declination_deg = 0, inclination_deg = 0;
        // out of range input is reported as such...
        EXPECT_FALSE(AP_Declination::get_mag_field_ef(p.lat, p.lon,
                                                      intensity_gauss,
                                                      declination_deg,
                                                      inclination_deg));
        // ...and the outputs are still numbers.  Only the index is
        // constrained onto the table, not the position used for the
        // interpolation, so a long way outside the sampled area the
        // extrapolation is meaningless - that is why nothing stronger
        // than "finite" is checked here
        EXPECT_TRUE(isfinite(declination_deg));
        EXPECT_TRUE(isfinite(inclination_deg));
        EXPECT_TRUE(isfinite(intensity_gauss));
    }
}

AP_GTEST_MAIN()
int hal = 0;
