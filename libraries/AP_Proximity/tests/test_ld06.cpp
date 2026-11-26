#include <AP_gtest.h>

#include <AP_Proximity/AP_Proximity_LD06.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_PROXIMITY_LD06_ENABLED

using ScanPoint = AP_Proximity_LD06::ScanPoint;

// one rotation at 10Hz: 450 points 0.8 degrees apart, all without a return
static constexpr uint16_t SPEED_DPS = 3600;
static constexpr uint16_t POINTS = 450;
static constexpr float STEP_DEG = 0.8f;

static void empty_rotation(ScanPoint *points)
{
    for (uint16_t i = 0; i < POINTS; i++) {
        points[i] = {};
        points[i].angle_deg = 0.4f + i * STEP_DEG;
    }
}

// put an object of count points at distance_mm, starting at point first
static void add_object(ScanPoint *points, uint16_t first, uint16_t count, uint16_t distance_mm, uint8_t intensity)
{
    for (uint16_t i = first; i < first + count; i++) {
        points[i].distance_mm = distance_mm;
        points[i].intensity = intensity;
    }
}

static bool all_kept(const ScanPoint *points, uint16_t first, uint16_t count)
{
    for (uint16_t i = first; i < first + count; i++) {
        if (!points[i].keep) {
            return false;
        }
    }
    return true;
}

static bool none_kept(const ScanPoint *points, uint16_t first, uint16_t count)
{
    for (uint16_t i = first; i < first + count; i++) {
        if (points[i].keep) {
            return false;
        }
    }
    return true;
}

// a wall is kept, a lone false return next to it is dropped
TEST(AP_Proximity_LD06, GroupFilterWallAndNoise)
{
    ScanPoint points[POINTS];
    empty_rotation(points);
    add_object(points, 100, 20, 2000, 200);     // wall at 2m
    add_object(points, 200, 1, 1500, 100);      // single point
    add_object(points, 300, 2, 3000, 100);      // two points
    AP_Proximity_LD06::group_filter(points, POINTS, SPEED_DPS);

    EXPECT_TRUE(all_kept(points, 100, 20));
    EXPECT_FALSE(points[100].small_group);
    EXPECT_TRUE(none_kept(points, 200, 1));
    EXPECT_TRUE(none_kept(points, 300, 2));
}

// small groups need a strong return, larger groups only a weak one
TEST(AP_Proximity_LD06, GroupFilterIntensity)
{
    ScanPoint points[POINTS];
    empty_rotation(points);
    add_object(points, 50, 2, 1000, 230);       // two strong points
    add_object(points, 150, 3, 1000, 10);       // three very weak points
    add_object(points, 250, 3, 1000, 100);      // three points
    add_object(points, 350, 20, 1000, 10);      // a large weak group
    AP_Proximity_LD06::group_filter(points, POINTS, SPEED_DPS);

    EXPECT_TRUE(all_kept(points, 50, 2));
    EXPECT_TRUE(points[50].small_group);
    EXPECT_TRUE(none_kept(points, 150, 3));
    EXPECT_TRUE(all_kept(points, 250, 3));
    EXPECT_TRUE(points[250].small_group);
    EXPECT_TRUE(all_kept(points, 350, 20));
    EXPECT_FALSE(points[350].small_group);
}

// a group across 0/360 degrees is one group
TEST(AP_Proximity_LD06, GroupFilterWrap)
{
    ScanPoint points[POINTS];
    empty_rotation(points);
    add_object(points, 0, 2, 2500, 100);
    add_object(points, POINTS - 2, 2, 2500, 100);
    AP_Proximity_LD06::group_filter(points, POINTS, SPEED_DPS);

    EXPECT_TRUE(all_kept(points, 0, 2));
    EXPECT_TRUE(all_kept(points, POINTS - 2, 2));
}

// a small group is used only when the previous sweep saw it too,
// a wall is used at once
TEST(AP_Proximity_LD06, ConfirmSmallGroups)
{
    uint16_t last_sweep_mm[LD06_SWEEP_BINS] {};
    uint16_t scratch_mm[LD06_SWEEP_BINS];
    ScanPoint points[POINTS];

    for (uint8_t sweep = 0; sweep < 2; sweep++) {
        empty_rotation(points);
        add_object(points, 100, 20, 2000, 200);     // wall
        add_object(points, 200, 3, 3000, 100);      // small object
        AP_Proximity_LD06::group_filter(points, POINTS, SPEED_DPS);
        AP_Proximity_LD06::confirm_small_groups(points, POINTS, last_sweep_mm, scratch_mm);

        EXPECT_TRUE(all_kept(points, 100, 20));
        if (sweep == 0) {
            EXPECT_TRUE(none_kept(points, 200, 3));
        } else {
            EXPECT_TRUE(all_kept(points, 200, 3));
        }
    }

    // a small group somewhere else is not confirmed by it
    empty_rotation(points);
    add_object(points, 300, 3, 3000, 100);
    AP_Proximity_LD06::group_filter(points, POINTS, SPEED_DPS);
    AP_Proximity_LD06::confirm_small_groups(points, POINTS, last_sweep_mm, scratch_mm);
    EXPECT_TRUE(none_kept(points, 300, 3));
}

#endif // AP_PROXIMITY_LD06_ENABLED

AP_GTEST_MAIN()
