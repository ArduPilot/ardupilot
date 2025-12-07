/*
 * Test to verify that when roll=0 and pitch=0, the BODY_FRD transformation
 * using the full Tbn matrix is equivalent to LOCAL_FRD using rotate_xy(yaw).
 *
 * This validates the bug fix in AC_PrecLand where LOCAL_FRD was using
 * rotate_xy(-yaw_rad) instead of rotate_xy(yaw_rad).
 *
 * Related issue: Sign error in LOCAL_FRD support for AC_PrecLand
 */

#include "math_test.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Disable float equality warnings for this test file - we're doing careful
// comparisons of specific float values for test logic, not numeric results
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

/*
 * Test that when roll=0 and pitch=0 (gravity-aligned, like LOCAL_FRD),
 * Tbn * vector == rotate_xy(vector, yaw)
 *
 * This proves the bug fix is correct: we should use positive yaw, not negative.
 */
TEST(LocalFrdRotationTest, TbnEquivalentToRotateXyWhenLevel)
{
    const float accuracy = 1.0e-6f;

    // Test various yaw angles
    float test_yaws_deg[] = {0, 45, 90, 135, 180, -45, -90, 270};

    // Test various input vectors (simulating sensor readings)
    Vector3f test_vectors[] = {
        Vector3f(1.0f, 0.0f, 0.0f),    // Forward
        Vector3f(0.0f, 1.0f, 0.0f),    // Right
        Vector3f(0.0f, 0.0f, 1.0f),    // Down
        Vector3f(1.0f, 1.0f, 0.0f),    // Forward-Right diagonal
        Vector3f(0.707f, 0.707f, 0.5f) // Arbitrary vector
    };

    for (float yaw_deg : test_yaws_deg) {
        float yaw_rad = radians(yaw_deg);

        // Create Tbn matrix with roll=0, pitch=0 (LOCAL_FRD is gravity-aligned)
        Matrix3f Tbn;
        Tbn.from_euler(0.0f, 0.0f, yaw_rad);

        for (const Vector3f& input_vec : test_vectors) {
            // BODY_FRD method: full matrix multiplication
            Vector3f result_body_frd = Tbn * input_vec;

            // LOCAL_FRD method (CORRECTED - positive yaw)
            Vector3f result_local_frd_correct = input_vec;
            result_local_frd_correct.rotate_xy(yaw_rad);

            // LOCAL_FRD method (BUGGED - negative yaw)
            Vector3f result_local_frd_bugged = input_vec;
            result_local_frd_bugged.rotate_xy(-yaw_rad);

            // The corrected LOCAL_FRD should match BODY_FRD
            Vector3f diff_correct = result_body_frd - result_local_frd_correct;
            EXPECT_LE(diff_correct.length(), accuracy)
                << "Yaw=" << yaw_deg << "deg, vec=" << input_vec
                << ": BODY_FRD=" << result_body_frd
                << " but LOCAL_FRD(+yaw)=" << result_local_frd_correct;

            // For non-zero yaw (except 180), the bugged version should NOT match
            if (yaw_deg != 0 && yaw_deg != 180 && yaw_deg != -180) {
                Vector3f diff_bugged = result_body_frd - result_local_frd_bugged;
                // The bugged version should be DIFFERENT (unless vector is purely Z)
                if (input_vec.x != 0 || input_vec.y != 0) {
                    EXPECT_GT(diff_bugged.length(), accuracy)
                        << "Yaw=" << yaw_deg << "deg: Bugged version should NOT match!";
                }
            }
        }
    }
}

/*
 * Detailed test for yaw=90° (facing East) - the clearest demonstration of the bug
 */
TEST(LocalFrdRotationTest, Yaw90DegreesBugDemonstration)
{
    const float accuracy = 1.0e-6f;
    const float yaw_rad = radians(90.0f);

    // Create Tbn for drone facing East (yaw=90°), level flight
    Matrix3f Tbn;
    Tbn.from_euler(0.0f, 0.0f, yaw_rad);

    // Input: "target is ahead" in body frame
    Vector3f target_ahead(1.0f, 0.0f, 0.0f);

    // BODY_FRD: Full matrix multiplication
    Vector3f ned_body_frd = Tbn * target_ahead;

    // Expected: If drone faces East and target is ahead, target is to the East
    // East in NED is [0, 1, 0]
    EXPECT_NEAR(ned_body_frd.x, 0.0f, accuracy);
    EXPECT_NEAR(ned_body_frd.y, 1.0f, accuracy);
    EXPECT_NEAR(ned_body_frd.z, 0.0f, accuracy);

    // LOCAL_FRD with CORRECT positive yaw
    Vector3f ned_local_correct = target_ahead;
    ned_local_correct.rotate_xy(yaw_rad);

    EXPECT_NEAR(ned_local_correct.x, 0.0f, accuracy);
    EXPECT_NEAR(ned_local_correct.y, 1.0f, accuracy);  // East - CORRECT!
    EXPECT_NEAR(ned_local_correct.z, 0.0f, accuracy);

    // LOCAL_FRD with BUGGED negative yaw
    Vector3f ned_local_bugged = target_ahead;
    ned_local_bugged.rotate_xy(-yaw_rad);

    EXPECT_NEAR(ned_local_bugged.x, 0.0f, accuracy);
    EXPECT_NEAR(ned_local_bugged.y, -1.0f, accuracy);  // West - WRONG!
    EXPECT_NEAR(ned_local_bugged.z, 0.0f, accuracy);

    // The correct version should match BODY_FRD
    Vector3f diff_correct = ned_body_frd - ned_local_correct;
    EXPECT_LE(diff_correct.length(), accuracy);

    // The bugged version should NOT match (it's 180° off!)
    Vector3f diff_bugged = ned_body_frd - ned_local_bugged;
    EXPECT_GT(diff_bugged.length(), 1.9f);  // Should be ~2.0 (opposite directions)
}

/*
 * Test that extracting yaw from Tbn and using it in rotate_xy gives same result
 * This mimics exactly what AC_PrecLand does for LOCAL_FRD
 */
TEST(LocalFrdRotationTest, ExtractYawAndRotateMatchesTbnMultiply)
{
    const float accuracy = 1.0e-6f;

    float test_yaws_deg[] = {0, 30, 45, 60, 90, 120, 135, 150, 180, -30, -90, -135};

    for (float yaw_deg : test_yaws_deg) {
        float original_yaw_rad = radians(yaw_deg);

        // Create Tbn with only yaw (roll=0, pitch=0)
        Matrix3f Tbn;
        Tbn.from_euler(0.0f, 0.0f, original_yaw_rad);

        // Extract yaw back (this is what AC_PrecLand does)
        float roll_rad, pitch_rad, yaw_rad;
        Tbn.to_euler(&roll_rad, &pitch_rad, &yaw_rad);

        // The extracted yaw should match original
        EXPECT_NEAR(yaw_rad, original_yaw_rad, accuracy)
            << "Yaw extraction failed for " << yaw_deg << " degrees";

        // Now test that Tbn * vec == vec.rotate_xy(extracted_yaw)
        Vector3f test_vec(1.0f, 2.0f, 3.0f);

        Vector3f result_tbn = Tbn * test_vec;

        Vector3f result_rotate_xy = test_vec;
        result_rotate_xy.rotate_xy(yaw_rad);  // Using POSITIVE yaw (the fix!)

        Vector3f diff = result_tbn - result_rotate_xy;
        EXPECT_LE(diff.length(), accuracy)
            << "Mismatch at yaw=" << yaw_deg << "deg";
    }
}

#pragma GCC diagnostic pop

AP_GTEST_MAIN()
