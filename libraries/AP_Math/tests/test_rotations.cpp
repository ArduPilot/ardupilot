#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_CustomRotations/AP_CustomRotations.h>


AP_CustomRotations cust_rot;
const AP_HAL::HAL& hal = AP_HAL::get_HAL();


static void test_euler(enum Rotation rotation, float roll, float pitch, float yaw)
{
    Vector3f v, v1, v2, diff;
    Matrix3f rotmat;
    const float accuracy = 1.0e-6f;

    v.x = 1;
    v.y = 2;
    v.z = 3;
    v1 = v;

    v1.rotate(rotation);

    rotmat.from_euler(radians(roll), radians(pitch), radians(yaw));
    v2 = v;
    v2 = rotmat * v2;

    diff = (v2 - v1);
    EXPECT_LE(diff.length(), accuracy);

    // quaternion rotation test
    const float q_accuracy = 1.0e-3f;
    Quaternion q, qe;
    q.from_rotation(rotation);
    qe.from_euler(radians(roll), radians(pitch), radians(yaw));
    float q_roll, q_pitch, q_yaw, qe_roll, qe_pitch, qe_yaw;
    q.to_euler(q_roll, q_pitch, q_yaw);
    qe.to_euler(qe_roll, qe_pitch, qe_yaw);
    float roll_diff = fabsf(wrap_PI(q_roll - qe_roll));
    float pitch_diff = fabsf(wrap_PI(q_pitch - qe_pitch));
    float yaw_diff = fabsf(wrap_PI(q_yaw - qe_yaw));
    EXPECT_LE(roll_diff, q_accuracy);
    EXPECT_LE(pitch_diff, q_accuracy);
    EXPECT_LE(yaw_diff, q_accuracy);

    // test custom rotations
    AP::custom_rotations().set(ROTATION_CUSTOM_1, roll, pitch, yaw);
    v1 = v;
    v1.rotate(ROTATION_CUSTOM_1);

    diff = (v2 - v1);
    EXPECT_LE(diff.length(), accuracy);

    Quaternion qc;
    qc.from_rotation(ROTATION_CUSTOM_1);
    float qc_roll, qc_pitch, qc_yaw;
    qc.to_euler(qc_roll, qc_pitch, qc_yaw);
    roll_diff = fabsf(wrap_PI(qc_roll - qe_roll));
    pitch_diff = fabsf(wrap_PI(qc_pitch - qe_pitch));
    yaw_diff = fabsf(wrap_PI(qc_yaw - qe_yaw));
    EXPECT_LE(roll_diff, q_accuracy);
    EXPECT_LE(pitch_diff, q_accuracy);
    EXPECT_LE(yaw_diff, q_accuracy);
}

TEST(RotationsTest, TestEulers)
{
    test_euler(ROTATION_NONE,               0,   0,   0);
    test_euler(ROTATION_YAW_45,             0,   0,  45);
    test_euler(ROTATION_YAW_90,             0,   0,  90);
    test_euler(ROTATION_YAW_135,            0,   0, 135);
    test_euler(ROTATION_YAW_180,            0,   0, 180);
    test_euler(ROTATION_YAW_225,            0,   0, 225);
    test_euler(ROTATION_YAW_270,            0,   0, 270);
    test_euler(ROTATION_YAW_315,            0,   0, 315);
    test_euler(ROTATION_ROLL_180,         180,   0,   0);
    test_euler(ROTATION_ROLL_180_YAW_45,  180,   0,  45);
    test_euler(ROTATION_ROLL_180_YAW_90,  180,   0,  90);
    test_euler(ROTATION_ROLL_180_YAW_135, 180,   0, 135);
    test_euler(ROTATION_PITCH_180,          0, 180,   0);
    test_euler(ROTATION_ROLL_180_YAW_225, 180,   0, 225);
    test_euler(ROTATION_ROLL_180_YAW_270, 180,   0, 270);
    test_euler(ROTATION_ROLL_180_YAW_315, 180,   0, 315);
    test_euler(ROTATION_ROLL_90,           90,   0,   0);
    test_euler(ROTATION_ROLL_90_YAW_45,    90,   0,  45);
    test_euler(ROTATION_ROLL_90_YAW_90,    90,   0,  90);
    test_euler(ROTATION_ROLL_90_YAW_135,   90,   0, 135);
    test_euler(ROTATION_ROLL_270,         270,   0,   0);
    test_euler(ROTATION_ROLL_270_YAW_45,  270,   0,  45);
    test_euler(ROTATION_ROLL_270_YAW_90,  270,   0,  90);
    test_euler(ROTATION_ROLL_270_YAW_135, 270,   0, 135);
    test_euler(ROTATION_PITCH_90,           0,  90,   0);
    test_euler(ROTATION_PITCH_270,          0, 270,   0);
    test_euler(ROTATION_PITCH_180_YAW_90,   0, 180,  90);
    test_euler(ROTATION_PITCH_180_YAW_270,  0, 180, 270);
    test_euler(ROTATION_ROLL_90_PITCH_90,  90,  90,   0);
    test_euler(ROTATION_ROLL_180_PITCH_90,180,  90,   0);
    test_euler(ROTATION_ROLL_270_PITCH_90,270,  90,   0);
    test_euler(ROTATION_ROLL_90_PITCH_180, 90, 180,   0);
    test_euler(ROTATION_ROLL_270_PITCH_180,270,180,   0);
    test_euler(ROTATION_ROLL_90_PITCH_270, 90, 270,   0);
    test_euler(ROTATION_ROLL_180_PITCH_270,180,270,   0);
    test_euler(ROTATION_ROLL_270_PITCH_270,270,270,   0);
    test_euler(ROTATION_ROLL_90_PITCH_180_YAW_90, 90, 180,  90);
    test_euler(ROTATION_ROLL_90_YAW_270,   90,   0, 270);
    test_euler(ROTATION_ROLL_90_PITCH_68_YAW_293,90,68.8,293.3);
    test_euler(ROTATION_ROLL_45,45,0,0);
    test_euler(ROTATION_ROLL_315,315,0,0);
    test_euler(ROTATION_PITCH_7, 0, 7, 0);
}


TEST(RotationsTest, TestRotationInverse)
{
    // rotate inverse test(Vector (1,1,1))
    Vector3f vec(1.0f,1.0f,1.0f), cmp_vec(1.0f, 1.0f, 1.0f);
    for (enum Rotation r = ROTATION_NONE;
         r < ROTATION_MAX;
         r = (enum Rotation)((uint8_t)r+1)) {
        vec.rotate(r);
        vec.rotate_inverse(r);
        EXPECT_LE((vec - cmp_vec).length(), 1e-5);
    }
}

TEST(RotationsTest, TestRotateMatrix)
{
    for (enum Rotation r = ROTATION_NONE;
         r < ROTATION_MAX;
         r = (enum Rotation)((uint8_t)r+1)) {
        Vector3f vec(1,2,3);
        Vector3f vec2 = vec;
        vec.rotate(r);
        Matrix3f m;
        m.from_rotation(r);
        vec2 = m * vec2;
        EXPECT_LE((vec - vec2).length(), 1e-5);
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
TEST(RotationsTest, TestFailedGetLinux)
{
    AP::custom_rotations().set(ROTATION_CUSTOM_OLD, 0, 0, 0);
    Vector3f vec(1,2,3);
    Vector3f vec2 = vec;
    AP::custom_rotations().rotate(ROTATION_CUSTOM_OLD, vec2);
    EXPECT_TRUE(vec == vec2);
    Vector3d vecd(1,2,3);
    Vector3d vecd2 = vecd;
    AP::custom_rotations().rotate(ROTATION_CUSTOM_OLD, vecd2);
    EXPECT_TRUE(vecd == vecd2);
    Quaternion q(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion q2(1.0f, 0.0f, 0.0f, 0.0f);
    AP::custom_rotations().from_rotation(ROTATION_CUSTOM_OLD, q2);
    for (int a = 0; a < 4; ++a) {
        EXPECT_FLOAT_EQ(q[a], q2[a]);
    }
    QuaternionD qd(1.0, 0.0, 0.0, 0.0);
    QuaternionD qd2(1.0, 0.0, 0.0, 0.0);
    AP::custom_rotations().from_rotation(ROTATION_CUSTOM_OLD, qd2);
    for (int a = 0; a < 4; ++a) {
        EXPECT_FLOAT_EQ(qd[a], qd2[a]);
    }
}
#endif

static void breakSingleton()
{
    AP_CustomRotations cust_rot1;
}
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
TEST(RotationsTest, TestFailedGetLinux)
{
    EXPECT_EXIT(AP::custom_rotations().set(ROTATION_CUSTOM_OLD, 0, 0, 0), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bad_rotation");
    EXPECT_EXIT(AP::custom_rotations().set(ROTATION_CUSTOM_END, 0, 0, 0), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bad_rotation");
    EXPECT_EXIT(breakSingleton(), testing::KilledBySignal(SIGABRT), "AP_CustomRotations must be singleton");
}
#endif

/*TEST(RotationsTest, TestRotateEqual)
{
    for (enum Rotation r = (enum Rotation)((uint8_t)ROTATION_MAX-1); r > ROTATION_NONE; r = (enum Rotation)((uint8_t)r-1)) {
        for (enum Rotation r2 = ROTATION_NONE; r2 < r; r2 = (enum Rotation)((uint8_t)r2+1)) {
            if (rotation_equal(r,r2)) {
                hal.console->printf("Rotation %i same as %i\n", r, r2);
            }
        }
    }
}*/

/*
  rotate a matrix using a give order, specified as a string
  for example "321"
 */
static void rotate_ordered(Matrix3f &m, const char *order,
                           const float roll_deg,
                           const float pitch_deg,
                           const float yaw_deg)
{
    while (*order) {
        Matrix3f m2;
        switch (*order) {
        case '1':
            m2.from_euler(radians(roll_deg), 0, 0);
            break;
        case '2':
            m2.from_euler(0, radians(pitch_deg), 0);
            break;
        case '3':
            m2.from_euler(0, 0, radians(yaw_deg));
            break;
        }
        m *= m2;
        order++;
    }
}

/*
  test the two euler orders we use in ArduPilot
 */
TEST(RotationsTest, TestEulerOrder)
{
    const float roll_deg = 20;
    const float pitch_deg = 31;
    const float yaw_deg = 72;
    float r, p, y;
    Vector3f v;

    Matrix3f m;

    // apply in 321 ordering
    m.identity();
    rotate_ordered(m, "321", roll_deg, pitch_deg, yaw_deg);

    // get using to_euler
    m.to_euler(&r, &p, &y);

    EXPECT_FLOAT_EQ(degrees(r), roll_deg);
    EXPECT_FLOAT_EQ(degrees(p), pitch_deg);
    EXPECT_FLOAT_EQ(degrees(y), yaw_deg);

    // get using to_euler312, should not match
    v = m.to_euler312();

    EXPECT_GE(fabsf(degrees(v.x)-roll_deg), 1);
    EXPECT_GE(fabsf(degrees(v.y)-pitch_deg), 1);
    EXPECT_GE(fabsf(degrees(v.z)-yaw_deg), 1);

    // apply in 312 ordering
    m.identity();
    rotate_ordered(m, "312", roll_deg, pitch_deg, yaw_deg);

    // get using to_euler312
    v = m.to_euler312();

    EXPECT_FLOAT_EQ(degrees(v.x), roll_deg);
    EXPECT_FLOAT_EQ(degrees(v.y), pitch_deg);
    EXPECT_FLOAT_EQ(degrees(v.z), yaw_deg);

    // get using to_euler, should not match
    m.to_euler(&r, &p, &y);

    EXPECT_GE(fabsf(degrees(r)-roll_deg), 1);
    EXPECT_GE(fabsf(degrees(p)-pitch_deg), 1);
    EXPECT_GE(fabsf(degrees(y)-yaw_deg), 1);
}


AP_GTEST_PANIC()
AP_GTEST_MAIN()
