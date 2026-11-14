// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/Bindings.h>
#include <AP_AHRS/AP_AHRS_config.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_MODULE_ENABLED
#include <AP_Module/BindingSupport.h>
#include <AP_Module/HostApi.h>
#include <AP_Math/AP_Math.h>
#include <initializer_list>
#include <AP_InternalError/AP_InternalError.h>
#if defined(__linux__)
#include <unistd.h>
#endif

using AP_Module::BindingWire;

static void check(uint32_t id, std::initializer_list<float> input, std::initializer_list<float> expected)
{
    // Deliberately unaligned wire buffers; no C++ value layout may leak here.
    uint8_t in[33] {}, out[18], want[16] {};
    memset(out, 0xa5, sizeof(out));
    unsigned offset = 1;
    for (const float value : input) {
        BindingWire::write_float(in + offset, value);
        offset += 4;
    }
    offset = 0;
    for (const float value : expected) {
        BindingWire::write_float(want + offset, value);
        offset += 4;
    }
    ASSERT_EQ(AP_Module::binding_dispatch(nullptr, static_cast<AP_Module::BindingId>(id),
                                        in + 1, input.size() * 4, out + 1, expected.size() * 4),
              AP_MODULE_STATUS_OK) << id;
    EXPECT_EQ(memcmp(out + 1, want, expected.size() * 4), 0) << id;
    EXPECT_EQ(out[0], 0xa5);
    EXPECT_EQ(out[1 + expected.size() * 4], 0xa5);
}

static void check_bool(uint32_t id, const Vector3f &v, bool expected)
{
    uint8_t in[12], out = 0xa5;
    BindingWire::write_float(in, v.x);
    BindingWire::write_float(in + 4, v.y);
    BindingWire::write_float(in + 8, v.z);
    ASSERT_EQ(AP_Module::binding_dispatch(nullptr, static_cast<AP_Module::BindingId>(id),
                                        in, sizeof(in), &out, 1), AP_MODULE_STATUS_OK);
    EXPECT_EQ(out, uint8_t(expected));
}

TEST(ModuleMath, VectorCppParity)
{
    for (unsigned i = 0; i < 100; i++) {
        const Vector3f a{0.5f + i * 0.13f, -2.7f + i * 0.27f, 7.3f - i * 0.07f};
        const Vector3f b{-3.2f + i * 0.18f, 1.9f - i * 0.17f, -0.6f + i * 0.02f};
        check(AP_MODULE_BINDING_VECTOR3F_LENGTH, {a.x, a.y, a.z}, {a.length()});
        check(AP_MODULE_BINDING_VECTOR3F_DOT, {a.x, a.y, a.z, b.x, b.y, b.z}, {a.dot(b)});
        check(AP_MODULE_BINDING_VECTOR3F_ANGLE, {a.x, a.y, a.z, b.x, b.y, b.z}, {a.angle(b)});
        const Vector3f cross = a.cross(b), add = a + b, sub = a - b, scale = a.scale(-0.3f);
        check(AP_MODULE_BINDING_VECTOR3F_CROSS, {a.x, a.y, a.z, b.x, b.y, b.z}, {cross.x, cross.y, cross.z});
        check(AP_MODULE_BINDING_VECTOR3F_ADD, {a.x, a.y, a.z, b.x, b.y, b.z}, {add.x, add.y, add.z});
        check(AP_MODULE_BINDING_VECTOR3F_SUB, {a.x, a.y, a.z, b.x, b.y, b.z}, {sub.x, sub.y, sub.z});
        check(AP_MODULE_BINDING_VECTOR3F_SCALE, {a.x, a.y, a.z, -0.3f}, {scale.x, scale.y, scale.z});
        Vector3f normal = a, rotated = a;
        normal.normalize();
        rotated.rotate_xy(0.7f);
        check(AP_MODULE_BINDING_VECTOR3F_NORMALIZE, {a.x, a.y, a.z}, {normal.x, normal.y, normal.z});
        check(AP_MODULE_BINDING_VECTOR3F_ROTATE_XY, {a.x, a.y, a.z, 0.7f}, {rotated.x, rotated.y, rotated.z});
        check_bool(AP_MODULE_BINDING_VECTOR3F_IS_NAN, a, a.is_nan());
        check_bool(AP_MODULE_BINDING_VECTOR3F_IS_INF, a, a.is_inf());
        check_bool(AP_MODULE_BINDING_VECTOR3F_IS_ZERO, a, a.is_zero());
    }
}

TEST(ModuleMath, UncheckedFloatComponents)
{
    const uint32_t patterns[] = {0, 0x80000000, 1, 0x7f7fffff, 0x7f800000, 0xff800000, 0x7fc12345};
    for (const uint32_t bits : patterns) {
        float value;
        memcpy(&value, &bits, 4);
        const Vector3f v{value, -0.0f, 0.0f};
        check_bool(AP_MODULE_BINDING_VECTOR3F_IS_NAN, v, v.is_nan());
        check_bool(AP_MODULE_BINDING_VECTOR3F_IS_INF, v, v.is_inf());
        check_bool(AP_MODULE_BINDING_VECTOR3F_IS_ZERO, v, v.is_zero());
        const Vector3f scaled = v.scale(1.0f);
        check(AP_MODULE_BINDING_VECTOR3F_SCALE, {v.x, v.y, v.z, 1.0f}, {scaled.x, scaled.y, scaled.z});
    }
}

#if AP_AHRS_ENABLED && AP_INTERNALERROR_ENABLED && defined(__linux__) && !defined(HAL_DEBUG_BUILD)
TEST(ModuleMath, ZeroQuaternionPreservesInternalError)
{
    // Isolate the persistent error flag from the remainder of the test process.
    ASSERT_EXIT({
        uint8_t in[16] {};
        uint8_t out[16] {};
        const uint32_t before = AP::internalerror().count();
        const int32_t status = AP_Module::binding_dispatch(
                               nullptr, static_cast<AP_Module::BindingId>(AP_MODULE_BINDING_QUATERNION_NORMALIZE),
                               in, sizeof(in), out, sizeof(out));
        const bool ok = status == AP_MODULE_STATUS_OK && AP::internalerror().count() == before + 1 &&
                        (AP::internalerror().errors() & uint32_t(AP_InternalError::error_t::flow_of_control)) != 0 &&
                        memcmp(in, out, sizeof(in)) == 0;
        _exit(ok ? 0 : 1);
    }, ::testing::ExitedWithCode(0), "");
}
#endif

#if AP_AHRS_ENABLED
TEST(ModuleMath, QuaternionCppParity)
{
    for (unsigned i = 0; i < 100; i++) {
        const float roll = -0.7f + i * 0.013f, pitch = 0.4f - i * 0.011f, yaw = 1.1f - i * 0.05f;
        Quaternion q;
        q.from_euler(roll, pitch, yaw);
        check(AP_MODULE_BINDING_QUATERNION_FROM_EULER, {1, 0, 0, 0, roll, pitch, yaw}, {q.q1, q.q2, q.q3, q.q4});
        check(AP_MODULE_BINDING_QUATERNION_LENGTH, {q.q1, q.q2, q.q3, q.q4}, {q.length()});
        check(AP_MODULE_BINDING_QUATERNION_GET_EULER_ROLL, {q.q1, q.q2, q.q3, q.q4}, {q.get_euler_roll()});
        check(AP_MODULE_BINDING_QUATERNION_GET_EULER_PITCH, {q.q1, q.q2, q.q3, q.q4}, {q.get_euler_pitch()});
        check(AP_MODULE_BINDING_QUATERNION_GET_EULER_YAW, {q.q1, q.q2, q.q3, q.q4}, {q.get_euler_yaw()});
        const Quaternion inverse = q.inverse(), product = q * inverse;
        check(AP_MODULE_BINDING_QUATERNION_INVERSE, {q.q1, q.q2, q.q3, q.q4},
        {inverse.q1, inverse.q2, inverse.q3, inverse.q4});
        check(AP_MODULE_BINDING_QUATERNION_MUL, {q.q1, q.q2, q.q3, q.q4, inverse.q1, inverse.q2, inverse.q3, inverse.q4},
        {product.q1, product.q2, product.q3, product.q4});
        Quaternion normal{q.q1 * 1.7f, q.q2 * 1.7f, q.q3 * 1.7f, q.q4 * 1.7f};
        const Quaternion scaled = normal;
        normal.normalize();
        check(AP_MODULE_BINDING_QUATERNION_NORMALIZE, {scaled.q1, scaled.q2, scaled.q3, scaled.q4},
        {normal.q1, normal.q2, normal.q3, normal.q4});
        Vector3f v{1.7f, -2.3f, 4.1f}, axis;
        q.earth_to_body(v);
        q.to_axis_angle(axis);
        check(AP_MODULE_BINDING_QUATERNION_EARTH_TO_BODY, {q.q1, q.q2, q.q3, q.q4, 1.7f, -2.3f, 4.1f}, {v.x, v.y, v.z});
        check(AP_MODULE_BINDING_QUATERNION_TO_AXIS_ANGLE, {q.q1, q.q2, q.q3, q.q4}, {axis.x, axis.y, axis.z});
        Quaternion rotation, delta;
        rotation.from_axis_angle(Vector3f{0, 1, 0}, roll);
        delta.from_angular_velocity(Vector3f{roll, pitch, yaw}, 0.025f);
        check(AP_MODULE_BINDING_QUATERNION_FROM_AXIS_ANGLE, {1, 0, 0, 0, 0, 1, 0, roll},
        {rotation.q1, rotation.q2, rotation.q3, rotation.q4});
        check(AP_MODULE_BINDING_QUATERNION_FROM_ANGULAR_VELOCITY, {1, 0, 0, 0, roll, pitch, yaw, 0.025f},
        {delta.q1, delta.q2, delta.q3, delta.q4});
    }
}
#endif

TEST(ModuleMath, ValidWireBuffersStayWithinBounds)
{
    // id, input bytes, output bytes, AHRS dependency (independent of id order).
    const uint32_t sizes[][4] = {
        {AP_MODULE_BINDING_QUATERNION_EARTH_TO_BODY, 28, 12, 1},
        {AP_MODULE_BINDING_QUATERNION_FROM_ANGULAR_VELOCITY, 32, 16, 1},
        {AP_MODULE_BINDING_QUATERNION_FROM_AXIS_ANGLE, 32, 16, 1},
        {AP_MODULE_BINDING_QUATERNION_FROM_EULER, 28, 16, 1},
        {AP_MODULE_BINDING_QUATERNION_GET_EULER_PITCH, 16, 4, 1},
        {AP_MODULE_BINDING_QUATERNION_GET_EULER_ROLL, 16, 4, 1},
        {AP_MODULE_BINDING_QUATERNION_GET_EULER_YAW, 16, 4, 1},
        {AP_MODULE_BINDING_QUATERNION_INVERSE, 16, 16, 1},
        {AP_MODULE_BINDING_QUATERNION_LENGTH, 16, 4, 1},
        {AP_MODULE_BINDING_QUATERNION_NORMALIZE, 16, 16, 1},
        {AP_MODULE_BINDING_QUATERNION_TO_AXIS_ANGLE, 16, 12, 1},
        {AP_MODULE_BINDING_QUATERNION_MUL, 32, 16, 1},
        {AP_MODULE_BINDING_VECTOR3F_ANGLE, 24, 4},
        {AP_MODULE_BINDING_VECTOR3F_CROSS, 24, 12},
        {AP_MODULE_BINDING_VECTOR3F_DOT, 24, 4},
        {AP_MODULE_BINDING_VECTOR3F_IS_INF, 12, 1},
        {AP_MODULE_BINDING_VECTOR3F_IS_NAN, 12, 1},
        {AP_MODULE_BINDING_VECTOR3F_IS_ZERO, 12, 1},
        {AP_MODULE_BINDING_VECTOR3F_LENGTH, 12, 4},
        {AP_MODULE_BINDING_VECTOR3F_NORMALIZE, 12, 12},
        {AP_MODULE_BINDING_VECTOR3F_ROTATE_XY, 16, 12},
        {AP_MODULE_BINDING_VECTOR3F_SCALE, 16, 12},
        {AP_MODULE_BINDING_VECTOR3F_ADD, 24, 12},
        {AP_MODULE_BINDING_VECTOR3F_SUB, 24, 12},
    };
    uint8_t in[34], out[18];
    for (const auto &entry : sizes) {
        const unsigned id = entry[0], ni = entry[1], no = entry[2];
        // Generated wrappers and the contract hash guarantee the wire sizes.
        // Exercise valid, unaligned packets rather than violating that contract.
        memset(in, 0xa5, sizeof(in));
        memset(out, 0xa5, sizeof(out));
        for (unsigned offset = 0; offset < ni; offset += sizeof(float)) {
            BindingWire::write_float(in + 1 + offset, 0.25f);
        }
        const int32_t status = AP_Module::binding_dispatch(
                                   nullptr, static_cast<AP_Module::BindingId>(id), in + 1, ni, out + 1, no);
#if !AP_AHRS_ENABLED
        EXPECT_EQ(status, entry[3] != 0 ? AP_MODULE_STATUS_UNAVAILABLE : AP_MODULE_STATUS_OK) << id;
#else
        EXPECT_EQ(status, AP_MODULE_STATUS_OK) << id;
#endif
        EXPECT_EQ(in[0], 0xa5);
        EXPECT_EQ(in[ni + 1], 0xa5);
        EXPECT_EQ(out[0], 0xa5);
        for (unsigned offset = no + 1; offset < sizeof(out); offset++) {
            EXPECT_EQ(out[offset], 0xa5) << id;
        }
    }
    EXPECT_EQ(AP_Module::binding_dispatch(nullptr, static_cast<AP_Module::BindingId>(UINT32_MAX),
                                        in, 12, out, 12), AP_MODULE_STATUS_INVALID);
}
#endif

AP_GTEST_MAIN()
