#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

TEST(QuaternionConversionTest, SpecialRotationVectorToQuaternion) {
    const Vector3f small_vec(1e-4f, -2e-4f, 3e-4f);

    Quaternion res;
    res.from_axis_angle(small_vec);

    // Check if the Taylor series approximations of quaternion coefficients violate the unit-length constraint
    EXPECT_FLOAT_EQ(res.length(), 1.0f);
    EXPECT_NEAR(res.q1, 1.0f, 1e-6f);
    EXPECT_NEAR(res.q2, 5e-5f, 1e-6f);
    EXPECT_NEAR(res.q3, -1e-4, 1e-6f);
    EXPECT_NEAR(res.q4, 1.5e-4f, 1e-6f);
}

TEST(QuaternionConversionTest, GeneralRotationVectorToQuaternion) {
    const Vector3f vec(-0.43192759f, -0.21596379f, 0.43192759f);

    Quaternion res;
    res.from_axis_angle(vec);
    const float half_theta = 0.323945691482636f;
    const float re_coeff = cosf(half_theta);
    const float im_coeff = sinf(half_theta);

    EXPECT_NEAR(res.q1, re_coeff, 1e-6f);
    EXPECT_NEAR(res.q2, -2.0f / 3.0f * im_coeff, 1e-6f);
    EXPECT_NEAR(res.q3, -1.0f / 3.0f * im_coeff, 1e-6f);
    EXPECT_NEAR(res.q4, 2.0f / 3.0f * im_coeff, 1e-6f);
}

TEST(QuaternionConversionTest, SpecialQuaternionToRotationVector) {
    Vector3f res;

    Quaternion small_quat(1.0f, 1.15e-3f, 2.5e-4f, -7.5e-5f);
    small_quat.to_axis_angle(res);
    EXPECT_NEAR(res.x, 0.0023f, 1e-6f);
    EXPECT_NEAR(res.y, 5e-4f, 1e-6f);
    EXPECT_NEAR(res.z, -1.5e-4f, 1e-6f);

    Quaternion approx_gt_pi_quat(-0.0042036f, 0.6329564f, 0.1941979f, 0.7494236f);
    approx_gt_pi_quat.to_axis_angle(res);
    EXPECT_NEAR(res.x, -1.9831872f, 1e-6f);
    EXPECT_NEAR(res.y, -0.6084635f, 1e-6f);
    EXPECT_NEAR(res.z, -2.3481037f, 1e-6f);

    Quaternion approx_lt_pi_quat(0.0057963f, 0.7191047f, 0.3164257f, 0.6186515f);
    approx_lt_pi_quat.to_axis_angle(res);
    EXPECT_NEAR(res.x, 2.2508355f, 1e-6f);
    EXPECT_NEAR(res.y, 0.990429f, 1e-6f);
    EXPECT_NEAR(res.z, 1.9364116f, 1e-6f);
}

TEST(QuaternionConversionTest, GeneralQuaternionToRotationVector) {
    Vector3f res;

    Quaternion lt_pi_quat(0.3623578f, 0.6702449f, 0.2949261f, 0.576617f);
    lt_pi_quat.to_axis_angle(res);
    EXPECT_NEAR(res.x, 1.7258802f, 1e-6f);
    EXPECT_NEAR(res.y, 0.7594344f, 1e-6f);
    EXPECT_NEAR(res.z, 1.4847885f, 1e-6f);

    Quaternion gt_pi_quat(-0.4161468f, 0.653891f, 0.2877299f, 0.5625476f);
    gt_pi_quat.to_axis_angle(res);
    EXPECT_NEAR(res.x, -1.6418768f, 1e-6f);
    EXPECT_NEAR(res.y, -0.7224706f, 1e-6f);
    EXPECT_NEAR(res.z, -1.4125197f, 1e-6f);
}

// Tests that the quaternion to rotation matrix conversion formula is correctly derived from the Hamilton's quaternion
// multiplication convention. This specific example is taken from "Why and How to Avoid the Flipped Quaternion
// Multiplication" (https://arxiv.org/pdf/1801.07478.pdf)
TEST(QuaternionConversionTest, QuaternionToRotationMatrix) {
    Matrix3f res;
    Quaternion(0.5f * sqrtf(2.0f), 0.0f, 0.0f, 0.5f * sqrtf(2.0f)).rotation_matrix(res);

    EXPECT_NEAR(res.a.x, 0.0f, 1e-6f);
    EXPECT_NEAR(res.a.y, -1.0f, 1e-6f);
    EXPECT_NEAR(res.a.z, 0.0f, 1e-6f);
    EXPECT_NEAR(res.b.x, 1.0f, 1e-6f);
    EXPECT_NEAR(res.b.y, 0.0f, 1e-6f);
    EXPECT_NEAR(res.b.z, 0.0f, 1e-6f);
    EXPECT_NEAR(res.c.x, 0.0f, 1e-6f);
    EXPECT_NEAR(res.c.y, 0.0f, 1e-6f);
    EXPECT_NEAR(res.c.z, 1.0f, 1e-6f);
}

// Tests that quaternion multiplication obeys Hamilton's quaternion multiplication convention
// i*i == j*j == k*k == i*j*k == -1
TEST(QuaternionAlgebraTest, QuaternionMultiplicationOfBases) {
    const Quaternion unit(1.0f, 0.0f, 0.0f, 0.0f);
    const Quaternion i(0.0f, 1.0f, 0.0f, 0.0f);
    const Quaternion j(0.0f, 0.0f, 1.0f, 0.0f);
    const Quaternion k(0.0f, 0.0f, 0.0f, 1.0f);

    Quaternion ii, ij, ik, ji, jj, jk, ki, kj, kk, ijk;
    ii = i * i;
    ij = i * j;
    ik = i * k;
    ji = j * i;
    jj = j * j;
    jk = j * k;
    ki = k * i;
    kj = k * j;
    kk = k * k;
    ijk = i * j * k;

    for (int a = 0; a < 4; ++a) {
        EXPECT_FLOAT_EQ(ii[a], jj[a]);
        EXPECT_FLOAT_EQ(jj[a], kk[a]);
        EXPECT_FLOAT_EQ(kk[a], ijk[a]);
        EXPECT_FLOAT_EQ(ijk[a], -unit[a]);
        EXPECT_FLOAT_EQ(ij[a], k[a]);
        EXPECT_FLOAT_EQ(ii[a], -unit[a]);
        EXPECT_FLOAT_EQ(ik[a], -j[a]);
        EXPECT_FLOAT_EQ(ji[a], -k[a]);
        EXPECT_FLOAT_EQ(jj[a], -unit[a]);
        EXPECT_FLOAT_EQ(jk[a], i[a]);
        EXPECT_FLOAT_EQ(ki[a], j[a]);
        EXPECT_FLOAT_EQ(kj[a], -i[a]);
        EXPECT_FLOAT_EQ(kk[a], -unit[a]);
        EXPECT_FLOAT_EQ(ijk[a], -unit[a]);
    }
}

// Tests that quaternion multiplication is homomorphic with rotation matrix
// multiplication, or C(q0 * q1) = C(q0) * C(q1)
TEST(QuaternionAlgebraTest, QuaternionMultiplicationIsHomomorphism) {
    Quaternion l_quat(0.8365163f, 0.48296291f, 0.22414387f, -0.12940952f);
    Quaternion r_quat(0.9576622f, 0.03378266f, 0.12607862f, 0.25660481f);

    Matrix3f res_mat_0;
    (l_quat * r_quat).rotation_matrix(res_mat_0);

    Matrix3f res_mat_1, l_mat, r_mat;
    l_quat.rotation_matrix(l_mat);
    r_quat.rotation_matrix(r_mat);
    res_mat_1 = l_mat * r_mat;

    EXPECT_NEAR(res_mat_0.a.x, res_mat_1.a.x, 1e-6f);
    EXPECT_NEAR(res_mat_0.a.y, res_mat_1.a.y, 1e-6f);
    EXPECT_NEAR(res_mat_0.a.z, res_mat_1.a.z, 1e-6f);
    EXPECT_NEAR(res_mat_0.b.x, res_mat_1.b.x, 1e-6f);
    EXPECT_NEAR(res_mat_0.b.y, res_mat_1.b.y, 1e-6f);
    EXPECT_NEAR(res_mat_0.b.z, res_mat_1.b.z, 1e-6f);
    EXPECT_NEAR(res_mat_0.c.x, res_mat_1.c.x, 1e-6f);
    EXPECT_NEAR(res_mat_0.c.y, res_mat_1.c.y, 1e-6f);
    EXPECT_NEAR(res_mat_0.c.z, res_mat_1.c.z, 1e-6f);
}

// Tests that applying a rotation by a unit quaternion does nothing
TEST(QuaternionAlgebraTest, QuatenionRotationByUnitQuaternion) {
    Vector3f v(1.0f, 2.0f, 3.0f);
    Quaternion q(1.0f, 0.0f, 0.0f, 0.0f);

    Vector3f res = q * v;

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(res[i], v[i]);
    }
}

// Tests that applying a rotation by a quaternion whose axis is parallel to the vector does nothing
TEST(QuaternionAlgebraTest, QuatenionRotationByParallelQuaternion) {
    Vector3f v(1.0f, 2.0f, 3.0f);
    Quaternion q(0.7302967f, 0.1825742f, 0.3651484f, 0.5477226f);

    Vector3f res = q * v;

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(res[i], v[i]);
    }
}

// Tests that applying a rotation by any quaternion does not change the vector's length
TEST(QuaternionAlgebraTest, QuatenionRotationLengthPreserving) {
    Vector3f v(1.0f, 2.0f, 3.0f);
    Quaternion q(0.8365163f, 0.48296291f, 0.22414387f, -0.12940952f);

    Vector3f res = q * v;

    EXPECT_FLOAT_EQ(res.length(), v.length());
}

// Tests that calling the quaternion rotation operator is equivalent to the formula q * v * q.inverse(), and to
// converting to rotation matrix followed by matrix multiplication
TEST(QuaternionAlgebraTest, QuatenionRotationFormulaEquivalence) {
    Vector3f res_1, res_0, res_2;
    Vector3f v(1.0f, 2.0f, 3.0f);
    Quaternion q(0.8365163f, 0.48296291f, 0.22414387f, -0.12940952f);

    res_0 = q * v;

    Quaternion qv(0.0f, v.x, v.y, v.z);
    Quaternion res_qv = q * qv * q.inverse();
    res_1 = Vector3f(res_qv.q2, res_qv.q3, res_qv.q4);

    Matrix3f q_equiv_mat;
    q.rotation_matrix(q_equiv_mat);
    res_2 = q_equiv_mat * v;

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(res_0[i], res_1[i]);
        EXPECT_FLOAT_EQ(res_0[i], res_2[i]);
    }
}

// Tests that the calling the rotation operator on a inverted quaternion is equivalent to q.inverse() * v * q, and to
// converting to rotation matrix, taking transpose, followed by matrix multiplication
TEST(QuaternionAlgebraTest, QuatenionInverseRotationFormulaEquivalence) {
    Vector3f res_0, res_1, res_2;
    Vector3f v(1.0f, 2.0f, 3.0f);
    Quaternion q(0.8365163f, 0.48296291f, 0.22414387f, -0.12940952f);

    res_0 = q.inverse() * v;

    Quaternion qv(0.0f, v.x, v.y, v.z);
    Quaternion res_qv = q.inverse() * qv * q;
    res_1 = Vector3f(res_qv.q2, res_qv.q3, res_qv.q4);

    Matrix3f q_equiv_mat;
    q.rotation_matrix(q_equiv_mat);
    res_2 = q_equiv_mat.transposed() * v;

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(res_0[i], res_1[i]);
        EXPECT_FLOAT_EQ(res_0[i], res_2[i]);
    }
}

AP_GTEST_MAIN()
int hal = 0; // Needed to ensure test builds, see last line of test_3d_lines.cpp