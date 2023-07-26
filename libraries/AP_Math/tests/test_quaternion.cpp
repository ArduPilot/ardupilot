#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Tests that quaternion multiplication obeys Hamilton's quaternion multiplication convention
// i*i == j*j == k*k == i*j*k == -1
TEST(QuaternionTest, QuaternionMultiplicationOfBases) {
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

// Tests that the quaternion to rotation matrix conversion formula is correctly derived from the Hamilton's quaternion
// multiplication convention. This specific example is taken from "Why and How to Avoid the Flipped Quaternion
// Multiplication" (https://arxiv.org/pdf/1801.07478.pdf)
TEST(QuaternionTest, QuaternionToRotationMatrix) {
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

// Tests that quaternion multiplication is homomorphic with rotation matrix
// multiplication, or C(q0 * q1) = C(q0) * C(q1)
TEST(QuaternionTest, QuaternionMultiplicationIsHomomorphism) {
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
TEST(QuaternionTest, QuatenionRotationByUnitQuaternion) {
    Vector3f v(1.0f, 2.0f, 3.0f);
    Quaternion q(1.0f, 0.0f, 0.0f, 0.0f);

    Vector3f res = q * v;

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(res[i], v[i]);
    }
}

// Tests that applying a rotation by a quaternion whose axis is parallel to the vector does nothing
TEST(QuaternionTest, QuatenionRotationByParallelQuaternion) {
    Vector3f v(1.0f, 2.0f, 3.0f);
    Quaternion q(0.730296743340221, 0.182574185835055, 0.365148371670111, 0.547722557505166f);

    Vector3f res = q * v;

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(res[i], v[i]);
    }
}

// Tests that applying a rotation by a unit quaternion does not change the vector's length
TEST(QuaternionTest, QuatenionRotationLengthPreserving) {
    Vector3f v(1.0f, 2.0f, 3.0f);
    Quaternion q(0.8365163f, 0.48296291f, 0.22414387f, -0.12940952f);

    Vector3f res = q * v;

    EXPECT_FLOAT_EQ(res.length(), v.length());
}

// Tests that calling the quaternion rotation operator is equivalent to the formula q * v * q.inverse(), and to
// converting to rotation matrix followed by matrix multiplication
TEST(QuaternionTest, QuatenionRotationFormulaEquivalence) {
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
TEST(QuaternionTest, QuatenionInverseRotationFormulaEquivalence) {
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

// Test zero()
TEST(QuaternionTest, Quaternion_zero)
{
    Quaternion q {0.0, 0.0, 0.0, 0.0};
    q.zero();
    EXPECT_TRUE(q.is_zero());

    q = Quaternion{0.8365163, 0.48296291, 0.22414387, -0.12940952};
    q.zero();
    EXPECT_TRUE(q.is_zero());

    // unit length
    q = Quaternion{1.0, 0.0, 0.0, 0.0};
    q.zero();
    EXPECT_TRUE(q.is_zero());
}

// Tests is_zero()
TEST(QuaternionTest, Quaternion_is_zero)
{
    Quaternion q {0.0, 0.0, 0.0, 0.0};
    EXPECT_TRUE(q.is_zero());

    q = Quaternion{0.8365163, 0.48296291, 0.22414387, -0.12940952};
    EXPECT_FALSE(q.is_zero());

    q = Quaternion{0.9, 0.0, 0.0, 0.0};
    EXPECT_FALSE(q.is_zero());
}

// Tests is_unit_length()
TEST(QuaternionTest, Quaternion_is_unit_length)
{
    // zero length
    Quaternion q {0.0, 0.0, 0.0, 0.0};
    EXPECT_FALSE(q.is_unit_length());

    // Length == 1.0 - 0.0009, slightly within the tolerance
    q = Quaternion{0.8361398, 0.4827455, 0.2240430, -0.1293512};
    EXPECT_TRUE(q.is_unit_length());

    // unit length
    q  = Quaternion{0.8365163, 0.48296291, 0.22414387, -0.12940952};
    EXPECT_TRUE(q.is_unit_length());

    // unit length
    q = Quaternion{1.0, 0.0, 0.0, 0.0};
    EXPECT_TRUE(q.is_unit_length());

    // Length == 1.0 + 0.0009, slightly within the tolerance
    q = Quaternion{0.8368926, 0.4831802, 0.2242447, -0.1294677};
    EXPECT_TRUE(q.is_unit_length());

    // Length == 1.2
    q = Quaternion{1.00382, 0.579555, 0.268973, -0.155291};
    EXPECT_FALSE(q.is_unit_length());
}

// Tests length_squared()
TEST(QuaternionTest, Quaternion_length_squared)
{
    // zero length
    Quaternion q {0.0, 0.0, 0.0, 0.0};
    EXPECT_FLOAT_EQ(q.length_squared(), 0.0);

    // Length == 1.0 - 0.0009, slightly within the tolerance
    q = Quaternion{0.8361398, 0.4827455, 0.2240430, -0.1293512};
    EXPECT_FLOAT_EQ(q.length_squared(), 1.0 - 0.0009);

    // unit length
    q  = Quaternion{0.8365163, 0.48296291, 0.22414387, -0.12940952};
    EXPECT_FLOAT_EQ(q.length_squared(), 1.0);

    // unit length
    q = Quaternion{1.0, 0.0, 0.0, 0.0};
    EXPECT_FLOAT_EQ(q.length_squared(), 1.0);

    // Length == 1.0 + 0.0009, slightly within the tolerance
    q = Quaternion{0.8368926, 0.4831802, 0.2242447, -0.1294677};
    EXPECT_FLOAT_EQ(q.length_squared(), 1.0 + 0.0009);

    // Length == 1.2
    q = Quaternion{1.00382, 0.579555, 0.268973, -0.155291};
    EXPECT_FLOAT_EQ(q.length_squared(), 1.44);
}

AP_GTEST_MAIN()