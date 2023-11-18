/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "math_test.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// given we are in the Math library, you're epected to know what
// you're doing when directly comparing floats:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#define AP_EXPECT_IDENTITY_MATRIX(m_) {\
    EXPECT_NEAR(1.0f, m_.a.x, 1.0e-6); \
    EXPECT_NEAR(0.0f, m_.a.y, 1.0e-6); \
    EXPECT_NEAR(0.0f, m_.a.z, 1.0e-6); \
    EXPECT_NEAR(0.0f, m_.b.x, 1.0e-6); \
    EXPECT_NEAR(1.0f, m_.b.y, 1.0e-6); \
    EXPECT_NEAR(0.0f, m_.b.z, 1.0e-6); \
    EXPECT_NEAR(0.0f, m_.c.x, 1.0e-6); \
    EXPECT_NEAR(0.0f, m_.c.y, 1.0e-6); \
    EXPECT_NEAR(1.0f, m_.c.z, 1.0e-6); \
}

class TestParam {
public:
    /**
     * The matrix for this param.
     */
    Matrix3f m;
    /**
     * The expected determinant for #m.
     */
    float det;
};

AP_GTEST_PRINTATBLE_PARAM_MEMBER(TestParam, m);



class Matrix3fTest : public ::testing::TestWithParam<TestParam> {};

//Testing for Skew
class SkewSymmetricTestParam {
public:
    Vector3f v; // Input vector
    Matrix3f expected; // Expected output matrix
};


// Define a new test class
class SkewSymmetricTest : public ::testing::TestWithParam<SkewSymmetricTestParam> {};

//Testing from angular velocity 
class AngularVelocityTestParam {
public:
    Vector3f M; // Input vector
    Matrix3f expected; // Expected output matrix
};

class AngularVelocityTest : public ::testing::TestWithParam<AngularVelocityTestParam> {};


static TestParam invertible[] = {
    {
        .m = {
            {1.0f,  2.0f,  3.0f},
            {4.0f,  6.0f,  2.0f},
            {9.0f, 18.0f, 27.0f}
        },
        .det = 0.0f,
    },
};

static TestParam non_invertible[] = {
    {
        .m = {
            { 6.0f,  2.0f,  20.0f},
            { 1.0f, -9.0f,   4.0f},
            {-4.0f,  7.0f, -27.0f}
        },
        .det = 732.0f,
    },
    {
        .m = {
            {-6.0f, -2.0f, -20.0f},
            {-1.0f,  9.0f,  -4.0f},
            { 4.0f, -7.0f,  27.0f}
        },
        .det = -732.0f,
    },
};

// Define new test parameters
static SkewSymmetricTestParam skew_symmetric_params[] = {
    {
        .v = {1.0f, 2.0f, 3.0f},
        .expected = {
            {0, -3, 2},
            {3, 0, -1},
            {-2, 1, 0}
        }
    },
    // ... add more test parameters as needed ...
};

//Define test parameters for angular velocity
static AngularVelocityTestParam angular_velocity_params[] = {
    {
        .M = {1.0f, 1.0f, 1.0f},
        .expected = {
        {0.22629564f, -0.18300792f, 0.95671228f},
        {0.95671228f, 0.22629564f, -0.18300792f},
        {-0.18300792f, 0.95671228f, 0.22629564f}
        }

        
    },
    // ... add more test parameters as needed ...
};

TEST_P(Matrix3fTest, Determinants)
{
    auto param = GetParam();
    EXPECT_FLOAT_EQ(param.det, param.m.det());
}

TEST_P(Matrix3fTest, Inverses)
{
    auto param = GetParam();
    Matrix3f inv;
    bool success = param.m.inverse(inv);

    if (param.det == 0.0f) {
        EXPECT_FALSE(success);
    } else {
        ASSERT_TRUE(success);
        auto identity = inv * param.m;
        AP_EXPECT_IDENTITY_MATRIX(identity);
    }
}

//skew
// Define a new test function
TEST_P(SkewSymmetricTest, SkewSymmetric)
{
    auto param = GetParam();
    Matrix3f result = Matrix3f::skew_symmetric(param.v);
    //test if equal
    EXPECT_FLOAT_EQ(result.a.x, param.expected.a.x);
    EXPECT_FLOAT_EQ(result.a.y, param.expected.a.y);
    EXPECT_FLOAT_EQ(result.a.z, param.expected.a.z);
    EXPECT_FLOAT_EQ(result.b.x, param.expected.b.x);
    EXPECT_FLOAT_EQ(result.b.y, param.expected.b.y);
    EXPECT_FLOAT_EQ(result.b.z, param.expected.b.z);
    EXPECT_FLOAT_EQ(result.c.x, param.expected.c.x);
    EXPECT_FLOAT_EQ(result.c.y, param.expected.c.y);
    EXPECT_FLOAT_EQ(result.c.z, param.expected.c.z);
    //test skew_to_vector 
    Vector3f result_vector = Matrix3f::skew_to_vector(result);
    EXPECT_FLOAT_EQ(result_vector.x, param.v.x);
    EXPECT_FLOAT_EQ(result_vector.y, param.v.y);
    EXPECT_FLOAT_EQ(result_vector.z, param.v.z);
}

//angular velocity
// Define a new test function
TEST_P(AngularVelocityTest, AngularVelocity)
{
    auto param = GetParam();
    Matrix3f result = Matrix3f::from_angular_velocity(param.M);
    //test if equal
    EXPECT_FLOAT_EQ(result.a.x, param.expected.a.x);
    EXPECT_FLOAT_EQ(result.a.y, param.expected.a.y);
    EXPECT_FLOAT_EQ(result.a.z, param.expected.a.z);
    EXPECT_FLOAT_EQ(result.b.x, param.expected.b.x);
    EXPECT_FLOAT_EQ(result.b.y, param.expected.b.y);
    EXPECT_FLOAT_EQ(result.b.z, param.expected.b.z);
    EXPECT_FLOAT_EQ(result.c.x, param.expected.c.x);
    EXPECT_FLOAT_EQ(result.c.y, param.expected.c.y);
    EXPECT_FLOAT_EQ(result.c.z, param.expected.c.z);
    
}

INSTANTIATE_TEST_CASE_P(InvertibleMatrices,
                        Matrix3fTest,
                        ::testing::ValuesIn(invertible));

INSTANTIATE_TEST_CASE_P(NonInvertibleMatrices,
                        Matrix3fTest,
                        ::testing::ValuesIn(non_invertible));

// Instantiate new test cases
INSTANTIATE_TEST_CASE_P(SkewSymmetricTests,
                        SkewSymmetricTest,
                        ::testing::ValuesIn(skew_symmetric_params));

// Instantiate new test cases
//Angular velocity
INSTANTIATE_TEST_CASE_P(AngularVelocityTests,
                        AngularVelocityTest,
                        ::testing::ValuesIn(angular_velocity_params));

AP_GTEST_MAIN()

#pragma GCC diagnostic pop
