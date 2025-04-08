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

INSTANTIATE_TEST_CASE_P(InvertibleMatrices,
                        Matrix3fTest,
                        ::testing::ValuesIn(invertible));

INSTANTIATE_TEST_CASE_P(NonInvertibleMatrices,
                        Matrix3fTest,
                        ::testing::ValuesIn(non_invertible));

AP_GTEST_MAIN()

#pragma GCC diagnostic pop
