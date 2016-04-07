/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
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

INSTANTIATE_TEST_CASE_P(InvertibleMatrices,
                        Matrix3fTest,
                        ::testing::ValuesIn(invertible));

INSTANTIATE_TEST_CASE_P(NonInvertibleMatrices,
                        Matrix3fTest,
                        ::testing::ValuesIn(non_invertible));

AP_GTEST_MAIN()
