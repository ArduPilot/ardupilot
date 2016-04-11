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
#include <vector>

#include "math_test.h"
#include <AP_Math/AP_GeodesicGrid.h>

class TestParam {
public:
    /**
     * Vector to be tested.
     */
    Vector3f v;
    /**
     * Expected section if when AP_GeodesicGrid::section() is called with
     * inclusive set as false.
     */
    int section;
};

class GeodesicGridTest : public ::testing::TestWithParam<TestParam> {
protected:
    static AP_GeodesicGrid grid;
};

AP_GeodesicGrid GeodesicGridTest::grid;

AP_GTEST_PRINTATBLE_PARAM_MEMBER(TestParam, v);

TEST_P(GeodesicGridTest, Sections)
{
    auto p = GetParam();
    EXPECT_EQ(p.section, grid.section(p.v));
}

static TestParam icosahedron_vertices[] = {
    {{ M_GOLDEN,  1.0f, 0.0f}, -1},
    {{ M_GOLDEN, -1.0f, 0.0f}, -1},
    {{-M_GOLDEN,  1.0f, 0.0f}, -1},
    {{-M_GOLDEN, -1.0f, 0.0f}, -1},
    {{ 1.0f, 0.0f,  M_GOLDEN}, -1},
    {{-1.0f, 0.0f,  M_GOLDEN}, -1},
    {{ 1.0f, 0.0f, -M_GOLDEN}, -1},
    {{-1.0f, 0.0f, -M_GOLDEN}, -1},
    {{0.0f,  M_GOLDEN,  1.0f}, -1},
    {{0.0f,  M_GOLDEN, -1.0f}, -1},
    {{0.0f, -M_GOLDEN,  1.0f}, -1},
    {{0.0f, -M_GOLDEN, -1.0f}, -1},
};
INSTANTIATE_TEST_CASE_P(IcosahedronVertices,
                        GeodesicGridTest,
                        ::testing::ValuesIn(icosahedron_vertices));

/* Generate vectors that pass through the centroid of each section's triangle. */
static std::vector<TestParam> centroid_vectors = []()
{
    std::vector<TestParam> params;
    AP_GeodesicGrid grid;
    for (int i = 0; i < 20 * AP_GeodesicGrid::NUM_SUBTRIANGLES; i++) {
        Vector3f a, b, c;
        TestParam p;
        grid.section_triangle(i, a, b, c);
        p.section = i;
        p.v = (a + b + c) / 3;
        params.push_back(p);
    }
    return params;
}();
INSTANTIATE_TEST_CASE_P(MidVectors,
                        GeodesicGridTest,
                        ::testing::ValuesIn(centroid_vectors));

/* Other hardcoded vectors, so we don't rely just on the centroid vectors
 * (which are dependent on how the triangles are *defined by the
 * implementation*)
 *
 * See AP_GeodesicGrid.h for the notation on the comments below.
 */
static TestParam hardcoded_vectors[] = {
    /* a + 2 * m_a + .5 * m_c for T_4 */
    {{.25f * M_GOLDEN, -.25f * (13.0f * M_GOLDEN + 1.0f), - 1.25f}, 17},
    /* 3 * m_a + 2 * m_b 0 * m_c for T_4 */
    {{M_GOLDEN, -4.0f * M_GOLDEN -1.0f, 1.0f}, -1},
    /* 2 * m_c + (1 / 3) * m_b + .1 * c for T_17 */
    {{-.2667f, .1667f * M_GOLDEN, 2.2667f * M_GOLDEN + .1667f}, 71},
    /* .25 * m_a + 5 * b + 2 * m_b for T_8 */
    {{-.875f, 6.125f * M_GOLDEN, -1.125f * M_GOLDEN - 6.125f}, 34},
};
INSTANTIATE_TEST_CASE_P(HardcodedVectors,
                        GeodesicGridTest,
                        ::testing::ValuesIn(hardcoded_vectors));

AP_GTEST_MAIN()
