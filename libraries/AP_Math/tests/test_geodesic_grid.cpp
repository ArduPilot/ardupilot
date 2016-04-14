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
#include <cassert>
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
    /**
     * Array terminated with -1. This doesn't have to be touched if #section
     * isn't negative. If #section is -1, then calling
     * AP_GeodesicGrid::section() with inclusive set as true expects a return
     * value as one of the values in #inclusive_sections.
     */
    int inclusive_sections[7];
};

class GeodesicGridTest : public ::testing::TestWithParam<TestParam> {
protected:
    static AP_GeodesicGrid grid;

    /**
     * Test the functions for triangles indexes.
     *
     * @param p[in] The test parameter.
     */
    void test_triangles_indexes(const TestParam& p) {
        if (p.section >= 0) {
            int expected_triangle = p.section / grid.NUM_SUBTRIANGLES;
            int triangle = grid._triangle_index(p.v, false);
            ASSERT_EQ(expected_triangle, triangle);

            int expected_subtriangle = p.section % grid.NUM_SUBTRIANGLES;
            int subtriangle = grid._subtriangle_index(triangle, p.v, false);
            ASSERT_EQ(expected_subtriangle, subtriangle);
        } else {
            int triangle = grid._triangle_index(p.v, false);
            if (triangle >= 0) {
                int subtriangle = grid._subtriangle_index(triangle, p.v, false);
                ASSERT_EQ(-1, subtriangle) << "triangle is " << triangle;
            }
        }
    }
};

AP_GeodesicGrid GeodesicGridTest::grid;

AP_GTEST_PRINTATBLE_PARAM_MEMBER(TestParam, v);

TEST_P(GeodesicGridTest, Sections)
{
    auto p = GetParam();

    test_triangles_indexes(p);
    EXPECT_EQ(p.section, grid.section(p.v));

    if (p.section < 0) {
        int s = grid.section(p.v, true);
        int i;
        for (i = 0; p.inclusive_sections[i] > 0; i++) {
            assert(i < 7);
            if (s == p.inclusive_sections[i]) {
                break;
            }
        }
        if (p.inclusive_sections[i] < 0) {
            ADD_FAILURE() << "section " << s << " with inclusive=true not found in inclusive_sections";
        }
    }
}

static TestParam icosahedron_vertices[] = {
    {{ M_GOLDEN,  1.0f, 0.0f}, -1, {27, 30, 43, 46, 49, -1}},
    {{ M_GOLDEN, -1.0f, 0.0f}, -1, {19, 23, 25, 41, 78, -1}},
    {{-M_GOLDEN,  1.0f, 0.0f}, -1, { 1, 38, 59, 63, 65, -1}},
    {{-M_GOLDEN, -1.0f, 0.0f}, -1, { 3,  6,  9, 67, 70, -1}},
    {{ 1.0f, 0.0f,  M_GOLDEN}, -1, {42, 45, 53, 75, 79, -1}},
    {{-1.0f, 0.0f,  M_GOLDEN}, -1, {55, 62, 66, 69, 73, -1}},
    {{ 1.0f, 0.0f, -M_GOLDEN}, -1, {15, 22, 26, 29, 33, -1}},
    {{-1.0f, 0.0f, -M_GOLDEN}, -1, { 2,  5, 13, 35, 39, -1}},
    {{0.0f,  M_GOLDEN,  1.0f}, -1, {47, 50, 54, 57, 61, -1}},
    {{0.0f,  M_GOLDEN, -1.0f}, -1, {31, 34, 37, 51, 58, -1}},
    {{0.0f, -M_GOLDEN,  1.0f}, -1, {11, 18, 71, 74, 77, -1}},
    {{0.0f, -M_GOLDEN, -1.0f}, -1, { 7, 10, 14, 17, 21, -1}},
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
    {{M_GOLDEN, -4.0f * M_GOLDEN -1.0f, 1.0f}, -1, {16, 18, -1}},
    /* 2 * m_c + (1 / 3) * m_b + .1 * c for T_13 */
    {{-.2667f, .1667f * M_GOLDEN, 2.2667f * M_GOLDEN + .1667f}, 55},
    /* .25 * m_a + 5 * b + 2 * m_b for T_8 */
    {{-.875f, 6.125f * M_GOLDEN, -1.125f * M_GOLDEN - 6.125f}, 34},
};
INSTANTIATE_TEST_CASE_P(HardcodedVectors,
                        GeodesicGridTest,
                        ::testing::ValuesIn(hardcoded_vectors));

AP_GTEST_MAIN()
