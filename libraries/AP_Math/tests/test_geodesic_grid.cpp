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

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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
    /**
     * Test the functions for triangles indexes.
     *
     * @param p[in] The test parameter.
     */
    void test_triangles_indexes(const TestParam &p) {
        if (p.section >= 0) {
            int expected_triangle =
                    p.section / AP_GeodesicGrid::NUM_SUBTRIANGLES;
            int triangle = AP_GeodesicGrid::_triangle_index(p.v, false);
            ASSERT_EQ(expected_triangle, triangle);

            int expected_subtriangle =
                    p.section % AP_GeodesicGrid::NUM_SUBTRIANGLES;
            int subtriangle =
                    AP_GeodesicGrid::_subtriangle_index(triangle, p.v, false);
            ASSERT_EQ(expected_subtriangle, subtriangle);
        } else {
            int triangle = AP_GeodesicGrid::_triangle_index(p.v, false);
            if (triangle >= 0) {
                int subtriangle = AP_GeodesicGrid::_subtriangle_index(triangle,
                                                                      p.v,
                                                                      false);
                ASSERT_EQ(-1, subtriangle) << "triangle is " << triangle;
            }
        }
    }
};

static const Vector3f triangles[20][3] = {
    {{-M_GOLDEN, 1, 0}, {-1, 0,-M_GOLDEN}, {-M_GOLDEN,-1, 0}},
    {{-1, 0,-M_GOLDEN}, {-M_GOLDEN,-1, 0}, { 0,-M_GOLDEN,-1}},
    {{-M_GOLDEN,-1, 0}, { 0,-M_GOLDEN,-1}, { 0,-M_GOLDEN, 1}},
    {{-1, 0,-M_GOLDEN}, { 0,-M_GOLDEN,-1}, { 1, 0,-M_GOLDEN}},
    {{ 0,-M_GOLDEN,-1}, { 0,-M_GOLDEN, 1}, { M_GOLDEN,-1, 0}},
    {{ 0,-M_GOLDEN,-1}, { 1, 0,-M_GOLDEN}, { M_GOLDEN,-1, 0}},
    {{ M_GOLDEN,-1, 0}, { 1, 0,-M_GOLDEN}, { M_GOLDEN, 1, 0}},
    {{ 1, 0,-M_GOLDEN}, { M_GOLDEN, 1, 0}, { 0, M_GOLDEN,-1}},
    {{ 1, 0,-M_GOLDEN}, { 0, M_GOLDEN,-1}, {-1, 0,-M_GOLDEN}},
    {{ 0, M_GOLDEN,-1}, {-M_GOLDEN, 1, 0}, {-1, 0,-M_GOLDEN}},

    {{ M_GOLDEN,-1, 0}, { 1, 0, M_GOLDEN}, { M_GOLDEN, 1, 0}},
    {{ 1, 0, M_GOLDEN}, { M_GOLDEN, 1, 0}, { 0, M_GOLDEN, 1}},
    {{ M_GOLDEN, 1, 0}, { 0, M_GOLDEN, 1}, { 0, M_GOLDEN,-1}},
    {{ 1, 0, M_GOLDEN}, { 0, M_GOLDEN, 1}, {-1, 0, M_GOLDEN}},
    {{ 0, M_GOLDEN, 1}, { 0, M_GOLDEN,-1}, {-M_GOLDEN, 1, 0}},
    {{ 0, M_GOLDEN, 1}, {-1, 0, M_GOLDEN}, {-M_GOLDEN, 1, 0}},
    {{-M_GOLDEN, 1, 0}, {-1, 0, M_GOLDEN}, {-M_GOLDEN,-1, 0}},
    {{-1, 0, M_GOLDEN}, {-M_GOLDEN,-1, 0}, { 0,-M_GOLDEN, 1}},
    {{-1, 0, M_GOLDEN}, { 0,-M_GOLDEN, 1}, { 1, 0, M_GOLDEN}},
    {{ 0,-M_GOLDEN, 1}, { M_GOLDEN,-1, 0}, { 1, 0, M_GOLDEN}},
};

static bool section_triangle(unsigned int section_index,
                             Vector3f &a,
                             Vector3f &b,
                             Vector3f &c) {
    if (section_index >= 80) {
        return false;  // LCOV_EXCL_LINE
    }

    unsigned int i = section_index / 4;
    unsigned int j = section_index % 4;
    auto &t = triangles[i];
    Vector3f mt[3]{(t[0] + t[1]) / 2, (t[1] + t[2]) / 2, (t[2] + t[0]) / 2};

    switch (j) {
    case 0:
        a = mt[0];
        b = mt[1];
        c = mt[2];
        break;
    case 1:
        a = t[0];
        b = mt[0];
        c = mt[2];
        break;
    case 2:
        a = mt[0];
        b = t[1];
        c = mt[1];
        break;
    case 3:
        a = mt[2];
        b = mt[1];
        c = t[2];
        break;
    }

    return true;
}

AP_GTEST_PRINTATBLE_PARAM_MEMBER(TestParam, v);

TEST_P(GeodesicGridTest, Sections)
{
    auto p = GetParam();

    test_triangles_indexes(p);
    EXPECT_EQ(p.section, AP_GeodesicGrid::section(p.v));

    if (p.section < 0) {
        int s = AP_GeodesicGrid::section(p.v, true);
        int i;
        for (i = 0; p.inclusive_sections[i] > 0; i++) {
            assert(i < 7);
            if (s == p.inclusive_sections[i]) {
                break;
            }
        }
        if (p.inclusive_sections[i] < 0) {
            ADD_FAILURE() << "section " << s << " with inclusive=true not found in inclusive_sections";  // LCOV_EXCL_LINE
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

/* Generate vectors for each triangle */
static std::vector<TestParam> general_vectors = []()
{
    std::vector<TestParam> params;
    for (int i = 0; i < 20 * AP_GeodesicGrid::NUM_SUBTRIANGLES; i++) {
        Vector3f a, b, c;
        TestParam p;
        section_triangle(i, a, b, c);
        p.section = i;

        /* Vector that crosses the centroid */
        p.v = a + b + c;
        params.push_back(p);

        /* Vectors that cross the triangle close to the edges */
        p.v = a + b + c * 0.001f;
        params.push_back(p);
        p.v = a + b * 0.001f + c;
        params.push_back(p);
        p.v = a * 0.001f + b + c;
        params.push_back(p);

        /* Vectors that cross the triangle close to the vertices */
        p.v = a + b * 0.001 + c * 0.001f;
        params.push_back(p);
        p.v = a * 0.001f + b + c * 0.001f;
        params.push_back(p);
        p.v = a * 0.001f + b * 0.001f + c;
        params.push_back(p);
    }
    return params;
}();
INSTANTIATE_TEST_CASE_P(GeneralVectors,
                        GeodesicGridTest,
                        ::testing::ValuesIn(general_vectors));

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
