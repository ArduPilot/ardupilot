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

/*
 * This comment section explains the basic idea behind the implementation.
 *
 * Vectors difference notation
 * ===========================
 * Let v and w be vectors. For readability purposes, unless explicitly
 * otherwise noted, the notation vw will be used to represent w - v.
 *
 * Relationship between a vector and a triangle in 3d space
 * ========================================================
 * Vector in the area of a triangle
 * --------------------------------
 * Let T = (a, b, c) be a triangle, where a, b and c are also vectors and
 * linearly independent. A vector inside that triangle can be written as one of
 * its vertices plus the sum of the positively scaled vectors from that vertex
 * to the other ones. Taking a as the first vertex, a vector p in the area
 * formed by T can be written as:
 *
 *     p = a + w_ab * ab + w_ac * ac
 *
 * It's fairly easy to see that if p is in the area formed by T, then w_ab >= 0
 * and w_ac >= 0. That vector p can also be written as:
 *
 *     p = b + w_ba * ba + w_bc * bc
 *
 * It's easy to check that the triangle formed by (a + w_ab * ab, b + w_ba *
 * ba, p) is similar to T and, with the correct algebraic manipulations, we can
 * come to the conclusion that:
 *
 *     w_ba = 1 - w_ab - w_ac
 *
 * Since we know that w_ba >= 0, then w_ab + w_ac <= 1. Thus:
 *
 *     ----------------------------------------------------------
 *     | p = a + w_ab * ab + w_ac * ac is in the area of T iff: |
 *     | w_ab >= 0 and w_ac >= 0 and w_ab + w_ac <= 1           |
 *     ----------------------------------------------------------
 *
 * Proving backwards shouldn't be difficult.
 *
 * Vector p can also be written as:
 *
 *     p = (1 - w_ab - w_ba) * a + w_ab * b + w_ba * c
 *
 *
 * Vector that crosses a triangle
 * ------------------------------
 * Let T be the same triangle discussed above and let v be a vector such that:
 *
 *     v = x * a + y * b + z * c
 *     where x >= 0, y >= 0, z >= 0, and x + y + z > 0.
 *
 * It's geometrically easy to see that v crosses the triangle T. But that can
 * also be verified analytically.
 *
 * The vector v crosses the triangle T iff there's a positive alpha such that
 * alpha * v is in the area formed by T, so we need to prove that such value
 * exists. To find alpha, we solve the equation alpha * v = p, which will lead
 * us to the system, for the variables alpha, w_ab and w_ac:
 *
 *    alpha * x = 1 - w_ab - w_ac
 *    alpha * y = w_ab
 *    alpha * z = w_ac,
 *    where w_ab >= 0 and w_ac >= 0 and w_ab + w_ac <= 1
 *
 * That will lead to alpha = 1 / (x + y + z), w_ab = y / (x + y + b) and
 * w_ac = z / (x + y + z) and the following holds:
 *  - alpha does exist because x + y + z > 0.
 *  - w_ab >= 0 and w_ac >= 0 because y >= 0 and z >= 0 and x + y + z > 0.
 *  - 0 <= 1 - w_ab - w_ac <= 1 because 0 <= (y + z) / (x + y + z) <= 1.
 *
 * Thus:
 *
 *     ----------------------------------------------------------
 *     | v = x * a + y * b + z * c crosses T = (a, b, c), where |
 *     | a, b and c are linearly independent, iff:              |
 *     | x >= 0, y >= 0, z >= 0 and x + y + z > 0               |
 *     ----------------------------------------------------------
 *
 * Moreover:
 *  - if one of the coefficients is zero, then v crosses the edge formed by the
 *  vertices multiplied by the non-zero coefficients.
 *  - if two of the coefficients are zero, then v crosses the vertex multiplied
 *  by the non-zero coefficient.
 */

#include <assert.h>

#include "AP_GeodesicGrid.h"

AP_GeodesicGrid::AP_GeodesicGrid()
    : _triangles{
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
      }
    , _neighbor_umbrellas{
          {{ 9,  8,  7, 12, 14}},
          {{ 1,  2,  4,  5,  3}},
          {{16, 15, 13, 18, 17}},
          {{19, 18, 17,  2,  4}},
          {{11, 12, 14, 15, 13}},
          {{ 6,  5,  3,  8,  7}},
      }
{
    _init_opposite_triangles();
    _init_mid_triangles();
    _init_all_inverses();
}

int AP_GeodesicGrid::section(const Vector3f& v,
                             const bool inclusive) const
{
    int i = _triangle_index(v, inclusive);
    if (i < 0) {
        return -1;
    }

    int j = _subtriangle_index(i, v, inclusive);
    if (j < 0) {
        return -1;
    }

    return 4 * i + j;
}

bool AP_GeodesicGrid::section_triangle(unsigned int section_index,
                                       Vector3f& a,
                                       Vector3f& b,
                                       Vector3f& c) const
{
    if (section_index >= 20 * NUM_SUBTRIANGLES) {
        return false;
    }

    unsigned int i = section_index / NUM_SUBTRIANGLES;
    unsigned int j = section_index % NUM_SUBTRIANGLES;
    auto& t = _triangles[i];
    auto& mt = _mid_triangles[i];

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

void AP_GeodesicGrid::_init_opposite_triangles()
{
    for (int i = 0, j = 10; i < 10; i++, j++) {
        _triangles[j][0] = -_triangles[i][0];
        _triangles[j][1] = -_triangles[i][1];
        _triangles[j][2] = -_triangles[i][2];
    }
}

void AP_GeodesicGrid::_init_mid_triangles()
{
    for (int i = 0; i < 20; i++) {
        _mid_triangles[i][0] = (_triangles[i][0] + _triangles[i][1]) / 2;
        _mid_triangles[i][1] = (_triangles[i][1] + _triangles[i][2]) / 2;
        _mid_triangles[i][2] = (_triangles[i][2] + _triangles[i][0]) / 2;
    }
}

void AP_GeodesicGrid::_init_all_inverses()
{
    for (int i = 0; i < 20; i++) {
        auto& t = _triangles[i];
        _inverses[i]({t[0].x, t[1].x, t[2].x},
                     {t[0].y, t[1].y, t[2].y},
                     {t[0].z, t[1].z, t[2].z});
        _inverses[i].invert();

        auto& mt = _mid_triangles[i];
        _mid_inverses[i]({mt[0].x, mt[1].x, mt[2].x},
                         {mt[0].y, mt[1].y, mt[2].y},
                         {mt[0].z, mt[1].z, mt[2].z});
        _mid_inverses[i].invert();
    }
}

int AP_GeodesicGrid::_from_neighbor_umbrella(int idx,
                                             const Vector3f& v,
                                             const Vector3f& u,
                                             bool inclusive) const
{
    const struct neighbor_umbrella& umbrella = _neighbor_umbrellas[idx];
    for (int i = 0; i < 5; i++) {
        auto w = _inverses[umbrella.components[i]] * v;

        if (!is_zero(w.x) && w.x < 0) {
            continue;
        }
        if (!is_zero(w.y) && w.y < 0) {
            continue;
        }
        if (!is_zero(w.z) && w.z < 0) {
            continue;
        }

        if ((is_zero(w.x) || is_zero(w.y) || is_zero(w.z)) && !inclusive) {
            return -1;
        }

        return umbrella.components[i];
    }

    return -1;
}

int AP_GeodesicGrid::_triangle_index(const Vector3f& v,
                                     const bool inclusive) const
{
    /* w holds the coordinates of v with respect to the basis comprised by the
     * vectors of _triangles[0] */
    auto w = _inverses[0] * v;
    int zero_count = 0;
    int balance = 0;
    int umbrella = -1;

    if (is_zero(w.x)) {
        zero_count++;
    } else if (w.x > 0) {
        balance++;
    } else {
        balance--;
    }

    if (is_zero(w.y)) {
        zero_count++;
    } else if (w.y > 0) {
        balance++;
    } else {
        balance--;
    }

    if (is_zero(w.z)) {
        zero_count++;
    } else if (w.z > 0) {
        balance++;
    } else {
        balance--;
    }

    switch (balance) {
    case 3:
        /* All coefficients are positive, thus return the first triangle. */
        return 0;
    case -3:
        /* All coefficients are negative, which means that the coefficients for
         * -w are positive, thus return the first triangle's opposite. */
        return 10;
    case 2:
        /* Two coefficients are positive and one is zero, thus v crosses one of
         * the edges of the first triangle. */
        return inclusive ? 0 : -1;
    case -2:
        /* Analogous to the previous case, but for the opposite of the first
         * triangle. */
        return inclusive ? 10 : -1;
    case 1:
        /* There are two possible cases when balance is 1:
         *
         * 1) Two coefficients are zero, which means v crosses one of the
         * vertices of the first triangle.
         *
         * 2) Two coefficients are positive and one is negative. Let a and b be
         * vertices with positive coefficients and c the one with the negative
         * coefficient. That means that v crosses the triangle formed by a, b
         * and -c. The vector -c happens to be the 3-th vertex, with respect to
         * (a, b), of the first triangle's neighbor umbrella with respect to a
         * and b. Thus, v crosses one of the components of that umbrella. */
        if (zero_count == 2) {
            return inclusive ? 0 : -1;
        }

        if (!is_zero(w.x) && w.x < 0) {
            umbrella = 1;
        } else if (!is_zero(w.y) && w.y < 0) {
            umbrella = 2;
        } else {
            umbrella = 0;
        }

        break;
    case -1:
        /* Analogous to the previous case, but for the opposite of the first
         * triangle. */
        if (zero_count == 2) {
            return inclusive ? 10 : -1;
        }

        if (!is_zero(w.x) && w.x > 0) {
            umbrella = 4;
        } else if (!is_zero(w.y) && w.y > 0) {
            umbrella = 5;
        } else {
            umbrella = 3;
        }
        w = -w;

        break;
    case 0:
        /* There are two possible cases when balance is 1:
         *
         * 1) The vector v is the null vector, which doesn't cross any section.
         *
         * 2) One coefficient is zero, another is positive and yet another is
         * negative. Let a, b and c be the respective vertices for those
         * coefficients, then the statements in case (2) for when balance is 1
         * are also valid here.
         */
        if (zero_count == 3) {
            return -1;
        }

        if (!is_zero(w.x) && w.x < 0) {
            umbrella = 1;
        } else if (!is_zero(w.y) && w.y < 0) {
            umbrella = 2;
        } else {
            umbrella = 0;
        }

        break;
    }

    assert(umbrella >= 0);

    switch (umbrella % 3) {
    case 0:
        w.z = -w.z;
        break;
    case 1:
        w(w.y, w.z, -w.x);
        break;
    case 2:
        w(w.z, w.x, -w.y);
        break;
    }

    return _from_neighbor_umbrella(umbrella, v, w, inclusive);
}

int AP_GeodesicGrid::_subtriangle_index(const unsigned int triangle_index,
                                        const Vector3f& v,
                                        const bool inclusive) const
{
    /* w holds the coordinates of v with respect to the basis comprised by the
     * vectors of _mid_triangles[triangle_index] */
    auto w = _mid_inverses[triangle_index] * v;

    if ((is_zero(w.x) || is_zero(w.y) || is_zero(w.z)) && !inclusive) {
        return -1;
    }

    /* At this point, we know that v crosses the icosahedron triangle pointed
     * by triangle_index. Thus, we can geometrically see that if v doesn't
     * cross its middle triangle, then one of the coefficients will be negative
     * and the other ones positive. Let a and b be the non-negative
     * coefficients and c the negative one. In that case, v will cross the
     * triangle with vertices (a, b, -c). Since we know that v crosses the
     * icosahedron triangle and the only sub-triangle that contains the set of
     * points (seen as vectors) that cross the triangle (a, b, -c) is the
     * middle triangle's neighbor with respect to a and b, then that
     * sub-triangle is the one crossed by v. */
    if (!is_zero(w.x) && w.x < 0) {
        return 3;
    }
    if (!is_zero(w.y) && w.y < 0) {
        return 1;
    }
    if (!is_zero(w.z) && w.z < 0) {
        return 2;
    }

    /* If x >= 0 and y >= 0 and z >= 0, then v crosses the middle triangle. */
    return 0;
}
