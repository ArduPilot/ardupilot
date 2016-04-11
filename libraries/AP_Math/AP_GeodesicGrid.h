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
#pragma once

#include "AP_Math.h"

/**
 * AP_GeodesicGrid is a class for working on geodesic sections.
 *
 * For quick information regarding geodesic grids, see:
 * https://en.wikipedia.org/wiki/Geodesic_grid
 *
 * The grid is formed by a tessellation of an icosahedron by a factor of 2,
 * i.e., each triangular face of the icosahedron is divided into 4 by splitting
 * each edge into 2 line segments and projecting the vertices to the
 * icosahedron's circumscribed sphere. That will give a total of 80 triangular
 * faces, which are called sections in this context.
 *
 * A section index is given by the icosahedron's triangle it belongs to and by
 * its index in that triangle. Let i in [0,20) be the icosahedron's triangle
 * index and j in [0,4) be the sub-triangle's (which is the section) index
 * inside the greater triangle. Then the section index is given by
 * s = 4 * i + j .
 *
 * The icosahedron's triangles are defined by the tuple (T_0, T_1, ..., T_19),
 * where T_i is the i-th triangle. Each triangle is represented with a tuple of
 * the form (a, b, c), where a, b and c are the triangle vertices in the space.
 *
 * Given the definitions above and the golden ration as g, the triangles must
 * be defined in the following order:
 *
 *     (
 *         ((-g, 1, 0), (-1, 0,-g), (-g,-1, 0)),
 *         ((-1, 0,-g), (-g,-1, 0), ( 0,-g,-1)),
 *         ((-g,-1, 0), ( 0,-g,-1), ( 0,-g, 1)),
 *         ((-1, 0,-g), ( 0,-g,-1), ( 1, 0,-g)),
 *         (( 0,-g,-1), ( 0,-g, 1), ( g,-1, 0)),
 *         (( 0,-g,-1), ( 1, 0,-g), ( g,-1, 0)),
 *         (( g,-1, 0), ( 1, 0,-g), ( g, 1, 0)),
 *         (( 1, 0,-g), ( g, 1, 0), ( 0, g,-1)),
 *         (( 1, 0,-g), ( 0, g,-1), (-1, 0,-g)),
 *         (( 0, g,-1), (-g, 1, 0), (-1, 0,-g)),
 *         ((-g, 1, 0), (-1, 0, g), (-g,-1, 0)),
 *         ((-1, 0, g), (-g,-1, 0), ( 0,-g, 1)),
 *         ((-1, 0, g), ( 0,-g, 1), ( 1, 0, g)),
 *         (( 0,-g, 1), ( 1, 0, g), ( g,-1, 0)),
 *         (( g,-1, 0), ( 1, 0, g), ( g, 1, 0)),
 *         (( 1, 0, g), ( g, 1, 0), ( 0, g, 1)),
 *         (( g, 1, 0), ( 0, g, 1), ( 0, g,-1)),
 *         (( 1, 0, g), ( 0, g, 1), (-1, 0, g)),
 *         (( 0, g, 1), ( 0, g,-1), (-g, 1, 0)),
 *         (( 0, g, 1), (-g, 1, 0), (-1, 0, g))
 *     )
 *
 * Let an icosahedron triangle T be defined as T = (a, b, c). The "middle
 * triangle" M is defined as the triangle formed by the points that bisect the
 * edges of T. M is defined by:
 *
 *     M = (m_a, m_b, m_c) = ((a + b) / 2, (b + c) / 2, (c + a) / 2)
 *
 * Let elements of the tuple (W_0, W_1, W_2, W_3) comprise the sub-triangles of
 * T, so that W_j is the j-th sub-triangle of T. The sub-triangles are defined
 * as the following:
 *
 *    W_0 = M
 *    W_1 = (a, m_a, m_c)
 *    W_2 = (m_a, b, m_b)
 *    W_3 = (m_c, m_b, c)
 */
class AP_GeodesicGrid {
public:
    /*
     * The following concepts are used by the description of this class'
     * members.
     *
     * Vector crossing objects
     * -----------------------
     * We say that a vector v crosses an object in space (point, line, line
     * segment, plane etc) iff the line, being Q the set of points of that
     * object, the vector v crosses it iff there exists a positive scalar alpha
     * such that alpha * v is in Q.
     */

    /**
     * Number of sub-triangles for an icosahedron triangle.
     */
    static const int NUM_SUBTRIANGLES = 4;

    AP_GeodesicGrid();

    /**
     * Find which section is crossed by \p v.
     *
     * @param v[in] The vector to be verified.
     *
     * @param inclusive[in] If true, then if \p v crosses one of the edges of
     * one of the sections, then that section is returned. If \p inclusive is
     * false, then \p v is considered to cross no section. Note that, if \p
     * inclusive is true, then \p v can belong to more than one section and
     * only the first one found is returned. The order in which the triangles
     * are checked is unspecified.  The default value for \p inclusive is
     * false.
     *
     * @return The index of the section. The value -1 is returned if the
     * section isn't found, which might happen when \p inclusive is false.
     */
    int section(const Vector3f& v, const bool inclusive = false) const;

    /**
     * Get the triangle that defines the section at index \p section_index.
     *
     * @param section_index[in] The section index.
     *
     * @param a[out] The triangle's first vertex.
     * @param b[out] The triangle's second vertex.
     * @param c[out] The triangle's third vertex.
     *
     * @return If \p section_index is valid, true is returned and the triangle
     * vertices are assigned to \p a, \p b and \p c, in that order. Otherwise,
     * false is returned and the values of the vertices parameters are left
     * unmodified.
     */
    bool section_triangle(unsigned int section_index,
                          Vector3f& a,
                          Vector3f& b,
                          Vector3f& c) const;

private:
    /**
     * The icosahedron's triangles. The item `_triangles[i]` represents T_i.
     */
    const Vector3f _triangles[20][3];

    /**
     * The inverses of the change-of-basis matrices for the icosahedron
     * triangles.
     *
     * The i-th matrix is the inverse of the change-of-basis matrix from
     * natural basis to the basis formed by T_i's vectors.
     */
    Matrix3f _inverses[20];

    /**
     * The middle triangles. The item `_mid_triangles[i]` represents the middle
     * triangle of T_i.
     */
    Vector3f _mid_triangles[20][3];

    /**
     * The inverses of the change-of-basis matrices for the middle triangles.
     *
     * The i-th matrix is the inverse of the change-of-basis matrix from
     * natural basis to the basis formed by T_i's middle triangle's vectors.
     */
    Matrix3f _mid_inverses[20];

    /**
     * Initialize the vertices of the middle triangles as specified by
     * #_mid_triangles.
     */
    void _init_mid_triangles();

    /**
     * Initialize the matrices in #_inverses and #_mid_inverses.
     */
    void _init_all_inverses();

    /**
     * Find which icosahedron's triangle is crossed by \p v.
     *
     * @param v[in] The vector to be verified.
     *
     * @param inclusive[in] This parameter follow the same rules defined in
     * #section() const.
     *
     * @return The index of the triangle. The value -1 is returned if the
     * triangle isn't found, which might happen when \p inclusive is false.
     */
    int _triangle_index(const Vector3f& v, const bool inclusive) const;

    /**
     * Find which sub-triangle of the icosahedron's triangle pointed by \p
     * triangle_index is crossed by \p v.
     *
     * The vector \p v must belong to the super-section formed by the triangle
     * pointed by \p triangle_index, otherwise the result is undefined.
     *
     * @param triangle_index[in] The icosahedron's triangle index, it must be in
     * the interval [0,20). Passing invalid values is undefined behavior.
     *
     * @param v[in] The vector to be verified.
     *
     * @param inclusive[in] This parameter follow the same rules defined in
     * #section() const.
     *
     * @return The index of the sub-triangle. The value -1 is returned if the
     * triangle isn't found, which might happen when \p inclusive is false.
     */
    int _subtriangle_index(const unsigned int triangle_index,
                           const Vector3f& v,
                           const bool inclusive) const;
};
