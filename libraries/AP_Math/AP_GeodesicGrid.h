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
 *         -T_0,
 *         -T_1,
 *         -T_2,
 *         -T_3,
 *         -T_4,
 *         -T_5,
 *         -T_6,
 *         -T_7,
 *         -T_8,
 *         -T_9,
 *     )
 *
 * Where for a given T_i = (a, b, c), -T_i = (-a, -b, -c). We call -T_i the
 * opposite triangle of T_i in this specification. For any i in [0,20), T_j is
 * the opposite of T_i iff j = (i + 10) % 20.
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
    friend class GeodesicGridTest;

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
     * @return The index of the section. The value -1 is returned if \p v is
     * the null vector or the section isn't found, which might happen when \p
     * inclusive is false.
     */
    static int section(const Vector3f &v, bool inclusive = false);

private:
    /*
     * The following are concepts used in the description of the private
     * members.
     *
     * Neighbor triangle with respect to an edge
     * -----------------------------------------
     * Let T be a triangle. The triangle W is a neighbor of T with respect to
     * edge e if T and W share that edge. If e is formed by vectors a and b,
     * then W can be said to be a neighbor of T with respect to a and b.
     *
     * Umbrella of a vector
     * --------------------
     * Let v be one vertex of the icosahedron. The umbrella of v is the set of
     * icosahedron triangles that share that vertex. The vector v is called the
     * umbrella's pivot.
     *
     * Let T have vertices v, a and b. Then, with respect to (a, b):
     *  - The vector a is the umbrella's 0-th vertex.
     *  - The vector b is the 1-th vertex.
     *  - The triangle formed by the v, the i-th and ((i + 1) % 5)-th vertex is
     *  the umbrella's i-th component.
     *  - For i in [2,5), the i-th vertex is the vertex that, with the
     *  (i - 1)-th and v, forms the neighbor of the (i - 2)-th component with
     *  respect to v and the (i - 1)-th vertex.
     *
     * Still with respect to (a, b), the umbrella's i-th component is the
     * triangle formed by the i-th and ((i + 1) % 5)-th vertices and the pivot.
     *
     * Neighbor umbrella with respect to an icosahedron triangle's edge
     * ----------------------------------------------------------------
     * Let T be an icosahedron triangle. Let W be the T's neighbor triangle wrt
     * the edge e. Let w be the W's vertex that is opposite to e. Then the
     * neighbor umbrella of T with respect to e is the umbrella of w.
     */

    /**
     * The inverses of the change-of-basis matrices for the icosahedron
     * triangles.
     *
     * The i-th matrix is the inverse of the change-of-basis matrix from
     * natural basis to the basis formed by T_i's vectors.
     */
    static const Matrix3f _inverses[10];

    /**
     * The inverses of the change-of-basis matrices for the middle triangles.
     *
     * The i-th matrix is the inverse of the change-of-basis matrix from
     * natural basis to the basis formed by T_i's middle triangle's vectors.
     */
    static const Matrix3f _mid_inverses[10];

    /**
     * The representation of the neighbor umbrellas of T_0.
     *
     * The values for the neighbors of T_10 can be derived from the values for
     * T_0. How to find the correct values is explained on each member.
     *
     * Let T_0 = (a, b, c). Thus, 6 indexes can be used for this data
     * structure, so that:
     *  - index 0 represents the neighbor of T_0 with respect to (a, b).
     *  - index 1 represents the neighbor of T_0 with respect to (b, c).
     *  - index 2 represents the neighbor of T_0 with respect to (c, a).
     *  - index 3 represents the neighbor of T_10 with respect to (-a, -b).
     *  - index 4 represents the neighbor of T_10 with respect to (-b, -c).
     *  - index 5 represents the neighbor of T_10 with respect to (-c, -a).
     *
     * Those indexes are mapped to this array with index % 3.
     *
     * The edges are represented with pairs because the order of the vertices
     * matters to the order the triangles' indexes are defined - the order of
     * the umbrellas' vertices and components is convertioned to be with
     * respect to those pairs.
     */
    static const struct neighbor_umbrella {
        /**
         * The umbrella's components. The value of #components[i] is the
         * icosahedron triangle index of the i-th component.
         *
         * In order to find the components for T_10, the following just finding
         * the index of the opposite triangle is enough. In other words,
         * (#components[i] + 10) % 20.
         */
        uint8_t components[5];
        /**
         * The fields with name in the format vi_cj are interpreted as the
         * following: vi_cj is the index of the vector, in the icosahedron
         * triangle pointed by #components[j], that matches the umbrella's i-th
         * vertex.
         *
         * The values don't change for T_10.
         */
        uint8_t v0_c0;
        uint8_t v1_c1;
        uint8_t v2_c1;
        uint8_t v4_c4;
        uint8_t v0_c4;
    } _neighbor_umbrellas[3];

    /**
     * Get the component_index-th component of the umbrella_index-th neighbor
     * umbrella.
     *
     * @param umbrella_index[in] The neighbor umbrella's index.
     *
     * @param component_index[in] The component's index.
     *
     * @return The icosahedron triangle's index of the component.
     */
    static int _neighbor_umbrella_component(int umbrella_index, int component_idx);

    /**
     * Find the icosahedron triangle index of the component of
     * #_neighbor_umbrellas[umbrella_index] that is crossed by \p v.
     *
     * @param umbrella_index[in] The umbrella index. Must be in [0,6).
     *
     * @param v[in] The vector to be tested.
     *
     * @param u[in] The vector \p u must be \p v  expressed with respect to the
     * base formed by the umbrella's 0-th, 1-th and 3-th vertices, in that
     * order.
     *
     * @param inclusive[in] This parameter follows the same rules defined in
     * #section() const.
     *
     * @return The index of the icosahedron triangle. The value -1 is returned
     * if \p v is the null vector or the triangle isn't found, which might
     * happen when \p inclusive is false.
     */
    static int _from_neighbor_umbrella(int umbrella_index,
                                       const Vector3f &v,
                                       const Vector3f &u,
                                       bool inclusive);

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
    static int _triangle_index(const Vector3f &v, bool inclusive);

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
    static int _subtriangle_index(const unsigned int triangle_index,
                                  const Vector3f &v,
                                  bool inclusive);
};
