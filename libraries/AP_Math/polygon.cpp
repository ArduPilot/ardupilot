/*
 * polygon.cpp
 * Copyright (C) Andrew Tridgell 2011
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

#include "AP_Math.h"

/*
 *  The point in polygon algorithm is based on:
 *  https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
 */


/*
 *  Polygon_outside(): test for a point in a polygon
 *     Input:   P = a point,
 *              V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
 *     Return:  true if P is outside the polygon
 *
 *  This does not take account of the curvature of the earth, but we
 *  expect that to be very small over the distances involved in the
 *  fence boundary
 */
template <typename T>
bool Polygon_outside(const Vector2<T> &P, const Vector2<T> *V, unsigned n)
{
    const bool complete = Polygon_complete(V, n);
    if (complete) {
        // the last point is the same as the first point; treat as if
        // the last point wasn't passed in
        n--;
    }

    unsigned i, j;
    // step through each edge pair-wise looking for crossings:
    bool outside = true;
    for (i=0; i<n; i++) {
        j = i+1;
        if (j >= n) {
            j = 0;
        }
        if ((V[i].y > P.y) == (V[j].y > P.y)) {
            continue;
        }
        const T dx1 = P.x - V[i].x;
        const T dx2 = V[j].x - V[i].x;
        const T dy1 = P.y - V[i].y;
        const T dy2 = V[j].y - V[i].y;
        const int8_t dx1s = (dx1 < 0) ? -1 : 1;
        const int8_t dx2s = (dx2 < 0) ? -1 : 1;
        const int8_t dy1s = (dy1 < 0) ? -1 : 1;
        const int8_t dy2s = (dy2 < 0) ? -1 : 1;
        const int8_t m1 = dx1s * dy2s;
        const int8_t m2 = dx2s * dy1s;
        // we avoid the 64 bit multiplies if we can based on sign checks.
        if (dy2 < 0) {
            if (m1 > m2) {
                outside = !outside;
            } else if (m1 < m2) {
                continue;
            } else {
                if (std::is_floating_point<T>::value) {
                    if ( dx1 * dy2 > dx2 * dy1 ) {
                        outside = !outside;
                    }
                } else {
                    if ( dx1 * (int64_t)dy2 > dx2 * (int64_t)dy1 ) {
                        outside = !outside;
                    }
                }
            }
        } else {
            if (m1 < m2) {
                outside = !outside;
            } else if (m1 > m2) {
                continue;
            } else {
                if (std::is_floating_point<T>::value) {
                    if ( dx1 * dy2 < dx2 * dy1 ) {
                        outside = !outside;
                    }
                } else {
                    if ( dx1 * (int64_t)dy2 < dx2 * (int64_t)dy1 ) {
                        outside = !outside;
                    }
                }
            }
        }
    }
    return outside;
}

/*
 *  check if a polygon is complete.
 *
 *  We consider a polygon to be complete if we have at least 4 points,
 *  and the first point is the same as the last point. That is the
 *  minimum requirement for the Polygon_outside function to work
 */
template <typename T>
bool Polygon_complete(const Vector2<T> *V, unsigned n)
{
    return (n >= 4 && V[n-1] == V[0]);
}

// Necessary to avoid linker errors
template bool Polygon_outside<int32_t>(const Vector2l &P, const Vector2l *V, unsigned n);
template bool Polygon_complete<int32_t>(const Vector2l *V, unsigned n);
template bool Polygon_outside<float>(const Vector2f &P, const Vector2f *V, unsigned n);
template bool Polygon_complete<float>(const Vector2f *V, unsigned n);
