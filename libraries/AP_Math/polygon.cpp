/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *  http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
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
    unsigned i, j;
    bool outside = true;
    for (i = 0, j = n-1; i < n; j = i++) {
        if ((V[i].y > P.y) == (V[j].y > P.y)) {
            continue;
        }
        int32_t dx1, dx2, dy1, dy2;
        dx1 = P.x - V[i].x;
        dx2 = V[j].x - V[i].x;
        dy1 = P.y - V[i].y;
        dy2 = V[j].y - V[i].y;
        int8_t dx1s, dx2s, dy1s, dy2s, m1, m2;
#define sign(x) ((x)<0 ? -1 : 1)
        dx1s = sign(dx1);
        dx2s = sign(dx2);
        dy1s = sign(dy1);
        dy2s = sign(dy2);
        m1 = dx1s * dy2s;
        m2 = dx2s * dy1s;
        // we avoid the 64 bit multiplies if we can based on sign checks.
        if (dy2 < 0) {
            if (m1 > m2) {
                outside = !outside;
            } else if (m1 < m2) {
                continue;
            } else if ( dx1 * (int64_t)dy2 > dx2 * (int64_t)dy1 ) {
                outside = !outside;
            }
        } else {
            if (m1 < m2) {
                outside = !outside;
            } else if (m1 > m2) {
                continue;
            } else if ( dx1 * (int64_t)dy2 < dx2 * (int64_t)dy1 ) {
                outside = !outside;
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
    return (n >= 4 && V[n-1].x == V[0].x && V[n-1].y == V[0].y);
}

// Necessary to avoid linker errors
template bool Polygon_outside<int32_t>(const Vector2l &P, const Vector2l *V, unsigned n);
template bool Polygon_complete<int32_t>(const Vector2l *V, unsigned n);
template bool Polygon_outside<float>(const Vector2f &P, const Vector2f *V, unsigned n);
template bool Polygon_complete<float>(const Vector2f *V, unsigned n);
