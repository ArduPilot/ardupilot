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
  The point in polygon algorithm is based on:
  http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
*/


/*
  Polygon_outside(): test for a point in a polygon
      Input:   P = a point,
               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
      Return:  true if P is outside the polygon
*/
bool Polygon_outside(const Vector2f &P, const Vector2f *V, unsigned n)
{
    unsigned i, j;
    bool outside = true;
    for (i = 0, j = n-1; i < n; j = i++) {
        if ( ((V[i].y > P.y) != (V[j].y > P.y)) &&
             (P.x < (V[j].x - V[i].x) * (P.y - V[i].y) / (V[j].y - V[i].y) + V[i].x) )
            outside = !outside;
    }
    return outside;
}

/*
  check if a polygon is complete. 

  We consider a polygon to be complete if we have at least 4 points,
  and the first point is the same as the last point. That is the
  minimum requirement for the Polygon_outside function to work
 */
bool Polygon_complete(const Vector2f *V, unsigned n)
{
    return (n >= 4 && V[n-1].x == V[0].x && V[n-1].y == V[0].y);
}
