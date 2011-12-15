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
  NOTE: the winding number crossing algorithm is based on 
  the code from
  http://www.softsurfer.com/Archive/algorithm_0103/algorithm_0103.htm

  which has the following copyright notice:

// Copyright 2001, softSurfer (www.softsurfer.com)
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.

*/

/* isLeft(): tests if a point is Left|On|Right of an infinite line.
    Input:  three points P0, P1, and P2
    Return: >0 for P2 left of the line through P0 and P1
            =0 for P2 on the line
            <0 for P2 right of the line
    See: the January 2001 Algorithm "Area of 2D and 3D Triangles and Polygons"
*/
static int isLeft(const Vector2f *P0, const Vector2f *P1, const Vector2f *P2)
{
    float ret = ( (P1->x - P0->x) * (P2->y - P0->y)
                  - (P2->x - P0->x) * (P1->y - P0->y) );
    if (ret > 0) return 1;
    if (ret < 0) return -1;
    return 0;
}

/*
 Polygon_outside(): winding number test for a point in a polygon
      Input:   P = a point,
               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
      Return:  true if P is outside the polygon
*/
bool Polygon_outside(const Vector2f *P, const Vector2f *V, unsigned n)
{
	int    wn = 0;    // the winding number counter

    // loop through all edges of the polygon
    for (unsigned i=0; i<n; i++) {   // edge from V[i] to V[i+1]
        if (V[i].y <= P->y) {         // start y <= P.y
            if (V[i+1].y > P->y)      // an upward crossing
                if (isLeft(&V[i], &V[i+1], P) > 0)  // P left of edge
                    ++wn;            // have a valid up intersect
        }
        else {                       // start y > P.y (no test needed)
            if (V[i+1].y <= P->y)     // a downward crossing
                if (isLeft(&V[i], &V[i+1], P) < 0)  // P right of edge
                    --wn;            // have a valid down intersect
        }
    }
    return wn == 0;
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
