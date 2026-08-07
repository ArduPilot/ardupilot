/*
 * polygon.h
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
#pragma once

#include "vector2.h"

template <typename T>
bool        Polygon_outside(const Vector2<T> &P, const Vector2<T> *V, unsigned n) WARN_IF_UNUSED;
template <typename T>
bool        Polygon_complete(const Vector2<T> *V, unsigned n) WARN_IF_UNUSED;

/*
  determine if the polygon of N verticies defined by points V is
  intersected by a line from point p1 to point p2
  intersection argument returns the intersection closest to p1
 */
bool Polygon_intersects(const Vector2f *V, unsigned N, const Vector2f &p1, const Vector2f &p2, Vector2f &intersection) WARN_IF_UNUSED;


/*
  return the closest distance that a line from p1 to p2 comes to an
  edge of closed polygon V, defined by N points
  negative numbers indicate the line cross into the polygon with the negative size being the distance from p2 to the intersection point closest to p1
 */
float Polygon_closest_distance_line(const Vector2f *V, unsigned N, const Vector2f &p1, const Vector2f &p2);

/*
  return the closest distance that a point p in cartesian comes to an edge of
  closed polygon V, defined by N points of cartesian. Returns true if successful, false otherwise
 */
 bool Polygon_closest_distance_point(const Vector2f *V, unsigned N, const Vector2f &p, Vector2f& closest_segment);
 