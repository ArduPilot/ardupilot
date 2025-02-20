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
#include "float.h"

#pragma GCC optimize("O2")

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

/*
  determine if the polygon of N verticies defined by points V is
  intersected by a line from point p1 to point p2
  intersection argument returns the intersection closest to p1
 */
bool Polygon_intersects(const Vector2f *V, unsigned N, const Vector2f &p1, const Vector2f &p2, Vector2f &intersection)
{
    const bool complete = Polygon_complete(V, N);
    if (complete) {
        // if the last point is the same as the first point
        // treat as if the last point wasn't passed in
        N--;
    }

    float intersect_dist_sq = FLT_MAX;
    for (uint8_t i=0; i<N; i++) {
        uint8_t j = i+1;
        if (j >= N) {
            j = 0;
        }
        const Vector2f &v1 = V[i];
        const Vector2f &v2 = V[j];
        // optimisations for common cases
        if (v1.x > p1.x && v2.x > p1.x && v1.x > p2.x && v2.x > p2.x) {
            continue;
        }
        if (v1.y > p1.y && v2.y > p1.y && v1.y > p2.y && v2.y > p2.y) {
            continue;
        }
        if (v1.x < p1.x && v2.x < p1.x && v1.x < p2.x && v2.x < p2.x) {
            continue;
        }
        if (v1.y < p1.y && v2.y < p1.y && v1.y < p2.y && v2.y < p2.y) {
            continue;
        }
        Vector2f intersect_tmp;
        if (Vector2f::segment_intersection(v1,v2,p1,p2,intersect_tmp)) {
            float dist_sq = sq(intersect_tmp.x - p1.x) + sq(intersect_tmp.y - p1.y);
            if (dist_sq < intersect_dist_sq) {
                intersect_dist_sq = dist_sq;
                intersection = intersect_tmp;
            }
        }
    }
    return (intersect_dist_sq < FLT_MAX);
}

/*
  return the closest distance that a line from p1 to p2 comes to an
  edge of closed polygon V, defined by N points
  negative numbers indicate the line cross into the polygon with the negative size being the distance from p2 to the intersection point closest to p1
 */
float Polygon_closest_distance_line(const Vector2f *V, unsigned N, const Vector2f &p1, const Vector2f &p2)
{
    Vector2f intersection;
    if (Polygon_intersects(V,N,p1,p2,intersection)) {
        return -sqrtf(sq(intersection.x - p2.x) + sq(intersection.y - p2.y));
    }
    float closest_sq = FLT_MAX;
    for (uint8_t i=0; i<N-1; i++) {
        const Vector2f &v1 = V[i];
        const Vector2f &v2 = V[i+1];

        float dist_sq = Vector2f::closest_distance_between_lines_squared(v1, v2, p1, p2);
        if (dist_sq < closest_sq) {
            closest_sq = dist_sq;
        }
    }
    return sqrtf(closest_sq);
}

const ftype EARTH_RADIUS_METERS = 6378137.0; // Earth's radius in meters (WGS84)

// Converts 1e7 degrees to radians
static ftype to_radians(ftype degrees_1e7)
{
    return radians(degrees_1e7 / 1.0e7);
}

// Converts (latitude, longitude) in radians to 3D Cartesian coordinates on a unit sphere
static void latlng_to_cartesian(ftype latRad, ftype lonRad, Vector3F& v)
{
    v.x = cosF(latRad) * cosF(lonRad);
    v.y = cosF(latRad) * sinF(lonRad);
    v.z = sinF(latRad);
}

// Haversine distance between two points given in 1e7 degrees
static ftype haversine(long lat1_1e7, long lon1_1e7, long lat2_1e7, long lon2_1e7)
{
    ftype lat1 = to_radians(lat1_1e7);
    ftype lat2 = to_radians(lat2_1e7);

    ftype dLat = to_radians(lat2_1e7 - lat1_1e7);
    ftype dLon = to_radians(lon2_1e7 - lon1_1e7);

    // (d/2r)^2
    ftype a = dLat * dLat / 4 + cosF(lat2) * cosF(lat1) * dLon * dLon / 4;
    ftype d = sqrtF(a) * 2 * EARTH_RADIUS_METERS;

    return d;
}

// Compute closest distance from a point to a line segment on a sphere
static ftype closest_distance_to_segment(const Vector2l& point, const Vector2l& line_start, const Vector2l& line_end)
{
    // Convert lat/lon to 3D Cartesian coordinates on a unit sphere
    Vector3F v1, v2, vp;

    latlng_to_cartesian(to_radians(line_start.x), to_radians(line_start.y), v1);
    latlng_to_cartesian(to_radians(line_end.x), to_radians(line_end.y), v2);
    latlng_to_cartesian(to_radians(point.x), to_radians(point.y), vp);


    // Compute vector from lineStart to lineEnd (v) and from lineStart to point (w)
    Vector3F v = v2 - v1;
    Vector3F w = vp - v1;

    // Project w onto v
    ftype dot_vw = v * w;
    ftype dot_vv = v * v;
    // For reasons I don't understand when v.x and v.y are precisely zero, meaning that dot_vv is precisely
    // zero, the division below crashes in SITL. According to the C++ spec this should result in nan or inf,
    // so unclear why this is. We can't check dot_vv for zero as it can be very small and still valid.
    if (is_zero(v.x) && is_zero(v.y)) {
        return FLT_MAX;
    }
    ftype t = dot_vw / dot_vv;

    if (isnan(t) || isinf(t)) {
        return FLT_MAX;
    }

    Vector3F closest;
    if (is_negative(t)) {
        // Closest to line start
        closest = v1;
    } else if (t > 1) {
        // Closest to line end
        closest = v2;
    } else {
        // Closest point is on the line
        closest = v1 + v * t;
    }

    // Convert closest point back to lat/lon (reverse cartesian)
    ftype norm = closest.length();
    ftype closestLat = safe_asin(closest.z / norm); // Latitude from z-coordinate
    ftype closestLon = atan2F(closest.y, closest.x); // Longitude from x and y

    // Convert back to 1e7 degrees
    long closestLat_1e7 = static_cast<long>(degrees(closestLat) * 1e7);
    long closestLon_1e7 = static_cast<long>(degrees(closestLon) * 1e7);

    // Calculate the Haversine distance from the point to the closest point on the line
    return haversine(point.x, point.y, closestLat_1e7, closestLon_1e7);
}

/*
  return the closest distance in meters that point p in lat/lng 1e7 comes to an edge of closed
  polygon V, defined by N points of lat/lng 1e7
 */
bool Polygon_closest_distance_point(const Vector2l *V, unsigned N, const Vector2l &p, ftype& closest)
{
    closest = FLT_MAX;
    const bool complete = Polygon_complete(V, N);
    if (complete) {
        // the last point is the same as the first point; treat as if
        // the last point wasn't passed in
        N--;
    }
    if (N < 3) {    // not a polygon
        return false;
    }
    for (uint8_t i=0; i<N; i++) {
        const Vector2l &v1 = V[i];
        const Vector2l &v2 = V[(i+1) % N];

        ftype distance = closest_distance_to_segment(p, v1, v2);
        if (distance < closest) {
            closest = distance;
        }
    }
    if (is_equal(closest, (ftype)FLT_MAX)) {
        closest = 0.0f;
        return false;
    }
    return true;
}
