/*
 * vector3.cpp
 * Copyright (C) Andrew Tridgell 2012
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

#pragma GCC optimize("O3")

#include "AP_Math.h"

template <typename T>
float Vector2<T>::length(void) const
{
	return norm(x, y);
}


// dot product
template <typename T>
T Vector2<T>::operator *(const Vector2<T> &v) const
{
    return x*v.x + y*v.y;
}

// cross product
template <typename T>
T Vector2<T>::operator %(const Vector2<T> &v) const
{
    return x*v.y - y*v.x;
}

template <typename T>
Vector2<T> &Vector2<T>::operator *=(const T num)
{
    x*=num; y*=num;
    return *this;
}

template <typename T>
Vector2<T> &Vector2<T>::operator /=(const T num)
{
    x /= num; y /= num;
    return *this;
}

template <typename T>
Vector2<T> &Vector2<T>::operator -=(const Vector2<T> &v)
{
    x -= v.x; y -= v.y;
    return *this;
}

template <typename T>
bool Vector2<T>::is_nan(void) const
{
    return isnan(x) || isnan(y);
}

template <typename T>
bool Vector2<T>::is_inf(void) const
{
    return isinf(x) || isinf(y);
}

template <typename T>
Vector2<T> &Vector2<T>::operator +=(const Vector2<T> &v)
{
    x+=v.x; y+=v.y;
    return *this;
}

template <typename T>
Vector2<T> Vector2<T>::operator /(const T num) const
{
    return Vector2<T>(x/num, y/num);
}

template <typename T>
Vector2<T> Vector2<T>::operator *(const T num) const
{
    return Vector2<T>(x*num, y*num);
}

template <typename T>
Vector2<T> Vector2<T>::operator -(const Vector2<T> &v) const
{
    return Vector2<T>(x-v.x, y-v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator +(const Vector2<T> &v) const
{
    return Vector2<T>(x+v.x, y+v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator -(void) const
{
    return Vector2<T>(-x,-y);
}

template <typename T>
bool Vector2<T>::operator ==(const Vector2<T> &v) const
{
    return (is_equal(x,v.x) && is_equal(y,v.y));
}

template <typename T>
bool Vector2<T>::operator !=(const Vector2<T> &v) const
{
    return (!is_equal(x,v.x) || !is_equal(y,v.y));
}

template <typename T>
float Vector2<T>::angle(const Vector2<T> &v2) const
{
    const float len = this->length() * v2.length();
    if (len <= 0) {
        return 0.0f;
    }
    const float cosv = ((*this)*v2) / len;
    if (cosv >= 1) {
        return 0.0f;
    }
    if (cosv <= -1) {
        return M_PI;
    }
    return acosf(cosv);
}

// find the intersection between two line segments
// returns true if they intersect, false if they do not
// the point of intersection is returned in the intersection argument
template <typename T>
bool Vector2<T>::segment_intersection(const Vector2<T>& seg1_start, const Vector2<T>& seg1_end, const Vector2<T>& seg2_start, const Vector2<T>& seg2_end, Vector2<T>& intersection)
{
    // implementation borrowed from http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    const Vector2<T> r1 = seg1_end - seg1_start;
    const Vector2<T> r2 = seg2_end - seg2_start;
    const Vector2<T> ss2_ss1 = seg2_start - seg1_start;
    const float r1xr2 = r1 % r2;
    const float q_pxr = ss2_ss1 % r1;
    if (fabsf(r1xr2) < FLT_EPSILON) {
        // either collinear or parallel and non-intersecting
        return false;
    } else {
        // t = (q - p) * s / (r * s)
        // u = (q - p) * r / (r * s)
        const float t = (ss2_ss1 % r2) / r1xr2;
        const float u = q_pxr / r1xr2;
        if ((u >= 0) && (u <= 1) && (t >= 0) && (t <= 1)) {
            // lines intersect
            // t can be any non-negative value because (p, p + r) is a ray
            // u must be between 0 and 1 because (q, q + s) is a line segment
            intersection = seg1_start + (r1*t);
            return true;
        } else {
            // non-parallel and non-intersecting
            return false;
        }
    }
}

// find the intersection between a line segment and a circle
// returns true if they intersect and intersection argument is updated with intersection closest to seg_start
// solution adapted from http://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
template <typename T>
bool Vector2<T>::circle_segment_intersection(const Vector2<T>& seg_start, const Vector2<T>& seg_end, const Vector2<T>& circle_center, float radius, Vector2<T>& intersection)
{
    // calculate segment start and end as offsets from circle's center
    const Vector2f seg_start_local = seg_start - circle_center;

    // calculate vector from start to end
    const Vector2f seg_end_minus_start = seg_end - seg_start;

    const float a = sq(seg_end_minus_start.x) + sq(seg_end_minus_start.y);
    const float b = 2 * ((seg_end_minus_start.x * seg_start_local.x) + (seg_end_minus_start.y * seg_start_local.y));
    const float c = sq(seg_start_local.x) + sq(seg_start_local.y) - sq(radius);
    const float delta = sq(b) - (4.0f * a * c);

    // check for invalid data
    if (::is_zero(a)) {
        return false;
    }
    if (isnan(a) || isnan(b) || isnan(c) || isnan(delta)) {
       return false;
    }

    // check for invalid delta (i.e. discriminant)
    if (delta < 0.0f) {
        return false;
    }

    const float delta_sqrt = sqrtf(delta);
    const float t1 = (-b + delta_sqrt) / (2.0f * a);
    const float t2 = (-b - delta_sqrt) / (2.0f * a);

    // Three hit cases:
    //          -o->             --|-->  |            |  --|->
    // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

    // Three miss cases:
    //       ->  o                     o ->              | -> |
    // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

    // intersection = new Vector3(E.x + t1 * d.x, secondPoint.y, E.y + t1 * d.y);
    //   intersection.x = seg_start.x + t1 * seg_end_minus_start.x;
    //   intersection.y = seg_start.y + t1 * seg_end_minus_start.y;

    if ((t1 >= 0.0f) && (t1 <= 1.0f)) {
        // t1 is the intersection, and it is closer than t2 (since t1 uses -b - discriminant)
        // Impale, Poke
        intersection = seg_start + (seg_end_minus_start * t1);
        return true;
    }

    // here t1 did not intersect so we are either started inside the sphere or completely past it
    if ((t2 >= 0.0f) && (t2 <= 1.0f)) {
        // ExitWound
        intersection = seg_start + (seg_end_minus_start * t2);
        return true;
    }

    // no intersection: FallShort, Past or CompletelyInside
    return false;
}

// only define for float
template float Vector2<float>::length(void) const;
template float Vector2<float>::operator *(const Vector2<float> &v) const;
template float Vector2<float>::operator %(const Vector2<float> &v) const;
template Vector2<float> &Vector2<float>::operator *=(const float num);
template Vector2<float> &Vector2<float>::operator /=(const float num);
template Vector2<float> &Vector2<float>::operator -=(const Vector2<float> &v);
template Vector2<float> &Vector2<float>::operator +=(const Vector2<float> &v);
template Vector2<float> Vector2<float>::operator /(const float num) const;
template Vector2<float> Vector2<float>::operator *(const float num) const;
template Vector2<float> Vector2<float>::operator +(const Vector2<float> &v) const;
template Vector2<float> Vector2<float>::operator -(const Vector2<float> &v) const;
template Vector2<float> Vector2<float>::operator -(void) const;
template bool Vector2<float>::operator ==(const Vector2<float> &v) const;
template bool Vector2<float>::operator !=(const Vector2<float> &v) const;
template bool Vector2<float>::is_nan(void) const;
template bool Vector2<float>::is_inf(void) const;
template float Vector2<float>::angle(const Vector2<float> &v) const;
template bool Vector2<float>::segment_intersection(const Vector2<float>& seg1_start, const Vector2<float>& seg1_end, const Vector2<float>& seg2_start, const Vector2<float>& seg2_end, Vector2<float>& intersection);
template bool Vector2<float>::circle_segment_intersection(const Vector2<float>& seg_start, const Vector2<float>& seg_end, const Vector2<float>& circle_center, float radius, Vector2<float>& intersection);

template bool Vector2<long>::operator ==(const Vector2<long> &v) const;

// define for int
template bool Vector2<int>::operator ==(const Vector2<int> &v) const;
