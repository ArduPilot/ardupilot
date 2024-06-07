/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Copyright 2010 Michael Smith, all rights reserved.

// Derived closely from:
/****************************************
* 3D Vector Classes
* By Bill Perone (billperone@yahoo.com)
* Original: 9-16-2002
* Revised: 19-11-2003
*          11-12-2003
*          18-12-2003
*          06-06-2004
*
* Copyright 2003, This code is provided "as is" and you can use it freely as long as
* credit is given to Bill Perone in the application it is used in
*
* Notes:
* if a*b = 0 then a & b are orthogonal
* a%b = -b%a
* a*(b%c) = (a%b)*c
* a%b = a(cast to matrix)*b
* (a%b).length() = area of parallelogram formed by a & b
* (a%b).length() = a.length()*b.length() * sin(angle between a & b)
* (a%b).length() = 0 if angle between a & b = 0 or a.length() = 0 or b.length() = 0
* a * (b%c) = volume of parallelpiped formed by a, b, c
* vector triple product: a%(b%c) = b*(a*c) - c*(a*b)
* scalar triple product: a*(b%c) = c*(a%b) = b*(c%a)
* vector quadruple product: (a%b)*(c%d) = (a*c)*(b*d) - (a*d)*(b*c)
* if a is unit vector along b then a%b = -b%a = -b(cast to matrix)*a = 0
* vectors a1...an are linearly dependent if there exists a vector of scalars (b) where a1*b1 + ... + an*bn = 0
*           or if the matrix (A) * b = 0
*
****************************************/
#pragma once

#ifndef MATH_CHECK_INDEXES
#define MATH_CHECK_INDEXES 0
#endif

#include <cmath>
#include <float.h>
#include <string.h>
#if MATH_CHECK_INDEXES
#include <assert.h>
#endif

#include "rotations.h"

#include "ftype.h"

template <typename T>
class Matrix3;

template <typename T>
class Vector2;

template <typename T>
class Vector3
{

public:
    T        x, y, z;

    // trivial ctor
    constexpr Vector3<T>()
        : x(0)
        , y(0)
        , z(0) {}

    // setting ctor
    constexpr Vector3<T>(const T x0, const T y0, const T z0)
        : x(x0)
        , y(y0)
        , z(z0) {}

    //Create a Vector3 from a Vector2 with z
    constexpr Vector3<T>(const Vector2<T> &v0, const T z0)
        : x(v0.x)
        , y(v0.y)
        , z(z0) {}

    // test for equality
    bool operator ==(const Vector3<T> &v) const;

    // test for inequality
    bool operator !=(const Vector3<T> &v) const;

    // negation
    Vector3<T> operator -(void) const;

    // addition
    Vector3<T> operator +(const Vector3<T> &v) const;

    // subtraction
    Vector3<T> operator -(const Vector3<T> &v) const;

    // uniform scaling
    Vector3<T> operator *(const T num) const;

    // uniform scaling
    Vector3<T> operator  /(const T num) const;

    // addition
    Vector3<T> &operator +=(const Vector3<T> &v);

    // subtraction
    Vector3<T> &operator -=(const Vector3<T> &v);

    // uniform scaling
    Vector3<T> &operator *=(const T num);

    // uniform scaling
    Vector3<T> &operator /=(const T num);

    // non-uniform scaling
    Vector3<T> &operator *=(const Vector3<T> &v) {
        x *= v.x; y *= v.y; z *= v.z;
        return *this;
    }

    // allow a vector3 to be used as an array, 0 indexed
    T & operator[](uint8_t i) {
        T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    const T & operator[](uint8_t i) const {
        const T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    // dot product
    T operator *(const Vector3<T> &v) const;

    // dot product for Lua
    T dot(const Vector3<T> &v) const {
        return *this * v;
    }
    
    // multiply a row vector by a matrix, to give a row vector
    Vector3<T> row_times_mat(const Matrix3<T> &m) const;

    // multiply a column vector by a row vector, returning a 3x3 matrix
    Matrix3<T> mul_rowcol(const Vector3<T> &v) const;

    // cross product
    Vector3<T> operator %(const Vector3<T> &v) const;

    // cross product for Lua
    Vector3<T> cross(const Vector3<T> &v) const {
        return *this % v;
    }

    // scale a vector3
    Vector3<T> scale(const T v) const {
        return *this * v;
    }
    
    // computes the angle between this vector and another vector
    T angle(const Vector3<T> &v2) const;

    // check if any elements are NAN
    bool is_nan(void) const WARN_IF_UNUSED;

    // check if any elements are infinity
    bool is_inf(void) const WARN_IF_UNUSED;

    // check if all elements are zero
    bool is_zero(void) const WARN_IF_UNUSED {
        return x == 0 && y == 0 && z == 0;
    }


    // rotate by a standard rotation
    void rotate(enum Rotation rotation);
    void rotate_inverse(enum Rotation rotation);

    // rotate vector by angle in radians in xy plane leaving z untouched
    void rotate_xy(T rotation_rad);

    // return xy components of a vector3 as a vector2.
    // this returns a reference to the original vector3 xy data
    const Vector2<T> &xy() const {
        return *(const Vector2<T> *)this;
    }
    Vector2<T> &xy() {
        return *(Vector2<T> *)this;
    }

    // gets the length of this vector squared
    T  length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    T length(void) const;

    // limit xy component vector to a given length. returns true if vector was limited
    bool limit_length_xy(T max_length);

    // normalizes this vector
    void normalize()
    {
        *this /= length();
    }

    // zero the vector
    void zero()
    {
        x = y = z = 0;
    }

    // returns the normalized version of this vector
    Vector3<T> normalized() const
    {
        return *this/length();
    }

    // reflects this vector about n
    void  reflect(const Vector3<T> &n)
    {
        Vector3<T>        orig(*this);
        project(n);
        *this = *this*2 - orig;
    }

    // projects this vector onto v
    void project(const Vector3<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    Vector3<T> projected(const Vector3<T> &v) const
    {
        return v * (*this * v)/(v*v);
    }

    // distance from the tip of this vector to another vector squared (so as to avoid the sqrt calculation)
    T distance_squared(const Vector3<T> &v) const {
        const T dist_x = x-v.x;
        const T dist_y = y-v.y;
        const T dist_z = z-v.z;
        return (dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
    }

    // distance from the tip of this vector to a line segment specified by two vectors
    T distance_to_segment(const Vector3<T> &seg_start, const Vector3<T> &seg_end) const;

    // extrapolate position given bearing and pitch (in degrees) and distance
    void offset_bearing(T bearing, T pitch, T distance);

    /*
      conversion to/from double
     */
    Vector3<float> tofloat() const {
        return Vector3<float>{float(x),float(y),float(z)};
    }
    Vector3<double> todouble() const {
        return Vector3<double>{x,y,z};
    }

    // convert from right-front-up to front-right-down
    // or ENU to NED
    Vector3<T> rfu_to_frd() const {
        return Vector3<T>{y,x,-z};
    }

    // given a position p1 and a velocity v1 produce a vector
    // perpendicular to v1 maximising distance from p1.  If p1 is the
    // zero vector the return from the function will always be the
    // zero vector - that should be checked for.
    static Vector3<T> perpendicular(const Vector3<T> &p1, const Vector3<T> &v1)
    {
        const T d = p1 * v1;
        if (::is_zero(d)) {
            return p1;
        }
        const Vector3<T> parallel = (v1 * d) / v1.length_squared();
        Vector3<T> perpendicular = p1 - parallel;

        return perpendicular;
    }

    // Shortest distance between point(p) to a point contained in the line segment defined by w1,w2
    static T closest_distance_between_line_and_point(const Vector3<T> &w1, const Vector3<T> &w2, const Vector3<T> &p);

    // Point in the line segment defined by w1,w2 which is closest to point(p)
    static Vector3<T> point_on_line_closest_to_other_point(const Vector3<T> &w1, const Vector3<T> &w2, const Vector3<T> &p);

    // This implementation is borrowed from: http://geomalgorithms.com/a07-_distance.html
    // INPUT: 4 points corresponding to start and end of two line segments

    // OUTPUT: closest point on segment 2, from segment 1, gets passed on reference as "closest_point"
    static void segment_to_segment_closest_point(const Vector3<T>& seg1_start, const Vector3<T>& seg1_end, const Vector3<T>& seg2_start, const Vector3<T>& seg2_end, Vector3<T>& closest_point);

    // Returns true if the passed 3D segment passes through a plane defined by plane normal, and a point on the plane
    static bool segment_plane_intersect(const Vector3<T>& seg_start, const Vector3<T>& seg_end, const Vector3<T>& plane_normal, const Vector3<T>& plane_point);
};

// check if all elements are zero
template<> inline bool Vector3<float>::is_zero(void) const {
    return ::is_zero(x) && ::is_zero(y) && ::is_zero(z);
}

template<> inline bool Vector3<double>::is_zero(void) const {
    return ::is_zero(x) && ::is_zero(y) && ::is_zero(z);
}

// The creation of temporary vector objects as return types creates a significant overhead in certain hot 
// code paths. This allows callers to select the inline versions where profiling shows a significant benefit
#if defined(AP_INLINE_VECTOR_OPS) && !defined(HAL_DEBUG_BUILD)

// vector cross product
template <typename T>
inline Vector3<T> Vector3<T>::operator %(const Vector3<T> &v) const
{
    return Vector3<T>(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
}

// dot product
template <typename T>
inline T Vector3<T>::operator *(const Vector3<T> &v) const
{
    return x*v.x + y*v.y + z*v.z;
}

template <typename T>
inline Vector3<T> &Vector3<T>::operator *=(const T num)
{
    x*=num; y*=num; z*=num;
    return *this;
}

template <typename T>
inline Vector3<T> &Vector3<T>::operator /=(const T num)
{
    x /= num; y /= num; z /= num;
    return *this;
}

template <typename T>
inline Vector3<T> &Vector3<T>::operator -=(const Vector3<T> &v)
{
    x -= v.x; y -= v.y; z -= v.z;
    return *this;
}

template <typename T>
inline Vector3<T> &Vector3<T>::operator +=(const Vector3<T> &v)
{
    x+=v.x; y+=v.y; z+=v.z;
    return *this;
}

template <typename T>
inline Vector3<T> Vector3<T>::operator /(const T num) const
{
    return Vector3<T>(x/num, y/num, z/num);
}

template <typename T>
inline Vector3<T> Vector3<T>::operator *(const T num) const
{
    return Vector3<T>(x*num, y*num, z*num);
}

template <typename T>
inline Vector3<T> Vector3<T>::operator -(const Vector3<T> &v) const
{
    return Vector3<T>(x-v.x, y-v.y, z-v.z);
}

template <typename T>
inline Vector3<T> Vector3<T>::operator +(const Vector3<T> &v) const
{
    return Vector3<T>(x+v.x, y+v.y, z+v.z);
}

template <typename T>
inline Vector3<T> Vector3<T>::operator -(void) const
{
    return Vector3<T>(-x,-y,-z);
}
#endif

typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<float>                  Vector3f;
typedef Vector3<double>                 Vector3d;
