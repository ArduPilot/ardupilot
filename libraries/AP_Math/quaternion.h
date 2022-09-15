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

// Copyright 2012 Andrew Tridgell, all rights reserved.
// Refactored by Jonathan Challinger
#pragma once

#include "definitions.h"
#include "matrix3.h"
#include <cmath>
#if MATH_CHECK_INDEXES
#include <assert.h>
#endif
#include <math.h>

template <typename T>
class QuaternionT {
public:
    T        q1, q2, q3, q4;

    // constructor creates a quaternion equivalent
    // to roll=0, pitch=0, yaw=0
    QuaternionT()
    {
        q1 = 1;
        q2 = q3 = q4 = 0;
    }

    // setting constructor
    QuaternionT(const T _q1, const T _q2, const T _q3, const T _q4) :
        q1(_q1), q2(_q2), q3(_q3), q4(_q4)
    {
    }

    // setting constructor
    QuaternionT(const T _q[4]) :
        q1(_q[0]), q2(_q[1]), q3(_q[2]), q4(_q[3])
    {
    }

    // function call operator
    void operator()(const T _q1, const T _q2, const T _q3, const T _q4)
    {
        q1 = _q1;
        q2 = _q2;
        q3 = _q3;
        q4 = _q4;
    }

    // check if any elements are NAN
    bool        is_nan(void) const WARN_IF_UNUSED
    {
        return isnan(q1) || isnan(q2) || isnan(q3) || isnan(q4);
    }

    // populate the supplied rotation matrix equivalent from this quaternion
    void        rotation_matrix(Matrix3f &m) const;
    void        rotation_matrix(Matrix3d &m) const;

    // make this quaternion equivalent to the supplied matrix
    void		from_rotation_matrix(const Matrix3<T> &m);

    // create a quaternion from a given rotation
    void        from_rotation(enum Rotation rotation);

    // rotate this quaternion by the given rotation
    void        rotate(enum Rotation rotation);

    // convert a vector from earth to body frame
    void        earth_to_body(Vector3<T> &v) const;

    // create a quaternion from Euler angles
    void        from_euler(T roll, T pitch, T yaw);
    void        from_euler(const Vector3<T> &v);

    // create a quaternion from Euler angles applied in yaw, roll, pitch order
    // instead of the normal yaw, pitch, roll order
    void        from_vector312(T roll, T pitch, T yaw);

    // convert this quaternion to a rotation vector where the direction of the vector represents
    // the axis of rotation and the length of the vector represents the angle of rotation
    void        to_axis_angle(Vector3<T> &v) const;

    // create a quaternion from a rotation vector where the direction of the vector represents
    // the axis of rotation and the length of the vector represents the angle of rotation
    void        from_axis_angle(Vector3<T> v);

    // create a quaternion from its axis-angle representation
    // the axis vector must be length 1. the rotation angle theta is in radians
    void        from_axis_angle(const Vector3<T> &axis, T theta);

    // rotate by the provided rotation vector
    void        rotate(const Vector3<T> &v);

    // create a quaternion from a rotation vector
    // only use with small angles.  I.e. length of v should less than 0.17 radians (i.e. 10 degrees)
    void        from_axis_angle_fast(Vector3<T> v);

    // create a quaternion from its axis-angle representation
    // the axis vector must be length 1, theta should less than 0.17 radians (i.e. 10 degrees)
    void        from_axis_angle_fast(const Vector3<T> &axis, T theta);

    void        from_angular_velocity(const Vector3<T>& angular_velocity, float time_delta);

    // rotate by the provided rotation vector
    // only use with small angles.  I.e. length of v should less than 0.17 radians (i.e. 10 degrees)
    void        rotate_fast(const Vector3<T> &v);

    // get euler roll angle
    T       get_euler_roll() const;

    // get euler pitch angle
    T       get_euler_pitch() const;

    // get euler yaw angle
    T       get_euler_yaw() const;

    // create eulers from a quaternion
    void        to_euler(float &roll, float &pitch, float &yaw) const;
    void        to_euler(double &roll, double &pitch, double &yaw) const;

    // create eulers from a quaternion
    Vector3<T>    to_vector312(void) const;

    T length_squared(void) const;
    T length(void) const;
    void normalize();

    // Checks if each element of the quaternion is zero
    bool is_zero(void) const;

    // zeros the quaternion to [0, 0, 0, 0], an invalid quaternion
    // See initialize() if you want the zero rotation quaternion
    void zero(void);

    // Checks if the quaternion is unit_length within a tolerance
    // Returns True: if its magnitude is close to unit length +/- 1E-3
    // This limit is somewhat greater than sqrt(FLT_EPSL)
    bool is_unit_length(void) const;

    // initialise the quaternion to no rotation
    void initialise()
    {
        q1 = 1.0f;
        q2 = q3 = q4 = 0.0f;
    }

    QuaternionT<T> inverse(void) const;

    // reverse the rotation of this quaternion
    void invert();

    // allow a quaternion to be used as an array, 0 indexed
    T & operator[](uint8_t i)
    {
        T *_v = &q1;
#if MATH_CHECK_INDEXES
        assert(i < 4);
#endif
        return _v[i];
    }

    const T & operator[](uint8_t i) const
    {
        const T *_v = &q1;
#if MATH_CHECK_INDEXES
        assert(i < 4);
#endif
        return _v[i];
    }

    QuaternionT<T> operator*(const QuaternionT<T> &v) const;
    Vector3<T> operator*(const Vector3<T> &v) const;
    QuaternionT<T> &operator*=(const QuaternionT<T> &v);
    QuaternionT<T> operator/(const QuaternionT<T> &v) const;

    // angular difference between quaternions
    QuaternionT<T> angular_difference(const QuaternionT<T> &v) const;

    // absolute (e.g. always positive) earth-frame roll-pitch difference (in radians) between this Quaternion and another
    T roll_pitch_difference(const QuaternionT<T> &v) const;

    // double/float conversion
    QuaternionT<double> todouble(void) const {
        return QuaternionT<double>(q1,q2,q3,q4);
    }
    QuaternionT<float> tofloat(void) const {
        return QuaternionT<float>(q1,q2,q3,q4);
    }
};

typedef QuaternionT<float> Quaternion;
typedef QuaternionT<double> QuaternionD;



