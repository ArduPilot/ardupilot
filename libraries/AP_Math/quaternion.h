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

#include <cmath>
#if MATH_CHECK_INDEXES
#include <assert.h>
#endif
#include <math.h>

class Quaternion {
public:
    float        q1, q2, q3, q4;

    // constructor creates a quaternion equivalent
    // to roll=0, pitch=0, yaw=0
    Quaternion()
    {
        q1 = 1;
        q2 = q3 = q4 = 0;
    }

    // setting constructor
    Quaternion(const float _q1, const float _q2, const float _q3, const float _q4) :
        q1(_q1), q2(_q2), q3(_q3), q4(_q4)
    {
    }

    // setting constructor
    Quaternion(const float _q[4]) :
        q1(_q[0]), q2(_q[1]), q3(_q[2]), q4(_q[3])
    {
    }

    // function call operator
    void operator()(const float _q1, const float _q2, const float _q3, const float _q4)
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

    // return the rotation matrix equivalent for this quaternion
    void        rotation_matrix(Matrix3f &m) const;

    // return the rotation matrix equivalent for this quaternion after normalization
    void        rotation_matrix_norm(Matrix3f &m) const;

    // return the rotation matrix equivalent for this quaternion
    void		from_rotation_matrix(const Matrix3f &m);

    // create a quaternion from a given rotation
    void        from_rotation(enum Rotation rotation);

    // rotate this quaternion by the given rotation
    void        rotate(enum Rotation rotation);

    // convert a vector from earth to body frame
    void        earth_to_body(Vector3f &v) const;

    // create a quaternion from Euler angles
    void        from_euler(float roll, float pitch, float yaw);

    // create a quaternion from Euler angles applied in yaw, roll, pitch order
    // instead of the normal yaw, pitch, roll order
    void        from_vector312(float roll, float pitch, float yaw);

    // convert this quaternion to a rotation vector where the direction of the vector represents
    // the axis of rotation and the length of the vector represents the angle of rotation
    void        to_axis_angle(Vector3f &v);

    // create a quaternion from a rotation vector where the direction of the vector represents
    // the axis of rotation and the length of the vector represents the angle of rotation
    void        from_axis_angle(Vector3f v);

    // create a quaternion from its axis-angle representation
    // the axis vector must be length 1. the rotation angle theta is in radians
    void        from_axis_angle(const Vector3f &axis, float theta);

    // rotate by the provided rotation vector
    void        rotate(const Vector3f &v);

    // create a quaternion from a rotation vector
    // only use with small angles.  I.e. length of v should less than 0.17 radians (i.e. 10 degrees)
    void        from_axis_angle_fast(Vector3f v);

    // create a quaternion from its axis-angle representation
    // the axis vector must be length 1, theta should less than 0.17 radians (i.e. 10 degrees)
    void        from_axis_angle_fast(const Vector3f &axis, float theta);

    // rotate by the provided rotation vector
    // only use with small angles.  I.e. length of v should less than 0.17 radians (i.e. 10 degrees)
    void        rotate_fast(const Vector3f &v);

    // get euler roll angle
    float       get_euler_roll() const;

    // get euler pitch angle
    float       get_euler_pitch() const;

    // get euler yaw angle
    float       get_euler_yaw() const;

    // create eulers from a quaternion
    void        to_euler(float &roll, float &pitch, float &yaw) const;

    // create eulers from a quaternion
    Vector3f    to_vector312(void) const;

    float length(void) const;
    void normalize();

    // initialise the quaternion to no rotation
    void initialise()
    {
        q1 = 1.0f;
        q2 = q3 = q4 = 0.0f;
    }

    Quaternion inverse(void) const;

    // reverse the rotation of this quaternion
    void invert();

    // allow a quaternion to be used as an array, 0 indexed
    float & operator[](uint8_t i)
    {
        float *_v = &q1;
#if MATH_CHECK_INDEXES
        assert(i < 4);
#endif
        return _v[i];
    }

    const float & operator[](uint8_t i) const
    {
        const float *_v = &q1;
#if MATH_CHECK_INDEXES
        assert(i < 4);
#endif
        return _v[i];
    }

    Quaternion operator*(const Quaternion &v) const;
    Quaternion &operator*=(const Quaternion &v);
    Quaternion operator/(const Quaternion &v) const;

    // angular difference between quaternions
    Quaternion angular_difference(const Quaternion &v) const;
};
