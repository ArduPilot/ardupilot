/*
 * quaternion.cpp
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

#pragma GCC optimize("O2")

#include "quaternion.h"
#include "AP_Math.h"
#include <AP_InternalError/AP_InternalError.h>
#include <AP_CustomRotations/AP_CustomRotations.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#define HALF_SQRT_2_PlUS_SQRT_2 0.92387953251128673848313610506011 // sqrt(2 + sqrt(2)) / 2
#define HALF_SQRT_2_MINUS_SQTR_2 0.38268343236508972626808144923416 // sqrt(2 - sqrt(2)) / 2
#define HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO 0.65328148243818828788676000840496 // sqrt((2 + sqrt(2))/2) / 2
#define HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO 0.27059805007309845059637609665515 // sqrt((2 - sqrt(2))/2) / 2

// return the rotation matrix equivalent for this quaternion
template <typename T>
void QuaternionT<T>::rotation_matrix(Matrix3d &m) const
{
    const T q3q3 = q3 * q3;
    const T q3q4 = q3 * q4;
    const T q2q2 = q2 * q2;
    const T q2q3 = q2 * q3;
    const T q2q4 = q2 * q4;
    const T q1q2 = q1 * q2;
    const T q1q3 = q1 * q3;
    const T q1q4 = q1 * q4;
    const T q4q4 = q4 * q4;

    m.a.x = 1.0f-2.0f*(q3q3 + q4q4);
    m.a.y = 2.0f*(q2q3 - q1q4);
    m.a.z = 2.0f*(q2q4 + q1q3);
    m.b.x = 2.0f*(q2q3 + q1q4);
    m.b.y = 1.0f-2.0f*(q2q2 + q4q4);
    m.b.z = 2.0f*(q3q4 - q1q2);
    m.c.x = 2.0f*(q2q4 - q1q3);
    m.c.y = 2.0f*(q3q4 + q1q2);
    m.c.z = 1.0f-2.0f*(q2q2 + q3q3);
}

// populate the supplied rotation matrix equivalent from this quaternion
template <typename T>
void QuaternionT<T>::rotation_matrix(Matrix3f &m) const
{
    const T q3q3 = q3 * q3;
    const T q3q4 = q3 * q4;
    const T q2q2 = q2 * q2;
    const T q2q3 = q2 * q3;
    const T q2q4 = q2 * q4;
    const T q1q2 = q1 * q2;
    const T q1q3 = q1 * q3;
    const T q1q4 = q1 * q4;
    const T q4q4 = q4 * q4;

    m.a.x = 1.0f-2.0f*(q3q3 + q4q4);
    m.a.y = 2.0f*(q2q3 - q1q4);
    m.a.z = 2.0f*(q2q4 + q1q3);
    m.b.x = 2.0f*(q2q3 + q1q4);
    m.b.y = 1.0f-2.0f*(q2q2 + q4q4);
    m.b.z = 2.0f*(q3q4 - q1q2);
    m.c.x = 2.0f*(q2q4 - q1q3);
    m.c.y = 2.0f*(q3q4 + q1q2);
    m.c.z = 1.0f-2.0f*(q2q2 + q3q3);
}

// make this quaternion equivalent to the supplied matrix
// Thanks to Martin John Baker
// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
template <typename T>
void QuaternionT<T>::from_rotation_matrix(const Matrix3<T> &m)
{
    const T &m00 = m.a.x;
    const T &m11 = m.b.y;
    const T &m22 = m.c.z;
    const T &m10 = m.b.x;
    const T &m01 = m.a.y;
    const T &m20 = m.c.x;
    const T &m02 = m.a.z;
    const T &m21 = m.c.y;
    const T &m12 = m.b.z;
    T &qw = q1;
    T &qx = q2;
    T &qy = q3;
    T &qz = q4;

    const T tr = m00 + m11 + m22;

    if (tr > 0) {
        const T S = sqrtF(tr+1) * 2;
        qw = 0.25f * S;
        qx = (m21 - m12) / S;
        qy = (m02 - m20) / S;
        qz = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)) {
        const T S = sqrtF(1.0f + m00 - m11 - m22) * 2.0f;
        qw = (m21 - m12) / S;
        qx = 0.25f * S;
        qy = (m01 + m10) / S;
        qz = (m02 + m20) / S;
    } else if (m11 > m22) {
        const T S = sqrtF(1.0f + m11 - m00 - m22) * 2.0f;
        qw = (m02 - m20) / S;
        qx = (m01 + m10) / S;
        qy = 0.25f * S;
        qz = (m12 + m21) / S;
    } else {
        const T S = sqrtF(1.0f + m22 - m00 - m11) * 2.0f;
        qw = (m10 - m01) / S;
        qx = (m02 + m20) / S;
        qy = (m12 + m21) / S;
        qz = 0.25f * S;
    }
}

// create a quaternion from a given rotation
template <typename T>
void QuaternionT<T>::from_rotation(enum Rotation rotation)
{
    // the constants below can be calculated using the following formula:
    //     Matrix3f m_from_rot;
    //     m_from_rot.from_rotation(rotation);
    //     Quaternion q_from_m;
    //     from_rotation_matrix(m_from_rot);

    switch (rotation) {
    case ROTATION_NONE:
        q1 = 1;
        q2 = q3 = q4 = 0;
        return;

    case ROTATION_YAW_45:
        q1 = HALF_SQRT_2_PlUS_SQRT_2;
        q2 = q3 = 0;
        q4 = HALF_SQRT_2_MINUS_SQTR_2;
        return;

    case ROTATION_YAW_90:
        q1 = HALF_SQRT_2;
        q2 = q3 = 0;
        q4 = HALF_SQRT_2;
        return;

    case ROTATION_YAW_135:
        q1 = HALF_SQRT_2_MINUS_SQTR_2;
        q2 = q3 = 0;
        q4 = HALF_SQRT_2_PlUS_SQRT_2;
        return;

    case ROTATION_YAW_180:
        q1 = q2 = q3 = 0;
        q4=1;
        return;

    case ROTATION_YAW_225:
        q1 = -HALF_SQRT_2_MINUS_SQTR_2;
        q2 = q3 = 0;
        q4 = HALF_SQRT_2_PlUS_SQRT_2;
        return;

    case ROTATION_YAW_270:
        q1 = HALF_SQRT_2;
        q2 = q3 = 0;
        q4 = -HALF_SQRT_2;
        return;

    case ROTATION_YAW_315:
        q1 = HALF_SQRT_2_PlUS_SQRT_2;
        q2 = q3 = 0;
        q4 = -HALF_SQRT_2_MINUS_SQTR_2;
        return;

    case ROTATION_ROLL_180:
        q1 = q3 = q4 = 0;
        q2 = 1;
        return;

    case ROTATION_ROLL_180_YAW_45:
        q1 = q4 = 0;
        q2 = HALF_SQRT_2_PlUS_SQRT_2;
        q3 = HALF_SQRT_2_MINUS_SQTR_2;
        return;

    case ROTATION_ROLL_180_YAW_90:
    case ROTATION_PITCH_180_YAW_270:
        q1 = q4 = 0;
        q2 = q3 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_180_YAW_135:
        q1 = q4 = 0;
        q2 = HALF_SQRT_2_MINUS_SQTR_2;
        q3 = HALF_SQRT_2_PlUS_SQRT_2;
        return;

    case ROTATION_PITCH_180:
        q1 = q2 = q4 = 0;
        q3 = 1;
        return;

    case ROTATION_ROLL_180_YAW_225:
        q1 = q4 = 0;
        q2 = -HALF_SQRT_2_MINUS_SQTR_2;
        q3 = HALF_SQRT_2_PlUS_SQRT_2;
        return;

    case ROTATION_ROLL_180_YAW_270:
    case ROTATION_PITCH_180_YAW_90:
        q1 = q4 = 0;
        q2 = -HALF_SQRT_2;
        q3 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_180_YAW_315:
        q1 = q4 = 0;
        q2 = HALF_SQRT_2_PlUS_SQRT_2;
        q3 = -HALF_SQRT_2_MINUS_SQTR_2;
        return;

    case ROTATION_ROLL_90:
        q1 = q2 = HALF_SQRT_2;
        q3 = q4 = 0;
        return;

    case ROTATION_ROLL_90_YAW_45:
        q1 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q2 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q3 = q4 = HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        return;

    case ROTATION_ROLL_90_YAW_90:
        q1 = q2 = q3 = q4 = 0.5f;
        return;

    case ROTATION_ROLL_90_YAW_135:
        q1 = q2 = HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        q3 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q4 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        return;

    case ROTATION_ROLL_270:
        q1 = HALF_SQRT_2;
        q2 = -HALF_SQRT_2;
        q3 = q4 = 0;
        return;

    case ROTATION_ROLL_270_YAW_45:
        q1 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q2 = -HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q3 = -HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        q4 = HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        return;

    case ROTATION_ROLL_270_YAW_90:
        q1 = q4 = 0.5f;
        q2 = q3 = -0.5f;
        return;

    case ROTATION_ROLL_270_YAW_135:
        q1 = HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        q2 = -HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        q3 = -HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q4 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        return;

    case ROTATION_PITCH_90:
        q1 = q3 = HALF_SQRT_2;
        q2 = q4 = 0;
        return;

    case ROTATION_PITCH_270:
        q1 = HALF_SQRT_2;
        q2 = q4 = 0;
        q3 = -HALF_SQRT_2;
        return;

    case ROTATION_ROLL_90_PITCH_90:
        q1 = q2 = q3 = -0.5f;
        q4 = 0.5f;
        return;

    case ROTATION_ROLL_180_PITCH_90:
        q1 = q3 = 0;
        q2 = -HALF_SQRT_2;
        q4 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_270_PITCH_90:
        q1 = q3 = q4 = 0.5f;
        q2 = -0.5f;
        return;

    case ROTATION_ROLL_90_PITCH_180:
        q1 = q2 = 0;
        q3 = -HALF_SQRT_2;
        q4 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_270_PITCH_180:
        q1 = q2 = 0;
        q3 = q4 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_90_PITCH_270:
        q1 = q2 = q4 = 0.5f;
        q3 = -0.5;
        return;

    case ROTATION_ROLL_180_PITCH_270:
        q1 = q3 = 0;
        q2 = q4 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_270_PITCH_270:
        q1 = -0.5f;
        q2 = q3 = q4 = 0.5f;
        return;

    case ROTATION_ROLL_90_PITCH_180_YAW_90:
        q1 = q3 = -0.5f;
        q2 = q4 = 0.5f;
        return;

    case ROTATION_ROLL_90_YAW_270:
        q1 = q2 = -0.5f;
        q3 = q4 = 0.5f;
        return;

    case ROTATION_ROLL_90_PITCH_68_YAW_293:
        q1 = 0.26774500501681575137524760066299;
        q2 = 0.70698804688952421315661922562867;
        q3 = 0.012957683254962659713527273197542;
        q4 = -0.65445596665363614530264158020145;
        return;

    case ROTATION_PITCH_315:
        q1 = HALF_SQRT_2_PlUS_SQRT_2;
        q2 = q4 = 0;
        q3 = -HALF_SQRT_2_MINUS_SQTR_2;
        return;

    case ROTATION_ROLL_90_PITCH_315:
        q1 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q2 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q3 = -HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        q4 = HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        return;

    case ROTATION_PITCH_7:
        q1 = 0.99813479842186692003735970502021;
        q2 = q4 = 0;
        q3 = 0.061048539534856872956769535676358;
        return;

    case ROTATION_ROLL_45:
        q1 = HALF_SQRT_2_PlUS_SQRT_2;
        q2 = HALF_SQRT_2_MINUS_SQTR_2;
        q3 = q4 = 0.0;
        return;

    case ROTATION_ROLL_315:
        q1 = HALF_SQRT_2_PlUS_SQRT_2;
        q2 = -HALF_SQRT_2_MINUS_SQTR_2;
        q3 = q4 = 0.0;
        return;

    case ROTATION_CUSTOM_1:
    case ROTATION_CUSTOM_2:
#if !APM_BUILD_TYPE(APM_BUILD_AP_Periph)
        // Do not support custom rotations on Periph
        AP::custom_rotations().from_rotation(rotation, *this);
        return;
#endif
    case ROTATION_MAX:
    case ROTATION_CUSTOM_OLD:
    case ROTATION_CUSTOM_END:
        break;
    }
    // rotation invalid
    INTERNAL_ERROR(AP_InternalError::error_t::bad_rotation);
}

// rotate this quaternion by the given rotation
template <typename T>
void QuaternionT<T>::rotate(enum Rotation rotation)
{
    // create quaternion from rotation matrix
    QuaternionT<T> q_from_rot;
    q_from_rot.from_rotation(rotation);

    // rotate this quaternion
    *this *= q_from_rot;
}

// convert a vector from earth to body frame
template <typename T>
void QuaternionT<T>::earth_to_body(Vector3<T> &v) const
{
    Matrix3<T> m;
    rotation_matrix(m);
    v = m * v;
}

// create a quaternion from Euler angles
template <typename T>
void QuaternionT<T>::from_euler(T roll, T pitch, T yaw)
{
    const T cr2 = cosF(roll*0.5);
    const T cp2 = cosF(pitch*0.5);
    const T cy2 = cosF(yaw*0.5);
    const T sr2 = sinF(roll*0.5);
    const T sp2 = sinF(pitch*0.5);
    const T sy2 = sinF(yaw*0.5);

    q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}
template <typename T>
void QuaternionT<T>::from_euler(const Vector3<T> &v)
{
    from_euler(v[0], v[1], v[2]);
}

// create a quaternion from Euler angles applied in yaw, roll, pitch order
// instead of the normal yaw, pitch, roll order
template <typename T>
void QuaternionT<T>::from_vector312(T roll, T pitch, T yaw)
{
    Matrix3<T> m;
    m.from_euler312(roll, pitch, yaw);

    from_rotation_matrix(m);
}

// create a quaternion from its axis-angle representation
template <typename T>
void QuaternionT<T>::from_axis_angle(Vector3<T> v)
{
    const T theta = v.length();
    if (::is_zero(theta)) {
        q1 = 1.0f;
        q2=q3=q4=0.0f;
        return;
    }
    v /= theta;
    from_axis_angle(v,theta);
}

// create a quaternion from its axis-angle representation
// the axis vector must be length 1, theta is in radians
template <typename T>
void QuaternionT<T>::from_axis_angle(const Vector3<T> &axis, T theta)
{
    // axis must be a unit vector as there is no check for length
    if (::is_zero(theta)) {
        q1 = 1.0f;
        q2=q3=q4=0.0f;
        return;
    }
    const T st2 = sinF(0.5*theta);

    q1 = cosF(0.5*theta);
    q2 = axis.x * st2;
    q3 = axis.y * st2;
    q4 = axis.z * st2;
}

// rotate by the provided axis angle
template <typename T>
void QuaternionT<T>::rotate(const Vector3<T> &v)
{
    QuaternionT<T> r;
    r.from_axis_angle(v);
    (*this) *= r;
}

// convert this quaternion to a rotation vector where the direction of the vector represents
// the axis of rotation and the length of the vector represents the angle of rotation
template <typename T>
void QuaternionT<T>::to_axis_angle(Vector3<T> &v) const
{
    const T l = sqrtF(sq(q2)+sq(q3)+sq(q4));
    v = Vector3<T>(q2,q3,q4);
    if (!::is_zero(l)) {
        v /= l;
        v *= wrap_PI(2.0f * atan2F(l,q1));
    }
}

// create a quaternion from its axis-angle representation
// only use with small angles.  I.e. length of v should less than 0.17 radians (i.e. 10 degrees)
template <typename T>
void QuaternionT<T>::from_axis_angle_fast(Vector3<T> v)
{
    const T theta = v.length();
    if (::is_zero(theta)) {
        q1 = 1.0f;
        q2=q3=q4=0.0f;
        return;
    }
    v /= theta;
    from_axis_angle_fast(v,theta);
}

// create a quaternion from its axis-angle representation
// theta should less than 0.17 radians (i.e. 10 degrees)
template <typename T>
void QuaternionT<T>::from_axis_angle_fast(const Vector3<T> &axis, T theta)
{
    const T t2 = 0.5*theta;
    const T sqt2 = sq(t2);
    const T st2 = t2-sqt2*t2/6.0f;

    q1 = 1.0f-(0.5*sqt2)+sq(sqt2)/24.0f;
    q2 = axis.x * st2;
    q3 = axis.y * st2;
    q4 = axis.z * st2;
}

// rotate by the provided axis angle
// only use with small angles.  I.e. length of v should less than 0.17 radians (i.e. 10 degrees)
template <typename T>
void QuaternionT<T>::rotate_fast(const Vector3<T> &v)
{
    const T theta = v.length();
    if (::is_zero(theta)) {
        return;
    }
    const T t2 = 0.5*theta;
    const T sqt2 = sq(t2);
    T st2 = t2-sqt2*t2/6.0f;
    st2 /= theta;

    //"rotation quaternion"
    const T w2 = 1.0f-(0.5*sqt2)+sq(sqt2)/24.0f;
    const T x2 = v.x * st2;
    const T y2 = v.y * st2;
    const T z2 = v.z * st2;

    //copy our quaternion
    const T w1 = q1;
    const T x1 = q2;
    const T y1 = q3;
    const T z1 = q4;

    //do the multiply into our quaternion
    q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

// get euler roll angle
template <typename T>
T QuaternionT<T>::get_euler_roll() const
{
    return (atan2F(2.0f*(q1*q2 + q3*q4), 1.0f - 2.0f*(q2*q2 + q3*q3)));
}

// get euler pitch angle
template <typename T>
T QuaternionT<T>::get_euler_pitch() const
{
    return safe_asin(2.0f*(q1*q3 - q4*q2));
}

// get euler yaw angle
template <typename T>
T QuaternionT<T>::get_euler_yaw() const
{
    return atan2F(2.0f*(q1*q4 + q2*q3), 1.0f - 2.0f*(q3*q3 + q4*q4));
}

// create eulers from a quaternion
template <typename T>
void QuaternionT<T>::to_euler(double &roll, double &pitch, double &yaw) const
{
    roll = get_euler_roll();
    pitch = get_euler_pitch();
    yaw = get_euler_yaw();
}

template <typename T>
void QuaternionT<T>::to_euler(float &roll, float &pitch, float &yaw) const
{
    roll = get_euler_roll();
    pitch = get_euler_pitch();
    yaw = get_euler_yaw();
}

// create eulers from a quaternion
template <typename T>
Vector3<T> QuaternionT<T>::to_vector312(void) const
{
    Matrix3<T> m;
    rotation_matrix(m);
    return m.to_euler312();
}

template <typename T>
T QuaternionT<T>::length(void) const
{
    return sqrtF(sq(q1) + sq(q2) + sq(q3) + sq(q4));
}

// gets the length squared of the quaternion
template <typename T>
T QuaternionT<T>::length_squared() const
{
    return (T)(q1*q1 + q2*q2 + q3*q3 + q4*q4);
}

// return the reverse rotation of this quaternion
template <typename T>
QuaternionT<T> QuaternionT<T>::inverse(void) const
{
    return QuaternionT<T>(q1, -q2, -q3, -q4);
}

// reverse the rotation of this quaternion
template <typename T>
void QuaternionT<T>::invert()
{
    q2 = -q2;
    q3 = -q3;
    q4 = -q4;
}

template <typename T>
void QuaternionT<T>::normalize(void)
{
    const T quatMag = length();
    if (!::is_zero(quatMag)) {
        const T quatMagInv = 1.0f/quatMag;
        q1 *= quatMagInv;
        q2 *= quatMagInv;
        q3 *= quatMagInv;
        q4 *= quatMagInv;
    } else {
        // The code goes here if the quaternion is [0,0,0,0]. This shouldn't happen.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
}

// Checks if each element of the quaternion is zero
template <typename T>
bool QuaternionT<T>::is_zero(void) const {
    return ::is_zero(q1) && ::is_zero(q2) && ::is_zero(q3) && ::is_zero(q4);
}

// zeros the quaternion to [0, 0, 0, 0], an invalid quaternion
// See initialize() if you want the zero rotation quaternion
template <typename T>
void QuaternionT<T>::zero(void)
{
    q1 = q2 = q3 = q4 = 0.0;
}

// Checks if the quaternion is unit_length within a tolerance
// Returns True: if its magnitude is close to unit length +/- 1E-3
// This limit is somewhat greater than sqrt(FLT_EPSL)
template <typename T>
bool QuaternionT<T>::is_unit_length(void) const
{
    if (fabsF(length_squared() - 1) < 1E-3) {
        return true;
    }

    return false;
}

template <typename T>
QuaternionT<T> QuaternionT<T>::operator*(const QuaternionT<T> &v) const
{
    QuaternionT<T> ret;
    const T &w1 = q1;
    const T &x1 = q2;
    const T &y1 = q3;
    const T &z1 = q4;

    const T w2 = v.q1;
    const T x2 = v.q2;
    const T y2 = v.q3;
    const T z2 = v.q4;

    ret.q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    ret.q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    ret.q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    ret.q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;

    return ret;
}

// Optimized quaternion rotation operator, equivalent to converting
// (*this) to a rotation matrix then multiplying it to the argument `v`.
//
// 15 multiplies and 15 add / subtracts. Caches 3 floats
template <typename T>
Vector3<T> QuaternionT<T>::operator*(const Vector3<T> &v) const
{
    // This uses the formula
    //
    //    v2 = v1 + 2 q1 * qv x v1 + 2 qv x qv x v1
    //
    // where "x" is the cross product (explicitly inlined for performance below), 
    // "q1" is the scalar part and "qv" is the vector part of this quaternion

    Vector3<T> ret = v;

    // Compute and cache "qv x v1"
    T uv[] = {q3 * v.z - q4 * v.y, q4 * v.x - q2 * v.z, q2 * v.y - q3 * v.x};

    uv[0] += uv[0];
    uv[1] += uv[1];
    uv[2] += uv[2];
    ret.x += q1 * uv[0] + q3 * uv[2] - q4 * uv[1];
    ret.y += q1 * uv[1] + q4 * uv[0] - q2 * uv[2];
    ret.z += q1 * uv[2] + q2 * uv[1] - q3 * uv[0];
    return ret;
}

template <typename T>
QuaternionT<T> &QuaternionT<T>::operator*=(const QuaternionT<T> &v)
{
    const T w1 = q1;
    const T x1 = q2;
    const T y1 = q3;
    const T z1 = q4;

    const T w2 = v.q1;
    const T x2 = v.q2;
    const T y2 = v.q3;
    const T z2 = v.q4;

    q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;

    return *this;
}

template <typename T>
QuaternionT<T> QuaternionT<T>::operator/(const QuaternionT<T> &v) const
{
    QuaternionT<T> ret;
    const T &quat0 = q1;
    const T &quat1 = q2;
    const T &quat2 = q3;
    const T &quat3 = q4;

    if (is_zero()) {
        // The code goes here if the quaternion is [0,0,0,0]. This shouldn't happen.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    const T rquat0 = v.q1;
    const T rquat1 = v.q2;
    const T rquat2 = v.q3;
    const T rquat3 = v.q4;

    ret.q1 = (rquat0*quat0 + rquat1*quat1 + rquat2*quat2 + rquat3*quat3);
    ret.q2 = (rquat0*quat1 - rquat1*quat0 - rquat2*quat3 + rquat3*quat2);
    ret.q3 = (rquat0*quat2 + rquat1*quat3 - rquat2*quat0 - rquat3*quat1);
    ret.q4 = (rquat0*quat3 - rquat1*quat2 + rquat2*quat1 - rquat3*quat0);
    return ret;
}

// angular difference in radians between quaternions
template <typename T>
QuaternionT<T> QuaternionT<T>::angular_difference(const QuaternionT<T> &v) const
{
    return v.inverse() * *this;
}

// absolute (e.g. always positive) earth-frame roll-pitch difference (in radians) between this Quaternion and another
template <typename T>
T QuaternionT<T>::roll_pitch_difference(const QuaternionT<T> &v) const
{
    // convert Quaternions to rotation matrices
    Matrix3<T> m, vm;
    rotation_matrix(m);
    v.rotation_matrix(vm);

    // rotate earth frame vertical vector by each rotation matrix
    const Vector3<T> z_unit_vec{0,0,1};
    const Vector3<T> z_unit_m = m.mul_transpose(z_unit_vec);
    const Vector3<T> z_unit_vm = vm.mul_transpose(z_unit_vec);
    const Vector3<T> vec_diff = z_unit_vm - z_unit_m;
    const T vec_len_div2 = constrain_float(vec_diff.length() * 0.5, 0.0, 1.0);

    // calculate and return angular difference
    return (2.0 * asinF(vec_len_div2));
}

// define for float and double
template class QuaternionT<float>;
template class QuaternionT<double>;
