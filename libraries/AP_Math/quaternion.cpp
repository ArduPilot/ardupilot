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

#include "AP_Math.h"

// return the rotation matrix equivalent for this quaternion
void Quaternion::rotation_matrix(Matrix3f &m) const
{
    const float q3q3 = q3 * q3;
    const float q3q4 = q3 * q4;
    const float q2q2 = q2 * q2;
    const float q2q3 = q2 * q3;
    const float q2q4 = q2 * q4;
    const float q1q2 = q1 * q2;
    const float q1q3 = q1 * q3;
    const float q1q4 = q1 * q4;
    const float q4q4 = q4 * q4;

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

// return the rotation matrix equivalent for this quaternion after normalization
void Quaternion::rotation_matrix_norm(Matrix3f &m) const
{
    const float q1q1 = q1 * q1;
    const float q1q2 = q1 * q2;
    const float q1q3 = q1 * q3;
    const float q1q4 = q1 * q4;
    const float q2q2 = q2 * q2;
    const float q2q3 = q2 * q3;
    const float q2q4 = q2 * q4;
    const float q3q3 = q3 * q3;
    const float q3q4 = q3 * q4;
    const float q4q4 = q4 * q4;
    const float invs = 1.0f / (q1q1 + q2q2 + q3q3 + q4q4);

    m.a.x = ( q2q2 - q3q3 - q4q4 + q1q1)*invs;
    m.a.y = 2.0f*(q2q3 - q1q4)*invs;
    m.a.z = 2.0f*(q2q4 + q1q3)*invs;
    m.b.x = 2.0f*(q2q3 + q1q4)*invs;
    m.b.y = (-q2q2 + q3q3 - q4q4 + q1q1)*invs;
    m.b.z = 2.0f*(q3q4 - q1q2)*invs;
    m.c.x = 2.0f*(q2q4 - q1q3)*invs;
    m.c.y = 2.0f*(q3q4 + q1q2)*invs;
    m.c.z = (-q2q2 - q3q3 + q4q4 + q1q1)*invs;
}

// return the rotation matrix equivalent for this quaternion
// Thanks to Martin John Baker
// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
void Quaternion::from_rotation_matrix(const Matrix3f &m)
{
    const float &m00 = m.a.x;
    const float &m11 = m.b.y;
    const float &m22 = m.c.z;
    const float &m10 = m.b.x;
    const float &m01 = m.a.y;
    const float &m20 = m.c.x;
    const float &m02 = m.a.z;
    const float &m21 = m.c.y;
    const float &m12 = m.b.z;
    float &qw = q1;
    float &qx = q2;
    float &qy = q3;
    float &qz = q4;

    const float tr = m00 + m11 + m22;

    if (tr > 0) {
        const float S = sqrtf(tr+1) * 2;
        qw = 0.25f * S;
        qx = (m21 - m12) / S;
        qy = (m02 - m20) / S;
        qz = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)) {
        const float S = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        qw = (m21 - m12) / S;
        qx = 0.25f * S;
        qy = (m01 + m10) / S;
        qz = (m02 + m20) / S;
    } else if (m11 > m22) {
        const float S = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        qw = (m02 - m20) / S;
        qx = (m01 + m10) / S;
        qy = 0.25f * S;
        qz = (m12 + m21) / S;
    } else {
        const float S = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        qw = (m10 - m01) / S;
        qx = (m02 + m20) / S;
        qy = (m12 + m21) / S;
        qz = 0.25f * S;
    }
}

// convert a vector from earth to body frame
void Quaternion::earth_to_body(Vector3f &v) const
{
    Matrix3f m;
    rotation_matrix(m);
    v = m * v;
}

// create a quaternion from Euler angles
void Quaternion::from_euler(float roll, float pitch, float yaw)
{
    const float cr2 = cosf(roll*0.5f);
    const float cp2 = cosf(pitch*0.5f);
    const float cy2 = cosf(yaw*0.5f);
    const float sr2 = sinf(roll*0.5f);
    const float sp2 = sinf(pitch*0.5f);
    const float sy2 = sinf(yaw*0.5f);

    q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}

// create a quaternion from Euler angles
void Quaternion::from_vector312(float roll ,float pitch, float yaw)
{
    Matrix3f m;
    m.from_euler312(roll, pitch, yaw);

    from_rotation_matrix(m);
}

void Quaternion::from_axis_angle(Vector3f v)
{
    const float theta = v.length();
    if (is_zero(theta)) {
        q1 = 1.0f;
        q2=q3=q4=0.0f;
        return;
    }
    v /= theta;
    from_axis_angle(v,theta);
}

void Quaternion::from_axis_angle(const Vector3f &axis, float theta)
{
    // axis must be a unit vector as there is no check for length
    if (is_zero(theta)) {
        q1 = 1.0f;
        q2=q3=q4=0.0f;
        return;
    }
    const float st2 = sinf(theta/2.0f);

    q1 = cosf(theta/2.0f);
    q2 = axis.x * st2;
    q3 = axis.y * st2;
    q4 = axis.z * st2;
}

void Quaternion::rotate(const Vector3f &v)
{
    Quaternion r;
    r.from_axis_angle(v);
    (*this) *= r;
}

void Quaternion::to_axis_angle(Vector3f &v)
{
    const float l = sqrtf(sq(q2)+sq(q3)+sq(q4));
    v = Vector3f(q2,q3,q4);
    if (!is_zero(l)) {
        v /= l;
        v *= wrap_PI(2.0f * atan2f(l,q1));
    }
}

void Quaternion::from_axis_angle_fast(Vector3f v)
{
    const float theta = v.length();
    if (is_zero(theta)) {
        q1 = 1.0f;
        q2=q3=q4=0.0f;
        return;
    }
    v /= theta;
    from_axis_angle_fast(v,theta);
}

void Quaternion::from_axis_angle_fast(const Vector3f &axis, float theta)
{
    const float t2 = theta/2.0f;
    const float sqt2 = sq(t2);
    const float st2 = t2-sqt2*t2/6.0f;

    q1 = 1.0f-(sqt2/2.0f)+sq(sqt2)/24.0f;
    q2 = axis.x * st2;
    q3 = axis.y * st2;
    q4 = axis.z * st2;
}

void Quaternion::rotate_fast(const Vector3f &v)
{
    const float theta = v.length();
    if (is_zero(theta)) {
        return;
    }
    const float t2 = theta/2.0f;
    const float sqt2 = sq(t2);
    float st2 = t2-sqt2*t2/6.0f;
    st2 /= theta;

    //"rotation quaternion"
    const float w2 = 1.0f-(sqt2/2.0f)+sq(sqt2)/24.0f;
    const float x2 = v.x * st2;
    const float y2 = v.y * st2;
    const float z2 = v.z * st2;

    //copy our quaternion
    const float w1 = q1;
    const float x1 = q2;
    const float y1 = q3;
    const float z1 = q4;

    //do the multiply into our quaternion
    q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

// get euler roll angle
float Quaternion::get_euler_roll() const
{
    return (atan2f(2.0f*(q1*q2 + q3*q4), 1.0f - 2.0f*(q2*q2 + q3*q3)));
}

// get euler pitch angle
float Quaternion::get_euler_pitch() const
{
    return safe_asin(2.0f*(q1*q3 - q4*q2));
}

// get euler yaw angle
float Quaternion::get_euler_yaw() const
{
    return atan2f(2.0f*(q1*q4 + q2*q3), 1.0f - 2.0f*(q3*q3 + q4*q4));
}

// create eulers from a quaternion
void Quaternion::to_euler(float &roll, float &pitch, float &yaw) const
{
    roll = get_euler_roll();
    pitch = get_euler_pitch();
    yaw = get_euler_yaw();
}

// create eulers from a quaternion
Vector3f Quaternion::to_vector312(void) const
{
    Matrix3f m;
    rotation_matrix(m);
    return m.to_euler312();
}

float Quaternion::length(void) const
{
    return sqrtf(sq(q1) + sq(q2) + sq(q3) + sq(q4));
}

Quaternion Quaternion::inverse(void) const
{
    return Quaternion(q1, -q2, -q3, -q4);
}

void Quaternion::normalize(void)
{
    const float quatMag = length();
    if (!is_zero(quatMag)) {
        const float quatMagInv = 1.0f/quatMag;
        q1 *= quatMagInv;
        q2 *= quatMagInv;
        q3 *= quatMagInv;
        q4 *= quatMagInv;
    }
}

Quaternion Quaternion::operator*(const Quaternion &v) const
{
    Quaternion ret;
    const float &w1 = q1;
    const float &x1 = q2;
    const float &y1 = q3;
    const float &z1 = q4;

    const float w2 = v.q1;
    const float x2 = v.q2;
    const float y2 = v.q3;
    const float z2 = v.q4;

    ret.q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    ret.q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    ret.q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    ret.q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;

    return ret;
}

Quaternion &Quaternion::operator*=(const Quaternion &v)
{
    const float w1 = q1;
    const float x1 = q2;
    const float y1 = q3;
    const float z1 = q4;

    const float w2 = v.q1;
    const float x2 = v.q2;
    const float y2 = v.q3;
    const float z2 = v.q4;

    q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;

    return *this;
}

Quaternion Quaternion::operator/(const Quaternion &v) const
{
    Quaternion ret;
    const float &quat0 = q1;
    const float &quat1 = q2;
    const float &quat2 = q3;
    const float &quat3 = q4;

    const float rquat0 = v.q1;
    const float rquat1 = v.q2;
    const float rquat2 = v.q3;
    const float rquat3 = v.q4;

    ret.q1 = (rquat0*quat0 + rquat1*quat1 + rquat2*quat2 + rquat3*quat3);
    ret.q2 = (rquat0*quat1 - rquat1*quat0 - rquat2*quat3 + rquat3*quat2);
    ret.q3 = (rquat0*quat2 + rquat1*quat3 - rquat2*quat0 - rquat3*quat1);
    ret.q4 = (rquat0*quat3 - rquat1*quat2 + rquat2*quat1 - rquat3*quat0);
    return ret;
}

// angular difference in radians between quaternions
Quaternion Quaternion::angular_difference(const Quaternion &v) const
{
    return v.inverse() * *this;
}
