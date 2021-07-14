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

#pragma GCC optimize("O2")

#include "AP_Math.h"
#include <AP_InternalError/AP_InternalError.h>

// rotate a vector by a standard rotation, attempting
// to use the minimum number of floating point operations
template <typename T>
void Vector3<T>::rotate(enum Rotation rotation)
{
    T tmp;
    switch (rotation) {
    case ROTATION_NONE:
        return;
    case ROTATION_YAW_45: {
        tmp = HALF_SQRT_2*(ftype)(x - y);
        y   = HALF_SQRT_2*(ftype)(x + y);
        x = tmp;
        return;
    }
    case ROTATION_YAW_90: {
        tmp = x; x = -y; y = tmp;
        return;
    }
    case ROTATION_YAW_135: {
        tmp = -HALF_SQRT_2*(ftype)(x + y);
        y   =  HALF_SQRT_2*(ftype)(x - y);
        x = tmp;
        return;
    }
    case ROTATION_YAW_180:
        x = -x; y = -y;
        return;
    case ROTATION_YAW_225: {
        tmp = HALF_SQRT_2*(ftype)(y - x);
        y   = -HALF_SQRT_2*(ftype)(x + y);
        x = tmp;
        return;
    }
    case ROTATION_YAW_270: {
        tmp = x; x = y; y = -tmp;
        return;
    }
    case ROTATION_YAW_315: {
        tmp = HALF_SQRT_2*(ftype)(x + y);
        y   = HALF_SQRT_2*(ftype)(y - x);
        x = tmp;
        return;
    }
    case ROTATION_ROLL_180: {
        y = -y; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_45: {
        tmp = HALF_SQRT_2*(ftype)(x + y);
        y   = HALF_SQRT_2*(ftype)(x - y);
        x = tmp; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_90: {
        tmp = x; x = y; y = tmp; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_135: {
        tmp = HALF_SQRT_2*(ftype)(y - x);
        y   = HALF_SQRT_2*(ftype)(y + x);
        x = tmp; z = -z;
        return;
    }
    case ROTATION_PITCH_180: {
        x = -x; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_225: {
        tmp = -HALF_SQRT_2*(ftype)(x + y);
        y   =  HALF_SQRT_2*(ftype)(y - x);
        x = tmp; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_270: {
        tmp = x; x = -y; y = -tmp; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_315: {
        tmp =  HALF_SQRT_2*(ftype)(x - y);
        y   = -HALF_SQRT_2*(ftype)(x + y);
        x = tmp; z = -z;
        return;
    }
    case ROTATION_ROLL_90: {
        tmp = z; z = y; y = -tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_45: {
        tmp = z; z = y; y = -tmp;
        tmp = HALF_SQRT_2*(ftype)(x - y);
        y   = HALF_SQRT_2*(ftype)(x + y);
        x = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_90: {
        tmp = z; z = y; y = -tmp;
        tmp = x; x = -y; y = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_135: {
        tmp = z; z = y; y = -tmp;
        tmp = -HALF_SQRT_2*(ftype)(x + y);
        y   =  HALF_SQRT_2*(ftype)(x - y);
        x = tmp;
        return;
    }
    case ROTATION_ROLL_270: {
        tmp = z; z = -y; y = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_45: {
        tmp = z; z = -y; y = tmp;
        tmp = HALF_SQRT_2*(ftype)(x - y);
        y   = HALF_SQRT_2*(ftype)(x + y);
        x = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_90: {
        tmp = z; z = -y; y = tmp;
        tmp = x; x = -y; y = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_135: {
        tmp = z; z = -y; y = tmp;
        tmp = -HALF_SQRT_2*(ftype)(x + y);
        y   =  HALF_SQRT_2*(ftype)(x - y);
        x = tmp;
        return;
    }
    case ROTATION_PITCH_90: {
        tmp = z; z = -x; x = tmp;
        return;
    }
    case ROTATION_PITCH_270: {
        tmp = z; z = x; x = -tmp;
        return;
    }
    case ROTATION_PITCH_180_YAW_90: {
        z = -z;
        tmp = -x; x = -y; y = tmp;
        return;
    }
    case ROTATION_PITCH_180_YAW_270: {
        x = -x; z = -z;
        tmp = x; x = y; y = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_90: {
        tmp = z; z = y; y = -tmp;
        tmp = z; z = -x; x = tmp;
        return;
    }
    case ROTATION_ROLL_180_PITCH_90: {
        y = -y; z = -z;
        tmp = z; z = -x; x = tmp;
        return;
    }
    case ROTATION_ROLL_270_PITCH_90: {
        tmp = z; z = -y; y = tmp;
        tmp = z; z = -x; x = tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_180: {
        tmp = z; z = y; y = -tmp;
        x = -x; z = -z;
        return;
    }
    case ROTATION_ROLL_270_PITCH_180: {
        tmp = z; z = -y; y = tmp;
        x = -x; z = -z;
        return;
    }
    case ROTATION_ROLL_90_PITCH_270: {
        tmp = z; z = y; y = -tmp;
        tmp = z; z = x; x = -tmp;
        return;
    }
    case ROTATION_ROLL_180_PITCH_270: {
        y = -y; z = -z;
        tmp = z; z = x; x = -tmp;
        return;
    }
    case ROTATION_ROLL_270_PITCH_270: {
        tmp = z; z = -y; y = tmp;
        tmp = z; z = x; x = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_180_YAW_90: {
        tmp = z; z = y; y = -tmp;
        x = -x; z = -z;
        tmp = x; x = -y; y = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_270: {
        tmp = z; z = y; y = -tmp;
        tmp = x; x = y; y = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_68_YAW_293: {
        T tmpx = x;
        T tmpy = y;
        T tmpz = z;
        x =  0.143039f * tmpx +  0.368776f * tmpy + -0.918446f * tmpz;
        y = -0.332133f * tmpx + -0.856289f * tmpy + -0.395546f * tmpz;
        z = -0.932324f * tmpx +  0.361625f * tmpy +  0.000000f * tmpz;
        return;
    }
    case ROTATION_PITCH_315: {
        tmp = HALF_SQRT_2*(ftype)(x - z);
        z   = HALF_SQRT_2*(ftype)(x + z);
        x = tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_315: {
        tmp = z; z = y; y = -tmp;
        tmp = HALF_SQRT_2*(ftype)(x - z);
        z   = HALF_SQRT_2*(ftype)(x + z);
        x = tmp;
        return;
    }
    case ROTATION_PITCH_7: {
        const T sin_pitch = 0.12186934340514748f; // sinF(pitch);
        const T cos_pitch = 0.992546151641322f; // cosF(pitch);
        T tmpx = x;
        T tmpz = z;
        x =  cos_pitch * tmpx + sin_pitch * tmpz;
        z = -sin_pitch * tmpx + cos_pitch * tmpz;
        return;
    }
    case ROTATION_CUSTOM: 
        // Error: caller must perform custom rotations via matrix multiplication
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    case ROTATION_MAX:
        break;
    }
    // rotation invalid
    INTERNAL_ERROR(AP_InternalError::error_t::bad_rotation);
}

template <typename T>
void Vector3<T>::rotate_inverse(enum Rotation rotation)
{
    Vector3<T> x_vec(1.0f,0.0f,0.0f);
    Vector3<T> y_vec(0.0f,1.0f,0.0f);
    Vector3<T> z_vec(0.0f,0.0f,1.0f);

    x_vec.rotate(rotation);
    y_vec.rotate(rotation);
    z_vec.rotate(rotation);

    Matrix3<T> M(
        x_vec.x, y_vec.x, z_vec.x,
        x_vec.y, y_vec.y, z_vec.y,
        x_vec.z, y_vec.z, z_vec.z
    );

    (*this) = M.mul_transpose(*this);
}

// rotate vector by angle in radians in xy plane leaving z untouched
template <typename T>
void Vector3<T>::rotate_xy(T angle_rad)
{
    const T cs = cosF(angle_rad);
    const T sn = sinF(angle_rad);
    T rx = x * cs - y * sn;
    T ry = x * sn + y * cs;
    x = rx;
    y = ry;
}

// vector cross product
template <typename T>
Vector3<T> Vector3<T>::operator %(const Vector3<T> &v) const
{
    Vector3<T> temp(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    return temp;
}

// dot product
template <typename T>
T Vector3<T>::operator *(const Vector3<T> &v) const
{
    return x*v.x + y*v.y + z*v.z;
}

template <typename T>
T Vector3<T>::length(void) const
{
    return norm(x, y, z);
}

// limit xy component vector to a given length. returns true if vector was limited
template <typename T>
bool Vector3<T>::limit_length_xy(T max_length)
{
    const T length_xy = norm(x, y);
    if ((length_xy > max_length) && is_positive(length_xy)) {
        x *= (max_length / length_xy);
        y *= (max_length / length_xy);
        return true;
    }
    return false;
}

template <typename T>
Vector3<T> &Vector3<T>::operator *=(const T num)
{
    x*=num; y*=num; z*=num;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator /=(const T num)
{
    x /= num; y /= num; z /= num;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator -=(const Vector3<T> &v)
{
    x -= v.x; y -= v.y; z -= v.z;
    return *this;
}

template <typename T>
bool Vector3<T>::is_nan(void) const
{
    return isnan(x) || isnan(y) || isnan(z);
}

template <typename T>
bool Vector3<T>::is_inf(void) const
{
    return isinf(x) || isinf(y) || isinf(z);
}

template <typename T>
Vector3<T> &Vector3<T>::operator +=(const Vector3<T> &v)
{
    x+=v.x; y+=v.y; z+=v.z;
    return *this;
}

template <typename T>
Vector3<T> Vector3<T>::operator /(const T num) const
{
    return Vector3<T>(x/num, y/num, z/num);
}

template <typename T>
Vector3<T> Vector3<T>::operator *(const T num) const
{
    return Vector3<T>(x*num, y*num, z*num);
}

template <typename T>
Vector3<T> Vector3<T>::operator -(const Vector3<T> &v) const
{
    return Vector3<T>(x-v.x, y-v.y, z-v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator +(const Vector3<T> &v) const
{
    return Vector3<T>(x+v.x, y+v.y, z+v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator -(void) const
{
    return Vector3<T>(-x,-y,-z);
}

template <typename T>
bool Vector3<T>::operator ==(const Vector3<T> &v) const
{
    return (is_equal(x,v.x) && is_equal(y,v.y) && is_equal(z,v.z));
}

template <typename T>
bool Vector3<T>::operator !=(const Vector3<T> &v) const
{
    return (!is_equal(x,v.x) || !is_equal(y,v.y) || !is_equal(z,v.z));
}

template <typename T>
T Vector3<T>::angle(const Vector3<T> &v2) const
{
    const T len = this->length() * v2.length();
    if (len <= 0) {
        return 0.0f;
    }
    const T cosv = ((*this)*v2) / len;
    if (fabsF(cosv) >= 1) {
        return 0.0f;
    }
    return acosF(cosv);
}

// multiplication of transpose by a vector
template <typename T>
Vector3<T> Vector3<T>::operator *(const Matrix3<T> &m) const
{
    return Vector3<T>(*this * m.colx(),
                      *this * m.coly(),
                      *this * m.colz());
}

// multiply a column vector by a row vector, returning a 3x3 matrix
template <typename T>
Matrix3<T> Vector3<T>::mul_rowcol(const Vector3<T> &v2) const
{
    const Vector3<T> v1 = *this;
    return Matrix3<T>(v1.x * v2.x, v1.x * v2.y, v1.x * v2.z,
                      v1.y * v2.x, v1.y * v2.y, v1.y * v2.z,
                      v1.z * v2.x, v1.z * v2.y, v1.z * v2.z);
}

// extrapolate position given bearing and pitch (in degrees) and distance
template <typename T>
void Vector3<T>::offset_bearing(T bearing, T pitch, T distance)
{
    y += cosF(radians(pitch)) * sinF(radians(bearing)) * distance;
    x += cosF(radians(pitch)) * cosF(radians(bearing)) * distance;
    z += sinF(radians(pitch)) * distance;
}

// distance from the tip of this vector to a line segment specified by two vectors
template <typename T>
T Vector3<T>::distance_to_segment(const Vector3<T> &seg_start, const Vector3<T> &seg_end) const
{
    // triangle side lengths
    const T a = (*this-seg_start).length();
    const T b = (seg_start-seg_end).length();
    const T c = (seg_end-*this).length();

    // protect against divide by zero later
    if (::is_zero(b)) {
        return 0.0f;
    }

    // semiperimeter of triangle
    const T s = (a+b+c) * 0.5f;

    T area_squared = s*(s-a)*(s-b)*(s-c);
    // area must be constrained above 0 because a triangle could have 3 points could be on a line and float rounding could push this under 0
    if (area_squared < 0.0f) {
        area_squared = 0.0f;
    }
    const T area = safe_sqrt(area_squared);
    return 2.0f*area/b;
}

// Shortest distance between point(p) to a point contained in the line segment defined by w1,w2
template <typename T>
T Vector3<T>::closest_distance_between_line_and_point(const Vector3<T> &w1, const Vector3<T> &w2, const Vector3<T> &p)
{    
    const Vector3<T> nearest = point_on_line_closest_to_other_point(w1, w2, p);
    const T dist = (nearest - p).length();
    return dist;
}

// Point in the line segment defined by w1,w2 which is closest to point(p)
// this is based on the explanation given here: www.fundza.com/vectors/point2line/index.html
template <typename T>
Vector3<T> Vector3<T>::point_on_line_closest_to_other_point(const Vector3<T> &w1, const Vector3<T> &w2, const Vector3<T> &p)
{   
    const Vector3<T> line_vec = w2-w1;
    const Vector3<T> p_vec = p - w1;
    
    const T line_vec_len = line_vec.length();
    // protection against divide by zero
    if(::is_zero(line_vec_len)) {
        return {0.0f, 0.0f, 0.0f};
    }

    const T scale = 1/line_vec_len;
    const Vector3<T> unit_vec = line_vec * scale;
    const Vector3<T> scaled_p_vec = p_vec * scale;

    T dot_product = unit_vec * scaled_p_vec;
    dot_product = constrain_ftype(dot_product,0.0f,1.0f);
 
    const Vector3<T> closest_point = line_vec * dot_product;
    return (closest_point + w1);
}

// Closest point between two line segments
// This implementation is borrowed from: http://geomalgorithms.com/a07-_distance.html
// INPUT: 4 points corresponding to start and end of two line segments
// OUTPUT: closest point on segment 2, from segment 1, gets passed on reference as "closest_point"
template <typename T>
void Vector3<T>::segment_to_segment_closest_point(const Vector3<T>& seg1_start, const Vector3<T>& seg1_end, const Vector3<T>& seg2_start, const Vector3<T>& seg2_end, Vector3<T>& closest_point)
{
    // direction vectors
    const Vector3<T> line1 = seg1_end - seg1_start;
    const Vector3<T> line2 = seg2_end - seg2_start;

    const Vector3<T> diff = seg1_start - seg2_start;

    const T a = line1*line1;
    const T b = line1*line2;
    const T c = line2*line2;
    const T d = line1*diff;
    const T e = line2*diff;

    const T discriminant = (a*c) - (b*b);
    T sN, sD = discriminant;           // default sD = D >= 0
    T tc, tN, tD = discriminant;       // tc = tN / tD, default tD = D >= 0

    if (discriminant < FLT_EPSILON) {
        sN = 0.0;         // force using point seg1_start on line 1
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    } else {
        // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {
            // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        } else if (sN > sD) {
            // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {
        // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0) {
            sN = 0.0;
        } else if (-d > a) {
            sN = sD;
        } else {
            sN = -d;
            sD = a;
        }
    } else if (tN > tD) {
        // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0) {
            sN = 0;
        } else if ((-d + b) > a) {
            sN = sD;
        } else {
            sN = (-d +  b);
            sD = a;
        }
    }
    // finally do the division to get tc
    tc = (fabsf(tN) < FLT_EPSILON ? 0.0 : tN / tD);

    // closest point on seg2
    closest_point = seg2_start + line2*tc;
}

// Returns true if the passed 3D segment passes through a plane defined by plane normal, and a point on the plane
template <typename T>
bool Vector3<T>::segment_plane_intersect(const Vector3<T>& seg_start, const Vector3<T>& seg_end, const Vector3<T>& plane_normal, const Vector3<T>& plane_point)
{
    Vector3<T> u = seg_end - seg_start;
    Vector3<T> w = seg_start - plane_point;

    T D = plane_normal * u;
    T N = -(plane_normal * w);

    if (fabsf(D) < FLT_EPSILON) {
        if (::is_zero(N)) {
            // segment lies in this plane
            return true;
        } else {
            // does not intersect
            return false;
        }
    }
    const T sI = N / D;
    if (sI < 0 || sI > 1) {
        // does not intersect
        return false;
    }
    // intersects at unique point
    return true;
}

// define for float and double
template class Vector3<float>;
template class Vector3<double>;

// define needed ops for Vector3l
template Vector3<int32_t> &Vector3<int32_t>::operator +=(const Vector3<int32_t> &v);
