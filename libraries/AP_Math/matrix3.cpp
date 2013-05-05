/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * matrix3.cpp
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

#include "AP_Math.h"

#define HALF_SQRT_2 0.70710678118654757

#define MATRIX_ROTATION_NONE               Matrix3f(1, 0, 0, 0, 1, 0, 0,0, 1)
#define MATRIX_ROTATION_YAW_45             Matrix3f(HALF_SQRT_2, -HALF_SQRT_2, 0, HALF_SQRT_2, HALF_SQRT_2, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_90             Matrix3f(0, -1, 0, 1, 0, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_135            Matrix3f(-HALF_SQRT_2, -HALF_SQRT_2, 0, HALF_SQRT_2, -HALF_SQRT_2, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_180            Matrix3f(-1, 0, 0, 0, -1, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_225            Matrix3f(-HALF_SQRT_2, HALF_SQRT_2, 0, -HALF_SQRT_2, -HALF_SQRT_2, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_270            Matrix3f(0, 1, 0, -1, 0, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_315            Matrix3f(HALF_SQRT_2, HALF_SQRT_2, 0, -HALF_SQRT_2, HALF_SQRT_2, 0, 0, 0, 1)
#define MATRIX_ROTATION_ROLL_180           Matrix3f(1, 0, 0, 0, -1, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_45    Matrix3f(HALF_SQRT_2, HALF_SQRT_2, 0, HALF_SQRT_2, -HALF_SQRT_2, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_90    Matrix3f(0, 1, 0, 1, 0, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_135   Matrix3f(-HALF_SQRT_2, HALF_SQRT_2, 0, HALF_SQRT_2, HALF_SQRT_2, 0, 0, 0, -1)
#define MATRIX_ROTATION_PITCH_180          Matrix3f(-1, 0, 0, 0, 1, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_225   Matrix3f(-HALF_SQRT_2, -HALF_SQRT_2, 0, -HALF_SQRT_2, HALF_SQRT_2, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_270   Matrix3f(0, -1, 0, -1, 0, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_315   Matrix3f(HALF_SQRT_2, -HALF_SQRT_2, 0, -HALF_SQRT_2, -HALF_SQRT_2, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_90            Matrix3f(1, 0,  0,  0, 0, -1,  0,  1, 0)
#define MATRIX_ROTATION_ROLL_90_YAW_45     Matrix3f(HALF_SQRT_2, 0, HALF_SQRT_2, HALF_SQRT_2, 0, -HALF_SQRT_2, 0, 1, 0)
#define MATRIX_ROTATION_ROLL_90_YAW_90     Matrix3f(0, 0, 1, 1, 0, 0, 0, 1, 0)
#define MATRIX_ROTATION_ROLL_90_YAW_135    Matrix3f(-HALF_SQRT_2, 0, HALF_SQRT_2, HALF_SQRT_2, 0, HALF_SQRT_2, 0, 1, 0)
#define MATRIX_ROTATION_ROLL_270           Matrix3f(1, 0,  0,  0, 0,  1,  0, -1, 0)
#define MATRIX_ROTATION_ROLL_270_YAW_45    Matrix3f(HALF_SQRT_2, 0, -HALF_SQRT_2, HALF_SQRT_2, 0, HALF_SQRT_2, 0, -1, 0)
#define MATRIX_ROTATION_ROLL_270_YAW_90    Matrix3f(0, 0, -1, 1, 0, 0, 0, -1, 0)
#define MATRIX_ROTATION_ROLL_270_YAW_135   Matrix3f(-HALF_SQRT_2, 0, -HALF_SQRT_2, HALF_SQRT_2, 0, -HALF_SQRT_2, 0, -1, 0)
#define MATRIX_ROTATION_PITCH_90           Matrix3f(0, 0,  1,  0, 1,  0, -1,  0, 0)
#define MATRIX_ROTATION_PITCH_270          Matrix3f(0, 0, -1,  0, 1,  0,  1,  0, 0)

// fill in a matrix with a standard rotation
template <typename T>
void Matrix3<T>::rotation(enum Rotation r)
{
    switch (r) {
    case ROTATION_NONE:
    case ROTATION_MAX:
        *this = MATRIX_ROTATION_NONE;
        break;
    case ROTATION_YAW_45:
        *this = MATRIX_ROTATION_YAW_45;
        break;
    case ROTATION_YAW_90:
        *this = MATRIX_ROTATION_YAW_90;
        break;
    case ROTATION_YAW_135:
        *this = MATRIX_ROTATION_YAW_135;
        break;
    case ROTATION_YAW_180:
        *this = MATRIX_ROTATION_YAW_180;
        break;
    case ROTATION_YAW_225:
        *this = MATRIX_ROTATION_YAW_225;
        break;
    case ROTATION_YAW_270:
        *this = MATRIX_ROTATION_YAW_270;
        break;
    case ROTATION_YAW_315:
        *this = MATRIX_ROTATION_YAW_315;
        break;
    case ROTATION_ROLL_180:
        *this = MATRIX_ROTATION_ROLL_180;
        break;
    case ROTATION_ROLL_180_YAW_45:
        *this = MATRIX_ROTATION_ROLL_180_YAW_45;
        break;
    case ROTATION_ROLL_180_YAW_90:
        *this = MATRIX_ROTATION_ROLL_180_YAW_90;
        break;
    case ROTATION_ROLL_180_YAW_135:
        *this = MATRIX_ROTATION_ROLL_180_YAW_135;
        break;
    case ROTATION_PITCH_180:
        *this = MATRIX_ROTATION_PITCH_180;
        break;
    case ROTATION_ROLL_180_YAW_225:
        *this = MATRIX_ROTATION_ROLL_180_YAW_225;
        break;
    case ROTATION_ROLL_180_YAW_270:
        *this = MATRIX_ROTATION_ROLL_180_YAW_270;
        break;
    case ROTATION_ROLL_180_YAW_315:
        *this = MATRIX_ROTATION_ROLL_180_YAW_315;
        break;
    case ROTATION_ROLL_90:
        *this = MATRIX_ROTATION_ROLL_90;
        break;
    case ROTATION_ROLL_90_YAW_45:
        *this = MATRIX_ROTATION_ROLL_90_YAW_45;
        break;
    case ROTATION_ROLL_90_YAW_90:
        *this = MATRIX_ROTATION_ROLL_90_YAW_90;
        break;
    case ROTATION_ROLL_90_YAW_135:
        *this = MATRIX_ROTATION_ROLL_90_YAW_135;
        break;
    case ROTATION_ROLL_270:
        *this = MATRIX_ROTATION_ROLL_270;
        break;
    case ROTATION_ROLL_270_YAW_45:
        *this = MATRIX_ROTATION_ROLL_270_YAW_45;
        break;
    case ROTATION_ROLL_270_YAW_90:
        *this = MATRIX_ROTATION_ROLL_270_YAW_90;
        break;
    case ROTATION_ROLL_270_YAW_135:
        *this = MATRIX_ROTATION_ROLL_270_YAW_135;
        break;
    case ROTATION_PITCH_90:
        *this = MATRIX_ROTATION_PITCH_90;
        break;
    case ROTATION_PITCH_270:
        *this = MATRIX_ROTATION_PITCH_270;
        break;
    }
}

// create a rotation matrix given some euler angles
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
template <typename T>
void Matrix3<T>::from_euler(float roll, float pitch, float yaw)
{
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float sr = sinf(roll);
    float cr = cosf(roll);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    a.x = cp * cy;
    a.y = (sr * sp * cy) - (cr * sy);
    a.z = (cr * sp * cy) + (sr * sy);
    b.x = cp * sy;
    b.y = (sr * sp * sy) + (cr * cy);
    b.z = (cr * sp * sy) - (sr * cy);
    c.x = -sp;
    c.y = sr * cp;
    c.z = cr * cp;
}

// calculate euler angles from a rotation matrix
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
template <typename T>
void Matrix3<T>::to_euler(float *roll, float *pitch, float *yaw)
{
    if (pitch != NULL) {
        *pitch = -safe_asin(c.x);
    }
    if (roll != NULL) {
        *roll = atan2f(c.y, c.z);
    }
    if (yaw != NULL) {
        *yaw = atan2f(b.x, a.x);
    }
}

// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
template <typename T>
void Matrix3<T>::rotate(const Vector3<T> &g)
{
    Matrix3f temp_matrix;
    temp_matrix.a.x = a.y * g.z - a.z * g.y;
    temp_matrix.a.y = a.z * g.x - a.x * g.z;
    temp_matrix.a.z = a.x * g.y - a.y * g.x;
    temp_matrix.b.x = b.y * g.z - b.z * g.y;
    temp_matrix.b.y = b.z * g.x - b.x * g.z;
    temp_matrix.b.z = b.x * g.y - b.y * g.x;
    temp_matrix.c.x = c.y * g.z - c.z * g.y;
    temp_matrix.c.y = c.z * g.x - c.x * g.z;
    temp_matrix.c.z = c.x * g.y - c.y * g.x;

    (*this) += temp_matrix;
}


// multiplication by a vector
template <typename T>
Vector3<T> Matrix3<T>::operator *(const Vector3<T> &v) const
{
    return Vector3<T>(a.x * v.x + a.y * v.y + a.z * v.z,
                      b.x * v.x + b.y * v.y + b.z * v.z,
                      c.x * v.x + c.y * v.y + c.z * v.z);
}

// multiplication by a vector, extracting only the xy components
template <typename T>
Vector2<T> Matrix3<T>::mulXY(const Vector3<T> &v) const
{
    return Vector2<T>(a.x * v.x + a.y * v.y + a.z * v.z,
                      b.x * v.x + b.y * v.y + b.z * v.z);
}

// multiplication of transpose by a vector
template <typename T>
Vector3<T> Matrix3<T>::mul_transpose(const Vector3<T> &v) const
{
    return Vector3<T>(a.x * v.x + b.x * v.y + c.x * v.z,
                      a.y * v.x + b.y * v.y + c.y * v.z,
                      a.z * v.x + b.z * v.y + c.z * v.z);
}

// multiplication by another Matrix3<T>
template <typename T>
Matrix3<T> Matrix3<T>::operator *(const Matrix3<T> &m) const
{
    Matrix3<T> temp (Vector3<T>(a.x * m.a.x + a.y * m.b.x + a.z * m.c.x,
                                a.x * m.a.y + a.y * m.b.y + a.z * m.c.y,
                                a.x * m.a.z + a.y * m.b.z + a.z * m.c.z),
                     Vector3<T>(b.x * m.a.x + b.y * m.b.x + b.z * m.c.x,
                                b.x * m.a.y + b.y * m.b.y + b.z * m.c.y,
                                b.x * m.a.z + b.y * m.b.z + b.z * m.c.z),
                     Vector3<T>(c.x * m.a.x + c.y * m.b.x + c.z * m.c.x,
                                c.x * m.a.y + c.y * m.b.y + c.z * m.c.y,
                                c.x * m.a.z + c.y * m.b.z + c.z * m.c.z));
    return temp;
}

template <typename T>
Matrix3<T> Matrix3<T>::transposed(void) const
{
    return Matrix3<T>(Vector3<T>(a.x, b.x, c.x),
                      Vector3<T>(a.y, b.y, c.y),
                      Vector3<T>(a.z, b.z, c.z));
}

template <typename T>
void Matrix3<T>::zero(void)
{
    a.x = a.y = a.z = 0;
    b.x = b.y = b.z = 0;
    c.x = c.y = c.z = 0;
}


// only define for float
template void Matrix3<float>::rotation(enum Rotation);
template void Matrix3<float>::zero(void);
template void Matrix3<float>::rotate(const Vector3<float> &g);
template void Matrix3<float>::from_euler(float roll, float pitch, float yaw);
template void Matrix3<float>::to_euler(float *roll, float *pitch, float *yaw);
template Vector3<float> Matrix3<float>::operator *(const Vector3<float> &v) const;
template Vector3<float> Matrix3<float>::mul_transpose(const Vector3<float> &v) const;
template Matrix3<float> Matrix3<float>::operator *(const Matrix3<float> &m) const;
template Matrix3<float> Matrix3<float>::transposed(void) const;
template Vector2<float> Matrix3<float>::mulXY(const Vector3<float> &v) const;
