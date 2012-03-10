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

#define MATRIX_ROTATION_NONE               Matrix3f(1, 0, 0, 0, 1, 0, 0 ,0, 1)
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

// fill in a matrix with a standard rotation
template <typename T>
void Matrix3<T>::rotation(enum Rotation r)
{
    switch (r) {
    case ROTATION_NONE:
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
    }
}

// create a rotation matrix given some euler angles
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
template <typename T>
void Matrix3<T>::from_euler(float roll, float pitch, float yaw)
{
	float cp = cos(pitch);
	float sp = sin(pitch);
	float sr = sin(roll);
	float cr = cos(roll);
	float sy = sin(yaw);
	float cy = cos(yaw);

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
		*roll = atan2(c.y, c.z);
	}
	if (yaw != NULL) {
		*yaw = atan2(b.x, a.x);
	}
}

// only define for float
template void Matrix3<float>::rotation(enum Rotation);
template void Matrix3<float>::from_euler(float roll, float pitch, float yaw);
template void Matrix3<float>::to_euler(float *roll, float *pitch, float *yaw);
