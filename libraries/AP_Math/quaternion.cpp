/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include "AP_Math.h"

// return the rotation matrix equivalent for this quaternion
void Quaternion::rotation_matrix(Matrix3f &m)
{
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
    float q4q4 = q4 * q4;

    m.a.x = 1-2*(q3q3 + q4q4);
    m.a.y =   2*(q2q3 - q1q4);
    m.a.z =   2*(q2q4 + q1q3);
    m.b.x =   2*(q2q3 + q1q4);
    m.b.y = 1-2*(q2q2 + q4q4);
    m.b.z =   2*(q3q4 - q1q2);
    m.c.x =   2*(q2q4 - q1q3);
    m.c.y =   2*(q3q4 + q1q2);
    m.c.z = 1-2*(q2q2 + q3q3);
}

// convert a vector from earth to body frame
void Quaternion::earth_to_body(Vector3f &v)
{
    Matrix3f m;
    // we reverse z before and afterwards because of the differing
    // quaternion conventions from APM conventions.
    v.z = -v.z;
    rotation_matrix(m);
    v = m * v;
    v.z = -v.z;
}

// create a quaternion from Euler angles
void Quaternion::from_euler(float roll, float pitch, float yaw)
{
    float cr2 = cos(roll*0.5);
    float cp2 = cos(pitch*0.5);
    float cy2 = cos(yaw*0.5);
    float sr2 = sin(roll*0.5);
    float sp2 = sin(pitch*0.5);
    float sy2 = sin(yaw*0.5);

    q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}

// create eulers from a quaternion
void Quaternion::to_euler(float *roll, float *pitch, float *yaw)
{
    if (roll) {
        *roll = (atan2(2.0*(q1*q2 + q3*q4),
                       1 - 2.0*(q2*q2 + q3*q3)));
    }
    if (pitch) {
        // we let safe_asin() handle the singularities near 90/-90 in pitch
        *pitch = safe_asin(2.0*(q1*q3 - q4*q2));
    }
    if (yaw) {
        *yaw = atan2(2.0*(q1*q4 + q2*q3),
                     1 - 2.0*(q3*q3 + q4*q4));
    }
}
