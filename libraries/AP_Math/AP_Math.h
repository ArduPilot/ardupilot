// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Assorted useful math operations for ArduPilot(Mega)

#include <AP_Common.h>
#include <stdint.h>
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"
#include "quaternion.h"
#include "polygon.h"

// define AP_Param types AP_Vector3f and Ap_Matrix3f
AP_PARAMDEFV(Matrix3f, Matrix3f, AP_PARAM_MATRIX3F);
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);

// a varient of asin() that always gives a valid answer.
float safe_asin(float v);

// a varient of sqrt() that always gives a valid answer.
float safe_sqrt(float v);

// create a rotation matrix given some euler angles
void rotation_matrix_from_euler(Matrix3f &m, float roll, float pitch, float yaw);

// calculate euler angles from a rotation matrix
void calculate_euler_angles(const Matrix3f &m, float *roll, float *pitch, float *yaw);

// create a quaternion from Euler angles
void quaternion_from_euler(Quaternion &q, float roll, float pitch, float yaw);

// create eulers from a quaternion
void euler_from_quaternion(const Quaternion &q, float *roll, float *pitch, float *yaw);

// convert a quaternion to a rotation matrix
void quaternion_to_rotation_matrix(const Quaternion &q, Matrix3f &m);

// convert a vector in earth frame to a vector in body frame,
// assuming body current rotation is given by a quaternion
void quaternion_earth_to_body(const Quaternion &q, Vector3f &v);
