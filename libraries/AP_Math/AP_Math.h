// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef AP_MATH_H
#define AP_MATH_H

// Assorted useful math operations for ArduPilot(Mega)

#include <AP_Common.h>
#include <stdint.h>
#include "rotations.h"
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

#endif
