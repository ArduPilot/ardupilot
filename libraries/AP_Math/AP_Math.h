// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Assorted useful math operations for ArduPilot(Mega)

#include <AP_Common.h>
#include <stdint.h>
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"
#include "polygon.h"

// define AP_Param types AP_Vector3f and Ap_Matrix3f
AP_PARAMDEFV(Matrix3f, Matrix3f, AP_PARAM_MATRIX3F);
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);
