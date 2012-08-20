// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef AP_MATH_H
#define AP_MATH_H

// Assorted useful math operations for ArduPilot(Mega)

#include <AP_Common.h>
#include <AP_Param.h>
#include <math.h>
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
float           safe_asin(float v);

// a varient of sqrt() that always gives a valid answer.
float           safe_sqrt(float v);

// find a rotation that is the combination of two other
// rotations. This is used to allow us to add an overall board
// rotation to an existing rotation of a sensor such as the compass
enum Rotation           rotation_combination(enum Rotation r1, enum Rotation r2, bool *found = NULL);

// return distance in meters between two locations
float                   get_distance(const struct Location *loc1, const struct Location *loc2);

// return distance in centimeters between two locations
int32_t                 get_distance_cm(const struct Location *loc1, const struct Location *loc2);

// return bearing in centi-degrees between two locations
int32_t                 get_bearing_cd(const struct Location *loc1, const struct Location *loc2);

// see if location is past a line perpendicular to
// the line between point1 and point2. If point1 is
// our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool        location_passed_point(struct Location & location,
                                  struct Location & point1,
                                  struct Location & point2);

//  extrapolate latitude/longitude given bearing and distance
void        location_update(struct Location *loc, float bearing, float distance);

// extrapolate latitude/longitude given distances north and east
void        location_offset(struct Location *loc, float ofs_north, float ofs_east);

#endif // AP_MATH_H

