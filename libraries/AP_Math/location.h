#pragma once

#include <inttypes.h>

#include "vector2.h"
#include "vector3.h"

/*
 * LOCATION
 */

// return horizontal distance in centimeters between two positions
float        get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);

// return bearing in centi-degrees between two positions
float        get_bearing_cd(const Vector3f &origin, const Vector3f &destination);

// Converts from WGS84 geodetic coordinates (lat, lon, height)
// into WGS84 Earth Centered, Earth Fixed (ECEF) coordinates
// (X, Y, Z)
void        wgsllh2ecef(const Vector3d &llh, Vector3d &ecef);

// Converts from WGS84 Earth Centered, Earth Fixed (ECEF)
// coordinates (X, Y, Z), into WHS84 geodetic
// coordinates (lat, lon, height)
void        wgsecef2llh(const Vector3d &ecef, Vector3d &llh);

// return true when lat and lng are within range
bool        check_lat(float lat);
bool        check_lng(float lng);
bool        check_lat(int32_t lat);
bool        check_lng(int32_t lng);
bool        check_latlng(float lat, float lng);
bool        check_latlng(int32_t lat, int32_t lng);
