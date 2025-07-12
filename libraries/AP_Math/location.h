#pragma once

#include <inttypes.h>

#include "vector2.h"
#include "vector3.h"

/*
 * LOCATION
 */

// Computes straight-line distance in the horizontal plane between two positions.
// Input units (e.g., meters, centimeters) must match; no unit conversion is performed.
template <typename T>
float get_horizontal_distance(const Vector2<T> &origin, const Vector2<T> &destination)
{
    return (destination - origin).length();
}

// return bearing in radians between two positions
float        get_bearing_rad(const Vector2f &origin, const Vector2f &destination);

// return bearing in centi-degrees between two positions
float        get_bearing_cd(const Vector2f &origin, const Vector2f &destination);

// Converts from WGS84 geodetic coordinates (lat, lon, height)
// into WGS84 Earth Centered, Earth Fixed (ECEF) coordinates
// (X, Y, Z)
void        wgsllh2ecef(const Vector3d &llh, Vector3d &ecef);

// Converts from WGS84 Earth Centered, Earth Fixed (ECEF)
// coordinates (X, Y, Z), into WHS84 geodetic
// coordinates (lat, lon, height)
void        wgsecef2llh(const Vector3d &ecef, Vector3d &llh);

// return true when lat and lng are within range
bool        check_lat(float lat) WARN_IF_UNUSED;
bool        check_lng(float lng) WARN_IF_UNUSED;
bool        check_lat(int32_t lat) WARN_IF_UNUSED;
bool        check_lng(int32_t lng) WARN_IF_UNUSED;
bool        check_latlng(float lat, float lng) WARN_IF_UNUSED;
bool        check_latlng(int32_t lat, int32_t lng) WARN_IF_UNUSED;
