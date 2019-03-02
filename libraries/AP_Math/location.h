#pragma once

#include <inttypes.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "vector2.h"
#include "vector3.h"

// scaling factor from 1e-7 degrees to meters at equator
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

/*
 * LOCATION
 */
// longitude_scale - returns the scaler to compensate for shrinking longitude as you move north or south from the equator
// Note: this does not include the scaling to convert longitude/latitude points to meters or centimeters
float        longitude_scale(const struct Location &loc);

// return horizontal distance in centimeters between two positions
float        get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);

// return bearing in centi-degrees between two locations
int32_t      get_bearing_cd(const struct Location &loc1, const struct Location &loc2);

// return bearing in centi-degrees between two positions
float        get_bearing_cd(const Vector3f &origin, const Vector3f &destination);

// see if location is past a line perpendicular to
// the line between point1 and point2. If point1 is
// our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool        location_passed_point(const struct Location & location,
                                  const struct Location & point1,
                                  const struct Location & point2);

/*
  return the proportion we are along the path from point1 to
  point2. This will be less than >1 if we have passed point2
 */
float       location_path_proportion(const struct Location &location,
                               const struct Location &point1,
                               const struct Location &point2);

//  extrapolate latitude/longitude given bearing and distance
void        location_update(struct Location &loc, float bearing, float distance);

/*
  return the distance in meters in North/East plane as a N/E vector
  from loc1 to loc2
 */
Vector2f    location_diff(const struct Location &loc1, const struct Location &loc2);

/*
  return the distance in meters in North/East/Down plane as a N/E/D vector
  from loc1 to loc2
 */
Vector3f    location_3d_diff_NED(const struct Location &loc1, const struct Location &loc2);

/*
 * check if lat and lng match. Ignore altitude and options
 */
bool        locations_are_same(const struct Location &loc1, const struct Location &loc2);

/*
 * convert invalid waypoint with useful data. return true if location changed
 */
bool        location_sanitize(const struct Location &defaultLoc, struct Location &loc);

/*
  print a int32_t lat/long in decimal degrees
 */
void        print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon);

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
bool        check_latlng(Location loc);

