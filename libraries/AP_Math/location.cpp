/*
 * location.cpp
 * Copyright (C) Andrew Tridgell 2011
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

/*
 *  this module deals with calculations involving struct Location
 */
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include "AP_Math.h"
#include "location.h"

float longitude_scale(const struct Location &loc)
{
    float scale = cosf(loc.lat * (1.0e-7f * DEG_TO_RAD));
    return constrain_float(scale, 0.01f, 1.0f);
}



// return distance in meters between two locations
float get_distance(const struct Location &loc1, const struct Location &loc2)
{
    float dlat              = (float)(loc2.lat - loc1.lat);
    float dlong             = ((float)(loc2.lng - loc1.lng)) * longitude_scale(loc2);
    return norm(dlat, dlong) * LOCATION_SCALING_FACTOR;
}

// return distance in centimeters to between two locations
uint32_t get_distance_cm(const struct Location &loc1, const struct Location &loc2)
{
    return get_distance(loc1, loc2) * 100;
}

// return horizontal distance between two positions in cm
float get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination)
{
    return norm(destination.x-origin.x,destination.y-origin.y);
}

// return bearing in centi-degrees between two locations
int32_t get_bearing_cd(const struct Location &loc1, const struct Location &loc2)
{
    int32_t off_x = loc2.lng - loc1.lng;
    int32_t off_y = (loc2.lat - loc1.lat) / longitude_scale(loc2);
    int32_t bearing = 9000 + atan2f(-off_y, off_x) * DEGX100;
    if (bearing < 0) bearing += 36000;
    return bearing;
}

// return bearing in centi-degrees between two positions
float get_bearing_cd(const Vector3f &origin, const Vector3f &destination)
{
    float bearing = atan2f(destination.y-origin.y, destination.x-origin.x) * DEGX100;
    if (bearing < 0) {
        bearing += 36000.0f;
    }
    return bearing;
}

// see if location is past a line perpendicular to
// the line between point1 and point2. If point1 is
// our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool location_passed_point(const struct Location &location,
                           const struct Location &point1,
                           const struct Location &point2)
{
    return location_path_proportion(location, point1, point2) >= 1.0f;
}


/*
  return the proportion we are along the path from point1 to
  point2, along a line parallel to point1<->point2.

  This will be less than >1 if we have passed point2
 */
float location_path_proportion(const struct Location &location,
                               const struct Location &point1,
                               const struct Location &point2)
{
    Vector2f vec1 = location_diff(point1, point2);
    Vector2f vec2 = location_diff(point1, location);
    float dsquared = sq(vec1.x) + sq(vec1.y);
    if (dsquared < 0.001f) {
        // the two points are very close together
        return 1.0f;
    }
    return (vec1 * vec2) / dsquared;
}

/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of 
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void location_update(struct Location &loc, float bearing, float distance)
{
    float ofs_north = cosf(radians(bearing))*distance;
    float ofs_east  = sinf(radians(bearing))*distance;
    location_offset(loc, ofs_north, ofs_east);
}

/*
 *  extrapolate latitude/longitude given distances north and east
 */
void location_offset(struct Location &loc, float ofs_north, float ofs_east)
{
    if (!is_zero(ofs_north) || !is_zero(ofs_east)) {
        int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
        int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(loc);
        loc.lat += dlat;
        loc.lng += dlng;
    }
}

/*
  return the distance in meters in North/East plane as a N/E vector
  from loc1 to loc2
 */
Vector2f location_diff(const struct Location &loc1, const struct Location &loc2)
{
    return Vector2f((loc2.lat - loc1.lat) * LOCATION_SCALING_FACTOR,
                    (loc2.lng - loc1.lng) * LOCATION_SCALING_FACTOR * longitude_scale(loc1));
}

/*
  return the distance in meters in North/East/Down plane as a N/E/D vector
  from loc1 to loc2
 */
Vector3f location_3d_diff_NED(const struct Location &loc1, const struct Location &loc2)
{
    return Vector3f((loc2.lat - loc1.lat) * LOCATION_SCALING_FACTOR,
                    (loc2.lng - loc1.lng) * LOCATION_SCALING_FACTOR * longitude_scale(loc1),
                    (loc1.alt - loc2.alt) * 0.01f);
}

/*
  return true if lat and lng match. Ignores altitude and options
 */
bool locations_are_same(const struct Location &loc1, const struct Location &loc2) {
    return (loc1.lat == loc2.lat) && (loc1.lng == loc2.lng);
}

/*
 * convert invalid waypoint with useful data. return true if location changed
 */
bool location_sanitize(const struct Location &defaultLoc, struct Location &loc)
{
    bool has_changed = false;
    // convert lat/lng=0 to mean current point
    if (loc.lat == 0 && loc.lng == 0) {
        loc.lat = defaultLoc.lat;
        loc.lng = defaultLoc.lng;
        has_changed = true;
    }

    // convert relative alt=0 to mean current alt
    if (loc.alt == 0 && loc.relative_alt) {
        loc.relative_alt = false;
        loc.alt = defaultLoc.alt;
        has_changed = true;
    }

    // limit lat/lng to appropriate ranges
    if (!check_latlng(loc)) {
        loc.lat = defaultLoc.lat;
        loc.lng = defaultLoc.lng;
        has_changed = true;
    }

    return has_changed;
}

/*
  print a int32_t lat/long in decimal degrees
 */
void print_latlon(AP_HAL::BetterStream *s, int32_t lat_or_lon)
{
    int32_t dec_portion, frac_portion;
    int32_t abs_lat_or_lon = labs(lat_or_lon);

    // extract decimal portion (special handling of negative numbers to ensure we round towards zero)
    dec_portion = abs_lat_or_lon / 10000000UL;

    // extract fractional portion
    frac_portion = abs_lat_or_lon - dec_portion*10000000UL;

    // print output including the minus sign
    if( lat_or_lon < 0 ) {
        s->printf("-");
    }
    s->printf("%ld.%07ld",(long)dec_portion,(long)frac_portion);
}

// return true when lat and lng are within range
bool check_lat(float lat)
{
    return fabsf(lat) <= 90;
}
bool check_lng(float lng)
{
    return fabsf(lng) <= 180;
}
bool check_lat(int32_t lat)
{
    return labs(lat) <= 90*1e7;
}
bool check_lng(int32_t lng)
{
    return labs(lng) <= 180*1e7;
}
bool check_latlng(float lat, float lng)
{
    return check_lat(lat) && check_lng(lng);
}
bool check_latlng(int32_t lat, int32_t lng)
{
    return check_lat(lat) && check_lng(lng);
}
bool check_latlng(Location loc)
{
    return check_lat(loc.lat) && check_lng(loc.lng);
}
