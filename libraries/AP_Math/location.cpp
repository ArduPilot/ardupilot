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
#include <stdlib.h>
#include "AP_Math.h"
#include "location.h"

// return horizontal distance between two positions in cm
float get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination)
{
    return norm(destination.x-origin.x,destination.y-origin.y);
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

