// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// position_vector.pde related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
Vector3f pv_location_to_vector(const Location& loc)
{
    const struct Location &temp_home = ahrs.get_home();
    Vector3f tmp((loc.lat-temp_home.lat) * LATLON_TO_CM, (loc.lng-temp_home.lng) * LATLON_TO_CM * scaleLongDown, loc.alt);
    return tmp;
}

//BEV added this one. Just inverted pv_location_to_vector
struct Location pv_vector_to_location(const Vector3f& vect)
{
    const struct Location &temp_home = ahrs.get_home();
    struct Location temp;// = {};
    temp.lat = vect.x/LATLON_TO_CM + temp_home.lat;
    temp.lng = vect.y/(LATLON_TO_CM * scaleLongDown) + temp_home.lng;
    temp.alt = vect.z;

    return temp;
}

// pv_get_bearing_cd - return bearing in centi-degrees between two positions
float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination)
{
    float bearing = 9000 + fast_atan2(-(destination.x-origin.x), destination.y-origin.y) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

// pv_get_horizontal_distance_cm - return distance between two positions in cm
float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination)
{
    return pythagorous2(destination.x-origin.x,destination.y-origin.y);
}
