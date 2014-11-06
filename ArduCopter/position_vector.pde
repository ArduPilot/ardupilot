// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// position_vector.pde related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_location_to_vector - convert lat/lon coordinates to a position vector
Vector3f pv_location_to_vector(const Location& loc)
{
    const struct Location &temp_home = ahrs.get_home();
    Vector3f tmp((loc.lat-temp_home.lat) * LATLON_TO_CM, (loc.lng-temp_home.lng) * LATLON_TO_CM * scaleLongDown, loc.alt);
    return tmp;
}

// pv_location_to_vector_with_default - convert lat/lon coordinates to a position vector,
//     defaults to the default_posvec's lat/lon and alt if the provided lat/lon or alt are zero
Vector3f pv_location_to_vector_with_default(const Location& loc, const Vector3f& default_posvec)
{
    Vector3f pos = pv_location_to_vector(loc);

    // set target altitude to default if not provided
    if (loc.alt == 0) {
        pos.z = default_posvec.z;
    }

    // set target position to default if not provided
    if (loc.lat == 0 && loc.lng == 0) {
        pos.x = default_posvec.x;
        pos.y = default_posvec.y;
    }

    return pos;
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
