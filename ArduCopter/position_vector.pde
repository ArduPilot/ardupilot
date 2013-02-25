// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// position_vector.pde related utility functions

// position vectors are Vector2f
//    .x = latitude from home in cm
//    .y = longitude from home in cm

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
const Vector2f pv_latlon_to_vector(int32_t lat, int32_t lon)
{
    Vector2f tmp(lat-home.lat * LATLON_TO_CM, lon-home.lng * LATLON_TO_CM * scaleLongDown);
    return tmp;
}

// pv_get_lon - extract latitude from position vector
const int32_t pv_get_lat(const Vector2f pos_vec)
{
    return home.lat + (int32_t)(pos_vec.x / LATLON_TO_CM);
}

// pv_get_lon - extract longitude from position vector
const int32_t pv_get_lon(const Vector2f pos_vec)
{
    return home.lng + (int32_t)(pos_vec.y / LATLON_TO_CM * scaleLongUp);
}

// pv_get_distance_cm - return distance between two positions in cm
const float pv_get_distance_cm(const Vector2f origin, const Vector2f destination)
{
    Vector2f dist = destination - origin;
    return pythagorous2(dist.x,dist.y);
}

// pv_get_bearing_cd - return bearing in centi-degrees between two locations
const float pv_get_bearing_cd(const Vector2f origin, const Vector2f destination)
{
    Vector2f dist = destination - origin;
    int32_t bearing = 9000 + atan2f(dist.x, dist.y) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}