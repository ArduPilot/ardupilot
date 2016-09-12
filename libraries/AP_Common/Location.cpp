/*
 * Location.cpp
 */

#include "Location.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_AHRS/AP_AHRS_NavEKF.h>
#include <AP_Terrain/AP_Terrain.h>

extern const AP_HAL::HAL& hal;

const AP_AHRS_NavEKF *Location_Class::_ahrs = NULL;
AP_Terrain *Location_Class::_terrain = NULL;

// scalers to convert latitude and longitude to meters.  Duplicated from location.cpp
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

/// constructors
Location_Class::Location_Class()
{
    zero();
}

Location_Class::Location_Class(int32_t latitude, int32_t longitude, int32_t alt_in_cm, ALT_FRAME frame)
{
    lat = latitude;
    lng = longitude;
    options = 0;
    set_alt_cm(alt_in_cm, frame);
}

Location_Class::Location_Class(const Location& loc)
{
    lat = loc.lat;
    lng = loc.lng;
    alt = loc.alt;
    options = loc.options;
}

Location_Class::Location_Class(const Vector3f &ekf_offset_neu)
{
    // store alt and alt frame
    set_alt_cm(ekf_offset_neu.z, ALT_FRAME_ABOVE_ORIGIN);

    // calculate lat, lon
    if (_ahrs != NULL) {
        Location ekf_origin;
        if (_ahrs->get_origin(ekf_origin)) {
            lat = ekf_origin.lat;
            lng = ekf_origin.lng;
            offset(ekf_offset_neu.x / 100.0f, ekf_offset_neu.y / 100.0f);
        }
    }
}

Location_Class& Location_Class::operator=(const struct Location &loc)
{
    lat = loc.lat;
    lng = loc.lng;
    alt = loc.alt;
    options = loc.options;
    return *this;
}

void Location_Class::set_alt_cm(int32_t alt_cm, ALT_FRAME frame)
{
    alt = alt_cm;
    flags.relative_alt = false;
    flags.terrain_alt = false;
    flags.origin_alt = false;
    switch (frame) {
        case ALT_FRAME_ABSOLUTE:
            // do nothing
            break;
        case ALT_FRAME_ABOVE_HOME:
            flags.relative_alt = true;
            break;
        case ALT_FRAME_ABOVE_ORIGIN:
            flags.origin_alt = true;
            break;
        case ALT_FRAME_ABOVE_TERRAIN:
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            flags.relative_alt = true;
            flags.terrain_alt = true;
            break;
    }
}

// converts altitude to new frame
bool Location_Class::change_alt_frame(ALT_FRAME desired_frame)
{
    int32_t new_alt_cm;
    if (!get_alt_cm(desired_frame, new_alt_cm)) {
        return false;
    }
    set_alt_cm(new_alt_cm, desired_frame);
    return true;
}

// get altitude frame
Location_Class::ALT_FRAME Location_Class::get_alt_frame() const
{
    if (flags.terrain_alt) {
        return ALT_FRAME_ABOVE_TERRAIN;
    }
    if (flags.origin_alt) {
        return ALT_FRAME_ABOVE_ORIGIN;
    }
    if (flags.relative_alt) {
        return ALT_FRAME_ABOVE_HOME;
    }
    return ALT_FRAME_ABSOLUTE;
}

/// get altitude in desired frame
bool Location_Class::get_alt_cm(ALT_FRAME desired_frame, int32_t &ret_alt_cm) const
{
    Location_Class::ALT_FRAME frame = get_alt_frame();

    // shortcut if desired and underlying frame are the same
    if (desired_frame == frame) {
        ret_alt_cm = alt;
        return true;
    }

    // check for terrain altitude
    float alt_terr_cm = 0;
    if (frame == ALT_FRAME_ABOVE_TERRAIN || desired_frame == ALT_FRAME_ABOVE_TERRAIN) {
#if AP_TERRAIN_AVAILABLE
        if (_ahrs == NULL || _terrain == NULL || !_terrain->height_amsl(*(Location *)this, alt_terr_cm, true)) {
            return false;
        }
        // convert terrain alt to cm
        alt_terr_cm *= 100.0f;
#else
        return false;
#endif
    }

    // convert alt to absolute
    int32_t alt_abs;
    switch (frame) {
        case ALT_FRAME_ABSOLUTE:
            alt_abs = alt;
            break;
        case ALT_FRAME_ABOVE_HOME:
            alt_abs = alt + _ahrs->get_home().alt;
            break;
        case ALT_FRAME_ABOVE_ORIGIN:
            {
                // fail if we cannot get ekf origin
                Location ekf_origin;
                if (_ahrs == NULL || !_ahrs->get_origin(ekf_origin)) {
                    return false;
                }
                alt_abs = alt + ekf_origin.alt;
            }
            break;
        case ALT_FRAME_ABOVE_TERRAIN:
            alt_abs = alt + alt_terr_cm;
            break;
        default:
            // unknown conversion to absolute alt, this should never happen
            return false;
    }

    // convert absolute to desired frame
    switch (desired_frame) {
        case ALT_FRAME_ABSOLUTE:
            ret_alt_cm = alt_abs;
            return true;
        case ALT_FRAME_ABOVE_HOME:
            ret_alt_cm = alt_abs - _ahrs->get_home().alt;
            return true;
        case ALT_FRAME_ABOVE_ORIGIN:
            {
                // fail if we cannot get ekf origin
                Location ekf_origin;
                if (_ahrs == NULL || !_ahrs->get_origin(ekf_origin)) {
                    return false;
                }
                ret_alt_cm = alt_abs - ekf_origin.alt;
                return true;
            }
        case ALT_FRAME_ABOVE_TERRAIN:
            ret_alt_cm = alt_abs - alt_terr_cm;
            return true;
        default:
            // should never happen
            return false;
    }
}

bool Location_Class::get_vector_xy_from_origin_NEU(Vector3f &vec_neu) const
{
    // convert to neu
    Location ekf_origin;
    if (!_ahrs->get_origin(ekf_origin)) {
        return false;
    }
    vec_neu.x = (lat-ekf_origin.lat) * LATLON_TO_CM;
    vec_neu.y = (lng-ekf_origin.lng) * LATLON_TO_CM * longitude_scale(ekf_origin);
    return true;
}

bool Location_Class::get_vector_from_origin_NEU(Vector3f &vec_neu) const
{
    // convert lat, lon
    if (!get_vector_xy_from_origin_NEU(vec_neu)) {
        return false;
    }

    // convert altitude
    int32_t alt_above_origin_cm = 0;
    if (!get_alt_cm(ALT_FRAME_ABOVE_ORIGIN, alt_above_origin_cm)) {
        return false;
    }
    vec_neu.z = alt_above_origin_cm;

    return true;
}

// return distance in meters between two locations
float Location_Class::get_distance(const struct Location &loc2) const
{
    float dlat = (float)(loc2.lat - lat);
    float dlng = ((float)(loc2.lng - lng)) * longitude_scale(loc2);
    return norm(dlat, dlng) * LOCATION_SCALING_FACTOR;
}

// extrapolate latitude/longitude given distances (in meters) north and east
void Location_Class::offset(float ofs_north, float ofs_east)
{
    // use is_equal() because is_zero() is a local class conflict and is_zero() in AP_Math does not belong to a class
    if (!is_equal(ofs_north, 0.0f) || !is_equal(ofs_east, 0.0f)) {
        int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
        int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(*this);
        lat += dlat;
        lng += dlng;
    }
}
