/*
 * Location.cpp
 */

#include "Location.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Terrain/AP_Terrain.h>

AP_Terrain *Location::_terrain = nullptr;

/// constructors
Location::Location()
{
    zero();
}

const Location definitely_zero{};
bool Location::is_zero(void) const
{
    return !memcmp(this, &definitely_zero, sizeof(*this));
}

void Location::zero(void)
{
    memset(this, 0, sizeof(*this));
}

// Construct location using position (NEU) from ekf_origin for the given altitude frame
Location::Location(int32_t latitude, int32_t longitude, int32_t alt_in_cm, AltFrame frame)
{
    zero();
    lat = latitude;
    lng = longitude;
    set_alt_cm(alt_in_cm, frame);
}

Location::Location(const Vector3f &ekf_offset_neu, AltFrame frame)
{
    zero();

    // store alt and alt frame
    set_alt_cm(ekf_offset_neu.z, frame);

    // calculate lat, lon
    Location ekf_origin;
    if (AP::ahrs().get_origin(ekf_origin)) {
        lat = ekf_origin.lat;
        lng = ekf_origin.lng;
        offset(ekf_offset_neu.x / 100.0f, ekf_offset_neu.y / 100.0f);
    }
}

void Location::set_alt_cm(int32_t alt_cm, AltFrame frame)
{
    alt = alt_cm;
    relative_alt = false;
    terrain_alt = false;
    origin_alt = false;
    switch (frame) {
        case AltFrame::ABSOLUTE:
            // do nothing
            break;
        case AltFrame::ABOVE_HOME:
            relative_alt = true;
            break;
        case AltFrame::ABOVE_ORIGIN:
            origin_alt = true;
            break;
        case AltFrame::ABOVE_TERRAIN:
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            relative_alt = true;
            terrain_alt = true;
            break;
    }
}

// converts altitude to new frame
bool Location::change_alt_frame(AltFrame desired_frame)
{
    int32_t new_alt_cm;
    if (!get_alt_cm(desired_frame, new_alt_cm)) {
        return false;
    }
    set_alt_cm(new_alt_cm, desired_frame);
    return true;
}

// get altitude frame
Location::AltFrame Location::get_alt_frame() const
{
    if (terrain_alt) {
        return AltFrame::ABOVE_TERRAIN;
    }
    if (origin_alt) {
        return AltFrame::ABOVE_ORIGIN;
    }
    if (relative_alt) {
        return AltFrame::ABOVE_HOME;
    }
    return AltFrame::ABSOLUTE;
}

/// get altitude in desired frame
bool Location::get_alt_cm(AltFrame desired_frame, int32_t &ret_alt_cm) const
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!initialised()) {
        AP_HAL::panic("Should not be called on invalid location: Location cannot be (0, 0, 0)");
    }
#endif
    Location::AltFrame frame = get_alt_frame();

    // shortcut if desired and underlying frame are the same
    if (desired_frame == frame) {
        ret_alt_cm = alt;
        return true;
    }

    // check for terrain altitude
    float alt_terr_cm = 0;
    if (frame == AltFrame::ABOVE_TERRAIN || desired_frame == AltFrame::ABOVE_TERRAIN) {
#if AP_TERRAIN_AVAILABLE
        if (_terrain == nullptr || !_terrain->height_amsl(*this, alt_terr_cm, true)) {
            return false;
        }
        // convert terrain alt to cm
        alt_terr_cm *= 100.0f;
#else
        return false;
#endif
    }

    // convert alt to absolute
    int32_t alt_abs = 0;
    switch (frame) {
        case AltFrame::ABSOLUTE:
            alt_abs = alt;
            break;
        case AltFrame::ABOVE_HOME:
            if (!AP::ahrs().home_is_set()) {
                return false;
            }
            alt_abs = alt + AP::ahrs().get_home().alt;
            break;
        case AltFrame::ABOVE_ORIGIN:
            {
                // fail if we cannot get ekf origin
                Location ekf_origin;
                if (!AP::ahrs().get_origin(ekf_origin)) {
                    return false;
                }
                alt_abs = alt + ekf_origin.alt;
            }
            break;
        case AltFrame::ABOVE_TERRAIN:
            alt_abs = alt + alt_terr_cm;
            break;
    }

    // convert absolute to desired frame
    switch (desired_frame) {
        case AltFrame::ABSOLUTE:
            ret_alt_cm = alt_abs;
            return true;
        case AltFrame::ABOVE_HOME:
            if (!AP::ahrs().home_is_set()) {
                return false;
            }
            ret_alt_cm = alt_abs - AP::ahrs().get_home().alt;
            return true;
        case AltFrame::ABOVE_ORIGIN:
            {
                // fail if we cannot get ekf origin
                Location ekf_origin;
                if (!AP::ahrs().get_origin(ekf_origin)) {
                    return false;
                }
                ret_alt_cm = alt_abs - ekf_origin.alt;
                return true;
            }
        case AltFrame::ABOVE_TERRAIN:
            ret_alt_cm = alt_abs - alt_terr_cm;
            return true;
    }
    return false;
}

bool Location::get_vector_xy_from_origin_NE(Vector2f &vec_ne) const
{
    Location ekf_origin;
    if (!AP::ahrs().get_origin(ekf_origin)) {
        return false;
    }
    vec_ne.x = (lat-ekf_origin.lat) * LATLON_TO_CM;
    vec_ne.y = (lng-ekf_origin.lng) * LATLON_TO_CM * ekf_origin.longitude_scale();
    return true;
}

bool Location::get_vector_from_origin_NEU(Vector3f &vec_neu) const
{
    // convert lat, lon
    Vector2f vec_ne;
    if (!get_vector_xy_from_origin_NE(vec_ne)) {
        return false;
    }
    vec_neu.x = vec_ne.x;
    vec_neu.y = vec_ne.y;

    // convert altitude
    int32_t alt_above_origin_cm = 0;
    if (!get_alt_cm(AltFrame::ABOVE_ORIGIN, alt_above_origin_cm)) {
        return false;
    }
    vec_neu.z = alt_above_origin_cm;

    return true;
}

// return distance in meters between two locations
float Location::get_distance(const struct Location &loc2) const
{
    float dlat = (float)(loc2.lat - lat);
    float dlng = ((float)(loc2.lng - lng)) * loc2.longitude_scale();
    return norm(dlat, dlng) * LOCATION_SCALING_FACTOR;
}

// return ETA (Estimated Time of Arrival) from Location to loc2 in Seconds taking current Wind & Airspeed estimates into account
// Should be used with caution - the return value's sliding average of a reasonably long time period (ie 5..30 seconds) is suggested for estimation
// Negative return value should never be considered as part of any sliding average, as -1 is used to indicate infinately long ETA
// Note that AP::ahrs().wind_estimate() only appears to provide realistic estimate after flying a course including at least 180 degree turns - currently
// Note that this implementation does not (yet) take into account the turning phase - considered negligable compared to distance between waypoints
// Note that max airspeed available to fight wind is considered currently equal to currently achieved airspeed (by using calc_gndspee_undershoot); should be refined later
float Location::get_ETA(const struct Location &loc2) const
{
	float airspeed_measured;
	float c_ETA=-1; //Assume first that location is unreachable; use -1 to indicate infinitely long flying due to headwind>=max airspeed; update if location is reachable
    if (AP::ahrs().airspeed_estimate(airspeed_measured)) {
	    if (airspeed_measured>0) {
		  float airspeed_available = airspeed_measured; //Assume that our available max airspeed to fight wind is equal to currently achieved airspeed (by using calc_gndspee_undershoot); should be refined later
		  Vector3f nedWind = AP::ahrs().wind_estimate();
		  Vector2f horizontal_wind_vect; //use only the horizontal components for this calculation
		  horizontal_wind_vect.x = nedWind.x;  
		  horizontal_wind_vect.y = nedWind.y;
		  float l_bearing = radians(get_bearing_to(loc2)*0.01f); //radians
		  float w_bearing = atan2f(-horizontal_wind_vect.y,-horizontal_wind_vect.x); //wind vector direction
          float l_dis = get_distance(loc2); //distance to location in meters
		  float angle_diff = l_bearing-w_bearing; //difference between bearing angle to location VS wind vector direction
		  float wind_on_bearing = cosf(angle_diff)*horizontal_wind_vect.length(); //Calculate the Wind vector component along the bearing-to-location direction
		  float wind_prep_to_bearing = sinf(angle_diff)*horizontal_wind_vect.length(); //Calculate the Wind vector component prependicular to the bearing-to-location direction - we need to compensate this by heading
		  float wind_counter_angle = acosf(wind_prep_to_bearing / airspeed_available); //Get the Heading angle correction eeded to compensate the effect of wind vector
		  float airspeed_on_bearing = airspeed_available*sinf(wind_counter_angle); //Get the Airspeed component we have remaining after wind-compensation - we can use this to proceed towards target location
		  float expected_groundspeed_towards_location = airspeed_on_bearing-wind_on_bearing; //We still have to find Wind (projected component in lint with bearing-to-location vector)
		  if (expected_groundspeed_towards_location>0) { //Only take into consideration if expected Ground speed towards Location is reachable with current Wind conditions
		      c_ETA= l_dis / expected_groundspeed_towards_location; //Current Estimated Time of Arrival to currently examined loc2, taking Wind speed & direction into account
			}
		}
	}
	return(c_ETA); //-1 if locations appears to be unreachable because of headwind component is larger than airspeed - Should be averaged over a reasonable period of time!
}

/*
  return the distance in meters in North/East plane as a N/E vector
  from loc1 to loc2
 */
Vector2f Location::get_distance_NE(const Location &loc2) const
{
    return Vector2f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    (loc2.lng - lng) * LOCATION_SCALING_FACTOR * longitude_scale());
}

// return the distance in meters in North/East/Down plane as a N/E/D vector to loc2
Vector3f Location::get_distance_NED(const Location &loc2) const
{
    return Vector3f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    (loc2.lng - lng) * LOCATION_SCALING_FACTOR * longitude_scale(),
                    (alt - loc2.alt) * 0.01f);
}

// extrapolate latitude/longitude given distances (in meters) north and east
void Location::offset(float ofs_north, float ofs_east)
{
    const int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
    const int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale();
    lat += dlat;
    lng += dlng;
}

/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void Location::offset_bearing(float bearing_deg, float distance)
{
    const float ofs_north = cosf(radians(bearing_deg)) * distance;
    const float ofs_east  = sinf(radians(bearing_deg)) * distance;
    offset(ofs_north, ofs_east);
}

// extrapolate latitude/longitude given bearing, pitch and distance
void Location::offset_bearing_and_pitch(float bearing_deg, float pitch_deg, float distance)
{
    const float ofs_north =  cosf(radians(pitch_deg)) * cosf(radians(bearing_deg)) * distance;
    const float ofs_east  =  cosf(radians(pitch_deg)) * sinf(radians(bearing_deg)) * distance;
    offset(ofs_north, ofs_east);
    const int32_t dalt =  sinf(radians(pitch_deg)) * distance *100.0f;
    alt += dalt; 
}


float Location::longitude_scale() const
{
    float scale = cosf(lat * (1.0e-7f * DEG_TO_RAD));
    return MAX(scale, 0.01f);
}

/*
 * convert invalid waypoint with useful data. return true if location changed
 */
bool Location::sanitize(const Location &defaultLoc)
{
    bool has_changed = false;
    // convert lat/lng=0 to mean current point
    if (lat == 0 && lng == 0) {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    // convert relative alt=0 to mean current alt
    if (alt == 0 && relative_alt) {
        relative_alt = false;
        alt = defaultLoc.alt;
        has_changed = true;
    }

    // limit lat/lng to appropriate ranges
    if (!check_latlng()) {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    return has_changed;
}

// make sure we know what size the Location object is:
assert_storage_size<Location, 16> _assert_storage_size_Location;


// return bearing in centi-degrees from location to loc2
int32_t Location::get_bearing_to(const struct Location &loc2) const
{
    const int32_t off_x = loc2.lng - lng;
    const int32_t off_y = (loc2.lat - lat) / loc2.longitude_scale();
    int32_t bearing = 9000 + atan2f(-off_y, off_x) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

/*
  return true if lat and lng match. Ignores altitude and options
 */
bool Location::same_latlon_as(const Location &loc2) const
{
    return (lat == loc2.lat) && (lng == loc2.lng);
}

// return true when lat and lng are within range
bool Location::check_latlng() const
{
    return check_lat(lat) && check_lng(lng);
}

// see if location is past a line perpendicular to
// the line between point1 and point2 and passing through point2.
// If point1 is our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool Location::past_interval_finish_line(const Location &point1, const Location &point2) const
{
    return this->line_path_proportion(point1, point2) >= 1.0f;
}

/*
  return the proportion we are along the path from point1 to
  point2, along a line parallel to point1<->point2.

  This will be more than 1 if we have passed point2
 */
float Location::line_path_proportion(const Location &point1, const Location &point2) const
{
    const Vector2f vec1 = point1.get_distance_NE(point2);
    const Vector2f vec2 = point1.get_distance_NE(*this);
    const float dsquared = sq(vec1.x) + sq(vec1.y);
    if (dsquared < 0.001f) {
        // the two points are very close together
        return 1.0f;
    }
    return (vec1 * vec2) / dsquared;
}
