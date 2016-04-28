/*
 * Location.h
 *
 */


#ifndef LOCATION_H
#define LOCATION_H

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

class AP_AHRS_NavEKF;
class AP_Terrain;

class Location_Class : public Location
{
public:

    /// enumeration of possible altitude types
    enum ALT_FRAME {
        ALT_FRAME_ABSOLUTE = 0,
        ALT_FRAME_ABOVE_HOME = 1,
        ALT_FRAME_ABOVE_ORIGIN = 2,
        ALT_FRAME_ABOVE_TERRAIN = 3
    };

    /// constructors
    Location_Class();
    Location_Class(int32_t latitude, int32_t longitude, int32_t alt_in_cm, ALT_FRAME frame);
    Location_Class(const Location& loc);
    Location_Class(const Vector3f &ekf_offset_neu);

    /// accept reference to ahrs and (indirectly) EKF
    static void set_ahrs(const AP_AHRS_NavEKF* ahrs) { _ahrs = ahrs; }
    static void set_terrain(AP_Terrain* terrain) { _terrain = terrain; }

    // operators
    Location_Class& operator=(const struct Location loc);

    // set altitude
    void set_alt_cm(int32_t alt_cm, ALT_FRAME frame);

    // get altitude in desired frame
    bool get_alt_cm(ALT_FRAME desired_frame, int32_t &ret_alt_cm) const;

    // get altitude frame
    ALT_FRAME get_alt_frame() const;

    // converts altitude to new frame
    bool change_alt_frame(ALT_FRAME desired_frame);

    // get position as a vector from home (x,y only or x,y,z)
    bool get_vector_xy_from_origin_NEU(Vector3f &vec_neu) const;
    bool get_vector_from_origin_NEU(Vector3f &vec_neu) const;

    // return distance in meters between two locations
    float get_distance(const struct Location &loc2) const;

    // return bearing in centi-degrees from this location to loc2
    int32_t get_bearing_cd(const struct Location &loc2) const;

    // see if location is past a line perpendicular to the line between point1 and point2.
    // if point1 is our previous waypoint and point2 is our target waypoint then this function
    // returns true if we have flown past the target waypoint
    bool passed_point(const struct Location & point1, const struct Location & point2) const;

    // return the proportion we are along the path from point1 to
    // point2. This will be less than >1 if we have passed point2
    float path_proportion(const struct Location &point1, const struct Location &point2) const;

    // extrapolate latitude/longitude given bearing and distance
    // bearing in degrees, distance in meters
    void extrapolate(float bearing, float distance);

    // extrapolate latitude/longitude given distances (in meters) north and east
    void offset(float ofs_north, float ofs_east);

    // return the distance in meters in North/East plane as a N/E vector from loc1 to loc2
    Vector2f diff_2D(const struct Location &loc2) const;

private:
    static const AP_AHRS_NavEKF *_ahrs;
    static AP_Terrain *_terrain;
};

#endif /* LOCATION_H */
