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

#define LOCATION_ALT_MAX_M  83000   // maximum altitude (in meters) that can be fit into Location structure's alt field

class Location
{
public:

    uint8_t relative_alt : 1;           // 1 if altitude is relative to home
    uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
    uint8_t terrain_alt  : 1;           // this altitude is above terrain
    uint8_t origin_alt   : 1;           // this altitude is above ekf origin
    uint8_t loiter_xtrack : 1;          // 0 to crosstrack from center of waypoint, 1 to crosstrack from tangent exit location

    // note that mission storage only stores 24 bits of altitude (~ +/- 83km)
    int32_t alt;
    int32_t lat;
    int32_t lng;

    /// enumeration of possible altitude types
    enum ALT_FRAME {
        ALT_FRAME_ABSOLUTE = 0,
        ALT_FRAME_ABOVE_HOME = 1,
        ALT_FRAME_ABOVE_ORIGIN = 2,
        ALT_FRAME_ABOVE_TERRAIN = 3
    };

    /// constructors
    Location();
    Location(int32_t latitude, int32_t longitude, int32_t alt_in_cm, ALT_FRAME frame);
    Location(const Vector3f &ekf_offset_neu);

    static void set_terrain(AP_Terrain* terrain) { _terrain = terrain; }

    // set altitude
    void set_alt_cm(int32_t alt_cm, ALT_FRAME frame);

    // get altitude (in cm) in the desired frame
    // returns false on failure to get altitude in the desired frame which
    // can only happen if the original frame or desired frame is above-terrain
    bool get_alt_cm(ALT_FRAME desired_frame, int32_t &ret_alt_cm) const;

    // get altitude frame
    ALT_FRAME get_alt_frame() const;

    // converts altitude to new frame
    // returns false on failure to convert which can only happen if
    // the original frame or desired frame is above-terrain
    bool change_alt_frame(ALT_FRAME desired_frame);

    // get position as a vector from origin (x,y only or x,y,z)
    // return false on failure to get the vector which can only
    // happen if the EKF origin has not been set yet
    // x, y and z are in centimetres
    bool get_vector_xy_from_origin_NE(Vector2f &vec_ne) const;
    bool get_vector_from_origin_NEU(Vector3f &vec_neu) const;

    // return distance in meters between two locations
    float get_distance(const struct Location &loc2) const;

    // extrapolate latitude/longitude given distances (in meters) north and east
    void offset(float ofs_north, float ofs_east);

    bool is_zero(void) const;

    void zero(void);

private:
    static AP_Terrain *_terrain;
};

#endif /* LOCATION_H */
