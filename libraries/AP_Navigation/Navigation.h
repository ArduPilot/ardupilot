
#ifndef AP_NAVIGATION_h
#define AP_NAVIGATION_h

#define XTRACK_GAIN 10                                  // Amount to compensate for crosstrack (degrees/100 per meter)
#define XTRACK_ENTRY_ANGLE 3000                 // Max angle used to correct for track following	degrees*100
#include <GPS.h>                                                // ArduPilot GPS Library
#include "Waypoints.h"                                  // ArduPilot Waypoints Library
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define T7 10000000

class Navigation {
public:
    Navigation(GPS *withGPS, Waypoints *withWP);

    void                    update_gps(void);                                   // called 50 Hz
    void                    set_home(Waypoints::WP loc);
    void                    set_next_wp(Waypoints::WP loc);
    void                    load_first_wp(void);
    void                    load_wp_with_index(uint8_t i);
    void                    load_home(void);
    void                    return_to_home_with_alt(uint32_t alt);

    void                    reload_wp(void);
    void                    load_wp_index(uint8_t i);
    void                    hold_location();
    void                    set_wp(Waypoints::WP loc);

    void                    set_hold_course(int16_t b);                         // -1 deisables, 0-36000 enables
    int16_t                 get_hold_course(void);

    int32_t                 get_distance(Waypoints::WP *loc1, Waypoints::WP *loc2);
    int32_t                 get_bearing(Waypoints::WP *loc1, Waypoints::WP *loc2);
    void                    set_bearing_error(int32_t error);

    void                    set_loiter_vector(int16_t v);
    void                    update_crosstrack(void);

    int32_t                 wrap_180(int32_t error);                    // utility
    int32_t                 wrap_360(int32_t angle);                    // utility

    int32_t                 bearing;                                                    // deg * 100 : 0 to 360 current desired bearing to navigate
    int32_t                 distance;                                                   // meters : distance plane to next waypoint
    int32_t                 altitude_above_home;                        // meters * 100 relative to home position
    int32_t                 total_distance;                                     // meters : distance between waypoints
    int32_t                 bearing_error;                                      // deg * 100 : 18000 to -18000
    int32_t                 altitude_error;                                     // deg * 100 : 18000 to -18000

    int16_t                 loiter_sum;
    Waypoints::WP           home, location, prev_wp, next_wp;

private:
    void                    calc_int32_t_scaling(int32_t lat);
    void                    calc_bearing_error(void);
    void                    calc_altitude_error(void);
    void                    calc_distance_error(void);
    void                    calc_long_scaling(int32_t lat);
    void                    reset_crosstrack(void);

    int16_t                 _old_bearing;                                       // used to track delta on the bearing
    GPS *                   _gps;
    Waypoints *             _wp;
    int32_t                 _crosstrack_bearing;                                // deg * 100 : 0 to 360 desired angle of plane to target
    float                   _crosstrack_error;                                  // deg * 100 : 18000 to -18000  meters we are off trackline
    int16_t                 _hold_course;                                       // deg * 100 dir of plane
    int32_t                 _target_altitude;                   // used for
    int32_t                 _offset_altitude;                   // used for
    float                   _altitude_estimate;
    float                   _scaleLongUp;                       // used to reverse int32_ttitude scaling
    float                   _scaleLongDown;                     // used to reverse int32_ttitude scaling
    int16_t                 _loiter_delta;
};


#endif // AP_NAVIGATION_h
