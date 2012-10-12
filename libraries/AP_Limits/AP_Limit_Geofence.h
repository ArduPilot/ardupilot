// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	limits.h
/// @brief	Imposes limits on location (geofence), altitude and other parameters.
///         Each limit breach will trigger an action or set of actions to recover. Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#ifndef __AP_LIMIT_GEOFENCE_H__
#define __AP_LIMIT_GEOFENCE_H__

#include "AP_Limits.h"
#include "AP_Limit_Module.h"
#include <AP_Math.h>
#include <AP_Param.h>
#include <GPS.h>

#define MAX_FENCEPOINTS 6

class AP_Limit_Geofence : public AP_Limit_Module {

public:
    AP_Limit_Geofence(uint32_t _eeprom_fence_start, uint8_t fpsize, uint8_t max_fp, GPS *&gps, struct Location *home_loc, struct Location *current_loc);
    bool        init();
    bool        triggered();

    AP_Int8        fence_total();
    void        set_fence_point_with_index(Vector2l &point, uint8_t i);
    Vector2l        get_fence_point_with_index(uint8_t i);
    void            update_boundary();
    bool            boundary_correct();



    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // pointers to gps, current location and home
    GPS *&                  _gps;
    struct Location *       _current_loc;
    struct Location *       _home;


    // Simple mode, just radius
    AP_Int8                 _simple;             // 1 = simple, 0 = complex
    AP_Int16                _radius;             // in meters, for simple mode

    // Complex mode, defined fence points
    AP_Int8                 _fence_total;
    AP_Int8                 _num_points;

private:
    const uint32_t          _eeprom_fence_start;
    const unsigned          _fence_wp_size;
    const unsigned          _max_fence_points;
    bool                    _boundary_uptodate;
    Vector2l                _boundary[MAX_FENCEPOINTS];      // complex mode fence

};


// Helper functions
uint32_t        get_distance_meters(struct Location *loc1, struct Location *loc2); // distance in meters between two locations

#endif // __AP_LIMIT_GEOFENCE_H__
