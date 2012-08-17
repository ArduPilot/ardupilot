// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	limits.h
/// @brief	Imposes limits on location (geofence), altitude and other parameters.
///         Each limit breach will trigger an action or set of actions to recover. Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#include <AP_Limits.h>
#include <AP_Limit_Module.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>

#ifndef AP_Limit_Altitude_H
 #define  AP_Limit_Altitude_H
#endif  // AP_Limit_Altitude_H

class AP_Limit_Altitude : public AP_Limit_Module {

public:
    AP_Limit_Altitude(struct Location *current_loc);

    AP_Int32        min_alt();
    AP_Int32        max_alt();

    bool            init();
    bool            triggered();

    static const struct AP_Param::GroupInfo         var_info[];

protected:
    struct Location *                               _current_loc;
    AP_Int32        _min_alt;
    AP_Int32        _max_alt;


};
