// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	limits.h
/// @brief	Imposes limits on location (geofence), altitude and other parameters.
///         Each limit breach will trigger an action or set of actions to recover. Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#ifndef __AP_LIMIT_GPSLOCK_H__
#define __AP_LIMIT_GPSLOCK_H__

#include "AP_Limits.h"
#include "AP_Limit_Module.h"

#include <GPS.h>

class AP_Limit_GPSLock : public AP_Limit_Module {

public:
    AP_Limit_GPSLock(GPS *&gps);
    bool        init();
    bool        triggered();

    static const struct AP_Param::GroupInfo         var_info[];

protected:
    GPS *&                                          _gps;
};

#endif // __AP_LIMIT_GPSLOCK_H__
