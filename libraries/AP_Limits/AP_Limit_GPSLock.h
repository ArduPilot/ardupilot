// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	limits.h
/// @brief	Imposes limits on location (geofence), altitude and other parameters.
///         Each limit breach will trigger an action or set of actions to recover. Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#include <AP_Limits.h>
#include <AP_Limit_Module.h>

#ifndef GPS_h
 #include <GPS.h>
#endif

#ifndef AP_Limit_GPSLock_H
 #define  AP_Limit_GPSLock_H
#endif  // AP_Limit_GPSLock_H

class AP_Limit_GPSLock : public AP_Limit_Module {

public:
    AP_Limit_GPSLock(GPS *&gps);
    bool        init();
    bool        triggered();

    static const struct AP_Param::GroupInfo         var_info[];

protected:
    GPS *&                                          _gps;


};
