// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  EM406 GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jason Short, Jordi Muñoz and Jose Julio. DIYDrones.com
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//

#ifndef AP_GPS_406_h
#define AP_GPS_406_h

#include "AP_GPS_SIRF.h"

#define GPS_406_BITRATE         57600

class AP_GPS_406 : public AP_GPS_SIRF
{
public:
    // Methods
    AP_GPS_406(Stream *port);
    virtual void        init(enum GPS_Engine_Setting nav_setting = GPS_ENGINE_NONE);

private:
    void                _change_to_sirf_protocol(void);
    void                _configure_gps(void);
};

#endif

