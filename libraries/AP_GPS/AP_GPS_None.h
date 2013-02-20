// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef __AP_GPS_NONE_H__
#define __AP_GPS_NONE_H__

#include <AP_HAL.h>
#include "GPS.h"

class AP_GPS_None : public GPS
{
public:
	AP_GPS_None() : GPS() {}

    virtual void        init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting = GPS_ENGINE_NONE) {
		_port = s;
    };
    virtual bool        read(void) {
        return false;
    };
};
#endif // __AP_GPS_NONE_H__
