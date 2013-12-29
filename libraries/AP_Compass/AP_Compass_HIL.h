/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_HIL_H
#define AP_Compass_HIL_H

#include "Compass.h"

class AP_Compass_HIL : public Compass
{
public:
    AP_Compass_HIL();
    bool        read(void);
    void        accumulate(void);
    void        setHIL(float roll, float pitch, float yaw);
    void        setHIL(const Vector3i &mag);

private:
    Vector3f    _hil_mag;
    Vector3f    _Bearth;
    float		_last_declination;
    void        _setup_earth_field();
};

#endif
