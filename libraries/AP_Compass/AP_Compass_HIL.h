/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_HIL_H
#define AP_Compass_HIL_H

#include "Compass.h"

class AP_Compass_HIL : public AP_Compass_Backend
{
public:
    AP_Compass_HIL(Compass &compass);
    void read(void);
    bool init(void);
    void accumulate(void);

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

private:
	uint8_t     _compass_instance;
	Vector3f _sum;
    uint32_t _count;
};

#endif
