/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_HIL_H
#define AP_Compass_HIL_H

#include "Compass.h"

#if COMPASS_MAX_INSTANCES == 1
# define HIL_NUM_COMPASSES 1
#else
# define HIL_NUM_COMPASSES 2
#endif

class AP_Compass_HIL : public AP_Compass_Backend
{
public:
    AP_Compass_HIL(Compass &compass);
    void read(void);
    bool init(void);

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

private:
    uint8_t     _compass_instance[HIL_NUM_COMPASSES];
};

#endif
