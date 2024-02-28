#pragma once

#include <AP_AHRS/AP_AHRS.h>

/*
  compass learning using magnetic field tables from AP_Declination and GSF
 */

class CompassLearn {
public:
    CompassLearn(Compass &compass);

    // called on each compass read
    void update(void);

private:
    Compass &compass;
};
