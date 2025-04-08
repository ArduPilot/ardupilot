#pragma once

/*
  compass learning using magnetic field tables from AP_Declination and GSF
 */

class CompassLearn {
public:
    CompassLearn(class Compass &compass);

    // called on each compass read
    void update(void);

private:
    Compass &compass;
};
