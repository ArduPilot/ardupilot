#pragma once

#include "AP_Compass.h"

#define HIL_NUM_COMPASSES 1

class AP_Compass_HIL : public AP_Compass_Backend
{
public:
    AP_Compass_HIL();
    void read(void) override;
    bool init(void);

    // detect the sensor
    static AP_Compass_Backend *detect();

private:
    uint8_t     _compass_instance[HIL_NUM_COMPASSES];
};
