/*
  fake baro backend
 */
#pragma once

#include "AP_Baro_Backend.h"

class AP_Baro_FAKE : public AP_Baro_Backend
{
public:
    AP_Baro_FAKE(AP_Baro &baro);
    void update(void);

    static AP_Baro_Backend *probe(AP_Baro &baro);

private:
    uint8_t instance;
    const float fake_pressure = 931.0;
    const float fake_temperature = 25.0;
};
