/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  dummy backend for HIL (and SITL). This doesn't actually need to do
  any work, as setHIL() is in the frontend
 */

#ifndef __AP_BARO_HIL_H__
#define __AP_BARO_HIL_H__

#include "AP_Baro.h"

class AP_Baro_HIL : public AP_Baro_Backend
{
public:
    AP_Baro_HIL(AP_Baro &baro);
    void update(void);

private:
    uint8_t _instance;
};

#endif //  __AP_BARO_HIL_H__
