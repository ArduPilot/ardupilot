/* SpeedToFly class by Samuel Tabor, 2021.

Calculates the optimal speed to fly given drag polar, expected climb rate in next thermal and
horizontal and vertical air movement between thermals.
*/
#pragma once

#include "Variometer.h"

class SpeedToFly {

    float _CL_estimate = -1.0f;

    const Variometer::PolarParams &_polarParams;

public:
    SpeedToFly(const Variometer::PolarParams &polarParams) :_polarParams(polarParams) {}

    void update(float Wx, float Wz, float Wexp, float CLmin, float CLmax);

    float speed_to_fly(void) {return _CL_estimate>0 ? sqrtf(_polarParams.K/_CL_estimate) : -1.0f;};
};
