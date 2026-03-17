#pragma once

#include "SIM_SerialRangeFinder.h"

#if AP_SIM_RF_BENEWAKE_TFA1500_ENABLED

namespace SITL
{

    class RF_Benewake_TFA1500 : public SerialRangeFinder
    {
    public:
        static SerialRangeFinder *create() { return NEW_NOTHROW RF_Benewake_TFA1500(); }

        uint32_t packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen) override;
    };

}

#endif // AP_SIM_RF_BENEWAKE_TFA1500_ENABLED
