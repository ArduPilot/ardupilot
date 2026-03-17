#include "SIM_RF_Benewake_TFA1500.h"

using namespace SITL;

uint32_t RF_Benewake_TFA1500::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    if (buflen < 5)
    {
        return 0;
    }

    const uint32_t alt_cm = alt_m * 100;

    buffer[0] = 0x5C;
    buffer[1] = alt_cm & 0xFF;
    buffer[2] = (alt_cm >> 8) & 0xFF;
    buffer[3] = (alt_cm >> 16) & 0xFF;
    buffer[4] = (uint8_t)~(buffer[1] + buffer[2] + buffer[3]);

    return 5;
}