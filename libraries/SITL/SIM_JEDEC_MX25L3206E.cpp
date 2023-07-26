#include "SIM_JEDEC_MX25L3206E.h"

#if AP_SIM_JEDEC_MX25L3206E_ENABLED

using namespace SITL;

void JEDEC_MX25L3206E::fill_rdid(uint8_t *buffer, uint8_t len)
{
    buffer[0] = 0xC2;
    buffer[1] = type;
    buffer[2] = density;
}

void JEDEC_MX25L3206E::fill_rdsr(uint8_t *buffer, uint8_t len)
{
    // we never allow multiple operations at a time
    // hence we can never be busy while reading the status register
    buffer[0] = 0x00;
}

#endif  // AP_SIM_JEDEC_MX25L3206E_ENABLED
