#include "Bitmask.h"

#include <AP_HAL/AP_HAL.h>

Bitmask &Bitmask::operator=(const Bitmask&other)
{
    if (other.numwords != numwords || other.numbits != numbits) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        // we really should not be assigning from a bitmask of a
        // different number of bits!
        AP_HAL::panic("attempt to copy from bitmask of different size");
#endif
        // ... but try to cope if it happens in real life:
        if (numwords != other.numwords) {
            delete bits;
            bits = new uint32_t[numwords];
            numwords = other.numwords;
        }
        numbits = other.numbits;
    }
    memcpy(bits, other.bits, sizeof(bits[0])*other.numwords);

    return *this;
}
