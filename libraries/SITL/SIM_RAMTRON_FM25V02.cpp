#include "SIM_RAMTRON_FM25V02.h"

#if AP_SIM_RAMTRON_FM25V02_ENABLED

using namespace SITL;

void RAMTRON_FM25V02::fill_rdid(uint8_t *buffer, uint8_t len)
{
    memcpy(buffer, &manufacturer, ARRAY_SIZE(manufacturer));
    buffer[7] = id1();
    buffer[8] = id2();
}

#endif  // AP_SIM_RAMTRON_FM25V02_ENABLED
