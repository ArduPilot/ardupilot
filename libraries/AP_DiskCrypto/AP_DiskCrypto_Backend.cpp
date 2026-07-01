#include "AP_DiskCrypto_Backend.h"

#if AP_DISKCRYPTO_ENABLED

#include "AP_DiskCrypto_SW.h"
#include <AP_Common/AP_Common.h>

AP_DiskCrypto_Backend *AP_DiskCrypto_Backend::create()
{
#if AP_DISKCRYPTO_HW_ENABLED
    // A hardware-accelerated backend (e.g. one wrapping the STM32 CRYP
    // peripheral via ChibiOS hal_crypto) would be selected here, falling
    // through to software if it is unavailable on the running board.
#endif
    return NEW_NOTHROW AP_DiskCrypto_SW;
}

#endif  // AP_DISKCRYPTO_ENABLED
