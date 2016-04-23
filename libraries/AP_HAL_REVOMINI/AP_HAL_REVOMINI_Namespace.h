
#ifndef __AP_HAL_REVOMINI_NAMESPACE_H__
#define __AP_HAL_REVOMINI_NAMESPACE_H__

/* While not strictly required, names inside the REVOMINI namespace are prefixed
 * with REVOMINI for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace REVOMINI {
    class REVOMINIUARTDriver;
    class REVOMINII2CDriver;
    class REVOMINISPIDeviceManager;
    class REVOMINISPI1DeviceDriver;
    class REVOMINISPI2DeviceDriver;
    class REVOMINISPI3DeviceDriver;
    class REVOMINIAnalogSource;
    class REVOMINIAnalogIn;
    class REVOMINIStorage;
    class REVOMINIGPIO;
    class REVOMINIDigitalSource;
    class REVOMINIRCInput;
    class REVOMINIRCOutput;
    class REVOMINISemaphore;
    class REVOMINIScheduler;
    class REVOMINIUtil;
}

#endif // __AP_HAL_REVOMINI_NAMESPACE_H__

