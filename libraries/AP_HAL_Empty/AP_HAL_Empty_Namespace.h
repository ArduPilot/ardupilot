
#ifndef __AP_HAL_EMPTY_NAMESPACE_H__
#define __AP_HAL_EMPTY_NAMESPACE_H__

/* While not strictly required, names inside the Empty namespace are prefixed
 * with Empty for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace Empty {
    class EmptyUARTDriver;
    class EmptyI2CDriver;
    class EmptySPIDeviceManager;
    class EmptySPIDeviceDriver;
    class EmptyAnalogSource;
    class EmptyAnalogIn;
    class EmptyStorage;
    class EmptyGPIO;
    class EmptyDigitalSource;
    class EmptyRCInput;
    class EmptyRCOutput;
    class EmptySemaphore;
    class EmptyScheduler;
    class EmptyUtil;
    class EmptyPrivateMember;
}

#endif // __AP_HAL_EMPTY_NAMESPACE_H__

