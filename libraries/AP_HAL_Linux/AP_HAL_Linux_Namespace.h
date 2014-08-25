
#ifndef __AP_HAL_LINUX_NAMESPACE_H__
#define __AP_HAL_LINUX_NAMESPACE_H__

/* While not strictly required, names inside the Linux namespace are prefixed
 * with Linux for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace Linux {
    class LinuxUARTDriver;
    class LinuxI2CDriver;
    class LinuxSPIDeviceManager;
    class LinuxSPIDeviceDriver;
    class LinuxAnalogSource;
    class LinuxAnalogIn;
    class LinuxStorage;
    class LinuxGPIO_BBB;
    class LinuxGPIO_RPI;
    class LinuxStorage;
    class LinuxStorage_FRAM;
    class LinuxDigitalSource;
    class LinuxRCInput;
    class LinuxRCInput_PRU;
    class LinuxRCInput_Navio;
    class LinuxRCOutput_PRU;
    class LinuxRCOutput_Navio;
    class LinuxSemaphore;
    class LinuxScheduler;
    class LinuxUtil;
}

#endif // __AP_HAL_LINUX_NAMESPACE_H__

