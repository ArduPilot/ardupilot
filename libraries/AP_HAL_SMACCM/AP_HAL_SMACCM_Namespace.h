
#ifndef __AP_HAL_SMACCM_NAMESPACE_H__
#define __AP_HAL_SMACCM_NAMESPACE_H__

/* While not strictly required, names inside the SMACCM namespace are prefixed
 * with SMACCM for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace SMACCM {
    class SMACCMUARTDriver;
    class SMACCMI2CDriver;
    class SMACCMSPIDeviceManager;
    class SMACCMSPIDeviceDriver;
    class SMACCMAnalogSource;
    class SMACCMAnalogIn;
    class SMACCMStorage;
    class SMACCMConsoleDriver;
    class SMACCMGPIO;
    class SMACCMDigitalSource;
    class SMACCMRCInput;
    class SMACCMRCOutput;
    class SMACCMSemaphore;
    class SMACCMScheduler;
    class SMACCMUtil;
    class SMACCMPrivateMember;
}

#endif // __AP_HAL_SMACCM_NAMESPACE_H__

