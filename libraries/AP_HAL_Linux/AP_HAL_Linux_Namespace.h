
#ifndef __AP_HAL_LINUX_NAMESPACE_H__
#define __AP_HAL_LINUX_NAMESPACE_H__

namespace Linux {
    class UARTDriver;
    class SPIUARTDriver;
    class RPIOUARTDriver;
    class I2CDriver;
    class SPIDeviceManager;
    class SPIDeviceDriver;
    class AnalogSource;
    class AnalogIn;
    class Storage;
    class GPIO_BBB;
    class GPIO_RPI;
    class Storage;
    class Storage_FRAM;
    class DigitalSource;
    class RCInput;
    class RCInput_PRU;
    class RCInput_AioPRU;
    class RCInput_Navio;
    class RCInput_Raspilot;
    class RCInput_ZYNQ;
    class RCInput_UDP;
    class RCOutput_PRU;
    class RCOutput_AioPRU;
    class RCOutput_PCA9685;
    class RCOutput_Raspilot;
    class RCOutput_ZYNQ;
    class RCOutput_Bebop;
    class Semaphore;
    class Scheduler;
    class Util;
    class UtilRPI;
    class ToneAlarm;
    class Heat;
    class HeatPwm;
}

#endif // __AP_HAL_LINUX_NAMESPACE_H__

