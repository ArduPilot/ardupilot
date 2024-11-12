#pragma once

namespace ChibiOS {
    class AnalogIn;
    class AnalogSource;
    class DigitalSource;
#if HAL_WITH_IO_MCU
    class IOMCU_DigitalSource;
#endif
    class DSP;
    class GPIO;
    class I2CBus;
    class I2CDevice;
    class I2CDeviceManager;
    class OpticalFlow;
    class RCInput;
    class RCOutput;
    class Scheduler;
    class Semaphore;
    class BinarySemaphore;
    class SPIBus;
    class SPIDesc;
    class SPIDevice;
    class SPIDeviceDriver;
    class SPIDeviceManager;
    class WSPIBus;
    class WSPIDesc;
    class WSPIDevice;
    class WSPIDeviceManager;
    class Storage;
    class UARTDriver;
    class Util;
    class Shared_DMA;
    class SoftSigReader;
    class SoftSigReaderInt;
    class CANIface;
    class Flash;
}
