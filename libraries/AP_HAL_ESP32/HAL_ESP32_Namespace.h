#pragma once

namespace ESP32
{
    class AnalogIn;
    class AnalogSource;
    class DigitalSource;
    //class DSP; not on esp32
    class GPIO;
    class I2CBus;
    class I2CDevice;
    class I2CDeviceManager;
    //class OpticalFlow; not on esp32
    class RCInput;
    class RCOutput;
    class Scheduler;
    class Semaphore;
    class BinarySemaphore;
    //class Semaphore_Recursive; no longer used

    class SPIBus;
    //class SPIDesc; not on esp32
    class SPIDevice;
    //class SPIDeviceDriver; not on esp32
    class SPIDeviceManager;

    // class WSPIBus;           not on esp32
    // class WSPIDesc;          not on esp32
    // class WSPIDevice;        not on esp32
    // class WSPIDeviceManager; not on esp32

    class Storage;
    //class EEPROMStorage; no longer used

    class UARTDriver;
    class WiFiDriver;   // its actual a uart driver variant
    class WiFiUdpDriver;// its actual a uart driver variant

    class Util;

    //three ways of reading RC-in signals.
    class RmtSigReader; // uses RMT peripheral and better?
    class SoftSigReader;
    class SoftSigReaderInt;

    class CANIface; //AP_Periph builds need this 

    class DeviceBus; 



}  // namespace ESP32
