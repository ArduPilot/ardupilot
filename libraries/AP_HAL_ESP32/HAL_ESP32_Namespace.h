#pragma once

namespace ESP32
{
class UARTDriver;
class WiFiDriver;
class WiFiUdpDriver;
class Scheduler;
class EEPROMStorage;
class AnalogIn;
class RCInput;
class RCOutput;
class ADCSource;
class RCInput;
class Util;
class Semaphore;
class Semaphore_Recursive;
class BinarySemaphore;
class GPIO;
class DigitalSource;
class Storage;
class RmtSigReader;
#if HAL_NUM_CAN_IFACES
class CANIface;
#endif  
}  // namespace ESP32
