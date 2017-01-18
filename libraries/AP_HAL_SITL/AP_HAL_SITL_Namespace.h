#pragma once

namespace HALSITL {
class UARTDriver;
class Scheduler;
class SITL_State;
class EEPROMStorage;
class AnalogIn;
class RCInput;
class RCOutput;
class ADCSource;
class RCInput;
class Util;
class Semaphore;
class GPIO;
class DigitalSource;
class HALSITLCAN;
class HALSITLCANDriver;

// Number of input channels
static const uint8_t SITL_INPUT_MAX_CHANNELS = 12;

// Number of rc input channels
static const uint8_t SITL_RC_INPUT_CHANNELS = 16;

// Number of rc output channels
static const uint8_t SITL_RC_OUTPUT_CHANNELS = 16;

}  // namespace HALSITL
