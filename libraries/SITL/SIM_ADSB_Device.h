#pragma once

#include "SIM_SerialDevice.h"

namespace SITL {

class ADSB_Device : public SerialDevice
{
    using SerialDevice::SerialDevice;
};

};  // namespace SITL
