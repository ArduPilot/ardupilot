#pragma once

#include "SIM_SPI.h"

#include <SITL/SIM_Aircraft.h>

namespace SITL {

class SPIDevice {
public:
    virtual void init() {}

    virtual void update(const class Aircraft &aircraft) { }

    virtual int rdwr(uint8_t count, SPI::spi_ioc_transfer *&data) = 0;
};

} // namespace SITL
