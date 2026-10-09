#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_MAX31865_ENABLED

/*
  Simulator for the MAX31865 RTD temperature converter

  DataSheet: https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf
*/

#include "SIM_SPIDevice.h"

namespace SITL {

class MAX31865 : public SPIDevice
{
public:

    void update(const class Aircraft &aircraft) override;
    int rdwr(uint8_t count, SPI::spi_ioc_transfer *&data) override;

private:

    uint8_t config_reg;
    float _temperature_degC;

};

} // namespace SITL

#endif  // AP_SIM_TEMPERATURE_MAX31865_ENABLED
