#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_MCP9600_ENABLED

#include "SIM_I2CDevice.h"

/*
  Simulator for the MCP9600 temperature sensor

  DataSheet; https://www.microchip.com/content/dam/mchp/documents/OTH/ProductDocuments/DataSheets/MCP960X-Data-Sheet-20005426.pdf

*/

namespace SITL {

class MCP9600DevReg : public I2CRegEnum {
public:
    static constexpr uint8_t HOT_JUNC        { 0x00 };
    static constexpr uint8_t SENSOR_STATUS   { 0x04 };
    static constexpr uint8_t SENSOR_CONFIG   { 0x05 };
    static constexpr uint8_t WHOAMI          { 0x20 };
};

class MCP9600 : public I2CDevice, private I2CRegisters_ConfigurableLength
{
public:

    void init() override;

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    // should be a call on aircraft:
    float some_temperature = 26.5;

    uint32_t last_temperature_update_ms;
};

} // namespace SITL

#endif  // AP_SIM_TEMPERATURE_MCP9600_ENABLED
