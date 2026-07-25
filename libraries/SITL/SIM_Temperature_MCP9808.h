#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_MCP9808_ENABLED

#include "SIM_I2CDevice.h"

/*
  Simulator for the MCP9808 temperature sensor

  Datasheet:
  https://ww1.microchip.com/downloads/en/DeviceDoc/25095A.pdf

*/

namespace SITL {

class MCP9808DevReg : public I2CRegEnum {
public:
    static constexpr uint8_t CONFIG        { 0x01 };
    static constexpr uint8_t AMBIENT_TEMP  { 0x05 };
    static constexpr uint8_t RESOLUTION    { 0x08 };
};

class MCP9808 : public I2CDevice, private I2CRegisters_ConfigurableLength
{
public:

    void init() override;

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    // should be a call on aircraft:
    float some_temperature = 25;

    uint32_t last_temperature_update_ms;
};

} // namespace SITL

#endif  // AP_SIM_TEMPERATURE_MCP9808_ENABLED