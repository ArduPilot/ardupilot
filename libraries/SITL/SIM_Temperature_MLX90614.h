#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_MLX90614_ENABLED

#include "SIM_I2CDevice.h"

/*
  Simulator for the MLX90614 infrared temperature sensor

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduSub -A --speedup=1

param set TEMP1_TYPE 7
param set TEMP_LOG 1
reboot

The object temperature is the ambient temperature plus
OBJECT_OFFSET_DEGC, as if the sensor was pointed at something warm
like a motor, so the object and ambient readings can be told apart.
*/

namespace SITL {

class MLX90614 : public I2CDevice
{
public:

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    static constexpr float OBJECT_OFFSET_DEGC = 40.0f;

private:

    enum class Command : uint8_t {
        // RAM access is the RAM address with the top three bits clear
        TA    = 0x06,  // ambient (sensor) temperature
        TOBJ1 = 0x07,  // object temperature
    };

    // RAM values, 0.02K per bit
    uint16_t ta_raw;
    uint16_t tobj1_raw;

    static uint16_t raw_for_degC(float degC);
};

} // namespace SITL

#endif  // AP_SIM_TEMPERATURE_MLX90614_ENABLED
