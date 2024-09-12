#include "SIM_config.h"

#if AP_SIM_TSYS03_ENABLED

#include "SIM_I2CDevice.h"

/*
  Simulator for the TSYS03 temperature sensor

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -f Callisto -A --speedup=1
param set TEMP1_TYPE 4
param fetch
param set TEMP1_BUS 2
param set TEMP1_SRC 3
param set TEMP1_SRC_ID 1
graph BATTERY_STATUS.temperature*0.01
reboot

arm throttle
rc 3 2000

fly around, check BAT[0].temperature

*/

namespace SITL {

class TSYS03 : public I2CDevice
{
public:

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    int rdwr_handle_read(I2C::i2c_rdwr_ioctl_data *&data);
    int rdwr_handle_write(I2C::i2c_rdwr_ioctl_data *&data);

    // should be a call on aircraft:
    float last_temperature = -1000.0f;

    enum class State {
        UNKNOWN = 22,
        RESET = 23,
        READ_SERIAL = 24,
        IDLE = 25,
        CONVERTING = 26,
        CONVERTED = 27,
    } state = State::RESET;

    uint32_t state_start_time_ms;

    void set_state(State new_state) {
        state = new_state;
        state_start_time_ms = AP_HAL::millis();
    }
    uint32_t time_in_state_ms() const {
        return AP_HAL::millis() - state_start_time_ms;
    }

    float get_sim_temperature(const class Aircraft &aircraft) const;

    float temperature_for_adc(uint16_t adc) const;
    uint16_t calculate_adc(float temperature) const;
    uint32_t adc;

    enum class Command {
        RESET       = 0x1E,
        READ_SERIAL = 0x0A,
        CONVERT     = 0x46,
        READ_ADC    = 0x00,
    };

    static constexpr uint8_t serial[] { 0xAB, 0xCD, 0xEF };
};

} // namespace SITL

#endif  // AP_SIM_TSYS03_ENABLED
