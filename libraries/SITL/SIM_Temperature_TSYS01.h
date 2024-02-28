#include "SIM_I2CDevice.h"

/*
  Simulator for the TSYS01 temperature sensor

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduSub -A --speedup=1

*/

namespace SITL {

class TSYS01 : public I2CDevice
{
public:

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    // should be a call on aircraft:
    float last_temperature = -1000.0f;

    enum class State {
        UNKNOWN = 22,
        RESET = 23,
        READ_PROM = 24,
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

    float get_sim_temperature() const;

    float temperature_for_adc(uint32_t adc) const;
    uint32_t calculate_adc(float temperature) const;
    uint32_t adc;

    enum class Command {
        RESET       = 0x1E,
        READ_PROM0  = 0xA0,
        READ_PROM1  = 0xA2,
        READ_PROM2  = 0xA4,
        READ_PROM3  = 0xA6,
        READ_PROM4  = 0xA8,
        READ_PROM5  = 0xAA,
        CONVERT     = 0x40,
        READ_ADC    = 0x00,
    };

    static constexpr int32_t _k[] { 40781, 32791, 36016, 24926, 28446 };
};

} // namespace SITL
