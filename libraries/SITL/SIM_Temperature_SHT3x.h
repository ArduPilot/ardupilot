#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_SHT3X_ENABLED

#include "SIM_I2CDevice.h"

/*
  Simulator for the SHT3X temperature sensor

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduSub -A --speedup=1

param set TEMP1_TYPE 8
param set TEMP1_ADDR 0x44

*/

namespace SITL {

class SHT3x : public I2CDevice
{
public:

    SHT3x();

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    // should be a call on aircraft:
    float last_temperature = -1000.0f;

    enum class State {
        UNKNOWN = 22,
        RESET = 23,
        CLEAR_STATUS = 24,
        IDLE = 25,
        MEASURING = 26,
        MEASURED = 27,
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

    void pack_reading(SITL::I2C::i2c_msg &msg);

    enum class Command {
        RESET         = 0x30A2,
        READ_STATUS   = 0xF32D,
        CLEAR_STATUS  = 0x3041,
        MEASURE       = 0x2C06,  // clock-stretching-enabled, high-repatability
        READ_SN       = 0x3780,
    };

    union {
        struct {
            uint8_t checksum_status : 1;
            uint8_t command_status : 1;
            uint8_t reserved1 : 2;
            uint8_t reset_detected : 1;
            uint8_t reserved2 : 5;
            uint8_t T_tracking_alert : 1;
            uint8_t RH_tracking_alert : 1;
            uint8_t reserved3 : 1;
            uint8_t header_status : 1;
            uint8_t reserved4 : 1;
            uint8_t alert_pending : 1;
        };
        uint16_t value;
    } status;

    struct {
        float temperature;
        float humidity;
    } measurement;

    void reset();
};

} // namespace SITL

#endif  // AP_SIM_TEMPERATURE_SHT3X_ENABLED
