#include "SIM_Temperature_TSYS01.h"

#include <stdio.h>

constexpr const int32_t SITL::TSYS01::_k[5];

int SITL::TSYS01::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2) {
        // something is expecting a response....
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint8_t command = data->msgs[0].buf[0];
        switch ((Command)command) {
        case Command::RESET:
            AP_HAL::panic("Bad RESET");
        case Command::READ_PROM0:
        case Command::READ_PROM1:
        case Command::READ_PROM2:
        case Command::READ_PROM3:
        case Command::READ_PROM4:
        case Command::READ_PROM5: {
            if (state != State::RESET) {
                AP_HAL::panic("reading prom outside RESET state");
            }
            if (data->msgs[1].len != 2) {
                AP_HAL::panic("Unexpected prom read length");
            }
            uint8_t offs = 5-((uint8_t(command) - uint8_t(Command::READ_PROM0))/2);
            const uint16_t k = _k[offs];
            data->msgs[1].buf[0] = k >> 8;
            data->msgs[1].buf[1] = k & 0xFF;
            break;
        }
        case Command::CONVERT:
            AP_HAL::panic("Bad CONVERT");
        case Command::READ_ADC: {
            uint8_t registers[3] {};
            if (data->msgs[1].len != sizeof(registers)) {
                AP_HAL::panic("Unexpected prom read length");
            }
            if (state == State::CONVERTING) {
                // we've been asked for values while still converting.
                // Return zeroes per data sheet
            } else if (state == State::CONVERTED) {
                uint32_t value = adc;
                registers[2] = value & 0xff;
                value >>= 8;
                registers[1] = value & 0xff;
                value >>= 8;
                registers[0] = value & 0xff;
                set_state(State::IDLE);
            } else {
                // AP_HAL::panic("READ_ADC in bad state");
                // this happens at startup
                return -1;
            }
            for (uint8_t i=0; i<ARRAY_SIZE(registers); i++) {
                data->msgs[1].buf[i] = registers[i];
            }
            break;
        }
        }
        return 0;
    }

    if (data->nmsgs == 1) {
        // incoming write-only command
        const auto &msg = data->msgs[0];
        const uint8_t cmd = msg.buf[0];

        switch ((Command)cmd) {
        case Command::RESET:
            set_state(State::RESET);
            break;
        case Command::READ_PROM0:
        case Command::READ_PROM1:
        case Command::READ_PROM2:
        case Command::READ_PROM3:
        case Command::READ_PROM4:
        case Command::READ_PROM5:
            AP_HAL::panic("bad prom read");
        case Command::CONVERT:
            if (state != State::RESET &&
                state != State::CONVERTING &&
                state != State::IDLE &&
                state != State::READ_PROM) {
                AP_HAL::panic("Convert outside reset/idle");
            }
            set_state(State::CONVERTING);
            break;
        case Command::READ_ADC:
            AP_HAL::panic("bad READ_ADC");
        }
        return 0;
    }
    return -1;
}

// swiped from the driver:
float SITL::TSYS01::temperature_for_adc(uint32_t _adc) const
{
    const float adc16 = _adc/256.0;
    // const uint32_t _k[] { 28446, 24926, 36016, 32791, 40781 };
    return
        -2   * _k[4] * powf(10, -21) * powf(adc16, 4) +
        4    * _k[3] * powf(10, -16) * powf(adc16, 3) +
        -2   * _k[2] * powf(10, -11) * powf(adc16, 2) +
        1    * _k[1] * powf(10, -6)  * adc16 +
        -1.5 * _k[0] * powf(10, -2);
}

uint32_t SITL::TSYS01::calculate_adc(float temperature) const
{
    // bisect to find the adc24 value:
    uint32_t min_adc = 0;
    uint32_t max_adc = 1<<24;
    uint32_t current_adc = (min_adc+(uint64_t)max_adc)/2;
    float current_error = fabsf(temperature_for_adc(current_adc) - temperature);
    bool bisect_down = false;

    // temperature_for_adc(9378708);  // should be 10.59

    while (labs(int32_t(max_adc - min_adc)) > 1 && current_error > 0.05) {
        uint32_t candidate_adc;
        if (bisect_down) {
            candidate_adc = (min_adc+(uint64_t)current_adc)/2;
        } else {
            candidate_adc = (max_adc+(uint64_t)current_adc)/2;
        }
        const float candidate_temp = temperature_for_adc(candidate_adc);
        const float candidate_error = fabsf(candidate_temp - temperature);
        if (candidate_error > current_error) {
            // worse result
            if (bisect_down) {
                min_adc = candidate_adc;
                bisect_down = false;
            } else {
                max_adc = candidate_adc;
                bisect_down = true;
            }
        } else {
            // better result
            if (bisect_down) {
                max_adc = current_adc;
                bisect_down = false;
            } else {
                min_adc = current_adc;
                bisect_down = true;
            }
            current_adc = candidate_adc;
            current_error = candidate_error;
        }
    }
    return current_adc;
}

void SITL::TSYS01::update(const class Aircraft &aircraft)
{
    switch (state) {
    case State::UNKNOWN:
        break;
    case State::RESET:
        if (time_in_state_ms() > 10) {
            set_state(State::READ_PROM);
        }
        break;
    case State::READ_PROM:
        break;
    case State::IDLE:
        break;
    case State::CONVERTING:
        if (time_in_state_ms() > 5) {
            const float temperature = get_sim_temperature();
            if (!is_equal(last_temperature, temperature)) {
                last_temperature = temperature;
                adc = calculate_adc(temperature);
            }
            set_state(State::CONVERTED);
        }
        break;
    case State::CONVERTED:
        break;
    }
}

float SITL::TSYS01::get_sim_temperature() const
{
    float sim_alt = AP::sitl()->state.altitude;
    sim_alt += 2 * rand_float();

    // To Do: Add a sensor board temperature offset parameter
    return AP_Baro::get_temperatureC_for_alt_amsl(sim_alt) + 25;
}
