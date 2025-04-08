#include "SIM_config.h"

#if AP_SIM_TSYS03_ENABLED

#include "SIM_Temperature_TSYS03.h"

#include <stdio.h>

#include <GCS_MAVLink/GCS.h>

constexpr const uint8_t SITL::TSYS03::serial[3];

int SITL::TSYS03::rdwr_handle_read(I2C::i2c_rdwr_ioctl_data *&data)
{
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
    case Command::READ_SERIAL: {
        if (state != State::RESET) {
            // not sure if this is illegal or not?
            AP_HAL::panic("reading serial outside RESET state");
        }
        if (data->msgs[1].len != 4) {
            AP_HAL::panic("Unexpected serial read length");
        }
        memcpy(data->msgs[1].buf, serial, 3);
        uint8_t crc = 0;
        for (uint8_t i=0; i<3; i++) {
            crc = crc8_dvb(crc, serial[i], 0x31);
        }
        data->msgs[1].buf[3] = crc;
        break;
    }
    case Command::CONVERT:
        AP_HAL::panic("Bad CONVERT");
    case Command::READ_ADC: {
        if (data->msgs[1].len != 3) {
            AP_HAL::panic("Unexpected adc read length");
        }
        if (state == State::CONVERTING) {
            // we've been asked for values while still converting.
            // Return zeroes
        } else if (state == State::CONVERTED) {
            data->msgs[1].buf[1] = adc & 0xff;
            data->msgs[1].buf[0] = adc >> 8;

            uint8_t crc = 0;
            for (uint8_t i=0; i<2; i++) {
                crc = crc8_dvb(crc, data->msgs[1].buf[i], 0x31);
            }
            data->msgs[1].buf[2] = crc;

            set_state(State::IDLE);
        } else {
            // AP_HAL::panic("READ_ADC in bad state");
            // this happens at startup
            return -1;
        }
        break;
    }
    }
    return 0;
}

int SITL::TSYS03::rdwr_handle_write(I2C::i2c_rdwr_ioctl_data *&data)
{
    // incoming write-only command
    const auto &msg = data->msgs[0];
    const uint8_t cmd = msg.buf[0];

    switch ((Command)cmd) {
    case Command::RESET:
        set_state(State::RESET);
        break;
    case Command::READ_SERIAL:
        AP_HAL::panic("bad serial read");
    case Command::CONVERT:
        if (state != State::RESET &&
            state != State::CONVERTING &&
            state != State::IDLE &&
            state != State::READ_SERIAL) {
            AP_HAL::panic("Convert outside reset/idle");
        }
        set_state(State::CONVERTING);
        break;
    case Command::READ_ADC:
        AP_HAL::panic("bad READ_ADC");
    }
    return 0;
}

int SITL::TSYS03::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2) {
        return rdwr_handle_read(data);
    }

    if (data->nmsgs == 1) {
        return rdwr_handle_write(data);
    }
    return -1;
}

// swiped from the driver:
float SITL::TSYS03::temperature_for_adc(const uint16_t _adc) const
{
    const float temperature = -40.0 + _adc * 165 / (powf(2, 16) - 1.0);

    return temperature;
}

uint16_t SITL::TSYS03::calculate_adc(float temperature) const
{
    // bisect to find the adc24 value:
    uint16_t min_adc = 0;
    uint16_t max_adc = 0xffff;
    uint16_t current_adc = (min_adc+(uint64_t)max_adc)/2;
    float current_error = fabsf(temperature_for_adc(current_adc) - temperature);
    bool bisect_down = false;

    // temperature_for_adc(9378708);  // should be 10.59

    while (labs(int32_t(max_adc - min_adc)) > 1 && current_error > 0.05) {
        uint16_t candidate_adc;
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

void SITL::TSYS03::update(const class Aircraft &aircraft)
{
    switch (state) {
    case State::UNKNOWN:
        break;
    case State::RESET:
        if (time_in_state_ms() > 2) {
            set_state(State::IDLE);
        }
        break;
    case State::READ_SERIAL:
        break;
    case State::IDLE:
        break;
    case State::CONVERTING:
        if (time_in_state_ms() > 5) {
            const float temperature = get_sim_temperature(aircraft);
            if (!is_equal(last_temperature, temperature)) {
                last_temperature = temperature;
                adc = calculate_adc(KELVIN_TO_C(temperature));
            }
            set_state(State::CONVERTED);
        }
        break;
    case State::CONVERTED:
        break;
    }
}

float SITL::TSYS03::get_sim_temperature(const Aircraft &aircraft) const
{
    return aircraft.get_battery_temperature();
}

#endif  // AP_SIM_TSYS03_ENABLED
