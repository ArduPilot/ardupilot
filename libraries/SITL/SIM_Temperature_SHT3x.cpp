#include "SIM_config.h"

#if AP_SIM_TEMPERATURE_SHT3X_ENABLED

#include "SIM_Temperature_SHT3x.h"

#include <stdio.h>

using namespace SITL;

SHT3x::SHT3x()
{
    reset();
}

void SHT3x::reset()
{
    status.alert_pending = 1;
    status.reset_detected = 1;
}

void SHT3x::pack_reading(SITL::I2C::i2c_msg &msg)
{
    if (msg.len != 6) {
        AP_HAL::panic("Unexpected length");
    }

    const uint16_t h_deconverted = (measurement.humidity*0.01) * 65535;
    const uint16_t t_deconverted = (((measurement.temperature + 45) / 175) * 65535);

    msg.buf[0] = t_deconverted >> 8;
    msg.buf[1] = t_deconverted & 0xff;
    msg.buf[2] = crc8_generic(&msg.buf[0], 2, 0x31, 0xff);
    msg.buf[3] = h_deconverted >> 8;
    msg.buf[4] = h_deconverted & 0xff;
    msg.buf[5] = crc8_generic(&msg.buf[3], 2, 0x31, 0xff);
}

int SHT3x::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2) {
        // something is expecting a response....
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint16_t command = data->msgs[0].buf[0] << 8 | data->msgs[0].buf[1];
        switch ((Command)command) {
        case Command::RESET:
        case Command::CLEAR_STATUS:
        case Command::MEASURE:
            AP_HAL::panic("Command is write-only?!");
        case Command::READ_STATUS:
            data->msgs[1].buf[0] = status.value >> 16;
            data->msgs[1].buf[1] = status.value & 0xff;
            data->msgs[1].len = 2;
            return 0;
        case Command::READ_SN:
            if (data->msgs[1].len != 6) {
                AP_HAL::panic("Unexpwcted SN length");
            }
            for (uint8_t i=0; i<6; i++) {
                data->msgs[1].buf[i] = i;
            }
            return 0;
        AP_HAL::panic("Bad command 0x%02x", (uint8_t)command);
        }
    }

    if (data->nmsgs == 1 && data->msgs[0].flags == I2C_M_RD) {
        // driver is attempting to retrieve a measurement
        switch (state) {
        case State::UNKNOWN:
        case State::RESET:
        case State::CLEAR_STATUS:
        case State::IDLE:
            AP_HAL::panic("Bad state for measurement retrieval");
        case State::MEASURING:
            AP_HAL::panic("Too soon for measurement (15.5ms for high repeatability, table 4)");
        case State::MEASURED:
            pack_reading(data->msgs[0]);
            return 0;
        }
        AP_HAL::panic("Bad state");
    }

    if (data->nmsgs == 1) {
        // incoming write-only command
        const auto &msg = data->msgs[0];
        const uint16_t cmd = msg.buf[0] << 8 | msg.buf[1];
        switch ((Command)cmd) {
        case Command::RESET:
            set_state(State::RESET);
            return 0;
        case Command::READ_SN:
        case Command::READ_STATUS:
            AP_HAL::panic("Should not get here");
        case Command::CLEAR_STATUS:
            set_state(State::CLEAR_STATUS);
            return 0;
        case Command::MEASURE:
            set_state(State::MEASURING);
            return 0;
        }
        AP_HAL::panic("Bad command 0x%02x", (uint8_t)cmd);
    }
    return -1;
}

void SHT3x::update(const class Aircraft &aircraft)
{
    switch (state) {
    case State::UNKNOWN:
        break;
    case State::RESET:
        if (time_in_state_ms() > 2) {
            set_state(State::IDLE);
        }
        break;
    case State::CLEAR_STATUS:
        if (time_in_state_ms() < 1) {
            return;
        }
        status.value = 0;
        status.command_status = 1;  // cleared succesfully.  Irony.
        set_state(State::IDLE);
        break;
    case State::MEASURING:
        if (time_in_state_ms() > 16) {
            measurement.temperature = get_sim_temperature();
            measurement.humidity = 33.3;
            set_state(State::MEASURED);
        }
        break;
    case State::MEASURED:
        break;
    case State::IDLE:
        break;
    }
}

float SHT3x::get_sim_temperature() const
{
    float sim_alt = AP::sitl()->state.altitude;
    sim_alt += 2 * rand_float();

    // To Do: Add a sensor board temperature offset parameter
    return AP_Baro::get_temperatureC_for_alt_amsl(sim_alt) + 25;
}

#endif  // AP_SIM_TEMPERATURE_SHT3X_ENABLED
