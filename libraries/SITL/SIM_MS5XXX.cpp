#include "SIM_MS5XXX.h"

#include <SITL/SITL.h>

#include <stdio.h>

using namespace SITL;

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

MS5XXX::MS5XXX() :
    I2CDevice()
{
}

void MS5XXX::reset()
{
    // load prom from internal register:
    prom_loaded = true;
    load_prom(loaded_prom, sizeof(loaded_prom));
}

void MS5XXX::convert_D1()
{
    float P_Pa;
    float temperature;
    get_pressure_temperature_readings(P_Pa, temperature);

    if (P_Pa < 0.1) {
        // maths breaks down on very, very low numbers, or there's a
        // bug in the conversion code.  The simulation can pass in
        // very, very low numbers.  Clamp it.
        P_Pa = 0.1;
    }

    uint32_t D1;
    uint32_t D2;
    convert(P_Pa, temperature, D1, D2);

    // Check the accuracy of the returned conversion by utilizing a copy of the drivers
    check_conversion_accuracy(P_Pa, temperature, D1, D2);

    convert_out[2] = D1 & 0xff;
    D1 >>= 8;
    convert_out[1] = D1 & 0xff;
    D1 >>= 8;
    convert_out[0] = D1 & 0xff;
}
void MS5XXX::convert_D2()
{
    float P_Pa;
    float temperature;
    get_pressure_temperature_readings(P_Pa, temperature);

    if (P_Pa < 0.1) {
        // maths breaks down on very, very low numbers, or there's a
        // bug in the conversion code.  The simulation can pass in
        // very, very low numbers.  Clamp it.
        P_Pa = 0.1;
    }

    uint32_t D1;
    uint32_t D2;
    convert(P_Pa, temperature, D1, D2);
    convert_out[2] = D2 & 0xff;
    D2 >>= 8;
    convert_out[1] = D2 & 0xff;
    D2 >>= 8;
    convert_out[0] = D2 & 0xff;
}


void MS5XXX::update(const class Aircraft &aircraft)
{
    const uint32_t now_us = AP_HAL::micros();
    // static uint32_t then_us = 0;
    // ::fprintf(stderr, "update: s=%u now=%u delta=%u cmd-age=%u\n", (unsigned)state, (unsigned)now_us, (unsigned)(now_us - then_us), (unsigned)(now_us-command_start_us));
    // then_us = now_us;

    switch (state) {
    case State::COLD:
        command_start_us = now_us;
        prom_loaded = false;
        state = State::COLD_WAIT;
        break;
    case State::COLD_WAIT:
        // 1ms to do anything....
        if (now_us - command_start_us < 1) {
            break;
        }
        state = State::UNINITIALISED;
        FALLTHROUGH;
    case State::UNINITIALISED:
        break;
    case State::RESET_START:
        command_start_us = now_us;
        state = State::RESET_WAIT;
        break;
    case State::RESET_WAIT:
        // 2ms for reset to complete (data sheet does not specify?)
        if (now_us - command_start_us > 2000) {
            reset();
            state = State::RUNNING;
            break;
        }
        break;

    case State::CONVERSION_D1_START:
        command_start_us = now_us;
        convert_out[0] = 0;
        convert_out[1] = 0;
        convert_out[2] = 0;
        state = State::CONVERSION_D1_WAIT;
        break;
    case State::CONVERSION_D1_WAIT:
        // driver allows for 10ms for a conversion to happen
        if (now_us - command_start_us > conversion_time_osr_1024_us) {
            convert_D1();
            state = State::RUNNING;
            break;
        }
        break;

    case State::CONVERSION_D2_START:
        command_start_us = now_us;
        convert_out[0] = 0;
        convert_out[1] = 0;
        convert_out[2] = 0;
        state = State::CONVERSION_D2_WAIT;
        break;
    case State::CONVERSION_D2_WAIT:
        // driver allows for 10ms for a conversion to happen
        if (now_us - command_start_us > conversion_time_osr_1024_us) {
            convert_D2();
            state = State::RUNNING;
            break;
        }
        break;

    case State::RUNNING:
        break;
    }

    // float pressure = AP::sitl()->state.airspeed_raw_pressure[0];
    // float temperature = 25.0f;
}

int MS5XXX::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    struct I2C::i2c_msg &msg = data->msgs[0];
    // if (data->nmsgs != 1) {
    //     AP_HAL::panic("nmsgs=%u", data->nmsgs);
    // }
    if (msg.flags == I2C_M_RD) {
        AP_HAL::panic("Read (%u)",msg.len);
        return 0;
    }

    if (msg.len != 1) {
        AP_HAL::panic("bad command length");
    }
    const Command cmd = (Command)msg.buf[0];
    if (state != State::RUNNING) {
        if (state == State::UNINITIALISED &&
            cmd == Command::RESET) {
            // this is OK - RESET is OK in UNINITIALISED
        } else {
            hal.console->printf("Command (0x%02x) received while not running (state=%u)\n", (unsigned)cmd, (unsigned)state);
            return -1;  // we could die instead...
        }
    }

    switch (cmd) {
    case Command::RESET:
        state = State::RESET_START;
        break;
    case Command::READ_C0:
    case Command::READ_C1:
    case Command::READ_C2:
    case Command::READ_C3:
    case Command::READ_C4:
    case Command::READ_C5:
    case Command::READ_C6:
    case Command::READ_CRC: {
        if (data->msgs[1].len != 2) {
            AP_HAL::panic("Unexpected length");
        }
        const uint8_t addr = ((unsigned)cmd - (unsigned)Command::READ_C0)/2;
        const uint16_t val = htobe16(loaded_prom[addr]);
        data->msgs[1].buf[0] = val & 0xff;
        data->msgs[1].buf[1] = val >> 8;
        break;
    }
    case Command::CONVERT_D1_OSR_1024:
        state = State::CONVERSION_D1_START;
        break;
    case Command::CONVERT_D2_OSR_1024:
        state = State::CONVERSION_D2_START;
        break;
    case Command::READ_CONVERSION:
        if (data->msgs[1].len == 0) {
            // upon not getting a reading back the driver commands a
            // conversion-read but doesn't wait for a response!
            hal.console->printf("read of length zero\n");
            return -1;
        }
        if (data->msgs[1].len != 3) {
            AP_HAL::panic("Unexpected length=%u", data->msgs[1].len);
        }
        data->msgs[1].buf[0] = convert_out[0];
        data->msgs[1].buf[1] = convert_out[1];
        data->msgs[1].buf[2] = convert_out[2];
        break;
    default:
        AP_HAL::panic("Unknown command %u (0x%02x)", (unsigned)cmd, (unsigned)cmd);
    }
    return 0;
}
