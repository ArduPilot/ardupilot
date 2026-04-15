#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

class AP_HawkEncoder
{
public:
    static constexpr uint8_t NUM_ENCODERS = 3;

    AP_HawkEncoder() :
        _initialized(false)
    {
        for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
            _state[i].raw_angle = 0;
            _state[i].angle_rad = 0.0f;
            _state[i].last_update_us = 0;
            _state[i].is_healthy = false;
        }
    }

    void init()
    {
        constexpr uint8_t I2C_BUS = 0;

        _dev = AP_HAL::get_HAL().i2c_mgr->get_device(I2C_BUS, AS5600_ADDR);
        if ((bool)_dev) {
            _dev->set_retries(3);
        }

        if (USE_TCA9548A) {
            _mux = AP_HAL::get_HAL().i2c_mgr->get_device(I2C_BUS, TCA9548A_ADDR);
            if ((bool)_mux) {
                _mux->set_retries(3);
            }
        }

        _initialized = (bool)_dev && (!USE_TCA9548A || (bool)_mux);

        if (!_initialized) {
            for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
                _state[i].is_healthy = false;
            }
        }
    }

    void update()
    {
        if (!_initialized) {
            for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
                _state[i].is_healthy = false;
            }
            return;
        }

        for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
            if (!read_one(i)) {
                mark_unhealthy(i);
            }
        }
    }

    bool healthy(uint8_t idx) const
    {
        if (idx >= NUM_ENCODERS) {
            return false;
        }
        return _state[idx].is_healthy;
    }

    bool stale(uint8_t idx, uint32_t timeout_us) const
    {
        if (idx >= NUM_ENCODERS) {
            return true;
        }
        if (!_state[idx].is_healthy) {
            return true;
        }

        const uint32_t now = AP_HAL::micros();
        return (now - _state[idx].last_update_us) > timeout_us;
    }

    float get_angle_rad(uint8_t idx) const
    {
        if (idx >= NUM_ENCODERS) {
            return 0.0f;
        }
        return _state[idx].angle_rad;
    }

    uint16_t get_raw_angle(uint8_t idx) const
    {
        if (idx >= NUM_ENCODERS) {
            return 0;
        }
        return _state[idx].raw_angle;
    }

    uint32_t get_last_update_us(uint8_t idx) const
    {
        if (idx >= NUM_ENCODERS) {
            return 0;
        }
        return _state[idx].last_update_us;
    }

private:
    static constexpr uint8_t AS5600_ADDR = 0x36;
    static constexpr uint8_t AS5600_RAW_ANGLE = 0x0C;

    static constexpr bool USE_TCA9548A = true;
    static constexpr uint8_t TCA9548A_ADDR = 0x70;

    struct EncoderState {
        uint16_t raw_angle;
        float angle_rad;
        uint32_t last_update_us;
        bool is_healthy;
    };

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _mux;

    EncoderState _state[NUM_ENCODERS];
    bool _initialized;

    bool select_mux_channel(uint8_t chan)
    {
        if (!USE_TCA9548A) {
            return true;
        }

        if (!(bool)_mux || chan > 7) {
            return false;
        }

        uint8_t value = (1U << chan);
        return _mux->transfer(&value, 1, nullptr, 0);
    }

    bool read_one(uint8_t idx)
    {
        if (idx >= NUM_ENCODERS || !(bool)_dev) {
            return false;
        }

        if (!select_mux_channel(idx)) {
            return false;
        }

        uint8_t reg = AS5600_RAW_ANGLE;
        uint8_t buf[2] = {0, 0};

        if (!_dev->transfer(&reg, 1, buf, 2)) {
            return false;
        }

        const uint16_t raw = ((((uint16_t)buf[0]) << 8) | buf[1]) & 0x0FFF;
        const float angle_rad = raw * (2.0f * M_PI / 4096.0f);

        _state[idx].raw_angle = raw;
        _state[idx].angle_rad = angle_rad;
        _state[idx].last_update_us = AP_HAL::micros();
        _state[idx].is_healthy = true;

        return true;
    }

    void mark_unhealthy(uint8_t idx)
    {
        if (idx >= NUM_ENCODERS) {
            return;
        }
        _state[idx].is_healthy = false;
    }
};