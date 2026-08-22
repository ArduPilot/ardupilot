/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Driver by Voltafield, July 2026
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_AF9838_ENABLED

#include <AP_Compass/AP_Compass_Backend.h>
#include <AP_Compass/AP_Compass.h>

#ifndef HAL_COMPASS_AF9838_I2C_ADDR
#define HAL_COMPASS_AF9838_I2C_ADDR 0x0C
#endif

class AP_Compass_AF9838 : public AP_Compass_Backend
{
public:
    AP_Compass_AF9838(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                      bool force_external,
                      enum Rotation rotation);

    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    void read() override;

private:
    bool init();
    void timer();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    bool _force_external;
    bool _single_pending = false;
    uint32_t _single_start_us = 0;

    enum Rotation _rotation;
    // Resolution: 0.1 uT/LSB, converted to milligauss for AP_Compass.
    static constexpr float AF9838_MILLIGAUSS_PER_LSB = 1.0f;

    static constexpr uint8_t REG_PCODE   = 0x00;
    static constexpr uint8_t REG_DATA    = 0x03;
    static constexpr uint8_t REG_STATUS  = 0x09;
    static constexpr uint8_t REG_STATE   = 0x0A;
    static constexpr uint8_t REG_SWR     = 0x11;

    // REG_STATE values
    static constexpr uint8_t STATE_STANDBY     = 0x00;
    static constexpr uint8_t STATE_SINGLE      = 0x01;
    static constexpr uint8_t STATE_SELF_TEST   = 0x09;
    static constexpr uint8_t SWR_TRIGGER       = 0x81;

    // STATUS bits
    static constexpr uint8_t STATUS_ACQ   = 1U << 0;
    static constexpr uint8_t STATUS_STERR = 1U << 1;
    static constexpr uint8_t STATUS_HOFL  = 1U << 2;

    // PCODE expected value
    static constexpr uint8_t PCODE_EXPECT = 0x70;
};
#endif  // AP_COMPASS_AF9838_ENABLED
