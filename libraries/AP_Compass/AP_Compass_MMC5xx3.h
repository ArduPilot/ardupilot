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
#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_MMC5XX3_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_MMC5xx3_I2C_ADDR
# define HAL_COMPASS_MMC5xx3_I2C_ADDR 0x30
#endif

class AP_Compass_MMC5XX3 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    void read() override;

    static constexpr const char *name = "MMC5xx3";

private:
    AP_Compass_MMC5XX3(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    // Which chip variant was detected at init()
    enum class ChipVariant : uint8_t {
        MMC5983,  // 16-bit output, 6 bytes, regs at 0x09/0x0A/0x0B/0x08
        MMC5603,  // 20-bit output, 9 bytes, regs at 0x1B/0x1C/0x1D/0x18
    } chip_variant;

    enum class MMCState {
        STATE_SET,
        STATE_SET_MEASURE,
        STATE_SET_WAIT,
        STATE_RESET_MEASURE,
        STATE_RESET_WAIT,
        STATE_MEASURE,
    } state;

    bool init();
    void timer();
    bool probe_mmc5983();
    bool probe_mmc5603();

    bool force_external;
    Vector3f offset;
    uint16_t measure_count;
    bool have_initial_offset;

    // 9 bytes to accommodate MMC5603 20-bit (9-byte) reads;
    // MMC5983 only uses the first 6 bytes.
    uint8_t data0[9];

    enum Rotation rotation;
};

#endif  // AP_COMPASS_MMC5XX3_ENABLED
