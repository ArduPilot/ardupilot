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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_MMC5xx3_I2C_ADDR
# define HAL_COMPASS_MMC5xx3_I2C_ADDR 0x30
#endif

class AP_Compass_MMC5XX3 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    void read() override;

    static constexpr const char *name = "MMC5883";

private:
    AP_Compass_MMC5XX3(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    enum class MMCState {
        STATE_SET,
        STATE_SET_MEASURE,
        STATE_SET_WAIT,
        STATE_RESET_MEASURE,
        STATE_RESET_WAIT,
        STATE_MEASURE,
    } state;
    
    /**
     * Device periodic callback to read data from the sensor.
     */
    bool init();
    void timer();
    void accumulate_field(Vector3f &field);

    uint8_t compass_instance;
    bool force_external;
    Vector3f offset;
    uint16_t measure_count;
    bool have_initial_offset;
    uint32_t refill_start_ms;
    uint32_t last_sample_ms;
    
    uint16_t data0[3];
    
    enum Rotation rotation;
};
