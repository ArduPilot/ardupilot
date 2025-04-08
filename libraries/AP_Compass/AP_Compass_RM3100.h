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

#if AP_COMPASS_RM3100_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_RM3100_I2C_ADDR
# define HAL_COMPASS_RM3100_I2C_ADDR1 0x20
# define HAL_COMPASS_RM3100_I2C_ADDR2 0x21
# define HAL_COMPASS_RM3100_I2C_ADDR3 0x22
# define HAL_COMPASS_RM3100_I2C_ADDR4 0x23
#endif

class AP_Compass_RM3100 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    void read() override;

    static constexpr const char *name = "RM3100";

private:
    AP_Compass_RM3100(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    
    /**
     * Device periodic callback to read data from the sensor.
     */
    bool init();
    void timer();

    uint8_t compass_instance;
    bool force_external;
    enum Rotation rotation;
    float _scaler = 1.0;
};

#endif  // AP_COMPASS_RM3100_ENABLED
