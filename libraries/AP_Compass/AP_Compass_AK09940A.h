/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
   AK09940A Compass Driver 
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_AK09940A_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#define HAL_COMPASS_AK09940A_I2C_ADDR 0x0C

class AP_Compass_AK09940A : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    static constexpr const char *name = "AK09940A";

    void read() override;

private:
    AP_Compass_AK09940A(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                      bool force_external,
                      enum Rotation rotation);

    bool init();
    
    bool _reset();
    bool _setup_mode(uint8_t mode);
    bool _check_id();
    bool _set_sensordrive(uint8_t sensordrive);
    int32_t combine(uint8_t hl, uint8_t hm, uint8_t hh);

    void _update();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint8_t _compass_instance;
    bool _force_external;
    enum Rotation _rotation;
};

#endif  // AP_COMPASS_AK09940A_ENABLED
