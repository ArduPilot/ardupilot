/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include "AP_InertialSensor_LSM6DSO.h"

class AP_InertialSensor_LSM6DSO32 : public AP_InertialSensor_LSM6DSO
{
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);
private:
    using AP_InertialSensor_LSM6DSO::AP_InertialSensor_LSM6DSO;

    DevTypes devtype() const override;
    const char *sensor_name() const override;
    uint8_t accel_fs_bits() const override;
};
