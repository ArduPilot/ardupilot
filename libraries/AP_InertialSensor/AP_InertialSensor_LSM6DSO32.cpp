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
/*
 *  driver for ST LSM6DSO32 IMU
 *
 *  Identical to the LSM6DSO apart from the accelerometer full-scale
 *  ladder, which is shifted up one range: 00 = +/-4g, 01 = +/-32g,
 *  10 = +/-8g, 11 = +/-16g. Both parts report WHO_AM_I 0x6C and cannot be
 *  told apart in software, so the board's hwdef has to name the right one.
 *  Getting it wrong reads half or double scale rather than failing.
*/

#include "AP_InertialSensor_LSM6DSO32.h"
#include "AP_InertialSensor_ASM330_registers.h"

#include <utility>

// CTRL1_XL FS_XL = 11b, the LSM6DSO32 coding for +/-16g. Sensitivity is
// then 0.488 mg/LSB, matching the scale the base driver applies.
#define LSM6DSO32_CTRL1_XL_FS_XL_16G    (0x3 << 2)

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DSO32::probe(AP_InertialSensor &imu,
        AP_HAL::OwnPtr<AP_HAL::Device> device,
        enum Rotation rotation)
{
    if (!device) {
        return nullptr;
    }

    AP_InertialSensor_LSM6DSO32 *sensor =
        NEW_NOTHROW AP_InertialSensor_LSM6DSO32(imu, std::move(device), rotation);
    if (!sensor || !sensor->init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_InertialSensor_Backend::DevTypes AP_InertialSensor_LSM6DSO32::devtype() const
{
    return DEVTYPE_INS_LSM6DSO32;
}

const char *AP_InertialSensor_LSM6DSO32::sensor_name() const
{
    return "LSM6DSO32";
}

uint8_t AP_InertialSensor_LSM6DSO32::accel_fs_bits() const
{
    return LSM6DSO32_CTRL1_XL_FS_XL_16G;
}
