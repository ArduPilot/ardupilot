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

#if AP_COMPASS_BMM350_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#define BMM350_I2C_ADDR_MIN 0x14
#define BMM350_I2C_ADDR_MAX 0x17


class AP_Compass_BMM350 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    void read() override;

    static constexpr const char *name = "BMM350";

private:
    AP_Compass_BMM350(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /**
     * @brief BMM350 offset/sensitivity coefficient structure
     */
    struct vector4f
    {
        float x;    // x axis
        float y;    // y axis
        float z;    // z axis
        float temp; // Temperature
    };

    /**
     * @brief BMM350 magnetometer cross axis compensation structure
     */
    struct cross_axis
    {
        float cross_x_y;
        float cross_y_x;
        float cross_z_x;
        float cross_z_y;
    };

    /**
     * @brief BMM350 magnetometer compensate structure
     */
    struct mag_compensate
    {
        struct vector4f offset_coef;    // Offset coefficient
        struct vector4f sensit_coef;    // Sensitivity coefficient
        Vector3f tco;                   // Temperature coefficient of the offset
        Vector3f tcs;                   // Temperature coefficient of the sensitivity
        float t0_reading;               // Initialize T0_reading parameter
        struct cross_axis cross_axis;   // Cross axis compensation
    };

    enum power_mode
    {
        POWER_MODE_SUSPEND     = 0,
        POWER_MODE_NORMAL      = 1,
        POWER_MODE_FORCED      = 3,
        POWER_MODE_FORCED_FAST = 4
    };

    /**
     * Device periodic callback to read data from the sensor.
     */
    bool init();
    void timer();
    bool read_otp_data();
    bool wait_pmu_cmd_ready(const uint8_t cmd, const uint32_t timeout);
    bool mag_reset_and_wait();
    bool set_power_mode(const enum power_mode mode);
    bool read_bytes(const uint8_t reg, uint8_t *out, const uint16_t read_len);

    uint8_t _compass_instance;
    bool _force_external;
    enum Rotation _rotation;
    struct mag_compensate _mag_comp;  // Structure for mag compensate
};

#endif  // AP_COMPASS_BMM350_ENABLED
