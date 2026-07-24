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

#include "AP_InertialSensor_ASM330.h"

class AP_InertialSensor_LSM6DSO : public AP_InertialSensor_ASM330
{
public:
    virtual ~AP_InertialSensor_LSM6DSO();

    void start(void) override;
    bool get_output_banner(char* banner, uint8_t banner_len) override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);
protected:
    AP_InertialSensor_LSM6DSO(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);

    uint8_t expected_whoami() const override;
    DevTypes devtype() const override;
    const char *sensor_name() const override;
    uint8_t temperature_decimation() const override;

    // CTRL1_XL FS_XL field selecting +/-16g. The LSM6DSO32 shifts this
    // ladder by one range, so it overrides.
    virtual uint8_t accel_fs_bits() const;

private:
    void fifo_init() override;
    void gyro_init() override;
    void accel_init() override;
    void poll_data() override;

    // pick the ODR from the fast sampling parameters. Only valid once the
    // accel instance number is known, so this runs from start().
    void set_backend_rate();

    bool read_fifo_block(uint16_t n_words);

    AP_HAL::Device::PeriodicHandle periodic_handle;

    // index into the ODR ladder, see odr_table in the implementation
    uint8_t odr_index;
    uint16_t backend_rate_hz;
    uint32_t backend_period_us;
    bool fast_sampling;

    // this part's ODR deviation from nominal, from INTERNAL_FREQ_FINE
    int8_t freq_fine;

    uint8_t *fifo_buffer;
};
