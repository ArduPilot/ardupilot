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
#include "AP_Compass_BMM350.h"

#if AP_COMPASS_BMM350_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <utility>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#define BMM350_REG_CHIP_ID              0x00
#define BMM350_REG_PMU_CMD_AGGR_SET     0x04
#define BMM350_REG_PMU_CMD_AXIS_EN      0x05
#define BMM350_REG_PMU_CMD              0x06
#define BMM350_REG_PMU_CMD_STATUS_0     0x07
#define BMM350_REG_INT_CTRL             0x2E
#define BMM350_REG_MAG_X_XLSB           0x31
#define BMM350_REG_OTP_CMD              0x50
#define BMM350_REG_OTP_DATA_MSB         0x52
#define BMM350_REG_OTP_DATA_LSB         0x53
#define BMM350_REG_OTP_STATUS           0x55
#define BMM350_REG_CMD                  0x7E

// OTP(one-time programmable memory)
#define BMM350_OTP_CMD_DIR_READ         (0x01<<5U)
#define BMM350_OTP_CMD_PWR_OFF_OTP      (0x04<<5U)

#define BMM350_OTP_STATUS_ERROR_MASK    0xE0
#define BMM350_OTP_STATUS_CMD_DONE      0x01

#define BMM350_TEMP_OFF_SENS            0x0D
#define BMM350_MAG_OFFSET_X             0x0E
#define BMM350_MAG_OFFSET_Y             0x0F
#define BMM350_MAG_OFFSET_Z             0x10
#define BMM350_MAG_SENS_X               0x10
#define BMM350_MAG_SENS_Y               0x11
#define BMM350_MAG_SENS_Z               0x11
#define BMM350_MAG_TCO_X                0x12
#define BMM350_MAG_TCO_Y                0x13
#define BMM350_MAG_TCO_Z                0x14
#define BMM350_MAG_TCS_X                0x12
#define BMM350_MAG_TCS_Y                0x13
#define BMM350_MAG_TCS_Z                0x14
#define BMM350_MAG_DUT_T_0              0x18
#define BMM350_CROSS_X_Y                0x15
#define BMM350_CROSS_Y_X                0x15
#define BMM350_CROSS_Z_X                0x16
#define BMM350_CROSS_Z_Y                0x16
#define BMM350_SENS_CORR_Y              0.01f
#define BMM350_TCS_CORR_Z               0.0001f

#define BMM350_CMD_SOFTRESET            0xB6
#define BMM350_INT_MODE_PULSED          (0<<0U)
#define BMM350_INT_POL_ACTIVE_HIGH      (1<<1U)
#define BMM350_INT_OD_PUSHPULL          (1<<2U)
#define BMM350_INT_OUTPUT_DISABLE       (0<<3U)
#define BMM350_INT_DRDY_EN              (1<<7U)

// Averaging performance
#define BMM350_AVERAGING_4              (0x02 << 4U)
#define BMM350_AVERAGING_8              (0x03 << 4U)

// Output data rate
#define BMM350_ODR_100HZ                0x04
#define BMM350_ODR_50HZ                 0x05

// Power modes
#define BMM350_PMU_CMD_SUSPEND_MODE     0x00
#define BMM350_PMU_CMD_NORMAL_MODE      0x01
#define BMM350_PMU_CMD_UPD_OAE          0x02
#define BMM350_PMU_CMD_FGR              0x05
#define BMM350_PMU_CMD_BR               0x07

// OTP data length
#define BMM350_OTP_DATA_LENGTH          32U

// Chip ID of BMM350
#define BMM350_CHIP_ID                  0x33

#define BMM350_XY_SENSITIVE             14.55f
#define BMM350_Z_SENSITIVE              9.0f
#define BMM350_TEMP_SENSITIVE           0.00204f
#define BMM350_XY_INA_GAIN              19.46f
#define BMM350_Z_INA_GAIN               31.0f
#define BMM350_ADC_GAIN                 (1.0f / 1.5f)
#define BMM350_LUT_GAIN                 0.714607238769531f
#define BMM350_POWER                    ((float)(1000000.0 / 1048576.0))

#define BMM350_XY_SCALE                 (BMM350_POWER / (BMM350_XY_SENSITIVE * BMM350_XY_INA_GAIN * BMM350_ADC_GAIN * BMM350_LUT_GAIN))
#define BMM350_Z_SCALE                  (BMM350_POWER / (BMM350_Z_SENSITIVE * BMM350_Z_INA_GAIN * BMM350_ADC_GAIN * BMM350_LUT_GAIN))
#define BMM350_TEMP_SCALE               (1.0f / (BMM350_TEMP_SENSITIVE * BMM350_ADC_GAIN * BMM350_LUT_GAIN * 1048576))

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_BMM350::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_BMM350 *sensor = NEW_NOTHROW AP_Compass_BMM350(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_BMM350::AP_Compass_BMM350(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                       bool force_external,
                                       enum Rotation rotation)
    : _dev(std::move(dev))
    , _force_external(force_external)
    , _rotation(rotation)
{
}

/**
 * @brief Read out OTP(one-time programmable memory) data of sensor which is the compensation coefficients
 * @see https://github.com/boschsensortec/BMM350-SensorAPI
 */
bool AP_Compass_BMM350::read_otp_data()
{
    uint16_t otp_data[BMM350_OTP_DATA_LENGTH];

    for (uint8_t index = 0; index < BMM350_OTP_DATA_LENGTH; index++) {
        // Set OTP address
        if (!_dev->write_register(BMM350_REG_OTP_CMD, (BMM350_OTP_CMD_DIR_READ | index))) {
            return false;
        }

        // Wait for OTP status be ready
        int8_t tries = 3;    // Try polling 3 times
        while (tries--)
        {
            uint8_t status;
            hal.scheduler->delay_microseconds(300);
            // Read OTP status
            if (!read_bytes(BMM350_REG_OTP_STATUS, &status, 1) || ((status & BMM350_OTP_STATUS_ERROR_MASK) != 0)) {
                return false;
            }
            if (status & BMM350_OTP_STATUS_CMD_DONE) {
                break;
            }
        }

        if (tries == -1) {
            return false;
        }

        // Read OTP data
        be16_t reg_data;
        if (!read_bytes(BMM350_REG_OTP_DATA_MSB, (uint8_t *)&reg_data, 2)) {
            return false;
        }

        otp_data[index] = be16toh(reg_data);
    }

    // Update magnetometer offset and sensitivity data.
    // 12-bit unsigned integer to be left-aligned in a 16-bit integer
    _mag_comp.offset_coef.x = float(int16_t((otp_data[BMM350_MAG_OFFSET_X] & 0x0FFF) << 4) >> 4);
    _mag_comp.offset_coef.y = float(int16_t((((otp_data[BMM350_MAG_OFFSET_X] & 0xF000) >> 4) +
                                              (otp_data[BMM350_MAG_OFFSET_Y] & 0x00FF)) << 4) >> 4);
    _mag_comp.offset_coef.z = float(int16_t(((otp_data[BMM350_MAG_OFFSET_Y] & 0x0F00) +
                                             (otp_data[BMM350_MAG_OFFSET_Z] & 0x00FF)) << 4) >> 4);
    _mag_comp.offset_coef.temp = float(int8_t(otp_data[BMM350_TEMP_OFF_SENS])) * (1.0f / 5.0f);

    _mag_comp.sensit_coef.x = float(int8_t((otp_data[BMM350_MAG_SENS_X] & 0xFF00) >> 8)) * (1.0f / 256.0f);
    _mag_comp.sensit_coef.y = float(int8_t(otp_data[BMM350_MAG_SENS_Y])) * (1.0f / 256.0f) + BMM350_SENS_CORR_Y;
    _mag_comp.sensit_coef.z = float(int8_t((otp_data[BMM350_MAG_SENS_Z] & 0xFF00) >> 8)) * (1.0f / 256.0f);
    _mag_comp.sensit_coef.temp = float(int8_t((otp_data[BMM350_TEMP_OFF_SENS] & 0xFF00) >> 8)) * (1.0f / 512.0f);

    _mag_comp.tco.x = float(int8_t(otp_data[BMM350_MAG_TCO_X])) * (1.0f / 32.0f);
    _mag_comp.tco.y = float(int8_t(otp_data[BMM350_MAG_TCO_Y])) * (1.0f / 32.0f);
    _mag_comp.tco.z = float(int8_t(otp_data[BMM350_MAG_TCO_Z])) * (1.0f / 32.0f);

    _mag_comp.tcs.x = float(int8_t((otp_data[BMM350_MAG_TCS_X] & 0xFF00) >> 8)) * (1.0f / 16384.0f);
    _mag_comp.tcs.y = float(int8_t((otp_data[BMM350_MAG_TCS_Y] & 0xFF00) >> 8)) * (1.0f / 16384.0f);
    _mag_comp.tcs.z = float(int8_t((otp_data[BMM350_MAG_TCS_Z] & 0xFF00) >> 8)) * (1.0f / 16384.0f) - BMM350_TCS_CORR_Z;

    _mag_comp.t0_reading = float(int16_t(otp_data[BMM350_MAG_DUT_T_0])) * (1.0f / 512.0f) + 23.0f;

    _mag_comp.cross_axis.cross_x_y = float(int8_t(otp_data[BMM350_CROSS_X_Y])) * (1.0f / 800.0f);
    _mag_comp.cross_axis.cross_y_x = float(int8_t((otp_data[BMM350_CROSS_Y_X] & 0xFF00) >> 8)) * (1.0f / 800.0f);
    _mag_comp.cross_axis.cross_z_x = float(int8_t(otp_data[BMM350_CROSS_Z_X])) * (1.0f / 800.0f);
    _mag_comp.cross_axis.cross_z_y = float(int8_t((otp_data[BMM350_CROSS_Z_Y] & 0xFF00) >> 8)) * (1.0f / 800.0f);

    return true;
}

/**
 * @brief Wait PMU_CMD register operation completed. Need to specify which command just sent
 */
bool AP_Compass_BMM350::wait_pmu_cmd_ready(const uint8_t cmd, const uint32_t timeout)
{
    const uint32_t start_tick = AP_HAL::millis();

    do {
        hal.scheduler->delay(1);
        uint8_t status;
        if (!read_bytes(BMM350_REG_PMU_CMD_STATUS_0, &status, 1)) {
            return false;
        }
        if (((status & 0x01) == 0x00) && (((status & 0xE0) >> 5) == cmd)) {
            return true;
        }
    } while ((AP_HAL::millis() - start_tick) < timeout);

    return false;
}

/**
 * @brief Reset bit of magnetic register and wait for change to normal mode
 */
bool AP_Compass_BMM350::mag_reset_and_wait()
{
    uint8_t data;

    // Get PMU command status 0 data
    if (!read_bytes(BMM350_REG_PMU_CMD_STATUS_0, &data, 1)) {
        return false;
    }

    // Check whether the power mode is Normal
    if (data & 0x08) {
        // Set PMU command to suspend mode
        if (!_dev->write_register(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_SUSPEND_MODE)) {
            return false;
        }
        wait_pmu_cmd_ready(BMM350_PMU_CMD_SUSPEND_MODE, 6);
    }

    // Set BR(bit reset) to PMU_CMD register
    // In offical example, it wait for 14ms. But it may not be enough, so we wait an extra 5ms
    if (!_dev->write_register(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_BR) ||
        !wait_pmu_cmd_ready(BMM350_PMU_CMD_BR, 19)) {
        return false;
    }

    // Set FGR(flux-guide reset) to PMU_CMD register
    // 18ms got from offical example, we wait an extra 5ms
    if (!_dev->write_register(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_FGR) ||
        !wait_pmu_cmd_ready(BMM350_PMU_CMD_FGR, 23)) {
        return false;
    }

    // Switch to normal mode
    if (!set_power_mode(POWER_MODE_NORMAL)) {
        return false;
    }

    return true;
}

/**
 * @brief Switch sensor power mode
 */
bool AP_Compass_BMM350::set_power_mode(const enum power_mode mode)
{
    uint8_t pmu_cmd;

    // Get PMU register data as current mode
    if (!read_bytes(BMM350_REG_PMU_CMD, &pmu_cmd, 1)) {
        return false;
    }

    if (pmu_cmd == BMM350_PMU_CMD_NORMAL_MODE || pmu_cmd == BMM350_PMU_CMD_UPD_OAE) {
        // Set PMU command to suspend mode
        if (!_dev->write_register(BMM350_REG_PMU_CMD, POWER_MODE_SUSPEND)) {
            return false;
        }
        // Wait for sensor switch to suspend mode
        // Wait for maximum 6ms, it got from the official example, not explained in datasheet
        wait_pmu_cmd_ready(POWER_MODE_SUSPEND, 6);
    }

    // Set PMU command to target mode
    if (!_dev->write_register(BMM350_REG_PMU_CMD, mode)) {
        return false;
    }

    // Wait for mode change. When switch from suspend mode to normal mode, we wait for at most 38ms.
    // It got from official example too
    wait_pmu_cmd_ready(mode, 38);

    return true;
}

/**
 * @brief Read bytes from sensor
 */
bool AP_Compass_BMM350::read_bytes(const uint8_t reg, uint8_t *out, const uint16_t read_len)
{
    uint8_t data[read_len + 2];

    if (!_dev->read_registers(reg, data, read_len + 2)) {
        return false;
    }

    memcpy(out, &data[2], read_len);

    return true;
}

bool AP_Compass_BMM350::init()
{
    _dev->get_semaphore()->take_blocking();

    // 10 retries for init
    _dev->set_retries(10);

    // Use checked registers to cope with bus errors
    _dev->setup_checked_registers(4);

    int8_t boot_retries = 5;
    while (boot_retries--) {
        // Soft reset
        if (!_dev->write_register(BMM350_REG_CMD, BMM350_CMD_SOFTRESET)) {
            continue;
        }
        hal.scheduler->delay(24);   // Wait 24ms for soft reset complete, it got from offical example

        // Read and verify chip ID
        uint8_t chip_id;
        if (!read_bytes(BMM350_REG_CHIP_ID, &chip_id, 1)) {
            continue;
        }
        if (chip_id == BMM350_CHIP_ID) {
            break;
        }
    }
    
    if (boot_retries == -1) {
        goto err;
    }

    // Read out OTP data
    if (!read_otp_data()) {
        goto err;
    }

    // Power off OTP
    if (!_dev->write_register(BMM350_REG_OTP_CMD, BMM350_OTP_CMD_PWR_OFF_OTP)) {
        goto err;
    }

    // Magnetic reset
    if (!mag_reset_and_wait()) {
        goto err;
    }

    // Configure interrupt settings and enable DRDY
    // Set INT mode as PULSED | active_high polarity | PUSH_PULL | unmap | DRDY interrupt
    if (!_dev->write_register(BMM350_REG_INT_CTRL, (BMM350_INT_MODE_PULSED |
                                                    BMM350_INT_POL_ACTIVE_HIGH |
                                                    BMM350_INT_OD_PUSHPULL |
                                                    BMM350_INT_OUTPUT_DISABLE |
                                                    BMM350_INT_DRDY_EN))) {
        goto err;
    }

    // Setup ODR and performance. 100Hz ODR and 4 average for lownoise
    if (!_dev->write_register(BMM350_REG_PMU_CMD_AGGR_SET, (BMM350_AVERAGING_4 | BMM350_ODR_100HZ))) {
        goto err;
    }

    // Update ODR and avg parameter
    if (!_dev->write_register(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_UPD_OAE)) {
        goto err;
    }
    // Wait at most 20ms for update ODR and avg paramter
    wait_pmu_cmd_ready(BMM350_PMU_CMD_UPD_OAE, 20);

    // Enable measurement of 3 axis
    if (!_dev->write_register(BMM350_REG_PMU_CMD_AXIS_EN, 0x07)) {
        goto err;
    }

    // Switch power mode to normal mode
    if (!set_power_mode(POWER_MODE_NORMAL)) {
        goto err;
    }

    // Lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    /* Register the compass instance in the frontend */
    _dev->set_device_type(DEVTYPE_BMM350);
    if (!register_compass(_dev->get_bus_id(), _compass_instance)) {
        return false;
    }
    set_dev_id(_compass_instance, _dev->get_bus_id());

    // printf("BMM350: Found at address 0x%x as compass %u\n", _dev->get_bus_address(), _compass_instance);

    set_rotation(_compass_instance, _rotation);

    if (_force_external) {
        set_external(_compass_instance, true);
    }
    
    // Call timer() at 100Hz
    _dev->register_periodic_callback(1000000U/100U, FUNCTOR_BIND_MEMBER(&AP_Compass_BMM350::timer, void));

    return true;

err:
    _dev->get_semaphore()->give();
    return false;
}

void AP_Compass_BMM350::timer()
{
    struct PACKED {
        uint8_t magx[3];
        uint8_t magy[3];
        uint8_t magz[3];
        uint8_t temp[3];
    } data;

    // Read out measurement data
    if (!read_bytes(BMM350_REG_MAG_X_XLSB, (uint8_t *)&data, sizeof(data))) {
        return;
    }

    // Converts 24-bit raw data to signed integer value
    const int32_t magx_raw = int32_t(((uint32_t)data.magx[0] << 8) + ((uint32_t)data.magx[1] << 16) + ((uint32_t)data.magx[2] << 24)) >> 8;
    const int32_t magy_raw = int32_t(((uint32_t)data.magy[0] << 8) + ((uint32_t)data.magy[1] << 16) + ((uint32_t)data.magy[2] << 24)) >> 8;
    const int32_t magz_raw = int32_t(((uint32_t)data.magz[0] << 8) + ((uint32_t)data.magz[1] << 16) + ((uint32_t)data.magz[2] << 24)) >> 8;
    const int32_t temp_raw = int32_t(((uint32_t)data.temp[0] << 8) + ((uint32_t)data.temp[1] << 16) + ((uint32_t)data.temp[2] << 24)) >> 8;
    
    // Convert mag lsb to uT and temp lsb to degC
    float magx = (float)magx_raw * BMM350_XY_SCALE;
    float magy = (float)magy_raw * BMM350_XY_SCALE;
    float magz = (float)magz_raw * BMM350_Z_SCALE;
    float temp = (float)temp_raw * BMM350_TEMP_SCALE;

    if (temp > 0.0f) {
        temp -= 25.49f;
    } else if (temp < 0.0f) {
        temp += 25.49f;
    }

    // Apply compensation
    temp = ((1 + _mag_comp.sensit_coef.temp) * temp) + _mag_comp.offset_coef.temp;
    // Compensate raw magnetic data
    magx = ((1 + _mag_comp.sensit_coef.x) * magx) + _mag_comp.offset_coef.x + (_mag_comp.tco.x * (temp - _mag_comp.t0_reading));
    magx /= 1 + _mag_comp.tcs.x * (temp - _mag_comp.t0_reading);

    magy = ((1 + _mag_comp.sensit_coef.y) * magy) + _mag_comp.offset_coef.y + (_mag_comp.tco.y * (temp - _mag_comp.t0_reading));
    magy /= 1 + _mag_comp.tcs.y * (temp - _mag_comp.t0_reading);

    magz = ((1 + _mag_comp.sensit_coef.z) * magz) + _mag_comp.offset_coef.z + (_mag_comp.tco.z * (temp - _mag_comp.t0_reading));
    magz /= 1 + _mag_comp.tcs.z * (temp - _mag_comp.t0_reading);

    const float cr_ax_comp_x = (magx - _mag_comp.cross_axis.cross_x_y * magy) / (1 - _mag_comp.cross_axis.cross_y_x * _mag_comp.cross_axis.cross_x_y);
    const float cr_ax_comp_y = (magy - _mag_comp.cross_axis.cross_y_x * magx) / (1 - _mag_comp.cross_axis.cross_y_x * _mag_comp.cross_axis.cross_x_y);
    const float cr_ax_comp_z = (magz + (magx * (_mag_comp.cross_axis.cross_y_x * _mag_comp.cross_axis.cross_z_y - _mag_comp.cross_axis.cross_z_x) - 
                                        magy * (_mag_comp.cross_axis.cross_z_y - _mag_comp.cross_axis.cross_x_y * _mag_comp.cross_axis.cross_z_x)) /
                                       (1 - _mag_comp.cross_axis.cross_y_x * _mag_comp.cross_axis.cross_x_y));

    // Store in field vector and convert uT to milligauss
    Vector3f field { cr_ax_comp_x * 10.0f, cr_ax_comp_y * 10.0f, cr_ax_comp_z * 10.0f };
    accumulate_sample(field, _compass_instance);
}

void AP_Compass_BMM350::read()
{
    drain_accumulated_samples(_compass_instance);
}

#endif  // AP_COMPASS_BMM350_ENABLED
