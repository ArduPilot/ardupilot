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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#ifndef BMI270_DEFAULT_ROTATION
#define BMI270_DEFAULT_ROTATION ROTATION_NONE
#endif

class AP_InertialSensor_BMI270 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation=BMI270_DEFAULT_ROTATION);

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation=BMI270_DEFAULT_ROTATION);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;

    bool update() override;

private:
    AP_InertialSensor_BMI270(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                             enum Rotation rotation);

    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);
    bool write_register(uint8_t reg, uint8_t v);

    /**
     * If the macro BMI270_DEBUG is defined, check if there are errors reported
     * on the device's error register and panic if so. The implementation is
     * empty if the BMI270_DEBUG isn't defined.
     */
    void check_err_reg();

    /**
     * Try to perform initialization of the BMI270 device.
     *
     * The device semaphore must be taken and released by the caller.
     *
     * @return true on success, false otherwise.
     */
    bool hardware_init();

    /**
     * Try to initialize this driver.
     *
     * Do sensor and other required initializations.
     *
     * @return true on success, false otherwise.
     */
    bool init();

    /**
     * Configure accelerometer sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    void configure_accel();

    /**
     * Configure gyroscope sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    void configure_gyro();

    /**
     * Reset FIFO.
     */
    void fifo_reset();

    /**
     * Configure FIFO.
     */
    void configure_fifo();

    /**
     * Read samples from fifo.
     */
    void read_fifo();
    void parse_accel_frame(const uint8_t* d);
    void parse_gyro_frame(const uint8_t* d);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_HAL::Device::PeriodicHandle periodic_handle;

    enum Rotation _rotation;

    uint8_t temperature_counter;

    static const uint8_t maximum_fifo_config_file[];
};
