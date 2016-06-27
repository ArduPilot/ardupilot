/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
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

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#define BMI160_DEBUG 0

class AP_InertialSensor_BMI160 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            SamplingConfig sampling_config);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;

    bool update() override;

private:
    enum OSR {
        OSR4 = 0,
        OSR2,
        NORMAL,
    };

    enum ODR {
        ODR_25Hz=6,
        ODR_50Hz,
        ODR_100Hz,
        ODR_200Hz,
        ODR_400Hz,
        ODR_800Hz,
        ODR_1600Hz,
    };

    enum AccelRange {
        /* For convenience, use sequence starting at 0 instead of bits defined
         * in the datasheet. See #_configure_accel().*/
        RANGE_2G = 0,
        RANGE_4G,
        RANGE_8G,
        RANGE_16G,
    };

    struct AccelConfig {
        OSR osr;
        ODR odr;
        AccelRange range;
    };

    enum GyroRange {
        RANGE_2000DPS = 0,
        RANGE_1000DPS,
        RANGE_500DPS,
        RANGE_250DPS,
        RANGE_125DPS,
    };

    struct GyroConfig {
        OSR osr;
        ODR odr;
        GyroRange range;
    };

    struct PACKED SensorRawData {
        struct {
            uint16_t x;
            uint16_t y;
            uint16_t z;
        } gyro;
        struct {
            uint16_t x;
            uint16_t y;
            uint16_t z;
        } accel;
    };

    AP_InertialSensor_BMI160(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                             SamplingConfig sampling_config);

#if BMI160_DEBUG
    void _check_err_reg();
#endif

    /**
     * Try to perform initialization of the BMI160 device.
     *
     * The device semaphore must be taken and released by the caller.
     *
     * @return true on success, false otherwise.
     */
    bool _hardware_init();

    /**
     * Try to initialize this driver.
     *
     * Do sensor and other required initializations.
     *
     * @return true on success, false otherwise.
     */
    bool _init();

    /**
     * Configure accelerometer sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_accel(AccelConfig &cfg);

    /**
     * Configure gyroscope sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_gyro(GyroConfig &cfg);

    /**
     * Timer routine to read data from the sensors.
     */
    void _poll_data();

    /**
     * Read sample from data registers.
     */
    void _read_sample();

    /**
     * Read samples from fifo.
     */
    void _read_fifo();

    /**
     * Process raw samples and notify them.
     */
    void _accumulate(SensorRawData *data, uint8_t num_samples);

    SamplingConfig _sampling_config;
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint8_t _accel_instance;
    float _accel_scale;

    uint8_t _gyro_instance;
    float _gyro_scale;

    AP_HAL::DigitalSource *_drdy_pin;
};
