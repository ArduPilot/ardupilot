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
 */
/**
 * SRF02 SPEC
 * https://www.dfrobot.com/wiki/index.php/SRF02_Ultrasonic_sensor_(SKU:SEN0005)
 */
#include "AP_RangeFinder_SRF02I2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/**
 * Constructor
 *
 */
AP_RangeFinder_SRF02I2C::AP_RangeFinder_SRF02I2C(RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
            : AP_RangeFinder_Backend(_state)
            , _dev(std::move(dev)){}

/**
 * Detect
 *
 * @return nullptr is Sensor not found or Initialization failure. <BR>
 * Other is Instance.
 */
AP_RangeFinder_Backend *AP_RangeFinder_SRF02I2C::detect(RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    AP_RangeFinder_SRF02I2C *sensor = new AP_RangeFinder_SRF02I2C(_state, std::move(dev));

    // Device initialization
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/**
 * update the state of the sensor
 *
 */
void AP_RangeFinder_SRF02I2C::update(void)
{
    if (_sem->take_nonblocking()) {
        if (_new_distance) {
            // Measurement valid
            state.distance_cm = _distance;
            _new_distance = false;  // Set invalid measurement
            update_status();
        } else {
            // Invalid measurement
            set_status(RangeFinder::RangeFinder_NoData);
        }
        _sem->give();
    }
}

/**
 * Initialization
 *
 * @retval true Successful completion
 * @retval false Abnormal termination
 */
bool AP_RangeFinder_SRF02I2C::_init()
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    _dev->set_retries(3);

    // Issuing measurement start command
    bool ret = _startMeasurement();

    _dev->get_semaphore()->give();

    if(!ret) {
        return false;
    }

    hal.scheduler->delay(SRF02_I2C_RANGING_TIME);

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // Whether the device is the SRF02
    uint8_t info[6];
    _dev->read_registers(SRF02_I2C_REG_R_SOFTWARE_REVISION, info, sizeof(info));
    if (SRF02_I2C_CHECK_CODE_DEFAULT != info[SRF02_I2C_REG_R_CHECK_CODE] && SRF02_I2C_CHECK_CODE != info[SRF02_I2C_REG_R_CHECK_CODE]) {
        _dev->get_semaphore()->give();
        return false;
    }
    // Get Autotune Minimum
    _autotuneMinimum = info[SRF02_I2C_REG_R_AUTOTUNE_MINIMUM_HIGH_BYTE] << 8 | info[SRF02_I2C_REG_R_AUTOTUNE_MINIMUM_LOW_BYTE];

    // Issuing measurement start command
    ret = _startMeasurement();

    _dev->get_semaphore()->give();

    if (ret) {
        // call timer() at 14Hz
        _dev->register_periodic_callback(70000,
                                         FUNCTOR_BIND_MEMBER(&AP_RangeFinder_SRF02I2C::_timer, void));
    }

    return ret;
}

/**
 * Instruct start of measurement
 *
 * @retval true Successful measurement start instruction
 * @retval false Measurement start instruction failure (with retry)
 */
bool AP_RangeFinder_SRF02I2C::_startMeasurement(void)
{
    // Set Real range mode and Set unit cm
    return _dev->write_register(SRF02_I2C_REG_W_COMMAND, SRF02_I2C_CMD_REAL_RANGE_MODE_RESULT_IN_CENTIMETERS);
}

/**
 * Get measured distance
 *
 * @param [out] outMeasuredDistance measured distance
 * @retval true Measurement normal
 * @retval false Measurement abnormality
 */
bool AP_RangeFinder_SRF02I2C::_getMeasuredDistance(uint16_t &outMeasuredDistance)
{
    uint8_t distance[2];
    bool ret = _dev->read_registers(SRF02_I2C_REG_R_RANGE_HIGH_BYTE, distance, sizeof(distance));
    if (ret) {
        outMeasuredDistance = distance[0] << 8 | distance[1];
        if (_autotuneMinimum > outMeasuredDistance) {
            ret = false;
        }
    }

    // Start measurement
    _startMeasurement();

    return ret;
}

/**
 * timer called at 14Hz
 *
 */
void AP_RangeFinder_SRF02I2C::_timer(void)
{
    uint16_t outMeasuredDistance;

    // Acquire measurement distance
    if (_getMeasuredDistance(outMeasuredDistance)) {
        // Acquire semaphore
        if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            // Set as static variable
            _distance = outMeasuredDistance;
            _new_distance = true;

            // Release of semaphore
            _sem->give();
        }
    }
}
