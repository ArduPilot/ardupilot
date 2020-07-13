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

#include "AP_Baro_KellerLD.h"

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>

#define KELLER_DEBUG 0

#if KELLER_DEBUG
# define Debug(fmt, args ...)  do {printf(fmt "\n", ## args);} while(0)
#else
# define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL &hal;

// Measurement range registers
static const uint8_t CMD_PRANGE_MIN_MSB = 0x13;
static const uint8_t CMD_PRANGE_MIN_LSB = 0x14;
static const uint8_t CMD_PRANGE_MAX_MSB = 0x15;
static const uint8_t CMD_PRANGE_MAX_LSB = 0x16;

// write to this address to start pressure measurement
static const uint8_t CMD_REQUEST_MEASUREMENT = 0xAC;

AP_Baro_KellerLD::AP_Baro_KellerLD(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
{
}

// Look for the device on the bus and see if it responds appropriately
AP_Baro_Backend *AP_Baro_KellerLD::probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_KellerLD *sensor = new AP_Baro_KellerLD(baro, std::move(dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

// The hardware does not need to be reset/initialized
// We read out the measurement range to be used in raw value conversions
bool AP_Baro_KellerLD::_init()
{
    if (!_dev) {
        return false;
    }

    _dev->get_semaphore()->take_blocking();

    // high retries for init
    _dev->set_retries(10);
    
    bool cal_read_ok = true;
    
    uint8_t data[3];
    uint16_t ms_word, ls_word;

    // This device has some undocumented finicky quirks and requires delays when reading out the
    // measurement range, but for some reason this isn't an issue when requesting measurements.
    // This is why we need to split the transfers with delays like this.
    // (Using AP_HAL::I2CDevice::set_split_transfers will not work with these sensors)

    // Read out pressure measurement range
    cal_read_ok &= _dev->transfer(&CMD_PRANGE_MIN_MSB, 1, nullptr, 0);
    hal.scheduler->delay(1);
    cal_read_ok &= _dev->transfer(nullptr, 0, &data[0], 3);
    hal.scheduler->delay(1);

    ms_word = (data[1] << 8) | data[2];
    Debug("0x13: %d [%d, %d, %d]", ms_word, data[0], data[1], data[2]);

    cal_read_ok &= _dev->transfer(&CMD_PRANGE_MIN_LSB, 1, nullptr, 0);
    hal.scheduler->delay(1);
    cal_read_ok &= _dev->transfer(nullptr, 0, &data[0], 3);
    hal.scheduler->delay(1);

    ls_word = (data[1] << 8) | data[2];
    Debug("0x14: %d [%d, %d, %d]", ls_word, data[0], data[1], data[2]);

    uint32_t cal_data = (ms_word << 16) | ls_word;
    memcpy(&_p_min, &cal_data, sizeof(_p_min));
    Debug("data: %d, p_min: %.2f", cal_data, _p_min);

    cal_read_ok &= _dev->transfer(&CMD_PRANGE_MAX_MSB, 1, nullptr, 0);
    hal.scheduler->delay(1);
    cal_read_ok &= _dev->transfer(nullptr, 0, &data[0], 3);
    hal.scheduler->delay(1);

    ms_word = (data[1] << 8) | data[2];
    Debug("0x15: %d [%d, %d, %d]", ms_word, data[0], data[1], data[2]);

    cal_read_ok &= _dev->transfer(&CMD_PRANGE_MAX_LSB, 1, nullptr, 0);
    hal.scheduler->delay(1);
    cal_read_ok &= _dev->transfer(nullptr, 0, &data[0], 3);
    hal.scheduler->delay(1);

    ls_word = (data[1] << 8) | data[2];
    Debug("0x16: %d [%d, %d, %d]", ls_word, data[0], data[1], data[2]);

    cal_data = (ms_word << 16) | ls_word;
    memcpy(&_p_max, &cal_data, sizeof(_p_max));
    Debug("data: %d, p_max: %.2f", cal_data, _p_max);

    cal_read_ok &= !isnan(_p_min) && !isinf(_p_min) && !isnan(_p_max) && !isinf(_p_max);

    cal_read_ok &= _p_max > _p_min;

    if (!cal_read_ok) {
        printf("Cal read bad!\n");
        _dev->get_semaphore()->give();
        return false;
    }

    printf("Keller LD found on bus %u address 0x%02x\n", _dev->bus_num(), _dev->get_bus_address());

    // Send a command to take a measurement
    _dev->transfer(&CMD_REQUEST_MEASUREMENT, 1, nullptr, 0);

    memset(&_accum, 0, sizeof(_accum));

    _instance = _frontend.register_sensor();

    _frontend.set_type(_instance, AP_Baro::BARO_TYPE_WATER);

    // lower retries for run
    _dev->set_retries(3);
    
    _dev->get_semaphore()->give();

    // The sensor needs time to take a deep breath after reading out the calibration...
    hal.scheduler->delay(150);

    // Request 50Hz update
    // The sensor really struggles with any jitter in timing at 100Hz, and will sometimes start reading out all zeros
    _dev->register_periodic_callback(20 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_Baro_KellerLD::_timer, void));
    return true;
}

// Read out most recent measurement from sensor hw
bool AP_Baro_KellerLD::_read()
{
    uint8_t data[5];
    if (!_dev->transfer(0x0, 1, data, sizeof(data))) {
        Debug("Keller LD read failed!");
        return false;
    }

    //uint8_t status = data[0];
    uint16_t pressure_raw = (data[1] << 8) | data[2];
    uint16_t temperature_raw = (data[3] << 8) | data[4];

#if KELLER_DEBUG
    static uint8_t samples = 0;
    if (samples < 3) {
        samples++;
        Debug("data: [%d, %d, %d, %d, %d]", data[0], data[1], data[2], data[3], data[4]);
        Debug("pressure_raw: %d\ttemperature_raw: %d", pressure_raw, temperature_raw);
    }
#endif

    if (pressure_raw == 0 || temperature_raw == 0) {
        Debug("Keller: bad read");
        return false;
    }

    if (!pressure_ok(pressure_raw)) {
        return false;
    }
    
    WITH_SEMAPHORE(_sem);
    
    _update_and_wrap_accumulator(pressure_raw, temperature_raw, 128);
    
    return true;
}

// Periodic callback, regular update at 50Hz
// Read out most recent measurement, and request another
// Max conversion time according to datasheet is ~8ms, so
// max update rate is ~125Hz, yet we struggle to get consistent
// performance/data at 100Hz
void AP_Baro_KellerLD::_timer(void)
{
    _read();
    _dev->transfer(&CMD_REQUEST_MEASUREMENT, 1, nullptr, 0);
}

// Accumulate a reading, shrink if necessary to prevent overflow
void AP_Baro_KellerLD::_update_and_wrap_accumulator(uint16_t pressure, uint16_t temperature, uint8_t max_count)
{
    _accum.sum_pressure += pressure;
    _accum.sum_temperature += temperature;
    _accum.num_samples += 1;

    if (_accum.num_samples == max_count) {
        _accum.sum_pressure /= 2;
        _accum.sum_temperature /= 2;
        _accum.num_samples /= 2;
    }
}

// Take the average of accumulated values and push to frontend
void AP_Baro_KellerLD::update()
{
    float sum_pressure, sum_temperature;
    float num_samples;

    {
        WITH_SEMAPHORE(_sem);

        if (_accum.num_samples == 0) {
            return;
        }

        sum_pressure = _accum.sum_pressure;
        sum_temperature = _accum.sum_temperature;
        num_samples = _accum.num_samples;
        memset(&_accum, 0, sizeof(_accum));
    }

    uint16_t raw_pressure_avg = sum_pressure / num_samples;
    uint16_t raw_temperature_avg = sum_temperature / num_samples;

    // per datasheet
    float pressure = (raw_pressure_avg - 16384) * (_p_max - _p_min) / 32768 + _p_min;
    pressure *= 100000; // bar -> Pascal
    pressure += 101300; // MSL pressure offset
    float temperature = ((raw_temperature_avg >> 4) - 24) * 0.05f - 50;

    _copy_to_frontend(_instance, pressure, temperature);
}
