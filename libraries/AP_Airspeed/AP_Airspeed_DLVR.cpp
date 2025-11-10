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
/*
  driver for DLVR differential airspeed sensor
  https://www.allsensors.com/products/DLVR-L01D
 */

#include "AP_Airspeed_DLVR.h"

#if AP_AIRSPEED_DLVR_ENABLED

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#define DLVR_I2C_ADDR 0x28

#ifdef DLVR_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif


AP_Airspeed_DLVR::AP_Airspeed_DLVR(AP_Airspeed &_frontend, uint8_t _instance, const float _range_inH2O) :
    AP_Airspeed_Backend(_frontend, _instance),
    range_inH2O(_range_inH2O)
{}

/*
  probe for a sensor on a given i2c address
 */
AP_Airspeed_Backend *AP_Airspeed_DLVR::probe(AP_Airspeed &_frontend,
                                             uint8_t _instance,
                                             AP_HAL::I2CDevice *_dev,
                                             const float _range_inH2O)
{
    if (!_dev) {
        return nullptr;
    }
    AP_Airspeed_DLVR *sensor = NEW_NOTHROW AP_Airspeed_DLVR(_frontend, _instance, _range_inH2O);
    if (!sensor) {
        return nullptr;
    }
    sensor->dev = _dev;
    sensor->setup();
    return sensor;
}

// initialise the sensor
void AP_Airspeed_DLVR::setup()
{
    WITH_SEMAPHORE(dev->get_semaphore());
    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    dev->set_retries(2);
    dev->set_device_type(uint8_t(DevType::DLVR));
    set_bus_id(dev->get_bus_id());

    dev->register_periodic_callback(1000000UL/50U,
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_DLVR::timer, void));
}

// probe and initialise the sensor
bool AP_Airspeed_DLVR::init()
{
    dev = hal.i2c_mgr->get_device_ptr(get_bus(), DLVR_I2C_ADDR);
    if (!dev) {
        return false;
    }
    setup();
    return true;
}

#define STATUS_SHIFT 30
#define TEMPERATURE_SHIFT 5
#define TEMPERATURE_MASK ((1 << 11) - 1)
#define PRESSURE_SHIFT 16
#define PRESSURE_MASK ((1 << 14) - 1)

#define DLVR_OFFSET 8192.0f
#define DLVR_SCALE 16384.0f

// 50Hz timer
void AP_Airspeed_DLVR::timer()
{
    uint8_t raw_bytes[4];
    if (!dev->read((uint8_t *)&raw_bytes, sizeof(raw_bytes))) {
        return;
    }

    uint32_t data = (raw_bytes[0] << 24) |
                    (raw_bytes[1] << 16) |
                    (raw_bytes[2] << 8)  |
                    raw_bytes[3];

    if ((data >> STATUS_SHIFT)) {
        // anything other then 00 in the status bits is an error
        Debug("DLVR: Bad status read %u", (unsigned int)(data >> STATUS_SHIFT));
        return;
    }

    uint32_t pres_raw = (data >> PRESSURE_SHIFT)    & PRESSURE_MASK;
    uint32_t temp_raw = (data >> TEMPERATURE_SHIFT) & TEMPERATURE_MASK;

    float press_h2o = 1.25f * 2.0f * range_inH2O * ((pres_raw - DLVR_OFFSET) / DLVR_SCALE);

    if ((press_h2o > range_inH2O) || (press_h2o < -range_inH2O)) {
        Debug("DLVR: Out of range pressure %f", press_h2o);
        return;
    }

    float temp = temp_raw * (200.0f / 2047.0f) - 50.0f;

    WITH_SEMAPHORE(sem);

    const uint32_t now = AP_HAL::millis();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal" // suppress -Wfloat-equal as we are only worried about the case where we had never read
    if ((temperature != 0) && (fabsf(temperature - temp) > 10) && ((now - last_sample_time_ms) < 2000)) {
        Debug("DLVR: Temperature swing %f", temp);
        return;
    }
#pragma GCC diagnostic pop

    pressure_sum += INCH_OF_H2O_TO_PASCAL * press_h2o;
    temperature_sum += temp;
    press_count++;
    temp_count++;
    last_sample_time_ms = now;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_DLVR::get_differential_pressure(float &_pressure)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (press_count > 0) {
        pressure = pressure_sum / press_count;
        press_count = 0;
        pressure_sum = 0;
    }

    _pressure = pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_DLVR::get_temperature(float &_temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (temp_count > 0) {
        temperature = temperature_sum / temp_count;
        temp_count = 0;
        temperature_sum = 0;
    }

    _temperature = temperature;
    return true;
}

#endif
