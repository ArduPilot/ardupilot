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
  driver for the TE Connectivity SM9541 series differential pressure sensor

  The part is factory calibrated and temperature compensated and has no
  configuration registers and no chip id: a plain I2C read returns the latest
  measurement packet. It is addressed at a fixed 0x28.
 */

#include "AP_Airspeed_SM9541.h"

#if AP_AIRSPEED_SM9541_ENABLED

#include <AP_Math/AP_Math.h>
#include <utility>

extern const AP_HAL::HAL &hal;

#define SM9541_I2C_ADDR 0x28

// output count endpoints, 10% and 90% of the 14-bit range, common to the family
#define SM9541_OUT_MIN 1638.0f
#define SM9541_OUT_MAX 14745.0f

#define CMH2O_TO_PASCAL 98.0665f

// status is the two MSBs of the first byte; anything but 00 is not usable data
#define SM9541_STATUS_NORMAL 0

AP_Airspeed_SM9541::AP_Airspeed_SM9541(AP_Airspeed &_frontend, uint8_t _instance, float _range_cmH2O) :
    AP_Airspeed_Backend(_frontend, _instance),
    range_cmH2O(_range_cmH2O)
{}

bool AP_Airspeed_SM9541::init()
{
    dev = hal.i2c_mgr->get_device_ptr(get_bus(), SM9541_I2C_ADDR);
    if (!dev) {
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    dev->set_retries(2);

    // the sensor measures every 2ms; a read that comes back with a good status
    // is the only confirmation available that something is actually there
    uint8_t raw_bytes[4];
    bool found = false;
    for (uint8_t i = 0; i < 5; i++) {
        if (dev->read(raw_bytes, sizeof(raw_bytes)) &&
            (raw_bytes[0] >> 6) == SM9541_STATUS_NORMAL) {
            found = true;
            break;
        }
        hal.scheduler->delay(5);
    }
    if (!found) {
        return false;
    }

    dev->set_device_type(uint8_t(DevType::SM9541));
    set_bus_id(dev->get_bus_id());

    dev->register_periodic_callback(1000000UL/50U,
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_SM9541::timer, void));
    return true;
}

// 50Hz timer
void AP_Airspeed_SM9541::timer()
{
    // Read_DF4: 2 status bits + 14 pressure bits, then 11 temperature bits in
    // the upper bits of the last two bytes
    uint8_t raw_bytes[4];
    if (!dev->read(raw_bytes, sizeof(raw_bytes))) {
        return;
    }

    if ((raw_bytes[0] >> 6) != SM9541_STATUS_NORMAL) {
        // stale data, command mode or a diagnostic fault
        return;
    }

    const uint16_t press_raw = (uint16_t(raw_bytes[0] & 0x3F) << 8) | raw_bytes[1];
    const uint16_t temp_raw = (uint16_t(raw_bytes[2]) << 3) | (raw_bytes[3] >> 5);

    // the part spans -range_cmH2O at OUT_MIN to +range_cmH2O at OUT_MAX. Only
    // the differential variants are symmetric like this; the compound gauge
    // ones would need an offset as well as a span.
    const float press_cmH2O = (float(press_raw) - SM9541_OUT_MIN) *
                              (2 * range_cmH2O) / (SM9541_OUT_MAX - SM9541_OUT_MIN) -
                              range_cmH2O;

    // the TE datasheet does not print the temperature transfer function. This
    // packet is the Honeywell TruStability digital format, whose datasheet
    // (50099148-A) gives temp[C] = counts * 200/2047 - 50.
    const float temp = float(temp_raw) * (200.0f / 2047.0f) - 50.0f;

    WITH_SEMAPHORE(sem);

    pressure_sum += press_cmH2O * CMH2O_TO_PASCAL;
    temperature_sum += temp;
    press_count++;
    temp_count++;
    last_sample_time_ms = AP_HAL::millis();
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_SM9541::get_differential_pressure(float &_pressure)
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
bool AP_Airspeed_SM9541::get_temperature(float &_temperature)
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

#endif  // AP_AIRSPEED_SM9541_ENABLED
