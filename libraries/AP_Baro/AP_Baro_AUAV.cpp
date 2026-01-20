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
#include "AP_Baro_AUAV.h"

#if AP_BARO_AUAV_ENABLED

extern const AP_HAL::HAL &hal;

AP_Baro_AUAV::AP_Baro_AUAV(AP_Baro &baro, AP_HAL::Device *_dev)
    : AP_Baro_Backend(baro)
    , dev(_dev)
{
}

AP_Baro_Backend *AP_Baro_AUAV::probe(AP_Baro &baro, AP_HAL::Device &_dev)
{
    AP_Baro_AUAV *sensor = NEW_NOTHROW AP_Baro_AUAV(baro, &_dev);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_AUAV::init()
{
    if (!dev) {
        return false;
    }

    {
        // Take semaphore for i2c functions
        WITH_SEMAPHORE(dev->get_semaphore());
        dev->set_retries(10);

        // Request a measurement
        if (!sensor.measure()) {
            return false;
        }
        hal.scheduler->delay(40);

        // Test read and discard result as the compensation coefficients are not configured
        float PComp, temperature;
        if (sensor.collect(PComp, temperature) != AUAV_Pressure_sensor::Status::Normal) {
            return false;
        }
    }

    // Register sensor and set dev-id
    instance = _frontend.register_sensor();
    dev->set_device_type(DEVTYPE_BARO_AUAV);
    set_bus_id(instance, dev->get_bus_id());

    dev->register_periodic_callback(40000,
                                     FUNCTOR_BIND_MEMBER(&AP_Baro_AUAV::timer, void));

    return true;
}

//  accumulate a new sensor reading
void AP_Baro_AUAV::timer(void)
{
    if (sensor.stage != AUAV_Pressure_sensor::CoefficientStage::Done) {
        sensor.read_coefficients();
        return;
    }

    if (measurement_requested) {
        // Read in result of last measurement
        float Pcomp, temp_C;
        switch (sensor.collect(Pcomp, temp_C)) {
        case AUAV_Pressure_sensor::Status::Normal: {
            // Convert to correct units
            const float pressure_mbar = 250 + (1.25 * ((Pcomp-1677722)/16777216.0) * 1000.0);
            {
                WITH_SEMAPHORE(_sem);
                pressure_sum += pressure_mbar * 100;
                temperature_sum += temp_C;
                count++;
            }
            break;
        }
        case AUAV_Pressure_sensor::Status::Busy:
            // Don't request a new measurement
            return;

        case AUAV_Pressure_sensor::Status::Fault:
            break;
        }
    }

    // Request a new measurement
    measurement_requested = sensor.measure();
}

// transfer data to the frontend
void AP_Baro_AUAV::update(void)
{
    if (count == 0) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    _copy_to_frontend(instance, pressure_sum/count, temperature_sum/count);

    pressure_sum = 0;
    temperature_sum = 0;
    count = 0;
}

#endif  // AP_BARO_AUAV_ENABLED
