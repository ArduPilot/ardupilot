/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       APM_Baro.cpp - barometer driver
 *
 */

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Baro.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Baro::var_info[] PROGMEM = {
    // NOTE: Index numbers 0 and 1 were for the old integer
    // ground temperature and pressure

    // @Param: ABS_PRESS
    // @DisplayName: Absolute Pressure
    // @Description: calibrated ground pressure in Pascals
    // @Units: pascals
    // @Increment: 1
    AP_GROUPINFO("ABS_PRESS", 2, AP_Baro, sensors[0].ground_pressure, 0),

    // @Param: TEMP
    // @DisplayName: ground temperature
    // @Description: calibrated ground temperature in degrees Celsius
    // @Units: degrees celsius
    // @Increment: 1
    AP_GROUPINFO("TEMP", 3, AP_Baro, sensors[0].ground_temperature, 0),

    // @Param: ALT_OFFSET
    // @DisplayName: altitude offset
    // @Description: altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.
    // @Units: meters
    // @Range: -128 127
    // @Increment: 1
    AP_GROUPINFO("ALT_OFFSET", 4, AP_Baro, _alt_offset, 0),

    AP_GROUPEND
};

/*
  AP_Baro constructor
 */
AP_Baro::AP_Baro() :
        _num_drivers(0),
        _num_sensors(0),
        _primary(0),
        _last_altitude_EAS2TAS(0.0f),
        _EAS2TAS(0.0f),
        _external_temperature(0.0f),
        _last_external_temperature_ms(0)
{
    memset(sensors, 0, sizeof(sensors));

    AP_Param::setup_object_defaults(this, var_info);
}

// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void AP_Baro::calibrate()
{
    // reset the altitude offset when we calibrate. The altitude
    // offset is supposed to be for within a flight
    _alt_offset.set_and_save(0);

    // start by assuming all sensors are calibrated (for healthy() test)
    for (uint8_t i=0; i<_num_sensors; i++) {
        sensors[i].calibrated = true;
        sensors[i].alt_ok = true;
    }

    // let the barometer settle for a full second after startup
    // the MS5611 reads quite a long way off for the first second,
    // leading to about 1m of error if we don't wait
    for (uint8_t i = 0; i < 10; i++) {
        uint32_t tstart = hal.scheduler->millis();
        do {
            update();
            if (hal.scheduler->millis() - tstart > 500) {
                hal.scheduler->panic(PSTR("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [2]\r\n"));
            }
        } while (!healthy());
        hal.scheduler->delay(100);
    }

    // now average over 5 values for the ground pressure and
    // temperature settings
    float sum_pressure[BARO_MAX_INSTANCES] = {0};
    float sum_temperature[BARO_MAX_INSTANCES] = {0};
    uint8_t count[BARO_MAX_INSTANCES] = {0};
    const uint8_t num_samples = 5;

    for (uint8_t c = 0; c < num_samples; c++) {
        uint32_t tstart = hal.scheduler->millis();
        do {
            update();
            if (hal.scheduler->millis() - tstart > 500) {
                hal.scheduler->panic(PSTR("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [3]\r\n"));
            }
        } while (!healthy());
        for (uint8_t i=0; i<_num_sensors; i++) {
            if (healthy(i)) {
                sum_pressure[i] += sensors[i].pressure;
                sum_temperature[i] += sensors[i].temperature;
                count[i] += 1;
            }
        }
        hal.scheduler->delay(100);
    }
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (count[i] == 0) {
            sensors[i].calibrated = false;
        } else {
            sensors[i].ground_pressure.set_and_save(sum_pressure[i] / count[i]);
            sensors[i].ground_temperature.set_and_save(sum_temperature[i] / count[i]);
        }
    }

    // panic if all sensors are not calibrated
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (sensors[i].calibrated) {
            return;
        }
    }
    hal.scheduler->panic(PSTR("AP_Baro: all sensors uncalibrated"));
}

/*
   update the barometer calibration
   this updates the baro ground calibration to the current values. It
   can be used before arming to keep the baro well calibrated
*/
void AP_Baro::update_calibration()
{
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (healthy(i)) {
            sensors[i].ground_pressure.set(get_pressure(i));
        }
        float last_temperature = sensors[i].ground_temperature;
        sensors[i].ground_temperature.set(get_calibration_temperature(i));
        if (fabsf(last_temperature - sensors[i].ground_temperature) > 3) {
            // reset _EAS2TAS to force it to recalculate. This happens
            // when a digital airspeed sensor comes online
            _EAS2TAS = 0;
        }
    }
}

// return altitude difference in meters between current pressure and a
// given base_pressure in Pascal
float AP_Baro::get_altitude_difference(float base_pressure, float pressure) const
{
    float ret;
#if HAL_CPU_CLASS <= HAL_CPU_CLASS_16
    // on slower CPUs use a less exact, but faster, calculation
    float scaling = base_pressure / pressure;
    float temp    = get_calibration_temperature() + 273.15f;
    ret = logf(scaling) * temp * 29.271267f;
#else
    // on faster CPUs use a more exact calculation
    float scaling = pressure / base_pressure;
    float temp    = get_calibration_temperature() + 273.15f;

    // This is an exact calculation that is within +-2.5m of the standard atmosphere tables
    // in the troposphere (up to 11,000 m amsl).
	ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));
#endif
    return ret;
}


// return current scale factor that converts from equivalent to true airspeed
// valid for altitudes up to 10km AMSL
// assumes standard atmosphere lapse rate
float AP_Baro::get_EAS2TAS(void)
{
    float altitude = get_altitude();
    if ((fabsf(altitude - _last_altitude_EAS2TAS) < 100.0f) && (_EAS2TAS != 0.0f)) {
        // not enough change to require re-calculating
        return _EAS2TAS;
    }

    float tempK = get_calibration_temperature() + 273.15f - 0.0065f * altitude;
    _EAS2TAS = safe_sqrt(1.225f / ((float)get_pressure() / (287.26f * tempK)));
    _last_altitude_EAS2TAS = altitude;
    return _EAS2TAS;
}

// return current climb_rate estimeate relative to time that calibrate()
// was called. Returns climb rate in meters/s, positive means up
// note that this relies on read() being called regularly to get new data
float AP_Baro::get_climb_rate(void)
{
    // we use a 7 point derivative filter on the climb rate. This seems
    // to produce somewhat reasonable results on real hardware
    return _climb_rate_filter.slope() * 1.0e3f;
}


/*
  set external temperature to be used for calibration (degrees C)
 */
void AP_Baro::set_external_temperature(float temperature)
{
    _external_temperature = temperature;
    _last_external_temperature_ms = hal.scheduler->millis();
}

/*
  get the temperature in degrees C to be used for calibration purposes
 */
float AP_Baro::get_calibration_temperature(uint8_t instance) const
{
    // if we have a recent external temperature then use it
    if (_last_external_temperature_ms != 0 && hal.scheduler->millis() - _last_external_temperature_ms < 10000) {
        return _external_temperature;
    }
    // if we don't have an external temperature then use the minimum
    // of the barometer temperature and 25 degrees C. The reason for
    // not just using the baro temperature is it tends to read high,
    // often 30 degrees above the actual temperature. That means the
    // EAS2TAS tends to be off by quite a large margin
    float ret = get_temperature(instance);
    if (ret > 25) {
        ret = 25;
    }
    return ret;    
}


/*
  initialise the barometer object, loading backend drivers
 */
void AP_Baro::init(void)
{
#if HAL_BARO_DEFAULT == HAL_BARO_PX4 || HAL_BARO_DEFAULT == HAL_BARO_VRBRAIN
    drivers[0] = new AP_Baro_PX4(*this);
    _num_drivers = 1;
#elif HAL_BARO_DEFAULT == HAL_BARO_HIL
    drivers[0] = new AP_Baro_HIL(*this);
    _num_drivers = 1;
#elif HAL_BARO_DEFAULT == HAL_BARO_BMP085
    {
        drivers[0] = new AP_Baro_BMP085(*this);
        _num_drivers = 1;
    }
#elif HAL_BARO_DEFAULT == HAL_BARO_MS5611
    {
        drivers[0] = new AP_Baro_MS5611(*this, new AP_SerialBus_I2C(MS5611_I2C_ADDR), false);
        _num_drivers = 1;
    }
#elif HAL_BARO_DEFAULT == HAL_BARO_MS5611_SPI
    {
        drivers[0] = new AP_Baro_MS5611(*this, 
                                        new AP_SerialBus_SPI(AP_HAL::SPIDevice_MS5611, 
                                                             AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH),
                                        true);
        _num_drivers = 1;
    }
#endif    
    if (_num_drivers == 0 || _num_sensors == 0 || drivers[0] == NULL) {
        hal.scheduler->panic(PSTR("Baro: unable to initialise driver"));
    }
}


/*
  call update on all drivers
 */
void AP_Baro::update(void)
{
    for (uint8_t i=0; i<_num_drivers; i++) {
        drivers[i]->update();
    }

    // consider a sensor as healthy if it has had an update in the
    // last 0.5 seconds
    uint32_t now = hal.scheduler->millis();
    for (uint8_t i=0; i<_num_sensors; i++) {
        sensors[i].healthy = (now - sensors[i].last_update_ms < 500) && sensors[i].pressure != 0;
    }

    for (uint8_t i=0; i<_num_sensors; i++) {
        if (sensors[i].healthy) {
            // update altitude calculation
            if (sensors[i].ground_pressure == 0) {
                sensors[i].ground_pressure = sensors[i].pressure;
            }
            sensors[i].altitude = get_altitude_difference(sensors[i].ground_pressure, sensors[i].pressure);
            // sanity check altitude
            sensors[i].alt_ok = !(isnan(sensors[i].altitude) || isinf(sensors[i].altitude));
        }
    }

    // choose primary sensor
    _primary = 0;
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (healthy(i)) {
            _primary = i;
            break;
        }
    }

    // ensure the climb rate filter is updated
    _climb_rate_filter.update(get_altitude(), get_last_update());
}

/*
  call accululate on all drivers
 */
void AP_Baro::accumulate(void)
{
    for (uint8_t i=0; i<_num_drivers; i++) {
        drivers[i]->accumulate();
    }
}


/* register a new sensor, claiming a sensor slot. If we are out of
   slots it will panic
*/
uint8_t AP_Baro::register_sensor(void)
{
    if (_num_sensors >= BARO_MAX_INSTANCES) {
        hal.scheduler->panic(PSTR("Too many barometers"));
    }
    return _num_sensors++;
}


/*
  check if all barometers are healthy
 */
bool AP_Baro::all_healthy(void) const
{
     for (uint8_t i=0; i<_num_sensors; i++) {
         if (!healthy(i)) {
             return false;
         }
     }
     return _num_sensors > 0;
}
