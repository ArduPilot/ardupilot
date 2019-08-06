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
#include "AP_Baro.h"

#include <utility>
#include <stdio.h>

#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "AP_Baro_SITL.h"
#include "AP_Baro_BMP085.h"
#include "AP_Baro_BMP280.h"
#include "AP_Baro_HIL.h"
#include "AP_Baro_KellerLD.h"
#include "AP_Baro_MS5611.h"
#include "AP_Baro_ICM20789.h"
#include "AP_Baro_LPS2XH.h"
#include "AP_Baro_FBM320.h"
#include "AP_Baro_DPS280.h"
#if HAL_WITH_UAVCAN
#include "AP_Baro_UAVCAN.h"
#endif

#include <AP_Logger/AP_Logger.h>

#define INTERNAL_TEMPERATURE_CLAMP 35.0f

#ifndef HAL_BARO_FILTER_DEFAULT
 #define HAL_BARO_FILTER_DEFAULT 0 // turned off by default
#endif

#if !defined(HAL_PROBE_EXTERNAL_I2C_BAROS) && !HAL_MINIMIZE_FEATURES
#define HAL_PROBE_EXTERNAL_I2C_BAROS
#endif

#ifndef HAL_BARO_PROBE_EXT_DEFAULT
 #define HAL_BARO_PROBE_EXT_DEFAULT 0
#endif

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Baro::var_info[] = {
    // NOTE: Index numbers 0 and 1 were for the old integer
    // ground temperature and pressure

    // @Param: ABS_PRESS
    // @DisplayName: Absolute Pressure
    // @Description: calibrated ground pressure in Pascals
    // @Units: Pa
    // @Increment: 1
    // @ReadOnly: True
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO("ABS_PRESS", 2, AP_Baro, sensors[0].ground_pressure, 0),

    // @Param: TEMP
    // @DisplayName: ground temperature
    // @Description: User provided ambient ground temperature in degrees Celsius. This is used to improve the calculation of the altitude the vehicle is at. This parameter is not persistent and will be reset to 0 every time the vehicle is rebooted. A value of 0 means use the internal measurement ambient temperature.
    // @Units: degC
    // @Increment: 1
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO("TEMP", 3, AP_Baro, _user_ground_temperature, 0),

    // index 4 reserved for old AP_Int8 version in legacy FRAM
    //AP_GROUPINFO("ALT_OFFSET", 4, AP_Baro, _alt_offset, 0),

    // @Param: ALT_OFFSET
    // @DisplayName: altitude offset
    // @Description: altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.
    // @Units: m
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ALT_OFFSET", 5, AP_Baro, _alt_offset, 0),

    // @Param: PRIMARY
    // @DisplayName: Primary barometer
    // @Description: This selects which barometer will be the primary if multiple barometers are found
    // @Values: 0:FirstBaro,1:2ndBaro,2:3rdBaro
    // @User: Advanced
    AP_GROUPINFO("PRIMARY", 6, AP_Baro, _primary_baro, 0),

    // @Param: EXT_BUS
    // @DisplayName: External baro bus
    // @Description: This selects the bus number for looking for an I2C barometer. When set to -1 it will probe all external i2c buses based on the GND_PROBE_EXT parameter.
    // @Values: -1:Disabled,0:Bus0,1:Bus1
    // @User: Advanced
    AP_GROUPINFO("EXT_BUS", 7, AP_Baro, _ext_bus, -1),

    // @Param: SPEC_GRAV
    // @DisplayName: Specific Gravity (For water depth measurement)
    // @Description: This sets the specific gravity of the fluid when flying an underwater ROV.
    // @Values: 1.0:Freshwater,1.024:Saltwater
    AP_GROUPINFO_FRAME("SPEC_GRAV", 8, AP_Baro, _specific_gravity, 1.0, AP_PARAM_FRAME_SUB),

#if BARO_MAX_INSTANCES > 1
    // @Param: ABS_PRESS2
    // @DisplayName: Absolute Pressure
    // @Description: calibrated ground pressure in Pascals
    // @Units: Pa
    // @Increment: 1
    // @ReadOnly: True
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO("ABS_PRESS2", 9, AP_Baro, sensors[1].ground_pressure, 0),

    // Slot 10 used to be TEMP2
#endif

#if BARO_MAX_INSTANCES > 2
    // @Param: ABS_PRESS3
    // @DisplayName: Absolute Pressure
    // @Description: calibrated ground pressure in Pascals
    // @Units: Pa
    // @Increment: 1
    // @ReadOnly: True
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO("ABS_PRESS3", 11, AP_Baro, sensors[2].ground_pressure, 0),

    // Slot 12 used to be TEMP3
#endif

    // @Param: FLTR_RNG
    // @DisplayName: Range in which sample is accepted
    // @Description: This sets the range around the average value that new samples must be within to be accepted. This can help reduce the impact of noise on sensors that are on long I2C cables. The value is a percentage from the average value. A value of zero disables this filter.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    AP_GROUPINFO("FLTR_RNG", 13, AP_Baro, _filter_range, HAL_BARO_FILTER_DEFAULT),

#ifdef HAL_PROBE_EXTERNAL_I2C_BAROS
    // @Param: PROBE_EXT
    // @DisplayName: External barometers to probe
    // @Description: This sets which types of external i2c barometer to look for. It is a bitmask of barometer types. The I2C buses to probe is based on GND_EXT_BUS. If GND_EXT_BUS is -1 then it will probe all external buses, otherwise it will probe just the bus number given in GND_EXT_BUS.
    // @Bitmask: 0:BMP085,1:BMP280,2:MS5611,3:MS5607,4:MS5637,5:FBM320,6:DPS280,7:LPS25H,8:Keller
    // @Values: 1:BMP085,2:BMP280,4:MS5611,8:MS5607,16:MS5637,32:FBM320,64:DPS280,128:LPS25H,256:Keller
    // @User: Advanced
    AP_GROUPINFO("PROBE_EXT", 14, AP_Baro, _baro_probe_ext, HAL_BARO_PROBE_EXT_DEFAULT),
#endif

    AP_GROUPEND
};

// singleton instance
AP_Baro *AP_Baro::_singleton;

/*
  AP_Baro constructor
 */
AP_Baro::AP_Baro()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void AP_Baro::calibrate(bool save)
{
    // start by assuming all sensors are calibrated (for healthy() test)
    for (uint8_t i=0; i<_num_sensors; i++) {
        sensors[i].calibrated = true;
        sensors[i].alt_ok = true;
    }

    if (hal.util->was_watchdog_reset()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Baro: skipping calibration");
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Calibrating barometer");

    // reset the altitude offset when we calibrate. The altitude
    // offset is supposed to be for within a flight
    _alt_offset.set_and_save(0);

    // let the barometer settle for a full second after startup
    // the MS5611 reads quite a long way off for the first second,
    // leading to about 1m of error if we don't wait
    for (uint8_t i = 0; i < 10; i++) {
        uint32_t tstart = AP_HAL::millis();
        do {
            update();
            if (AP_HAL::millis() - tstart > 500) {
                AP_HAL::panic("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [2]\r\n");
            }
            hal.scheduler->delay(10);
        } while (!healthy());
        hal.scheduler->delay(100);
    }

    // now average over 5 values for the ground pressure settings
    float sum_pressure[BARO_MAX_INSTANCES] = {0};
    uint8_t count[BARO_MAX_INSTANCES] = {0};
    const uint8_t num_samples = 5;

    for (uint8_t c = 0; c < num_samples; c++) {
        uint32_t tstart = AP_HAL::millis();
        do {
            update();
            if (AP_HAL::millis() - tstart > 500) {
                AP_HAL::panic("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [3]\r\n");
            }
        } while (!healthy());
        for (uint8_t i=0; i<_num_sensors; i++) {
            if (healthy(i)) {
                sum_pressure[i] += sensors[i].pressure;
                count[i] += 1;
            }
        }
        hal.scheduler->delay(100);
    }
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (count[i] == 0) {
            sensors[i].calibrated = false;
        } else {
            if (save) {
                sensors[i].ground_pressure.set_and_save(sum_pressure[i] / count[i]);
            }
        }
    }

    _guessed_ground_temperature = get_external_temperature();

    // panic if all sensors are not calibrated
    uint8_t num_calibrated = 0;
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (sensors[i].calibrated) {
            gcs().send_text(MAV_SEVERITY_INFO, "Barometer %u calibration complete", i+1);
            num_calibrated++;
        }
    }
    if (num_calibrated) {
        return;
    }
    AP_HAL::panic("AP_Baro: all sensors uncalibrated");
}

/*
   update the barometer calibration
   this updates the baro ground calibration to the current values. It
   can be used before arming to keep the baro well calibrated
*/
void AP_Baro::update_calibration()
{
    const uint32_t now = AP_HAL::millis();
    const bool do_notify = now - _last_notify_ms > 10000;
    if (do_notify) {
        _last_notify_ms = now;
    }
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (healthy(i)) {
            float corrected_pressure = get_pressure(i) + sensors[i].p_correction;
            sensors[i].ground_pressure.set(corrected_pressure);
        }

        // don't notify the GCS too rapidly or we flood the link
        if (do_notify) {
            sensors[i].ground_pressure.notify();
        }
    }

    // always update the guessed ground temp
    _guessed_ground_temperature = get_external_temperature();

    // force EAS2TAS to recalculate
    _EAS2TAS = 0;
}

// return altitude difference in meters between current pressure and a
// given base_pressure in Pascal
float AP_Baro::get_altitude_difference(float base_pressure, float pressure) const
{
    float ret;
    float temp    = get_ground_temperature() + C_TO_KELVIN;
    float scaling = pressure / base_pressure;

    // This is an exact calculation that is within +-2.5m of the standard
    // atmosphere tables in the troposphere (up to 11,000 m amsl).
    ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));

    return ret;
}


// return current scale factor that converts from equivalent to true airspeed
// valid for altitudes up to 10km AMSL
// assumes standard atmosphere lapse rate
float AP_Baro::get_EAS2TAS(void)
{
    float altitude = get_altitude();
    if ((fabsf(altitude - _last_altitude_EAS2TAS) < 25.0f) && !is_zero(_EAS2TAS)) {
        // not enough change to require re-calculating
        return _EAS2TAS;
    }

    // only estimate lapse rate for the difference from the ground location
    // provides a more consistent reading then trying to estimate a complete
    // ISA model atmosphere
    float tempK = get_ground_temperature() + C_TO_KELVIN - ISA_LAPSE_RATE * altitude;
    const float eas2tas_squared = SSL_AIR_DENSITY / ((float)get_pressure() / (ISA_GAS_CONSTANT * tempK));
    if (!is_positive(eas2tas_squared)) {
        return 1.0;
    }
    _EAS2TAS = sqrtf(eas2tas_squared);
    _last_altitude_EAS2TAS = altitude;
    return _EAS2TAS;
}

// return air density / sea level density - decreases as altitude climbs
float AP_Baro::get_air_density_ratio(void)
{
    const float eas2tas = get_EAS2TAS();
    if (eas2tas > 0.0f) {
        return 1.0f/(sq(eas2tas));
    } else {
        return 1.0f;
    }
}

// return current climb_rate estimate relative to time that calibrate()
// was called. Returns climb rate in meters/s, positive means up
// note that this relies on read() being called regularly to get new data
float AP_Baro::get_climb_rate(void)
{
    if (_hil.have_alt) {
        return _hil.climb_rate;
    }
    // we use a 7 point derivative filter on the climb rate. This seems
    // to produce somewhat reasonable results on real hardware
    return _climb_rate_filter.slope() * 1.0e3f;
}

// returns the ground temperature in degrees C, selecting either a user
// provided one, or the internal estimate
float AP_Baro::get_ground_temperature(void) const
{
    if (is_zero(_user_ground_temperature)) {
        return _guessed_ground_temperature;
    } else {
        return _user_ground_temperature;
    }
}


/*
  set external temperature to be used for calibration (degrees C)
 */
void AP_Baro::set_external_temperature(float temperature)
{
    _external_temperature = temperature;
    _last_external_temperature_ms = AP_HAL::millis();
}

/*
  get the temperature in degrees C to be used for calibration purposes
 */
float AP_Baro::get_external_temperature(const uint8_t instance) const
{
    // if we have a recent external temperature then use it
    if (_last_external_temperature_ms != 0 && AP_HAL::millis() - _last_external_temperature_ms < 10000) {
        return _external_temperature;
    }
    
    // if we don't have an external temperature then try to use temperature
    // from the airspeed sensor
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr) {
        float temperature;
        if (airspeed->healthy() && airspeed->get_temperature(temperature)) {
            return temperature;
        }
    }
    
    // if we don't have an external temperature and airspeed temperature
    // then use the minimum of the barometer temperature and 35 degrees C.
    // The reason for not just using the baro temperature is it tends to read high,
    // often 30 degrees above the actual temperature. That means the
    // EAS2TAS tends to be off by quite a large margin, as well as
    // the calculation of altitude difference betweeen two pressures
    // reporting a high temperature will cause the aircraft to
    // estimate itself as flying higher then it actually is.
    return MIN(get_temperature(instance), INTERNAL_TEMPERATURE_CLAMP);
}


bool AP_Baro::_add_backend(AP_Baro_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (_num_drivers >= BARO_MAX_DRIVERS) {
        AP_HAL::panic("Too many barometer drivers");
    }
    drivers[_num_drivers++] = backend;
    return true;
}

/*
  macro to add a backend with check for too many sensors
 We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(backend) \
    do { _add_backend(backend);     \
       if (_num_drivers == BARO_MAX_DRIVERS || \
          _num_sensors == BARO_MAX_INSTANCES) { \
          return; \
       } \
    } while (0)

/*
  initialise the barometer object, loading backend drivers
 */
void AP_Baro::init(void)
{
    // ensure that there isn't a previous ground temperature saved
    if (!is_zero(_user_ground_temperature)) {
        _user_ground_temperature.set_and_save(0.0f);
        _user_ground_temperature.notify();
    }

    if (_hil_mode) {
        drivers[0] = new AP_Baro_HIL(*this);
        _num_drivers = 1;
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    ADD_BACKEND(new AP_Baro_SITL(*this));
    return;
#endif

#if HAL_WITH_UAVCAN
    // Detect UAVCAN Modules, try as many times as there are driver slots
    for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
        ADD_BACKEND(AP_Baro_UAVCAN::probe(*this));
    }
#endif

#if defined(HAL_BARO_PROBE_LIST)
    // probe list from BARO lines in hwdef.dat
    HAL_BARO_PROBE_LIST;
#elif AP_FEATURE_BOARD_DETECT
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PX4V1:
#ifdef HAL_BARO_MS5611_I2C_BUS
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.i2c_mgr->get_device(HAL_BARO_MS5611_I2C_BUS, HAL_BARO_MS5611_I2C_ADDR))));
#endif
        break;

    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
    case AP_BoardConfig::PX4_BOARD_PHMINI:
    case AP_BoardConfig::PX4_BOARD_AUAV21:
    case AP_BoardConfig::PX4_BOARD_PH2SLIM:
    case AP_BoardConfig::PX4_BOARD_PIXHAWK_PRO:
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.spi->get_device(HAL_BARO_MS5611_NAME))));
        break;

    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
    case AP_BoardConfig::PX4_BOARD_SP01:
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.spi->get_device(HAL_BARO_MS5611_SPI_EXT_NAME))));
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.spi->get_device(HAL_BARO_MS5611_NAME))));
        break;

    case AP_BoardConfig::PX4_BOARD_MINDPXV2:
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.spi->get_device(HAL_BARO_MS5611_NAME))));
        break;

    case AP_BoardConfig::PX4_BOARD_AEROFC:
#ifdef HAL_BARO_MS5607_I2C_BUS
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.i2c_mgr->get_device(HAL_BARO_MS5607_I2C_BUS, HAL_BARO_MS5607_I2C_ADDR)),
                                          AP_Baro_MS56XX::BARO_MS5607));
#endif
        break;

    case AP_BoardConfig::VRX_BOARD_BRAIN54:
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.spi->get_device(HAL_BARO_MS5611_NAME))));
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.spi->get_device(HAL_BARO_MS5611_SPI_EXT_NAME))));
#ifdef HAL_BARO_MS5611_SPI_IMU_NAME
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.spi->get_device(HAL_BARO_MS5611_SPI_IMU_NAME))));
#endif
        break;

    case AP_BoardConfig::VRX_BOARD_BRAIN51:
    case AP_BoardConfig::VRX_BOARD_BRAIN52:
    case AP_BoardConfig::VRX_BOARD_BRAIN52E:
    case AP_BoardConfig::VRX_BOARD_CORE10:
    case AP_BoardConfig::VRX_BOARD_UBRAIN51:
    case AP_BoardConfig::VRX_BOARD_UBRAIN52:
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.spi->get_device(HAL_BARO_MS5611_NAME))));
        break;

    case AP_BoardConfig::PX4_BOARD_PCNC1:
        ADD_BACKEND(AP_Baro_ICM20789::probe(*this,
                                            std::move(hal.i2c_mgr->get_device(1, 0x63)),
                                            std::move(hal.spi->get_device(HAL_INS_MPU60x0_NAME))));
        break;

    case AP_BoardConfig::PX4_BOARD_FMUV5:
    case AP_BoardConfig::PX4_BOARD_FMUV6:
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.spi->get_device(HAL_BARO_MS5611_NAME))));
        break;

    default:
        break;
    }
#elif HAL_BARO_DEFAULT == HAL_BARO_HIL
    drivers[0] = new AP_Baro_HIL(*this);
    _num_drivers = 1;
#elif HAL_BARO_DEFAULT == HAL_BARO_BMP085
    ADD_BACKEND(AP_Baro_BMP085::probe(*this,
                                      std::move(hal.i2c_mgr->get_device(HAL_BARO_BMP085_BUS, HAL_BARO_BMP085_I2C_ADDR))));
#elif HAL_BARO_DEFAULT == HAL_BARO_BMP280_I2C
    ADD_BACKEND(AP_Baro_BMP280::probe(*this,
                                      std::move(hal.i2c_mgr->get_device(HAL_BARO_BMP280_BUS, HAL_BARO_BMP280_I2C_ADDR))));
#elif HAL_BARO_DEFAULT == HAL_BARO_BMP280_SPI
    ADD_BACKEND(AP_Baro_BMP280::probe(*this,
                                      std::move(hal.spi->get_device(HAL_BARO_BMP280_NAME))));
#elif HAL_BARO_DEFAULT == HAL_BARO_MS5611_I2C
    ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                      std::move(hal.i2c_mgr->get_device(HAL_BARO_MS5611_I2C_BUS, HAL_BARO_MS5611_I2C_ADDR))));
#elif HAL_BARO_DEFAULT == HAL_BARO_MS5611_SPI
    ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                      std::move(hal.spi->get_device(HAL_BARO_MS5611_NAME))));
#elif HAL_BARO_DEFAULT == HAL_BARO_MS5607_I2C
    ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                      std::move(hal.i2c_mgr->get_device(HAL_BARO_MS5607_I2C_BUS, HAL_BARO_MS5607_I2C_ADDR)),
                                      AP_Baro_MS56XX::BARO_MS5607));
#elif HAL_BARO_DEFAULT == HAL_BARO_MS5637_I2C
    ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                      std::move(hal.i2c_mgr->get_device(HAL_BARO_MS5637_I2C_BUS, HAL_BARO_MS5637_I2C_ADDR)),
                                      AP_Baro_MS56XX::BARO_MS5637));
#elif HAL_BARO_DEFAULT == HAL_BARO_FBM320_I2C
    ADD_BACKEND(AP_Baro_FBM320::probe(*this,
                                      std::move(hal.i2c_mgr->get_device(HAL_BARO_FBM320_I2C_BUS, HAL_BARO_FBM320_I2C_ADDR))));
#elif HAL_BARO_DEFAULT == HAL_BARO_DPS280_I2C
    ADD_BACKEND(AP_Baro_DPS280::probe(*this,
                                      std::move(hal.i2c_mgr->get_device(HAL_BARO_DPS280_I2C_BUS, HAL_BARO_DPS280_I2C_ADDR))));
#elif HAL_BARO_DEFAULT == HAL_BARO_DPS280_SPI
    ADD_BACKEND(AP_Baro_DPS280::probe(*this, std::move(hal.spi->get_device("dps280"))));
#elif HAL_BARO_DEFAULT == HAL_BARO_LPS25H
	ADD_BACKEND(AP_Baro_LPS2XH::probe(*this,
                                      std::move(hal.i2c_mgr->get_device(HAL_BARO_LPS25H_I2C_BUS, HAL_BARO_LPS25H_I2C_ADDR))));
#elif HAL_BARO_DEFAULT == HAL_BARO_LPS25H_IMU_I2C
	ADD_BACKEND(AP_Baro_LPS2XH::probe_InvensenseIMU(*this,
                                                    std::move(hal.i2c_mgr->get_device(HAL_BARO_LPS25H_I2C_BUS, HAL_BARO_LPS25H_I2C_ADDR)),
                                                    HAL_BARO_LPS25H_I2C_IMU_ADDR));
#elif HAL_BARO_DEFAULT == HAL_BARO_20789_I2C_I2C
    ADD_BACKEND(AP_Baro_ICM20789::probe(*this,
                                        std::move(hal.i2c_mgr->get_device(HAL_BARO_20789_I2C_BUS, HAL_BARO_20789_I2C_ADDR_PRESS)),
                                        std::move(hal.i2c_mgr->get_device(HAL_BARO_20789_I2C_BUS, HAL_BARO_20789_I2C_ADDR_ICM))));
#elif HAL_BARO_DEFAULT == HAL_BARO_20789_I2C_SPI
    ADD_BACKEND(AP_Baro_ICM20789::probe(*this,
                                        std::move(hal.i2c_mgr->get_device(HAL_BARO_20789_I2C_BUS, HAL_BARO_20789_I2C_ADDR_PRESS)),
                                        std::move(hal.spi->get_device("icm20789"))));
#elif HAL_BARO_DEFAULT == HAL_BARO_LPS22H_SPI
    ADD_BACKEND(AP_Baro_LPS2XH::probe(*this,
                                      std::move(hal.spi->get_device(HAL_BARO_LPS22H_NAME))));
#elif HAL_BARO_DEFAULT == HAL_BARO_LPS22H_I2C
    ADD_BACKEND(AP_Baro_LPS2XH::probe(*this,
                                      std::move(hal.i2c_mgr->get_device(HAL_BARO_LPS22H_I2C_BUS, HAL_BARO_LPS22H_I2C_ADDR))));
#endif

    // can optionally have baro on I2C too
    if (_ext_bus >= 0) {
#if APM_BUILD_TYPE(APM_BUILD_ArduSub)
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.i2c_mgr->get_device(_ext_bus, HAL_BARO_MS5837_I2C_ADDR)), AP_Baro_MS56XX::BARO_MS5837));

        ADD_BACKEND(AP_Baro_KellerLD::probe(*this,
                                          std::move(hal.i2c_mgr->get_device(_ext_bus, HAL_BARO_KELLERLD_I2C_ADDR))));
#else
        ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                          std::move(hal.i2c_mgr->get_device(_ext_bus, HAL_BARO_MS5611_I2C_ADDR))));
#endif
    }

#ifdef HAL_PROBE_EXTERNAL_I2C_BAROS
    _probe_i2c_barometers();
#endif

#if !defined(HAL_BARO_ALLOW_INIT_NO_BARO) // most boards requires external baro

    if (_num_drivers == 0 || _num_sensors == 0 || drivers[0] == nullptr) {
        AP_BoardConfig::sensor_config_error("Baro: unable to initialise driver");
    }
#endif
}

/*
  probe all the i2c barometers enabled with GND_PROBE_EXT. This is
  used on boards without a builtin barometer
 */
void AP_Baro::_probe_i2c_barometers(void)
{
    uint32_t probe = _baro_probe_ext.get();
    uint32_t mask = hal.i2c_mgr->get_bus_mask_external();
    if (AP_BoardConfig::get_board_type() == AP_BoardConfig::PX4_BOARD_PIXHAWK2) {
        // for the purpose of baro probing, treat CubeBlack internal i2c as external. It has
        // no internal i2c baros, so this is safe
        mask |= hal.i2c_mgr->get_bus_mask_internal();
    }
    // if the user has set GND_EXT_BUS then probe the bus given by that parameter
    int8_t ext_bus = _ext_bus;
    if (ext_bus >= 0) {
        mask = 1U << (uint8_t)ext_bus;
    }
    if (probe & PROBE_BMP085) {
        FOREACH_I2C_MASK(i,mask) {
            ADD_BACKEND(AP_Baro_BMP085::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_BMP085_I2C_ADDR))));
        }
    }
    if (probe & PROBE_BMP280) {
        FOREACH_I2C_MASK(i,mask) {
            ADD_BACKEND(AP_Baro_BMP280::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_BMP280_I2C_ADDR))));
            ADD_BACKEND(AP_Baro_BMP280::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_BMP280_I2C_ADDR2))));
        }
    }
    if (probe & PROBE_MS5611) {
        FOREACH_I2C_MASK(i,mask) {
            ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_MS5611_I2C_ADDR))));
            ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_MS5611_I2C_ADDR2))));
        }
    }
    if (probe & PROBE_MS5607) {
        FOREACH_I2C_MASK(i,mask) {
            ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_MS5607_I2C_ADDR)),
                                              AP_Baro_MS56XX::BARO_MS5607));
        }
    }
    if (probe & PROBE_MS5637) {
        FOREACH_I2C_MASK(i,mask) {
            ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_MS5637_I2C_ADDR)),
                                              AP_Baro_MS56XX::BARO_MS5637));
        }
    }
    if (probe & PROBE_FBM320) {
        FOREACH_I2C_MASK(i,mask) {
            ADD_BACKEND(AP_Baro_FBM320::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_FBM320_I2C_ADDR))));
            ADD_BACKEND(AP_Baro_FBM320::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_FBM320_I2C_ADDR2))));
        }
    }
    if (probe & PROBE_DPS280) {
        FOREACH_I2C_MASK(i,mask) {
            ADD_BACKEND(AP_Baro_DPS280::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_DPS280_I2C_ADDR))));
            ADD_BACKEND(AP_Baro_DPS280::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_DPS280_I2C_ADDR2))));
        }
    }
    if (probe & PROBE_LPS25H) {
        FOREACH_I2C_MASK(i,mask) {
            ADD_BACKEND(AP_Baro_LPS2XH::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_LPS25H_I2C_ADDR))));
        }
    }
    if (probe & PROBE_LPS25H) {
        FOREACH_I2C_MASK(i,mask) {
            ADD_BACKEND(AP_Baro_KellerLD::probe(*this,
                                                std::move(hal.i2c_mgr->get_device(i, HAL_BARO_KELLERLD_I2C_ADDR))));
        }
    }
    if (probe & PROBE_MS5837) {
        FOREACH_I2C_MASK(i,mask) {
            ADD_BACKEND(AP_Baro_MS56XX::probe(*this,
                                              std::move(hal.i2c_mgr->get_device(i, HAL_BARO_MS5837_I2C_ADDR)), AP_Baro_MS56XX::BARO_MS5837));
        }
    }
}

bool AP_Baro::should_log() const
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return false;
    }
    if (_log_baro_bit == (uint32_t)-1) {
        return false;
    }
    if (!logger->should_log(_log_baro_bit)) {
        return false;
    }
    return true;
}

/*
  call update on all drivers
 */
void AP_Baro::update(void)
{
    WITH_SEMAPHORE(_rsem);

    if (fabsf(_alt_offset - _alt_offset_active) > 0.01f) {
        // If there's more than 1cm difference then slowly slew to it via LPF.
        // The EKF does not like step inputs so this keeps it happy.
        _alt_offset_active = (0.95f*_alt_offset_active) + (0.05f*_alt_offset);
    } else {
        _alt_offset_active = _alt_offset;
    }

    if (!_hil_mode) {
        for (uint8_t i=0; i<_num_drivers; i++) {
            drivers[i]->backend_update(i);
        }
    }

    for (uint8_t i=0; i<_num_sensors; i++) {
        if (sensors[i].healthy) {
            // update altitude calculation
            float ground_pressure = sensors[i].ground_pressure;
            if (!is_positive(ground_pressure) || isnan(ground_pressure) || isinf(ground_pressure)) {
                sensors[i].ground_pressure = sensors[i].pressure;
            }
            float altitude = sensors[i].altitude;
            float corrected_pressure = sensors[i].pressure + sensors[i].p_correction;
            if (sensors[i].type == BARO_TYPE_AIR) {
                altitude = get_altitude_difference(sensors[i].ground_pressure, corrected_pressure);
            } else if (sensors[i].type == BARO_TYPE_WATER) {
                //101325Pa is sea level air pressure, 9800 Pascal/ m depth in water.
                //No temperature or depth compensation for density of water.
                altitude = (sensors[i].ground_pressure - corrected_pressure) / 9800.0f / _specific_gravity;
            }
            // sanity check altitude
            sensors[i].alt_ok = !(isnan(altitude) || isinf(altitude));
            if (sensors[i].alt_ok) {
                sensors[i].altitude = altitude + _alt_offset_active;
            }
        }
        if (_hil.have_alt) {
            sensors[0].altitude = _hil.altitude;
        }
        if (_hil.have_last_update) {
            sensors[0].last_update_ms = _hil.last_update_ms;
        }
    }

    // ensure the climb rate filter is updated
    if (healthy()) {
        _climb_rate_filter.update(get_altitude(), get_last_update());
    }

    // choose primary sensor
    if (_primary_baro >= 0 && _primary_baro < _num_sensors && healthy(_primary_baro)) {
        _primary = _primary_baro;
    } else {
        _primary = 0;
        for (uint8_t i=0; i<_num_sensors; i++) {
            if (healthy(i)) {
                _primary = i;
                break;
            }
        }
    }

    // logging
    if (should_log() && !AP::ahrs().have_ekf_logging()) {
        AP::logger().Write_Baro();
    }
}

/*
  call accumulate on all drivers
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
        AP_HAL::panic("Too many barometers");
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

// set a pressure correction from AP_TempCalibration
void AP_Baro::set_pressure_correction(uint8_t instance, float p_correction)
{
    if (instance < _num_sensors) {
        sensors[instance].p_correction = p_correction;
    }
}

namespace AP {

AP_Baro &baro()
{
    return *AP_Baro::get_singleton();
}

};
