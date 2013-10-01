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
 *   APM_Airspeed.cpp - airspeed (pitot) driver
 */


#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_ADC.h>
#include <AP_Airspeed.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
 #include <AP_ADC_AnalogSource.h>
 #define ARSPD_DEFAULT_PIN 64
 extern AP_ADC_ADS7844 apm1_adc;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
 #define ARSPD_DEFAULT_PIN 0
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 #define ARSPD_DEFAULT_PIN 0
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
 #include <sys/stat.h>
 #include <sys/types.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <systemlib/airspeed.h>
 #include <drivers/drv_airspeed.h>
 #include <uORB/topics/differential_pressure.h>
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
 #define ARSPD_DEFAULT_PIN 11
#else
 #define ARSPD_DEFAULT_PIN 15
#endif
#elif CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
 #define ARSPD_DEFAULT_PIN 16
#else
 #define ARSPD_DEFAULT_PIN 0
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Airspeed::var_info[] PROGMEM = {

    // @Param: ENABLE
    // @DisplayName: Airspeed enable
    // @Description: enable airspeed sensor
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO("ENABLE",    0, AP_Airspeed, _enable, 1),

    // @Param: USE
    // @DisplayName: Airspeed use
    // @Description: use airspeed for flight control
    // @Values: 1:Use,0:Don't Use
    AP_GROUPINFO("USE",    1, AP_Airspeed, _use, 0),

    // @Param: OFFSET
    // @DisplayName: Airspeed offset
    // @Description: Airspeed calibration offset
    // @Increment: 0.1
    AP_GROUPINFO("OFFSET", 2, AP_Airspeed, _offset, 0),

    // @Param: RATIO
    // @DisplayName: Airspeed ratio
    // @Description: Airspeed calibration ratio
    // @Increment: 0.1
    AP_GROUPINFO("RATIO",  3, AP_Airspeed, _ratio, 1.9936f),

    // @Param: PIN
    // @DisplayName: Airspeed pin
    // @Description: The analog pin number that the airspeed sensor is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated airspeed port on the end of the board. Set to 11 on PX4 for the analog airspeed port. Set to 15 on the Pixhawk for the analog airspeed port. Set to 65 on the PX4 or Pixhawk for an EagleTree or MEAS I2C airspeed sensor.
    // @User: Advanced
    AP_GROUPINFO("PIN",  4, AP_Airspeed, _pin, ARSPD_DEFAULT_PIN),

    // @Param: AUTOCAL
    // @DisplayName: Automatic airspeed ratio calibration
    // @Description: If this is enabled then the APM will automatically adjust the ARSPD_RATIO during flight, based upon an estimation filter using ground speed and true airspeed. The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5%
    // @User: Advanced
    AP_GROUPINFO("AUTOCAL",  5, AP_Airspeed, _autocal, 0),

    AP_GROUPEND
};


/*
  this scaling factor converts from the old system where we used a 
  0 to 4095 raw ADC value for 0-5V to the new system which gets the
  voltage in volts directly from the ADC driver
 */
#define SCALING_OLD_CALIBRATION 819 // 4095/5

void AP_Airspeed::init()
{
    _last_pressure = 0;
    _calibration.init(_ratio);
    _last_saved_ratio = _ratio;
    _counter = 0;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    if (_pin == 65) {
        _ets_fd = open(AIRSPEED_DEVICE_PATH, O_RDONLY);
        if (_ets_fd == -1) {
            hal.console->println("Failed to open ETS airspeed driver");
            _enable.set(0);
            return;
        }
        if (OK != ioctl(_ets_fd, SENSORIOCSPOLLRATE, 100) ||
            OK != ioctl(_ets_fd, SENSORIOCSQUEUEDEPTH, 15)) {
            hal.console->println("Failed to setup ETS driver rate and queue");
        }
        return;
    }
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    if (_pin == 64) {
        _source = new AP_ADC_AnalogSource( &apm1_adc, 7, 1.0f);
        return;
    }
#endif
    _source = hal.analogin->channel(_pin);
}

// read the airspeed sensor
float AP_Airspeed::get_pressure(void)
{
    if (!_enable) {
        return 0;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    if (_ets_fd != -1) {
        // read from the ETS airspeed sensor
        float sum = 0;
        uint16_t count = 0;
        struct differential_pressure_s report;
        static uint64_t last_timestamp;

        while (::read(_ets_fd, &report, sizeof(report)) == sizeof(report) &&
               report.timestamp != last_timestamp) {
            sum += report.differential_pressure_pa;
            count++;
            last_timestamp = report.timestamp;
        }
        // hal.console->printf("count=%u\n", (unsigned)count);
        if (count == 0) {
            return _last_pressure;
        }
        _last_pressure = sum / count;
        return _last_pressure;
    }
#endif

    if (_source == NULL) {
        return 0;
    }
    _source->set_pin(_pin);
    _last_pressure = _source->voltage_average_ratiometric() * SCALING_OLD_CALIBRATION;
    return _last_pressure;
}

// calibrate the airspeed. This must be called at least once before
// the get_airspeed() interface can be used
void AP_Airspeed::calibrate()
{
    float sum = 0;
    uint8_t c;
    if (!_enable) {
        return;
    }
    // discard first reading
    get_pressure();
    for (c = 0; c < 10; c++) {
        hal.scheduler->delay(100);
        sum += get_pressure();
    }
    float raw = sum/c;
    _offset.set_and_save(raw);
    _airspeed = 0;
    _raw_airspeed = 0;
}

// read the airspeed sensor
void AP_Airspeed::read(void)
{
    float airspeed_pressure;
    if (!_enable) {
        return;
    }
    float raw               = get_pressure();
    airspeed_pressure       = max(raw - _offset, 0);
    _raw_airspeed           = sqrtf(airspeed_pressure * _ratio);
    _airspeed               = 0.7f * _airspeed  +  0.3f * _raw_airspeed;
}
