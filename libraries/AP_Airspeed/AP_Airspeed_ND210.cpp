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
  backend driver for airspeed from a I2C ND210 sensor
 */
#include "AP_Airspeed_ND210.h"

#if AP_AIRSPEED_ND210_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_Baro/AP_Baro.h>

#include <stdio.h>

// default device address
#define ND210_I2C_ADDR            	0x28

// used in pressure calculation. (.90*2^15)
#define ND210_SCALE_PRESSURE_ND210	29491.2

// 1 H20 =  249.08890833333 Pa
#define H20_TO_PASCAL 249.089

/*
 * MODE_CONTROL_REGISTER
 *
 * PressureRange
 * Bits 0-2 control the output pressure range.
 *   Pressure Range, 3 bits, in H20
 *     000 0.25  -- gives a max airspeed of 11 m/s
 *     001 0.25
 *     010 0.5
 *     011 1.0
 *     100 2.0
 *     101 4.0
 *     110 5.0    -- gives a max airspeed of ~50 m/s
 *     111 10.0   -- gives a max airspeed of ~75 m/s
 * _selected_pressure_range should match (or drive) the value of PressureRange
 *
 * Watchdog Enable bit
 * Bit 3 is the I/O Watchdog Enable bit. When set, the I/O watchdog is enabled.
 * When enabled, the I/O watchdog will monitor the I/O activity.
 * If I/O activity is not detected for the I/O Watchdog timeout time, the pressure sensor will reset itself.
 * The I/O watchdog timeout time is determined by the currently active bandwidth setting.
 *
 * Bandwidth Limit Select, 3 bits
 * Bits 4-6 control the BW Limit Filter.
 * 000 1 Hz
 * 001 2 Hz
 * 010 5 Hz
 * 011 10 Hz
 * 100 20 Hz
 * 101 50 Hz
 * 110 100 Hz
 * 111 200 Hz
 *
 * Bit 7 is the Notch Filter Enable bit. When enabled, the 50/60Hz notch filter is active *
 *
 *  ModeControlRegister default value is 0xF6 (b1 111 0 110 ) Notch (1), 200Hz (b111), No Watchdog, PressureRange = 5 (b110)
 *  (NotchFilterEnable << 7) | (BandWidth << 6) | (WatchdogEnable << 4) | PressureRange;
 */
static constexpr uint8_t MODE_CONTROL_REGISTER = 0xF7; // 0xF6 is default

/*
 * The Rate Control Register controls the rate at which the DAV
 * pin is asserted indicating new data is available. This register
 * is primarily used to throttle down the actual data transfer
 * rate (when using the DAV as the trigger to sample) since the
 * general industrial requirement is less than the internal 444Hz update rate.
 * The value of the Rate Control Register is
 * the divisor of the 444 Hz internal data rate. Since a divisor of
 * zero is not possible, a zero value will select the auto-select
 * rate mode. In this mode, the rate is selected based on the
 * selected bandwidth limit. The auto rate value is roughly 3x
 * the corner frequency of the selected bandwidth limit in all
 * auto selected rates (where possible).
 * 00000000 = AutoSelect (based on BW)
 * 00000001 = 444Hz
 * 00000010 = 222Hz ...
 */
static constexpr uint8_t RATE_CONTROL_REGISTER = 0x00; // Auto Select aka (BW = 1Hz, so Rate will be 3Hz)

extern const AP_HAL::HAL &hal;

AP_Airspeed_ND210::AP_Airspeed_ND210(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

/*
  send a 16 bit command code
 */
bool AP_Airspeed_ND210::_send_configuration_command()
{
    uint8_t b[2] {MODE_CONTROL_REGISTER, RATE_CONTROL_REGISTER};
    return _dev->transfer(b, 2, nullptr, 0);
}

// probe and initialise the sensor
bool AP_Airspeed_ND210::init()
{
    _dev = hal.i2c_mgr->get_device(get_bus(), ND210_I2C_ADDR);
    if (!_dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ND210 no sensor found at 0x%02X", ND210_I2C_ADDR);
        return false;
    }
    _dev->get_semaphore()->take_blocking();
    // lots of retries during probe
    _dev->set_retries(10);

    // set Mode and Rate modes
    if (!_send_configuration_command()) {
        _dev->get_semaphore()->give();
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ND210[%u]: Init Send Command Failed.", get_instance());
        return false;
    }

    _dev->get_semaphore()->give();

    switch(MODE_CONTROL_REGISTER & 0x07) {
        case 0b000:
        case 0b001:
            _selected_pressure_range = 0.25;
            break;
        case 0b010:
            _selected_pressure_range = 0.5;
            break;
        case 0b011:
            _selected_pressure_range = 1.0;
            break;
        case 0b100:
            _selected_pressure_range = 2.0;
            break;
        case 0b101:
            _selected_pressure_range = 4.0;
            break;
        case 0b110:
            _selected_pressure_range = 5.0;
            break;
        case 0b111:
            _selected_pressure_range = 10.0;
            break;
    }

    /*
      this sensor uses zero offset and skips cal
     */
#if 0  // Airspeed seems more accurate without these.
    set_use_zero_offset();
    set_skip_cal();
    set_offset(0);
#endif

    _dev->set_device_type(uint8_t(DevType::ND210));
    set_bus_id(_dev->get_bus_id());

    // drop to 2 retries for runtime
    _dev->set_retries(2);

    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&AP_Airspeed_ND210::_timer, void));

    _print_info();

    return true;
}

void AP_Airspeed_ND210::_print_info()
{
#if HAL_GCS_ENABLED
    _dev->get_semaphore()->take_blocking();

    uint8_t val[22];
    bool ret = _dev->transfer(nullptr, 0, val, sizeof(val));
    if (ret == false) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ND210 Init Read Failed");
        _dev->get_semaphore()->give();
        return;
    }
    _dev->get_semaphore()->give();

    // Model, 5-12, 8 byte, ASCII, null terminated,   
    // Right reading ASCII with null terminator, 
    // eg  4EH,44H,32H,31H,30H,00H,xxH,xxH = ND210
    char model[8];
    memcpy(model, &val[4], sizeof(model));
    // Serial Number, 13-16, 4 byte, Hex, 
    // Unique 4 byte serial for each part, 
    // eg  2FD627A4H
    uint32_t serialNumber = ( (((uint32_t)val[12]) << 24) | (((uint32_t)val[13]) << 16) | ( ((uint32_t)val[14]) << 8 ) | (uint32_t)val[15] );
    // Build Number,  17-22, 6 byte, ASCII, null terminated,  
    // Right reading ASCII with null  termination, 
    // eg 30H,30H,30H,33H,43H,00H = 0003C
    char buildNumber[6];
    memcpy(buildNumber, &val[16], sizeof(buildNumber) );

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "ND210: Model=%s at address=0x%02x",
                  model,
                  ND210_I2C_ADDR
                  );
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "ND210: Serial#=0x%04X, Build#=%s",
                  (unsigned int) serialNumber,
                  buildNumber
                  );
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "ND210: _selected_pressure_range=%f",
                  _selected_pressure_range
                  );
#endif
}


// read the values from the sensor. Called at 50Hz
void AP_Airspeed_ND210::_timer()
{
    // read 4 bytes from the sensor
    uint8_t val[4];
    bool ret = _dev->transfer(nullptr, 0, val, sizeof(val));
    uint32_t now = AP_HAL::millis();
    if (ret == false) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ND210 Read Failed");
        if (now - _last_sample_time_ms > 200) {
            // reset configuration
            _send_configuration_command();
        }
        return;
    }

    int16_t P = (((int16_t)val[0]) << 8) | val[1];
    float diff_pressure_h20 = (float(P) / ND210_SCALE_PRESSURE_ND210) * _selected_pressure_range;
    // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ND210 Raw Pressure = 0x%02X", P);

    uint8_t temperatureInteger = val[2];
    float temperatureFractional = val[3] / 256; // convert byte to fraction.
    float temperatureCelsius = temperatureInteger + temperatureFractional;
    // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ND210 Temperature Celcius = %f", temperatureCelsius);

    WITH_SEMAPHORE(sem);

    _press_sum += (diff_pressure_h20 * H20_TO_PASCAL);
    _temp_sum += temperatureCelsius;
    _press_count++;
    _temp_count++;
    _last_sample_time_ms = now;
}


// return the current differential_pressure in Pascal
bool AP_Airspeed_ND210::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);

    if (AP_HAL::millis() - _last_sample_time_ms > 100) {
        return false;
    }

    if (_press_count > 0) {
        _press = _press_sum / _press_count;
        _press_count = 0;
        _press_sum = 0;
    }

    pressure = _press;

    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_ND210::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_temp_count > 0) {
        _temp = _temp_sum / _temp_count;
        _temp_count = 0;
        _temp_sum = 0;
    }

    temperature = _temp;

    return true;
}

#endif  // AP_AIRSPEED_SDP3X_ENABLED
