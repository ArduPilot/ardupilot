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
  backend driver for airspeed from a I2C SDP3X sensor

  with thanks to https://github.com/PX4/Firmware/blob/master/src/drivers/sdp3x_airspeed
 */
#include "AP_Airspeed_SDP3X.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Baro/AP_Baro.h>

#include <stdio.h>

#define SDP3X_SCALE_TEMPERATURE		200.0f

#define SDP3XD0_I2C_ADDR            	0x21
#define SDP3XD1_I2C_ADDR            	0x22
#define SDP3XD2_I2C_ADDR            	0x23

#define SDP3X_CONT_MEAS_AVG_MODE    	0x3615
#define SDP3X_CONT_MEAS_STOP        	0x3FF9

#define SDP3X_SCALE_PRESSURE_SDP31	60
#define SDP3X_SCALE_PRESSURE_SDP32	240
#define SDP3X_SCALE_PRESSURE_SDP33	20

extern const AP_HAL::HAL &hal;

AP_Airspeed_SDP3X::AP_Airspeed_SDP3X(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

/*
  send a 16 bit command code
 */
bool AP_Airspeed_SDP3X::_send_command(uint16_t cmd)
{
    uint8_t b[2] {uint8_t(cmd >> 8), uint8_t(cmd & 0xFF)};
    return _dev->transfer(b, 2, nullptr, 0);
}

// probe and initialise the sensor
bool AP_Airspeed_SDP3X::init()
{
    const uint8_t addresses[3] = { SDP3XD0_I2C_ADDR,
                                   SDP3XD1_I2C_ADDR,
                                   SDP3XD2_I2C_ADDR
                                 };
    bool found = false;
    bool ret = false;

    for (uint8_t i=0; i<ARRAY_SIZE(addresses) && !found; i++) {
        _dev = hal.i2c_mgr->get_device(get_bus(), addresses[i]);
        if (!_dev) {
            continue;
        }
        _dev->get_semaphore()->take_blocking();

        // lots of retries during probe
        _dev->set_retries(10);

        // stop continuous average mode
        if (!_send_command(SDP3X_CONT_MEAS_STOP)) {
            _dev->get_semaphore()->give();
            continue;
        }

        // these delays are needed for reliable operation
        _dev->get_semaphore()->give();
        hal.scheduler->delay_microseconds(20000);
        _dev->get_semaphore()->take_blocking();

        // start continuous average mode
        if (!_send_command(SDP3X_CONT_MEAS_AVG_MODE)) {
            _dev->get_semaphore()->give();
            continue;
        }

        // these delays are needed for reliable operation
        _dev->get_semaphore()->give();
        hal.scheduler->delay_microseconds(20000);
        _dev->get_semaphore()->take_blocking();

        // step 3 - get scale
        uint8_t val[9];
        ret = _dev->transfer(nullptr, 0, &val[0], sizeof(val));
        if (!ret) {
            _dev->get_semaphore()->give();
            continue;
        }

        // Check the CRC
        if (!_crc(&val[0], 2, val[2]) || !_crc(&val[3], 2, val[5]) || !_crc(&val[6], 2, val[8])) {
            _dev->get_semaphore()->give();
            continue;
        }

        _scale = (((uint16_t)val[6]) << 8) | val[7];

        _dev->get_semaphore()->give();

        found = true;

        char c = 'X';
        switch (_scale) {
        case SDP3X_SCALE_PRESSURE_SDP31:
            c = '1';
            break;
        case SDP3X_SCALE_PRESSURE_SDP32:
            c = '2';
            break;
        case SDP3X_SCALE_PRESSURE_SDP33:
            c = '3';
            break;
        }
        hal.console->printf("SDP3%c: Found on bus %u address 0x%02x scale=%u\n",
                            c, get_bus(), addresses[i], _scale);
    }

    if (!found) {
        return false;
    }

    /*
      this sensor uses zero offset and skips cal
     */
    set_use_zero_offset();
    set_skip_cal();
    set_offset(0);
    
    // drop to 2 retries for runtime
    _dev->set_retries(2);

    _dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_SDP3X::_timer, void));
    return true;
}

// read the values from the sensor. Called at 50Hz
void AP_Airspeed_SDP3X::_timer()
{
    // read 6 bytes from the sensor
    uint8_t val[6];
    int ret = _dev->transfer(nullptr, 0, &val[0], sizeof(val));
    uint32_t now = AP_HAL::millis();
    if (!ret) {
        if (now - _last_sample_time_ms > 200) {
            // try and re-connect
            _send_command(SDP3X_CONT_MEAS_AVG_MODE);
        }
        return;
    }
    // Check the CRC
    if (!_crc(&val[0], 2, val[2]) || !_crc(&val[3], 2, val[5])) {
        return;
    }

    int16_t P = (((int16_t)val[0]) << 8) | val[1];
    int16_t temp = (((int16_t)val[3]) << 8) | val[4];

    float diff_press_pa = float(P) / float(_scale);
    float temperature = float(temp) / SDP3X_SCALE_TEMPERATURE;

    WITH_SEMAPHORE(sem);

    _press_sum += diff_press_pa;
    _temp_sum += temperature;
    _press_count++;
    _temp_count++;
    _last_sample_time_ms = now;
}

/*
  correct pressure for barometric height
  With thanks to:
   https://github.com/PX4/Firmware/blob/master/Tools/models/sdp3x_pitot_model.py
 */
float AP_Airspeed_SDP3X::_correct_pressure(float press)
{
    float sign = 1.0f;
    
    // fix for tube order
    AP_Airspeed::pitot_tube_order tube_order = get_tube_order();
    switch (tube_order) {
    case AP_Airspeed::PITOT_TUBE_ORDER_NEGATIVE:
        press = -press;
        sign = -1.0f;
        break;
    case AP_Airspeed::PITOT_TUBE_ORDER_POSITIVE:
        break;
    case AP_Airspeed::PITOT_TUBE_ORDER_AUTO:
    default:
        if (press < 0.0f) {
            sign = -1.0f;
            press = -press;
        }
        break;
    }

    if (press <= 0.0f) {
        return 0.0f;
    }

    AP_Baro *baro = AP_Baro::get_singleton();

    if (baro == nullptr) {
        return press;
    }

    float temperature;
    if (!get_temperature(temperature)) {
        return press;
    }

    float rho_air = baro->get_pressure() / (ISA_GAS_CONSTANT * (temperature + C_TO_KELVIN));

    /*
      the constants in the code below come from a calibrated test of
      the drotek pitot tube by Sensiron. They are specific to the droktek pitot tube 

      At 25m/s, the rough proportions of each pressure correction are:

       - dp_pitot: 5%
       - press_correction: 14%
       - press: 81%

       dp_tube has been removed from the Sensiron model as it is
       insignificant (less than 0.02% over the supported speed ranges)
     */
    
    // flow through sensor
    float flow_SDP3X = (300.805f - 300.878f / (0.00344205f * (float)powf(press, 0.68698f) + 1.0f)) * 1.29f / rho_air;
    if (flow_SDP3X < 0.0f) {
        flow_SDP3X = 0.0f;
    }

    // diffential pressure through pitot tube
    float dp_pitot = 28557670.0f * (1.0f - 1.0f / (1.0f + (float)powf((flow_SDP3X / 5027611.0f), 1.227924f)));

    // uncorrected pressure
    float press_uncorrected = (press + dp_pitot) / SSL_AIR_DENSITY;

    // correction for speed at pitot-tube tip due to flow through sensor
    float dv = 0.0331582f * flow_SDP3X;

    // airspeed ratio
    float ratio = get_airspeed_ratio();

    // calculate equivalent pressure correction. This formula comes
    // from turning the dv correction above into an equivalent
    // pressure correction. We need to do this so the airspeed ratio
    // calibrator can work, as it operates on pressure values
    float press_correction = sq(sqrtf(press_uncorrected*ratio)+dv)/ratio - press_uncorrected;

    return (press_uncorrected + press_correction) * sign;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_SDP3X::get_differential_pressure(float &pressure)
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

    pressure = _correct_pressure(_press);
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_SDP3X::get_temperature(float &temperature)
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

/*
  check CRC for a set of bytes
 */
bool AP_Airspeed_SDP3X::_crc(const uint8_t data[], unsigned size, uint8_t checksum)
{
    uint8_t crc_value = 0xff;

    // calculate 8-bit checksum with polynomial 0x31 (x^8 + x^5 + x^4 + 1)
    for (uint8_t i = 0; i < size; i++) {
        crc_value ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc_value & 0x80) {
                crc_value = (crc_value << 1) ^ 0x31;
            } else {
                crc_value = (crc_value << 1);
            }
        }
    }
    // verify checksum
    return (crc_value == checksum);
}
