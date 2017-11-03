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

#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C             1.225f                  /* kg/m^3               */
#define CONSTANTS_AIR_GAS_CONST                         287.1f                  /* J/(kg * K)           */
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS                 -273.15f                /* °C                   */

extern const AP_HAL::HAL &hal;

AP_Airspeed_SDP3X::AP_Airspeed_SDP3X(AP_Airspeed &_frontend) :
    AP_Airspeed_Backend(_frontend)
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
        if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            continue;
        }

        // lots of retries during probe
        _dev->set_retries(10);

        // stop any precending measurements
        if (!_send_command(SDP3X_CONT_MEAS_STOP)) {
            _dev->get_semaphore()->give();
            continue;
        }

        // start continuous average mode
        if (!_send_command(SDP3X_CONT_MEAS_AVG_MODE)) {
            _dev->get_semaphore()->give();
            continue;
        }

        hal.scheduler->delay_microseconds(20000);

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
    set_allow_zero_offset();
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

    float P_float = (float)P;
    float temp_float = (float)temp;

    float scale_float = (float)_scale;

    float diff_press_pa = P_float / scale_float;
    float temperature = temp_float / (float)SDP3X_SCALE_TEMPERATURE;

    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _press_sum += diff_press_pa;
        _temp_sum += temperature;
        _press_count++;
        _temp_count++;
        _last_sample_time_ms = now;
        sem->give();
    }
}

/*
  correct pressure for barometric height
  With thanks to:
   https://github.com/PX4/Firmware/blob/master/Tools/models/sdp3x_pitot_model.py
 */
float AP_Airspeed_SDP3X::_correct_pressure(float press)
{
    const float tube_len = 0.2;
    float temperature;
    AP_Baro *baro = AP_Baro::get_instance();

    if (baro == nullptr) {
        return press;
    }

    // fix for tube order
    AP_Airspeed::pitot_tube_order tube_order = get_tube_order();
    switch (tube_order) {
    case AP_Airspeed::PITOT_TUBE_ORDER_NEGATIVE:
        press = -press;
    //FALLTHROUGH;
    case AP_Airspeed::PITOT_TUBE_ORDER_POSITIVE:
        break;
    case AP_Airspeed::PITOT_TUBE_ORDER_AUTO:
    default:
        press = fabsf(press);
        break;
    }
    if (press <= 0) {
        return 0;
    }

    get_temperature(temperature);
    float rho_air = baro->get_pressure() / (CONSTANTS_AIR_GAS_CONST * (temperature - CONSTANTS_ABSOLUTE_NULL_CELSIUS));

    // flow through sensor
    float flow_SDP3X = (300.805f - 300.878f / (0.00344205f * (float)powf(press, 0.68698f) + 1)) * 1.29f / rho_air;
    if (flow_SDP3X < 0.0f) {
        flow_SDP3X = 0.0f;
    }

    // diffential pressure through pitot tube
    float dp_pitot = 28557670.0f - 28557670.0f / (1 + (float)powf((flow_SDP3X / 5027611.0f), 1.227924f));

    // pressure drop through tube
    float dp_tube = flow_SDP3X * 0.000746124f * tube_len * rho_air;

    // uncorrected pressure
    float press_uncorrected = (press + dp_tube + dp_pitot) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C;

    // correction for speed at pitot-tube tip due to flow through sensor
    float dv = 0.0331582 * flow_SDP3X;

    // airspeed ratio
    float ratio = get_airspeed_ratio();

    // calculate equivalent pressure correction
    float press_correction = sq(sqrtf(press_uncorrected*ratio)+dv)/ratio - press_uncorrected;

    return press_uncorrected + press_correction;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_SDP3X::get_differential_pressure(float &pressure)
{
    uint32_t now = AP_HAL::millis();
    if (now - _last_sample_time_ms > 100) {
        return false;
    }
    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_press_count > 0) {
            _press = _press_sum / _press_count;
            _press_count = 0;
            _press_sum = 0;
        }
        sem->give();
    }
    pressure = _correct_pressure(_press);
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_SDP3X::get_temperature(float &temperature)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_temp_count > 0) {
            _temp = _temp_sum / _temp_count;
            _temp_count = 0;
            _temp_sum = 0;
        }
        sem->give();
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
