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
  base class for CAN simulated devices
*/
#include "SIM_DroneCANDevice.h"
#if AP_TEST_DRONECAN_DRIVERS

#include <canard/publisher.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Baro/AP_Baro_SITL.h>
#include <dronecan_msgs.h>
#include <SITL/SITL.h>
#include <AP_DroneCAN/AP_Canard_iface.h>


using namespace SITL;

void DroneCANDevice::update_baro() {
    const uint64_t now = AP_HAL::micros64();
    if (((now - _baro_last_update_us) < 10000) && (_baro_last_update_us != 0)) {
        return;
    }
    _baro_last_update_us = now;
    const uint32_t now_ms = AP_HAL::millis();
    float sim_alt = AP::sitl()->state.altitude;

    if (AP::sitl()->baro_count < 1) {
        // barometer is disabled
        return;
    }

    sim_alt += AP::sitl()->baro[0].drift * now_ms * 0.001f;
    sim_alt += AP::sitl()->baro[0].noise * rand_float();


    // add baro glitch
    sim_alt += AP::sitl()->baro[0].glitch;

    // add delay
    uint32_t best_time_delta = 200;  // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index = 0;  // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now_ms - _last_store_time >= 10) {  // store data every 10 ms.
        _last_store_time = now_ms;
        if (_store_index > _buffer_length - 1) {  // reset buffer index if index greater than size of buffer
            _store_index = 0;
        }

        // if freezed barometer, report altitude to last recorded altitude
        if (AP::sitl()->baro[0].freeze == 1) {
            sim_alt = _last_altitude;
        } else {
            _last_altitude = sim_alt;
        }

        _buffer[_store_index].data = sim_alt;  // add data to current index
        _buffer[_store_index].time = _last_store_time;  // add time_stamp to current index
        _store_index = _store_index + 1;  // increment index
    }

    // return delayed measurement
    const uint32_t delayed_time = now_ms - AP::sitl()->baro[0].delay;  // get time corresponding to delay

    // find data corresponding to delayed time in buffer
    for (uint8_t i = 0; i <= _buffer_length - 1; i++) {
        // find difference between delayed time and time stamp in buffer
        uint32_t time_delta = abs(
                (int32_t)(delayed_time - _buffer[i].time));
        // if this difference is smaller than last delta, store this time
        if (time_delta < best_time_delta) {
            best_index = i;
            best_time_delta = time_delta;
        }
    }
    if (best_time_delta < 200) {  // only output stored state if < 200 msec retrieval error
        sim_alt = _buffer[best_index].data;
    }

#if !APM_BUILD_TYPE(APM_BUILD_ArduSub)
    float sigma, delta, theta;

    AP_Baro::SimpleAtmosphere(sim_alt * 0.001f, sigma, delta, theta);
    float p = SSL_AIR_PRESSURE * delta;
    float T = KELVIN_TO_C(SSL_AIR_TEMPERATURE * theta);

    AP_Baro_SITL::temperature_adjustment(p, T);
    T = C_TO_KELVIN(T);
#else
    float rho, delta, theta;
    AP_Baro::SimpleUnderWaterAtmosphere(-sim_alt * 0.001f, rho, delta, theta);
    float p = SSL_AIR_PRESSURE * delta;
    float T = SSL_AIR_TEMPERATURE * theta;
#endif

    // add in correction for wind effects
    p += AP_Baro_SITL::wind_pressure_correction(0);
    static Canard::Publisher<uavcan_equipment_air_data_StaticPressure> press_pub{CanardInterface::get_test_iface()};
    static Canard::Publisher<uavcan_equipment_air_data_StaticTemperature> temp_pub{CanardInterface::get_test_iface()};
    uavcan_equipment_air_data_StaticPressure press_msg {};
    press_msg.static_pressure = p;
    press_pub.broadcast(press_msg);
    uavcan_equipment_air_data_StaticTemperature temp_msg {};
    temp_msg.static_temperature = T;
    temp_pub.broadcast(temp_msg);
}

void DroneCANDevice::update_airspeed() {
    const uint32_t now = AP_HAL::micros64();
    if ((now - _airspeed_last_update_us < 50000) && (_airspeed_last_update_us != 0)) {
        return;
    }
    _airspeed_last_update_us = now;
    uavcan_equipment_air_data_RawAirData msg {};
    msg.differential_pressure = AP::sitl()->state.airspeed_raw_pressure[0];

    // this was mostly swiped from SIM_Airspeed_DLVR:
    const float sim_alt = AP::sitl()->state.altitude;

    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(sim_alt * 0.001f, sigma, delta, theta);

    // To Do: Add a sensor board temperature offset parameter
    msg.static_air_temperature = SSL_AIR_TEMPERATURE * theta + 25.0;

    static Canard::Publisher<uavcan_equipment_air_data_RawAirData> raw_air_pub{CanardInterface::get_test_iface()};
    raw_air_pub.broadcast(msg);
}

void DroneCANDevice::_setup_eliptical_correcion(uint8_t i)
{
    Vector3f diag = AP::sitl()->mag_diag[i].get();
    if (diag.is_zero()) {
        diag = {1,1,1};
    }
    const Vector3f &diagonals = diag;
    const Vector3f &offdiagonals = AP::sitl()->mag_offdiag[i];
    
    if (diagonals == _last_dia && offdiagonals == _last_odi) {
        return;
    }
    
    _eliptical_corr = Matrix3f(diagonals.x,    offdiagonals.x, offdiagonals.y,
                               offdiagonals.x, diagonals.y,    offdiagonals.z,
                               offdiagonals.y, offdiagonals.z, diagonals.z);
    if (!_eliptical_corr.invert()) {
        _eliptical_corr.identity();
    }
    _last_dia = diag;
    _last_odi = offdiagonals;
}

void DroneCANDevice::update_compass() {

    // Sampled at 100Hz
    const uint32_t now = AP_HAL::micros64();
    if ((now - _compass_last_update_us < 10000) && (_compass_last_update_us != 0)) {
        return;
    }
    _compass_last_update_us = now;

    // calculate sensor noise and add to 'truth' field in body frame
    // units are milli-Gauss
    Vector3f noise = rand_vec3f() * AP::sitl()->mag_noise;
    Vector3f new_mag_data = AP::sitl()->state.bodyMagField + noise;

    _setup_eliptical_correcion(0);
    Vector3f f = (_eliptical_corr * new_mag_data) - AP::sitl()->mag_ofs[0].get();
    // rotate compass
    f.rotate_inverse((enum Rotation)AP::sitl()->mag_orient[0].get());
    f.rotate(AP::compass().get_board_orientation());
    // scale the compass to simulate sensor scale factor errors
    f *= AP::sitl()->mag_scaling[0];

    static Canard::Publisher<uavcan_equipment_ahrs_MagneticFieldStrength> mag_pub{CanardInterface::get_test_iface()};
    uavcan_equipment_ahrs_MagneticFieldStrength mag_msg {};
    mag_msg.magnetic_field_ga[0] = f.x/1000.0f;
    mag_msg.magnetic_field_ga[1] = f.y/1000.0f;
    mag_msg.magnetic_field_ga[2] = f.z/1000.0f;
    mag_msg.magnetic_field_covariance.len = 0;
    mag_pub.broadcast(mag_msg);
    static Canard::Publisher<uavcan_equipment_ahrs_MagneticFieldStrength2> mag2_pub{CanardInterface::get_test_iface()};
    uavcan_equipment_ahrs_MagneticFieldStrength2 mag2_msg;
    mag2_msg.magnetic_field_ga[0] = f.x/1000.0f;
    mag2_msg.magnetic_field_ga[1] = f.y/1000.0f;
    mag2_msg.magnetic_field_ga[2] = f.z/1000.0f;
    mag2_msg.sensor_id = 0;
    mag2_msg.magnetic_field_covariance.len = 0;
    mag2_pub.broadcast(mag2_msg);
}

void DroneCANDevice::update_rangefinder() {

    // Sampled at 100Hz
    const uint32_t now = AP_HAL::micros64();
    if ((now - _rangefinder_last_update_us < 10000) && (_rangefinder_last_update_us != 0)) {
        return;
    }
    _rangefinder_last_update_us = now;
    static Canard::Publisher<uavcan_equipment_range_sensor_Measurement> pub{CanardInterface::get_test_iface()};
    uavcan_equipment_range_sensor_Measurement msg;
    msg.timestamp.usec = AP_HAL::micros64();
    msg.sensor_id = 0;
    msg.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
    const float dist = AP::sitl()->get_rangefinder(0);
    if (!isnan(dist)) {
        msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
        msg.range = MAX(0, dist);
    } else {
        msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
        msg.range = 0;
    }
    pub.broadcast(msg);
}

void DroneCANDevice::update()
{
    update_baro();
    update_airspeed();
    update_compass();
    update_rangefinder();
}

#endif // AP_TEST_DRONECAN_DRIVERS
