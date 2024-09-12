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
  simulate InertialLabs external AHRS
*/
#include "SIM_InertialLabs.h"
#include <GCS_MAVLink/GCS.h>
#include "SIM_GPS.h"

using namespace SITL;

InertialLabs::InertialLabs() : SerialDevice::SerialDevice()
{
}

void InertialLabs::send_packet(void)
{
    const auto &fdm = _sitl->state;

    pkt.msg_len = sizeof(pkt)-2;

    pkt.accel_data_hr.x = (fdm.yAccel * 1.0e6)/GRAVITY_MSS;
    pkt.accel_data_hr.y = (fdm.xAccel * 1.0e6)/GRAVITY_MSS;
    pkt.accel_data_hr.z = (-fdm.zAccel * 1.0e6)/GRAVITY_MSS;

    pkt.gyro_data_hr.y = fdm.rollRate*1.0e5;
    pkt.gyro_data_hr.x = fdm.pitchRate*1.0e5;
    pkt.gyro_data_hr.z = -fdm.yawRate*1.0e5;

    float p, t_K;
    AP_Baro::get_pressure_temperature_for_alt_amsl(fdm.altitude+rand_float()*0.25, p, t_K);

    pkt.baro_data.pressure_pa2 = p;
    pkt.baro_data.baro_alt = fdm.altitude;
    pkt.temperature = KELVIN_TO_C(t_K);

    pkt.mag_data.x = (fdm.bodyMagField.y / NTESLA_TO_MGAUSS)*0.1;
    pkt.mag_data.y = (fdm.bodyMagField.x / NTESLA_TO_MGAUSS)*0.1;
    pkt.mag_data.z = (-fdm.bodyMagField.z / NTESLA_TO_MGAUSS)*0.1;

    pkt.orientation_angles.roll = fdm.rollDeg*100;
    pkt.orientation_angles.pitch = fdm.pitchDeg*100;
    pkt.orientation_angles.yaw = fdm.yawDeg*100;

    pkt.velocity.x = fdm.speedE*100;
    pkt.velocity.y = fdm.speedN*100;
    pkt.velocity.z = -fdm.speedD*100;

    pkt.position.lat = fdm.latitude*1e7;
    pkt.position.lon = fdm.longitude*1e7;
    pkt.position.alt = fdm.altitude*1e2;

    pkt.kf_vel_covariance.x = 10;
    pkt.kf_vel_covariance.z = 10;
    pkt.kf_vel_covariance.z = 10;

    pkt.kf_pos_covariance.x = 20;
    pkt.kf_pos_covariance.z = 20;
    pkt.kf_pos_covariance.z = 20;

    const auto gps_tow = GPS_Backend::gps_time();

    pkt.gps_ins_time_ms = gps_tow.ms;

    pkt.gnss_new_data = 0;

    if (packets_sent % gps_frequency == 0) {
        // update GPS data at 5Hz
        pkt.gps_week = gps_tow.week;
        pkt.gnss_pos_timestamp = gps_tow.ms;
        pkt.gnss_new_data = 3;
        pkt.gps_position.lat = pkt.position.lat;
        pkt.gps_position.lon = pkt.position.lon;
        pkt.gps_position.alt = pkt.position.alt;

        pkt.num_sats = 32;
        pkt.gnss_vel_track.hor_speed = norm(fdm.speedN,fdm.speedE)*100;
        Vector2d track{fdm.speedN,fdm.speedE};
        pkt.gnss_vel_track.track_over_ground = wrap_360(degrees(track.angle()))*100;
        pkt.gnss_vel_track.ver_speed = -fdm.speedD*100;

        pkt.gnss_extended_info.fix_type = 2;
    }

    pkt.differential_pressure = 0.5*sq(fdm.airspeed+fabsF(rand_float()*0.25))*1.0e4;
    pkt.supply_voltage = 12.3*100;
    pkt.temperature = 23.4*10;

    const uint8_t *buffer = (const uint8_t *)&pkt;
    pkt.crc = crc_sum_of_bytes_16(&buffer[2], sizeof(pkt)-4);

    write_to_autopilot((char *)&pkt, sizeof(pkt));

    packets_sent++;
}


/*
  send InertialLabs data
 */
void InertialLabs::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }
    const uint32_t us_between_packets = 5000; // 200Hz
    const uint32_t now = AP_HAL::micros();
    if (now - last_pkt_us >= us_between_packets) {
        last_pkt_us = now;
        send_packet();
    }

}
