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

    const auto gps_tow = GPS_Backend::gps_time();

    // 0x01 GPS INS Time (round)
    pkt.gnss_ins_time_ms = gps_tow.ms;

    // 0x23 Accelerometer data HR
    pkt.accel_data_hr.x = (fdm.yAccel / GRAVITY_MSS) * 1.0e6;  // g*1.0e6
    pkt.accel_data_hr.y = (fdm.xAccel / GRAVITY_MSS) * 1.0e6;  // g*1.0e6
    pkt.accel_data_hr.z = (-fdm.zAccel / GRAVITY_MSS) * 1.0e6; // g*1.0e6

    // 0x21 Gyro data HR
    pkt.gyro_data_hr.y = fdm.rollRate * 1.0e5;  // deg/s*1.0e5
    pkt.gyro_data_hr.x = fdm.pitchRate * 1.0e5; // deg/s*1.0e5
    pkt.gyro_data_hr.z = -fdm.yawRate * 1.0e5;  // deg/s*1.0e5

    // 0x25 Barometer data
    float p, t_K;
    AP_Baro::get_pressure_temperature_for_alt_amsl(fdm.altitude+rand_float()*0.25, p, t_K);

    pkt.baro_data.pressure_pa2 = p * 0.5; // Pa/2
    pkt.baro_data.baro_alt = fdm.altitude * 100; // m

    // 0x24 Magnetometer data
    pkt.mag_data.x = (fdm.bodyMagField.y / NTESLA_TO_MGAUSS) * 0.1;  // nT/10
    pkt.mag_data.y = (fdm.bodyMagField.x / NTESLA_TO_MGAUSS) * 0.1;  // nT/10
    pkt.mag_data.z = (-fdm.bodyMagField.z / NTESLA_TO_MGAUSS) * 0.1; // nT/10

    // 0x07 Orientation angles
    float roll_rad, pitch_rad, yaw_rad, heading_deg;
    fdm.quaternion.to_euler(roll_rad, pitch_rad, yaw_rad);
    heading_deg = fmodf(degrees(yaw_rad), 360.0f);
    if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    pkt.orientation_angles.roll = roll_rad * RAD_TO_DEG * 100; // deg*100
    pkt.orientation_angles.pitch = pitch_rad * RAD_TO_DEG * 100; // deg*100
    pkt.orientation_angles.yaw = heading_deg * 100; // deg*100

    // 0x12 Velocities
    pkt.velocity.x = fdm.speedE * 100;  // m/s*100
    pkt.velocity.y = fdm.speedN * 100;  // m/s*100
    pkt.velocity.z = -fdm.speedD * 100; // m/s*100

    // 0x10 Position
    pkt.position.lat = fdm.latitude * 1e7;  // deg*1.0e7
    pkt.position.lon = fdm.longitude * 1e7; // deg*1.0e7
    pkt.position.alt = fdm.altitude * 1e2;  // m*100

    // 0x58 KF velocity covariance
    pkt.kf_vel_covariance.x = 10; // mm/s
    pkt.kf_vel_covariance.y = 10; // mm/s
    pkt.kf_vel_covariance.z = 10; // mm/s

    // 0x57 KF position covariance HR
    pkt.kf_pos_covariance.x = 20; // mm
    pkt.kf_pos_covariance.y = 20; // mm
    pkt.kf_pos_covariance.z = 20; // mm

    // 0x53 Unit status word (USW)
    pkt.unit_status = 0; // INS data is valid

    // 0x28 Differential pressure
    pkt.differential_pressure = fdm.airspeed_raw_pressure[0] * 0.01 * 1.0e4; // mbar*1.0e4 (0.01 - Pa to mbar)

    // 0x86 True airspeed (TAS)
    pkt.true_airspeed = fdm.airspeed * 100; // m/s*100

    // 0x8A Wind speed
    pkt.wind_speed.x = fdm.wind_ef.y * 100;
    pkt.wind_speed.y = fdm.wind_ef.x * 100;
    pkt.wind_speed.z = 0;

    // 0x8D ADU status
    pkt.air_data_status = 0; // ADU data is valid

    // 0x50 Supply voltage
    pkt.supply_voltage = 12.3 * 100; // VDC*100

    // 0x52 Temperature
    pkt.temperature = KELVIN_TO_C(t_K)*10; // degC

    // 0x5A Unit status word (USW2)
    pkt.unit_status2 = 0; // INS data is valid

    // 0x54 INS (Navigation) solution status
    pkt.ins_sol_status = 0; // INS solution is good

    pkt.gnss_new_data = 0;
    if (packets_sent % gnss_frequency == 0) {
        // 0x3C GPS week
        pkt.gnss_week = gps_tow.week;

        // 0x4A GNSS extended info
        pkt.gnss_extended_info.fix_type = 2; // 3D fix
        pkt.gnss_extended_info.spoofing_status = 1; // no spoofing indicated

        // 0x3B Number of satellites used in solution
        pkt.num_sats = 32;

        // 0x30 GNSS Position
        pkt.gnss_position.lat = fdm.latitude * 1e7;  // deg*1.0e7
        pkt.gnss_position.lon = fdm.longitude * 1e7; // deg*1.0e7
        pkt.gnss_position.alt = fdm.altitude * 1e2;  // m*100

        // 0x32 GNSS Velocity, Track over ground
        Vector2d track{fdm.speedN,fdm.speedE};
        pkt.gnss_vel_track.hor_speed = norm(fdm.speedN,fdm.speedE) * 100; // m/s*100
        pkt.gnss_vel_track.track_over_ground = wrap_360(degrees(track.angle())) * 100; // deg*100
        pkt.gnss_vel_track.ver_speed = -fdm.speedD * 100; // m/s*100

        // 0x3E GNSS Position timestamp
        pkt.gnss_pos_timestamp = gps_tow.ms;

        // 0x36 GNSS info short
        pkt.gnss_info_short.info1 = 0; // 0 – Single point position
        pkt.gnss_info_short.info2 = 12; // bit 2 and 3 are set (Time is fine set and is being steered)

        // 0x41 New GPS
        pkt.gnss_new_data = 3; // GNSS position and velocity data update

        // 0xС0 u-blox jamming status
        pkt.gnss_jam_status = 1; // ok (no significant jamming)

        // 0x33 GNSS Heading, GNSS Pitch
        pkt.gnss_angles.heading = 0; // only for dual-antenna GNSS receiver
        pkt.gnss_angles.pitch = 0; // only for dual-antenna GNSS receiver

        // 0x3A GNSS Angles position type
        pkt.gnss_angle_pos_type = 0; // only for dual-antenna GNSS receiver

        // 0x40 GNSS Heading timestamp
        pkt.gnss_heading_timestamp = 0; // only for dual-antenna GNSS receiver

        // 0x42 Dilution of precision
        pkt.gnss_dop.gdop = 1000; // *1.0e3
        pkt.gnss_dop.pdop = 1000; // *1.0e3
        pkt.gnss_dop.hdop = 1000; // *1.0e3
        pkt.gnss_dop.vdop = 1000; // *1.0e3
        pkt.gnss_dop.tdop = 1000; // *1.0e3
    }

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
