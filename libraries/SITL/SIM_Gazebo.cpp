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
  simulator connector for ardupilot version of Gazebo
*/

#include "SIM_Gazebo.h"

#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

Gazebo::Gazebo(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    last_timestamp(0)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // Gazebo keeps sending us packets. Not strictly necessary but
    // useful for debugging
    fprintf(stdout, "Starting SITL Gazebo\n");

}

/*
  decode and send servos
*/
void Gazebo::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    // should rename servo_command
    // 16 because struct sitl_input.servos is 16 large in SIM_Aircraft.h
    for (unsigned i = 0; i < 16; ++i)
    {
      pkt.motor_speed[i] = (input.servos[i]-1000) / 1000.0f;
    }
    socket_out.send(&pkt, sizeof(pkt));
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Gazebo::recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;

    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (socket_in.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        send_servos(input);
    }

    // get imu stuff
    accel_body = Vector3f(pkt.imu_linear_acceleration_xyz[0],
                          pkt.imu_linear_acceleration_xyz[1],
                          pkt.imu_linear_acceleration_xyz[2]);

    gyro = Vector3f(pkt.imu_angular_velocity_rpy[0],
                    pkt.imu_angular_velocity_rpy[1],
                    pkt.imu_angular_velocity_rpy[2]);

    // compute dcm from imu orientation
    Quaternion quat(pkt.imu_orientation_quat[0],
                    pkt.imu_orientation_quat[1],
                    pkt.imu_orientation_quat[2],
                    pkt.imu_orientation_quat[3]);
    quat.rotation_matrix(dcm);

    double speedN =  pkt.velocity_xyz[0];
    double speedE =  pkt.velocity_xyz[1];
    double speedD =  pkt.velocity_xyz[2];
    velocity_ef = Vector3f(speedN, speedE, speedD);

    position = Vector3f(pkt.position_xyz[0],
                        pkt.position_xyz[1],
                        pkt.position_xyz[2]);


    // auto-adjust to simulation frame rate
    double deltat = pkt.timestamp - last_timestamp;
    time_now_us += deltat * 1.0e6;

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(1.0/deltat);
    }
    last_timestamp = pkt.timestamp;

    /* copied below from iris_ros.py */
    /*
    bearing = to_degrees(atan2(position.y, position.x));
    distance = math.sqrt(self.position.x**2 + self.position.y**2)
    (self.latitude, self.longitude) = util.gps_newpos(
      self.home_latitude, self.home_longitude, bearing, distance)
    self.altitude  = self.home_altitude - self.position.z
    velocity_body = self.dcm.transposed() * self.velocity
    self.accelerometer = self.accel_body.copy()
    */
}

/*
  update the Gazebo simulation by one time step
 */
void Gazebo::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    update_position();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
