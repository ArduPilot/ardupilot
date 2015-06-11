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
  gimbal simulator class for MAVLink gimbal
*/

#include "SIM_Aircraft.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "SIM_Gimbal.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

Gimbal::Gimbal(const struct sitl_fdm &_fdm) :
    fdm(_fdm),
    target_address("127.0.0.1"),
    target_port(5762),
    lower_joint_limits(radians(-40), radians(-135), radians(-7.5)),
    upper_joint_limits(radians(40),  radians(45),   radians(7.5)),
    travelLimitGain(20),
    reporting_period_ms(10),
    seen_heartbeat(false),
    seen_gimbal_control(false),
    mav_socket(false)
{
    memset(&mavlink, 0, sizeof(mavlink));
    dcm.from_euler(radians(fdm.rollDeg), radians(fdm.pitchDeg), radians(fdm.yawDeg));
}


/*
  update the gimbal state
*/
void Gimbal::update(void)
{
    // calculate delta time in seconds
    uint32_t now_us = hal.scheduler->micros();

    float delta_t = (now_us - last_update_us) * 1.0e-6f;
    last_update_us = now_us;

    Matrix3f vehicle_dcm;
    vehicle_dcm.from_euler(radians(fdm.rollDeg), radians(fdm.pitchDeg), radians(fdm.yawDeg));

    Vector3f vehicle_gyro = Vector3f(radians(fdm.rollRate), 
                                     radians(fdm.pitchRate), 
                                     radians(fdm.yawRate));
    Vector3f vehicle_accel_body = Vector3f(fdm.xAccel, fdm.yAccel, fdm.zAccel);

    // take a copy of the demanded rates to bypass the limiter function for testing
    Vector3f demRateRaw = demanded_angular_rate;

    // 1)  Rotate the copters rotation rates into the gimbals frame of reference
    // copterAngRate_G = transpose(DCMgimbal)*DCMcopter*copterAngRate
    Vector3f copterAngRate_G = dcm.transposed()*vehicle_dcm*vehicle_gyro;

    // 2) Subtract the copters body rates to obtain a copter relative rotational
    // rate vector (X,Y,Z) in gimbal sensor frame
    // relativeGimbalRate(X,Y,Z) = gimbalRateDemand - copterAngRate_G
    Vector3f relativeGimbalRate = demanded_angular_rate - copterAngRate_G;

    // calculate joint angles (euler312 order)
    // calculate copter -> gimbal rotation matrix
    Matrix3f rotmat_copter_gimbal = dcm.transposed() * vehicle_dcm;

    joint_angles = rotmat_copter_gimbal.transposed().to_euler312();
        
    /* 4)  For each of the three joints, calculate upper and lower rate limits
       from the corresponding angle limits and current joint angles

       upperRatelimit = (jointAngle - lowerAngleLimit) * travelLimitGain
       lowerRatelimit = (jointAngle - upperAngleLimit) * travelLimitGain

       travelLimitGain is equal to the inverse of the bump stop time constant and
       should be set to something like 20 initially. If set too high it can cause
       the rates to 'ring' when they the limiter is in force, particularly given
       we are using a first order numerical integration.
    */
    Vector3f upperRatelimit = -(joint_angles - upper_joint_limits) * travelLimitGain;
    Vector3f lowerRatelimit = -(joint_angles - lower_joint_limits) * travelLimitGain;

    /*
      5) Calculate the gimbal joint rates (roll, elevation, azimuth)

      gimbalJointRates(roll, elev, azimuth) = Matrix*relativeGimbalRate(X,Y,Z)

      where matrix =
      +-                                                                  -+
      |          cos(elevAngle),        0,         sin(elevAngle)          |
      |                                                                    |
      |  sin(elevAngle) tan(rollAngle), 1, -cos(elevAngle) tan(rollAngle)  |
      |                                                                    |
      |           sin(elevAngle)                   cos(elevAngle)          |
      |         - --------------,       0,         --------------          |
      |           cos(rollAngle)                   cos(rollAngle)          |
      +-                                                                  -+
    */
    float rollAngle = joint_angles.x;
    float elevAngle = joint_angles.y;
    Matrix3f matrix = Matrix3f(Vector3f(cosf(elevAngle),                  0,  sinf(elevAngle)),
                               Vector3f(sinf(elevAngle)*tanf(rollAngle),  1, -cosf(elevAngle)*tanf(rollAngle)),
                               Vector3f(-sinf(elevAngle)/cosf(rollAngle), 0,  cosf(elevAngle)/cosf(rollAngle)));
    Vector3f gimbalJointRates = matrix * relativeGimbalRate;

    // 6) Apply the rate limits from 4)
    gimbalJointRates.x = constrain_float(gimbalJointRates.x, lowerRatelimit.x, upperRatelimit.x);
    gimbalJointRates.y = constrain_float(gimbalJointRates.y, lowerRatelimit.y, upperRatelimit.y);
    gimbalJointRates.z = constrain_float(gimbalJointRates.z, lowerRatelimit.z, upperRatelimit.z);
    /*
      7) Convert the modified gimbal joint rates to body rates (still copter
      relative)
      relativeGimbalRate(X,Y,Z) = Matrix * gimbalJointRates(roll, elev, azimuth)

      where Matrix =

      +-                                                   -+
      |  cos(elevAngle), 0, -cos(rollAngle) sin(elevAngle)  |
      |                                                     |
      |         0,       1,         sin(rollAngle)          |
      |                                                     |
      |  sin(elevAngle), 0,  cos(elevAngle) cos(rollAngle)  |
      +-                                                   -+
    */
    matrix = Matrix3f(Vector3f(cosf(elevAngle), 0, -cosf(rollAngle)*sinf(elevAngle)),
                      Vector3f(0,               1,  sinf(rollAngle)),
                      Vector3f(sinf(elevAngle), 0,  cosf(elevAngle)*cosf(rollAngle)));
    relativeGimbalRate = matrix * gimbalJointRates;

    // 8) Add to the result from step 1) to obtain the demanded gimbal body rates
    //    in an inertial frame of reference
    // demandedGimbalRatesInertial(X,Y,Z)  = relativeGimbalRate(X,Y,Z) + copterAngRate_G
    // Vector3f demandedGimbalRatesInertial = relativeGimbalRate + copterAngRate_G;
            
    // for the moment we will set gyros equal to demanded_angular_rate
    gimbal_angular_rate = demRateRaw; // demandedGimbalRatesInertial + true_gyro_bias - supplied_gyro_bias

    // update rotation of the gimbal
    dcm.rotate(gimbal_angular_rate*delta_t);
    dcm.normalize();

    // calculate copter -> gimbal rotation matrix
    rotmat_copter_gimbal = dcm.transposed() * vehicle_dcm;

    // calculate joint angles (euler312 order)
    joint_angles = rotmat_copter_gimbal.transposed().to_euler312();

    // update observed gyro
    gyro = gimbal_angular_rate + true_gyro_bias;

    // update delta_angle (integrate)
    delta_angle += gyro * delta_t;

    // calculate accel in gimbal body frame
    Vector3f copter_accel_earth = vehicle_dcm * vehicle_accel_body;
    Vector3f accel = dcm.transposed() * copter_accel_earth;

    // integrate velocity
    delta_velocity += accel * delta_t;

    // see if we should do a report
    send_report();
}

/*
  send a report to the vehicle control code over MAVLink
*/
void Gimbal::send_report(void)
{
    if (!mavlink.connected && mav_socket.connect(target_address, target_port)) {
        ::printf("Gimbal connected to %s:%u\n", target_address, (unsigned)target_port);
        mavlink.connected = true;
    }
    if (!mavlink.connected) {
        return;
    }

    // check for incoming MAVLink messages
    uint8_t buf[100];
    ssize_t ret;

    while ((ret=mav_socket.recv(buf, sizeof(buf), 0)) > 0) {
        for (uint8_t i=0; i<ret; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status,
                                          buf[i], 
                                          &msg, &status) == MAVLINK_FRAMING_OK) {
                switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    if (!seen_heartbeat) {
                        seen_heartbeat = true;
                        vehicle_component_id = msg.compid;
                        vehicle_system_id = msg.sysid;
                        ::printf("Gimbal using srcSystem %u\n", (unsigned)vehicle_system_id);
                    }
                    break;
                }
                case MAVLINK_MSG_ID_GIMBAL_CONTROL: {
                    mavlink_gimbal_control_t pkt;
                    mavlink_msg_gimbal_control_decode(&msg, &pkt);
                    demanded_angular_rate = Vector3f(pkt.demanded_rate_x,
                                                     pkt.demanded_rate_y,
                                                     pkt.demanded_rate_z);
                    // no longer supply a bias
                    supplied_gyro_bias.zero();
                    seen_gimbal_control = true;
                    break;
                }
                }
            }
        }
    }

    if (!seen_heartbeat) {
        return;
    }
    uint32_t now = hal.scheduler->millis();
    mavlink_message_t msg;
    uint16_t len;

    if (now - last_heartbeat_ms >= 1000) {
        mavlink_heartbeat_t heartbeat;
        heartbeat.type = MAV_TYPE_GIMBAL;
        heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
        heartbeat.base_mode = 0;
        heartbeat.system_status = 0;
        heartbeat.mavlink_version = 0;
        heartbeat.custom_mode = 0;

        /*
          save and restore sequence number for chan0, as it is used by
          generated encode functions
         */
        mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
        uint8_t saved_seq = chan0_status->current_tx_seq;
        chan0_status->current_tx_seq = mavlink.seq;
        len = mavlink_msg_heartbeat_encode(vehicle_system_id, 
                                           vehicle_component_id, 
                                           &msg, &heartbeat);
        chan0_status->current_tx_seq = saved_seq;

        mav_socket.send(&msg.magic, len);
        last_heartbeat_ms = now;
    }

    /*
      send a GIMBAL_REPORT message
     */
    uint32_t now_us = hal.scheduler->micros();
    if (now_us - last_report_us > reporting_period_ms*1000UL) {
        mavlink_gimbal_report_t gimbal_report;
        float delta_time = (now_us - last_report_us) * 1.0e-6f;
        last_report_us = now_us;
        gimbal_report.target_system = vehicle_system_id;
        gimbal_report.target_component = vehicle_component_id;
        gimbal_report.delta_time = delta_time;
        gimbal_report.delta_angle_x = delta_angle.x;
        gimbal_report.delta_angle_y = delta_angle.y;
        gimbal_report.delta_angle_z = delta_angle.z;
        gimbal_report.delta_velocity_x = delta_velocity.x;
        gimbal_report.delta_velocity_y = delta_velocity.y;
        gimbal_report.delta_velocity_z = delta_velocity.z;
        gimbal_report.joint_roll = joint_angles.x;
        gimbal_report.joint_el = joint_angles.y;
        gimbal_report.joint_az = joint_angles.z;

        mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
        uint8_t saved_seq = chan0_status->current_tx_seq;
        chan0_status->current_tx_seq = mavlink.seq;
        len = mavlink_msg_gimbal_report_encode(vehicle_system_id, 
                                               vehicle_component_id, 
                                               &msg, &gimbal_report);
        chan0_status->current_tx_seq = saved_seq;

        mav_socket.send(&msg.magic, len);
        
        delta_velocity.zero();
        delta_angle.zero();
    }
}
#endif // CONFIG_HAL_BOARD
