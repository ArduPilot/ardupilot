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
  gimbal simulator class for gimbal
*/

#include "SIM_Gimbal.h"

#if AP_SIM_GIMBAL_ENABLED

#include <stdio.h>

#include "SIM_Aircraft.h"

extern const AP_HAL::HAL& hal;

#define GIMBAL_DEBUG 0

#if GIMBAL_DEBUG
#define debug(fmt, args...)  do { printf("GIMBAL: " fmt, ##args); } while(0)
#else
#define debug(fmt, args...)  do { } while(0)
#endif

namespace SITL {

/*
  update the gimbal state
*/
void Gimbal::update(const class Aircraft &aircraft)
{
    // calculate delta time in seconds
    uint32_t now_us = AP_HAL::micros();

    float delta_t = (now_us - last_update_us) * 1.0e-6f;
    last_update_us = now_us;

    const Matrix3f &vehicle_dcm = aircraft.get_dcm();
    if (!init_done) {
        dcm = vehicle_dcm;
    }

    const Vector3f &vehicle_gyro = AP::ins().get_gyro();
    const Vector3f &vehicle_accel_body = AP::ins().get_accel();

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
}

void Gimbal::get_deltas(Vector3f &_delta_angle, Vector3f &_delta_velocity, uint32_t &_delta_time_us)
{
    const uint32_t now_us = AP_HAL::micros();

    _delta_angle = delta_angle;
    _delta_velocity = delta_velocity;
    _delta_time_us = now_us - delta_start_us;

    delta_angle.zero();
    delta_velocity.zero();
    delta_start_us = now_us;
}

} // namespace SITL

#endif  // AP_SIM_GIMBAL_ENABLED
