#pragma once

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
  Estimate of each rotor's shaft speed, for the "rpm" key sent to SITL.

  Webots cannot report a Propeller's actual shaft speed: Propeller.device takes
  a single RotationalMotor, so no PositionSensor can sit beside it, and
  wb_motor_get_velocity() returns the setpoint, not the measured speed.

  The rotor does lag its setpoint, and measuring it in Webots R2025a (thrust
  on a free body in zero gravity, basicTimeStep 1) shows how: the shaft slews
  towards the setpoint at a constant maxTorque rad/s^2, as if it had a unit
  moment of inertia, and holds it exactly once there.  maxTorque 5000 reaches
  hover speed in about 0.054 s; small changes around hover are tracked within
  a step or two.  So model it as a rate limit rather than a lag, which matches
  the measured speed to within 1 rpm for maxTorque 90, 1000 and 5000.
*/

/* advance each speed estimate by dt seconds towards its setpoint, slewing at
   no more than accel_max[i] rad/s^2 */
static inline void rotor_speed_update(double *speed, const double *setpoint,
                                      const double *accel_max, int count,
                                      double dt)
{
  for (int i = 0; i < count; ++i) {
    const double step = accel_max[i] * dt;
    const double diff = setpoint[i] - speed[i];
    speed[i] += (diff > step) ? step : ((diff < -step) ? -step : diff);
  }
}

/*
  Format the speed estimates as the "rpm" JSON fragment for SIM_Webots.cpp,
  which hands them to AP_RPM's SITL backend (RPM1_TYPE 10) and ESC telemetry.
  Both index rotors by SITL servo channel, so entry k is the rotor driven by
  SERVO(k+1): speed[i] goes to entry servo_channel[i], and a channel that
  drives no rotor reads 0.
*/
static inline void rotor_speed_format_rpm(char *buf, size_t buflen,
                                          const double *speed,
                                          const int *servo_channel, int count,
                                          int channels)
{
  const double rad_s_to_rpm = 60.0 / (2.0 * M_PI);
  size_t n = (size_t)snprintf(buf, buflen, ",\"rpm\": [");

  for (int ch = 0; ch < channels && n < buflen; ++ch) {
    double rpm = 0.0;
    for (int i = 0; i < count; ++i) {
      if (servo_channel[i] == ch) {
        rpm = speed[i] * rad_s_to_rpm;
      }
    }
    n += (size_t)snprintf(buf + n, buflen - n, "%s%.1f", ch ? ", " : "", rpm);
  }
  if (n < buflen) {
    snprintf(buf + n, buflen - n, "]");
  }
}
