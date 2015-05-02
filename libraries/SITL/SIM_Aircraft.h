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
  parent class for aircraft simulators
*/

#ifndef _SIM_AIRCRAFT_H
#define _SIM_AIRCRAFT_H

#include "SITL.h"
#include <AP_Common.h>
#include <AP_Math.h>

/*
  parent class for all simulator types
 */
class Aircraft 
{
public:
    Aircraft(const char *home_str);

    /*
      structure passed in giving servo positions as PWM values in
      microseconds
     */
    struct sitl_input {
        uint16_t servos[16];
    };

    /*
      step the FDM by one time step
     */
    virtual void update(const struct sitl_input &input) = 0;

    /* fill a sitl_fdm structure from the simulator state */
    void fill_fdm(struct sitl_fdm &fdm) const;

protected:
    Location home;
    Location location;

    float ground_level;
    float frame_height;
    Matrix3f dcm;  // rotation matrix, APM conventions, from body to earth
    Vector3f gyro; // rad/s
    Vector3f velocity_ef; // m/s, earth frame
    Vector3f velocity_body; // m/s, body frame
    Vector3f position; // meters, NED from origin
    float mass; // kg
    float update_frequency;
    Vector3f accel_body; // m/s/s NED, body frame

    uint64_t time_now_us;

    const float gyro_noise;
    const float accel_noise;
    float rate_hz;
    float achieved_rate_hz;
    float target_speedup;
    float frame_time_us;
    float scaled_frame_time_us;
    uint64_t last_wall_time_us;

    bool on_ground(const Vector3f &pos) const;

    /* update location from position */
    void update_position(void);

    /* rotate to the given yaw */
    void set_yaw_degrees(float yaw_degrees);
        
    /* advance time by deltat in seconds */
    void time_advance(float deltat);

    /* setup the frame step time */
    void setup_frame_time(float rate, float speedup);

    /* adjust frame_time calculation */
    void adjust_frame_time(float rate);

    /* try to synchronise simulation time with wall clock time, taking
       into account desired speedup */
    void sync_frame_time(void);

    /* add noise based on throttle level (from 0..1) */
    void add_noise(float throttle);

    /* return wall clock time in microseconds since 1970 */
    uint64_t get_wall_time_us(void) const;

    /* return normal distribution random numbers */
    double rand_normal(double mean, double stddev);
};

#endif // _SIM_AIRCRAFT_H

