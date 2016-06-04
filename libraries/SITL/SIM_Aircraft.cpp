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

#include "SIM_Aircraft.h"

#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#ifdef __CYGWIN__
#include <windows.h>
#include <time.h>
#include <Mmsystem.h>
#endif

#include <DataFlash/DataFlash.h>

namespace SITL {

/*
  parent class for all simulator types
 */

Aircraft::Aircraft(const char *home_str, const char *frame_str) :
    ground_level(0),
    frame_height(0),
    dcm(),
    gyro(),
    velocity_ef(),
    mass(0),
    accel_body(0, 0, -GRAVITY_MSS),
    time_now_us(0),
    gyro_noise(radians(0.1f)),
    accel_noise(0.3),
    rate_hz(1200),
    autotest_dir(NULL),
    frame(frame_str),
#ifdef __CYGWIN__
    min_sleep_time(20000)
#else
    min_sleep_time(5000)
#endif
{
    parse_home(home_str, home, home_yaw);
    location = home;
    ground_level = home.alt*0.01;

    dcm.from_euler(0, 0, radians(home_yaw));

    set_speedup(1);

    last_wall_time_us = get_wall_time_us();
    frame_counter = 0;
}


/*
  parse a home string into a location and yaw
 */
bool Aircraft::parse_home(const char *home_str, Location &loc, float &yaw_degrees)
{
    char *saveptr=NULL;
    char *s = strdup(home_str);
    if (!s) {
        return false;
    }
    char *lat_s = strtok_r(s, ",", &saveptr);
    if (!lat_s) {
        free(s);
        return false;
    }
    char *lon_s = strtok_r(NULL, ",", &saveptr);
    if (!lon_s) {
        free(s);
        return false;
    }
    char *alt_s = strtok_r(NULL, ",", &saveptr);
    if (!alt_s) {
        free(s);
        return false;
    }
    char *yaw_s = strtok_r(NULL, ",", &saveptr);
    if (!yaw_s) {
        free(s);
        return false;
    }

    memset(&loc, 0, sizeof(loc));
    loc.lat = strtof(lat_s, NULL) * 1.0e7;
    loc.lng = strtof(lon_s, NULL) * 1.0e7;
    loc.alt = strtof(alt_s, NULL) * 1.0e2;

    yaw_degrees = strtof(yaw_s, NULL);
    free(s);

    return true;
}
    
/*
   return true if we are on the ground
*/
bool Aircraft::on_ground(const Vector3f &pos) const
{
    return (-pos.z) + home.alt*0.01f <= ground_level + frame_height;
}

/*
   update location from position
*/
void Aircraft::update_position(void)
{
    location = home;
    location_offset(location, position.x, position.y);

    location.alt  = home.alt - position.z*100.0f;

    // we only advance time if it hasn't been advanced already by the
    // backend
    if (last_time_us == time_now_us) {
        time_now_us += frame_time_us;
    }
    last_time_us = time_now_us;
    if (use_time_sync) {
        sync_frame_time();
    }

#if 0
    // logging of raw sitl data
    Vector3f accel_ef = dcm * accel_body;
    DataFlash_Class::instance()->Log_Write("SITL", "TimeUS,VN,VE,VD,AN,AE,AD,PN,PE,PD", "Qfffffffff",
                                           AP_HAL::micros64(),
                                           velocity_ef.x, velocity_ef.y, velocity_ef.z,
                                           accel_ef.x, accel_ef.y, accel_ef.z,
                                           position.x, position.y, position.z);
#endif
}

/* advance time by deltat in seconds */
void Aircraft::time_advance(float deltat)
{
    time_now_us += deltat * 1.0e6f;
}

/* setup the frame step time */
void Aircraft::setup_frame_time(float new_rate, float new_speedup)
{
    rate_hz = new_rate;
    target_speedup = new_speedup;
    frame_time_us = 1.0e6f/rate_hz;

    scaled_frame_time_us = frame_time_us/target_speedup;
    last_wall_time_us = get_wall_time_us();
    achieved_rate_hz = rate_hz;
}

/* adjust frame_time calculation */
void Aircraft::adjust_frame_time(float new_rate)
{
    if (rate_hz != new_rate) {
        rate_hz = new_rate;
        frame_time_us = 1.0e6f/rate_hz;
        scaled_frame_time_us = frame_time_us/target_speedup;
    }
}

/*
   try to synchronise simulation time with wall clock time, taking
   into account desired speedup
   This tries to take account of possible granularity of
   get_wall_time_us() so it works reasonably well on windows
*/
void Aircraft::sync_frame_time(void)
{
    frame_counter++;
    uint64_t now = get_wall_time_us();
    if (frame_counter >= 40 &&
        now > last_wall_time_us) {
        float rate = frame_counter * 1.0e6f/(now - last_wall_time_us);
        achieved_rate_hz = (0.99f*achieved_rate_hz) + (0.01f*rate);
        if (achieved_rate_hz < rate_hz * target_speedup) {
            scaled_frame_time_us *= 0.999f;
        } else {
            scaled_frame_time_us /= 0.999f;
        }
#if 0
        ::printf("achieved_rate_hz=%.3f rate=%.2f rate_hz=%.3f sft=%.1f\n",
                 (double)achieved_rate_hz,
                 (double)rate,
                 (double)rate_hz,
                 (double)scaled_frame_time_us);
#endif
        uint32_t sleep_time = scaled_frame_time_us*frame_counter;
        if (sleep_time > min_sleep_time) {
            usleep(sleep_time);
        }
        last_wall_time_us = now;
        frame_counter = 0;
    }
}

/* add noise based on throttle level (from 0..1) */
void Aircraft::add_noise(float throttle)
{
    gyro += Vector3f(rand_normal(0, 1),
                     rand_normal(0, 1),
                     rand_normal(0, 1)) * gyro_noise * fabsf(throttle);
    accel_body += Vector3f(rand_normal(0, 1),
                           rand_normal(0, 1),
                           rand_normal(0, 1)) * accel_noise * fabsf(throttle);
}

/*
  normal distribution random numbers
  See
  http://en.literateprograms.org/index.php?title=Special:DownloadCode/Box-Muller_transform_%28C%29&oldid=7011
*/
double Aircraft::rand_normal(double mean, double stddev)
{
    static double n2 = 0.0;
    static int n2_cached = 0;
    if (!n2_cached)
    {
        double x, y, r;
        do
        {
            x = 2.0*rand()/RAND_MAX - 1;
            y = 2.0*rand()/RAND_MAX - 1;

            r = x*x + y*y;
        }
        while (r == 0.0 || r > 1.0);
        {
            double d = sqrt(-2.0*log(r)/r);
            double n1 = x*d;
            n2 = y*d;
            double result = n1*stddev + mean;
            n2_cached = 1;
            return result;
        }
    }
    else
    {
        n2_cached = 0;
        return n2*stddev + mean;
    }
}




/*
   fill a sitl_fdm structure from the simulator state
*/
void Aircraft::fill_fdm(struct sitl_fdm &fdm) const
{
    fdm.timestamp_us = time_now_us;
    fdm.latitude  = location.lat * 1.0e-7;
    fdm.longitude = location.lng * 1.0e-7;
    fdm.altitude  = location.alt * 1.0e-2;
    fdm.heading   = degrees(atan2f(velocity_ef.y, velocity_ef.x));
    fdm.speedN    = velocity_ef.x;
    fdm.speedE    = velocity_ef.y;
    fdm.speedD    = velocity_ef.z;
    fdm.xAccel    = accel_body.x;
    fdm.yAccel    = accel_body.y;
    fdm.zAccel    = accel_body.z;
    fdm.rollRate  = degrees(gyro.x);
    fdm.pitchRate = degrees(gyro.y);
    fdm.yawRate   = degrees(gyro.z);
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    fdm.rollDeg  = degrees(r);
    fdm.pitchDeg = degrees(p);
    fdm.yawDeg   = degrees(y);
    fdm.airspeed = airspeed_pitot;
    fdm.battery_voltage = battery_voltage;
    fdm.battery_current = battery_current;
    fdm.rpm1 = rpm1;
    fdm.rpm2 = rpm2;
    fdm.rcin_chan_count = rcin_chan_count;
    memcpy(fdm.rcin, rcin, rcin_chan_count*sizeof(float));
}

uint64_t Aircraft::get_wall_time_us() const
{
#ifdef __CYGWIN__
    static DWORD tPrev;
    static uint64_t last_ret_us;
    if (tPrev == 0) {
        tPrev = timeGetTime();
        return 0;
    }
    DWORD now = timeGetTime();
    last_ret_us += (uint64_t)((now - tPrev)*1000UL);
    tPrev = now;
    return last_ret_us;
#else
    struct timeval tp;
    gettimeofday(&tp,NULL);
    return tp.tv_sec*1.0e6 + tp.tv_usec;
#endif
}

/*
  set simulation speedup
 */
void Aircraft::set_speedup(float speedup)
{
    setup_frame_time(rate_hz, speedup);
}

/*
  update the simulation attitude and relative position
 */
void Aircraft::update_dynamics(const Vector3f &rot_accel)
{
    float delta_time = frame_time_us * 1.0e-6f;
    
    // update rotational rates in body frame
    gyro += rot_accel * delta_time;

    gyro.x = constrain_float(gyro.x, -radians(2000), radians(2000));
    gyro.y = constrain_float(gyro.y, -radians(2000), radians(2000));
    gyro.z = constrain_float(gyro.z, -radians(2000), radians(2000));
    
    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // if we're on the ground, then our vertical acceleration is limited
    // to zero. This effectively adds the force of the ground on the aircraft
    if (on_ground(position) && accel_earth.z > 0) {
        accel_earth.z = 0;
    }

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    Vector3f old_position = position;
    position += velocity_ef * delta_time;

    // velocity relative to air mass, in earth frame
    velocity_air_ef = velocity_ef - wind_ef;
    
    // velocity relative to airmass in body frame
    velocity_air_bf = dcm.transposed() * velocity_air_ef;
    
    // airspeed 
    airspeed = velocity_air_ef.length();

    // airspeed as seen by a fwd pitot tube (limited to 120m/s)
    airspeed_pitot = constrain_float(velocity_air_bf * Vector3f(1, 0, 0), 0, 120);
    
    // constrain height to the ground
    if (on_ground(position)) {
        if (!on_ground(old_position) && AP_HAL::millis() - last_ground_contact_ms > 1000) {
            printf("Hit ground at %f m/s\n", velocity_ef.z);
            last_ground_contact_ms = AP_HAL::millis();
        }
        position.z = -(ground_level + frame_height - home.alt*0.01f);
    }
}

/*
  update wind vector
*/
void Aircraft::update_wind(const struct sitl_input &input)
{
    // wind vector in earth frame
    wind_ef = Vector3f(cosf(radians(input.wind.direction)), sinf(radians(input.wind.direction)), 0) * input.wind.speed;
}
    
} // namespace SITL
