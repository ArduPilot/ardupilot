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
#include <AP_Param/AP_Param.h>

namespace SITL {

/*
  parent class for all simulator types
 */

Aircraft::Aircraft(const char *home_str, const char *frame_str) :
    ground_level(0.0f),
    frame_height(0.0f),
    dcm(),
    gyro(),
    gyro_prev(),
    ang_accel(),
    velocity_ef(),
    mass(0.0f),
    accel_body(0.0f, 0.0f, -GRAVITY_MSS),
    time_now_us(0),
    gyro_noise(radians(0.1f)),
    accel_noise(0.3f),
    rate_hz(1200.0f),
    autotest_dir(nullptr),
    frame(frame_str),
#ifdef __CYGWIN__
    min_sleep_time(20000)
#else
    min_sleep_time(5000)
#endif
{
    // make the SIM_* variables available to simulator backends
    sitl = (SITL *)AP_Param::find_object("SIM_");
    parse_home(home_str, home, home_yaw);
    location = home;
    ground_level = home.alt * 0.01f;

    dcm.from_euler(0.0f, 0.0f, radians(home_yaw));

    set_speedup(1.0f);

    last_wall_time_us = get_wall_time_us();
    frame_counter = 0;

    terrain = (AP_Terrain *)AP_Param::find_object("TERRAIN_");
}


/*
  parse a home string into a location and yaw
 */
bool Aircraft::parse_home(const char *home_str, Location &loc, float &yaw_degrees)
{
    char *saveptr = nullptr;
    char *s = strdup(home_str);
    if (!s) {
        free(s);
        return false;
    }
    char *lat_s = strtok_r(s, ",", &saveptr);
    if (!lat_s) {
        free(s);
        return false;
    }
    char *lon_s = strtok_r(nullptr, ",", &saveptr);
    if (!lon_s) {
        free(s);
        return false;
    }
    char *alt_s = strtok_r(nullptr, ",", &saveptr);
    if (!alt_s) {
        free(s);
        return false;
    }
    char *yaw_s = strtok_r(nullptr, ",", &saveptr);
    if (!yaw_s) {
        free(s);
        return false;
    }

    memset(&loc, 0, sizeof(loc));
    loc.lat = static_cast<int32_t>(strtof(lat_s, nullptr) * 1.0e7f);
    loc.lng = static_cast<int32_t>(strtof(lon_s, nullptr) * 1.0e7f);
    loc.alt = static_cast<int32_t>(strtof(alt_s, nullptr) * 1.0e2f);

    yaw_degrees = strtof(yaw_s, nullptr);
    free(s);

    return true;
}

/*
   return difference in altitude between home position and current loc
*/
float Aircraft::ground_height_difference() const
{
    float h1, h2;
    if (sitl->terrain_enable && terrain &&
        terrain->height_amsl(home, h1, false) &&
        terrain->height_amsl(location, h2, false)) {
        return h2 - h1;
    }
    return 0.0f;
}

/*
   return current height above ground level (metres)
*/
float Aircraft::hagl() const
{
    return (-position.z) + home.alt * 0.01f - ground_level - frame_height - ground_height_difference();
}
/*
   return true if we are on the ground
*/
bool Aircraft::on_ground() const
{
    return hagl() <= 0;
}

/*
   update location from position
*/
void Aircraft::update_position(void)
{
    location = home;
    location_offset(location, position.x, position.y);

    location.alt  = static_cast<int32_t>(home.alt - position.z * 100.0f);

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

/*
   update body magnetic field from position and rotation
*/
void Aircraft::update_mag_field_bf()
{
    // get the magnetic field intensity and orientation
    float intensity;
    float declination;
    float inclination;
    get_mag_field_ef(location.lat * 1e-7f, location.lng * 1e-7f, intensity, declination, inclination);

    // create a field vector and rotate to the required orientation
    Vector3f mag_ef(1e3f * intensity, 0.0f, 0.0f);
    Matrix3f R;
    R.from_euler(0.0f, -ToRad(inclination), ToRad(declination));
    mag_ef = R * mag_ef;

    // calculate frame height above ground
    const float frame_height_agl = fmaxf((-position.z) + home.alt * 0.01f - ground_level, 0.0f);

    // calculate scaling factor that varies from 1 at ground level to 1/8 at sitl->mag_anomaly_hgt
    // Assume magnetic anomaly strength scales with 1/R**3
    float anomaly_scaler = (sitl->mag_anomaly_hgt / (frame_height_agl + sitl->mag_anomaly_hgt));
    anomaly_scaler = anomaly_scaler * anomaly_scaler * anomaly_scaler;

    // add scaled anomaly to earth field
    mag_ef += sitl->mag_anomaly_ned.get() * anomaly_scaler;

    // Rotate into body frame
    mag_bf = dcm.transposed() * mag_ef;

    // add motor interference
    mag_bf += sitl->mag_mot.get() * battery_current;
}

/* advance time by deltat in seconds */
void Aircraft::time_advance()
{
    // we only advance time if it hasn't been advanced already by the
    // backend
    if (last_time_us == time_now_us) {
        time_now_us += frame_time_us;
    }
    last_time_us = time_now_us;
    if (use_time_sync) {
        sync_frame_time();
    }
}

/* setup the frame step time */
void Aircraft::setup_frame_time(float new_rate, float new_speedup)
{
    rate_hz = new_rate;
    target_speedup = new_speedup;
    frame_time_us = static_cast<uint64_t>(1.0e6f/rate_hz);

    scaled_frame_time_us = frame_time_us/target_speedup;
    last_wall_time_us = get_wall_time_us();
    achieved_rate_hz = rate_hz;
}

/* adjust frame_time calculation */
void Aircraft::adjust_frame_time(float new_rate)
{
    if (rate_hz != new_rate) {
        rate_hz = new_rate;
        frame_time_us = static_cast<uint64_t>(1.0e6f/rate_hz);
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
        const float rate = frame_counter * 1.0e6f/(now - last_wall_time_us);
        achieved_rate_hz = (0.99f*achieved_rate_hz) + (0.01f * rate);
        if (achieved_rate_hz < rate_hz * target_speedup) {
            scaled_frame_time_us *= 0.999f;
        } else {
            scaled_frame_time_us /= 0.999f;
        }
#if 0
        ::printf("achieved_rate_hz=%.3f rate=%.2f rate_hz=%.3f sft=%.1f\n",
                 static_cast<double>(achieved_rate_hz),
                 static_cast<double>(rate),
                 static_cast<double>(rate_hz),
                 static_cast<double>(scaled_frame_time_us));
#endif
        const uint32_t sleep_time = static_cast<uint32_t>(scaled_frame_time_us * frame_counter);
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
    if (!n2_cached) {
        double x, y, r;
        do
        {
            x = 2.0 * rand()/RAND_MAX - 1;
            y = 2.0 * rand()/RAND_MAX - 1;
            r = x*x + y*y;
        } while (is_zero(r) || r > 1.0);
        const double d = sqrt(-2.0 * log(r)/r);
        const double n1 = x * d;
        n2 = y * d;
        const double result = n1 * stddev + mean;
        n2_cached = 1;
        return result;
    } else {
        n2_cached = 0;
        return n2 * stddev + mean;
    }
}




/*
   fill a sitl_fdm structure from the simulator state
*/
void Aircraft::fill_fdm(struct sitl_fdm &fdm)
{
    if (use_smoothing) {
        smooth_sensors();
    }
    fdm.timestamp_us = time_now_us;
    if (fdm.home.lat == 0 && fdm.home.lng == 0) {
        // initialise home
        fdm.home = home;
    }
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
    fdm.angAccel.x = degrees(ang_accel.x);
    fdm.angAccel.y = degrees(ang_accel.y);
    fdm.angAccel.z = degrees(ang_accel.z);
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    fdm.rollDeg  = degrees(r);
    fdm.pitchDeg = degrees(p);
    fdm.yawDeg   = degrees(y);
    fdm.quaternion.from_rotation_matrix(dcm);
    fdm.airspeed = airspeed_pitot;
    fdm.battery_voltage = battery_voltage;
    fdm.battery_current = battery_current;
    fdm.rpm1 = rpm1;
    fdm.rpm2 = rpm2;
    fdm.rcin_chan_count = rcin_chan_count;
    fdm.range = range;
    memcpy(fdm.rcin, rcin, rcin_chan_count * sizeof(float));
    fdm.bodyMagField = mag_bf;

    if (smoothing.enabled) {
        fdm.xAccel = smoothing.accel_body.x;
        fdm.yAccel = smoothing.accel_body.y;
        fdm.zAccel = smoothing.accel_body.z;
        fdm.rollRate  = degrees(smoothing.gyro.x);
        fdm.pitchRate = degrees(smoothing.gyro.y);
        fdm.yawRate   = degrees(smoothing.gyro.z);
        fdm.speedN    = smoothing.velocity_ef.x;
        fdm.speedE    = smoothing.velocity_ef.y;
        fdm.speedD    = smoothing.velocity_ef.z;
        fdm.latitude  = smoothing.location.lat * 1.0e-7;
        fdm.longitude = smoothing.location.lng * 1.0e-7;
        fdm.altitude  = smoothing.location.alt * 1.0e-2;
    }

    if (last_speedup != sitl->speedup && sitl->speedup > 0) {
        set_speedup(sitl->speedup);
        last_speedup = sitl->speedup;
    }
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
    gettimeofday(&tp, nullptr);
    return static_cast<uint64_t>(tp.tv_sec * 1.0e6 + tp.tv_usec);
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
    const float delta_time = frame_time_us * 1.0e-6f;

    // update rotational rates in body frame
    gyro += rot_accel * delta_time;

    gyro.x = constrain_float(gyro.x, -radians(2000.0f), radians(2000.0f));
    gyro.y = constrain_float(gyro.y, -radians(2000.0f), radians(2000.0f));
    gyro.z = constrain_float(gyro.z, -radians(2000.0f), radians(2000.0f));

    // estimate angular acceleration using a first order difference calculation
    // TODO the simulator interface should provide the angular acceleration
    ang_accel = (gyro - gyro_prev) / delta_time;
    gyro_prev = gyro;

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0.0f, 0.0f, GRAVITY_MSS);

    // if we're on the ground, then our vertical acceleration is limited
    // to zero. This effectively adds the force of the ground on the aircraft
    if (on_ground() && accel_earth.z > 0) {
        accel_earth.z = 0;
    }

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0.0f, 0.0f, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    const bool was_on_ground = on_ground();
    // new position vector
    position += velocity_ef * delta_time;

    // velocity relative to air mass, in earth frame
    velocity_air_ef = velocity_ef + wind_ef;

    // velocity relative to airmass in body frame
    velocity_air_bf = dcm.transposed() * velocity_air_ef;

    // airspeed
    airspeed = velocity_air_ef.length();

    // airspeed as seen by a fwd pitot tube (limited to 120m/s)
    airspeed_pitot = constrain_float(velocity_air_bf * Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 120.0f);

    // constrain height to the ground
    if (on_ground()) {
        if (!was_on_ground && AP_HAL::millis() - last_ground_contact_ms > 1000) {
            printf("Hit ground at %f m/s\n", velocity_ef.z);
            last_ground_contact_ms = AP_HAL::millis();
        }
        position.z = -(ground_level + frame_height - home.alt * 0.01f + ground_height_difference());

        switch (ground_behavior) {
        case GROUND_BEHAVIOR_NONE:
            break;
        case GROUND_BEHAVIOR_NO_MOVEMENT: {
            // zero roll/pitch, but keep yaw
            float r, p, y;
            dcm.to_euler(&r, &p, &y);
            dcm.from_euler(0.0f, 0.0f, y);
            // no X or Y movement
            velocity_ef.x = 0.0f;
            velocity_ef.y = 0.0f;
            if (velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
            gyro.zero();
            use_smoothing = true;
            break;
        }
        case GROUND_BEHAVIOR_FWD_ONLY: {
            // zero roll/pitch, but keep yaw
            float r, p, y;
            dcm.to_euler(&r, &p, &y);
            dcm.from_euler(0.0f, 0.0f, y);
            // only fwd movement
            Vector3f v_bf = dcm.transposed() * velocity_ef;
            v_bf.y = 0.0f;
            if (v_bf.x < 0.0f) {
                v_bf.x = 0.0f;
            }
            velocity_ef = dcm * v_bf;
            if (velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
            gyro.zero();
            use_smoothing = true;
            break;
        }
        case GROUND_BEHAVIOR_TAILSITTER: {
            // point straight up
            float r, p, y;
            dcm.to_euler(&r, &p, &y);
            dcm.from_euler(0.0f, radians(90), y);
            // no movement
            if (accel_earth.z > -1.1*GRAVITY_MSS) {
                velocity_ef.zero();
            }
            gyro.zero();
            use_smoothing = true;
            break;
        }
        }
    }
}

/*
  update wind vector
*/
void Aircraft::update_wind(const struct sitl_input &input)
{
    // wind vector in earth frame
    wind_ef = Vector3f(cosf(radians(input.wind.direction)), sinf(radians(input.wind.direction)), 0) * input.wind.speed;

    const float wind_turb = input.wind.turbulence * 10.0f;  // scale input.wind.turbulence to match standard deviation when using iir_coef=0.98
    const float iir_coef = 0.98f;  // filtering high frequencies from turbulence

    if (wind_turb > 0 && !on_ground()) {

        turbulence_azimuth = turbulence_azimuth + (2 * rand());

        turbulence_horizontal_speed =
                static_cast<float>(turbulence_horizontal_speed * iir_coef+wind_turb * rand_normal(0, 1) * (1 - iir_coef));

        turbulence_vertical_speed = static_cast<float>((turbulence_vertical_speed * iir_coef) + (wind_turb * rand_normal(0, 1) * (1 - iir_coef)));

        wind_ef += Vector3f(
            cosf(radians(turbulence_azimuth)) * turbulence_horizontal_speed,
            sinf(radians(turbulence_azimuth)) * turbulence_horizontal_speed,
            turbulence_vertical_speed);
    }
}

/*
 calculate magnetic field intensity and orientation
*/
bool Aircraft::get_mag_field_ef(float latitude_deg, float longitude_deg, float &intensity_gauss, float &declination_deg, float &inclination_deg)
{
    bool valid_input_data = true;

    /* round down to nearest sampling resolution */
    int32_t min_lat = static_cast<int32_t>(static_cast<int32_t>(latitude_deg / SAMPLING_RES) * SAMPLING_RES);
    int32_t min_lon = static_cast<int32_t>(static_cast<int32_t>(longitude_deg / SAMPLING_RES) * SAMPLING_RES);

    /* for the rare case of hitting the bounds exactly
     * the rounding logic wouldn't fit, so enforce it.
     */

    /* limit to table bounds - required for maxima even when table spans full globe range */
    if (latitude_deg <= SAMPLING_MIN_LAT) {
        min_lat = static_cast<int32_t>(SAMPLING_MIN_LAT);
        valid_input_data = false;
    }

    if (latitude_deg >= SAMPLING_MAX_LAT) {
        min_lat = static_cast<int32_t>(static_cast<int32_t>(latitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES);
        valid_input_data = false;
    }

    if (longitude_deg <= SAMPLING_MIN_LON) {
        min_lon = static_cast<int32_t>(SAMPLING_MIN_LON);
        valid_input_data = false;
    }

    if (longitude_deg >= SAMPLING_MAX_LON) {
        min_lon = static_cast<int32_t>(static_cast<int32_t>(longitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES);
        valid_input_data = false;
    }

    /* find index of nearest low sampling point */
    uint32_t min_lat_index = static_cast<uint32_t>((-(SAMPLING_MIN_LAT) + min_lat)  / SAMPLING_RES);
    uint32_t min_lon_index = static_cast<uint32_t>((-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES);

    /* calculate intensity */

    float data_sw = intensity_table[min_lat_index][min_lon_index];
    float data_se = intensity_table[min_lat_index][min_lon_index + 1];;
    float data_ne = intensity_table[min_lat_index + 1][min_lon_index + 1];
    float data_nw = intensity_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    float data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    float data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    intensity_gauss = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    /* calculate declination */

    data_sw = declination_table[min_lat_index][min_lon_index];
    data_se = declination_table[min_lat_index][min_lon_index + 1];;
    data_ne = declination_table[min_lat_index + 1][min_lon_index + 1];
    data_nw = declination_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    declination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    /* calculate inclination */

    data_sw = inclination_table[min_lat_index][min_lon_index];
    data_se = inclination_table[min_lat_index][min_lon_index + 1];;
    data_ne = inclination_table[min_lat_index + 1][min_lon_index + 1];
    data_nw = inclination_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    inclination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    return valid_input_data;
}

/*
  smooth sensors for kinematic consistancy when we interact with the ground
 */
void Aircraft::smooth_sensors(void)
{
    uint64_t now = time_now_us;
    Vector3f delta_pos = position - smoothing.position;
    if (smoothing.last_update_us == 0 || delta_pos.length() > 10) {
        smoothing.position = position;
        smoothing.rotation_b2e = dcm;
        smoothing.accel_body = accel_body;
        smoothing.velocity_ef = velocity_ef;
        smoothing.gyro = gyro;
        smoothing.last_update_us = now;
        smoothing.location = location;
        printf("Smoothing reset at %.3f\n", now * 1.0e-6f);
        return;
    }
    const float delta_time = (now - smoothing.last_update_us) * 1.0e-6f;

    // calculate required accel to get us to desired position and velocity in the time_constant
    const float time_constant = 0.1f;
    Vector3f dvel = (velocity_ef - smoothing.velocity_ef) + (delta_pos / time_constant);
    Vector3f accel_e = dvel / time_constant + (dcm * accel_body + Vector3f(0.0f, 0.0f, GRAVITY_MSS));
    const float accel_limit = 14 * GRAVITY_MSS;
    accel_e.x = constrain_float(accel_e.x, -accel_limit, accel_limit);
    accel_e.y = constrain_float(accel_e.y, -accel_limit, accel_limit);
    accel_e.z = constrain_float(accel_e.z, -accel_limit, accel_limit);
    smoothing.accel_body = smoothing.rotation_b2e.transposed() * (accel_e + Vector3f(0.0f, 0.0f, -GRAVITY_MSS));

    // calculate rotational rate to get us to desired attitude in time constant
    Quaternion desired_q, current_q, error_q;
    desired_q.from_rotation_matrix(dcm);
    desired_q.normalize();
    current_q.from_rotation_matrix(smoothing.rotation_b2e);
    current_q.normalize();
    error_q = desired_q / current_q;
    error_q.normalize();

    Vector3f angle_differential;
    error_q.to_axis_angle(angle_differential);
    smoothing.gyro = gyro + angle_differential / time_constant;

    float R, P, Y;
    smoothing.rotation_b2e.to_euler(&R, &P, &Y);
    float R2, P2, Y2;
    dcm.to_euler(&R2, &P2, &Y2);

#if 0
    DataFlash_Class::instance()->Log_Write("SMOO", "TimeUS,AEx,AEy,AEz,DPx,DPy,DPz,R,P,Y,R2,P2,Y2",
                                           "Qffffffffffff",
                                           AP_HAL::micros64(),
                                           degrees(angle_differential.x),
                                           degrees(angle_differential.y),
                                           degrees(angle_differential.z),
                                           delta_pos.x, delta_pos.y, delta_pos.z,
                                           degrees(R), degrees(P), degrees(Y),
                                           degrees(R2), degrees(P2), degrees(Y2));
#endif


    // integrate to get new attitude
    smoothing.rotation_b2e.rotate(smoothing.gyro * delta_time);
    smoothing.rotation_b2e.normalize();

    // integrate to get new position
    smoothing.velocity_ef += accel_e * delta_time;
    smoothing.position += smoothing.velocity_ef * delta_time;

    smoothing.location = home;
    location_offset(smoothing.location, smoothing.position.x, smoothing.position.y);
    smoothing.location.alt  = static_cast<int32_t>(home.alt - smoothing.position.z * 100.0f);

    smoothing.last_update_us = now;
    smoothing.enabled = true;
}

/*
  return a filtered servo input as a value from -1 to 1
  servo is assumed to be 1000 to 2000, trim at 1500
 */
float Aircraft::filtered_idx(float v, uint8_t idx)
{
    if (sitl->servo_speed <= 0) {
        return v;
    }
    const float cutoff = 1.0f / (2 * M_PI * sitl->servo_speed);
    servo_filter[idx].set_cutoff_frequency(cutoff);
    return servo_filter[idx].apply(v, frame_time_us * 1.0e-6f);
}


/*
  return a filtered servo input as a value from -1 to 1
  servo is assumed to be 1000 to 2000, trim at 1500
 */
float Aircraft::filtered_servo_angle(const struct sitl_input &input, uint8_t idx)
{
    const float v = (input.servos[idx] - 1500)/500.0f;
    return filtered_idx(v, idx);
}

/*
  return a filtered servo input as a value from 0 to 1
  servo is assumed to be 1000 to 2000
 */
float Aircraft::filtered_servo_range(const struct sitl_input &input, uint8_t idx)
{
    const float v = (input.servos[idx] - 1000)/1000.0f;
    return filtered_idx(v, idx);
}

}  // namespace SITL
