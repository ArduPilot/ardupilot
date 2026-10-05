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

#include "AP_GPS_SITL.h"

#if AP_SIM_GPS_ENABLED

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>

extern const AP_HAL::HAL& hal;

/*
  return GPS time of week in milliseconds
 */
/*
  get timeval using simulation time
 */
static void simulation_timeval(struct timeval *tv)
{
    uint64_t now = AP_HAL::micros64();
    static uint64_t first_usec;
    static struct timeval first_tv;
    if (first_usec == 0) {
        first_usec = now;
        first_tv.tv_sec = AP::sitl()->start_time_UTC;
    }
    *tv = first_tv;
    tv->tv_sec += now / 1000000ULL;
    uint64_t new_usec = tv->tv_usec + (now % 1000000ULL);
    tv->tv_sec += new_usec / 1000000ULL;
    tv->tv_usec = new_usec % 1000000ULL;
}
static void gps_time(uint16_t *time_week, uint32_t *time_week_ms)
{
    struct timeval tv;
    simulation_timeval(&tv);
    const uint32_t epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - (GPS_LEAPSECONDS_MILLIS / 1000ULL);
    uint32_t epoch_seconds = tv.tv_sec - epoch;
    *time_week = epoch_seconds / AP_SEC_PER_WEEK;
    uint32_t t_ms = tv.tv_usec / 1000;
    // round time to nearest 200ms
    *time_week_ms = (epoch_seconds % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC + ((t_ms/200) * 200);
}

bool AP_GPS_SITL::read(void)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 200) {
        return false;
    }
    last_update_ms = now;

    auto *sitl = AP::sitl();

    SITL::GPS_Data d {};
    d.latitude = sitl->state.latitude;
    d.longitude = sitl->state.longitude;
    d.altitude = sitl->state.altitude;
    d.speedN = sitl->state.speedN;
    d.speedE = sitl->state.speedE;
    d.speedD = sitl->state.speedD;
    d.have_lock = true;
    d.num_sats = 15;

    if (state.instance < ARRAY_SIZE(sitl->gps)) {
        apply_sim_faults(d, sitl->gps[state.instance]);
    }

    uint16_t time_week;
    uint32_t time_week_ms;

    gps_time(&time_week, &time_week_ms);

    state.time_week = time_week;
    state.time_week_ms = time_week_ms;
    state.status = d.have_lock ? AP_GPS_FixType::FIX_3D : AP_GPS_FixType::NONE;
    state.num_sats = d.num_sats;

    state.location = Location{
        int32_t(d.latitude*1e7),
        int32_t(d.longitude*1e7),
        int32_t(d.altitude*100),
        Location::AltFrame::ABSOLUTE
    };

    state.hdop = 100;
    state.vdop = 100;

    state.have_vertical_velocity = true;
    state.velocity.x = d.speedN;
    state.velocity.y = d.speedE;
    state.velocity.z = d.speedD;

    velocity_to_speed_course(state);

    state.have_speed_accuracy = true;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;
    state.horizontal_accuracy = d.horizontal_acc;
    state.vertical_accuracy = d.vertical_acc;
    state.speed_accuracy = d.speed_acc;

    state.last_gps_time_ms = now;

    return true;
}

/*
  apply the SIM_GPSn_ fault parameters the same way the simulated
  serial GPS devices do (see SITL::GPS::update). With the parameters
  at their defaults the sample is left unchanged
 */
void AP_GPS_SITL::apply_sim_faults(SITL::GPS_Data &d, SITL::SIM::GPSParms &sim_params)
{
    const uint32_t now_ms = AP_HAL::millis();

    // a GPS selected with GPS_TYPE 100 is enabled unless the user has
    // set SIM_GPSn_ENABLE, so the default fix is unchanged
    if (!sim_params.enabled.configured()) {
        sim_params.enabled.set(1);
    }

    // SIM_GPSn_ENABLE and SIM_GPSn_LCKTIME
    d.have_lock = sim_params.enabled && now_ms >= sim_params.lock_time*1000UL;

    // SIM_GPSn_VERR
    const Vector3f vel_err = sim_params.vel_err;
    d.speedN += vel_err.x * rand_float();
    d.speedE += vel_err.y * rand_float();
    d.speedD += vel_err.z * rand_float();
    d.speed_acc = vel_err.xy().length();

    // SIM_GPSn_GLTCH
    const Vector3f glitch = sim_params.glitch;
    d.latitude += glitch.x;
    d.longitude += glitch.y;
    d.altitude += glitch.z;

    // SIM_GPSn_JAM
    if (sim_params.jam == 1) {
        jamming.simulate(d);
    }
}

#endif  // AP_SIM_GPS_ENABLED
