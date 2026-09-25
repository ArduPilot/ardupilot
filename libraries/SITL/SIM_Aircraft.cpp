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

#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_JSON/AP_JSON.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL_SITL/HAL_SITL_Class.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/SITL_SwarmInfo.h>
#endif

using namespace SITL;

extern const AP_HAL::HAL& hal;

// the SITL HAL can add information about pausing the simulation and its effect on the UART.  Not present when we're compiling for simulation-on-hardware
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
extern const HAL_SITL& hal_sitl;
#endif

Aircraft *Aircraft::instances[MAX_SIM_INSTANCES];

/*
  parent class for all simulator types
 */

Aircraft::Aircraft(const char *frame_str)
{
    // make the SIM_* variables available to simulator backends
    sitl = AP::sitl();

    set_speedup(1.0f);

    last_wall_time_us = get_wall_time_us();

    // allow for orientation settings, such as with tailsitters
    enum ap_var_type ptype;
    ahrs_orientation = (AP_Int8 *)AP_Param::find("AHRS_ORIENTATION", &ptype);

    // ahrs_orientation->get() returns ROTATION_NONE here, regardless of the actual value
    enum Rotation imu_rotation = ahrs_orientation?(enum Rotation)ahrs_orientation->get():ROTATION_NONE;
    last_imu_rotation = imu_rotation;
    // sitl is null if running example program
    if (sitl) {
        sitl->ahrs_rotation.from_rotation(imu_rotation);
        sitl->ahrs_rotation_inv = sitl->ahrs_rotation.transposed();
    }

    // init rangefinder array to NaN to signify no data
    for (uint8_t i = 0; i < ARRAY_SIZE(rangefinder_m); i++){
        rangefinder_m[i] = nanf("");
    }
}

void Aircraft::set_start_location(const Location &start_loc, const float start_yaw)
{
    home = start_loc;
    origin = home;
    position.xy().zero();
    home_yaw = start_yaw;
    home_is_set = true;

    ::printf("Home: %f %f alt=%fm hdg=%f\n",
             home.lat*1e-7,
             home.lng*1e-7,
             home.alt*0.01,
             home_yaw);

    location = home;
    ground_level = home.alt * 0.01f;

#if 0
    // useful test for home position being very different from origin
    home.offset(-3000*1000, 1800*1000);
#endif

    dcm.from_euler(0.0f, 0.0f, radians(home_yaw));
}

/*
   return difference in altitude between home position and current loc
*/
float Aircraft::ground_height_difference() const
{
#if AP_TERRAIN_AVAILABLE
    AP_Terrain *terrain = AP::terrain();
    float h1, h2;
    if (sitl &&
        terrain != nullptr &&
        sitl->terrain_enable &&
        terrain->height_amsl(home, h1, false) &&
        terrain->height_amsl(location, h2, false)) {
        h2 += local_ground_level;
        return h2 - h1;
    }
#endif
    return local_ground_level;
}

float Aircraft::ambient_outside_temperature_degC() const
{
    // FIXME: stop applying autopilot warming to this value
    return baro_temperature_degC();
}

// the ambient temperature for the autopilot baro sensors, *not* the
// ambient temperature for the outside of the vehicle.  This
// temperature is adjusted for things like the simulated board heating
// up
float Aircraft::baro_temperature_degC() const
{
    // FIXME: AP_Baro_SITL should be getting temperature from the
    // simulated aircraft, not the other way around!
#if !APM_BUILD_TYPE(APM_BUILD_AP_Periph)    // Periph does not instantiate Baro
    return AP::baro().get_temperature();
#endif
    return 25.0;
}

// returns the expected ambient pressure for the vehicle.  So pressure
// drops preceived by the sensors due to airflow should not be
// included in this number.
float Aircraft::ambient_outside_pressure_Pascal() const
{
    // FIXME: this includes airflow-related things
    return AP::baro().get_pressure();
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
    return hagl() <= 0.001f;  // prevent bouncing around ground
}

/*
   update location from position
*/
void Aircraft::update_position(void)
{
    location = origin;
    location.offset(position.x, position.y);

    location.alt  = static_cast<int32_t>(home.alt - position.z * 100.0f);

#if 0
    Vector3d pos_home = position;
    pos_home.xy() += home.get_distance_NE_double(origin);

    // logging of raw sitl data
    Vector3f accel_ef = dcm * accel_body;
// @LoggerMessage: SITL
// @Description: Simulation data
// @Field: TimeUS: Time since system startup
// @Field: VN: Velocity - North component
// @Field: VE: Velocity - East component
// @Field: VD: Velocity - Down component
// @Field: AN: Acceleration - North component
// @Field: AE: Acceleration - East component
// @Field: AD: Acceleration - Down component
// @Field: PN: Position - North component
// @Field: PE: Position - East component
// @Field: PD: Position - Down component
    AP::logger().WriteStreaming("SITL", "TimeUS,VN,VE,VD,AN,AE,AD,PN,PE,PD", "Qfffffffff",
                                           AP_HAL::micros64(),
                                           velocity_ef.x, velocity_ef.y, velocity_ef.z,
                                           accel_ef.x, accel_ef.y, accel_ef.z,
                                           pos_home.x, pos_home.y, pos_home.z);
#endif

    if (!disable_origin_movement) {
        uint32_t now = AP_HAL::millis();
        if (now - last_one_hz_ms >= 1000) {
            // shift origin of position at 1Hz to current location
            // this prevents spherical errors building up in the GPS data
            last_one_hz_ms = now;
            Vector2d diffNE = origin.get_distance_NE_double(location);
            position.xy() -= diffNE;
            smoothing.position.xy() -= diffNE;
            origin.lat = location.lat;
            origin.lng = location.lng;
        }
    }
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
    AP_Declination::get_mag_field_ef(location.lat * 1e-7f, location.lng * 1e-7f, intensity, declination, inclination);

    // create a field vector and rotate to the required orientation
    Vector3f mag_ef(1e3f * intensity, 0.0f, 0.0f);
    Matrix3f R;
    R.from_euler(0.0f, -radians(inclination), radians(declination));
    mag_ef = R * mag_ef;

    // calculate frame height above ground
    const float frame_height_agl = fmaxf((-position.z) + home.alt * 0.01f - ground_level, 0.0f);

    if (!sitl) {
        // running example program
        return;
    }

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
        uint64_t step_us = frame_time_us;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
        /*
          under the adaptive rate governor the commanded speedup is a
          CONSTRAINT, not a target: simulated time is slaved to the
          wall clock so achieved speedup equals the commanded value by
          construction, on any hardware. Each frame's timestep
          stretches by exactly the current schedule deficit - on a
          host with headroom the deficit is zero and the nominal
          timestep (and the pacing sleeps) are untouched; on a host
          that cannot keep up, granularity degrades in real time by
          precisely the amount required and time never falls behind.
          The per-step rotation clamp in update_dynamics() keeps the
          integration bounded at any stretched timestep. The stretch
          is capped per frame so a single OS stall smears over many
          frames instead of one giant step.
        */
        static int8_t slaved = -1;
        if (slaved == -1) {
            slaved = getenv("SITL_ADAPTIVE_RATE") != nullptr;
        }
        static bool engaged;
        static uint64_t epoch_wall_us, epoch_sim_us;
        if (slaved == 1 && !engaged) {
            // the published epoch IS the engagement signal: the armed
            // instant is transient (flags flicker around crashes and
            // relands), and a member that misses it must still adopt
            // the cluster schedule the moment one exists - a
            // non-engaged member crawling at natural pace beside
            // slaved peers was measured at 54,000 sim-seconds of skew
            uint64_t w = 0, t = 0;
            if (hal_sitl.get_sitl_state()->
                    _shared_mem.peek_slave_epoch(w, t)) {
                epoch_wall_us = w;
                epoch_sim_us = t;
                engaged = true;
                ::printf("SIM: wall-slaved schedule engaged: adopted "
                         "existing cluster epoch\n");
            }
        }
        if (slaved == 1 && !engaged &&
            hal_sitl.get_sitl_state()->_shared_mem.all_members_armed()) {
            // the schedule engages when EVERY cluster member is armed,
            // and stays engaged: boot, configuration and arming are
            // wall-bound interactive work, and slaving time through
            // them makes every sim-paced wait (parameter round-trips,
            // arming checks, EKF settling) expire in wall-instants -
            // and one armed member racing ahead drags the still-unarmed
            // rest through catch-up they cannot sustain mid-takeoff.
            // From the moment the whole formation flies, the commanded
            // speedup is guaranteed.
            engaged = true;
            ::printf("SIM: wall-slaved schedule engaged: all cluster "
                     "members armed (own soft_armed=%u)\n",
                     (unsigned)hal.util->get_soft_armed());
        }
        slave_engaged = (slaved == 1 && engaged);
        if (slaved == 1 && engaged) {
            const uint64_t wall_us = get_wall_time_us();
            if (epoch_wall_us == 0) {
                // ONE schedule for the whole cluster: the first member
                // to get here publishes the epoch into the shared
                // segment and everyone slaves to that single timeline.
                // Per-member epochs would pin members to permanently
                // offset schedules, and a slaved member re-stretches
                // over every barrier wait - measured as tens of
                // thousands of sim-seconds of skew.
                uint64_t w = wall_us, t = time_now_us;
                if (hal_sitl.get_sitl_state()->
                        _shared_mem.get_or_set_slave_epoch(w, t)) {
                    epoch_wall_us = w;
                    epoch_sim_us = t;
                } else {
                    // not clustered (or segment gone): local epoch
                    epoch_wall_us = wall_us;
                    epoch_sim_us = time_now_us;
                }
            }
            /*
              the schedule targets 5% ABOVE the commanded speedup:
              measurement and stretch-cap losses otherwise leave a
              converged run reading ~95% forever. 95% is a FAIL,
              100% is good, 105% is margin.
            */
            static int8_t margin_seeded = -1;
            if (margin_seeded == -1) {
                margin_seeded = 1;
                /*
                  the margin scales with the commanded speedup:
                  margin% = speedup/5, so 20% at 100x, 10% at 50x,
                  4% at 20x - higher speedups accumulate more
                  measurement and stretch-cap loss to cover (the
                  original speedup/10 law under-resolved ~11% short
                  in practice). SITL_SPEEDUP_MARGIN (percent)
                  overrides.
                */
                slave_margin_floor = constrain_float(
                    target_speedup * 0.002f, 0.0f, 0.30f);
                const char *m = getenv("SITL_SPEEDUP_MARGIN");
                if (m != nullptr) {
                    slave_margin_floor = constrain_float(
                        atof(m) * 0.01f, 0.0f, 0.5f);
                }
                slave_margin = slave_margin_floor;
            }
            const uint64_t scheduled_sim_us = epoch_sim_us +
                (uint64_t)((wall_us - epoch_wall_us) *
                           (double)target_speedup *
                           (1.0 + slave_margin));
            if (scheduled_sim_us > time_now_us + step_us) {
                const uint64_t deficit_us =
                    scheduled_sim_us - (time_now_us + step_us);
                // the stretch cap grows smoothly with height: ground contact demands
                // nearly nominal frames, altitude tolerates coarser steps, and a hard
                // band edge put a 2.5x cadence cliff mid-climb vehicles oscillated on
                /*
                  dense frames near the ground AND whenever the
                  attitude is upset: the ramp starts at 10m (climbing
                  through 3-5m at a 9ms step put the takeoff transient
                  inside the measured instability band and copters
                  tumbled at full throttle), and a tilt beyond ~30
                  degrees snaps the cap back to dense regardless of
                  height - a tumble is the plant demanding fidelity,
                  and recovery at a coarse step only feeds it.
                */
                /*
                  ceiling +5ms: zero-order-held aerodynamic forces at
                  20ms steps pumped energy into the plane (1.6km above
                  its LOITER target, still climbing 3 m/s against a
                  protesting TECS - the tamed remnant of the old
                  escaped-the-atmosphere failures). With the nominal
                  rate floored at 100Hz the ceiling costs almost no
                  throughput, and the aero bias shrinks fourfold.
                */
                const float agl =
                    constrain_float(hagl() - 10.0f, 0.0f, 30.0f);
                uint64_t stretch_cap_us =
                    1000 + (uint64_t)(agl * (4000.0f / 30.0f));
                if (dcm.c.z < 0.85f) {
                    stretch_cap_us = 1000;
                }
                step_us += MIN(deficit_us, stretch_cap_us);
            }
        }
#endif
        effective_frame_time_us = step_us;
        time_now_us += step_us;
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
    frame_time_us = uint64_t(1.0e6f/rate_hz);

    last_wall_time_us = get_wall_time_us();
}

/* adjust frame_time calculation */
void Aircraft::adjust_frame_time(float new_rate)
{
    frame_time_us = uint64_t(1.0e6f/new_rate);
    rate_hz = new_rate;
}

// try to synchronise simulation time with wall clock time, taking into
// account desired speedup, allowing for get_wall_time_us() granularity
// (notably on Windows)
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
// runtime physics-rate governor, evaluated once per wall-time
// reporting interval from sync_frame_time()
void Aircraft::adaptive_rate_governor(const uint32_t now_ms)
{
    // runtime governor (SITL_ADAPTIVE_RATE=1): trades physics frame rate
    // for achieved speedup, restoring fidelity with headroom; the
    // achieved-vs-rate curve is non-monotonic per host, so measure at runtime
    static int8_t adaptive = -1;
    if (adaptive == -1) {
        adaptive = getenv("SITL_ADAPTIVE_RATE") != nullptr;
    }
    if (adaptive == 1 && sitl != nullptr && slave_engaged) {
        /*
          track the nominal frame rate to MEASURED throughput so the
          stretch stays near zero in steady state: the vehicle then
          runs self-consistently at whatever rate the host affords
          (the regime the governed campaign proved flyable down to
          50Hz) while the wall-slaved stretch covers transients and
          guarantees the schedule. Pinning nominal at a rate the host
          cannot deliver left every frame stretched - nominal !=
          actual - and vehicle-side constants that assume the nominal
          rate (gyro filtering above all) degraded the attitude loop
          into a full-throttle tumble.
        */
        static uint32_t last_track_ms;
        /*
          track slowly, and never during takeoff/landing: the nominal
          rate re-derives the vehicle's filter and controller
          constants, and re-tuning a copter every 2s from a
          thermally-noisy fps reading destabilised exactly the
          delicate phases - solo (steady fps) flew at the same
          timestep the cluster (swinging fps) bounced at.
        */
        if (now_ms - last_track_ms > 4000 && hagl() > 10.0f) {
            last_track_ms = now_ms;
            if (achieved_rate_hz > 1 && target_speedup >= 1) {
                /*
                  the law: achieved speedup = fps / nominal rate, so
                  the nominal rate that delivers the COMMANDED speedup
                  on this host is measured fps divided by it. Floored
                  at 50Hz - the proven edge of formation flight - so a
                  host too weak even for that runs at the floor with
                  the stretch covering the (bounded) shortfall.
                */
                /*
                  floor 100Hz: measured tonight, a copter at 50Hz/21ms
                  goes inverted on takeoff - the flyable floor is
                  ~10ms. Below the floor the gate reports the honest
                  shortfall; the only true fixes are cheaper
                  iterations or a cooler host.
                */
                /*
                  track against the MARGIN-INCLUSIVE schedule: the
                  slaved clock demands target*(1+margin), and tracking
                  to bare target left every frame stretched by exactly
                  the margin fraction - a permanent nominal-vs-actual
                  gap that degraded the plane's rate-derived control
                  into an unbounded climb (per-sub-step aero
                  re-evaluation did NOT fix it; this consistency did).
                */
                float track_hz = constrain_float(
                    achieved_rate_hz /
                        (target_speedup * (1.0f + slave_margin)),
                    100.0f, 1200.0f);
                /*
                  asymmetric damping: converge fast while far from the
                  operating point (10%/10s from the boot rate took
                  minutes - longer than an entire run), fine-tune
                  gently once within 25% of it
                */
                static bool first_track = true;
                if (!first_track) {
                    const float cur = sitl->loop_rate_hz.get();
                    const float lo = (track_hz < cur * 0.75f)
                                         ? cur * 0.75f : cur * 0.9f;
                    track_hz = constrain_float(track_hz, lo, cur * 1.1f);
                }
                // the first step jumps straight to the operating
                // point: walking down from the boot rate spent the
                // whole run budget mid-descent
                first_track = false;
                sitl->loop_rate_hz.set(track_hz);
            }
        }
    } else if (adaptive == 1 && sitl != nullptr) {
        /*
          un-slaved (the wall-bound prelude, boot to group-arm):
          vehicle HEALTH outranks throughput here, so the frame rate
          is simply pinned at 3x the loop rate - the proven arming
          and flight sweet spot, and comfortably above the 2x line
          under which ArduPilot refuses to arm (a governor that once
          descended past it deadlocked a CI run at 9Hz frames). The
          wall-slaved schedule takes over from the moment of
          engagement.
        */
        static bool prelude_rate_set;
        if (!prelude_rate_set && hal.scheduler->is_system_initialized()) {
            prelude_rate_set = true;
            float loop_hz = 400;
            AP_Scheduler *sched = AP_Scheduler::get_singleton();
            if (sched != nullptr) {
                loop_hz = sched->get_loop_rate_hz();
            }
            const float start_hz =
                constrain_float(3.0f * loop_hz, 400.0f, 1200.0f);
            sitl->loop_rate_hz.set(start_hz);
            ::printf("SIM: adaptive rate: prelude at %.0fHz\n",
                     (double)start_hz);
        }
    }
}
#endif  // HAL_BOARD_SITL && !HAL_BUILD_AP_PERIPH

void Aircraft::sync_frame_time(void)
{
    frame_counter++;
    uint64_t now = get_wall_time_us();
    uint64_t dt_us = now - last_wall_time_us;

    /*
      under the adaptive rate governor, pace to 2% ABOVE the commanded
      speedup: the pacer is a hard ceiling, so a host paced to exactly
      the commanded rate measures fractionally below it forever
      (99.x%) no matter how much headroom it has - the governor then
      keeps sacrificing physics rate chasing a value the pacer will
      never let it reach. 2% of overspeed lets a converged host measure
      at or above the commanded rate; the governor's thresholds keep
      the real operating point within a couple of percent of it.
    */
    static int8_t adaptive_pacing = -1;
    if (adaptive_pacing == -1) {
        adaptive_pacing = getenv("SITL_ADAPTIVE_RATE") != nullptr;
    }
    const float pacing_speedup =
        adaptive_pacing == 1 ? target_speedup * 1.10f : target_speedup;
    const float target_dt_us = 1.0e6/(rate_hz*pacing_speedup);

    // accumulate sleep debt if we're running too fast
    sleep_debt_us += target_dt_us - dt_us;

    /*
      the debt floor is the pacer's memory of how far behind wall
      schedule the sim is. With only 100ms of memory, any OS stall
      longer than that becomes unrepayable lost sim time - at 100x a
      single 300ms scheduler stall permanently costs 20 sim seconds
      against the achieved average, and on a busy CI runner such
      stalls are constant (measured as the ~5% shortfall between
      windows that touch the pacing ceiling and a steady-state that
      does not). Two seconds of memory plus the 10% pacing overspeed
      lets a stalled vehicle sprint the debt back down.
    */
    if (sleep_debt_us < -2.0e6) {
        // don't let an unbounded negative debt build up
        sleep_debt_us = -2.0e6;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
    /*
      Cluster work is decimated: it runs every Nth frame where N is the
      requested speedup, so the number of barrier crossings per WALL
      second is constant regardless of speedup. Synchronising every frame
      would scale sync work linearly with speedup (40000 crossings per
      wall-second at 100x) and, with the skew window fixed in sim time,
      would demand microsecond wall alignment no OS can deliver - which
      is exactly what capped clustered runs near realtime.

      For the same reason the skew tolerance is fixed in WALL time (the
      per-frame default's 5ms at 1x) and scaled into sim time by the
      speedup: scheduling jitter is a wall-clock quantity, so the minimum
      achievable sim-time skew is jitter multiplied by speedup, whatever
      the code asks for. Between crossings every instance advances its
      own clock in the original manner, entirely self-paced.

      One aircraft per SITL process (as the process-global hal_sitl use
      here already assumes), so function-statics hold the state.
    */
    static uint64_t cross_interval_us;
    static uint64_t next_cross_us;
    static bool catching_up_state;
    if (cross_interval_us == 0) {
        /*
          crossings are scheduled on the SHARED sim-time grid, NOT on
          per-vehicle frame counts: under the adaptive rate governor
          each vehicle runs its own physics rate, so frame-count
          schedules drift apart between members and skew accumulates
          without bound (measured: 4s of skew where the time-based
          grid holds under 2s). The interval scales with the speedup
          so crossings per wall second are constant, sparse enough to
          limit exposure to the instantaneous slowest member.
        */
#ifdef CYGWIN_BUILD
        // Cygwin runs each vehicle at a fraction of a core's duty
        // cycle, so members rarely run simultaneously and every
        // crossing pays the currently-descheduled member's wakeup
        // latency - measured as a 3.4x throughput loss (57x solo vs
        // 17x clustered) where Linux loses almost nothing. A quarter
        // of the crossings.
        cross_interval_us = 13333ULL * (uint64_t)target_speedup;
#else
        cross_interval_us = 3333ULL * (uint64_t)target_speedup;
#endif
        cross_interval_us = MAX(cross_interval_us, (uint64_t)1000);
    }
    const uint64_t max_skew_us = 6ULL * cross_interval_us;
    const bool at_crossing = time_now_us >= next_cross_us;
    if (at_crossing) {
        // align to the shared grid so every member crosses at the same
        // sim times regardless of its own physics rate
        next_cross_us = time_now_us - (time_now_us % cross_interval_us)
                        + cross_interval_us;
    }
    if (at_crossing) {
        // on a fresh start, instantly snap our clock to match peers
        // instead of sprinting to catch up
        hal_sitl.get_sitl_state()->_shared_mem.instant_catchup_if_new(time_now_us);

        // if we've fallen behind the swarm (e.g. just rebooted), halve
        // our sleep to run at ~2x speedup and fast-forward back into
        // lock-step, rather than staying permanently excluded. The state
        // persists across the frames in between, so the sprint pacing
        // applies to every frame, not just the checked ones.
        catching_up_state = hal_sitl.get_sitl_state()->_shared_mem.is_behind_peers(
            time_now_us, max_skew_us);
    }
    const float catchup_sleep_scale = catching_up_state ? 0.5f : 1.0f;
#else
    const float catchup_sleep_scale = 1.0f;
#endif
    (void)catchup_sleep_scale;

    if (sleep_debt_us > min_sleep_time) {
        // sleep if we have built up a debt of min_sleep_tim
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        usleep((uint64_t)(sleep_debt_us * catchup_sleep_scale));
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        hal.scheduler->delay_microseconds((uint64_t)(sleep_debt_us * catchup_sleep_scale));
#else
        // ??
#endif
        sleep_debt_us -= (get_wall_time_us() - now);
    }
    last_wall_time_us = get_wall_time_us();

    uint32_t now_ms = last_wall_time_us / 1000ULL;
    float dt_wall = (now_ms - last_fps_report_ms) * 0.001;
    if (dt_wall > 0.01) {  // 0.01s average
        achieved_rate_hz = (frame_counter - last_frame_count) / dt_wall;
        last_frame_count = frame_counter;
        last_fps_report_ms = now_ms;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
        // clustered runs report their achieved speedup so a cluster that
        // cannot reach the requested rate is visible on the console
        // rather than a mystery; quiet for ordinary single-vehicle SITL
        static uint32_t last_rate_print_ms;
        static uint64_t last_print_sim_us;
        static uint32_t last_print_frames;
        if (hal_sitl.get_sitl_state()->_shared_mem.is_initialised() &&
            now_ms - last_rate_print_ms > 2000) {
            last_rate_print_ms = now_ms;
            // average EFFECTIVE timestep this window: sim advanced per
            // frame - the one number that says whether control density
            // is flyable and where the speedup is really coming from
            float dt_avg_ms = 0;
            if (frame_counter > last_print_frames &&
                last_print_sim_us != 0) {
                dt_avg_ms = (time_now_us - last_print_sim_us) * 1.0e-3f /
                            (frame_counter - last_print_frames);
            }
            /*
              close the loop on the margin using the TRUE window
              speedup (sim advanced over wall elapsed): a window under
              commanded raises the margin one point, sustained
              overshoot beyond commanded+margin trims one, clamped to
              [base-ish, 30%]. The schedule then aims however high
              this host needs for the reported number to meet the ask.
            */
            static uint64_t last_margin_wall_us;
            const uint64_t wall_now2 = get_wall_time_us();
            // TRUE speedup this window: sim advanced over wall elapsed.
            // The instantaneous fps sample above is a ~10ms snapshot of
            // a vehicle that alternates sprinting with waiting at the
            // barrier, so it legitimately swings 70-150% of schedule
            // window to window; this is the number that means something.
            float win_speedup = 0;
            if (last_margin_wall_us != 0 && wall_now2 > last_margin_wall_us) {
                win_speedup =
                    (float)(time_now_us - last_print_sim_us) /
                    (float)(wall_now2 - last_margin_wall_us);
            }
            if (slave_engaged && win_speedup > 0) {
                static bool short_last_window;
                if (win_speedup < target_speedup) {
                    /*
                      raise proportionally to the shortfall (fixed
                      1-point steps took 10-20x the run length to
                      settle), but only after TWO consecutive short
                      windows and at most 5 points per step: a single
                      short window is as often a compile burst or
                      scheduler hiccup as a real deficit, and chasing
                      it drove the margin to the ceiling against a
                      transiently-loaded host.
                    */
                    if (short_last_window) {
                        const float shortfall =
                            (target_speedup - win_speedup) /
                            target_speedup;
                        slave_margin = MIN(
                            slave_margin +
                            constrain_float(shortfall, 0.01f, 0.05f),
                            0.50f);
                    }
                    short_last_window = true;
                } else if (win_speedup > target_speedup * 1.02f) {
                    /*
                      trim whenever a window is comfortably above
                      TARGET (not above target+margin, which at high
                      margin is unreachable - that one-way ratchet
                      railed the margin at its cap and the schedule
                      permanently over-aimed 50%, oscillating the
                      cluster 80-116% around an aim it could never
                      satisfy). Proportional and gentle: the margin
                      settles where windows straddle the target.
                    */
                    const float excess =
                        (win_speedup - target_speedup) / target_speedup;
                    slave_margin = MAX(
                        slave_margin -
                        constrain_float(excess * 0.5f, 0.005f, 0.02f),
                        slave_margin_floor);
                    short_last_window = false;
                } else {
                    short_last_window = false;
                }
            }
            last_margin_wall_us = wall_now2;
            last_print_sim_us = time_now_us;
            last_print_frames = frame_counter;
            // ~8s smoothed headline (4 windows) so the reported trend is
            // readable; the raw 2s window stays visible in parentheses
            static float speedup_ema;
            if (win_speedup > 0) {
                speedup_ema = (speedup_ema <= 0)
                    ? win_speedup
                    : 0.75f * speedup_ema + 0.25f * win_speedup;
            }
            const float headline = (speedup_ema > 0)
                ? speedup_ema : achieved_rate_hz / rate_hz;
            // keep under STATUSTEXT's 50 chars or the GCS line chunks
            // into two messages ("...mgn 3" / "0%"). The target is in
            // every harness [follow] line already.
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "speedup %.1fx (2s %.0fx) dt %.1fms mgn %.0f%%",
                          headline, win_speedup,
                          dt_avg_ms, slave_margin * 100.0f);
            ::printf("SIM: sim_time=%.0fs achieved %.1fx of requested "
                     "%.0fx (frames %.0fHz, dt %.1fms, %.0f fps)\n",
                     time_now_us * 1.0e-6,
                     headline, target_speedup, rate_hz,
                     (double)dt_avg_ms, (double)achieved_rate_hz);
        }

        adaptive_rate_governor(now_ms);
#endif
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
    if (at_crossing) {
    // publish basic swarm telemetry (position/velocity/heading) for peer
    // instances to read - see AP_SITL_SwarmInfo for the payload layout.
    {
        AP_SITL_SwarmInfo swarm {};
        swarm.sysid       = mavlink_system.sysid;
        swarm.sim_time_us = time_now_us;
        swarm.lat         = location.lat;
        swarm.lng         = location.lng;
        swarm.alt_cm      = location.alt;
        swarm.vx          = velocity_ef.x;
        swarm.vy          = velocity_ef.y;
        swarm.vz          = velocity_ef.z;
        float roll, pitch, yaw;
        dcm.to_euler(&roll, &pitch, &yaw);
        swarm.heading_deg = wrap_360(degrees(yaw));
        hal_sitl.get_sitl_state()->_shared_mem.write_payload(&swarm, sizeof(swarm));
    }

    // When running multiple SITL instances in parallel, keep them within
    // a bounded sim-time window of each other so inter-vehicle state is
    // consistent. The window is wall-fixed (hence scaled by speedup, see
    // above) and the barrier is a no-op for single-instance runs.
    hal_sitl.get_sitl_state()->_shared_mem.sync_with_peers(time_now_us, max_skew_us);
    }
#endif
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
    bool is_smoothed = false;
    // under SITL_ADAPTIVE_RATE the chase filter never engages AT ALL: its
    // 0.1s chase rings against stretched timesteps (bouncing takeoffs with
    // it, steady flight without), and switching sources mid-run at schedule
    // engagement fed the controllers a sensor discontinuity they convulsed
    // on (7-13 m/s impacts right at arm). EKF type 10 reads truth directly.
    static int8_t adaptive_mode = -1;
    if (adaptive_mode == -1) {
        adaptive_mode = getenv("SITL_ADAPTIVE_RATE") != nullptr;
    }
    if (use_smoothing && adaptive_mode != 1) {
        smooth_sensors();
        is_smoothed = true;
    }
    fdm.timestamp_us = time_now_us;
    // keep track of the number of frames processed so that the IMUs can follow
    if (flightaxis_sync_imus_to_frames) {
        fdm.flightaxis_imu_frame_num++;
    }

    if (fdm.home.lat == 0 && fdm.home.lng == 0) {
        // initialise home
        fdm.home = home;
    }
    fdm.is_lock_step_scheduled = lock_step_scheduled;
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
    fdm.quaternion.from_rotation_matrix(dcm);
    fdm.airspeed = airspeed_pitot;
    fdm.velocity_air_bf = velocity_air_bf;
    fdm.battery_voltage = battery_voltage;
    fdm.battery_current = battery_current;
    fdm.motor_mask = motor_mask | sitl->vibe_motor_mask;
    memcpy(fdm.rpm, rpm, sizeof(fdm.rpm));
    fdm.rcin_chan_count = rcin_chan_count;
    fdm.range = rangefinder_range();
    memcpy(fdm.rcin, rcin, rcin_chan_count * sizeof(float));
    fdm.bodyMagField = mag_bf;

    fdm.bodyMagField.rotate(sitl->imu_orientation);

    // copy laser scanner results
    fdm.scanner.points = scanner.points;
    fdm.scanner.ranges = scanner.ranges;

    // copy rangefinder
    memcpy(fdm.rangefinder_m, rangefinder_m, sizeof(fdm.rangefinder_m));

    fdm.wind_vane_apparent.direction = wind_vane_apparent.direction;
    fdm.wind_vane_apparent.speed = wind_vane_apparent.speed;

    fdm.wind_ef = wind_ef;

    if (is_smoothed) {
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


    if (ahrs_orientation != nullptr) {
        enum Rotation imu_rotation = (enum Rotation)ahrs_orientation->get();
        if (imu_rotation != last_imu_rotation) {
            sitl->ahrs_rotation.from_rotation(imu_rotation);
            sitl->ahrs_rotation_inv = sitl->ahrs_rotation.transposed();
            last_imu_rotation = imu_rotation;
        }
        if (imu_rotation != ROTATION_NONE) {
            Matrix3f m = dcm;
            m = m * sitl->ahrs_rotation_inv;

            m.to_euler(&r, &p, &y);
            fdm.rollDeg  = degrees(r);
            fdm.pitchDeg = degrees(p);
            fdm.yawDeg   = degrees(y);
            fdm.quaternion.from_rotation_matrix(m);
        }
    }

    // in the first call here, if a speedup option is specified, overwrite it
    if (is_equal(last_speedup, -1.0f) && !is_equal(get_speedup(), 1.0f)) {
        sitl->speedup.set(get_speedup());
    }
    
    if (!is_equal(last_speedup, float(sitl->speedup)) && sitl->speedup > 0) {
        set_speedup(sitl->speedup);
        last_speedup = sitl->speedup;
    }

#if HAL_LOGGING_ENABLED
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // the SITL HAL can add information about pausing the simulation
    // and its effect on the UART.  Not present when we're compiling
    // for simulation-on-hardware
    const uint32_t full_count = hal_sitl.get_uart_output_full_queue_count();
#else
    const uint32_t full_count = 0;
#endif
    // for EKF comparison log relhome pos and velocity at loop rate
    static uint16_t last_ticks;
    // FIXME: stop using AP_Scheduler here!  It's from "the other side"
    const auto *scheduler = AP_Scheduler::get_singleton();
    if (scheduler == nullptr) {
        // not instantiated in examples
        return;
    }
    uint16_t ticks = scheduler->ticks();
    if (last_ticks != ticks) {
        last_ticks = ticks;
// @LoggerMessage: SIM2
// @Description: Additional simulator state
// @Field: TimeUS: Time since system startup
// @Field: PN: North position from home
// @Field: PE: East position from home
// @Field: PD: Down position from home
// @Field: VN: Velocity north
// @Field: VE: Velocity east
// @Field: VD: Velocity down
// @Field: As: Airspeed
// @Field: ASpdU: Achieved simulation speedup value
// @Field: UFC: Number of times simulation paused for serial0 output
        Vector3d pos = get_position_relhome();
        Vector3f vel = get_velocity_ef();
        AP::logger().WriteStreaming(
            "SIM2",
            "TimeUS,PN,PE,PD,VN,VE,VD,As,ASpdU,UFC",
            "QdddfffffI",
            AP_HAL::micros64(),
            pos.x, pos.y, pos.z,
            vel.x, vel.y, vel.z,
            airspeed_pitot,
            achieved_rate_hz/rate_hz,
            full_count
        );
    }
#endif
}

/*
  rover and copter have special handling for horizontal rangefinders
  as distance to obstacles - this takes effect for yaw-only
  orientations
 */
#define SITL_RANGEFINDER_AS_OBJECT_SENSOR (APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_Rover))
#define SITL_RANGEFINDER_IS_YAW_ONLY(orientation) (orientation <= ROTATION_YAW_315)

// returns perpendicular height to surface rangefinder is bouncing off
float Aircraft::perpendicular_distance_to_rangefinder_surface() const
{
#if SITL_RANGEFINDER_AS_OBJECT_SENSOR
    const auto orientation = (Rotation)sitl->sonar_rot.get();
    if (SITL_RANGEFINDER_IS_YAW_ONLY(orientation)) {
        // assume these are avoidance sensors
        return sitl->measure_distance_at_angle_bf(location, sitl->sonar_rot.get()*45);
    }
#endif

    // default is ground sensing rangefinders
    return sitl->state.height_agl;
}

float Aircraft::rangefinder_range() const
{
    float altitude = perpendicular_distance_to_rangefinder_surface();

    // sensor position offset in body frame
    const Vector3f relPosSensorBF = sitl->rngfnd_pos_offset;

    // n.b. the following code is assuming rotation-pitch-270:
    // adjust altitude for position of the sensor on the vehicle if position offset is non-zero
    if (!relPosSensorBF.is_zero()) {
        // get a rotation matrix following DCM conventions (body to earth)
        Matrix3f rotmat;
        sitl->state.quaternion.rotation_matrix(rotmat);
        // rotate the offset into earth frame
        const Vector3f relPosSensorEF = rotmat * relPosSensorBF;
        // correct the altitude at the sensor
        altitude -= relPosSensorEF.z;
    }

    altitude += sitl->sonar_offset;

    const auto orientation = (Rotation)sitl->sonar_rot.get();
#if SITL_RANGEFINDER_AS_OBJECT_SENSOR

    float roll = sitl->state.rollDeg;
    float pitch = sitl->state.pitchDeg;

    if (roll > 0) {
        roll -= rangefinder_beam_width();
        if (roll < 0) {
            roll = 0;
        }
    } else {
        roll += rangefinder_beam_width();
        if (roll > 0) {
            roll = 0;
        }
    }
    if (pitch > 0) {
        pitch -= rangefinder_beam_width();
        if (pitch < 0) {
            pitch = 0;
        }
    } else {
        pitch += rangefinder_beam_width();
        if (pitch > 0) {
            pitch = 0;
        }
    }

    /*
      rover and copter using SITL rangefinders as obstacle sensors
     */
    if (SITL_RANGEFINDER_IS_YAW_ONLY(orientation)) {
        if (fabs(roll) >= 90.0 || fabs(pitch) >= 90.0) {
            // not going to hit the ground....
            return INFINITY;
        }
        altitude /= cosf(radians(roll)) * cosf(radians(pitch));
    } else
#endif
    {
        // adjust for rotation based on orientation of the sensor
        Matrix3f rotmat;
        sitl->state.quaternion.rotation_matrix(rotmat);
        Vector3f v{1, 0, 0};
        v.rotate(orientation);
        v = rotmat * v;

        if (!is_positive(v.z)) {
            return INFINITY;
        }
        altitude /= v.z;

        // this is awful, but there are drawbacks to assuming an
        // infinite plane.  If we don't do this here then we end up
        // with a ridiculous rangefinder range, and that can cause
        // floating point exceptions when we return a distance in cm
        // from the AP_RangeFinder_SITL.
        if (altitude > 100000) {
            return INFINITY;
        }
    }

    // Add some noise on reading
    altitude += sitl->sonar_noise * rand_float();

    // our starting positions can disagree with the terrain database:
    if (altitude < 0) {
        altitude = 0;
    }

    return altitude;
}

#if defined(__CYGWIN__) || defined(__CYGWIN64__)
extern "C" { uint32_t timeGetTime(); }
#endif

// potentially replace this with a call to AP_HAL::Util::get_hw_rtc
uint64_t Aircraft::get_wall_time_us() const
{
#if defined(__CYGWIN__) || defined(__CYGWIN64__)
    static uint32_t tPrev;
    static uint64_t last_ret_us;
    if (tPrev == 0) {
        tPrev = timeGetTime();
        return 0;
    }
    uint32_t now = timeGetTime();
    last_ret_us += (uint64_t)((now - tPrev)*1000UL);
    tPrev = now;
    return last_ret_us;
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
#else
    return AP_HAL::micros64();
#endif
}

/*
  set simulation speedup
 */
void Aircraft::set_speedup(float speedup)
{
    setup_frame_time(rate_hz, speedup);
}

void Aircraft::update_home()
{
    if (!home_is_set) {
        if (sitl == nullptr) {
            return;
        }
        const Location loc{
            int32_t(sitl->opos.lat.get() * 1.0e7),
            int32_t(sitl->opos.lng.get() * 1.0e7),
            int32_t(sitl->opos.alt.get() * 1.0e2),
            Location::AltFrame::ABSOLUTE
        };
        set_start_location(loc, sitl->opos.hdg.get());
    }
}

void Aircraft::update_model(const struct sitl_input &input)
{
    local_ground_level = 0.0f;
    if (sitl != nullptr) {
        update(input);
    } else {
        time_advance();
    }
}

/*
  update the simulation attitude and relative position
 */
void Aircraft::update_dynamics(const Vector3f &rot_accel)
{
    WITH_SEMAPHORE(pose_sem);

    // integrate with the frame's ACTUAL timestep: when the adaptive
    // governor stretches a frame to hold the wall-clock schedule, the
    // physics must advance by the same amount as the clock
    const float delta_time =
        (effective_frame_time_us != 0 ? effective_frame_time_us
                                      : frame_time_us) * 1.0e-6f;

    // update eas2tas and air density
    eas2tas = AP_Baro::get_EAS2TAS_for_alt_amsl(location.alt*0.01);
    air_density = AP_Baro::get_air_density_for_alt_amsl(location.alt*0.01);

    /*
      sub-stepped integration: the expensive parts of a frame (sensor
      generation, scheduler coupling) run once, but the pure
      integration below costs about a microsecond, so a frame whose
      timestep was stretched by the wall-slaved schedule is integrated
      as N internal sub-steps of at most 5ms each - near full physics
      quality at any stretch (single-step Euler at large timesteps
      turned precise loiters into 50km orbits even with the safety
      clamps). At normal frame rates N is 1 and nothing changes. The
      model's forces are held constant across the sub-steps
      (zero-order hold): control latency remains at the frame rate,
      integration accuracy does not.
    */
    const uint32_t nsub =
        constrain_int32((int32_t)ceilf(delta_time / 0.005f), 1, 4);
    const float dt_sub = delta_time / nsub;
    const bool was_on_ground = on_ground();
    Vector3f accel_earth;
    // hold the body-frame force input constant across sub-steps (true
    // ZOH): re-integrating the rewritten accelerometer view as force misread
    // thrust by the ground-clamp amount - copters bounced on takeoff forever
    const Vector3f accel_body_in = accel_body;
    for (uint32_t isub = 0; isub < nsub; isub++) {
        // update rotational rates in body frame
        gyro += rot_accel * dt_sub;

        float gyro_limit_rads = radians(2000.0f);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
        // bound the rotation per integration step so no single step
        // can rotate more than ~45 degrees: keeps the integration
        // bounded at ANY timestep. Inert at sub-steps of 5ms or less.
        if (dt_sub > 0.011f) {
            gyro_limit_rads = MIN(gyro_limit_rads,
                                  radians(45.0f) / dt_sub);
        }
#endif
        gyro.x = constrain_float(gyro.x, -gyro_limit_rads, gyro_limit_rads);
        gyro.y = constrain_float(gyro.y, -gyro_limit_rads, gyro_limit_rads);
        gyro.z = constrain_float(gyro.z, -gyro_limit_rads, gyro_limit_rads);

        // limit body accel to 64G
        const float accel_limit = 64*GRAVITY_MSS;
        accel_body.x = constrain_float(accel_body.x, -accel_limit, accel_limit);
        accel_body.y = constrain_float(accel_body.y, -accel_limit, accel_limit);
        accel_body.z = constrain_float(accel_body.z, -accel_limit, accel_limit);

        // update attitude
        dcm.rotate(gyro * dt_sub);
        dcm.normalize();

        accel_earth = dcm * accel_body_in;
        accel_earth += Vector3f(0.0f, 0.0f, GRAVITY_MSS);

        // if we're on the ground, then our vertical acceleration is limited
        // to zero. This effectively adds the force of the ground on the aircraft
        if (on_ground() && accel_earth.z > 0) {
            accel_earth.z = 0;
        }

        // new velocity vector
        velocity_ef += accel_earth * dt_sub;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
        // translational counterpart of the rotation clamp: bound the
        // velocity to a generous flight envelope during STRETCHED
        // FRAMES - gated on the frame timestep, not the sub-step:
        // sub-stepping keeps sub-steps at 5ms or less by design, so a
        // sub-step gate would never fire (measured: a plane flew
        // 6,600km with the envelopes silently inert).
        if (delta_time > 0.011f) {
            const float vmax = 120.0f;
            const float vlen = velocity_ef.length();
            if (vlen > vmax) {
                velocity_ef *= vmax / vlen;
            }
        }
#endif

        // new position vector
        position += (velocity_ef * dt_sub).todouble();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
        // position envelope: a loosely-controlled vehicle at heavily
        // stretched timesteps can drift for tens of thousands of sim
        // seconds - one plane climbed out of the atmosphere and the
        // barometric math died of a floating point exception; another
        // rode the velocity cap 2,348km sideways to the same end.
        // Bound the whole state box (NED: -z is up; 45km above, 1km
        // below, 100km of horizontal range) so no downstream math can
        // ever receive an unbounded input. Gated on the FRAME
        // timestep - sub-steps are always small by design.
        if (delta_time > 0.011f) {
            position.z = constrain_double(position.z, -45000.0, 1000.0);
            const double r2 = position.x * position.x +
                              position.y * position.y;
            const double rmax = 100000.0;
            if (r2 > rmax * rmax) {
                const double scale = rmax / sqrt(r2);
                position.x *= scale;
                position.y *= scale;
            }
        }
#endif
    }

    // acceleration as seen by the accelerometers (kinematic plus
    // gravity), computed ONCE from the final attitude and possibly
    // ground-clamped earth-frame acceleration, as the single-step path did
    accel_body = dcm.transposed() *
        (accel_earth + Vector3f(0.0f, 0.0f, -GRAVITY_MSS));

    // velocity relative to air mass, in earth frame
    velocity_air_ef = velocity_ef - wind_ef;

    // velocity relative to airmass in body frame
    velocity_air_bf = dcm.transposed() * velocity_air_ef;

    // airspeed
    update_eas_airspeed();

    // constrain height to the ground
    if (on_ground()) {
        // no more than one report per 10 simulated seconds: a vehicle
        // bouncing on its gear otherwise floods the telemetry stream,
        // but the message must re-arm - autotests crash deliberately
        // and wait for it long after an earlier normal touchdown
        if (!was_on_ground &&
            time_now_us - last_hit_ground_report_us > 10000000ULL) {
            last_hit_ground_report_us = time_now_us;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SIM Hit ground at %f m/s (dt=%.0fms)",
                          velocity_ef.z,
                          (effective_frame_time_us != 0 ?
                           effective_frame_time_us : frame_time_us) * 1.0e-3);
            last_ground_contact_ms = AP_HAL::millis();
        }
        position.z = -(ground_level + frame_height - home.alt * 0.01f + ground_height_difference());

        // get speed of ground movement (for ship takeoff/landing)
        float yaw_rate = 0;
#if AP_SIM_SHIP_ENABLED
        const Vector2f ship_movement = sitl->models.shipsim.get_ground_speed_adjustment(location, yaw_rate);
        const Vector3f gnd_movement(ship_movement.x, ship_movement.y, 0);
#else
        const Vector3f gnd_movement;
#endif
        switch (ground_behavior) {
        case GROUND_BEHAVIOR_NONE:
            break;
        case GROUND_BEHAVIOR_NO_MOVEMENT: {
            // zero roll/pitch, but keep yaw
            float r, p, y;
            dcm.to_euler(&r, &p, &y);
            y = y + yaw_rate * delta_time;
            dcm.from_euler(0.0f, 0.0f, y);
            // X, Y movement tracks ground movement
            velocity_ef.x = gnd_movement.x;
            velocity_ef.y = gnd_movement.y;
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
            if (velocity_ef.length() < 5) {
                // at high speeds don't constrain pitch, otherwise we
                // can get stuck in takeoff
                p = 0;
            } else {
                p = MAX(p, 0);
            }
            y = y + yaw_rate * delta_time;
            dcm.from_euler(0.0f, p, y);
            // only fwd movement
            Vector3f v_bf = dcm.transposed() * velocity_ef;
            v_bf.y = 0.0f;
            if (v_bf.x < 0.0f) {
                v_bf.x = 0.0f;
            }

            Vector3f gnd_movement_bf = dcm.transposed() * gnd_movement;

            // lateral speed equals ground movement
            v_bf.y = gnd_movement_bf.y;

            if (!gnd_movement_bf.is_zero()) {
                // fwd speed slowly approaches ground movement to simulate wheel friction
                const float tconst = 20; // seconds
                const float alpha = delta_time/(delta_time+tconst/M_2PI);
                v_bf.x += (gnd_movement.x - v_bf.x) * alpha;
            }

            velocity_ef = dcm * v_bf;
            if (velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
            gyro.zero();
            gyro.z = yaw_rate;
            use_smoothing = true;
            break;
        }
        case GROUND_BEHAVIOR_TAILSITTER: {
            // rotate normal refernce frame to get yaw angle, then rotate back
            Matrix3f rot;
            rot.from_rotation(ROTATION_PITCH_270);
            float r, p, y;
            (dcm * rot).to_euler(&r, &p, &y);
            y = y + yaw_rate * delta_time;
            dcm.from_euler(0.0, 0.0, y);
            rot.from_rotation(ROTATION_PITCH_90);
            dcm *= rot;
            // X, Y movement tracks ground movement
            velocity_ef.x = gnd_movement.x;
            velocity_ef.y = gnd_movement.y;
            if (velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
            gyro.zero();
            gyro.x = yaw_rate;
            use_smoothing = true;
            break;
        }
        }
    }

    // update slung payload
#if AP_SIM_SLUNGPAYLOAD_ENABLED
    sitl->models.slung_payload_sim.update(get_position_relhome(), velocity_ef, accel_earth, wind_ef);
#endif

    // update tether
#if AP_SIM_TETHER_ENABLED
    sitl->models.tether_sim.update(location);
#endif

    // allow for changes in physics step
    adjust_frame_time(constrain_float(sitl->loop_rate_hz, rate_hz-1, rate_hz+1));
}

/*
  update wind vector
*/
void Aircraft::update_wind(const struct sitl_input &input)
{
    // wind vector in earth frame
    wind_ef = Vector3f(cosf(radians(input.wind.direction))*cosf(radians(input.wind.dir_z)), 
                       sinf(radians(input.wind.direction))*cosf(radians(input.wind.dir_z)), 
                       sinf(radians(input.wind.dir_z))) * input.wind.speed;

    wind_ef.z += get_local_updraft(position + home.get_distance_NED_double(origin));

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

    // the AHRS wants wind with opposite sense
    wind_ef = -wind_ef;
}

/*
  smooth sensors for kinematic consistancy when we interact with the ground
 */
void Aircraft::smooth_sensors(void)
{
    uint64_t now = time_now_us;
    Vector3d delta_pos = position - smoothing.position;
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
    if (delta_time < 0 || delta_time > 0.1) {
        return;
    }

    // calculate required accel to get us to desired position and velocity in the time_constant
    const float time_constant = 0.1f;
    Vector3f dvel = (velocity_ef - smoothing.velocity_ef) + (delta_pos / time_constant).tofloat();
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
// @LoggerMessage: SMOO
// @Description: Smoothed sensor data fed to EKF to avoid inconsistencies
// @Field: TimeUS: Time since system startup
// @Field: AEx: Angular Velocity (around x-axis)
// @Field: AEy: Angular Velocity (around y-axis)
// @Field: AEz: Angular Velocity (around z-axis)
// @Field: DPx: Velocity (along x-axis)
// @Field: DPy: Velocity (along y-axis)
// @Field: DPz: Velocity (along z-axis)
// @Field: R: Roll
// @Field: P: Pitch
// @Field: Y: Yaw
// @Field: R2: DCM Roll
// @Field: P2: DCM Pitch
// @Field: Y2: DCM Yaw
    AP::logger().WriteStreaming("SMOO", "TimeUS,AEx,AEy,AEz,DPx,DPy,DPz,R,P,Y,R2,P2,Y2",
                                           "Qffffffffffff",
                                           AP_HAL::micros64(),
                                           degrees(angle_differential.x),
                                           degrees(angle_differential.y),
                                           degrees(angle_differential.z),
                                           delta_pos.x, delta_pos.y, delta_pos.z,
                                           degrees(R), degrees(P), degrees(Y),
                                           degrees(R2), degrees(P2), degrees(Y2));
#endif


    // integrate the chase in sub-steps of <=10ms: explicit Euler on this
    // 0.1s time constant is badly underdamped at ~40ms stretched frames, and
    // the ringing sensor state threw a hovering copter down from metres up
    const uint32_t smooth_nsub =
        constrain_int32((int32_t)ceilf(delta_time / 0.010f), 1, 8);
    const float dt_smooth = delta_time / smooth_nsub;
    for (uint32_t i = 0; i < smooth_nsub; i++) {
        // integrate to get new attitude
        smoothing.rotation_b2e.rotate(smoothing.gyro * dt_smooth);
        smoothing.rotation_b2e.normalize();

        // integrate to get new position
        smoothing.velocity_ef += accel_e * dt_smooth;
        smoothing.position +=
            (smoothing.velocity_ef * dt_smooth).todouble();
    }

    smoothing.location = origin;
    smoothing.location.offset(smoothing.position.x, smoothing.position.y);
    smoothing.location.alt  = static_cast<int32_t>(home.alt - smoothing.position.z * 100.0f);

    smoothing.last_update_us = now;
}

/*
  return a filtered servo input as a value from -1 to 1
  servo is assumed to be 1000 to 2000, trim at 1500
 */
float Aircraft::filtered_servo_angle(const struct sitl_input &input, uint8_t idx)
{
    uint16_t pwm =  input.servos[idx];
    if (pwm == 0) {
        // invalid input, servo does not move, apply current value.
        // glad we have a zero-momentum system.
        pwm = servo_filter[idx].angle_pwm();
    }
    // filter the actuators with the frame's ACTUAL timestep, not the
    // nominal one: at nominal rate a stretched frame made every servo and
    // motor respond several times slower in sim time - enough to bounce
    return servo_filter[idx].filter_angle(
        pwm, (effective_frame_time_us != 0 ?
              effective_frame_time_us : frame_time_us) * 1.0e-6);
}

/*
  return a filtered servo input as a value from 0 to 1
  servo is assumed to be 1000 to 2000
 */
float Aircraft::filtered_servo_range(const struct sitl_input &input, uint8_t idx)
{
    return servo_filter[idx].filter_range(
        input.servos[idx], (effective_frame_time_us != 0 ?
                            effective_frame_time_us : frame_time_us) * 1.0e-6);
}

// setup filtering for servo
void Aircraft::filtered_servo_setup(uint8_t idx, uint16_t pwm_min, uint16_t pwm_max, float deflection_deg)
{
    servo_filter[idx].set_pwm_range(pwm_min, pwm_max);
    servo_filter[idx].set_deflection(deflection_deg);
}

// extrapolate sensors by a given delta time in seconds
void Aircraft::extrapolate_sensors(float delta_time)
{
    Vector3f accel_earth = dcm * accel_body;
    accel_earth.z += GRAVITY_MSS;

    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0,0,-GRAVITY_MSS));

    // new velocity and position vectors
    velocity_ef += accel_earth * delta_time;
    position += (velocity_ef * delta_time).todouble();
    velocity_air_ef = velocity_ef - wind_ef;
    velocity_air_bf = dcm.transposed() * velocity_air_ef;
}

bool Aircraft::Clamp::clamped(Aircraft &aircraft, const struct sitl_input &input)
{
    const auto clamp_ch = AP::sitl()->clamp_ch;
    if (clamp_ch < 1) {
        return false;
    }
    const uint32_t clamp_idx = clamp_ch - 1;
    if (clamp_idx > ARRAY_SIZE(input.servos)) {
        return false;
    }
    const uint16_t servo_pos = input.servos[clamp_idx];
    bool new_clamped = currently_clamped;
    if (servo_pos == 0) {
        // invalid value, do nothing
    } else if (servo_pos < 1200) {
        if (currently_clamped) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SITL: Clamp: released vehicle");
            new_clamped = false;
        }
        grab_attempted = false;
    } else {
        // re-clamp if < 10cm from home
        if (servo_pos > 1800 && !grab_attempted) {
            const Vector3d pos = aircraft.get_position_relhome();
            const float distance_from_home = pos.length();
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SITL: Clamp: dist=%f", distance_from_home);
            if (distance_from_home < 0.5) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SITL: Clamp: grabbed vehicle");
                new_clamped = true;
            } else if (!grab_attempted) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SITL: Clamp: missed vehicle");
            }
            grab_attempted = true;
        }
    }

    currently_clamped = new_clamped;

    return currently_clamped;
}

// simple battery consumption model
// does not support the behavior documented by SIM_BATT_VOLTAGE and SIM_BATT_CAP_AH parameters
void Aircraft::update_battery()
{
    // lose 0.7V at full throttle (from the user-specified voltage)
    battery_voltage = sitl->batt_voltage - 0.7f * fabsf(sitl->throttle);
    // assume 50A at full throttle
    battery_current = 50.0f * fabsf(sitl->throttle);
}

void Aircraft::update_battery(const struct sitl_input &input)
{
    update_battery();
}

void Aircraft::update_external_payload(const struct sitl_input &input)
{
    external_payload_mass = 0;

#if AP_SIM_SPRAYER_ENABLED
    // update sprayer
    if (sitl->sprayer_sim.is_enabled()) {
        sitl->sprayer_sim.update(input);
        external_payload_mass += sitl->sprayer_sim.payload_mass();
    }
#endif  // AP_SIM_SPRAYER_ENABLED

    {
        const float range = rangefinder_range();
        if (!isinf(range) && range > 100000) {
            AP_HAL::panic("Bad rangefinder calculation");
        }
        for (uint8_t i=0; i<ARRAY_SIZE(rangefinder_m); i++) {
            rangefinder_m[i] = range;
        }
    }

    // update i2c
    sitl->i2c_sim.update(*this);

#if AP_SIM_BUZZER_ENABLED
    // update buzzer
    if (sitl->buzzer_sim.is_enabled()) {
        sitl->buzzer_sim.update(input);
    }
#endif  // AP_SIM_BUZZER_ENABLED

    // update grippers
#if AP_SIM_GRIPPER_ENABLED
    if (sitl->gripper_sim.is_enabled()) {
        sitl->gripper_sim.set_alt(hagl());
        sitl->gripper_sim.update(input);
        external_payload_mass += sitl->gripper_sim.payload_mass();
    }
#endif  // AP_SIM_GRIPPER_ENABLED
#if AP_SIM_GRIPPER_EPM_ENABLED
    if (sitl->gripper_epm_sim.is_enabled()) {
        sitl->gripper_epm_sim.update(input);
        external_payload_mass += sitl->gripper_epm_sim.payload_mass();
    }
#endif  // AP_SIM_GRIPPER_EPM_ENABLED

#if AP_SIM_PARACHUTE_ENABLED
    // update parachute
    if (sitl->parachute_sim.is_enabled()) {
        sitl->parachute_sim.update(input);
        // TODO: add drag to vehicle, presumably proportional to velocity
    }
#endif  // AP_SIM_PARACHUTE_ENABLED

#if AP_SIM_PRECLAND_ENABLED
    // update precland
    if (sitl->precland_sim.is_enabled()) {
        sitl->precland_sim.update(get_location());
        if (sitl->precland_sim._over_precland_base) {
            local_ground_level += sitl->precland_sim._device_height;
        }
    }
#endif  // AP_SIM_PRECLAND_ENABLED

    // update RichenPower generator
    if (richenpower) {
        richenpower->update(input);
    }

#if AP_SIM_LOWEHEISER_ENABLED
    // update Loweheiser generator
    if (loweheiser) {
        loweheiser->update(*this);
    }
#endif

    if (fetteconewireesc) {
        fetteconewireesc->update(*this);
    }

#if AP_SIM_VOLZ_ENABLED
    if (volz) {
        volz->update(*this);
    }
#endif  // AP_SIM_VOLZ_ENABLED

#if AP_SIM_SHIP_ENABLED
    sitl->models.shipsim.update();
#endif

    // update IntelligentEnergy 2.4kW generator
    if (ie24) {
        ie24->update(input);
    }

#if AP_TEST_DRONECAN_DRIVERS
    sitl->dronecan_sim.update();
#endif

#if AP_SIM_GPIO_LED_1_ENABLED
    sim_led1.update(*this);
#endif
#if AP_SIM_GPIO_LED_2_ENABLED
    sim_led2.update(*this);
#endif
#if AP_SIM_GPIO_LED_3_ENABLED
    sim_led3.update(*this);
#endif
#if AP_SIM_GPIO_LED_RGB_ENABLED
    sim_ledrgb.update(*this);
#endif

#if AP_SIM_MOUNT_ENABLED
    for (uint8_t i = 0; i < GIMBAL_SIM_MAX; i++) {
        if (gimbal_sims[i] != nullptr) {
            gimbal_sims[i]->update(*this);
        }
    }
#endif
}

void Aircraft::add_shove_forces(Vector3f &rot_accel, Vector3f &body_accel)
{
    const uint32_t now = AP_HAL::millis();
    if (sitl == nullptr) {
        return;
    }
    if (sitl->shove.t == 0) {
        return;
    }
    if (sitl->shove.start_ms == 0) {
        sitl->shove.start_ms = now;
    }
    if (now - sitl->shove.start_ms < uint32_t(sitl->shove.t)) {
        // FIXME: can we get a vector operation here instead?
        body_accel.x += sitl->shove.x;
        body_accel.y += sitl->shove.y;
        body_accel.z += sitl->shove.z;
    } else {
        sitl->shove.start_ms = 0;
        // save as well as set: the parameter was written to storage to
        // ask for the shove, so clearing only the live value leaves
        // storage still asking for one.  A GCS - or the test suite
        // putting parameters back after a test - then sees the live
        // value already at zero and has no reason to write, and the
        // next reboot loads the old duration and shoves again.
        sitl->shove.t.set_and_save(0);
    }
}

float Aircraft::get_local_updraft(const Vector3d &currentPos)
{
    int scenario = sitl->thermal_scenario;

    #define MAX_THERMALS 10

    float thermals_w[MAX_THERMALS];
    float thermals_r[MAX_THERMALS];
    float thermals_x[MAX_THERMALS];
    float thermals_y[MAX_THERMALS];

    int n_thermals = 0;

    switch (scenario) {
        case 0:
            return 0;
        case 1:
            n_thermals = 1;
            thermals_w[0] =  2.0;
            thermals_r[0] =  80.0;
            thermals_x[0] = -180.0;
            thermals_y[0] = -260.0;
            break;
        case 2:
            n_thermals = 1;
            thermals_w[0] =  4.0;
            thermals_r[0] =  30.0;
            thermals_x[0] = -180.0;
            thermals_y[0] = -260.0;
            break;
        case 3:
            n_thermals = 1;
            thermals_w[0] =  2.0;
            thermals_r[0] =  30.0;
            thermals_x[0] = -180.0;
            thermals_y[0] = -260.0;
            break;
        case 4:
            n_thermals = 1;
            thermals_w[0] =  5.0;
            thermals_r[0] =  30.0;
            thermals_x[0] =  0;
            thermals_y[0] =  0;
            break;
        default:
            AP_BoardConfig::config_error("Bad thermal scenario");
    }

    // Wind drift at this altitude
    float driftX = sitl->wind_speed * (currentPos.z+100) * cosf(sitl->wind_direction * DEG_TO_RAD);
    float driftY = sitl->wind_speed * (currentPos.z+100) * sinf(sitl->wind_direction * DEG_TO_RAD);

    int iThermal;
    float w = 0.0f;
    float r2;
    for (iThermal=0;iThermal<n_thermals;iThermal++) {
        Vector3d thermalPos(thermals_x[iThermal] + driftX/thermals_w[iThermal],
                            thermals_y[iThermal] + driftY/thermals_w[iThermal],
                            0);
        Vector3d relVec = currentPos - thermalPos;
        r2 = relVec.x*relVec.x + relVec.y*relVec.y;
        w += thermals_w[iThermal]*exp(-r2/(thermals_r[iThermal]*thermals_r[iThermal]));
    }

    return w;
}

void Aircraft::add_twist_forces(Vector3f &rot_accel)
{
    if (sitl == nullptr) {
        return;
    }
    if (sitl->gnd_behav != -1) {
        ground_behavior = (GroundBehaviour)sitl->gnd_behav.get();
    }
    const uint32_t now = AP_HAL::millis();
    if (sitl == nullptr) {
        return;
    }
    if (sitl->twist.t == 0) {
        return;
    }
    if (sitl->twist.start_ms == 0) {
        sitl->twist.start_ms = now;
    }
    if (now - sitl->twist.start_ms < uint32_t(sitl->twist.t)) {
        // FIXME: can we get a vector operation here instead?
        rot_accel.x += sitl->twist.x;
        rot_accel.y += sitl->twist.y;
        rot_accel.z += sitl->twist.z;
    } else {
        sitl->twist.start_ms = 0;
        sitl->twist.t.set(0);
    }
}

// add body-frame force due to slung payload and tether
void Aircraft::add_external_forces(Vector3f &body_accel)
{
    Vector3f total_force;
#if AP_SIM_SLUNGPAYLOAD_ENABLED
    Vector3f forces_ef_slung;
    sitl->models.slung_payload_sim.get_forces_on_vehicle(forces_ef_slung);
    total_force += forces_ef_slung;
#endif

#if AP_SIM_TETHER_ENABLED
    Vector3f forces_ef_tether;
    sitl->models.tether_sim.get_forces_on_vehicle(forces_ef_tether);
    total_force += forces_ef_tether;
#endif

    // convert ef forces to body-frame accelerations (acceleration = force / mass)
    const Vector3f accel_bf_tether = dcm.transposed() * total_force / mass;
    body_accel += accel_bf_tether;
}

/*
  get position relative to home
 */
Vector3d Aircraft::get_position_relhome() const
{
    Vector3d pos = position;
    pos.xy() += home.get_distance_NE_double(origin);
    return pos;
}

// get air density in kg/m^3
float Aircraft::get_air_density(float alt_amsl) const
{
    return AP_Baro::get_air_density_for_alt_amsl(alt_amsl);
}

/*
  update EAS airspeed and pitot speed
 */
void Aircraft::update_eas_airspeed()
{
    airspeed = velocity_air_ef.length() / eas2tas;

    /*
      airspeed as seen by a fwd pitot tube (limited to 120m/s)
    */
    airspeed_pitot = airspeed;

    // calculate angle between the local flow vector and a pitot tube aligned with the X body axis
    const float pitot_aoa =  atan2f(sqrtf(sq(velocity_air_bf.y) + sq(velocity_air_bf.z)), velocity_air_bf.x);

    /*
      assume the pitot can correctly capture airspeed up to 20 degrees off the nose
      and follows a cose law outside that range
    */
    const float max_pitot_aoa = radians(20);
    if (pitot_aoa > radians(90)) {
        airspeed_pitot = 0;
    } else if (pitot_aoa > max_pitot_aoa) {
        const float gain_factor = M_PI_2 / (radians(90) - max_pitot_aoa);
        airspeed_pitot *= cosf((pitot_aoa - max_pitot_aoa) * gain_factor);
    }
}

/*
  set pose on the aircraft, called from scripting
 */
bool Aircraft::set_pose(uint8_t instance, const Location &loc, const Quaternion &quat, const Vector3f &velocity_ef, const Vector3f &gyro_rads)
{
    if (instance >= MAX_SIM_INSTANCES || instances[instance] == nullptr) {
        return false;
    }
    auto &aircraft = *instances[instance];
    WITH_SEMAPHORE(aircraft.pose_sem);

    quat.rotation_matrix(aircraft.dcm);
    aircraft.velocity_ef = velocity_ef;
    aircraft.location = loc;
    aircraft.position = aircraft.home.get_distance_NED_double(loc);
    aircraft.smoothing.position = aircraft.position;
    aircraft.smoothing.rotation_b2e = aircraft.dcm;
    aircraft.smoothing.velocity_ef = velocity_ef;
    aircraft.smoothing.location = loc;
    aircraft.gyro = gyro_rads;

    return true;
}

/*
  wrapper for scripting access
 */
bool SITL::SIM::set_pose(uint8_t instance, const Location &loc, const Quaternion &quat, const Vector3f &velocity_ef, const Vector3f &gyro_rads)
{
    return Aircraft::set_pose(instance, loc, quat, velocity_ef, gyro_rads);
}

