/*
 * File:          ugv_teleop.c
 * Description:   Keyboard teleop for the ugv_rover world: front wheels steer
 *                (position control), rear wheels drive (velocity control).
 *                Webots has no mechanical differential node, so the rear
 *                wheel speeds are derived kinematically from a bicycle-model
 *                turn radius (steering angle + wheelbase/track), giving the
 *                inner wheel a lower speed and the outer wheel a higher one
 *                while turning, instead of commanding both wheels identically.
 *                GPS/IMU/distance-sensor readings, plus
 *                the commanded steering angle, the *actual* steering angle
 *                measured by the steering-joint position sensors, and both
 *                rear wheel speeds are printed once per second and logged
 *                every simulation step to ugv_teleop_log.csv (next to this
 *                controller) so oscillation/overshoot can be inspected after
 *                a driving session, not just guessed at from the console.
 *
 *                Like a real car, this only turns while moving - there is
 *                no in-place pivot the way skid-steer had.
 *
 *                Sensor reading and motor writing are kept in their own
 *                functions so the keyboard layer can later be swapped for
 *                an ArduPilot SITL socket bridge without touching the rest.
 *
 *                The CSV log carries the full rover state every step (not
 *                just steering/wheel speed) so events like driving into an
 *                obstacle and getting stuck show up as a frozen pos/heading
 *                trace, not just something you had to catch live in the
 *                console.
 *
 *                Three drive modes, switched with dedicated keys (edge
 *                triggered, so holding the key doesn't spam the switch):
 *                  'M' - MANUAL: arrow keys drive directly.
 *                  'A' - AUTONOMOUS: autonomous_decide() closes the loop
 *                        itself from ds_left/ds_right telemetry (drive
 *                        forward when clear, steer away from whichever
 *                        side reads closer, reverse-and-turn if boxed in),
 *                        with a stuck-detection fallback that watches
 *                        actual GPS displacement and forces a reverse/turn
 *                        recovery if it isn't moving despite trying to -
 *                        this runs at full simulation rate inside the
 *                        controller, so unlike REMOTE mode there is no
 *                        external round-trip delay.
 *                  'R' - REMOTE: drive/steer are read from a plain text
 *                        command file (rover_command.txt, next to this
 *                        controller) instead of the keyboard. An external
 *                        process - or a human editing the file by hand -
 *                        can write "drive steer" (each in [-1,1]) to it at
 *                        any time; the last value written just holds until
 *                        a new one is written. Combined with reading
 *                        ugv_teleop_log.csv back, this is a real (if
 *                        turn-based rather than continuous) send-command /
 *                        read-telemetry loop.
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/distance_sensor.h>
#include <webots/radar.h>
#include <webots/camera.h>
#include <webots/keyboard.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 16
#define MAX_WHEEL_SPEED 8.0   // rad/s baseline; with the 0.229 m (18 in) wheels this is ~1.8 m/s

// Runtime speed multiplier adjusted by PageUp/'+' and PageDown/'-'. Capped so
// MAX_WHEEL_SPEED * scale stays within the drive motors' 20 rad/s maxVelocity.
#define SPEED_SCALE_STEP 0.2
#define SPEED_SCALE_MIN 0.2
#define SPEED_SCALE_MAX 2.5
#define MAX_STEER_ANGLE 0.575  // rad (~33 deg), matches the front HingeJoints' minStop/maxStop
#define PRINT_PERIOD_MS 1000
#define RAD2DEG (180.0 / M_PI)

// Must match the wheel anchor positions in the .wbt: front anchors at x=+-0.60
// and rear at x=-0.60 -> wheelbase 1.20; left/right anchors at y=+-0.52 ->
// track 1.04 (scaled rover: 1.83 x 1.22 x 0.61 m body, 18 in wheels).
#define WHEELBASE 1.20
#define TRACK_WIDTH 1.10

// Autonomous obstacle-avoidance thresholds, in meters. Scaled up for the
// ~1.8 m-long vehicle and the 8 m sensor range: the old 0.25 m stop distance
// was shorter than the vehicle itself.
#define OBSTACLE_NEAR_M 2.5   // start steering away from the closer side
#define OBSTACLE_STOP_M 1.2   // too close on both sides - reverse and turn instead

#define RADAR_MAX_RANGE 8.0  // must match maxRange on the corner Radar nodes in the .wbt

// Stuck detection: range thresholds alone don't catch "wedged against
// something and not actually moving". We use the GPS's own speed reading
// (wb_gps_get_speed, m/s) rather than integrating position, because in the
// OSM world the GPS is in WGS84 mode and returns lat/lon *degrees* - the old
// position-displacement check compared those tiny degree deltas to a metre
// threshold and so ALWAYS read "stuck", trapping the rover in perpetual
// recovery even on completely open ground. Speed in m/s is the same in both
// coordinate systems, so this works in the arena and the real-map world.
#define STUCK_CHECK_PERIOD_S 1.2   // must be commanding drive but slow this long to count as stuck
#define STUCK_SPEED_M_S 0.15       // below this ground speed (while trying to drive) = not moving

// Recovery commitment: logged evidence of the failure mode this fixes -
// re-deciding "reverse or not" fresh every 16ms step let the rover chatter
// between reverse and forward 10+ times a second at a wall corner (position
// pinned the whole time), because tiny sensor fluctuations while reversing
// kept crossing the OBSTACLE_STOP_M threshold back and forth. Once a
// recovery starts (from either the proximity check or the stuck-displacement
// check), it now commits to reverse-and-turn for a minimum duration
// regardless of momentary readings, and the stuck-check is suspended while
// a recovery is already active so it can't re-trigger mid-maneuver and flip
// the escape direction. Repeated recoveries in the same spot escalate the
// duration so a stubborn corner gets backed away from further each attempt
// instead of retrying the same too-short maneuver forever.
#define RECOVERY_BASE_S 1.5             // commitment duration per escalation level
#define RECOVERY_MAX_LEVEL 3            // cap: level 3 = 4.5s single commitment
#define RECOVERY_ESCALATION_WINDOW_S 6.0  // re-stuck within this long after a recovery counts as still-stuck

#define COMMAND_FILE "rover_command.txt"

// Steering PID auto-tune (relay-feedback / Astrom-Hagglund method), triggered
// by the 'T' key. The steering motor is briefly switched to torque control
// and driven with a relay (bang-bang) around zero; the resulting limit-cycle
// amplitude a and period Tu give the ultimate gain Ku = 4*d/(pi*a), from
// which Ziegler-Nichols yields P/I/D. The tuned gains are applied live via
// wb_motor_set_control_pid() to both steering motors and printed.
#define AUTOTUNE_RELAY_TORQUE 250.0  // N*m relay amplitude: above standstill tyre scrub, well under 2000 N*m maxTorque
#define AUTOTUNE_MAX_S 12.0          // give up if no clean limit cycle by now
#define AUTOTUNE_CYCLES 6            // measure this many half-cycles before computing gains

typedef enum { MODE_MANUAL, MODE_AUTONOMOUS, MODE_REMOTE } DriveMode;

static const char *mode_name(DriveMode mode) {
  switch (mode) {
    case MODE_AUTONOMOUS:
      return "AUTONOMOUS";
    case MODE_REMOTE:
      return "REMOTE";
    default:
      return "MANUAL";
  }
}

typedef struct {
  WbDeviceTag steer_fl;
  WbDeviceTag steer_fr;
  WbDeviceTag wheel_rl;
  WbDeviceTag wheel_rr;
} Motors;

typedef struct {
  WbDeviceTag gps;
  WbDeviceTag imu;
  WbDeviceTag ds_left;
  WbDeviceTag ds_right;
  WbDeviceTag steer_fl_sensor;
  WbDeviceTag steer_fr_sensor;
  WbDeviceTag radar_fl;
  WbDeviceTag radar_fr;
  WbDeviceTag radar_rl;
  WbDeviceTag radar_rr;
  WbDeviceTag camera;      // front FPV camera
  WbDeviceTag map_camera;  // top-down overhead "map" camera
} Sensors;

typedef struct {
  double pos[3];
  double heading_deg;
  double speed;  // GPS ground speed (m/s), coordinate-system independent
  double ds_left;
  double ds_right;
  double radar_fl;
  double radar_fr;
  double radar_rl;
  double radar_rr;
} RoverState;

typedef struct {
  int in_recovery;
  double recovery_until_time;
  double recovery_steer_sign;

  int consecutive_recoveries;
  double last_recovery_end_time;

  double low_speed_since;  // time we first went slow while trying to drive; -1 = not currently
} AutonomousState;

static void autonomous_state_init(AutonomousState *as) {
  as->in_recovery = 0;
  as->recovery_until_time = -1.0;
  as->recovery_steer_sign = 1.0;
  as->consecutive_recoveries = 0;
  as->last_recovery_end_time = -1000.0;
  as->low_speed_since = -1.0;
}

// Starts (or escalates) a committed reverse-and-turn recovery. Repeated
// triggers within RECOVERY_ESCALATION_WINDOW_S of the last one ending push
// the escalation level up (capped), giving each retry more time to actually
// clear the obstacle instead of repeating the same too-short maneuver.
static void start_recovery(AutonomousState *as, double now_s, double steer_sign) {
  if (now_s - as->last_recovery_end_time < RECOVERY_ESCALATION_WINDOW_S)
    as->consecutive_recoveries++;
  else
    as->consecutive_recoveries = 1;
  if (as->consecutive_recoveries > RECOVERY_MAX_LEVEL)
    as->consecutive_recoveries = RECOVERY_MAX_LEVEL;

  as->in_recovery = 1;
  as->recovery_until_time = now_s + RECOVERY_BASE_S * as->consecutive_recoveries;
  as->recovery_steer_sign = steer_sign;
}

static void motors_init(Motors *m) {
  m->steer_fl = wb_robot_get_device("steer_fl");
  m->steer_fr = wb_robot_get_device("steer_fr");
  m->wheel_rl = wb_robot_get_device("wheel_rl");
  m->wheel_rr = wb_robot_get_device("wheel_rr");

  // rear drive wheels: infinite target position puts them in pure velocity mode
  wb_motor_set_position(m->wheel_rl, INFINITY);
  wb_motor_set_position(m->wheel_rr, INFINITY);
  wb_motor_set_velocity(m->wheel_rl, 0.0);
  wb_motor_set_velocity(m->wheel_rr, 0.0);

  // front steering motors stay in the default position-control mode
  wb_motor_set_position(m->steer_fl, 0.0);
  wb_motor_set_position(m->steer_fr, 0.0);
}

static void sensors_init(Sensors *s) {
  s->gps = wb_robot_get_device("gps");
  s->imu = wb_robot_get_device("imu");
  s->ds_left = wb_robot_get_device("ds_left");
  s->ds_right = wb_robot_get_device("ds_right");
  s->steer_fl_sensor = wb_robot_get_device("steer_fl_sensor");
  s->steer_fr_sensor = wb_robot_get_device("steer_fr_sensor");
  s->radar_fl = wb_robot_get_device("radar_fl");
  s->radar_fr = wb_robot_get_device("radar_fr");
  s->radar_rl = wb_robot_get_device("radar_rl");
  s->radar_rr = wb_robot_get_device("radar_rr");
  s->camera = wb_robot_get_device("camera");
  s->map_camera = wb_robot_get_device("map_camera");

  wb_gps_enable(s->gps, TIME_STEP);
  wb_inertial_unit_enable(s->imu, TIME_STEP);
  wb_distance_sensor_enable(s->ds_left, TIME_STEP);
  wb_distance_sensor_enable(s->ds_right, TIME_STEP);
  wb_position_sensor_enable(s->steer_fl_sensor, TIME_STEP);
  wb_position_sensor_enable(s->steer_fr_sensor, TIME_STEP);
  wb_radar_enable(s->radar_fl, TIME_STEP);
  wb_radar_enable(s->radar_fr, TIME_STEP);
  wb_radar_enable(s->radar_rl, TIME_STEP);
  wb_radar_enable(s->radar_rr, TIME_STEP);
  // Enabling the cameras makes Webots show their live feeds as overlays in
  // the 3D view: "camera" = front FPV, "map_camera" = top-down minimap.
  wb_camera_enable(s->camera, TIME_STEP);
  wb_camera_enable(s->map_camera, TIME_STEP);
}

// Closest target's distance on a radar, or RADAR_MAX_RANGE if it sees nothing
// (mirrors how ds_left/ds_right read "3.00" - their max range - when clear).
static double read_radar_min_distance(WbDeviceTag radar) {
  int n = wb_radar_get_number_of_targets(radar);
  if (n <= 0)
    return RADAR_MAX_RANGE;

  const WbRadarTarget *targets = wb_radar_get_targets(radar);
  double min_distance = targets[0].distance;
  for (int i = 1; i < n; i++) {
    if (targets[i].distance < min_distance)
      min_distance = targets[i].distance;
  }
  return min_distance;
}

// Reads all pending key events this step and turns them into a manual
// drive/steer command, each in [-1, 1], plus an edge-triggered mode
// request: *requested_mode is set to the newly-pressed mode key's mode
// (M/A/R just went down this step, not just "is held"), or left unchanged
// (still whatever it was) if no mode key was pressed this step. Callers
// should check requested_mode against the current mode before switching.
static void read_keyboard_command(double *drive, double *steer, DriveMode *requested_mode, int *mode_requested,
                                   int *autotune_requested, int *speed_adjust) {
  static int m_was_down = 0, a_was_down = 0, r_was_down = 0, t_was_down = 0, up_was_down = 0, dn_was_down = 0;
  int m_down = 0, a_down = 0, r_down = 0, t_down = 0, spd_up = 0, spd_dn = 0;

  *drive = 0.0;
  *steer = 0.0;
  *mode_requested = 0;
  *autotune_requested = 0;
  *speed_adjust = 0;

  int key = wb_keyboard_get_key();
  while (key >= 0) {
    // Mask off SHIFT/CONTROL/ALT bits so e.g. '+' (shift + '=') still matches.
    int k = key & WB_KEYBOARD_KEY;
    switch (k) {
      case WB_KEYBOARD_UP:
        *drive += 1.0;
        break;
      case WB_KEYBOARD_DOWN:
        *drive -= 1.0;
        break;
      case WB_KEYBOARD_LEFT:
        *steer += 1.0;
        break;
      case WB_KEYBOARD_RIGHT:
        *steer -= 1.0;
        break;
      case 'M':
        m_down = 1;
        break;
      case 'A':
        a_down = 1;
        break;
      case 'R':
        r_down = 1;
        break;
      case 'T':
        t_down = 1;
        break;
      case WB_KEYBOARD_PAGEUP:
      case '+':
      case '=':  // unshifted '+' key
        spd_up = 1;
        break;
      case WB_KEYBOARD_PAGEDOWN:
      case '-':
      case '_':
        spd_dn = 1;
        break;
      default:
        break;
    }
    key = wb_keyboard_get_key();
  }

  if (m_down && !m_was_down) {
    *requested_mode = MODE_MANUAL;
    *mode_requested = 1;
  } else if (a_down && !a_was_down) {
    *requested_mode = MODE_AUTONOMOUS;
    *mode_requested = 1;
  } else if (r_down && !r_was_down) {
    *requested_mode = MODE_REMOTE;
    *mode_requested = 1;
  }
  *autotune_requested = (t_down && !t_was_down);
  if (spd_up && !up_was_down)
    *speed_adjust = 1;
  else if (spd_dn && !dn_was_down)
    *speed_adjust = -1;

  m_was_down = m_down;
  a_was_down = a_down;
  r_was_down = r_down;
  t_was_down = t_down;
  up_was_down = spd_up;
  dn_was_down = spd_dn;

  if (*drive > 1.0)
    *drive = 1.0;
  if (*drive < -1.0)
    *drive = -1.0;
  if (*steer > 1.0)
    *steer = 1.0;
  if (*steer < -1.0)
    *steer = -1.0;
}

// Relay-feedback (Astrom-Hagglund) auto-tune of the front steering motors'
// position PID. Briefly switches both steer motors to torque control and
// drives a bang-bang relay around zero angle: the motor swings one way,
// overshoots, the relay flips, and a self-sustaining limit cycle forms. Its
// amplitude a and period Tu characterise the plant. From the ultimate gain
// Ku = 4*d/(pi*a) (d = relay torque), Ziegler-Nichols gives PID gains, which
// are applied live to both steer motors and printed. Runs its own step loop
// (it needs many steps) and blocks normal driving until it finishes.
static void run_steering_autotune(const Motors *m, const Sensors *s) {
  printf("autotune: starting relay-feedback steering tune (rover will wiggle its front wheels)...\n");
  fflush(stdout);

  double t0 = wb_robot_get_time();
  double relay = AUTOTUNE_RELAY_TORQUE;
  double sign = 1.0;
  double pos = wb_position_sensor_get_value(s->steer_fl_sensor);
  int prev_side = (pos >= 0.0) ? 1 : -1;

  double peak_pos = 0.0, peak_neg = 0.0;  // running extremes within the current swing
  double last_cross_t = -1.0;
  double period_sum = 0.0, amp_sum = 0.0;
  int period_n = 0, amp_n = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    double now = wb_robot_get_time();
    pos = wb_position_sensor_get_value(s->steer_fl_sensor);

    // Relay: push toward zero; when we cross zero the drive direction flips,
    // sustaining an oscillation about the setpoint.
    sign = (pos > 0.0) ? -1.0 : 1.0;
    wb_motor_set_torque(m->steer_fl, sign * relay);
    wb_motor_set_torque(m->steer_fr, sign * relay);

    if (pos > peak_pos)
      peak_pos = pos;
    if (pos < peak_neg)
      peak_neg = pos;

    int side = (pos >= 0.0) ? 1 : -1;
    if (side != prev_side) {  // zero crossing = half-cycle boundary
      if (last_cross_t > 0.0) {
        period_sum += 2.0 * (now - last_cross_t);  // full period = 2 half-cycles
        period_n++;
        amp_sum += (peak_pos - peak_neg);  // peak-to-peak of the swing just finished
        amp_n++;
      }
      last_cross_t = now;
      peak_pos = 0.0;
      peak_neg = 0.0;
      prev_side = side;
    }

    if (period_n >= AUTOTUNE_CYCLES || (now - t0) > AUTOTUNE_MAX_S)
      break;
  }

  // Restore position control (hold straight) regardless of outcome.
  wb_motor_set_position(m->steer_fl, 0.0);
  wb_motor_set_position(m->steer_fr, 0.0);

  if (period_n < 3 || amp_n < 3) {
    printf("autotune: no clean limit cycle formed - keeping existing PID. Try adjusting AUTOTUNE_RELAY_TORQUE.\n");
    fflush(stdout);
    return;
  }

  double Tu = period_sum / period_n;
  double a = 0.5 * (amp_sum / amp_n);  // amplitude = half of peak-to-peak
  double Ku = (4.0 * relay) / (M_PI * a);

  // Ziegler-Nichols "classic" PID.
  double Kp = 0.6 * Ku;
  double Ti = 0.5 * Tu;
  double Td = 0.125 * Tu;
  double Ki = Kp / Ti;
  double Kd = Kp * Td;

  wb_motor_set_control_pid(m->steer_fl, Kp, Ki, Kd);
  wb_motor_set_control_pid(m->steer_fr, Kp, Ki, Kd);

  printf("autotune done: Tu=%.3fs a=%.4frad Ku=%.1f -> controlPID P=%.1f I=%.1f D=%.2f (applied)\n", Tu, a, Ku, Kp, Ki,
         Kd);
  printf("autotune: to make permanent, put 'controlPID %.1f %.1f %.2f' on both steer motors in the .wbt\n", Kp, Ki, Kd);
  fflush(stdout);
}

// Reads "drive steer" (each in [-1,1]) from COMMAND_FILE. If the file is
// missing or unparsable, the previous command just holds - an external
// commander writes a value once and it stays in effect until overwritten,
// rather than needing to be refreshed every step.
static void read_remote_command(double *drive, double *steer) {
  static double last_drive = 0.0, last_steer = 0.0;

  FILE *f = fopen(COMMAND_FILE, "r");
  if (f != NULL) {
    double d, s;
    if (fscanf(f, "%lf %lf", &d, &s) == 2) {
      if (d > 1.0)
        d = 1.0;
      if (d < -1.0)
        d = -1.0;
      if (s > 1.0)
        s = 1.0;
      if (s < -1.0)
        s = -1.0;
      last_drive = d;
      last_steer = s;
    }
    fclose(f);
  }

  *drive = last_drive;
  *steer = last_steer;
}

// Reactive obstacle avoidance: drive forward when the path is clear, steer
// away from whichever side reads closer once something is near, and commit
// to a reverse-and-turn recovery (toward the more-open side) if boxed in
// close on both sides or if it isn't actually moving despite commanding
// drive (wedged on something that doesn't read as "too close" on any
// sensor). Positive steer turns left (see mix_ackermann_differential). Each
// side's proximity is the closest of that side's front distance sensor and
// its front/rear corner radars.
//
// Once a recovery starts it holds for a minimum committed duration (see
// start_recovery) instead of being re-decided every step - logged evidence
// showed the rover chattering between reverse and forward many times a
// second at a corner, pinned in place, because instantaneous proximity
// readings flickered across OBSTACLE_STOP_M while reversing. The
// not-moving check is suspended during a recovery so it can't re-trigger
// mid-maneuver and flip the escape direction.
//
// "Not moving" is judged from GPS ground speed (m/s), NOT position delta:
// the OSM world's GPS is WGS84 (lat/lon degrees), where a position-delta
// check falsely reads "stuck" everywhere. Speed is unit-consistent.
static void autonomous_decide(AutonomousState *as, double now_s, const RoverState *state, double *drive,
                               double *steer) {
  double left_proximity = fmin(state->ds_left, fmin(state->radar_fl, state->radar_rl));
  double right_proximity = fmin(state->ds_right, fmin(state->radar_fr, state->radar_rr));

  // Continue a committed recovery to completion.
  if (as->in_recovery) {
    if (now_s < as->recovery_until_time) {
      *drive = -0.6;
      *steer = as->recovery_steer_sign;
      return;
    }
    as->in_recovery = 0;
    as->last_recovery_end_time = now_s;
    as->low_speed_since = -1.0;  // fresh baseline after backing up
  }

  // Reactive command from the range sensors.
  if (left_proximity < OBSTACLE_STOP_M && right_proximity < OBSTACLE_STOP_M) {
    double steer_sign = (left_proximity > right_proximity) ? 1.0 : -1.0;
    start_recovery(as, now_s, steer_sign);
    *drive = -0.6;
    *steer = as->recovery_steer_sign;
    return;
  }
  if (left_proximity < OBSTACLE_NEAR_M || right_proximity < OBSTACLE_NEAR_M) {
    *drive = 0.5;
    *steer = (left_proximity < right_proximity) ? -1.0 : 1.0;
  } else {
    *drive = 1.0;
    *steer = 0.0;
  }

  // Not-moving check: only meaningful when we're actually asking to drive.
  if (fabs(*drive) > 0.1 && state->speed < STUCK_SPEED_M_S) {
    if (as->low_speed_since < 0.0)
      as->low_speed_since = now_s;
    else if (now_s - as->low_speed_since >= STUCK_CHECK_PERIOD_S) {
      double steer_sign = (left_proximity > right_proximity) ? 1.0 : -1.0;  // toward the more-open side
      start_recovery(as, now_s, steer_sign);
      *drive = -0.6;
      *steer = as->recovery_steer_sign;
      as->low_speed_since = -1.0;
    }
  } else {
    as->low_speed_since = -1.0;  // moving fine (or not trying to)
  }
}

// Kinematic stand-in for a mechanical differential: given the commanded
// center-line wheel speed and steering angle, derive the left/right rear
// wheel speeds from the bicycle-model turn radius so the inner wheel goes
// slower and the outer wheel goes faster while turning.
static void mix_ackermann_differential(double wheel_speed, double steer_angle, double *left_speed,
                                        double *right_speed) {
  const double MIN_STEER_ANGLE = 0.01;  // rad; avoid a near-zero-angle blow-up
  if (fabs(steer_angle) < MIN_STEER_ANGLE) {
    *left_speed = wheel_speed;
    *right_speed = wheel_speed;
    return;
  }

  // Turn radius at the rear axle. Positive steer_angle turns left, putting
  // the turn center to the vehicle's left, so turn_radius comes out positive
  // and the sign convention below handles both turn directions on its own.
  double turn_radius = WHEELBASE / tan(steer_angle);
  *left_speed = wheel_speed * (turn_radius - TRACK_WIDTH / 2.0) / turn_radius;
  *right_speed = wheel_speed * (turn_radius + TRACK_WIDTH / 2.0) / turn_radius;
}

static void read_rover_state(const Sensors *s, RoverState *state) {
  const double *pos = wb_gps_get_values(s->gps);
  state->pos[0] = pos[0];
  state->pos[1] = pos[1];
  state->pos[2] = pos[2];

  const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(s->imu);
  state->heading_deg = rpy[2] * RAD2DEG;  // yaw about Z (up) in this ENU world

  state->speed = wb_gps_get_speed(s->gps);  // m/s, same in local- and WGS84-GPS worlds

  state->ds_left = wb_distance_sensor_get_value(s->ds_left);
  state->ds_right = wb_distance_sensor_get_value(s->ds_right);

  state->radar_fl = read_radar_min_distance(s->radar_fl);
  state->radar_fr = read_radar_min_distance(s->radar_fr);
  state->radar_rl = read_radar_min_distance(s->radar_rl);
  state->radar_rr = read_radar_min_distance(s->radar_rr);
}

static void motors_write(const Motors *m, double left_speed, double right_speed, double steer_angle) {
  wb_motor_set_velocity(m->wheel_rl, left_speed);
  wb_motor_set_velocity(m->wheel_rr, right_speed);

  wb_motor_set_position(m->steer_fl, steer_angle);
  wb_motor_set_position(m->steer_fr, steer_angle);
}

static FILE *log_open(void) {
  FILE *f = fopen("ugv_teleop_log.csv", "w");
  if (f != NULL) {
    fprintf(f,
            "time_s,mode,pos_x,pos_y,pos_z,heading_deg,ds_left_m,ds_right_m,"
            "radar_fl_m,radar_fr_m,radar_rl_m,radar_rr_m,"
            "steer_cmd_deg,steer_fl_actual_deg,steer_fr_actual_deg,wheel_rl_speed_rad_s,wheel_rr_speed_rad_s\n");
    fflush(f);
  }
  return f;
}

static void log_write(FILE *f, double time_s, DriveMode mode, const RoverState *state, double steer_cmd_deg,
                       double steer_fl_deg, double steer_fr_deg, double wheel_rl_speed, double wheel_rr_speed) {
  if (f == NULL)
    return;
  fprintf(f, "%.3f,%s,%.3f,%.3f,%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f\n", time_s,
          mode_name(mode), state->pos[0], state->pos[1], state->pos[2], state->heading_deg, state->ds_left,
          state->ds_right, state->radar_fl, state->radar_fr, state->radar_rl, state->radar_rr, steer_cmd_deg,
          steer_fl_deg, steer_fr_deg, wheel_rl_speed, wheel_rr_speed);
}

static void print_status(DriveMode mode, const RoverState *state, double steer_cmd_deg, double steer_fl_deg,
                          double steer_fr_deg, double wheel_rl_speed, double wheel_rr_speed) {
  printf(
      "[%s] pos=(%.2f, %.2f, %.2f) m  heading=%.1f deg  ds_left=%.2f m  ds_right=%.2f m\n"
      "  radar fl=%.2f fr=%.2f rl=%.2f rr=%.2f m\n"
      "  steer_cmd=%.1f deg  steer_fl=%.1f deg  steer_fr=%.1f deg  wheel_rl=%.2f rad/s  wheel_rr=%.2f rad/s\n",
      mode_name(mode), state->pos[0], state->pos[1], state->pos[2], state->heading_deg, state->ds_left, state->ds_right,
      state->radar_fl, state->radar_fr, state->radar_rl, state->radar_rr, steer_cmd_deg, steer_fl_deg, steer_fr_deg,
      wheel_rl_speed, wheel_rr_speed);
  fflush(stdout);
}

int main(int argc, char **argv) {
  wb_robot_init();

  Motors motors;
  Sensors sensors;
  motors_init(&motors);
  sensors_init(&sensors);
  wb_keyboard_enable(TIME_STEP);

  FILE *log_file = log_open();
  if (log_file == NULL) {
    printf("ugv_teleop: warning - could not open ugv_teleop_log.csv for writing\n");
    fflush(stdout);
  }

  printf(
      "ugv_teleop: click the 3D view then use arrow keys (Up/Down = drive, Left/Right = steer).\n"
      "Press 'M' for MANUAL, 'A' for AUTONOMOUS, 'R' for REMOTE (reads %s), 'T' to auto-tune steering PID.\n"
      "PageUp/'+' speed up, PageDown/'-' slow down. Starting in MANUAL at 1.0x speed.\n",
      COMMAND_FILE);
  fflush(stdout);

  DriveMode mode = MODE_MANUAL;
  AutonomousState auto_state;
  autonomous_state_init(&auto_state);
  int since_last_print_ms = 0;
  double speed_scale = 1.0;

  while (wb_robot_step(TIME_STEP) != -1) {
    double drive, steer;
    DriveMode requested_mode;
    int mode_requested, autotune_requested, speed_adjust;
    read_keyboard_command(&drive, &steer, &requested_mode, &mode_requested, &autotune_requested, &speed_adjust);

    if (autotune_requested) {
      run_steering_autotune(&motors, &sensors);
      continue;  // consumed many steps; restart the loop cleanly
    }

    if (speed_adjust != 0) {
      speed_scale += speed_adjust * SPEED_SCALE_STEP;
      if (speed_scale < SPEED_SCALE_MIN)
        speed_scale = SPEED_SCALE_MIN;
      if (speed_scale > SPEED_SCALE_MAX)
        speed_scale = SPEED_SCALE_MAX;
      printf("ugv_teleop: speed -> %.1fx (%.1f rad/s max, ~%.1f m/s)\n", speed_scale, MAX_WHEEL_SPEED * speed_scale,
             MAX_WHEEL_SPEED * speed_scale * 0.2286);
      fflush(stdout);
    }

    if (mode_requested && requested_mode != mode) {
      mode = requested_mode;
      if (mode == MODE_AUTONOMOUS)
        autonomous_state_init(&auto_state);  // fresh stuck-detection baseline on (re-)entry
      printf("ugv_teleop: mode -> %s\n", mode_name(mode));
      fflush(stdout);
    }

    // Telemetry has to be read before the drive decision: autonomous/remote
    // modes need this step's state to decide this step's command.
    RoverState state;
    read_rover_state(&sensors, &state);

    if (mode == MODE_AUTONOMOUS)
      autonomous_decide(&auto_state, wb_robot_get_time(), &state, &drive, &steer);
    else if (mode == MODE_REMOTE)
      read_remote_command(&drive, &steer);

    double wheel_speed = drive * MAX_WHEEL_SPEED * speed_scale;
    double steer_angle = steer * MAX_STEER_ANGLE;

    double left_speed, right_speed;
    mix_ackermann_differential(wheel_speed, steer_angle, &left_speed, &right_speed);
    motors_write(&motors, left_speed, right_speed, steer_angle);

    double steer_fl_actual_deg = wb_position_sensor_get_value(sensors.steer_fl_sensor) * RAD2DEG;
    double steer_fr_actual_deg = wb_position_sensor_get_value(sensors.steer_fr_sensor) * RAD2DEG;
    double steer_cmd_deg = steer_angle * RAD2DEG;

    log_write(log_file, wb_robot_get_time(), mode, &state, steer_cmd_deg, steer_fl_actual_deg, steer_fr_actual_deg,
              left_speed, right_speed);

    since_last_print_ms += TIME_STEP;
    if (since_last_print_ms >= PRINT_PERIOD_MS) {
      print_status(mode, &state, steer_cmd_deg, steer_fl_actual_deg, steer_fr_actual_deg, left_speed, right_speed);
      since_last_print_ms = 0;
    }
  }

  if (log_file != NULL)
    fclose(log_file);

  wb_robot_cleanup();
  return 0;
}
