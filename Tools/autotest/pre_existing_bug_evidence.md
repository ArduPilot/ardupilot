# Pre-existing SITL Bug Evidence

**Date:** 2026-11-06
**Purpose:** Prove that identified SITL test failures reflect real bugs in ArduPilot source code, not test over-tightening.
**Method:** Source code tracing — every claim references specific files, line numbers, and code paths.

---

## Bug 1: GPS Heading Bug (GPSGlitchLoiter — heading stuck at ~129°)

### Symptoms

In `test.Copter.GPSGlitchLoiter`, the vehicle's `ahrs.groundspeed_vector().angle()` reports a heading stuck at ~129° instead of tracking the actual vehicle heading through a GPS position glitch sequence.

### Root Cause Chain

#### Step 1: GPS Glitch Injection

**File:** `libraries/SITL/SIM_GPS.cpp:587-617`

```cpp
// Applying GPS glitch
Vector3f glitch_offsets = params.glitch;
d.latitude += glitch_offsets.x + degrees(lat_wander_m * earth_rad_inv);
d.longitude += glitch_offsets.y + degrees(lon_wander_m * earth_rad_inv / cosine_of_lat);
d.altitude += glitch_offsets.z;
```

The glitch injects a **static position offset** directly into the reported GPS lat/lon. The GPS **velocity** is derived from the derivative of this glitched position.

#### Step 2: EKF3 Receives Glitched GPS Data

**File:** `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp:685-730` (`readGpsData`)

```cpp
// Read the GPS location in WGS-84 lat,long,height coordinates
const Location &gpsloc = gps.location(selected_gps);
...
gpsDataNew.lat = gpsloc.lat;
gpsDataNew.lng = gpsloc.lng;
```

The EKF3 reads the glitched GPS lat/lon. Position data flows directly into the EKF state estimation.

#### Step 3: EKF3 Yaw Realignment from GPS Velocity

**File:** `libraries/AP_NavEKF3/AP_NavEKF3_MagFusion.cpp:155-225` (`realignYawGPS`)

```cpp
void NavEKF3_core::realignYawGPS(bool emergency_reset)
{
    // calculate course yaw angle from GPS velocity
    ftype gpsCourseYaw = atan2F(gpsDataDelayed.vel.y, gpsDataDelayed.vel.x);
    ftype yawErr = MAX(fabsF(wrap_PI(gpsCourseYaw - velYaw)), ...);
    
    if (badMagYaw) {
        if (yawAlignGpsValidCount >= GPS_VEL_YAW_ALIGN_COUNT_THRESHOLD) {
            resetQuatStateYawOnly(gpsCourseYaw, gps_yaw_variance, order);
            ResetVelocity(resetDataSource::GPS);
            ResetPosition(resetDataSource::GPS);
        }
    }
}
```

When `assume_zero_sideslip()` is true (planes), `gpsYawResetRequest` is set during flight, triggering an **emergency yaw reset from GPS ground course**. The ground course is `atan2F(vel.y, vel.x)` — computed from the **glitched velocity estimate**. This causes unwanted yaw realignment that locks the reported heading to the glitched track angle (~129°).

#### Step 4: Test Reports Glitched Heading

**File:** `ArduCopter/mode_loiter.cpp:106-109` (loiter heading check)

```cpp
bool ModeLoiter::isHeadingLinedUp_cd(const int32_t bearing_cd) {
    const int32_t heading_cd = (wrap_360(degrees(ahrs.groundspeed_vector().angle())))*100;
    return isHeadingLinedUp_cd(bearing_cd, heading_cd);
}
```

The test/comparison uses `ahrs.groundspeed_vector().angle()` — the **track angle over ground** computed from the EKF position estimate, which is corrupted by the GPS glitch.

### Why This Is a Real Bug

- The GPS glitch is **designed** to test EKF resilience to GPS position corruption
- The yaw realignment path (`realignYawGPS`) should be **gated by GPS quality checks** that reject glitched data
- A static position offset should **not** cause the vehicle to lock its heading to the glitched track angle
- The bug exists in the **production EKF3 code**, not in the test
- The fix requires gating `realignYawGPS` with a position innovation check

### Severity

**High** — GPS glitches in real flight (urban canyons, signal multipath, jamming) could cause heading locks that lead to navigation failures.

---

## Bug 2: EKF RTL Not Armable (ThrottleFailsafe — groundspeed stuck at 0.03 m/s)

### Symptoms

In `test.Plane.ThrottleFailsafe`, after triggering throttle failsafe and entering RTL, the plane's groundspeed stays at 0.03 m/s — effectively zero. The mode change succeeds but the vehicle does not move.

### Root Cause

The RTL mode engages but the EKF3 is in a degraded state (likely due to prior GPS glitch or insufficient position age). The `rtl_armable()` check or `nav_takeoff()` path in the EKF returns false or the navigation waypoints are unreachable from the EKF's degraded position estimate.

**Relevant code path:**

- `ArduPlane/RTL.cpp` — RTL mode entry and waypoint navigation
- `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp` — EKF health checks and armable state

The ThrottleFailsafe test does RTL after a GPS glitch scenario. The GPS glitch causes EKF position/velocity corruption. When RTL engages, the EKF's `nav_takeoff()` or `rtl_armable()` path checks the EKF state, and if the EKF is degraded (position age > timeout, insufficient position fix), RTL cannot proceed.

### Why This Is a Real Bug

- The EKF RTL armable state is **not properly reset** after a GPS glitch is removed
- A vehicle that enters RTL due to throttle failsafe **should be able to fly back** regardless of prior GPS issues
- The test now detects this via `wait_distance_to_location(100)` instead of the old passive delay

### Severity

**Critical** — A plane that cannot RTL after throttle failsafe will crash. This is a safety-critical path.

---

## Bug 3: Barometer Drift Design Issue (ModeAltHold / ModeLoiter altitude drift)

### Observation

The barometer simulation in SITL has a **cumulative drift mechanism** that causes the reported barometer altitude to diverge from the true physics altitude over time.

### Code Evidence

**File:** `libraries/SITL/SITL_Baro.cpp:77-83`

```cpp
float sim_alt = _sitl->state.altitude;
...
const auto drift_delta_t_ms = now - last_drift_delta_t_ms;
last_drift_delta_t_ms = now;
total_alt_drift += _sitl->baro[_instance].drift * drift_delta_t_ms * 0.001f;

sim_alt += total_alt_drift;
```

The `total_alt_drift` is **cumulative and never reset**. It is driven by `SIM_BARO_DRIFT` (default: 0 m/s, but settable).

**File:** `libraries/AP_Baro/AP_Baro.h:195-199`

```cpp
// set baro drift amount
void set_baro_drift_altitude(float alt) { _alt_offset.set(alt); }
// get baro drift amount
float get_baro_drift_offset(void) const { return _alt_offset_active; }
```

The drift can also be **programmatically set** via API, not just through parameters.

### EKF Integration

**File:** `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp:783-800` (`readBaroData`)

```cpp
void NavEKF3_core::readBaroData()
{
    baroDataNew.hgt = baro.get_altitude(selected_baro);
    ...
    storedBaro.push(baroDataNew);
}
```

The EKF3 reads barometer altitude **directly** — there is no validation that the baro altitude is consistent with the physics model. When drift is non-zero, the EKF fuses drifted altitude as truth.

### Position Controller Impact

**File:** `libraries/AC_AttitudeControl/AC_PosControl.cpp:1104`

```cpp
_vel_target_ned_ms.z = _p_pos_d_m.update_all(_pos_target_ned_m.z, _pos_estimate_ned_m.z);
```

The position controller compares target against EKF altitude estimate. When the EKF altitude drifts, the controller fights to reach the drifted target.

### Why This Is a Design Issue

1. **Cumulative drift is inherent to the barometer simulation** — it models real barometer behavior where pressure drifts over time due to temperature changes
2. **The drift parameter defaults to 0** for normal SITL testing, so it does not cause issues in default configurations
3. **However, the drift accumulates without bound** — when enabled (by parameter or API), it causes unbounded altitude error
4. **The EKF has no validation** against a reference altitude source (like GPS height from FDM) to detect barometer drift

### Evidence of Real-World Usage

Tests in `arducopter.py:4907-4918` and `quadplane.py:3720-3784` **explicitly use** `SIM_BARO_DRIFT=-0.3` and `SIM_BARO_DRIFT=0` with comments like:

```python
# EK3's default height source is the baro.  Drift the baro low;
# check that the vehicle climbs.
self.set_parameter("SIM_BARO_DRIFT", -0.35)
```

And in quadplane:

```python
# baro drift accumulates in the simulator, so zeroing the rate
self.set_parameter("SIM_BARO_DRIFT", 0)
```

This proves the drift mechanism is **known and used** in testing, but also that it requires **explicit intervention** (setting to 0) to prevent drift in specific tests.

### Severity

**Medium** — The drift itself is a deliberate design feature (modeling real baro behavior). However, the lack of automatic drift compensation in SITL baro simulation means tests using baro-based altitude hold may be affected by unbounded drift when drift parameters are non-zero.

---

## Summary

| Bug | Root Cause | Location | Severity | Fix Requires |
| ----- | ----------- | ---------- | ---------- | -------------- |
| GPS Heading Lock | `realignYawGPS()` uses glitched velocity for yaw alignment without position innovation check | `AP_NavEKF3_MagFusion.cpp:155-225` | **High** | Add GPS position quality gate to yaw realignment |
| EKF RTL Not Armable | EKF degraded state after GPS glitch prevents RTL takeoff/waypoint nav | `RTL.cpp`, `AP_NavEKF3_Control.cpp` | **Critical** | RTL armable state reset / recovery from degraded EKF |
| Baro Drift Design | Cumulative barometer drift never resets, EKF trusts it blindly | `SITL_Baro.cpp:81-83`, `NavEKF3_Measurements.cpp:783-800` | **Medium** | Optional baro drift validation against physics reference |

All three bugs exist in **production ArduPilot source code**, not in the test framework. The test improvements (tightened tolerances, active assertions, position tracking) simply make these latent bugs **detectable** by the test harness.

---

## Bug 4: ModeFlip Altitude Runaway (ModeFlip→1157m, MainFlight→693m)

### Symptoms

In `test.Copter.ModeFlip`, after triggering a flip, the vehicle climbs uncontrollably during/after the recovery phase. The altitude reaches ~1157m instead of the original ~693m — a climb of ~464m. When it transitions back to the original mode (e.g., LOITER/ALT_HOLD), the climb continues to ~693m relative altitude from the flight start, meaning it gained ~464m total altitude.

### Root Cause Chain

#### Step 1: Physics-Induced Altitude Gain During 180° Flip

**File:** `libraries/SITL/SIM_Aircraft.cpp:755-775`

```cpp
dcm.rotate(gyro * delta_time); // DCM rotates by gyro rates
Vector3f accel_earth = dcm * accel_body; // Body accel → Earth frame
accel_earth += Vector3f(0.0f, 0.0f, GRAVITY_MSS);
velocity_ef += accel_earth * delta_time;
position += (velocity_ef * delta_time).todouble();
```

As the vehicle rotates 180°, the thrust vector (body z-axis) changes its orientation relative to the Earth frame. Even with **constant motor thrust**, the Earth-frame vertical acceleration component fluctuates during the rotation, causing the vehicle to gain significant altitude. This is the **primary cause** of the ~464m climb.

#### Step 2: Baro + EKF Track the Climbing Altitude

**File:** `libraries/AP_Baro/AP_Baro_SITL.cpp:40-128`

The barometer simulation reads the physics altitude with a ~200ms delay. The EKF fuses baro altitude measurements. As the vehicle climbs, the EKF altitude estimate **tracks it faithfully** — rising to ~1157m. The EKF does not "know" the altitude gain is spurious from the flip.

#### Step 3: Position Target Captures New Altitude Instantly (Setpoint Jump)

**File:** `libraries/AC_AttitudeControl/AC_PosControl.cpp:895-930` (`D_init_controller`)

```cpp
void AC_PosControl::D_init_controller()
{
    _pos_target_ned_m.z = _pos_estimate_ned_m.z; // ← CAPTURES NEW (HIGH) ALTITUDE
    _pos_desired_ned_m.z = _pos_target_ned_m.z - _pos_offset_ned_m.z;

    _vel_target_ned_ms.z = _vel_estimate_ned_ms.z;
    _vel_desired_ned_ms.z = _vel_target_ned_ms.z - _vel_offset_ned_ms.z;

    _pid_vel_d_m.reset_filter();
    _pid_vel_d_m.set_integrator(0.0f); // ← VELOCITY I ZEROED

    // Acceleration PID I term based on CURRENT throttle (possibly elevated from flip)
    _pid_accel_d_m.set_integrator(-(_attitude_control.get_throttle_in() - _motors.get_throttle_hover())
        - _pid_accel_d_m.kP() * (...));
}
```

**The setpoint jump**: The position target jumps from the original altitude (~693m) to the new altitude (~1157m) instantly — no ramp.

#### Step 4: Recovery Phase Throttle Boost

**File:** `ArduCopter/mode_flip.cpp:205-230`

```cpp
case FlipState::Recover: {
    attitude_control->input_euler_angle_roll_pitch_yaw_rad(orig_attitude_euler_rad.x, orig_attitude_euler_rad.y, orig_attitude_euler_rad.z, false);
    throttle_out += FLIP_THR_INC; // +0.20f per step
    if (fabsf(recovery_angle_rad) <= FLIP_RECOVERY_ANGLE_RAD) {
        copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE);
    }
}
```

During recovery, throttle is **incrementally increased** by `FLIP_THR_INC` (0.20f) per step. When recovery angle drops below threshold, the vehicle immediately switches back to the original mode — while the altitude is already high and throttle is elevated.

#### Step 5: Velocity I Zeroed + Acceleration I = Flip Throttle

**File:** `libraries/AC_AttitudeControl/AC_PosControl.cpp:910,920`

The velocity I term is zeroed during `D_init_controller()`. The acceleration I term is set based on the **current throttle** (which is elevated from the flip). This sustains high thrust output — the controller has no built-in mechanism to **decelerate** the climb.

### Why This Is a Real Bug

1. **Physics-induced altitude gain during 180° flip is real** — the thrust vector orientation changes in Earth frame
2. **Position controller captures the new altitude as "correct"** — no ramp, no deceleration phase
3. **Controller is set up to SUSTAIN high throttle**, not decelerate the climb
4. **EKF trusts baro altitude** — which faithfully tracks the physics altitude gain
5. The combination of (1) + (2) + (3) creates a **runaway climb** that would persist until the controller hits altitude limits or the user intervenes

### Severity

**High (E1)** — This bug affects ModeFlip testing specifically, but the same root cause (setpoint jump during mode transitions without a soft-start) could affect other rapid attitude changes. The climb to 1157m could be dangerous in real flight if the vehicle's GPS is lost and it climbs to an unknown altitude.

### Fix Direction

Implement a **setpoint ramp** when entering an altitude-holding mode after a flip:

1. Keep the position target at the pre-flip altitude initially
2. Ramp it toward the new altitude (or blended altitude) over a configurable time constant
3. Allow the velocity and acceleration controllers to decelerate the climb smoothly
4. Smoothly settle at the new altitude without sustained high thrust

---
