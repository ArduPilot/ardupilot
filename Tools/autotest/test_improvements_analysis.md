# Autotest Robustness & Bug-Catching Analysis

**Date:** 2026-11-06
**Scope:** Copter, Plane, Rover (UGV) autotest suites

---

## Executive Summary

The ArduPilot autotest suite has ~17,000 lines of copter tests, ~8,400 lines of plane tests, and ~7,300 lines of rover tests. Many tests use tight tolerances with short timeouts, rely on precise timing assumptions, or lack state cleanup — creating flakiness that masks real bugs. This analysis identifies specific tests that should be improved for robustness and better bug-catching coverage.

---

## 1. ArduCopter Tests (arducopter.py — ~17,207 lines)

### 1.1 Tests That Should Be More Robust

| Test | Issue | Suggested Fix |
|------|-------|---------------|
| **`ModeFlip`** | Uses `wait_attitude(despitch=0, desroll=45, tolerance=30)` — 30° tolerance is extremely wide for a flip test. The tolerance allows the test to pass even when the flip is barely initiated. Also, `wait_attitude` uses `ATTITUDE` message at ~100Hz but has no `minimum_duration`, so a single out-of-range spike can falsely pass. | Tighten `tolerance` to 15°. Add `minimum_duration=0.5` to each `wait_attitude` call to ensure the attitude is *maintained*, not just momentarily hit. |
| **`OpticalFlow`** | The `CheckOpticalFlow` hook raises `NotAchievedException` when `abs(of_speed - gps_speed) > 3` m/s — a 3 m/s tolerance on optical flow vs GPS speed is very loose. This means the test passes even when optical flow is significantly inaccurate. | Tighten to 1.5 m/s. Add a warning path at 2 m/s that logs data but doesn't fail, so regression visibility improves before failures. |
| **`OpticalFlowLimits`** | Hardcoded timeout of 60s with no minimum speed assertion after the speed limit engages. The test verifies speed is limited but doesn't assert that the limit is actually enforced. | Add assertion that speed never exceeds the 10 m/s limit at any point during the test, not just at sampled intervals. |
| **`Landing` / `PrecisionLanding`** | `PrecisionLanding` iterates over backend types [4, 2] with `ex = None; try/except` that silently catches failures in some paths. If a backend fails to set up properly, the test may skip it without reporting. | Report the caught exception in the fallback path instead of silently continuing. Add explicit per-backend pass/fail tracking. |
| **`test_mount_pitch`** | Uses a `hold=0` default — the test considers the pitch achieved the instant the value is correct, with no dwell time. This is the same `wait_and_maintain` pattern but with `minimum_duration=0`. | Set `hold=1.0` (1 second dwell) as default so transient spikes don't count as achievement. |
| **`GPSGlitchLoiter` / `GPSGlitchLoiter2` / `GPSGlitchAuto`** | Three separate GPS glitch tests that all inject position jumps. `GPSGlitchLoiter` used `self.drain_mav()` during the glitch phase — no position tracking, so the test passed even when the vehicle flew off-course. `GPSGlitchLoiter2` only checked attitude angles (roll/pitch < 2°) but not position. `GPSGlitchAuto` looped through glitch values without actively tracking position. | **Fixed (2026-11-06):** All three tests now track position during the glitch phase and raise `NotAchievedException` if the vehicle moves beyond a defined threshold (20m for Loiter, 10m for Loiter2, 30m for Auto). |
| **`SMARM_RTL_EnterLeave` / `SMARM_RTL_Repeat`** | Currently in `disabled_tests()`: "Causes a panic" and "Currently fails due to issue with loop detection". These are known-failing tests that should drive fixes. | These are disabled *because* of bugs. Once fixed, add more thorough RTL transition state assertions (mode switch timing, waypoint resume behavior). |

### 1.2 Tests That Should Catch More Bugs

| Area | Current Coverage Gap | Suggested Test |
|------|---------------------|----------------|
| **EKF compass degradation during flight** | `TestEKF3CompassFailover` tests compass switch on disarm, but not switch *during* active flight. | Add test that degrades compass quality mid-flight (e.g., `SIM_MAG1_FAIL=0.5` for partial failure) and verifies EKF fallback doesn't cause instability. |
| **Visual odom source switching** | `VisionPosition` exists but doesn't test switching between visual odom and GPS while in flight. | Add test that toggles `SIM_FLOW_ENABLE` mid-flight and verifies the EKF handles the gap without position jumps. |
| **RC protocol failsafe edge cases** | `RCProtocolFailsafe` exists but doesn't test mixed protocol scenarios (e.g., CRSF fails, SBUS recovers). | Add test simulating intermittent RC signal with rapid fail/clear cycles. |
| **Mission item speed changes mid-mission** | `DO_CHANGE_SPEED` exists but doesn't test speed changes between waypoints with tight tolerance on groundspeed. | Add test that changes speed at waypoint N and asserts groundspeed at waypoint N+1 within ±1 m/s. |
| **Battery monitor cross-sensitivity** | `IE24`, `BatteryInternalUseOnly` exist but no test for battery param changes mid-flight affecting failsafe behavior. | Add test that changes `BATT_ARM_VOLT` mid-flight and verifies the arm guard correctly blocks arming on next boot. |
| **Fence parameter interaction with terrain** | Fence tests (`HorizontalFence`, `MaxAltFence`, etc.) don't test fence behavior with terrain data enabled (`SIM_TERRAIN=1`). | Add fence tests with terrain data to catch fence/terrain interaction bugs. |

---

## 2. ArduPlane Tests (arduplane.py — ~8,400 lines)

### 2.1 Tests That Should Be More Robust

| Test | Issue | Suggested Fix |
|------|-------|---------------|
| **`test_stabilize`** | `wait_roll(±150, accuracy=90)` — 90° accuracy on a ±150° target means the test considers 60°-240° as "achieved". This is so wide the test can't detect a partially-failing roll. | Tighten `accuracy` to 15°. Also add `minimum_duration=1.0` to ensure the roll is sustained. |
| **`test_acro`** | Same 90° accuracy issue on roll tests. The loop test uses `wait_pitch(-60, accuracy=20)` which allows 40°-80° — a 40° error band. | Tighten roll accuracy to 15° and pitch accuracy to 10°. |
| **`test_FBWB`** | Altitude maintenance check uses `if abs(final_alt - initial_alt) > 20` — a 20m tolerance on a ~50m altitude is 40% error. | Tighten to 5m. Add intermediate altitude checks during the circuit, not just at the end. |
| **`ThrottleFailsafe`** | Uses `wait_mode('RTL')` with default 60s timeout but no assertion that the plane actually *flew* to RTL before the failsafe triggered. | Add assertion that the plane moved at least 100m from home after the failsafe triggered. |
| **`DO_REPOSITION`** | Uses `wait_altitude(new_alt-10, new_alt, timeout=30, relative=True)` — the 10m tolerance is wide and doesn't check horizontal accuracy. | Add horizontal position assertion within 5m of target. Check terrain-altitude mode separately. |

### 2.2 Tests That Should Catch More Bugs

| Area | Current Coverage Gap | Suggested Test |
|------|---------------------|----------------|
| **Quadplane transition robustness** | No dedicated test for quadplane motor fail during transition (VTOL→FW or FW→VTOL). | Add test that triggers motor failure during VTOL transition and verifies graceful recovery. |
| **ELEVREV / elevon reversal during ACRO** | `test_acro` tests roll but not pitch reversals specific to taildragger/elevrev frames. | Add pitch reversal test specific to elevrev frame configuration. |
| **Fence breach during takeoff** | `test_fence_breach_circle_at` tests fence breach but only from a stationary/cruising state. | Add fence breach test during takeoff climb-out (low altitude, high pitch). |
| **Takeoff with varying runway conditions** | `takeoff_in_FBWA` hardcodes elevator/throttle inputs and uses `wait_groundspeed(6, 100)` then `wait_groundspeed(12, 100)` with generous 100s timeouts. | Add parameterized takeoff test with varying throttle/elevator inputs and tighter groundspeed assertions. |
| **Autoland with wind** | No test for autoland behavior under wind conditions (SIM_WIND). | Add autoland test with 5 m/s crosswind, verifying landing position within tolerance. |

---

## 3. Rover / UGV Tests (rover.py — ~7,300 lines)

### 3.1 Tests That Should Be More Robust

| Test | Issue | Suggested Fix |
|------|-------|---------------|
| **`test_gcs_failsafe`** | Uses `delay_sim_time(5)` after setting GCS failsafe — this is a fixed delay that assumes the failsafe triggers within 5s. If the GCS timeout parameter changes, the test silently passes even if the failsafe hasn't triggered. | Replace fixed delay with `wait_mode("RTL")` or `wait_mode("MANUAL")` assertions. Use `delay_sim_time` only for non-critical waiting. |
| **`DriveSquare`** | Uses `wait_distance(50, accuracy=7)` — 7m accuracy on a 50m leg is 14% error. For a square drive test, this means the rover could be 14m off and still pass. | Tighten to 3m accuracy. Add heading assertion at each corner to verify the turn was completed. |
| **`drive_left_circuit`** | Same 14% accuracy issue (`wait_distance(50, accuracy=7)`). No altitude/height check (less critical for rover but relevant for ATVs). | Tighten to 3m accuracy. For ATV frames, add height variation check. |
| **`test_poly_fence_*` tests** | Multiple fence tests upload large polygons (50+ exclusion circles, 4+ polygon vertices). These are slow (100s+ per test) and timeout-prone. | Split large polygon tests into smaller, focused tests. Add timeout progress logging. |
| **`test_scripting_simple_loop`** | Checks `count >= 3` for "hello, world" messages — too few assertions. If the script crashes after printing once, the test passes. | Check for specific message count ranges (e.g., 3-10) and verify the loop ran for the expected duration. |
| **`ThrottleFailsafe`** | Sets `SIM_RC_FAIL=1` (no-pulses failsafe) but doesn't verify the rover actually entered the failsafe mode — it only checks `wait_mode` which may timeout silently in edge cases. | Add `wait_mode` with explicit error handling. Assert groundspeed drops to zero after the failsafe triggers. |

### 3.2 Tests That Should Catch More Bugs

| Area | Current Coverage Gap | Suggested Test |
|------|---------------------|----------------|
| **RTCM correction loss during navigation** | No test for rover behavior when RTK/RTCM corrections are lost mid-mission. | Add test that enables RTK, starts a mission, drops RTCM corrections mid-mission, and verifies graceful degradation. |
| **Compass interference during steering** | `CompassPrearms` exists but no test for compass interference during active steering (e.g., motor magnetic field). | Add test with simulated compass distortion during steering maneuvers. |
| **Motor test with reverse direction** | `MotorTest` exists but doesn't test motor test in reverse gear (`NAV_SBWR_REVERSE`). | Add motor test with reverse gear enabled. |
| **Fence crossing with object avoidance** | PolyFence avoidance tests exist but don't test repeated fence boundary crossings (in/out/in). | Add test for repeated fence boundary crossings to catch state-machine bugs. |
| **Scripting with GPS-denied navigation** | All scripting tests use GPS. No test for scripting in GPS-denied mode (visual odom, dead reckoning). | Add scripting test that operates in optical-flow-only mode. |

---

## 4. Cross-Vehicle Infrastructure Improvements

### 4.1 `wait_and_maintain` Base Infrastructure

**Current behavior:** The `wait_and_maintain` / `wait_and_maintain_range` methods support `minimum_duration` but most tests don't use it. This means a value that briefly enters the target range and immediately leaves still counts as success.

**Suggested change:** Set a default `minimum_duration=0.5` in the base `wait_and_maintain` method for all assertions except explicit spike/transient tests. This would catch tests that pass because of a single measurement spike.

### 4.2 Timeout Defaults

**Current state:** Timeouts vary widely:
- `wait_altitude`: default 30s
- `wait_mode`: default 60s
- `wait_heading`: default 30s
- `wait_groundspeed`: default 30s

**Issue:** For complex maneuvers (RTL, terrain-following, EKF recovery), 30-60s is often insufficient, causing false negatives. For simple maneuvers (takeoff, landing), 30s is excessive, masking slow-start bugs.

**Suggested:**
- Add `aggressive_timeout` parameter to test methods for CI speed runs
- Default timeouts: simple ops 15s, moderate 30s, complex 120s+
- Log timeout values in test output for visibility

### 4.3 State Cleanup Between Tests

**Issue:** Several tests modify parameters (`set_parameters`) without restoring originals. The `context_push`/`context_pop` mechanism handles some cleanup (message hooks, statustext collection) but not parameter state.

**Suggested:** Add a `context_collect_params` / `context_restore_params` mechanism similar to the existing message context system. Tests that modify parameters should use it.

### 4.4 Disabled Tests Inventory

| Test | Vehicle | Reason | Priority |
|------|---------|--------|----------|
| `Parachute` | Copter | Issue #4702 | High — safety-critical |
| `GroundEffectCompensation_*` | Copter | Flapping behavior | High — safety-relevant |
| `SMART_RTL_EnterLeave` | Copter | Causes panic | High — crash bug |
| `SMART_RTL_Repeat` | Copter | Loop detection failure | High — crash bug |
| `RTLStoppingDistanceSpeed` | Copter | Vehicle goes off-course | Medium |
| `FlyMissionTwice` | Copter | Unknown (PR #18561) | Medium |
| `LandingDrift` | Plane | Flapping test — Issue #20054 | High |
| `InteractTest` | Plane | Requires user interaction | Low (expected) |
| `ClimbThrottleSaturation` | Plane | Depends on unmerged PR | Medium |
| `SoaringClimbRate` | Plane | Bad sink rate | Medium |
| `SlewRate` | Rover | CI timing report failure | Medium |
| `PolyFenceObjectAvoidanceBendyRuler` | Rover | Unreliable | High |
| `DriveBrake` | Rover | Frequent failures | Medium |
| `GCSRally` | Rover | Needs MAVProxy fixes | Low |

---

## 5. Prioritized Recommendations

### Priority 1 — Fix Flaky Tests That Mask Real Bugs

1. ~~**Copter `ModeFlip`** — tighten tolerances, add minimum_duration~~ **Fixed (2026-11-06):** tolerance tightened from 30° to 15°.
2. ~~**Plane `test_stabilize` / `test_acro`** — 90° accuracy is non-functional~~ **Fixed (2026-11-06):** roll accuracy tightened from 90° to 15°, pitch from 20° to 10°.
3. **Rover `test_gcs_failsafe`** — replace fixed delays with mode assertions
4. ~~**Copter `GPSGlitch*` tests** — add off-course distance assertions~~ **Fixed (2026-11-06):** all three tests (`GPSGlitchLoiter`, `GPSGlitchLoiter2`, `GPSGlitchAuto`) now track position during the glitch phase.
5. **Rover `DriveSquare`** — tighten distance accuracy

### Priority 2 — Add Missing Bug-Catching Coverage

6. **New: EKF compass degradation mid-flight test** (Copter)
7. **New: Quadplane motor fail during transition** (Plane)
8. **New: Fence breach during takeoff** (Plane)
9. **New: RTK/RTCM loss during rover mission** (Rover)
10. **New: Repeated fence boundary crossing** (Rover)

### Priority 3 — Infrastructure Improvements

11. **`wait_and_maintain` default minimum_duration** — prevents spike-based false passes
12. **Parameter context cleanup** — prevents test contamination
13. **Aggressive timeout mode** — faster CI for simple test paths
14. **Fix disabled high-priority tests** — parachute, SMART_RTL, LandingDrift
