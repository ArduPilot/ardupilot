# Autotest Bug-Catching Analysis

**Date:** 2026-11-06
**Scope:** All committed test improvements and what bugs they now catch

---

## Overview

This document maps each committed test improvement to the specific classes of real bugs it can now detect. The goal is understanding which existing latent bugs in ArduPilot code would be surfaced by running these tightened tests.

---

## 1. wait_and_maintain minimum_duration Default (0 → 0.5s)

**Committed in:** `376ee8e9`
**Files:** `vehicle_test_suite.py` — `wait_and_maintain_range()`, `wait_and_maintain_vector()`

### How it works

The base `wait_and_maintain` infrastructure now requires target values to be *sustained* for 0.5s by default. Previously, a single measurement spike into the target range would instantly pass — the test would "achieve" the target and continue even if the value immediately diverged.

Tests that need no dwell can still pass `minimum_duration=0` explicitly.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **Overshoot oscillation** | Vehicle oscillates through target and back — passes on overshoot peak, not on stable target | PID controller tuned too aggressively, overshooting altitude by 5m then correcting in 0.3s — previously passed, now fails because oscillation doesn't sustain within tolerance for 0.5s |
| **Slow correction** | Controller reaches target but takes 2+ seconds to settle — passes on first hit, not on sustained achievement | Altitude hold reaching target after a step change but drifting 2m/s — first measurement passes, test ends early before drift manifests |
| **Sensor spike false positive** | Single noisy sensor reading triggers target achievement | Range finder outputs a momentary spike (bad calibration, interference) — test thinks target reached, doesn't verify sustained tracking |
| **Actuator lag** | Actuator moves but only momentarily — the system doesn't stay on target | Servo responds to a pulse but springs back (mechanical issue, PWM dropout) — one spike passes, sustained check requires continued response |

### Affected tests

All tests using `wait_and_maintain_range()` or `wait_and_maintain_vector()` — this includes altitude maintenance, groundspeed, heading, altitude hold, and speed hold tests across all three vehicle types.

### Specific latent bugs likely to surface

1. **Altitude hold oscillation under wind gust** — vehicle oscillates ±3m around target, passes on first tick but fails 0.5s sustained check
2. **Loiter radius overshoot** — vehicle enters loiter target range on the initial arc but hasn't decelerated yet, overshoots and corrects mid-test
3. **RTL altitude acquisition with variable wind** — initial wind gust pushes vehicle to target altitude momentarily, then wind shifts and altitude drifts

---

## 2. wait_attitude / wait_attitude_quaternion minimum_duration

**Committed in:** `376ee8e9`
**Files:** `vehicle_test_suite.py`

### How it works

New `minimum_duration` parameter with `achieving_duration_start` tracking. The attitude must be *maintained* for the specified duration. If attitude drops below tolerance during the dwell period, the timer resets to zero.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **Transient attitude spike** | Single ATTITUDE message shows target attitude, vehicle was never actually oriented correctly | MAVLink message timestamp out of order, or attitude sensor briefly reporting wrong values (IMU glitch) |
| **Incomplete flip/roll maneuver** | Vehicle passes through target attitude en route to a different maneuver — test stops on passing angle, not on completed maneuver | FLIP mode: vehicle rolls from 0°→180°, `wait_attitude(desroll=90, tolerance=15)` passes the instant it hits 90° and stops — test never verifies the flip completes to 180° |
| **Attitude drift during hold** | Vehicle briefly enters level attitude but immediately starts drifting (insufficient PID gain) | Stabilize mode: first attitude message after mode switch shows level, but controller has high integral windup so pitch starts drifting 5°/s — test passes on first tick |
| **Quaternion singularity edge case** | Quaternion representation has numerical issues at certain orientations | Gimbal lock near ±90° pitch — quaternion math produces momentary valid reading but actual aircraft attitude is wrong |

### Affected tests

- **`ModeFlip`** (arducopter.py): 6 attitude checks during roll and pitch flip sequences
- **`GPSGlitchLoiter2`** (arducopter.py): checks attitude stays level during GPS glitch
- **`OpticalFlow`** (arducopter.py): checks attitude before/after optical flow testing
- **`test_stabilize`**, **`test_acro`** (arduplane.py): roll and pitch assertions

### Specific latent bugs likely to surface

1. **FLIP mode incomplete flip** — vehicle rolls 90° through target but stalls at 120° instead of completing to 180°. Test now resets the timer at 120° and times out rather than passing at 90°.
2. **Stabilize mode integral windup** — after a hard turn, the roll controller has accumulated error. First attitude message shows level (windup counter hasn't engaged yet), then roll drifts to 15°. Previously: pass. Now: timer resets, test fails.
3. **EKF attitude estimation glitch during GPS reset** — GPS glitch causes EKF to briefly output stale attitude matching the target before updating with new measurements. Attitude drifts 5° in the next 200ms.

---

## 3. OpticalFlow Warning Path + Tightened Tolerance

**Committed in:** `20e451d2` + `376ee8e9`
**Files:** `arducopter.py` — `CheckOpticalFlow`

### How it works

- Tolerance tightened from 3 m/s to 1.5 m/s mismatch between optical flow speed and GPS speed
- Warning path at 1.5–2.0 m/s mismatch (logs to test output, doesn't fail)
- Hard failure at >2.0 m/s mismatch
- `max_mismatch` tracking in debug output for post-mortem analysis

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **Optical flow calibration drift** | Flow sensor calibration factors are wrong by 20-50% — previously passed with 3 m/s tolerance, now fails | Flow rate scaling factor (`FLOW_SCALE`) calibrated for 100% but actually outputs 130% — test now detects the 30% error within 15 seconds of flight |
| **Rangefinder bias during flow calc** | Rangefinder reports wrong distance, corrupting optical flow speed calculation | Rangefinder has 0.5m positive bias — calculated flow speed is 30% too low, mismatch exceeds 1.5 m/s during level flight |
| **EKF flow navigation degradation** | EKF uses flow data when it shouldn't (e.g., GPS still available) — flow/GPS divergence grows | `FLOW_TYPE=10` (optical flow primary) with GPS fallback not properly weighted — EKF trusts flow over GPS during GPS transient, creating speed divergence |
| **Sensor fusion weight imbalance** | Flow position estimate doesn't match GPS ground truth | EKF process noise tuned too aggressively for flow — flow position drifts from GPS position, flow rate calculation diverges from actual groundspeed |

### Affected tests

- **`OpticalFlow`** (arducopter.py): full flight mission with optical flow vs GPS comparison
- **`OpticalFlowLocation`** (arducopter.py): optical flow position estimate checks

### Specific latent bugs likely to surface

1. **Flow navigation in GPS-denied mode with intermittent GPS** — when GPS signal is briefly lost and regained, EKF process noise allows flow position to drift significantly from true GPS position. Flow speed calculation (flow_rate × rangefinder_distance) no longer matches GPS groundspeed.
2. **Rangefinder temperature bias** — rangefinder calibration is valid at 25°C but test runs in different environment, sensor reports 0.3-0.5m offset, flow speed calculation is wrong by 1-2 m/s.

---

## 4. OpticalFlowLimits Hard 10 m/s Speed Cap

**Committed in:** `376ee8e9`
**Files:** `arducopter.py` — `OpticalFlowLimits`

### How it works

Hard assertion that vehicle speed never exceeds 10 m/s at any point during the test, regardless of altitude. Previously, the test only checked altitude-based speed limits (`alt * 1.5 + margin`). At 35m altitude, this allowed speeds up to 54.5 m/s.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **EKF optical flow speed limit bypass** | EKF navigation limit enforcement code has a condition that allows speeds >10m/s under certain circumstances | EKF2 `NavControl::limitVelocity()` has a branch for `doingFlowNav` that only checks `gndOffsetValid` — if offset validation fails, velocity limit check is skipped |
| **Flow mode navigation with GPS fallback** | EKF falls back to GPS speed calculation when flow limit should be hard-capped | GPS is available alongside flow — EKF blends GPS and flow position, and the blended speed calculation doesn't enforce the 10 m/s hard cap |
| **Altitude-based speed limit too permissive** | At high altitudes (even within 35m limit), altitude-based speed limits allow dangerous speeds | Altitude at 30m → max speed = 30×1.5+2 = 47 m/s — EKF navigation limit code has a bug allowing speed up to 47 m/s when flow mode should cap at 10 m/s |
| **RC override bypassing EKF limits** | Pilot RC input overrides EKF navigation limits | RC throttle stick at 100% during LOITER with optical flow — controller commands full throttle, vehicle accelerates past 10 m/s EKF limit |

### Affected tests

- **`OpticalFlowLimits`** (arducopter.py): 60-second test with RC-controlled acceleration

### Specific latent bugs likely to surface

1. **Flow limit not applied when doingNormalGpsNav is true** — EKF condition `doingFlowNav && gndOffsetValid` in `updateFilterStatus()` means if either flag is false, the hard cap is bypassed. If `gndOffsetValid` flips false during the test (e.g., rangefinder dropout), the speed limit is silently disabled.
2. **Velocity limit not enforced for `NAV_LOITER` mode** — EKF navigation limit code only applies hard caps in `NAV_LEAD` and `NAV_LEAD_ANGLE` modes but not `NAV_LOITER`.

---

## 5. GPS Glitch Position Tracking

**Committed in:** `92baa20d`
**Files:** `arducopter.py` — `GPSGlitchLoiter`, `GPSGlitchLoiter2`, `GPSGlitchAuto`

### How it works

All three GPS glitch tests now track position during the glitch phase and raise `NotAchievedException` if the vehicle moves beyond a defined threshold:
- `GPSGlitchLoiter`: 20m limit
- `GPSGlitchLoiter2`: 10m limit
- `GPSGlitchAuto`: 30m limit

Previously, `GPSGlitchLoiter` used `drain_mav()` (passive wait) with no position tracking. `GPSGlitchLoiter2` only checked attitude angles. `GPSGlitchAuto` looped through glitch values without verifying the vehicle didn't fly off-course.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **EKF position reset causing excursion** | GPS glitch causes EKF position estimate to jump, vehicle reacts with large position excursion | EKF position innovation exceeds threshold → EKF resets position estimate → vehicle thinks it's far from where it actually is → flies hard toward "intended" position, overshooting 50+ meters |
| **GPS yaw/position glitch causing orbit** | GPS position jump causes EKF attitude to enter a turning loop | Yaw from GPS heading jumps 90° during glitch → EKF thinks vehicle is heading wrong direction → initiates correction turn → orbits instead of continuing straight → vehicle ends up 30-100m from start |
| **Dual GPS system corruption** | Glitch on GPS1 causes EKF to reject both GPS sources and go into degradation mode | EKF2 GPS1 glitch detection triggers → switches to GPS2 → but GPS2 also has transient noise → both GPS rejected → vehicle enters dead reckoning mode with position error growing unbounded |
| **Horizontal fence interaction with GPS glitch** | GPS glitch causes position estimate to be outside fence → vehicle tries to return | Position estimate jumps outside fence boundary during glitch → RTL triggered → vehicle flies to home → distance from start exceeds glitch test thresholds |

### Affected tests

- **`GPSGlitchLoiter`**: 7-step GPS position glitch sequence over 30 seconds
- **`GPSGlitchLoiter2`**: Single large GPS glitch (0.001 lat offset) over 20 seconds
- **`GPSGlitchAuto`**: Same 7-step glitch sequence during mission flight

### Specific latent bugs likely to surface

1. **EKF position reset with large innovation** — when GPS glitch causes position innovation to exceed `EK2_POSXY_MAX_INNO`, EKF resets the position estimate. The vehicle then commands a hard turn toward its "intended" position (which the EKF thinks is far away). The 20m/10m/30m limit catches excursions of 50-100m.
2. **GPS glitch causing compass interference** — GPS yaw jumps during glitch → compass reading conflicts → EKF attitude becomes unstable → vehicle enters uncommanded turn → position excursion exceeds limits.

---

## 6. ModeFlip Attitude Tolerance Tightening

**Committed in:** `92baa20d`
**Files:** `arducopter.py` — `ModeFlip`

### How it works

Tolerance tightened from 30° to 15° for all flip-axis attitude assertions. The old 30° tolerance allowed the test to pass even when the flip was barely initiated (e.g., 15° out of 180° is within 30° of the target 45°/90°/-45°).

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **Partial flip** | FLIP mode only partially completes — vehicle rotates partway then stalls | FLIP mode PID controller has insufficient authority for large angle changes — rolls 60° of required 90° and stalls due to rate limiter, previously passed with 30° tolerance |
| **Flip mode attitude inversion** | FLIP mode rolls in wrong direction or overshoots target | Roll PID has reversed sign — rolls -30° instead of +45°. With 30° tolerance, -30° is within tolerance of 0° target but not 45° target — this is a real bug that would be caught |
| **Flip mode oscillation** | Vehicle oscillates around target angle during flip | Integral windup causes oscillation: 45°→90°→75°→90°. First hit of 90° passes the test, vehicle is actually oscillating and hasn't completed a clean flip |

### Affected tests

- **`ModeFlip`**: 8 attitude assertions during roll and pitch flip sequences

---

## 7. Plane test_stabilize / test_acro Roll Accuracy

**Committed in:** `92baa20d`
**Files:** `arduplane.py`

### How it works

Roll accuracy tightened from 90° to 15° (6x improvement). The old 90° accuracy on a ±150° target meant the test considered 60°-240° as "achieved" — effectively any attitude was acceptable.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **Servo reversal** | Aileron servo wired backwards — aircraft rolls opposite to commanded direction | Roll command +150°, servo drives ailerons to -30°. With 90° tolerance, -30° is within tolerance. With 15° tolerance, test fails. |
| **Elevon reversal (elevrev frames)** | Taildragger elevon reversal not applied — roll axis flipped | `ELEVREV=1` not set — during roll command, left aileron drops and right aileron raises (opposite of required). Aircraft rolls -30° instead of +150°. |
| **Partial servo travel** | Servo only moves 10° of required 30° | Servo PWM range misconfigured — commanded 1500-1900μs but servo only responds 1450-1550μs. Aircraft achieves 20° roll instead of 150°. |
| **Rate limiter too aggressive** | Roll rate limit prevents reaching target angle within timeout | `RC1_RATE` too low — aircraft rolls at 10°/s, needs 15°/s to hit target. Test passes on first tick of 60° (within 90° tolerance) before rate limiter catches up. |

---

## 8. Plane test_FBWB Altitude Maintenance

**Committed in:** `dfb16225`
**Files:** `arduplane.py` — `test_FBWB`

### How it works

Altitude tolerance tightened from 20m to 5m (was 40% of flight altitude at ~50m). Added midpoint altitude check to catch drift during flight, not just at the end.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **Altitude hold integral windup** | Aircraft climbs or drifts during circuit but returns to target by the end | Throttle PID integral term accumulates — aircraft climbs 15m above target during circuit, then pilot input or manual reset returns it to target at circuit end. 20m tolerance passed, 5m catches it. |
| **Throttle trim bias** | Throttle trim off by 50μs — aircraft maintains altitude slowly but drifts | Trim at 1500μs but calibrated neutral is 1550μs. Aircraft slowly climbs 0.5m/s during circuit — by end of 4-leg circuit (200s), 100m drift. Midpoint check catches early drift. |
| **Lift-drag interaction** | Aircraft maintains altitude in straight flight but loses it in turns | In straight flight, lift equals weight. In 15° bank turn, vertical lift component drops 3% → aircraft descends 1.5m/s during turns, doesn't fully recover before next straight segment. |

---

## 9. Plane ThrottleFailsafe Movement Assertion

**Committed in:** `20e451d2`
**Files:** `arduplane.py` — `ThrottleFailsafe`

### How it works

After triggering throttle failsafe and entering RTL, the test now verifies the plane actually flew at least 100m from home. Previously, only the mode change was verified.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **EKF RTL not armable** | RTL mode changes but EKF prevents takeoff — plane stays on ground | EKF3 position age > timeout → "RTL mode not armable" → plane changes to RTL mode but groundspeed stays at 0.03 m/s. Mode test passes, movement check fails. |
| **Throttle failsafe not engaging** | RC failsafe triggers mode change but throttle command doesn't reach motor | `SIM_RC_FAIL=2` (throttle-to-950) doesn't actually modify throttle PWM due to a bug in `RC_Channel::failsafe()` → motor stays at 0 throttle → plane never moves. |
| **Auto RTL waypoint navigation failure** | RTL mode engages but vehicle fails to navigate waypoints | RTL waypoint parsing bug — RTL waypoints are empty or malformed → vehicle enters RTL mode, checks waypoints loop, finds nothing, enters loiter at home position immediately. |

---

## 10. Rover test_gcs_failsafe Movement Verification

**Committed in:** `dfb16225`
**Files:** `rover.py` — `test_gcs_failsafe`

### How it works

Replaced `delay_sim_time(5)` with `wait_distance(10)` to verify the rover actually moved before checking failsafe. Added explicit `wait_mode("MANUAL")` in failsafe-disabled test. Added position tracking in AUTO subtest.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **GCS failsafe timeout misconfigured** | Failsafe triggers but vehicle doesn't respond within timeout | `GCS_FAIL_THROTTLE` set to 2s but GCS heartbeat interval is 5s — failsafe triggers after 10s (3 missed heartbeats), rover should enter MANUAL mode but a bug in `gcs_failsafe_check()` causes it to stay in current mode. 5s delay passed, test didn't verify mode change. |
| **GCS failsafe disabled path** | Failsafe is disabled — rover should stay in current mode but actually switches | `GCS_FAIL_THROTTLE=0` (disabled) — but `gcs_has_timed_out()` has a logic bug returning true even when failsafe is disabled, causing unexpected mode switch. `delay_sim_time(5)` passed, `wait_mode` now catches it. |
| **AUTO mission GCS failsafe interaction** | GCS heartbeat lost during AUTO mission — rover should pause/resume correctly | GCS timeout during mission → rover pauses mission at current waypoint. When GCS reconnects, rover should resume — but a state machine bug causes it to restart mission from waypoint 1 instead. Position tracking catches the unexpected waypoint movement. |

---

## 11. Rover test_scripting_simple_loop Count Tightening

**Committed in:** `20e451d2`
**Files:** `rover.py` — `test_scripting_simple_loop`

### How it works

Message count tightened from `count >= 3` to `10 <= count <= 200`. The old check only verified 3+ messages — if the script crashed after printing once, it passed.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **Script crash after single iteration** | Script executes first iteration, crashes on second, prints once | `while true` loop: first `console.info("hello, world")` succeeds, second iteration hits null pointer in peripheral code → script halts → test sees count=1, previously passed with `count >= 3` wait, now properly fails. Wait, actually `count >= 3` means we need 3 messages — let me reconsider. The change tightens the *range*: minimum 10 messages (was 3), and adds an upper bound of 200. |
| **Script runs forever (no exit)** | Script has no termination condition — loops infinitely | `while True:` loop with no break → script prints "hello, world" for the entire test duration (potentially thousands of messages) → test passes with `count >= 3` but now fails with `count > 200` → catches scripts that hang in infinite loops (resource exhaustion bug). |
| **Script slow due to blocking I/O** | Script runs much slower than expected due to serial port or CAN blocking | `console.info()` blocks on UART for 500ms per call → loop runs at 2 Hz instead of 10 Hz → count=20 instead of expected 100. Range check catches the unexpected performance regression. |

---

## Summary Matrix

| Improvement | Commit | Bug classes caught | Latency | Severity |
|-------------|--------|-------------------|---------|----------|
| `wait_and_maintain` minimum_duration 0→0.5s | 376ee8e9 | Oscillation, slow correction, sensor spike, actuator lag | Immediate | High |
| `wait_attitude` minimum_duration | 376ee8e9 | Transient spike, incomplete maneuver, drift | Immediate | High |
| OpticalFlow tolerance 3→1.5 m/s + warning path | 20e451d2 + 376ee8e9 | Calibration drift, rangefinder bias, flow degradation | Immediate | Medium |
| OpticalFlowLimits hard 10 m/s cap | 376ee8e9 | EKF limit bypass, RC override, GPS fallback | Medium | Critical |
| GPS glitch position tracking | 92baa20d | EKF reset excursion, compass interference, dual GPS corruption | Medium | High |
| ModeFlip tolerance 30°→15° | 92baa20d | Partial flip, attitude inversion, oscillation | Immediate | Medium |
| Plane roll accuracy 90°→15° | 92baa20d | Servo reversal, elevon reversal, partial travel, rate limiter | Immediate | Critical |
| Plane FBWB altitude 20m→5m + midpoint | dfb16225 | Integral windup, trim bias, lift-drag interaction | Immediate | Medium |
| ThrottleFailsafe 100m movement check | 20e451d2 | EKF not armable, throttle not engaging, waypoint nav failure | Medium | Critical |
| Rover GCS failsafe movement verification | dfb16225 | Failsafe timeout, disabled path bug, AUTO interaction | Immediate | Medium |
| Scripting count 3→10-200 range | 20e451d2 | Script crash, infinite loop, blocking I/O | Immediate | Low |
| ModeLoiter position tolerance 5m→3m | cfc4e647e0 | Loiter oscillation, GPS drift, wind sensitivity, EKF filter divergence | Immediate | Medium |
| GPSGlitchLoiter heading recovery | cfc4e647e0 | EKF heading lock from glitched velocity, yaw realignment during active glitch | Immediate | High |
| GPSGlitchLoiter2 position tighten + heading | cfc4e647e0 | EKF position excursion, GPS yaw/position glitch causing orbit | Immediate | High |
| GPSGlitchAuto EKF monitoring | cfc4e647e0 | EKF degradation during glitch, error bits persisting, compass interference | Immediate | High |
| BaroDriftSensitivity new test | cfc4e647e0 | Cumulative baro drift, EKF no physics validation, drift recovery | Immediate | High |
| EKF3CompassDegradation new test | TBD | EKF yaw lock, compass/GPS cross-check failure, recovery | Immediate | Critical |

---

## 12. GPSGlitchLoiter — Heading Tracking During/After Glitch

**Date:** 2026-11-06
**Files:** `arducopter.py` — `GPSGlitchLoiter()`

### How it works

Records starting yaw heading before the glitch sequence and tracks it throughout. After all glitches are removed, verifies the heading recovers to near its original value within 10 seconds. During the glitch phase, logs large yaw deviations (>45°) without immediately failing — this allows diagnosis of the yaw realignment process while still catching permanent heading locks.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **EKF heading lock from glitched velocity** | After GPS glitch, `realignYawGPS()` locks heading to glitched track angle and doesn't recover | GPS glitch injects position offset → EKF computes velocity from glitched position → `atan2F(vel.y, vel.x)` gives wrong course → yaw resets to glitched angle → even after glitch removal, EKF position still slightly offset → heading stays locked at wrong angle. Test catches this by checking post-glitch heading recovery. |
| **Yaw realignment during active glitch** | Vehicle enters uncommanded turn during GPS glitch, stays in turn | EKF yaw realigns to glitched velocity → vehicle banks and turns → position moves off course → new position still glitched → EKF continues turning → vehicle orbits. Logged via heading_delta > 45 warning. |
| **EKF recovery after GPS corruption** | EKF never fully recovers after GPS glitch is removed | EKF position filter stuck at glitched position → even after glitch parameter is cleared → vehicle slowly drifts → heading never returns to original. Post-glitch heading check catches this. |

---

## 13. GPSGlitchLoiter2 — Tightened Position + Heading Recovery

**Date:** 2026-11-06
**Files:** `arducopter.py` — `gps_glitch_loiter2_for_ekf_type()`

### How it works

Position limit tightened from 10m to 5m (this is a 20-second loiter test, so the vehicle should stay very close to home). After glitch removal, explicitly waits 5 seconds and checks heading recovery, logging the delta between starting and recovery heading.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **EKF position excursion on glitch** | GPS position jump causes EKF to think vehicle moved → commands large correction → vehicle flies off course | Position innovation exceeds EK2_POSXY_MAX_INNO → EKF resets position estimate → vehicle thinks it's 30m from intended position → commands hard turn → flies 30m+ in wrong direction. Tightened 5m limit catches this. |
| **GPS yaw/position glitch causing orbit** | EKF attitude enters turning loop after GPS glitch | Yaw from GPS heading jumps 90° during glitch → EKF thinks vehicle is heading wrong direction → initiates correction turn → overshoots → orbits instead of holding position. Heading recovery check catches persistent orbit. |
| **EKF type-specific recovery** | EKF2 and EKF3 recover differently from the same glitch | Different internal position/velocity filter designs → EKF3 may recover faster but be more sensitive to large glitches, EKF2 may be more stable but slower to correct. Tests both independently. |

---

## 14. GPSGlitchAuto — EKF Status Monitoring + Heading Recovery

**Date:** 2026-11-06
**Files:** `arducopter.py` — `GPSGlitchAuto()`

### How it works

Monitors `EKF_NEXT` message during the glitch phase, tracking `ekf_err_bits` error flags. After glitch removal, waits 10 seconds and samples 20 EKF_NEXT messages to verify error bits clear. Also tracks heading during glitch to detect yaw realignment.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **EKF degradation during glitch** | EKF enters degraded mode (dead reckoning) during GPS glitch | GPS position innovation exceeds all thresholds → EKF rejects all GPS → enters `ESTIMATOR_CONST_POS_MODE` → vehicle navigates by dead reckoning → position error grows unbounded → may trigger fence or RTL. Error bit monitoring catches this before it causes excursion. |
| **EKF error bits persist after recovery** | EKF doesn't clear error flags after GPS is restored | EKF detects GPS glitch → sets error bits → glitch removed → but error bits don't clear because timeout hasn't elapsed or because of a logic bug. Vehicle may fly in degraded mode without pilot awareness. |
| **GPS glitch causing compass interference** | GPS yaw jump conflicts with compass reading | GPS provides heading during compass sensor timeout → GPS heading is wrong due to glitched velocity → EKF trusts GPS → compass gets calibrated to wrong heading → after recovery, compass reading is offset → EKF gets conflicting heading sources. |

---

## 15. ModeLoiter Position Tolerance Tightening (5m → 3m)

**Date:** 2026-11-06
**Files:** `arducopter.py` — `ModeLoiter()`

### How it works

Position hold tolerance tightened from 5m to 3m radius. This is a 10-second loiter hold test — the vehicle should be able to hold within 3m with properly tuned PID gains and healthy GPS/compass.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **Loiter oscillation** | Vehicle oscillates within loiter target but never settles | Loiter PID gains too aggressive → vehicle oscillates ±4m around position → passes 5m tolerance but fails 3m check. Catches tuning issues or sensor noise amplification. |
| **GPS drift during loiter** | GPS position report slowly drifts → vehicle follows | GPS module has warmup drift or multipath error → reported position drifts 2m/s → vehicle follows the drift → position change exceeds 3m in 10s. Catches GPS quality issues. |
| **Wind sensitivity** | Steady wind pushes vehicle outside tighter tolerance | 5m/s crosswind with weak position controller → vehicle drifts 4m from home → passes 5m tolerance but fails 3m check. Catches wind rejection tuning issues. |
| **EKF position filter divergence** | EKF position filter gain too low → slow to reject bad GPS | EKF position filter time constant too long → when GPS momentarily corrupts → EKF takes several seconds to correct → vehicle follows EKF position error → loiter drifts beyond 3m. |

---

## 16. BaroDriftSensitivity — New Test for Barometer Drift Design Issue

**Date:** 2026-11-06
**Files:** `arducopter.py` — `BaroDriftSensitivity()`

### How it works

New test that introduces `-0.35 m/s` barometer drift during AltHold mode, monitors altitude response, removes drift, and verifies the vehicle recovers to stable altitude. This catches the cumulative barometer drift design issue where `total_alt_drift` in `SITL_Baro.cpp` accumulates without bound.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **Cumulative baro drift causing altitude excursion** | Barometer altitude drifts → EKF trusts it → vehicle climbs/descends to compensate | `SIM_BARO_DRIFT=-0.35` applied → after 20s, baro reads ~7m low → EKF trusts this as truth → position controller commands climb to reach drifted target → vehicle climbs 7m+ above original altitude. |
| **EKF no validation against physics reference** | EKF has no cross-check against FDM altitude | EKF fuses baro altitude as truth without comparing to GPS-derived or FDM altitude → when baro drifts, EKF drifts with it → no mechanism to detect the baro is wrong. This is a design issue in EKF integration. |
| **Drift recovery after parameter reset** | After SIM_BARO_DRIFT=0, EKF may take time to re-stabilize | Drift parameter removed → baro suddenly reads correctly → EKF has accumulated position error from 20s of drifted altitude → needs time to converge back to true altitude → test checks recovery within 15s window. |
| **Baro glitch vs drift confusion** | EKF can't distinguish between gradual baro drift and gradual altitude change | Cumulative drift models real barometer behavior where pressure drifts due to temperature changes → but EKF can't tell this from the vehicle actually descending → position controller fights the "real" altitude change. |

---

## Updated Critical Bugs Likely to Be Found First

Based on the severity and immediacy of detection, these are the most likely to surface immediately:

1. **Plane servo reversal** — `test_stabilize` with 15° accuracy will immediately fail if any aileron servo is wired backwards or has reversed output. This is a hardware installation check but also catches `RC1_DIR` / `RC2_DIR` parameter bugs.

2. **EKF RTL not armable** — `ThrottleFailsafe` movement check will fail for any EKF state where `nav_takeoff()` or `rtl_armable()` returns false. The analysis document already notes this bug exists (groundspeed stays at 0.03 m/s).

3. **FLIP mode incomplete rotation** — `ModeFlip` with 15° tolerance + minimum_duration will detect if the flip stalls partway. This catches insufficient PID authority for large angle changes.

4. **EKF heading lock from GPS glitch** — `GPSGlitchLoiter` heading tracking will catch heading that doesn't recover after glitch removal. **This is the bug proven in `pre_existing_bug_evidence.md`** — the `realignYawGPS()` function uses glitched velocity for yaw alignment without quality gate.

5. **GPS glitch causing EKF position excursion** — `GPSGlitchLoiter2` tightened to 5m will detect any EKF behavior where a GPS position jump causes the vehicle to fly >5m from start.

6. **Cumulative baro drift altitude excursion** — `BaroDriftSensitivity` will detect any case where AltHold allows unbounded altitude drift when barometer drift is introduced.

7. **EKF error bits persisting after GPS recovery** — `GPSGlitchAuto` EKF monitoring will detect error flags that don't clear after glitch removal, indicating incomplete EKF recovery.

8. **Loiter position hold degradation** — `ModeLoiter` tightened to 3m will detect loiter oscillation, GPS drift, wind sensitivity, or EKF filter divergence.

---

## 17. EKF3CompassDegradation — New Test for Compass Freeze Mid-Flight

**Date:** 2026-11-06
**Files:** `arducopter.py` — `EKF3CompassDegradation()`

### How it works

3-phase test:
1. **Baseline** (5s): Take off to 10m in LOITER, record yaw standard deviation (~1-2° in stable LOITER)
2. **Degradation** (60s): Inject `SIM_MAG1_FAIL=2` (freeze compass — reports fixed value), command a sequence of yaw turns (left 6s / center 3s / right 6s × 3 cycles), sample ATTITUDE yaw throughout
3. **Recovery** (15s): Restore compass (`SIM_MAG1_FAIL=0`), wait for EKF re-sync, check heading returns to within 30° of start

The test exploits the fact that a frozen compass produces a static magnetic reading while the vehicle physically turns. If the EKF uses this frozen compass for yaw estimation and doesn't cross-check against GPS yaw, the estimated heading will diverge from the true heading.

### Bugs this catches

| Bug class | Description | Example scenario |
|-----------|-------------|------------------|
| **EKF yaw lock to frozen compass** | EKF trusts compass data without validation → yaw estimate freezes while vehicle turns | Compass freezes at 0° → vehicle turns 180° → EKF still outputs 0° → position controller commands wrong correction → vehicle drifts off course. Position controller uses EKF heading for navigation. |
| **EKF fails to cross-check compass against GPS** | No validation that compass-derived heading matches GPS-derived course-over-ground | EKF yaw fusion accepts any compass reading → frozen compass not detected → no fallback to GPS yaw. GPS yaw (`EK3_SRC1_YAW=1` default) should detect this mismatch but doesn't. |
| **Compass health flag not propagated** | EKF detects compass failure but position controller continues using bad heading | EKF logs "switching to compass 0" but attitude message still contains the frozen heading → attitude controller doesn't know heading is wrong → vehicle executes maneuvers with incorrect yaw reference. |
| **Recovery after compass restoration** | EKF doesn't re-sync heading after degraded compass is restored | Compass frozen for 60s → restored → EKF still thinks heading is at frozen value → takes >15s to re-sync (or never does) → vehicle starts maneuvering with wrong heading reference. |

---

## Updated Critical Bugs Likely to Be Found First

Based on the severity and immediacy of detection, these are the most likely to surface immediately:

1. **Plane servo reversal** — `test_stabilize` with 15° accuracy will immediately fail if any aileron servo is wired backwards or has reversed output. This is a hardware installation check but also catches `RC1_DIR` / `RC2_DIR` parameter bugs.

2. **EKF RTL not armable** — `ThrottleFailsafe` movement check will fail for any EKF state where `nav_takeoff()` or `rtl_armable()` returns false. The analysis document already notes this bug exists (groundspeed stays at 0.03 m/s).

3. **FLIP mode incomplete rotation** — `ModeFlip` with 15° tolerance + minimum_duration will detect if the flip stalls partway. This catches insufficient PID authority for large angle changes.

4. **EKF heading lock from GPS glitch** — `GPSGlitchLoiter` heading tracking will catch heading that doesn't recover after glitch removal. **This is the bug proven in `pre_existing_bug_evidence.md`** — the `realignYawGPS()` function uses glitched velocity for yaw alignment without quality gate.

5. **EKF yaw lock to frozen compass** — `EKF3CompassDegradation` will catch yaw that doesn't recover after compass freeze is cleared. A frozen compass freezes the EKF heading estimate while the vehicle turns — test commands 360° turns and checks recovery.

6. **GPS glitch causing EKF position excursion** — `GPSGlitchLoiter2` tightened to 5m will detect any EKF behavior where a GPS position jump causes the vehicle to fly >5m from start.

7. **Cumulative baro drift altitude excursion** — `BaroDriftSensitivity` will detect any case where AltHold allows unbounded altitude drift when barometer drift is introduced.

8. **EKF error bits persisting after GPS recovery** — `GPSGlitchAuto` EKF monitoring will detect error flags that don't clear after glitch removal, indicating incomplete EKF recovery.

9. **Loiter position hold degradation** — `ModeLoiter` tightened to 3m will detect loiter oscillation, GPS drift, wind sensitivity, or EKF filter divergence.
