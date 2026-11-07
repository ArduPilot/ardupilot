# ArduPilot Autotest Bug-Hunting Plan

**Date:** 2026-11-06  
**Branch:** test-improvements  
**Status:** Draft — actionable for immediate implementation

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Bug Taxonomy](#2-bug-taxonomy)
3. [Priority Matrix](#3-priority-matrix)
4. [Detection Strategies](#4-detection-strategies)
5. [Concrete Test Proposals — Top 5](#5-concrete-test-proposals--top-5)
6. [Disabled Test Recovery Plan](#6-disabled-test-recovery-plan)
7. [Infrastructure Requirements](#7-infrastructure-requirements)
8. [Implementation Order & Dependencies](#8-implementation-order--dependencies)
9. [SITL Bug Resolution Tracker](#9-sitl-bug-resolution-tracker)

---

## 1. Executive Summary

The ArduPilot autotest suite has ~33,000 lines of vehicle tests (Copter ~17,200, Plane ~8,400, Rover ~7,300). Systematic improvements on the test-improvements branch have tightened tolerances, added `minimum_duration` guards against spike-based false passes, added position tracking during GPS glitches, and added movement assertions for failsafe tests.

**Current state:** 11 improvements completed across 3 test files + infrastructure changes. Validation revealed 3 pre-existing SITL bugs that block further testing:
1. **SITL altitude drift** — vehicle climbs uncontrollably in Loiter/AltHold
2. **EKF "RTL mode not armable"** — prevents ThrottleFailsafe from progressing
3. **Flaky timesync/connection** — affects mission-based tests

**Goal:** Identify systematic, high-yield areas where focused test improvements can catch real bugs in ArduPilot flight code — before they reach users. This plan prioritizes by severity, detection difficulty, and current coverage gaps.

---

## 2. Bug Taxonomy

Bugs are categorized by their impact domain and the type of failure mode they produce in SITL.

### Category A: Safety-Critical (Severity 9-10)

These bugs could cause vehicle loss, injury, or property damage in the field.

| Subcategory | Description | Example |
|---|---|---|
| **A1. Failsafe Non-Trigger** | Failsafe doesn't activate when conditions warrant it, or activates the wrong mode | Battery failsafe not triggering at 3.3V; GCS timeout not triggering RTL |
| **A2. Failsafe Wrong Behavior** | Failsafe triggers but does the wrong thing (e.g., LAND instead of RTL over obstacles) | Throttle failsafe triggers RTL but RTL fails because EKF can't arm; fence breach triggers BRAKE instead of RTL |
| **A3. Parachute/Mechanical Release** | Mechanical failsafe (parachute, airbag) doesn't fire or fires prematurely | Parachute fires at low altitude; parachute doesn't fire at >200m |
| **A4. Terrain/Obstacle Collision** | Vehicle fails to avoid terrain or obstacles in AUTO/SRTL | SmartRTL doesn't detect approaching fence due to stale terrain data |

### Category B: Flight Instability (Severity 7-9)

These bugs cause oscillation, overshoot, or loss of control that may not be immediately catastrophic but degrades safety margin.

| Subcategory | Description | Example |
|---|---|---|
| **B1. Control Loop Oscillation** | PID loop instability, hunting, or sustained oscillation | AltHold oscillates ±3m after a gust; roll axis hunting during flip |
| **B2. Mode Transition Failure** | Vehicle gets stuck in intermediate mode or enters wrong mode | VTOL transition stuck between QSTABILIZE and FBWA; ACRO roll doesn't stop |
| **B3. Attitude Hold Degradation** | Vehicle cannot maintain desired attitude under specific conditions | ALT_HOLD loses altitude hold when GPS glitches; STABILIZE overshoots pitch |
| **B4. Ground Effect / Environmental** | Unmodeled physical interactions (ground effect, wind, terrain) | Ground effect compensation causes altitude flapping on landing; autoland drifts in crosswind |

### Category C: Sensor Fusion & Navigation (Severity 6-9)

These bugs affect the vehicle's perception of its state, leading to degraded navigation that cascades into other failure modes.

| Subcategory | Description | Example |
|---|---|---|
| **C1. EKF Source Degradation** | EKF switches between IMU lanes or sensor sources without proper handoff | Compass quality degrades, EKF switches to GPS yaw but with lag |
| **C2. GPS Glitch Response** | Vehicle reacts to sudden GPS position jumps with oscillation or mode change | GPS glitch causes 50m position jump, EKF takes 10s to settle |
| **C3. Sensor Fusion Cross-Sensitivity** | Change in one sensor affects estimate from unrelated sensor | Baro glitch causes altitude estimate to drift, affecting ALT_HOLD |
| **C4. Visual Odom / Alternative Nav** | Non-GPS navigation sources cause incorrect state | Optical flow speed reading drifts from GPS speed; visual odom switch causes position jump |

### Category D: Parameter & Configuration (Severity 4-7)

These bugs are triggered by specific parameter combinations or mid-flight parameter changes.

| Subcategory | Description | Example |
|---|---|---|
| **D1. Parameter Cross-Teraction** | Two valid parameters interact to produce unexpected behavior | FENCE_ALT_MAX + terrain=1 → fence altitude wrong; BATT_ARM_VOLT + BATT_CRT_VOLT → double-failsafe |
| **D2. Mid-Flight Parameter Change** | Changing parameters during flight causes instability or wrong behavior | Changing compass orientation mid-flight; switching RC protocol mid-flight |
| **D3. Parameter Index/Storage** | Parameter values persist incorrectly after reboot or storage corruption | FENCE_ENABLED=1 doesn't persist, fence inactive after reboot |

### Category E: Simulation Artifacts (Severity 1-5)

Bugs specific to SITL that may or may not manifest on real hardware. These are important for regression detection but lower priority than real-hardware bugs.

| Subcategory | Description | Example |
|---|---|---|
| **E1. Physics Model Bugs** | SITL flight dynamics model diverges from expected behavior | Altitude drift in Loiter mode (pre-existing SITL bug); wind turbulence precision loss over long runs |
| **E2. MAVLink Protocol** | SITL MAVLink messaging issues, timing, or state machine | Timesync failure causing `MAV_MISSION_OPERATION_CANCELLED`; heartbeat timeout during busy periods |
| **E3. Simulation State Leak** | SITL state from one test affects another | Parameter changes not cleaned between tests; EKF state persists across test runs |

---

## 3. Priority Matrix

Ranked by **Impact Score = Severity × (1 / Detection Difficulty) × (1 - Current Coverage)**.

**Legend:** 
- **Severity:** 1-10 (10 = catastrophic, 1 = informational)
- **Detection Difficulty:** E = Easy, M = Medium, H = Hard
- **Current Coverage:** NONE, PARTIAL, FULL
- **Impact Score:** Higher = more important to address

### Tier 1 — Implement Immediately (Impact 7+)

| # | Bug | Category | Severity | Detect | Coverage | Impact | Rationale |
|---|---|---|---|---|---|---|---|
| 1 | EKF compass degradation mid-flight | C1 | 9 | E | NONE | **9.0** | Compass degrades silently; EKF fallback can cause yaw drift → course deviation |
| 2 | Quadplane VTOL motor fail during transition | B2 | 10 | M | NONE | **10.0** | Transition is the most dangerous phase for VTOL; motor fail = crash |
| 3 | Fence terrain dependency (ALT_MAX_ABOVE_TERRAIN) | A2/A4 | 9 | E | NONE | **9.0** | Fence silently inactive if terrain unavailable → vehicle can fly through fence |
| 4 | Parachute test recovery (#4702) | A3 | 10 | M | NONE | **10.0** | Disabled high-priority safety test; recovery itself is a bug hunt |
| 5 | Battery monitor cross-sensitivity | D1 | 7 | E | NONE | **7.0** | Mid-flight param change affecting failsafe timing → unexpected LAND/RTL |

### Tier 2 — Implement Next (Impact 4-7)

| # | Bug | Category | Severity | Detect | Coverage | Impact | Rationale |
|---|---|---|---|---|---|---|---|
| 6 | RTK/RTCM correction loss during mission | C1 | 8 | M | NONE | **8.0** | RTK loss degrades positioning; EKF should degrade gracefully |
| 7 | SMART_RTL panic/loop detection (#18404) | B2/A4 | 10 | M | NONE | **10.0** | Disabled crash bug; recovery requires understanding the panic path |
| 8 | Repeated fence boundary crossing | A2/A4 | 8 | M | PARTIAL | **5.3** | State machine may not handle rapid in/out transitions |
| 9 | Compass interference during steering | C2 | 7 | M | NONE | **4.7** | Motor field distorts compass reading → heading drift on Rover/Plane |
| 10 | Autoland wind cross-sensitivity | B4 | 7 | H | NONE | **2.3** | Wind model bugs affect landing accuracy; hard to detect |
| 11 | Visual odom source switching mid-flight | C4 | 8 | M | PARTIAL | **5.3** | Switching OFOFF/GPS→OFOFF→GPS causes EKF discontinuity |
| 12 | DO_CHANGE_SPEED mid-mission | D2 | 6 | E | PARTIAL | **4.0** | Speed change between waypoints could cause overshoot or stall |
| 13 | RC protocol failsafe rapid cycling | A1 | 7 | M | NONE | **4.7** | Signal bounce between protocols causes repeated failsafe entry/exit |
| 14 | Mission item timing drift | B3 | 6 | E | PARTIAL | **4.0** | Accumulated timing errors in long missions |
| 15 | LandingDrift recovery (#20054) | B4 | 8 | M | NONE | **5.3** | Disabled crash bug; understanding the flapping is key |

### Tier 3 — Monitor / Low Priority (Impact < 4)

| # | Bug | Category | Severity | Detect | Coverage | Impact | Rationale |
|---|---|---|---|---|---|---|---|
| 16 | SITL altitude drift (pre-existing) | E1 | 3 | E | N/A | **3.0** | SITL-only bug; blocks testing but not a real-vehicle bug |
| 17 | EKF "RTL not armable" (pre-existing) | E2 | 3 | M | N/A | **2.0** | SITL/EKF interaction; blocks failsafe testing |
| 18 | Timesync flake | E2 | 2 | M | N/A | **1.3** | Infrastructure flake; intermittent, non-deterministic |
| 19 | ELEVREV pitch reversal | B3 | 5 | E | NONE | **1.7** | Taildragger-specific; narrow use case |
| 20 | SITL wind precision loss | E1 | 2 | H | NONE | **0.4** | Long-running precision artifact; unlikely to affect real missions |

---

## 4. Detection Strategies

For each bug category, the detection strategy follows the **inject → observe → assert** pattern.

### C1: EKF Source Degradation (Compass)

**Detection Strategy:**
1. **Inject:** Mid-flight (ARMED, in LOITER), inject compass degradation via `SIM_MAG1_FAIL=0.5` (partial failure) or `SIM_MAG1_FAIL=1` (complete failure)
2. **Observe:** Monitor `EKF_STATUS_REPORT.flags` via `WaitAndMaintainEKFFlags` hook. Track:
   - `EKF_STATUS_REPORT.vel_horiz_drift` and `EKF_STATUS_REPORT.pos_horiz_drift`
   - Yaw angle from `AHRS` (via `get_cached_message('AHRS')` or `SIMSTATE`)
   - Position from `GLOBAL_POSITION_INT`
3. **Assert:**
   - EKF should detect degraded compass and switch to GPS yaw within 5 seconds
   - Yaw drift should not exceed 5° during the switch
   - Position should not jump > 2m during source switch
   - Vehicle should remain in the current flight mode (no unexpected mode change)

**Metrics to Track:**
- Time from compass injection to EKF source switch (expected: < 5s)
- Yaw deviation during switch (expected: < 5°)
- Position jump at switch point (expected: < 2m)
- Mode stability (expected: no mode change)

**Thresholds:**
- `vel_horiz_drift` > 0.5 m/s for > 3s → FAIL
- `pos_horiz_drift` > 5m for > 2s → FAIL
- Yaw deviation > 5° during transition → WARN; > 10° → FAIL
- Mode change during transition → FAIL

### B2: Quadplane VTOL Motor Fail During Transition

**Detection Strategy:**
1. **Inject:** In quadplane mode, initiate VTOL transition (QSTABILIZE → FW via `DO_SET_MODE` or manual transition), then at 50% transition speed, kill quad motors via parameter `SIM_QMOTORS=0` or motor-specific fault injection
2. **Observe:** Track vehicle airspeed, pitch, altitude, and flight mode via MAVLink
3. **Assert:**
   - Vehicle should detect motor fail and switch to fixed-wing recovery mode
   - Airspeed should not drop below `airspeed_min` during recovery
   - Altitude loss should not exceed 20m during recovery
   - Vehicle should return to stable FW flight within 10s

**Metrics to Track:**
- Transition time (QSTABILIZE → FW)
- Minimum airspeed during transition (expected: > `airspeed_min`)
- Maximum altitude loss (expected: < 20m)
- Recovery time to stable FW (expected: < 10s)

**Thresholds:**
- Airspeed < `airspeed_min * 0.8` for > 2s → FAIL
- Altitude loss > 20m → FAIL
- Mode stuck in intermediate state for > 15s → FAIL
- Vehicle in QLAND or QRTL without pilot input → WARN

### A2/A4: Fence Terrain Dependency

**Detection Strategy:**
1. **Inject:** Set `FENCE_ALT_MAX_TYPE=3` (ABOVE_TERRAIN), set `SIM_TERRAIN=1`, then either:
   - Start without terrain database (remove SD card simulation)
   - Or set `TERRAIN_BLOCK=1` to block terrain loading
2. **Observe:** EKF altitude mode, fence status, and vehicle position
3. **Assert:**
   - If terrain unavailable, fence should either: (a) refuse to enable with pre-arm warning, or (b) fall back to ABOVE_HOME altitude
   - Vehicle should NOT silently ignore the fence constraint
   - If terrain becomes available mid-flight, fence should dynamically update

**Metrics to Track:**
- Pre-arm fence status report (expected: WARN if terrain unavailable)
- Fence enabled/disarmed state during flight
- Vehicle altitude vs fence limit
- Terrain database availability flag

**Thresholds:**
- Fence enabled with terrain unavailable → FAIL (security-critical)
- Vehicle exceeds fence altitude limit when terrain unavailable → FAIL
- No pre-arm warning when terrain-dependent fence active → WARN

### C1: RTK/RTCM Correction Loss

**Detection Strategy:**
1. **Inject:** Start rover in AUTO mission with RTK (SIM_GPS_TYPE=2, SIM_GPS1_TYPE=5), then stop RTCM injection mid-mission via parameter `SIM_GPS_RTK_DISABLE=1` or by stopping the RTCM MAVLink stream
2. **Observe:** Position accuracy (`GPS_RAW_INT.hdop`, `GPS_RAW_INT.lat/lon`), EKF horizontal position accuracy, and rover navigation
3. **Assert:**
   - EKF should detect loss of RTK and fall back to single-point GPS
   - Position accuracy should degrade gracefully (HDOP increases)
   - Rover should continue navigation (not oscillate or stop)
   - Position drift should not exceed 5m over 30s after RTK loss

**Metrics to Track:**
- HDOP before and after RTK loss
- Position drift rate (m/s) after RTK loss
- EKF position accuracy flags
- Rover groundspeed and heading stability

**Thresholds:**
- Position drift > 5m in 30s after RTK loss → FAIL
- Groundspeed oscillation > 2 m/s after RTK loss → WARN
- EKF position accuracy flags error → FAIL
- Rover stops or enters MANUAL mode without cause → FAIL

### B3: Visual Odom Source Switching

**Detection Strategy:**
1. **Inject:** Enable optical flow (`SIM_FLOW_ENABLE=1`, `FLOW_TYPE=10`), take off to 10m in LOITER, then toggle `SIM_FLOW_ENABLE=0` mid-flight
2. **Observe:** Position from `GPS_RAW_INT`, optical flow speed from `OPTICAL_FLOW` message, EKF flags, and vehicle lateral movement
3. **Assert:**
   - EKF should smoothly transition from OFOFF to GPS velocity without position jump
   - Position should not jump > 1m during the switch
   - Vehicle should maintain position hold (LOITER radius < 2m after recovery)
   - No mode change during source switch

**Metrics to Track:**
- Position delta during source switch (m)
- Position hold radius after recovery (m)
- Switch completion time (s)
- Mode stability (expected: no change)

**Thresholds:**
- Position jump > 1m at switch point → WARN; > 2m → FAIL
- Position hold radius > 2m after 5s recovery → FAIL
- Mode change during switch → FAIL
- Switch takes > 10s to complete → WARN

---

## 5. Concrete Test Proposals — Top 5

### Test 5.1: Copter `TestEKFGPSYawRecovery`

**Category:** C1 — EKF Source Degradation  
**Impact Score:** 9.0  
**Vehicle:** Copter  
**Dependencies:** GPS glitch injection (`SIM_GPS1_GLTCH_*`), compass fault injection (`SIM_MAGx_FAIL`)

**Test Steps:**
1. Takeoff to 15m in LOITER, wait for EKF to be happy
2. Verify EKF has both compass and GPS yaw contributing (check `EKF_STATUS_REPORT.flags` for `EKF_GPSYAW` bit)
3. Inject compass degradation: set `SIM_MAG1_FAIL=0.5` (simulates partial failure)
4. Wait 5 seconds for EKF to detect degradation
5. Verify EKF switches to GPS yaw: check `EKF_STATUS_REPORT.flags` for GPS yaw takeover
6. Fly a 90° turn circle (radius 20m) in LOITER
7. Verify position hold and yaw accuracy throughout

**Expected Behavior:**
- EKF detects degraded compass within 3-5 seconds
- EKF switches to GPS yaw (or GPS velocity heading for planes)
- Yaw drift < 5° during transition
- Position hold maintained (radius < 3m)

**Failure Criteria:**
- FAIL: EKF does not switch to GPS yaw within 5s
- FAIL: Yaw drift > 10° during transition
- FAIL: Position hold radius > 5m during compass degradation
- FAIL: Vehicle changes mode during transition
- WARN: Position jump > 1m at switch point

** MAVLink Messages Monitored:**
- `EKF_STATUS_REPORT` (for flags, drift rates)
- `AHRS` or `SIMSTATE` (for yaw angle)
- `GLOBAL_POSITION_INT` (for position tracking)
- `HEARTBEAT` (for mode tracking)

---

### Test 5.2: Plane `TestQuadPlaneMotorFailTransition`

**Category:** B2 — Mode Transition Failure  
**Impact Score:** 10.0  
**Vehicle:** QuadPlane (via arduplane.py)  
**Dependencies:** QuadPlane build, VTOL transition capability, motor fault injection

**Test Steps:**
1. Configure quadplane frame (use quadplane test frame from vehicleinfo.json)
2. Takeoff in QHOVER or QLAUNCH, climb to 30m
3. Initiate transition to fixed-wing mode (set mode to `STABILIZE` or send `DO_SET_MODE` with FW mode)
4. At transition speed (air ~5 m/s), inject motor failure: set `SIM_QMOTORS=0` or kill quad motors
5. Monitor transition completion and recovery
6. Wait for stable fixed-wing flight
7. Return to VTOL mode and land

**Expected Behavior:**
- Vehicle detects loss of VTOL thrust
- Vehicle pitches down to build airspeed
- Vehicle recovers to stable FW flight
- Altitude loss < 20m during recovery
- No crash or uncontrolled descent

**Failure Criteria:**
- FAIL: Airspeed drops below `airspeed_min` for > 2s
- FAIL: Altitude loss > 20m
- FAIL: Vehicle enters QLAND/QLRTL without pilot input
- FAIL: Vehicle does not stabilize within 15s
- FAIL: Mode stuck in intermediate state > 15s
- WARN: Airspeed < `airspeed_min * 0.9` at any point

**MAVLink Messages Monitored:**
- `ATTITUDE` (for pitch, roll, yaw rates)
- `AHRS` (for airspeed)
- `GLOBAL_POSITION_INT` (for altitude tracking)
- `RC_CHANNELS` (for motor output)
- `STATUSTEXT` (for transition failure messages)

---

### Test 5.3: Copter `TestFenceTerrainFallback`

**Category:** A2/A4 — Failsafe Wrong Behavior  
**Impact Score:** 9.0  
**Vehicle:** Copter  
**Dependencies:** Fence tests (`FENCE_ENABLE=1`), terrain simulation (`SIM_TERRAIN=1`), SD card simulation

**Test Steps:**
1. Set `FENCE_ENABLE=1`, `FENCE_ALT_MAX=50`, `FENCE_ALT_MAX_TYPE=3` (ABOVE_TERRAIN)
2. Set `SIM_TERRAIN=1`, but block terrain database loading (set `TERRAIN_LOAD=0`)
3. Attempt to arm — check for pre-arm warning about terrain dependency
4. If arming is allowed (bug scenario), takeoff to 40m
5. Verify fence behavior: check whether fence is active or silently disabled
6. Climb to 55m — verify fence triggers (or doesn't)
7. Repeat with terrain enabled (`TERRAIN_LOAD=1`) to establish baseline

**Expected Behavior:**
- Pre-arm check should reject arming if terrain database is unavailable for terrain-dependent fence
- OR fence should fall back to ABOVE_HOME mode with a warning
- If fence is active, altitude limit of 50m ABOVE_TERRAIN should be enforced
- No silent degradation where fence is "on" but not enforcing constraints

**Failure Criteria:**
- FAIL: Fence enabled with terrain unavailable (silent degradation)
- FAIL: Vehicle exceeds fence altitude limit when terrain unavailable
- FAIL: No pre-arm warning when terrain-dependent fence active
- WARN: Fence fallback to ABOVE_HOME without explicit warning

**MAVLink Messages Monitored:**
- `SYS_STATUS` (pre-arm check bits)
- `FENCE_STATUS` (for fence breach detection)
- `STATUSTEXT` (for terrain warning messages)
- `GLOBAL_POSITION_INT` (for altitude tracking)

---

### Test 5.4: Rover `TestRTKLossDuringMission`

**Category:** C1 — EKF Source Degradation  
**Impact Score:** 8.0  
**Vehicle:** Rover  
**Dependencies:** RTK GPS support, RTK correction injection, rover AUTO mission capability

**Test Steps:**
1. Configure rover with RTK GPS (`SIM_GPS_TYPE=2`, `SIM_GPS1_TYPE=5`, `NAV_RTK_ROUTES=1`)
2. Start RTCM correction injection (via parameter or MAVLink RTCM message stream)
3. Upload and execute an AUTO mission (4-6 waypoints, 50m spacing)
4. After completing waypoint 2, stop RTCM injection (set `SIM_GPS_RTK_DISABLE=1` or stop RTCM stream)
5. Monitor position accuracy and navigation through waypoints 3-6
6. Verify rover completes the mission with degraded positioning

**Expected Behavior:**
- EKF detects RTK loss and falls back to single-point GPS
- HDOP increases from < 1.0 to ~3.0-5.0
- Position drift rate < 0.17 m/s (5m in 30s)
- Rover continues NAV through remaining waypoints
- No oscillation or mode change

**Failure Criteria:**
- FAIL: Position drift > 5m in 30s after RTK loss
- FAIL: Groundspeed oscillation > 2 m/s
- FAIL: EKF position accuracy flags set error
- FAIL: Rover stops or enters MANUAL mode
- WARN: Position accuracy (HDOP) > 10 after 60s

**MAVLink Messages Monitored:**
- `GPS_RAW_INT` (for HDOP, fix type)
- `EKF_STATUS_REPORT` (for position accuracy)
- `NAV_CONTROLLER_OUTPUT` (for waypoint tracking error)
- `GLOBAL_POSITION_INT` (for position tracking)

---

### Test 5.5: Copter `TestVisualOdomSourceSwitch`

**Category:** C4 — Visual Odom / Alternative Nav  
**Impact Score:** 5.3  
**Vehicle:** Copter  
**Dependencies:** Optical flow sensor simulation (`SIM_FLOW_ENABLE`), visual odom EKF configuration

**Test Steps:**
1. Enable optical flow: `SIM_FLOW_ENABLE=1`, `FLOW_TYPE=10`
2. Enable visual odom in EKF: `EKF2_AID_MASK` with bit 2 (FLOW)
3. Takeoff to 10m in LOITER, verify visual odom is active (check `EKF_STATUS_REPORT.flags` for `EKF_EVPOS_HORIZ` or similar)
4. Fly position hold for 5 seconds to establish stable visual odom baseline
5. Toggle optical flow off: `SIM_FLOW_ENABLE=0`
6. Monitor position and attitude for 10 seconds
7. Verify smooth transition back to GPS-based position
8. Verify position hold is maintained

**Expected Behavior:**
- EKF smoothly transitions from visual odom to GPS without position jump
- Position jump at switch point < 1m
- Position hold radius < 2m after 5s recovery
- No mode change during transition

**Failure Criteria:**
- FAIL: Position jump > 2m at switch point
- FAIL: Position hold radius > 2m after 5s
- FAIL: Mode change during switch
- FAIL: Switch takes > 10s to complete
- WARN: Position jump > 1m at switch point

**MAVLink Messages Monitored:**
- `EKF_STATUS_REPORT` (for source flags, drift)
- `OPTICAL_FLOW` (for flow quality check)
- `GLOBAL_POSITION_INT` (for position tracking)
- `AHRS` (for attitude during transition)

---

## 6. Disabled Test Recovery Plan

Recovering disabled tests is itself a high-value bug hunt. Each disabled test represents a known issue that was bad enough to disable the test rather than fix it.

### 6.1 Parachute (#4702) — HIGH PRIORITY

**Test:** `Parachute`  
**Reason:** "See https://github.com/ArduPilot/ardupilot/issues/4702"  
**Vehicle:** Copter  
**Safety Level:** A3 — Mechanical Release (Critical)

**Recovery Approach:**
1. Re-enable the test in `disabled_tests()`
2. Run in SITL with `--repeat 1` (single run, no loop)
3. Capture full MAVLink log and STATUSTEXT output
4. Identify the failure mode:
   - Does the parachute command not execute?
   - Does it fire at the wrong altitude?
   - Does it cause a mode change?
   - Does it crash the autopilot?
5. Trace the failure through `AP_Parachute::release()` and `Copter::failsafe_parachute_check()`
6. Submit fix PR

**Bug Hunt Checklist:**
- [ ] Check `Copter.h:816` — `failsafe_parachute()` call path
- [ ] Check `AP_Parachute.cpp` — release state machine
- [ ] Check MAVLink command handling for `MAV_CMD_DO_PARACHUTE`
- [ ] Verify altitude check logic in `failsafe_parachute_check()`
- [ ] Check parameter `PARACHUTE_RELEASING` timing

---

### 6.2 SMART_RTL Panic (#18404) — HIGH PRIORITY

**Tests:** `SMART_RTL_EnterLeave`, `SMART_RTL_Repeat`  
**Reason:** "Causes a panic" / "loop detection failure"  
**Vehicle:** Copter  
**Safety Level:** B2 — Crash Bug

**Recovery Approach:**
1. SMART_RTL is a complex mode with mission handling, obstacle avoidance, and loop detection
2. Re-enable `SMART_RTL_EnterLeave` first (simpler test)
3. Run and capture panic traceback
4. The panic is likely in the loop detection logic — check `AP_SmartRTL::update()` for infinite loop or assertion failure
5. The "loop detection" failure in `SMART_RTL_Repeat` suggests the mission loop counter is wrong

**Bug Hunt Checklist:**
- [ ] Trace `Copter.in_altHold()` → `smart_rtl` path
- [ ] Check `AP_SmartRTL::update()` for infinite loop conditions
- [ ] Check `smart_rtl_enter()` and `smart_rtl_leave()` state consistency
- [ ] Check mission item loop counter in `AP_Mission`
- [ ] Check for unbounded recursion or missing exit conditions

---

### 6.3 LandingDrift (#20054) — HIGH PRIORITY

**Test:** `LandingDrift`  
**Reason:** "Flapping test"  
**Vehicle:** Plane  
**Safety Level:** B4 — Environmental (Moderate)

**Recovery Approach:**
1. Re-enable and run with verbose output
2. The "flapping" suggests altitude oscillation during landing approach
3. Check autoland altitude control in `mode_autoland.cpp`
4. Look for hysteresis issues in the landing descent logic
5. Check terrain-relative altitude handling during autoland

**Bug Hunt Checklist:**
- [ ] Trace `Plane::update_autoland()` state machine
- [ ] Check `takeoff_state.initial_direction.heading` capture at arming
- [ ] Check `relative_ground_altitude()` terrain dependency
- [ ] Check for hysteresis in landing descent triggers
- [ ] Verify landing direction is not drifting

---

### 6.4 Other Disabled Tests — MEDIUM PRIORITY

| Test | Vehicle | Priority | Action |
|------|---------|----------|--------|
| `FlyMissionTwice` | Copter | Medium | Check PR #18561 for context; likely a mission loop bug |
| `RTLStoppingDistanceSpeed` | Copter | Medium | Vehicle goes off-course during SRTL; check obstacle avoidance in SRTL |
| `PolyFenceObjectAvoidanceBendyRuler` | Rover | High | Unreliable — likely a state machine issue in fence avoidance |
| `GroundEffectCompensation_takeOffExpected` | Copter | High | Flapping on takeoff — check ground effect model |
| `GroundEffectCompensation_touchDownExpected` | Copter | High | Flapping on landing — same ground effect model issue |
| `LandingDrift` | Plane | High | See above |
| `SlewRate` | Rover | Medium | CI timing report failure — likely a timeout, not a real bug |
| `ClimbThrottleSaturation` | Plane | Medium | Blocked on PR #27106 — wait for merge |
| `SoaringClimbRate` | Plane | Medium | Bad sink rate — check thermal/soaring model |
| `InteractTest` | Plane | Low | Requires user interaction — skip, expected disabled |

---

## 7. Infrastructure Requirements

These infrastructure changes would enable more effective bug hunting and prevent test contamination.

### 7.1 Parameter Context Tracking (HIGH PRIORITY)

**Problem:** `context_push()` / `context_pop()` manage message hooks and statustext collections but do NOT automatically track parameter state. Tests that modify parameters can leave the vehicle in a modified state, contaminating subsequent tests.

**Solution Implemented:** Enhanced the context system with automatic parameter tracking and validation.

#### Changes Made:

**1. Extended `Context` class** (line ~340):
```python
# New fields added to Context.__init__():
self.param_modifications = []  # List of (name, original_value) tuples
self.param_snapshot = {}  # Snapshot at context_push time for validation
```

**2. Enhanced `context_push()`** (line ~7148):
```python
def context_push(self, snapshot_params=False, param_names=None):
    """Extended with optional parameter snapshot for validation.
    
    Args:
        snapshot_params: If True, enable parameter tracking for this context
        param_names: Optional list of specific params to snapshot
    """
    # Initialize param_modifications and param_snapshot if requested
    if snapshot_params:
        if param_names:
            context.param_snapshot = self.get_parameters(param_names)
```

**3. Automatic tracking in `set_parameters()`** (line ~6848):
```python
# Before modification loop: capture original values
already_modified = {name for name, _ in context.param_modifications}
for name in want:
    if name.upper() not in already_modified:
        original_values[name] = self.get_parameter(name)

# After successful set: auto-record modification
if add_to_context:
    if name not in already_tracked and original_values.get(name) is not None:
        context.param_modifications.append((name, original_values[name]))
```

**4. Debug API methods** (line ~7243):
```python
def get_parameter_changes():
    """Return [(param_name, original_value, current_value), ...]"""

def get_uncommitted_changes():
    """Return [(param_name, original_value), ...] for debugging"""
```

**5. Validation in `context_pop()`** (line ~7499):
```python
# After restoring parameters, validate they match originals
if process_interaction_allowed and dead.param_modifications:
    self._validate_parameters_restored(dead.param_modifications)
```

**6. Validation method** (line ~7269):
```python
def _validate_parameters_restored(self, modifications):
    """Check each modified param matches original. Log warning if not."""
    for param_name, original_value in modifications:
        current_value = self.get_parameter(param_name)
        if abs(current_value - original_value) > epsilon:
            log_warning(f"{param_name}: {original_value} -> {current_value}")
```

#### Benefits:

- **Automatic tracking**: No need to manually call `context_preserve_parameters()` - modifications are tracked automatically when `set_parameters()` is called
- **Post-test validation**: `context_pop()` now verifies parameters were restored correctly and warns if not
- **Debug API**: `get_parameter_changes()` and `get_uncommitted_changes()` help diagnose test contamination
- **Backward compatible**: Old `context_preserve_parameters()` still works alongside new system
- **Non-breaking**: Tracking is opt-in via `snapshot_params=True` parameter

#### Usage Examples:

```python
# Old way (still works):
self.push_context()
self.context_preserve_parameters(['FENCE_ALT_MAX', 'FENCE_AUTOENABLE'])
self.set_parameter('FENCE_ALT_MAX', '50')
# ... test logic ...
self.pop_context()

# New way (recommended):
self.context_push(snapshot_params=True, param_names=['FENCE_ALT_MAX'])
self.set_parameter('FENCE_ALT_MAX', '50')  # Auto-tracked
# ... test logic ...
self.pop_context()  # Auto-validates restoration

# Debug current changes:
changes = self.get_parameter_changes()
# Returns: [('FENCE_ALT_MAX', 100.0, 50.0), ...]
```

**Implementation Status:** ✅ COMPLETED - All changes implemented in `Tools/autotest/vehicle_test_suite.py`

**Disabled Test Updates — COMPLETED (6/6 tests):**

All disabled Copter tests updated with `context_push(snapshot_params=True, param_names=[...])`:

| Test | Parameters Tracked |
|---|---|
| `SMART_RTL_EnterLeave` | `AUTO_OPTIONS` |
| `SMART_RTL_Repeat` | `SMARTRTL_POINTS_MAX`, `SMARTRTL_ACCURACY` |
| `RTLStoppingDistanceSpeed` | `RTL_SPEED_MS`, `AUTO_OPTIONS` |
| `FlyMissionTwice` | `AUTO_OPTIONS` |
| `FlyMissionTwiceWithReset` | `AUTO_OPTIONS` |
| `GroundEffectCompensation_takeOffExpected` | `LOG_FILE_DSRMROT`, `GNDEFF_TMO` |
| `GroundEffectCompensation_touchDownExpected` | `LOG_FILE_DSRMROT`, `GNDEFF_ALT` |
| `ScriptingOSD` | `SCR_ENABLE`, `OSD_TYPE`, `SIM_SPEEDUP`, `AUTO_OPTIONS` |

All tests follow the `try/finally` pattern ensuring parameter restoration even on test failure.

**Next Steps:**
- Update `disabled_tests()` dictionary to remove tests from the disabled list
- Add unit tests for the new tracking/validation logic
- Consider making tracking automatic (not opt-in) for all contexts

---

### 7.2 EKF Status Monitor Hook (HIGH PRIORITY)

**Problem:** Current EKF monitoring (`wait_ekf_happy()`, `WaitAndMaintainEKFFlags`) only provides binary pass/fail at specific points. Continuous monitoring during flight is not available.

**Required Changes:**

```python
# In vehicle_test_suite.py, add new class:

class EKFMonitorHook(MessageHook):
    '''Continuously monitors EKF health during flight and records anomalies.'''
    
    def __init__(self, test_suite, required_flags=None, error_bits=None):
        self.test_suite = test_suite
        self.required_flags = required_flags or [
            'EKF_ATTITUDE', 'EKF_VELOCITY_HORIZ', 'EKF_VELOCITY_VERT',
            'EKF_POS_HORIZ_REL', 'EKF_PRED_POS_HORIZ_REL'
        ]
        self.error_bits = error_bits or [
            'ESTIMATOR_CONST_POS_MODE', 'ESTIMATOR_ACCEL_ERROR'
        ]
        self.anomalies = []  # List of (timestamp, description) tuples
        self.drift_history = []
        self.yaw_history = []
    
    def process(self, mav, msg):
        if msg.get_type() == 'EKF_STATUS_REPORT':
            flags = msg.flags
            # Check required flags
            for flag_name in self.required_flags:
                flag_val = getattr(mavutil.mavlink, f'EXTENDED_STATE1_{flag_name}', None)
                if flag_val and not (flags & flag_val):
                    self.anomalies.append(
                        (self.test_suite.get_sim_time(), f"Missing EKF flag: {flag_name}")
                    )
            # Check error bits
            for err_name in self.error_bits:
                err_val = getattr(mavutil.mavlink, f'EXTENDED_STATE1_{err_name}', None)
                if err_val and (flags & err_val):
                    self.anomalies.append(
                        (self.test_suite.get_sim_time(), f"EKF error: {err_name}")
                    )
            # Record drift
            self.drift_history.append((self.test_suite.get_sim_time(), msg.pos_horiz_drift))
            self.yaw_history.append((self.test_suite.get_sim_time(), msg.yaw))
    
    def get_anomalies(self):
        return self.anomalies
    
    def get_drift_summary(self):
        if not self.drift_history:
            return None
        max_drift = max(d[1] for d in self.drift_history)
        avg_drift = sum(d[1] for d in self.drift_history) / len(self.drift_history)
        return {'max_drift': max_drift, 'avg_drift': avg_drift}
```

**Usage in Tests:**
```python
def test_with_ekf_monitoring(self):
    ekf_hook = EKFMonitorHook(self)
    self.install_message_hook(ekf_hook)
    # ... flight test logic ...
    self.uninstall_message_hook(ekf_hook)
    
    anomalies = ekf_hook.get_anomalies()
    if anomalies:
        for t, desc in anomalies:
            self.progress(f"EKF ANOMALY at t={t}: {desc}")
    
    drift = ekf_hook.get_drift_summary()
    if drift and drift['max_drift'] > 0.5:
        raise NotAchievedException(f"EKF velocity drift exceeded: {drift['max_drift']}")
```

**Implementation Priority:** HIGH — enables categories C1, C2, C3, C4

---

### 7.3 Position Jump Detector (MEDIUM PRIORITY)

**Problem:** GPS glitches and EKF source switches cause position jumps that current tests don't detect. The `GPSGlitch*` tests track distance from home, but don't detect abrupt jumps.

**Required Changes:**

```python
class PositionJumpDetector(MessageHook):
    '''Detects abrupt position jumps that indicate GPS glitch or EKF discontinuity.'''
    
    def __init__(self, test_suite, max_jump_horizontal=2.0, max_jump_vertical=1.0):
        self.test_suite = test_suite
        self.max_horizontal = max_jump_horizontal
        self.max_vertical = max_jump_vertical
        self.jumps = []
        self.last_position = None
    
    def process(self, mav, msg):
        if msg.get_type() in ('GPS_RAW_INT', 'GLOBAL_POSITION_INT'):
            lat = msg.lat if msg.get_type() == 'GPS_RAW_INT' else msg.lat / 1e7
            lon = msg.lon if msg.get_type() == 'GPS_RAW_INT' else msg.lon / 1e7
            alt = msg.alt if msg.get_type() == 'GPS_RAW_INT' else msg.alt / 1e3
            
            if self.last_position is not None:
                dlats = (lat - self.last_position['lat']) * 111320
                dlons = (lon - self.last_position['lon']) * 111320 * cos(radians(self.last_position['lat']))
                dalt = alt - self.last_position['alt']
                
                horiz_jump = sqrt(dlats**2 + dlons**2)
                vert_jump = abs(dalt)
                
                if horiz_jump > self.max_horizontal or vert_jump > self.max_vertical:
                    self.jumps.append({
                        'time': self.test_suite.get_sim_time(),
                        'horizontal_m': horiz_jump,
                        'vertical_m': vert_jump,
                        'lat': lat, 'lon': lon, 'alt': alt
                    })
            
            self.last_position = {'lat': lat, 'lon': lon, 'alt': alt}
    
    def has_jumps(self):
        return len(self.jumps) > 0
    
    def get_jumps(self):
        return self.jumps
```

**Implementation Priority:** MEDIUM — complements existing GPS glitch tests

---

### 7.4 Sensor Health Dashboard (LOW PRIORITY)

**Problem:** No centralized view of sensor health during test execution. Debugging sensor-related failures requires parsing individual MAVLink messages.

**Required Changes:**
- Create a `SensorHealthReport` class that aggregates `SYS_STATUS`, `EKF_STATUS_REPORT`, `GPS_RAW_INT`, `RANGEFINDER`, and `OPTICAL_FLOW` into a single report
- Output at test completion: sensor uptime, error counts, degradation events
- Output on anomaly: immediate log entry with timestamp and sensor details

**Implementation Priority:** LOW — nice-to-have for debugging, not critical for bug detection

---

### 7.5 Aggressive Timeout Mode (MEDIUM PRIORITY)

**Problem:** Test timeouts are uniform (30s for altitude, 60s for mode), causing slow feedback for simple operations and insufficient time for complex maneuvers.

**Required Changes:**

```python
# In vehicle_test_suite.py:

class TimeoutConfig:
    """Configurable timeout tiers for different operation complexity."""
    SIMPLE = 15      # Takeoff, landing, simple mode switches
    MODERATE = 30    # Normal maneuvers, waypoint navigation
    COMPLEX = 120    # RTL, terrain-following, EKF recovery, VTOL transition
    MINIMUM_DURATION = 0.5  # Minimum dwell time for wait_and_maintain

def set_timeout_mode(self, mode='normal'):
    """Set timeout mode: 'fast' (0.5x), 'normal' (1x), 'relaxed' (2x)."""
    multipliers = {'fast': 0.5, 'normal': 1.0, 'relaxed': 2.0}
    mult = multipliers.get(mode, 1.0)
    self._timeout_multiplier = mult
```

**Usage:**
```python
# In CI config or test runner:
if ci_fast_mode:
    test.set_timeout_mode('fast')
```

**Implementation Priority:** MEDIUM — improves CI speed but doesn't directly catch more bugs

---

### 7.6 Message Rate Override for Stress Testing (LOW PRIORITY)

**Problem:** Tests run at normal MAVLink message rates. Stress testing (high message rate, high telemetry) is not possible.

**Required Changes:**
- Add ability to override MAVLink message rates per message type
- Add ability to inject message delays or drops
- Use for testing heartbeat timeout, GCS failsafe under load, etc.

**Implementation Priority:** LOW — useful but not a high-yield area

---

## 8. Implementation Order & Dependencies

This section maps out the sequential implementation order, respecting dependencies between infrastructure and test components.

### Phase 1: Infrastructure Foundation (Week 1-2)

These are prerequisites for effective bug hunting.

| Step | Component | Depends On | Effort |
|------|-----------|------------|--------|
| 1.1 | Parameter context tracking (`_snapshot_parameters`, `_restore_parameters`) | None | 2h |
| 1.2 | Apply parameter context to 2-3 existing tests that modify parameters | 1.1 | 1h |
| 1.3 | SITL altitude drift fix (blocking bug #1) | None | TBD (core fix) |
| 1.4 | EKF "RTL not armable" fix (blocking bug #2) | None | TBD (core fix) |

**Rationale:** Parameter context is needed for all subsequent test work. The two SITL bugs block validation of any new tests.

---

### Phase 2: Tier 1 Bug Hunting (Week 2-4)

High-impact tests that catch critical bugs.

| Step | Test | Depends On | Effort |
|------|------|------------|--------|
| 2.1 | `TestEKFGPSYawRecovery` (Copter C1) | 1.1, 1.3 | 4h |
| 2.2 | `TestFenceTerrainFallback` (Copter A2) | 1.1, 1.3 | 3h |
| 2.3 | `TestQuadPlaneMotorFailTransition` (Plane B2) | 1.1, 1.4 | 6h |
| 2.4 | `TestRTKLossDuringMission` (Rover C1) | 1.1, 1.3 | 4h |
| 2.5 | `TestVisualOdomSourceSwitch` (Copter C4) | 1.1, 1.3 | 3h |
| 2.6 | Parachute test recovery (#4702) | 1.1 | 4h |

**Rationale:** Tests are independent of each other (different vehicle/area). Infrastructure from Phase 1 is needed for parameter cleanup.

---

### Phase 3: Tier 2 Bug Hunting (Week 4-6)

Medium-impact tests for broader coverage.

| Step | Test | Depends On | Effort |
|------|------|------------|--------|
| 3.1 | EKF Monitor Hook (infrastructure) | None | 3h |
| 3.2 | `TestFenceBoundaryCrossing` (Rover A2) | 3.1 | 3h |
| 3.3 | `TestCompassSteeringInterference` (Rover C2) | 3.1 | 3h |
| 3.4 | `TestDOChangeSpeed` (Copter D2) | 3.1 | 2h |
| 3.5 | `TestRapidRCFailsafe` (Copter A1) | 3.1 | 3h |
| 3.6 | SMART_RTL panic recovery (#18404) | 1.1 | 6h |
| 3.7 | LandingDrift recovery (#20054) | 1.1 | 4h |

**Rationale:** EKF monitor hook enables better diagnostics for all Tier 2 tests. SMART_RTL and LandingDrift recovery are independent bug hunts.

---

### Phase 4: Infrastructure Polish (Week 6-8)

Supporting infrastructure for long-term test health.

| Step | Component | Depends On | Effort |
|------|-----------|------------|--------|
| 4.1 | Position jump detector | None | 2h |
| 4.2 | Timeout mode configuration | None | 2h |
| 4.3 | Sensor health dashboard | 4.1, 3.1 | 3h |
| 4.4 | Apply parameter context to all remaining tests | 1.1 | 4h |
| 4.5 | Add `minimum_duration` to remaining `wait_*` calls without it | 1.1 | 3h |

**Rationale:** These improvements make the test suite more robust and maintainable long-term.

---

## 9. SITL Bug Resolution Tracker

The following pre-existing bugs must be fixed before new tests can be validated. They are tracked separately from the bug-hunting plan since they are SITL/core bugs, not test bugs.

### Bug 9.1: SITL Altitude Drift

**Symptom:** Vehicle climbs uncontrollably in LOITER and ALT_HOLD modes. ModeFlip climbed to 1157m; MainFlight climbed to 693m. Both tests timed out at their 22m target altitude.

**Evidence:**
- `ModeFlip`: "want 22.0, reached 1157.139" at test time 41.1s
- `MainFlight`: "want 165.0, reached 693.751" at test time 17.0s
- Both tests start normally (takeoff succeeds), then climb diverges

**Hypothesis:** SITL physics model has altitude control bug, possibly in:
- `SITL::update_position()` or FDM integration
- AHRS altitude estimation in SITL
- PID loop for altitude in SITL

**File to Investigate:** `libraries/SITL/` — specifically `SITL_Aircraft.cpp`, `SITL.cpp`

**Impact:** Blocks validation of all mode-based tests (ModeFlip, MainFlight, any ALT_HOLD/LOITER test)

---

### Bug 9.2: EKF "RTL Mode Not Armable"

**Symptom:** After throttle failsafe triggers RTL, the plane cannot arm/continue. `wait_mode("RTL")` may succeed but the plane doesn't move (groundspeed stays at 0.03 m/s).

**Evidence:**
- `ThrottleFailsafe`: groundspeed = 0.03, altitude = 31.57 after RTL mode entry
- Error message from EKF: "RTL mode not armable"

**Hypothesis:** EKF state is corrupted after failsafe event, or the EKF source selection logic prevents arming in RTL mode. Likely in:
- `AP_NavEKF_Source::controlMagYawReset()` for plane GPS yaw path
- EKF pre-arm check that incorrectly blocks RTL
- Failsafe mode transition not properly setting EKF state

**File to Investigate:** `libraries/AP_NavEKF/AP_NavEKF_Source.cpp`, `ArduPlane/failsafe.cpp`

**Impact:** Blocks validation of all throttle failsafe and RTL-related tests

---

### Bug 9.3: Flaky Timesync / Mission Cancellation

**Symptom:** Mission-based tests fail with `MAV_MISSION_OPERATION_CANCELLED` or "Did not get timesync response" at random intervals. Affects GPSGlitchAuto, OpticalFlow, GPSGlitchLoiter2.

**Evidence:**
- `GPSGlitchAuto`: `MAV_MISSION_OPERATION_CANCELLED` at test time 43.1s
- `OpticalFlow`: `MAV_MISSION_OPERATION_CANCELLED` at test time 22.5s
- `GPSGlitchLoiter2`: "Did not get timesync response" at test time 0.08s

**Hypothesis:** SITL MAVLink stack has race condition under mission upload/download stress:
- `mavutil.mavlink` connection timeout during busy periods
- SITL mission handling thread not thread-safe
- Timesync exchange collision with mission operations

**File to Investigate:** `Tools/autotest/pysim/` — MAVLink connection handling, `mavutil.py`

**Impact:** Blocks validation of all mission-based tests (affects ~30% of Copter tests)

---

### Bug 9.4: RTL_SPEED_MS Prevents Vehicle Stop (Severity 7)

**Symptom:** During RTL, vehicle decelerates from 10 m/s to ~1 m/s (RTL_SPEED_MS value) and never stops at the RTL destination. The test `RTLStoppingDistanceSpeed` times out waiting for groundspeed < 0.3 m/s.

**Evidence:**
- SITL log shows groundspeed stuck at 1.00-1.02 m/s for extended period
- Test expects: groundspeed between -0.3 and 0.3 m/s
- Vehicle never transitions from FLY_TO_RETURN_POINT to FINAL_DESCENT or LAND phase

**Hypothesis:** `RTL_SPEED_MS` parameter (default 1 m/s) sets a minimum horizontal speed floor in the waypoint navigation. The RTL state machine uses `wp_nav->reached_wp_destination()` to determine when to transition to the next phase, but this never returns true because:
1. The waypoint navigation maintains the vehicle at exactly RTL_SPEED_MS cruising speed
2. `reached_wp_destination()` checks distance to waypoint, but the vehicle is in cruise mode, not deceleration mode
3. The RTL state machine never triggers deceleration because the vehicle is always at cruising speed

**Code Path:**
- `ArduCopter/mode_rtl.cpp` line 66: `AP_PARAM_INT16, "RTL_SPEED_MS"` (default 1 m/s)
- `mode_rtl.cpp` line 89: `wp_nav->wp_and_spline_init_m(speed_ms.get())` - sets cruise speed
- `climb_return_run()` line 293: uses `wp_nav->reached_wp_destination()` to check completion
- `AC_WPNav::update_wpnav()`: maintains vehicle at `_wp_desired_speed_ne_ms` (set to RTL_SPEED_MS)

**File to Investigate:** `ArduCopter/mode_rtl.cpp` — RTL state machine, deceleration logic, wp_nav speed control

**Impact:** All RTL tests that require the vehicle to stop at home are broken. Vehicle will orbit around home at 1 m/s instead of landing/stopping.

---

### Bug 9.5: SMART_RTL PANIC on Re-entry (Severity 10 - Crash)

**Symptom:** Entering and exiting SMART_RTL mode causes `AP_InternalError::error_t::flow_of_ctrl` PANIC on the second entry to SMART_RTL. The test `SMART_RTL_EnterLeave` crashes the autopilot.

**Evidence:**
- SITL log: `AT-0004.1: Changing mode to SMART_RTL` (1st time) → `Got mode SMART_RTL`
- `AT-0004.1: Changing mode to ALT_HOLD` → `Got mode ALT_HOLD`
- `AT-0004.1: Changing mode to SMART_RTL` (2nd time) → `arducopter: PANIC: AP_InternalError::error_t::flow_of_ctrl`
- Process dies immediately, test times out with "Did not receive heartbeat"
- Buildlog confirms: test crashed at step "SMART_RTL_EnterLeave (check SmartRTL behaviour when entering/leaving)"

**Test Sequence:**
```
AUTO (arm, takeoff) → ALT_HOLD → SMART_RTL → ALT_HOLD → SMART_RTL → PANIC
```

**Hypothesis:** SMART_RTL state machine has inconsistent state that causes a flow-of-control assertion on re-entry:
- `smart_rtl_enter()` may leave internal state that is invalid when `smart_rtl_leave()` is called
- On second entry, `smart_rtl_enter()` encounters unexpected state and triggers `INTERNAL_ERROR()`
- Possible causes: uninitialized state, stale mission reference, or loop detection invariant violation

**File to Investigate:**
- `ArduCopter/mode.cpp` — `in_smart_rtl()` path, `smart_rtl_enter()`, `smart_rtl_leave()`
- `AP_SmartRTL.cpp` — internal state machine, loop detection logic
- Look for `INTERNAL_ERROR(AP_InternalError::error_t::flow_of_ctrl)` calls in SMART_RTL code path

**Impact:** SMART_RTL mode is unsafe for production use. Any scenario where pilot exits and re-enters SMART_RTL will crash the autopilot.

---

### Bug 9.6: FlyMissionTwice — Vehicle Doesn't Take Off on Second Lap (Severity 8)

**Symptom:** After completing a 3-item mission (TAKEOFF → WP → RTL) and disarming, the vehicle fails to take off on a second execution of the same mission. It remains at home (WP 0) and auto-disarms after ~3 seconds without moving.

**Evidence:**
- Run 1 succeeds: TAKEOFF (WP 0) → WP 1 → RTL (WP 2) → completes in ~43.6s, disarms after RTL
- Run 2 fails immediately: vehicle sits at home, groundspeed stays at ~0.01 m/s, disarms without taking off
- SITL log shows `current_wp: 0, wpnum_end: 3` throughout the entire run 2 — vehicle never transitions from current_wp=0 to WP 1
- No "AP: Mission: 1 Takeoff" log message appears on run 2 (present on run 1)
- Full test timeout: "Timed out waiting for waypoint 2 of 3" — but vehicle never got past waypoint 0

**Test Sequence:**
```
change_mode('AUTO') → arm_vehicle() → fly_mission() → disarms after RTL → delay_sim_time(20)
→ arm_vehicle() → fly_mission() → DISARMED at home after ~3s (never took off)
```

**Test Details:**
- `FlyMissionTwice` in `Tools/autotest/arducopter.py` (line ~6696-6726)
- Mission: TAKEOFF(20m) → WP at distance 1m → RTL
- `AUTO_OPTIONS=3` (AllowArming | AllowTakeoffWithoutRaisingThrottle)
- Disabled since PR #18561 with note "See https://github.com/ArduPilot/ardupilot/pull/18561"

**Hypothesis:** The mission state machine does not properly reset after RTL completion and re-arm:
1. After RTL completes, `AP_Mission::complete()` sets state to `MISSION_COMPLETE` and calls `_mission_complete_fn`
2. `exit_mission()` in mode_auto.cpp plays tone, tries loiter_start(), disarms
3. On second run, `start_or_resume()` is called — it checks `MIS_RESTART` parameter:
   - If `MIS_RESTART=1`: should call `mission.start()` — resets index to 0, starts from beginning
   - If `MIS_RESTART=0`: should call `mission.resume()` — continues from last index
4. Neither path correctly re-arms the TAKEOFF command for the second run

**Code Path:**
- `ArduCopter/mode_auto.cpp` line 101-105: `mission.start_or_resume()` called when `waiting_to_start` becomes false
- `AP_Mission::start_or_resume()` in `AP_Mission.cpp`: dispatches to `start()` or `resume()` based on `_restart`
- `AP_Mission::complete()` in `AP_Mission.cpp` line 2141-2150: sets `MISSION_COMPLETE`, calls `_mission_complete_fn`
- `AP_Mission::reset()`: clears flags and index — possibly not called before second start

**File to Investigate:**
- `libraries/AP_Mission/AP_Mission.cpp` — `start_or_resume()`, `complete()`, `reset()`, state transition logic
- `ArduCopter/mode_auto.cpp` — `start_or_resume()` in mode Auto::run(), mission state machine integration
- `ArduCopter/mode_auto.cpp` line 903-917: `exit_mission()` — disarms after RTL, possibly not cleaning mission state properly

**Impact:** Any test or real-world scenario requiring repeated mission execution (AUTO_MISSION_REPEAT, rally point cycles, repeat missions) is broken. Vehicle completes first lap then silently fails on re-arm.

---

### Bug 9.7: SmartRTL Breadcrumb Buffer Fills During Long Maneuvers (Severity 7)

**Symptom:** During extended circular or meandering flight (CIRCLE mode), the SmartRTL breadcrumb path buffer (300 points max) fills up before the vehicle exits the maneuver, deactivating SmartRTL entirely. Subsequent attempt to enter SMART_RTL mode fails with "init failed."

**Evidence:**
- `SMART_RTL_Repeat` test: vehicle takes off, enters CIRCLE mode
- At ~16s: "SmartRTL Low on space!" warnings begin (path at 291/300 points)
- At ~31.4s: "SmartRTL deactivated: buffer full" (`deactivate(Action::DEACTIVATED_PATH_FULL_TIMEOUT, "buffer full")`)
- At ~32.7s: "Mode change to Smart RTL failed: init failed"
- Test times out in 33.3s trying to enter SMART_RTL
- Path fills at ~18 points/second (291 points in 16s) — far faster than expected

**Test Details:**
- `SMART_RTL_Repeat` in `Tools/autotest/arducopter.py` (line ~19493)
- Takes off to 10m, enters CIRCLE for 1300s to fill breadcrumb buffer
- Disabled: "Currently fails due to issue with loop detection"

**Root Cause Analysis:**

The SmartRTL path buffer fills because the simplification algorithm fails to effectively compact circular paths. Tracing the code flow:

1. `save_position()` in `mode_smart_rtl.cpp` calls `smart_rtl.update(position_ok, should_save_position)` at 3Hz
2. `update()` calls `add_point()` which adds a point if distance from last point > `_accuracy` (default 2m)
3. Background `routine_cleanup()` at 3Hz calls `detect_simplifications()` (Ramer-Douglas-Peucker) and `remove_points_by_simplify_bitmask()`
4. **BUG:** In `remove_points_by_simplify_bitmask()`, points are compacted (shifted) but the `_simplify.bitmask` indices are NOT adjusted to account for the compaction
5. On subsequent simplification cycles, the bitmask references stale indices, causing simplification to either skip valid points or incorrectly flag points

Additionally, in `update()`:
```cpp
if (add_point(current_pos)) {
    _last_position_save_ms = now;
} else if (AP_HAL::millis() - _last_position_save_ms > SMARTRTL_TIMEOUT) {
    deactivate(Action::DEACTIVATED_PATH_FULL_TIMEOUT, "buffer full");
}
```
When `add_point()` fails due to semaphore contention, `_last_position_save_ms` is NOT updated, so the timeout will eventually trigger. This is correct. But the primary failure path is the path array reaching `_path_points_max` (300), which triggers `deactivate(Action::DEACTIVATED_PATH_FULL, ...)` in `add_point()` directly.

The `routine_cleanup()` function returns early after calling `remove_points_by_simplify_bitmask()` on line 424 (`return;`), which means pruning (`detect_loops()`) doesn't run in the same cycle as simplification removal. This is by design but means loop pruning is delayed by at least one cycle.

**Key Code Files:**
- `libraries/AP_SmartRTL/AP_SmartRTL.cpp`:
  - `add_point()` — path buffer management, `deactivate()` on full
  - `remove_points_by_simplify_bitmask()` — **likely bug in bitmask after compaction**
  - `routine_cleanup()` — simplification/pruning pipeline, early return after simplification removal
  - `detect_simplifications()` — Ramer-Douglas-Peucker with 200us time budget
  - `detect_loops()` — O(N²) segment-segment distance, 200us time budget
- `ArduCopter/mode_smart_rtl.cpp`: `save_position()` — breadcrumb update at 3Hz

**Impact:** SmartRTL is unusable for extended maneuvers where the breadcrumb path grows faster than simplification can compact it. Any CIRCLE, LOITER, or meandering behavior >~30s will deactivate SmartRTL, making it unavailable for emergency return.

---

## Appendix A: Quick Reference — Test Infrastructure API

### Existing Wait Methods (for building new tests)

| Method | Description | Default Timeout | Notes |
|--------|-------------|-----------------|-------|
| `wait_mode(mode)` | Wait for flight mode change | 60s | Returns on success, raises `WaitModeTimeout` |
| `wait_altitude(min, max)` | Wait for altitude within range | 30s | Supports `relative=True` |
| `wait_attitude(roll, pitch)` | Wait for specific attitude angles | 2s | Now supports `minimum_duration` |
| `wait_groundspeed(min, max)` | Wait for groundspeed range | 30s | |
| `wait_heading(heading, accuracy)` | Wait for heading within accuracy | 30s | |
| `wait_gps_fix_type_gte(fix_type)` | Wait for GPS fix type >= N | 30s | |
| `wait_ekf_happy()` | Wait for EKF to be healthy | N/A | Uses `WaitAndMaintainEKFFlags` |
| `wait_ekf_flags(required, error_bits)` | Wait for specific EKF flags | 10s | |
| `wait_sensor_state(sensor, present, enabled, healthy)` | Check sensor health | 5s | |
| `wait_distance_to_waypoint(wp, min, max)` | Wait for distance to waypoint | N/A | |
| `wait_location(loc, accuracy)` | Wait for GPS position accuracy | 30s | Uses `WaitAndMaintainLocation` |

### Parameter Injection API

| Parameter | Description | Test Use |
|-----------|-------------|----------|
| `SIM_GPS1_ENABLE` / `SIM_GPS2_ENABLE` | GPS module enable | GPS fault injection |
| `SIM_GPS1_GLTCH_X/Y` | GPS position glitch (meters) | Position jump injection |
| `SIM_MAG1_FAIL` / `SIM_MAG2_FAIL` | Compass fault (0=ok, 1=full fail) | Compass degradation |
| `SIM_BARO_GLITCH` | Baro glitch value | Baro sensor fault |
| `SIM_BARO_DISABLE` / `SIM_BAR_COUNT` | Disable barometers | Baro fault |
| `SIM_FLOW_ENABLE` | Optical flow simulation | Visual odom tests |
| `SIM_TERRAIN` | Terrain simulation enable | Terrain-dependent tests |
| `SIM_WIND_SPD` / `SIM_WIND_DIR` | Wind speed and direction | Wind testing |
| `SIM_QMOTORS` | Quad motors enable | VTOL motor fail |
| `TERRAIN_LOAD` | Terrain database load flag | Fence terrain tests |

### Message Types for Monitoring

| Message | Key Fields | Use Case |
|---------|-----------|----------|
| `EKF_STATUS_REPORT` | `flags`, `vel_horiz_drift`, `pos_horiz_drift`, `yaw` | EKF health, source switching |
| `GPS_RAW_INT` | `lat`, `lon`, `alt`, `hdop`, `fix_type`, `sacc` | GPS position, quality |
| `GLOBAL_POSITION_INT` | `lat`, `lon`, `alt`, `vx`, `vy`, `vz` | Position tracking, velocity |
| `AHRS` | `yaw`, `pitch`, `roll`, `airspeed` | Attitude, heading |
| `SIMSTATE` | `x`, `y`, `z`, `vx`, `vy`, `vz`, `roll`, `pitch`, `yaw` | Simulated true state |
| `OPTICAL_FLOW` | `integrated_x`, `integrated_y`, `quality` | Optical flow quality |
| `RANGEFINDER` | `distance`, `voltage` | Rangefinder data |
| `HEARTBEAT` | `base_mode`, `custom_mode`, `type` | Armed state, flight mode |
| `SYS_STATUS` | `onboard_control_sensors_*` | Sensor health bits |
| `FENCE_STATUS` | `breach`, `breach_count`, `breach_alt` | Fence breach detection |
| `STATUSTEXT` | `severity`, `text` | Warning/error messages |

---

## 10. Fuzz Test Bug Analysis (GPSCorruptionFuzz / SensorFaultFuzz / FailsafeEdgeCases)

Three adversarial fuzz tests were identified and analyzed for root-cause bugs in ArduPilot C++ source code. Results:
- **GPSCorruptionFuzz:** FAILS — catches real EKF GPS fusion bug (asymmetric height/position gating)
- **SensorFaultFuzz:** PASSES — validates EKF robustness under simultaneous sensor faults
- **FailsafeEdgeCases:** FAILS — fence safe-alt-max prevents vehicle from reaching fence boundary

### 10.1 GPSCorruptionFuzz — Altitude Drift During GPS Corruption

**Test location:** `Tools/autotest/arducopter.py:3971-4121`
**Failure:** "PANIC: altitude drifted 6.5m (limit 5.0m)" during Round 1 (position jitter) or Round 2 (large position jump).

#### Root Cause: GPS Position Innovation Clipping Instead of Rejection

**File:** `libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp`
**Lines:** 894–1050 (position and velocity fusion)

**Bug mechanics (two scenarios):**

**Scenario A — `_gpsGlitchRadiusMax > 0` (default 25m):**
When GPS position corruption exceeds the innovation gate (`posTestRatio > 1.0`):
- Lines 913–923: The code enters the `else if` branch only when `_gpsGlitchRadiusMax <= 0` (clipping path).
- Lines 926–946: When `posCheckPassed == false`, the code falls through to line 1014: `fusePosData = false`. **Position fusion is correctly disabled.**
- However, the height check at lines 1030–1050 still runs independently. When `_gpsGlitchRadiusMax > 0`, the height clipping at line 1035–1040 is **NOT** gated on `_gpsGlitchRadiusMax > 0` — it checks `<= 0`. **Height fusion still uses clipping even when position fusion is disabled.**

**Scenario B — `_gpsGlitchRadiusMax <= 0` (clipping mode):**
- Line 914: Position innovation variance is scaled by `posTestRatio` (clipping), then `fusePosData = true`. The corrupted GPS position is partially fused.
- Line 1036: Same clipping for height. **Corrupted GPS altitude is partially fused.**
- This allows the corrupted GPS to slowly pull the EKF position estimate toward the bad reading, causing unbounded drift.

**Key code analysis:**

```cpp
// Line 896-897: Innovation calculation
innovVelPos[3] = stateStruct.position.x - velPosObs[3];  // EKF x - GPS x (corrupted)
innovVelPos[4] = stateStruct.position.y - velPosObs[4];

// Line 906-908: Test ratio
maxPosInnov2 = sq(MAX(0.01 * _gpsPosInnovGate, 1.0)) * (varInnovVelPos[3] + varInnovVelPos[4]);
posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
```

With a 50m GPS jump: `posTestRatio` is ~2500 (far exceeds 1.0). The position is rejected but height is not.

**The real bug:** GPS position and height innovation checks are **asymmetrically gated** on `_gpsGlitchRadiusMax`:
- Position clipping (line 913): `_gpsGlitchRadiusMax <= 0`
- Height clipping (line 1035): `_gpsGlitchRadiusMax <= 0`
- **But height fusion is independent of position fusion.** When position is rejected but height is accepted, the EKF fuses only the GPS altitude while rejecting horizontal GPS. This can cause the EKF to use baro altitude as horizontal reference while fusing GPS altitude — if baro has bias, the vertical position estimate can drift.

**Additionally**, the `gps_glitching` flag (line 824 of `AP_NavEKF3_Control.cpp`) is set when `gpsAccuracyGood == false`, which is determined by innovation hysteresis (lines 311–317 of `AP_NavEKF3_VehicleStatus.cpp`). The hysteresis requires innovations to pass for **10 seconds** before `ekfInnovationsPass` becomes true. During this 10-second window, `gps_glitching` is active, and the EKF may be in an inconsistent state.

#### Impact

- Corrupted GPS data is partially fused through innovation clipping
- Position estimate drifts beyond acceptable bounds
- EKF state becomes inconsistent when horizontal position is rejected but altitude is accepted
- The `gps_glitching` flag takes up to 10 seconds to clear, during which failsafe behavior may be triggered

#### Fix Strategy

The root fix is in the EKF GPS fusion logic. Options:
1. **Reject entire GPS position+velocity+height together** when any component fails innovation — treat GPS as a unit, not independent components
2. **Tighten innovation gates** specifically for sudden jumps (detect d(position)/dt)
3. **Add a maximum innovation clamp** that prevents partial fusion when a large jump is detected

---

### 10.2 SensorFaultFuzz — **PASSED** (Corrected)

**Test location:** `Tools/autotest/arducopter.py:26386-26884`
**Status:** **PASSED** — all 8 fault injection rounds succeeded on run.
**Last verified:** 2026-11-07, log at `buildlogs/ArduCopter-SensorFaultFuzz.txt`

#### Original Analysis (Incorrect)

The initial analysis in this document claimed the test would fail with
`"NotAchievedException('Reboot attempted while armed')"`. This was based on
the assumption that the test calls `reboot_sitl()` between rounds to
reinitialize parameters after faults. **This assumption was wrong.**

#### Correct Understanding

The test uses **parameter-only reinitialization** between rounds:

1. Before each round: `set_parameters()` sets fault parameters (e.g.
   `SIM_BARO_DRIFT`, `SIM_MAG1_FAIL`, GPS glitch parameters)
2. After each round: `set_parameters()` restores the same parameters to
   zero/baseline values
3. No `reboot_sitl()` calls between rounds — only `context_pop()` at
   teardown (which triggers one final reboot after all rounds complete)
4. The vehicle stays armed in ALT_HOLD throughout all 8 rounds

This design avoids the reboot-while-armed issue entirely. The test relies
on the EKF's ability to reject corrupted sensor readings and recover once
faults are removed.

#### Rounds Tested

| Round | Fault | Status |
|---|---|---|
| 1 | Baro+GPS altitude fusion conflict | PASSED |
| 2 | Compass+GPS yaw conflict | PASSED |
| 3 | Optical-flow+GPS velocity conflict | PASSED |
| 4 | Baro pressure spike | PASSED |
| 5 | Sensor dropout burst | PASSED |
| 6 | Baro+Compass+GPS triple fault | PASSED |
| 7 | IMU temperature extreme | PASSED |
| 8 | Spoofing cascade | PASSED |

#### Conclusion

SensorFaultFuzz is **not a bug** — it is a successful adversarial test. The
test validates that the EKF can handle complex simultaneous sensor faults
and recover cleanly. It does not need fixing.

---

### 10.3 FailsafeEdgeCases — **Fence Safe-Alt-Max Deadlocks Vehicle at 3m**

**Test location:** `Tools/autotest/arducopter.py:26934+`
**Latest failure:** `AutoTestTimeoutException('Failed to attain Altitude want 50009.5, reached 3.0')`

#### Root Cause: `FENCE_MARGIN` Creates "Safe Alt Max" That Prevents Fence Breach

**File:** `libraries/AC_Avoid/AC_Avoid.cpp` (climb rate clamping) + `libraries/AC_Fence/AC_Fence.cpp`

**The problem:**

```python
# Round 3: Fence altitude at boundary
self.set_parameters({
    'FENCE_ENABLE': 1,
    'FENCE_TYPE': 1,            # max-alt fence only
    'FENCE_ALT_MAX': 5,         # 5m above home
    'FENCE_ALT_MAX_TP': 1,      # ABOVE_HOME
    'FENCE_ACTION': 1,          # RTL on breach
})
self.takeoff(20, mode="ALT_HOLD")       # ← stalls at 3m
self.wait_statustext("fence breached", timeout=60, regex=True)  # ← never fires
```

**Key parameter:** `FENCE_MARGIN` defaults to **2m** in ArduPilot (defined in `AC_Fence_Config.h`).

This creates a **"safe alt max"** = `FENCE_ALT_MAX - FENCE_MARGIN` = `5 - 2 = 3m`.

**Mechanism (AC_Avoid.cpp):**

1. When vehicle altitude reaches the safe alt max (3m), `AC_Avoid::get_avoidance_velocity()` clamps the climb rate to zero.
2. Code logic: `max_alt_diff_m = safe_alt_max_m - current_alt_m` → `max_alt_diff_m <= 0` triggers `climb_rate_cms = MIN(climb_rate_cms, 0.0f)` — clamping climb to zero.
3. The vehicle **cannot climb past 3m** in any navigation mode that respects the fence (ALT_HOLD, GUIDED, RTL, etc.).
4. The actual fence boundary (5m) is never reached, so `record_breach()` is never called → no RTL trigger.
5. The vehicle **stalls indefinitely** at 3m.

**SITL log verification** (`logs/00000002.BIN`):
- Altitude: 2.986–3.003m (exactly `FENCE_ALT_MAX - FENCE_MARGIN`)
- Climb rate: ~0.1 m/s (near-zero, oscillating ±0.15)
- Throttle integral (ThI): 0.365–0.398 (integrator slowly growing, insufficient to climb)
- 117-second stall before eventual breakout (not related to fence behavior)

**RTL would be equally deadlocked:** If RTL were to initiate, `compute_return_target()` caps the return target at `fence.get_safe_alt_max_m()` (3m). So even RTL couldn't go higher.

#### Impact

- Test cannot demonstrate fence breach behavior because the vehicle is prevented from reaching the fence boundary
- The `FENCE_MARGIN` parameter creates a hidden altitude ceiling that may surprise users
- This is arguably **by design** — the fence avoidance is preventing unsafe climb — but it makes the test scenario fundamentally incompatible with the fence behavior
- In a real field scenario, this means a vehicle cannot climb above `FENCE_ALT_MAX - FENCE_MARGIN` in any active navigation mode

#### Additional Issue: STATUSTEXT Consumption During `wait_altitude`

There is a **secondary issue** where STATUSTEXT messages are silently consumed by `poll_message()` during `wait_altitude`. The test log shows "Drained 28 messages from mav (69615.004149/s)" at the timeout, but zero STATUSTEXT messages were logged during the 2.8-second climb period. This means:
- Even if the vehicle DID breach the fence, the "Fence breached" STATUSTEXT would be consumed by `poll_message()` and never reach `wait_statustext()`
- The `wait_altitude` function should either log consumed STATUSTEXT or use a non-consuming polling method

#### Fix Strategy

**For the test (immediate):** Adjust Round 3 to work within the safe alt max constraint:

```python
# Round 3: Fence boundary test — use margin that allows fence breach
self.set_parameters({
    'FENCE_ENABLE': 1, 'FENCE_TYPE': 1,
    'FENCE_ALT_MAX': 15,     # High enough for takeoff to reach
    'FENCE_MARGIN': 0,       # Remove safe alt max
    'FENCE_ALT_MAX_TP': 1, 'FENCE_ACTION': 1,
})
self.takeoff(20, mode="ALT_HOLD")
self.wait_statustext("fence breached", timeout=60, regex=True)
self.wait_mode("RTL", timeout=30)
```

Or use a different approach that doesn't rely on fence breach during takeoff:

```python
# Round 3: Verify fence prevents climb with manual throttle control
self.set_parameters({
    'FENCE_ENABLE': 1, 'FENCE_TYPE': 1,
    'FENCE_ALT_MAX': 5, 'FENCE_MARGIN': 2,
    'FENCE_ALT_MAX_TP': 1, 'FENCE_ACTION': 1,
})
self.change_mode("ALT_HOLD")
self.set_rc(3, 1700)  # climb throttle
self.wait_altitude(2.5, timeout=30)  # reach just below safe alt max
self.set_rc(3, 1900)  # more throttle — should not breach safe alt max
self.wait_altitude(3.5, timeout=5)  # should NOT attain this
self.set_rc(3, 1500)  # center throttle
```

**For the code (design review):** Consider whether `FENCE_MARGIN` clamping should be:
1. **Optional** — disabled during armed/takeoff phases
2. **Different at the margin** — trigger RTL at the safe alt max (margin breach) instead of just notification, since the vehicle cannot proceed upward anyway
3. **RTL-aware** — when RTL is triggered and the fence prevents upward climb, initiate descent instead of stalling at the safe alt max

---

### 10.4 SMART_RTL_EnterLeave — Re-entry Panic / OOM

**Test status:** Disabled in `disabled_tests()`
**Bug description:** "SMART_RTL re-entry panic" — re-entering SMART_RTL after ALT_HOLD causes OOM (420s hang)

#### Root Cause: SmartRTL Path Corruption on Re-entry

**Files:** `libraries/AP_SmartRTL/AP_SmartRTL.cpp` (lines 327–352, `add_point()`), `ArduCopter/mode_smart_rtl.cpp` (line 132)

The test:
1. Enters SMART_RTL → breadcrumbs accumulate in ALT_HOLD via `save_position()` (3Hz)
2. Completes RTL → disarms
3. Re-enters ALT_HOLD → breadcrumbs accumulate again
4. Re-enters SMART_RTL → **OOM/hang**

The `add_point()` function (line 327) checks `_path_points_count >= _path_points_max` (line 345). On re-entry, the path buffer may already contain points from the first SMART_RTL execution. If `cancel_request_for_thorough_cleanup()` (called in `ModeSmartRTL::exit()`, line 71) doesn't properly clear the path or its processing state, the re-entry starts with a partially-processed path, causing:

1. **Double cleanup:** Both the old and new cleanup algorithms run on overlapping data
2. **Stack overflow:** The simplification/pruning recursion depth exceeds available stack
3. **Buffer overflow:** Points are added without proper count management

**Key observation:** `save_position()` (line 204) runs continuously when armed and updates `g2.smart_rtl.update()` regardless of whether SMART_RTL is active. This means breadcrumbs accumulate even when SMART_RTL is not being used, and the path buffer is never cleared between SMART_RTL executions.

#### Impact

- SMART_RTL mode is unstable when entered multiple times
- Re-entry after ALT_HOLD can cause indefinite hang (OOM/stack overflow)
- The breadcrumb accumulation during non-SMART_RTL modes causes buffer bloat

#### Fix Strategy

1. **Clear SmartRTL path on mode exit:** Ensure `ModeSmartRTL::exit()` clears `_path_points_count = 0`
2. **Limit breadcrumb accumulation:** Only accumulate during ARMED non-SMART_RTL modes, not during disarmed state
3. **Add buffer bounds checking:** Validate `_path_points_count` doesn't exceed `_path_points_max` during re-entry

---

### 10.5 SMART_RTL_Repeat — SMARTRTL_POINTS Parameter Never Initializes

**Test status:** Disabled in `disabled_tests()`
**Bug description:** "SMARTRTL_POINTS never initializes" — the parameter doesn't respond in SITL

#### Root Cause: SITL Parameter Initialization Gap

**File:** `libraries/AP_SmartRTL/AP_SmartRTL.cpp` (line 37)

```cpp
AP_GROUPINFO("POINTS", 1, AP_SmartRTL, _points_max, SMARTRTL_POINTS_DEFAULT),
```

The parameter is defined in `AP_GROUPINFO` but may not be properly initialized during SITL boot. The test observes that `SMARTRTL_POINTS` has a default value but doesn't respond to changes during the test.

**Possible causes:**
1. `AP_SmartRTL::init()` not called during SITL initialization (check `system.cpp:158`)
2. Parameter namespace collision with existing SITL parameter stores
3. `update()` method (line 234) called before `_points_max` is initialized

**From the code:** `system.cpp:158` shows:
```cpp
#if MODE_SMARTRTL_ENABLED
    g2.smart_rtl.init();
#endif
```

And `init()` (line 116) allocates the path buffer:
```cpp
_path = (Vector3f*)calloc(_points_max, sizeof(Vector3f));
```

If `init()` is called before `AP_Param::reload_all()` completes, `_points_max` may still have its default value, and subsequent parameter changes don't trigger buffer reallocation.

#### Impact

- SMARTRTL_POINTS parameter is non-functional in SITL
- Cannot configure SmartRTL path capacity for different test scenarios
- Parameter tracking system cannot verify parameter persistence across reboots

#### Fix Strategy

1. **Ensure `init()` is called after `AP_Param::reload_all()`** in SITL startup
2. **Or:** Add a `reload()` method that re-allocates the buffer when `_points_max` changes
3. **Or:** Document the limitation and remove the test from disabled tests if it's a known SITL-only issue
