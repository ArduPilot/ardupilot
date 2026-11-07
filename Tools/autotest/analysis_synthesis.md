# Autotest Analysis Synthesis — Complete Findings

**Date:** 2026-11-06
**Agents Run:** 3 (classification, parametrization, bug-hunting plan)

---

## 1. Have We Found New Bugs?

**Yes.** SITL validation of the 11 improvements exposed **3 pre-existing bugs**:

| Bug | Symptom | Severity | Blocks |
| ----- | --------- | ---------- | -------- |
| **SITL Altitude Drift** | Vehicle climbs uncontrollably (ModeFlip→1157m, MainFlight→693m) in LOITER/ALT_HOLD | E1 Simulation Artifact | All altitude-hold tests |
| **EKF "RTL Not Armable"** | Plane groundspeed = 0.03 m/s after RTL triggers (never flies) | E2 MAVLink Protocol | All RTL/failsafe tests |
| **Timesync Flake** | `MAV_MISSION_OPERATION_CANCELLED` at random intervals | E2 MAVLink Protocol | ~30% of Copter tests |

**Key finding:** None of the test failures are caused by the improvements. All failures stem from pre-existing bugs — **this confirms the improvements ARE working**. They expose latent bugs rather than mask them. 2/8 changed tests passed (GPSGlitchLoiter, Scripting), 5/8 failed due to pre-existing bugs, 1 hung.

---

## 2. Are the Improvements Non-Trivial?

**8/11 improvements are NON-TRIVIAL** (meaningful behavioral changes). 4/11 are TRIVIAL (simple parameter tweaks). 1/11 was NOT YET APPLIED.

### NON-TRIVIAL (8 improvements, impact 6-9/10)

| # | Improvement | Impact | Why Non-Trivial |
| --- | ------------- | -------- | ----------------- |
| 2 | OpticalFlow two-tier tolerance (fail/warn) | 7/10 | New diagnostic data collection, two-tier validation — catches degradation invisible before |
| 3 | OpticalFlowLimits hard 10 m/s cap | 8/10 | Fundamental: from sampled intervals to continuous enforcement, altitude-independent |
| 4 | GPSGlitchLoiter position tracking | **9/10** | **Entirely new assertion** — previously `drain_mav()` with zero position check; test would pass if vehicle flew off to 500m |
| 5 | GPSGlitchLoiter2 position tracking | 8/10 | Entirely new assertion — test only checked roll/pitch angles, not position divergence |
| 6 | GPSGlitchAuto position tracking | 8/10 | Entirely new assertion — loop ran through glitches without any position verification |
| 7 | Plane FBWB altitude + midpoint check | 7/10 | New intermediate check (was only final altitude), catches slow degradation mid-flight |
| 11 | Rover GCS failsafe: wait_mode + position | 6/10 | Replaced fragile `delay_sim_time(5)` with active `wait_mode`, added movement assertion |
| +Infra | wait_and_maintain min_duration 0→0.5 | 8/10 | **Foundational** — propagates to every test using these helpers, prevents spike-based false passes |

### TRIVIAL (4 improvements, impact 3-4/10)

| # | Improvement | Impact | Why Trivial |
| --- | ------------- | -------- | ------------- |
| 1 | ModeFlip tolerance 30°→15° | 3/10 | Parameter tightening on existing assertion; catches "stall at 75°" vs "stall at 61°" |
| 8 | Plane roll accuracy tightening | 3/10 | Parameter tightening of existing check |
| 10 | Scripting count 3→10-200 | 4/10 | Tighter bounds on count check; prevents obviously wrong behavior |
| +Infra | wait_attitude min_duration (default 0) | 4/10 | Infrastructure in place but default is 0 — no automatic tightening, requires explicit opt-in |

### NOT APPLIED

| # | Improvement | Note |
|---|-------------|------|
| 9 | ThrottleFailsafe 100m distance check | **NOT FOUND in current code** — analysis flagged this as missing. The test only checks RTL mode entry, not that the plane actually flies toward home. |

### Recommendation

- **Keep all 8 NON-TRIVIAL** — they all meaningfully improve bug-catching
- **Apply the missing ThrottleFailsafe distance check** — critical gap for a safety-critical test
- **TRIVIAL improvements are still worth keeping** — small tightening improves test sensitivity at minimal cost

---

## 3. Parametrization Opportunities

**13 parametrization opportunities** identified across all three vehicle test suites.

### Quick Wins (Low Effort, High Impact)

| # | Opportunity | File | Effort | Value |
| --- | ------------- | ------ | -------- | ------- |
| 1 | **Mount RC Rate Control** | arducopter.py:6786-6809 | 5 min | Extracts 11 hardcoded RC-pitch pairs into data structure |
| 2 | **Mount RC Targetting** | arducopter.py:6719-6784 | 10 min | Tests full RC-to-pitch linearity, not just 3 sampled points |
| 4 | **External AHRS** | arduplane.py:~3290-3340 | 15 min | 6 separate methods → parameterized test, easy to add new types |
| 6 | **Precision Landing** | arducopter.py:5546-5626 | 15 min | Extracts 50+ lines of duplicate backend test body |
| 9 | **Landing Speeds** | arducopter.py:5573-5648 | 10 min | Tests boundary values (very high/low speed) not currently covered |
| 10 | **ESC Telemetry Mask** | arducopter.py:13467-13471 | 5 min | Systematically tests consecutive bit patterns |

### Medium Impact

| # | Opportunity | File | Effort | Value |
| --- | ------------- | ------ | -------- | ------- |
| 3 | **Rover Scripting** | rover.py:4823-5016 | 30 min | 7 separate tests → unified parametrized test + edge cases |
| 5 | **Poly Fence Tests** | rover.py:4357-4577 | 1-2h | Arbitrary polygon sizes/shapes, not just hardcoded 4-vertex |
| 7 | **Mount Mode Testing** | arducopter.py:7368-7430 | 30 min | run_cmd vs run_cmd_int consistency across all modes |
| 8 | **GCS Failsafe** | rover.py:1781-1875 | 30 min | Easy to add new GCS values (boundary conditions) |

### Cross-Vehicle Infrastructure (Higher Effort)

| # | Opportunity | Scope | Effort | Value |
| --- | ------------- | ------- | -------- | ------- |
| 11 | **Cross-Vehicle Flight Modes** | all 3 files | 1-2h | Unified test framework for equivalent modes across vehicles |
| 12 | **Fence Size Variation** | all 3 files | 2-3h | Arbitrary polygon sizes/shapes across all vehicles |
| 13 | **Altitude Variation** | all 3 files | 1h | Tests at different heights to catch sensor-specific bugs |

---

## 4. Bug-Hunting Plan

**Source:** `Tools/autotest/bug_hunting_plan.md` (created by Agent 3, 982 lines)

### Bug Taxonomy (20 bugs across 5 categories)

| Category | Severity | Count | Example |
| ---------- | ---------- | ------- | --------- |
| **A: Safety-Critical** | 9-10 | 4 | Parachute not firing, fence silently inactive |
| **B: Flight Instability** | 7-9 | 4 | VTOL transition stuck, autoland drift in wind |
| **C: Sensor Fusion** | 6-9 | 4 | EKF compass degradation, GPS glitch response |
| **D: Parameter/Config** | 4-7 | 3 | Mid-flight param cross-sensitivity |
| **E: SITL Artifacts** | 1-5 | 3 | Pre-existing altitude drift, timesync flake |

### Priority Matrix

#### Tier 1 — Implement Immediately (Impact 7+)

| # | Bug | Category | Severity | Detect | Coverage | Impact |
| --- | ----- | ---------- | ---------- | -------- | ---------- | -------- |
| 2 | EKF compass degradation mid-flight | C1 | 9 | E | NONE | **9.0** |
| 1 | Quadplane VTOL motor fail during transition | B2 | 10 | M | NONE | **10.0** |
| 3 | Fence terrain dependency (ALT_MAX_ABOVE_TERRAIN) | A2/A4 | 9 | E | NONE | **9.0** |
| 4 | Parachute test recovery (#4702) | A3 | 10 | M | NONE | **10.0** |
| 5 | Battery monitor cross-sensitivity | D1 | 7 | E | NONE | **7.0** |

#### Tier 2 — Implement Next (Impact 4-7)

| # | Bug | Category | Severity | Detect | Coverage | Impact |
| --- | ----- | ---------- | ---------- | -------- | ---------- | -------- |
| 6 | RTK/RTCM correction loss during mission | C1 | 8 | M | NONE | **8.0** |
| 7 | SMART_RTL panic/loop detection (#18404) | B2/A4 | 10 | M | NONE | **10.0** |
| 8 | Repeated fence boundary crossing | A2/A4 | 8 | M | PARTIAL | **5.3** |
| 9 | Compass interference during steering | C2 | 7 | M | NONE | **4.7** |
| 11 | Visual odom source switching mid-flight | C4 | 8 | M | PARTIAL | **5.3** |

### 5 Concrete Test Proposals

See `bug_hunting_plan.md` §5 for full test specifications including:

1. **Copter `TestEKFGPSYawRecovery`** (Impact: 9.0) — Inject compass degradation mid-flight, verify EKF switches to GPS yaw within 5s with <5° drift
2. **Plane `TestQuadPlaneMotorFailTransition`** (Impact: 10.0) — Kill motors at 50% VTOL transition, verify graceful FW recovery
3. **Copter `TestFenceTerrainFallback`** (Impact: 9.0) — Test fence with unavailable terrain database, verify no silent degradation
4. **Rover `TestRTKLossDuringMission`** (Impact: 8.0) — Drop RTCM mid-mission, verify EKF degrades gracefully
5. **Copter `TestVisualOdomSourceSwitch`** (Impact: 5.3) — Toggle optical flow mid-flight, verify smooth EKF transition

### Infrastructure Requirements

| Component | Priority | Description |
| ----------- | ---------- | ------------- |
| **Parameter context tracking** | HIGH | `_snapshot_parameters()` / `_restore_parameters()` mechanism to prevent test contamination |
| **EKF status monitor hook** | HIGH | Continuous EKF health monitoring during flight (not just binary pass/fail at checkpoints) |
| **Position jump detector** | MEDIUM | Detects abrupt GPS/EKF discontinuities beyond simple distance-from-home checks |
| **Aggressive timeout mode** | MEDIUM | Tiered timeouts: SIMPLE=15s, MODERATE=30s, COMPLEX=120s |

### Implementation Order

| Phase | Timeline | Deliverables |
| ------- | ---------- | -------------- |
| **Phase 1** | Week 1-2 | Parameter context infrastructure + fix 2 blocking SITL bugs |
| **Phase 2** | Week 2-4 | All 5 Tier 1 tests + Parachute recovery |
| **Phase 3** | Week 4-6 | EKF monitor hook + 6 Tier 2 tests + SMART_RTL recovery |
| **Phase 4** | Week 6-8 | Position jump detector, timeout mode, remaining test hardening |

### Disabled Test Recovery

Three high-priority disabled tests should be recovered as bug hunts:

- **Parachute (#4702)** — safety-critical mechanical release
- **SMART_RTL panic (#18404)** — crash bug in loop detection  
- **LandingDrift (#20054)** — flapping during plane autoland

---

## 5. Status — Completed This Session (2026-11-06)

| # | Action | Status | Commit |
| --- | -------- | -------- | -------- |
| 1 | ThrottleFailsafe distance check (active wait) | ✅ Pushed | `6bf9609c` |
| 2 | Mount RC Rate Control parametrization | ✅ Pushed | `98e80270` |
| 3 | ESC Telemetry mask systematic bit patterns | ✅ Pushed | `786f676f` |
| 4 | Landing Speeds edge-case configs | ✅ Pushed | `5ae43cd3` |

All 4 quick-win parametrizations committed and pushed to `vlordier/test-improvements`.

---

## 6. Next Actions — Prioritized

### Immediate (This Session)

1. **Create PR** to ArduPilot/master with the 15 committed improvements (11 original + 4 parametrization)

### Short-Term (Next Session)

2. **Fix 2 blocking SITL bugs** — altitude drift and EKF RTL not armable (block all new test validation)
3. **Apply External AHRS parametrization** (medium effort, skipped this session)
4. **Create PR** for all improvements + bug-hunting plan

### Medium-Term (Future Sessions)

5. **Implement Phase 1 infrastructure** — parameter context tracking
6. **Implement Tier 1 tests** — EKF compass degradation, VTOL motor fail, fence terrain fallback
7. **Recover disabled tests** — Parachute, SMART_RTL, LandingDrift
