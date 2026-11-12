# SkyBrush native module: implementation plan

Goal: reimplement the Skybrush drone-show fork (see [FEATURES.md](FEATURES.md)) as
one Rust native module on top of stock Copter plus small, generic library
changes.

Acceptance:
- **Test**: the Skybrush SITL tester suite passes: 13 entries, 36 flights.
- **Firmware**: a SITL Copter built from this branch, with the module
  embedded in ROMFS.
- **Results**: metrics comparable to the CMCopter-4.7 reference run, covering
  start delay, trajectory deviation and landing error.

## 1. Architecture

One applet, `libraries/AP_Module/applets/SkyBrush/`, built with
`Tools/modules/build.py` like `aerobatics`.
- It runs one worker thread with a 50 Hz tick, paced on `micros64`.
- Setpoint deadlines (`SHOW_CTRL_RATE`, up to 50 Hz) are checked on every
  tick against an accumulating deadline, not "elapsed ≥ period".
- Lights, events and fences run on the even ticks; the odd ticks repeat the
  LED output, as in the fork.

```
main.rs        tick loop, startup ordering, keepalive
params.rs      SHOW_ table (40 entries), change detection, volatile reset
show/          no_std port of the libskybrush subset (pure, host-tested)
  binary.rs trajectory.rs stats.rs builder.rs lights.rs controller.rs
  yaw.rs events.rs rth_plan.rs time_axis.rs screenplay.rs fmath.rs
timing.rs      GPS/countdown clock, week resolution, authorization
stage.rs       DRONE_SHOW stage machine (ModeDroneShow logic)
coords.rs      show frame <-> global <-> NED-from-origin
fences.rs      hard + bubble fence state machines and actions
geofence.rs    SIMPLE_GEOFENCE_SETUP -> FENCE_* params + polygon
mavlink.rs     DATA16-96 / LED_CONTROL / TUNNEL / COMMAND_INT, acks
telemetry.rs   DATA16/DATA96 status, stream-interval profile
lights_out.rs  colour state machine + LED backends
pyro.rs        safety gate, events, backends, test command
logging.rs     SHOW/FNCS/SBEV/SBSP/CRTH
```

`show/` has no SDK dependency. It is unit-tested on the host with
`cargo test`, against two oracles:
- **libskybrush itself**, built in a scratch directory: statistics, the
  builder and landing replacement, and the light player.
- **The tester's Python player**: trajectory positions.

### Startup ordering

1. Register the `SHOW_` table and its parameters. L9 applies the defaults
   files to them.
2. Reset the volatile `START_TIME/MSEC/AUTH`.
3. Register mode 127 and set its options (L1).
4. Initialise MAVLink receive and the command blocks, and create
   `/COLLMOT` (L7 mkdir).
5. Load the show.
6. Apply the telemetry profile.
7. Only then act on `SHOW_MODE_BOOT` bit0, by calling `set_mode(127)`.

### Flight mode

Mode 127 is a `ModeGuidedCustom` registered by the module with
`register_custom_mode(127, "DRONE_SHOW", "SHOW")`.
- The stage machine runs in the module thread.
- Controller-request lifecycle operations (init/run of takeoff, Land, RTL
  and Loiter) run on the main thread inside `ModeGuidedCustom::run()`. They
  are selected by an owner-checked controller request (L1).
- Guided target setters keep the existing path under the scheduler semaphore.

| Stage | Vehicle action |
|---|---|
| WaitForStartTime | Guided ground handling. At T-10 s, if disarmed: baro calibration via `gcs:run_command_int(PREFLIGHT_CALIBRATION, baro)`, then `ahrs:set_home(current)`. At T-lead: `arm()`. 1 s retry backoff. Re-arm after an auto-disarm, detected with `last_disarm_method` (L3). Disarm on revocation only if the module armed the vehicle. |
| Takeoff | Controller request TAKEOFF(alt above current). The fork's semantics live in core (L1). Hand-over to Performing uses the fork's rule: 70% of the takeoff height and trajectory ≥ current altitude, or WP reached + 10 s. |
| Performing | `set_target_posvelaccel_NED(pos, vel, acc, yaw, yaw_rate)` at `SHOW_CTRL_RATE`. With L1 this commands the active custom-mode instance. |
| Landing / RTL / Loiter | Controller request LAND / RTL (no disarm on land) / LOITER. The mode number stays 127. |
| Landed | Reached when the latched `complete` status for the current request generation is set. Then `disarm()`. |
| Error / TestingLights | `disarm()`. A disarm while airborne is handled after the completion check, so a successful landing is never reported as Error. |

Fence and EKF aborts are real mode changes (RTL/LAND with reason 10 or 6) or
forced disarms (method 15 or 24). After them the module reports stage Error
or Off as the fork does. Successful internal delegation never adds a MODE log
entry. Status telemetry keeps running after disarm.

### Timing

- **"Now"** is `gps:time_epoch_usec(0)`, clamped to be monotonic.
  - Epoch arithmetic stays in integers. The start epoch is subtracted before
    any conversion to float.
- **SHOW log**: ClockMS is sampled immediately before `logger.write`, which
  stamps TimeUS itself, so the module omits TimeUS.
  - The start-delay check is a consistency check between those two values. It
    is measured under concurrent accelerated SITL in P3.
  - If the jitter is too large, add a generic logger option that supplies a
    paired timestamp.
- **Setpoints** are evaluated for the tick on which they are sent. The guided
  pos/vel/accel controller integrates between updates, exactly as in the fork.

## 2. Library changes (one commit per subsystem)

Each change is generic and usable by Lua and native modules. Custom-mode
pieces stay under `AP_EXTERNAL_BINDING_ENABLED`. New state is zero-initialised,
so existing custom modes are unaffected.

### L1 Copter + AP_Vehicle: custom-mode controller ownership and requests

1. **Controller identity (bug fix).** `ModeGuidedCustom` is a separate
   `ModeGuided` instance. However, `Copter::start_takeoff()`,
   `set_target_*()` and `is_taking_off()` act on `copter.mode_guided`. Route
   them to the active guided-derived mode, through a helper that returns
   `flightmode` when it is a custom mode and `mode_guided` otherwise. This
   also fixes existing Lua custom modes. Test: custom mode doing
   takeoff → PVA.
2. **Controller request** in `custom_mode_state`. Fields:
   - `request` — enum: GUIDED, TAKEOFF, LAND, RTL, LOITER
   - `takeoff_alt_m` — relative to the current altitude
   - `generation` — u16, incremented on each request
   - status: `applied_generation`, `init_ok`, `complete` (latched until the
     next request), and `active`

   Semantics on the main thread, in `ModeGuidedCustom::run()`:
   - A new generation calls the delegate's `init(true)`, or for TAKEOFF a
     core `takeoff_start`. The takeoff matches the fork:
     - target = current altitude above origin + `takeoff_alt_m`
     - `D_init_controller`
     - `auto_takeoff.start_m`
     - reset yaw target/rate and the rate I-terms
     - yaw fixed at the heading captured at arming
     - `set_land_complete(false)` and `set_auto_armed(true)`
   - Each loop then calls the delegate's `run()`. RTL runs with
     `disarm_on_land=false`.
   - Completion conditions:
     - Land: `land_complete` and spool at GROUND_IDLE **or SHUT_DOWN**
     - RTL: state complete in FINAL_DESCENT/LAND at ground idle
     - TAKEOFF: `auto_takeoff` complete
   - Guided target setters are accepted only when the *current generation
     has been applied* as GUIDED. The module waits for that acknowledgement
     before it sends the first PVA target, so `ModeGuided::init()` can't
     discard it.
   - A failed delegate init publishes `init_ok=false`; the delegate is then
     not run.
   - `is_landing`, `is_taking_off`, the WP getters and
     `move_vehicle_on_ekf_reset` answer for the active controller, not for a
     stale inherited guided submode. RTL/Loiter references are guarded by
     their build options.
   - Status is captured inside custom `run()`. Completion is recorded only if
     the custom mode is still the active `flightmode` after the delegate's
     `run()`, since delegates can change mode themselves (e.g. RTL landing
     escape).
   - Exit: `set_mode()` inits the incoming mode before the outgoing `exit()`.
     Custom `exit()` therefore only invalidates its own request, status and
     activation. It never calls delegate exit or resets shared controllers or
     yaw. Generations keep counting across re-entry, so stale numbers never
     become valid again.
   - `Copter::is_taking_off()` already dispatches via `flightmode`; only the
     custom-mode override needs fixing.
3. **Options** in `custom_mode_state`:
   - `NO_EXTERNAL_GUIDED`: one acceptance predicate, checked before any
     target mutation or `hold_position()`, in `GCS_MAVLink_Copter.cpp`:

     | Path | Treatment |
     |---|---|
     | `SET_ATTITUDE_TARGET` | reject |
     | `SET_POSITION_TARGET_LOCAL_NED`, `SET_POSITION_TARGET_GLOBAL_INT` | reject |
     | `DO_REPOSITION` without CHANGE_MODE | reject, before the destination mutation |
     | legacy guided mission item (`MAV_CMD_NAV_WAYPOINT` in guided) | reject |
     | `DO_PAUSE_CONTINUE` | reject |
     | `NAV_TAKEOFF` | turned into a takeoff event (item 6) |

     Kept as in the fork:
     - `CONDITION_YAW`
     - `DO_REPOSITION` with CHANGE_MODE, which leaves the show for Guided
     - `DO_CHANGE_SPEED`, which is harmless in PVA

     `in_guided_mode()` stays true, so FS_OPTIONS bit2 radio-failsafe
     continuation still applies. No RC failsafe option is needed.
   - `EKF_FS_DISARM`: when the mode is active, `failsafe_ekf_event()` disarms
     with method EKFFAILSAFE, without checks, and sends "EKF Failsafe: forced
     disarm". The decision stays in the main-thread handler. The module sets
     this option iff `FS_EKF_ACTION==42`. *Deviation: the fork applies 42 in
     every mode.*
   - `ARMING_GATED` + `arming_ready` flag: when enabled, it *replaces*
     Guided's arming-method policy. MAVLink arming is always allowed (motor
     tests). Other methods, SCRIPTING included, need `arming_ready`. That
     reproduces the fork's `allows_arming`. The module publishes
     `arming_ready` = show loaded, plausible, and origin and orientation set.
     The module also sets `allow_entry` explicitly, because native
     registration starts it false.
4. **Owner supervision (opt-in).** The owner sets a timeout; 0, the
   default, disables supervision, so existing custom modes are unaffected.
   It calls `keepalive` each tick and refreshes it during long startup work
   before enabling supervision.
   - On expiry, or on owner cleanup (published to the main thread as a loss
     event), with the mode active, the main thread latches owner loss
     **once**:
     - it invalidates pending owner requests
     - it initiates LAND once, or disarms if landed
     - it inhibits arming and re-entry
     - it warns the GCS
   - A late keepalive does not resume the show. The latch clears only on
     a new registration by a live owner while disarmed.
   - Starting threshold: 1 s, measured under accelerated SITL.
   - Ordinary `GUID_TIMEOUT` semantics are unchanged.
5. **Takeoff event.** A GCS `NAV_TAKEOFF` in this mode doesn't start an
   ordinary Guided takeoff. It sets a mode-scoped "takeoff requested" event,
   ACKed when accepted. The module acknowledges the event, issues a TAKEOFF
   request with `SHOW_TAKEOFF_ALT` and then goes to Loiter, as the fork's
   test takeoff does. Ordinary `vehicle:start_takeoff()` keeps its
   altitude semantics.
6. **Bindings** in `BindingCustomMode`, all owner-checked:
   - `set_request(kind, alt)` → generation
   - `get_status()` → request, applied generation, init_ok, complete,
     takeoff event, owner-loss latch
   - `set_options(mask)`
   - `set_arming_ready(bool)`
   - `set_supervision(timeout_ms)`
   - `keepalive()`
   - `ack_takeoff_event()`

   Setters only store values under the scheduler semaphore.

### L2 Copter/AP_Vehicle: set mode with reason

Checked adapter `vehicle:set_mode_reason(mode, reason)` with an allowlist
{SCRIPTING, FENCE_BREACHED, EKF_FAILSAFE}, called under the scheduler
semaphore.

### L3 AP_Arming: disarm with method, last disarm method, locking

- `arming:disarm_method(method, do_checks)`, allowlist
  {SCRIPTING, FENCEBREACH, EKFFAILSAFE}.
- `arming:last_disarm_method()`.
- Existing `arm`/`disarm` adapters take the scheduler semaphore, since Copter
  arming mutates main-loop state.
- Not reproduced: the fork's fallback that sets the motors to disarmed
  directly. Disarming without checks only fails when already disarmed.

### L4 AC_Fence: polygon upload and breach count

- `fence:write_inclusion_polygon(points[≤21])`: a bounded adapter that calls
  `polyfence().write_fence()` under the fence/scheduler semaphore. It must
  keep the "Fence indexed OK" notification path.
- `fence:get_breach_count()`, for FNCS.GeoCnt.

### L5 AP_Notify: flags snapshot

`notify:get_flags()` packs named `AP_Notify::flags` fields into a defined
u32 wire bitmask. The fields:
- armed, flying, have_pos_abs
- gps_glitching, ekf_bad
- failsafe_battery/gcs/radio
- compass_cal_running, initialising
- pre_arm_check, pre_arm_gps_check

### L6 AP_Param: exact int32 dynamic parameters

- Typed registration `add_param_int32`, with a type-compatibility rule on
  re-registration.
- Exact `i32` get/set/set_and_save bindings, both by name and through the
  cached `Parameter`.
- Used for `SHOW_START_TIME`, `SHOW_ORIGIN_LAT/LNG/AMSL` and
  `SHOW_PYRO_TCOMP/ITIME`.
- Initially only exact by-name access. The cached `Parameter` variant can
  wait until P6.
- Parity only: the tester's values are exactly representable as float32.

### L7 AP_Module

- Generator/adapters for L1-L6, plus SDK tests and contract revisions.
- Add a filesystem `mkdir` adapter.
- Optional (parity, P6): `logger:write_parameter(name)`, for PARM records on
  USER_2/fence changes made while armed.

### L8 RC_Channel/Copter: aux 248

Add `AUX_FUNC::DRONE_SHOW_START = 248` as a recognised no-op that modules
poll, like the scripting aux functions. Only shared `RC_Channel` code
changes: the enum and the init and runtime no-op cases. Copter already
forwards unknown cases. Stock firmware currently raises a
fatal "Config Error: Failed to init: RC7_OPTION: 248" with the tester's
parameter file, and boot stops before modules load. This was verified in
SITL.

### L9 AP_Param: defaults files for dynamic parameters

`add_param()` installs only the coded default. Entries in the defaults files
for names that did not yet exist were skipped at boot. As a result, the
tester's `SHOW_TEL_PROFILE=1`, `SHOW_OPTIONS=2`, `SHOW_POST_ACTION=3`, etc.
would be lost.

Fix: a targeted lookup for the newly registered name.
- It scans the defaults file list, preserving file precedence (the last file
  wins). It is independent of `done_all_default_params`, which is assigned
  per file and is unreliable with comma-separated lists.
- The result is installed as the parameter's *effective default* before
  `load()`, so a failed storage load restores it. Saved values still win.
- No whole-file reload: that would replace the global `param_overrides` and
  could reset unrelated unsaved values.
- This fixes Lua parameters too.

### Existing bindings used unchanged

These were verified in `bindings.desc` and the native generator:
- vehicle and navigation: `gps:time_epoch_usec`, `set_target_posvelaccel_NED`,
  `start_takeoff`, `ahrs:set_home`, `is_taking_off`/`is_landing`,
  `get_likely_flying`
- arming: `arm`/`disarm`, aux auth
- GCS and MAVLink: MAVLink receive/send/`block_command`,
  `gcs:run_command_int` (group commands, baro calibration),
  `gcs:set_message_interval` (locking to be audited in L3/L7 alongside arming)
- outputs: SRV outputs, SerialLED, I2C, scripting serial, relay
- data: dynamic parameters, `logger.write`, file reads
- `fence:get_breaches`/`get_enabled_fences`

## 3. Feature mapping and deviations

| Fork feature | Module implementation |
|---|---|
| SHOW_ group at G2 idx 62 | Dynamic table with prefix `SHOW_`: same names, order and defaults. START_* are reset at module start. Saved values from the fork do not migrate. |
| Boot into show mode | At the end of module startup (later than INITIAL_MODE). |
| Show upload | Unchanged MAVFTP. The module reads `/COLLMOT/show.skyb` (`collmot/show.skyb` on SITL) and creates the directory. |
| Telemetry 101/102 | The module sends DATA16/DATA96 on each existing channel. Base streams use `set_message_interval`. |
| inject_command_int_packet | `gcs:run_command_int` |
| RTCM counters, ESC error rate, compass-cal latch, OreoLED, capability bit | Deferred; encoded as zero/never. |
| EKF action 42 | Only while in mode 127 (L1 option). |
| Cobra pyro on SERIAL protocol 14 (taken over from Volz) | SERIALx_PROTOCOL = Scripting (28); Volz stays usable. |
| Relay pyro `get_index_mask` | The module scans `RELAYn_FUNCTION`. |
| Pyro test (no interlock) | Adds a "disarmed" interlock. |
| Pyro auto-off | The module turns outputs off at ITIME. `set_output_pwm_chan_timeout` is backup only: the off baseline (SERVOn_MIN/trim) is set up explicitly and tested. Relay and serial backends rely on the module, with Cobra's own timeout. |
| Set home at T-10 | `ahrs:set_home`. Unlike the fork, the SmartRTL home is not updated (minor). |
| Initial armed bearing | Captured in core at arming for L1 TAKEOFF. |
| LogEvent 248, CM_FLOCK_OPTIONS, PPMSum hack | Out of scope. |

## 4. Phases

**P0 Harness (done).** `./waf configure --module=applets/SkyBrush` builds the module
into ROMFS with `MOD_ENABLE` defaulting to 1, and the tester runs the resulting SITL
binary unchanged.

**P1 libskybrush port**, in progress:
- First: container/CRC, trajectory and player, statistics, builder and
  landing replacement, light program, single-scene controller.
- Then: yaw, events, RTH plan, time axis, multi-scene screenplay.

**P2 Library changes L1-L9** with bindings and SDK tests. Autotests:
- custom-mode request sequencing: takeoff → PVA → LAND/RTL/LOITER
- keepalive loss
- EKF disarm option
- arming gate
- dynamic parameter defaults
- aux 248 boot

**P3 Module core**: params, loader, timing, START_CONFIG, the stage machine,
DATA96 status and the SHOW/FNCS logs. Gate: `fly circle-7m-radius.skyc` passes
all checks, and the ClockMS jitter measured with `-j 5` is well under 50 ms.

**P4 Safety**: the geofence setup packet (ACK, then "Fence indexed OK"), the
hard fence, the bubble fence (exact texts, 1 s action and 5 s text cadence),
and EKF-42. Gate: all 11 scenarios pass.

**P5 Full suite** with `-j 5`, compared with the reference metrics.

**P6 Parity**, not covered by the tester:
- LED backends and colour logic, LED_CONTROL, pixel grid
- pyro
- USER_1/USER_2
- TIME_AXIS_CONFIG and collective RTH
- the standard telemetry profile
- the RC start switch
- the SBEV/SBSP/CRTH logs

Add targeted SITL checks: servo LED outputs, a crafted TIME_AXIS_CONFIG for
collective RTH, and USER_2 precision.

### Tester acceptance gates

- Completed runs have stage sequence `[2,3,4,5,8]` or `[2,3,4,7,8]`, and at
  least 10 LANDED status packets after disarm.
- Stage 3 is visible in DATA96. Takeoff lasts well over 1 s, so at 1 Hz
  status this holds, but it is checked.
- Only DATA96 0x5B is sent; never DATA16 0x5B in the compact profile.
- Fence and EKF aborts: the first MODE change after the start is 6 or 9, with
  reason 10 or 6. Forced disarms use method 15 or 24 and are followed by a
  SHOW row with stage Error.
- The bubble and hard fence messages match the exact strings and timings
  (±0.3 s). No other text may contain "breach".
- Takeoff drift ≤ 0.1 m. Landing within 0.4 m of the actual takeoff point.

## 5. Risks

- **Controller ownership/delegation** is the riskiest core change. The fork
  does the same thing (init/run of sub-modes inside the show mode), so the
  behaviour is known; it needs SITL tests for every transition.
- **Thread timing**: worker jitter affects setpoints and the log clock. It is
  measured in SITL, and later on H7 with `MOD_THD_PRIO_MAX`.
- **Heap**: the whole show file stays resident. Set `--heap-size` from
  measured peaks.
- **Dynamic parameter budget**: the 40 parameters share Lua's table limits
  (63 per table, 10 tables by default).
- **GPL**: `show/` is a derived work of libskybrush (GPL-3.0-or-later,
  CollMot Robotics); keep the attribution.
