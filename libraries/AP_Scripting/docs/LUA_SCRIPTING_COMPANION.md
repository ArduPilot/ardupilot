# The ArduPilot Lua Scripting Companion

This document compiles verified knowledge about ArduPilot's onboard Lua 5.3 scripting system. It is intended as a practical reference for developers and AI assistants working in the repository.

**Last Refreshed:** 2026-06-16 (tree refresh; re-explored `libraries/AP_Scripting/` and cross-references across the repo)  
**Scope:** Onboard Lua scripting (`libraries/AP_Scripting/`), its bindings, applets, runtime behavior, and integration points throughout the ArduPilot codebase.

**Compliance Note:** All information is verified against current source files, commit history, and in-tree documentation. The Companion follows the spirit and accuracy standards of the root `AGENTS.md` file: factual, no fabrication, and useful for real work.

## Table of Contents

- [1. High-Level Overview](#1-high-level-overview)
- [2. Architecture & Runtime](#2-architecture--runtime)
- [3. Directory Structure & Script Categories](#3-directory-structure--script-categories)
- [4. Cross-Repository Integration and Testing](#4-cross-repository-integration-and-testing)
- [5. Bindings & API Surface](#5-bindings--api-surface)
- [6. Common Implementation Patterns](#6-common-implementation-patterns)
- [7. Setup & Deployment](#7-setup--deployment)
- [8. Debugging & Limitations](#8-debugging--limitations)
- [9. Notable Applets & Scripts](#9-notable-applets--scripts)
- [10. Community & Knowledge Sources](#10-community--knowledge-sources)
- [11. Extending the System](#11-extending-the-system)
- [12. Maintenance Process](#12-maintenance-process)
- [13. Current Knowledge Gaps](#13-current-knowledge-gaps)

---

## 1. High-Level Overview

ArduPilot provides a sandboxed Lua 5.3.6 environment (customized, vendored in `lua/`) for extending vehicle behavior without modifying core C++ flight code. Scripts run at low priority, are instruction-budgeted for safety, and live primarily on the SD card.

**Key Characteristics**
- Cooperative callback model (not threads or continuous loops).
- Each script runs in its own isolated sandbox (own globals, persistent state between callbacks).
- Protected by VM instruction limits and CPU-time hooks.
- Multiple scripts can run concurrently.
- Primary use cases: custom automation, hardware drivers for unsupported devices, enhanced pre-arm checks, tuning aids, payload logic, sensor fusion, and vehicle-specific glue code.

Scripts are **not** intended for hard real-time control loops.

---

## 2. Architecture & Runtime

**Core Components (C++)**
- `AP_Scripting.cpp/h` — Parameter definitions (`SCR_ENABLE`, `SCR_HEAP_SIZE`, `SCR_VM_I_COUNT`, `SCR_DEBUG_OPTS`, `SCR_USER1`–`6`, `SCR_DIR_DISABLE`, checksums).
- `lua_scripts.cpp/h` — Main runtime: heap management, script loading (ROMFS + SD), per-script Lua state, instruction hook (`hook()` that sets `overtime` and calls `luaL_error`), error handling, GCS printing, CRC checksums for pre-arm.
- Binding layer (`lua_bindings.cpp`, generated from `bindings.desc`).
- Heap is configurable and can auto-expand in newer firmware (with warnings).

**Execution Contract (the fundamental pattern)**
```lua
function update()
    -- work here
    return update, 1000   -- reschedule self in N milliseconds
end
return update, 1000
```
- Top-level code runs once on load.
- Returning `nil` or nothing stops the script.
- `micros()` / `millis()` are available for timing.

**Safety Mechanisms**
- Per-script VM instruction quota (`SCR_VM_I_COUNT`).
- Overtime hook aborts with “Exceeded CPU time”.
- Sandboxing prevents one script from corrupting another’s globals.
- Low scheduling priority.
- Callback reschedule delays are capped (recent change: never more than 65535 ms).

---

## 3. Directory Structure & Script Categories

| Location              | Purpose                                      | Characteristics                              | Documentation |
|-----------------------|----------------------------------------------|----------------------------------------------|-------------|
| `applets/`            | Ready-to-use features                        | Copy to SD, no editing required. Each has a matching `.md` (wiki entries are derived from these) | `applets/README.md` + individual `.md` |
| `drivers/`            | Hardware protocol drivers (EFI, mounts, generators, sensors, modems, etc.) | Talk to serial/CAN devices and feed ArduPilot backends (`efi:get_backend`, mount scripting drivers) | `.md` per driver |
| `examples/`           | Learning & demonstration                     | Illustrative, often multi-stage or focused on one binding or new feature (ADSB, timing, motors, etc.) | `examples/README.md` |
| `modules/`            | Reusable code for `require()`                | Return a table; closure-based “classes” common (e.g. `pid.lua`, `crsf_helper.lua`) | — |
| `tests/`              | Internal validation                          | luacheck, docs checks, binding tests, scheduler accuracy (`callback_time_test.lua`) | — |
| `generator/description/bindings.desc` | Binding definition DSL                  | Source of truth for what Lua can call. Generated outputs appear under `generator/gen/` after build | Wiki “How to Add New Bindings” |
| `docs/docs.lua`       | EmmyLua annotations + doc source             | Powers IDE support and generated API docs    | — |

**Loading Order / Locations**
- ROMFS (manufacturer-embedded scripts)
- `APM/scripts/` on SD card (or `scripts/` in SITL)
- Controlled by `SCR_DIR_DISABLE`

**Note on Growth:** The `applets/` collection is actively expanded (new safety, engine control, and satellite scripts added in recent refreshes).

## 4. Cross-Repository Integration and Testing

Scripting is referenced and integrated far beyond `libraries/AP_Scripting/`. It is a first-class optional feature across the tree (controlled by the `AP_SCRIPTING_ENABLED` compile-time define and `SCR_ENABLE` runtime parameter). It appears in vehicle logic, other libraries, build configuration, SITL testing, and hardware bring-up docs.

### Build System, Configuration & Feature Flags
- Root `wscript` exposes command-line options: `--disable-scripting`, `--enable-scripting`, `--scripting-checks` (runtime sanity), and `--scripting-docs` (regenerates `docs/docs.lua` + bindings glue).
- `Tools/scripts/build_options.py` lists the base `SCRIPTING` feature (for CustomBuild server) and dependents such as `AP_CRSF_SCRIPTING_ENABLED` ("CRSF Menu Scripting" – requires SCRIPTING + CRSF + OSD_PARAM + FrSky passthrough).
- `AP_SCRIPTING_ENABLED` is guarded in:
  - Vehicle `Parameters.cpp` (ArduCopter, ArduPlane, Blimp, Tools/AP_Periph) – e.g., `{ &scripting, scripting.var_info, N }` for param embedding.
  - Periph: `AP_Periph.cpp` calls `scripting.init()`; supports reboot-to-bootloader scripting notes.
- Related compile/runtime features exposed to scripts:
  - RC aux functions 300–307 ("Scripting1" through "Scripting8").
  - Mount backends (`MNTx_TYPE=9` "Scripting"), Camera scripting (`CAMx_TYPE=7`).
  - Servo outputs for scripting LEDs (SERVOx_FUNCTION in the scripting range, 94–109 range historically).
  - `SCR_*` params (HEAP_SIZE, VM_I_COUNT, DEBUG_OPTS, USER1–6, DIR_DISABLE, checksums) are declared in AP_Scripting but referenced/required everywhere scripts are enabled.

### Autotest / SITL Integration Testing (`Tools/autotest/`)
Scripting is one of the most heavily exercised optional features in the SITL test suite (primary way the project verifies Lua works across vehicles and bindings).

- Shared infrastructure (`vehicle_test_suite.py`): `scripting_restart()` helper, MAVLink FTP script upload + verification, basic Lua MAVLink generation for tests.
- Test scripts exercised: `scripting_test.lua`, `scripting_require_test_2.lua`, `simple_loop.lua`, `hello_world.lua`, plus many in-tree examples (copter-circle-speed, set-target-velocity, ahrs-source, get-target-location-mount-backend, callback_time_test, etc.).
- Representative test methods (vehicle-specific, all typically do `self.set_parameter("SCR_ENABLE", 1)` and often boost `SCR_VM_I_COUNT` or set `SCR_DEBUG_OPTS`):
  - ArduCopter: `LuaCopterCircleSpeed`, mount scripting backend (all modes + camera), 6DoF motor matrix, velocity vector control, custom modes via scripting, ahrs-source-gps-optflow/wheelencoders, param lockdown (LuaParamLockdown), FTP script load, flip mode, etc.
  - ArduPlane: multiple nav_scripting / AUTO scripting tests, quadplane scripting blocks, ship landing / payload place abort paths.
  - Rover: `test_scripting_simple_loop`, `test_scripting_internal_test`, auxfunc, hello_world, set-home, serial loopback, callback_time, print-home-origin.
  - Others (QuadPlane, Sub): repeated `self.scripting_restart()`, `SCR_ENABLE=1` blocks for terrain avoidance, follow, etc.
- Schema: `pysim/vehicleinfo_schema.md` documents `frame_example_script` (points to a Lua file in `examples/` for SITL frame-specific motor mixing or behavior).
- `test_build_options.py` whitelists scripting-related defines (e.g., `AP_SCRIPTING_BINDING_MOTORS_ENABLED`).

These tests are the best "living documentation" of supported bindings and runtime behavior.

### Vehicle-Specific Logic (Navigation, Mission, Modes, Safety)
- **ArduPlane** (strongest nav integration):
  - `Plane.h` and `commands_logic.cpp`: `nav_scripting` struct (enabled, id, timeouts, rates for roll/pitch/yaw/throttle, rudder_offset_pct, run_yaw_rate_controller).
  - Supports `NAV_SCRIPT_TIME` mission items (script takes control of nav for a duration; see `nav_script_time` handling, verify, and timeout logic).
  - `nav_scripting_active()`, `nav_scripting_enable(uint8_t mode)`.
  - Used in AUTO, LOITER, and acro/aerobatic contexts.
  - Rudder offset override and yaw rate controller bypass specifically "for use with scripting controllers" and aerobatics.
  - QuadPlane: abort landing hooks "used by scripting for payload place and ship landing abort".
  - `mode_auto.cpp`, `mode_loiter.cpp`, `commands_logic.cpp` all have `#if AP_SCRIPTING_ENABLED` blocks that short-circuit normal nav when a script is active.
- ArduCopter: scripting custom modes, motor matrix allocation (6DoF, fault tolerant, etc.), velocity/position targets, source switching (EKF/ahrs).
- All vehicles: RC_Channel aux function handling for 300-307, param tables embedding `scripting.var_info`, GCS interaction points.
- Safety impact: scripts participate in arming (aux auth), pre-arm (e.g. arming-checks applet), param lockdown (intercepts PARAM_SET), fence/RTL checks, throttle kill (fixed-wing turbines), etc.

### Other Libraries & Backends
- **AP_BattMonitor**: `handle_scripting(uint8_t idx, const BattMonitorScript_State &state)` and `AP_BattMonitor_Scripting.cpp`. Lua can supply full battery state (voltage, current, consumed, cell voltages, temperature, health, etc.) for custom monitors.
- **AP_Notify**:
  - `ScriptingLED.cpp/h` (and SITL variants): dedicated device for Lua-driven LEDs.
  - `RGBLed.cpp`: `give_rgb()` and flash rate hooks "used with scripting".
  - Ties directly to `serialLED:` singleton and SERVO_FUNCTION scripting outputs.
- **AP_ESC_Telem**: Comments note data "can also be called from scripting".
- **AP_Networking**: Utility functions (e.g. address stringification) annotated "for scripting".
- **AP_Filesystem**: Notes on cache sizing for "(scripting) and very large files".
- **AP_Mount / AP_Camera**: Scripting backends (type 9 for mount, 7 for camera) allow full Lua control of gimbals/cameras (POI, mode, settings, ONVIF, ViewPro, DJI RS2 drivers in the `drivers/` folder).
- **GCS_MAVLink / MAVLink handling**: Scripts can send/receive via low-level mavlink (see adsb_send.lua example and bindings), block commands, set message intervals, takeover PARAM_SET (param-lockdown pattern). Internal `lua_bindings.cpp` shows mavlink command block lists and rx buffers.
- **Periph / Cross-vehicle**: AP_Periph supports scripting (for CAN peripherals, etc.). Blimp, Plane, Copter all embed the scripting singleton in their Parameters.

### Hardware, Bring-up & Documentation
- Board hwdef READMEs (e.g. F4BY_H743) explicitly list "micro SD Card for logging, Lua scripts etc."
- Many vehicle ReleaseNotes.txt entries document Lua evolution (custom modes, rally points binding, CAMERA_INFORMATION / VIDEO_STREAM_INFORMATION, battery scripting fixes, MAVLink commands from scripts, Lua 5.3.6 upgrade, arming checks applet, etc.).
- Frame simulation (SITL) can embed example Lua scripts for motor mixing via the vehicleinfo schema.

### Implications for Maintenance
- New vehicle features, library backends (battery, notify, mount, esc_telem), or MAVLink messages frequently get Lua exposure + corresponding autotest coverage.
- When refreshing, you **must** search the whole tree (especially `Tools/autotest/`, vehicle `*.cpp`/ `*.h` under Ardu*, `libraries/AP_*` (BattMonitor, Notify, Mount, etc.), `wscript*`, `build_options.py`, and ReleaseNotes) in addition to AP_Scripting itself.
- Scripts can be safety-critical (arming, param integrity, mission control, throttle/engine kill) — cross-refs to arming, fence, nav, and GCS code are important context.
- The `nav_scripting` mechanism in Plane and `handle_scripting` battery pattern are good examples of how C++ "opens the door" for Lua to provide or override behavior.

This wider surface means the effective "Lua API" includes not just the bindings in `lua_bindings.cpp` but also the places in the rest of the firmware that call into or are called by the scripting subsystem.

---

## 5. Bindings & API Surface

**Generated from** `bindings.desc` (singletons, userdatas, methods, fields, enums, conditionals via `depends`).

**Major Exposed Singletons** (publicly documented on wiki + `docs/docs.lua`):
- `ahrs`, `arming`, `battery`, `gps`, `mission`, `param`, `gcs`, `SRV_Channels`, `rc`, `serial`, `logger`, `vehicle`, `terrain`, `relay`, `baro`, `esc_telem`, `efi`, `notify`, `RPM`, `button`, `scripting`, etc.

**Key Userdata Types**
- `Location`, `Vector2f`/`Vector3f`, `Quaternion`
- `mavlink_mission_item_int_t`
- `Parameter` (two styles) + dynamic table creation
- `EFI_State` / `Cylinder_Status`
- `CANFrame`, sockets, PWMSource, etc.

**Notable Capabilities**
- Dynamic parameter creation: `param:add_table(key, prefix, count)` + `param:add_param(...)` (heavily used; scripts document their own params with `// @Param:` comments).
- Auxiliary arming authentication (`arming:get_aux_auth_id()`, `set_aux_auth_passed/failed`).
- Serial driver ports: `serial:find_serial(n)` (requires `SERIALx_PROTOCOL=28`).
- Simulated devices via `find_simulated_device` + `SCR_SDEV*` params.
- `logger:write()` with full units/multipliers support.
- Mission editing, terrain queries, vehicle mode/target control, etc.

**`docs/docs.lua`** is the current canonical API surface (EmmyLua format). It can be (partly) regenerated with `./waf ... --scripting-docs`.

---

## 6. Common Implementation Patterns

- **Dynamic params helper** (seen in almost every non-trivial applet/driver):
  ```lua
  local PARAM_TABLE_KEY = 80
  local PARAM_TABLE_PREFIX = "ALAND_"
  assert(param:add_table(...))
  local function bind_add_param(name, idx, default)
      assert(param:add_param(...))
      return Parameter(PARAM_TABLE_PREFIX .. name)
  end
  ```

- **Class-style modules** (see `modules/pid.lua`): return a table; use closures for private state.

- **Arming-checks style**: Build a table of checks, use aux auth, drive severity from parameters.

- **Hardware drivers**: Early `return` if disabled; locate serial/CAN; locate backend; parse protocol in `update()` loop; push data back into ArduPilot.

- **Script_Controller.lua** pattern: RC-switch-driven script set loader using subdirectories + `scripting:restart_all()`.

- Defensive coding: Always nil-check `ahrs:get_location()`, `ahrs:get_home()`, etc.

---

## 7. Setup & Deployment

**Minimum Requirements**
- Board with ≥2 MB flash + sufficient RAM (generally not F4).
- SD card (or embed scripts in firmware via ROMFS for OEMs/no-SD boards).
- `SCR_ENABLE = 1` + reboot.

**Key Parameters**
- `SCR_HEAP_SIZE` (critical; monitor free RAM)
- `SCR_VM_I_COUNT`
- `SCR_DEBUG_OPTS` (bitmask)
- `SCR_USER1`–`6`
- `RCx_OPTION = 300`–`307` (Scripting1–8)
- `SERIALx_PROTOCOL = 28` for scripting serial ports

**Workflow**
1. Enable scripting + reboot.
2. Place `.lua` files in `APM/scripts/`.
3. Use MAVFTP or direct SD access.
4. Observe output in GCS Messages tab.

**Embedding Scripts in Firmware** — supported for manufacturers.

---

## 8. Debugging & Limitations (Heavily Reinforced by Forum)

**Most Common Real-World Failures** (from the canonical Discourse thread):
- Saving GitHub HTML page instead of clicking **Raw**.
- Forgetting reboot after `SCR_ENABLE`.
- Wrong path (`APM/scripts/` exactly).
- Insufficient heap (`SCR_HEAP_SIZE`).
- Exceeding VM instruction count (tight loops or heavy math in one callback).
- No GCS connected → no visible `gcs:send_text()` output.

**Recommended Debugging**
- `gcs:send_text()` liberally (different severities).
- `SCR_DEBUG_OPTS`.
- Start with `hello_world.lua`.
- Increase `SCR_VM_I_COUNT` and `SCR_HEAP_SIZE` when developing complex logic.
- Break long work across multiple `update()` invocations.

**Hard Limits**
- Not hard real-time.
- Memory is shared; one heavy script can starve others.
- Instruction budget protects the flight stack.

---

## 9. Notable Applets & Scripts

**Heavily Referenced in Community**
- **QuikTune family** (`VTOL-quicktune.lua`, `rover-quicktune.lua`, etc.) — automated PID tuning via maneuvers. One of the most praised real-world uses.
- `plane_package_place.lua` (payload dropping with rangefinder/LIDAR support) — associated with active community contributor yuri_rage.
- `arming-checks.lua` — sophisticated dynamic pre-arm validation using aux auth and per-vehicle parameter tables.
- `Script_Controller.lua` — RC-switchable script sets.

**Newer Applets (verified in refreshed tree)**
- `param-lockdown.lua` — MAVLink `PARAM_SET` interceptor / whitelist. When enabled (`PARAM_LOCK_ENAB`), takes over parameter writes via `gcs:set_allow_param_set()` and only permits a hardcoded list of safe parameters (battery, fence, RTL, logging, BRD_OPTIONS, etc.). Blocks and logs denied sets. Useful for automated testing or shared-GCS environments. Requires ArduPilot 4.7+. Whitelist is inside the script.
- `throttle_kill.lua` — Fixed-wing (esp. turbine) engine kill helper. Allows PWM values below `SERVO3_MIN` on a chosen channel to kill the engine when a scripting aux function (typically RCx_OPTION 300+) is active. Creates `THR_KILL_*` parameters for channel, function, value, and default state. Forces exact kill PWM when triggered.
- `RockBlock-9704.lua` — Satellite comms driver for RockBlock 9704 SBD modem. Sends periodic `HIGH_LATENCY2` MAVLink packets (via companion `rockblock2mav` at GCS). Uses a real scripting serial port + a virtual `SCR_SDEV*` port (MAVLinkHL), relay for power control (I_EN), and GPIO for boot detect (I_BTD). Configurable via many `RK9_*` parameters (period, force high-latency modes, timeouts, ports, relay/GPIO pins). Only HL2; no heartbeats or full MAVLink.

Many other applets exist for camera/gimbal control, SmartAudio, net services, aerobatics (schedules + rate-based), slung payload, terrain avoidance, winch, etc. Applets continue to be added (see `applets/` + matching `.md` files; each `.md` is intended to be mirrored on the wiki).

**Supporting Modules (refreshed tree)**
- `crsf_helper.lua` — Reusable library for building CRSF (TBS Crossfire) telemetry menus. Handles binary packing/unpacking and the event loop (parameter read/write, command poll/status). Used by CRSF-related applets (e.g. crsf-calibrate).

**Notable New/Updated Examples & Tests**
- `adsb_send.lua` — Demonstrates low-level sending of `ADSB_VEHICLE` MAVLink messages (manual packing via `mavlink:init()` + `mavlink:send_chan()`; no high-level MAVLink module required). Example simulates a circling traffic target.
- `callback_time_test.lua` (in `tests/`) — Stress-tests the Lua callback scheduler accuracy. Steps through doubling delays (1 ms → ~262 s) and measures error using `micros()`. Fails loudly if scheduling deviates beyond thresholds. Valuable for runtime validation.
- Various motor mixer, mount driver, video relay, and source-switching examples have also been added/expanded.

---

## 10. Community & Knowledge Sources

- **Official Wiki** — best structured reference (API, setup, binding extension process).
- **In-tree applets + .md files** — the actual “productized” scripts; wiki entries are derived from them.
- **discuss.ardupilot.org** (Onboard Lua Scripting category) — richest source of practical gotchas, real usage reports, and shared experiments. The “Cannot get lua scripts running” thread is essential reading.
- **GitHub** — primary distribution for examples and applets (always use Raw).
- **Discord** (Developer Chat) — mentioned in the wiki as a real-time space. Not directly readable with current tools (no Discord MCP integration). Community activity exists there but is not part of this compiled dossier unless excerpts are provided.
- Active contributors (forum): yuri_rage and others frequently help debug and upstream improvements.

---

## 11. Extending the System

1. Add desired method/field to `generator/description/bindings.desc`.
2. Add corresponding entry to `docs/docs.lua`.
3. Clean + rebuild (waf automatically reruns the generator).
4. Document the new binding.

---


## 12. Maintenance Process (Keeping The Companion Current)

The Companion is a **manually maintained synthesized reference**, not auto-generated. To ensure it stays accurate when the code changes:

1. **Trigger**: After any tree refresh (`git pull`, `git submodule update`, or user says "refreshed the src tree"), explicitly ask the AI to "update / refresh the companion" (or similar).
2. **AI Actions on Refresh** (repeatable checklist):
   - Re-`list_dir` (or terminal `find`/`ls`) on `libraries/AP_Scripting/` and key subdirs (applets, drivers, examples, modules, tests, generator, docs).
   - Read any new or changed `*.md` files (especially new applets) and sample their `.lua`.
   - Whole-repo search (excluding `libraries/AP_Scripting`): grep for "scripting", "Lua", "SCR_ENABLE", "AP_SCRIPTING", "scripting_restart", "LuaCopter", etc. in `*.py`, `*.md`, `wscript`, `*.cpp`, `*.h`, `build_options.py`.
   - Check recent history: `git log --oneline -20 -- libraries/AP_Scripting/ Tools/autotest/ Tools/scripts/build_options.py`.
   - Specifically scan `Tools/autotest/` for new `def Lua*` or `test_scripting_*` methods and `SCR_*` usage in tests.
   - Re-read `AGENTS.md` (root) if it has changed.
   - Update this file via precise edits: refresh date, Notable/Examples/Tests lists, Cross-Repository section, runtime/build notes, and this Maintenance section.
3. **Sources to Always Re-Check**:
   - In-tree: applets/*.md, examples/ new .lua, modules/ new helpers, tests/, generator/description/bindings.desc, docs/docs.lua.
   - Elsewhere: Tools/autotest/* (primary integration tests), Tools/scripts/build_options.py (features), libraries/AP_Notify (ScriptingLED), vehicle Parameters/RC/modes/GCS files, wscript rules.
4. **Best Practice**:
   - Verify every claim against actual current source (per AGENTS.md: "Always verify against actual source code. Do not fabricate").
   - The human user drives the refresh; the AI assists with search + synthesis.
   - Companion changes are documentation-only (subject to Markdown linting in CI). They do not require the full PR/test rig that new applets or bindings would.

A small helper script (e.g. `docs/scan_scripting.py`) could be added in the future to emit structured lists of current applets + autotest methods, but any such addition would itself follow normal contribution rules.

## 13. Current Knowledge Gaps (for Completeness)

- Real-time Discord channel content (ArduPilot developer Discord) has not been directly ingested (no native Discord MCP/tools available in this session; would require the user to paste relevant excerpts or logs).
- Deep internal C++ details of the Lua VM integration or every single binding evolution (only the public surface from `bindings.desc` + `docs/docs.lua` + usage in scripts is documented here).
- Exhaustive catalog of every one-off user script shared in forum build logs (focus is on canonical applets, drivers, modules, and notable examples that ship in-tree).

All entries above were re-verified by direct file reads, directory listings, and whole-repo searches against the refreshed tree. No APIs, parameters, behaviors, or limits were invented.

---

*The Companion is a living internal reference. It is updated when the source tree is refreshed so that it stays compliant with actual code and documentation (per root `AGENTS.md` emphasis on accuracy and verification against source).*
