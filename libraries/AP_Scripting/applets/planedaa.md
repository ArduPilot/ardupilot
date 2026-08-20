# Plane DAA

This script implements DAA (Detect, Alert, Avoid) for fixed wing and VTOL/quadplanes
using the "Bendy Ruler" algorithm.

Rather than using the Bendy Ruler implementation for Copter enshrined in the AC_Avoidance library,
this implementation tries to make minimial changes to the c++ core libraries required to surface
the required data to Lua and then allows the Lua to implement most of the Alert and Avoid logic
in order to allow for the maximum implementation flexibility in the face of varying regulatory
environments across the globe.

This script needs the `AP_OAScripting` bindings, which are **not in a default
firmware build** on most boards. See _Firmware build_ below.

## Setup

DAA is not a single switch — it stitches together the scripting engine, the ADS-B
avoidance subsystem, the geofence subsystem and (optionally) terrain. The
parameters below are grouped by what each piece enables. After changing any
`*_ENABLE` parameter you must reboot the autopilot before the dependent
parameters appear.

### 1. Firmware build

The applet talks to the `OAScripting` singleton, which the `AP_OAScripting`
library in AC_Avoidance exposes to Lua. The script aborts at startup if that
object is not present, so the firmware must contain it.

DAA costs roughly **7.1 kB of flash** (measured on Durandal: 1,599,176 bytes
without, 1,606,288 with), so it is deliberately **not** in a default build. It is
compiled in by default only on targets with more than 2 MB of program space —
SITL, Linux, and boards carrying external program flash such as CubeRedPrimary.
On every other board, including ordinary 2 MB boards like QiotekZealotH743,
CubeOrange+, Durandal, MatekH743 and the Pixhawk6X, you must ask for it:

- **Custom Build Server**: tick **Enable Scripted Detect and Avoid (DAA)** under
  the _Plane_ category. Its dependencies — Scripting, Object Avoidance and "ADSB"
  Avoidance — are selected for you.
- **Building yourself**: `./waf configure --board <board> --enable-AP_OASCRIPTING`
  then `./waf plane`.

To confirm a binary has it, `Tools/scripts/extract_features.py <binary>` lists
`AP_OA_SCRIPTING_ENABLED` when the feature is present.

### 2. Scripting engine

| Parameter | Value | Notes |
|-----------|-------|-------|
| `SCR_ENABLE` | `1` | Enable Lua scripting (reboot required). |
| `SCR_VM_I_COUNT` | `1000000` | **Increase from the default.** `planedaa.lua` is large and the per-loop instruction budget must be raised or the VM will fault. The applet warns at startup below `150000`. |

Do not treat `SCR_VM_I_COUNT` as a value to trim. It is an _instruction_ budget,
not a memory allocation — raising it costs no RAM. `1000000` is the top of the
parameter's documented range, and the scripting thread runs at the lowest
priority in the system, so a script that runs long cannot delay flight control;
it only delays other scripts and its own next update.

Overrunning the budget, by contrast, is not recoverable in flight: the Lua VM
does not skip an update, it kills the script outright, so avoidance stops for
the rest of the flight and the resulting sticky error then fails the pre-arm
check on the next arm. The heaviest thing the applet does is the candidate-heading
sweep when the aircraft is boxed in and no heading clears, which is exactly the
situation where losing avoidance matters most.

To measure the applet's real cost on your airframe, set `SCR_DEBUG_OPTS` bit 3
(value `8`). That logs an `SCR` record per script run carrying its runtime and
Lua heap use, which is the only place either is visible in a dataflash log.

**You must install TWO files, and they go in DIFFERENT places.** This trips
people up: `planedaa.lua` depends on the `mavlink_wrappers` module, which it
loads with `require("mavlink_wrappers")` at startup. If that module is not on
the SD card in the right place, the applet fails to load immediately — it does
not start in a degraded mode.

The two files live in different source folders and must be copied to different
destination folders:

| Source (in the ArduPilot tree) | Destination (on the SD card) |
|--------------------------------|------------------------------|
| `AP_Scripting/applets/planedaa.lua` | `APM/scripts/planedaa.lua` |
| `AP_Scripting/modules/mavlink_wrappers.lua` | `APM/scripts/modules/mavlink_wrappers.lua` |

Notes:

- `mavlink_wrappers.lua` is a **module**, so it lives under `modules/`, not next
  to the applet. ArduPilot's `require` searches `scripts/` and
  `scripts/modules/`, so the module must end up in `APM/scripts/modules/`.
- Do **not** rename either file — `require("mavlink_wrappers")` looks for a file
  named exactly `mavlink_wrappers.lua`.
- If you only copy `planedaa.lua`, the script will not run. The most common
  setup failure is forgetting the module or putting it in the wrong folder.

### 3. Dynamic traffic detection (aircraft / drones / ADS-B)

Detection of aircraft, drones and other MAVLink/ADS-B traffic is served from the
`AP_Avoidance` database, which is only populated when avoidance is enabled.

| Parameter | Value | Notes |
|-----------|-------|-------|
| `AVD_ENABLE` | `1` | **Required.** Without it the avoidance database stays empty and `OAScripting:find_aircraft()`/`find_threats()` see no dynamic traffic. |
| `ADSB_TYPE` | per hardware | Set to match your ADS-B receiver so `ADSB_VEHICLE` messages are processed. Use `1` (MAVLink) when the traffic arrives as `ADSB_VEHICLE` over a MAVLink link with no ADS-B hardware attached, such as a feed from a companion computer. In SITL this is the path used to inject simulated traffic. |

The Well Clear and Near Mid-Air Collision (NMAC) volumes that DAA reads at
startup default to the **ASTM F3442M-23** thresholds for crewed aircraft (with
NMAC falling back to the FAA / RTCA DO-396 figures, as ASTM F3442M does not
itself define NMAC). The firmware defaults are already these values — change
them only with good reason:

| Parameter | Default | Equivalent | Source |
|-----------|---------|------------|--------|
| `AVD_WCLR_XY` | `609.6` m | 2000 ft | ASTM F3442M-23 Well Clear horizontal |
| `AVD_WCLR_Z` | `76.2` m | 250 ft | ASTM F3442M-23 Well Clear vertical |
| `AVD_NMAC_XY` | `152.4` m | 500 ft | FAA NMAC horizontal |
| `AVD_NMAC_Z` | `30.48` m | 100 ft | RTCA DO-396 (TCAS MOPS) NMAC vertical |

### 4. Fence avoidance

| Parameter | Value | Notes |
|-----------|-------|-------|
| `FENCE_ENABLE` | `1` | **Required** for any fence avoidance — `find_threats()` only walks fences when the fence subsystem is enabled. |
| `FENCE_TYPE` | bitmask | Selects which fences are active. Altitude-fence avoidance specifically needs **bit 0 (`ALT_MAX`)** and/or **bit 3 (`ALT_MIN`)**. |
| `FENCE_ACTION` | non-zero (autopilot default RTL) | The autopilot's own breach response, active in **all** modes. It fires immediately on a fence breach and **pre-empts** planedaa's trapped-failsafe for the fence case. planedaa adds proactive avoidance and a `DAA_TRAP_ACT` backstop, but only in nav modes (AUTO/GUIDED/RTL/LOITER/CRUISE/FBWB), so keeping `FENCE_ACTION` non-zero is recommended as the all-mode backstop. Set `0` (report-only) only if planedaa should own the fence trap and you never rely on fence protection while hand-flying. |
| `FENCE_ALT_MAX_TP` / `FENCE_ALT_MIN_TP` | frame | Frame of the altitude fences; set to match your intent (see terrain note below). |

When flying near fences in wind, `DAA_WIND_MARG` widens the standoff in proportion
to wind speed (above `DAA_WIND_MIN`) so cross-track drift is less likely to carry
the aircraft across a boundary. See the Parameters table for both.

### 5. Terrain (default altitude frame)

`DAA_AVD_ALT_TP` defaults to **3 (above terrain)**, and altitude-fence avoidance
reads terrain-relative safe altitudes when the fence frames are terrain. If you
use any terrain frame you must enable terrain and have terrain data available:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `TERRAIN_ENABLE` | `1` | Required when `DAA_AVD_ALT_TP` or `FENCE_ALT_*_TP` use the terrain frame. |

### 6. Activation

DAA **defaults to ON** at boot. To toggle it in flight, assign the DAA scripting
function to an RC channel:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `DAA_ACT_FN` | `308` | Scripting/aux function used by the script. |
| `RCx_OPTION` | `308` | Assign to a switch channel to toggle DAA. Logic is inverted: switch **Low = enabled**, switch **High = disabled**. |

For quadplanes, set `Q_ENABLE = 1` as usual (avoidance manoeuvres are only
commanded while the vehicle is in fixed-wing flight).

## Parameters

The script adds the following `DAA_` parameters to control its behaviour. It also
reuses the existing core `AVD_` parameters (`AVD_WCLR_XY`, `AVD_WCLR_Z`,
`AVD_NMAC_XY`, `AVD_NMAC_Z`, etc.) that define the Well Clear and Near Mid-Air
Collision volumes, and the `FENCE_*` parameters for the altitude/geo fences.

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `DAA_ACT_FN` | 308 | | RC option / scripting function used to activate the DAA capability. |
| `DAA_MARGIN_FENCE` | 0 | m | Avoidance margin kept clear of the geofence. `0` (default) uses `WP_LOITER_RAD`, so the standoff equals one loiter circle and fences don't thrash; set a non-zero value to override. |
| `DAA_LKAHD` | 1000 | m | Avoidance lookahead distance. |
| `DAA_UPDATE_RATE` | 10 | Hz | Rate at which avoidance is processed. |
| `DAA_MARGIN_CA` | 50 | m | Avoidance margin for crewed aircraft (fixed wing, helicopter, eVTOL), over and above the Well Clear margin `AVD_WCLR_XY`. |
| `DAA_MARGIN_CA_Z` | 30 | m | Vertical avoidance margin for crewed aircraft, over and above the Well Clear vertical separation `AVD_WCLR_Z`. An aircraft triggers the loiter-to-altitude only while its altitude difference from the vehicle is below `AVD_WCLR_Z + DAA_MARGIN_CA_Z`. The vertical mirror of `DAA_MARGIN_CA`. |
| `DAA_MARGIN_WTH` | 173 | m | Avoidance radius for weather/clouds/rain. |
| `DAA_MARGIN_BIRD` | 100 | m | Avoidance margin for migratory birds. |
| `DAA_MARGIN_PREY` | 200 | m | Avoidance radius for birds of prey. |
| `DAA_MARGIN_UAV` | 50 | m | Avoidance radius for UAVs/drones (MAVLink sourced). |
| `DAA_MARGIN_AIS` | 50 | m | Avoidance radius for AIS ship contacts (MAVLink sourced). |
| `DAA_MARGIN_PRX` | 50 | m | Avoidance radius for obstacles detected by proximity sensors. |
| `DAA_BR_RATIO` | 1.5 | | BendyRuler will avoid changing bearing unless the ratio of the previous margin to the newly calculated margin is at least this much. |
| `DAA_BR_ANGLE` | 45 | deg | BendyRuler resists changing the current bearing if the change exceeds this angle. |
| `DAA_AVD_ALT` | 50 | m | Altitude to loiter/descend to when avoiding a crewed aircraft contact. Ignored if zero. |
| `DAA_AVD_ALT_TP` | 3 | | Frame of `DAA_AVD_ALT` (0: absolute, 1: above home, 2: above origin, 3: above terrain). |
| `DAA_LTR_COOL_S` | 10 | s | Time the aircraft loiter-to-altitude is held after the aircraft was last detected before releasing back to the mission. Hysteresis against a briefly-dropped or laggy ADS-B feed thrashing the vehicle between GUIDED (loiter) and AUTO (mission). Set to 0 to release as soon as the aircraft is no longer detected. |
| `DAA_AVD_ALERT` | 1 | | Whether to alert on avoidance (0: none, 1: alert). |
| `DAA_AVD_ACTION` | 1 | | Whether to act on avoidance (0: none, 1: avoid). |
| `DAA_MARGIN_ALT` | 20 | m | Proactive buffer inside the safe altitude-fence limits at which DAA starts clamping the commanded altitude. |
| `DAA_ALT_HYST_M` | 10 | m | Hysteresis band for altitude-fence avoidance, preventing chatter as the plane levels off. |
| `DAA_ALT_COOL_S` | 15 | s | Minimum time between altitude-fence "levelling off" notices, so brief re-engagements do not re-spam the GCS. |
| `DAA_HEADING_INC` | 1.5 | deg | Angular step used when searching candidate headings around the target bearing for a collision-free path. The search sweeps a full circle in increments of this size, alternating left and right. Smaller values search more finely but cost more CPU per update. |
| `DAA_WIND_MIN` | 2 | m/s | Minimum wind speed before the wind-aware avoidance behaviour engages. Below this, the still-air path is used so calm-air behaviour is unchanged. Gates both the wind-aware look-ahead projection and the wind-scaled fence margin. |
| `DAA_WIND_MARG` | 5 | m per m/s | Extra fence avoidance margin added per m/s of wind above `DAA_WIND_MIN`. The standoff from fences is widened by `DAA_WIND_MARG * max(0, wind_speed - DAA_WIND_MIN)`, giving the controller buffer to absorb cross-track drift so the aircraft is less likely to be blown across the boundary in wind. Set to 0 to disable wind scaling. |
| `DAA_SLEW_DPS` | 20 | deg/s | Maximum rate the commanded avoidance heading is allowed to change. Rate-limiting the heading smooths the oscillation a per-cycle bendy ruler produces against moving obstacles and near fences. Bypassed when the estimated time-to-conflict is below `DAA_SLEW_URG`, so an urgent manoeuvre keeps full authority. Set to 0 to disable the slew limit. Must be below the turn rate the airframe can actually fly or it has no effect — see _Tuning for your vehicle_. |
| `DAA_SLEW_URG` | 4 | s | If the estimated time-to-conflict with a moving obstacle is below this, the `DAA_SLEW_DPS` slew limit is bypassed so the aircraft can turn at full authority. Set to 0 to always apply the slew limit. |
| `DAA_SIDE_HOLD` | 3 | s | Once a left/right avoidance side is committed for an obstacle, the opposite side must be preferred by the bendy ruler for at least this long before the aircraft is allowed to switch sides. Stops the left/right flip-flop when avoiding a moving obstacle. Set to 0 to disable side commitment. |
| `DAA_CPA_MIN` | 2 | m/s | Minimum closing speed for a moving obstacle to be treated as a conflict. CPA (Closest Point of Approach) is the predicted minimum separation between the vehicle and a moving obstacle on their current tracks. An obstacle whose CPA stays beyond the well-clear distance and which is opening range faster than this is not avoided (it is leaving). Set to 0 to avoid regardless of closing speed. |
| `DAA_STALE_S` | 3 | s | Traffic-feed staleness warning threshold. When avoiding a network-sourced moving obstacle (ADS-B drone/aircraft) whose position has not updated for longer than this, a `traffic stale Ns` warning is sent to the GCS (the DAA is acting on lagged data, e.g. from an intermittent telemetry/ADS-B link), and a `lost <label>` warning if it then disappears. Fences are on-board and never go stale. Set to 0 to disable. |
| `DAA_TRAP_ACT` | 0 | | Trapped-failsafe action when avoidance cannot find a way out (boxed in, or unable to keep clear of an obstacle for `DAA_TRAP_S`). `0`: disabled (avoidance just keeps trying), `1`: RTL, `2`: QRTL, `3`: QLOITER, `4`: QLAND. QLOITER/QRTL/QLAND have zero turn radius (safest in a tight space); the VTOL options fall back to RTL if `Q_ENABLE=0`. See _Trapped failsafe_ below. |
| `DAA_TRAP_S` | 5 | s | Avoidance must be unable to find a way out continuously for this long before `DAA_TRAP_ACT` fires. Rejects transient clutter. |
| `DAA_TRAP_CLR_S` | 4 | s | For a trap caused by a MOVING obstacle (drone/aircraft), resume the previous mode this long after the failsafe fired (if the obstacle has not passed, forward flight simply re-triggers it). A trap caused by a fixed obstacle (fence) is not auto-recovered — it is held until the pilot changes mode. |
| `DAA_TRAP_ESC_ACT` | 2 | | Escalation action for when `DAA_TRAP_ACT` would command the mode the aircraft is ALREADY in (e.g. trapped mid-RTL with `DAA_TRAP_ACT=RTL`) — commanding it again would do nothing, so it escalates to this. `1`: RTL, `2`: QRTL, `3`: QLOITER, `4`: QLAND (VTOL options fall back to RTL). Set equal to `DAA_TRAP_ACT` to disable escalation. |

## Tuning for your vehicle

The margins and the bendy-ruler behaviour are not one-size-fits-all: the same
fence is "tight" for a fast, agile aircraft and "roomy" for a slow one. Two
airframe properties drive the tuning.

**Turn radius.** The radius the aircraft actually flies while avoiding is set by
L1 and the bank limit, roughly `R ≈ v² / (g·tan φ)` — it grows with the _square_
of cruise speed (`AIRSPEED_CRUISE`) and shrinks with the achievable bank angle
(`ROLL_LIMIT_DEG`). This is **not** `WP_LOITER_RAD`, which is only the
loiter-circle radius and is usually far more conservative than the airframe can
actually turn — do not size DAA margins from it. A very agile VTOL, for example,
may fly a ~30 m avoidance turn while `WP_LOITER_RAD` is 90 m.

**Command bandwidth.** How faithfully the airframe tracks a changing commanded
heading. This is the property that decides how much the _command-stability_
parameters matter.

### Command stability and `DAA_BR_RATIO`

`DAA_BR_RATIO` is the side-commit hysteresis: the bendy ruler only switches to
the other side of an obstacle when the new side is this many times clearer.
Because an agile, high-bandwidth airframe reproduces every commanded heading
change immediately, any side-to-side indecision in the bendy ruler shows up as a
**visible dogleg** — the aircraft banks hard one way, then the other. A slower
airframe hides the same indecision because its own roll response low-passes it.
So the more agile the aircraft, the more command stability matters — and,
importantly, _"it looks smooth on the slow airframe"_ does **not** mean the
bendy ruler is well tuned; the airframe may just be masking it.

When the aircraft is forced to hug a fence standoff (the geometry leaves no room
to go wide), the left/right choice becomes bistable. The applet holds the
committed escape direction through the hug rather than flip-flopping, and only
switches to the far side if that side _genuinely_ clears (positive clearance) —
so containment is never traded away for smoothness. `DAA_BR_RATIO` still governs
how much clearer the far side must be before a normal (non-hugging) side switch;
raise it if you see doglegs when skirting fences, lower it if avoidance feels
sluggish to re-plan.

### Margins and look-ahead

The fence standoff (`DAA_MARGIN_FENCE`) defaults to `0`, which uses the turn
radius `WP_LOITER_RAD` — one loiter circle, and never below the turn radius so
fences don't thrash (the startup check warns if a non-zero margin is set below
the turn radius). To override, size it from how far the aircraft travels while
reacting — i.e. from airspeed — but keep it at least the turn radius. Size
`DAA_LKAHD` the same way and keep it at least a few times the turn radius (the
startup check warns below 3×R); a very long look-ahead (10×+) can over-commit
the far-field path. In wind, `DAA_WIND_MARG` widens the fence standoff
automatically.

The look-ahead also explains why an `AVOIDING:` distance can be much larger than
the obstacle's standoff. The bendy-ruler projects your path forward up to
`DAA_LKAHD` (e.g. 1000 m) and starts deviating as soon as that projected path
would pass within the standoff of the obstacle (for a drone, `AVD_UAV_XY +
DAA_MARGIN_UAV`; for a fence, `DAA_MARGIN_FENCE`). So with an obstacle sitting on
or near your path, the aircraft begins avoiding — and the `AVOIDING:` message
reports the true range — while the obstacle is still hundreds of metres ahead.
The standoff is the _lateral_ clearance held around the obstacle, not the range
at which avoidance begins.

### Moving obstacles and closest-approach

Moving obstacles (drones, birds, AIS) are gated by a closest-point-of-approach
(CPA) test as well as position: one that is opening range and whose predicted
miss stays beyond its keep-out radius (for a drone, `AVD_UAV_XY`) is treated as
_leaving_ and is not avoided — this is what stops the aircraft from manoeuvring
for traffic that is already past or diverging. A crewed aircraft uses the same
test with the `AVD_WCLR_XY` well-clear radius, and inside that radius it is always
treated as a conflict (a missing or uncertain velocity therefore always errs
toward avoiding — safer first).

This CPA decision is re-made from the obstacle's **current** geometry every
update cycle, with no hold, so a manoeuvring or unpredictable drone is always
tracked on fresh data. Near a marginal crossing that can cost a few extra heading
reversals; this is deliberate — the heading is slew-rate limited (`DAA_SLEW_DPS`)
so the motion stays bounded, and responsiveness is preferred over a smoother but
laggier committed path.

### Sizing the traffic standoff

The effective standoff held around a drone is `AVD_UAV_XY + DAA_MARGIN_UAV` (and
around a crewed aircraft, `AVD_WCLR_XY + DAA_MARGIN_CA`). **Size it larger than the
avoidance turn radius `R`**, or the geometry is unwinnable: once the aircraft is
already inside the standoff it cannot turn out of it, and the bendy ruler can only
spiral — which shows up in the log as a heading that sweeps through hundreds of
degrees while the separation keeps shrinking. Symptomatically, avoidance that is
_acquired_ at long range resolves with a small, smooth heading change, while the
same aircraft acquiring the same traffic from inside the standoff produces a violent
turn and still misses by less than the standoff. The startup check warns when
`AVD_UAV_XY` is below `WP_LOITER_RAD`, which is a deliberately conservative proxy —
see the turn-radius note above, and prefer sizing from `R ≈ v²/(g·tan φ)`.

Add reaction distance on top of the turn radius: closing speed times the time it
takes to acquire, decide and roll in. For a head-on encounter that closing speed is
the sum of both groundspeeds, so a standoff sized only for the turn radius is
already too small.

`DAA_SLEW_DPS` must be **below the turn rate the airframe can actually fly**,
`deg(AIRSPEED_CRUISE / R)`, or the limiter never binds and the smoothing does
nothing. The startup check only warns above `1.5 ×` that rate, so a value between
the achievable rate and 1.5× is silently inert — check it by hand. A limit that
does bind is still safe for urgent manoeuvres: it is bypassed when the estimated
time-to-conflict falls below `DAA_SLEW_URG`.

### Network-fed traffic and data-link quality

If traffic reaches the autopilot over a network link (a companion computer, an LTE
or telemetry feed) rather than a directly attached receiver, the feed usually
arrives in bursts rather than at a steady cadence. Two consequences:

- The obstacle's position is held stale between bursts, so the geometry the bendy
  ruler works from lags reality by up to one gap. **Add that lag, times the closing
  speed, to the standoff** — a one-second hold at a 40 m/s closing speed is 40 m of
  position error before any of the turn-radius sizing applies.
- Set `DAA_STALE_S` just above the feed's normal gap, not far above it. Too high and
  it only reports a link that has already collapsed rather than one degrading.

The `Age` field in the `DAAV` log record measures the time since the last update
_arrived_, so it reveals gaps and jitter but **cannot see a constant transport
delay** — a uniformly delayed feed looks perfectly fresh. To characterise the feed
itself, set `ADSB_LOG=2` (log all) for a per-message `ADSB` record across the whole
flight; `DAAV` is only written while actively avoiding, so on its own it cannot show
what the feed was doing before an obstacle was acquired.

### Trapped failsafe

If avoidance cannot find any clear heading — boxed in by fences, or unable to
keep clear of an obstacle — continuously for `DAA_TRAP_S` seconds, `DAA_TRAP_ACT`
fires a failsafe mode. The recommended actions have zero turn radius so they get
the aircraft out of a tight space without needing room to turn: `QLOITER` (stop
and hover), `QRTL` (VTOL return) or `QLAND`. On a non-VTOL airframe (`Q_ENABLE=0`)
these fall back to `RTL`. Set `DAA_TRAP_ACT=0` to disable the failsafe entirely
(avoidance just keeps trying). The trap fires on a sustained fence breach **or**
aircraft near-miss. Because the autopilot's own `FENCE_ACTION` (if non-zero) acts
first on a fence breach in every mode, it pre-empts the trap's fence case — leaving
the trap as the nav-mode backstop for aircraft (and for fences too when
`FENCE_ACTION=0`). The fence case additionally **stands down entirely** when
`FENCE_ACTION` is non-zero _and_ `FENCE_OPTIONS` bit 0 (`DISABLE_MODE_CHANGE`) is
set, because the core would refuse the trap's mode change anyway; the startup check
reports this. Between that and `DAA_TRAP_ACT=0`, it is easy to end up with no escape
mechanism at all in exactly the boxed-in situation the trap exists for — worth
confirming against the startup messages rather than assuming it is armed.

Recovery depends on what caused the trap:

- **Fixed obstacle (fence)** — sticky: held until the pilot changes mode, because
  a fence will not move out of the way.
- **Moving obstacle (drone/aircraft)** — auto-recovers to the previous mode after
  `DAA_TRAP_CLR_S`, on the assumption it has passed. If it has not, resuming
  forward flight simply re-triggers the failsafe.

If `DAA_TRAP_ACT` would command the mode the aircraft is _already_ in (e.g.
trapped mid-`RTL` with `DAA_TRAP_ACT=RTL`), commanding it again would do nothing,
so it escalates to `DAA_TRAP_ESC_ACT` (default `QRTL`) to actually stop the
aircraft. Set `DAA_TRAP_ESC_ACT` equal to `DAA_TRAP_ACT` to disable escalation.

## Logging

The script writes the following messages to the dataflash log to record its
DAA (Detect, Alert, Avoid) decisions. All location fields (`TLat`/`TLng`) are in
degrees, altitudes (`TAlt`) in metres, and distances in metres. The altitude
frame field `TFra` is `0` = AMSL, `1` = home-relative, `3` = terrain-relative.

Two settings outside the applet are worth enabling when investigating its
behaviour, neither of which is on by default:

| Parameter | Value | What it adds |
|-----------|-------|--------------|
| `SCR_DEBUG_OPTS` | bit 3 (`8`) | an `SCR` record per script run with its runtime and Lua heap use — the only place either is visible in a log, and the way to check headroom against `SCR_VM_I_COUNT` and `SCR_HEAP_SIZE` |
| `ADSB_LOG` | `2` | an `ADSB` record per received `ADSB_VEHICLE`, across the whole flight rather than only while avoiding |

### DAAD — Detect

Written once per avoidance cycle when an obstacle has been detected and a new
target to dodge it has been computed.

| Field | Description |
|-------|-------------|
| `Obs`  | Obstacle found (1/0) |
| `DstF` | Distance to the detected obstacle (m) |
| `DstT` | Distance to the proposed new target that avoids the obstacle (m) |
| `HdgB` | Best bearing found to avoid the obstacle (deg) |
| `Tfnd` | Avoidance target found (1/0) |
| `TLat` | Latitude of the proposed new target (deg) |
| `TLng` | Longitude of the proposed new target (deg) |
| `TAlt` | Altitude of the proposed new target (m) |
| `TFra` | Altitude frame of `TAlt` |
| `ObjT` | `OBSTACLE_TYPE` of the detected object (see table below) |

### DAAV — aVoid

Written when the script commands an avoidance manoeuvre towards a DAA target.

| Field | Description |
|-------|-------------|
| `DstO` | Distance to the obstacle being avoided (m) |
| `TLat` | Latitude of the DAA target (deg) |
| `TLng` | Longitude of the DAA target (deg) |
| `TAlt` | Altitude of the DAA target (m) |
| `TFra` | Altitude frame of `TAlt` |
| `DstH` | Horizontal distance to the obstacle (m) |
| `DstZ` | Vertical distance to the obstacle (+ve is up) (m) |
| `ObjT` | `OBSTACLE_TYPE` of the obstacle (see table below) |

### DAAG — General aviation aircraft

Written when an aircraft (typically with an ICAO/ADSB identifier) is detected.

| Field | Description |
|-------|-------------|
| `DstF` | Distance to the detected aircraft (m) |
| `TLat` | Latitude of the aircraft (deg) |
| `TLng` | Longitude of the aircraft (deg) |
| `TAlt` | Altitude of the aircraft (m) |
| `TFra` | Altitude frame of `TAlt` |
| `DstH` | Horizontal distance to the aircraft (m) |
| `DstZ` | Vertical distance to the aircraft (+ve is up) (m) |
| `ICAO` | Integer value of the aircraft's ICAO code, if available |

### OBSTACLE_TYPE values

The `ObjT` field uses the following `OBSTACLE_TYPE` enumeration:

| Value | Name | Description |
|-------|------|-------------|
| 0  | GENERAL | Generic obstacle of unknown type |
| 1  | MAV_SYSID | Another MAVLink drone with a MAV_SYSID |
| 2  | CREWED_AIRCRAFT | Crewed aircraft, usually with an ICAO ADSB identifier |
| 3  | WEATHER | Weather |
| 4  | BIRD_MIGRATORY | Migratory bird(s), e.g. Canada Geese |
| 5  | BIRD_OF_PREY | A bird that might attack the vehicle |
| 6  | FENCE_HOME | All fixed/unmovable fences |
| 7  | FENCE_CIRCLE_INCLUSION | Circular inclusion fence |
| 8  | FENCE_CIRCLE_EXCLUSION | Circular exclusion fence |
| 9  | FENCE_POLYGON_INCLUSION | Polygon inclusion fence |
| 10 | FENCE_POLYGON_EXCLUSION | Polygon exclusion fence |
| 11 | FENCE_LUA | Fence defined in Lua |
| 12 | PROXIMITY | Detected by a proximity sensor, typically close |
| 13 | AIS | AIS-tracked maritime (ship) vehicle |
| 14 | FENCE_ALT_MAX | Max altitude fence (FENCE_TYPE bit 0) |
| 15 | FENCE_ALT_MIN | Min altitude fence (FENCE_TYPE bit 3) |
