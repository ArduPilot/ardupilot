# Plane DAA

This script implements DAA (Detect, Alert, Avoid) for fixed wing and VTOL/quadplanes
using the "Bendy Ruler" algorithm.

Rather than using the Bendy Ruler implementation for Copter enshrined in the AC_Avoidance library,
this implementation tries to make minimial changes to the c++ core libraries required to surface
the required data to Lua and then allows the Lua to implement most of the Alert and Avoid logic
in order to allow for the maximum implementation flexibility in the face of varying regulatory
environments across the globe.

This means that it's quite likely that if you have policies you need to apply that don't fit this
implementation with its _extensive_ set of parameters, you should be able to make most changes you
might need by **changing the lua script** to fit your requirements.

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

DAA costs roughly **7 kB of flash**, so it is deliberately **not** in a default build. It is
compiled in by default only on targets with more than 2 MB of program space. This includes
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
| `SCR_VM_I_COUNT` | `250000` | **Increase from the default.** `planedaa.lua` is large and the per-loop instruction budget must be raised or the VM will fault. The applet warns at startup below `150000`. |

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

**Known limitation: a headwind on approach to a fence gets little or no extra
margin.** `DAA_WIND_MARG` scales off `ahrs:get_wind()` - the AHRS/EKF's own wind
estimate, which planedaa reads and does not second-guess. A wind blowing along the
current track (a straight-in headwind or tailwind) is aliased with airspeed-sensor
bias in that estimator: without a heading change relative to the wind, the two
cannot be told apart, and the reported estimate can read near zero even in a real,
substantial wind. A crosswind has no such ambiguity and is estimated correctly.
Confirmed on a live SITL comparison: identical 5 m/s wind, only the direction
relative to a fixed approach track differed - a pure headwind gave an EKF estimate
of ~0.04 m/s, a pure crosswind gave the correct ~5 m/s, and `DAA_WIND_MARG` widened
the standoff in one case and not the other. This is a property of the estimator,
not of planedaa's use of it, and there is no fix at this layer - raise
`DAA_MARGIN_FENCE` itself if you routinely fly headwind-on-approach geometries near
fences and want margin that does not depend on the wind estimate.

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
| `DAA_LKAHD_M` | 1000 | m | How far along each candidate heading the bendy ruler probes for a clear path (the second leg probes a further 2×). |
| `DAA_DETECT_M` | 1000 | m | How far ahead obstacles are detected at all, and the range beyond which one is not announced. Shortening this costs detection — at 250 m the drone-avoidance and fence-alert autotests stop firing. Crewed traffic is unaffected: `detect_aircraft()` uses `AVD_WCLR_XY + DAA_MARGIN_CA`. |
| `DAA_PLAN_M` | 1000 | m | Minimum distance along the chosen bearing at which the commanded avoidance target is placed — see _Where the commanded avoidance target is placed_. Raise before lowering. |
| `DAA_UPDATE_RATE` | 10 | Hz | Rate at which avoidance is processed. |
| `DAA_MARGIN_CA` | 50 | m | Avoidance margin for crewed aircraft (fixed wing, helicopter, eVTOL), over and above the Well Clear margin `AVD_WCLR_XY`. |
| `DAA_MARGIN_CA_Z` | 30 | m | Vertical avoidance margin for crewed aircraft, over and above the Well Clear vertical separation `AVD_WCLR_Z`. An aircraft triggers the loiter-to-altitude only while its altitude difference from the vehicle is below `AVD_WCLR_Z + DAA_MARGIN_CA_Z`. The vertical mirror of `DAA_MARGIN_CA`. |
| `DAA_MARGIN_UAV` | 50 | m | Avoidance radius for UAVs/drones (MAVLink sourced). |
| `DAA_MARGIN_AIS` | 50 | m | Avoidance radius for AIS ship contacts. **Not active by default — see "Proximity and AIS obstacles" below.** |
| `DAA_MARGIN_PRX` | 50 | m | Avoidance radius for obstacles detected by proximity sensors. **Not active by default — see "Proximity and AIS obstacles" below.** |
| `DAA_BR_RATIO` | 1.5 | | BendyRuler will avoid changing bearing unless the ratio of the previous margin to the newly calculated margin is at least this much. |
| `DAA_BR_ANGLE` | 45 | deg | BendyRuler resists changing the current bearing if the change exceeds this angle. A change above it is also the point at which the smoothing asks whether there is time to damp it — see `DAA_SLEW_DPS`. |
| `DAA_AVD_ALT` | 50 | m | Altitude to loiter/descend to when avoiding a crewed aircraft contact. Ignored if zero. |
| `DAA_AVD_ALT_TP` | 3 | | Frame of `DAA_AVD_ALT` (0: absolute, 1: above home, 2: above origin, 3: above terrain). |
| `DAA_LTR_COOL_S` | 10 | s | Time the aircraft loiter-to-altitude is held after the aircraft was last detected before releasing back to the mission. Hysteresis against a briefly-dropped or laggy ADS-B feed thrashing the vehicle between GUIDED (loiter) and AUTO (mission). Set to 0 to release as soon as the aircraft is no longer detected. |
| `DAA_AVD_ALERT` | 1 | | Whether to alert on avoidance (0: none, 1: alert). |
| `DAA_AVD_ACTION` | 1 | | Whether to act on avoidance (0: none, 1: avoid). |
| `DAA_MARGIN_ALT` | 20 | m | Proactive buffer inside the safe altitude-fence limits at which DAA starts clamping the commanded altitude. |
| `DAA_ALT_HYST_M` | 10 | m | Hysteresis band for altitude-fence avoidance, preventing chatter as the plane levels off. |
| `DAA_ALT_COOL_S` | 15 | s | Minimum time between altitude-fence "levelling off" notices, so brief re-engagements do not re-spam the GCS. |
| `DAA_HEADING_INC` | 1.5 | deg | Angular step used when searching candidate headings around the target bearing for a collision-free path. The search sweeps a full circle in increments of this size, alternating left and right. Smaller values search more finely but cost more CPU per update. |
| `DAA_WIND_MIN` | 2 | m/s | Minimum wind speed before the wind-scaled fence margin (`DAA_WIND_MARG`) engages. Below this the standoff is not widened. It no longer conditions the look-ahead projection: that projection is a _turn_ lead and is always applied — see _Why a candidate heading is judged from where the turn ends_. |
| `DAA_WIND_MARG` | 5 | m per m/s | Extra fence avoidance margin added per m/s of wind above `DAA_WIND_MIN`. The standoff from fences is widened by `DAA_WIND_MARG * max(0, wind_speed - DAA_WIND_MIN)`, giving the controller buffer to absorb cross-track drift so the aircraft is less likely to be blown across the boundary in wind. Set to 0 to disable wind scaling. |
| `DAA_SLEW_DPS` | 20 | deg/s | Maximum rate the commanded avoidance heading is allowed to change. Rate-limiting the heading smooths the oscillation a per-cycle bendy ruler produces against moving obstacles and near fences. Bypassed when the estimated time-to-conflict is below `DAA_SLEW_URG`, and also when a change larger than `DAA_BR_ANGLE` could not be slewed through before the conflict, so an urgent manoeuvre keeps full authority. Set to 0 to disable the slew limit. Must be below the turn rate the airframe can actually fly or it has no effect — see _Tuning for your vehicle_. |
| `DAA_SLEW_URG` | 4 | s | If the estimated time-to-conflict with a moving obstacle is below this, the `DAA_SLEW_DPS` slew limit is bypassed so the aircraft can turn at full authority. Set to 0 to always apply the slew limit. |
| `DAA_SIDE_HOLD` | 3 | s | Once a left/right avoidance side is committed for an obstacle, the opposite side must be preferred by the bendy ruler for at least this long before the aircraft is allowed to switch sides. Stops the left/right flip-flop when avoiding a moving obstacle. Set to 0 to disable side commitment. |
| `DAA_CPA_MIN` | 2 | m/s | Minimum closing speed for a moving obstacle to be treated as a conflict. CPA (Closest Point of Approach) is the predicted minimum separation between the vehicle and a moving obstacle on their current tracks. An obstacle whose CPA stays beyond the well-clear distance and which is opening range faster than this is not avoided (it is leaving). Set to 0 to avoid regardless of closing speed. |
| `DAA_STALE_S` | 3 | s | Traffic-feed staleness warning threshold. When avoiding a network-sourced moving obstacle (ADS-B drone/aircraft) whose position has not updated for longer than this, a `traffic stale Ns` warning is sent to the GCS (the DAA is acting on lagged data, e.g. from an intermittent telemetry/ADS-B link), and a `lost <label>` warning if it then disappears. Fences are on-board and never go stale. Set to 0 to disable. |
| `DAA_TRAP_ACT` | 0 | | Trapped-failsafe action when avoidance cannot get the vehicle out of trouble — a sustained fence breach, a crewed aircraft inside the near-miss volume, or a hung avoidance (see `DAA_HUNG_ALRT_S`) — for `DAA_TRAP_S`. `0`: disabled (avoidance just keeps trying), `1`: RTL, `2`: QRTL, `3`: QLOITER, `4`: QLAND. QLOITER/QRTL/QLAND have zero turn radius (safest in a tight space); the VTOL options fall back to RTL if `Q_ENABLE=0`. See _Trapped failsafe_ below. |
| `DAA_TRAP_S` | 5 | s | The vehicle must be compromised continuously for this long before `DAA_TRAP_ACT` fires. Rejects transient clutter. |
| `DAA_TRAP_CLR_S` | 4 | s | For a trap caused by a MOVING obstacle (drone/aircraft), resume the previous mode this long after the failsafe fired (if the obstacle has not passed, forward flight simply re-triggers it). A trap caused by a fixed obstacle (fence) is not auto-recovered — it is held until the pilot changes mode. |
| `DAA_HUNG_ALRT_S` | 60 | s | Avoidance is **hung** when it has been running this long without the vehicle getting any closer to its navigation target. The classic case is a waypoint inside an exclusion-fence standoff: it cannot be reached while avoiding, so the mission pull and the fence push never reconcile and the vehicle orbits indefinitely — without ever breaching anything, which is why neither the autopilot's `FENCE_ACTION` nor the other two trap causes can see it. A hung avoidance raises a `HUNG` alert and then counts as a compromise for `DAA_TRAP_ACT`, so with the default `DAA_TRAP_ACT=0` this is **alert-only**. Set 0 to disable. |
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
`DAA_LKAHD_M` the same way and keep it at least a few times the turn radius (the
startup check warns below 3×R); a very long look-ahead (10×+) can over-commit
the far-field path. In wind, `DAA_WIND_MARG` widens the fence standoff
automatically.

The look-ahead also explains why an `AVOIDING:` distance can be much larger than
the obstacle's standoff. The bendy-ruler projects your path forward up to
`DAA_DETECT_M` (e.g. 1000 m) and starts deviating as soon as that projected path
would pass within the standoff of the obstacle (for a drone, `AVD_UAV_XY +
DAA_MARGIN_UAV`; for a fence, `DAA_MARGIN_FENCE`). So with an obstacle sitting on
or near your path, the aircraft begins avoiding — and the `AVOIDING:` message
reports the true range — while the obstacle is still hundreds of metres ahead.
The standoff is the _lateral_ clearance held around the obstacle, not the range
at which avoidance begins.

### Where the commanded avoidance target is placed

While avoiding, the applet replaces the vehicle's `next_WP_loc` with a target along the
chosen bearing, at `max(distance_to_target, DAA_PLAN_M)`.

Until 4.8.0-080 that floor was `DAA_LKAHD`, so it could not be set independently of how
far the sweep probed — one parameter was doing three jobs (probe length, detection
horizon and this). They are now `DAA_LKAHD_M`, `DAA_DETECT_M` and `DAA_PLAN_M`, all
defaulting to 1000 m so the split changes nothing on its own.

**Do not shorten `DAA_PLAN_M` casually.**

ArduPlane's past-the-waypoint test draws its finish line _through_ `next_WP_loc`. A
target a kilometre out puts that line out of reach. A near one puts it alongside the
aircraft — and the moment the avoidance bearing has any component back towards the
previous waypoint, the mission completes and moves on. It also costs clearance: replacing
the floor with `max(WP_LOITER_RAD, 2 × WP_RADIUS)` was tried, and `PlaneDAAHungTrapFires`
skipped its waypoint at 106 m and finished **7 m** off the fence instead of clearing it.

Anything that shortens this has to beat two conditions, not one:

- clear the waypoint acceptance distance (`WP_RADIUS` scaled by EAS2TAS², see
  `AP_L1_Control::turn_distance`), or the target reads as _Reached waypoint_;
- stay beyond the projection of the _previous_ waypoint onto the avoidance bearing, or
  the target reads as _Passed waypoint_.

The visible cost of leaving it long is a wide excursion away from the route before the
aircraft comes back to it, most pronounced at the 1000 m default on a short leg.

### Avoidance can skip a waypoint, and says so

Even with a long target, the finish line drawn through a laterally offset avoidance
target can still be crossed early — observed at 100–177 m short on a real flight,
reported by the vehicle as `Passed waypoint #n dist 140m`.

**This is the correct outcome**: containment beats mission fidelity, and the aircraft
should not fly back through a fence to tick off a waypoint. It is inherent to steering by
target replacement and cannot be tuned away.

What it should not be is silent, so the applet announces it:

```text
pDAA WP3 skipped by 140m while avoiding
```

Sent when the mission index moves **forward** while an avoidance target is commanded and
the aircraft is further from the waypoint it just left than `WP_RADIUS`. A backwards move
is never reported — a GCS can `DO_JUMP` the mission while avoidance happens to be running,
and that is not the applet's doing.

### Why a candidate heading is judged from where the turn ends

A candidate heading is not flown from where the aircraft is now. Getting onto it
costs a turn, and the arc of that turn carries the aircraft up to twice the turn
radius towards whatever lies on the inside of it. At 25 m/s with 60° of roll the
radius is about 35 m, so a reversal displaces roughly 70 m sideways — more than a
typical fence standoff.

So each candidate is probed from the position the aircraft will actually occupy
once it has turned onto that course, and the chord covering the turn itself is
probed as well as the straight leg that follows it. Without this a heading that
needs a large deflection reads as perfectly clear and then flies the aircraft
through the fence while it is still turning onto it — with the roll on the stop
the whole way, which is what makes it look in a log like a controller fault
rather than a planning one.

Two consequences worth knowing:

- Large-deflection headings are now **penalised**, so the ruler prefers a smaller
  deflection that it can actually reach. Near a fence this reads as the aircraft
  committing earlier and turning less.
- The turn cost scales with `ROLL_LIMIT_DEG` and airspeed. A low roll limit makes
  every deflection more expensive and pushes the ruler towards shallower
  avoidance started further out, which is the correct trade — but it needs the
  detection horizon (`DAA_DETECT_M`) to be long enough to see the obstacle by then.

### Moving obstacles and closest-approach

Moving obstacles (drones, AIS) are gated by a closest-point-of-approach
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

A worked case: flight `log_87` (2026-08-27) ran `AVD_UAV_XY 75` against
`WP_LOITER_RAD 90` and duly raised `tight drone avoid` at startup. Against the one
real drone that was enough — the protected volume was never entered — but the
warning is worth heeding rather than dismissing.

Add reaction distance on top of the turn radius: closing speed times the time it
takes to acquire, decide and roll in. For a head-on encounter that closing speed is
the sum of both groundspeeds, so a standoff sized only for the turn radius is
already too small.

`DAA_SLEW_DPS` must be **below the turn rate the airframe can actually fly**,
`ω = deg(g · tan φ / v)` for roll limit `φ` (`ROLL_LIMIT_DEG`) at speed `v` — about
31 °/s at 45° and 18 m/s — or the limiter never binds and the smoothing does
nothing. Note this is the **max-bank** rate, not the gentler loiter rate
`deg(AIRSPEED_CRUISE / WP_LOITER_RAD)`; using the loiter rate under-reads the real
figure roughly threefold and makes a perfectly effective slew limit look inert.
The startup check only warns above `1.5 ×` the max-bank rate, so a value between
the achievable rate and 1.5× is silently inert — check it by hand. A limit that
does bind is still safe for urgent manoeuvres: it is bypassed when the estimated
time-to-conflict falls below `DAA_SLEW_URG`, and also whenever slewing at
`DAA_SLEW_DPS` could not complete the required change before the conflict.

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

If the vehicle stays compromised — a fence breach, a crewed aircraft inside the
near-miss volume, or a hung avoidance — continuously for `DAA_TRAP_S` seconds,
`DAA_TRAP_ACT` fires a failsafe mode. The recommended actions have zero turn radius so they get
the aircraft out of a tight space without needing room to turn: `QLOITER` (stop
and hover), `QRTL` (VTOL return) or `QLAND`. On a non-VTOL airframe (`Q_ENABLE=0`)
these fall back to `RTL`. Set `DAA_TRAP_ACT=0` to disable the failsafe entirely
(avoidance just keeps trying). The trap fires on a sustained fence breach, an
aircraft near-miss, **or** a hung avoidance. Because the autopilot's own
`FENCE_ACTION` (if non-zero) acts
first on a fence breach in every mode, it pre-empts the trap's fence case — leaving
the trap as the nav-mode backstop for aircraft (and for fences too when
`FENCE_ACTION=0`). The fence case additionally **stands down entirely** when
`FENCE_ACTION` is non-zero _and_ `FENCE_OPTIONS` bit 0 (`DISABLE_MODE_CHANGE`) is
set, because the core would refuse the trap's mode change anyway; the startup check
reports this. The **hung** case is deliberately not stood down by that pairing: a hung
vehicle never breaches, so the core fence library never acts and there is nothing to
defer to — it is precisely the situation in which everything else has already declined
to help. Still worth confirming against the startup messages rather than assuming the
fence branch is armed.

`DAA_HUNG_ALRT_S` measures a lack of _progress_, not a lack of headings. Detecting that
the bendy ruler is genuinely boxed in — no clear heading exists at all — is a separate
condition and is **not implemented**; a boxed-in vehicle is caught only once it stops
making progress, or breaches something.

Recovery depends on what caused the trap:

- **Fixed obstacle (fence)** — sticky: held until the pilot changes mode, because
  a fence will not move out of the way.
- **Moving obstacle (drone/aircraft)** — auto-recovers to the previous mode after
  `DAA_TRAP_CLR_S`, on the assumption it has passed. If it has not, resuming
  forward flight simply re-triggers the failsafe.
- **Hung avoidance** — released when the mission moves on: the pilot advancing it, or
  a new mission. Progress toward the target cannot be the release test, because the
  trap's own action retargets the vehicle (RTL heads for home, which reads as instant
  progress) and would drop it straight back into the same stall. A pilot mode change
  releases it as well.

If `DAA_TRAP_ACT` would command the mode the aircraft is _already_ in (e.g.
trapped mid-`RTL` with `DAA_TRAP_ACT=RTL`), commanding it again would do nothing,
so it escalates to `DAA_TRAP_ESC_ACT` (default `QRTL`) to actually stop the
aircraft. Set `DAA_TRAP_ESC_ACT` equal to `DAA_TRAP_ACT` to disable escalation.

## Files, and what belongs where

The applet is split so that **`planedaa.lua` holds avoidance _policy_ and the modules hold
_mechanism_**. If you need behaviour these parameters do not cover — a different alert, a
different action, your own failsafe — you should only have to edit `planedaa.lua`.

| file | goes in `scripts/` or `scripts/modules/` | holds |
|---|---|---|
| `planedaa.lua` | `scripts/` | every parameter, all alerting, all commanding (target hijack, loiter, altitude clamp), the trapped/hung failsafe, the skipped-waypoint notice |
| `daageo.lua` | `scripts/modules/` | `DAAgeometry` — angles, locations, turn radius and rate, wind-corrected ground speed |
| `daacore.lua` | `scripts/modules/` | `DAAcore` — the avoidance **mechanism**: the bendy-ruler sweep and its turn lead, closest-point-of-approach assessment, aircraft and altitude-fence detection, and the `DAAD`/`DAAS`/`DAAG` logging |
| `daaobs.lua` | `scripts/modules/` | `DAAobstacles` — the `OBSTACLE_TYPE` taxonomy, obstacle labelling, standoffs, `find_closest_obstacle` |
| `daaltr.lua` | `scripts/modules/` | `DAAloiter` — one implementation of the altitude-loiter policy (see below) |
| `mavlink_wrappers.lua` | `scripts/modules/` | MAVLink command helpers |

**All four must be installed.** A missing module is a load failure, not a degraded mode.
`@Param` documentation has to stay in `planedaa.lua`: the parameter metadata tool only
scans `libraries/AP_Scripting/applets` and `drivers`, so a `@Param` block moved into a
module silently disappears from the documentation.

### Replacing a policy: the loiter as the pattern

`daaltr.lua` is deliberately not privileged. The applet talks to it through five members
and nothing else:

| member | |
|---|---|
| `.active` | true while the loiter is running |
| `start(alt_m, frame, right, speed)` | begin; returns true if a loiter is running after the call |
| `stop(force)` | end it; returns false if it declined (the cool-down is still running) |
| `update()` | call regularly while active; notices the pilot leaving GUIDED |
| `aircraft_seen()` | refresh the cool-down timer |

So a different policy is a different module providing those five. Write `daaltr2.lua`,
change one line in `planedaa.lua`:

```lua
loiteralt = need("daaltr2").new({ ... })
```

and nothing else moves. Your module decides what "loiter" means — orbit the other way,
descend instead of hold, refuse below a height, hand back to a different mode — while the
applet keeps deciding _when_ a loiter is called for. That is the seam: **the applet owns
which policy applies and when; the module owns how it is carried out.**

The same shape is available for the mechanism. `daacore.lua` is reached through
`configure()`, `update_state()`, `detect()`, `clamp_alt_to_fence()` and
`assess_obstacle_motion()`, and `detect()` hands back a report rather than setting shared
state:

```lua
{ target_loc, obstacle, aircraft }
```

Replacing that is a much larger undertaking than replacing a loiter, but the seam is the
same one.

### Distinct names are a budget

Lua's parser keeps one table of **every distinct identifier, string literal and number in a
chunk**, and it is sized in powers of two. Crossing **1024 entries** doubles it from 32 KB
to 64 KB _while parsing_, and because the collector does not run inside `luaL_loadfile`
that lands straight on the peak. A large script that fits comfortably once loaded can fail
to load with `Insufficent memory`.

This is not hypothetical: v4.8.0-078 sat at **1020 entries — four names from the wall**.
Adding two parameters took it to 1029 and it stopped loading.

Things worth knowing before adding to this applet:

- It is the **count** of distinct names, not their length. Shortening names does nothing;
  reusing a name that already exists is free.
- Comments are free — the lexer discards them without buffering. Do not trim comments for
  memory; it was measured and recovers nothing.
- Every parameter costs **two** entries: the global (`DAA_MARGIN_FENCE`) and the string
  passed to `bind_add_param` (`'MARGIN_FENCE'`).
- **Splitting into a module is the only real lever** — each chunk gets its own 1024.

Current occupancy: `planedaa.lua` 607, `daacore.lua` 383, `daaobs.lua` 171, `daaltr.lua` 102,
`daageo.lua` 78.

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
| `DstF` | Clearance of the **worst** heading in the sweep (m) — the closest any candidate came to an obstacle, which is what names the obstacle being avoided. It is normal for this to be very negative: some candidate heading usually points straight at the obstacle. **It is not the clearance of the path being flown** — read `DstB` for that. |
| `DstB` | Clearance of the heading actually **chosen**, `HdgB` (m). Negative means no heading cleared and the best available one still breaches — the genuine boxed-in signal. Clamped to ±9999; a heading that clears everything reports the clamp. For a fence this is exact. For a moving obstacle it is the clearance at the point the clearance-hysteresis check ran; the side-commitment and slew-rate smoothing that can still adjust `HdgB` afterwards do not re-measure, so treat it as the best available figure rather than an exact one in that case. |
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
| `Age` | Age of the obstacle's reported position (s); 0 means on-board/fresh |
| `TrR` | Turn radius available at `ROLL_LIMIT_DEG` and the current airspeed (m); 0 when there is no usable airspeed |

`TrR` records the turn radius the standoff sizing assumes at that moment,
`R = V² / (g · tan φ)`. Compare it against the radius actually flown over the same
window — airspeed divided by turn rate, or `ATT.Roll` — to check whether the aircraft
achieves the modelled performance. That comparison is what decides whether a standoff
sized on the roll limit is enough, or whether the wider `WP_LOITER_RAD` is warranted.

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
| 3  | FENCE_HOME | All fixed/unmovable fences |
| 4  | FENCE_CIRCLE_INCLUSION | Circular inclusion fence |
| 5  | FENCE_CIRCLE_EXCLUSION | Circular exclusion fence |
| 6  | FENCE_POLYGON_INCLUSION | Polygon inclusion fence |
| 7  | FENCE_POLYGON_EXCLUSION | Polygon exclusion fence |
| 8  | FENCE_LUA | Fence defined in Lua |
| 9  | PROXIMITY | Detected by a proximity sensor, typically close (not active by default) |
| 10 | AIS | AIS-tracked maritime (ship) vehicle (not active by default) |
| 11 | FENCE_ALT_MAX | Max altitude fence (FENCE_TYPE bit 0) |
| 12 | FENCE_ALT_MIN | Min altitude fence (FENCE_TYPE bit 3) |

## Proximity and AIS obstacles

`PROXIMITY` and `AIS` obstacles are **not active in a default build**, and neither
`DAA_MARGIN_PRX` nor `DAA_MARGIN_AIS` has any effect unless the whole path is enabled.

Those two obstacle types reach the script from `AP_OADatabase`, and on Plane that database
does not exist: it is a member of `AP_OAPathPlanner`, which only Copter and Rover
instantiate. `AP::oadatabase()` is therefore null on Plane and the query returns
immediately. AIS has a second, independent blocker — `AP_AIS` compiles to dummy methods on
anything that is not Rover (`AP_AIS_ENABLED == 2`), so nothing decodes vessels in the first
place.

The query is compiled out by default so it costs no flash. Build it in with:

```bash
./waf configure --board <board> --enable-AP_OASCRIPTING --enable-AP_OASCRIPTING_OADB
```

That flag alone is **not sufficient to make the feature work** on Plane — it only restores
the query. Making proximity or AIS obstacles actually reach the script still needs:

1. something on the vehicle that owns an `AP_OADatabase`, calls `init()` on it and pumps
   `process_queue()` / `update()` (roughly 7 KB of RAM at the default sizes, plus the
   `OA_DB_*` parameters);
2. for AIS, a non-dummy `AP_AIS` build on Plane;
3. `OA_DB_EXPIRE` raised well above its 10 s default — AIS is a low-rate source and an
   anchored vessel need only report every 3 minutes, so a short timeout makes contacts
   flicker in and out.

`AP_OADatabase` also has no lock around its item array, so a vehicle reading it from the
scripting thread while its avoidance thread mutates it would need that fixed first.
