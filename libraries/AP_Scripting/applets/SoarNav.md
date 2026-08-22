# SoarNav

SoarNav is an advanced Lua script for intelligent, autonomous soaring
that enhances ArduPilot's capabilities. It uses a suite of strategic
features to maximize flight time within a defined area.

The script employs a hybrid exploration strategy, dynamically balancing
the search for new grid cells with guided re-visiting of the least
explored areas based on recent thermal success. This is coupled with an
intelligent thermal memory that saves thermal core locations. A
sophisticated clustering algorithm then identifies promising hot zones
using a weighted score based on nearby thermals' strength, age, and
proximity.

Real-time thermal analysis is performed using adaptive sampling for a
responsive strength calculation. The script is architected for robust
performance with a modular state machine and parameter caching. Safety
is enhanced with dynamic anti-stuck logic, tactical rerouting, and
thermal street detection capabilities.

---
# Key Features

## Hybrid Exploration Strategy (Virtual Grid)
Employs a dual-mode grid search, dynamically alternating between
**Guided Exploration** (re-visiting the least-explored cells) and
**Pure Exploration** (seeking new, unvisited cells) based on recent
thermal success.

## Efficient & Optimized
The script is optimized to reduce CPU load by leveraging a **modular
state machine**, parameter caching, native ArduPilot APIs, and efficient
pre-calculation of grid cell centers.

## Advanced Thermal Analysis (Strength & Quality)
Utilizes **Adaptive Sampling** (a weighted EMA based on climb rate) to
rapidly and accurately calculate a thermal's average strength, peak strength,
and **consistency** (steady vs. variable lift).

## Intelligent Thermal Memory (Core, Clustering & Focus Mode)
Saves the **thermal core's** location (strongest lift) for greater accuracy.
A sophisticated **clustering** algorithm identifies "hot zones" using a
weighted score of nearby thermals' strength, age, and proximity. Can engage a
**Focus Mode** to search high-density areas.

## Energy-Aware & Dual-Strategy Decision Making
Uses a **time-corrected low-pass filter** on normalized altitude to
determine energy state, ensuring smooth and stable transitions. At low energy,
it deterministically navigates to the strongest known thermal. At normal
energy, it probabilistically balances exploring new areas with revisiting
known hotspots.

## Thermal Streeting (Pattern Recognition)
Attempts to identify **"thermal streets"**—lines of lift aligned with the
wind—and proactively navigates along their projected axis to find the
next thermal in the chain.

## Dynamic Thermal Drift Prediction
Uses the current wind vector and a hotspot's age to predict its new,
drifted position, increasing the chances of a successful intercept.

## Terrain-Aware Dynamic Altitude (Glide Cone)
Integrates with ArduPilot's terrain database to ensure safe flight in
mountainous areas. The dynamic altitude logic (Glide Cone) will
automatically raise the `SOAR_ALT_MIN` floor to maintain a safe
clearance above rising terrain, using the initial `SOAR_ALT_MIN` value
as the desired safety margin. This prevents the aircraft from flying
below a safe altitude relative to the ground, while still allowing it
to use the full soaring envelope over valleys.

## Ridge Lift Navigation (Terrain-Aware)
This feature exploits orographic (ridge) lift by flying a
**parallel-to-crest**, **upwind-offset** track when the wind faces
the slope. It is attempted when generating a new waypoint
**also in ENERGY = NORMAL** (not only LOW/CRITICAL), **before**
Thermal Street.
It works by using **Terrain** (enabled) to estimate the local slope
direction (uphill) and the **wind-from** vector, computing a ridge
score based on windward alignment, slope steepness, and wind strength.
Under a strict time budget (≤ ~4 ms, ≤ 8 cells), the best ridge
cell is selected. The target is then built by offsetting **upwind**
and nudging **along the crest** to stay in lift while keeping clearance.
The **upwind standoff** distance equals **`SNAV_WP_RADIUS`**,
and the **along-ridge** advance scales as **≈ `1.5 × SNAV_WP_RADIUS`**
(clamped 60–300 m).
The feature **requires Terrain enabled** and automatically defers
if Terrain is unavailable, wind is too weak, or the ridge score
is below threshold.
If the ridge option cannot produce a valid target, SoarNav falls back
to the next strategy in the normal selection order.

## Safety & Tactical Navigation
Includes pre-flight parameter validation, robust and **dynamic
anti-stuck logic**that increases repositioning distance and varies
the upwind angle on subsequent attempts, and a tactical mid-flight
re-route.

## Terrain Evasion (Terrain Avoidance)
When Terrain data is available, SoarNav continuously checks the predicted
clearance ahead (AGL) over a configurable lookahead time. If the projected
path falls below the safety buffer, it temporarily **overrides navigation**
and commands an evasive turn until a safe corridor is restored, then transitions
through an egress phase and resumes normal targets.
This is controlled by `SNAV_TE_LOOK_S` (lookahead sensitivity) and
`SNAV_TE_BUF_MIN` (minimum AGL buffer).

## Adaptive Waypoint Timeout
Intelligently adjusts the time allowed to reach a waypoint based on
current wind conditions, allowing more time in strong headwinds.

## Intuitive Pilot Override & Stick Gestures
Allows temporary override via **pitch or yaw** stick input. A persistent
override is toggled by a **roll gesture**. Re-centering of the
search area is done using a **pitch gesture**. The script requires an initial
**roll gesture** to start autonomous navigation.

## Flexible Area System (Radius, Rally Point, Polygon)
Defines the operational area using a three-tiered priority system: a
circular **Radius** (highest priority), the flight controller's
**Rally Points**, or a **Polygon file** from the SD card.
Priority is: **Radius > Rally Points > File**.

## Advanced Logging System
A multi-level logging system provides clear operational feedback, from
key events to detailed real-time status for debugging.

-------------------------------------------------------------------

# Parameters
The script adds the following `SNAV_*` parameters to control its
behavior.

## SNAV_ENABLE
This parameter serves as the master switch for the script. A value of 0
disables it completely, while any value greater than 0 enables it.
When using Polygon File Mode (i.e., `SNAV_RADIUS_M` is set to 0 and
fewer than 3 Rally Points are present), this parameter's value also
selects which polygon file to use (e.g., 1 for 'snav1.poly', 2 for
'snav2.poly', etc., up to 9). If the selected file (e.g., `snav3.poly`) 
is not found, the script will attempt to load other files as a fallback, 
starting from `snav1.poly` upwards, until it finds a valid polygon file.

## SNAV_LOG_LVL
Sets the verbosity level of messages sent to the Ground Control Station.
- **0 (Silent):** Only critical script errors are reported.
- **1 (Events):** [Default] Reports key events like script start/stop,
  waypoint reached, new waypoint generation, and thermal memory recording.

## SNAV_RADIUS_M
Defines the radius in meters of the circular flight area. The center
is Home by default but can be re-centered in-flight with a **pitch
gesture**. If this parameter is set to `0`, the script will check for
Rally Points and then for a polygon file.

**Note on Area Mode Selection**
The script automatically selects the area mode based on the following
priority:
- **1. Radius Mode (Highest Priority):** If `SNAV_RADIUS_M` is set
to a value greater than 0, this mode is always active.
- **2. Rally Point Mode:** If `SNAV_RADIUS_M` is 0 and the flight
controller has **3 or more** Rally Points loaded, the script will
automatically use them to define the flight area.
- **3. Polygon File Mode (Fallback):** If `SNAV_RADIUS_M` is 0 and
there are **fewer than 3** Rally Points, the script will fall back
to loading a `snavX.poly` file from the SD card, selected by the
`SNAV_ENABLE` parameter.

## SNAV_ROLL_LIMIT
The maximum roll (bank) angle, in degrees, that the script will
command during autonomous navigation. The default is `30` degrees.

## SNAV_WP_RADIUS
The acceptance radius in meters. Defines the distance to a waypoint
at which it is considered 'reached' and a new target is generated.
The default is `50` meters.

## SNAV_NAV_P & SNAV_NAV_D
These are the gains for the script's internal PD (Proportional-Derivative)
navigation controller, which commands roll based on heading error.
`SNAV_NAV_P` is the proportional gain; higher values result in a more
aggressive response. `SNAV_NAV_D` is the derivative gain, which helps to
dampen the response for smoother control.

## SNAV_TMEM_ENABLE
Enables (`1`) or disables (`0`) the Thermal Memory feature. When enabled,
the script will remember and return to previously found lift areas. The
probability of navigating to a known thermal versus exploring the grid
is dynamically adjusted based on recent success.

## SNAV_TMEM_LIFE
The lifetime in seconds of a 'hotspot' (a stored thermal). After
this time, the point is considered expired and is removed from memory.
The default is `1200` seconds (20 minutes).

## SNAV_STUCK_EFF
The minimum path efficiency (progress towards target / distance flown)
required to not be considered "stuck". A higher value makes the anti-stuck
logic more sensitive. Default is `0.35`.

## SNAV_STUCK_TIME
The grace period in seconds after reaching a new waypoint before the
anti-stuck logic becomes active. Default is `30` seconds.

## SNAV_REENG_DWELL
The time in seconds that the aircraft will loiter at a re-engagement
point while attempting to re-acquire a weak thermal. Default is `7` seconds.

## SNAV_RETRY_THR
A score threshold (0-100) that determines if a weak thermal is worth
retrying. The score is a combination of the thermal's weakness and the
aircraft's altitude. A lower value makes the script more aggressive in
retrying thermals. Default is `30`.

## SNAV_NEC_WEIGHT
Controls the "weight" of the necessity factor (i.e., low altitude) in the
thermal retry score. Higher values make the script more conservative,
increasing the likelihood of retrying a thermal when at low altitude.
Default is `50`.

## SNAV_STREET_TOL
The angle tolerance in degrees for detecting a thermal street. This is
the maximum allowed angle difference between the wind direction and the
line formed by multiple thermals. Default is `30` degrees.

## SNAV_REROUTE_P
The probability (0-100%) of executing a tactical reroute mid-way to a
waypoint. Higher values make the script more opportunistic and less
predictable in its exploration path. Default is `50` percent.

## SNAV_TMEM_MIN_S
The absolute minimum strength (climb rate in m/s) a thermal must have to be
saved into memory. This helps filter out very weak lift. Default is `0.2` m/s.

## SNAV_FOCUS_THR
The cluster density score a new thermal must achieve to trigger Focus Mode.
A lower value makes the script more likely to focus on areas with even
minor thermal activity. Default is `1.0`.

## SNAV_STRAT_HIST
The time window in seconds for the script's thermal success history. This
value influences the strategic balance between exploring new areas and
revisiting known ones. Default is `900` seconds (15 minutes).

## SNAV_WP_TIMEOUT
The base time in seconds that the aircraft has to reach a waypoint. This
value is automatically adapted in-flight based on wind conditions, but this
parameter sets the starting point. Default is `300` seconds (5 minutes).

## SNAV_REROUTE_MIN & SNAV_REROUTE_MAX
These define the minimum and maximum angle in degrees for a tactical
reroute maneuver. The script will choose a random new heading within this
range relative to the original path. Defaults are `80` and `100` degrees.

## SNAV_DYN_SOALT

Controls the behavior of the dynamic altitude safety feature (Glide Cone),
which can now operate with terrain awareness.

- **0 - (Disabled):** [Default] The feature is completely turned off.
- **1 - (Fully Linked):** The script modulates `SOAR_ALT_MIN` based on the
  greater of two values: the altitude required to safely glide home, or the
  altitude required to clear underlying terrain. If the required minimum
  altitude exceeds 2/3 of the initial `SOAR_ALT_CUTOFF`, both `CUTOFF` and
  `SOAR_ALT_MAX` are raised along with it to preserve the soaring envelope.
- **2 - (MIN Only, Capped):** The script only modulates `SOAR_ALT_MIN` based
  on the glide cone and terrain, and will not allow it to exceed 2/3 of the
  initial `SOAR_ALT_CUTOFF`. `CUTOFF` and `MAX` are never modified.

**Terrain Integration Note:**
For terrain awareness to function, ArduPilot's terrain following must be
enabled (`TERRAIN_ENABLE = 1`) and terrain data must be available on the
SD card. When active, the script uses your initial `SOAR_ALT_MIN` setting
as the desired safety margin above the ground. For example, if the terrain
below is 100m above home and your `SOAR_ALT_MIN` is 50m, the script will
enforce a dynamic minimum altitude of 150m AHL.

## SNAV_GC_MARGIN

The safety altitude margin in meters that is added on top of the
calculated glide altitude required to return home. Default is 25 meters.

## SNAV_GC_PAD

An extra altitude padding in meters to ensure the aircraft arrives over
the home point, not at ground level. This is added to the glide cone
calculation. Default is 20 meters.

## SNAV_TE_LOOK_S

Lookahead time (seconds) for Terrain Evasion. Higher values are more
sensitive / conservative (checks further ahead along the predicted path).

## SNAV_TE_BUF_MIN

Absolute minimum AGL buffer (meters) required by Terrain Evasion.

---
# Operation

## Installation and Setup
First, place the `SoarNav.lua` script into the `APM/SCRIPTS` directory on
the flight controller's microSD card. Then, set `SCR_ENABLE` to `1` and
reboot the flight controller. After rebooting, refresh parameters in your
ground station; the `SNAV_*` parameters should now be visible. Set
`SNAV_ENABLE` to `1` (or higher if using polygons). Finally, configure an
RC channel switch to activate the Soaring feature by setting an
`RCx_OPTION` to `88` (Soaring Active). SoarNav will only run when this
ArduPilot feature is active.

## Area Configuration
SoarNav offers three methods to define the operational flight area,
chosen automatically based on a clear priority hierarchy.

**1. Radius Mode (Highest Priority)**
Set `SNAV_RADIUS_M` to your desired search radius in meters (e.g., `500`).
The script will always use this mode if the radius is greater than zero.

**2. Rally Point Mode (Second Priority)**
If Radius Mode is disabled (`SNAV_RADIUS_M = 0`) and your flight
controller has at least three Rally Points loaded, SoarNav will
automatically use these points to create a flight polygon. This allows
for easy area definition directly from your ground station without
needing to manage files on the SD card.

**3. Polygon File Mode (Fallback)**
If both Radius and Rally Point modes are not active, the script falls
back to this mode. It requires polygon files (e.g., 'snav1.poly') in
the `APM/SCRIPTS` directory. The specific file is selected with the
`SNAV_ENABLE` parameter. The file should contain a list of GPS
coordinates (latitude longitude, space-separated) defining the
vertices of your flight area, one vertex per line.

## In-Flight Use
Take off and enter a compatible flight mode (Cruise or FBWB). Ensure
you are within the configured flight area and above the SOAR_ALT_MIN
altitude. Activate the main ArduPilot Soaring feature using the switch
you configured (RCx_OPTION = 88).
Once all pre-flight conditions are met, SoarNav enters a waiting state
and shows: `SoarNav: Awaiting pilot activation (ROLL gesture).`
To start the mission, perform a **rapid roll gesture** (L–R–L–R) on
your sticks. Upon detection, the script will begin
generating waypoints and navigating autonomously.
To stop SoarNav completely, disable the Soaring feature with your switch.

# Pilot Interaction and Stick Gestures

SoarNav is designed to be interactive, allowing the pilot to influence
its behavior without changing flight modes or disabling the feature switch.

## Temporary Override
Any input on the **pitch** or **yaw** sticks will immediately pause
SoarNav's navigation commands, giving you full manual control. The script
will automatically resume navigation a few seconds after the sticks are
returned to center.

## Persistent Manual Override (Roll Gesture)
A rapid sequence of full **roll** stick movements (e.g., R–L–R–L)
toggles a persistent manual override. Unlike temporary override,
navigation will **not** resume automatically when sticks are centered;
the pilot retains full control. Repeat the gesture to return control
to SoarNav. GCS messages confirm the state change.
_Note:_ this gesture is only active **after** the script has been
started with the initial roll gesture.

## Area Re-Centering (Pitch Gesture)
A rapid sequence of full **pitch** stick movements (e.g., U-D-U-D)
re-centers the search area to the aircraft's current location. This
works in all area modes (Radius, Rally Point, and Polygon) and is
available after the script has been activated.

## RTL→Home Override (Rally Mode)
This feature ensures the aircraft returns directly to the Home point when RTL
is activated in a Rally Point-defined area, instead of navigating to a
Rally Point first.

It works by monitoring the aircraft's progress after RTL is engaged. After
approximately 3 seconds, if the aircraft is not making sufficient progress
towards Home, the script takes control by switching to GUIDED mode and
commands the aircraft to fly directly to the Home point at the `RTL_ALTITUDE`.

Once the aircraft is within `RTL_RADIUS` of Home, the script's intervention
concludes. The aircraft will **remain in GUIDED mode**, performing a precise
loiter directly over the Home point. This prevents ArduPilot from re-engaging
its default logic, which would target a Rally Point. The override is
automatically suspended if the pilot manually changes flight modes. This
feature is enabled by default via an internal toggle.

## Online Polar Learning — When It Runs and Required Conditions
**When it runs**
- In flight while the script is active (normal navigation loop).
- Commits happen only after a valid fit **and** all commit gates are satisfied.

**Sample collection conditions**
- Motor inactive (throttle servo output below `THROTTLE_ACTIVE_THRESHOLD`).
- Not in `THERMAL` mode.
- “Clean” attitude: |roll| ≤ ~10° (AHRS roll).
- Descending airmass: NED vertical speed `z` > 0.1 m/s (sink).
- Airspeed estimate `|V_ground − V_wind|` > ~8 m/s.

**Windowing & fitting**
- Exponential decay window with ~120 s time constant.
- Airmass bias removal: sink’ = sink − EMA(sink) with ~60 s time constant.
- Fit executes only with ≥ ~500 effective samples and not more often than ~10 s.

**Update rule (shape + magnitude)**
- Let current params be `(CD0, B)` and fit give `(a, b)` for `sink ≈ a·V³ + b/V`.
- Shape correction (limits change in best-glide speed):
  - `h = sqrt((b/a) / (B/CD0))`, clamped to `[0.90, 1.10]`.
  - `CD0_shape = CD0 / h`, `B_shape = B * h`.
- Magnitude (product) correction:
  - `target_prod = 1 / (4 · eff²)` at `AIRSPEED_CRUISE`, where `eff = 1 / (a·Vc² + b/Vc²)`.
  - `s = sqrt(target_prod / (CD0_shape · B_shape))`, clamped to `[0.90, 1.15]`.
  - Proposed params: `CD0_new = CD0_shape · s`, `B_new = B_shape · s`.

**Commit gating**
- Quantization: `CD0_new` and `B_new` rounded to 0.001.
- Bounds: `B ∈ [0.005, 0.060]`, `CD0 ∈ [0.005, 0.500]`.
- Rate limit: ≥ 60 s since the last commit.
- Stability: ≥ 3 consecutive qualified fits.
- Significance: relative change `ΔCD0/CD0 ≥ 2%` **or** `ΔB/B ≥ 5%`.
- Quality gate: model error EMA ≤ ~0.3 m/s.

**What gets updated**
- On commit, the script writes:
  - `SOAR_POLAR_CD0 ← CD0_new`
  - `SOAR_POLAR_B   ← B_new`
