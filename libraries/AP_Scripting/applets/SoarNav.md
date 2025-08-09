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

# Key Features

## Hybrid Exploration Strategy (Virtual Grid)

Employs a dual-mode grid search, dynamically alternating between
__Guided Exploration__ (re-visiting the least-explored cells) and
__Pure Exploration__ (seeking new, unvisited cells) based on recent
thermal success.

## Efficient & Optimized

The script is optimized to reduce CPU load by leveraging a __modular
state machine__, parameter caching, native ArduPilot APIs, and efficient
pre-calculation of grid cell centers.

## Advanced Thermal Analysis (Strength & Quality)

Utilizes __Adaptive Sampling__ (a weighted EMA based on climb rate) to
rapidly and accurately calculate a thermal's average strength, peak strength,
and __consistency__ (steady vs. variable lift).

## Intelligent Thermal Memory (Core, Clustering & Focus Mode)

Saves the __thermal core's__ location (strongest lift) for greater accuracy.
A sophisticated __clustering__ algorithm identifies "hot zones" using a
weighted score of nearby thermals' strength, age, and proximity. Can engage a
__Focus Mode__ to search high-density areas.

## Energy-Aware & Dual-Strategy Decision Making

Uses a __time-corrected low-pass filter__ on normalized altitude to
determine energy state, ensuring smooth and stable transitions. At low energy,
it deterministically navigates to the strongest known thermal. At normal
energy, it probabilistically balances exploring new areas with revisiting
known hotspots.

## Thermal Streeting (Pattern Recognition)

Attempts to identify __"thermal streets"__—lines of lift aligned with the
wind—and proactively navigates along their projected axis to find the
next thermal in the chain.

## Dynamic Thermal Drift Prediction

Uses the current wind vector and a hotspot's age to predict its new,
drifted position, increasing the chances of a successful intercept.

## Safety & Tactical Navigation

Includes pre-flight parameter validation, robust and __dynamic anti-stuck logic__
that increases repositioning distance and varies the upwind angle on
subsequent attempts, and a tactical mid-flight re-route.

## Adaptive Waypoint Timeout

Intelligently adjusts the time allowed to reach a waypoint based on
current wind conditions, allowing more time in strong headwinds.

## Intuitive Pilot Override & Stick Gestures

Allows for temporary override via stick input, a persistent override
toggled by a roll gesture, and dynamic re-centering of the search area
using a pitch gesture.

## Dual Area System (Radius or Polygon)

Supports both a circular area and custom polygon files loaded from the
SD card, selectable via the `SNAV_ENABLE` parameter.

## Advanced Logging System

A multi-level logging system provides clear operational feedback, from
key events to detailed real-time status for debugging.

# Parameters

The script adds the following `SNAV_*` parameters to control its
behavior.

## SNAV_ENABLE

This parameter serves as the master switch for the script. A value of `0`
disables the script completely, while any value greater than `0` enables it.

When using Polygon Mode (i.e., `SNAV_MAX_DIST` is set to `0`), this
parameter's value also selects which polygon file to load from the SD
card (e.g., a value of `1` loads 'snav1.poly', `2` loads 'snav2.poly', etc.,
up to 9). If the selected file is not found, the script will
automatically attempt to load the lowest-numbered polygon available as
a safety fallback.

## SNAV_LOG_LVL

Sets the verbosity level of messages sent to the Ground Control Station.
__0 (Silent):__ Only critical script errors are reported.
__1 (Events):__ [Default] Reports key events like script start/stop,
waypoint reached, new waypoint generation, and thermal memory recording.
__2 (Detailed):__ Includes all Level 1 events plus a periodic status
report with detailed navigation, grid, and thermal memory data for
in-depth debugging.

## SNAV_MAX_DIST

Defines the radius in meters of the circular flight area. The center
is Home by default but can be re-centered in-flight with a stick
gesture. If this parameter is set to `0`, a polygon file will be used
instead.

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

You must choose one of two methods to define the operational area.

__Radius Mode:__ Set `SNAV_MAX_DIST` to your desired search radius in
meters (e.g., `500`). The script will operate within this circle.

__Polygon Mode:__ Set `SNAV_MAX_DIST` to `0`. This requires polygon files
(e.g., 'snav1.poly') in the `APM/SCRIPTS` directory. The specific file
is selected with the `SNAV_ENABLE` parameter. The file should contain
a list of GPS coordinates (latitude longitude, space-separated)
defining the vertices of your flight area, one vertex per line.

## In-Flight Use

Take off and enter a compatible flight mode (__Cruise__ or __FBWB__). Ensure
you are within the configured flight area and above the `SOAR_ALT_MIN`
altitude. Activate the main ArduPilot Soaring feature using the switch
you configured (`RCx_OPTION = 88`). When SoarNav is active, it will begin
generating waypoints and navigating autonomously. To stop SoarNav
completely, disable the Soaring feature with your switch.

# Pilot Interaction and Stick Gestures

SoarNav is designed to be interactive, allowing the pilot to influence
its behavior without changing flight modes or disabling the feature switch.

## Temporary Override

Any input on the __pitch__ or __yaw__ sticks will immediately pause
SoarNav's navigation commands, giving you full manual control. The script
will automatically resume navigation a few seconds after the sticks are
returned to center.

## Persistent Manual Override (Roll Gesture)

A rapid sequence of full __roll__ stick movements (e.g., right-left-right-left)
toggles a persistent manual override. This allows the pilot to fly freely
for an extended period without the script resuming automatically. Repeat
the gesture to deactivate the override and return control to SoarNav.
GCS messages will confirm the state change.

## Area Re-Centering (Pitch Gesture)

While in Radius Mode, a rapid sequence of full __pitch__ stick movements
(e.g., up-down-up-down) will re-center the circular search area to the
aircraft's current location. This is useful for shifting the focus of
exploration during a long flight.
