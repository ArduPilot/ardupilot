# Tilt Servo Failure Detection

[ServoFailDetect.lua](ServoFailDetect.lua) monitors two tilt servos on a
QuadPlane using servo position telemetry. It compares the reported angles
with a slew limited model of the PWM outputs and reports stuck or
disconnected servos. When one servo has failed and the other is healthy,
it can stop the motor attached to the failed servo.

## Installation and setup

1. Enable scripting with `SCR_ENABLE = 1` and reboot.
2. Edit the `servo_config` table near the top of `ServoFailDetect.lua` for
   your aircraft, as described below.
3. Copy the configured script to the autopilot's `APM/scripts` directory,
   then restart scripting or reboot. Repeat this after changing the table.
4. With propellers removed, check the reported channel mapping, move both
   servos through their range and verify the expected positions in the logs.
   Test each failure separately and both together before using motor shutdown.

The script runs whenever it is loaded and creates no parameters. To stop
motor shutdown while retaining monitoring, set both motor channels in the
table to `nil`. To disable the script entirely, remove it from `APM/scripts`
and restart scripting or reboot.

This applet requires CAN servo position telemetry through `servo_telem`.
Telemetry must be indexed by the corresponding SERVO output channel.
`Q_M_PWM_MIN` must be the correct motor shutdown PWM.

## Servo configuration table

The table must contain exactly two rows: left first, then right. The supplied
settings are:

```lua
local servo_config = {
    -- Servo   Name          Motor   Angle at MIN   Angle at MAX
    {  15,     "Left Tilt",    11,        -57.5,          57.5 },
    {  16,     "Right Tilt",   13,        -57.5,          57.5 },
}
```

| Column | Meaning |
| -------- | --------- |
| Servo | 1-based SERVO output channel (1–32), or `0` for automatic discovery. |
| Name | Servo name used in status messages. |
| Motor | 1-based SERVO output channel (1–32), `0` for automatic discovery, or `nil` to report failures without stopping this motor. |
| Angle at MIN | Reported servo angle in degrees at this servo's `SERVOx_MIN`. |
| Angle at MAX | Reported servo angle in degrees at this servo's `SERVOx_MAX`. |

Calibrate the angle endpoints for each servo separately. They describe raw
servo feedback, rather than aircraft tilt angles. `SERVOx_REVERSED` is
already reflected in the commanded PWM and does not swap these calibration
values. Decreasing feedback with increasing PWM is supported; each row's
two angles must differ.

`Q_TILT_MASK` must select exactly two motors. It contains logical motor
numbers, not output channel numbers or tilt servo channels. With `0` in a
motor column, the script takes the two set bits in ascending motor-number
order, assumes left then right, and finds the outputs with the corresponding
Motor functions. With `0` in a servo column, it finds function 75
(TiltMotorLeft) for the first row and 76 (TiltMotorRight) for the second.
Use explicit channels if the motor order is different or the servo outputs
use other tilt functions.

The supplied table is for servos on outputs 15/16 and motors on outputs
11/13. If these are Motor3/Motor5, use `Q_TILT_MASK = 20`. If outputs 15/16
also have functions 75/76, all four channel entries can instead be `0`.

Invalid or ambiguous mappings stop the script with an error. Configured
output channels must be distinct, and each configured motor must correspond
to a different motor selected by `Q_TILT_MASK`.

## Telemetry limitation

The CAN servos this script is intended for do not produce circuit-status
messages. The script assumes that each telemetry update includes a fresh
measured position.

The serial Volz backend can republish cached position data for up to five
seconds after the last servo response, so it cannot provide the position
freshness guarantee this applet requires.

The current `servo_telem` API has one timestamp for all telemetry fields.
Circuit-status messages can refresh that timestamp with voltage/current
updates while leaving an old measured position in place. With such a servo,
loss of position feedback could go undetected, a healthy servo could be
reported stuck, or the other servo could incorrectly remain marked healthy.
The script does not distinguish these updates. This limitation is accepted
for the intended servos; using servos or backends that refresh telemetry
without a fresh position requires a separate position-freshness mechanism.

## Detection and motor shutdown

The applet requests updates at 50 Hz. The model slews at 500 microseconds of
PWM per second, using the actual time between updates. A position error
larger than 14% of the calibrated angle range must persist for more than two
seconds, with new telemetry samples, before a servo is marked stuck.

Telemetry is invalid if it is absent, older than one second, or has no finite
measured position. A continuous further two seconds of invalid telemetry
marks the servo as disconnected for motor shutdown. Missing telemetry at
startup is treated the same way. A disconnected warning is sent immediately
when previously valid telemetry becomes invalid.

At startup, after safety activation, after arming or disarming, or after a
scheduling gap exceeding 100 ms, the applet resets its model and failure
timers. It allows a full servo-range travel time before checking position
errors. Telemetry recovery also restarts this settling period unless the servo
had already failed. A confirmed failure is retained through telemetry recovery
until fresh telemetry shows the servo within the position error limit. A stall
within an update prevents that update from issuing a motor shutdown and forces
detection to restart.

Motor shutdown is allowed only while armed and either no safety switch is
available or the safety is off, in VTOL modes or assisted forward flight after
an observed completed forward transition. It is inhibited during the forward
transition and unassisted forward flight. After a scheduling gap, the
transition history is treated as unknown until VTOL mode or unassisted forward
flight is observed again.

Both servo results are checked before acting. If both servos have failed,
or the other servo is still settling or has a pending position error,
neither motor is stopped. Any existing override is released. A change of
failed side releases the previous override and waits before stopping the
other motor, so overrides cannot overlap.

A motor is stopped by overriding its output to `Q_M_PWM_MIN` for 120 ms. This
spans the accepted scheduling-gap window so a late callback cannot briefly
restore the commanded motor output before renewing the override. The applet
renews the override while the failure and shutdown conditions persist. The
main vehicle loop expires the override if scripting stops running.
Position recovery, disarming, safety activation, or leaving an eligible flight
mode releases it. Releasing an override preserves the current PWM, including
when a scheduling delay has already allowed the override to expire.

This applet is intended for aircraft that can remain controllable with one
of these motors stopped. It does not command a landing or change flight mode.
Poor scheduling deliberately suspends failure protection while detection
recovers; it cannot provide continuous protection when scripting is stalled.

## Messages and logging

On initialization, `SRVFAIL: <name> servo <channel> motor <channel>` reports
each row's selected output channels. A motor shown as `none` means that row
only reports failures. Failure messages use the names from the table.

`SCHK` logs each servo's zero-based channel, expected feedback angle,
telemetry connection status, and state: 0 inactive/settling, 1 active,
2 stuck, 3 sustained telemetry loss. A pending position error can still have
state 1, but is not considered healthy for the other motor's shutdown.
`SFD` logs whether the flight mode permits stopping a motor, VTOL/assisted
flight status, and `Dt`, the elapsed time between updates in milliseconds.

## Automated tests

The Plane CI workflow runs both tests in SITL using ArduPilot's Lua
interpreter. With an existing SITL Plane build, run them from the repository
root with:

```sh
Tools/autotest/autotest.py test.QuadPlane.ServoFailDetectRegression test.QuadPlane.ServoFailDetect
```

`ServoFailDetectRegression` runs deterministic cases with simulated time,
telemetry and motor outputs, using the firmware's native `uint32_t` timestamps
and number representation. It covers scheduling gaps, fault combinations,
telemetry loss, mode and safety gating, calibration, channel discovery and
timer rollover. Each case runs many simulated updates in one callback, so
this test uses a higher instruction budget.

`ServoFailDetect` uses the real telemetry and motor-output bindings with the
normal instruction budget. It injects DroneCAN position feedback into an
armed, stationary QuadPlane and checks individual and simultaneous failures,
telemetry loss and recovery, override expiry while callbacks are paused, and
resumption without a shutdown pulse. The tests do not validate aircraft
controllability after a servo failure or replace calibration and testing on
the aircraft.
