# Tilt Servo Failure Detection

[ServoFailDetect.lua](ServoFailDetect.lua) monitors two tilt servos on a
QuadPlane using servo position telemetry. It compares the reported angles
with a slew limited model of the PWM outputs and reports stuck or
disconnected servos. When one servo has failed and the other is healthy,
it can stop the motor attached to the failed servo.

## Installation and setup

1. Enable scripting with `SCR_ENABLE = 1` and reboot.
2. Edit the `servo_config` table near the top of `ServoFailDetect.lua` for
   your aircraft, as described below. Set the motor channel in both rows to
   `nil` so the applet only monitors the servos during the test flights.
3. Copy the configured script to the autopilot's `APM/scripts` directory,
   then restart scripting or reboot. Repeat this after changing the table.
4. With propellers removed, check the reported channel mapping, move both
   servos through their range and verify the expected positions in the logs.
   Test each failure separately and both together before using motor shutdown.
5. Fly several normal flights with both motor channels still set to `nil`.
   After each flight, review the servo warnings and the `SCHK` log for both
   servos. Investigate any false stuck or disconnected reports and repeat
   the monitor-only flights after correcting the cause.
6. Once several flights show no false detections, enter the corresponding
   motor output channel in each row and reload the script as in step 3.
   With propellers removed, verify that each simulated servo failure stops
   only the motor on that side before flying with motor shutdown enabled.

The script runs whenever it is loaded and creates no parameters. To stop
motor shutdown while retaining monitoring, set both motor channels in the
table to `nil`. To disable the script entirely, remove it from `APM/scripts`
and restart scripting or reboot.

This applet requires CAN servo position telemetry through `servo_telem`.
The applet uses zero-based telemetry slots. For example, outputs 15 and 16
must report in slots 14 and 15. The Himark backend in Plane 4.7.3
subtracts one from an already zero-based servo ID. Himark servos require
firmware with the [Himark indexing fix](https://github.com/ArduPilot/ardupilot/commit/e204ca77a8012342b52af17beb78adaca598113c).

`Q_M_PWM_MIN` must be the correct motor shutdown PWM.

While disarmed, the applet checks both telemetry slots, including with the
safety switch engaged or motor channels set to `nil`. Arming requires a
finite position from each slot with a telemetry timestamp at most one second
old. A pre-arm failure identifies the servo output and missing or stale slot.
The check clears when both slots have valid feedback. Verify the physical
left/right mapping during setup since another actuator can occupy a slot.

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
| Servo | 1-based SERVO output channel (1–32). |
| Name | Servo name used in status messages. |
| Motor | 1-based SERVO output channel (1–32), or `nil` to report failures without stopping this motor. |
| Angle at MIN | Reported servo angle in degrees at this servo's `SERVOx_MIN`. |
| Angle at MAX | Reported servo angle in degrees at this servo's `SERVOx_MAX`. |

Calibrate the angle endpoints for each servo separately. They describe raw
servo feedback, rather than aircraft tilt angles. `SERVOx_REVERSED` is
already reflected in the commanded PWM and does not swap these calibration
values. Decreasing feedback with increasing PWM is supported; each row's
two angles must differ.

Each row pairs a tilt servo output with the motor output on the same side.
`Q_TILT_MASK` must select exactly two logical motors. Each configured motor
output must use a different motor function selected by that mask. All configured
output channels must be distinct.

The supplied table pairs servo 15 with motor output 11 and servo 16 with
motor output 13. If outputs 11 and 13 are Motor3 and Motor5, use
`Q_TILT_MASK = 20`.

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

The applet resets its model and failure timers at startup, on arming or
disarming, and after a scheduling gap exceeding 100 ms. It also holds them
in reset while disarmed with safety engaged. It allows a full servo-range
travel time before checking position
errors. Telemetry recovery also restarts this settling period unless the servo
had already failed. A confirmed failure is retained through telemetry recovery
until fresh telemetry shows the servo within the position error limit.
Before issuing a shutdown, the applet checks the time spent in the update,
including the shutdown PWM lookup. If this exceeds 100 ms or the previous
override has reached its 120 ms renewal deadline, the applet releases the
override and restarts detection. Repeated delays keep overrides suspended
until timely updates allow both servos to be assessed again.

Motor shutdown is allowed only while armed, in VTOL modes or assisted
forward flight after an observed completed forward transition. It is inhibited during the forward
transition and unassisted forward flight. After a scheduling gap, the
transition history is treated as unknown until VTOL mode or unassisted forward
flight is observed again.

While disarmed, the applet monitors servo movement when the safety switch is
off. Once armed, it assumes the servos are active regardless of the reported
safety state. This permits detection on stock Plane 4.7 on boards without a
safety switch. Re-engaging safety while armed leaves detection active.

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
Position recovery, disarming, or leaving an eligible flight
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
If setup fails, the pre-arm message says `SRVFAIL: setup incomplete; see Messages`.
The Lua error in the ground station's messages identifies the configuration problem.

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
telemetry loss, mode and safety gating, calibration, channel validation and
timer rollover. Each case runs many simulated updates in one callback, so
this test uses a higher instruction budget.

`ServoFailDetect` uses the real telemetry and motor-output bindings with the
normal instruction budget. It checks that shifted telemetry slots prevent
arming and that correcting the slots allows arming. On the armed, stationary
QuadPlane it checks individual and simultaneous failures,
telemetry loss and recovery, override expiry while callbacks are paused, and
resumption without a shutdown pulse. The tests do not validate aircraft
controllability after a servo failure or replace calibration and testing on
the aircraft.
