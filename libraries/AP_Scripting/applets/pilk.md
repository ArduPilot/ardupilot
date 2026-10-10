# Plane arm/emergency-stop interlock

`pilk.lua` uses a three-position action switch and a momentary
interlock switch on the transmitter for Plane, including QuadPlane. Hold the
momentary switch while changing the action switch to authorize an arm or
emergency-stop request. This guards against accidental switch movement;
it also means that an emergency stop requires operating both switches.

`pilk` stands for Plane Interlock. Keep this short filename so scripting error
messages have more room for diagnostic details, including line numbers.

## Installation and setup

1. Use firmware and an autopilot that support Lua scripting. Set `SCR_ENABLE=1`,
   copy `pilk.lua` to `APM/scripts` on the SD card, and reboot.
2. Assign a three-position action switch to an RC channel, for example
   `RC7_OPTION=306`.
3. Assign the momentary interlock switch to a different RC channel, for example
   `RC8_OPTION=307`. Configure it to return to low when released and send high
   while held (middle is also accepted).
4. Remove any existing `RCx_OPTION=165` (Arm/Emergency Stop) assignments,
   including the third channel required by earlier applet versions. Set those
   options to 0 (Do Nothing), or deliberately reassign the freed channels.
   Only the action and interlock channels are needed.
5. Reboot after setting the channel options. Refresh the ground station's
   parameter list to see the `INTLCK_` parameters.

The channel numbers above are examples. Choose unused channels and scripting
options that do not conflict with other functions or scripts. Install only one
copy of the applet; replace an existing `pilk.lua` when updating.

The applet invokes auxiliary function 165 directly; it does not override another
channel's PWM. With no channel assigned option 165, active emergency stop causes
`PreArm: Motors Emergency Stopped`. This is intentional: clear the stop with an
authorized MIDDLE or HIGH action before expecting pre-arm checks to pass. HIGH
clears the stop and requests arming, still subject to normal arming checks.
Leaving an option-165 channel assigned can suppress this pre-arm failure when
that channel reads LOW, and native switch actions can bypass the interlock.

## Parameters

| Parameter | Default | Range | Meaning |
| --- | --- | --- | --- |
| `INTLCK_ACT_FN` | 306 | 300–307 | Scripting RC option assigned to the action switch. |
| `INTLCK_LCK_FN` | 307 | 300–307 | Scripting RC option assigned to the interlock switch. |

Use distinct values and match each value to its channel's `RCx_OPTION`.
If the two functions are equal, the applet reports `RC functions must differ`
and ignores action-switch changes, preserving the current motor-stop and arming
state. Correcting the parameters does not replay a discarded change; move the
action switch again with the interlock held. Changing `INTLCK_ACT_FN` also
requires a new action-switch movement before any action is accepted.
The applet reserves scripting parameter table key 194 with prefix `INTLCK_`;
other installed scripts must not use that key for a different table.
When upgrading from a version using key 104, record and reapply your
`INTLCK_ACT_FN` and `INTLCK_LCK_FN` values: saved values do not migrate to the
new key. Key 104 belongs to the TOFSense-M CAN driver.

## Operation

Press and hold the momentary interlock **before** moving the three-position
action switch. Release the interlock after selecting the desired action.

| Action switch position | Requested action while interlock is held |
| --- | --- |
| Low | Enable emergency motor stop. This does not disarm the vehicle. |
| Middle | Clear emergency motor stop; do not arm or disarm. |
| High | Clear emergency motor stop and attempt to arm using normal arming checks. |

Starting from a disarmed vehicle, selecting middle with the interlock held
leaves it **not emergency stopped and not armed**. This is distinct from high,
which also requests arming. Middle preserves the existing arming state: if the
vehicle is already armed, it stays armed.

The applet checks switch positions at 20 Hz and acts only when the action switch
changes. A change without the interlock is rejected with `AEST-Lock no Interlock`.
Holding the interlock afterward does not replay that change: move the action
switch again while holding it. Releasing the interlock does not stop motors or
undo the last accepted action. Switch position alone therefore does not indicate
the current motor-stop state.

When loaded while disarmed, the applet requests emergency motor stop immediately
and then waits 25 seconds before processing switches.
Leave the action switch low and the interlock released during startup. The first
sample after the delay can act on a middle or high action switch if the interlock
is already held. When loaded while armed, it skips the startup delay and initial
stop request, but still processes the current switch positions immediately.

Runtime errors are reported and retried after one second. Messages such as `motors ON` and `arming ...`
report requests, not confirmation that motors are running or arming succeeded.

This applet gates only its own switch requests. It does not prevent arming or
motor-stop changes through other RC options, the ground station, or other scripts.
It does not implement an RC-loss failsafe or guarantee motor stop if scripting
stops. Keep the vehicle's normal arming checks and failsafes configured.

## Ground verification

With propellers removed, verify the actual receiver channel mapping and:

- Confirm the loaded script version and that no RC channel is assigned option 165.
- Disarmed startup requests motor stop and switch handling starts after 25 seconds.
- Active emergency stop causes a pre-arm failure; a direct arm request is rejected.
- Authorized MIDDLE clears stop and leaves a disarmed vehicle disarmed.
- Authorized HIGH clears stop and requests arming with normal checks.
- Action switch changes with the interlock released produce no requested action.
- Holding the interlock enables each of the three actions in the table above.
- Pressing the interlock after a rejected change does not replay it.
- Releasing the interlock leaves the last accepted state unchanged.
- Arming checks still reject arming when their requirements are not met.
- Receiver failsafe settings do not generate an unintended authorized action.
- Repeat with the actual receiver, other installed scripts, and each vehicle's
  motor outputs; SITL results do not establish real-vehicle behavior.

Confirm vehicle arming and emergency-stop status in the ground station; do not
use the applet's request messages as proof of either state.
