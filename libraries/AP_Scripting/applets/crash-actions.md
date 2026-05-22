# Crash Actions

Crash Actions (`crash-actions.lua`) augments `FS_CRASH_CHECK` (Copter/Rover) and `CRASH_DETECT` (Plane) in firmware v4.8 and later. If crash detection is enabled, this script enables additional, configurable post-crash actions via the following parameters:

- `CRASH_ACT1`: Action type; see definitions below.
- `CRASH_ACT1_INST`: Instance number of relay/servo, as required.
- `CRASH_ACT1_TIME`: Time in seconds after crash detection to trigger the action.
- `CRASH_ACT2`
- `CRASH_ACT2_INST`
- `CRASH_ACT2_TIME`
- ...and so on, through `CRASH_ACT4`.

## Configuration

- Enable `FS_CRASH_CHECK` (or `CRASH_DETECT` for Plane).
- Set `CRASH_ACTx` (where `x` is 1-4) to one of the following action types:
  - `0`: Disable (default)
  - `1`: Disarm
  - `2`: Relay off
  - `3`: Relay on
  - `4`: Servo min
  - `5`: Servo trim
  - `6`: Servo max

- Set `CRASH_ACTx_INST` to the desired relay number or servo channel (unused for disarm).
- Set `CRASH_ACTx_TIME` to the desired time in seconds after crash detection to trigger the action.

## Use Cases

This script was developed for use with large Rovers, where fire risk or other damage could result from an unmitigated mechanical failure or stuck condition. Once a crash is detected, the script can automatically disengage autopilot controlled PTO drives, command a combustion engine to idle, kill ignition, etc.

These features may be less useful on airborne platforms, but the script is vehicle agnostic, should a use case arise.

## Advanced Configuration

If more than 4 post-crash actions are desired, edit the script's `NUM_CRASH_ACTIONS` variable to the desired number of actions. This will increase the number of available `CRASH_ACTx` parameters accordingly.
