# Parameter Set Filter Script

## Description

This Lua script adds a safty-focused layer to MAVLink parameter setting. When enabled, it intercepts all incoming `PARAM_SET` messages and only allows modification of a predefined whitelist of parameters.

This is especially useful in scenarios where parameter integrity is critical (e.g., automated testing, payload-hosted GCS systems, or shared telemetry environments).

When enabled, the script disables ArduPilot's default GCS handling for parameter writes and takes over that responsibility with additional filtering.

## How It Works

- Intercepts MAVLink `PARAM_SET` messages targeted at the vehicle.
- Validates that the parameter ID is in the whitelist.
- Only whitelisted parameters will be updated.
- Dynamically toggles ArduPilot’s `gcs:set_allow_param_set()` based on enable state.

## Requirements

- ArduPilot 4.7 or later with Lua scripting enabled (`SCR_ENABLE = 1`)
- copy the script to the vehicle autopilot's "scripts" directory
- The script must be placed in appropriate directories:

## Parameters

This script introduces a new parameter table named `PARAM_SET_`. It includes:

| Name              | Type   | Default | Description                            |
|-------------------|--------|---------|----------------------------------------|
| `ENABLE`          | `int`  | `1`     | Enables (1) or disables (0) the script |

You can disable the script by setting `PARAM_SET_ENABLE = 0`. **Note:** disabling `SCR_ENABLE` while this script is actively blocking parameter sets is not recommended.

## Whitelisted Parameters

Only the following parameters are allowed to be set when the script is enabled:

- `MAV_OPTIONS`
- `PARAM_SET_ENABLE`
- Battery parameters (e.g., `BATT_ARM_MAH`, `BATT_FS_CRT_ACT`, etc.)
- `BRD_OPTIONS`
- `COMPASS_USE3`
- Geofence and RTL parameters (e.g., `FENCE_TYPE`, `RTL_ALT`)
- `LOG_*`, `LIGHTS_ON`

(See the full script for the complete whitelist.)

## Usage

1. Copy the script into the SD card's scripts directory.
2. Ensure `SCR_ENABLE` and `PARAM_SET_ENABLE` are both set to `1`.
3. Reboot the autopilot or reload the script if necessary.

## Logging & Debugging

The script sends diagnostic messages to the GCS:

- Allowed parameter updates show a `param set received` message.
- Blocked updates show a `param set denied` message with the offending parameter name.

Example output:

[param-set] param set received
[param-set] param set denied (LOG_BACKEND_TYPE)


## Limitations

- Only handles `PARAM_SET` messages (not `PARAM_REQUEST_*`).
- The parameter whitelist is hardcoded; changes require script edits.
- ArduPilot must support the `gcs:set_allow_param_set()` interface.
- parameter upload via FTP is denied when this script is running

## Author

[Contributed via PR #29989](https://github.com/ArduPilot/ardupilot/pull/29989)  
May 2025
