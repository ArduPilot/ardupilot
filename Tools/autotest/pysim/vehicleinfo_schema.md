# vehicleinfo.json schema

`vehicleinfo.json` is the canonical source describing every SITL vehicle and
frame variant ArduPilot can simulate. It is loaded by
`Tools/autotest/pysim/vehicleinfo.py` for tooling (`sim_vehicle.py`,
`autotest.py`, `vehicle_test_suite.py`, etc.) and is also embedded in the SITL
binary's ROMFS so the binary can resolve `--model=<frame>` to a default
parameter list without the source tree present.

## Structure

```json
{
  "<VehicleName>": {
    "default_frame": "<frame>",
    "frames": {
      "<frame>": { ...frame fields... },
      ...
    }
  },
  ...
}
```

Top-level keys are vehicle names: `ArduCopter`, `ArduPlane`, `Rover`,
`ArduSub`, `AntennaTracker`, `Helicopter`, `Blimp`, `sitl_periph_universal`.

## Per-frame fields

| Field | Type | Required | Meaning |
|---|---|---|---|
| `waf_target` | string | yes (with one exception) | Target name passed to `waf --target` to build the binary, e.g. `"bin/arducopter"`. May be omitted on rare entries (e.g. `calibration`) where the frame is purely a MAVProxy mode. |
| `default_params_filename` | string OR list of strings OR `[]` | no | Path(s) to default parameter `.parm` files, relative to `Tools/autotest/`. Multiple paths are loaded in order; later files override earlier. An empty list means defaults are loaded internally by the SITL physics backend (e.g. `plane-3d`). Paths are typically `default_params/<name>.parm` or `models/<name>.parm`. |
| `periph_params_filename` | list of strings | no | Default parameter files for the CAN peripheral side of CAN-enabled simulations (`quad-can`, `quadplane-can`). |
| `external` | bool | no | True when the physics is simulated by an external program (Gazebo, AirSim, X-Plane, JSBSim, Scrimmage, etc.) and SITL is just the autopilot. |
| `model` | string | no | Override of the physics model string passed to the SITL binary's `--model` argument. Defaults to the frame name. May embed a JSON model path (e.g. `"X:@ROMFS/models/freestyle.json"`). |
| `frame_example_script` | string | no | Lua script in `libraries/AP_Scripting/examples/` that this frame uses for motor mixing, etc. |
| `configure_target` | string | no | Used only by sitl_periph variants — overrides the `waf configure --board` target. |
| `extra_mavlink_cmds` | string | no | Extra commands fed to MAVProxy after startup. |

`sitl-port` is computed at lookup time, not stored in the JSON.

## Default-load resolution at runtime

When the SITL binary is invoked with `--model=<frame>`, the binary's startup
code (`libraries/AP_HAL_SITL/SITL_cmdline.cpp`) resolves the matching frame's
`default_params_filename` list to a comma-separated `@ROMFS/<path>` string
and **prepends** it to whatever was passed via `--defaults`. AP_Param loads
the comma-separated list left-to-right with later files overriding earlier
ones, so:

- The ROMFS-embedded per-frame defaults always load first (base layer).
- Any user-supplied `--defaults` file (typically from `sim_vehicle.py`'s
  `--add-param-file` / `-P`) is loaded after, so its values override.

If `--model` doesn't match any frame in `vehicleinfo.json`, only the explicit
`--defaults` files (if any) are loaded — the binary still boots cleanly with
no embedded defaults.

## Editing

Edit `vehicleinfo.json` directly. Whitespace is `indent=4`. Run
`python3 -c "import json; json.load(open('Tools/autotest/pysim/vehicleinfo.json'))"`
to validate.

After editing, rebuild SITL so the embedded copy in
`build/sitl/ap_romfs_embedded.h` picks up the change.

## Discovering supported models

Any SITL binary with this support compiled in answers `--list-models`:

```sh
./build/sitl/bin/arduplane --list-models | jq .
```

Ground stations (Mission Planner, QGC) can use this to populate a model
dropdown that always matches the binary's actual capabilities.
