
# kml_fence.lua

## Summary

`kml_fence.lua` enforces geofence compliance using complex polygon or circular regions with both AMSL and AGL altitude constraints. Fences are defined by a separate Lua file generated from a KML dataset using the `kml_to_lua.py` tool.

The script monitors vehicle position and altitude at 1Hz. If a vehicle enters a defined region and exceeds its corresponding altitude limit, it triggers a failsafe (mode switch to RTL) and reports the breach to the GCS.

## Required Files

- `kml_fence.lua`: The main Lua script, to be placed in `APM/scripts/` or `APM/scripts/modules/`.
- `fence.lua`: A data file generated from KML using the Python tool. It must be saved as `APM/scripts/modules/fence.lua`.

## Parameter Table

```lua
-- @Param: FEN_KML_ENABLE
-- @DisplayName: KML Fence Enable
-- @Description: Enable or disable KML-based geofence checking
-- @Values: 0:Disabled,1:Enabled
-- @User: Standard
```

## Setup Instructions

1. Convert your KML geofence file into a Lua-compatible format using:

   ```bash
   python3 kml_to_lua.py your_file.kml > fence.lua
   ```

   Optional arguments:
   - `--circle-tolerance <meters>`: Circle detection tolerance (default: 10).
   - `--polygon-tolerance <meters>`: Polygon simplification threshold (default: 5).
   - `--exclude <pattern1,pattern2,...>`: Wildcard names to exclude.
   - `--exclude-nested-nfz`: Automatically exclude nested NFZs (e.g., fully-contained no-fly zones).
   - `--generate-kml <output.kml>`: Outputs a new KML file visualizing kept/excluded zones.

2. Place the generated `fence.lua` into `APM/scripts/modules/`.

3. Update the `limits` table in `kml_fence.lua` to associate names from the KML file with altitude constraints:

   ```lua
   local limits = {
       { name="Area 1", AGL=400, AMSL=nil },
       { name="Area 2", inclusion=true },
       { name="NFZ 1", exclusion=true },
       { name="High Alt Zone", AGL=nil, AMSL=4900 },
   }
   ```

Note that individual polygons can be marked as inclusion or exclusion
horizontal fences as well as having altitude limits. You can combine
these into a single entry or do them as separate lines.

4. Load the Lua script onto your SD card.

5. Reboot the autopilot or reload scripts.

## Behavior

- At 1Hz, the script:
  - Checks if the vehicle is inside any polygon from the fence file.
  - Evaluates altitude margins using AGL and AMSL limits (in feet).
  - Publishes the remaining margin as `KML_MARGIN`.
  - If a violation is detected inside a defined zone, switches mode to RTL and emits an alert.

## Notes

- Height units in the `limits` table are in **feet**.
- Requires working terrain data (for AGL).
- The Lua parameter `FEN_KML_ENABLE` can be used to enable/disable operation at runtime.
- Polygons or circles with less than 3 points are ignored and logged as errors.
