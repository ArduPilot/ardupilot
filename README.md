# ArduPilot Grid Fin Control Fork

[![Discord](https://img.shields.io/discord/674039678562861068.svg)](https://ardupilot.org/discord)  

This repository contains modifications to **ArduPilot** to support **tri-fin control surfaces** for grid-fin gliders and other experimental plane-like vehicles. These changes are primarily implemented in **ArduPlane/servos.cpp** and allow flexible fin mixing for enhanced maneuverability.

---

## BRGR Glider Example

*This fork was developed with the BRGR high-altitude glider in mind.*  

![BRGR Glider](path/to/brgr_glider_image.png)  
*Example vehicle that benefits from tri-fin control.*

---

## Key Modifications

- **Tri-Fin Mixing**
  - Implemented in `ArduPlane/servos.cpp`
  - Supports 3-fin control allocation for pitch, yaw, and roll
  - Configurable azimuth angles for each fin
- **Parameters Added**
  - `TRI_MIX_GAIN` – Overall mixer gain
  - `TRI_TRIM_DZ` – I-term deadzone to prevent servo chatter
  - `TRI_FIN1_AZIMUTH`, `TRI_FIN2_AZIMUTH`, `TRI_FIN3_AZIMUTH` – Define fin placement
- **SITL Interface**
  - VSCode `launch.json` and `tasks.json` included for PLAV SITL compatibility
- **Extensible for Multiple Fins**
  - Code supports adding standard or grid fins with minimal changes

---

## Repository Structure

params/
└── grid_fin_vehicle.param # Reference vehicle parameters
ArduPlane/
└── servos.cpp # Tri-fin control implementation
.vscode/
└── launch.json # SITL debugging config
└── tasks.json # SITL tasks
README.md


---


**Parameter Notes:**

Parameters:

```text
TRI_MIX_GAIN      = 1.0    # Overall tri-fin mixer gain
TRI_TRIM_DZ       = 0.01   # Deadzone for trim (prevents servo chatter)
```
Are configurable in flight (Prameters.ccp)

Parameters:

```text
TRI_FIN1_AZIMUTH  = 90°    # Clockwise from +Y in Y-Z plane
TRI_FIN2_AZIMUTH  = 210°
TRI_FIN3_AZIMUTH  = 330°
```

Are compiled in code can be chagned preflight (config.h)

---

## SITL Setup

This fork includes configurations for PLAV SITL for rapid testing/debugging:

launch.json: Configures VSCode to launch PLAV SITL (self-implemented SITL connecting via UDP socket)

tasks.json: Starts SITL in the background with console, map, and OSD output

"program": "${workspaceFolder}/build/sitl/bin/arduplane",
"args": ["-S", "--model", "+", "--speedup", "1", "--slave", "0", "--sim-address=127.0.0.1", "-IO"]
Changes From Upstream ArduPilot


## Example Use Case

This fork is ideal for plane-like vehicles that use grid fins or multi-fin control surfaces.

## License

This fork follows the same license as ArduPilot: GNU GPL v3.

GPLv3 Overview
