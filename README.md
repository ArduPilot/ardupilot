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
```text
brgr/
└── grid_fin_vehicle.param # Reference vehicle parameters
ArduPlane/
└── servos.cpp # Tri-fin control implementation
.vscode/
└── launch.json # SITL debugging config
└── tasks.json # SITL tasks
README.md
```

---


**Parameter Notes:**

Parameters:

```text
TRI_MIX_GAIN      = 1.0    # Overall tri-fin mixer gain
TRI_TRIM_DZ       = 0.01   # Deadzone for trim (prevents servo chatter)
```
Are configurable in flight (Parameters.ccp)

Parameters:

```text
TRI_FIN1_AZIMUTH  = 90°    # Clockwise from +Y in Y-Z plane
TRI_FIN2_AZIMUTH  = 210°
TRI_FIN3_AZIMUTH  = 330°
```

Are compiled in code can be chagned preflight (config.h)

---

## SITL

This project integrates with [Python Laptop Air Vehicles (PLAV)](https://github.com/adinkojic/PLAV), a 6 Degree-of-Freedom flight simulator written in Python with real-time hardware-in-the-loop support.

It uses ArduPilot’s SITL with JSON interface:  
https://ardupilot.org/dev/docs/sitl-with-JSON.html  

This repository also demonstrates how to properly debug SITL with JSON in VSCode, which is not clearly documented in the official ArduPilot developer docs.

---

### Running the Simulation

1. **Start PLAV first**
   ```bash
   plav sitl-sim --noise --live brgrBalloonDrop
   ```
  For more details, refer to the PLAV README.

2. **Start ArduPilot SITL**
    - **For debugging in VSCode:**
        - Press `F5` to launch the debugger
        - `tasks.json` will automatically start SITL and attach GDB
    
    - **For a normal run (no debugger):**
    
    ```bash
    ../Tools/autotest/sim_vehicle.py -f JSON:127.0.0.1 --console --map -L BRGRBalloon

**Configuration Notes**

The `-L BRGRBalloon` parameter defines the vehicle spawn location and must exist in:
```text
locations.txt
```
Example entry:
```text
BRGRBalloon=40.269712,-73.769042,34000.0,0
```
The PLAV model (e.g., brgrBalloonDrop) must match your vehicle configuration.

**WSL / Cross-Platform Notes**

When using WSL, PLAV may require binding to:
```bash
0.0.0.0
```
To find the Windows host IP from WSL:
```bash
ip route | awk '/^default/ {print $3}'
```

Use this IP when launching ArduPilot SITL (both manually and in the launch and task JSON for debugging):
```bash
sim_vehicle.py -f JSON:[WINDOWS_IP]
```


## Example Use Case

This fork is ideal for plane-like vehicles that use grid fins or multi-fin control surfaces.

## License

This fork follows the same license as ArduPilot: GNU General Public
License, version 3.

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)

