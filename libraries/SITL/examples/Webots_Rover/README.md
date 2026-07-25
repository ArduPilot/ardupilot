# Webots UGV Rover

A [Webots](https://cyberbotics.com/) simulation of an autonomous four-wheel
UGV rover, bridged to **ArduPilot Rover SITL**. The rover uses **front-wheel
steering + rear-wheel drive** (with an Ackermann kinematic differential),
independent suspension on every wheel, and a full sensor suite.

It runs in two ways:

- **ArduPilot SITL** — fly real missions (AUTO/GUIDED) from a ground station,
  with Webots supplying the GPS/IMU and ArduPilot driving the wheels.
- **Manual / reactive** — drive from the keyboard, run a reactive
  obstacle-avoidance loop, or take commands from an external file.

Worlds are included for both, on a flat arena and on a **real-world map**
generated from OpenStreetMap.

> Developed and tested with **Webots R2025a** on Windows.

---

## Features

- **Full-size rover** — 1.83 × 0.80 × 0.61 m body (6 × 4 × 2 ft), 18 in (0.457 m)
  wheels, ~170 kg, 0.36 m ground clearance, softer long-travel suspension.
- **Three drive modes** — Manual (keyboard), Autonomous (reactive obstacle
  avoidance), Remote (command file).
- **Sensor suite** — front FPV camera, top-down "map" camera, GPS, IMU, gyro,
  accelerometer, two front distance sensors, four corner radars, and steering
  position sensors.
- **Steering PID auto-tune** — relay-feedback (Åström–Hägglund) tuning on a
  key press.
- **Runtime speed control**, full state logging to CSV, and colour-coded
  components for easy visual identification (green = front, red = rear).
- **Real-world map** — a 500 × 500 m OpenStreetMap area imported as a drivable
  world, with solid road curbs and a physical geofence boundary.

---

## Repository layout

```
worlds/
  ardupilot_flat.wbt    # flat arena  + ArduPilot bridge  (best for missions)
  ardupilot_rover.wbt   # real OSM map + ArduPilot bridge
  ugv_rover.wbt         # test arena: rocks, trees, plants, dunes  (manual)
  osm_rover.wbt         # real OSM map (Greater Noida) + geofence  (manual)
controllers/
  ardupilot_bridge/     # Webots <-> ArduPilot SITL bridge (Python)
    ardupilot_bridge.py
    rover.parm          # ArduPilot params: servo map + navigation tuning
  ugv_teleop/           # manual / reactive rover controller (C)
    ugv_teleop.c
  target_car/           # moving Solid the corner radars can actually detect
    target_car.py
tools/
  mav_ctl.py            # MAVLink helper: arm, missions, tuning measurement
  radar_mav.py           # WSL companion: Webots radar -> real HLK-LD2451 UART
                          # frames -> ArduPilot AP_Proximity_HLK_LD2451 (SITL)
docs/
  layoutOfRover.png     # rover design blueprint
LICENSE
README.md
```

The rover model is defined inline in each world (identical Robot node in all
four); the `controller` field selects the bridge or the manual controller.

---

## Getting started

### 1. Prerequisites
- Webots **R2025a** (other recent versions may work but are untested).

### 2. Clone
```bash
git clone https://github.com/kumarnitish378/ardupilot-webots-rover.git
```

### 3. Open a world
In Webots: **File → Open World…** and choose one of:

*ArduPilot SITL* (see [Running with ArduPilot SITL](#running-with-ardupilot-sitl)):
- `worlds/ardupilot_flat.wbt` — flat arena, best for missions.
- `worlds/ardupilot_rover.wbt` — the real OSM map.

*Manual / reactive driving:*
- `worlds/osm_rover.wbt` — the real-world map.
- `worlds/ugv_rover.wbt` — the obstacle test arena.

### 4. Build the controller (first run only)
Webots compiles the controller automatically the first time you press Play.
If it doesn't, open `controllers/ugv_teleop/ugv_teleop.c` in Webots' editor
and click **Build**. (A C compiler is required — Webots bundles one on
Windows.)

### 5. Run
Press **▶ Play**, then **click inside the 3D view** so it has keyboard focus.

### 6. Show the camera overlays
Use the **Overlays** menu in Webots' menu bar to show the `camera` (front FPV)
and `map_camera` (top-down) feeds if they aren't already visible.

---

## Running with ArduPilot SITL

The bridge speaks ArduPilot's `webots-python` SITL protocol: Webots sends the
FDM (GPS/IMU/velocity, converted ENU→NED) on UDP `:9003`, and ArduPilot sends
back servo outputs on `:9002` (SERVO1 = steering, SERVO3 = throttle).

1. Open `worlds/ardupilot_flat.wbt` (or `ardupilot_rover.wbt`) and press
   **▶ Play**. The bridge waits for SITL to connect.
2. Start ArduPilot Rover SITL, pointing it at the machine running Webots:
   ```bash
   sim_vehicle.py -v Rover --model webots-python \
     --sim-address=<windows-host-ip> \
     --custom-location=28.5016472,77.3921611,0,0 \
     --add-param-file=controllers/ardupilot_bridge/rover.parm \
     --console --map
   ```
   On WSL, `--sim-address` is the Windows host IP (`ip route show default`),
   and `SITL_ADDRESS` in `ardupilot_bridge.py` is the WSL IP (`hostname -I`) —
   WSL2 does not share `localhost`. Allow inbound UDP 9002 in Windows Firewall.
3. In a second WSL terminal, start the radar bridge so the front corner
   radars feed ArduPilot's real HLK-LD2451 proximity driver (`PRX1_TYPE=19`
   in `rover.parm`):
   ```bash
   python3 tools/radar_mav.py --side front --port 5763
   ```
   It waits for and reconnects to SITL automatically; start it before or
   after SITL, order doesn't matter.
4. Plan and run a mission from any ground station (QGroundControl, Mission
   Planner), or use the bundled helper:
   ```bash
   python tools/mav_ctl.py arm            # force-arm
   python tools/mav_ctl.py square 20      # drive a 20 m square in AUTO
   python tools/mav_ctl.py track 20       # same, but measure tracking error
   python tools/mav_ctl.py setp WP_SPEED 1.5   # set any parameter
   python tools/mav_ctl.py radar 15       # watch live HLK-LD2451 proximity end to end
   ```

### Navigation tuning

`rover.parm` carries a tuned navigation set. Stock ArduPilot defaults track
this rover poorly (`WP_SPEED 5` m/s is far faster than a 1.42 m turn radius can
corner), so the key changes are a corner-able speed, a tuned velocity
controller, and a wider steering limit:

| | Cross-track RMS | Max corner error |
|---|---|---|
| Stock defaults | 1.90 m | 4.28 m |
| Tuned (`rover.parm`) | **0.47 m** | **1.22 m** |

Measured with `tools/mav_ctl.py track 20` on a 20 m square.

---

## Controls

| Key | Action |
|-----|--------|
| **↑ / ↓** | Drive forward / reverse |
| **← / →** | Steer left / right |
| **M** | Manual mode |
| **A** | Autonomous mode (reactive obstacle avoidance) |
| **R** | Remote mode (reads `rover_command.txt`) |
| **T** | Auto-tune the steering PID |
| **PageUp / + / =** | Increase speed (up to 2.5×) |
| **PageDown / -** | Decrease speed (down to 0.2×) |

### Drive modes
- **Manual** — you drive with the arrow keys.
- **Autonomous** — the rover drives itself, steering away from obstacles seen
  by the front distance sensors and reversing out if it gets boxed in or stuck
  (stuck detection uses GPS ground speed, so it works in both the local-metre
  arena and the WGS84 map world).
- **Remote** — the controller reads `drive steer` values (each in `[-1, 1]`)
  from `controllers/ugv_teleop/rover_command.txt`; the last value written is
  held until replaced. Useful for scripted control.

### Telemetry / logging
Every step is logged to `controllers/ugv_teleop/ugv_teleop_log.csv`
(position, heading, all sensor readings, steering angles, wheel speeds) and a
summary line is printed to the console once per second.

---

## Rover at a glance

| | |
|---|---|
| Body | 1.83 × 0.80 × 0.61 m (6 × 4 × 2 ft) |
| Wheels | 0.457 m dia (18 in), front steer / rear drive |
| Mass | ~170 kg |
| Wheelbase / track | 1.20 m / 1.10 m |
| Ground clearance | 0.36 m |
| Suspension | independent per wheel (spring + damper, ±0.12 m travel) |

**Colour code:** green = front (bumper + wheel stripes), red = rear; cyan =
front sensor pod, blue = camera, yellow = distance sensors, magenta = radars,
orange = nav-sensor cluster.

---

## Regenerating the OSM map (optional / advanced)

`osm_rover.wbt` was produced from OpenStreetMap data with Webots' bundled
`osm_importer` (`<WEBOTS_HOME>/resources/osm_importer/importer.py`). To build a
different area:

1. Download an `.osm` extract for your bounding box, e.g.:
   ```
   curl "https://api.openstreetmap.org/api/0.6/map?bbox=<min_lon>,<min_lat>,<max_lon>,<max_lat>" -o map.osm
   ```
2. Install the importer's Python dependencies:
   `pip install pyproj shapely numpy webcolors lxml`
3. Run the importer, then paste the rover Robot node into the generated world.

---

## Known limitations

- The four corner **radars only ever see moving targets**: Webots' Radar node
  only detects `Solid` nodes with `radarCrossSection > 0` *and* radial motion
  past `minAbsoluteRadialSpeed`, and the rocks, walls, buildings and curbs
  don't set a cross-section. In practice that means only `target_car` (the
  scripted moving vehicle) is ever detected - which matches the real
  HLK-LD2451's motion-only Doppler behaviour, so it's by design, not a bug.
  Static-obstacle avoidance in the manual/teleop worlds still relies on the
  two front **distance sensors**; the SITL path (`tools/radar_mav.py` ->
  `PRX1_TYPE=19`) only ever sees `target_car`-like moving obstacles.
- Physics values (masses, spring/damper rates, motor torques, steering PID)
  are reasonable engineering estimates, not lab-calibrated; expect to tune for
  new scenarios.

---

## Roadmap

- ~~ArduPilot Rover SITL bridge~~ — done (`controllers/ardupilot_bridge/`).
- All-around obstacle sensing (fix the radar cross-section issue).
- Obstacle avoidance fed into ArduPilot (proximity / simple-avoidance).

---

## License

Released under the [MIT License](LICENSE).
