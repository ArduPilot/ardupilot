# HLK-LD2451 Radar Mounting Reference (Webots rover)

Reference geometry for the 4 corner radars on the Webots Ackermann rover,
taken from `ardupilot_rover.wbt`'s `Radar` node transforms. Cross-check
actual reported detection angles against this once real radar data starts
coming through the driver.

## Position — body frame (x = forward, y = left, z = up), metres

| Radar | Corner | translation (x, y, z) |
|---|---|---|
| `radar_fl` | front-left | (+0.865, +0.35, −0.10) |
| `radar_fr` | front-right | (+0.865, −0.35, −0.10) |
| `radar_rl` | rear-left | (−0.865, +0.35, −0.10) |
| `radar_rr` | rear-right | (−0.865, −0.35, −0.10) |

- Longitudinal: ±0.865 m from body centre (body is 1.83 m long → ~5 cm inside each end)
- Lateral: ±0.35 m from centreline (body 0.80 m wide)
- Height: −0.10 m from body centre (≈ 0.47 m above ground at spawn ride-height)

## Aim — yaw about z (0° = forward, + = left/CCW)

| Radar | rotation (rad) | = degrees | Points |
|---|---|---|---|
| `radar_fl` | 0.20944 | +12° | forward, toed 12° left |
| `radar_fr` | −0.20944 | −12° | forward, toed 12° right |
| `radar_rl` | 2.93215 | 168° | rearward, 12° off straight-back (left) |
| `radar_rr` | −2.93215 | −168° (192°) | rearward, 12° off straight-back (right) |

## Beam / range (all four identical — HLK-LD2451 spec)

| Field | Value |
|---|---|
| `horizontalFieldOfView` | 0.698 rad = 40° (±20°) |
| `verticalFieldOfView` | 0.2 rad = ~11.5° (±5.7°) |
| `minRange` / `maxRange` | 0.5 m / 100 m |

## Resulting coverage

- **Front arc**: −32° … +32° (fl + fr), overlap −8°…+8° dead-ahead
- **Rear arc**: 148° … 212° (rl + rr), overlap 172°…188° dead-astern
- **Side gaps**: roughly ±32°–148° (inherent to a 4× ±20° corner layout)

## ArduPilot side (`myRover.parm`) — mounting angle told to the EKF

Each radar's yaw is passed to its proximity instance so detections rotate
into the body frame via `PRXn_YAW_CORR`:

| Serial | PRX instance | Corner | `PRXn_YAW_CORR` |
|---|---|---|---|
| `SERIAL2` | `PRX1` | fl | `+12` |
| `SERIAL5` | `PRX2` | fr | `−12` |
| `SERIAL6` | `PRX3` | rl | `+168` |
| `SERIAL7` | `PRX4` | rr | `−168` |

These `YAW_CORR` values match the world's mounting rotation exactly (in
degrees) — that's what keeps the sim geometry and ArduPilot's proximity
picture consistent. Verified against the committed `myRover.parm` on
2026-07-26.
