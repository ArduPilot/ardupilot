# Sony AS-DT1 LiDAR Driver

Sony AS-DT1 LiDAR depth sensor driver lua script. Feeds the sensor's multipoint
distance frame into ArduPilot's proximity library (PRX1_TYPE = 15) for obstacle
avoidance. Copter/Rover/Plane 4.6 and above.

The AS-DT1 is a compact solid-state dToF LiDAR (29 x 29 x 31 mm, 50 g or less)
with up to 576 distance measurement points over a ~37 x 29 degree field of view,
an eye-safe Class 1 940 nm laser and a range of roughly 20 m outdoors (bright
ambient light) and up to 40 m indoors. It is connected to the autopilot over a
serial (UART) port.

## Wiring and power

The sensor's 8-pin JST-GH connector carries both power and UART:

| Pin | Signal  | Connect to                              |
|-----|---------|-----------------------------------------|
| 1   | GND     | common ground (supply and autopilot)    |
| 2   | VCC     | 12 V to 24 V from battery / BEC         |
| 5   | UART TX | autopilot serial RX                     |
| 6   | UART RX | autopilot serial TX                     |

**The AS-DT1 needs 12 V to 24 V (about 0.2 A at 12 V).** A standard autopilot
telemetry/UART port only supplies 5 V, which will not power the sensor. Feed
12-24 V to pins 1/2 from the battery or a BEC and connect only TX, RX and GND
to the autopilot. The UART uses 3.3 V logic (RX is 5 V tolerant).

**Out of the box the AS-DT1 outputs its data over its USB-C port, not over the
8-pin connector.** Before wiring it to the autopilot, connect the sensor to a PC
over USB-C once and use Sony's AS-DT1 sample application (part of the AS-DT1
SDK) to switch it to UART output and set the UART speed to 460800 baud (the
driver default; if you choose another speed, set ASDT1_BAUD accordingly). The
setting is stored in the sensor.

## How to use - Pre-Configuration

- Switch the sensor to UART output at 460800 baud with Sony's AS-DT1 application (see "Wiring and power")
- Connect the sensor to one of the autopilot's serial ports as described above
- Set SERIALx_PROTOCOL = 28 (Scripting) on that port (SERIALx_BAUD does not matter, the script sets the baud rate)
- Set SCR_ENABLE = 1 to enable scripting
- Set SCR_VM_I_COUNT = 50000 (the default instruction budget is not enough for the serial parser)
- Set PRX1_TYPE = 15 (Scripting) to enable the proximity scripting backend
- Optionally set AVOID_ENABLE and/or OA_TYPE so the vehicle acts on the obstacles
- Reboot the autopilot
- Copy the Sony_AS-DT1.lua script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot

## How to use - Script Parameter Configuration

If everything above is done correctly, new "ASDT1_" parameters should be visible
(only after the script loads, refresh parameters if not visible). ASDT1_MODE,
ASDT1_SP and ASDT1_BAUD are read at boot and need a reboot to change; the other
parameters take effect immediately.

### ASDT1_MODE

Operating mode, see below. 2 = 2D (default), 4 = 2D+, 3 = 3D.

### ASDT1_BAND

2D and 2D+ only. Only measurement points whose elevation is within +/- this
angle (degrees) of the sensor horizon (2D) or of the true horizon (2D+) are
used. The sensor covers about +/- 14.6 degrees vertically, so 15 or more uses
the full vertical field of view (default 90). Small values (3 to 6) keep a thin
horizontal slice and reduce false ground or ceiling detections.

### ASDT1_MIN_M / ASDT1_MAX_M

Measurements closer than ASDT1_MIN_M (default 0.15 m) or farther than
ASDT1_MAX_M (default 30 m) are discarded. These are also reported as the
minimum/maximum distance of the proximity sensor.

### ASDT1_PITCH_OFF

Pitch of the sensor relative to the airframe in degrees, nose-up positive. Use
it to correct a tilted mount. In 2D+ mode it is added to the AHRS pitch, which
shifts the usable pitch window (see below).

### ASDT1_DEBUG

Set to 1 (default) to get driver status messages on the GCS, 0 to silence them.

### ASDT1_SP

Which scripting serial port the sensor is connected to. 1 = the first port with
SERIALx_PROTOCOL = 28, 2 = the second, and so on.

### ASDT1_BAUD

Baud rate of the sensor UART, default 460800. Must match the UART speed that
was set in the sensor with Sony's AS-DT1 application (see "Wiring and power").

## Operating modes

### 2D (ASDT1_MODE = 2)

The 24 x 24 frame is collapsed into horizontal azimuth sectors. For each sector
the minimum distance is converted to a horizontal distance and pushed to the
proximity library with pitch 0. ArduPilot streams the result to the GCS as
MAVLink DISTANCE_SENSOR and it can be used by Simple Avoidance (AVOID_ENABLE)
and the path planners (OA_TYPE). Because the sensor's ~37 degree field of view
fits inside ArduPilot's front 45 degree proximity sector, only the forward
sector is populated.

### 2D+ (ASDT1_MODE = 4)

A multicopter pitches nose-down to accelerate and fly forward, so a fixed
forward-facing sensor ends up looking at the ground and can report it as an
obstacle, causing unwanted stops. In 2D+ the driver reads the AHRS pitch and
keeps the measurement rows that look at the true horizon instead of the rows
around the sensor axis, so obstacles ahead stay in view during forward flight.

Use 2D+ together with a narrow ASDT1_BAND (3 to 6 degrees). The compensation is
limited by the sensor's vertical field of view: the horizon can only be tracked
while the pitch stays within about +/- (14.6 - ASDT1_BAND/2) degrees of the
sensor axis. To extend this window in the nose-down direction, mount the sensor
tilted up by the typical cruise nose-down angle and set ASDT1_PITCH_OFF to that
angle; the window is then centred on the cruise attitude.

### 3D (ASDT1_MODE = 3)

The frame is reduced to a 5 x 5 grid of cells and the nearest point of each
cell is pushed as a 3D obstacle vector, giving full 3D obstacle avoidance and
path planning.

Note that "3D" refers to the vertical resolution within the sensor's field of
view: the AS-DT1 only sees a cone of roughly 37 x 29 degrees in front of the
vehicle, so 3D mode does not give all-round coverage like a 360 degree lidar or
several sensors would.

## What to expect

With ASDT1_DEBUG = 1 the following messages appear within a few seconds of
boot:

```
AS-DT1: sync ok
AS-DT1: streaming @ 10 Hz, 2D 9 sectors, band +/-90 deg -> MAVLink
AS-DT1[2D]: 9 sectors, nearest 1.24 m (MAVLink DISTANCE_SENSOR)
```

Objects 0.5 m to 3 m in front of the sensor should show up in the GCS
proximity view. Very close objects return nothing: below roughly 0.5 m the
sensor is in its near dead zone.

If "no scripting serial" is reported, check SERIALx_PROTOCOL and ASDT1_SP. If
"resync failed" is reported, check wiring, power, ASDT1_BAUD and that the sensor was switched to UART output. If the script
reports "exceeded time limit", raise SCR_VM_I_COUNT.

## Notes

The lookup tables embedded in the script (measurement-point stream order and
per-point ray directions) are derived from the order documented in the AS-DT1
API manual and the ray geometry reported by the sensor's `svminfo` command,
and are generated by a small generator script (UAV-DEV GmbH). The grid
resolution used for the 3D cells and the 2D sectors is fixed at generation
time.
