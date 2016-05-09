# MAVProxy modules #

This folder contains modules for MAVProxy specifically for ArduPilot. Add the
path to this folder to your `PYTHONPATH` in order to use it.

# Modules #

## `sitl_calibration` ##
This module interfaces with the `calibration` model of SITL. It provides
commands to actuate on the vehicle's rotation to simulate a calibration
process.

### Accelerometer Calibration ###
The command `sitl_accelcal` listens to the accelerometer calibration status
texts and set the vehicle in the desired attitude. Example:
```
accelcal
sitl_accelcal
```

### Compass Calibration ###
The command `sitl_magcal` applies angular velocity on the vehicle in order to
get the compasses calibrated. Example:
```
magcal start
sitl_magcal
```

### Other commands ###
There are other commands you can use with this module:
 - `sitl_attitude`: set vehicle at a desired attitude
 - `sitl_angvel`: apply angular velocity on the vehicle
 - `sitl_stop`: stop any of this module's currently active command
