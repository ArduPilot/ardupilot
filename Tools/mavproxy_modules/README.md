# MAVProxy modules #

This folder contains modules for MAVProxy specifically for ArduPilot. Add the
path to this folder to your `PYTHONPATH` in order to use it.

# Modules #

## `sitl_calibration` ##
This module interfaces with the `calibration` model of SITL. It provides
commands to actuate on the vehicle's rotation to simulate a calibration
process.

Make sure to pass `--model calibration` to the SITL binary in order to be able
use this module's commands. You can also use
`[sim_vehicle.py](../autotest/sim_vehicle.py)` with `--frame calibration`.

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

## `magcal_graph` ##
This module shows the geodesic sections hit by the samples collected during
compass calibration, and also some status data. The objective of this module is
to provide a reference on how to interpret the field `completion_mask` from the
`MAG_CAL_PROGRESS` mavlink message. That information can be used in order to
guide the vehicle user during calibration.

The plot shown by this module isn't very helpful to the end user, but it might
help developers during development of internal calibration support in ground
control stations.

The only command provided by this module is `magcal_graph`, which will open the
graphical user interface.
