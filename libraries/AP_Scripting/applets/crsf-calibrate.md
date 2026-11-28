# CRSF Calibration Menu

This script implements a vehicle calibration menu via CRSF parameter commands
which can be accessed from your radio transmitter via [TBS Agent Lite](https://www.team-blacksheep.com/products/prod:agentx)
You can calibrate compass, accels, gyros and trim via the menu

On firmware that runs a script's calibrations on the main thread, the gyro,
accel, trim and force calibrations show "Calibrating" until they finish,
then their result until you press OK.

## Operation

- Install the lua script in the APM/scripts directory on the flight
controller's microSD card.
- Install the helper module modules/crsf_helper.lua in the APM/scripts/modules directory on the flight
controller's microSD card.
- Enable scripting via SCR_ENABLE.
