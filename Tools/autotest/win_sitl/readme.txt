Running SITL:

There are several options for running SITL. In all cases, SITL will output a mavlink stream on 127.0.0.1:14550 (UDP) for connection to any GCS software (Such as Mission Planner).

An EEPROM (containing the state of the APM's flash memory - paramters, waypoints, etc) is saved for each vehicle type and (by default) will be re-loaded each time SITL is run. This can be overidden to start with a new EEPROM containing all default values.

To continue with the current EEPROM, run:

    RunCopter.bat for a quadcopter running APM:Copter
    RunPlane.bat for running APM:Plane
    RunRover.bat for running APM:Rover

4) Updating the APM source code

To update the APM code with the latest of Github, run UpdateAPMSource.bat. Note this is bleeding-edge source code and some bugs may be present.
Deleting the SITL environment

To uninstall/delete the SITL environment, simply delete the "cygwin64" folder in the C:\ drive.
