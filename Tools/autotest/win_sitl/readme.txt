SITL for Windows Automatic Installation Scripts

This is a collection of batch files to automatically install and run APM SITL (http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) on a Windows-based PC/laptop.

The scripts are based on the SITL setup instructions here: http://ardupilot.org/dev/docs/sitl-native-on-windows.html
Prerequisites:

    None! The scripts will take care of everything.
    You will need an Internet connection and an hour of time (will be less with a fast Internet connection) for the scripts to download and install everything.

Assumptions:

    The scripts will install Cygwin, MAVProxy and the APM source to C:\cygwin. So access to the C:\ drive is required.

Installing:

    Download these scripts and unzip
    Run the script "InstallCygwinAPM.ps1" (right click -> Run in Powershell). This will install Cygwin to C:\cygwin and MAVProxy to C:\Program Files (x86)\MAVProxy

Running SITL:

There are several options for running SITL. In all cases, SITL will output a mavlink stream on 127.0.0.1:14550 (UDP) for connection to any GCS software (Such as Mission Planner).

An EEPROM (containing the state of the APM's flash memory - paramters, waypoints, etc) is saved for each vehicle type and (by default) will be re-loaded each time SITL is run. This can be overidden to start with a new EEPROM containing all default values.

To continue with the current EEPROM, run:

    RunCopter.bat for a quadcopter running APM:Copter
    RunPlane.bat for running APM:Plane
    RunRover.bat for running APM:Rover

To delete the old EEPROM and start with all default settings, run:

    ResetRunCopter.bat for a quadcopter running APM:Copter
    ResetRunPlane.bat for running APM:Plane
    ResetRunRover.bat for running APM:Rover

Updating the APM source code

To update the APM code with the latest of Github, run UpdateAPMSource.bat. Note this is bleeding-edge source code and some bugs may be present.
Deleting the SITL environment

To uninstall/delete the SITL environment, simply delete the "cygwin" folder in the C:\ drive.
