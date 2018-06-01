SITL for Windows Automatic Installation Scripts

This is a collection of batch files to automatically install and run APM SITL required libraries (http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) on a Windows-based PC/laptop.

The scripts are based on the SITL setup instructions here: http://ardupilot.org/dev/docs/sitl-native-on-windows.html
Prerequisites:

    None! The scripts will take care of everything.
    You will need an Internet connection and an hour of time (will be less with a fast Internet connection) for the scripts to download and install everything.

1) Assumptions:

    The scripts will install Cygwin (to C:\cygwin) and MAVProxy. So access to the C:\ drive is required.
    
    There are 2 options for installation: Just the development environment (2A) , or the development environment plus APM source code (2B). The first option is for users that already have the APM source code on their computer (or wish to have it in a particular folder). The second option will download and configure the APM source code too, which is easier for new users.

2A) Installing just the Development Environment:

    Download and run the script "InstallDevEnvironment.ps1" (right click -> Run in Powershell). This will install Cygwin to C:\cygwin and MAVProxy to C:\Program Files (x86)\MAVProxy
    
2B) Installing the Development Environment and APM source code:

    Download and run the script "InstallDevEnvironmentAndAPMSource.ps1" (right click -> Run in Powershell). This will install Cygwin to C:\cygwin and MAVProxy to C:\Program Files (x86)\MAVProxy. The APM source code will be in C:\cygwin\home\<username>\ardupilot
    
NOTE: When running the above script, you may recieve an error due to the default Windows security policy not allowing downloaded scripts to run. See https://www.howtogeek.com/106273/how-to-allow-the-execution-of-powershell-scripts-on-windows-7/ for details on how to disable this policy.
    
3) Running SITL:

There are several options for running SITL. In all cases, SITL will output a mavlink stream on 127.0.0.1:14550 (UDP) for connection to any GCS software (Such as Mission Planner).

An EEPROM (containing the state of the APM's flash memory - paramters, waypoints, etc) is saved for each vehicle type and (by default) will be re-loaded each time SITL is run. This can be overidden to start with a new EEPROM containing all default values.

To continue with the current EEPROM, run:

    RunCopter.bat for a quadcopter running APM:Copter
    RunPlane.bat for running APM:Plane
    RunRover.bat for running APM:Rover

4) Updating the APM source code

To update the APM code with the latest of Github, run UpdateAPMSource.bat. Note this is bleeding-edge source code and some bugs may be present.
Deleting the SITL environment

To uninstall/delete the SITL environment, simply delete the "cygwin" folder in the C:\ drive.
