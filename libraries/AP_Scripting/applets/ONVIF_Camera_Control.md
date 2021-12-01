# ONVIF_Camera_Control Lua Script

This is a script that converts AntennaTracker into a Drone Tracking Camera using ONVIF capable Camera. The Camera needs to support  PTZ AbsoluteMove method. The camera is sent absolute Pan Tilt commands based on requirement from Antenna Tracker lib.

User just needs to edit the script to set string username, password and IP address:port of the camera. Currently we only support running as part of SITL/Linux instance. Which can simply be done by adding `--enable-onvif` the rest including adding this script will be taken care of by build system. Also you will need to set `PITCH_MAX` `PITCH_MIN` and `YAW_RANGE` parameters per camera specifications.

Following steps are required before doing ONVIF build:
```
cd modules/gsoap
autoreconf -f -i
./configure
make
make install #add sudo if needed
```
