# Mount RealFlight Driver

This script provides a driver for a simulated gimbal in RealFlight. This script
sends three servo outputs to control a camera: pitch, yaw, and zoom.

# How To Use

An invisible camera gimbal can be added to any model in RealFlight, even if the
3D assets for that camera gimbal are not there. Check the Titan Cobra model for
an example of how to add and configure a camera for use with this script.

Note that there is a 12 channel limit for servo outputs to RealFlight, so this
can only be done with a vehicle that uses 9 or fewer channels (or 10 if you
don't need the zoom control).

Some key settings to configure:
- Add three new servos on the Electronics tab: Gimbal Pitch, Gimbal Yaw, and
  Camera Zoom.
- Add a camera model in the Physics tab.
  - Zoom Control Method: Servo
  - Minimum Field of View: 10
  - Maximum Field of View: 60
    - These can be set to any values you want, but you need to adjust the
      `RFG_FOV_MIN` and `RFG_FOV_MAX` parameters to match. These default to 10
      and 60.
  - Zoom Servo: Camera Zoom
  - Simulate Gimbal Stabilization: Yes
  - Gimbal Rotation (Minimum)(deg): x=-89.9,y=-180.0,z=-180.0
  - Gimbal Rotation (Maximum)(deg): x=89.9,y=180.0,z=180.0
    - Use these angles regardless of the actual range of motion you want. Set
      your limits in the `MNT1_` parameters instead
  - Gimbal Yaw Control: Gimbal Yaw
  - Gimbal Pitch Control: Gimbal Pitch
  - Gimbal Max Rotation Rate (deg/sec): 220
  - Gimbal Follow Yaw: Yes
  - Gimbal Follow Gain: 3.0
  - Gimbal Follow Deadband (deg): 0

Recommended ArduPilot parameter changes:
```
CAM1_TYPE,7
MNT1_DEFLT_MODE,1
MNT1_NEUTRAL_Y,-15
MNT1_PITCH_MAX,45
MNT1_PITCH_MIN,-45
MNT1_RETRACT_Y,-180
MNT1_RETRACT_Z,-45
MNT1_ROLL_MAX,60
MNT1_ROLL_MIN,-60
MNT1_TYPE,9
SCR_ENABLE,1
SERVO10_FUNCTION,7
SERVO10_MAX,2000
SERVO10_MIN,1000
SERVO11_FUNCTION,94
SERVO11_MAX,2000
SERVO11_MIN,1000
SERVO9_FUNCTION,6
SERVO9_MAX,2000
SERVO9_MIN,1000
```
