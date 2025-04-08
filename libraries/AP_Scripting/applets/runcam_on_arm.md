# `runcam_on_arm.lua`: start and stop video recording on arm/disarm

ArduPilot can control RunCam devices out of the box.
Normally, you use a 2-position switch with `RCx_OPTION=78:RunCamControl` to start and stop video recording.
This applet starts recording automatically when your vehicle arms and stops on disarm.

Information on setting up RunCam devices with ArduPilot is on this page: https://ardupilot.org/plane/docs/common-camera-runcam.html
You need a free serial port.
You need to set these three parameters: `SERIALx_PROTOCOL=26`, `SERIALx_BAUD=115` (where `x` is your serial port) and `CAM_RC_TYPE=y` where `y` depends on the RunCam model you are using.
You do not need to configure any `RCx_OPTION`s.

## If camera commands get out of sync

RUNCAM serial protocol controls the camera by simulating a button press both to start and stop recording--it's the same button.
If you send commands faster than the camera can process them, it can ignore a command and interpret the next one as the opposite of what you intend, e.g., you may lose a button press to stop recording, and the camera keeps rolling; when you're ready to start recording again, the camera interprets your button press as a
command to stop recording.
To address this, AP has a special parameter, `CAM_RC_BTN_DELAY`, that controls minimum time between simulated button presses.
If you find your commands getting out of sync--camera starting when you want it to stop--try increasing this value.
I find that with my RunCam Split HD, the default 300ms is inadequate, and 1500ms gives reliable performance.
