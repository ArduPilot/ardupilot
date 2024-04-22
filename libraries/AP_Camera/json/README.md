# Summary

These files define MAVLink messages for the Camera in JSON. Each file is named
for the MAVLink message it is for, e.g.:
* _"camera_information.json"_ defines the `CAMERA_INFORMATION` message for the Camera.

**NOTE**: ArduPilot cannot currently disambiguate MAVLink Camera messages between backends -- all
messages will appear to come from the same Camera instance. As such, only one message definition
can be defined here for each message.

Currently supported messages are:
* [`CAMERA_INFORMATION`](https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION)
* [`VIDEO_STREAM_INFORMATION`](https://mavlink.io/en/messages/common.html#VIDEO_STREAM_INFORMATION)

# Usage
To use these with SITL:
* these files need to be in the `<repo_root>/mav_msg_def/AP_Camera/` folder.

To use these with a vehicle:
* the `AP_CAMERA_JSON_INFO_ENABLED` define needs to be set to 1 in the hwdef of your board.
* Add these files to either:
    * a `/mav_msg_def/AP_Camera/` folder on the SD card; or
    * a `/mav_msg_def/AP_Camera/` in the hwdef folder of your board.
* If the files are present in both of the above, they will be loaded from the SD card first.
