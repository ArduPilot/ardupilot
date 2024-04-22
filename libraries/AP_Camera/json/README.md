# Summary

These files define MAVLink messages for the Camera in JSON. Each file is named
for the MAVLink message, and has a numeric suffix to indicate which Camera
instance it is for, e.g.:
* _"camera_information_1.json"_ defines the `CAMERA_INFORMATION` message for Camera 1.

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
