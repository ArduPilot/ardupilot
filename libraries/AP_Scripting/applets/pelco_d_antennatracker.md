# Pelco-D antennatracker lua script

This scripts uses the scaled output from the antennatracker servos and map them to corresponding Pelco-D messages to be sent via a RS-485 interface to a motorized base (can be anything from motorized tracker to a PTZ camera). If your FCU doesnt offer a RS-485 interface by default, you can use or TTL-RS485- or USB-RS485-adapters. I tested this script using a USB-RS485 adapter using Linux/Obal board and a Hikvision PTZ camera.

Pelco-D allows to control using either speed-/differential- or absolute-control control of the pan-/tilt-axis. Currently the script uses speed based control using by mapping the "ContinuousRotation" type servos outputs to the corresponding Pelco-D messages. The absolute control messages are implemented nevertheless for future use.

The script assumes the following parameters to be set:

SCR_ENABLE = 1
SERVO_PITCH_TYPE = 2  # ContinuousRotation type servo 
SERVO_YAW_TYPE = 2    # ContinuousRotation type servo
SERIALx_PROTOCOL = 28 # replace 'x' with the serial port used by luascript

Additionally the PITCH2SRV, YAW2SRV tuning needs to be done as described by the antennatracker description.
Also keep attention to the PITCH_MIN, PITCH_MAX and YAW_RANGE parameters to fit your Pelco-D hardware!

