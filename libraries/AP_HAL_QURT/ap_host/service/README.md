# ArduPilot on Voxl-2

This directory provides a systemd service file for ArduPilot on Voxl2
by ModalAI

To build use:

 - ./waf configure --board QURT
 - ./waf copter

To install copy files as follows:

 - voxl-ardupilot.service to /etc/systemd/system/
 - voxl-ardupilot to /usr/bin/
 - build/QURT/ardupilot to /usr/bin/
 - build/QURT/bin/arducopter to /usr/lib/rfsa/adsp/ArduPilot.so
 - copy the right parameter file from Tools/Frame_params/ModalAI/ to /data/APM/defaults.parm

You can then use:

 - systemctl enable voxl-ardupilot.service
 - systemctl start voxl-ardupilot



