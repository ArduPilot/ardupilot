#!/bin/sh

mavproxy.py --master=/dev/ttyACM0 --baud=115200 --console <<<EOF
long MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN 3 0 0 0 0 0 0
EOF
