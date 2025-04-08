# Idle Control

Allows manual or automatic rpm control for heli while on ground idle condition

# Parameters

IDLE_GAIN_I = integrative gain of the controller
IDLE_GAIN_P = proportional gain of the controller
IDLE_GAIN_MAX = IMAX for integrative
IDLE_MAX = maximum throttle position, shall be set low enough to prevent clutch engagement/slipping or lower than first point of throttle curve
IDLE_RANGE = rpm range of operation for the idle control
IDLE_SETPOINT = desired rpm setpoint
IDLE_RPM_ENABLE = 0 for manual throttle control // 1 for RPM1 targeting // 2 for RPM2 targeting

# How To Use

set RCx_OPTION to 301 to enable idle control from an auxiliary potentiometer
