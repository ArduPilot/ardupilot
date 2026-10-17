# Copter Companion Computer Failsafe

This script changes Copter's flight mode if a companion computer that is
flying the vehicle in GUIDED mode stops sending MAVLink HEARTBEAT
messages.

When a companion computer commands the vehicle in GUIDED mode and its
software stops (a crash, loss of power or a cable fault), the vehicle
is left holding position with nothing in control. With this script the
vehicle will instead switch to a mode of your choice, by default LAND.

The script only acts when all of these are true:

- the vehicle is armed and in GUIDED mode
- a HEARTBEAT from the companion was seen during this GUIDED flight, or
  within CC_FS_TIMEOUT seconds before it started
- no HEARTBEAT from the companion has been seen for CC_FS_TIMEOUT seconds

GUIDED mode flown from a ground station alone, with no companion
HEARTBEAT, is not affected.

The GCS failsafe (FS_GCS_ENABLE) monitors the ground station. This
failsafe is separate from it and watches one component, so both can be
used together.

ArduPilot uses GUIDED mode for both operator and companion computer
control, so this script cannot tell them apart by mode alone. It relies
on having seen the companion's HEARTBEAT during the GUIDED flight.

## Parameters

### CC_FS_ENABLE

Set to 1 to enable the failsafe, 0 to disable it.

### CC_FS_TIMEOUT

Time in seconds without a companion HEARTBEAT before the flight mode is
changed. The default is 3 seconds.

### CC_FS_SYSID

MAVLink system ID of the companion computer. The default of 0 matches
any system ID.

### CC_FS_COMPID

MAVLink component ID of the companion computer. The default is 191
(MAV_COMP_ID_ONBOARD_COMPUTER). Set to 0 to match any component ID.

### CC_FS_MODE

Flight mode number to change to when the HEARTBEAT is lost. The default
is 9 (LAND). Other useful values are 5 (LOITER), 6 (RTL), 17 (BRAKE)
and 21 (SmartRTL).

If the vehicle refuses this mode, for example RTL without a home
position, the script changes to LAND instead. The mode change is tried
once per GUIDED flight and not repeated.

## Operation

Install the script in the APM/scripts folder on the flight controller's
microSD card, along with the MAVLink modules folder
(APM/scripts/modules/MAVLink) from the ArduPilot scripting directory.
Set SCR_ENABLE to 1 and reboot.

The companion computer must send a HEARTBEAT at 1Hz or faster, using
the component ID set in CC_FS_COMPID. With pymavlink:

```python
master = mavutil.mavlink_connection(address, source_system=1, source_component=191)
master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                          mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
```

The script reports "Companion FS: companion heartbeat detected" when the
HEARTBEAT is first seen, and "Companion FS: heartbeat lost, mode 9"
when it changes mode.

Test the behaviour in SITL, or on the ground with propellers removed,
before relying on it in flight.
