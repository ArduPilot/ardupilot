# DroneCAN CircuitStatus Battery Driver

This driver implements support for the DroneCAN
uavcan.equipment.power.CircuitStatus message, mapping individual
circuits onto ArduPilot battery monitor instances. This allows
per-circuit voltage and current from a CAN power distribution board to
be monitored, logged and reported as batteries.

## Parameters

The script uses the following parameters:

## DCS_NUM_CIRCUITS

The number of CircuitStatus battery monitor instances, up to a maximum
of 9. The per-instance DCSx parameters are created based on this
value, so a reboot is needed after changing it. Set to 0 to disable
the driver.

## DCSx_CIRCUIT_ID

The circuit_id in the CircuitStatus message to use for instance x. Set
to -1 to disable the instance.

## DCSx_BATT_IDX

The battery monitor index to feed with data from this circuit. Set to
1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR parameter
must be set to 29 (scripting). Set to 0 to disable the instance.

## Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

- SCR_ENABLE should be set to 1
- CAN_Px_DRIVER should be set to 1 or 2 to assign the physical CAN
  port to a CAN driver
- CAN_Dn_PROTOCOL should be set to 1 (DroneCAN) for that driver
- BATTn_MONITOR should be set to 29 for each battery instance used

then the flight controller should be rebooted and parameters should be
refreshed. The DCS_NUM_CIRCUITS parameter will then appear and should
be set to the number of circuits to monitor, followed by another
reboot. The DCSx_CIRCUIT_ID and DCSx_BATT_IDX parameters will then
appear and should be set to map each circuit to a battery monitor.
Changes to the circuit mappings take effect without a reboot.

Each configured battery instance reports the voltage and current from
the matching circuit. The current is passed through with the sign sent
by the device (the DSDL does not define a sign convention). If a
circuit reports a non-zero error_flags then a warning is sent to the
GCS. If a circuit stops sending data then the battery monitor will be
marked unhealthy after 5 seconds.
