# EFI EDGE Serial Driver

This driver implements support for the Edge Autonomy EFI engine
control units using the RS232 serial protocol, as used on the
PenguinB.

## Parameters

The script uses the following parameters:

## EFI_EDGE_ENABLE

This must be set to 1 to enable the driver.

## EFI_EDGE_KILL_FN

Aux function number for ignition control. Set to -1 to disable
(default). On ArduPilot 4.6.x use a scripting aux function (e.g. 300)
as ICE_START_STOP (179) is not available to scripts.

## EFI_EDGE_TLM_RT

Telemetry rate in Hz requested from the ECU. Defaults to 10. Range
1 to 20.

## EFI_FUEL_DENS

ECU fuel density in kg/m3. Used to convert fuel flow from grams to
volume. If set to 0 then a default of 800 kg/m3 is used.

## Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

- SCR_ENABLE should be set to 1
- EFI_TYPE should be set to 7
- EFI_EDGE_ENABLE should be set to 1
- EFI_EDGE_KILL_FN should be set to the desired aux function for
  ignition control (e.g. 300 on ArduPilot 4.6.x)
- SERIALn_PROTOCOL should be set to 28 for the connected EFI serial
- SERIALn_BAUD should be set to 115
- SERIALn_OPTIONS should be set to 3 for inverted TX and RX
- RPM_TYPE1 should be set to 3
- ICE_ENABLE should be set to 1

then the flight controller should be rebooted and parameters should be
refreshed.

The driver requests telemetry from the ECU at the rate set by
EFI_EDGE_TLM_RT (default 10Hz). The GCS will receive EFI_STATUS
MAVLink messages which includes RPM, cylinder head temperature, ECU
temperature, intake manifold temperature, atmospheric pressure, fuel
consumption rate, throttle position and ignition voltage.

Ignition is controlled via the RC aux function set by EFI_EDGE_KILL_FN
(default 179, ICE_START_STOP). When the switch is low the driver sends
an ignition disable command to the ECU. When mid or high the driver
sends an ignition enable command. A message is sent to the GCS when
the ignition state changes.

## Logging

The driver logs three messages to the onboard log:

- **EFE1**: Fuel data from telemetry 1 packets. Fields: TFuel (total
  fuel consumed, 0.1kg), FSR (fuel since restart, g), EWT (engine
  working time, 0.01h), RFuel (remaining fuel, g).

- **EFE2**: Main engine data from telemetry 2 packets. Fields: RPM,
  FF (fuel flow, g/h), Bar (barometer, 0.1kPa), MAP (manifold
  pressure, 0.1kPa), MAT (manifold air temp), CLT (coolant temp),
  TPS (throttle position, 0.1%), AFR (air fuel ratio, 0.1), V
  (voltage, 0.1V), ET (ECU temp, degC), GT (generator temp, degC),
  ES (engine status flags).

- **EFE3**: Additional telemetry 2 data. Fields: T (time since
  restart, 0.01s), DI (duct current, mA), TI (throttle current, mA),
  FI (fuel pump current, mA), DP (duct position, %), TP (throttle
  position, %), St (system status flags), TS (throttle source), PW
  (pulse width), Av (advance, 0.1 deg).
