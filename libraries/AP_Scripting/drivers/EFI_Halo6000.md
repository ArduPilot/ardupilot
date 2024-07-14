# EFI Halo6000 Driver

This driver implements support for the Halo6000 generator for hybrid
multicopters, using CAN protocol.

# Parameters

The script used the following parameters:

## EFI_H6K_ENABLE

this must be set to 1 to enable the driver

## EFI_H6K_CANDRV

This sets the CAN scripting driver number to attach to. This is
normally set to 1 to use a CAN driver with CAN_Dx_PROTOCOL=10. To use
the 2nd scripting CAN driver set this to 2 and set CAN_Dx_PROTOCOL=12.

## EFI_H6K_START_FN

This is the RC option to use to monitor start control. This should be
set to one of the scripting RC options (from 300 to 307). Then an
RCn_OPTION should be set to the same value. When this switch goes high
the generator start function will be sent to the ECU. When this switch
goes low a generator stop will be sent. A value of 0 disables the starter
control.

## EFI_H6K_TELEM_RT

This is the rate in Hz at which NAMED_VALUE_FLOAT messages are used to
send additional telemetry data to the GCS for display to the operator.

## EFI_H6K_FUELTOT

This is the total fuel tank capacity in litres

## EFI_H6K_OPTIONS

This provides additional options. Currently just one option is
available. If you set EFI_H6K_OPTIONS to 1 then all CAN frames will be
logged in the message CANF.

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - EFI_TYPE should be set to 7

then the flight controller should rebooted and parameters should be
refreshed.

Once loaded the EFI_H6K parameters will appear and should be set
according to the parameter list above.

The GCS will receive EFI_STATUS MAVLink messages which includes RPM,
and temperatures.
