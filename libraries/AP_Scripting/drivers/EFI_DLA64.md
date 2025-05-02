# EFI DLA64 Driver

This driver implements support for the DLA64 EFI serial protocol

# Parameters

The script used the following parameters:

## EFI_DFA64_ENABLE

this must be set to 1 to enable the driver

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - EFI_TYPE should be set to 7
 - EFI_DFA64_ENABLE should be set to 1
 - SERIALn_PROTOCOL should be set to 28 for the connected EFI serial
 - RPM1_TYPE should be set to 3

then the flight controller should rebooted and parameters should be
refreshed.

Once loaded the EFI_DLA64 parameters will appear and should be set
according to the parameter list above.

The GCS will receive EFI_STATUS MAVLink messages which includes RPM,
cylinder head temperature, fuel consumption rate, atmospheric pressure
and ECU voltage.
