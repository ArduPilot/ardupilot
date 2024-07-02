# NMEA 2000 EFI driver

This driver implements support for marine EFI systems using NMEA 2000
CAN messages.

# Parameters

The script used the following parameters:

## EFI_2K_ENABLE

this must be set to 1 to enable the driver

## EFI_2K_OPTIONS

This sets options for the driver. Currently the only option is to set
EFI_2K_OPTIONS to 1 to enable logging of the raw CAN frames for
debugging purposes.

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The NMEA_2000.lua module from
AP_Scripting/modules/ also needs to be put in the APM/SCRIPTS/MODULES directory.

The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - EFI_TYPE should be set to 7
 - EFI_2K_ENABLE should be set to 1
 - EFI_2K_CANDRV needs to be set to the CAN driver number
 - CAN_Pn_BITRATE needs to be set to 250000
 - CAN_Dn_PROTOOCOL needs to be set to 10 for scripting

then the flight controller should rebooted and parameters should be
refreshed.

Once loaded the EFI_2K_ENABLE parameters will appear and should be set
according to the parameter list above.

The GCS will receive EFI_STATUS MAVLink messages which includes RPM,
cylinder head temperature, exhaust gas temperature, injection timing,
engine load, fuel consumption rate, throttle position atmospheric
pressure and ignition voltage.
