# INF Inject Driver

This driver implements support for the INF Inject EFI engine
control units.

 https://innoflighttechnology.com/efi/

# Parameters

The script used the following parameters:

## EFI_INF_ENABLE

this must be set to 1 to enable the driver

## EFI_INF_OPTIONS

This sets options for the driver. Currently the only option is to set
EFI_INF_OPTIONS to 1 to enable logging of the raw serial bytes to a
file called INF_Inject.log

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - SCR_VM_I_COUNT should be set to at least 50000
 - EFI_TYPE should be set to 7
 - EFI_INF_ENABLE should be set to 1
 - SERIALn_PROTOCOL should be set to 28 for the connected EFI serial
 - RPM_TYPE1 should be set to 3
 - ICE_ENABLE should be set to 1

then the flight controller should rebooted and parameters should be
refreshed.

Once loaded the EFI_INF_ENABLE parameters will appear and should be set
according to the parameter list above.

The GCS will receive EFI_STATUS MAVLink messages which includes RPM,
cylinder head temperature, exhaust gas temperature, injection timing,
engine load, fuel consumption rate, throttle position atmospheric
pressure and ignition voltage.
