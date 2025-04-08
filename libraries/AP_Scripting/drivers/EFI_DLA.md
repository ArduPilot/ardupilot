# EFI DLA Driver

This driver implements support for the DLA EFI serial protocol for
this system:

https://www.austars-model.com/dla-232cc-uavuas-engine-optional-one-key-startauto-startergenerator_g17937.html

# Parameters

The script used the following parameters:

## EFI_DFA_ENABLE

this must be set to 1 to enable the driver

## EFI_DFA_LPS

This sets the fuel consumption rate in litres per second of injector
time. This will need to be tuned per engine to give the right value
for fuel usage and total fuel

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - EFI_TYPE should be set to 7
 - EFI_DFA_ENABLE should be set to 1
 - SERIALn_PROTOCOL should be set to 28 for the connected EFI serial
 - RPM_TYPE1 should be set to 3
 - ICE_ENABLE should be set to 1

then the flight controller should rebooted and parameters should be
refreshed.

Once loaded the EFI_DLA parameters will appear and should be set
according to the parameter list above.

The GCS will receive EFI_STATUS MAVLink messages which includes RPM,
cylinder head temperature, injection timing, engine load, fuel
consumption rate, throttle position atmospheric pressure and ECU
voltage.

Note that this EFI system only sends data when the enable PWM enable
signal is high. If that is hooked to the ignition control in ArduPilot
ICE system then you won't see any data until you start the engine.


