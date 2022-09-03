# Secure Boot Support

To assist with vendors needing high levels of tamper resistance with
RemoteID, you can optionally use secure boot with ArduPilot. This
involves installing a bootloader with up to 10 public keys included
and signing the ArduPilot vehicle firmware with one secret key. The
bootloader will refuse to boot the firmware if the signature on the
firmware doesn't match any of the public keys in the bootloader.

## Generating Keys

To generate a public/private key pair, run the following command:

```
  python3 -m pip install pymonocypher
  Tools/scripts/signing/generate_keys.py NAME
```

That will create two files:
 - NAME_private_key.dat
 - NAME_public_key.dat

NAME can be any string, but would usually be your vendor name. It is
only used for the local filenames.

The generated private key should be kept in a secure location. The
public key will be used to create a secure bootloader that will only
accept firmwares signed with one of the public keys in the bootloader.

## Building secure bootloader

To build a secure bootloader run this command:

```
 Tools/scripts/build_bootloaders.py BOARDNAME --signing-key=NAME_public_key.dat
```

That will update the bootloader in Tools/bootloaders/BOARDNAME_bl.bin
to enable secure boot with the specified public key. Next time you
build a firmware for this board then that bootloader will be included
in ROMFS.

Note that this will include the 3 ArduPilot signing keys by default as
well as your key. This is done so that your users can update to a
standard ArduPilot firmware release and also prevents issues with
vendors who can no longer provide firmware updates to users. If you
have a very good reason for not including the ArduPilot signing keys
then you can pass the option --omit-ardupilot-keys to the
make_secure_bl.py script.

## Building Signed Firmware

To build a signed firmware run this command (example is for a copter build):

```
 ./waf configure --board BOARDNAME --signed-fw
 ./waf copter
 ./Tools/scripts/signing/make_secure_fw.py build/BOARDNAME/bin/arducopter.apj NAME_private_key.dat
```

The final step signs the apj firmware with your private key. You can
then load that secure firmware as usual with your ground station, for
example using load custom firmware in MissionPlanner or
Tools/scripts/uploader.py on Linux.

Alternatively you can set the private key in the configure step, which
allows for build and upload in one step for faster development:

```
 ./waf configure --board BOARDNAME --signed-fw --private-key NAME_private_key.dat
 ./waf copter --upload
```

## Flashing the secure bootloader

There are two methods of getting the secure bootloader onto the
board. The simplest is to follow the above steps and then follow the
usual method of updating the bootloader, which involves sending a
MAVLink command to ask the firmware to flash the embedded bootloader
from ROMFS. The firmware generated using the above steps will have
your secure bootloader included in ROMFS, so when the users asks for
the bootloader to update it will flash the secure bootloader.

The second method is to put the board into DFU mode. If your hwdef.dat
and hwdef-bl.dat include the ENABLE_DFU_BOOT options and your board is
based on a STM32H7 then your ground station should be able to put the
board into DFU mode. You can then flash the bootloader bin file to
address 0x08000000 using any DFU capable client.

Note that the flight controller will refuse a switch to DFU mode if it
is running a secure bootloader already.

## How to tell you are using secure boot

When using a secure bootloader the USB ID presented by the bootloader
will have a "Secure" string added. For example, you would see this in
"dmesg" in Linux:

```
  Product: BOARDNAME-Secure-BL-v10
```

On Windows you can look at the device properties in device manager
when the bootloader is running and look for the "Bus reported device
description". It will have the above "Secure" string. Note that this
string only appears when in the bootloader. To ensure the board stays
in the bootloader for long enough to see this string just flash a
normal unsigned firmware. With a secure bootloader and an unsigned
firmware the board will stay in the bootloader forever as it will be
failing the secure boot checks.

## Reverting to normal boot

If you have installed secure boot on a board then to revert to normal
boot you would need to flash a new bootloader that does not have
secure boot enabled. To do that you should replace
Tools/bootloaders/BOARDNAME_bl.bin with the normal bootloader for your
board then build and sign a firmware as above. Then ask the flight
controller to flash the updated bootloader using the GCS interface and
you will then be running a normal bootloader.

## Supported Boards

Secure boot is only supported on boards with at least 32k of flash
space for the bootloader. This includes all boards based on the
STM32H7 and STM32F7. You can use secure boot on older other boards if
you change the hwdef.dat and hwdef-bl.dat to add more space for the
bootloader.

## Public key update over MAVLink

If you have a private key corresponding to one of the public keys in
the bootloader on a board then you can use the MAVLink2 SECURE_COMMAND
messages to change the public keys, or even remove all public keys to
allow the use of unsigned firmwares.

MAVProxy version 1.8.55 and later has a "securecommand" module which
gives you commands for:

 - generating a session key for remote update
 - fetching the current public keys
 - setting new public keys as additonal or replacement keys
 - removing all public keys

It is expected that future versions of MissionPlanner will include a
plugin with the same functionality.

Using SECURE_COMMAND in combination with MAVLink forwarding you can
hand over management of a vehicle between vendors.
