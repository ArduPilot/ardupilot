# CAN Playback

The CAN_playback.lua script plays back captured CAN frames from a
flight log. This can be used to reproduce servo movement from a flight
which is useful for measuing the expected lifetime of CAN servos.

## Capturing CAN traffic

To capture all CAN traffic on a CAN bus while flying set bit zero of
CAN_Pn_OPTIONS, so if no other bits are set then set:

 - CAN_P1_OPTIONS=1
 - CAN_P2_OPTIONS=1

This assumes you want to capture all CAN bus traffic on both CAN
buses.

Note that this will result in a large flight log. It will capture CAN
frames into CANF log messages.

## Processing flight log

After capturing the flight log from a real flight you need to process
the log so that it only contains the CANF messages. To do that run the
following pymavlink command on the command line:

```
 mavlogdump.py -q -o CANF.bin inlog.BIN --type CANF
```

that will create a file CANF.bin which contains just the CANF frames
from inlog.bin.

## Setup for playback

On the playback system copy the CANF.bin file to the root of the
microSD card. This assumes you are using the script unmodified to
playback ArrayCommand servo actuator commands for servo testing.

Then set the following parameters for the first CAN bus:

 - CAN_P1_PROTOCOL2 = 10
 - CAN_D1_UC_SRV_BM = 0
 - CAN_D1_UC_OPTION = 1024

and the following parameters for the second CAN bus:

 - CAN_P2_PROTOCOL2 = 12
 - CAN_D2_UC_SRV_BM = 0
 - CAN_D2_UC_OPTION = 1024

You may also wish to set CAN_Dn_UC_ESC_BM to 0 to stop ESC control
traffic on the CAN bus to make the bus less busy.

Then load CAN_playback.lua into the APM/SCRIPTS/ directory on the
microSD card and set:

 - SCR_ENABLE = 1
 - SCR_VM_I_COUNT = 200000

After you reboot you should see a message:

``
 CAN_playback: starting
```

and if you monitor the CAN bus you should see the ArrayCommand
messages from your flight on the CAN bus. Your CAN servos should move
in an identical way to the movement in flight.

When the end of the CANF.bin file is reached you will see a message:

``
 CAN_playback: rewind
```

and the playback will automatically restart from the start of the log.

