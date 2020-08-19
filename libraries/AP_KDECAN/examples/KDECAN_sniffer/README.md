This is a KDECAN sniffer designed to run on an ArduPilot board. It can
be used to watch traffic on a KDECAN bus as well as simulate a 8 ESCs (macro changeable).

To build and upload for a Pixhawk style board run this:

```
 ./waf configure --board fmuv3 
 ./waf --target examples/KDECAN_sniffer --upload
```
 
then connect on the USB console. You will see 1Hz packet stats.

You can respond to enumeration by writing 'e X', with X being the ESC number, zero index based.
