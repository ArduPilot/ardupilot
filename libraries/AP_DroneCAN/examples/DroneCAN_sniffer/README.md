This is a UAVCAN sniffer designed to run on an ArduPilot board. It can
be used to watch traffic on a UAVCAN bus, showing exactly what would
be received by another node.

To build and upload for a Pixhawk style board run this:

```
 ./waf configure --board fmuv3 
 ./waf --target examples/UAVCAN_sniffer --upload
```
 
then connect on the USB console. You will see 1Hz packet stats like
this:

```
uavcan.equipment.air_data.StaticPressure: 29
uavcan.equipment.air_data.StaticTemperature: 29
uavcan.equipment.ahrs.MagneticFieldStrength: 20
uavcan.protocol.NodeStatus: 6
uavcan.equipment.gnss.Fix: 10
uavcan.equipment.gnss.Auxiliary: 1
uavcan.equipment.actuator.ArrayCommand: 45
uavcan.equipment.esc.RawCommand: 368
```

note that the code requires you to add new msg types you want to
see. Look for the MSG_CB() and START_CB() macros in the code
