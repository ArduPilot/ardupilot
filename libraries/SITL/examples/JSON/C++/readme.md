# C++

This library links a simulator written in C++ with ArduPilot through a UDP JSON interface. This simplifies adding support for ArduPilot to a simulator. A minimal example is included in `minimal.cpp`. This can be built-in Linux with the command `g++ minimal.cpp -o minimal.o`. The minimal example shows how each method in the library is used and can be used to test the library as well. The `simpleRover.cpp` example is a 1-D model of a rover that shows how a physics model can be integrated with this library.

Connection to SITL is made via a UDP link. The physics backend should listen for incoming messages on port 9002. The sim should then reply to the IP and port the messages were received from. This is handled by libAP_JSON. This removes the need to configure the target IP and port for SITL in the physics backend. ArduPilot SITL will send an output message every 10 seconds allowing the physics backend to auto-detect.

After a connection has been made, the optional values should be set. Then the vehicle state can be sent with `SendState`. This call includes all of the required values. The `servo_out` array supports 16 servo outputs from ArduPilot.

### Running the `simpleRover` example

The examples can be built using `cmake`:

```bash
$ mkdir build && cd build
$ cmake ..
$ make
```

Run the `simpleRover` physics engine:

```bash
$ ./simpleRover
```

Run SITL with the JSON backend:

```bash
sim_vehicle.py -v Rover -f JSON --console --map
```

The rover responds to throttle commands on RC channel 3:

```bash
# arm throttle
MANUAL> arm throttle

# move forwards at max throttle (expected velocity 1 m/s)
MANUAL> rc 3 1900

# move backwards at max throttle (expected velocity -1 m/s)
MANUAL> rc 3 1100

# stop
MANUAL> rc 3 1500
```
