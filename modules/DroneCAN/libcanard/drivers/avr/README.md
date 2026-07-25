# Libcanard Driver for AVR Microcontrollers

This driver is based on the Universal CAN Library for AVR controllers (AT90CAN, MCP2515, SJA1000) by RCA
([Roboterclub Aachen e.V.](http://www.roboterclub.rwth-aachen.de/)).

Github: <https://github.com/dergraaf/avr-can-lib>

## Important

**Only tested for AT90CAN128!**

Currently the driver works only with additional buffer from the avr-can-lib (see below) as the internal
CAN Buffer (MOb's) are not read fast enough and therefore overflow occur even with hardware filter enabled.

```c
#define CAN_RX_BUFFER_SIZE              16
#define CAN_TX_BUFFER_SIZE              8
#define CAN_FORCE_TX_ORDER              1
```

## How-to

### Setup library

Adjust file `can_config.h` to your needs.
In file `CMakeLists.txt` set your controller and corresponding frequency:

```
set(MCU   at90can128)
set(F_CPU 16000000)
```

After finishing configuration:

```shell
$ cmake .
$ make
```

Include the following files in your project:

* `avr-can-lib/can.h`
* `avr-can-lib/libcan.a`
* `can_config.h`

### Use

1. Initialize `canardAVRInit(CAN_BITRATE)` with appropriate bitrate (bits/sec.)
2. Retrieve (allocation of dynamic node ID) or set node ID manually
3. Set CAN hardware filter `canardAVRConfigureAcceptanceFilters(NODE_ID)` with node ID from above
